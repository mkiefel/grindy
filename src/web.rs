use defmt::*;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel, mutex, signal, watch};
use embassy_time::{Duration, Instant};
use picoserve::futures::Either;
use picoserve::response::ws;
use picoserve::routing::{get, get_service};
use picoserve::{make_static, AppBuilder, AppRouter};
use serde::Serialize;

use crate::scale::{GrinderStateMachine, ScaleSetting, WeightReading, WEIGHT_BATCH_CHANNEL_SIZE};
use crate::ui::{UserEvent, USER_EVENT_CHANNEL_SIZE};

pub const WEB_TASK_POOL_SIZE: usize = 8;

// WebSocket message types
#[derive(Serialize, Clone)]
enum WsMessage {
    Connected {
        state: UserEvent,
        scale_setting: ScaleSetting,
        timestamp_ms: u64,
    },
    StateChange {
        state: UserEvent,
        timestamp_ms: u64,
    },
    WeightBatch {
        readings: heapless::Vec<WeightReading, 4>,
    },
}

// WebSocket connection registry
const MAX_WS_CONNECTIONS: usize = 4;

pub struct WsConnectionRegistry {
    connections:
        [Option<&'static signal::Signal<CriticalSectionRawMutex, WsMessage>>; MAX_WS_CONNECTIONS],
}

impl WsConnectionRegistry {
    pub const fn new() -> Self {
        Self {
            connections: [None; MAX_WS_CONNECTIONS],
        }
    }

    fn register(
        &mut self,
        signal: &'static signal::Signal<CriticalSectionRawMutex, WsMessage>,
    ) -> Option<usize> {
        for (idx, slot) in self.connections.iter_mut().enumerate() {
            if slot.is_none() {
                *slot = Some(signal);
                return Some(idx);
            }
        }
        None
    }

    fn unregister(&mut self, idx: usize) {
        if idx < MAX_WS_CONNECTIONS {
            self.connections[idx] = None;
        }
    }

    fn broadcast(&self, msg: &WsMessage) {
        for slot in self.connections.iter() {
            if let Some(signal) = slot {
                signal.signal(msg.clone());
            }
        }
    }
}

// WebSocket handler
struct GrinderWebSocket {
    registry: &'static mutex::Mutex<CriticalSectionRawMutex, WsConnectionRegistry>,
    grinder_state_machine: &'static mutex::Mutex<CriticalSectionRawMutex, GrinderStateMachine>,
    signal: &'static signal::Signal<CriticalSectionRawMutex, WsMessage>,
}

impl ws::WebSocketCallback for GrinderWebSocket {
    async fn run<R: picoserve::io::Read, W: picoserve::io::Write<Error = R::Error>>(
        self,
        mut rx: ws::SocketRx<R>,
        mut tx: ws::SocketTx<W>,
    ) -> Result<(), W::Error> {
        // Register connection
        let conn_id = {
            let mut registry = self.registry.lock().await;
            registry.register(self.signal)
        };

        let conn_id = match conn_id {
            Some(id) => id,
            None => {
                warn!("WebSocket connection limit reached");
                let _ = tx.close(None).await;
                return Ok(());
            }
        };

        info!("WebSocket client {} connected", conn_id);

        // Send initial connected message with current state
        let (current_state, scale_setting) = {
            let grinder_state_machine = self.grinder_state_machine.lock().await;
            (
                grinder_state_machine.as_user_event(),
                grinder_state_machine.get_scale_setting().clone(),
            )
        };
        let connected_msg = WsMessage::Connected {
            state: current_state,
            scale_setting,
            timestamp_ms: Instant::now().as_millis(),
        };

        let mut buf = [0u8; 256];
        if let Ok(bytes) = postcard::to_slice(&connected_msg, &mut buf) {
            let _ = tx.send_binary(bytes).await;
        }

        // Main loop: handle both broadcast messages and client messages
        let mut buffer = [0u8; 128];
        loop {
            match rx.next_message(&mut buffer, self.signal.wait()).await {
                Ok(Either::First(msg)) => {
                    match msg {
                        Ok(ws::Message::Close(_)) => {
                            info!("WebSocket client {} closed connection", conn_id);
                            break;
                        }
                        Ok(ws::Message::Ping(data)) => {
                            if tx.send_pong(data).await.is_err() {
                                break;
                            }
                        }
                        Ok(_) => {} // Ignore other messages
                        Err(_) => {
                            warn!("WebSocket client {} read error", conn_id);
                            break;
                        }
                    }
                }
                Ok(Either::Second(msg)) => {
                    let mut buf = [0u8; 128];
                    if let Ok(bytes) = postcard::to_slice(&msg, &mut buf) {
                        if tx.send_binary(bytes).await.is_err() {
                            // Client disconnected.
                            break;
                        }
                    } else {
                        warn!(
                            "WebSocket client {} failed to serialize WebSocket message for client",
                            conn_id
                        );
                    }
                }
                Err(_) => {
                    warn!("WebSocket client synchronization error {}", conn_id);
                    break;
                }
            }
        }

        // Unregister connection on disconnect
        {
            let mut registry = self.registry.lock().await;
            registry.unregister(conn_id);
            info!("WebSocket client {} disconnected and unregistered", conn_id);
        }

        Ok(())
    }
}

struct AppProps {
    grinder_state_machine: &'static mutex::Mutex<CriticalSectionRawMutex, GrinderStateMachine>,
    ws_registry: &'static mutex::Mutex<CriticalSectionRawMutex, WsConnectionRegistry>,
}

impl AppBuilder for AppProps {
    type PathRouter = impl picoserve::routing::PathRouter;

    fn build_app(self) -> picoserve::Router<Self::PathRouter> {
        let Self {
            grinder_state_machine,
            ws_registry,
        } = self;
        picoserve::Router::new()
            .route(
                "/",
                get_service(picoserve::response::File::html(include_str!("index.html"))),
            )
            .route(
                "/index.js",
                get_service(picoserve::response::File::javascript(include_str!(
                    "index.js"
                ))),
            )
            .route(
                "/ws",
                get(
                    move |upgrade: picoserve::response::WebSocketUpgrade| async move {
                        // Allocate a new signal for this connection
                        // Note: This is a simplified version - in production we'd need a pool
                        static WS_SIGNALS: [signal::Signal<CriticalSectionRawMutex, WsMessage>;
                            MAX_WS_CONNECTIONS] = [
                            signal::Signal::new(),
                            signal::Signal::new(),
                            signal::Signal::new(),
                            signal::Signal::new(),
                        ];

                        // Find an available signal
                        let signal_idx = {
                            let registry = ws_registry.lock().await;
                            let mut idx = 0;
                            for (i, conn) in registry.connections.iter().enumerate() {
                                if conn.is_none() {
                                    idx = i;
                                    break;
                                }
                            }
                            idx
                        };

                        upgrade
                            .on_upgrade(GrinderWebSocket {
                                registry: ws_registry,
                                grinder_state_machine,
                                signal: &WS_SIGNALS[signal_idx],
                            })
                            .with_protocol("grindy")
                    },
                ),
            )
    }
}

#[embassy_executor::task(pool_size = WEB_TASK_POOL_SIZE)]
async fn web_task(
    task_id: usize,
    stack: embassy_net::Stack<'static>,
    app: &'static AppRouter<AppProps>,
    config: &'static picoserve::Config<Duration>,
) -> ! {
    let port = 80;
    let mut tcp_rx_buffer = [0; 1024 * 2];
    let mut tcp_tx_buffer = [0; 1024 * 2];
    let mut http_buffer = [0; 2048 * 2];

    picoserve::Server::new(app, config, &mut http_buffer)
        .listen_and_serve(task_id, stack, port, &mut tcp_rx_buffer, &mut tcp_tx_buffer)
        .await
        .into_never()
}

#[embassy_executor::task]
pub async fn websocket_broadcaster_task(
    mut state_receiver: watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        UserEvent,
        USER_EVENT_CHANNEL_SIZE,
    >,
    weight_batch_receiver: channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        heapless::Vec<WeightReading, 4>,
        WEIGHT_BATCH_CHANNEL_SIZE,
    >,
    ws_registry: &'static mutex::Mutex<CriticalSectionRawMutex, WsConnectionRegistry>,
) {
    let mut state_initial = state_receiver.get().await;
    loop {
        embassy_futures::select::select(
            // Listen for state changes
            async {
                let new_state = state_receiver
                    .changed_and(|&state| state != state_initial)
                    .await;
                let msg = WsMessage::StateChange {
                    state: new_state,
                    timestamp_ms: Instant::now().as_millis(),
                };
                let registry = ws_registry.lock().await;
                registry.broadcast(&msg);
                state_initial = new_state;
            },
            // Listen for weight batches
            async {
                let batch = weight_batch_receiver.receive().await;
                if !batch.is_empty() {
                    let msg = WsMessage::WeightBatch { readings: batch };
                    let registry = ws_registry.lock().await;
                    registry.broadcast(&msg);
                }
            },
        )
        .await;
    }
}

pub fn bringup_web_server(
    spawner: &Spawner,
    stack: embassy_net::Stack<'static>,
    grinder_state_machine: &'static mutex::Mutex<CriticalSectionRawMutex, GrinderStateMachine>,
    ws_registry: &'static mutex::Mutex<CriticalSectionRawMutex, WsConnectionRegistry>,
) {
    let app = make_static!(
        AppRouter<AppProps>,
        AppProps {
            grinder_state_machine,
            ws_registry,
        }
        .build_app()
    );
    let config = make_static!(
        picoserve::Config::<Duration>,
        picoserve::Config::new(picoserve::Timeouts {
            start_read_request: Some(Duration::from_secs(5)),
            persistent_start_read_request: Some(Duration::from_secs(1)),
            read_request: Some(Duration::from_secs(1)),
            write: Some(Duration::from_secs(1)),
        })
        .keep_connection_alive()
    );

    for task_id in 0..WEB_TASK_POOL_SIZE {
        spawner.must_spawn(web_task(task_id, stack, app, config));
    }
}
