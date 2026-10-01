use defmt::*;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel, mutex, watch};
use embassy_time::{Duration, Instant};
use picoserve::futures::Either;
use picoserve::response::ws;
use picoserve::extract::Form;
use picoserve::response::StatusCode;
use picoserve::routing::{get, get_service, post};
use picoserve::{make_static, AppBuilder, AppRouter};
use serde::{Deserialize, Serialize};

use crate::scale::{
    CalibrationError, ControllerEvent, GrindFinished, GrinderStateMachine, HolderConfigError,
    ScaleSetting, WeightReading, CONTROLLER_EVENT_CHANNEL_SIZE,
};
use crate::storage::{self, Holder, HolderConfig, SharedFlash, WifiConfig};
use crate::ui::{UserEvent, USER_EVENT_CHANNEL_SIZE};
use crate::wifi::{WifiMode, WIFI_CONFIG_CHANGED, WIFI_STATUS};

pub const WEB_TASK_POOL_SIZE: usize = 8;

// WebSocket message types
#[derive(Serialize, Clone)]
enum WsMessage {
    Connected {
        state: UserEvent,
        scale_setting: ScaleSetting,
        target_weight: f32,
        timestamp_ms: u64,
        lead_time: f32,
    },
    StateChange {
        state: UserEvent,
        scale_setting: ScaleSetting,
        target_weight: f32,
        timestamp_ms: u64,
        lead_time: f32,
    },
    Weight(WeightReading),
    TargetWeightChanged {
        target_weight: f32,
    },
    GrindFinished(GrindFinished),
}

// WebSocket connection registry
const MAX_WS_CONNECTIONS: usize = 4;
/// Messages buffered per connection; further messages are dropped for a client
/// that falls this far behind.
const WS_QUEUE_SIZE: usize = 16;

type WsQueue = channel::Channel<CriticalSectionRawMutex, WsMessage, WS_QUEUE_SIZE>;

static WS_QUEUES: [WsQueue; MAX_WS_CONNECTIONS] = [const { WsQueue::new() }; MAX_WS_CONNECTIONS];

pub struct WsConnectionRegistry {
    connected: [bool; MAX_WS_CONNECTIONS],
}

impl WsConnectionRegistry {
    pub const fn new() -> Self {
        Self {
            connected: [false; MAX_WS_CONNECTIONS],
        }
    }

    /// Claims a free slot and returns its index; its queue is `WS_QUEUES[idx]`.
    fn register(&mut self) -> Option<usize> {
        let idx = self.connected.iter().position(|&connected| !connected)?;
        self.connected[idx] = true;
        // Drop anything left over from the slot's previous connection.
        WS_QUEUES[idx].clear();
        Some(idx)
    }

    fn unregister(&mut self, idx: usize) {
        if idx < MAX_WS_CONNECTIONS {
            self.connected[idx] = false;
        }
    }

    fn broadcast(&self, msg: &WsMessage) {
        for (idx, queue) in WS_QUEUES.iter().enumerate() {
            if self.connected[idx] && queue.try_send(msg.clone()).is_err() {
                warn!("WebSocket client {} queue full, dropping message", idx);
            }
        }
    }
}

// WebSocket handler
struct GrinderWebSocket {
    registry: &'static mutex::Mutex<CriticalSectionRawMutex, WsConnectionRegistry>,
    grinder_state_machine: &'static mutex::Mutex<CriticalSectionRawMutex, GrinderStateMachine>,
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
            registry.register()
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
        let queue = &WS_QUEUES[conn_id];

        // Send initial connected message with current state
        let (current_state, scale_setting, target_weight, lead_time) = {
            let grinder_state_machine = self.grinder_state_machine.lock().await;
            (
                grinder_state_machine.as_user_event(),
                grinder_state_machine.get_scale_setting().clone(),
                grinder_state_machine.get_target_weight(),
                grinder_state_machine.get_lead_time(),
            )
        };
        let connected_msg = WsMessage::Connected {
            state: current_state,
            scale_setting,
            target_weight,
            timestamp_ms: Instant::now().as_millis(),
            lead_time,
        };

        let mut buf = [0u8; 256];
        if let Ok(bytes) = postcard::to_slice(&connected_msg, &mut buf) {
            let _ = tx.send_binary(bytes).await;
        }

        // Main loop: handle both broadcast messages and client messages
        let mut buffer = [0u8; 128];
        loop {
            match rx.next_message(&mut buffer, queue.receive()).await {
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

#[derive(Deserialize)]
struct CalibrateForm {
    weight: f32,
}

/// Largest reference weight we accept for calibration in grams.
const MAX_CALIBRATION_WEIGHT: f32 = 5000.0;

fn calibration_response(result: Result<(), CalibrationError>) -> (StatusCode, &'static str) {
    match result {
        Ok(()) => (StatusCode::OK, "OK"),
        Err(CalibrationError::Busy) => (StatusCode::CONFLICT, "Grinder is busy"),
        Err(CalibrationError::NotCalibrating) => (StatusCode::CONFLICT, "Not calibrating"),
    }
}

#[derive(Deserialize)]
struct HoldersForm {
    single_weight: f32,
    single_target: f32,
    double_weight: f32,
    double_target: f32,
}

#[derive(Deserialize)]
struct WifiForm {
    ssid: heapless::String<{ storage::MAX_SSID_LEN }>,
    password: heapless::String<{ storage::MAX_PASSWORD_LEN }>,
}

#[derive(Serialize)]
struct WifiStatusResponse {
    mode: &'static str,
    ssid: Option<heapless::String<{ storage::MAX_SSID_LEN }>>,
}

async fn wifi_status() -> picoserve::response::Json<WifiStatusResponse> {
    let status = WIFI_STATUS.lock().await;
    let mode = match status.mode {
        WifiMode::Connecting => "connecting",
        WifiMode::Client => "client",
        WifiMode::SetupAccessPoint => "setup",
    };
    picoserve::response::Json(WifiStatusResponse {
        mode,
        ssid: status.ssid.clone(),
    })
}

struct AppProps {
    grinder_state_machine: &'static mutex::Mutex<CriticalSectionRawMutex, GrinderStateMachine>,
    ws_registry: &'static mutex::Mutex<CriticalSectionRawMutex, WsConnectionRegistry>,
    flash: SharedFlash,
}

impl AppBuilder for AppProps {
    type PathRouter = impl picoserve::routing::PathRouter;

    fn build_app(self) -> picoserve::Router<Self::PathRouter> {
        let Self {
            grinder_state_machine,
            ws_registry,
            flash,
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
                "/calibrate",
                post(move |Form(CalibrateForm { weight })| async move {
                    if !(weight > 0.0 && weight <= MAX_CALIBRATION_WEIGHT) {
                        return (StatusCode::BAD_REQUEST, "Invalid calibration weight");
                    }
                    let result = grinder_state_machine
                        .lock()
                        .await
                        .start_calibration(weight);
                    calibration_response(result)
                }),
            )
            .route(
                "/calibrate/cancel",
                post(move || async move {
                    let result = grinder_state_machine.lock().await.cancel_calibration();
                    calibration_response(result)
                }),
            )
            .route(
                "/holders",
                get(move || async move {
                    let config = *grinder_state_machine.lock().await.get_holder_config();
                    picoserve::response::Json(config)
                })
                .post(
                    move |Form(HoldersForm {
                              single_weight,
                              single_target,
                              double_weight,
                              double_target,
                          })| async move {
                        let config = HolderConfig {
                            single: Holder {
                                weight: single_weight,
                                target: single_target,
                            },
                            double: Holder {
                                weight: double_weight,
                                target: double_target,
                            },
                        };
                        let target_weight = {
                            let mut grinder_state_machine = grinder_state_machine.lock().await;
                            grinder_state_machine
                                .set_holder_config(config)
                                .map(|()| grinder_state_machine.get_target_weight())
                        };
                        match target_weight {
                            Ok(target_weight) => {
                                // Also tells other pages to reload the holders.
                                ws_registry
                                    .lock()
                                    .await
                                    .broadcast(&WsMessage::TargetWeightChanged { target_weight });
                                (StatusCode::OK, "OK")
                            }
                            Err(HolderConfigError::OutOfRange) => (
                                StatusCode::BAD_REQUEST,
                                "Holder weights must be 100-2000 g and targets 1-100 g",
                            ),
                            Err(HolderConfigError::Storage) => (
                                StatusCode::INTERNAL_SERVER_ERROR,
                                "Failed to store holders",
                            ),
                        }
                    },
                ),
            )
            .route(
                "/wifi",
                get(wifi_status).post(
                    move |Form(WifiForm { ssid, password })| async move {
                        let config = WifiConfig { ssid, password };
                        if !config.is_valid() {
                            return (
                                StatusCode::BAD_REQUEST,
                                "SSID must not be empty and the password must be empty or at least 8 characters",
                            );
                        }
                        if !flash.lock(|flash| {
                            storage::write_wifi_config(&mut flash.borrow_mut(), &config)
                        }) {
                            return (StatusCode::INTERNAL_SERVER_ERROR, "Failed to store WiFi config");
                        }
                        WIFI_CONFIG_CHANGED.signal(config);
                        (StatusCode::OK, "OK")
                    },
                ),
            )
            .route(
                "/ws",
                get(
                    move |upgrade: picoserve::response::WebSocketUpgrade| async move {
                        upgrade
                            .on_upgrade(GrinderWebSocket {
                                registry: ws_registry,
                                grinder_state_machine,
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
    event_receiver: channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        ControllerEvent,
        CONTROLLER_EVENT_CHANNEL_SIZE,
    >,
    ws_registry: &'static mutex::Mutex<CriticalSectionRawMutex, WsConnectionRegistry>,
    grinder_state_machine: &'static mutex::Mutex<CriticalSectionRawMutex, GrinderStateMachine>,
) {
    let mut state_initial = state_receiver.get().await;
    loop {
        embassy_futures::select::select(
            // Listen for state changes
            async {
                let new_state = state_receiver
                    .changed_and(|&state| state != state_initial)
                    .await;
                let (scale_setting, target_weight, lead_time) = {
                    let grinder_state_machine = grinder_state_machine.lock().await;
                    (
                        grinder_state_machine.get_scale_setting().clone(),
                        grinder_state_machine.get_target_weight(),
                        grinder_state_machine.get_lead_time(),
                    )
                };
                let msg = WsMessage::StateChange {
                    state: new_state,
                    scale_setting,
                    target_weight,
                    timestamp_ms: Instant::now().as_millis(),
                    lead_time,
                };
                let registry = ws_registry.lock().await;
                registry.broadcast(&msg);
                state_initial = new_state;
            },
            // Listen for controller events
            async {
                let msg = match event_receiver.receive().await {
                    ControllerEvent::Reading(reading) => WsMessage::Weight(reading),
                    ControllerEvent::GrindFinished(finished) => WsMessage::GrindFinished(finished),
                };
                let registry = ws_registry.lock().await;
                registry.broadcast(&msg);
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
    flash: SharedFlash,
) {
    let app = make_static!(
        AppRouter<AppProps>,
        AppProps {
            grinder_state_machine,
            ws_registry,
            flash,
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
