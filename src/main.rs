#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use cyw43::{Control, NetDriver};
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{self, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel, mutex, signal, watch};
use embassy_time::{Delay, Duration, Instant, Timer};
use gpio::{Input, Level, Output};
use loadcell::{hx711::GainMode, LoadCell};
use num_traits::float::FloatCore;
use picoserve::response::ws;
use picoserve::routing::{get, get_service};
use picoserve::{make_static, AppBuilder, AppRouter};
use serde::Serialize;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Grindy - Coffee Grinder Controller"),
    embassy_rp::binary_info::rp_program_description!(
        c"An automated coffee grinder controller using a Raspberry Pi Pico"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut stack: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

// WebSocket message types
#[derive(Serialize, Clone)]
enum WsMessage {
    Connected {
        state: UserEvent,
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

#[derive(Serialize, Clone, Copy)]
struct WeightReading {
    timestamp_ms: u64,
    weight: f32,
    state: UserEvent,
    coffee_weight: Option<f32>,
}

// WebSocket connection registry
const MAX_WS_CONNECTIONS: usize = 4;

struct WsConnectionRegistry {
    connections:
        [Option<&'static signal::Signal<CriticalSectionRawMutex, WsMessage>>; MAX_WS_CONNECTIONS],
}

impl WsConnectionRegistry {
    const fn new() -> Self {
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
        let current_state = self.grinder_state_machine.lock().await.as_user_event();
        let connected_msg = WsMessage::Connected {
            state: current_state,
            timestamp_ms: Instant::now().as_millis(),
        };

        let mut buf = [0u8; 256];
        if let Ok(bytes) = postcard::to_slice(&connected_msg, &mut buf) {
            let _ = tx.send_binary(bytes).await;
        }

        // Main loop: handle both broadcast messages and client messages
        let mut buffer = [0u8; 128];
        loop {
            // Check for broadcast message or client message.
            let input =
                embassy_futures::select::select(self.signal.wait(), rx.next_message(&mut buffer))
                    .await;

            match input {
                embassy_futures::select::Either::First(msg) => {
                    let mut buf = [0u8; 128];
                    if let Ok(bytes) = postcard::to_slice(&msg, &mut buf) {
                        if tx.send_binary(bytes).await.is_err() {
                            // Client disconnected.
                            break;
                        }
                    } else {
                        warn!(
                            "Failed to serialize WebSocket message for client {}",
                            conn_id
                        );
                    }
                }
                embassy_futures::select::Either::Second(result) => match result {
                    Ok(msg) => match msg {
                        ws::Message::Close(_) => {
                            info!("WebSocket client {} closed connection", conn_id);
                            break;
                        }
                        ws::Message::Ping(data) => {
                            if tx.send_pong(data).await.is_err() {
                                break;
                            }
                        }
                        _ => {} // Ignore other messages
                    },
                    Err(_) => {
                        warn!("WebSocket client {} read error", conn_id);
                        break;
                    }
                },
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

                        upgrade.on_upgrade(GrinderWebSocket {
                            registry: ws_registry,
                            grinder_state_machine,
                            signal: &WS_SIGNALS[signal_idx],
                        })
                    },
                ),
            )
    }
}

const WEB_TASK_POOL_SIZE: usize = 8;

#[embassy_executor::task(pool_size = WEB_TASK_POOL_SIZE)]
async fn web_task(
    task_id: usize,
    stack: embassy_net::Stack<'static>,
    app: &'static AppRouter<AppProps>,
    config: &'static picoserve::Config<Duration>,
) -> ! {
    let port = 80;
    let mut tcp_rx_buffer = [0; 1024];
    let mut tcp_tx_buffer = [0; 1024];
    let mut http_buffer = [0; 2048];

    picoserve::listen_and_serve(
        task_id,
        app,
        config,
        stack,
        port,
        &mut tcp_rx_buffer,
        &mut tcp_tx_buffer,
        &mut http_buffer,
    )
    .await
}

const SCALE_CHANNEL_SIZE: usize = 5;

#[embassy_executor::task]
async fn scale_task(
    sck: Output<'static>,
    dt: Input<'static>,
    sender: channel::Sender<'static, CriticalSectionRawMutex, f32, SCALE_CHANNEL_SIZE>,
) {
    debug!("Setting up scale...");
    let delay = Delay {};
    let mut scale = loadcell::hx711::HX711::new(sck, dt, delay);
    scale.set_gain_mode(GainMode::A64);

    loop {
        if scale.is_ready() {
            match scale.read() {
                Ok(r) => {
                    sender.send(r as f32).await;
                }
                Err(_) => {
                    warn!("Failed to read scale although it was ready.");
                    Timer::after(Duration::from_millis(50)).await;
                    continue;
                }
            };
        }
        // Let other tasks run.
        // TODO: Implement this with async PIO to predict HX711 readiness.
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
enum UserEvent {
    Idle,
    Stabilizing,
    Grinding,
    WaitingForRemoval,
}

#[embassy_executor::task]
async fn led_task(
    mut control: Control<'static>,
    mut state_receiver: watch::Receiver<'static, CriticalSectionRawMutex, UserEvent, 2>,
) {
    loop {
        match state_receiver.get().await {
            UserEvent::Idle => {
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(1000)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(1000)).await;
            }
            UserEvent::Stabilizing => {
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(250)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(250)).await;
            }
            UserEvent::Grinding => {
                control.gpio_set(0, true).await;
                state_receiver.changed().await;
            }
            UserEvent::WaitingForRemoval => {
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(100)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
}

fn bringup_network_stack(
    spawner: &Spawner,
    net_device: NetDriver<'static>,
) -> embassy_net::Stack<'static> {
    let (stack, runner) = embassy_net::new(
        net_device,
        embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
            address: embassy_net::Ipv4Cidr::new(core::net::Ipv4Addr::new(192, 168, 22, 1), 24),
            gateway: None,
            dns_servers: Default::default(),
        }),
        make_static!(
            embassy_net::StackResources::<WEB_TASK_POOL_SIZE>,
            embassy_net::StackResources::new()
        ),
        embassy_rp::clocks::RoscRng.next_u64(),
    );
    spawner.must_spawn(net_task(runner));
    stack
}

fn bringup_web_server(
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

struct ScaleSetting {
    offset: f32,
    inv_variance: f32,
    factor: f32,
}

impl ScaleSetting {
    fn translate(&self, raw: f32) -> f32 {
        (raw - self.offset) * self.factor
    }
}

/// Compute mean and variance with additional removal of outlier based on Median Absolute Deviation
/// (MAD).
pub fn compute_mean_variance<const N: usize>(
    data: &mut heapless::Vec<f32, N>,
    threshold: f32,
) -> Option<(f32, f32)> {
    if data.is_empty() {
        return None;
    }

    let median = compute_median(data);
    let mut deviations: heapless::Vec<f32, N> = data.iter().map(|&x| (x - median).abs()).collect();
    let mad = compute_median(&mut deviations);

    // Avoid division by zero if MAD is 0 (all values identical).
    if mad < f32::EPSILON {
        return None;
    }

    // Filter outliers using modified z-score.
    // TODO: Remove duplication.
    let mean = data
        .iter()
        .filter(|&&x| {
            let modified_z = 0.6745f32 * (x - median).abs() / mad;
            modified_z < threshold
        })
        .fold((0.0f32, 0), |(mean, count), v| {
            (
                mean * (count as f32) / (count as f32 + 1.0f32) + v / (count as f32 + 1.0f32),
                count + 1,
            )
        })
        .0;
    let variance = data
        .iter()
        .filter(|&&x| {
            let modified_z = 0.6745f32 * (x - median).abs() / mad;
            modified_z < threshold
        })
        .map(|&x| (x - mean).powi(2))
        .fold((0.0f32, 0), |(mean, count), v| {
            (
                mean * (count as f32) / (count as f32 + 1.0f32) + v / (count as f32 + 1.0f32),
                count + 1,
            )
        })
        .0;
    Some((mean, variance))
}

/// Compute median in place.
fn compute_median<const N: usize>(data: &mut heapless::Vec<f32, N>) -> f32 {
    data.sort_unstable_by(|a: &f32, b: &f32| a.partial_cmp(b).unwrap());

    let len = data.len();
    if len % 2 == 0 {
        (data[len / 2 - 1] + data[len / 2]) / 2.0
    } else {
        data[len / 2]
    }
}

const MAX_GRIND_TIME_IN_SECS: usize = 50;

struct GrinderStateMachine {
    grinder: Output<'static>,
    scale_setting: ScaleSetting,
    state: Option<GrinderState>,
}

enum GrinderState {
    Tare {
        samples: heapless::Vec<f32, 100>,
    },
    WaitingForCalibration {},
    Calibrating {
        samples: heapless::Vec<f32, 100>,
    },
    WaitingForPortafilter {},
    Stabilizing {
        start_time: Instant,
        portafilter_weight: f32,
        sample_count: usize,
    },
    Grinding {
        start_time: Instant,
        portafilter_weight: f32,
    },
    WaitingForRemoval {},
}

impl GrinderStateMachine {
    fn new(grinder: Output<'static>) -> Self {
        Self {
            grinder,
            scale_setting: ScaleSetting {
                offset: 0.0,
                inv_variance: 0.0,
                factor: 200.0 / 85314.55,
            },
            state: Some(GrinderState::Tare {
                samples: heapless::Vec::new(),
            }),
        }
    }

    fn as_user_event(&self) -> UserEvent {
        match self.state.as_ref().unwrap() {
            GrinderState::Tare { .. }
            | GrinderState::WaitingForPortafilter { .. }
            | GrinderState::WaitingForCalibration {} => UserEvent::Idle,
            GrinderState::Calibrating { .. } => UserEvent::Stabilizing,
            GrinderState::Stabilizing { .. } => UserEvent::Stabilizing,
            GrinderState::Grinding { .. } => UserEvent::Grinding,
            GrinderState::WaitingForRemoval { .. } => UserEvent::WaitingForRemoval,
        }
    }

    fn get_coffee_weight(&self, current_weight: f32) -> Option<f32> {
        match self.state.as_ref().unwrap() {
            GrinderState::Grinding {
                portafilter_weight, ..
            } => Some(current_weight - portafilter_weight),
            _ => None,
        }
    }

    fn update_weight(&mut self, raw_weight: f32) {
        // Minimum weight to detect portafilter placement.
        const CALIBRATION_WEIGHT: f32 = 200.0;
        // Minimum weight to detect portafilter placement.
        const PORTAFILTER_THRESHOLD: f32 = 100.0;
        // Weight below which we consider portafilter removed.
        const REMOVAL_THRESHOLD: f32 = 10.0;
        // Target coffee weight in grams.
        const TARGET_COFFEE_WEIGHT: f32 = 18.0;
        // Time to wait for weight stabilization.
        const STABILIZATION_TIME: Duration = Duration::from_secs(2);
        // Tolerance for weight stability (grams).
        const WEIGHT_STABILITY_TOLERANCE: f32 = 1.0;

        let weight = self.scale_setting.translate(raw_weight);
        debug!("weight: {}", weight);
        self.state = Some(match self.state.take().unwrap() {
            GrinderState::Tare { mut samples } => {
                if samples.is_full() {
                    let (new_mean_offset, variance) =
                        compute_mean_variance(&mut samples, 3.0).unwrap_or((0.0, 0.0));
                    info!(
                        "Tare complete. Offset: {}, Variance: {}",
                        new_mean_offset, variance
                    );
                    self.scale_setting.offset = new_mean_offset;
                    self.scale_setting.inv_variance = 1.0 / variance.max(1.0);
                    GrinderState::WaitingForCalibration {}
                } else {
                    samples.push(raw_weight).unwrap();
                    GrinderState::Tare { samples }
                }
            }

            GrinderState::WaitingForCalibration {} => {
                if weight > CALIBRATION_WEIGHT * 0.8 {
                    info!(
                        "Known weight detected: {}g - starting calibration...",
                        weight
                    );
                    GrinderState::Calibrating {
                        samples: heapless::Vec::new(),
                    }
                } else {
                    GrinderState::WaitingForCalibration {}
                }
            }

            GrinderState::Calibrating { mut samples } => {
                if samples.is_full() {
                    let (new_mean_weight, _variance) = compute_mean_variance(&mut samples, 3.0)
                        .unwrap_or((CALIBRATION_WEIGHT, 0.0));
                    let factor = CALIBRATION_WEIGHT / new_mean_weight;
                    info!(
                        "Calibration complete. Offset: {}g,  Factor: {}",
                        new_mean_weight, factor
                    );
                    self.scale_setting.factor *= factor;
                    GrinderState::WaitingForRemoval {}
                } else {
                    samples.push(weight).unwrap();
                    GrinderState::Calibrating { samples }
                }
            }

            GrinderState::WaitingForPortafilter {} => {
                if weight > PORTAFILTER_THRESHOLD {
                    info!("Portafilter detected! Weight: {}g - stabilizing...", weight);

                    GrinderState::Stabilizing {
                        start_time: Instant::now(),
                        portafilter_weight: weight,
                        sample_count: 1,
                    }
                } else {
                    GrinderState::WaitingForPortafilter {}
                }
            }

            GrinderState::Stabilizing {
                start_time,
                portafilter_weight,
                sample_count,
            } => {
                // Check if weight is stable (within tolerance of initial portafilter weight).
                let weight_diff = (weight - portafilter_weight).abs();

                if weight < PORTAFILTER_THRESHOLD {
                    // Portafilter removed during stabilization
                    info!("Portafilter removed during stabilization - waiting for placement");
                    GrinderState::WaitingForPortafilter {}
                } else if weight_diff > WEIGHT_STABILITY_TOLERANCE && sample_count >= 5 {
                    warn!(
                        "Weight unstable during stabilization (weight: {}g, portafilter_weight: {}g, diff: {}g) - restarting stabilization",
                        weight, portafilter_weight,
                        weight_diff
                    );
                    // TODO(mkiefel): Make this dependent on sample count.
                    // Weight changed significantly, restart stabilization.
                    GrinderState::Stabilizing {
                        start_time: Instant::now(),
                        portafilter_weight: weight,
                        sample_count: 1,
                    }
                } else if Instant::now() - start_time >= STABILIZATION_TIME {
                    // TODO(mkiefel): Make this dependent on sample count.
                    info!("Weight stabilized at {}g - starting grind!", weight);
                    self.grinder.set_low();
                    GrinderState::Grinding {
                        start_time: Instant::now(),
                        portafilter_weight,
                    }
                } else {
                    GrinderState::Stabilizing {
                        start_time,
                        portafilter_weight: (sample_count as f32) / (sample_count as f32 + 1.0)
                            * portafilter_weight
                            + weight / (sample_count as f32 + 1.0),
                        sample_count: sample_count + 1,
                    }
                }
            }

            GrinderState::Grinding {
                start_time,
                portafilter_weight,
            } => {
                let coffee_weight = weight - portafilter_weight;

                info!(
                    "Grinding... Coffee: {}g (Total: {}g)",
                    (coffee_weight * 10.0).floor() / 10.0,
                    weight
                );

                if coffee_weight >= TARGET_COFFEE_WEIGHT
                    || Instant::now() - start_time
                        >= Duration::from_secs(MAX_GRIND_TIME_IN_SECS as u64)
                {
                    info!(
                        "Target reached! {}g coffee ground - stopping grinder",
                        (coffee_weight * 10.0).floor() / 10.0
                    );
                    self.grinder.set_high();
                    GrinderState::WaitingForRemoval {}
                } else {
                    GrinderState::Grinding {
                        start_time,
                        portafilter_weight,
                    }
                }
            }

            GrinderState::WaitingForRemoval {} => {
                if weight < REMOVAL_THRESHOLD {
                    info!("Portafilter removed - ready for next cycle");
                    GrinderState::WaitingForPortafilter {}
                } else {
                    GrinderState::WaitingForRemoval {}
                }
            }
        })
    }
}

const WEIGHT_BATCH_CHANNEL_SIZE: usize = 2;

#[embassy_executor::task]
async fn websocket_broadcaster_task(
    mut state_receiver: watch::Receiver<'static, CriticalSectionRawMutex, UserEvent, 2>,
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

async fn controller_task(
    scale_receiver: channel::Receiver<'static, CriticalSectionRawMutex, f32, SCALE_CHANNEL_SIZE>,
    state_sender: watch::Sender<'static, CriticalSectionRawMutex, UserEvent, 2>,
    weight_batch_sender: channel::Sender<
        'static,
        CriticalSectionRawMutex,
        heapless::Vec<WeightReading, 4>,
        WEIGHT_BATCH_CHANNEL_SIZE,
    >,
    grinder_state_machine: &mutex::Mutex<CriticalSectionRawMutex, GrinderStateMachine>,
) {
    let mut last_event = {
        let grinder_state_machine_guard = grinder_state_machine.lock().await;
        let event = grinder_state_machine_guard.as_user_event();
        state_sender.send(event);
        event
    };

    let mut weight_batch = heapless::Vec::<WeightReading, 4>::new();
    const BATCH_INTERVAL: Duration = Duration::from_millis(100);

    loop {
        // Try to receive a weight reading with timeout.
        let raw_weight_opt =
            embassy_time::with_timeout(BATCH_INTERVAL, scale_receiver.receive()).await;

        if let Ok(raw_weight) = raw_weight_opt {
            let (event, weight, coffee_weight) = {
                let mut grinder_state_machine_guard = grinder_state_machine.lock().await;
                grinder_state_machine_guard.update_weight(raw_weight);
                let event = grinder_state_machine_guard.as_user_event();
                let weight = grinder_state_machine_guard
                    .scale_setting
                    .translate(raw_weight);
                let coffee_weight = grinder_state_machine_guard.get_coffee_weight(weight);
                (event, weight, coffee_weight)
            };
            if event != last_event {
                last_event = event;
                state_sender.send(event);
            }

            // Add weight reading to batch
            let reading = WeightReading {
                timestamp_ms: Instant::now().as_millis(),
                weight,
                state: event,
                coffee_weight,
            };
            let _ = weight_batch.push(reading);
        }

        if weight_batch.is_full() {
            weight_batch_sender.try_send(weight_batch.clone()).ok();
            weight_batch.clear();
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        RM2_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawner.must_spawn(cyw43_task(runner));
    control.init(clm).await;

    let stack = bringup_network_stack(&spawner, net_device);
    control.start_ap_wpa2("grindy", "grindyrockz", 8).await;

    let grinder = Output::new(p.PIN_0, Level::High);

    let dt = Input::new(p.PIN_17, Pull::Down);
    let sck = Output::new(p.PIN_16, Level::Low);

    static SCALE_CHANNEL: channel::Channel<CriticalSectionRawMutex, f32, SCALE_CHANNEL_SIZE> =
        channel::Channel::new();
    spawner.must_spawn(scale_task(sck, dt, SCALE_CHANNEL.sender()));

    static STATE_WATCH: watch::Watch<CriticalSectionRawMutex, UserEvent, 2> = watch::Watch::new();
    spawner.must_spawn(led_task(control, unwrap!(STATE_WATCH.receiver())));

    static GRINDER_STATE_MACHINE: StaticCell<
        mutex::Mutex<CriticalSectionRawMutex, GrinderStateMachine>,
    > = StaticCell::new();
    let grinder_state_machine: &'static mutex::Mutex<CriticalSectionRawMutex, GrinderStateMachine> =
        GRINDER_STATE_MACHINE.init(mutex::Mutex::new(GrinderStateMachine::new(grinder)));

    static WS_REGISTRY: StaticCell<mutex::Mutex<CriticalSectionRawMutex, WsConnectionRegistry>> =
        StaticCell::new();
    let ws_registry = WS_REGISTRY.init(mutex::Mutex::new(WsConnectionRegistry::new()));

    static WEIGHT_BATCH_CHANNEL: channel::Channel<
        CriticalSectionRawMutex,
        heapless::Vec<WeightReading, 4>,
        WEIGHT_BATCH_CHANNEL_SIZE,
    > = channel::Channel::new();

    spawner.must_spawn(websocket_broadcaster_task(
        unwrap!(STATE_WATCH.receiver()),
        WEIGHT_BATCH_CHANNEL.receiver(),
        ws_registry,
    ));

    bringup_web_server(&spawner, stack, &grinder_state_machine, ws_registry);

    info!("Hello, coffee world!");
    controller_task(
        SCALE_CHANNEL.receiver(),
        STATE_WATCH.sender(),
        WEIGHT_BATCH_CHANNEL.sender(),
        &grinder_state_machine,
    )
    .await;
}
