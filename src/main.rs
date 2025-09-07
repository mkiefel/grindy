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
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel, watch};
use embassy_time::{Delay, Duration, Instant, Timer};
use gpio::{Input, Level, Output};
use loadcell::{hx711::GainMode, LoadCell};
use num_traits::float::FloatCore;
use picoserve::routing::get;
use picoserve::{make_static, AppBuilder, AppRouter};
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

struct AppProps;

impl AppBuilder for AppProps {
    type PathRouter = impl picoserve::routing::PathRouter;

    fn build_app(self) -> picoserve::Router<Self::PathRouter> {
        picoserve::Router::new().route("/", get(|| async move { "Hello World" }))
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
        // TODO: Implement this with the proper predicate to wait for HX711 readiness.
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum UserEvent {
    Idle,
    Stabilizing,
    Grinding,
    WaitingForRemoval,
}

#[embassy_executor::task]
async fn led_task(
    mut control: Control<'static>,
    mut state_receiver: watch::Receiver<'static, CriticalSectionRawMutex, UserEvent, 1>,
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

fn bringup_web_server(spawner: &Spawner, stack: embassy_net::Stack<'static>) {
    let app = make_static!(AppRouter<AppProps>, AppProps.build_app());
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
    factor: f32,
}

impl ScaleSetting {
    fn translate(&self, raw: f32) -> f32 {
        (raw - self.offset) * self.factor
    }
}

struct GrinderStateMachine {
    grinder: Output<'static>,
    scale_setting: ScaleSetting,
    state: GrinderState,
}

enum GrinderState {
    Tare {
        mean_offset: f32,
        sample_count: usize,
    },
    WaitingForCalibration,
    Calibrating {
        mean_weight: f32,
        sample_count: usize,
    },
    WaitingForPortafilter,
    Stabilizing {
        start_time: Instant,
        portafilter_weight: f32,
        sample_count: usize,
    },
    Grinding {
        start_time: Instant,
        portafilter_weight: f32,
    },
    WaitingForRemoval,
}

impl GrinderStateMachine {
    fn as_user_event(&self) -> UserEvent {
        match self.state {
            GrinderState::Tare { .. } | GrinderState::WaitingForPortafilter => UserEvent::Idle,
            GrinderState::WaitingForCalibration {} => UserEvent::Idle,
            GrinderState::Calibrating { .. } => UserEvent::Stabilizing,
            GrinderState::Stabilizing { .. } => UserEvent::Stabilizing,
            GrinderState::Grinding { .. } => UserEvent::Grinding,
            GrinderState::WaitingForRemoval {} => UserEvent::WaitingForRemoval,
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
        self.state = match self.state {
            GrinderState::Tare {
                mean_offset,
                sample_count,
            } => {
                // TODO(mkiefel): Estimate variance here as well to use it later for the estimation
                // of weights.
                let new_mean_offset = (mean_offset * (sample_count as f32) + raw_weight)
                    / (sample_count as f32 + 1.0);
                if sample_count >= 30 {
                    info!("Tare complete. Offset: {}", new_mean_offset);
                    self.scale_setting.offset = new_mean_offset;
                    GrinderState::WaitingForCalibration {}
                } else {
                    GrinderState::Tare {
                        mean_offset: new_mean_offset,
                        sample_count: sample_count + 1,
                    }
                }
            }

            GrinderState::WaitingForCalibration => {
                if weight > CALIBRATION_WEIGHT * 0.8 {
                    info!(
                        "Known weight detected: {}g - starting calibration...",
                        weight
                    );
                    GrinderState::Calibrating {
                        mean_weight: weight,
                        sample_count: 1,
                    }
                } else {
                    GrinderState::WaitingForCalibration
                }
            }

            GrinderState::Calibrating {
                mean_weight,
                sample_count,
            } => {
                let new_mean_weight =
                    (mean_weight * (sample_count as f32) + weight) / (sample_count as f32 + 1.0);
                if sample_count >= 200 {
                    let factor = CALIBRATION_WEIGHT / new_mean_weight;
                    info!(
                        "Calibration complete. Offset: {}g, Factor: {}",
                        new_mean_weight, factor
                    );
                    self.scale_setting.factor *= factor;
                    GrinderState::WaitingForRemoval {}
                } else {
                    GrinderState::Calibrating {
                        mean_weight: new_mean_weight,
                        sample_count: sample_count + 1,
                    }
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
                } else if weight_diff > WEIGHT_STABILITY_TOLERANCE {
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
                    || Instant::now() - start_time >= Duration::from_secs(50)
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
        }
    }
}

async fn controller_task(
    scale_receiver: channel::Receiver<'static, CriticalSectionRawMutex, f32, SCALE_CHANNEL_SIZE>,
    state_sender: watch::Sender<'static, CriticalSectionRawMutex, UserEvent, 1>,
    grinder: Output<'static>,
) {
    let mut grinder_state_machine = GrinderStateMachine {
        grinder,
        scale_setting: ScaleSetting {
            offset: 0.0,
            factor: 200.0 / 85314.55,
        },
        state: GrinderState::Tare {
            mean_offset: 0.0,
            sample_count: 0,
        },
    };

    state_sender.send(grinder_state_machine.as_user_event());

    loop {
        let raw_weight = scale_receiver.receive().await;
        grinder_state_machine.update_weight(raw_weight);
        state_sender.send(grinder_state_machine.as_user_event());
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

    static STATE_WATCH: watch::Watch<CriticalSectionRawMutex, UserEvent, 1> = watch::Watch::new();
    spawner.must_spawn(led_task(control, unwrap!(STATE_WATCH.receiver())));

    bringup_web_server(&spawner, stack);

    info!("Hello, coffee world!");
    controller_task(SCALE_CHANNEL.receiver(), STATE_WATCH.sender(), grinder).await;
}
