#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{self, Pull};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Delay, Duration, Instant, Timer};
use gpio::{Input, Level, Output};
use loadcell::{hx711::GainMode, LoadCell};
use num_traits::float::FloatCore;
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

#[derive(Debug, Clone, Copy, PartialEq)]
enum GrinderState {
    WaitingForPortafilter,
    Stabilizing,
    Grinding,
    WaitingForRemoval,
}

static STATE_WATCH: Watch<CriticalSectionRawMutex, GrinderState, 1> = Watch::new();

#[embassy_executor::task]
async fn led_task(mut led: Output<'static>) {
    let mut state_receiver = STATE_WATCH.receiver().unwrap();
    loop {
        match state_receiver.get().await {
            GrinderState::WaitingForPortafilter => {
                led.set_high();
                Timer::after(Duration::from_millis(1000)).await;
                led.set_low();
                Timer::after(Duration::from_millis(1000)).await;
            }
            GrinderState::Stabilizing => {
                led.set_high();
                Timer::after(Duration::from_millis(250)).await;
                led.set_low();
                Timer::after(Duration::from_millis(250)).await;
            }
            GrinderState::Grinding => {
                led.set_high();
                state_receiver.changed().await;
            }
            GrinderState::WaitingForRemoval => {
                led.set_high();
                Timer::after(Duration::from_millis(100)).await;
                led.set_low();
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut grinder = Output::new(p.PIN_0, Level::High);

    let led = Output::new(p.PIN_25, Level::Low);

    let dt = Input::new(p.PIN_17, Pull::Down);
    let sck = Output::new(p.PIN_16, Level::Low);
    let delay = Delay {};

    let mut scale = loadcell::hx711::HX711::new(sck, dt, delay);
    scale.set_gain_mode(GainMode::A64);

    // The initial tare seems to be a bit off. Just throw it away.
    scale.tare(10);
    scale.tare(20);
    // scale.set_scale(1.0);
    // Get some reference weight and adjust the scale, with something like <reference_weight> /
    // <values that you get with scaling 1>. E.g.,
    scale.set_scale(403.0 / 180919.2);

    let mut buffer = heapless::HistoryBuffer::<_, 2>::new();
    let mut state = GrinderState::WaitingForPortafilter;
    let mut stabilization_start: Option<Instant> = None;
    let mut grind_start: Option<Instant> = None;
    let mut portafilter_weight = 0.0f32;

    // Minimum weight to detect portafilter placement.
    const PORTAFILTER_THRESHOLD: f32 = 100.0;
    // Weight below which we consider portafilter removed.
    const REMOVAL_THRESHOLD: f32 = 10.0;
    // Target coffee weight in grams.
    const TARGET_COFFEE_WEIGHT: f32 = 18.0;
    // Time to wait for weight stabilization.
    const STABILIZATION_TIME: Duration = Duration::from_secs(2);
    // Tolerance for weight stability (grams).
    const WEIGHT_STABILITY_TOLERANCE: f32 = 0.5;

    spawner.spawn(led_task(led)).unwrap();
    let state_sender = STATE_WATCH.sender();
    state_sender.send(state);

    info!("Hello, coffee world!");
    info!("Grindy is ready - waiting for portafilter...");

    loop {
        if scale.is_ready() {
            let reading = scale.read_scaled().unwrap();
            buffer.write(reading);

            let avg_weight = (buffer.iter().sum::<f32>() / (buffer.len() as f32)).abs();
            let rounded_weight = (avg_weight * 10.0).floor() / 10.0;

            debug!("Weight: {}g", rounded_weight);

            match state {
                GrinderState::WaitingForPortafilter => {
                    if avg_weight > PORTAFILTER_THRESHOLD {
                        info!(
                            "Portafilter detected! Weight: {}g - stabilizing...",
                            rounded_weight
                        );
                        state = GrinderState::Stabilizing;
                        state_sender.send(state);
                        stabilization_start = Some(Instant::now());
                        portafilter_weight = avg_weight;
                    }
                }

                GrinderState::Stabilizing => {
                    // Check if weight is stable (within tolerance of initial portafilter weight).
                    let weight_diff = (avg_weight - portafilter_weight).abs();

                    if avg_weight < PORTAFILTER_THRESHOLD {
                        // Portafilter removed during stabilization
                        info!("Portafilter removed during stabilization - waiting for placement");
                        state = GrinderState::WaitingForPortafilter;
                        state_sender.send(state);
                        stabilization_start = None;
                        portafilter_weight = 0.0;
                    } else if weight_diff > WEIGHT_STABILITY_TOLERANCE {
                        // Weight changed significantly, restart stabilization.
                        stabilization_start = Some(Instant::now());
                        portafilter_weight = avg_weight;
                        info!(
                            "Weight changing, restarting stabilization: {}g",
                            rounded_weight
                        );
                    } else if let Some(start_time) = stabilization_start {
                        if Instant::now() - start_time >= STABILIZATION_TIME {
                            info!("Weight stabilized at {}g - starting grind!", rounded_weight);
                            state = GrinderState::Grinding;
                            state_sender.send(state);
                            grinder.set_low();
                            grind_start = Some(Instant::now());
                        }
                    }
                }

                GrinderState::Grinding => {
                    let coffee_weight = avg_weight - portafilter_weight;
                    info!(
                        "Grinding... Coffee: {}g (Total: {}g)",
                        (coffee_weight * 10.0).floor() / 10.0,
                        rounded_weight
                    );

                    if coffee_weight >= TARGET_COFFEE_WEIGHT
                        || grind_start
                            .map_or(false, |t| Instant::now() - t >= Duration::from_secs(50))
                    {
                        info!(
                            "Target reached! {}g coffee ground - stopping grinder",
                            (coffee_weight * 10.0).floor() / 10.0
                        );
                        grinder.set_high();
                        grind_start = None;
                        state = GrinderState::WaitingForRemoval;
                        state_sender.send(state);
                        info!(
                            "Waiting for portafilter removal... Current weight: {}g",
                            rounded_weight
                        );
                    }
                }

                GrinderState::WaitingForRemoval => {
                    if avg_weight < REMOVAL_THRESHOLD {
                        info!("Portafilter removed - ready for next cycle");
                        state = GrinderState::WaitingForPortafilter;
                        state_sender.send(state);
                        stabilization_start = None;
                        portafilter_weight = 0.0;
                    }
                }
            }
        }
        // Let other tasks run.
        Timer::after(Duration::from_millis(50)).await;
    }
}
