use core::f32;
use defmt::*;
use embassy_rp::gpio::{Input, Output};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel, mutex, watch};
use embassy_time::{Delay, Duration, Instant, Timer};
use loadcell::{hx711::GainMode, LoadCell};
use num_traits::float::FloatCore;
use serde::Serialize;

use crate::ui::{UserEvent, USER_EVENT_CHANNEL_SIZE};

pub const WEIGHT_BATCH_CHANNEL_SIZE: usize = 2;
pub const SCALE_CHANNEL_SIZE: usize = 5;

#[derive(Serialize, Clone, Copy)]
pub struct WeightReading {
    timestamp_ms: u64,
    weight: f32,
    state: UserEvent,
    coffee_weight: Option<f32>,
}

impl WeightReading {
    pub fn new(
        timestamp_ms: u64,
        weight: f32,
        state: UserEvent,
        coffee_weight: Option<f32>,
    ) -> Self {
        Self {
            timestamp_ms,
            weight,
            state,
            coffee_weight,
        }
    }
}

#[derive(Clone, Serialize)]
pub struct ScaleSetting {
    offset: f32,
    inv_variance: f32,
    factor: f32,
}

impl ScaleSetting {
    fn translate(&self, raw: f32) -> f32 {
        (raw - self.offset) * self.factor
    }
}

/// Compute mean and variance in place with additional removal of outlier based on Median Absolute
/// Deviation (MAD).
fn compute_mean_variance<const N: usize>(
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

#[embassy_executor::task]
pub async fn scale_task(
    sck: Output<'static>,
    dt: Input<'static>,
    sender: channel::Sender<'static, CriticalSectionRawMutex, f32, SCALE_CHANNEL_SIZE>,
) {
    debug!("Setting up scale...");
    let delay = Delay {};
    let mut scale = loadcell::hx711::HX711::new(sck, dt, delay);
    scale.set_gain_mode(GainMode::A128);

    loop {
        if scale.is_ready() {
            match scale.read() {
                Ok(r) => {
                    sender.send(-r as f32).await;
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

const MAX_GRIND_TIME_IN_SECS: usize = 50;

pub struct GrinderStateMachine {
    grinder: Output<'static>,
    scale_setting: ScaleSetting,
    state: Option<GrinderState>,
}

const CALIBRATION_SAMPLE_COUNT: usize = 25;
const SAMPLE_COUNT: usize = 15;

enum GrinderState {
    Tare {
        samples: heapless::Vec<f32, CALIBRATION_SAMPLE_COUNT>,
    },
    WaitingForCalibration {},
    Calibrating {
        samples: heapless::Vec<f32, CALIBRATION_SAMPLE_COUNT>,
    },
    WaitingForPortafilter {},
    Stabilizing {
        samples: heapless::Vec<f32, SAMPLE_COUNT>,
    },
    Grinding {
        start_time: Instant,
        portafilter_weight: f32,
    },
    WaitingForRemoval {
        portafilter_weight: f32,
    },
}

impl GrinderStateMachine {
    pub fn new(grinder: Output<'static>) -> Self {
        Self {
            grinder,
            scale_setting: ScaleSetting {
                offset: 0.0,
                inv_variance: 0.0,
                factor: 200.0 / 85314.55 * 0.478242 * 1.049868,
            },
            state: Some(GrinderState::Tare {
                samples: heapless::Vec::new(),
            }),
        }
    }

    pub fn as_user_event(&self) -> UserEvent {
        match self.state.as_ref().unwrap() {
            GrinderState::Tare { .. } => UserEvent::Initializing,

            GrinderState::WaitingForPortafilter {} | GrinderState::WaitingForCalibration { .. } => {
                UserEvent::Idle
            }
            GrinderState::Calibrating { .. } => UserEvent::Stabilizing,
            GrinderState::Stabilizing { .. } => UserEvent::Stabilizing,
            GrinderState::Grinding { .. } => UserEvent::Grinding,
            GrinderState::WaitingForRemoval { .. } => UserEvent::WaitingForRemoval,
        }
    }

    pub fn get_scale_setting(&self) -> &ScaleSetting {
        &self.scale_setting
    }

    fn get_coffee_weight(&self, current_weight: f32) -> Option<f32> {
        match self.state.as_ref().unwrap() {
            GrinderState::Grinding {
                portafilter_weight, ..
            } => Some(current_weight - portafilter_weight),
            GrinderState::WaitingForRemoval {
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
                    GrinderState::WaitingForPortafilter {}
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
                    GrinderState::WaitingForRemoval {
                        portafilter_weight: 0.0,
                    }
                } else {
                    samples.push(weight).unwrap();
                    GrinderState::Calibrating { samples }
                }
            }

            GrinderState::WaitingForPortafilter {} => {
                if weight > PORTAFILTER_THRESHOLD {
                    info!("Portafilter detected! Weight: {}g - stabilizing...", weight);

                    GrinderState::Stabilizing {
                        samples: heapless::Vec::from_slice(&[weight]).unwrap(),
                    }
                } else {
                    GrinderState::WaitingForPortafilter {}
                }
            }

            GrinderState::Stabilizing { mut samples } => {
                samples.push(weight).unwrap();

                let (portafilter_weight, _) =
                    compute_mean_variance(&mut samples, 3.0).unwrap_or((weight, 0.0));

                let threshold = f32::math::sqrt(
                    1.0 / self.scale_setting.inv_variance * self.scale_setting.factor.powi(2),
                ) * 3.0;

                if weight < PORTAFILTER_THRESHOLD {
                    // Portafilter removed during stabilization
                    info!("Portafilter removed during stabilization - waiting for placement");
                    GrinderState::WaitingForPortafilter {}
                } else if samples.len() > 5 && (weight - portafilter_weight).abs() > threshold {
                    warn!(
                        "Weight unstable during stabilization (weight: {}g, portafilter_weight: {}g, threshold: {}g) - restarting stabilization",
                        weight, portafilter_weight, threshold

                    );
                    GrinderState::Stabilizing {
                        samples: heapless::Vec::from_slice(&[weight]).unwrap(),
                    }
                } else if samples.is_full() {
                    // TODO(mkiefel): Make this dependent on sample count.
                    info!("Weight stabilized at {}g - starting grind!", weight);
                    self.grinder.set_low();
                    GrinderState::Grinding {
                        start_time: Instant::now(),
                        portafilter_weight,
                    }
                } else {
                    GrinderState::Stabilizing { samples }
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
                    GrinderState::WaitingForRemoval { portafilter_weight }
                } else {
                    GrinderState::Grinding {
                        start_time,
                        portafilter_weight,
                    }
                }
            }

            GrinderState::WaitingForRemoval { portafilter_weight } => {
                if weight < REMOVAL_THRESHOLD {
                    info!("Portafilter removed - ready for next cycle");
                    GrinderState::WaitingForPortafilter {}
                } else {
                    GrinderState::WaitingForRemoval { portafilter_weight }
                }
            }
        })
    }
}

pub async fn controller_task(
    scale_receiver: channel::Receiver<'static, CriticalSectionRawMutex, f32, SCALE_CHANNEL_SIZE>,
    state_sender: watch::Sender<
        'static,
        CriticalSectionRawMutex,
        UserEvent,
        USER_EVENT_CHANNEL_SIZE,
    >,
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
            let reading =
                WeightReading::new(Instant::now().as_millis(), weight, event, coffee_weight);
            let _ = weight_batch.push(reading);
        }

        if weight_batch.is_full() {
            weight_batch_sender.try_send(weight_batch.clone()).ok();
            weight_batch.clear();
        }
    }
}
