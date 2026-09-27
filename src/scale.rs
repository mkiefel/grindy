use core::f32::math;
use defmt::*;
use embassy_rp::gpio::{Input, Output};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel, mutex, watch};
use embassy_time::{Delay, Duration, Instant, Timer};
use grindy_gp::lead_time;
use grindy_gp::{GrindEstimator, DEFAULT_LEAD_TIME, FITTED, RAMP_UP};
use loadcell::{hx711::GainMode, LoadCell};
use num_traits::float::FloatCore;
use serde::Serialize;

use crate::storage::{self, SharedFlash};
use crate::ui::{UserEvent, GRIND_PROGRESS_CHANNEL_SIZE, USER_EVENT_CHANNEL_SIZE};

pub const CONTROLLER_EVENT_CHANNEL_SIZE: usize = 4;
pub const SCALE_CHANNEL_SIZE: usize = 5;

/// A raw scale reading and when it was taken.
pub type ScaleSample = (Instant, f32);

/// Seconds until the grinder is expected to stop (10/50/90 % quantiles).
#[derive(Serialize, Clone, Copy)]
pub struct EtaReading {
    median: f32,
    lo: f32,
    hi: f32,
}

impl From<grindy_gp::Eta> for EtaReading {
    fn from(eta: grindy_gp::Eta) -> Self {
        Self {
            median: eta.median,
            lo: eta.lo,
            hi: eta.hi,
        }
    }
}

#[derive(Serialize, Clone, Copy)]
pub struct WeightReading {
    timestamp_ms: u64,
    weight: f32,
    state: UserEvent,
    coffee_weight: Option<f32>,
    /// Coffee weight estimated by the GP while grinding.
    filtered_weight: Option<f32>,
    eta: Option<EtaReading>,
}

impl WeightReading {
    pub fn new(
        timestamp_ms: u64,
        weight: f32,
        state: UserEvent,
        coffee_weight: Option<f32>,
        filtered_weight: Option<f32>,
        eta: Option<EtaReading>,
    ) -> Self {
        Self {
            timestamp_ms,
            weight,
            state,
            coffee_weight,
            filtered_weight,
            eta,
        }
    }
}

/// Why the grinder was stopped. The order is part of the wire format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Format)]
pub enum StopReason {
    /// The estimator forecast the target after the lead time.
    Prediction,
    /// The reading itself reached the target.
    RawWeight,
    /// `MAX_GRIND_TIME_IN_SECS` passed.
    Timeout,
}

/// Summary of a finished grind, sent once the weight settled (or the
/// portafilter was removed before).
#[derive(Serialize, Clone, Copy)]
pub struct GrindFinished {
    stop_reason: StopReason,
    /// Coffee weight reading when the grinder stopped.
    stop_weight: f32,
    /// Coffee weight after the flow stopped, if it could be measured.
    settled_weight: Option<f32>,
    /// Lead time this grind had, if it was plausible and used for learning.
    lead_time_observed: Option<f32>,
    /// Lead time after learning from this grind.
    lead_time: f32,
}

/// What the controller reports to the web clients.
#[derive(Clone, Copy)]
pub enum ControllerEvent {
    Reading(WeightReading),
    GrindFinished(GrindFinished),
}

/// Window after the stop in which the settled weight is measured.
const SETTLE_START: Duration = Duration::from_millis(1500);
const SETTLE_END: Duration = Duration::from_millis(2500);
const SETTLE_SAMPLE_COUNT: usize = 16;
/// Fewer readings in the settle window don't give a settled weight.
const MIN_SETTLE_SAMPLES: usize = 5;
/// Lead time changes smaller than this are not written to flash.
const LEAD_TIME_STORE_THRESHOLD: f32 = 0.01;

/// Measures the settled weight after a stop to learn the lead time.
struct Settle {
    stop_time: Instant,
    stop_reason: StopReason,
    stop_weight: f32,
    estimator: GrindEstimator,
    /// Coffee weight readings in the settle window.
    samples: heapless::Vec<f32, SETTLE_SAMPLE_COUNT>,
}

impl Settle {
    fn finish(
        &self,
        settled_weight: Option<f32>,
        lead_time_observed: Option<f32>,
        lead_time: f32,
    ) -> GrindFinished {
        GrindFinished {
            stop_reason: self.stop_reason,
            stop_weight: self.stop_weight,
            settled_weight,
            lead_time_observed,
            lead_time,
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
    sender: channel::Sender<'static, CriticalSectionRawMutex, ScaleSample, SCALE_CHANNEL_SIZE>,
) {
    debug!("Setting up scale...");
    let delay = Delay {};
    let mut scale = loadcell::hx711::HX711::new(sck, dt, delay);
    scale.set_gain_mode(GainMode::A128);

    loop {
        if scale.is_ready() {
            match scale.read() {
                Ok(r) => {
                    sender.send((Instant::now(), -r as f32)).await;
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
    flash: SharedFlash,
    scale_setting: ScaleSetting,
    /// Coffee weight in grams to grind to.
    target_weight: f32,
    /// Seconds of flow still arriving after the grinder stops.
    lead_time: f32,
    /// Lead time last persisted to flash.
    stored_lead_time: f32,
    state: Option<GrinderState>,
}

/// Target coffee weight in grams used until one is stored in flash.
const DEFAULT_TARGET_WEIGHT: f32 = 18.0;

/// Range of accepted target coffee weights in grams.
pub const MIN_TARGET_WEIGHT: f32 = 1.0;
pub const MAX_TARGET_WEIGHT: f32 = 100.0;

/// Factor derived from a manual calibration against a known weight, used
/// until a calibration is stored in flash.
const DEFAULT_FACTOR: f32 = 200.0 / 85314.55 * 0.478242 * 1.049868 / 50.3 * 48.0;

const CALIBRATION_SAMPLE_COUNT: usize = 25;
const SAMPLE_COUNT: usize = 15;

/// Fraction of the reference weight above which we consider the calibration
/// weight placed. Deliberately generous so that a badly off stored factor
/// still detects the weight.
const CALIBRATION_DETECTION_FRACTION: f32 = 0.5;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum CalibrationError {
    /// Calibration can only be started while idle.
    Busy,
    /// Calibration was not in progress.
    NotCalibrating,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum TargetWeightError {
    /// The weight is outside [`MIN_TARGET_WEIGHT`]..=[`MAX_TARGET_WEIGHT`].
    OutOfRange,
    /// Persisting the weight to flash failed.
    Storage,
}

enum GrinderState {
    Tare {
        samples: heapless::Vec<f32, CALIBRATION_SAMPLE_COUNT>,
        /// Reference weight in grams to calibrate against once taring is done.
        calibration_weight: Option<f32>,
    },
    WaitingForCalibration {
        calibration_weight: f32,
    },
    Calibrating {
        calibration_weight: f32,
        samples: heapless::Vec<f32, CALIBRATION_SAMPLE_COUNT>,
    },
    WaitingForPortafilter {},
    Stabilizing {
        samples: heapless::Vec<f32, SAMPLE_COUNT>,
    },
    Grinding {
        start_time: Instant,
        portafilter_weight: f32,
        estimator: GrindEstimator,
    },
    WaitingForRemoval {
        portafilter_weight: f32,
        /// Present after a grind until the settle window has passed.
        settle: Option<Settle>,
    },
}

impl GrinderStateMachine {
    pub fn new(grinder: Output<'static>, flash: SharedFlash) -> Self {
        let factor = flash
            .lock(|flash| storage::read_calibration_factor(&mut flash.borrow_mut()))
            .unwrap_or(DEFAULT_FACTOR);
        let target_weight = flash
            .lock(|flash| storage::read_target_weight(&mut flash.borrow_mut()))
            .filter(|weight| (MIN_TARGET_WEIGHT..=MAX_TARGET_WEIGHT).contains(weight))
            .unwrap_or(DEFAULT_TARGET_WEIGHT);
        let lead_time = flash
            .lock(|flash| storage::read_lead_time(&mut flash.borrow_mut()))
            .unwrap_or(DEFAULT_LEAD_TIME);
        Self {
            grinder,
            flash,
            target_weight,
            lead_time,
            stored_lead_time: lead_time,
            scale_setting: ScaleSetting {
                offset: 0.0,
                inv_variance: 0.0,
                factor,
            },
            state: Some(GrinderState::Tare {
                samples: heapless::Vec::new(),
                calibration_weight: None,
            }),
        }
    }

    /// Starts calibrating against a known `calibration_weight` in grams. The
    /// scale is re-tared first, so it has to be empty.
    pub fn start_calibration(&mut self, calibration_weight: f32) -> Result<(), CalibrationError> {
        match self.state.as_ref().unwrap() {
            GrinderState::WaitingForPortafilter {} => {
                info!(
                    "Starting calibration with {}g reference weight - taring...",
                    calibration_weight
                );
                self.state = Some(GrinderState::Tare {
                    samples: heapless::Vec::new(),
                    calibration_weight: Some(calibration_weight),
                });
                Ok(())
            }
            _ => Err(CalibrationError::Busy),
        }
    }

    /// Aborts a calibration started with [`Self::start_calibration`] and keeps
    /// the previous factor.
    pub fn cancel_calibration(&mut self) -> Result<(), CalibrationError> {
        match self.state.as_mut().unwrap() {
            GrinderState::Tare {
                calibration_weight: calibration_weight @ Some(_),
                ..
            } => {
                // Let the tare finish, it is still useful.
                *calibration_weight = None;
            }
            GrinderState::WaitingForCalibration { .. } | GrinderState::Calibrating { .. } => {
                self.state = Some(GrinderState::WaitingForPortafilter {});
            }
            _ => return Err(CalibrationError::NotCalibrating),
        }
        info!("Calibration cancelled");
        Ok(())
    }

    pub fn as_user_event(&self) -> UserEvent {
        match self.state.as_ref().unwrap() {
            GrinderState::Tare { .. } => UserEvent::Initializing,

            GrinderState::WaitingForPortafilter {} => UserEvent::Idle,
            GrinderState::WaitingForCalibration { .. } => UserEvent::WaitingForCalibration,
            GrinderState::Calibrating { .. } => UserEvent::Calibrating,
            GrinderState::Stabilizing { .. } => UserEvent::Stabilizing,
            GrinderState::Grinding { .. } => UserEvent::Grinding,
            GrinderState::WaitingForRemoval { .. } => UserEvent::WaitingForRemoval,
        }
    }

    pub fn get_scale_setting(&self) -> &ScaleSetting {
        &self.scale_setting
    }

    pub fn get_target_weight(&self) -> f32 {
        self.target_weight
    }

    pub fn get_lead_time(&self) -> f32 {
        self.lead_time
    }

    /// Largest deviation from the mean weight (3 sd of the scale noise) that
    /// still counts as stable.
    fn stability_threshold(&self) -> f32 {
        math::sqrt(1.0 / self.scale_setting.inv_variance * self.scale_setting.factor.powi(2)) * 3.0
    }

    /// Sets the coffee weight in grams to grind to and persists it. Takes
    /// effect immediately, even during a running grind.
    pub fn set_target_weight(&mut self, weight: f32) -> Result<(), TargetWeightError> {
        if !(MIN_TARGET_WEIGHT..=MAX_TARGET_WEIGHT).contains(&weight) {
            return Err(TargetWeightError::OutOfRange);
        }
        if !self
            .flash
            .lock(|flash| storage::write_target_weight(&mut flash.borrow_mut(), weight))
        {
            return Err(TargetWeightError::Storage);
        }
        info!("Target weight set to {}g", weight);
        self.target_weight = weight;
        Ok(())
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

    /// Estimated coffee weight and time until the grinder is expected to
    /// stop, while grinding and once the estimator has enough readings.
    fn grind_estimate(&self) -> Option<(f32, Option<grindy_gp::Eta>)> {
        match self.state.as_ref().unwrap() {
            GrinderState::Grinding { estimator, .. }
                if estimator.updates() >= lead_time::MIN_UPDATES_FOR_STOP =>
            {
                Some((
                    estimator.weight().mean,
                    lead_time::stop_eta(estimator, self.lead_time, self.target_weight),
                ))
            }
            _ => None,
        }
    }

    /// Measures the settled weight of a finished settle window and learns the
    /// lead time from it if the grind is plausible.
    fn learn_lead_time(&mut self, mut settle: Settle) -> GrindFinished {
        let max_deviation = self.stability_threshold();
        let enough_samples = settle.samples.len() >= MIN_SETTLE_SAMPLES;
        let settled_weight = compute_mean_variance(&mut settle.samples, 3.0)
            .map(|(mean, _)| mean)
            .filter(|&mean| {
                enough_samples
                    && settle
                        .samples
                        .iter()
                        .all(|&weight| (weight - mean).abs() <= max_deviation)
            });
        let lead_time_observed = match (settle.stop_reason, settled_weight) {
            (StopReason::Timeout, _) | (_, None) => None,
            (_, Some(settled_weight)) => {
                lead_time::observe_lead_time(&settle.estimator, settled_weight)
            }
        };

        match lead_time_observed {
            Some(observed) => {
                let lead_time = lead_time::update_lead_time(self.lead_time, observed);
                info!(
                    "Settled at {}g, lead time observed {}s, now {}s",
                    settled_weight, observed, lead_time
                );
                self.lead_time = lead_time;
                if (lead_time - self.stored_lead_time).abs() > LEAD_TIME_STORE_THRESHOLD
                    && self
                        .flash
                        .lock(|flash| storage::write_lead_time(&mut flash.borrow_mut(), lead_time))
                {
                    self.stored_lead_time = lead_time;
                }
            }
            None => {
                // Recompute what the estimator would have said the lead time
                // was, purely for the log: `observe_lead_time` already threw
                // this number away if it was out of range.
                let raw_lead_time = settled_weight.and_then(|weight| settle.estimator.eta(weight));
                info!(
                    "Not learning the lead time from this grind (stop reason: {}, {} settle samples, settled weight: {}g, lead time observed: {}s)",
                    settle.stop_reason,
                    settle.samples.len(),
                    settled_weight,
                    raw_lead_time.map(|eta| eta.median),
                )
            }
        }
        settle.finish(settled_weight, lead_time_observed, self.lead_time)
    }

    fn update_weight(&mut self, time: Instant, raw_weight: f32) -> Option<GrindFinished> {
        // Minimum weight to detect portafilter placement.
        const PORTAFILTER_THRESHOLD: f32 = 100.0;
        // Weight below which we consider portafilter removed.
        const REMOVAL_THRESHOLD: f32 = 10.0;

        let weight = self.scale_setting.translate(raw_weight);
        debug!("weight: {}", weight);
        let mut finished = None;
        self.state = Some(match self.state.take().unwrap() {
            GrinderState::Tare {
                mut samples,
                calibration_weight,
            } => {
                if samples.is_full() {
                    let (new_mean_offset, variance) =
                        compute_mean_variance(&mut samples, 3.0).unwrap_or((0.0, 0.0));
                    info!(
                        "Tare complete. Offset: {}, Variance: {}",
                        new_mean_offset, variance
                    );
                    self.scale_setting.offset = new_mean_offset;
                    self.scale_setting.inv_variance = 1.0 / variance.max(1.0);
                    match calibration_weight {
                        Some(calibration_weight) => {
                            info!("Waiting for {}g calibration weight", calibration_weight);
                            GrinderState::WaitingForCalibration { calibration_weight }
                        }
                        None => GrinderState::WaitingForPortafilter {},
                    }
                } else {
                    samples.push(raw_weight).unwrap();
                    GrinderState::Tare {
                        samples,
                        calibration_weight,
                    }
                }
            }

            GrinderState::WaitingForCalibration { calibration_weight } => {
                if weight > calibration_weight * CALIBRATION_DETECTION_FRACTION {
                    info!(
                        "Known weight detected: {}g - starting calibration...",
                        weight
                    );
                    GrinderState::Calibrating {
                        calibration_weight,
                        samples: heapless::Vec::from_slice(&[weight]).unwrap(),
                    }
                } else {
                    GrinderState::WaitingForCalibration { calibration_weight }
                }
            }

            GrinderState::Calibrating {
                calibration_weight,
                mut samples,
            } => {
                samples.push(weight).unwrap();

                let (mean_weight, _) =
                    compute_mean_variance(&mut samples, 3.0).unwrap_or((weight, 0.0));

                let threshold = self.stability_threshold();

                if weight < calibration_weight * CALIBRATION_DETECTION_FRACTION {
                    info!("Calibration weight removed during calibration - waiting for placement");
                    GrinderState::WaitingForCalibration { calibration_weight }
                } else if samples.len() > 5 && (weight - mean_weight).abs() > threshold {
                    warn!(
                        "Weight unstable during calibration (weight: {}g, mean: {}g, threshold: {}g) - restarting",
                        weight, mean_weight, threshold
                    );
                    GrinderState::Calibrating {
                        calibration_weight,
                        samples: heapless::Vec::from_slice(&[weight]).unwrap(),
                    }
                } else if samples.is_full() {
                    let factor = calibration_weight / mean_weight;
                    self.scale_setting.factor *= factor;
                    info!(
                        "Calibration complete. Measured: {}g, correction: {}, new factor: {}",
                        mean_weight, factor, self.scale_setting.factor
                    );
                    let factor = self.scale_setting.factor;
                    self.flash.lock(|flash| {
                        storage::write_calibration_factor(&mut flash.borrow_mut(), factor)
                    });
                    GrinderState::WaitingForRemoval {
                        portafilter_weight: 0.0,
                        settle: None,
                    }
                } else {
                    GrinderState::Calibrating {
                        calibration_weight,
                        samples,
                    }
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

                let threshold = self.stability_threshold();

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
                        start_time: time,
                        portafilter_weight,
                        estimator: GrindEstimator::new(FITTED),
                    }
                } else {
                    GrinderState::Stabilizing { samples }
                }
            }

            GrinderState::Grinding {
                start_time,
                portafilter_weight,
                mut estimator,
            } => {
                let coffee_weight = weight - portafilter_weight;
                let elapsed = time.saturating_duration_since(start_time);
                let t = elapsed.as_micros() as f32 / 1_000_000.0;
                if t >= RAMP_UP {
                    estimator.update(t, coffee_weight);
                }

                info!(
                    "Grinding... Coffee: {}g (Total: {}g)",
                    (coffee_weight * 10.0).floor() / 10.0,
                    weight
                );

                let stop_reason = if lead_time::should_stop(
                    &estimator,
                    t,
                    self.lead_time,
                    self.target_weight,
                ) {
                    Some(StopReason::Prediction)
                } else if coffee_weight >= self.target_weight {
                    Some(StopReason::RawWeight)
                } else if elapsed >= Duration::from_secs(MAX_GRIND_TIME_IN_SECS as u64) {
                    Some(StopReason::Timeout)
                } else {
                    None
                };

                match stop_reason {
                    Some(stop_reason) => {
                        info!(
                            "Stopping grinder ({}) at {}g coffee after {}s, lead time {}s",
                            stop_reason, coffee_weight, t, self.lead_time
                        );
                        self.grinder.set_high();
                        GrinderState::WaitingForRemoval {
                            portafilter_weight,
                            settle: Some(Settle {
                                stop_time: time,
                                stop_reason,
                                stop_weight: coffee_weight,
                                estimator,
                                samples: heapless::Vec::new(),
                            }),
                        }
                    }
                    None => GrinderState::Grinding {
                        start_time,
                        portafilter_weight,
                        estimator,
                    },
                }
            }

            GrinderState::WaitingForRemoval {
                portafilter_weight,
                settle,
            } => {
                if weight < REMOVAL_THRESHOLD {
                    info!("Portafilter removed - ready for next cycle");
                    if let Some(settle) = settle {
                        info!("Removed before the weight settled - not learning the lead time");
                        finished = Some(settle.finish(None, None, self.lead_time));
                    }
                    GrinderState::WaitingForPortafilter {}
                } else {
                    let settle = match settle {
                        Some(mut settle) => {
                            let since_stop = time.saturating_duration_since(settle.stop_time);
                            if since_stop >= SETTLE_END {
                                finished = Some(self.learn_lead_time(settle));
                                None
                            } else {
                                if since_stop >= SETTLE_START {
                                    // A full buffer just keeps the first readings.
                                    settle.samples.push(weight - portafilter_weight).ok();
                                }
                                Some(settle)
                            }
                        }
                        None => None,
                    };
                    GrinderState::WaitingForRemoval {
                        portafilter_weight,
                        settle,
                    }
                }
            }
        });
        finished
    }
}

pub async fn controller_task(
    scale_receiver: channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        ScaleSample,
        SCALE_CHANNEL_SIZE,
    >,
    state_sender: watch::Sender<
        'static,
        CriticalSectionRawMutex,
        UserEvent,
        USER_EVENT_CHANNEL_SIZE,
    >,
    grind_progress_sender: watch::Sender<
        'static,
        CriticalSectionRawMutex,
        f32,
        GRIND_PROGRESS_CHANNEL_SIZE,
    >,
    event_sender: channel::Sender<
        'static,
        CriticalSectionRawMutex,
        ControllerEvent,
        CONTROLLER_EVENT_CHANNEL_SIZE,
    >,
    grinder_state_machine: &mutex::Mutex<CriticalSectionRawMutex, GrinderStateMachine>,
) {
    let mut last_event = {
        let grinder_state_machine_guard = grinder_state_machine.lock().await;
        let event = grinder_state_machine_guard.as_user_event();
        state_sender.send(event);
        event
    };

    loop {
        let (time, raw_weight) = scale_receiver.receive().await;
        let (event, weight, coffee_weight, target_weight, estimate, finished) = {
            let mut grinder_state_machine_guard = grinder_state_machine.lock().await;
            let finished = grinder_state_machine_guard.update_weight(time, raw_weight);
            let event = grinder_state_machine_guard.as_user_event();
            let weight = grinder_state_machine_guard
                .scale_setting
                .translate(raw_weight);
            let coffee_weight = grinder_state_machine_guard.get_coffee_weight(weight);
            let target_weight = grinder_state_machine_guard.get_target_weight();
            let estimate = grinder_state_machine_guard.grind_estimate();
            (event, weight, coffee_weight, target_weight, estimate, finished)
        };
        let filtered_weight = estimate.map(|(filtered_weight, _)| filtered_weight);
        let eta = estimate.and_then(|(_, eta)| eta).map(EtaReading::from);
        // Sent before the state so the LED strip never picks up a stale
        // progress from the previous grind when grinding starts.
        if let (UserEvent::Grinding, Some(progress_weight)) =
            (event, filtered_weight.or(coffee_weight))
        {
            grind_progress_sender.send((progress_weight / target_weight).clamp(0.0, 1.0));
        }
        if event != last_event {
            last_event = event;
            state_sender.send(event);
        }

        let reading = WeightReading::new(
            time.as_millis(),
            weight,
            event,
            coffee_weight,
            filtered_weight,
            eta,
        );
        event_sender.try_send(ControllerEvent::Reading(reading)).ok();
        if let Some(finished) = finished {
            // Once per grind, so wait for room rather than dropping it.
            event_sender.send(ControllerEvent::GrindFinished(finished)).await;
        }
    }
}
