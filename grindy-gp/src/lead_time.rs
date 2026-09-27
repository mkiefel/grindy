//! When to stop the grinder, and learning how long coffee keeps arriving
//! after it stopped (the lead time).

use crate::{Eta, GrindEstimator};

/// Readings the estimator needs before its forecast may stop the grinder.
pub const MIN_UPDATES_FOR_STOP: u32 = 5;

/// Longest plausible lead time in s. Longer observations are rejected.
pub const MAX_LEAD_TIME: f32 = 2.0;

/// Weight of a new observation in the lead time's moving average.
const LEARNING_RATE: f32 = 0.2;

/// Whether to stop the grinder at `now` (the time of the last update), so
/// that the weight reached after `lead_time` more seconds of flow is `target`.
pub fn should_stop(est: &GrindEstimator, now: f32, lead_time: f32, target: f32) -> bool {
    est.updates() >= MIN_UPDATES_FOR_STOP && est.forecast(now + lead_time).mean >= target
}

/// Seconds until [`should_stop`] is expected to stop the grinder.
pub fn stop_eta(est: &GrindEstimator, lead_time: f32, target: f32) -> Option<Eta> {
    est.eta(target).map(|eta| Eta {
        median: (eta.median - lead_time).max(0.0),
        lo: (eta.lo - lead_time).max(0.0),
        hi: (eta.hi - lead_time).max(0.0),
    })
}

/// Lead time a grind actually had: how much longer it would have had to run
/// at the stop to reach the weight it `settled` at. `None` if that is
/// implausible, e.g. because the portafilter was pressed on.
pub fn observe_lead_time(est_at_stop: &GrindEstimator, settled: f32) -> Option<f32> {
    est_at_stop
        .eta(settled)
        .map(|eta| eta.median)
        .filter(|lead_time| (0.0..=MAX_LEAD_TIME).contains(lead_time))
}

/// Moves `lead_time` a step towards the `observed` one.
pub fn update_lead_time(lead_time: f32, observed: f32) -> f32 {
    lead_time + LEARNING_RATE * (observed - lead_time)
}
