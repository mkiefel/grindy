mod common;

use common::{feed_linear, TEST_PARAMS};
use grindy_gp::lead_time::{
    observe_lead_time, should_stop, stop_eta, update_lead_time, MAX_LEAD_TIME,
    MIN_UPDATES_FOR_STOP,
};
use grindy_gp::{GrindEstimator, DEFAULT_LEAD_TIME};

/// Grinds at 0.8 g/s from t = 1 s until the weight is about `until` g.
fn grinding_until(until: f32) -> (GrindEstimator, f32) {
    let mut est = GrindEstimator::new(TEST_PARAMS);
    let last = feed_linear(&mut est, 0.8, 1.0, until / 0.8);
    (est, last)
}

#[test]
fn should_stop_false_before_min_updates() {
    let mut est = GrindEstimator::new(TEST_PARAMS);
    for i in 0..MIN_UPDATES_FOR_STOP - 1 {
        est.update(1.0 + 0.1 * i as f32, 30.0);
    }
    assert!(!should_stop(&est, 1.3, 0.5, 18.0));
}

#[test]
fn should_stop_accounts_for_the_lead_time() {
    let (est, now) = grinding_until(17.5);
    // Forecasts of about 17.9 g and 18.3 g.
    assert!(!should_stop(&est, now, 0.5, 18.0));
    assert!(should_stop(&est, now, 1.0, 18.0));
}

#[test]
fn should_stop_when_target_lowered_below_current_weight() {
    let (est, now) = grinding_until(10.0);
    assert!(should_stop(&est, now, 0.0, 8.0));
}

#[test]
fn stalled_grind_does_not_stop_and_pushes_the_eta_out() {
    let (mut est, mut t) = grinding_until(8.0);
    let weight = est.weight().mean;
    for _ in 0..300 {
        t += 0.1;
        est.update(t, weight);
    }
    assert!(!should_stop(&est, t, DEFAULT_LEAD_TIME, 18.0));
    // The mean rate is constant per grind in this model, so a stall after
    // grinding is read as a slow grind, not as a stop: the ETA is pushed
    // out (or becomes unknown), never `None` from a stalled read alone.
    let eta = stop_eta(&est, DEFAULT_LEAD_TIME, 18.0);
    assert!(eta.is_none_or(|eta| eta.median > 30.0), "{eta:?}");
}

#[test]
fn stop_eta_is_shifted_by_the_lead_time() {
    let (est, _) = grinding_until(10.0);
    let eta = est.eta(18.0).unwrap();
    let stop = stop_eta(&est, 0.5, 18.0).unwrap();
    assert!((stop.median - (eta.median - 0.5)).abs() < 1e-5);
    assert!((stop.lo - (eta.lo - 0.5)).abs() < 1e-5);
    assert!((stop.hi - (eta.hi - 0.5)).abs() < 1e-5);
    // Never negative, even when the stop is due.
    let late = stop_eta(&est, 20.0, 18.0).unwrap();
    assert_eq!((late.median, late.lo, late.hi), (0.0, 0.0, 0.0));
}

#[test]
fn observe_lead_time_from_settled_weight() {
    let (est, _) = grinding_until(17.6);
    let observed = observe_lead_time(&est, est.weight().mean + 0.4).unwrap();
    assert!((observed - 0.5).abs() < 0.03, "{observed}");
    // Settled below the weight at the stop: no time was needed.
    assert_eq!(observe_lead_time(&est, est.weight().mean - 0.2), Some(0.0));
}

#[test]
fn observe_lead_time_rejects_implausible_settled_weight() {
    let (est, _) = grinding_until(17.6);
    // Someone pressed on the portafilter: 10 g more would take 12.5 s.
    assert_eq!(observe_lead_time(&est, est.weight().mean + 10.0), None);
    assert!(observe_lead_time(&est, est.weight().mean + 0.8 * MAX_LEAD_TIME + 0.2).is_none());
}

#[test]
fn update_lead_time_is_an_exponential_moving_average() {
    assert!((update_lead_time(0.5, 1.0) - 0.6).abs() < 1e-6);
    assert!((update_lead_time(0.5, 0.0) - 0.4).abs() < 1e-6);
}
