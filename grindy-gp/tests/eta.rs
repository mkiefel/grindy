mod common;

use common::{feed_linear, TEST_PARAMS};
use grindy_gp::{GrindEstimator, MAX_ETA};

#[test]
fn none_before_the_first_update() {
    assert_eq!(GrindEstimator::new(TEST_PARAMS).eta(10.0), None);
}

#[test]
fn zero_when_the_target_is_already_reached() {
    let mut est = GrindEstimator::new(TEST_PARAMS);
    feed_linear(&mut est, 0.8, 1.0, 12.5);
    let eta = est.eta(5.0).unwrap();
    assert_eq!(eta.median, 0.0);
    assert_eq!(eta.lo, 0.0);
}

#[test]
fn constant_rate() {
    let mut est = GrindEstimator::new(TEST_PARAMS);
    let last = feed_linear(&mut est, 0.8, 1.0, 10.0);
    let expected = (12.0 - 0.8 * last) / 0.8;
    let eta = est.eta(12.0).unwrap();
    assert!((eta.median - expected).abs() < 0.1, "{eta:?} vs {expected}");
    assert!(eta.lo < eta.median && eta.median < eta.hi, "{eta:?}");
}

#[test]
fn median_is_where_the_forecast_mean_reaches_the_target() {
    let mut est = GrindEstimator::new(TEST_PARAMS);
    let last = feed_linear(&mut est, 0.8, 1.0, 10.0);
    let eta = est.eta(14.0).unwrap();
    let mean = est.forecast(last + eta.median).mean;
    assert!((mean - 14.0).abs() < 1e-3, "{mean}");
}

#[test]
fn increases_with_the_target() {
    let mut est = GrindEstimator::new(TEST_PARAMS);
    feed_linear(&mut est, 0.8, 1.0, 10.0);
    let medians: Vec<f32> = [9.0, 10.0, 12.0, 15.0]
        .iter()
        .map(|&target| est.eta(target).unwrap().median)
        .collect();
    assert!(medians.windows(2).all(|m| m[0] < m[1]), "{medians:?}");
}

#[test]
fn none_for_a_flat_signal() {
    let mut est = GrindEstimator::new(TEST_PARAMS);
    let mut t = 1.0;
    while t < 30.0 {
        est.update(t, 5.0);
        t += 0.1;
    }
    assert_eq!(est.eta(20.0), None);
}

#[test]
fn hi_is_capped_at_max_eta() {
    let mut est = GrindEstimator::new(TEST_PARAMS);
    feed_linear(&mut est, 0.8, 1.0, 3.0);
    // Median about 50 s out, so the 90 % quantile may lie beyond MAX_ETA.
    let eta = est.eta(40.0).unwrap();
    assert!(eta.hi <= MAX_ETA, "{eta:?}");
}
