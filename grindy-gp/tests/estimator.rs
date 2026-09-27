mod common;

use common::{feed_linear, Lcg, SHORT_MEMORY_PARAMS, TEST_PARAMS};
use grindy_gp::{GrindEstimator, Params};

#[test]
fn first_update_anchors_the_weight() {
    let mut est = GrindEstimator::new(TEST_PARAMS);
    assert_eq!(est.updates(), 0);
    est.update(1.0, 0.5);
    assert_eq!(est.updates(), 1);
    let w = est.weight();
    assert_eq!(w.mean, 0.5);
    assert_eq!(w.var, TEST_PARAMS.noise_var);
}

#[test]
fn learns_a_constant_rate() {
    let mut est = GrindEstimator::new(TEST_PARAMS);
    let last = feed_linear(&mut est, 0.8, 1.0, 20.0);
    let w = est.weight();
    assert!((w.mean - 0.8 * last).abs() < 0.01, "weight {}", w.mean);
    assert!(w.var < TEST_PARAMS.noise_var);
    let f = est.forecast(last + 1.0);
    assert!((f.mean - 0.8 * (last + 1.0)).abs() < 0.03, "forecast {}", f.mean);
}

#[test]
fn forecast_variance_grows_with_horizon() {
    let mut est = GrindEstimator::new(TEST_PARAMS);
    let last = feed_linear(&mut est, 0.8, 1.0, 10.0);
    let vars: Vec<f32> = [0.0, 0.5, 1.0, 2.0, 5.0]
        .iter()
        .map(|h| est.forecast(last + h).var)
        .collect();
    assert!(vars.windows(2).all(|v| v[0] < v[1]), "{vars:?}");
    // Forecasting into the past is clamped to the last update.
    assert_eq!(est.forecast(last - 1.0), est.forecast(last));
}

#[test]
fn ignores_out_of_order_and_non_finite_input() {
    let mut est = GrindEstimator::new(TEST_PARAMS);
    est.update(1.0, 0.0);
    est.update(1.1, 0.1);
    let before = (est.updates(), est.weight(), est.covariance());
    est.update(1.1, 5.0);
    est.update(1.05, 5.0);
    est.update(f32::NAN, 5.0);
    est.update(1.2, f32::NAN);
    est.update(1.2, f32::INFINITY);
    assert_eq!((est.updates(), est.weight(), est.covariance()), before);
}

#[test]
fn survives_a_long_gap() {
    let mut est = GrindEstimator::new(TEST_PARAMS);
    feed_linear(&mut est, 0.8, 1.0, 10.0);
    est.update(15.0, 0.8 * 15.0);
    let w = est.weight();
    assert!((w.mean - 12.0).abs() < 0.05, "weight {}", w.mean);
    let f = est.forecast(16.0);
    assert!(f.mean.is_finite() && f.var.is_finite() && f.var > 0.0);
}

fn assert_covariance_stays_psd(params: Params) {
    let mut rng = Lcg(42);
    let mut est = GrindEstimator::new(params);
    let mut t = 1.0;
    for step in 0..2000 {
        t += 0.001 + 5.0 * rng.next() * rng.next();
        est.update(t, 0.8 * t + 0.1 * (rng.next() - 0.5));
        let p = est.covariance();
        let trace = p[0][0] + p[1][1] + p[2][2];
        assert!(p.iter().flatten().all(|v| v.is_finite()), "step {step}: {p:?}");
        for i in 0..3 {
            for j in 0..3 {
                assert_eq!(p[i][j], p[j][i], "step {step}: not symmetric {p:?}");
            }
        }
        for k in 0..50 {
            let x = [rng.next() - 0.5, rng.next() - 0.5, rng.next() - 0.5];
            let q: f32 = (0..3)
                .map(|i| (0..3).map(|j| x[i] * p[i][j] * x[j]).sum::<f32>())
                .sum();
            assert!(q >= -1e-6 * trace, "step {step}, direction {k}: {q} for {p:?}");
        }
    }
}

#[test]
fn covariance_stays_psd() {
    assert_covariance_stays_psd(TEST_PARAMS);
}

#[test]
fn covariance_stays_psd_with_short_memory() {
    assert_covariance_stays_psd(SHORT_MEMORY_PARAMS);
}
