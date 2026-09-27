mod common;

use grindy_gp::{GrindEstimator, FITTED, RAMP_UP};

/// The Kalman filter computes the same posterior as GP regression with the
/// integrated-OU kernel (full Gram-matrix solve, done in float64 by
/// tools/gp/export_fixtures.py).
#[test]
fn matches_batch_gp_reference() {
    let grinds = common::grinds();
    let mut checked = 0;
    for line in include_str!("data/reference.csv").lines().skip(1) {
        let cols: Vec<&str> = line.split(',').collect();
        let grind = grinds.iter().find(|g| g.name == cols[0]).unwrap();
        let n_obs: usize = cols[1].parse().unwrap();
        let t_query: f32 = cols[2].parse().unwrap();
        let mean: f32 = cols[3].parse().unwrap();
        let var: f32 = cols[4].parse().unwrap();

        let mut est = GrindEstimator::new(FITTED);
        for (&t, &y) in grind.t.iter().zip(&grind.y).filter(|(t, _)| **t >= RAMP_UP).take(n_obs) {
            est.update(t, y);
        }
        let f = est.forecast(t_query);
        assert!((f.mean - mean).abs() <= 2e-3, "{line}: mean {} vs {mean}", f.mean);
        assert!((f.var - var).abs() <= 1e-2 * var, "{line}: var {} vs {var}", f.var);
        checked += 1;
    }
    assert_eq!(checked, 21);
}
