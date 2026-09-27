#![allow(dead_code)]

use grindy_gp::{GrindEstimator, Params};

/// Hyperparameters with a noticeable rate memory, independent of the fit.
pub const TEST_PARAMS: Params = Params {
    length_scale: 0.5,
    rate_var: 0.04,
    noise_var: 0.0025,
    mean_rate_prior: 0.75,
    mean_rate_prior_var: 0.01,
};

/// Like the fit on the logs: the rate has (almost) no memory.
pub const SHORT_MEMORY_PARAMS: Params = Params {
    length_scale: 0.01,
    rate_var: 2.0,
    noise_var: 0.0006,
    mean_rate_prior: 0.75,
    mean_rate_prior_var: 0.01,
};

/// Feeds noise-free readings `rate * t` every 0.1 s for `from <= t <= to`.
/// Returns the last `t` fed.
pub fn feed_linear(est: &mut GrindEstimator, rate: f32, from: f32, to: f32) -> f32 {
    let mut t = from;
    let mut last = from;
    while t <= to {
        est.update(t, rate * t);
        last = t;
        t += 0.1;
    }
    last
}

/// Deterministic pseudo random numbers in [0, 1).
pub struct Lcg(pub u64);

impl Lcg {
    pub fn next(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 40) as f32 / (1u64 << 24) as f32
    }
}
