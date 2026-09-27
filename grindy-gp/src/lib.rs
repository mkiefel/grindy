//! Gaussian-process estimate of the coffee weight while grinding.
//!
//! The flow rate is an Ornstein–Uhlenbeck process around an unknown mean
//! rate, the weight is its integral and the scale adds white noise. Because
//! the OU kernel is Markov, exact GP inference is a Kalman filter on
//! `[weight, rate, mean rate]` with O(1) work per sample. See
//! `docs/superpowers/specs/2026-09-27-gp-grind-estimator-design.md`.
#![cfg_attr(not(test), no_std)]

mod estimator;
mod fitted;

pub use estimator::{Gaussian, GrindEstimator, Params};
pub use fitted::{DEFAULT_LEAD_TIME, FITTED};

/// Seconds after the grinder starts before readings go into the estimator.
/// The flow only ramps up in that time, which would bias the rate low.
pub const RAMP_UP: f32 = 1.0;
