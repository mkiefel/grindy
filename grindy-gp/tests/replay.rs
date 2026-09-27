//! Replays the logged grinds through the estimator, like the shoot-out in
//! the spec, and pins its accuracy as a regression floor.
mod common;

use grindy_gp::lead_time::{observe_lead_time, MIN_UPDATES_FOR_STOP};
use grindy_gp::{GrindEstimator, DEFAULT_LEAD_TIME, FITTED, RAMP_UP};

const TARGET: f32 = 17.0;

/// When the weight crossed `target`, from a centred 7-sample moving average,
/// linearly interpolated.
fn true_crossing(t: &[f32], y: &[f32], target: f32) -> Option<f32> {
    let n = y.len();
    let smooth: Vec<f32> = (0..n)
        .map(|i| {
            let (a, b) = (i.saturating_sub(3), (i + 4).min(n));
            y[a..b].iter().sum::<f32>() / (b - a) as f32
        })
        .collect();
    let i = smooth.iter().position(|&v| v >= target)?;
    if i == 0 {
        return None;
    }
    Some(t[i - 1] + (target - smooth[i - 1]) / (smooth[i] - smooth[i - 1]) * (t[i] - t[i - 1]))
}

struct Sample {
    remaining: f32,
    error: f32,
    covered: bool,
}

/// ETA to `TARGET` at every sample from 1 s into the grind until the true
/// crossing, feeding readings from `ramp_up` on.
fn replay(ramp_up: f32) -> Vec<Sample> {
    let mut samples = Vec::new();
    for grind in common::grinds() {
        let Some(crossing) = true_crossing(&grind.t, &grind.y, TARGET) else {
            continue;
        };
        let mut est = GrindEstimator::new(FITTED);
        for (&t, &y) in grind.t.iter().zip(&grind.y) {
            if t < ramp_up {
                continue;
            }
            est.update(t, y);
            if est.updates() < MIN_UPDATES_FOR_STOP || t < 1.0 || t >= crossing {
                continue;
            }
            let eta = est.eta(TARGET).expect("target reachable");
            let remaining = crossing - t;
            samples.push(Sample {
                remaining,
                error: eta.median - remaining,
                covered: eta.lo <= remaining && remaining <= eta.hi,
            });
        }
    }
    samples
}

fn in_range(samples: &[Sample], from: f32, to: f32) -> impl Iterator<Item = &Sample> {
    samples.iter().filter(move |s| from <= s.remaining && s.remaining < to)
}

fn mae(samples: &[Sample], from: f32, to: f32) -> f32 {
    let errors: Vec<f32> = in_range(samples, from, to).map(|s| s.error.abs()).collect();
    errors.iter().sum::<f32>() / errors.len() as f32
}

fn bias(samples: &[Sample], from: f32, to: f32) -> f32 {
    let errors: Vec<f32> = in_range(samples, from, to).map(|s| s.error).collect();
    errors.iter().sum::<f32>() / errors.len() as f32
}

#[test]
fn eta_accuracy_on_logged_grinds() {
    let samples = replay(RAMP_UP);
    let overall = mae(&samples, 0.0, f32::INFINITY);
    let last_second = mae(&samples, 0.0, 1.0);
    let coverage =
        samples.iter().filter(|s| s.covered).count() as f32 / samples.len() as f32;
    eprintln!(
        "{} samples: MAE {overall:.3} s, last second {last_second:.3} s, 80 % coverage {coverage:.2}",
        samples.len()
    );
    assert!(overall <= 0.8, "overall MAE {overall}");
    assert!(last_second <= 0.2, "last-second MAE {last_second}");
    assert!((0.65..=0.95).contains(&coverage), "coverage {coverage}");
}

#[test]
fn skipping_the_ramp_up_does_not_worsen_mid_range_bias() {
    let with_skip = bias(&replay(RAMP_UP), 3.0, 12.0);
    let without_skip = bias(&replay(0.0), 3.0, 12.0);
    eprintln!("3-12 s bias: {with_skip:.3} s skipping the ramp-up, {without_skip:.3} s without");
    assert!(with_skip.abs() <= without_skip.abs());
}

#[test]
fn lead_time_observed_on_logged_grinds_is_plausible() {
    let mut checked = 0;
    for grind in common::grinds() {
        let Some(settled) = grind.settled else { continue };
        let mut est = GrindEstimator::new(FITTED);
        for (&t, &y) in grind.t.iter().zip(&grind.y).filter(|(t, _)| **t >= RAMP_UP) {
            est.update(t, y);
        }
        let observed = observe_lead_time(&est, settled);
        eprintln!("{}: settled {settled:.2} g, lead time {observed:?}", grind.name);
        let observed = observed.unwrap();
        // Controller ruling (task-4): 0.1 lower bound, not 0.2 — grind
        // 2025-11-28_14:31:13#0 genuinely settled with a lead time of about
        // 0.165 s, so 0.2 was too tight a planning estimate.
        assert!((0.1..=1.0).contains(&observed), "{}: {observed}", grind.name);
        checked += 1;
    }
    assert_eq!(checked, 9);
}

#[test]
fn default_lead_time_is_plausible() {
    assert!((0.2..=1.0).contains(&DEFAULT_LEAD_TIME), "{DEFAULT_LEAD_TIME}");
}
