# GP Grind Estimator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the grinder based on a Gaussian-process forecast of the coffee weight, so that the *settled* weight hits the target, learn the "coffee still arriving" lead time per grind, and show ETA and filtered weight on the web page, LED strip and logger.

**Architecture:** A new `no_std` workspace crate `grindy-gp` holds the model and the decision logic: an exact Kalman filter for the integrated Ornstein–Uhlenbeck GP (state `[weight, rate, mean rate]`), ETA quantiles, the stop rule and lead-time learning. Its tests run on the host, including a replay over the logged grinds. The firmware (`src/scale.rs`) feeds it timestamped samples, stops on its forecast, measures the settled weight after each grind to learn the lead time (stored in flash), and publishes the estimates over the existing WebSocket. Python uv scripts in `tools/gp/` fit the hyperparameters and export test fixtures from `logs/`.

**Tech Stack:** Rust nightly, Embassy on RP2350 (`thumbv8m.main-none-eabihf`), `libm`, postcard/serde, picoserve; vanilla JS web page; Python 3.10+ with uv, numpy/pandas/pyarrow/scipy, pytest.

**Spec:** `docs/superpowers/specs/2026-09-27-gp-grind-estimator-design.md`

## Global Constraints

- Toolchain: Rust nightly via `rust-toolchain.toml`. Firmware target `thumbv8m.main-none-eabihf` (default in `.cargo/config.toml`). The firmware is `#![no_std]` and uses no heap.
- `grindy-gp` is `no_std` (`#![cfg_attr(not(test), no_std)]`) and depends only on `libm`. Host tests: `cargo test-gp` (alias for `cargo test -p grindy-gp --target host-tuple`).
- All Rust estimator math is in `f32`; times are in seconds and weights in grams.
- The model is an integrated OU GP. Inference is an exact Kalman filter with the F and Q from the spec. The first update anchors the weight (the limit of a broad prior).
- Constants: `RAMP_UP = 1.0` s, `MIN_UPDATES_FOR_STOP = 5`, `MAX_LEAD_TIME = 2.0` s, lead-time learning rate `0.2`, flash store threshold `0.01` s, settle window `1.5`–`2.5` s after the stop, `MAX_ETA = 60` s, ETA quantiles 10 % / 50 % / 90 %.
- Stop rule: first of prediction (`forecast(now + lead_time).mean ≥ target` with ≥ 5 updates), raw coffee weight ≥ target, `MAX_GRIND_TIME_IN_SECS = 50`.
- Wire format is postcard, which is not self-describing. Fields are only appended at the end of structs. `WsMessage::GrindFinished` is variant **4**. `StopReason` order is `Prediction = 0`, `RawWeight = 1`, `Timeout = 2`. Rust (`src/web.rs`, `src/scale.rs`), JS (`src/index.js`) and Python (`logger/src/grindy_logger/models.py`) must agree.
- Flash: lead-time sector at `TARGET_WEIGHT_OFFSET + ERASE_SIZE`, magic `0x6772_7461` ("grta"). Only values in `[0, MAX_LEAD_TIME]` are valid.
- The raw-weight stop and the 50 s timeout stay as safety nets.
- **Do not loosen a test threshold that this plan specifies.** If one fails, stop and report the measured numbers.
- The project directory is **not a git repository**. Run the commit steps only if it has been initialized with git; otherwise skip them.
- Python tools are uv scripts with inline dependencies, run from the repo root as `uv run tools/gp/<script>.py`.

## Review Focus

1. **Target lowered mid-grind below the current weight** (`POST /target-weight` while grinding) → the grinder stops on the next sample. Pinned in Task 4 (`should_stop_when_target_lowered_below_current_weight`).
2. **Scale gap of several seconds during a grind** (HX711 or channel stall) → the estimator keeps going with a larger Δt, and the weight estimate and forecast stay sane, with no NaN or panic. Pinned in Task 1 (`survives_a_long_gap`).
3. **Grind stalls** (beans run out, the chute clogs) → the prediction never stops the grinder early, the ETA becomes unknown, and the timeout still applies. Pinned in Task 4 (`stalled_grind_does_not_stop_and_has_no_eta`).
4. **Tiny target reached during the ramp-up, before the estimator has samples** → the prediction doesn't fire, and the raw-weight rule stops the grinder. Pinned in Task 4 (`should_stop_false_before_min_updates`).
5. **Portafilter pressed or bumped during the settle window** (settled weight far above the forecast) → the observed lead time is rejected and the learned lead time doesn't jump. Pinned in Task 4 (`observe_lead_time_rejects_implausible_settled_weight`).

---

## File Structure

| File | Responsibility |
|---|---|
| `Cargo.toml` (modify) | Make the root a workspace with member `grindy-gp`; firmware depends on it |
| `.cargo/config.toml` (modify) | `test-gp` alias |
| `grindy-gp/Cargo.toml` (create) | Crate manifest, `libm` only |
| `grindy-gp/src/lib.rs` (create) | Module wiring, re-exports, `RAMP_UP` |
| `grindy-gp/src/estimator.rs` (create) | `Params`, `Gaussian`, `Eta`, `GrindEstimator` (Kalman filter, forecast, ETA) |
| `grindy-gp/src/fitted.rs` (generated) | `FITTED`, `DEFAULT_LEAD_TIME`, written by `tools/gp/fit.py` |
| `grindy-gp/src/lead_time.rs` (create) | Stop rule, stop ETA, lead-time observation and learning |
| `grindy-gp/tests/common/mod.rs` (create) | Test params, synthetic data, fixture loader |
| `grindy-gp/tests/{estimator,eta,lead_time,batch_gp,replay}.rs` (create) | Host tests |
| `grindy-gp/tests/data/{grinds,settled,reference}.csv` (generated) | Fixtures from `logs/` |
| `tools/gp/common.py` (create) | Log loading, float64 mirror of the estimator |
| `tools/gp/fit.py` (create) | Hyperparameter and default lead-time fit → `fitted.rs` |
| `tools/gp/export_fixtures.py` (create) | Fixtures and batch-GP reference |
| `src/storage.rs` (modify) | Lead-time sector |
| `src/scale.rs` (modify) | Timestamped samples, estimator in `Grinding`, stop rule, settle/learning, `ControllerEvent`, `GrindFinished` |
| `src/main.rs` (modify) | Channel types |
| `src/web.rs` (modify) | `WsMessage` fields and variant, broadcaster |
| `src/index.js`, `src/index.html` (modify) | Decoder, ETA / lead time / grind summary |
| `logger/src/grindy_logger/{models,writer}.py` (modify) | Decoder, Parquet columns |
| `logger/tests/test_models.py` (create) | Decoder and writer tests |
| `logger/README.md`, `CLAUDE.md` (modify) | Docs |

---

### Task 1: Workspace and `grindy-gp` estimator core

**Files:**
- Modify: `Cargo.toml`, `.cargo/config.toml`
- Create: `grindy-gp/Cargo.toml`, `grindy-gp/src/lib.rs`, `grindy-gp/src/estimator.rs`
- Test: `grindy-gp/tests/common/mod.rs`, `grindy-gp/tests/estimator.rs`

**Interfaces:**
- Produces:
  - `grindy_gp::Params { length_scale, rate_var, noise_var, mean_rate_prior, mean_rate_prior_var: f32 }` (Clone, Copy, Debug, PartialEq)
  - `grindy_gp::Gaussian { mean: f32, var: f32 }`
  - `grindy_gp::GrindEstimator` (Clone, Copy, Debug) with `new(Params) -> Self`, `update(&mut self, t: f32, weight: f32)`, `updates(&self) -> u32`, `weight(&self) -> Gaussian`, `forecast(&self, t: f32) -> Gaussian`, `covariance(&self) -> [[f32; 3]; 3]`
  - test helpers `common::TEST_PARAMS`, `common::feed_linear(&mut GrindEstimator, rate: f32, from: f32, to: f32) -> f32` (returns the last t fed)

- [ ] **Step 1: Create the workspace and the empty crate**

Append to `Cargo.toml` (the root package stays as it is):

```toml
[workspace]
members = [".", "grindy-gp"]
```

Append to `.cargo/config.toml`:

```toml
[alias]
# The build target above is the MCU; grindy-gp's tests run on the host.
test-gp = "test -p grindy-gp --target host-tuple"
```

Create `grindy-gp/Cargo.toml`:

```toml
[package]
name = "grindy-gp"
version = "0.1.0"
edition = "2024"
license = "Apache-2.0"

[dependencies]
libm = "0.2"
```

Create `grindy-gp/src/lib.rs`:

```rust
//! Gaussian-process estimate of the coffee weight while grinding.
//!
//! The flow rate is an Ornstein–Uhlenbeck process around an unknown mean
//! rate, the weight is its integral and the scale adds white noise. Because
//! the OU kernel is Markov, exact GP inference is a Kalman filter on
//! `[weight, rate, mean rate]` with O(1) work per sample. See
//! `docs/superpowers/specs/2026-09-27-gp-grind-estimator-design.md`.
#![cfg_attr(not(test), no_std)]

mod estimator;

pub use estimator::{Gaussian, GrindEstimator, Params};
```

- [ ] **Step 2: Write the failing tests**

Create `grindy-gp/tests/common/mod.rs`:

```rust
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
```

Create `grindy-gp/tests/estimator.rs`:

```rust
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
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `cargo test-gp`
Expected: compile errors, because `grindy_gp::estimator` does not exist yet.

- [ ] **Step 4: Implement the estimator**

Create `grindy-gp/src/estimator.rs`:

```rust
use libm::expf;

/// Hyperparameters of the GP. Times in s, weights in g.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Params {
    /// OU length scale ℓ of the flow rate in s.
    pub length_scale: f32,
    /// Stationary variance σ² of the flow rate around its mean in (g/s)².
    pub rate_var: f32,
    /// Variance s² of the scale noise in g².
    pub noise_var: f32,
    /// Prior mean m₀ of the mean flow rate in g/s.
    pub mean_rate_prior: f32,
    /// Prior variance v₀ of the mean flow rate in (g/s)².
    pub mean_rate_prior_var: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gaussian {
    pub mean: f32,
    pub var: f32,
}

type Vec3 = [f32; 3];
type Mat3 = [[f32; 3]; 3];

fn mat_vec(a: &Mat3, x: &Vec3) -> Vec3 {
    core::array::from_fn(|i| (0..3).map(|k| a[i][k] * x[k]).sum())
}

fn mat_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    core::array::from_fn(|i| core::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}

fn transpose(a: &Mat3) -> Mat3 {
    core::array::from_fn(|i| core::array::from_fn(|j| a[j][i]))
}

/// Makes `p` exactly symmetric and keeps f32 rounding from producing negative
/// variances.
fn symmetrize(p: &mut Mat3) {
    for i in 0..3 {
        p[i][i] = p[i][i].max(0.0);
        for j in 0..i {
            let v = 0.5 * (p[i][j] + p[j][i]);
            p[i][j] = v;
            p[j][i] = v;
        }
    }
}

/// Kalman filter on `x = [weight, rate, mean rate]`, the exact posterior of
/// the integrated-OU GP.
#[derive(Debug, Clone, Copy)]
pub struct GrindEstimator {
    params: Params,
    t_last: f32,
    m: Vec3,
    p: Mat3,
    updates: u32,
}

impl GrindEstimator {
    /// The weight is unknown until the first [`Self::update`] anchors it; the
    /// rate starts at its stationary distribution around the mean rate prior.
    pub fn new(params: Params) -> Self {
        let v0 = params.mean_rate_prior_var;
        Self {
            params,
            t_last: 0.0,
            m: [0.0, params.mean_rate_prior, params.mean_rate_prior],
            p: [[0.0; 3], [0.0, params.rate_var + v0, v0], [0.0, v0, v0]],
            updates: 0,
        }
    }

    /// Number of readings taken into account.
    pub fn updates(&self) -> u32 {
        self.updates
    }

    /// State mean and covariance `dt` seconds after the last update.
    fn predicted(&self, dt: f32) -> (Vec3, Mat3) {
        let Params {
            length_scale: l,
            rate_var: s2,
            ..
        } = self.params;
        let a = expf(-dt / l);
        let c = l * (1.0 - a);
        let f = [[1.0, c, dt - c], [0.0, a, 1.0 - a], [0.0, 0.0, 1.0]];
        let q_ww = (s2 * l * (2.0 * dt - l * (3.0 - 4.0 * a + a * a))).max(0.0);
        let q_wr = s2 * l * (1.0 - a) * (1.0 - a);
        let q_rr = s2 * (1.0 - a * a);
        let mut p = mat_mul(&mat_mul(&f, &self.p), &transpose(&f));
        p[0][0] += q_ww;
        p[0][1] += q_wr;
        p[1][0] += q_wr;
        p[1][1] += q_rr;
        symmetrize(&mut p);
        (mat_vec(&f, &self.m), p)
    }

    /// Takes the scale reading `weight` at time `t` into account. Readings
    /// that are not finite or not after the last one are ignored.
    pub fn update(&mut self, t: f32, weight: f32) {
        if !t.is_finite() || !weight.is_finite() || (self.updates > 0 && t <= self.t_last) {
            return;
        }
        if self.updates == 0 {
            // Limit of a broad prior on the weight: the reading pins it down
            // and says nothing about the rate yet.
            self.m[0] = weight;
            self.p[0] = [self.params.noise_var, 0.0, 0.0];
            self.p[1][0] = 0.0;
            self.p[2][0] = 0.0;
        } else {
            let (m, p) = self.predicted(t - self.t_last);
            let s = p[0][0] + self.params.noise_var;
            let k = [p[0][0] / s, p[1][0] / s, p[2][0] / s];
            let e = weight - m[0];
            for i in 0..3 {
                self.m[i] = m[i] + k[i] * e;
                for j in 0..3 {
                    self.p[i][j] = p[i][j] - k[i] * k[j] * s;
                }
            }
            symmetrize(&mut self.p);
        }
        self.t_last = t;
        self.updates += 1;
    }

    /// Filtered weight at the last update. Meaningless before the first one.
    pub fn weight(&self) -> Gaussian {
        Gaussian {
            mean: self.m[0],
            var: self.p[0][0],
        }
    }

    /// Weight expected at time `t`; times before the last update are clamped
    /// to it. Meaningless before the first update.
    pub fn forecast(&self, t: f32) -> Gaussian {
        let (m, p) = self.predicted((t - self.t_last).max(0.0));
        Gaussian {
            mean: m[0],
            var: p[0][0],
        }
    }

    /// Covariance of `[weight, rate, mean rate]`, for tests and diagnostics.
    pub fn covariance(&self) -> [[f32; 3]; 3] {
        self.p
    }
}
```

- [ ] **Step 5: Run the tests and make sure they pass**

Run: `cargo test-gp`
Expected: all 7 tests in `estimator.rs` PASS.

Then run `cargo build --release`.
Expected: the firmware still builds. The workspace change must not break it.

- [ ] **Step 6: Commit** (only if the project is a git repository)

```bash
git add Cargo.toml Cargo.lock .cargo/config.toml grindy-gp
git commit -m "Add grindy-gp crate with integrated-OU Kalman estimator"
```

---

### Task 2: Fit tooling, fixtures and batch-GP equivalence

**Files:**
- Create: `tools/gp/common.py`, `tools/gp/fit.py`, `tools/gp/export_fixtures.py`
- Generated: `grindy-gp/src/fitted.rs`, `grindy-gp/tests/data/grinds.csv`, `grindy-gp/tests/data/settled.csv`, `grindy-gp/tests/data/reference.csv`
- Modify: `grindy-gp/src/lib.rs`, `grindy-gp/tests/common/mod.rs`
- Test: `grindy-gp/tests/batch_gp.rs`

**Interfaces:**
- Consumes: `GrindEstimator`, `Params` from Task 1.
- Produces:
  - `grindy_gp::FITTED: Params`, `grindy_gp::DEFAULT_LEAD_TIME: f32`, `grindy_gp::RAMP_UP: f32 = 1.0`
  - test helpers `common::Grind { name: String, t: Vec<f32>, y: Vec<f32>, settled: Option<f32> }` and `common::grinds() -> Vec<Grind>` (the last sample of each grind is the one that stopped the grinder)
  - Python `tools/gp/common.py`: `Params`, `Grind`, `load_grinds()`, `Estimator` (float64 mirror), `feed(grind, params, ramp_up=RAMP_UP) -> (Estimator, nll)`

- [ ] **Step 1: Write the shared Python module**

Create `tools/gp/common.py`:

```python
"""Shared helpers for the grind estimator tools: loading the logged grinds and
a float64 mirror of the Kalman filter in grindy-gp (keep the two in sync)."""

from __future__ import annotations

import glob
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]

# Keep in sync with grindy_gp::RAMP_UP.
RAMP_UP = 1.0
# Settle window after the stop in s, keep in sync with src/scale.rs.
SETTLE_START = 1.5
SETTLE_END = 2.5
# Keep in sync with grindy_gp::lead_time::MAX_LEAD_TIME.
MAX_LEAD_TIME = 2.0


@dataclass
class Params:
    length_scale: float
    rate_var: float
    noise_var: float
    mean_rate_prior: float
    mean_rate_prior_var: float


@dataclass
class Grind:
    name: str
    # Seconds since the grind started. The last sample is the one that
    # stopped the grinder (logged with the next state).
    t: np.ndarray
    y: np.ndarray  # coffee weight in g
    settled: float | None  # settled coffee weight after the stop in g


def mad_mean(values: np.ndarray, threshold: float = 3.0) -> float | None:
    """Mean without outliers by modified z-score, like compute_mean_variance
    in src/scale.rs."""
    if len(values) == 0:
        return None
    median = np.median(values)
    mad = np.median(np.abs(values - median))
    if mad < np.finfo(np.float32).eps:
        return None
    keep = 0.6745 * np.abs(values - median) / mad < threshold
    return float(np.mean(values[keep]))


def load_grinds() -> list[Grind]:
    grinds = []
    for path in sorted(glob.glob(str(ROOT / "logs" / "*.parquet"))):
        df = pd.read_parquet(path)
        w = df[df.message_type == "WeightReading"].reset_index(drop=True)
        ts = w.timestamp_ms.to_numpy(np.int64)
        states = w.state.to_numpy()
        coffee = w.coffee_weight.to_numpy(np.float64)
        grinding = states == "Grinding"
        starts = np.flatnonzero(grinding & ~np.r_[False, grinding[:-1]])
        for n, start in enumerate(starts):
            stop = start
            while stop < len(w) and grinding[stop]:
                stop += 1
            if stop >= len(w) or states[stop] != "WaitingForRemoval":
                continue
            since_stop = (ts - ts[stop]) / 1000.0
            window = (
                (since_stop >= SETTLE_START)
                & (since_stop < SETTLE_END)
                & (states == "WaitingForRemoval")
            )
            grinds.append(
                Grind(
                    name=f"{Path(path).stem}#{n}",
                    t=np.round((ts[start : stop + 1] - ts[start]) / 1000.0, 3),
                    y=coffee[start : stop + 1].astype(np.float32).astype(np.float64),
                    settled=mad_mean(coffee[window]),
                )
            )
    return grinds


class Estimator:
    """float64 mirror of grindy_gp::GrindEstimator."""

    def __init__(self, p: Params):
        self.p = p
        v0 = p.mean_rate_prior_var
        self.t_last = 0.0
        self.m = np.array([0.0, p.mean_rate_prior, p.mean_rate_prior])
        self.P = np.array([[0.0, 0.0, 0.0], [0.0, p.rate_var + v0, v0], [0.0, v0, v0]])
        self.updates = 0

    def predicted(self, dt: float):
        l, s2 = self.p.length_scale, self.p.rate_var
        a = math.exp(-dt / l)
        c = l * (1 - a)
        F = np.array([[1.0, c, dt - c], [0.0, a, 1 - a], [0.0, 0.0, 1.0]])
        q_ww = max(s2 * l * (2 * dt - l * (3 - 4 * a + a * a)), 0.0)
        q_wr = s2 * l * (1 - a) ** 2
        Q = np.array([[q_ww, q_wr, 0.0], [q_wr, s2 * (1 - a * a), 0.0], [0.0, 0.0, 0.0]])
        return F @ self.m, F @ self.P @ F.T + Q

    def update(self, t: float, y: float) -> float:
        """Adds a reading and returns its negative log marginal likelihood
        (0 for the first reading, which anchors the weight)."""
        if self.updates == 0:
            self.m[0] = y
            self.P[0, :] = 0.0
            self.P[:, 0] = 0.0
            self.P[0, 0] = self.p.noise_var
            nll = 0.0
        else:
            if t <= self.t_last:
                return 0.0
            m, P = self.predicted(t - self.t_last)
            s = P[0, 0] + self.p.noise_var
            k = P[:, 0] / s
            e = y - m[0]
            self.m = m + k * e
            self.P = P - np.outer(k, k) * s
            nll = 0.5 * (math.log(2 * math.pi * s) + e * e / s)
        self.t_last = t
        self.updates += 1
        return nll

    def forecast(self, t: float) -> tuple[float, float]:
        m, P = self.predicted(max(t - self.t_last, 0.0))
        return float(m[0]), float(P[0, 0])

    def mean_crossing(self, target: float, max_h: float = 60.0) -> float | None:
        """Seconds after the last update until the forecast mean reaches
        target, i.e. the ETA median."""
        if self.forecast(self.t_last)[0] >= target:
            return 0.0
        lo, hi = 0.0, 0.25
        while self.forecast(self.t_last + hi)[0] < target:
            lo, hi = hi, hi + 0.25
            if hi > max_h:
                return None
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if self.forecast(self.t_last + mid)[0] >= target:
                hi = mid
            else:
                lo = mid
        return hi


def feed(grind: Grind, p: Params, ramp_up: float = RAMP_UP) -> tuple[Estimator, float]:
    """Runs the estimator over a grind, skipping the ramp-up like the firmware.
    Returns the estimator after the stop sample and the negative log marginal
    likelihood."""
    est = Estimator(p)
    nll = 0.0
    for t, y in zip(grind.t, grind.y):
        if t >= ramp_up:
            nll += est.update(float(t), float(y))
    return est, nll
```

- [ ] **Step 2: Write the fit script and generate `fitted.rs`**

Create `tools/gp/fit.py`:

```python
# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "pandas", "pyarrow", "scipy"]
# ///
"""Fits the grind estimator's hyperparameters and default lead time to the
grinds in logs/ and writes them to grindy-gp/src/fitted.rs.

Run from the repo root: uv run tools/gp/fit.py
"""

import math

import numpy as np
from scipy.optimize import minimize

from common import MAX_LEAD_TIME, ROOT, Params, feed, load_grinds

# All logs share one grind setting, so they can't tell how much the mean flow
# rate varies between setups: fitting it drives it to 0, which would freeze
# the rate at the prior. A prior sd of 0.1 g/s keeps it learned per grind.
MEAN_RATE_PRIOR_VAR = 0.1**2


def params(theta) -> Params:
    return Params(
        length_scale=math.exp(theta[0]),
        rate_var=math.exp(theta[1]),
        noise_var=math.exp(theta[2]),
        mean_rate_prior=theta[3],
        mean_rate_prior_var=MEAN_RATE_PRIOR_VAR,
    )


def main() -> None:
    grinds = load_grinds()
    print(f"{len(grinds)} grinds")

    def nll(theta) -> float:
        return sum(feed(g, params(theta))[1] for g in grinds)

    result = minimize(
        nll,
        [math.log(0.02), math.log(1.0), math.log(0.025**2), 0.75],
        method="Nelder-Mead",
        options={"maxiter": 2000, "xatol": 1e-4, "fatol": 1e-3},
    )
    if not result.success:
        print(f"warning: optimizer did not converge: {result.message}")
    p = params(result.x)
    print(f"{p}\nnegative log likelihood {result.fun:.2f}")

    lead_times = []
    for g in grinds:
        if g.settled is None:
            print(f"{g.name}: no settled weight")
            continue
        est, _ = feed(g, p)
        lead_time = est.mean_crossing(g.settled)
        print(f"{g.name}: settled {g.settled:.2f} g, lead time {lead_time}")
        if lead_time is not None and 0.0 <= lead_time <= MAX_LEAD_TIME:
            lead_times.append(lead_time)
    default_lead_time = float(np.mean(lead_times))
    print(f"default lead time {default_lead_time:.3f} s from {len(lead_times)} grinds")

    (ROOT / "grindy-gp" / "src" / "fitted.rs").write_text(
        f"""//! Generated by `uv run tools/gp/fit.py` from the grinds in `logs/`.
//! Do not edit by hand, rerun the script instead.

use crate::Params;

/// GP hyperparameters maximizing the marginal likelihood of the logged
/// grinds (`mean_rate_prior_var` is fixed, see the script).
pub const FITTED: Params = Params {{
    length_scale: {p.length_scale:.6e},
    rate_var: {p.rate_var:.6e},
    noise_var: {p.noise_var:.6e},
    mean_rate_prior: {p.mean_rate_prior:.6e},
    mean_rate_prior_var: {p.mean_rate_prior_var:.6e},
}};

/// Mean lead time observed on the logged grinds, used until grindy has
/// learned its own.
pub const DEFAULT_LEAD_TIME: f32 = {default_lead_time:.6e};
"""
    )


if __name__ == "__main__":
    main()
```

Run: `uv run tools/gp/fit.py`
Expected: `9 grinds`; `length_scale` around 0.01 or below; `noise_var` around 6e-4 (sd ≈ 0.024 g); `mean_rate_prior` around 0.75–0.8; 9 lead times, each between about 0.2 and 1.0 s; default lead time around 0.5 s. `grindy-gp/src/fitted.rs` is written. If the values are far off (e.g. ℓ > 1 s or noise sd > 0.06 g), stop and report.

- [ ] **Step 3: Wire the generated constants into the crate**

Replace `grindy-gp/src/lib.rs` with:

```rust
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
```

- [ ] **Step 4: Write the fixture export script and generate the fixtures**

Create `tools/gp/export_fixtures.py`:

```python
# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "pandas", "pyarrow"]
# ///
"""Exports the logged grinds as test fixtures for grindy-gp, plus reference
forecasts from a batch GP (full Gram-matrix solve) with the fitted
hyperparameters from grindy-gp/src/fitted.rs.

Run from the repo root after tools/gp/fit.py: uv run tools/gp/export_fixtures.py
"""

import re

import numpy as np

from common import RAMP_UP, ROOT, Params, load_grinds

DATA = ROOT / "grindy-gp" / "tests" / "data"
# Prior variance of the weight at the first reading; large enough to match
# the filter's anchoring (its limit) to far below f32 precision.
ANCHOR_VAR = 1e4


def fitted_params() -> Params:
    text = (ROOT / "grindy-gp" / "src" / "fitted.rs").read_text()
    values = dict(re.findall(r"(\w+): ([-+0-9.e]+),", text))
    return Params(**{name: float(values[name]) for name in Params.__dataclass_fields__})


def batch_forecast(p: Params, t: np.ndarray, y: np.ndarray, t_query: float):
    """Posterior mean and variance of the weight at t_query given readings y
    at times t, by solving with the full Gram matrix of the weight kernel."""
    l = p.length_scale
    tau = t - t[0]
    tq = t_query - t[0]

    def k(a, b):
        integrated_ou = p.rate_var * (
            2 * l * np.minimum(a, b)
            - l * l * (1 - np.exp(-a / l) - np.exp(-b / l) + np.exp(-np.abs(a - b) / l))
        )
        return ANCHOR_VAR + integrated_ou + p.mean_rate_prior_var * a * b

    a, b = np.meshgrid(tau, tau, indexing="ij")
    gram = k(a, b) + p.noise_var * np.eye(len(tau))
    k_star = k(tq, tau)
    mean = p.mean_rate_prior * tq + k_star @ np.linalg.solve(gram, y - p.mean_rate_prior * tau)
    var = k(tq, tq) - k_star @ np.linalg.solve(gram, k_star)
    return float(mean), float(var)


def main() -> None:
    DATA.mkdir(parents=True, exist_ok=True)
    grinds = load_grinds()

    with open(DATA / "grinds.csv", "w") as f:
        f.write("grind,t,coffee_weight\n")
        for g in grinds:
            for t, y in zip(g.t, g.y):
                f.write(f"{g.name},{t:.3f},{y:.9g}\n")

    with open(DATA / "settled.csv", "w") as f:
        f.write("grind,settled_weight\n")
        for g in grinds:
            if g.settled is not None:
                f.write(f"{g.name},{g.settled:.9g}\n")

    p = fitted_params()
    g = grinds[0]
    keep = g.t >= RAMP_UP
    t, y = g.t[keep], g.y[keep]
    with open(DATA / "reference.csv", "w") as f:
        f.write("grind,n_obs,t_query,mean,var\n")
        for n in (1, 2, 5, 20, 50, 100, len(t)):
            for h in (0.0, 0.5, 3.0):
                t_query = round(t[n - 1] + h, 3)
                mean, var = batch_forecast(p, t[:n], y[:n], t_query)
                f.write(f"{g.name},{n},{t_query:.3f},{mean:.9g},{var:.9g}\n")
    print(f"wrote {len(grinds)} grinds to {DATA}")


if __name__ == "__main__":
    main()
```

Run: `uv run tools/gp/export_fixtures.py`
Expected: `wrote 9 grinds to …/grindy-gp/tests/data`. `grinds.csv` has about 2000 rows, `settled.csv` 9 rows, `reference.csv` 21 rows.

- [ ] **Step 5: Write the failing equivalence test**

Append to `grindy-gp/tests/common/mod.rs`:

```rust
/// A logged grind. The last sample is the one that stopped the grinder.
pub struct Grind {
    pub name: String,
    /// Seconds since the grind started.
    pub t: Vec<f32>,
    /// Coffee weight in g.
    pub y: Vec<f32>,
    /// Settled coffee weight after the stop in g.
    pub settled: Option<f32>,
}

/// The grinds in `tests/data`, exported from `logs/` by
/// `tools/gp/export_fixtures.py`.
pub fn grinds() -> Vec<Grind> {
    let mut grinds: Vec<Grind> = Vec::new();
    for line in include_str!("../data/grinds.csv").lines().skip(1) {
        let mut cols = line.split(',');
        let name = cols.next().unwrap();
        let t: f32 = cols.next().unwrap().parse().unwrap();
        let y: f32 = cols.next().unwrap().parse().unwrap();
        if grinds.last().is_none_or(|g| g.name != name) {
            grinds.push(Grind {
                name: name.to_string(),
                t: Vec::new(),
                y: Vec::new(),
                settled: None,
            });
        }
        let grind = grinds.last_mut().unwrap();
        grind.t.push(t);
        grind.y.push(y);
    }
    for line in include_str!("../data/settled.csv").lines().skip(1) {
        let (name, settled) = line.split_once(',').unwrap();
        let grind = grinds.iter_mut().find(|g| g.name == name).unwrap();
        grind.settled = Some(settled.parse().unwrap());
    }
    grinds
}
```

Create `grindy-gp/tests/batch_gp.rs`:

```rust
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
```

- [ ] **Step 6: Run the tests**

Run: `cargo test-gp`
Expected: all tests PASS, including `matches_batch_gp_reference`. This test checks the Task 1 filter against an independent computation. If it fails, the bug is in `estimator.rs` or in the Python reference, not in the tolerance; fix the math and don't widen the tolerance.

- [ ] **Step 7: Commit** (only if the project is a git repository)

```bash
git add tools/gp grindy-gp
git commit -m "Fit grind estimator hyperparameters and check against batch GP"
```

---

### Task 3: ETA quantiles

**Files:**
- Modify: `grindy-gp/src/estimator.rs`, `grindy-gp/src/lib.rs`
- Test: `grindy-gp/tests/eta.rs`

**Interfaces:**
- Consumes: `GrindEstimator::forecast` from Task 1.
- Produces:
  - `grindy_gp::Eta { median: f32, lo: f32, hi: f32 }` (Clone, Copy, Debug, PartialEq): seconds after the last update; 50 %, 10 % and 90 % quantiles
  - `GrindEstimator::eta(&self, target: f32) -> Option<Eta>`
  - `grindy_gp::MAX_ETA: f32 = 60.0`

- [ ] **Step 1: Write the failing tests**

Create `grindy-gp/tests/eta.rs`:

```rust
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
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test-gp --test eta`
Expected: compile errors, because `eta`, `Eta` and `MAX_ETA` don't exist yet.

- [ ] **Step 3: Implement `eta`**

In `grindy-gp/src/estimator.rs`, change the import to `use libm::{erfcf, expf, sqrtf};`, and add after the `Gaussian` struct:

```rust
/// Time until the weight reaches a target, in seconds after the last update.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Eta {
    /// 50 % quantile: where the forecast mean reaches the target.
    pub median: f32,
    /// 10 % quantile.
    pub lo: f32,
    /// 90 % quantile, at most [`MAX_ETA`].
    pub hi: f32,
}

/// ETAs further out than this are reported as unknown.
pub const MAX_ETA: f32 = 60.0;
/// Grid the ETA quantiles are first searched on, refined by bisection.
const ETA_GRID_STEP: f32 = 0.25;
const ETA_BISECTIONS: u32 = 16;
```

Add these methods to `impl GrindEstimator`:

```rust
    /// Probability that the weight reached `target` within `h` seconds after
    /// the last update, approximated by P(w(t_last + h) ≥ target).
    fn reached_probability(&self, h: f32, target: f32) -> f32 {
        let f = self.forecast(self.t_last + h);
        let sd = sqrtf(f.var.max(1e-12));
        0.5 * erfcf((target - f.mean) / (sd * core::f32::consts::SQRT_2))
    }

    /// First time after the last update at which the probability of having
    /// reached `target` is at least `quantile`, if within [`MAX_ETA`].
    fn eta_quantile(&self, target: f32, quantile: f32) -> Option<f32> {
        if self.reached_probability(0.0, target) >= quantile {
            return Some(0.0);
        }
        let mut before = 0.0;
        let mut h = ETA_GRID_STEP;
        while h <= MAX_ETA {
            if self.reached_probability(h, target) >= quantile {
                let (mut lo, mut hi) = (before, h);
                for _ in 0..ETA_BISECTIONS {
                    let mid = 0.5 * (lo + hi);
                    if self.reached_probability(mid, target) >= quantile {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                return Some(hi);
            }
            before = h;
            h += ETA_GRID_STEP;
        }
        None
    }

    /// When the weight is expected to reach `target`, or `None` if that is
    /// not expected within [`MAX_ETA`] (or before the first update).
    pub fn eta(&self, target: f32) -> Option<Eta> {
        if self.updates == 0 {
            return None;
        }
        let median = self.eta_quantile(target, 0.5)?;
        Some(Eta {
            median,
            lo: self.eta_quantile(target, 0.1).unwrap_or(median),
            hi: self.eta_quantile(target, 0.9).unwrap_or(MAX_ETA),
        })
    }
```

In `grindy-gp/src/lib.rs`, change the estimator re-export to:

```rust
pub use estimator::{Eta, Gaussian, GrindEstimator, Params, MAX_ETA};
```

- [ ] **Step 4: Run the tests and make sure they pass**

Run: `cargo test-gp`
Expected: all tests PASS.

- [ ] **Step 5: Commit** (only if the project is a git repository)

```bash
git add grindy-gp
git commit -m "Add ETA quantiles to the grind estimator"
```

---

### Task 4: Stop rule, lead time and replay over the logs

**Files:**
- Create: `grindy-gp/src/lead_time.rs`
- Modify: `grindy-gp/src/lib.rs`
- Test: `grindy-gp/tests/lead_time.rs`, `grindy-gp/tests/replay.rs`

**Interfaces:**
- Consumes: `GrindEstimator::{updates, forecast, eta}`, `Eta`, `FITTED`, `DEFAULT_LEAD_TIME`, `RAMP_UP`, test `common::{grinds, feed_linear, TEST_PARAMS}`.
- Produces (module `grindy_gp::lead_time`):
  - `MIN_UPDATES_FOR_STOP: u32 = 5`, `MAX_LEAD_TIME: f32 = 2.0`
  - `should_stop(est: &GrindEstimator, now: f32, lead_time: f32, target: f32) -> bool`
  - `stop_eta(est: &GrindEstimator, lead_time: f32, target: f32) -> Option<Eta>`: seconds until the grinder is expected to stop
  - `observe_lead_time(est_at_stop: &GrindEstimator, settled: f32) -> Option<f32>`
  - `update_lead_time(lead_time: f32, observed: f32) -> f32`

- [ ] **Step 1: Write the failing unit tests**

Create `grindy-gp/tests/lead_time.rs`:

```rust
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
fn stalled_grind_does_not_stop_and_has_no_eta() {
    let (mut est, mut t) = grinding_until(8.0);
    let weight = est.weight().mean;
    for _ in 0..300 {
        t += 0.1;
        est.update(t, weight);
    }
    assert!(!should_stop(&est, t, DEFAULT_LEAD_TIME, 18.0));
    assert_eq!(stop_eta(&est, DEFAULT_LEAD_TIME, 18.0), None);
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
```

- [ ] **Step 2: Write the failing replay tests**

Create `grindy-gp/tests/replay.rs`:

```rust
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
    assert!((0.65..=0.90).contains(&coverage), "coverage {coverage}");
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
        assert!((0.2..=1.0).contains(&observed), "{}: {observed}", grind.name);
        checked += 1;
    }
    assert_eq!(checked, 9);
}

#[test]
fn default_lead_time_is_plausible() {
    assert!((0.2..=1.0).contains(&DEFAULT_LEAD_TIME), "{DEFAULT_LEAD_TIME}");
}
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `cargo test-gp`
Expected: compile errors, because `grindy_gp::lead_time` does not exist yet.

- [ ] **Step 4: Implement the module**

Create `grindy-gp/src/lead_time.rs`:

```rust
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
```

In `grindy-gp/src/lib.rs`, add after `mod fitted;`:

```rust
pub mod lead_time;
```

- [ ] **Step 5: Run the tests and make sure they pass**

Run: `cargo test-gp -- --nocapture`
Expected: all tests PASS. Note the printed replay numbers: MAE, last-second MAE, coverage, bias with and without skipping the ramp-up, and the lead time per grind. They go into the final report. If a replay threshold fails, **stop and report the numbers**; don't change the thresholds or `RAMP_UP`.

- [ ] **Step 6: Commit** (only if the project is a git repository)

```bash
git add grindy-gp
git commit -m "Add stop rule and lead time learning with replay tests"
```

---

### Task 5: Firmware: timestamped samples, estimator-driven stop and estimates on the wire

**Files:**
- Modify: `Cargo.toml`, `src/storage.rs`, `src/scale.rs`, `src/main.rs`

**Interfaces:**
- Consumes: `grindy_gp::{GrindEstimator, FITTED, RAMP_UP, DEFAULT_LEAD_TIME, Eta}`, `grindy_gp::lead_time::{should_stop, stop_eta, MIN_UPDATES_FOR_STOP, MAX_LEAD_TIME}`.
- Produces:
  - `scale::ScaleSample = (Instant, f32)`; the scale channel carries it
  - `scale::StopReason { Prediction, RawWeight, Timeout }` (Serialize, Format, Clone, Copy, PartialEq, Eq, Debug)
  - `scale::EtaReading { median, lo, hi: f32 }` (Serialize), `From<grindy_gp::Eta>`
  - `WeightReading::new(timestamp_ms: u64, weight: f32, state: UserEvent, coffee_weight: Option<f32>, filtered_weight: Option<f32>, eta: Option<EtaReading>)`, fields serialized in that order
  - `GrinderStateMachine::get_lead_time(&self) -> f32`
  - `storage::read_lead_time(&mut FlashStorage) -> Option<f32>`

This firmware task has no host tests. It is verified by building, and on the device at the end.

- [ ] **Step 1: Record the warning baseline**

Run: `cargo build --release 2>&1 | grep -c '^warning'`
Note the number. Later steps must not increase it.

- [ ] **Step 2: Add the dependency and the storage sector**

In the root `Cargo.toml` `[dependencies]`, add:

```toml
grindy-gp = { path = "grindy-gp" }
```

In `src/storage.rs`, add after `TARGET_WEIGHT_MAGIC`:

```rust
/// Offset of the lead time sector, right after the target weight one.
const LEAD_TIME_OFFSET: u32 = TARGET_WEIGHT_OFFSET + ERASE_SIZE as u32;

const LEAD_TIME_MAGIC: u32 = 0x6772_7461; // "grta"
```

and after `write_target_weight`:

```rust
/// Reads the lead time previously stored with `write_lead_time`. Returns
/// `None` if nothing has been stored yet (or the stored data is corrupt or
/// out of range).
pub fn read_lead_time(flash: &mut FlashStorage) -> Option<f32> {
    let lead_time = read_f32(flash, LEAD_TIME_OFFSET, LEAD_TIME_MAGIC)
        .filter(|lead_time| (0.0..=grindy_gp::lead_time::MAX_LEAD_TIME).contains(lead_time));
    match lead_time {
        Some(lead_time) => info!("Loaded lead time {}s from flash", lead_time),
        None => info!("No lead time stored in flash yet"),
    }
    lead_time
}
```

- [ ] **Step 3: Timestamp the scale samples**

In `src/scale.rs`, add the imports:

```rust
use grindy_gp::lead_time;
use grindy_gp::{GrindEstimator, DEFAULT_LEAD_TIME, FITTED, RAMP_UP};
```

After `pub const SCALE_CHANNEL_SIZE: usize = 5;`, add:

```rust
/// A raw scale reading and when it was taken.
pub type ScaleSample = (Instant, f32);
```

In `scale_task`, change the sender parameter type to
`channel::Sender<'static, CriticalSectionRawMutex, ScaleSample, SCALE_CHANNEL_SIZE>` and the send to:

```rust
                    sender.send((Instant::now(), -r as f32)).await;
```

In `src/main.rs`, import `ScaleSample` from `crate::scale` and change the channel to:

```rust
    static SCALE_CHANNEL: channel::Channel<CriticalSectionRawMutex, ScaleSample, SCALE_CHANNEL_SIZE> =
        channel::Channel::new();
```

- [ ] **Step 4: Extend `WeightReading` and add `StopReason`/`EtaReading`**

In `src/scale.rs`, replace the `WeightReading` struct and its `impl` with:

```rust
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
```

- [ ] **Step 5: Add the lead time and estimator to the state machine**

In `GrinderStateMachine`, add the field after `target_weight`:

```rust
    /// Seconds of flow still arriving after the grinder stops.
    lead_time: f32,
```

In `GrinderStateMachine::new`, load it next to `target_weight`:

```rust
        let lead_time = flash
            .lock(|flash| storage::read_lead_time(&mut flash.borrow_mut()))
            .unwrap_or(DEFAULT_LEAD_TIME);
```

and add `lead_time,` to the `Self { … }` initializer.

Add the getter after `get_target_weight`:

```rust
    pub fn get_lead_time(&self) -> f32 {
        self.lead_time
    }
```

Change the `Grinding` variant of `GrinderState` to:

```rust
    Grinding {
        start_time: Instant,
        portafilter_weight: f32,
        estimator: GrindEstimator,
    },
```

Add a helper after `get_coffee_weight`:

```rust
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
```

- [ ] **Step 6: Drive `update_weight` with sample times and the estimator**

Change the signature to `fn update_weight(&mut self, time: Instant, raw_weight: f32)`.

In the `Stabilizing` arm, replace the transition to `Grinding` with:

```rust
                    GrinderState::Grinding {
                        start_time: time,
                        portafilter_weight,
                        estimator: GrindEstimator::new(FITTED),
                    }
```

Replace the whole `Grinding` arm with:

```rust
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
                            "Stopping grinder ({}) at {}g coffee, lead time {}s",
                            stop_reason, coffee_weight, self.lead_time
                        );
                        self.grinder.set_high();
                        GrinderState::WaitingForRemoval { portafilter_weight }
                    }
                    None => GrinderState::Grinding {
                        start_time,
                        portafilter_weight,
                        estimator,
                    },
                }
            }
```

- [ ] **Step 7: Publish the estimates from `controller_task`**

Change the `scale_receiver` parameter type to
`channel::Receiver<'static, CriticalSectionRawMutex, ScaleSample, SCALE_CHANNEL_SIZE>`. Replace the loop body with:

```rust
        let (time, raw_weight) = scale_receiver.receive().await;
        let (event, weight, coffee_weight, target_weight, estimate) = {
            let mut grinder_state_machine_guard = grinder_state_machine.lock().await;
            grinder_state_machine_guard.update_weight(time, raw_weight);
            let event = grinder_state_machine_guard.as_user_event();
            let weight = grinder_state_machine_guard
                .scale_setting
                .translate(raw_weight);
            let coffee_weight = grinder_state_machine_guard.get_coffee_weight(weight);
            let target_weight = grinder_state_machine_guard.get_target_weight();
            let estimate = grinder_state_machine_guard.grind_estimate();
            (event, weight, coffee_weight, target_weight, estimate)
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
        weight_sender.try_send(reading).ok();
```

- [ ] **Step 8: Build and check for warnings**

Run: `cargo build --release 2>&1 | grep -E '^(warning|error)' ; cargo build --release 2>&1 | grep -c '^warning'`
Expected: no errors, and the warning count equals the baseline from Step 1.
Also run: `cargo test-gp`. Expected: PASS (unchanged).

- [ ] **Step 9: Commit** (only if the project is a git repository)

```bash
git add Cargo.toml Cargo.lock src
git commit -m "Stop grinding on the GP forecast and publish estimates"
```

---

### Task 6: Firmware: settle window, lead-time learning and `GrindFinished`

**Files:**
- Modify: `src/storage.rs`, `src/scale.rs`, `src/web.rs`, `src/main.rs`

**Interfaces:**
- Consumes: Task 5 items; `grindy_gp::lead_time::{observe_lead_time, update_lead_time}`.
- Produces:
  - `storage::write_lead_time(&mut FlashStorage, f32) -> bool`
  - `scale::GrindFinished { stop_reason: StopReason, stop_weight: f32, settled_weight: Option<f32>, lead_time_observed: Option<f32>, lead_time: f32 }` (Serialize, Clone, Copy), fields in that order
  - `scale::ControllerEvent { Reading(WeightReading), GrindFinished(GrindFinished) }` (Clone, Copy); `scale::CONTROLLER_EVENT_CHANNEL_SIZE = 4` replaces `WEIGHT_CHANNEL_SIZE`
  - `GrinderStateMachine::update_weight(&mut self, time: Instant, raw_weight: f32) -> Option<GrindFinished>`
  - `WsMessage::{Connected, StateChange}` gain a final field `lead_time: f32`; new final variant `WsMessage::GrindFinished(GrindFinished)` (index 4)

This firmware task has no host tests. It is verified by building, and on the device at the end.

- [ ] **Step 1: Add `write_lead_time`**

In `src/storage.rs`, after `read_lead_time`:

```rust
/// Persists `lead_time` so it can be recovered on the next boot with
/// [`read_lead_time`]. Returns `false` if writing failed.
pub fn write_lead_time(flash: &mut FlashStorage, lead_time: f32) -> bool {
    let ok = write_f32(flash, LEAD_TIME_OFFSET, LEAD_TIME_MAGIC, lead_time);
    if ok {
        info!("Stored lead time {}s to flash", lead_time);
    }
    ok
}
```

Also change the doc comment of `read_lead_time` to link it: ``/// Reads the lead time previously stored with [`write_lead_time`]. …``

- [ ] **Step 2: Add the settle types and `ControllerEvent`**

In `src/scale.rs`, replace `pub const WEIGHT_CHANNEL_SIZE: usize = 4;` with:

```rust
pub const CONTROLLER_EVENT_CHANNEL_SIZE: usize = 4;
```

After `StopReason`, add:

```rust
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
```

Change the `WaitingForRemoval` variant of `GrinderState` to:

```rust
    WaitingForRemoval {
        portafilter_weight: f32,
        /// Present after a grind until the settle window has passed.
        settle: Option<Settle>,
    },
```

Add a field `stored_lead_time: f32` to `GrinderStateMachine` after `lead_time`, initialized with `stored_lead_time: lead_time,` in `new`.

- [ ] **Step 3: Share the stability threshold**

Add to `impl GrinderStateMachine`:

```rust
    /// Largest deviation from the mean weight (3 sd of the scale noise) that
    /// still counts as stable.
    fn stability_threshold(&self) -> f32 {
        math::sqrt(1.0 / self.scale_setting.inv_variance * self.scale_setting.factor.powi(2)) * 3.0
    }
```

In the `Calibrating` and `Stabilizing` arms, replace the `let threshold = math::sqrt(…) * 3.0;` expressions with `let threshold = self.stability_threshold();`.

- [ ] **Step 4: Learn the lead time from a settled grind**

Add to `impl GrinderStateMachine`:

```rust
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
            None => info!(
                "Not learning the lead time from this grind (settled weight {}g)",
                settled_weight
            ),
        }
        settle.finish(settled_weight, lead_time_observed, self.lead_time)
    }
```

- [ ] **Step 5: Wire the settle window into `update_weight`**

Change the signature to
`fn update_weight(&mut self, time: Instant, raw_weight: f32) -> Option<GrindFinished>`. Add `let mut finished = None;` before `self.state = Some(match …`, and end the function with `finished` after the assignment.

In the `Calibrating` arm, change the transition to:

```rust
                    GrinderState::WaitingForRemoval {
                        portafilter_weight: 0.0,
                        settle: None,
                    }
```

In the `Grinding` arm (from Task 5), change the stop branch's new state to:

```rust
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
```

Replace the `WaitingForRemoval` arm with:

```rust
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
```

- [ ] **Step 6: Send `ControllerEvent`s from `controller_task`**

Change the `weight_sender` parameter to:

```rust
    event_sender: channel::Sender<
        'static,
        CriticalSectionRawMutex,
        ControllerEvent,
        CONTROLLER_EVENT_CHANNEL_SIZE,
    >,
```

In the loop, capture the result:
`let finished = grinder_state_machine_guard.update_weight(time, raw_weight);`, return it from the lock block as an extra tuple element `finished`, and replace `weight_sender.try_send(reading).ok();` with:

```rust
        event_sender.try_send(ControllerEvent::Reading(reading)).ok();
        if let Some(finished) = finished {
            // Once per grind, so wait for room rather than dropping it.
            event_sender.send(ControllerEvent::GrindFinished(finished)).await;
        }
```

- [ ] **Step 7: Update `web.rs` and `main.rs`**

In `src/web.rs`:
- Import `ControllerEvent, GrindFinished, CONTROLLER_EVENT_CHANNEL_SIZE` from `crate::scale` instead of `WeightReading, WEIGHT_CHANNEL_SIZE` (keep `WeightReading`, since `WsMessage::Weight` uses it).
- Change `WsMessage` to:

```rust
#[derive(Serialize, Clone)]
enum WsMessage {
    Connected {
        state: UserEvent,
        scale_setting: ScaleSetting,
        target_weight: f32,
        timestamp_ms: u64,
        lead_time: f32,
    },
    StateChange {
        state: UserEvent,
        scale_setting: ScaleSetting,
        target_weight: f32,
        timestamp_ms: u64,
        lead_time: f32,
    },
    Weight(WeightReading),
    TargetWeightChanged {
        target_weight: f32,
    },
    GrindFinished(GrindFinished),
}
```

- In `GrinderWebSocket::run`, read `grinder_state_machine.get_lead_time()` together with the other values and pass `lead_time` into `WsMessage::Connected`.
- In `websocket_broadcaster_task`, rename the parameter to `event_receiver: channel::Receiver<'static, CriticalSectionRawMutex, ControllerEvent, CONTROLLER_EVENT_CHANNEL_SIZE>`. Read `get_lead_time()` next to `get_target_weight()` for `StateChange`, and replace the weight branch with:

```rust
            // Listen for controller events
            async {
                let msg = match event_receiver.receive().await {
                    ControllerEvent::Reading(reading) => WsMessage::Weight(reading),
                    ControllerEvent::GrindFinished(finished) => WsMessage::GrindFinished(finished),
                };
                let registry = ws_registry.lock().await;
                registry.broadcast(&msg);
            },
```

In `src/main.rs`, import `ControllerEvent, CONTROLLER_EVENT_CHANNEL_SIZE` instead of `WeightReading, WEIGHT_CHANNEL_SIZE`, and rename the channel:

```rust
    static CONTROLLER_EVENT_CHANNEL: channel::Channel<
        CriticalSectionRawMutex,
        ControllerEvent,
        CONTROLLER_EVENT_CHANNEL_SIZE,
    > = channel::Channel::new();
```

Pass `CONTROLLER_EVENT_CHANNEL.receiver()` to `websocket_broadcaster_task` and `CONTROLLER_EVENT_CHANNEL.sender()` to `controller_task`.

- [ ] **Step 8: Build and check for warnings**

Run: `cargo build --release 2>&1 | grep -E '^(warning|error)' ; cargo build --release 2>&1 | grep -c '^warning'`
Expected: no errors, and the warning count equals the Task 5 baseline.

- [ ] **Step 9: Commit** (only if the project is a git repository)

```bash
git add src
git commit -m "Learn the lead time from the settled weight and report finished grinds"
```

---

### Task 7: Web page: decode and show ETA, lead time and grind summary

**Files:**
- Modify: `src/index.js`, `src/index.html`

**Interfaces:**
- Consumes: the wire format from Tasks 5–6 (see Global Constraints).
- Produces: decoded message fields `reading.filteredWeight`, `reading.eta = {median, lo, hi} | null`, `msg.leadTime` on `connected`/`stateChange`, a message `{type: 'grindFinished', stopReason, stopWeight, settledWeight, leadTimeObserved, leadTime}`.

- [ ] **Step 1: Extend the decoder**

In `src/index.js`, in `PostcardDecoder`, replace `readWeightReading` and add the helpers:

```js
  // Decode EtaReading struct (seconds until the grinder is expected to stop)
  readEta() {
    return { median: this.readF32(), lo: this.readF32(), hi: this.readF32() };
  }

  // Decode StopReason enum (varint tag: 0=Prediction, 1=RawWeight, 2=Timeout)
  readStopReason() {
    return ['Prediction', 'RawWeight', 'Timeout'][this.readVarint()] || 'Unknown';
  }

  // Decode WeightReading struct
  readWeightReading() {
    return {
      timestampMs: this.readVarint(),
      weight: this.readF32(),
      state: this.readUserEvent(),
      coffeeWeight: this.readOption(this.readF32),
      filteredWeight: this.readOption(this.readF32),
      eta: this.readOption(this.readEta)
    };
  }
```

In `readWsMessage`, add `leadTime: this.readF32()` after `timestampMs` in both the `Connected` and the `StateChange` case, update the tag comment to `3=TargetWeightChanged, 4=GrindFinished`, and add before `default`:

```js
      case 4: // GrindFinished
        return {
          type: 'grindFinished',
          stopReason: this.readStopReason(),
          stopWeight: this.readF32(),
          settledWeight: this.readOption(this.readF32),
          leadTimeObserved: this.readOption(this.readF32),
          leadTime: this.readF32()
        };
```

- [ ] **Step 2: Add the metrics to the page**

In `src/index.html`, insert after the "Grind Progress" metric `div`:

```html
<div class="metric">
<div class="metric-label">Stops in</div>
<div class="metric-value" id="eta">--</div>
<div class="metric-label" id="eta-range">seconds</div>
</div>
<div class="metric">
<div class="metric-label">Lead time</div>
<div class="metric-value" id="lead-time">--</div>
<div class="metric-label">seconds</div>
</div>
```

- [ ] **Step 3: Show the values**

In `src/index.js`, add after `updateUI`:

```js
const stopReasonNames = {
  Prediction: 'forecast',
  RawWeight: 'weight reading',
  Timeout: 'timeout',
  Unknown: 'unknown reason'
};

function setLeadTime(leadTime) {
  document.getElementById('lead-time').textContent = leadTime.toFixed(2);
}

function updateEta(eta) {
  const value = document.getElementById('eta');
  const range = document.getElementById('eta-range');
  if (eta === null) {
    value.textContent = '--';
    range.textContent = 'seconds';
    return;
  }
  value.textContent = eta.median.toFixed(1);
  range.textContent = `seconds (${eta.lo.toFixed(1)}–${eta.hi.toFixed(1)})`;
}

function grindFinishedSummary(msg) {
  const target = targetWeight || 18.0;
  const settled = msg.settledWeight === null
    ? 'weight did not settle'
    : `settled at ${msg.settledWeight.toFixed(2)} g (target ${target.toFixed(1)} g)`;
  const learned = msg.leadTimeObserved === null
    ? `lead time stays ${msg.leadTime.toFixed(2)} s`
    : `lead time ${msg.leadTimeObserved.toFixed(2)} s observed, now ${msg.leadTime.toFixed(2)} s`;
  return `Stopped by ${stopReasonNames[msg.stopReason]} at ${msg.stopWeight.toFixed(2)} g, ` +
    `${settled}, ${learned}`;
}

function handleGrindFinished(msg) {
  setLeadTime(msg.leadTime);
  addLog(grindFinishedSummary(msg));
  if (grindTrace) {
    grindTrace.finished = msg;
    drawGrindChart();
  }
}
```

In `drawGrindChart`, replace the summary `if/else` with:

```js
  if (grindTrace.endMs === null) {
    summary.textContent = `Grinding... ${lastPoint.weight.toFixed(1)} g after ${duration.toFixed(1)} s`;
  } else if (grindTrace.finished) {
    summary.textContent = grindFinishedSummary(grindTrace.finished);
  } else {
    const grindTime = (grindTrace.endMs - grindTrace.startMs) / 1000;
    summary.textContent = `Last grind: ${lastPoint.weight.toFixed(1)} g in ${grindTime.toFixed(1)} s`;
  }
```

In `recordGrindReading`, add `finished: null` to the new `grindTrace` object literal.

In `handleWeight`, replace the coffee-weight branch and the final call with:

```js
  if (reading.coffeeWeight !== undefined && reading.coffeeWeight !== null) {
    displayWeight = reading.filteredWeight ?? reading.coffeeWeight;
    progress = Math.min(100, Math.round((displayWeight / TARGET_WEIGHT) * 100));
  } else if (reading.state === 'Grinding') {
    // Fallback: assume total weight includes ~100g portafilter
    const estimatedCoffeeWeight = Math.max(0, reading.weight - 100);
    displayWeight = estimatedCoffeeWeight;
    progress = Math.min(100, Math.round((estimatedCoffeeWeight / TARGET_WEIGHT) * 100));
  }

  updateEta(reading.eta);
  updateUI(reading.state, displayWeight, progress);
```

In `handleMessage`, add `setLeadTime(msg.leadTime);` to the `connected` and `stateChange` cases, and add:

```js
      case 'grindFinished':
        handleGrindFinished(msg);
        break;
```

- [ ] **Step 4: Check syntax and build**

Run: `node --check src/index.js && cargo build --release 2>&1 | grep -E '^error' ; echo done`
Expected: no syntax error, no build error. (The page is embedded with `include_str!`.)

- [ ] **Step 5: Commit** (only if the project is a git repository)

```bash
git add src/index.js src/index.html
git commit -m "Show grind ETA, lead time and grind summary on the web page"
```

---

### Task 8: Logger: decode and store the new fields

**Files:**
- Modify: `logger/src/grindy_logger/models.py`, `logger/src/grindy_logger/writer.py`, `logger/README.md`
- Test: `logger/tests/test_models.py`

**Interfaces:**
- Consumes: the wire format from Tasks 5–6.
- Produces:
  - Python `Eta(median, lo, hi)`; `WeightReading.filtered_weight`, `.eta`; `ConnectedMessage.lead_time`, `StateChangeMessage.lead_time`
  - `StopReason(IntEnum)`, `GrindFinishedMessage(stop_reason, stop_weight, settled_weight, lead_time_observed, lead_time)`
  - Parquet columns `filtered_weight, eta_median, eta_lo, eta_hi, lead_time, stop_reason, stop_weight, settled_weight, lead_time_observed`

- [ ] **Step 1: Write the failing tests**

Create `logger/tests/test_models.py`:

```python
import struct

import pyarrow.parquet as pq
import pytest

from grindy_logger.models import (
    GrindFinishedMessage,
    StateChangeMessage,
    StopReason,
    UserEvent,
    WeightMessage,
    parse_message,
)
from grindy_logger.writer import ArrowWriter


def varint(n: int) -> bytes:
    out = bytearray()
    while True:
        byte = n & 0x7F
        n >>= 7
        if n:
            out.append(byte | 0x80)
        else:
            out.append(byte)
            return bytes(out)


def f32(x: float) -> bytes:
    return struct.pack("<f", x)


def some_f32(x: float) -> bytes:
    return b"\x01" + f32(x)


NONE = b"\x00"


def weight_with_estimate() -> bytes:
    return (
        varint(2) + varint(123456) + f32(650.5) + varint(UserEvent.Grinding)
        + some_f32(10.25) + some_f32(10.0) + b"\x01" + f32(3.0) + f32(2.5) + f32(3.5)
    )


def grind_finished() -> bytes:
    return (
        varint(4) + varint(StopReason.Prediction) + f32(17.75)
        + some_f32(18.0) + some_f32(0.5) + f32(0.48)
    )


def test_weight_reading_with_estimate():
    msg = parse_message(weight_with_estimate())
    assert isinstance(msg, WeightMessage)
    r = msg.reading
    assert r.timestamp_ms == 123456
    assert r.state == UserEvent.Grinding
    assert r.coffee_weight == pytest.approx(10.25)
    assert r.filtered_weight == pytest.approx(10.0)
    assert (r.eta.median, r.eta.lo, r.eta.hi) == pytest.approx((3.0, 2.5, 3.5))


def test_weight_reading_without_estimate():
    data = varint(2) + varint(1) + f32(1.0) + varint(UserEvent.Idle) + NONE + NONE + NONE
    r = parse_message(data).reading
    assert r.coffee_weight is None and r.filtered_weight is None and r.eta is None


def test_state_change_with_lead_time():
    data = (
        varint(1) + varint(UserEvent.Grinding) + f32(1.0) + f32(2.0) + f32(3.0)
        + f32(18.0) + varint(99) + f32(0.45)
    )
    msg = parse_message(data)
    assert isinstance(msg, StateChangeMessage)
    assert msg.timestamp_ms == 99
    assert msg.lead_time == pytest.approx(0.45)


def test_grind_finished():
    msg = parse_message(grind_finished())
    assert isinstance(msg, GrindFinishedMessage)
    assert msg.stop_reason == StopReason.Prediction
    assert msg.stop_weight == pytest.approx(17.75)
    assert msg.settled_weight == pytest.approx(18.0)
    assert msg.lead_time_observed == pytest.approx(0.5)
    assert msg.lead_time == pytest.approx(0.48)


def test_grind_finished_without_settled_weight():
    data = varint(4) + varint(StopReason.Timeout) + f32(12.0) + NONE + NONE + f32(0.5)
    msg = parse_message(data)
    assert msg.stop_reason == StopReason.Timeout
    assert msg.settled_weight is None and msg.lead_time_observed is None


def test_writer_stores_new_columns(tmp_path):
    path = tmp_path / "out.parquet"
    with ArrowWriter(str(path)) as writer:
        writer.add_message(parse_message(weight_with_estimate()))
        writer.add_message(parse_message(grind_finished()))
    rows = pq.read_table(path).to_pylist()
    reading, finished = rows
    assert reading["filtered_weight"] == pytest.approx(10.0)
    assert (reading["eta_median"], reading["eta_lo"], reading["eta_hi"]) == pytest.approx((3.0, 2.5, 3.5))
    assert finished["message_type"] == "GrindFinished"
    assert finished["timestamp_ms"] == 123456
    assert finished["stop_reason"] == "Prediction"
    assert finished["stop_weight"] == pytest.approx(17.75)
    assert finished["settled_weight"] == pytest.approx(18.0)
    assert finished["lead_time_observed"] == pytest.approx(0.5)
    assert finished["lead_time"] == pytest.approx(0.48)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd logger && uv run --with pytest pytest -q`
Expected: an ImportError for `GrindFinishedMessage` / `StopReason`.

- [ ] **Step 3: Update the models**

In `logger/src/grindy_logger/models.py`:

Add after `UserEvent`:

```python
class StopReason(IntEnum):
    """Why the grinder stopped, matching the Rust StopReason enum."""
    Prediction = 0
    RawWeight = 1
    Timeout = 2


@dataclass
class Eta:
    """Seconds until the grinder is expected to stop (50/10/90 % quantiles)."""
    median: float
    lo: float
    hi: float
```

Add to `WeightReading` after `coffee_weight`:

```python
    filtered_weight: Optional[float]
    eta: Optional[Eta]
```

Add `lead_time: float` as the last field of `ConnectedMessage` and of `StateChangeMessage`.

Add after `TargetWeightChangedMessage`:

```python
@dataclass
class GrindFinishedMessage:
    """Summary of a finished grind, sent once the weight settled."""
    stop_reason: StopReason
    stop_weight: float
    settled_weight: Optional[float]
    lead_time_observed: Optional[float]
    lead_time: float
```

and add `GrindFinishedMessage` to the `WsMessage` union.

In `PostcardDecoder`, add:

```python
    def read_eta(self) -> Eta:
        """Decode EtaReading struct."""
        return Eta(median=self.read_f32(), lo=self.read_f32(), hi=self.read_f32())
```

Extend `read_weight_reading` with `filtered_weight=self.read_option(self.read_f32), eta=self.read_option(self.read_eta)` after `coffee_weight`. Add `lead_time=self.read_f32()` after `timestamp_ms` in variants 0 and 1. Add before the `else`:

```python
        elif variant == 4:
            return GrindFinishedMessage(
                stop_reason=StopReason(self.read_varint()),
                stop_weight=self.read_f32(),
                settled_weight=self.read_option(self.read_f32),
                lead_time_observed=self.read_option(self.read_f32),
                lead_time=self.read_f32(),
            )
```

Update the `parse_message` docstring to list GrindFinished.

- [ ] **Step 4: Update the writer**

In `logger/src/grindy_logger/writer.py`, import `GrindFinishedMessage`. Append to `SCHEMA`:

```python
    ("filtered_weight", pa.float32()),       # GP estimate of the coffee weight while grinding (nullable)
    ("eta_median", pa.float32()),            # Seconds until the grinder stops, median (nullable)
    ("eta_lo", pa.float32()),                # ... 10 % quantile (nullable)
    ("eta_hi", pa.float32()),                # ... 90 % quantile (nullable)
    ("lead_time", pa.float32()),             # Lead time in seconds (nullable)
    ("stop_reason", pa.string()),            # "Prediction" | "RawWeight" | "Timeout" (GrindFinished only)
    ("stop_weight", pa.float32()),           # Coffee weight when the grinder stopped (GrindFinished only)
    ("settled_weight", pa.float32()),        # Settled coffee weight (GrindFinished only, nullable)
    ("lead_time_observed", pa.float32()),    # Lead time observed in the grind (GrindFinished only, nullable)
```

and update the `message_type` comment to include `"GrindFinished"`.

Replace `_add_record` with a version that fills every column:

```python
    def _add_record(
        self,
        timestamp_ms: int,
        received_at: float,
        message_type: str,
        state: Optional[str],
        scale_setting: Optional[ScaleSetting] = None,
        **fields,
    ) -> None:
        """Add a single record to the batch; columns not given are null."""
        unknown = set(fields) - set(SCHEMA.names)
        assert not unknown, f"unknown columns {unknown}"
        self.last_timestamp_ms = max(self.last_timestamp_ms, timestamp_ms)
        record = dict.fromkeys(SCHEMA.names)
        record.update(
            timestamp_ms=timestamp_ms,
            received_at=received_at,
            message_type=message_type,
            state=state,
            **fields,
        )
        if scale_setting:
            record.update(
                scale_offset=scale_setting.offset,
                scale_inv_variance=scale_setting.inv_variance,
                scale_factor=scale_setting.factor,
            )
        self.batch.append(record)
```

In `add_message`, pass `lead_time=message.lead_time` for Connected/StateChange. For `WeightMessage`, add:

```python
                filtered_weight=reading.filtered_weight,
                eta_median=reading.eta.median if reading.eta else None,
                eta_lo=reading.eta.lo if reading.eta else None,
                eta_hi=reading.eta.hi if reading.eta else None,
```

and add a branch before the flush:

```python
        elif isinstance(message, GrindFinishedMessage):
            # Carries no device timestamp; reuse the last one seen.
            self._add_record(
                timestamp_ms=self.last_timestamp_ms,
                received_at=received_at,
                message_type="GrindFinished",
                state=None,
                stop_reason=message.stop_reason.name,
                stop_weight=message.stop_weight,
                settled_weight=message.settled_weight,
                lead_time_observed=message.lead_time_observed,
                lead_time=message.lead_time,
            )
```

- [ ] **Step 5: Run the tests and make sure they pass**

Run: `cd logger && uv run --with pytest pytest -q`
Expected: 6 passed.

- [ ] **Step 6: Update the logger README**

In `logger/README.md`:
- Add rows for the nine new columns to the Output Format table, with the descriptions from the `SCHEMA` comments.
- Add `"GrindFinished"` to the `message_type` row.
- Under Message Types, change "four types" to "five types" and add:
  `5. **GrindFinished**: Sent once per grind after the weight settled (or the portafilter was removed first): why the grinder stopped, the stop and settled coffee weights, and the observed and learned lead time. Like TargetWeightChanged it carries no device timestamp.`
- Mention that `Weight` now also carries the GP's filtered weight and ETA, and that Connected/StateChange carry the lead time.

- [ ] **Step 7: Commit** (only if the project is a git repository)

```bash
git add logger
git commit -m "Log grind estimates and finished grinds"
```

---

### Task 9: Docs and final verification

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Update CLAUDE.md**

Make these changes:
- **Build and Development Commands:** add a "Testing" subsection: `cargo test-gp` runs the host tests of `grindy-gp` (estimator, batch-GP equivalence, replay over `logs/`). Also `cd logger && uv run --with pytest pytest -q`. Refitting after new logs: `uv run tools/gp/fit.py && uv run tools/gp/export_fixtures.py`.
- **Key Architecture:** add a "Grind estimator" subsection. `grindy-gp/` is a `no_std` workspace crate with an integrated-OU GP computed exactly by a Kalman filter on `[weight, rate, mean rate]`. It provides the stop rule (`forecast(now + lead_time) ≥ target`) and lead-time learning. Hyperparameters live in the generated `grindy-gp/src/fitted.rs`. Link the spec.
- **State Machine:** in Grinding, the estimator is fed after `RAMP_UP` (1 s), and the grind stops by prediction, raw weight or timeout (`StopReason`). In WaitingForRemoval, the settle window (1.5–2.5 s after the stop) measures the settled weight and learns the lead time (EMA 0.2, stored in flash).
- **Communication Architecture:** the scale channel carries `ScaleSample = (Instant, f32)`. `controller_task → websocket_broadcaster_task` becomes `channel::Channel<ControllerEvent>` (size 4): one `Reading` per sample plus `GrindFinished` once per grind. LED progress uses the filtered weight.
- **Network/flash:** the lead-time sector follows the target-weight sector (magic "grta").
- **Important Constants:** add `RAMP_UP`, `MIN_UPDATES_FOR_STOP`, `MAX_LEAD_TIME`, `DEFAULT_LEAD_TIME` (fitted), and the settle window.
- **Code Organization:** the line ranges are stale. Replace the section with a short per-file list (`src/main.rs`, `scale.rs`, `web.rs`, `ui.rs`, `storage.rs`, `wifi.rs`, `grindy-gp/`, `tools/gp/`, `logger/`) without line numbers.

- [ ] **Step 2: Full verification**

Run each and confirm:
- `cargo test-gp -- --nocapture` → all PASS; note the replay numbers.
- `cargo build --release 2>&1 | grep -c '^warning'` → equals the Task 5 baseline; no errors.
- `cd logger && uv run --with pytest pytest -q` → 6 passed.
- `node --check src/index.js` → no output.

- [ ] **Step 3: Report**

Report to the user:
- the replay numbers (overall MAE, last-second MAE, coverage, bias with and without skipping the ramp-up, lead time per grind);
- the fitted `FITTED` and `DEFAULT_LEAD_TIME`;
- that `mean_rate_prior_var` was fixed at 0.1² in `tools/gp/fit.py`, and why;
- what they need to check on the device: flash with `cargo run --release`; run the logger (`grindy-logger run.parquet`); do several grinds; check that `stop_reason` is mostly `Prediction`, `|settled_weight − target| ≤ 0.2 g` on most grinds, and `lead_time` settles down; watch that the "Stops in" ETA on the page counts down sensibly.

- [ ] **Step 4: Commit** (only if the project is a git repository)

```bash
git add CLAUDE.md
git commit -m "Document the GP grind estimator"
```
