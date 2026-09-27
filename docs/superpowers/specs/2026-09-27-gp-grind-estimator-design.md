# GP grind estimator — design

Date: 2026-09-27
Status: approved design, awaiting spec review

## Goal

Replace grindy's "stop when the reading ≥ target" rule with a Gaussian-process
estimate of the coffee weight over time, so that

1. the grinder stops early enough that the **settled** weight hits the target
   (today it consistently overshoots by +0.2 to +0.6 g, mean ≈ +0.4 g), and
2. the web page shows an ETA ("stops in 3.4 s (3.0–3.9)") and the web page and
   LED strip show a smoothed progress.

The estimate must be correct for irregular sample times and gaps, and account
for the scale noise.

Inspiration: `inference.js` of mlProgressBar (OU-kernel GP on download rates,
O(1) recursive update, unknown mean rate `beta`).

## Findings that shaped the design

From the 9 grinds in `logs/` (all 18 g target, ~90 ms sampling, gaps up to
6 s in some sessions):

- Settled weight 18.19–18.57 g; the last reading while grinding is ≈ 17.9 g.
  About 0.07 g of the overshoot is stop latency, the rest is coffee still
  arriving after the relay opens (~0.5 s worth of flow).
- Marginal-likelihood fit of the model below (profiled over ℓ) strongly
  prefers ℓ → 0 (ℓ ≈ 0.01 s, σ ≈ 1.5 g/s, sensor noise sd ≈ 0.024 g,
  mean rate 0.74 g/s, √v₀ → 0). At this resolution the weight behaves like
  Brownian motion with drift 0.74 g/s and diffusion 2σ²ℓ ≈ 0.047 g²/s.
- Shoot-out (ETA to W = 15 g / 17 g, leave-one-grind-out fits):
  - A (the Kalman filter below): overall ETA MAE 0.68–0.89 s, 0.10 s MAE in
    the last second, 80 % interval coverage 68–80 %; unchanged with 70 % of
    samples dropped plus 2 s gaps.
  - B (port of `inference.js` on differenced weights): 2.5 s MAE even with
    parameters tuned on the test data; 18–77 s on raw 90 ms differences.
  - Naive (remaining / average rate so far): ties A in the last few seconds,
    1.5–1.7 s overall MAE, no uncertainty.
  - A shows a +0.2 to +0.6 s ETA bias at mid range, attributed to the grind's
    ramp-up (the rate starts near 0).
- Irreducible stop precision: the random walk alone gives ≈ 0.15 g sd over
  0.5 s, so the realistic goal is settled weight within ≈ ±0.2 g of target.

## Model

The flow rate r is an Ornstein–Uhlenbeck process around an unknown mean μ;
the weight w is its integral; readings y are noisy.

```
dr = -(r-μ)/ℓ dt + √(2σ²/ℓ) dW
w(t) = w₀ + ∫ r dt
y = w + ε,   ε ~ N(0, s²)
μ ~ N(m₀, v₀)
```

Equivalent GP on the weight (shown anchored at w(0) = 0, prior mean m₀·t;
the implementation instead puts a broad prior on w₀, which adds a constant
to the kernel):

```
k_w(t,t') = σ² [2ℓ·min(t,t') − ℓ²(1 − e^{-t/ℓ} − e^{-t'/ℓ} + e^{-|t−t'|/ℓ})] + v₀·t·t'
```

Inference is exact via a Kalman filter on x = [w, r, μ]. For a step Δt with
a = e^{-Δt/ℓ}:

```
F = ⎡1  ℓ(1-a)  Δt-ℓ(1-a)⎤      Q = σ² ⎡ℓ(2Δt-ℓ(3-4a+a²))  ℓ(1-a)²  0⎤
    ⎢0  a       1-a      ⎥             ⎢ℓ(1-a)²            1-a²     0⎥
    ⎣0  0       1        ⎦             ⎣0                  0        0⎦
```

Update with H = [1 0 0]: a scalar innovation variance S = P⁻_ww + s², no
matrix inversion. Forecasts at t + h apply F(h), Q(h) once without an
update. The filter's prediction-error decomposition gives the GP marginal
likelihood used for fitting. Kalman and batch-GP results were checked to be
identical on log data.

Hyperparameters are fitted offline and compiled in; μ is estimated online.
Not in scope: Student-t / online hyperparameter learning, smoothing,
modelling the ramp-up.

## Components

### 1. `grindy-gp` crate (new workspace member)

`no_std`, depends only on `libm`. The root `Cargo.toml` becomes a workspace.
Host tests run via a cargo alias `cargo test-gp`
(`test -p grindy-gp --target <host triple>`), since `.cargo/config.toml` pins
the build target to thumbv8m.

```rust
pub struct Params {
    pub length_scale: f32,        // ℓ, s
    pub rate_var: f32,            // σ², (g/s)²
    pub noise_var: f32,           // s², g²
    pub mean_rate_prior: f32,     // m₀, g/s
    pub mean_rate_prior_var: f32, // v₀, (g/s)²
}
pub const FITTED: Params = …; // output of tools/gp/fit.py

pub struct Gaussian { pub mean: f32, pub var: f32 }
pub struct Eta { pub median: f32, pub lo: f32, pub hi: f32 } // s from t_last; 50/10/90 %

pub struct GrindEstimator { /* params, t_last, m: [f32; 3], p: [[f32; 3]; 3], updates: u32 */ }

impl GrindEstimator {
    pub fn new(params: Params) -> Self;
    pub fn update(&mut self, t: f32, weight: f32);
    pub fn updates(&self) -> u32;
    pub fn weight(&self) -> Gaussian;             // filtered weight now
    pub fn forecast(&self, t: f32) -> Gaussian;   // weight at absolute t ≥ t_last
    pub fn eta(&self, target: f32) -> Option<Eta>;
}
```

- **Anchoring:** the weight starts with a broad prior (not w = 0), and the
  first `update` anchors it, so the caller can start feeding at any time.
  The rate starts at N(m₀, σ² + v₀), correlated with μ via v₀.
- **`eta`:** P(T ≤ h) ≈ Φ((mean_h − target)/sd_h). The median is where the
  forecast mean crosses the target. The median, 10 % and 90 % points are found
  by a coarse grid over h ∈ (0, 60 s] refined by bisection. Returns `None`
  if the target isn't reached within 60 s.
- **Numerics (f32):** P is kept symmetric after each update, diagonal
  variances are capped from below, `update` with t ≤ t_last is ignored, and
  a → 0 for Δt ≫ ℓ is fine.

`lead_time` module (the pure decision logic, so it can be tested on the host):

```rust
pub const DEFAULT_LEAD_TIME: f32 = …; // output of tools/gp/fit.py
pub const MIN_UPDATES_FOR_STOP: u32 = 5;
pub fn should_stop(est: &GrindEstimator, now: f32, lead_time: f32, target: f32) -> bool;
    // est.updates() ≥ MIN_UPDATES_FOR_STOP && est.forecast(now + lead_time).mean ≥ target
pub fn observe_lead_time(est_at_stop: &GrindEstimator, settled: f32) -> Option<f32>;
    // est_at_stop.eta(settled).median, accepted only within [0, 2] s
pub fn update_lead_time(lead_time: f32, observed: f32) -> f32;
    // lead_time + 0.2 * (observed - lead_time)
```

### 2. Firmware changes (`src/`)

**`scale_task`** sends `(Instant, f32)`, timestamped at the HX711 read; the
scale channel type changes accordingly.

**`GrinderStateMachine`** gets a `lead_time: f32`, loaded from flash with
`DEFAULT_LEAD_TIME` as the fallback.

**`Grinding { start, portafilter_weight, estimator }`**: each sample
computes coffee weight = reading − portafilter_weight. Samples within
`RAMP_UP = 1.0 s` of `start` don't update the estimator (the constant is
validated by the replay test). The grind stops at the first of:

1. `should_stop(...)` → `StopReason::Prediction`
2. coffee weight ≥ target (the current rule, kept as a safety net) →
   `StopReason::RawWeight`
3. `MAX_GRIND_TIME_IN_SECS` → `StopReason::Timeout`

The stop reason is logged via defmt. The target is read on each sample, so
changing it mid-grind takes effect immediately.

**`WaitingForRemoval { portafilter_weight, settle: Option<Settle> }`** with
`Settle { stop_time, stop_reason, stop_weight, estimator_at_stop, samples }`:

- Readings from 1.5 s to 2.5 s after the stop are collected. The settled
  weight is their MAD-filtered mean (`compute_mean_variance`).
- τ is updated only if all hold: the stop reason isn't `Timeout`; the window
  is stable (the same 3σ check as Stabilizing); `observe_lead_time` returns
  `Some`. Then `lead_time = update_lead_time(...)`, written to flash if the
  change exceeds 0.01 s.
- `GrindFinished` is sent (see below) when the window finishes, or with
  `None`s if the grind is rejected or the portafilter is removed first.
- Removal (weight < `REMOVAL_THRESHOLD`) still moves on to
  `WaitingForPortafilter` at any time.
- The calibration path that ends in `WaitingForRemoval` uses `settle: None`.

**`storage.rs`**: a new sector after the target-weight sector, magic
`0x6772_7461` ("grta"), with `read_lead_time` / `write_lead_time` built on
the existing `read_f32` / `write_f32`. Values outside [0, 2] s are ignored
on read.

### 3. Output

postcard is not self-describing: every wire change updates `src/web.rs`,
`src/index.js` and `logger/src/grindy_logger/{models,writer}.py` together.
New fields are appended at the end; the new variant takes the next index.

- `WeightReading` += `filtered_weight: Option<f32>`, `eta: Option<Eta>`.
  Both are `None` outside Grinding and before `MIN_UPDATES_FOR_STOP` updates.
  `eta` is the time until the **stop**: `eta(target)` minus `lead_time`,
  floored at 0 (median, lo and hi).
- `Connected`, `StateChange` += `lead_time: f32`.
- New `WsMessage::GrindFinished { stop_reason: StopReason, stop_weight: f32,
  settled_weight: Option<f32>, lead_time_observed: Option<f32>,
  lead_time: f32 }` (variant 4). It travels through the existing controller →
  broadcaster channel, whose item type becomes a `ControllerEvent` enum
  (`Reading(WeightReading)` | `GrindFinished(...)`).
- **Web page:** while grinding it shows "stops in X s (lo–hi)"; progress %
  and the weight shown use `filtered_weight` when present; after a grind it
  shows the settled weight vs target and the lead time.
- **LED strip:** progress = `filtered_weight / target` while grinding, falling
  back to the raw coffee weight until the estimator is ready. The existing
  hysteresis stays.
- **Logger:** new nullable Parquet columns `filtered_weight`, `eta_median`,
  `eta_lo`, `eta_hi`, `lead_time`, `stop_reason`, `settled_weight`,
  `lead_time_observed`; `message_type` gains "GrindFinished". README schema
  updated.

### 4. Tooling (`tools/gp/`, uv scripts with inline dependencies, run by hand)

- `export_fixtures.py`: logs → `grindy-gp/tests/data/grind_*.csv`
  (t, coffee_weight while grinding, plus the settled weight) and
  `reference.csv` (batch-GP forecast means and variances for one grind,
  computed from the full Gram-matrix solve).
- `fit.py`: marginal-likelihood fit of `Params` across all logged grinds,
  plus `DEFAULT_LEAD_TIME` as the mean τ_obs over the logs; prints the Rust
  constants.

## Testing

Host tests (`cargo test-gp`):

1. **Equivalence with the batch GP:** filter forecasts match `reference.csv`
   within f32 tolerance.
2. **Invariants:** P stays symmetric PSD over 2000 random-Δt steps; t ≤ t_last
   is ignored; `eta` is `None` for a flat signal; `eta` increases with the
   target; anchoring works.
3. **Replay regression over the 9 logs** (W = 17 g, estimator fed after
   `RAMP_UP`): overall ETA MAE ≤ 0.8 s; MAE ≤ 0.2 s in the last second;
   80 % interval coverage within [65 %, 90 %]; mid-range (3–12 s) bias no
   worse than without skipping the ramp-up.
4. **Lead time:** `observe_lead_time` on each logged grind (estimator at the
   actual stop sample, settled weight from the log) lies in [0.2, 1.0] s;
   rejections work on synthetic out-of-range cases; `update_lead_time` is
   the EMA.

Firmware: `cargo build --release` passes with no new warnings.

On the device (done by the user): several grinds with the logger on.
Success = |settled − target| ≤ 0.2 g on most grinds, and `lead_time`
converging.

## Docs

`CLAUDE.md` (architecture, crate, constants, the new flash sector, test
command) and `logger/README.md` (schema, GrindFinished).
