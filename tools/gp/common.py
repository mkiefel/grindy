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
