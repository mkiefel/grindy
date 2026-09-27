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
