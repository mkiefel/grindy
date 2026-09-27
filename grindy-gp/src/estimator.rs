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
