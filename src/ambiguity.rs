use crate::prelude::{Candidate, Carrier, Duration, Epoch, SV}; // Error
use log::{debug, error, warn};
use nyx::cosmic::SPEED_OF_LIGHT;
use polyfit_rs::polyfit_rs::polyfit;
use std::collections::HashMap;

/// Ambiguity, per SV and reference signal
pub type Ambiguities = HashMap<(SV, Carrier), Ambiguity>;

#[derive(Clone, Debug)]
pub struct Ambiguity {
    /// Reference signal ambiguity
    pub n_1: f64,
    /// Subsidary signal ambiguity
    pub n_2: f64,
    /// MW ambiguity
    pub n_w: f64,
}

/// Data averager
struct Averager {
    y: f64,
    sigma: f64,
    pub n: u64,
}

impl Averager {
    /// Builds new Averager
    pub fn new() -> Self {
        Self {
            y: 0.0,
            sigma: 0.0,
            n: 0,
        }
    }
    /// Updates average value, taking new `value` into account
    pub fn average(&mut self, x: f64, sigma: f64) -> (f64, f64) {
        self.y = (x + (self.n as f64) * self.y) / (self.n + 1) as f64;
        self.sigma = (sigma + (self.n as f64) * self.sigma) / (self.n + 1) as f64;
        self.n += 1;
        (self.y, self.sigma)
    }
    /// Hard reset
    pub fn reset(&mut self) {
        self.n = 0;
        self.y = 0.0;
        self.sigma = 0.0;
    }
}

struct Buffer {
    window: Duration,
    gap_tolerance: Duration,
    pub inner: Vec<(Epoch, f64)>,
}

impl Buffer {
    /// Allocates a new Buffer
    pub fn malloc(capacity: usize, window: Duration, gap_tolerance: Duration) -> Self {
        Self {
            window,
            gap_tolerance,
            inner: Vec::with_capacity(capacity),
        }
    }
    /// Push data into self
    pub fn push(&mut self, t: Epoch, y: f64) {
        if let Some((t_last, _)) = self.inner.last() {
            if (t - *t_last) > self.gap_tolerance {
                error!("{}: buffer reset on data gap", t);
                self.reset();
            }
        }
        self.inner.push((t, y));

        let t0 = self.inner[0].0;
        if (t - t0) > self.window {
            self.inner.remove(0);
        }
    }
    /// Resets self
    pub fn reset(&mut self) {
        self.inner.clear();
    }
    /// Returns current number of symbols
    pub fn len(&self) -> usize {
        self.inner.len()
    }
    // /// Returns std_mean over self
    // pub fn std_mean(&self) -> f64 {
    //     let mut v = 0.0_f64;
    //     for i in 0..self.inner.len() {
    //         v += self.inner[i].1;
    //     }
    //     v / self.inner.len() as f64
    // }
    // /// Returns std_dev over self
    // pub fn std_dev(&self) -> f64 {
    //     let mean = self.std_mean();
    //     let mut v = 0.0_f64;
    //     for i in 0..self.inner.len() {
    //         v += (self.inner[i].1 - mean).powi(2);
    //     }
    //     v / self.inner.len() as f64
    // }
    /// Performs polyfit (nth order) over self
    pub fn polyfit(&self, order: usize) -> Option<Vec<f64>> {
        if self.inner.len() > order {
            let x_s = self
                .inner
                .iter()
                .map(|(k, _)| k.duration.to_seconds())
                .collect::<Vec<f64>>();
            let y_s = self.inner.iter().map(|(_, v)| *v).collect::<Vec<f64>>();
            match polyfit(&x_s, &y_s, order) {
                Ok(fit) => Some(fit),
                Err(e) => {
                    warn!("polyfit error: {}", e);
                    None
                },
            }
        } else {
            None
        }
    }
}

struct SVTracker {
    /// last seen [Epoch]
    pub last_seen: Option<Epoch>,
    /// MW tracker per [SV]
    pub mw_tracker: Averager,
    /// N_1 tracker per [SV]
    pub n1_tracker: Averager,
    /// GF moving window
    pub gf_buffer: Buffer,
}

impl SVTracker {
    pub fn new(last_seen: Epoch, gap_tolerance: Duration) -> Self {
        Self {
            last_seen: Some(last_seen),
            n1_tracker: Averager::new(),
            mw_tracker: Averager::new(),
            gf_buffer: Buffer::malloc(
                128,
                //TODO: programmble
                Duration::from_seconds(1000.0),
                gap_tolerance,
            ),
        }
    }
    pub fn reset(&mut self) {
        self.n1_tracker.reset();
        self.mw_tracker.reset();
        self.gf_buffer.reset();
    }
}

/// [AmbiguitySolver] resolves phase range ambiguities in real time.
pub struct AmbiguitySolver {
    /// Sampling interval [Duration]
    gap_tolerance: Duration,
    /// [SV] tracker
    sv_trackers: HashMap<SV, SVTracker>,
    /// [SV] out of sight
    untracked: Vec<SV>,
}

impl AmbiguitySolver {
    /// Builds new [AmbiguitySolver] to work with given sample interval [Duration].
    pub fn new(gap_tolerance: Duration) -> Self {
        Self {
            gap_tolerance,
            untracked: Vec::with_capacity(8),
            sv_trackers: HashMap::with_capacity(8),
        }
    }
    /// Resolve [Ambiguities]
    pub fn resolve(&mut self, pool: &[Candidate]) -> Ambiguities {
        let mut ambiguities = Ambiguities::with_capacity(pool.len());

        for cd in pool {
            if self.sv_trackers.get(&cd.sv).is_none() {
                self.sv_trackers
                    .insert(cd.sv, SVTracker::new(cd.t, self.gap_tolerance));
                self.untracked.retain(|sv| *sv != cd.sv);
            }

            let sv_tracker = self.sv_trackers.get_mut(&cd.sv).unwrap();

            // manage loss of sight
            if let Some(last_seen) = sv_tracker.last_seen {
                let dt = cd.t - last_seen;
                if dt > self.gap_tolerance && !self.untracked.contains(&cd.sv) {
                    warn!("{}({}): tracker reset - {} loss of sight", cd.t, cd.sv, dt);
                    sv_tracker.reset();
                    self.untracked.push(cd.sv);
                } else {
                    sv_tracker.last_seen = Some(cd.t);
                    self.untracked.retain(|sv| *sv != cd.sv);
                }
            }

            // proceed
            if !self.untracked.contains(&cd.sv) {
                if let Some(cmb) = cd.phase_gf_combination() {
                    if let Some(fit) = sv_tracker.gf_buffer.polyfit(2) {
                        let t = cd.t.duration.to_seconds();
                        // let n = sv_tracker.gf_buffer.inner.len();
                        // let dt = t - sv_tracker.gf_buffer.inner[n - 2].0.duration.to_seconds();
                        let (a2, a1, a0) = (fit[2], fit[1], fit[0]);
                        let predicted = a2 * t.powi(2) + a1 * t + a0;
                        let err = (cmb.value - predicted).abs();
                        // let t0 = 60.0;
                        // let a0 = (cmb.lhs.wavelength() - cmb.reference.wavelength()) * 3.0 / 2.0;
                        // let threshold = a0 - a0 / 2.0 * (-dt / t0).exp();
                        let threshold = 5.0;
                        if err > threshold {
                            debug!(
                                "{}({}) gf cycle slip declared: {}/{}",
                                cd.t, cd.sv, err, threshold
                            );
                            sv_tracker.mw_tracker.reset();
                        }
                    }
                    sv_tracker.gf_buffer.push(cd.t, cmb.value);
                } else {
                    error!(
                        "{}({}): failed to form gf comb (missing signal)",
                        cd.t, cd.sv
                    );
                }
                if let Some(cmb) = cd.mw_combination() {
                    let (f_1, f_j) = (cmb.reference.frequency(), cmb.lhs.frequency());
                    let lambda_w = SPEED_OF_LIGHT / (f_1 + f_j);
                    let lamba_w = SPEED_OF_LIGHT / (f_1 - f_j);
                    let (n_w, sigma_n_w) = sv_tracker.mw_tracker.average(cmb.value / lambda_w, 0.0);
                    let n_w = n_w.round();

                    let (l_1, l_j) = (cd.l1_phaserange().unwrap(), cd.lj_phaserange().unwrap());
                    let (lambda_1, lambda_2) = (l_1.carrier.wavelength(), l_j.carrier.wavelength());
                    let (n_1, sigma_n_1) = sv_tracker.n1_tracker.average(
                        (l_1.value - l_j.value - lambda_2 * n_w) / (lambda_1 - lambda_2),
                        0.0,
                    );

                    let n_1 = n_1.round();
                    let n_2 = (n_1 - n_w).round();

                    if sv_tracker.mw_tracker.n > 10 {
                        debug!(
                            "{}({}): n_w: {}({}), n_1: {}({}) n_2: {}",
                            cd.t, cd.sv, n_w, sigma_n_w, n_1, sigma_n_1, n_2
                        );

                        let ambiguity = Ambiguity { n_1, n_2, n_w };
                        ambiguities.insert((cd.sv, l_1.carrier), ambiguity);
                    }
                } else {
                    error!("{}({}): fail to form mw comb (missing signal)", cd.t, cd.sv);
                }
            }
        }
        ambiguities
    }
}

#[cfg(test)]
mod test {
    use super::Averager;
    #[test]
    fn test_averager() {
        let mut avg = Averager::new();
        for (value, expect) in [(1.0, 1.0)] {
            assert_eq!(
                avg.average(value, 0.0).0,
                expect,
                "failed for +={}={}",
                value,
                expect
            );
        }
    }
}
