use crate::prelude::{Candidate, Carrier, Duration, Epoch, Error, SV};
use log::{debug, error, warn};
use nyx::cosmic::SPEED_OF_LIGHT;
use std::collections::HashMap;

/// Data averager
struct Averager {
    y: f64,
    n: u64,
}

impl Averager {
    /// Builds new Averager
    pub fn new() -> Self {
        Self { y: 0.0, n: 0 }
    }
    /// Updates average value, taking new `value` into account
    pub fn average(&mut self, x: f64) -> f64 {
        self.y = (x + (self.n as f64) * self.y) / (self.n + 1) as f64;
        self.n += 1;
        self.y
    }
    /// Hard reset
    pub fn reset(&mut self) {
        self.y = 0.0;
        self.n = 0;
    }
}

struct Buffer {
    window: Duration,
    gap_tolerance: Duration,
    inner: Vec<(Epoch, f64)>,
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
        let t0 = self.inner[0].0;
        if (t0 - t) > self.window {
            self.inner.remove(0);
        }
        self.inner.push((t, y));
    }
    /// Resets self
    pub fn reset(&mut self) {
        self.inner.clear();
    }
    /// Returns current number of symbols
    pub fn len(&self) -> usize {
        self.inner.len()
    }
    /// Returns std_mean over self
    pub fn std_mean(&self) -> f64 {
        let mut v = 0.0_f64;
        for i in 0..self.inner.len() {
            v += self.inner[i].1;
        }
        v / self.inner.len() as f64
    }
    /// Returns std_dev over self
    pub fn std_dev(&self) -> f64 {
        let mean = self.std_mean();
        let mut v = 0.0_f64;
        for i in 0..self.inner.len() {
            v += (self.inner[i].1 - mean).powi(2);
        }
        v / self.inner.len() as f64
    }
    // /// Performs polyfit (nth order) over self
    // pub fn polyfit(&self, order: usize) -> &[f64] {
    //     let x_s = self.inner.iter().map(|(k, _)| k).collect();
    //     let y_s = self.inner.iter().map(|(_, v)| v).collect();
    //     polyfit(self.inner, x_s, y_s)
    // }
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
    pub fn new(last_seen: Epoch) -> Self {
        Self {
            last_seen: Some(last_seen),
            n1_tracker: Averager::new(),
            mw_tracker: Averager::new(),
            gf_buffer: Buffer::malloc(128, Duration::from_hours(1.0), Duration::from_seconds(30.0)),
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
    sampling_interval: Duration,
    /// [SV] tracker
    sv_trackers: HashMap<SV, SVTracker>,
    /// [SV] out of sight
    untracked: Vec<SV>,
}

impl AmbiguitySolver {
    /// Builds new [AmbiguitySolver] to work with given sample interval [Duration].
    pub fn new(sampling_interval: Duration) -> Self {
        Self {
            sampling_interval,
            untracked: Vec::with_capacity(8),
            sv_trackers: HashMap::with_capacity(8),
        }
    }
    /// Resolve [Ambiguities]
    pub fn resolve(&mut self, pool: &mut [Candidate]) -> Result<(), Error> {
        // 1. account for possible new SV
        for cd in pool {
            match self.sv_trackers.get_mut(&cd.sv) {
                Some(tracker) => {
                    // manage loss of sight
                    if let Some(last_seen) = tracker.last_seen {
                        let dt = cd.t - last_seen;
                        if dt > self.sampling_interval && !self.untracked.contains(&cd.sv) {
                            warn!("{}({}): tracker reset - {} loss of sight", cd.t, cd.sv, dt);
                            tracker.reset();
                            self.untracked.push(cd.sv);
                        } else {
                            self.untracked.retain(|sv| *sv != cd.sv);
                        }
                    }
                    // proceed
                    if !self.untracked.contains(&cd.sv) {
                        if let Some(cmb) = cd.mw_combination() {
                            let (l_1, l_j) = (cmb.reference.frequency(), cmb.lhs.frequency());
                            let (lambda_1, lambda_j) =
                                (cmb.reference.wavelength(), cmb.lhs.wavelength());
                            let lambda_w = SPEED_OF_LIGHT / (l_1 + l_j);
                            let lamba_w = SPEED_OF_LIGHT / (l_1 - l_j);
                            let n_w = tracker.mw_tracker.average(cmb.value / lambda_w);
                            debug!("{}({}): n_w: {}", cd.t, cd.sv, n_w);
                        } else {
                            error!("{}({}): fail to form mw - missing signal", cd.t, cd.sv);
                        }
                    }
                },
                None => {
                    self.sv_trackers.insert(cd.sv, SVTracker::new(cd.t));
                    self.untracked.retain(|sv| *sv != cd.sv);
                },
            }
        }

        //// 3. proceed
        //for cd in pool {
        //    if let Some(cmb) = cd.phase_gf_combination() {
        //        let mut gf_buffer = self.gf_trackers.get_mut(&cd.sv).unwrap();
        //        gf_buffer.push(cd.t_rx, cmb.value);
        //        // let (a, b, c) = gf
        //        // TODO polyfit, threshold.. declare CS.. reset
        //    }
        //    if let Some(cmb) = cd.mw_combination() {
        //        // MW tracker
        //        // N_1 tracker
        //        let l_1 = cd
        //            .phase_range
        //            .iter()
        //            .filter(|p| p.carrier == cmb.reference)
        //            .reduce(|k, _| k)
        //            .unwrap();
        //        let l_j = cd
        //            .phase_range
        //            .iter()
        //            .filter(|p| p.carrier == cmb.lhs)
        //            .reduce(|k, _| k)
        //            .unwrap();
        //        let mut n1_tracker = self.n1_trackers.get_mut(&cd.sv).unwrap();
        //        let n1 = n1_tracker.average((l_1.value - l_j.value - lambda_j * nw) / (lambda_1 - lambda_j));

        //        let n2 = n1 - nw;
        //        debug!("{}({}}: nw: {} | n1: {} | n2: {}", cd.t_rx, cd.sv, nw, n1, n2);

        //        // TODO: fix ambiguities (&mut pool)
        //    }
        //}

        Ok(())
        //Ok(Ambiguities::default())
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
                avg.average(value),
                expect,
                "failed for +={}={}",
                value,
                expect
            );
        }
    }
}
