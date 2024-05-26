use std::collections::HashMap;
use nyx::cosmic::SPEED_OF_LIGHT;
use crate::prelude::{Duration, Epoch, Candidate};

/// Data averager
struct Averager {
    y: f64,
    n: u64,
}

impl Averager {
    /// Builds new Averager
    pub fn new() -> Self {
        Self {
            y: 0.0,
            n: 0,
        }
    }
    /// Updates average value, taking new `value` into account
    pub fn average(&mut self, x: f64) -> f64 {
        self.y = (x + (self.n as f64) * self.y) / (self.n +1) as f64;
        self.n += 1;
        self.y
    }
    /// Hard reset
    pub fn reset(&mut self) {
        self.y = 0.0;
        self.n = 0;
    }
}

struct Buffer<T> {
    window: Duration,
    gap_tolerance: Duration,
    inner: Vec<(Epoch, T)>,
}

impl<T> Buffer<T> {
    /// Allocates a new Buffer
    pub fn malloc(capacity: usize, window: Duration, gap_tolerance: Duration) -> Self {
        Self {
            window,
            gap_tolerance,
            inner: Vec::with_capacity(capacity),
        }
    }
    /// Push data into self
    pub fn push(&mut self, t: Epoch, y: T) {
        if let Some((t_last, _)) = self.inner.last() {
            if (t - t_last) > self.gap_tolerance {
                error!("{}: buffer reset on data gap");
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
    pub fn std_mean(&self) -> T {
        let mut v = T::default();
        for i in 0..self.inner.len() {
            v += self.inner[i];
        }
        v /= self.inner.len() as f64
    }
    /// Returns std_dev over self
    pub fn std_dev(&self) -> T {
        let mean = self.std_mean();
        let mut v = T::default();
        for i in 0..self.inner.len() {
            v += (self.inner[i] - mean).powi(2);
        }
        v /= self.inner.len() as f64
    }   
    /// Performs polyfit (nth order) over self
    pub fn polyfit(&self, order: usize) -> &[f64] {
        let x_s = self.inner.iter().map(|(k, _)| k).collect();
        let y_s = self.inner.iter().map(|(_, v)| v).collect();
        polyfit(self.inner, x_s, y_s)
    }
}

/// [AmbiguitySolver] resolves phase range ambiguities in real time.
pub struct AmbiguitySolver {
    /// Sampling interval [Duration]
    sampling_interval: Duration,
    /// [SV] last seen [Epoch]
    last_seen: Option<Epoch>,
    /// MW tracker per [SV]
    mw_trackers: Averager,
    /// N_1 tracker per [SV]
    n1_trackers: Averager,
    /// GF moving window
    gf_buffers: Buffer<f64>,
}

type Ambiguities = HashMap<(SV, Carrier), f64>;

impl AmbiguitySolver {
    /// Builds new [AmbiguitySolver] to work with given sample interval [Duration].
    pub fn new(sampling_interval: Duration) -> Self {
        Self {
            sampling_interval,
            last_seen: HashMap::with_capacity(16),
            gf_buffers: HashMap::with_capacity(16),
            n1_trackers: HashMap::with_capacity(16),
            mw_trackers: HashMap::with_capacity(16),
        }
    }
    /// Resolve [Ambiguities]
    pub fn resolve(&mut self, pool: &mut [Candidate]) -> Result<Ambiguities, Error> {
        // 1. account for possible new SV
        for cd in pool {
            if self.mw_trackers.get_mut(&cd.sv).is_none() {
                self.last_seen.insert(*cd.sv, pool.t_rx);
                self.mw_trackers.insert(*cd.sv, Tracker::new());
                self.n1_trackers.insert(*cd.sv, Tracker::new());
                self.gf_buffers.insert(*cd.sv, Buffer::malloc(128)),
            }
            // manage loss of sight
            let last_seen = self.last_seen.get(&cd.sv)
                .unwrap();
            if cd.t_rx - last_seen > self.sampling_interval {
                warn!("{}({}): tracker - loss of sight", cd.t_rx, cd.sv);
                // TODO: reset all trackers
            }
        }

        // 3. proceed
        for cd in pool {
            if let Some(cmb) = cd.phase_gf_combination() {
                let mut gf_buffer = self.gf_trackers.get_mut(&cd.sv).unwrap();
                gf_buffer.push(cd.t_rx, cmb.value);
                let (a, b, c) = gf
                // TODO polyfit, threshold.. declare CS.. reset
            }
            if let Some(cmb) = cd.mw_combination() {
                // MW tracker
                let mut mw_tracker = self.mw_tracker.get_mut(&cd.sv).unwrap();
                let (l_i, l_j) = (cmb.reference.frequency(), cmb.lhs.frequency());
                let lambda_w = SPEED_OF_LIGHT / (l_1 - l_j);
                let nw = mw_tracker.average(cmb.value / lambda_w);

                // N_1 tracker
                let l_1 = cd.phase_range.iter().filter(|p| p.carrier == cmb.reference).reduce(|k, _| k).unwrap();
                let l_j = cd.phase_range.iter().filter(|p| p.carrier == cmb.lhs).reduce(|k, _| k).unwrap();
                let mut n1_tracker = self.n1_trackers.get_mut(&cd.sv).unwrap();
                let n1 = n1_tracker.average((l_1 - l_j - lambda_j * nw) / (lambda_1 - lambda_j));

                let n2 = n1 - nw;
                debug!("{}({}}: nw: {} | n1: {} | n2: ", cd.t_rx, cd.sv, nw, n1, n2);

                // TODO: fix ambiguities (&mut pool)
            }
        }

        Ok(Ambiguities::default())
    }
}

#[cfg(test)]
mod test {
    use super::Averager;
    #[test]
    fn test_averager() {
        let mut avg = Averager::new();
        for (value, expect) in [
            (1.0, 1.0),
        ] {
            assert_eq!(averager.average(value), expect, "failed for +={}={}", value, expect);
        }
    }  
}
