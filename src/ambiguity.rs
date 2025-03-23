use crate::prelude::{Duration, Epoch, SPEED_OF_LIGHT_M_S};

use log::{error, warn};
use polyfit_rs::polyfit_rs::polyfit;

#[derive(Debug, Clone)]
struct Averager {
    pub m2: f64,
    pub mean: f64,
    pub count: u64,
}

impl Averager {
    /// Builds new Averager
    pub fn new() -> Self {
        Self {
            count: 0,
            mean: 0.0,
            m2: 0.0,
        }
    }

    /// Push new value into [Averager]
    pub fn add(&mut self, x: f64) {
        self.count += 1;
        let dx = x - self.mean;
        self.mean += dx / self.count as f64;
        let dx2 = x - self.mean;
        self.m2 += dx * dx2;
    }

    /// Reset [Averager]
    pub fn reset(&mut self) {
        self.count = 0;
        self.m2 = 0.0;
        self.mean = 0.0;
    }

    pub fn sigma(&self) -> f64 {
        (self.m2 / self.count as f64).sqrt()
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

#[derive(Debug, Clone)]
pub struct Input {
    pub f1: f64,
    pub c1: f64,
    pub l1: f64,
    pub f2: f64,
    pub c2: f64,
    pub l2: f64,
}

#[derive(Debug, Default, Clone)]
pub struct Output {
    pub n1: i32,
    pub n2: i32,
    pub sigma_nw: f64,
    pub sigma_n1: f64,
}

#[derive(Debug, Clone)]
pub struct Solver {
    nw: i32,
    nw_avg: Averager,
    n1_avg: Averager,
    cn_avg: Averager,
    l1_avg: Averager,
}

impl Solver {
    pub fn new() -> Self {
        Self {
            nw: 0,
            nw_avg: Averager::new(),
            cn_avg: Averager::new(),
            l1_avg: Averager::new(),
            n1_avg: Averager::new(),
        }
    }

    pub fn reset(&mut self) {
        self.nw = 0;
        self.nw_avg.reset();
        self.cn_avg.reset();
        self.l1_avg.reset();
    }

    pub fn solve(&mut self, input: Input) -> Output {
        let lambda_1 = SPEED_OF_LIGHT_M_S / input.f1;
        let lambda_2 = SPEED_OF_LIGHT_M_S / input.f2;
        let lambda_w = SPEED_OF_LIGHT_M_S / (input.f1 - input.f2);

        let lw = (input.f1 * input.l1 - input.f2 * input.l2) / (input.f1 - input.f2);
        let cn = (input.f1 * input.c1 + input.f2 * input.c2) / (input.f1 + input.f2);

        self.cn_avg.add(cn);
        self.l1_avg.add(input.l1);
        self.nw_avg.add((lw - cn) / lambda_w);

        self.nw = self.nw_avg.mean.round() as i32;

        let n1 = (input.l1 - input.l2 - lambda_2 * self.nw as f64) / (lambda_1 - lambda_2);

        self.n1_avg.add(n1);

        let sigma_nw = self.cn_avg.sigma() / lambda_w;
        let sigma_n1 = 2.0_f64.sqrt() * self.l1_avg.sigma() / (lambda_1 - lambda_2);

        let n1 = self.n1_avg.mean.round() as i32;
        let n2 = n1 - self.nw;

        Output {
            n1,
            n2,
            sigma_n1,
            sigma_nw,
        }
    }
}

// /// [AmbiguitySolver] resolves phase range ambiguities in real time.
// pub struct AmbiguitySolver {
//     /// Sampling interval [Duration]
//     gap_tolerance: Duration,
//     /// [SV] tracker
//     sv_trackers: HashMap<SV, SVTracker>,
//     /// [SV] out of sight
//     untracked: Vec<SV>,
// }

// impl AmbiguitySolver {
//     /// Builds new [AmbiguitySolver] to work with given sample interval [Duration].
//     pub fn new(gap_tolerance: Duration) -> Self {
//         Self {
//             gap_tolerance,
//             untracked: Vec::with_capacity(8),
//             sv_trackers: HashMap::with_capacity(8),
//         }
//     }
//     /// Resolve [Ambiguities]
//     pub fn resolve(&mut self, pool: &[Candidate]) -> Ambiguities {
//         let mut ambiguities = Ambiguities::with_capacity(pool.len());

//         for cd in pool {
//             if self.sv_trackers.get(&cd.sv).is_none() {
//                 self.sv_trackers
//                     .insert(cd.sv, SVTracker::new(cd.t, self.gap_tolerance));
//                 self.untracked.retain(|sv| *sv != cd.sv);
//             }

//             let sv_tracker = self.sv_trackers.get_mut(&cd.sv).unwrap();

//             // manage loss of sight
//             if let Some(last_seen) = sv_tracker.last_seen {
//                 let dt = cd.t - last_seen;
//                 if dt > self.gap_tolerance && !self.untracked.contains(&cd.sv) {
//                     warn!("{}({}): tracker reset - {} loss of sight", cd.t, cd.sv, dt);
//                     sv_tracker.reset();
//                     self.untracked.push(cd.sv);
//                 } else {
//                     sv_tracker.last_seen = Some(cd.t);
//                     self.untracked.retain(|sv| *sv != cd.sv);
//                 }
//             }

//             // proceed
//             if !self.untracked.contains(&cd.sv) {
//                 if let Some(cmb) = cd.phase_gf_combination() {
//                     if let Some(fit) = sv_tracker.gf_buffer.polyfit(2) {
//                         let t = cd.t.duration.to_seconds();
//                         // let n = sv_tracker.gf_buffer.inner.len();
//                         // let dt = t - sv_tracker.gf_buffer.inner[n - 2].0.duration.to_seconds();
//                         let (a2, a1, a0) = (fit[2], fit[1], fit[0]);
//                         let predicted = a2 * t.powi(2) + a1 * t + a0;
//                         let err = (cmb.value - predicted).abs();
//                         // let t0 = 60.0;
//                         // let a0 = (cmb.lhs.wavelength() - cmb.rhs.wavelength()) * 3.0 / 2.0;
//                         // let threshold = a0 - a0 / 2.0 * (-dt / t0).exp();
//                         let threshold = 5.0;
//                         if err > threshold {
//                             debug!(
//                                 "{}({}) gf cycle slip declared: {}/{}",
//                                 cd.t, cd.sv, err, threshold
//                             );
//                             sv_tracker.mw_tracker.reset();
//                         }
//                     }
//                     sv_tracker.gf_buffer.push(cd.t, cmb.value);
//                 } else {
//                     error!(
//                         "{}({}): failed to form gf comb (missing signal)",
//                         cd.t, cd.sv
//                     );
//                 }
//                 if let Some(cmb) = cd.mw_combination() {
//                     let (f_1, f_j) = (cmb.rhs.frequency(), cmb.lhs.frequency());
//                     let lambda_w = SPEED_OF_LIGHT_M_S / (f_1 + f_j);
//                     let (n_w, sigma_n_w) = sv_tracker.mw_tracker.average(cmb.value / lambda_w, 0.0);
//                     let n_w = n_w.round();

//                     let ((l_1, ph_1), (l_j, ph_j)) =
//                         (cd.l1_phaserange().unwrap(), cd.lj_phaserange().unwrap());
//                     let (lambda_1, lambda_2) = (l_1.wavelength(), l_j.wavelength());
//                     let (n_1, sigma_n_1) = sv_tracker
//                         .n1_tracker
//                         .average((ph_1 - ph_j - lambda_2 * n_w) / (lambda_1 - lambda_2), 0.0);

//                     let n_1 = n_1.round();
//                     let n_2 = (n_1 - n_w).round();

//                     if sv_tracker.mw_tracker.n > 10 {
//                         debug!(
//                             "{}({}): n_w: {}({}), n_1: {}({}) n_2: {}",
//                             cd.t, cd.sv, n_w, sigma_n_w, n_1, sigma_n_1, n_2
//                         );

//                         let ambiguity = Ambiguity { n_1, n_2, n_w };
//                         ambiguities.insert((cd.sv, l_1), ambiguity);
//                     }
//                 } else {
//                     error!("{}({}): fail to form mw comb (missing signal)", cd.t, cd.sv);
//                 }
//             }
//         }
//         ambiguities
//     }
// }

#[cfg(test)]
mod test {
    use super::Averager;

    #[test]
    fn test_averager() {
        let mut avg = Averager::new();

        for (x_i, mean, sigma) in [(1.0, 1.0, 0.0), (0.5, 0.75, 0.25)] {
            avg.add(x_i);
            assert_eq!(avg.mean, mean);
            assert_eq!(avg.sigma(), sigma);
        }
    }
}
