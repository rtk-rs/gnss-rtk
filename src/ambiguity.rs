use crate::prelude::{Duration, Epoch, SPEED_OF_LIGHT_M_S};

use log::{error, warn};
use polyfit_rs::polyfit_rs::polyfit;

#[derive(Debug, Clone)]
struct Averager {
    pub mean: f64,
    pub count: u64,
}

impl Averager {
    /// Builds new Averager
    pub fn new() -> Self {
        Self {
            count: 0,
            mean: 0.0,
        }
    }

    /// Push new value into [Averager]
    pub fn add(&mut self, x: f64) {
        self.count += 1;
        let k = self.count as f64;
        self.mean = x / k + self.mean * (k - 1.0) / k;
    }

    /// Reset [Averager]
    pub fn reset(&mut self) {
        self.count = 0;
        self.mean = 0.0;
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
    // pub sigma_nw: f64,
    // pub sigma_n1: f64,
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

        // let sigma_nw = self.cn_avg.sigma() / lambda_w;
        // let sigma_n1 = 2.0_f64.sqrt() * self.l1_avg.sigma() / (lambda_1 - lambda_2);

        let n1 = self.n1_avg.mean.round() as i32;
        let n2 = n1 - self.nw;

        Output {
            n1,
            n2,
            // sigma_n1,
            // sigma_nw,
        }
    }
}

#[cfg(test)]
mod test {
    use super::Averager;

    #[test]
    fn test_averager() {
        let mut avg = Averager::new();

        for (x_i, mean, sigma) in [(1.0, 1.0, 0.0), (0.5, 0.75, 0.25)] {
            avg.add(x_i);
            assert_eq!(avg.mean, mean);
        }
    }
}
