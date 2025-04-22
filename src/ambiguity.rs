use crate::{
    averager::Averager,
    constants::SPEED_OF_LIGHT_M_S,
    prelude::{Duration, Epoch},
};

use log::{error, warn};
use polyfit_rs::polyfit_rs::polyfit;

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
    pub f1_hz: f64,
    pub c1: f64,
    pub l1: f64,
    pub f2_hz: f64,
    pub c2: f64,
    pub l2: f64,
}

#[derive(Debug, Default, Clone)]
pub struct Output {
    pub n1: u32,
    pub n2: u32,
}

#[derive(Debug, Clone)]
pub struct Solver {
    nw_avg: Averager,
    n1_avg: Averager,
}

impl Solver {
    pub fn new() -> Self {
        Self {
            nw_avg: Averager::new(0.05),
            n1_avg: Averager::new(0.05),
        }
    }

    pub fn reset(&mut self) {
        self.nw_avg.reset();
        self.n1_avg.reset();
    }

    pub fn solve(&mut self, input: Input) -> Option<Output> {
        let lambda_1 = SPEED_OF_LIGHT_M_S / input.f1_hz;
        let lambda_2 = SPEED_OF_LIGHT_M_S / input.f2_hz;
        let lambda_wl = SPEED_OF_LIGHT_M_S / (input.f1_hz - input.f2_hz);

        let lw = (input.f1_hz * input.l1 - input.f2_hz * input.l2) / (input.f1_hz - input.f2_hz);
        let cn = (input.f1_hz * input.c1 + input.f2_hz * input.c2) / (input.f1_hz + input.f2_hz);

        self.nw_avg.add((lw - cn) / lambda_wl);

        let nw = self.nw_avg.mean.round();

        let n1 = (input.l1 - input.l2 - lambda_2 * nw) / (lambda_1 - lambda_2);
        self.n1_avg.add(n1);

        let n1 = self.n1_avg.mean.round() as u32;
        let n2 = n1 - nw as u32;

        Some(Output { n1, n2 })
    }
}
