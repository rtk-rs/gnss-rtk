use crate::prelude::{Carrier, SV};
use std::collections::HashMap;

use log::debug;

#[derive(Clone, PartialEq, Eq, Hash)]
struct Key {
    /// [SV]
    sv: SV,
    /// [Carrier]
    carrier: Carrier,
}

struct MovingAverage {
    pub n: usize,
    pub size: usize,
    pub c_n: f64,
    pub l_n: f64,
}

impl MovingAverage {
    pub fn new(size: usize) -> Self {
        Self {
            size,
            n: 0,
            c_n: 0.0,
            l_n: 0.0,
        }
    }

    pub fn add(&mut self, c_n: f64, l_n: f64) -> f64 {
        self.n += 1;
        let alpha = if self.n < self.size {
            1.0 / self.n as f64
        } else {
            1.0 / self.size as f64
        };

        let l_k = l_n - self.l_n;
        let c_k = alpha * c_n + (1.0 - alpha) * (self.c_n + l_k);
        debug!(
            "alpha={} | c_n={} | c_k={} | err={}",
            alpha,
            c_n,
            c_k,
            (c_n - c_k).abs()
        );

        self.l_n = l_n;

        if self.n == 1 {
            self.c_n = c_n;
        } else {
            self.c_n = c_k;
        }

        c_k
    }
}

pub struct Smoother {
    win_len: usize,
    inner: HashMap<Key, MovingAverage>,
}

impl Smoother {
    pub fn new(win_len: usize) -> Self {
        Self {
            win_len,
            inner: HashMap::with_capacity(8),
        }
    }

    pub fn smoothing(
        &mut self,
        carrier: Carrier,
        sv: SV,
        c_n: f64,
        n_1: f64,
        lambda_1: f64,
        l_n: f64,
    ) -> f64 {
        let k = Key { sv, carrier };
        if let Some(mov) = self.inner.get_mut(&k) {
            let c_k = mov.add(c_n, l_n - n_1 * lambda_1);
            c_k
        } else {
            let mut mov = MovingAverage::new(self.win_len);
            let c_k = mov.add(c_n, l_n - n_1 * lambda_1);
            self.inner.insert(k.clone(), mov);
            c_k
        }
    }
}
