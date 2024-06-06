use std::collections::HashMap;
use crate::prelude::{SV, Epoch};

#[derive(Debug, Clone)]
struct InnerData {
    t: Epoch,
    gf: f64,
    mw: f64,
}

#[derive(Debug, Clone)]
struct SVTracker {
    buffer: Vec<InnerData>,
}

impl SVTracker {
    fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(64),
        }
    }
    fn mean(&self) -> InnerData {
        let (mut gf, mut mw) = (0.0_f64, 0.0_f64);
        for i in 0..self.buffer.len() {
            gf += self.buffer[i].gf;
            mw += self.buffer[i].mw;
        }
        InnerData {
            t: self.buffer[self.buffer.len()-1].t,
            gf: gf / self.buffer.len() as f64,
            mw: mw / self.buffer.len() as f64,
        }
    }
    fn stddev(&self) -> InnerData {
        let mean = self.mean();
        let (mut gf, mut mw) = (0.0_f64, 0.0_f64);
        for i in 0..self.buffer.len() {
            gf += (self.buffer[i].gf - mean.gf).powi(2);
            mw += (self.buffer[i].mw - mean.mw).powi(2);
        }
        InnerData {
            t: self.buffer[self.buffer.len()-1].t,
            gf: gf / self.buffer.len() as f64,
            mw: mw / self.buffer.len() as f64,
        }
    }
    fn update(&mut self, v: InnerData, win_len: usize) {
        self.buffer.push(v);
        if self.buffer.len() > win_len {
            self.buffer.remove(0);
        }
    }
}

/// Signal tracker
#[derive(Debug, Clone)]
pub struct Tracker {
    win_len: usize,
    sv_tracker: HashMap<SV, SVTracker>,
}

impl Tracker {
    pub fn new(win_len: usize) -> Self {
        Self {
            win_len,
            sv_tracker: HashMap::with_capacity(32),
        }
    }
    pub fn update(&mut self, sv: SV, t: Epoch, gf: f64, mw: f64) {
        if let Some(tracker) = self.sv_tracker.get_mut(&sv) {
            let new = InnerData { t, gf, mw };
            tracker.update(new, self.win_len);
        } else {
            let mut tracker = SVTracker::new();
            tracker.update(InnerData { t, gf, mw }, self.win_len);
            self.sv_tracker.insert(sv, tracker);
        }
    }
}
