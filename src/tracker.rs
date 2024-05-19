use crate::prelude::SV;
use std::collections::HashMap;

/// Signal tracker
#[derive(Debug, Clone)]
pub struct Tracker {
    inner: HashMap<SV, f64>,
}

impl Tracker {
    pub fn new() -> Self {
        Self {
            inner: HashMap::with_capacity(16),
        }
    }
    pub fn update(&mut self, sv: SV, value: f64) {
        self.inner.insert(sv, value);
    }
    pub fn ambiguity(&self, sv: &SV) -> Option<&f64> {
        self.inner.get(sv)
    }
}

/// Phase Ambiguity fixing methods.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum FixingMethod {
    /// [WideLane]
    #[default]
    Kalman,
}
