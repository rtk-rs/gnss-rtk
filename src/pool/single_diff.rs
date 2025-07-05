use crate::prelude::{Carrier, SV};

use std::collections::HashMap;

#[derive(Debug, Clone, Copy, Default)]
pub struct SingleDifference {
    /// Code single difference
    pub code: Option<(Carrier, f64)>,

    /// Phase single difference
    pub phase: Option<(Carrier, f64, f64)>,
}

impl std::fmt::Display for SingleDifference {
    fn fmt(&self, fmt: &mut std::fmt::Formatter) -> std::fmt::Result {
        if let Some((carrier, code)) = self.code {
            write!(fmt, "code({})={} ", carrier, code)?;
        }
        if let Some((carrier, _, phase)) = self.phase {
            write!(fmt, "phase({})={} ", carrier, phase)?;
        }
        Ok(())
    }
}

impl SingleDifference {
    pub fn from_code(value: (Carrier, f64)) -> Self {
        Self {
            phase: None,
            code: Some(value),
        }
    }

    pub fn with_code(mut self, value: (Carrier, f64)) -> Self {
        self.code = Some(value);
        self
    }

    pub fn from_phase(value: (Carrier, f64, f64)) -> Self {
        Self {
            code: None,
            phase: Some(value),
        }
    }

    pub fn with_phase(mut self, value: (Carrier, f64, f64)) -> Self {
        self.phase = Some(value);
        self
    }
}

#[derive(Clone, Debug, Default)]
pub struct SingleDifferences {
    pub inner: HashMap<SV, SingleDifference>,
}

impl SingleDifferences {
    /// Insert SD(code)
    pub fn insert_code(&mut self, sv: SV, carrier: Carrier, value: f64) {
        if let Some(inner) = self.inner.get_mut(&sv) {
            inner.code = Some((carrier, value));
        } else {
            self.inner
                .insert(sv, SingleDifference::from_code((carrier, value)));
        }
    }

    /// Returns SD(code)
    pub fn code(&self, sv: SV) -> Option<(Carrier, f64)> {
        let data = self.inner.get(&sv)?;
        data.code
    }

    /// Insert SD(phase)
    pub fn insert_phase(&mut self, sv: SV, value: (Carrier, f64, f64)) {
        if let Some(inner) = self.inner.get_mut(&sv) {
            inner.phase = Some(value);
        } else {
            self.inner.insert(sv, SingleDifference::from_phase(value));
        }
    }

    /// Returns SD(Cn)
    pub fn phase(&self, sv: SV) -> Option<(Carrier, f64, f64)> {
        let data = self.inner.get(&sv)?;
        data.phase
    }
}
