use std::collections::HashMap;

use crate::prelude::{Carrier, SV};

#[derive(Default, Debug)]
pub(crate) struct Difference {
    /// Code difference
    pub code: Option<(Carrier, f64)>,

    /// MW difference
    pub mw: Option<(Carrier, f64, f64)>,

    /// Phase (j) difference
    pub phase_j: Option<(Carrier, f64, f64)>,

    /// Phase IF difference
    pub phase_if: Option<(Carrier, f64, f64)>,
}

impl std::fmt::Display for Difference {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        if let Some((carrier, code)) = &self.code {
            write!(f, "code({})={} ", carrier, code)?;
        }

        if let Some((carrier, _, phase_j)) = &self.phase_j {
            write!(f, "phase_j({})={} ", carrier, phase_j)?;
        }

        if let Some((carrier, _, mw)) = &self.mw {
            write!(f, "mw({})={} ", carrier, mw)?;
        }

        if let Some((carrier, _, phase_if)) = &self.phase_if {
            write!(f, "phase_if({})={} ", carrier, phase_if)?;
        }

        Ok(())
    }
}

impl Difference {
    pub fn from_code(value: (Carrier, f64)) -> Self {
        Self {
            mw: None,
            phase_j: None,
            phase_if: None,
            code: Some(value),
        }
    }

    pub fn with_code(mut self, value: (Carrier, f64)) -> Self {
        self.code = Some(value);
        self
    }

    pub fn from_phase_if(value: (Carrier, f64, f64)) -> Self {
        Self {
            mw: None,
            code: None,
            phase_j: None,
            phase_if: Some(value),
        }
    }

    pub fn with_phase_if(mut self, value: (Carrier, f64, f64)) -> Self {
        self.phase_if = Some(value);
        self
    }

    pub fn from_phase_j(value: (Carrier, f64, f64)) -> Self {
        assert!(!value.0.is_l1(), "internal issue: this is L1 signal");

        Self {
            mw: None,
            code: None,
            phase_if: None,
            phase_j: Some(value),
        }
    }

    pub fn with_phase_j(mut self, value: (Carrier, f64, f64)) -> Self {
        assert!(!value.0.is_l1(), "internal issue: this is L1 signal");

        self.phase_j = Some(value);
        self
    }

    pub fn from_mw(value: (Carrier, f64, f64)) -> Self {
        Self {
            code: None,
            phase_j: None,
            phase_if: None,
            mw: Some(value),
        }
    }

    pub fn with_mw(mut self, value: (Carrier, f64, f64)) -> Self {
        self.mw = Some(value);
        self
    }
}

/// [Differences] is a set to manage Single or Double [Difference]s.
#[derive(Default, Debug)]
pub(crate) struct Differences {
    /// [Difference] per [SV].
    pub inner: HashMap<SV, Difference>,
}

impl Differences {
    pub fn insert(&mut self, sv: SV, value: Difference) {
        self.inner.insert(sv, value);
    }

    pub fn insert_code(&mut self, sv: SV, value: (Carrier, f64)) {
        if let Some(inner) = self.inner.get_mut(&sv) {
            inner.code = Some(value);
        } else {
            self.inner.insert(sv, Difference::from_code(value));
        }
    }

    pub fn insert_phase_j(&mut self, sv: SV, value: (Carrier, f64, f64)) {
        if let Some(inner) = self.inner.get_mut(&sv) {
            inner.phase_j = Some(value);
        } else {
            self.inner.insert(sv, Difference::from_phase_j(value));
        }
    }

    pub fn insert_phase_if(&mut self, sv: SV, value: (Carrier, f64, f64)) {
        if let Some(inner) = self.inner.get_mut(&sv) {
            inner.phase_if = Some(value);
        } else {
            self.inner.insert(sv, Difference::from_phase_if(value));
        }
    }

    pub fn insert_mw(&mut self, sv: SV, value: (Carrier, f64, f64)) {
        if let Some(inner) = self.inner.get_mut(&sv) {
            inner.mw = Some(value);
        } else {
            self.inner.insert(sv, Difference::from_mw(value));
        }
    }

    /// Obtain [Difference] for this [SV] if it exists.
    pub fn difference(&self, sv: SV) -> Option<&Difference> {
        self.inner.get(&sv)
    }

    pub fn ndf(&self) -> usize {
        self.inner.len()
    }
}
