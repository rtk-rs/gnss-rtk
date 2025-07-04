use std::collections::HashMap;

use crate::prelude::{Carrier, SV};

#[derive(Default, Debug)]
pub(crate) struct DoubleDifference {
    /// Code Double difference
    pub code: Option<(Carrier, f64)>,

    /// Phase Double Difference
    pub phase: Option<(Carrier, f64, f64)>,
}

impl std::fmt::Display for DoubleDifference {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        if let Some((carrier, code)) = &self.code {
            write!(f, "code({})={} ", carrier, code)?;
        }

        if let Some((carrier, _, phase)) = &self.phase {
            write!(f, "phase({})={} ", carrier, phase)?;
        }

        Ok(())
    }
}

impl DoubleDifference {
    pub fn from_code(value: (Carrier, f64)) -> Self {
        Self {
            code: Some(value),
            phase: None,
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

#[derive(Default, Debug)]
pub(crate) struct DoubleDifferences {
    /// [DoubleDifference]s per [SV] and [Carrier].
    pub inner: HashMap<SV, DoubleDifference>,
}

impl DoubleDifferences {
    pub fn insert_code(&mut self, sv: SV, value: (Carrier, f64)) {
        if let Some(inner) = self.inner.get_mut(&sv) {
            inner.code = Some(value);
        } else {
            self.inner.insert(sv, DoubleDifference::from_code(value));
        }
    }

    pub fn insert_phase(&mut self, sv: SV, value: (Carrier, f64, f64)) {
        if let Some(inner) = self.inner.get_mut(&sv) {
            inner.phase = Some(value);
        } else {
            self.inner.insert(sv, DoubleDifference::from_phase(value));
        }
    }

    pub fn double_difference(&self, sv: SV) -> Option<&DoubleDifference> {
        self.inner.get(&sv)
    }

    /// Returns total phase NdF
    pub fn phase_ndf(&self) -> usize {
        let mut size = 0;
        for (_, value) in self.inner.iter() {
            if value.phase.is_none() {
                size += 1;
            }
        }
        size
    }

    pub fn ndf(&self) -> usize {
        self.inner.len()
    }
}
