use crate::prelude::{Candidate, Carrier, Epoch, SV};
use std::collections::HashMap;

/// Any [RTKBase] provides remote data by implementing the [RemoteSource] trait.
pub trait RTKBase {
    /// Provide a meaningful name of each reference station.
    /// This is only used when reporting status and results and is not vital.
    fn name(&self) -> String;

    /// Notifies the remote station that we're about to process a new [Epoch].
    /// Since GNSS-RTK expects measurements in chronological order, and simply
    /// follows the measurements, you should consider that any past [Epoch]
    /// will no longer be processed.
    fn new_epoch(&mut self, epoch: Epoch);

    /// Provide as many remote synchronous measurements (wrapped as [Candidate]s), at specified sampling [Epoch], as you can.
    /// In practice, it is most often impossible to be truly be synchronous, you should
    /// always minimize the time-difference, especially when the base is moving.
    fn observe(&self, epoch: Epoch) -> Vec<Candidate>;

    /// Provide the reference position of the base station, at sampling [Epoch].
    /// Any moving base station should keep its position up to date and garatee the age of the
    /// observations are kept to a minimal, the time-difference between directly related to the
    /// base velocity.
    /// Failure to provide this reference state will prohibit this RTK cycle completely.
    fn reference_position_ecef_m(&self, epoch: Epoch) -> Option<(f64, f64, f64)>;
}

pub(crate) struct NullRTK {}

impl RTKBase for NullRTK {
    fn name(&self) -> String {
        "NULL".to_string()
    }

    fn new_epoch(&mut self, _: Epoch) {}

    fn observe(&self, _: Epoch) -> Vec<Candidate> {
        Default::default()
    }

    fn reference_position_ecef_m(&self, _: Epoch) -> Option<(f64, f64, f64)> {
        Default::default()
    }
}

#[derive(Default, Debug)]
pub(crate) struct DoubleDifference {
    /// Double difference in meters
    pub code: Option<f64>,

    /// Double difference in meters
    pub phase: Option<f64>,
}

impl std::fmt::Display for DoubleDifference {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "code={:?} phase={:?}", self.code, self.phase)?;
        Ok(())
    }
}

impl DoubleDifference {
    pub fn from_code(value: f64) -> Self {
        Self {
            code: Some(value),
            phase: None,
        }
    }

    pub fn from_phase(value: f64) -> Self {
        Self {
            code: None,
            phase: Some(value),
        }
    }
}

#[derive(Default, Debug)]
pub(crate) struct DoubleDifferences {
    /// [DoubleDifference]s per [SV] and [Carrier].
    pub inner: HashMap<(SV, Carrier), DoubleDifference>,
}

impl DoubleDifferences {
    pub fn insert_code(&mut self, sv: SV, carrier: Carrier, value: f64) {
        if let Some(inner) = self.inner.get_mut(&(sv, carrier)) {
            inner.code = Some(value);
        } else {
            self.inner
                .insert((sv, carrier), DoubleDifference::from_code(value));
        }
    }

    pub fn insert_phase(&mut self, sv: SV, carrier: Carrier, value: f64) {
        if let Some(inner) = self.inner.get_mut(&(sv, carrier)) {
            inner.phase = Some(value);
        } else {
            self.inner
                .insert((sv, carrier), DoubleDifference::from_phase(value));
        }
    }

    pub fn double_difference(&self, sv: SV, carrier: Carrier) -> Option<&DoubleDifference> {
        self.inner.get(&(sv, carrier))
    }
}
