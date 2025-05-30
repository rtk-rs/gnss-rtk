use crate::{
    prelude::{
        Candidate, Epoch, 
        SV,
    },
};

/// Any [RTKBase] provides remote data by implementing the [RemoteSource] trait.
pub trait RTKBase {
    /// Provide a meaningful name of each reference station.
    /// This is only used when reporting status and results and is not vital.
    fn name(&self) -> String;

    /// Provide remote synchronous [Candidate] observations for this [SV] at this [Epoch].
    /// If you fail to provide and fulfill the navigation technique requirements,
    /// the [SV] will be dropped for this [Epoch]. In other words, it will not contribute to the process.
    /// The [Observation] should be synchronous, ideally you should use an interpolation scheme.
    /// For high sampling rates, taking the closest neighbouring point in time will do, but that only stands
    /// if your [RTKBase] remained static in the meantime !
    fn observe(&mut self, t: Epoch, sv: SV) -> Option<Candidate>;

    /// Any [RTKBase] should be able to desribe its absolute position, at all times,
    /// with highest accuracy. These coordinates (in meters, ECEF), should correspond to the precise
    /// position where all remote [Observation]s were made.
    /// Any moving [RTKBase] should keep the position up to date and it should reflect through this method.
    fn reference_position_ecef_m(&self, t: Epoch) -> Option<(f64, f64, f64)>;
}

pub(crate) struct NullRTK {}

impl RTKBase for NullRTK {
    fn name(&self) -> String {
        "UNUSED".to_string()
    }

    fn observe(&mut self, _: Epoch, _: SV) -> Option<Candidate> {
        None
    }

    fn reference_position_ecef_m(&self, _: Epoch) -> Option<(f64, f64, f64)> {
        None
    }
}
