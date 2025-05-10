use crate::prelude::{Epoch, Frame, Orbit, SV};

/// Any [Orbit] provider should be declared as [OrbitSource] to propose [SV] that
/// will contribute to the solutions.
pub trait OrbitSource {
    /// Provide Antenna Phase Center coordinates at requested [Epoch] for proposed [SV].
    /// The [Orbit] should be constructed using the [Frame] that the solver is internally using.
    ///
    /// If you don't have such information, simply return [None] and this [SV] will not
    /// contribute to the next solution.
    ///
    /// Because GNSS-RTK is synchronous and you should provide measurements in chronological order,
    /// and this is the latest [Epoch] processed for this [SV].
    fn next_at(&self, epoch: Epoch, sv: SV, fr: Frame) -> Option<Orbit>;
}
