use crate::prelude::{Epoch, Frame, Orbit, SV};

/// Applications need to implement the [OrbitSource] to propose [SV] that
/// will contribute to the solutions.
pub trait OrbitSource {
    /// Provide Antenna Phase Center state at requested [Epoch] and [SV],
    /// which you need to express as ANISE [Orbit] in the selected [Frame].  
    /// If you don't have such information, simply return [None] and this [SV]
    /// will not contribute to the next solution.  
    ///
    /// Because GNSS-RTK is synchronous and we're currently processing this [Epoch],
    /// any past [Epoch] will never be proposed here.
    fn next_at(&mut self, epoch: Epoch, sv: SV, fr: Frame) -> Option<Orbit>;
}
