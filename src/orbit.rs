use crate::prelude::{Epoch, Frame, Orbit, SV};

/// Any [Orbit] provider should implement the [OrbitSource] trait to provide
/// information that will contribute to the solving process.
pub trait OrbitSource {
    /// Provide [Orbit]al state that will contribute to the solving process.
    ///
    /// The state should represent the [SV] Antenna Phase Center coordinates,
    /// at requested [Epoch], expressed in requested [Frame].
    /// Any error here directly reflects on the accuracy of the solution.
    ///
    /// `GNSS-RTK` is fully synchronous and expects measurements
    /// in chronological order. Because it follows your measurements,
    /// requests will then also be in chronological order, and this is
    /// always the "latest" [Epoch] processed for this [SV].
    ///
    /// :warning: it is mandatory to provide [Orbit]al states
    /// and not possible to only navigate using ephemeris frames using the current version.
    fn state_at(&self, epoch: Epoch, sv: SV, fr: Frame) -> Option<Orbit>;
}
