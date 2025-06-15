use crate::prelude::{Epoch, Frame, Orbit, SV};

/// Any [Orbit] provider should implement the [OrbitSource] trait to provide
/// information that will contribute to the solving process.
pub trait OrbitSource {
    /// Provide [Orbit]al state to help the solving process.
    ///
    /// The state should represent the Antenna Phase Center coordinates of requested
    /// [SV] at requested [Epoch], and should be expressed in the proposed [Frame].
    ///
    /// GNSS-RTK is fully synchronous and _physically_ expects measurements
    /// in chronological order. The requests issued here follow the provided measurements,
    /// therefore arrive in chronological order.
    fn state_at(&self, epoch: Epoch, sv: SV, fr: Frame) -> Option<Orbit>;
}
