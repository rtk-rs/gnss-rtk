use crate::prelude::{Epoch, Frame, Orbit, SV};

/// Any [Orbit] provider should implement the [OrbitSource] trait to provide
/// information that will contribute to the solving process.
pub trait OrbitSource {
    /// Provide [Orbit]al state to help the solving process.
    /// The state should represent the Antenna Phase Center coordinates of requested
    /// [SV] at requested [Epoch], and should be expressed in the proposed [Frame].
    ///
    /// [Orbit]al states are always prefered over states that we might resolve
    /// from possible
    ///
    /// If you have provided [Ephemeris] in parallel and some are still valid,
    /// it is okay to return None here. But it is mandatory
    ///
    /// Because GNSS-RTK is synchronous and you should provide measurements in chronological order,
    /// and this is the latest [Epoch] processed for this [SV].
    fn state_at(&self, epoch: Epoch, sv: SV, fr: Frame) -> Option<Orbit>;
}
