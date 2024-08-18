use crate::prelude::{Epoch, Frame, Orbit, SV};

/// OrbitalStateProvider must be implemented
/// and provide SV state at specified `t` for the solving process can proceed.
pub trait OrbitSource {
    /// Provide Antenna Phase Center state as [Orbit] at requested [Epoch] for requested [SV]
    /// and expressed in required [Frame]. If you happen to use other [Frame]s,
    /// you can apply [Frame] conversion (rotations) by means of an [Almanac].
    /// In case interpolation is used, we propose an interpolation order,
    /// that would fit current setup, which you can choose to ignore.
    /// If None is returned for too long, this [Epoch] will eventually get dropped out
    /// and we will proceed to the next.
    fn next_at(&mut self, t: Epoch, sv: SV, fr: Frame, order: usize) -> Option<Orbit>;
}
