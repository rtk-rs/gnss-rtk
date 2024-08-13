use crate::prelude::{Epoch, Orbit, SV};

/// OrbitalStateProvider must be implemented
/// and provide SV state at specified `t` for the solving process can proceed.
pub trait OrbitalStateProvider {
    /// Provide Antenna Phase Center state as [Orbit] at requested [Epoch] for requested [SV].
    /// In case interpolation is used, we propose an interpolation order,
    /// that would fit current setup, which you can choose to ignore.
    /// If None is returned for too long, this [Epoch] will eventually get dropped out
    /// and we will proceed to the next.
    fn next_at(&mut self, t: Epoch, sv: SV, order: usize) -> Option<Orbit>;
}
