use crate::prelude::{Epoch, TimeScale};

/// [AbsoluteTime] is implemented by applications where precise temporal
/// solutions need to be resolved at all times and in all scenarios.
/// Correct implementation of this trait allows the temporal solution to follow
/// the desired [TimeScale] behavior.
pub trait AbsoluteTime {
    /// Take into account that we're transitioning to a new [Epoch] (measurement Epoch).
    /// Because GNSS-RTK is synchronous, it is considered "now" and any past [Epoch]s will not
    /// be ever processed again.
    fn new_epoch(&mut self, now: Epoch);

    /// Convert provided [Epoch] into specified [TimeScale].
    /// The default should be [Epoch::to_time_scale], which
    /// applies at all times. Failure to implement this transition will offset your temporal
    /// solution.
    fn epoch_correction(&self, epoch: Epoch, timescale: TimeScale) -> Epoch;
}
