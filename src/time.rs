use crate::prelude::{Epoch, TimeScale};

/// [AbsoluteTime] is implemented by applications where precise temporal
/// solutions need to be resolved at all times and in all scenarios.
/// Correct implementation of this trait allows the temporal solution to follow
/// the desired [TimeScale] behavior.
pub trait AbsoluteTime {
    /// Inform your solver that we're currently processing this new [Epoch].
    /// Because GNSS-RTK is synchronous, it is considered "now" and any past [Epoch]s will not
    /// be processed again.
    fn new_epoch(&mut self, now: Epoch);

    /// Perform the temporal translation of proposed [Epoch] in to targeted [TimeScale].
    /// This method needs to apply at all times. If you don't know how to do this, simply return a copy
    /// and your temporal solutions will be off by some offset.
    fn epoch_correction(&self, t: Epoch, target: TimeScale) -> Epoch;
}
