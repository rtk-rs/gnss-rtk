use crate::prelude::{AbsoluteTime, Epoch, TimeScale};

pub struct NullTime {}

impl AbsoluteTime for NullTime {
    fn new_epoch(&mut self, _: Epoch) {}

    fn epoch_correction(&self, t: Epoch, target: TimeScale) -> Epoch {
        t.to_time_scale(target)
    }
}
