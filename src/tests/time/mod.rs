mod gst;

use crate::time::Time;

pub struct NullTime {}

impl Time for NullTime {
    fn time_offset_update(
        &mut self,
        _: hifitime::Epoch,
        _: hifitime::TimeScale,
        _: hifitime::TimeScale,
    ) -> Option<crate::time::TimeOffset> {
        None
    }
}
