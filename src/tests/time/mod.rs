mod gst;

use crate::time::Time;

pub struct NullTime {}

impl Time for NullTime {
    fn bdt_gpst_time_offset(&mut self, _: hifitime::Epoch) -> Option<crate::prelude::TimeOffset> {
        None
    }

    fn bdt_gst_time_offset(&mut self, _: hifitime::Epoch) -> Option<crate::prelude::TimeOffset> {
        None
    }

    fn bdt_utc_time_offset(&mut self, _: hifitime::Epoch) -> Option<crate::prelude::TimeOffset> {
        None
    }

    fn gpst_utc_time_offset(&mut self, _: hifitime::Epoch) -> Option<crate::prelude::TimeOffset> {
        None
    }

    fn gst_gpst_time_offset(&mut self, _: hifitime::Epoch) -> Option<crate::prelude::TimeOffset> {
        None
    }

    fn gst_utc_time_offset(&mut self, _: hifitime::Epoch) -> Option<crate::prelude::TimeOffset> {
        None
    }
}
