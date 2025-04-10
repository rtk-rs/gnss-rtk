#[cfg(test)]
mod gst_gpst_basic_test {
    use crate::{
        prelude::TimeScale,
        time::{AbsoluteTime, Time, TimeOffset},
    };

    use std::str::FromStr;

    struct TimeUpdater {}

    impl Time for TimeUpdater {
        fn time_offset_update(
            &mut self,
            t: Epoch,
            lhs: TimeScale,
            rhs: TimeScale,
        ) -> Option<TimeOffset> {
            None
        }
    }

    #[test]
    fn test_gst_gpst_basic() {
        let updater = TimeUpdater {};
        let absolute_time = AbsoluteTime::new(updater);

        let t = Epoch::from_str("2020-01-01T00:00:00 GST").unwrap();

        let time_correction = absolute_time
            .correction(t, TimeScale::GST, TimeScale::GPST)
            .unwrap();
    }
}
