#[cfg(test)]
mod gst_gpst_basic_test {

    use crate::{
        prelude::{Epoch, TimeScale},
        time::{AbsoluteTime, Time, TimeOffset},
    };

    use std::str::FromStr;

    const WEEK_N: u32 = 2111;

    struct TimeUpdater {}

    impl Time for TimeUpdater {
        fn time_offset_update(
            &mut self,
            _: Epoch,
            lhs: TimeScale,
            rhs: TimeScale,
        ) -> Option<TimeOffset> {
            match (lhs, rhs) {
                (TimeScale::GST, TimeScale::GPST) => {
                    let t_ref = (2111, 345_600);
                    let polynomials = (2.3574102670E-09, 3.996802889E-15, 0.0);
                    Some(TimeOffset::from_time_of_week(t_ref, polynomials))
                },
                (TimeScale::GST, TimeScale::UTC) => {
                    let t_ref = (2111, 345_600);
                    let polynomials = (-9.3132257462E-10, 0.000000000E+00, 0.0);
                    Some(TimeOffset::from_time_of_week(t_ref, polynomials))
                },
                (TimeScale::GPST, TimeScale::UTC) => {
                    let t_ref = (2111, 589_824);
                    let polynomials = (9.3132257462E-10, 2.664535259E-15, 0.0);
                    Some(TimeOffset::from_time_of_week(t_ref, polynomials))
                },
                _ => None,
            }
        }
    }

    #[test]
    fn test_gst_gpst_basic() {
        let updater = TimeUpdater {};
        let mut absolute_time = AbsoluteTime::new(updater);

        let t = Epoch::from_time_of_week(WEEK_N, 345_600, TimeScale::GST);
        absolute_time.update(t);

        let t = Epoch::from_time_of_week(WEEK_N, 345_600 + 10, TimeScale::GST);

        let time_correction = absolute_time
            .time_correction_nanos(t, TimeScale::GST, TimeScale::GPST)
            .unwrap();

        assert!((time_correction - 2.357).abs() < 1.0E-3);

        let time_correction = absolute_time
            .time_correction_nanos(t, TimeScale::GPST, TimeScale::GST)
            .unwrap();

        assert!(
            (time_correction - -2.357).abs() < 1.0E-3,
            "{}",
            time_correction
        );
    }

    #[test]
    fn test_gst_utc_basic() {
        let updater = TimeUpdater {};
        let mut absolute_time = AbsoluteTime::new(updater);

        let t = Epoch::from_time_of_week(WEEK_N, 345_600, TimeScale::GST);
        absolute_time.update(t);

        let t = Epoch::from_time_of_week(WEEK_N, 345_600 + 10, TimeScale::GST);

        let time_correction = absolute_time
            .time_correction_nanos(t, TimeScale::GST, TimeScale::UTC)
            .unwrap();

        assert!((time_correction - -0.9313).abs() < 1.0E-3);

        let time_correction = absolute_time
            .time_correction_nanos(t, TimeScale::UTC, TimeScale::GST)
            .unwrap();

        assert!((time_correction - 0.9313).abs() < 1.0E-3);

        let t = Epoch::from_time_of_week(WEEK_N, 345_600 + 100_000, TimeScale::GST);

        let time_correction = absolute_time
            .time_correction_nanos(t, TimeScale::GST, TimeScale::UTC)
            .unwrap();

        assert!((time_correction - -0.9313).abs() < 1.0E-3);

        let time_correction = absolute_time
            .time_correction_nanos(t, TimeScale::UTC, TimeScale::GST)
            .unwrap();

        assert!((time_correction - 0.9313).abs() < 1.0E-3);
    }
}
