use crate::{
    prelude::{Epoch, TimeScale},
    time::{AbsoluteTime, Time, TimeOffset},
};

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
fn test_gst_gpst_advanced() {
    // GAUT -9.3132257462E-10 0.000000000E+00 345600 2111
    // GPUT  9.3132257462E-10 2.664535259E-15 589824 2111
    // GAGP  2.3574102670E-09 3.996802889E-15 345600 2111

    let updater = TimeUpdater {};
    let mut absolute_time = AbsoluteTime::new(updater);

    let t = Epoch::from_time_of_week(WEEK_N, 589824, TimeScale::GST);
    absolute_time.update(t);

    let t = Epoch::from_time_of_week(WEEK_N, 345_600 + 10, TimeScale::GST);

    let time_correction = absolute_time
        .time_correction_nanos(t, TimeScale::GST, TimeScale::GPST)
        .unwrap();

    let expected_s = 2.3574102670E-09 + 3.996802889E-15 * 100.0;
    let expected_nanos = expected_s * 1.0E9;

    assert!(
        (time_correction - expected_nanos).abs() < 1.0E-3,
        "{} versus {}",
        time_correction,
        expected_nanos,
    );

    let time_correction = absolute_time
        .time_correction_nanos(t, TimeScale::GPST, TimeScale::GST)
        .unwrap();

    assert!(
        (time_correction - -expected_nanos).abs() < 1.0E-3,
        "{} versus {}",
        time_correction,
        expected_nanos,
    );
}
