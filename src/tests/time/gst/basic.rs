use crate::{
    prelude::{Epoch, TimeScale},
    time::{AbsoluteTime, Time, TimeOffset},
};

use hifitime::Unit;

const WEEK_N: u32 = 2111;

struct TimeUpdater {}

impl Time for TimeUpdater {
    fn bdt_gpst_time_offset(&mut self, _: Epoch) -> Option<TimeOffset> {
        None
    }

    fn bdt_gst_time_offset(&mut self, _: Epoch) -> Option<TimeOffset> {
        None
    }

    fn bdt_utc_time_offset(&mut self, _: Epoch) -> Option<TimeOffset> {
        None
    }

    fn gpst_utc_time_offset(&mut self, _: Epoch) -> Option<TimeOffset> {
        None
    }

    fn gst_gpst_time_offset(&mut self, _: Epoch) -> Option<TimeOffset> {
        let t_ref = (2111, 345_600);
        let polynomials = (2.3574102670E-09, 3.996802889E-15, 0.0);
        Some(TimeOffset::from_gst_gpst_time_of_week(t_ref, polynomials))
    }

    fn gst_utc_time_offset(&mut self, _: Epoch) -> Option<TimeOffset> {
        let t_ref = (2111, 345_600);
        let polynomials = (-9.3132257462E-10, 0.000000000E+00, 0.0);
        Some(TimeOffset::from_gst_utc_time_of_week(t_ref, polynomials))
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

    let expected_s = 2.3574102670E-09 + 3.996802889E-15 * 100.0;
    let expected_nanos = expected_s * 1.0E9_f64;
    let expected_nanos_rounded = expected_nanos.round() as u64;
    assert!((time_correction - expected_nanos).abs() < 1.0E-3);

    let corrected = absolute_time
        .epoch_time_correction(t, TimeScale::GPST)
        .unwrap();

    assert_eq!(
        corrected.time_scale,
        TimeScale::GPST,
        "timescale was not corrected!"
    );
    assert_eq!(
        corrected,
        t + expected_nanos_rounded as f64 * Unit::Nanosecond
    );

    let time_correction = absolute_time
        .time_correction_nanos(t, TimeScale::GPST, TimeScale::GST)
        .unwrap();

    assert!(
        (time_correction - -expected_nanos).abs() < 1.0E-3,
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

    let expected_s = -9.3132257462E-10;
    let expected_nanos = expected_s * 1.0E9_f64;
    let expected_nanos_rounded = expected_nanos.round() as u64;
    assert!((time_correction - expected_nanos).abs() < 1.0E-3);

    let corrected = absolute_time
        .epoch_time_correction(t, TimeScale::UTC)
        .unwrap();

    assert_eq!(
        corrected.time_scale,
        TimeScale::UTC,
        "timescale was not corrected!"
    );
    assert_eq!(
        corrected,
        t + expected_nanos_rounded as f64 * Unit::Nanosecond
    );

    let time_correction = absolute_time
        .time_correction_nanos(t, TimeScale::UTC, TimeScale::GST)
        .unwrap();

    assert!((time_correction - -expected_nanos).abs() < 1.0E-3);

    let t = Epoch::from_time_of_week(WEEK_N, 345_600 + 100_000, TimeScale::GST);

    let time_correction = absolute_time
        .time_correction_nanos(t, TimeScale::GST, TimeScale::UTC)
        .unwrap();

    let expected_s = -9.3132257462E-10;
    let expected_nanos = expected_s * 1.0E9;
    assert!((time_correction - expected_nanos).abs() < 1.0E-3);

    let time_correction = absolute_time
        .time_correction_nanos(t, TimeScale::UTC, TimeScale::GST)
        .unwrap();

    assert!((time_correction - -expected_nanos).abs() < 1.0E-3);
}
