use crate::{
    pool::Pool,
    prelude::{Almanac, Config, Epoch, Frame, TimeScale},
    tests::{
        data::CandidatesBuilder, ephemeris::NullEph, init_logger, time::NullTime, OrbitsData,
        TestEnvironment, TestSpacebornBiases,
    },
};

use std::rc::Rc;

use hifitime::Duration;
use rstest::*;
use std::str::FromStr;

#[fixture]
fn build_almanac() -> Almanac {
    use crate::tests::almanac;
    almanac()
}

#[fixture]
fn build_earth_frame() -> Frame {
    use crate::tests::earth_frame;
    earth_frame()
}

#[fixture]
fn build_orbit_source() -> OrbitsData {
    use crate::tests::test_orbits;
    test_orbits()
}

#[test]
fn ppp_pool_fit() {
    init_logger();

    let almanac = build_almanac();
    let default_cfg = Config::default();

    let null_eph = NullEph {};
    let null_time = NullTime {};

    let earth_frame = build_earth_frame();
    let orbits_data = Rc::new(build_orbit_source());
    let environment = TestEnvironment::new();
    let space_biases = TestSpacebornBiases::build();

    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

    let candidates = CandidatesBuilder::build_rover_at(t0_gpst);

    let mut pool = Pool::allocate(
        almanac,
        default_cfg,
        earth_frame,
        null_eph.into(),
        orbits_data,
        environment.into(),
        space_biases.into(),
    );

    pool.new_epoch(&candidates);

    assert_eq!(
        pool.candidates().len(),
        candidates.len(),
        "builder dropped some data!"
    );

    pool.pre_fit("rover", &null_time);
    pool.orbital_states_fit("rover");

    assert_eq!(
        pool.candidates().len(),
        candidates.len(),
        "pool fit dropped some data!"
    );
}

#[test]
fn ppp_gst_pool_fit() {
    init_logger();

    let almanac = build_almanac();
    let earth_frame = build_earth_frame();

    let mut default_cfg = Config::default();
    default_cfg.timescale = TimeScale::GST;

    let null_eph = NullEph {};
    let null_time = NullTime {};

    let orbits_data = Rc::new(build_orbit_source());
    let environment = TestEnvironment::new();
    let space_biases = TestSpacebornBiases::build();

    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

    let candidates = CandidatesBuilder::build_rover_at(t0_gpst);

    let mut pool = Pool::allocate(
        almanac,
        default_cfg,
        earth_frame,
        null_eph.into(),
        orbits_data,
        environment.into(),
        space_biases.into(),
    );

    pool.new_epoch(&candidates);
    pool.pre_fit("rover", &null_time);

    assert_eq!(
        pool.candidates().len(),
        candidates.len(),
        "pool prefit dropped some data!"
    );

    for cd in pool.candidates().iter() {
        assert_eq!(
            cd.epoch.time_scale,
            TimeScale::GST,
            "{}({}) - not correctly transposed to GST",
            cd.epoch,
            cd.sv
        );
    }

    pool.orbital_states_fit("rover");

    for cd in pool.candidates().iter() {
        assert_eq!(
            cd.epoch.time_scale,
            TimeScale::GST,
            "{}({}) - not correctly transposed to GST",
            cd.epoch,
            cd.sv
        );

        assert_eq!(
            cd.tx_epoch.time_scale,
            TimeScale::GST,
            "{}({}) - not correctly transposed to GST",
            cd.tx_epoch,
            cd.sv
        );

        assert!(
            cd.epoch > cd.tx_epoch,
            "{}({}) - physical non sense (after transposition to GST)",
            cd.epoch,
            cd.sv
        );

        let dt = cd.epoch - cd.tx_epoch;

        assert!(
            dt < Duration::from_milliseconds(100.0),
            "{}({}) - not compatible with MEO/LEO Earth nav (after transposition to GST)",
            cd.epoch,
            cd.sv
        );
    }
}
