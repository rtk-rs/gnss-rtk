use crate::{
    navigation::{apriori::Apriori, state::State},
    pool::Pool,
    prelude::{Almanac, Config, Epoch, Frame, Method, TimeScale},
    tests::{
        data::CandidatesBuilder, ephemeris::NullEph, init_logger, time::NullTime, OrbitsData,
        TestEnvironment, TestSpacebornBiases,
    },
};

use rstest::*;

use log::debug;
use nalgebra::U4;
use std::rc::Rc;

use hifitime::Duration;

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

#[fixture]
fn build_rover_apriori() -> Apriori {
    use crate::tests::rover_reference_apriori_at_ref_epoch;
    rover_reference_apriori_at_ref_epoch()
}

#[fixture]
fn build_base_apriori() -> Apriori {
    use crate::tests::base_reference_apriori_at_ref_epoch;
    base_reference_apriori_at_ref_epoch()
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

#[test]
fn rtk_spp_pool_fit() {
    init_logger();

    let almanac = build_almanac();

    let default_cfg = Config::default().with_navigation_method(Method::SPP);

    let null_time = NullTime {};
    let null_eph = Rc::new(NullEph {});

    let earth_frame = build_earth_frame();
    let orbits_data = Rc::new(build_orbit_source());
    let environment = Rc::new(TestEnvironment::new());
    let space_biases = Rc::new(TestSpacebornBiases::build());

    const NUM_SV: usize = 8;
    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

    let mut rover = Pool::allocate(
        almanac.clone(),
        default_cfg.clone(),
        earth_frame.clone(),
        null_eph.clone(),
        orbits_data.clone(),
        environment.clone(),
        space_biases.clone(),
    );

    let candidates = CandidatesBuilder::build_rover_at(t0_gpst);

    rover.new_epoch(&candidates);

    let mut base = Pool::allocate(
        almanac,
        default_cfg,
        earth_frame,
        null_eph,
        orbits_data,
        environment,
        space_biases,
    );

    let candidates = CandidatesBuilder::build_base_at(t0_gpst);

    base.new_epoch(&candidates);

    let apriori = build_rover_apriori();

    let rover_state = State::from_apriori(&apriori).unwrap_or_else(|e| {
        panic!("Failed to build rover initial state: {}", e);
    });

    let apriori = build_base_apriori();

    let base_state = State::from_apriori(&apriori).unwrap_or_else(|e| {
        panic!("Failed to build base initial state: {}", e);
    });

    base.pre_fit("base", &null_time);
    base.orbital_states_fit("base");

    base.post_fit("base", &base_state).unwrap_or_else(|e| {
        panic!("base station post-fit failed with {}", e);
    });

    rover.pre_fit("rover", &null_time);
    rover.orbital_states_fit("rover");

    rover.post_fit("rover", &rover_state).unwrap_or_else(|e| {
        panic!("rover post-fit failed with {}", e);
    });

    let dbl_diff = rover.rtk_post_fit(&mut base).unwrap_or_else(|e| {
        panic!("rtk post-fit failed with {}", e);
    });

    let expected_sat = NUM_SV - 2; // pivot+E13 below elevation mask

    assert!(
        dbl_diff.inner.len() == expected_sat,
        "did not form correct DD observations"
    );
}

#[test]
fn rtk_cpp_pool_fit() {
    init_logger();

    let almanac = build_almanac();

    let default_cfg = Config::default().with_navigation_method(Method::CPP);

    let null_time = NullTime {};
    let null_eph = Rc::new(NullEph {});

    let earth_frame = build_earth_frame();
    let orbits_data = Rc::new(build_orbit_source());
    let environment = Rc::new(TestEnvironment::new());
    let space_biases = Rc::new(TestSpacebornBiases::build());

    const NUM_SV: usize = 8;
    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

    let mut rover = Pool::allocate(
        almanac.clone(),
        default_cfg.clone(),
        earth_frame.clone(),
        null_eph.clone(),
        orbits_data.clone(),
        environment.clone(),
        space_biases.clone(),
    );

    let candidates = CandidatesBuilder::build_rover_at(t0_gpst);

    rover.new_epoch(&candidates);

    let mut base = Pool::allocate(
        almanac,
        default_cfg,
        earth_frame,
        null_eph,
        orbits_data,
        environment,
        space_biases,
    );

    let candidates = CandidatesBuilder::build_base_at(t0_gpst);

    base.new_epoch(&candidates);

    let apriori = build_rover_apriori();

    let rover_state = State::from_apriori(&apriori).unwrap_or_else(|e| {
        panic!("Failed to build rover initial state: {}", e);
    });

    let apriori = build_base_apriori();

    let base_state = State::from_apriori(&apriori).unwrap_or_else(|e| {
        panic!("Failed to build base initial state: {}", e);
    });

    base.pre_fit("base", &null_time);
    base.orbital_states_fit("base");

    base.post_fit("base", &base_state).unwrap_or_else(|e| {
        panic!("base station post-fit failed with {}", e);
    });

    rover.pre_fit("rover", &null_time);
    rover.orbital_states_fit("rover");

    rover.post_fit("rover", &rover_state).unwrap_or_else(|e| {
        panic!("rover post-fit failed with {}", e);
    });

    let dbl_diff = rover.rtk_post_fit(&mut base).unwrap_or_else(|e| {
        panic!("rtk post-fit failed with {}", e);
    });

    let expected_sat = NUM_SV - 2; // pivot+E13 below elevation mask

    assert!(
        dbl_diff.inner.len() == expected_sat,
        "did not form correct DD observations"
    );
}

#[test]
fn rtk_ppp_pool_fit() {
    init_logger();

    let almanac = build_almanac();

    let default_cfg = Config::default().with_navigation_method(Method::PPP);

    let null_time = NullTime {};
    let null_eph = Rc::new(NullEph {});

    let earth_frame = build_earth_frame();
    let orbits_data = Rc::new(build_orbit_source());
    let environment = Rc::new(TestEnvironment::new());
    let space_biases = Rc::new(TestSpacebornBiases::build());

    const NUM_SV: usize = 8;
    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

    let mut rover = Pool::allocate(
        almanac.clone(),
        default_cfg.clone(),
        earth_frame.clone(),
        null_eph.clone(),
        orbits_data.clone(),
        environment.clone(),
        space_biases.clone(),
    );

    let candidates = CandidatesBuilder::build_rover_at(t0_gpst);

    rover.new_epoch(&candidates);

    let mut base = Pool::allocate(
        almanac,
        default_cfg,
        earth_frame,
        null_eph,
        orbits_data,
        environment,
        space_biases,
    );

    let candidates = CandidatesBuilder::build_base_at(t0_gpst);

    base.new_epoch(&candidates);

    let apriori = build_rover_apriori();

    let rover_state = State::from_apriori(&apriori).unwrap_or_else(|e| {
        panic!("Failed to build rover initial state: {}", e);
    });

    let apriori = build_base_apriori();

    let base_state = State::from_apriori(&apriori).unwrap_or_else(|e| {
        panic!("Failed to build base initial state: {}", e);
    });

    base.pre_fit("base", &null_time);
    base.orbital_states_fit("base");

    base.post_fit("base", &base_state).unwrap_or_else(|e| {
        panic!("base station post-fit failed with {}", e);
    });

    rover.pre_fit("rover", &null_time);
    rover.orbital_states_fit("rover");

    rover.post_fit("rover", &rover_state).unwrap_or_else(|e| {
        panic!("rover post-fit failed with {}", e);
    });

    let dbl_diff = rover.rtk_post_fit(&mut base).unwrap_or_else(|e| {
        panic!("rtk post-fit failed with {}", e);
    });

    let expected_sat = NUM_SV - 2; // pivot+E13 below elevation mask

    assert!(
        dbl_diff.inner.len() == expected_sat,
        "did not form correct DD observations"
    );
}
