use crate::{
    constants::SPEED_OF_LIGHT_M_S,
    navigation::{apriori::Apriori, state::State},
    pool::Pool,
    prelude::{Almanac, Carrier, Config, Epoch, Frame, Method, TimeScale},
    tests::{
        data::CandidatesBuilder, ephemeris::NullEph, init_logger, time::NullTime, OrbitsData,
        TestEnvironment, TestSpacebornBiases, E01, E03, E05,
    },
};

use rstest::*;

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
        "Did not form correct DDs",
    );

    let mut e01_passed = false;
    let mut e03_passed = false;

    for (sat, dd) in dbl_diff.inner.iter() {
        assert!(*sat != E05, "did not remove pivot sat!");

        if *sat == E01 {
            let (carrier, code) = dd.code.expect("E01 missing DD(code)");

            assert_eq!(carrier, Carrier::L1);

            assert_eq!(
                code,
                27616185.992 - 23730317.923 - (27506424.743 - 23595077.027)
            );

            e01_passed = true;
        } else if *sat == E03 {
            let (carrier, code) = dd.code.expect("E03 missing DD(code)");

            assert_eq!(carrier, Carrier::L1);

            assert_eq!(
                code,
                27055946.391 - 23730317.923 - (26952639.751 - 23595077.027)
            );

            e03_passed = true;
        }
    }

    assert!(e01_passed, "E01 test failed");
    assert!(e03_passed, "E03 test failed");
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

    let mut e01_passed = false;
    let mut e03_passed = false;

    let (f1, f2) = (Carrier::L1.frequency_hz(), Carrier::E5b.frequency_hz());
    let (f1pow, f2pow) = (f1.powi(2), f2.powi(2));

    let pc_e01_rover = (f1pow * 27616185.992 - f2pow * 27616184.819) / (f1pow - f2pow);
    let pc_e03_rover = (f1pow * 27055946.391 - f2pow * 27055945.532) / (f1pow - f2pow);
    let pc_e05_rover = (f1pow * 23730317.923 - f2pow * 23730316.788) / (f1pow - f2pow);

    let pc_e01_base = (f1pow * 27506424.743 - f2pow * 27506425.902) / (f1pow - f2pow);
    let pc_e03_base = (f1pow * 26952639.751 - f2pow * 26952641.150) / (f1pow - f2pow);
    let pc_e05_base = (f1pow * 23595077.027 - f2pow * 23595078.180) / (f1pow - f2pow);

    for (sat, dd) in dbl_diff.inner.iter() {
        assert!(*sat != E05, "did not remove pivot sat!");

        if *sat == E01 {
            let (carrier, code) = dd.code.expect("E01 missing DD(code)");

            assert_eq!(carrier, Carrier::L1);

            assert_eq!(
                code,
                pc_e01_rover - pc_e05_rover - (pc_e01_base - pc_e05_base)
            );

            e01_passed = true;
        } else if *sat == E03 {
            let (carrier, code) = dd.code.expect("E03 missing DD(code)");

            assert_eq!(carrier, Carrier::L1);

            e03_passed = true;
        }
    }

    assert!(e01_passed, "E01 test failed");
    assert!(e03_passed, "E03 test failed");
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

    let mut e01_passed = false;
    let mut e03_passed = false;

    let (f1, f2) = (Carrier::L1.frequency_hz(), Carrier::E5b.frequency_hz());
    let (f1pow, f2pow) = (f1.powi(2), f2.powi(2));

    let pc_e01_rover = (f1pow * 27616185.992 - f2pow * 27616184.819) / (f1pow - f2pow);
    let pc_e03_rover = (f1pow * 27055946.391 - f2pow * 27055945.532) / (f1pow - f2pow);
    let pc_e05_rover = (f1pow * 23730317.923 - f2pow * 23730316.788) / (f1pow - f2pow);

    let lc_e01_rover = (f1pow * 145124050.106 - f2pow * 108371872.760) / (f1pow - f2pow);
    let lc_e03_rover = (f1pow * 142179967.778 - f2pow * 106173364.686) / (f1pow - f2pow);
    let lc_e05_rover = (f1pow * 124703702.220 - f2pow * 93122915.921) / (f1pow - f2pow);

    let pc_e01_base = (f1pow * 27506424.743 - f2pow * 27506425.902) / (f1pow - f2pow);
    let pc_e03_base = (f1pow * 26952639.751 - f2pow * 26952641.150) / (f1pow - f2pow);
    let pc_e05_base = (f1pow * 23595077.027 - f2pow * 23595078.180) / (f1pow - f2pow);

    let lc_e01_base = (f1pow * 144547262.730 - f2pow * 107941157.967) / (f1pow - f2pow);
    let lc_e03_base = (f1pow * 141637100.045 - f2pow * 105767978.933) / (f1pow - f2pow);
    let lc_e05_base = (f1pow * 123992999.980 - f2pow * 92592203.174) / (f1pow - f2pow);

    for (sat, dd) in dbl_diff.inner.iter() {
        assert!(*sat != E05, "did not remove pivot sat!");

        if *sat == E01 {
            let (carrier, code) = dd.code.expect("E01 missing DD(code)");

            assert_eq!(carrier, Carrier::L1);

            assert_eq!(
                code,
                pc_e01_rover - pc_e05_rover - (pc_e01_base - pc_e05_base)
            );

            let (carrier, lambda, phase) = dd.phase.expect("E01 missing DD(phase)");

            assert_eq!(carrier, Carrier::L1);

            let freq = f1 * f2 / (f1pow + f2pow).sqrt();
            assert_eq!(lambda, SPEED_OF_LIGHT_M_S / freq);

            assert_eq!(
                phase,
                lc_e01_rover - lc_e05_rover - (lc_e01_base - lc_e05_base)
            );

            e01_passed = true;
        } else if *sat == E03 {
            let (carrier, code) = dd.code.expect("E03 missing DD(code)");

            assert_eq!(carrier, Carrier::L1);

            e03_passed = true;
        }
    }

    assert!(e01_passed, "E01 test failed");
    assert!(e03_passed, "E03 test failed");
}
