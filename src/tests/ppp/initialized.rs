use log::info;
use rstest::*;
use std::str::FromStr;

use crate::{
    navigation::apriori::Apriori,
    prelude::{Almanac, Config, Epoch, Frame, Method, StaticSolver, UserParameters},
    tests::{
        bias::NullBias, ephemeris::NullEph, init_logger, time::NullTime, CandidatesBuilder,
        OrbitsData, REFERENCE_COORDS_ECEF_M,
    },
};

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
fn build_initial_apriori() -> Apriori {
    use crate::tests::reference_apriori_at_ref_epoch;
    reference_apriori_at_ref_epoch()
}

#[test]
fn static_ppp() {
    init_logger();

    let cfg = Config::default().with_navigation_method(Method::PPP);

    let default_params = UserParameters::default();

    let almanac = build_almanac();
    let earth_frame = build_earth_frame();

    let null_bias = NullBias {};
    let null_time = NullTime {};
    let null_eph = NullEph {};

    let orbits_data = OrbitsData::new(earth_frame);

    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();
    let candidates = CandidatesBuilder::build_at(t0_gpst);

    let mut solver = StaticSolver::new(
        almanac,
        earth_frame,
        cfg,
        null_eph.into(),
        orbits_data.into(),
        null_time,
        null_bias,
        Some(REFERENCE_COORDS_ECEF_M),
    );

    let status = solver.ppp_solving(t0_gpst, default_params, &candidates);

    match status {
        Err(e) => panic!("Static PPP process failed with invalid error: {}", e),
        Ok(pvt) => {
            info!("1st solution: {:#?}", pvt);
        },
    }

    // TODO continue
    // let t1_gpst = Epoch::from_str("2020-06-25T00:15:00 GPST").unwrap();
    // let candidates = CandidatesBuilder::build_at(t1_gpst);

    // let pvt = solver.resolve(default_user, t1_gpst, &candidates)
    //     .unwrap();
}
