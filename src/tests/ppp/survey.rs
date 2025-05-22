use rstest::*;
use std::str::FromStr;

use crate::{
    navigation::apriori::Apriori,
    ppp::PPP,
    prelude::{Almanac, Config, Epoch, Error, Frame, User},
    tests::{bias::NullBias, time::NullTime, CandidatesBuilder, OrbitsData},
};

#[fixture]
fn build_almanac() -> Almanac {
    use crate::tests::test_almanac;
    test_almanac()
}

#[fixture]
fn build_earth_frame() -> Frame {
    use crate::tests::test_earth_frame;
    test_earth_frame()
}

#[fixture]
fn build_initial_apriori() -> Apriori {
    use crate::tests::test_reference_apriori;
    test_reference_apriori()
}

#[test]
fn static_ppp_survey() {
    let default_cfg = Config::default();
    let default_user = User::default();

    let almanac = build_almanac();
    let earth_frame = build_earth_frame();

    let null_bias = NullBias {};
    let null_time = NullTime {};

    let orbits_data = OrbitsData::new(earth_frame);

    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();
    let candidates = CandidatesBuilder::build_at(t0_gpst);

    let mut solver = PPP::new_survey(
        almanac,
        earth_frame,
        default_cfg,
        orbits_data.into(),
        null_time,
        null_bias,
    );

    let status = solver.resolve(default_user, t0_gpst, &candidates);

    match status {
        Err(Error::InvalidatedFirstSolution) => {},
        Err(e) => panic!("Static PPP resolution failed with invalid error: {}", e),
        Ok(_) => panic!("first solution should be invalidated"),
    }

    // TODO continue
    // let t1_gpst = Epoch::from_str("2020-06-25T00:15:00 GPST").unwrap();
    // let candidates = CandidatesBuilder::build_at(t1_gpst);

    // let pvt = solver.resolve(default_user, t1_gpst, &candidates)
    //     .unwrap();
}
