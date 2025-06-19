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
fn static_spp() {
    init_logger();

    let cfg = Config::default().with_navigation_method(Method::SPP);

    let default_params = UserParameters::default();

    let almanac = build_almanac();
    let earth_frame = build_earth_frame();

    let null_bias = NullBias {};
    let null_time = NullTime {};
    let null_eph = NullEph {};

    let orbits_data = OrbitsData::new(earth_frame);

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

    for (nth, epoch_str) in [
        "2020-06-25T00:00:00 GPST",
        "2020-06-25T00:15:00 GPST",
        "2020-06-25T00:30:00 GPST",
        "2020-06-25T00:45:00 GPST",
        "2020-06-25T01:00:00 GPST",
    ]
    .iter()
    .enumerate()
    {
        let t_gpst = Epoch::from_str(epoch_str).unwrap();
        let candidates = CandidatesBuilder::build_at(t_gpst);
        assert!(
            candidates.len() > 0,
            "no measurements to propose at \"{}\"",
            epoch_str
        );
        let status = solver.ppp_solving(t_gpst, default_params, &candidates);

        match status {
            Err(e) => panic!("Static SPP process failed with invalid error: {}", e),
            Ok(pvt) => {
                info!("{}th solution {:#?}", nth, pvt);
            },
        }
    }
}
