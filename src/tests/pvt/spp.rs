use crate::{
    prelude::{Config, Epoch, Filter, Method, Orbit, PVTSolutionType, TimeScale, EARTH_J2000},
    tests::Tester,
};

use std::str::FromStr;

#[test]
#[ignore]
fn spp_lsq_static_survey() {
    let orbit = Orbit::from_position(
        0.0,
        0.0,
        0.0,
        Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap(),
        EARTH_J2000,
    );
    let tester = Tester::static_survey(TimeScale::GPST, orbit, (1.0, 1.0, 1.0));
    let mut cfg = Config::static_preset(Method::SPP);
    cfg.min_snr = None;
    cfg.min_sv_elev = None;
    cfg.solver.filter = Filter::LSQ;
    cfg.sol_type = PVTSolutionType::PositionVelocityTime;
    tester.deploy(&cfg);
}
