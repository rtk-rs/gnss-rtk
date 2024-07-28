use crate::{
    prelude::{Config, Error, Filter, Method, PVTSolutionType, Position, Solver, Vector3, TimeScale},
    tests::{
        fake_interpolator,
        Tester,
        data::gps::test_data as gps_test_data,
    },
};

#[test]
#[ignore]
fn spp_lsq_static_survey() {
    let tester = Tester::static_survey_geo(
        TimeScale::GPST,
        (55.493253, 8.458771, 0.0),
        (1.0, 1.0, 1.0),
    );
    let mut cfg = Config::static_preset(Method::SPP);
    cfg.min_snr = None;
    cfg.min_sv_elev = None;
    cfg.solver.filter = Filter::LSQ;
    cfg.sol_type = PVTSolutionType::PositionVelocityTime;
    tester.deploy(&cfg);
}
