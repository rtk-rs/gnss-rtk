use crate::{
    prelude::{Config, Error, Filter, Method, PVTSolutionType, Position, Solver, Vector3},
    tests::data::gps::test_data as gps_test_data,
    tests::fake_interpolator,
};

#[test]
fn test_3d_lsq() {
    let initial = Position::from_geo(Vector3::new(55.493253, 8.458771, 0.0));

    let mut cfg = Config::static_preset(Method::SPP);

    cfg.min_snr = None;
    cfg.min_sv_elev = None;
    cfg.solver.filter = Filter::LSQ;
    cfg.sol_type = PVTSolutionType::PositionVelocityTime;
    println!("Testing {:#?}", cfg);

    let mut solver =
        Solver::new(&cfg, Some(initial), fake_interpolator).expect("failed to build solver");

    for (index, test_data) in gps_test_data().iter().enumerate() {
        let solutions = solver.resolve(
            test_data.t_rx,
            &test_data.pool,
            &test_data.iono_bias,
            &test_data.tropo_bias,
        );
        if index == 0 {
            assert!(
                solutions.is_err(),
                "first solution should have been discarded"
            );
            let err = solutions.err().unwrap();
            assert_eq!(err, Error::NotEnoughMatchingCandidates); //TODO
        } else {
            assert!(
                solutions.is_ok(),
                "failed to resolve @{} - {}",
                test_data.t_rx,
                solutions.err().unwrap(),
            );

            let solutions = solutions.unwrap();
            println!("resolved: {:?}", solutions);
        }
    }
}
