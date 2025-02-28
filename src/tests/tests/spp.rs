use crate::{
    prelude::{Config, Error, Filter, Method, PVTSolutionType, Position, Solver, Vector3},
    tests::data::gps::test_data as gps_test_data,
    tests::fake_interpolator,
};

#[test]
fn gps_basic() {
    let initial = Position::from_geo(Vector3::new(55.493253, 8.458771, 0.0));

    let mut cfg = Config::static_preset(Method::SPP);

    cfg.min_snr = None;
    cfg.min_sv_elev = None;
    cfg.solver.filter = Filter::LSQ;
    cfg.sol_type = PVTSolutionType::PositionVelocityTime;
    println!("Testing {:#?}", cfg);

    let mut solver =
        Solver::new(&cfg, Some(initial), fake_interpolator).expect("failed to build solver");

    for (t_index, test_data) in gps_test_data().iter().enumerate() {
        let t = test_data.t_rx;
        let solution = solver.resolve(
            t,
            &test_data.pool,
            &test_data.iono_bias,
            &test_data.tropo_bias,
        );
        match solution {
            Ok((_, solution)) => {
                let pos = solution.position;
                let vel = solution.velocity;
                let dt = solution.dt;
                println!(
                    "{}(Sol): pos={:?} vel={:?} dt={}({})",
                    t, pos, vel, dt, solution.timescale
                );
            },
            Err(e) => {
                println!("{}(Error): {}", t, e);
            },
        }
    }
}
