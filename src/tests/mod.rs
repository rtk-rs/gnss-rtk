use crate::prelude::{
    Candidate, Config, Epoch, Error, InvalidationCause, Orbit, OrbitalStateProvider, PVTSolution,
    Solver, TimeScale, SV,
};

mod bancroft;
mod data;
mod pseudo_range;
mod pvt;

use data::{gps::test_data as gps_test_data, interp::interp_data};

struct Orbits {}

impl OrbitalStateProvider for Orbits {
    fn next_at(&mut self, t: Epoch, sv: SV, _: usize) -> Option<Orbit> {
        Some(
            interp_data()
                .iter()
                .filter(|(sv_i, _)| *sv_i == sv)
                .min_by_key(|(_, orb)| (orb.epoch - t).abs())?
                .1,
        )
    }
}

struct SolverInput {
    t_rx: Epoch,
    pool: Vec<Candidate>,
}

#[derive(Debug, Default, Clone)]
struct Tester {
    kinematic: bool,
    timescale: TimeScale,
    max_gdop: Option<f64>,
    max_tdop: Option<f64>,
    reference: Option<Orbit>,
    max_xyz_err_m: (f64, f64, f64),
    max_velocity_m_s: (f64, f64, f64),
}

impl Tester {
    /// Builds new Static Survey tester, for given ECEF [m]
    pub fn static_survey(
        timescale: TimeScale,
        reference: Orbit,
        max_xyz_err_m: (f64, f64, f64),
    ) -> Self {
        let mut s = Self::default();
        s.kinematic = false;
        s.timescale = timescale;
        s.max_xyz_err_m = max_xyz_err_m;
        s.reference = Some(reference);
        // on static applications, we tolerate this "erroneous" motion
        s.max_velocity_m_s = (1.0E-5, 1.0E-5, 1.0E-5);
        s
    }
    /// Set max tdop criteria
    pub fn with_max_tdop(&self, tdop: f64) -> Self {
        let mut s = self.clone();
        s.max_tdop = Some(tdop);
        s
    }
    /// Set max gdop criteria
    pub fn with_max_gdop(&self, gdop: f64) -> Self {
        let mut s = self.clone();
        s.max_gdop = Some(gdop);
        s
    }
    pub fn deploy(&self, cfg: &Config) {
        self.deploy_without_apriori(cfg);
        if self.reference.is_some() {
            self.deploy_with_apriori(cfg);
        }
    }
    fn deploy_without_apriori(&self, cfg: &Config) {
        let orbits = Orbits {};
        let mut solver = Solver::survey(&cfg, orbits)
            .unwrap_or_else(|e| panic!("failed to deploy solver with {:#?}: error={}", cfg, e));
        println!("deployed with {:#?}", cfg);
        self.run(&mut solver, cfg);
    }
    fn deploy_with_apriori(&self, cfg: &Config) {
        let orbits = Orbits {};
        let mut solver =
            Solver::new(&cfg, None, orbits) // TODO
                .unwrap_or_else(|e| panic!("failed to deploy solver with {:#?}: error={}", cfg, e));
        println!("deployed with {:#?}", cfg);
        self.run(&mut solver, cfg);
    }
    fn run<O: OrbitalStateProvider>(&self, solver: &mut Solver<O>, cfg: &Config) {
        for (data_index, data) in gps_test_data().iter_mut().enumerate() {
            match solver.resolve(data.t_rx, &mut data.pool) {
                Ok((_, solution)) => {
                    let state = solution.state;
                    let state = state.to_cartesian_pos_vel();
                    let (x_km, y_km, z_km, vel_x, vel_y, vel_z) =
                        (state[0], state[1], state[2], state[3], state[4], state[5]);
                    println!(
                        "iter={}, 3d=(x={}km, y={}km, z={}km) vel=(x={}km/s, y={}km/s, z={}km/s)",
                        data_index, x_km, y_km, z_km, vel_x, vel_y, vel_z,
                    );
                    self.static_run(&cfg, solution);
                },
                Err(e) => match e {
                    Error::NotEnoughCandidates => {},
                    Error::NotEnoughCandidatesBancroft => {},
                    Error::NotEnoughPreFitCandidates => {},
                    Error::NotEnoughPostFitCandidates => {},
                    Error::MatrixFormationError => {},
                    Error::UnknownClockCorrection => {},
                    Error::UnknownClockCorrectionBancroft => {},
                    Error::MissingRemoteRTKObservation(..) => {},
                    Error::MissingRemoteRTKObservations => {},
                    Error::MatrixInversionError => {},
                    Error::TimeIsNan => {
                        panic!("resolved dt is Not A Number");
                    },
                    Error::InvalidStrategy => {},
                    Error::NavigationError => {},
                    Error::MissingPseudoRange => {},
                    Error::PseudoRangeCombination => {},
                    Error::PhaseRangeCombination => {},
                    Error::InvalidatedSolution(cause) => match cause {
                        InvalidationCause::FirstSolution => {},
                        InvalidationCause::GDOPOutlier(..) => {},
                        InvalidationCause::TDOPOutlier(..) => {},
                        InvalidationCause::InnovationOutlier(..) => {},
                        InvalidationCause::CodeResidual(..) => {},
                    },
                    Error::UnresolvedState => {
                        panic!("navigation attempt while some states still remain unresolved or ambiguous");
                    },
                    Error::PhysicalNonSenseRxPriorTx | Error::PhysicalNonSenseRxTooLate => {
                        panic!("physics_err: error in signal propgation");
                    },
                    Error::Physics(e) => {
                        panic!("physics_err: {}", e);
                    },
                    Error::BancroftError => {
                        panic!("bancroft error");
                    },
                    Error::BancroftImaginarySolution => {
                        panic!("invalid bancroft solution");
                    },
                    Error::UnresolvedAmbiguity => {
                        panic!("navigation attempt while some ambiguities still remain");
                    },
                    Error::Almanac(e) => {
                        panic!("almanac determination error: {}", e);
                    },
                    Error::EarthFrame => {
                        panic!("earth frame error");
                    },
                },
            }
        }
    }
    fn static_run(&self, _cfg: &Config, sol: PVTSolution) {
        //let reference = self.reference.as_ref().unwrap();
        //// let (x0, y0, z0) = (xyz_ecef_m[0], xyz_ecef_m[1], xyz_ecef_m[2]);
        //let orbit = sol.state;
        //let state = orbit.to_cartesian_pos_vel();
        //let (x_km, y_km, z_km, vel_x_km, vel_y_km, vel_z_km) = (
        //    state[0],
        //    state[1],
        //    state[2],
        //    state[3],
        //    state[4],
        //    state[5],
        //);
        //let (x_err, y_err, z_err) = ((x_km * 1.0E3 - x0).abs(), (y_km * 1.0E3 - y0).abs(), (z_km * 1.0E3 - z0).abs());
        assert_eq!(
            sol.timescale, self.timescale,
            "solution expressed in wrong timescale"
        );
        //if let Some(max_gdop) = self.max_gdop {
        //    assert!(
        //        sol.gdop.abs() < max_gdop,
        //        "{} gdop limit exceeded",
        //        max_gdop
        //    );
        //}
        //if let Some(max_tdop) = self.max_tdop {
        //    assert!(
        //        sol.tdop.abs() < max_tdop,
        //        "{} tdop limit exceeded",
        //        max_tdop
        //    );
        //}
        //assert!(
        //    vel_x_km.abs() <= self.max_velocity_m_s.0,
        //    "{} vel_x component above tolerance",
        //    vel_x_km.abs()
        //);
    }
}
