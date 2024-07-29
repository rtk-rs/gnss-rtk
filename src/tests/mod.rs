use crate::prelude::*;

mod bancroft;
mod data;
mod pseudo_range;
mod pvt;

use data::{gps::test_data as gps_test_data, interp::interp_data};

struct Orbits {}

impl OrbitalStateProvider for Orbits {
    fn next_at_ecef(&self, t: Epoch, sv: SV, order: usize) -> Option<OrbitalState> {
        Some(
            interp_data()
                .iter()
                .filter(|k| k.1 == sv)
                .min_by_key(|k| (k.0 - t).abs())?
                .2,
        )
    }
}

struct SolverInput {
    t_rx: Epoch,
    pool: Vec<Candidate>,
    iono_bias: IonosphereBias,
    tropo_bias: TroposphereBias,
}

#[derive(Debug, Default, Clone)]
struct Tester {
    kinematic: bool,
    timescale: TimeScale,
    max_gdop: Option<f64>,
    max_tdop: Option<f64>,
    reference: Option<Position>,
    max_xyz_err_m: (f64, f64, f64),
    max_velocity_m_s: (f64, f64, f64),
}

impl Tester {
    /// Builds new Static Survey tester, for given ECEF [m]
    pub fn static_survey_ecef(
        timescale: TimeScale,
        reference_ecef_m: (f64, f64, f64),
        max_xyz_err_m: (f64, f64, f64),
    ) -> Self {
        let mut s = Self::default();
        s.kinematic = false;
        s.timescale = timescale;
        s.max_xyz_err_m = max_xyz_err_m;
        // on static applications, we tolerate this "erroneous" motion
        s.max_velocity_m_s = (1.0E-5, 1.0E-5, 1.0E-5);
        s.reference = Some(Position::from_ecef(Vector3::new(
            reference_ecef_m.0,
            reference_ecef_m.1,
            reference_ecef_m.2,
        )));
        s
    }
    /// Builds new Static Survey tester, for given GEO [ddeg]
    pub fn static_survey_geo(
        timescale: TimeScale,
        reference_geo_ddeg: (f64, f64, f64),
        max_xyz_err_m: (f64, f64, f64),
    ) -> Self {
        let mut s = Self::default();
        s.kinematic = false;
        s.timescale = timescale;
        s.max_xyz_err_m = max_xyz_err_m;
        // on static applications, we tolerate this "erroneous" motion
        s.max_velocity_m_s = (1.0E-5, 1.0E-5, 1.0E-5);
        s.reference = Some(Position::from_geo_ddeg(Vector3::new(
            reference_geo_ddeg.0,
            reference_geo_ddeg.1,
            reference_geo_ddeg.2,
        )));
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
        let orbits = Arc::new(Orbits {});
        let mut solver = Solver::new(&cfg, None, orbits)
            .unwrap_or_else(|e| panic!("failed to deploy solver with {:#?}: error={}", cfg, e));
        println!("deployed with {:#?}", cfg);
        self.run(&mut solver, cfg);
    }
    fn deploy_with_apriori(&self, cfg: &Config) {
        let orbits = Arc::new(Orbits {});
        let mut solver = Solver::new(&cfg, self.reference.clone(), orbits)
            .unwrap_or_else(|e| panic!("failed to deploy solver with {:#?}: error={}", cfg, e));
        println!("deployed with {:#?}", cfg);
        self.run(&mut solver, cfg);
    }
    fn run(&self, solver: &mut Solver, cfg: &Config) {
        for (data_index, data) in gps_test_data().iter().enumerate() {
            match solver.resolve(data.t_rx, &data.pool, &data.iono_bias, &data.tropo_bias) {
                Ok((t, solution)) => {
                    println!(
                        "iter={}, 3d={:?} vel={:?}",
                        data_index, solution.position, solution.velocity
                    );
                    self.static_run(&cfg, solution);
                },
                Err(e) => match e {
                    Error::NotEnoughCandidates => {},
                    Error::NotEnoughMatchingCandidates => {},
                    Error::MatrixError => {},
                    Error::FirstGuess => {},
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
                        InvalidationCause::GDOPOutlier(value) => {},
                        InvalidationCause::TDOPOutlier(value) => {},
                        InvalidationCause::InnovationOutlier(value) => {},
                        InvalidationCause::CodeResidual(value) => {},
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
                },
            }
        }
    }
    fn static_run(&self, cfg: &Config, sol: PVTSolution) {
        let reference = self.reference.as_ref().unwrap();
        let xyz_ecef_m = reference.ecef();
        let (x0, y0, z0) = (xyz_ecef_m[0], xyz_ecef_m[1], xyz_ecef_m[2]);
        let (x, y, z) = (sol.position[0], sol.position[1], sol.position[2]);
        let (vel_x, vel_y, vel_z) = (sol.velocity[0], sol.velocity[1], sol.velocity[2]);
        let (x_err, y_err, z_err) = ((x - x0).abs(), (y - y0).abs(), (z - z0).abs());
        assert_eq!(
            sol.timescale, self.timescale,
            "solution expressed in wrong timescale"
        );
        if let Some(max_gdop) = self.max_gdop {
            assert!(
                sol.gdop.abs() < max_gdop,
                "{} gdop limit exceeded",
                max_gdop
            );
        }
        if let Some(max_tdop) = self.max_tdop {
            assert!(
                sol.tdop.abs() < max_tdop,
                "{} tdop limit exceeded",
                max_tdop
            );
        }
        assert!(
            vel_x.abs() <= self.max_velocity_m_s.0,
            "{} vel_x component above tolerance",
            vel_x.abs()
        );
    }
}
