use crate::prelude::{
    Candidate, Config, Epoch, Error, Frame, InvalidationCause, Orbit, OrbitSource, PVTSolution,
    Solver, TimeScale, SV,
};

use std::cell::RefCell;

mod bancroft;
mod data;
mod pseudo_range;
mod pvt;

pub mod cfg;
use cfg::TestConfig;

pub mod output;
use output::TestOutput;

use std::collections::HashMap;

use data::{gps::test_data as gps_test_data, interp::interp_data};

#[derive(Debug, Clone)]
pub struct OrbitKey {
    t: Epoch,
    sv: SV,
}

#[derive(Debug, Clone)]
struct OrbitDataBase {
    inner: HashMap<OrbitKey, Orbit>,
}

impl OrbitSource for OrbitDataBase {
    fn next_at(&mut self, t: Epoch, sv: SV, fr: Frame, _: usize) -> Option<Orbit> {
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

struct Test {
    pub cfg: TestConfig,
    pub input: Vec<SolverInput>,
    pub orbit_db: OrbitDataBase,
    pub solver: Solver,
}

impl Test {
    pub fn run_test(&mut self) -> TestOutput {
        let mut output = TestOutput::default();

        while let Some(input) = self.input.iter_mut().next() {
            let (t_rx, pool) = (input.t_rx, &mut input.pool);

            println!("running test: {} [{}]", input.t_rx, pool.len());

            if let Some(solution) =
                Self::test_iter(&mut self.solver, t_rx, pool, self.orbit_db.clone())
            {
                output.nb_solutions += 1;
            }

            output.nb_iter += 1;
        }

        output
    }

    fn test_iter(
        solver: &mut Solver,
        t_rx: Epoch,
        pool: &mut Vec<Candidate>,
        orbit: OrbitDataBase,
    ) -> Option<PVTSolution> {
        match solver.resolve(t_rx, pool, orbit) {
            Ok((_, solution)) => {
                return Some(solution);
            },
            Err(e) => {
                match e {
                    Error::NotEnoughCandidates => {},
                    Error::NotEnoughCandidatesBancroft => {},
                    Error::NotEnoughPreFitCandidates => {},
                    Error::NotEnoughPostFitCandidates => {},
                    Error::MatrixFormationError => {},
                    Error::UnknownClockCorrection => {},
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
                    Error::UnresolvedStateBancroft => {
                        panic!("bancroft resolution attempt, without enough SV");
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
                    Error::MetaAlmanac(e) => {
                        panic!("almanac setup error: {}", e);
                    },
                    Error::Almanac(e) => {
                        panic!("almanac determination error: {}", e);
                    },
                    Error::EarthFrame(e) => {
                        panic!("earth frame error: {}", e);
                    },
                }
            },
        }
        None
    }
}
