use log::debug;
use nalgebra::{DMatrix, DVector, Vector3};
use nyx::cosmic::SPEED_OF_LIGHT;
use thiserror::Error;

use crate::{
    cfg::SolverOpts,
    prelude::{Candidate, PVTSolution},
};

#[derive(Clone, Debug, Error)]
pub enum SolutionInvalidation {
    #[error("gdop limit exceeded {0}")]
    GDOPOutlier(f64),
    #[error("tdop limit exceeded {0}")]
    TDOPOutlier(f64),
    #[error("innovation outlier |{0}|")]
    InnovationOutlier(f64),
    #[error("coderes limit exceeded {0}")]
    CodeResidual(f64),
}

pub(crate) struct SolutionValidator {
    gdop: f64,
    tdop: f64,
    residuals: DVector<f64>,
}

impl SolutionValidator {
    pub fn new(
        apriori_ecef: &Vector3<f64>,
        pool: &Vec<Candidate>,
        w: &DMatrix<f64>,
        solution: &PVTSolution,
    ) -> Self {
        let gdop = solution.gdop();
        let tdop = solution.tdop();
        let mut residuals = DVector::<f64>::zeros(pool.len());

        for (idx, cd) in pool.iter().enumerate() {
            let sv = solution
                .sv
                .iter()
                .filter_map(|(sv, data)| if *sv == cd.sv { Some(data) } else { None })
                .reduce(|k, _| k)
                .unwrap();

            let pr = cd.prefered_pseudorange().unwrap().value;
            let state = cd.state.unwrap().position();
            let estimate = apriori_ecef + solution.pos;

            let (sv_x, sv_y, sv_z) = (state[0], state[1], state[2]);
            let (x, y, z) = (estimate[0], estimate[1], estimate[2]);

            let rho = ((sv_x - x).powi(2) + (sv_y - y).powi(2) + (sv_z - z).powi(2)).sqrt();

            let dt = cd.clock_corr.to_seconds() - solution.dt;

            residuals[idx] = pr;
            residuals[idx] -= rho;
            residuals[idx] += dt * SPEED_OF_LIGHT;
            residuals[idx] -= sv.tropo_bias.value().unwrap_or(0.0);
            residuals[idx] -= sv.iono_bias.value().unwrap_or(0.0);
            residuals[idx] /= w[(idx, idx)];
            debug!(
                "{:?} ({}): coderes={}/w={}",
                cd.t,
                cd.sv,
                residuals[idx],
                w[(idx, idx)]
            );
        }
        Self {
            residuals,
            gdop,
            tdop,
        }
    }
    /*
     * Solution validation process
     */
    pub fn valid(&self, opts: &SolverOpts) -> Result<(), SolutionInvalidation> {
        if let Some(max_gdop) = opts.gdop_threshold {
            if self.gdop > max_gdop {
                return Err(SolutionInvalidation::GDOPOutlier(self.gdop));
            }
        }
        if let Some(max_tdop) = opts.tdop_threshold {
            if self.tdop > max_tdop {
                return Err(SolutionInvalidation::TDOPOutlier(self.tdop));
            }
        }
        Ok(())
    }
}
