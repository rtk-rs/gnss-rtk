//! Solver configuration preset

use nalgebra::{dimension::U8, OMatrix};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

const fn default_closed_loop() -> bool {
    true
}

const fn default_postfit_kf() -> bool {
    false
}

const fn default_gdop_threshold() -> Option<f64> {
    None
}

const fn default_tdop_threshold() -> Option<f64> {
    None
}

#[derive(Default, Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub struct ElevationMappingFunction {
    /// a + b * e-elev/c
    pub a: f64,
    /// a + b * e-elev/c
    pub b: f64,
    /// a + b * e-elev/c
    pub c: f64,
}

impl ElevationMappingFunction {
    pub(crate) fn eval(&self, elev_sv: f64) -> f64 {
        self.a + self.b * (elev_sv / self.c).exp()
    }
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub enum WeightMatrix {
    /// a + b e-elev/c
    MappingFunction(ElevationMappingFunction),
    /// Advanced measurement noise covariance matrix
    Covar,
}

fn default_weight_matrix() -> Option<WeightMatrix> {
    None
    //Some(WeightMatrix::MappingFunction(
    //    ElevationMappingFunction {
    //        a: 5.0,
    //        b: 0.0,
    //        c: 10.0,
    //    },
    //))
}

#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub enum LoopExitCriteria {
    Iteration(usize),
    Norm(f64),
}

impl Default for LoopExitCriteria {
    fn default() -> LoopExitCriteria {
        LoopExitCriteria::Iteration(10)
    }
}

#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub struct FilterOpts {
    /// Criteria to exit the Least Square fitting loop, for a given Epoch
    #[cfg_attr(feature = "serde", serde(default))]
    pub loop_exit: LoopExitCriteria,
    /// The navigation filter operates in closed loop by default.
    /// If you disable this feature, this will prohibit any dynamic applications,
    /// only static will remain possible. And convergence time will decrease significantly.
    #[cfg_attr(feature = "serde", serde(default = "default_closed_loop"))]
    pub closed_loop: bool,
    /// Weight Matrix
    #[cfg_attr(feature = "serde", serde(default = "default_weight_matrix"))]
    pub weight_matrix: Option<WeightMatrix>,
}

impl Default for FilterOpts {
    fn default() -> Self {
        Self {
            closed_loop: true,
            weight_matrix: None,
            loop_exit: LoopExitCriteria::default(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub struct SolverOpts {
    /// GDOP threshold to invalidate ongoing GDOP
    #[cfg_attr(feature = "serde", serde(default = "default_gdop_threshold"))]
    pub gdop_threshold: Option<f64>,
    /// TDOP threshold to invalidate ongoing TDOP
    #[cfg_attr(feature = "serde", serde(default = "default_tdop_threshold"))]
    pub tdop_threshold: Option<f64>,
    // /// Filter to use
    // #[cfg_attr(feature = "serde", serde(default))]
    // pub filter: Filter,
    /// Filter options
    #[cfg_attr(feature = "serde", serde(default))]
    pub filter: FilterOpts,
    /// Deploy a post-processing denoising Kalman filter,
    /// at the expense of more processing load. This may apply even
    /// when your navigation is also a Kalman filter.
    #[cfg_attr(feature = "serde", serde(default = "default_postfit_kf"))]
    pub postfit_kf: bool,
}

impl Default for SolverOpts {
    fn default() -> Self {
        Self {
            filter: Default::default(),
            postfit_kf: default_postfit_kf(),
            gdop_threshold: default_gdop_threshold(),
            tdop_threshold: default_tdop_threshold(),
        }
    }
}

impl SolverOpts {
    /*
     * form the weight matrix to be used in the solving process
     */
    pub(crate) fn weight_matrix(&self) -> OMatrix<f64, U8, U8> {
        match self.filter.weight_matrix {
            Some(WeightMatrix::Covar) => panic!("not implemented yet"),
            Some(WeightMatrix::MappingFunction(_)) => panic!("mapf: not implemented yet"),
            //                Some(WeightMatrix::MappingFunction(mapf)) => {
            //                    for i in 0..8 {
            //                        let sigma = mapf.a + mapf.b * ((-sv_elev[i]) / mapf.c).exp();
            //                        mat[(i, i)] = 1.0 / sigma.powi(2);
            //                    }
            //                },
            None => OMatrix::<f64, U8, U8>::identity(),
        }
    }
}
