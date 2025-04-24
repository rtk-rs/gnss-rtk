//! Solver configuration preset

use nalgebra::{dimension::U8, OMatrix};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

const fn default_max_gdop() -> f64 {
    5.0
}

const fn default_postfit_denoising() -> f64 {
    1000.0
}

#[derive(Default, Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SolverOpts {
    /// GDOP threshold to invalidate ongoing GDOP
    #[cfg_attr(feature = "serde", serde(default = "default_max_gdop"))]
    pub max_gdop: f64,
    /// Weight Matrix
    #[cfg_attr(feature = "serde", serde(default = "default_weight_matrix"))]
    pub weight_matrix: Option<WeightMatrix>,
    /// Possible extra denoising filter, at the expense
    /// of more processing time. The configuration is the denoising factor.
    /// 1000 for x1000 improvement attempt.
    #[cfg_attr(feature = "serde", serde(default = "default_postfit_denoising"))]
    pub postfit_denoising: f64,
}

impl Default for SolverOpts {
    fn default() -> Self {
        Self {
            max_gdop: default_max_gdop(),
            weight_matrix: default_weight_matrix(),
            postfit_denoising: default_postfit_denoising(),
        }
    }
}

impl SolverOpts {
    /// Parameter settings recommended for static ultra precise applications
    pub fn static_preset() -> Self {
        Self {
            max_gdop: 3.0,
            weight_matrix: default_weight_matrix(),
            postfit_denoising: default_postfit_denoising(),
        }
    }

    /*
     * form the weight matrix to be used in the solving process
     */
    pub(crate) fn weight_matrix(&self) -> OMatrix<f64, U8, U8> {
        match self.weight_matrix {
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
