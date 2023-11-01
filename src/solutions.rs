//! PVT Solutions

// use nalgebra::linalg::svd::SVD;
use nalgebra::base::{DVector, MatrixXx4};
use nyx_space::cosmic::SPEED_OF_LIGHT;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
/// PVT Solution, always expressed as the correction to apply
/// to an Apriori position.
pub struct PVTSolution {
    /// X coordinates correction in [m]
    pub dx: f64,
    /// Y coordinates correction in [m]
    pub dy: f64,
    /// Z coordinates correction in [m]
    pub dz: f64,
    /// Time correction in [s]
    pub dt: f64,
    /// Horizontal Dilution of Precision
    pub hdop: f64,
    /// Vertical Dilution of Precision
    pub vdop: f64,
    /// Time Dilution of Precision in [s]
    pub tdop: f64,
}

impl PVTSolution {
    /// Builds a new PVTSolution from
    /// g : the navigation matrix
    /// y: the navigation vector
    pub fn new(g: MatrixXx4<f64>, y: DVector<f64>) -> Option<Self> {
        let g_prime = g.clone().transpose();
        let q = (g_prime.clone() * g.clone()).try_inverse()?;
        let x = q * g_prime.clone();
        let x = x * y;
        let hdop = (q[(0, 0)] + q[(1, 1)]).sqrt();
        let vdop = q[(2, 2)].sqrt();
        let tdop = q[(3, 3)].sqrt();
        Some(Self {
            dx: x[0],
            dy: x[1],
            dz: x[2],
            dt: x[3] / SPEED_OF_LIGHT,
            hdop,
            vdop,
            tdop,
        })
    }
}
