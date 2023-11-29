//! PVT Solutions
use crate::prelude::{Vector3, SV};
use crate::Error;
use std::collections::HashMap;

#[derive(Debug, Copy, Clone, Default)]
pub enum PVTSolutionType {
    /// Default, complete solution with Position,
    /// Velocity and Time components. Requires either
    /// 4 vehicles in sight, or 3 if you're working in fixed altitude
    /// (provided ahead of time).
    #[default]
    PositionVelocityTime,
    /// Resolve Time component only. Requires 1 vehicle to resolve.
    TimeOnly,
}

impl std::fmt::Display for PVTSolutionType {
    /*
     * Prints self
     */
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Self::PositionVelocityTime => write!(f, "PVT"),
            Self::TimeOnly => write!(f, "TimeOnly"),
        }
    }
}

use nalgebra::base::{DMatrix, DVector, Matrix1x4, Matrix4, Matrix4x1, MatrixXx4};
use nyx::cosmic::SPEED_OF_LIGHT;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Modeled (estimated) or measured Time Delay.
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PVTBias {
    /// Measured delay [meters of delay]
    pub measured: Option<f64>,
    /// Modeled delay [meters of delay]
    pub modeled: Option<f64>,
}

impl PVTBias {
    /// Time Delay in [s], whether it was modeled
    /// or physically measured (prefered).
    pub fn value(&self) -> Option<f64> {
        if self.measured.is_none() {
            self.modeled
        } else {
            self.measured
        }
    }
    /// Builds a measured Time Delay from a measurement in [s]
    pub fn measured(measurement: f64) -> Self {
        Self {
            measured: Some(measurement),
            modeled: None,
        }
    }
    /// Builds a modeled Time Delay from a model in [s]
    pub fn modeled(model: f64) -> Self {
        Self {
            modeled: Some(model),
            measured: None,
        }
    }
}

/// Data attached to each individual SV that helped form the PVT solution.
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PVTSVData {
    /// Azimuth angle at resolution time
    pub azimuth: f64,
    /// Elevation angle at resolution time
    pub elevation: f64,
    /// Either measured or modeled Tropospheric Delay
    /// that impacted L1 signal
    pub tropo_bias: PVTBias,
    /// Either measured or modeled Ionospheric Delay
    /// that impacted L1 signal
    pub iono_bias: PVTBias,
}

#[derive(Default, Clone, Debug)]
pub struct Estimate {
    /* x: estimate */
    pub(crate) x: Matrix4x1<f64>,
    /* and its covariance matrix */
    pub(crate) covar: Matrix4<f64>,
}

/// PVT Solution, always expressed as the correction to apply
/// to an Apriori / static position.
#[derive(Debug, Clone, Default)]
// #[cfg_attr(feature = "serde", derive(Serialize))]
pub struct PVTSolution {
    /// Position errors (in [m] ECEF)
    pub p: Vector3<f64>,
    /// Absolute Velocity (in [m/s] ECEF).
    pub v: Vector3<f64>,
    /// Time correction in [s]
    pub dt: f64,
    /// Space Vehicles that helped form this solution
    /// and data associated to each individual SV
    pub sv: HashMap<SV, PVTSVData>,
    // estimate
    pub(crate) estimate: Estimate,
}

impl PVTSolution {
    /// Builds a new PVTSolution from
    /// "g": the navigation matrix
    /// "w": diagonal weight matrix
    /// "y": the navigation vector
    /// "sv": attached SV data
    pub(crate) fn new(
        g: MatrixXx4<f64>,
        w: DMatrix<f64>,
        y: DVector<f64>,
        sv: HashMap<SV, PVTSVData>,
        p_state: Option<Estimate>,
    ) -> Result<Self, Error> {
        let g_prime = g.clone().transpose();

        let estimate = match p_state {
            None => {
                let q = (g_prime.clone() * w.clone() * g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;
                let x = (q * g_prime.clone()) * w.clone() * y;
                Estimate { x, covar: q }
            },
            Some(state) => {
                let p_1 = state
                    .covar
                    .try_inverse()
                    .ok_or(Error::CovarMatrixInversionError)?;

                let q = g_prime.clone() * w.clone() * g.clone();
                let covar = (p_1 + q)
                    .try_inverse()
                    .ok_or(Error::CovarMatrixInversionError)?;

                let x = covar * (p_1 * state.x + (g_prime.clone() * w.clone() * y));
                Estimate { x, covar }
            },
        };

        let dt = estimate.x[3] / SPEED_OF_LIGHT;
        if dt.is_nan() {
            return Err(Error::TimeIsNan);
        }

        Ok(Self {
            sv,
            p: Vector3::new(estimate.x[0], estimate.x[1], estimate.x[2]),
            v: Vector3::<f64>::default(),
            dt,
            estimate,
        })
    }
    /// Returns list of Space Vehicles (SV) that help form this solution.
    pub fn sv(&self) -> Vec<SV> {
        self.sv.keys().copied().collect()
    }
    /// Returns Horizontal Dilution of Precision of this PVT
    pub fn hdop(&self) -> f64 {
        (self.estimate.covar[(0, 0)] + self.estimate.covar[(1, 1)]).sqrt()
    }
    /// Returns Vertical Dilution of Precision of this PVT
    pub fn vdop(&self) -> f64 {
        self.estimate.covar[(2, 2)].sqrt()
    }
    /// Returns Time Dilution of Precision of this PVT
    pub fn tdop(&self) -> f64 {
        self.estimate.covar[(3, 3)].sqrt()
    }
}
