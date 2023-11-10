//! PVT Solutions
use crate::{Error, Vector3D};

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
     * Prints self as XYY standard format
     */
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Self::PositionVelocityTime => write!(f, "PVT"),
            Self::TimeOnly => write!(f, "TimeOnly"),
        }
    }
}

use nalgebra::base::{DVector, MatrixXx4};
use nyx_space::cosmic::SPEED_OF_LIGHT;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Modeled (estimated) or measured Ionospheric Delay.
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PVTIonoDelay {
    /// Measured ionospheric delay [s] (best case scenario)
    pub measured: Option<f64>,
    /// Modeled ionospheric delay [s]
    pub modeled: Option<f64>,
}

impl PVTIonoDelay {
    /// Ionospheric delay (in [s)), whether it was modeled
    /// or measured physically (prefered).
    pub fn value(&self) -> Option<f64> {
        self.measured?;
        self.modeled
    }
}

#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
/// PVT Solution, always expressed as the correction to apply
/// to an Apriori position.
pub struct PVTSolution {
    /// X, Y, Z corrections (in [m])
    pub p: Vector3D,
    /// Absolute Velocity (in [m/s] ECEF).
    pub v: Vector3D,
    /// Time correction in [s]
    pub dt: f64,
    /// Horizontal Dilution of Precision
    pub hdop: f64,
    /// Vertical Dilution of Precision
    pub vdop: f64,
    /// Time Dilution of Precision in [s]
    pub tdop: f64,
    /// Modeled tropospheric delay [s]
    pub tropo: f64,
    /// Ionospheric delay [s] model or real measurement
    /// (either one)
    pub iono: PVTIonoDelay,
}

impl PVTSolution {
    /// Builds a new PVTSolution from
    /// "g": the navigation matrix
    /// "y": the navigation vector
    /// "tropo": modeled tropospheric delay
    /// "iono": modeled and/or measured ionospheric delay. Either one must exist.
    pub fn new(
        g: MatrixXx4<f64>,
        y: DVector<f64>,
        tropo: f64,
        iono: PVTIonoDelay,
    ) -> Result<Self, Error> {
        let g_prime = g.clone().transpose();

        let q = (g_prime.clone() * g.clone())
            .try_inverse()
            .ok_or(Error::MatrixInversionError)?;

        let x = q * g_prime.clone();
        let x = x * y;
        Ok(Self {
            p: Vector3D {
                x: x[0],
                y: x[1],
                z: x[2],
            },
            v: Vector3D::default(),
            dt: x[3] / SPEED_OF_LIGHT,
            hdop: (q[(0, 0)] + q[(1, 1)]).sqrt(),
            vdop: q[(2, 2)].sqrt(),
            tdop: q[(3, 3)].sqrt(),
            tropo,
            iono,
        })
    }
}
