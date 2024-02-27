//! PVT Solutions
use super::Input as NavInput;
use crate::solver::{FilterState, LSQState};
use crate::prelude::{FilterTypeEnum, Vector3, SV};
use crate::Error;
use std::collections::HashMap;

pub mod validator;

#[derive(Debug, Copy, Clone, Default)]
pub enum PVTSolutionType {
    /// Default, complete solution with Position,
    /// Velocity and Time components. Requires either
    /// 4 SV or 3 SV when working with fixed altitude (provided ahead of time).
    #[default]
    PositionVelocityTime,
    /// Resolve Time component only. Requires 1 SV to resolve.
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

use nyx::cosmic::SPEED_OF_LIGHT;
use nalgebra::base::{DMatrix, DVector, Matrix3, Matrix4, MatrixXx4};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Modeled or measured bias
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PVTBias {
    /// Measured delay expressed as meters of delay
    pub measured: Option<f64>,
    /// Modeled delay expressed as meters of delay
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
    /// Measured or modeled Tropospheric Delay 
    pub tropo_bias: PVTBias,
    /// Measured or modeled Ionospheric Delay 
    pub iono_bias: PVTBias,
}

/// PVT Solution, always expressed as the correction to apply
/// to an Apriori / static position.
#[derive(Debug, Clone, Default)]
// #[cfg_attr(feature = "serde", derive(Serialize))]
pub struct PVTSolution {
    /// Position errors (in [m] ECEF)
    pub pos: Vector3<f64>,
    /// Absolute Velocity (in [m/s] ECEF).
    pub vel: Vector3<f64>,
    /// Time correction in [s]
    pub dt: f64,
    /// Space Vehicles that helped form this solution
    /// and data associated to each individual SV
    pub sv: HashMap<SV, PVTSVData>,
    // Q matrix
    q_mat: Matrix4<f64>,
}

impl PVTSolution {
    /// Builds a new PVTSolution from Navigation Input
    /// using attached navigation filter and SV data.
    pub fn new(
        nav: NavInput,
        f: &mut NavFilter,
        sv: HashMap<SV, PVTSVData>,
        p_state: Option<FilterState>,
    ) -> Result<Self, Error> {
        let nav = f.resolve(nav)?;
        
        let dt = nav.r[3] / SPEED_OF_LIGHT;
        if dt.is_nan() {
            return Err(Error::TimeIsNan);
        }

        Ok((
            (Self {
                sv,
                pos: Vector3::new(nav.r[0], nav.r[1], nav.r[2]),
                vel: Vector3::<f64>::default(),
                dt,
                q_mat: nav.q.clone(),
            }),
        ))
    }
    /// Returns list of Space Vehicles (SV) that help form this solution.
    pub fn sv(&self) -> Vec<SV> {
        self.sv
            .keys()
            .copied()
            .collect()
    }
    /// Returns Geometric Dilution of Precision of Self
    pub fn gdop(&self) -> f64 {
        (self.q_mat[(0, 0)] + self.q_mat[(1, 1)] + self.q_mat[(2, 2)] + self.q_mat[(3, 3)]).sqrt()
    }
    /// Returns Position Diultion of Precision of Self
    pub fn pdop(&self) -> f64 {
        (self.q_mat[(0, 0)] + self.q_mat[(1, 1)] + self.q_mat[(2, 2)]).sqrt()
    }
    fn q_enu(&self, lat: f64, lon: f64) -> Matrix3<f64> {
        let r = Matrix3::<f64>::new(
            -lon.sin(),
            -lon.cos() * lat.sin(),
            lat.cos() * lon.cos(),
            lon.cos(),
            -lat.sin() * lon.sin(),
            lat.cos() * lon.sin(),
            0.0_f64,
            lat.cos(),
            lon.sin(),
        );
        let q_3 = Matrix3::<f64>::new(
            self.q_mat[(0, 0)],
            self.q_mat[(0, 1)],
            self.q_mat[(0, 2)],
            self.q_mat[(1, 0)],
            self.q_mat[(1, 1)],
            self.q_mat[(1, 2)],
            self.q_mat[(2, 0)],
            self.q_mat[(2, 1)],
            self.q_mat[(2, 2)],
        );
        r.clone().transpose() * q_3 * r
    }
    /// Returns Horizontal Dilution of Precision of Self
    pub fn hdop(&self, lat: f64, lon: f64) -> f64 {
        let q_enu = self.q_enu(lat, lon);
        (q_enu[(0, 0)] + q_enu[(1, 1)]).sqrt()
    }
    /// Returns Vertical Dilution of Precision of Self
    pub fn vdop(&self, lat: f64, lon: f64) -> f64 {
        let q_enu = self.q_enu(lat, lon);
        q_enu[(2, 2)].sqrt()
    }
    /// Returns Time Dilution of Precision of Self
    pub fn tdop(&self) -> f64 {
        self.q_mat[(3, 3)].sqrt()
    }
}
