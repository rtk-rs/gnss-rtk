//! PVT Solutions
use crate::prelude::{Filter, Vector3, SV};
use crate::solver::{FilterState, LSQState};
use crate::Error;
use std::collections::HashMap;

pub(crate) mod validator;

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

use nalgebra::base::{DMatrix, DVector, Matrix3, Matrix4, Matrix4x1, MatrixXx4};
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
    q: Matrix4<f64>,
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
        filter: Filter,
        p_state: Option<FilterState>,
    ) -> Result<(Self, Option<FilterState>), Error> {
        let (q, x, new_state) = match filter {
            Filter::None => {
                let g_prime = g.clone().transpose();
                let q = (g_prime.clone() * g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let p = (g_prime.clone() * w.clone() * g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let x = p * (g_prime.clone() * w.clone() * y);
                (q, x, None)
            },
            Filter::LSQ => {
                if let Some(FilterState::LSQState(p_state)) = p_state {
                    let p_1 = p_state
                        .p
                        .try_inverse()
                        .ok_or(Error::CovarMatrixInversionError)?;

                    let g_prime = g.clone().transpose();
                    let q = (g_prime.clone() * g.clone())
                        .try_inverse()
                        .ok_or(Error::MatrixInversionError)?;

                    let p = g_prime.clone() * w.clone() * g.clone();
                    let p = (p_1 + p)
                        .try_inverse()
                        .ok_or(Error::CovarMatrixInversionError)?;

                    let x = p * (p_1 * p_state.x + (g_prime.clone() * w.clone() * y));
                    (q, x, Some(FilterState::LSQState(LSQState { x, p })))
                } else {
                    let g_prime = g.clone().transpose();
                    let q = (g_prime.clone() * g.clone())
                        .try_inverse()
                        .ok_or(Error::MatrixInversionError)?;

                    let p = (g_prime.clone() * w.clone() * g.clone())
                        .try_inverse()
                        .ok_or(Error::MatrixInversionError)?;

                    let x = (p * g_prime.clone()) * w.clone() * y;
                    (q, x, Some(FilterState::LSQState(LSQState { x, p })))
                }
            },
            Filter::KF => {
                panic!("kalman filter not available yet");
                //if let Some(FilterState::KfState(p_state)) = p_state {
                //    let g_prime = g.clone().transpose();
                //    let q = (g_prime.clone() * g.clone())
                //        .try_inverse()
                //        .ok_or(Error::MatrixInversionError)?;

                //    //let p = (g_prime.clone() * w.clone() * g.clone())
                //    //    .try_inverse()
                //    //    .ok_or(Error::MatrixInversionError)?;
                //    //
                //    //let x = (p * g_prime.clone()) * w.clone() * y;
                //    //
                //    //(q, x, Some(FilterState::KfState(KfState {
                //    //    x,
                //    //    p,
                //    //})))
                //} else {
                //    return Err(Error::UninitializedKalmanFilter);
                //}
            },
        };

        let dt = x[3] / SPEED_OF_LIGHT;
        if dt.is_nan() {
            return Err(Error::TimeIsNan);
        }

        Ok((
            (Self {
                sv,
                pos: Vector3::new(x[0], x[1], x[2]),
                vel: Vector3::<f64>::default(),
                dt,
                q,
            }),
            new_state,
        ))
    }
    /// Returns list of Space Vehicles (SV) that help form this solution.
    pub fn sv(&self) -> Vec<SV> {
        self.sv.keys().copied().collect()
    }
    /// Returns Geometric Dilution of Precision of Self
    pub fn gdop(&self) -> f64 {
        (self.q[(0, 0)] + self.q[(1, 1)] + self.q[(2, 2)] + self.q[(3, 3)]).sqrt()
    }
    /// Returns Position Diultion of Precision of Self
    pub fn pdop(&self) -> f64 {
        (self.q[(0, 0)] + self.q[(1, 1)] + self.q[(2, 2)]).sqrt()
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
            self.q[(0, 0)],
            self.q[(0, 1)],
            self.q[(0, 2)],
            self.q[(1, 0)],
            self.q[(1, 1)],
            self.q[(1, 2)],
            self.q[(2, 0)],
            self.q[(2, 1)],
            self.q[(2, 2)],
        );
        r.clone().transpose() * q_3 * r.clone()
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
        self.q[(3, 3)].sqrt()
    }
}
