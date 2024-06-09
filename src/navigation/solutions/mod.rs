//! PVT Solutions
use std::collections::HashMap;

use crate::prelude::{Ambiguities, Carrier, Duration, TimeScale, Vector3, SV};

use super::SVInput;
use nalgebra::base::{Matrix3, Matrix4};

pub(crate) mod validator;
pub use validator::InvalidationCause;

/// InstrumentBias, estimated per SV and signal for each solution (ie., in Time),
/// when navigation is based on Phase Range observations.
pub type InstrumentBias = HashMap<(SV, Carrier), f64>;

#[cfg(feature = "serde")]
use serde::Deserialize;

#[derive(Debug, Copy, Clone, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub enum PVTSolutionType {
    /// Default, complete solution with Position,
    /// Velocity and Time components. Requires either
    /// 4 vehicles in sight, or 3 if you're working in fixed altitude
    /// (provided ahead of time).
    #[default]
    PositionVelocityTime,
    /// Resolve Time component only. Only requires 1 vehicle in sight.
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

/// PVT Solution, always expressed as the correction to apply
/// to an Apriori / static position.
#[derive(Debug, Clone)]
// #[cfg_attr(feature = "serde", derive(Serialize))]
pub struct PVTSolution {
    /// Position error (in [m] ECEF)
    pub position: Vector3<f64>,
    /// Absolute Velocity (in [m/s] ECEF).
    pub velocity: Vector3<f64>,
    /// Timescale
    pub timescale: TimeScale,
    /// Offset to timescale
    pub dt: Duration,
    /// Space Vehicles that helped form this solution
    /// and data associated to each individual SV
    pub sv: HashMap<SV, SVInput>,
    /// Geometric Dilution of Precision
    pub gdop: f64,
    /// Time Dilution of Precision
    pub tdop: f64,
    /// Position Dilution of Precision
    pub pdop: f64,
    /// Resolved ambiguities (at this point and time), per SV and signal.
    /// Ambiguities are null if navigation does not use them (see [Method]).
    /// This is useful for advanced applications that want or need this level of detail.
    pub ambiguities: Ambiguities,
    // // Instrument bias, determined from Phase Range based Navigation (see [Method])
    // // and internal signal ambiguity solving. If Navigation [Method] is not based on Phase Range,
    // // the bias cannot be estimated (null). This is useful for advanced applications that want or need this level of detail.
    // pub bias: InstrumentBias,
    // Q
    pub(crate) q: Matrix4<f64>,
}

impl PVTSolution {
    /// Returns list of Space Vehicles (SV) that help form this solution.
    pub fn sv(&self) -> Vec<SV> {
        self.sv.keys().copied().collect()
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
        r.clone().transpose() * q_3 * r
    }
    pub fn hdop(&self, lat: f64, lon: f64) -> f64 {
        let q = self.q_enu(lat, lon);
        (q[(0, 0)] + q[(1, 1)]).sqrt()
    }
    pub fn vdop(&self, lat: f64, lon: f64) -> f64 {
        self.q_enu(lat, lon)[(2, 2)].sqrt()
    }
}
