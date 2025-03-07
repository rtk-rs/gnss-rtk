//! PVT Solutions
use std::collections::HashMap;

use crate::prelude::{Carrier, Duration, Orbit, TimeScale, SV};

use super::SVInput;
use nalgebra::base::{Matrix3, Matrix4};

// pub(crate) mod validator;
// pub use validator::InvalidationCause;

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
    /// Receiver state, expressed as ECEF [Orbit]
    pub state: Orbit,
    /// Timescale
    pub timescale: TimeScale,
    /// Clock offset (s)
    pub clock_offset: Duration,
    /// Clock drift (s.s⁻¹)
    pub clock_drift_s_s: f64,
    /// Space Vehicles that helped form this solution
    /// and data associated to each individual SV
    pub sv: HashMap<SV, SVInput>,
    /// Geometric Dilution of Precision
    pub gdop: f64,
    /// Vertical Dilution of Precision
    pub vdop: f64,
    /// Horizontal Diultion of Precision
    pub hdop: f64,
    /// Temporal Dilution of Precision
    pub tdop: f64,
}
