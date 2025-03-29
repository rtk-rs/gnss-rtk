//! PVT Solution
use crate::{
    navigation::{DilutionOfPrecision, SVContribution, State},
    prelude::{Duration, TimeScale},
};

#[cfg(feature = "serde")]
use serde::Serialize;

/// PVT Solution, always expressed as the correction to apply
/// to an Apriori / static position.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct PVTSolution {
    /// Position in meters ECEF.
    pub pos_m: (f64, f64, f64),
    /// Velocity in m.s⁻¹ ECEF
    pub vel_m_s: (f64, f64, f64),
    /// Latitude, longitude and altitude above mean sea level,
    /// in degrees and meters.
    pub lat_long_alt_deg_deg_m: (f64, f64, f64),
    /// Timescale
    pub timescale: TimeScale,
    /// Clock offset (s)
    pub clock_offset: Duration,
    /// Clock drift (s.s⁻¹)
    pub clock_drift_s_s: f64,
    /// Space Vehicles that helped form this solution
    /// and data associated to each individual SV
    pub sv: Vec<SVContribution>,
    /// Geometric Dilution of Precision
    pub gdop: f64,
    /// Vertical Dilution of Precision
    pub vdop: f64,
    /// Horizontal Diultion of Precision
    pub hdop: f64,
    /// Temporal Dilution of Precision
    pub tdop: f64,
}

impl PVTSolution {
    pub(crate) fn new(
        state: &State,
        dop: &DilutionOfPrecision,
        contributions: &[SVContribution],
    ) -> Self {
        Self {
            gdop: dop.gdop,
            tdop: dop.tdop,
            vdop: dop.vdop,
            hdop: dop.hdop,
            pos_m: state.pos_m,
            vel_m_s: state.vel_m_s,
            sv: contributions.to_vec(),
            timescale: state.t.time_scale,
            clock_offset: state.clock_dt,
            clock_drift_s_s: state.clock_drift_s_s,
            lat_long_alt_deg_deg_m: (
                state.lat_long_alt_deg_deg_km.0,
                state.lat_long_alt_deg_deg_km.1,
                state.lat_long_alt_deg_deg_km.2 * 1.0E3,
            ),
        }
    }
}
