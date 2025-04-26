//! PVT Solution
use crate::{
    navigation::{DilutionOfPrecision, SVContribution, State},
    prelude::{Epoch, TimeScale},
};

use nalgebra::{allocator::Allocator, DefaultAllocator, DimName};

#[cfg(feature = "serde")]
use serde::Serialize;

/// PVT Solution, always expressed as the correction to apply
/// to an Apriori / static position.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct PVTSolution {
    /// [Epoch] of succesful solving
    pub epoch: Epoch,
    /// Position in meters ECEF.
    pub pos_m: (f64, f64, f64),
    /// Velocity in m.s⁻¹ ECEF
    pub vel_m_s: (f64, f64, f64),
    /// Latitude, longitude and altitude above mean sea level,
    /// in degrees and meters.
    pub lat_long_alt_deg_deg_m: (f64, f64, f64),
    /// [TimeScale] of clock_offset [Duration] expression.
    pub timescale: TimeScale,
    /// Clock offset (s)
    pub clock_offset_s: f64,
    /// Clock drift (s.s⁻¹) within [TimeScale]
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
    pub(crate) fn new<D: DimName>(
        epoch: Epoch,
        state: &State<D>,
        dop: &DilutionOfPrecision,
        contributions: &[SVContribution],
    ) -> Self
    where
        DefaultAllocator: Allocator<D> + Allocator<D, D>,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    {
        let pos_vel_ecef_m = state.position_velocity_ecef_m();
        let (clock_offset_s, clock_drift_s_s) = state.clock_profile_s();

        Self {
            epoch,
            gdop: dop.gdop,
            tdop: dop.tdop,
            vdop: dop.vdop,
            hdop: dop.hdop,
            sv: contributions.to_vec(),
            timescale: state.t.time_scale,
            clock_offset_s: clock_offset_s,
            clock_drift_s_s: clock_drift_s_s,
            lat_long_alt_deg_deg_m: (
                state.lat_long_alt_deg_deg_km.0,
                state.lat_long_alt_deg_deg_km.1,
                state.lat_long_alt_deg_deg_km.2 * 1.0E3,
            ),
            pos_m: (pos_vel_ecef_m[0], pos_vel_ecef_m[1], pos_vel_ecef_m[2]),
            vel_m_s: (pos_vel_ecef_m[3], pos_vel_ecef_m[4], pos_vel_ecef_m[5]),
        }
    }
}
