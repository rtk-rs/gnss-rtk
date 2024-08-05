use anise::prelude::{Frame, Orbit};

use crate::prelude::{Epoch, Vector3, SV};
use map_3d::ecef2geodetic;
use std::f64::consts::PI;

/// [OrbitalState] must be provide of each candidate.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct OrbitalState {
    /// Elevation compared to reference position and horizon in [°]
    pub elevation: f64,
    /// Azimuth compared to reference position and magnetic North in [°]
    pub azimuth: f64,
    /// APC Position vector in [m] ECEF
    pub position: Vector3<f64>,
    // Velocity vector in [m/s] ECEF that we calculated ourselves
    pub(crate) velocity: Option<Vector3<f64>>,
}

/// OrbitalStateProvider must be implemented
/// and provide SV state at specified `t` for the solving process can proceed.
pub trait OrbitalStateProvider {
    /// Provide [OrbitalState] at requested `t` so we can proceed.
    /// In case interpolation is used, we propose an interpolation order,
    /// that would fit current setup, which you can choose to ignore.
    /// If you can't provide (missing data?): simply return None.
    /// If None is returned for too long, this [Epoch] will eventually get dropped out
    /// and we will proceed to the next.
    fn next_at(&mut self, t: Epoch, sv: SV, order: usize) -> Option<OrbitalState>;
}

impl OrbitalState {
    /// Creates [Self] from Antenna Phase Center (APC) position coordinates,
    /// expressed in ECEF [m].
    pub fn from_position(position: (f64, f64, f64)) -> Self {
        Self {
            velocity: None,
            azimuth: 0.0_f64,
            elevation: 0.0_f64,
            position: Vector3::<f64>::new(position.0, position.1, position.2),
        }
    }
    /// Complete [Self] definition with both Elevation and Azimuth angles
    pub(crate) fn with_elevation_azimuth(&self, position: (f64, f64, f64)) -> Self {
        let (x, y, z) = position;
        let (lat_rad, lon_rad, _) = ecef2geodetic(x, y, z, map_3d::Ellipsoid::WGS84);

        let (sv_x, sv_y, sv_z) = (self.position[0], self.position[1], self.position[2]);
        let a_i = (sv_x - x, sv_y - y, sv_z - z);
        let norm = (a_i.0.powf(2.0) + a_i.1.powf(2.0) + a_i.2.powf(2.0)).sqrt();
        let a_i = (a_i.0 / norm, a_i.1 / norm, a_i.2 / norm);

        let ecef_to_ven = (
            (
                lat_rad.cos() * lon_rad.cos(),
                lat_rad.cos() * lon_rad.sin(),
                lat_rad.sin(),
            ),
            (-lon_rad.sin(), lon_rad.cos(), 0.0_f64),
            (
                -lat_rad.sin() * lon_rad.cos(),
                -lat_rad.sin() * lon_rad.sin(),
                lat_rad.cos(),
            ),
        );
        // ECEF to VEN transform
        let ven = (
            (ecef_to_ven.0 .0 * a_i.0 + ecef_to_ven.0 .1 * a_i.1 + ecef_to_ven.0 .2 * a_i.2),
            (ecef_to_ven.1 .0 * a_i.0 + ecef_to_ven.1 .1 * a_i.1 + ecef_to_ven.1 .2 * a_i.2),
            (ecef_to_ven.2 .0 * a_i.0 + ecef_to_ven.2 .1 * a_i.1 + ecef_to_ven.2 .2 * a_i.2),
        );
        let elevation = (PI / 2.0 - ven.0.acos()).to_degrees();
        let mut azimuth = (ven.1.atan2(ven.2)).to_degrees();
        if azimuth < 0.0 {
            azimuth += 360.0;
        }
        Self {
            azimuth,
            elevation,
            position: self.position,
            velocity: self.velocity,
        }
    }
    pub(crate) fn velocity(&self) -> Option<Vector3<f64>> {
        self.velocity
    }
    pub(crate) fn orbit(&self, dt: Epoch, frame: Frame) -> Orbit {
        let p = self.position;
        let v = self.velocity().unwrap_or_default();
        Orbit::cartesian(
            p[0] / 1.0E3,
            p[1] / 1.0E3,
            p[2] / 1.0E3,
            v[0] / 1.0E3,
            v[1] / 1.0E3,
            v[2] / 1.0E3,
            dt,
            frame,
        )
    }
    #[cfg(test)]
    fn set_elevation(&mut self, elev: f64) {
        self.elevation = elev;
    }
}
