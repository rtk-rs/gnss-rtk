use nalgebra::Vector4;

use nyx::cosmic::SPEED_OF_LIGHT_M_S;

use anise::{
    astro::PhysicsResult,
    math::{Vector3, Vector6},
    prelude::{Epoch, Frame, Unit},
};

use crate::prelude::{Duration, Orbit};

#[derive(Clone, Copy)]
pub struct State {
    /// [Epoch]
    pub t: Epoch,
    /// [Frame] we work with
    pub frame: Frame,
    pub first_update: bool,
    /// Clock state as [Duration]
    pub clock: Duration,
    /// Clock drift (s.s⁻¹)
    pub clock_drift_s_s: f64,
    /// ECEF position (m)
    pub pos_m: (f64, f64, f64),
    /// Geodeticy position (ddeg, ddeg, km above mean sea level)
    pub lat_long_alt_deg_deg_km: (f64, f64, f64),
    /// Velocity vector (ECEF m.s⁻¹)
    pub vel_m_s: (f64, f64, f64),
}

impl State {
    /// Create new [State] from ECEF coordinates.
    pub fn from_ecef_m(pos_m: Vector3, t: Epoch, frame: Frame) -> PhysicsResult<Self> {
        let pos_vel = Vector6::new(
            pos_m[0] / 1.0E3,
            pos_m[1] / 1.0E3,
            pos_m[2] / 1.0E3,
            0.0,
            0.0,
            0.0,
        );
        let orbit = Orbit::from_cartesian_pos_vel(pos_vel, t, frame);
        Self::from_orbit(&orbit, frame)
    }

    /// Create new [State] from [Orbit]al solution.
    pub fn from_orbit(orbit: &Orbit, frame: Frame) -> PhysicsResult<Self> {
        let pos_vel_m = orbit.to_cartesian_pos_vel() * 1.0E3;
        let latlongalt = orbit.latlongalt()?;
        Ok(Self {
            frame,
            t: orbit.epoch,
            first_update: true,
            clock_drift_s_s: 0.0_f64,
            clock: Default::default(),
            lat_long_alt_deg_deg_km: latlongalt,
            pos_m: (pos_vel_m[0], pos_vel_m[1], pos_vel_m[2]),
            vel_m_s: (pos_vel_m[3], pos_vel_m[4], pos_vel_m[5]),
        })
    }

    /// Converts [State] to [Orbit]
    pub fn to_orbit(&self) -> Orbit {
        let pos_vel_km = Vector6::new(
            self.pos_m.0 / 1.0E3,
            self.pos_m.1 / 1.0E3,
            self.pos_m.2 / 1.0E3,
            self.vel_m_s.0 / 1.0E3,
            self.vel_m_s.1 / 1.0E3,
            self.vel_m_s.2 / 1.0E3,
        );

        Orbit::from_cartesian_pos_vel(pos_vel_km, self.t, self.frame)
    }

    /// Update [State]
    pub fn temporal_update(&mut self, t: Epoch, dx: Vector4<f64>) -> PhysicsResult<()> {
        let new_pos_m = (
            self.pos_m.0 + dx[0],
            self.pos_m.1 + dx[1],
            self.pos_m.2 + dx[2],
        );

        let new_clock = dx[3] / SPEED_OF_LIGHT_M_S * Unit::Second;

        let dt_s = (t - self.t).to_seconds();

        if dt_s > 0.0 {
            self.vel_m_s = (
                (new_pos_m.0 - self.pos_m.0) / dt_s,
                (new_pos_m.1 - self.pos_m.1) / dt_s,
                (new_pos_m.2 - self.pos_m.2) / dt_s,
            );

            self.clock_drift_s_s = (new_clock - self.clock).to_seconds() / dt_s;
        }

        self.t = t;
        self.pos_m = new_pos_m;

        // update orbital attitude
        let new_orbit = self.to_orbit();
        self.lat_long_alt_deg_deg_km = new_orbit.latlongalt()?;

        self.clock = new_clock;
        self.first_update = false;

        Ok(())
    }
}
