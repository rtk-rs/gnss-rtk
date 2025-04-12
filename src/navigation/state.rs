use nalgebra::Vector4;

use nyx::cosmic::SPEED_OF_LIGHT_M_S;

use anise::{
    astro::PhysicsResult,
    math::{Vector3, Vector6},
    prelude::{Epoch, Frame, Unit},
};

use log::debug;

use crate::{
    navigation::Apriori,
    prelude::{Duration, Orbit},
};

pub struct Residual {
    pub err_dt: Duration,
    pub err_m: (f64, f64, f64),
}

impl std::fmt::Display for Residual {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "err x={}m, y={}m z={}m dt={}",
            self.err_m.0, self.err_m.1, self.err_m.2, self.err_dt
        )
    }
}

#[derive(Clone, Copy)]
pub struct State {
    /// [Epoch] of resolution
    pub t: Epoch,
    /// [Frame] we work with
    pub frame: Frame,
    /// Clock dt as [Duration]
    pub clock_dt: Duration,
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
    /// Create new [State] initialized from [Apriori]
    pub fn from_apriori(apriori: &Apriori) -> PhysicsResult<Self> {
        let orbit = apriori.to_orbit();
        Self::from_orbit(&orbit, apriori.frame)
    }

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

    /// Create new [State] from ECEF pos+vel
    pub fn from_pos_vel_ecef_m(pos_vel_m: Vector6, t: Epoch, frame: Frame) -> PhysicsResult<Self> {
        let pos_vel_km = pos_vel_m / 1.0E3;
        let orbit = Orbit::from_cartesian_pos_vel(pos_vel_km, t, frame);
        Self::from_orbit(&orbit, frame)
    }

    /// Create new [State] from [Orbit]al solution.
    pub fn from_orbit(orbit: &Orbit, frame: Frame) -> PhysicsResult<Self> {
        let pos_vel_m = orbit.to_cartesian_pos_vel() * 1.0E3;
        let latlongalt = orbit.latlongalt()?;
        Ok(Self {
            frame,
            t: orbit.epoch,
            clock_drift_s_s: 0.0_f64,
            clock_dt: Default::default(),
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

    /// Derivative mutable update
    pub fn velocity_update(
        &mut self,
        past_t: Epoch,
        past_pos_m: (f64, f64, f64),
        past_dt: Duration,
    ) {
        let dt_s = (self.t - past_t).to_seconds();

        self.vel_m_s = (
            (self.pos_m.0 - past_pos_m.0) / dt_s,
            (self.pos_m.1 - past_pos_m.1) / dt_s,
            (self.pos_m.2 - past_pos_m.2) / dt_s,
        );

        self.clock_drift_s_s = (self.clock_dt - past_dt).to_seconds() / dt_s;
    }

    /// Temporal mutable update
    pub fn temporal_update(&mut self, t: Epoch, dx: Vector4<f64>) -> PhysicsResult<()> {
        let new_pos_m = (
            self.pos_m.0 + dx[0],
            self.pos_m.1 + dx[1],
            self.pos_m.2 + dx[2],
        );

        let new_clock_dt = dx[3] / SPEED_OF_LIGHT_M_S * Unit::Second;

        let dt_s = (t - self.t).to_seconds();

        if dt_s > 0.0 {
            self.vel_m_s = (
                (new_pos_m.0 - self.pos_m.0) / dt_s,
                (new_pos_m.1 - self.pos_m.1) / dt_s,
                (new_pos_m.2 - self.pos_m.2) / dt_s,
            );

            self.clock_drift_s_s = (new_clock_dt - self.clock_dt).to_seconds() / dt_s;
        }

        self.t = t;
        self.pos_m = new_pos_m;

        // update attitude
        let new_orbit = self.to_orbit();
        self.lat_long_alt_deg_deg_km = new_orbit.latlongalt()?;

        self.clock_dt = new_clock_dt;

        Ok(())
    }

    /// Temporal postfit update
    pub fn temporal_postfit_update(&mut self, dx: Vector6) -> PhysicsResult<()> {
        self.pos_m = (dx[0], dx[1], dx[2]);
        self.vel_m_s = (dx[3], dx[4], dx[5]);

        let new_orbit = self.to_orbit();
        self.lat_long_alt_deg_deg_km = new_orbit.latlongalt()?;

        Ok(())
    }

    /// Compute residual between this [State] and rhs
    pub fn residual(&self, rhs: &State) -> Residual {
        let err_dt = self.clock_dt - rhs.clock_dt;

        let err_m = (
            self.pos_m.0 - rhs.pos_m.0,
            self.pos_m.1 - rhs.pos_m.1,
            self.pos_m.2 - rhs.pos_m.2,
        );

        Residual { err_m, err_dt }
    }
}
