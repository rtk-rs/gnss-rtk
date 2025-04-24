use anise::{
    math::{Vector3, Vector6},
    prelude::{Epoch, Frame},
};

use crate::{navigation::State, prelude::Orbit};

#[derive(Clone, Copy)]
pub struct Apriori {
    /// [Epoch] of resolution
    pub t: Epoch,
    /// [Frame] we work with
    pub frame: Frame,
    /// ECEF position (m)
    pub pos_m: (f64, f64, f64),
}

impl Apriori {
    /// Create new [Apriori] from past [State] and new [Epoch].
    pub fn from_state(t: Epoch, frame: Frame, state: &State) -> Self {
        let orbital = state.to_orbit(frame);

        let mut apriori = Self::from_orbit(&orbital, frame);
        apriori.t = t;

        apriori
    }

    /// Create new [Apriori] from ECEF coordinates.
    pub fn from_ecef_m(pos_m: Vector3, t: Epoch, frame: Frame) -> Self {
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

    /// Create new [Apriori] from [Orbit]al solution.
    pub fn from_orbit(orbit: &Orbit, frame: Frame) -> Self {
        let pos_vel_m = orbit.to_cartesian_pos_vel() * 1.0E3;

        Self {
            frame,
            t: orbit.epoch,
            pos_m: (pos_vel_m[0], pos_vel_m[1], pos_vel_m[2]),
            // vel_m_s: (pos_vel_m[3], pos_vel_m[4], pos_vel_m[5]),
        }
    }

    /// Converts [Apriori] to [Orbit]
    pub fn to_orbit(&self) -> Orbit {
        Orbit::from_position(
            self.pos_m.0 / 1.0E3,
            self.pos_m.1 / 1.0E3,
            self.pos_m.2 / 1.0E3,
            self.t,
            self.frame,
        )
    }
}
