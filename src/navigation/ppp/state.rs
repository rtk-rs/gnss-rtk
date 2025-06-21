use nalgebra::{
    //allocator::Allocator,
    DVector,
    // DefaultAllocator,
    DimName,
    OVector,
    U4,
};

use anise::{
    astro::PhysicsResult,
    math::{Vector3, Vector6},
    prelude::{Epoch, Frame},
};

use crate::{
    constants::SPEED_OF_LIGHT_M_S,
    navigation::{ambiguity::Ambiguity, Apriori, Navigation},
    prelude::Orbit,
};

#[derive(Debug, Clone, PartialEq)]
pub struct State {
    /// [Epoch]
    pub t: Epoch,

    /// Internal [OVector]
    x: OVector<f64, U4>,

    /// [Ambiguity] [DVector]
    ambiguities: DVector<Ambiguity>,

    /// Clock drift (s.s⁻¹)
    clock_drift_s_s: f64,

    /// Geodetic position (ddeg, ddeg, km above mean sea level)
    pub lat_long_alt_deg_deg_km: (f64, f64, f64),
}

impl Default for State {
    fn default() -> Self {
        Self {
            t: Default::default(),
            x: OVector::<f64, U4>::zeros(),
            clock_drift_s_s: Default::default(),
            ambiguities: DVector::<Ambiguity>::zeros(U4::USIZE),
            lat_long_alt_deg_deg_km: Default::default(),
        }
    }
}

impl std::fmt::Display for State {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let position_vel_m = self.position_velocity_ecef_m();
        let (offset, drift) = self.clock_profile_s();

        if self.ambiguities.is_empty() {
            write!(
                f,
                "{} dt={:.11E}s drift={:.11E}s/s {}",
                position_vel_m, offset, drift, self.ambiguities,
            )
        } else {
            write!(
                f,
                "{} dt={:.11E}s drift={:.11E}s/s",
                position_vel_m, offset, drift,
            )
        }
    }
}

impl State {
    /// Create new [State] initialized from [Apriori]
    pub fn from_apriori(apriori: &Apriori) -> PhysicsResult<Self> {
        let orbit = apriori.to_orbit();
        Self::from_orbit(&orbit)
    }

    /// Create a new [State] with updated [Epoch]
    pub fn with_epoch(&self, t: Epoch) -> Self {
        let mut s = self.clone();
        s.t = t;
        s
    }

    /// Create new [State] from [Orbit]al solution.
    pub fn from_orbit(orbit: &Orbit) -> PhysicsResult<Self> {
        let pos_vel_m = orbit.to_cartesian_pos_vel() * 1.0E3;
        let latlongalt = orbit.latlongalt()?;

        let mut x = OVector::<f64, U4>::zeros();

        for i in 0..U4::USIZE - 1 {
            x[i] = pos_vel_m[i];
        }

        Ok(Self {
            x,
            t: orbit.epoch,
            clock_drift_s_s: 0.0_f64,
            lat_long_alt_deg_deg_km: latlongalt,
            ambiguities: DVector::<Ambiguity>::zeros(U4::USIZE),
        })
    }

    /// Returns position in ECEF meters as [Vector3]
    pub fn position_ecef_m(&self) -> Vector3 {
        Vector3::new(self.x[0], self.x[1], self.x[2])
    }

    /// Returns position and velocity in ECEF meters as [Vector6]
    pub fn position_velocity_ecef_m(&self) -> Vector6 {
        Vector6::new(
            self.x[0], self.x[1], self.x[2], 0.0, // TODO
            0.0, // TODO
            0.0, // TODO
        )
    }

    /// Returns estimated clock (offset, drift) in seconds and s.s⁻¹.
    pub fn clock_profile_s(&self) -> (f64, f64) {
        (self.x[Navigation::clock_index()], self.clock_drift_s_s)
    }

    /// Converts [State] to [Orbit]
    pub fn to_orbit(&self, frame: Frame) -> Orbit {
        let pos_vel_km_s = self.position_velocity_ecef_m() / 1.0E3;
        Orbit::from_cartesian_pos_vel(pos_vel_km_s, self.t, frame)
    }

    ///  [State] correction with mutable access.
    pub fn correct_mut(
        &mut self,
        frame: Frame,
        pending_t: Epoch,
        dx: &DVector<f64>,
    ) -> PhysicsResult<()> {
        let nrows = dx.nrows();

        assert!(nrows >= U4::USIZE, "invalid state dim={}!", nrows);

        let dt = (pending_t - self.t).to_seconds();

        if dt > 0.0 {
            self.clock_drift_s_s = (dx[Navigation::clock_index()] / SPEED_OF_LIGHT_M_S
                - self.x[Navigation::clock_index()])
                / dt;
        }

        for i in 0..U4::USIZE {
            if i == Navigation::clock_index() {
                self.x[i] = dx[i] / SPEED_OF_LIGHT_M_S;
            } else {
                self.x[i] += dx[i];
            }
        }

        // update attitude
        let new_orbit = self.to_orbit(frame);
        self.lat_long_alt_deg_deg_km = new_orbit.latlongalt()?;

        self.t = pending_t;

        Ok(())
    }

    /// Temporal update
    pub fn postfit_update_mut(&mut self, frame: Frame, dx: DVector<f64>) -> PhysicsResult<()> {
        let nrows = std::cmp::min(dx.nrows(), self.x.nrows());

        for i in 0..nrows {
            if i != Navigation::clock_index() {
                self.x[i] = dx[i];
            }
        }

        let new_orbit = self.to_orbit(frame);
        self.lat_long_alt_deg_deg_km = new_orbit.latlongalt()?;

        Ok(())
    }

    // /// Compute residual between this [State] and other [State]
    // pub(crate) fn residual(&self, rhs: &State<D>) -> OVector<f64, D> {
    //     self.x - rhs.x
    // }
}

#[cfg(test)]
mod test {

    use crate::{
        navigation::{Apriori, Navigation, State},
        prelude::{Duration, Frame, Orbit, SPEED_OF_LIGHT_M_S},
        tests::REFERENCE_COORDS_ECEF_M,
    };

    use anise::math::Vector3;
    use nalgebra::{DVector, DimName, U4};

    use rstest::*;

    #[fixture]
    fn build_earth_frame() -> Frame {
        use crate::tests::earth_frame;
        earth_frame()
    }

    #[fixture]
    fn build_reference_apriori() -> Apriori {
        use crate::tests::reference_apriori_at_ref_epoch;
        reference_apriori_at_ref_epoch()
    }

    #[fixture]
    fn build_reference_orbit() -> Orbit {
        use crate::tests::{earth_frame, reference_orbit_at_ref_epoch};
        reference_orbit_at_ref_epoch(earth_frame())
    }

    #[test]
    fn state_u4_from_reference_apriori_position() {
        let earth_frame = build_earth_frame();
        let apriori = build_reference_apriori();

        let initial_state = State::from_apriori(&apriori).unwrap_or_else(|e| {
            panic!(
                "Failed to build initial state from reference apriori: {}",
                e
            )
        });

        let position_ecef_m = initial_state.position_ecef_m();

        assert_eq!(
            position_ecef_m,
            Vector3::new(
                REFERENCE_COORDS_ECEF_M.0,
                REFERENCE_COORDS_ECEF_M.1,
                REFERENCE_COORDS_ECEF_M.2
            )
        );

        let position_velocity_m_s = initial_state.position_velocity_ecef_m();

        assert_eq!(
            (
                position_velocity_m_s[0],
                position_velocity_m_s[1],
                position_velocity_m_s[2]
            ),
            (
                REFERENCE_COORDS_ECEF_M.0,
                REFERENCE_COORDS_ECEF_M.1,
                REFERENCE_COORDS_ECEF_M.2
            ),
            "initial position error!"
        );

        assert_eq!(
            (
                position_velocity_m_s[3],
                position_velocity_m_s[4],
                position_velocity_m_s[5],
            ),
            (0.0, 0.0, 0.0),
            "dynamics not modeled in U4!",
        );

        assert_eq!(
            initial_state.clock_profile_s(),
            (0.0, 0.0),
            "clock state should not be initialized!"
        );

        let mut state = initial_state.clone();

        let null_dx = DVector::zeros(U4::USIZE);
        let new_t = state.t;

        state
            .correct_mut(earth_frame, new_t, &null_dx)
            .unwrap_or_else(|e| panic!("Failed to apply null state correction! {}", e));

        assert_eq!(
            initial_state, state,
            "null correction should preserve current state!"
        );

        let mut state = initial_state.clone();

        let mut dx = DVector::zeros(U4::USIZE);

        dx[0] = 1.0;
        dx[1] = 2.0;
        dx[2] = 3.0;
        dx[3] = 4.0;

        state
            .correct_mut(earth_frame, state.t, &dx)
            .unwrap_or_else(|e| panic!("Failed to apply null state correction! {}", e));

        assert_eq!(state.t, initial_state.t, "Epoch should have been preserved");

        let position_ecef_m = state.position_ecef_m();

        assert_eq!(
            position_ecef_m,
            Vector3::new(
                REFERENCE_COORDS_ECEF_M.0 + 1.0,
                REFERENCE_COORDS_ECEF_M.1 + 2.0,
                REFERENCE_COORDS_ECEF_M.2 + 3.0,
            ),
            "invalid spatial state correction",
        );

        let (clock_offset_s, clock_drift_s) = state.clock_profile_s();

        assert_eq!(clock_offset_s, 4.0 / SPEED_OF_LIGHT_M_S);
        assert_eq!(clock_drift_s, 0.0, "clock drift should have been preserved");

        let mut state = initial_state.clone();

        let mut dx = DVector::zeros(U4::USIZE);

        dx[0] = 1.0;
        dx[1] = 2.0;
        dx[2] = 3.0;
        dx[Navigation::clock_index()] = 4.0;

        let new_t = state.t + Duration::from_seconds(30.0);

        state
            .correct_mut(earth_frame, new_t, &dx)
            .unwrap_or_else(|e| panic!("Failed to apply null state correction! {}", e));

        assert!(state.t > initial_state.t, "Epoch should have been updated!");
        assert_eq!(
            (state.t - initial_state.t),
            Duration::from_seconds(30.0),
            "Invalid sampling update"
        );

        let position_ecef_m = state.position_ecef_m();

        assert_eq!(
            position_ecef_m,
            Vector3::new(
                REFERENCE_COORDS_ECEF_M.0 + 1.0,
                REFERENCE_COORDS_ECEF_M.1 + 2.0,
                REFERENCE_COORDS_ECEF_M.2 + 3.0,
            ),
            "invalid spatial state correction",
        );

        let (clock_offset_s, clock_drift_s) = state.clock_profile_s();

        assert_eq!(clock_offset_s, 4.0 / SPEED_OF_LIGHT_M_S);

        assert_eq!(
            clock_drift_s,
            4.0 / SPEED_OF_LIGHT_M_S / Duration::from_seconds(30.0).to_seconds(),
            "invalid clock drift update!"
        );
    }

    #[test]
    fn state_u4_from_reference_orbital_state() {
        let orbit = build_reference_orbit();

        let state = State::from_orbit(&orbit).unwrap_or_else(|e| {
            panic!("Failed to build initial state from reference orbit: {}", e)
        });

        let position_ecef_m = state.position_ecef_m();

        assert_eq!(
            position_ecef_m,
            Vector3::new(
                REFERENCE_COORDS_ECEF_M.0,
                REFERENCE_COORDS_ECEF_M.1,
                REFERENCE_COORDS_ECEF_M.2
            )
        );

        let position_velocity_m_s = state.position_velocity_ecef_m();

        assert_eq!(
            (
                position_velocity_m_s[0],
                position_velocity_m_s[1],
                position_velocity_m_s[2]
            ),
            (
                REFERENCE_COORDS_ECEF_M.0,
                REFERENCE_COORDS_ECEF_M.1,
                REFERENCE_COORDS_ECEF_M.2
            ),
            "initial position error!"
        );

        assert_eq!(
            (
                position_velocity_m_s[3],
                position_velocity_m_s[4],
                position_velocity_m_s[5],
            ),
            (0.0, 0.0, 0.0),
            "dynamics not modeled in U4!",
        );

        assert_eq!(
            state.clock_profile_s(),
            (0.0, 0.0),
            "clock state should not be initialized!"
        );
    }
}
