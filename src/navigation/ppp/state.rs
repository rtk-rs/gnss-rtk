use nalgebra::{allocator::Allocator, DVector, DefaultAllocator, DimName, OVector, U4, U8};

use anise::{
    astro::PhysicsResult,
    math::{Vector3, Vector6},
    prelude::{Epoch, Frame},
};

use crate::{
    constants::SPEED_OF_LIGHT_M_S,
    navigation::{Navigation, State},
    prelude::Orbit,
};

#[derive(Debug, Clone, PartialEq)]
pub struct PPPState {
    /// [Epoch] of resolution
    pub t: Epoch,

    /// Internal [DVector]
    x: DVector<f64>,

    /// Clock drift (s.s⁻¹)
    clock_drift_s_s: f64,

    /// Geodeticy position (ddeg, ddeg, km above mean sea level)
    pub lat_long_alt_deg_deg_km: (f64, f64, f64),
}

impl Default for PPPState {
    fn default() -> Self {
        Self {
            t: Default::default(),
            x: DVector::<f64>::zeros(U8::USIZE),
            clock_drift_s_s: Default::default(),
            lat_long_alt_deg_deg_km: Default::default(),
        }
    }
}

impl std::fmt::Display for PPPState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let position_vel_m = self.position_velocity_ecef_m();
        let (offset, drift) = self.clock_profile_s();

        let x_amb = self.x.view_range(Navigation::<U4>::clock_index() +1.., 0);

        write!(
            f,
            "{} x_amb={}, dt={:.11E}s drift={:.11E}s/s",
            position_vel_m, x_amb, offset, drift,
        )
    }
}

impl PPPState {
    pub fn from_initial_state<D: DimName>(initial_state: &State<D>) -> Self
    where
        DefaultAllocator: Allocator<D> + Allocator<D, D>,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
    {
        let mut x = DVector::<f64>::zeros(D::USIZE + U4::USIZE);

        for i in 0..D::USIZE {
            x[i] = initial_state.x[i];
        }

        Self {
            x,
            t: initial_state.t,
            clock_drift_s_s: initial_state.clock_drift_s_s,
            lat_long_alt_deg_deg_km: initial_state.lat_long_alt_deg_deg_km,
        }
    }

    pub fn to_initial_state<D: DimName>(&self) -> State<D>
    where
        DefaultAllocator: Allocator<D> + Allocator<D, D>,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
    {
        let mut x = OVector::<f64, D>::zeros();

        for i in 0..D::USIZE {
            x[i] = self.x[i];
        }

        State {
            x,
            t: self.t,
            clock_drift_s_s: self.clock_drift_s_s,
            lat_long_alt_deg_deg_km: self.lat_long_alt_deg_deg_km,
        }
    }

    pub fn clock_profile_s(&self) -> (f64, f64) {
        (self.x[Navigation::<U4>::clock_index()], 0.0)
    }

    /// Returns position in ECEF meters as [Vector3]
    pub fn position_ecef_m(&self) -> Vector3 {
        Vector3::new(self.x[0], self.x[1], self.x[2])
    }

    pub fn position_velocity_ecef_m(&self) -> Vector6 {
        Vector6::new(self.x[0], self.x[1], self.x[2], 0.0, 0.0, 0.0)
    }

    /// Converts [PPPState] to [Orbit]
    pub fn to_orbit(&self, frame: Frame) -> Orbit {
        let pos_vel_km_s = self.position_velocity_ecef_m() / 1.0E3;
        Orbit::from_cartesian_pos_vel(pos_vel_km_s, self.t, frame)
    }

    ///  [PPPState] correction with mutable access.
    pub fn correct_mut(
        &mut self,
        frame: Frame,
        pending_t: Epoch,
        dx: &DVector<f64>,
        ndf: usize,
    ) -> PhysicsResult<()> {
        assert!(ndf >= U8::USIZE, "x vector does not look right");

        let size = self.x.nrows();

        if size != ndf {
            self.x.resize_vertically_mut(ndf, 0.0);
        }

        let dt = (pending_t - self.t).to_seconds();

        if dt > 0.0 {
            self.clock_drift_s_s = (dx[Navigation::<U4>::clock_index()] / SPEED_OF_LIGHT_M_S
                - self.x[Navigation::<U4>::clock_index()])
                / dt;
        }

        // apply correction
        for i in 0..ndf {
            if i < Navigation::<U4>::clock_index() {
                self.x[i] += dx[i];
            } else if i == Navigation::<U4>::clock_index() {
                self.x[i] = dx[i] / SPEED_OF_LIGHT_M_S;
            } else {
                self.x[i] = dx[i];
            }
        }

        // update attitude
        let new_orbit = self.to_orbit(frame);
        self.lat_long_alt_deg_deg_km = new_orbit.latlongalt()?;

        self.t = pending_t;

        Ok(())
    }
}

#[cfg(test)]
mod test {

    use super::PPPState;

    use crate::{
        navigation::{Apriori, State},
        prelude::{Frame, Orbit, SPEED_OF_LIGHT_M_S},
        tests::REFERENCE_COORDS_ECEF_M,
    };

    use anise::math::{Vector3, Vector6};
    use nalgebra::{DVector, DimName, U4, U8};

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
        let earth_frame = earth_frame();
        reference_orbit_at_ref_epoch(earth_frame)
    }

    #[test]
    fn ppp_state_from_u4() {
        let earth_frame = build_earth_frame();
        let apriori = build_reference_apriori();

        let initial_state = State::<U4>::from_apriori(&apriori).unwrap_or_else(|e| {
            panic!("Failed to build initial state from reference orbit: {}", e)
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

        let mut ppp_state = PPPState::from_initial_state(&initial_state);

        assert_eq!(
            ppp_state.position_ecef_m(),
            Vector3::new(
                REFERENCE_COORDS_ECEF_M.0,
                REFERENCE_COORDS_ECEF_M.1,
                REFERENCE_COORDS_ECEF_M.2,
            )
        );

        assert_eq!(
            ppp_state.position_velocity_ecef_m(),
            Vector6::new(
                REFERENCE_COORDS_ECEF_M.0,
                REFERENCE_COORDS_ECEF_M.1,
                REFERENCE_COORDS_ECEF_M.2,
                0.0,
                0.0,
                0.0,
            )
        );

        let null_dx = DVector::zeros(U8::USIZE);

        let new_t = ppp_state.t;

        ppp_state
            .correct_mut(earth_frame, new_t, &null_dx, U8::USIZE)
            .unwrap_or_else(|e| {
                panic!("failed to apply dim={} correction: {}", U8::USIZE, e);
            });

        assert_eq!(
            ppp_state.position_ecef_m(),
            Vector3::new(
                REFERENCE_COORDS_ECEF_M.0,
                REFERENCE_COORDS_ECEF_M.1,
                REFERENCE_COORDS_ECEF_M.2,
            )
        );

        assert_eq!(
            ppp_state.position_velocity_ecef_m(),
            Vector6::new(
                REFERENCE_COORDS_ECEF_M.0,
                REFERENCE_COORDS_ECEF_M.1,
                REFERENCE_COORDS_ECEF_M.2,
                0.0,
                0.0,
                0.0,
            )
        );

        let mut dx = DVector::zeros(U8::USIZE);

        dx[0] = 1.0;
        dx[1] = 2.0;
        dx[2] = 3.0;
        dx[3] = 4.0;

        let new_t = ppp_state.t;

        ppp_state
            .correct_mut(earth_frame, new_t, &dx, U8::USIZE)
            .unwrap_or_else(|e| {
                panic!("failed to apply dim={} correction: {}", U8::USIZE, e);
            });

        let position_ecef_m = ppp_state.position_ecef_m();

        assert_eq!(
            position_ecef_m,
            Vector3::new(
                REFERENCE_COORDS_ECEF_M.0 + 1.0,
                REFERENCE_COORDS_ECEF_M.1 + 2.0,
                REFERENCE_COORDS_ECEF_M.2 + 3.0,
            ),
            "invalid spatial state correction",
        );

        let (clock_offset_s, clock_drift_s) = ppp_state.clock_profile_s();

        assert_eq!(clock_offset_s, 4.0 / SPEED_OF_LIGHT_M_S);
        assert_eq!(clock_drift_s, 0.0, "clock drift should have been preserved");
    }
}
