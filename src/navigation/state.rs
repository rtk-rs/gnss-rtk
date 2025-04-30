use nalgebra::{allocator::Allocator, DVector, DefaultAllocator, DimName, OVector};

use anise::{
    astro::PhysicsResult,
    math::{Vector3, Vector6},
    prelude::{Epoch, Frame},
};

use crate::{
    constants::SPEED_OF_LIGHT_M_S,
    navigation::{Apriori, Navigation},
    prelude::Orbit,
};

#[derive(Clone, Copy)]
pub struct State<D: DimName>
where
    DefaultAllocator: Allocator<D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
{
    /// [Epoch] of resolution
    pub t: Epoch,

    /// Internal [Vector4]
    x: OVector<f64, D>,

    /// Clock drift (s.s⁻¹)
    clock_drift_s_s: f64,

    /// Geodeticy position (ddeg, ddeg, km above mean sea level)
    pub lat_long_alt_deg_deg_km: (f64, f64, f64),
}

impl<D: DimName> Default for State<D>
where
    DefaultAllocator: Allocator<D> + Allocator<D, D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
{
    fn default() -> Self {
        Self {
            t: Default::default(),
            x: OVector::<f64, D>::zeros(),
            clock_drift_s_s: Default::default(),
            lat_long_alt_deg_deg_km: Default::default(),
        }
    }
}

impl<D: DimName> std::fmt::Display for State<D>
where
    DefaultAllocator: Allocator<D> + Allocator<D, D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let position_vel_m = self.position_velocity_ecef_m();
        let (offset, drift) = self.clock_profile_s();

        write!(
            f,
            "{} dt={:.11E}s drift={:.11E}s/s",
            position_vel_m, offset, drift,
        )
    }
}

impl<D: DimName> State<D>
where
    DefaultAllocator: Allocator<D> + Allocator<D, D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
{
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
        assert!(
            D::USIZE > 3,
            "internal error: state minimal dimension implementation!"
        );

        let pos_vel_m = orbit.to_cartesian_pos_vel() * 1.0E3;
        let latlongalt = orbit.latlongalt()?;

        let mut x = OVector::<f64, D>::zeros();

        for i in 0..3 {
            x[i] = pos_vel_m[i];
        }

        Ok(Self {
            x,
            t: orbit.epoch,
            clock_drift_s_s: 0.0_f64,
            lat_long_alt_deg_deg_km: latlongalt,
        })
    }

    /// Returns position in ECEF meters as [Vector3]
    pub fn position_ecef_m(&self) -> Vector3 {
        Vector3::new(self.x[0], self.x[1], self.x[2])
    }

    /// Returns position and velocity in ECEF meters as [Vector6]
    pub fn position_velocity_ecef_m(&self) -> Vector6 {
        match D::USIZE {
            4 => Vector6::new(self.x[0], self.x[1], self.x[2], 0.0, 0.0, 0.0),
            7 => Vector6::new(
                self.x[0], self.x[1], self.x[2], self.x[3], self.x[4], self.x[5],
            ),
            _ => unreachable!("invalid dimensions!"),
        }
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
        let dt = (pending_t - self.t).to_seconds();

        if dt > 0.0 {
            self.clock_drift_s_s = (dx[Navigation::clock_index()] / SPEED_OF_LIGHT_M_S
                - self.x[Navigation::clock_index()])
                / dt;
        }

        for i in 0..D::USIZE {
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
    pub fn postfit_update_mut(&mut self, frame: Frame, dx: Vector6) -> PhysicsResult<()> {
        for i in 0..D::USIZE {
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
