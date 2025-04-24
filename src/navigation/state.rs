use nalgebra::{allocator::Allocator, DefaultAllocator, DimName, OVector, U4};

use anise::{
    astro::PhysicsResult,
    math::{Vector3, Vector4, Vector6},
    prelude::{Epoch, Frame},
};

use crate::{constants::SPEED_OF_LIGHT_M_S, navigation::Apriori, prelude::Orbit};

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
    /// Velocity (m.s⁻¹)
    velocity_m_s: Vector3,
    /// Geodeticy position (ddeg, ddeg, km above mean sea level)
    pub lat_long_alt_deg_deg_km: (f64, f64, f64),
}

impl<D: DimName> Default for State<D>
where
    DefaultAllocator: Allocator<D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
{
    fn default() -> Self {
        Self {
            t: Default::default(),
            x: OVector::<f64, D>::zeros(),
            velocity_m_s: Default::default(),
            clock_drift_s_s: Default::default(),
            lat_long_alt_deg_deg_km: Default::default(),
        }
    }
}

impl<D: DimName> std::fmt::Display for State<D>
where
    DefaultAllocator: Allocator<D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
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
    DefaultAllocator: Allocator<D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
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

    // /// Create new [State] from ECEF coordinates.
    // pub fn from_ecef_m(pos_m: Vector3, t: Epoch, frame: Frame) -> PhysicsResult<Self> {
    //     let pos_vel = Vector6::new(
    //         pos_m[0] / 1.0E3,
    //         pos_m[1] / 1.0E3,
    //         pos_m[2] / 1.0E3,
    //         0.0,
    //         0.0,
    //         0.0,
    //     );

    //     let orbit = Orbit::from_cartesian_pos_vel(pos_vel, t, frame);

    //     Self::from_orbit(&orbit)
    // }

    // /// Create new [State] from ECEF pos+vel
    // pub fn from_pos_vel_ecef_m(pos_vel_m: Vector6, t: Epoch, frame: Frame) -> PhysicsResult<Self> {
    //     let pos_vel_km = pos_vel_m / 1.0E3;
    //     let orbit = Orbit::from_cartesian_pos_vel(pos_vel_km, t, frame);
    //     Self::from_orbit(&orbit)
    // }

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
            velocity_m_s: Default::default(),
            lat_long_alt_deg_deg_km: latlongalt,
        })
    }

    /// Returns position in ECEF meters as [Vector3]
    pub fn position_ecef_m(&self) -> Vector3 {
        Vector3::new(self.x[0], self.x[1], self.x[2])
    }

    /// Returns position and velocity in ECEF meters as [Vector6]
    pub fn position_velocity_ecef_m(&self) -> Vector6 {
        Vector6::new(self.x[0], self.x[1], self.x[2], 0.0, 0.0, 0.0)
    }

    /// Returns estimated clock (offset, drift) in seconds and s.s⁻¹.
    pub fn clock_profile_s(&self) -> (f64, f64) {
        (self.x[3], self.clock_drift_s_s)
    }

    /// Converts [State] to [Orbit]
    pub fn to_orbit(&self, frame: Frame) -> Orbit {
        let pos_vel_km_s = self.position_velocity_ecef_m() / 1.0E3;
        Orbit::from_cartesian_pos_vel(pos_vel_km_s, self.t, frame)
    }

    /// [State] update
    pub fn update(&mut self, t: Epoch, frame: Frame, dx: &OVector<f64, D>) -> PhysicsResult<()> {
        let dt = (t - self.t).to_seconds();
        if dt > 0.0 {
            self.clock_drift_s_s = (dx[3] / SPEED_OF_LIGHT_M_S - self.x[3]) / dt;
            self.velocity_m_s = Vector3::new(dx[0] / dt, dx[1] / dt, dx[2] / dt);
        }

        for i in 0..4 {
            if i == 3 {
                self.x[i] = dx[i] / SPEED_OF_LIGHT_M_S;
            } else {
                self.x[i] += dx[i];
            }
        }

        self.t = t;

        // update attitude
        let new_orbit = self.to_orbit(frame);
        self.lat_long_alt_deg_deg_km = new_orbit.latlongalt()?;

        Ok(())
    }

    /// Temporal update
    pub fn postfit_update(&mut self, frame: Frame, dx: Vector6) -> PhysicsResult<()> {
        self.x[0] = dx[0];
        self.x[1] = dx[1];
        self.x[2] = dx[2];

        let new_orbit = self.to_orbit(frame);
        self.lat_long_alt_deg_deg_km = new_orbit.latlongalt()?;

        Ok(())
    }
}

impl State<U4> {
    /// Compute residual between this [State] and other [State]
    pub fn residual(&self, rhs: &State<U4>) -> Vector4 {
        self.x - rhs.x
    }
}
