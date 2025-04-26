use crate::prelude::{BiasRuntime, Candidate, Epoch, Vector3};

impl Candidate {
    /// Creates [BiasRuntime] that can be used of any bias solving for this [Candidate]
    pub(crate) fn to_bias_runtime(
        &self,
        t: Epoch,
        frequency_hz: f64,
        x0_y0_z0_m: Vector3<f64>,
        rx_lat_long_alt_deg_deg_km: (f64, f64, f64),
    ) -> Option<BiasRuntime> {
        let rx_orbit = self.orbit?;
        let sv_elevation_azimuth_deg_deg = self.attitude()?;
        let pos_vel_m = rx_orbit.to_cartesian_pos_vel() * 1.0E3;

        Some(BiasRuntime {
            t,
            frequency_hz,
            sv_elevation_azimuth_deg_deg,
            rx_position_m: x0_y0_z0_m,
            rx_lat_long_alt_deg_deg_km: rx_lat_long_alt_deg_deg_km,
            sv_position_m: (pos_vel_m[0], pos_vel_m[1], pos_vel_m[2]),
        })
    }
}
