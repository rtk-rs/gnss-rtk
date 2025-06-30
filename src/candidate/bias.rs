use crate::prelude::{BiasRuntime, Candidate, Vector3};

impl Candidate {
    /// Converts this [Candidate] to partial [BiasRuntime] which is
    /// enough to prefit some of the biases.
    pub(crate) fn to_partial_bias_runtime(&self) -> BiasRuntime {
        let mut rtm = BiasRuntime::default();
        rtm.sv = self.sv;
        rtm.epoch = self.epoch;
        rtm
    }

    /// Converts this [Candidate] to [BiasRuntime] that can then
    /// be used for complete biases compensation.
    pub(crate) fn to_bias_runtime(
        &self,
        rcvr_position_ecef_m: Vector3<f64>,
        rcvr_lat_long_alt_deg_deg_km: (f64, f64, f64),
    ) -> Option<BiasRuntime> {
        let rx_orbit = self.orbit?;
        let sv_elevation_azimuth_deg_deg = self.attitude()?;
        let pos_vel_m = rx_orbit.to_cartesian_pos_vel() * 1.0E3;

        Some(BiasRuntime {
            sv: self.sv,
            epoch: self.epoch,
            sv_elevation_azimuth_deg_deg,
            rcvr_lat_long_alt_deg_deg_km,
            rx_position_m: rcvr_position_ecef_m,
            frequency_hz: self
                .prefered_carrier()
                .expect("internal error: bias compensation with empty measurements")
                .frequency_hz(),
            sv_position_ecef_m: (pos_vel_m[0], pos_vel_m[1], pos_vel_m[2]),
        })
    }
}
