use crate::{
    navigation::state::State,
    prelude::{BiasRuntime, Candidate, Epoch},
};

impl Candidate {
    /// Creates [BiasRuntime] that can be used of any bias solving for this [Candidate]
    pub(crate) fn to_bias_runtime(&self, t: Epoch, state: &State) -> Option<BiasRuntime> {
        let rx_orbit = self.orbit?;
        let sv_elevation_azimuth_deg_deg = self.attitude()?;
        let pos_vel_m = rx_orbit.to_cartesian_pos_vel() * 1.0E3;

        let obs = self.best_snr_observation()?;
        let frequency_hz = obs.carrier.frequency();

        Some(BiasRuntime {
            t,
            frequency_hz,
            sv_elevation_azimuth_deg_deg,
            rx_position_m: state.pos_m,
            rx_lat_long_alt_deg_deg_km: state.lat_long_alt_deg_deg_km,
            sv_position_m: (pos_vel_m[0], pos_vel_m[1], pos_vel_m[2]),
        })
    }

    // /// Apply troposphere model.
    // pub(crate) fn apply_tropo_model(&mut self, _: &BiasRuntime) {}

    // /// Apply ionosphere model.
    // pub(crate) fn apply_iono_model(&mut self, _: &BiasRuntime) {}
}
