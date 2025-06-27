use crate::{
    navigation::{sv::SVContribution, vector::VectorContribution},
    prelude::{Candidate, Config, Epoch, Error, Vector3},
};

impl Candidate {
    /// Measurement vector contribution.
    /// This will pass if
    /// - State has been previously resolved
    /// - Range estimate is available
    /// - Preset modeling are matched
    ///
    /// ## Input
    /// - t: [Epoch] of computation
    /// - cfg: [Config] preset
    /// - x0_y0_z0: current state (metric)
    /// - rx_lat_long_alt_ddeg_km: state as geodetic lat, long both
    /// in decimal degrees, and altitude above mean sea level (km)
    /// - contribution: mutable [SVContribution]
    /// ## Returns
    /// - [VectorContribution]
    pub(crate) fn rtk_vector_contribution(
        epoch: Epoch,
        cfg: &Config,
        two_rows: bool,
        amb: Option<u64>,
        x0_y0_z0_m: Vector3<f64>,
        rx_lat_long_alt_deg_deg_km: (f64, f64, f64),
        contribution: &mut SVContribution,
    ) -> Result<VectorContribution, Error> {
        Ok(VectorContribution {
            row_1: 0.0,
            row_2: 0.0,
            sigma: 0.0,
        })
    }
}
