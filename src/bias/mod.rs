use crate::prelude::Epoch;

pub(crate) mod tropo;
pub use tropo::TroposphereModel;

pub(crate) mod iono;
pub use iono::{IonosphereBias, IonosphereModel, KbModel};

pub trait Bias {
    /// Compute bias as meters of propagation delay using the [BiasRuntime] infos.
    fn bias_m(&self, rtm: &BiasRuntime) -> f64;
}

/// [BiasRuntime] contains everything to compute your [Bias] estimate.
pub struct BiasRuntime {
    /// Current [Epoch]
    pub t: Epoch,
    /// SV position expressed in meters ECEF.
    pub sv_position_m: (f64, f64, f64),
    /// SV (elevation, azimuth) attitude in degrees.
    pub sv_elevation_azimuth_deg_deg: (f64, f64),
    /// Latest receiver position estimate, in meters ECEF
    pub rx_position_m: (f64, f64, f64),
    /// Latest receiver (latitude, longitude, altitude above sea) estimates, in degrees and kilometers.
    pub rx_lat_long_alt_deg_deg_km: (f64, f64, f64),
    /// Prefered signal frequency used in navigation process, in Hertz.
    pub frequency_hz: f64,
}
