use crate::prelude::{Epoch, Vector3};

pub(crate) mod tropo;
pub use tropo::TroposphereModel;

pub(crate) mod iono;
pub use iono::{IonosphereBias, IonosphereModel, KbModel};

#[cfg(doc)]
use crate::prelude::{Config, Method, SV};

/// [Bias] model implementation. You can either deploy one of the proposed models
/// or apply your own equations. Both may use the [BiasRuntime] structure that reflects
/// the ongoing conditions.
pub trait Bias {
    /// Return the propagation bias (environmental perturbation) for this [BiasRuntime],
    /// due to tropospheric perturbation, either by applying of our [TroposphereModel]
    /// or using your own equation.
    /// Otherwise, simply return 0 but it will impact the accuracy of your solutions.  
    fn troposphere_bias_m(&self, rtm: &BiasRuntime) -> f64;

    /// Return the propagation bias (environmental perturbation) for this [BiasRuntime],
    /// due to ionospheric perturbation, either by applying our [IonosphereModel] or
    /// using your own equation.
    /// Otherwise, simply return 0 but it will impact the accuracy of your solutions.
    /// This truly impacts your solution when using [Method::SPP] navigation technique,
    /// otherwise this is physically cancelled.
    fn ionosphere_bias_m(&self, rtm: &BiasRuntime) -> f64;
}

/// [BiasRuntime] contains everything to compute your [Bias] estimate.
pub struct BiasRuntime {
    /// [Epoch] of measurement.
    pub epoch: Epoch,

    /// Selected [SV] position, in meters ECEF.
    pub sv_position_m: (f64, f64, f64),

    /// Selected [SV] (elevation, azimuth) attitude in degrees.
    pub sv_elevation_azimuth_deg_deg: (f64, f64),

    /// Estimated receiver state, in meters ECEF.
    pub rx_position_m: Vector3<f64>,

    /// Estimated receiver state, as (latitude, longitude, altitude above sea),
    /// in degrees and kilometers.
    pub rx_lat_long_alt_deg_deg_km: (f64, f64, f64),

    /// Signal frequency (in Hertz) used in the navigation process.
    pub frequency_hz: f64,
}
