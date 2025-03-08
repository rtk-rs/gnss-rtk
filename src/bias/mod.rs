use crate::prelude::Epoch;

pub(crate) mod tropo;
pub use tropo::TroposphereModel;

pub(crate) mod iono;
pub use iono::{IonosphereBias, IonosphereModel, KbModel};

#[cfg(doc)]
use crate::prelude::{Config, Method, SV};

pub trait Bias {
    /// Return a metric propagation bias using your own Troposphere model
    /// using [BiasRuntime] evaluation parameters.
    /// You can use one of the models we propose in case you have no better options.
    /// Otherwise, simply return 0 but it will impact the accuracy of your solutions.  
    fn troposphere_bias_m(&self, rtm: &BiasRuntime) -> f64;

    /// Return a metric propagation bias using your own Ionosphere model
    /// using [BiasRuntime] evaluation parameters.
    /// You can use one of the models we propose in case you have no better options.
    /// Otherwise, simply return 0 but it will impact the accuracy of your solutions,
    /// unless you [Config]ured the solver to use one of the physical cancellation techniques,
    /// like [Method::CPP].
    fn ionosphere_bias_m(&self, rtm: &BiasRuntime) -> f64;
}

/// [BiasRuntime] contains everything to compute your [Bias] estimate.
pub struct BiasRuntime {
    /// [Epoch] of computation.
    pub t: Epoch,
    /// Selected [SV] position, in meters ECEF.
    pub sv_position_m: (f64, f64, f64),
    /// Selected [SV] (elevation, azimuth) attitude in degrees.
    pub sv_elevation_azimuth_deg_deg: (f64, f64),
    /// Estimated receiver state, in meters ECEF.
    pub rx_position_m: (f64, f64, f64),
    /// Estimated receiver state, as (latitude, longitude, altitude above sea),
    /// in degrees and kilometers.
    pub rx_lat_long_alt_deg_deg_km: (f64, f64, f64),
    /// Selected signal frequency, in Hertz.
    pub frequency_hz: f64,
}
