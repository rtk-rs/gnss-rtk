use crate::prelude::BiasRuntime;

pub(crate) mod tropo;
pub use tropo::TroposphereModel;

pub(crate) mod iono;
pub use iono::{IonosphereBias, IonosphereModel, KbModel};

#[cfg(doc)]
use crate::prelude::{Config, Method, SV};

/// [Bias] model implementation. You can either deploy one of the proposed models
/// or apply your own equations. Both may use the [BiasRuntime] structure that reflects
/// the ongoing conditions.
pub trait EnvironmentalBias {
    /// Return the propagation bias (environmental perturbation) for this [BiasRuntime],
    /// due to tropospheric perturbation, either by applying of our [TroposphereModel]
    /// or using your own equation.
    /// Otherwise, simply return 0 but it will impact the accuracy of your solutions.  
    fn troposphere_bias_m(&self, rtm: &BiasRuntime) -> f64;

    /// Return the propagation bias (environmental perturbation) for this [BiasRuntime],
    /// due to ionospheric perturbation, either by applying our [IonosphereModel] or
    /// using your own equation.
    /// Otherwise, simply return 0 but it will impact the accuracy of your solutions
    /// when operating in [Method::SPP] mode, any other methods use physical cancellation
    /// and this value does not impact otherwise.
    fn ionosphere_bias_m(&self, rtm: &BiasRuntime) -> f64;
}
