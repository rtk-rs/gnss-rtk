use crate::prelude::Epoch;

pub(crate) mod tropo;
pub use tropo::{TropoModel, TroposphereBias};

pub(crate) mod iono;
pub use iono::{BdModel, IonosphereBias, KbModel, NgModel};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Modeled (estimated) or measured bias
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Bias {
    /// Measured delay [meters of delay]
    pub measured: Option<f64>,
    /// Modeled delay [meters of delay]
    pub modeled: Option<f64>,
}

impl Bias {
    /// Bias value
    pub fn value(&self) -> Option<f64> {
        if self.measured.is_none() {
            self.modeled
        } else {
            self.measured
        }
    }
    /// Builds a measured Time Delay from a measurement in [s]
    pub fn measured(measurement: f64) -> Self {
        Self {
            measured: Some(measurement),
            modeled: None,
        }
    }
    /// Builds a modeled Time Delay from a model in [s]
    pub fn modeled(model: f64) -> Self {
        Self {
            modeled: Some(model),
            measured: None,
        }
    }
}

pub(crate) struct RuntimeParam {
    pub t: Epoch,
    // elevation in [°]
    pub elevation: f64,
    // azimmuth in [°]
    pub azimuth: f64,
    // Apriori geodetic coordinates with latitude
    // expressed as meters above sea level
    pub apriori_geo: (f64, f64, f64),
    // signal frequency
    pub frequency: f64,
}
