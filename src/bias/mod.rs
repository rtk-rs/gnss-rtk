use crate::prelude::Epoch;

pub(crate) mod tropo;
pub use tropo::{TropoModel, TroposphericBias};

pub(crate) mod iono;
pub use iono::{IonosphericBias, KbModel};

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
