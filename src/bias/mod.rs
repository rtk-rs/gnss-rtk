use crate::prelude::Epoch;

pub(crate) mod tropo;
pub use tropo::{TropoComponents, TropoModel};

pub(crate) mod iono;
pub use iono::{BdModel, IonoComponents, IonosphereBias, KbModel, NgModel};

pub(crate) struct RuntimeParams {
    pub t: Epoch,
    pub frequency: f64,
    pub elevation_deg: f64,
    pub azimuth_rad: f64,
    pub elevation_rad: f64,
    pub rx_geo: (f64, f64, f64),
    pub rx_rad: (f64, f64),
}
