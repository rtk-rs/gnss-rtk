pub(crate) mod tropo;
pub use tropo::TroposphericBias;

pub(crate) mod iono;
pub use iono::{IonosphericBias, KbModel};

pub(crate) trait Bias {
    /// True if such a bias needs internal modeling.
    fn needs_modeling(&self) -> bool;
}
