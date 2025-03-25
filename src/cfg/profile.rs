#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Rover or receiver [Profile], which is application dependent.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Profile {
    /// Receiver held in static.
    /// Typically used in Geodetic surveys (GNSS stations Referencing)
    /// and laboratories applications.
    #[default]
    Static,
    /// Roaming: Pedestrian (5 to 10 km/h)
    Pedestrian,
}
