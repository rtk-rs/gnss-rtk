use crate::cfg::Error;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Rover, receiver or user [Profile], which is application dependent.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Profile {
    /// Receiver held in static.
    /// Typically used in Geodetic surveys (GNSS stations Referencing)
    /// and laboratories applications.
    #[default]
    #[cfg_attr(feature = "serde", serde(alias = "static", alias = "Static"))]
    Static,
    /// [Profile::Pedestrian]: < 10 km/h very low velocity
    #[cfg_attr(feature = "serde", serde(alias = "pedestrian", alias = "Pedestrian"))]
    Pedestrian,
    /// [Profile::Car]: < 100 km/h slow velocity
    #[cfg_attr(feature = "serde", serde(alias = "car", alias = "Car"))]
    Car,
    /// [Profile::Airplane]: < 1000 km/h high velocity
    #[cfg_attr(feature = "serde", serde(alias = "airplane", alias = "airplane"))]
    Airplane,
    /// [Profile::Rocket]: > 1000 km/h ultra high velocity
    #[cfg_attr(feature = "serde", serde(alias = "rocket", alias = "rocket"))]
    Rocket,
}

impl Profile {
    /// True if this [Profile] is [Profile::Static]
    pub fn is_static(&self) -> bool {
        *self == Self::Static
    }
}

impl std::str::FromStr for Profile {
    type Err = Error;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.to_lowercase();
        let trimmed = s.trim();
        match trimmed {
            "static" => Ok(Self::Static),
            "pedestrian" => Ok(Self::Pedestrian),
            "car" => Ok(Self::Car),
            "airplane" => Ok(Self::Airplane),
            "rocket" => Ok(Self::Rocket),
            _ => Err(Error::InvalidUserProfile),
        }
    }
}

impl std::fmt::Display for Profile {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Static => write!(f, "Static"),
            Self::Pedestrian => write!(f, "Pedestrian"),
            Self::Car => write!(f, "car"),
            Self::Airplane => write!(f, "airplane"),
            Self::Rocket => write!(f, "rocket"),
        }
    }
}
