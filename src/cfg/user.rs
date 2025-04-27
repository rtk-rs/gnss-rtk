use crate::cfg::Error;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Default perturbation to clock prediction (in seconds)
const fn default_clock_sigma() -> f64 {
    1E-3_f64
}

/// Receiver [Profile], which is application dependent.
/// [Profile::Static] is our default value: any roaming application needs to customize its profile.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Profile {
    /// Typically used in geodetic marker surveys and laboratries applications.
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

/// [User] profile definition. High accuracy requires correct use
/// of these settings.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct User {
    /// Custom user [Profile] which is application dependent
    #[cfg_attr(feature = "serde", serde(default))]
    pub profile: Profile,

    /// Receiver clock prediction perturbation (instantaneous bias) in seconds.
    /// Standard values are:
    /// - 10ms for very bad clocks
    /// - 1ms for low quality clocks (this is our default value)
    /// - 1us for good quality laboratory clocks
    /// - lower for ultra high quality clocks
    #[cfg_attr(
        feature = "serde",
        serde(
            alias = "clock",
            alias = "clock_sigma",
            default = "default_clock_sigma"
        )
    )]
    pub clock_sigma_s: f64,
}

impl Default for User {
    fn default() -> Self {
        Self {
            profile: Profile::default(),
            clock_sigma_s: default_clock_sigma(),
        }
    }
}
