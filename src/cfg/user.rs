use crate::cfg::Error;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Default perturbation to clock prediction (in seconds)
const fn default_clock_sigma() -> f64 {
    1E-3_f64
}

/// Default user [Profile]
const fn default_user_profile() -> Option<Profile> {
    Some(Profile::Pedestrian)
}

/// Receiver [Profile], which is application dependent.
/// [Profile::Static] is our default value: any roaming application needs to customize its profile.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Profile {
    /// [Profile::Pedestrian]: < 10 km/h very low velocity
    #[cfg_attr(feature = "serde", serde(alias = "pedestrian", alias = "Pedestrian"))]
    #[default]
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

impl std::str::FromStr for Profile {
    type Err = Error;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.to_lowercase();
        let trimmed = s.trim();
        match trimmed {
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
            Self::Pedestrian => write!(f, "Pedestrian"),
            Self::Car => write!(f, "car"),
            Self::Airplane => write!(f, "airplane"),
            Self::Rocket => write!(f, "rocket"),
        }
    }
}

/// [User] profile definition. High accuracy requires correct use
/// of these settings.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct User {
    /// Custom user [Profile] which is application dependent.
    /// This is disregarded by solvers dedicated to static applications.
    #[cfg_attr(feature = "serde", serde(default))]
    pub profile: Option<Profile>,

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
            profile: default_user_profile(),
            clock_sigma_s: default_clock_sigma(),
        }
    }
}
