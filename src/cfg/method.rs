use crate::prelude::Error;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Solving method
#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub enum Method {
    /// Single Point Positioning (SPP).
    /// Code based navigation on a single carrier frequency.
    /// Phase observations are not required, and Ionosphere model must be provided
    /// for best results. Exhibits metric accuracy on high quality data.
    SPP,
    /// Code based Precise Point Positioning (CPP).
    /// Code based navigation on dual carrier frequencies.
    /// Both phase observations and Ionosphere modeling are not required.
    /// Exhibits metric accuracy on high quality data.
    #[default]
    CPP,
    /// Precise Point Positioning (PPP).
    /// Code and Carrier based navigation, requires Pseudo range and
    /// Carrier phase observations on two frequencies.
    /// Exhibits centimetric accuracy on high quality data.
    PPP,
}

impl std::fmt::Display for Method {
    fn fmt(&self, fmt: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Self::SPP => write!(fmt, "SPP"),
            Self::CPP => write!(fmt, "CPP"),
            Self::PPP => write!(fmt, "PPP"),
        }
    }
}

impl std::str::FromStr for Method {
    type Err = Error;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.trim().to_lowercase().as_str() {
            "spp" => Ok(Self::SPP),
            "cpp" => Ok(Self::CPP),
            "ppp" => Ok(Self::PPP),
            _ => Err(Error::InvalidStrategy),
        }
    }
}
