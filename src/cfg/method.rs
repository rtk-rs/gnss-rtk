use crate::prelude::Error;

#[cfg(feature = "serde")]
use serde::Deserialize; //, Serialize};

/// Solving method
#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub enum Method {
    /// Single Point Positioning (SPP).
    /// Code based navigation on a single carrier frequency.
    /// Phase observations are completely discarded and are not required from the RX side.
    /// When prefered signal is defined, we expect a pseudo range on that particular frequency.
    /// When prefered signal is not defined, we might switch depending on the best SNR.
    /// Expect +/- 10m accuracy is you correctly tune other parameters.
    /// You might hope for +/- 5m on high quality data.
    SPP,
    /// Code based Precise Point Positioning (CPP).
    /// Code based dual carrier frequency technique.
    /// Phase observations are completely discarded and are not required from the RX side.
    /// Expect +/- 5m accuracy is you correctly tune other parameters.
    /// You might hope for +/- 1m on high quality data.
    #[default]
    CPP,
    /// Precise Point Positioning (PPP).
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
