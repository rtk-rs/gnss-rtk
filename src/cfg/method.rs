use crate::prelude::Error;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Solving method
#[allow(non_camel_case_types)]
#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Method {
    /// Single Point Positioning is a code based navigation technique
    /// using a single carrier frequency. Any secondary frequency
    /// is disregarded by the entire process.
    /// Expect +/- 10m accuracy is you correctly tune other parameters.
    /// You might hope for +/- 5m on high quality data.
    SPP,

    /// Code Precise Positioning is a code based navigation technique,
    /// that uses two carrier frequencies to cancel the ionospheric perturbation.
    /// It is much more precise than SPP. When the full model is implemented, the
    /// accuracy is always smaller than 5m, you can reach decimeter accuracy
    /// on high quality setups. Phase range observations are disregarded by the entire
    /// process.
    #[default]
    CPP,

    /// Precise Point Positioning, uses both dual frequency code and phase navigation,
    /// including phase exploitation. The navigation process eventually
    /// only relies on the phase observations, at the expense of
    /// more preprocessing and computations and convergence of the PPP
    /// prefit. This is the most accurate solution, with at least 2 orders of magnitude
    /// improvements from [Method::CPP].
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
            _ => Err(Error::UnknownNavigationMethod),
        }
    }
}
