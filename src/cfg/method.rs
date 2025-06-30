use crate::prelude::Error;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Solving method
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

    /// Precise Point Positioning is a dual frequency navigation technique
    /// that requires both code and phase measurements. For both frequencies, you must
    /// provide both types of observation, otherwise the process cannot complete.
    /// It is about 10 times more accurate than [Method::CPP].
    PPP,

    /// Precise Point Positioning with advanced Ambiguity Resolution (AR),
    /// is the same algorithm as [Method::PPP], therefore has the same requirements,
    /// but deploys an advanced AR method, using a secondary filter and is more
    /// computationnally intense. [Method::PPP_AR] is not completed to this day,
    /// do not use.
    #[allow(non_snake_case)]
    PPP_AR,
}

impl std::fmt::Display for Method {
    fn fmt(&self, fmt: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Self::SPP => write!(fmt, "SPP"),
            Self::CPP => write!(fmt, "CPP"),
            Self::PPP => write!(fmt, "PPP"),
            Self::PPP_AR => write!(fmt, "PPP-AR"),
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
            "ppp-ar" => Ok(Self::PPP_AR),
            _ => Err(Error::UnknownNavigationMethod),
        }
    }
}

impl Method {
    pub(crate) fn is_ppp(&self) -> bool {
        matches!(self, Self::PPP | Self::PPP_AR)
    }
}
