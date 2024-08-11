use nyx::cosmic::SPEED_OF_LIGHT_M_S;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Default, PartialEq, PartialOrd, Eq, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Carrier {
    /// L1 (GPS/QZSS/SBAS) same frequency as E1 and B1aB1c
    #[default]
    L1,
    /// L2 (GPS/QZSS)
    L2,
    /// L5 (GPS/QZSS/SBAS) same frequency as E5A and B2A
    L5,
    /// L6 (GPS/QZSS) same frequency as E6
    L6,
    /// E1 (Galileo)
    E1,
    /// E5 (Galileo) same frequency as B2
    E5,
    /// E5A (Galileo) same frequency as L5
    E5A,
    /// E5B (Galileo) same frequency as B2iB2b
    E5B,
    /// E6 (Galileo) same frequency as L6
    E6,
    /// B1aB1c (BDS) same frequency as L1
    B1aB1c,
    /// B1I (BDS)
    B1I,
    /// B2I/B2B (BDS) same frequency as E5b
    B2iB2b,
    /// B2 (BDS) same frequency as E5
    B2,
    /// B2A (BDS) same frequency as L5 and E5A
    B2A,
    /// B3 (BDS)
    B3,
}

impl std::fmt::Display for Carrier {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> Result<(), std::fmt::Error> {
        match self {
            Self::L1 => write!(f, "L1"),
            Self::L2 => write!(f, "L2"),
            Self::L5 => write!(f, "L5"),
            Self::L6 => write!(f, "L6"),
            Self::E1 => write!(f, "E1"),
            Self::E5 => write!(f, "E5"),
            Self::E5A => write!(f, "E5A"),
            Self::E5B => write!(f, "E5B"),
            Self::E6 => write!(f, "E6"),
            Self::B1I => write!(f, "B1I"),
            Self::B1aB1c => write!(f, "B1A/B1C"),
            Self::B2iB2b => write!(f, "B2I/B2B"),
            Self::B2 => write!(f, "B2"),
            Self::B3 => write!(f, "B3"),
            Self::B2A => write!(f, "B2A"),
        }
    }
}

impl Carrier {
    pub fn frequency(&self) -> f64 {
        match self {
            Self::L1 | Self::E1 | Self::B1aB1c => 1575.42E6_f64,
            Self::L2 => 1227.60E6_f64,
            Self::L5 | Self::E5A | Self::B2A => 1176.45E6_f64,
            Self::E5 | Self::B2 => 1191.795E6_f64,
            Self::L6 | Self::E6 => 1278.750E6_f64,
            Self::B3 => 1268.52E6_f64,
            Self::E5B | Self::B2iB2b => 1207.14E6_f64,
            Self::B1I => 1561.098E6_f64,
        }
    }
    pub fn wavelength(&self) -> f64 {
        SPEED_OF_LIGHT_M_S / self.frequency()
    }
}

/// Signal used in [PVTSolution] resolution
#[derive(Debug, Clone)]
pub enum Signal {
    Single(Carrier),
    Dual((Carrier, Carrier)),
}

impl Signal {
    pub(crate) fn single(carrier: Carrier) -> Self {
        Self::Single(carrier)
    }
    pub(crate) fn dual(lhs: Carrier, rhs: Carrier) -> Self {
        Self::Dual((lhs, rhs))
    }
}

impl std::fmt::Display for Signal {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> Result<(), std::fmt::Error> {
        match self {
            Self::Single(carrier) => write!(f, "{}", carrier),
            Self::Dual((lhs, rhs)) => write!(f, "{}/{}", rhs, lhs),
        }
    }
}
