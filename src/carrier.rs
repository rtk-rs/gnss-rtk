#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum Carrier {
    // L1 (GPS/QZSS/SBAS)
    #[default]
    L1,
    // L2 (GPS/QZSS)
    L2,
    // L5 (GPS/QZSS/SBAS)
    L5,
    // L6 (GPS/QZSS)
    L6,
    // E1 (Galileo)
    E1,
    // E5 (Galileo)
    E5,
    // E6 (Galileo)
    E6,
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
            Self::E6 => write!(f, "E6"),
        }
    }
}

impl Carrier {
    pub(crate) fn frequency(&self) -> f64 {
        match self {
            Self::L1 | Self::E1 => 1575.42E6_f64,
            Self::L2 => 1227.60E6_f64,
            Self::L5 => 1176.45E6_f64,
            Self::E5 => 1191.795E6_f64,
            Self::L6 | Self::E6 => 1278.750E6_f64,
        }
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
