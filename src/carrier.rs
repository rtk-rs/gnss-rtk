use crate::prelude::Error;

use nyx::cosmic::SPEED_OF_LIGHT_M_S;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Default, PartialEq, PartialOrd, Eq, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Carrier {
    /// [Carrier::L1] (1575.42) (GPS +QZSS +SBAS +Galileo +BDS)
    #[default]
    L1,
    /// [Carrier::L2] (1227.60) (GPS +QZSS)
    L2,
    /// [Carrier::L5] (1176.45) (GPS +QZSS +SBAS +IRNSS)
    L5,
    /// [Carrier::E5b] (1207.140) (Galileo)
    E5b,
    /// [Carrier::E5a5b] (1191.795) (Galileo +BDS)
    E5a5b,
    /// [Carrier::E6Lex] (1278.75) (Galileo +QZSS)
    E6Lex,
    /// [Carrier::B1] (1561.098) (BDS)
    B1,
    /// [Carrier::B3] (1268.52) (BDS)
    B3,
    /// [Carrier::S] (2492.028) (IRNSS)
    S,
    /// [Carrier::G1] (1602.00) (Glonass)
    G1,
    /// [Carrier::G1a] (1600.995) (Glonass)
    G1a,
    /// [Carrier::G2] (1246.00) (Glonass)
    G2,
    /// [Carrier::G2a] (1248.06) (Glonass)
    G2a,
    /// [Carrier::G3] (1202.025) (Glonass)
    G3,
}

impl std::fmt::Display for Carrier {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> Result<(), std::fmt::Error> {
        match self {
            Self::L1 => write!(f, "L1"),
            Self::L2 => write!(f, "L2"),
            Self::L5 => write!(f, "L5"),
            Self::E5b => write!(f, "E5b"),
            Self::E5a5b => write!(f, "E5a5b"),
            Self::E6Lex => write!(f, "E6/LEX"),
            Self::B1 => write!(f, "B1"),
            Self::B3 => write!(f, "B3"),
            Self::S => write!(f, "S"),
            Self::G1 => write!(f, "G1"),
            Self::G1a => write!(f, "G1a"),
            Self::G2 => write!(f, "G2"),
            Self::G2a => write!(f, "G2a"),
            Self::G3 => write!(f, "G3"),
        }
    }
}

impl std::str::FromStr for Carrier {
    type Err = Error;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let trimmed = s.trim();
        let lowered = trimmed.to_lowercase();

        match lowered.as_str() {
            "l1" => Ok(Self::L1),
            "l2" => Ok(Self::L2),
            "l5" => Ok(Self::L5),
            "e5b" => Ok(Self::E5b),
            "e5a5b" => Ok(Self::E5a5b),
            "b1" => Ok(Self::B1),
            "b3" => Ok(Self::B3),
            "s" => Ok(Self::S),
            "g1" => Ok(Self::G1),
            "g1a" => Ok(Self::G1a),
            "g2" => Ok(Self::G2),
            "g2a" => Ok(Self::G2a),
            "g3" => Ok(Self::G3),
            "e6" | "lex" | "e6/lex" => Ok(Self::E6Lex),
            _ => Err(Error::InvalidFrequency),
        }
    }
}

impl Carrier {
    /// Build a new [Carrier] frequency from value in MHz.
    /// This requires a 1kHz accuracy on the description.
    pub fn from_frequency_mega_hz(freq_mhz: f64) -> Result<Self, Error> {
        if freq_mhz.is_sign_negative() {
            return Err(Error::InvalidFrequency);
        }

        let freq_khz = (freq_mhz * 1.0E3) as u32;

        match freq_khz {
            1575_420 => Ok(Self::L1),
            1227_600 => Ok(Self::L2),
            1176_450 => Ok(Self::L5),
            1207_140 => Ok(Self::E5b),
            1191_795 => Ok(Self::E5a5b),
            1278_750 => Ok(Self::E6Lex),
            1561_098 => Ok(Self::B1),
            1268_520 => Ok(Self::B3),
            2492_028 => Ok(Self::S),
            1602_000 => Ok(Self::G1),
            1600_995 => Ok(Self::G1a),
            1246_000 => Ok(Self::G2),
            1248_060 => Ok(Self::G2a),
            1202_025 => Ok(Self::G3),
            _ => Err(Error::UnknownCarrierFrequency),
        }
    }

    pub(crate) fn is_l1(&self) -> bool {
        *self == Carrier::L1
    }

    /// Returns unsigned frequency in kHz
    pub(crate) fn to_khz_unsigned(&self) -> u32 {
        let freq_mhz = self.frequency_mega_hz() * 1.0E3;
        freq_mhz as u32
    }

    /// Converts [Carrier] to exact frequency in Mhz with 1kHz garanteed accuracy.
    pub fn frequency_mega_hz(&self) -> f64 {
        match self {
            Self::L1 => 1575.420_f64,
            Self::L2 => 1227.600_f64,
            Self::L5 => 1176.450_f64,
            Self::E5b => 1207.140,
            Self::E5a5b => 1191.795,
            Self::E6Lex => 1278.750,
            Self::B1 => 1561.098,
            Self::B3 => 1268.520,
            Self::S => 2492.028,
            Self::G1 => 1602.000,
            Self::G1a => 1600.995,
            Self::G2 => 1246.000,
            Self::G2a => 1248.060,
            Self::G3 => 1202.025,
        }
    }

    pub fn frequency_hz(&self) -> f64 {
        self.frequency_mega_hz() * 1.0E6
    }

    pub fn wavelength(&self) -> f64 {
        SPEED_OF_LIGHT_M_S / self.frequency_hz()
    }
}

/// Signal used in [PVTSolution] resolution
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Signal {
    Single(Carrier),
    Dual((Carrier, Carrier)),
}

impl Default for Signal {
    fn default() -> Self {
        Self::Single(Default::default())
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

impl Signal {
    pub(crate) fn single(carrier: Carrier) -> Self {
        Self::Single(carrier)
    }
    pub(crate) fn dual(lhs: Carrier, rhs: Carrier) -> Self {
        Self::Dual((lhs, rhs))
    }
}

#[cfg(test)]
mod test {
    use super::Carrier;
    use std::str::FromStr;

    #[test]
    fn test_frequencies() {
        for carrier in [
            Carrier::L1,
            Carrier::L2,
            Carrier::L5,
            Carrier::E5b,
            Carrier::E5a5b,
            Carrier::E6Lex,
            Carrier::B1,
            Carrier::B3,
            Carrier::S,
            Carrier::G1,
            Carrier::G1a,
            Carrier::G2,
            Carrier::G2a,
            Carrier::G3,
        ] {
            let freq_mhz = carrier.frequency_mega_hz();

            let identified = Carrier::from_frequency_mega_hz(freq_mhz).unwrap_or_else(|e| {
                panic!(
                    "{} - failed to identify carrier from frequency: {} Mhz",
                    e, freq_mhz
                )
            });

            assert_eq!(
                identified, carrier,
                "Carrier frequency API: invalid for {} frequency",
                freq_mhz,
            );

            let formatted = carrier.to_string();
            let parsed = Carrier::from_str(&formatted).unwrap_or_else(|e| {
                panic!("{} - failed to identify Carrier from \"{}\"", e, formatted)
            });

            assert_eq!(
                parsed, carrier,
                "Carrier string API: invalid for {} frequency",
                carrier,
            )
        }
    }
}
