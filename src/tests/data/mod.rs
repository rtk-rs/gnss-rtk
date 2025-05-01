mod candidates;
pub use candidates::CandidatesBuilder;

mod orbits;
pub use orbits::OrbitsData;

use crate::prelude::{Constellation, Epoch, Frame, Orbit, SV};

pub const E01: SV = SV {
    constellation: Constellation::Galileo,
    prn: 1,
};
pub const E02: SV = SV {
    constellation: Constellation::Galileo,
    prn: 2,
};
pub const E03: SV = SV {
    constellation: Constellation::Galileo,
    prn: 3,
};
pub const E04: SV = SV {
    constellation: Constellation::Galileo,
    prn: 4,
};
pub const E05: SV = SV {
    constellation: Constellation::Galileo,
    prn: 5,
};
pub const E07: SV = SV {
    constellation: Constellation::Galileo,
    prn: 7,
};
pub const E08: SV = SV {
    constellation: Constellation::Galileo,
    prn: 8,
};
pub const E09: SV = SV {
    constellation: Constellation::Galileo,
    prn: 9,
};
pub const E11: SV = SV {
    constellation: Constellation::Galileo,
    prn: 11,
};
pub const E12: SV = SV {
    constellation: Constellation::Galileo,
    prn: 12,
};
pub const E13: SV = SV {
    constellation: Constellation::Galileo,
    prn: 13,
};
pub const E14: SV = SV {
    constellation: Constellation::Galileo,
    prn: 14,
};
pub const E15: SV = SV {
    constellation: Constellation::Galileo,
    prn: 15,
};
pub const E24: SV = SV {
    constellation: Constellation::Galileo,
    prn: 24,
};
pub const E31: SV = SV {
    constellation: Constellation::Galileo,
    prn: 31,
};
