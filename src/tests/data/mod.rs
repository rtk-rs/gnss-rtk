mod candidates;
pub use candidates::CandidatesBuilder;

mod orbits;
pub use orbits::OrbitsData;

use crate::prelude::{Constellation, SV};

pub const E01: SV = SV {
    constellation: Constellation::Galileo,
    prn: 1,
};
pub const E03: SV = SV {
    constellation: Constellation::Galileo,
    prn: 3,
};
pub const E05: SV = SV {
    constellation: Constellation::Galileo,
    prn: 5,
};
pub const E09: SV = SV {
    constellation: Constellation::Galileo,
    prn: 9,
};
pub const E13: SV = SV {
    constellation: Constellation::Galileo,
    prn: 13,
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
