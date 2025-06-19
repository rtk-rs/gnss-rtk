use rstest::*;

use rand::{prelude::*, rngs::SmallRng, SeedableRng};

use std::str::FromStr;

use crate::{
    navigation::apriori::Apriori,
    prelude::{
        Almanac, Config, Epoch, Error, Frame, Method, Orbit, OrbitSource, StaticSolver,
        UserParameters, SV,
    },
    tests::{
        bias::NullBias, ephemeris::NullEph, init_logger, time::NullTime, CandidatesBuilder,
        OrbitsData, TestNumber,
    },
};

#[derive(Default)]
pub struct FuzzTestOrbits {
    pub database: Vec<Orbit>,
}

impl FuzzTestOrbits {
    pub fn add(&mut self, orbit: Orbit) {
        self.database.push(orbit);
    }
}

impl OrbitSource for FuzzTestOrbits {
    fn state_at(&self, t: Epoch, sv: SV, frame: Frame) -> Option<Orbit> {
        None
    }
}
