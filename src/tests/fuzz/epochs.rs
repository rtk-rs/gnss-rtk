use rstest::*;

use rand::{prelude::*, rngs::SmallRng, SeedableRng};

use std::str::FromStr;

use crate::{
    navigation::apriori::Apriori,
    prelude::{
        Almanac, Candidate, Config, Epoch, Error, Frame, Method, StaticSolver, UserParameters,
    },
    tests::{
        ephemeris::NullEph, init_logger, time::NullTime, CandidatesBuilder, OrbitsData, TestNumber,
    },
};

pub struct FuzzTestEpoch {
    /// [Epoch] time that has been generated
    pub time: Epoch,

    /// [Candidates] that were generated
    pub candidates: Vec<Candidate>,
}
