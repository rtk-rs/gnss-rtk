#![doc = include_str!("../README.md")]
#![cfg_attr(docsrs, feature(doc_cfg))]

extern crate gnss_rs as gnss;
extern crate nyx_space as nyx;

// private modules
mod ambiguity;
mod bancroft;
mod bias;
mod candidate;
mod carrier;
mod cfg;
mod navigation;
mod orbit;
mod position;
mod solver;

pub(crate) mod constants;
// pub(crate) mod tides;

// mod tracker;
// pub(crate) mod utils;

#[cfg(test)]
mod tests;

// prelude
pub mod prelude {
    pub use crate::ambiguity::Ambiguities;
    pub use crate::bias::{
        BdModel, IonoComponents, IonosphereBias, KbModel, NgModel, TropoComponents, TropoModel,
    };
    pub use crate::candidate::{Candidate, ClockCorrection, Observation};
    pub use crate::carrier::Carrier;
    pub use crate::cfg::{Config, Method};
    pub use crate::navigation::{Filter, InvalidationCause, PVTSolution, PVTSolutionType};
    pub use crate::orbit::OrbitalStateProvider;
    pub use crate::position::Position;
    pub use crate::solver::{Error, Solver};
    // re-export
    pub use anise::{
        constants::frames::{EARTH_J2000, IAU_EARTH_FRAME, SUN_J2000},
        naif::SPK,
        prelude::{Aberration, Almanac, Frame, Orbit},
    };
    pub use gnss::prelude::{Constellation, SV};
    pub use hifitime::{Duration, Epoch, TimeScale};
    pub use nalgebra::Vector3;
    pub use nyx_space::{cosmic::SPEED_OF_LIGHT_M_S, md::prelude::Arc};
}
