#![doc = include_str!("../README.md")]
#![cfg_attr(docrs, feature(doc_cfg))]

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
mod position;
mod solver;

// mod tracker;
// pub(crate) mod utils;

#[cfg(test)]
mod tests;

// prelude
pub mod prelude {
    pub use crate::ambiguity::Ambiguities;
    pub use crate::bias::{BdModel, IonosphereBias, KbModel, NgModel, TroposphereBias};
    pub use crate::candidate::{Candidate, PhaseRange, PseudoRange};
    pub use crate::carrier::Carrier;
    pub use crate::cfg::{Config, Method};
    pub use crate::navigation::{Filter, InvalidationCause, PVTSolution, PVTSolutionType};
    pub use crate::position::Position;
    pub use crate::solver::{Error, InterpolationResult, Solver};
    // re-export
    pub use anise::constants::frames::{EARTH_J2000, SUN_J2000};
    pub use anise::naif::SPK;
    pub use anise::prelude::{Aberration, Almanac, Frame};
    pub use gnss::prelude::{Constellation, SV};
    pub use hifitime::{Duration, Epoch, TimeScale};
    pub use nalgebra::Vector3;
    pub use std::sync::Arc;
}
