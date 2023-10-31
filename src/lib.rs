#![doc = include_str!("../README.md")]
#![cfg_attr(docrs, feature(doc_cfg))]

extern crate gnss_rs as gnss;
extern crate nyx_space as nyx;

// private modules
mod apriori;
mod candidate;
mod cfg;
mod estimate;
mod solver;
mod vector;

// public modules
pub mod model;

// prelude
pub mod prelude {
    pub use crate::apriori::AprioriPosition;
    pub use crate::candidate::{Candidate, PseudoRange};
    pub use crate::cfg::Config;
    pub use crate::estimate::Estimate;
    pub use crate::model::Modeling;
    pub use crate::solver::{InterpolationResult, Mode, Solver};
    // re-export
    pub use gnss::prelude::{Constellation, SV};
    pub use hifitime::{Duration, Epoch, TimeScale};
}

// pub export
pub use solver::Error;
pub use vector::Vector3D;
