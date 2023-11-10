#![doc = include_str!("../README.md")]
#![cfg_attr(docrs, feature(doc_cfg))]

extern crate gnss_rs as gnss;
extern crate nyx_space as nyx;

// private modules
mod apriori;
mod candidate;
mod cfg;
mod solutions;
mod solver;
mod tropo;
mod vector;

// prelude
pub mod prelude {
    pub use crate::apriori::AprioriPosition;
    pub use crate::candidate::{Candidate, PseudoRange};
    pub use crate::cfg::Config;
    pub use crate::solutions::{PVTSolution, PVTSolutionType};
    pub use crate::solver::{InterpolationResult, Mode, Solver};
    pub use crate::tropo::TropoComponents;
    // re-export
    pub use gnss::prelude::{Constellation, SV};
    pub use hifitime::{Duration, Epoch, TimeScale};
}

// pub export
pub use solver::Error;
pub use vector::Vector3D;
