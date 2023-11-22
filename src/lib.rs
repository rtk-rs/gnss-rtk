#![doc = include_str!("../README.md")]
#![cfg_attr(docrs, feature(doc_cfg))]

extern crate gnss_rs as gnss;
extern crate nyx_space as nyx;

// private modules
mod apriori;
mod bias;
mod candidate;
mod cfg;
mod solutions;
mod solver;

// prelude
pub mod prelude {
    pub use crate::apriori::AprioriPosition;
    pub use crate::bias::{BdModel, IonosphericBias, KbModel, NgModel, TroposphericBias};
    pub use crate::candidate::{Candidate, Observation};
    pub use crate::cfg::Config;
    pub use crate::solutions::{PVTSolution, PVTSolutionType};
    pub use crate::solver::{InterpolationResult, Mode, Solver};
    // re-export
    pub use gnss::prelude::{Constellation, SV};
    pub use hifitime::{Duration, Epoch, TimeScale};
    pub use nalgebra::Vector3;
}

// pub export
pub use solver::Error;
