#![doc = include_str!("../README.md")]
#![cfg_attr(docrs, feature(doc_cfg))]

extern crate gnss_rs as gnss;
extern crate nyx_space as nyx;

mod cfg;
mod model;
mod solver;
mod vector;

mod apriori;
mod candidate;
mod estimate;

pub mod prelude {
    pub use crate::apriori::AprioriPosition;
    pub use crate::candidate::Candidate;
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
