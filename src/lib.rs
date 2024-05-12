#![doc = include_str!("../README.md")]
#![cfg_attr(docrs, feature(doc_cfg))]

extern crate gnss_rs as gnss;
extern crate nyx_space as nyx;

// private modules
mod apriori;
mod bias;
mod candidate;
mod cfg;
mod navigation;
mod solver;

#[cfg(test)]
mod tests;

// prelude
pub mod prelude {
    pub use crate::apriori::AprioriPosition;
    pub use crate::bias::{BdModel, IonosphereBias, KbModel, NgModel, TroposphereBias};
    pub use crate::candidate::{Candidate, Carrier, Observation};
    pub use crate::cfg::{Config, Method};
    pub use crate::navigation::{Filter, PVTSolution, PVTSolutionType};
    pub use crate::solver::{Error, InterpolationResult, Solver};
    // re-export
    pub use gnss::prelude::{Constellation, SV};
    pub use hifitime::{Duration, Epoch, TimeScale};
    pub use nalgebra::Vector3;
    pub use nyx::md::prelude::{Arc, Bodies, Cosm, Frame, LightTimeCalc};
}
