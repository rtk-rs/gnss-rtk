#![doc(
    html_logo_url = "https://github.com/rtk-rs/.github/blob/c458a3e2ce136793d281bc2e0f3161e69a663a2e/logos/logo2.jpg"
)]
#![doc = include_str!("../README.md")]
#![cfg_attr(docsrs, feature(doc_cfg))]

extern crate gnss_rs as gnss;

pub mod error;

// private modules
// mod ambiguity;
// mod averager;
mod bancroft;
mod bias;
mod candidate;
mod carrier;
mod cfg;
mod ephemeris;
mod implementations;
mod navigation;
mod orbit;
mod pool;
mod rtk;
mod smoothing;
mod time;
mod user;
// mod kinematic;

pub(crate) mod constants;
pub(crate) mod solver;

#[cfg(test)]
mod tests;

// prelude
pub mod prelude {
    // pub use crate::ambiguity::Ambiguities;
    pub use crate::{
        bias::{Bias, BiasRuntime, IonosphereBias, IonosphereModel, KbModel, TroposphereModel},
        candidate::{Candidate, ClockCorrection, Observation},
        carrier::{Carrier, Signal},
        cfg::{Config, Method},
        constants::SPEED_OF_LIGHT_M_S,
        ephemeris::{Ephemeris, EphemerisSource},
        error::Error,
        implementations::{KinematicSolver, StaticSolver},
        navigation::PVTSolution,
        orbit::OrbitSource,
        rtk::RTKBase,
        solver::Solver,
        time::AbsoluteTime,
        user::{ClockProfile, UserParameters, UserProfile},
    };

    // std types
    pub use std::rc::Rc;

    // gnss types
    pub use gnss::prelude::{Constellation, SV};

    // anise types
    pub use anise::{
        constants::frames::{EARTH_ITRF93, EARTH_J2000, IAU_EARTH_FRAME, SUN_J2000},
        naif::SPK,
        prelude::{Aberration, Almanac, Frame, Orbit},
    };

    // hifitime types
    pub use hifitime::{Duration, Epoch, TimeScale};

    // nalgebra
    pub use nalgebra::{Vector3, Vector4, Vector6};
}
