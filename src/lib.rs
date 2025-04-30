#![doc(
    html_logo_url = "https://github.com/rtk-rs/.github/blob/c458a3e2ce136793d281bc2e0f3161e69a663a2e/logos/logo2.jpg"
)]
#![doc = include_str!("../README.md")]
#![cfg_attr(docsrs, feature(doc_cfg))]

extern crate gnss_rs as gnss;
//extern crate nyx_space as nyx;

pub mod error;

// private modules
mod ambiguity;
mod averager;
mod bancroft;
mod bias;
mod candidate;
mod carrier;
mod cfg;
mod navigation;
mod orbit;
mod pool;
mod smoothing;
mod time;

pub(crate) mod constants;
pub(crate) mod ppp;
pub(crate) mod rtk;
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
        cfg::{Config, Method, Profile},
        constants::SPEED_OF_LIGHT_M_S,
        error::Error,
        navigation::PVTSolution,
        orbit::OrbitSource,
        ppp::{StaticPPP, PPP},
        rtk::{RTKBase, StaticRTK, RTK},
        time::AbsoluteTime,
    };

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
