//! Solver configuration preset

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

const fn default_max_gdop() -> f64 {
    5.0
}

const fn default_postfit_denoising() -> f64 {
    1000.0
}

const fn default_open_loop() -> bool {
    false
}

#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SolverOpts {
    /// GDOP threshold to invalidate ongoing GDOP
    #[cfg_attr(feature = "serde", serde(default = "default_max_gdop"))]
    pub max_gdop: f64,
    /// GNSS-RTK allows the user to open the navigation filter loop,
    /// which is totally forbidden in roaming applications.
    #[cfg_attr(feature = "serde", serde(default = "default_open_loop"))]
    pub open_loop: bool,
    /// Possible extra denoising filter, at the expense
    /// of more processing time. The configuration is the denoising factor.
    /// 1000 for x1000 improvement attempt.
    #[cfg_attr(feature = "serde", serde(default = "default_postfit_denoising"))]
    pub postfit_denoising: f64,
}

impl Default for SolverOpts {
    fn default() -> Self {
        Self {
            max_gdop: default_max_gdop(),
            open_loop: default_open_loop(),
            postfit_denoising: default_postfit_denoising(),
        }
    }
}

impl SolverOpts {
    /// Parameter settings recommended for static ultra precise applications
    pub fn static_preset() -> Self {
        Self {
            max_gdop: 3.0,
            open_loop: default_open_loop(),
            postfit_denoising: default_postfit_denoising(),
        }
    }
}
