use gnss_rtk::prelude::{Config as RTKConfig, Method as RTKNaviMethod, Position};
use serde::Deserialize;

fn default_rtk_config() -> RTKConfig {
    RTKConfig::static_preset(RTKNaviMethod::SPP)
}

// CI/CD thorough verification purposes
#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct Setup {
    #[serde(default = "default_rtk_config")]
    pub rtk: RTKConfig,
    #[serde(default)]
    pub test: TestConditions,
    #[serde(default)]
    pub validation: TestValidation,
}

impl Default for Setup {
    fn default() -> Self {
        Self {
            test: Default::default(),
            validation: Default::default(),
            rtk: RTKConfig::static_preset(RTKNaviMethod::CPP),
        }
    }
}

impl Setup {
    pub fn rtk_config(&self) -> RTKConfig {
        self.rtk.clone()
    }
    pub fn test_conditions(&self) -> TestConditions {
        self.test.clone()
    }
    pub fn test_validation(&self) -> TestValidation {
        self.validation.clone()
    }
}

fn default_max_candidates() -> usize {
    16
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct TestConditions {
    #[serde(default)]
    pub kinematics: bool,
    #[serde(default = "default_max_candidates")]
    pub max_candidates: usize,
    #[serde(default)]
    pub apriori: Option<Position>,
}

impl Default for TestConditions {
    fn default() -> Self {
        Self {
            apriori: Default::default(),
            kinematics: Default::default(),
            max_candidates: default_max_candidates(),
        }
    }
}

fn default_max_init_iterations() -> usize {
    10
}

fn default_max_absolute_x_err_m() -> f64 {
    50.0
}

fn default_max_absolute_y_err_m() -> f64 {
    50.0
}

fn default_max_absolute_z_err_m() -> f64 {
    50.0
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct TestValidation {
    #[serde(default = "default_max_init_iterations")]
    pub max_init_iterations: usize,
    #[serde(default = "default_max_absolute_x_err_m")]
    pub max_absolute_x_err_m: f64,
    #[serde(default = "default_max_absolute_y_err_m")]
    pub max_absolute_y_err_m: f64,
    #[serde(default = "default_max_absolute_z_err_m")]
    pub max_absolute_z_err_m: f64,
}

impl Default for TestValidation {
    fn default() -> Self {
        Self {
            max_init_iterations: default_max_init_iterations(),
            max_absolute_x_err_m: default_max_absolute_x_err_m(),
            max_absolute_y_err_m: default_max_absolute_y_err_m(),
            max_absolute_z_err_m: default_max_absolute_z_err_m(),
        }
    }
}
