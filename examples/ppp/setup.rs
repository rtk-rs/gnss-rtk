use gnss_rtk::prelude::{Config as RTKConfig, Method as RTKNaviMethod, Position};
use serde::Deserialize;

// CI/CD thorough verification purposes
#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct Setup {
    pub rtk: RTKConfig,
    pub test: TestConditions,
}

impl Default for Setup {
    fn default() -> Self {
        Self {
            test: Default::default(),
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
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct TestConditions {
    pub kinematics: bool,
    pub max_candidates: usize,
    pub apriori: Option<Position>,
}

impl Default for TestConditions {
    fn default() -> Self {
        Self {
            apriori: None,
            kinematics: false,
            max_candidates: 16,
        }
    }
}
