use crate::prelude::{BiasRuntime, EnvironmentalBias, TroposphereModel};

/// Helper null implementation, where
/// [Bias] is not needed during test phases
pub struct TestEnvironment {}

impl TestEnvironment {
    pub fn new() -> Self {
        Self {}
    }
}

impl EnvironmentalBias for TestEnvironment {
    fn ionosphere_bias_m(&self, _: &BiasRuntime) -> f64 {
        0.0
    }

    fn troposphere_bias_m(&self, rtm: &BiasRuntime) -> f64 {
        TroposphereModel::Niel.bias_m(rtm)
    }
}
