use crate::prelude::{Bias, BiasRuntime, TroposphereModel};

/// Helper null implementation, where
/// [Bias] is not needed during test phases
pub struct TestBias {}

impl Bias for TestBias {
    fn ionosphere_bias_m(&self, _: &BiasRuntime) -> f64 {
        0.0
    }

    fn troposphere_bias_m(&self, rtm: &BiasRuntime) -> f64 {
        TroposphereModel::Niel.bias_m(rtm)
    }
}
