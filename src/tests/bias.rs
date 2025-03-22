use crate::prelude::{Bias, BiasRuntime};

/// Helper null implementation, where
/// [Bias] is not needed during test phases
pub struct NullBias {}

impl Bias for NullBias {
    fn ionosphere_bias_m(&self, _: &BiasRuntime) -> f64 {
        0.0
    }

    fn troposphere_bias_m(&self, _: &BiasRuntime) -> f64 {
        0.0
    }
}
