use crate::{
    carrier::Carrier,
    prelude::SV,
};

#[derive(Debug, Default, Clone, PartialEq)]
pub struct Ambiguity {
    /// [SV]
    pub sv: SV,

    /// [Carrier]
    pub carrier: Carrier,

    /// Actual value
    pub value: f64,
}

impl std::ops::Add for Ambiguity {
    type Output = Ambiguity;

    fn add(self, rhs: Self) -> Self {
        Self {
            sv: self.sv,
            carrier: self.carrier,
            value: self.value + rhs.value,
        }
    }
}

impl num_traits::identities::Zero for Ambiguity {
    fn zero() -> Self {
        Self::default()
    }

    fn is_zero(&self) -> bool {
        self.value == 0.0
    }
}

impl std::fmt::Display for Ambiguity {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}({})={}", self.sv, self.carrier, self.value)
    }
}

