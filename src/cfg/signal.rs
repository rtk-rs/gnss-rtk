use crate::prelude::Carrier;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq)]
pub enum PreferedSignal {
    /// Single [Carrier]
    Single(Carrier),
    /// Dual [Carrier] combination
    Combination((Carrier, Carrier)),
}

impl Default for PreferedSignal {
    fn default() -> Self {
        Self::Single(Default::default())
    }
}
