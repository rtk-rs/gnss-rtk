use crate::prelude::Carrier;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub enum PreferedSignal {
    /// Single [Carrier]
    Single(Carrier),
    /// Dual [Carrier] combination
    Combination((Carrier, Carrier)),
}
