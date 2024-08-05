use crate::prelude::{Carrier, Epoch, Observation, SV};

/// The [BaseStation] trait needs to be implemented by all [BaseStation]s (remote reference sites).
pub trait BaseStation {
    /// Observe specified [SV] at requested [Epoch].
    /// If you can't, simply return None, but this [SV] will be dropped from current navigation.
    fn observe(&mut self, t: Epoch, sv: SV, carrier: Carrier) -> Option<Observation>;
}
