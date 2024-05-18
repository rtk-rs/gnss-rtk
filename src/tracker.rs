use crate::prelude::Duration;
use std::collections::HashMap;

/// Signal tracker
pub struct Tracker {
    gap_tolerance: Duration,
    buffer: HashMap<SV, Vec<f64>>,    
}

/// SignaL phase Ambiguity fixing methods,
/// only apply to strategies that involve Phase observations.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum AmbiguityFixing {
    /// [WideLane]
    #[default]
    WideLane,
}
