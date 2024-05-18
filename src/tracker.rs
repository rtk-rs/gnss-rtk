/// SignaL phase Ambiguity fixing methods,
/// only apply to strategies that involve Phase observations.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum AmbiguityFixing {
    /// [WideLane]
    #[default]
    WideLane,
}
