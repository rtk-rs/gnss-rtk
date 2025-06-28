#[derive(Debug, Copy, Clone, Default)]
pub struct VectorContribution {
    /// Row #1 contribution
    pub row_1: f64,

    /// Row #2 contribution
    pub row_2: f64,

    /// sigma
    pub sigma: f64,
}
