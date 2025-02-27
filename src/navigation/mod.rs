pub mod solutions;
pub use solutions::{InvalidationCause, PVTSolution, PVTSolutionType};

mod filter;
mod input;
mod output;

pub(crate) use input::Input;
pub(crate) use output::Output;

pub use filter::Filter;
pub(crate) use filter::FilterState;

use crate::prelude::{
    Duration,
    Error,
    IonosphereBias, //Method,
    Orbit,
};

/// SV Navigation information
#[derive(Debug, Clone, Default)]
pub struct SVInput {
    /// Possible [Orbit] state
    pub orbit: Option<Orbit>,
    /// Elevation from RX position
    pub elevation: f64,
    /// Azimuth from RX position
    pub azimuth: f64,
    /// Troposphere bias in meters of delay
    pub tropo_bias: Option<f64>,
    /// Ionosphere bias
    pub iono_bias: Option<IonosphereBias>,
    /// Correction to said constellation, expressed as [Duration]
    pub clock_correction: Option<Duration>,
}

#[derive(Debug, Clone)]
pub(crate) struct Navigation {
    filter: Filter,
    pending: Output,
    filter_state: Option<FilterState>,
}

impl Navigation {
    pub fn new(filter: Filter) -> Self {
        Self {
            filter,
            filter_state: None,
            pending: Default::default(),
        }
    }
    pub fn reset(&mut self) {
        self.filter_state = None;
        self.pending = Default::default();
    }
    pub fn resolve(&mut self, input: &Input) -> Result<Output, Error> {
        let out = self.filter.resolve(input, self.filter_state.clone())?;
        self.pending = out.clone();
        Ok(out)
    }
    pub fn validate(&mut self) {
        self.filter_state = Some(self.pending.state.clone());
    }
}
