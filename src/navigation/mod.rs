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
    filter_state: Option<FilterState>,
}

impl Navigation {
    /// Create new [Navigation] filter with desired [Filter] type
    pub fn new(filter: Filter) -> Self {
        Self {
            filter,
            filter_state: None,
        }
    }

    /// Reset [Navigation] filter
    pub fn reset(&mut self) {
        self.filter_state = None;
    }

    /// Resolve using provided input
    pub fn resolve(
        &self,
        cfg: &Config,
        apriori: (f64, f64, f64),
        cd: &[Candidate],
        w: MatrixXx4<f64>,
        ambiguities: &Ambiguities,
    ) -> Result<Output, Error> {
        let input = Input::new(cfg, apriori, cd, w, amgiguities);
        let out = self.filter.resolve(input, self.filter_state.clone())?;
        self.pending = out.clone();
        Ok(out)
    }

    //pub fn validate(&mut self) {
    //    self.filter_state = Some(self.pending.state.clone());
    //}
}
