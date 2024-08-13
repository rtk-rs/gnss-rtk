pub mod solutions;
pub use solutions::{InvalidationCause, PVTSolution, PVTSolutionType};

mod filter;

pub use filter::{Filter, FilterState};

use log::{debug, error};
use std::collections::HashMap;

use crate::{
    ambiguity::Ambiguities,
    candidate::Candidate,
    cfg::Config,
    constants::Constants,
    prelude::{Duration, Error, IonosphereBias, Method, SV},
};

use nalgebra::{
    base::dimension::{U4, U8},
    OMatrix, OVector,
};

use nyx::cosmic::SPEED_OF_LIGHT_M_S;

/// SV Navigation information
#[derive(Debug, Clone, Default)]
pub struct SVInput {
    /// SV azimuth angle in degrees
    pub azimuth: f64,
    /// SV elevation angle in degrees
    pub elevation: f64,
    /// Troposphere bias in meters of delay
    pub tropo_bias: Option<f64>,
    /// Ionosphere bias
    pub iono_bias: Option<IonosphereBias>,
    /// Correction to said constellation, expressed as [Duration]
    pub clock_correction: Option<Duration>,
}

/// Navigation Input
#[derive(Debug, Clone)]
pub struct Input {
    /// Measurement vector
    pub y: OVector<f64, U8>,
    /// NAV Matrix
    pub g: OMatrix<f64, U8, U8>,
    /// Weight Diagonal Matrix
    pub w: OMatrix<f64, U8, U8>,
    /// SV dependent data
    pub sv: HashMap<SV, SVInput>,
}

/// Navigation Output
#[derive(Debug, Clone, Default)]
pub struct Output {
    /// Time Dilution of Precision
    pub tdop: f64,
    /// Geometric Dilution of Precision
    pub gdop: f64,
    /// Position Dilution of Precision
    pub pdop: f64,
    /// Q covariance matrix
    pub q: OMatrix<f64, U8, U8>,
    /// Filter state
    pub state: FilterState,
}

impl Output {
    pub(crate) fn q_covar4x4(&self) -> OMatrix<f64, U4, U4> {
        OMatrix::<f64, U4, U4>::new(
            self.q[(0, 0)],
            self.q[(0, 1)],
            self.q[(0, 2)],
            self.q[(0, 3)],
            self.q[(1, 0)],
            self.q[(1, 1)],
            self.q[(1, 2)],
            self.q[(1, 3)],
            self.q[(2, 0)],
            self.q[(2, 1)],
            self.q[(2, 2)],
            self.q[(2, 3)],
            self.q[(3, 0)],
            self.q[(3, 1)],
            self.q[(3, 2)],
            self.q[(3, 3)],
        )
    }
}

impl Input {
    /// Forms new Navigation Input
    pub fn new(
        apriori: (f64, f64, f64),
        cfg: &Config,
        cd: &[Candidate],
        w: OMatrix<f64, U8, U8>,
        ambiguities: &Ambiguities,
    ) -> Result<Self, Error> {
        let mut y = OVector::<f64, U8>::zeros();
        let mut g = OMatrix::<f64, U8, U8>::zeros();
        let mut sv = HashMap::<SV, SVInput>::with_capacity(cd.len());
        /*
         * Compensate for ARP (if possible)
         */
        let apriori_ecef_m = match cfg.arp_enu {
            Some(offset) => (
                apriori.0 + offset.0,
                apriori.1 + offset.1,
                apriori.2 + offset.2,
            ),
            None => apriori,
        };

        // TODO: remove 8x8 size limitation
        let mut i = 0;
        let mut max = match cfg.sol_type {
            PVTSolutionType::TimeOnly => 1,
            _ => 4,
        };
        if cfg.fixed_altitude.is_some() {
            max -= 1;
        }
        while i < max {
            let mut sv_input = SVInput::default();

            match cd[i].matrix_contribution(cfg, i, &mut y, &mut g, apriori_ecef_m) {
                Ok(input) => {
                    sv.insert(cd[i].sv, input);
                },
                Err(e) => {
                    debug!("{}({}): cannot contribute - {}", cd[i].t, cd[i].sv, e);
                    continue;
                },
            }

            // TODO reestablish phase contribution
            g[(4 + i, 4 + i)] = 1.0_f64;
            y[4 + i] = y[i];
            //TODO phase contrib
            //if i > 3 {
            //    g[(i, i)] = 1.0_f64;

            //    if cfg.method == Method::PPP {
            //        let cmb = cd[index]
            //            .phase_if_combination()
            //            .ok_or(Error::PhaseRangeCombination)?;

            //        let f_1 = cmb.rhs.frequency();
            //        let lambda_j = cmb.lhs.wavelength();
            //        let f_j = cmb.lhs.frequency();

            //        let (lambda_n, lambda_w) = (
            //            SPEED_OF_LIGHT_M_S / (f_1 + f_j),
            //            SPEED_OF_LIGHT_M_S / (f_1 - f_j),
            //        );

            //        let bias = if let Some(ambiguity) = ambiguities.get(&(cd[index].sv, cmb.rhs)) {
            //            let (n_1, n_w) = (ambiguity.n_1, ambiguity.n_w);
            //            let b_c = lambda_n * (n_1 + (lambda_w / lambda_j) * n_w);
            //            debug!(
            //                "{} ({}/{}) b_c: {}",
            //                cd[index].t, cd[index].sv, cmb.rhs, b_c
            //            );
            //            b_c
            //        } else {
            //            error!(
            //                "{} ({}/{}): unresolved ambiguity",
            //                cd[index].t, cd[index].sv, cmb.rhs
            //            );
            //            return Err(Error::UnresolvedAmbiguity);
            //        };

            //        // TODO: conclude windup
            //        let windup = 0.0_f64;

            //        y[i] = cmb.value - rho - models - windup + bias;
            //    }
            //}
            i += 1;
        }

        // TODO: improve matrix formation
        if max == 3 {
            y[3] = y[2];
            g[(3, 3)] = 1.0_f64;
            y[4 + 3] = y[2];
            g[(4 + 3, 4 + 3)] = 1.0_f64;
        }

        // TODO: improve matrix formation
        if max == 1 {
            y[1] = y[0];
            y[2] = y[0];
            y[3] = y[0];

            g[(1, 1)] = 1.0_f64;
            g[(2, 2)] = 1.0_f64;
            g[(3, 3)] = 1.0_f64;

            y[4 + 1] = y[0];
            y[4 + 2] = y[0];
            y[4 + 3] = y[0];

            g[(4 + 1, 4 + 1)] = 1.0_f64;
            g[(4 + 2, 4 + 2)] = 1.0_f64;
            g[(4 + 3, 4 + 3)] = 1.0_f64;
        }

        debug!("y: {} g: {}, w: {}", y, g, w);
        Ok(Self { y, g, w, sv })
    }
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
    pub fn resolve(&mut self, input: &Input) -> Result<Output, Error> {
        let out = self.filter.resolve(input, self.filter_state.clone())?;
        self.pending = out.clone();
        Ok(out)
    }
    pub fn validate(&mut self) {
        self.filter_state = Some(self.pending.state.clone());
    }
}
