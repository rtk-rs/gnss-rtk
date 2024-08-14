pub mod solutions;
pub use solutions::{InvalidationCause, PVTSolution, PVTSolutionType};

mod filter;

pub use filter::Filter;
pub(crate) use filter::FilterState;

use log::{debug, error, warn};
use std::collections::HashMap;

use crate::{
    ambiguity::Ambiguities,
    candidate::Candidate,
    cfg::Config,
    constants::Constants,
    prelude::{Error, IonosphereBias, Method, Orbit, SV},
};

use nalgebra::{
    base::dimension::{U4, U8},
    OMatrix, OVector,
};

use nyx::cosmic::SPEED_OF_LIGHT_M_S;

/// SV Navigation information
#[derive(Debug, Clone, Default)]
pub struct SVInput {
    /// Possible [Orbit] state
    pub orbit: Option<Orbit>,
    /// Troposphere bias in meters of delay
    pub tropo_bias: Option<f64>,
    /// Ionosphere bias
    pub iono_bias: Option<IonosphereBias>,
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
pub(crate) struct Output {
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
        apriori_km: (f64, f64, f64),
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
        let apriori_km = match cfg.arp_enu {
            Some(offset) => {
                //apriori.0 + offset.0,
                //apriori.1 + offset.1,
                //apriori.2 + offset.2,
                warn!("ARP compensation is not feasible yet");
                apriori_km
            },
            None => apriori_km,
        };

        let (x0_km, y0_km, z0_km) = apriori_km;

        for i in 0..8 {
            let mut sv_input = SVInput::default();

            let index = if i >= cd.len() {
                if cfg.sol_type == PVTSolutionType::TimeOnly {
                    0
                } else {
                    i - cd.len()
                }
            } else {
                i
            };

            let orbit = cd[index].orbit.ok_or(Error::UnresolvedState)?;
            let clock_corr = cd[index].clock_corr.duration.to_seconds();
            sv_input.orbit = Some(orbit);

            let (sv_x_km, sv_y_km, sv_z_km) =
                (orbit.radius_km.x, orbit.radius_km.y, orbit.radius_km.z);

            let mut rho =
                ((sv_x_km - x0_km).powi(2) + (sv_y_km - y0_km).powi(2) + (sv_z_km - z0_km).powi(2))
                    .sqrt();

            if cfg.modeling.relativistic_path_range {
                let mu = Constants::EARTH_GRAVITATION;
                let r_sat = ((orbit.radius_km.x * 1.0E3).powi(2)
                    + (orbit.radius_km.y * 1.0E3).powi(2)
                    + (orbit.radius_km.z * 1.0E3).powi(2))
                .sqrt();
                let r_0 =
                    ((x0_km * 1.0E3).powi(2) + (y0_km * 1.0E3).powi(2) + (z0_km * 1.0E3).powi(2))
                        .sqrt();
                let r_sat_0 = r_0 - r_sat;
                let dr = 2.0 * mu / SPEED_OF_LIGHT_M_S / SPEED_OF_LIGHT_M_S
                    * ((r_sat + r_0 + r_sat_0) / (r_sat + r_0 - r_sat_0)).ln();
                debug!(
                    "{}({}) relativistic path range {:.3E}m",
                    cd[index].t, cd[index].sv, dr
                );
                rho += dr;
            }

            let (x_i, y_i, z_i) = (
                (x0_km - sv_x_km) / rho,
                (y0_km - sv_y_km) / rho,
                (z0_km - sv_z_km) / rho,
            );

            g[(i, 0)] = x_i;
            g[(i, 1)] = y_i;
            g[(i, 2)] = z_i;
            g[(i, 3)] = 1.0_f64;

            let mut models = 0.0_f64;

            if cfg.modeling.sv_clock_bias {
                models -= clock_corr * SPEED_OF_LIGHT_M_S;
            }
            if let Some(delay) = cfg.externalref_delay {
                models -= delay * SPEED_OF_LIGHT_M_S;
            }

            let (pr, frequency) = match cfg.method {
                Method::SPP => {
                    let pr = cd[index]
                        .prefered_pseudorange()
                        .ok_or(Error::MissingPseudoRange)?;
                    (pr.pseudo.unwrap(), pr.carrier.frequency())
                },
                Method::CPP | Method::PPP => {
                    let pr = cd[index]
                        .code_if_combination()
                        .ok_or(Error::PseudoRangeCombination)?;
                    (pr.value, pr.rhs.frequency())
                },
            };

            // frequency dependent delay
            for delay in &cfg.int_delay {
                if delay.frequency == frequency {
                    models += delay.delay * SPEED_OF_LIGHT_M_S;
                }
            }

            /*
             * TROPO
             */
            if cfg.modeling.tropo_delay {
                models += cd[index].tropo_bias;
                sv_input.tropo_bias = Some(cd[index].tropo_bias);
            }

            /*
             * IONO
             */
            if cfg.modeling.iono_delay {
                models += cd[index].iono_bias;
                if cfg.method == Method::SPP {
                    sv_input.iono_bias = Some(IonosphereBias::modeled(cd[index].iono_bias));
                } else {
                    sv_input.iono_bias = Some(IonosphereBias::measured(cd[index].iono_bias));
                }
            }

            y[i] = pr - rho - models;

            if i > 3 {
                g[(i, i)] = 1.0_f64;

                if cfg.method == Method::PPP {
                    let cmb = cd[index]
                        .phase_if_combination()
                        .ok_or(Error::PhaseRangeCombination)?;

                    let f_1 = cmb.rhs.frequency();
                    let lambda_j = cmb.lhs.wavelength();
                    let f_j = cmb.lhs.frequency();

                    let (lambda_n, lambda_w) = (
                        SPEED_OF_LIGHT_M_S / (f_1 + f_j),
                        SPEED_OF_LIGHT_M_S / (f_1 - f_j),
                    );

                    let bias = if let Some(ambiguity) = ambiguities.get(&(cd[index].sv, cmb.rhs)) {
                        let (n_1, n_w) = (ambiguity.n_1, ambiguity.n_w);
                        let b_c = lambda_n * (n_1 + (lambda_w / lambda_j) * n_w);
                        debug!(
                            "{} ({}/{}) b_c: {}",
                            cd[index].t, cd[index].sv, cmb.rhs, b_c
                        );
                        b_c
                    } else {
                        error!(
                            "{} ({}/{}): unresolved ambiguity",
                            cd[index].t, cd[index].sv, cmb.rhs
                        );
                        return Err(Error::UnresolvedAmbiguity);
                    };

                    // TODO: conclude windup
                    let windup = 0.0_f64;

                    y[i] = cmb.value - rho - models - windup + bias;
                }
            }

            if i < cd.len() {
                sv.insert(cd[i].sv, sv_input);
            }
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
