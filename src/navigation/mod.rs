pub mod solutions;
pub use solutions::{PVTSolution, PVTSolutionType};

#[cfg(feature = "serde")]
use serde::Deserialize;

use log::debug;
use std::collections::HashMap;

use crate::bias::{
    Bias, IonosphereBias, RuntimeParam as BiasRuntimeParams, TropoModel, TroposphereBias,
};
use crate::candidate::Candidate;
use crate::cfg::Config;
use crate::prelude::{Method, SV};
use crate::Error;
use nyx::cosmic::SPEED_OF_LIGHT;

use nalgebra::{DMatrix, DVector, Matrix4, Matrix4x1, MatrixXx4, Vector3, Vector4};

/// Navigation Filter.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub enum Filter {
    /// None: solver filter completely bypassed. Lighter calculations, no iterative behavior.
    None,
    #[default]
    /// LSQ Filter. Heavy computation, converges slower than Kalman filter.
    LSQ,
    /// Kalman Filter. Heavy+ computations, converges faster than LSQ.
    Kalman,
}

#[derive(Debug, Clone, Default)]
pub struct LSQState {
    pub p: Matrix4<f64>,
    pub x: Matrix4x1<f64>,
}

#[derive(Debug, Clone, Default)]
pub struct KFState {
    pub q: Matrix4<f64>,
    pub p: Matrix4<f64>,
    pub x: Matrix4x1<f64>,
    pub phi: Matrix4<f64>,
}

#[derive(Debug, Clone)]
pub enum FilterState {
    LSQ(LSQState),
    KF(KFState),
}

impl Default for FilterState {
    fn default() -> Self {
        Self::LSQ(Default::default())
    }
}

impl FilterState {
    pub fn lsq(state: LSQState) -> Self {
        Self::LSQ(state)
    }
    pub fn as_lsq(&self) -> Option<&LSQState> {
        match self {
            Self::LSQ(state) => Some(state),
            _ => None,
        }
    }
    pub fn kf(state: KFState) -> Self {
        Self::KF(state)
    }
    pub fn as_kf(&self) -> Option<&KFState> {
        match self {
            Self::KF(state) => Some(state),
            _ => None,
        }
    }
    pub fn estimate(&self) -> Matrix4x1<f64> {
        match self {
            Self::LSQ(state) => state.x,
            Self::KF(state) => state.x,
        }
    }
}

impl Filter {
    fn lsq_resolve(input: &Input, p_state: Option<FilterState>) -> Result<Output, Error> {
        match p_state {
            Some(FilterState::LSQ(p_state)) => {
                let p_1 = p_state.p.try_inverse().ok_or(Error::MatrixInversionError)?;

                let g_prime = input.g.clone().transpose();
                let q = (g_prime.clone() * input.g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let p = g_prime.clone() * input.w.clone() * input.g.clone();
                let p = (p_1 + p).try_inverse().ok_or(Error::MatrixInversionError)?;

                let x =
                    p * (p_1 * p_state.x + (g_prime.clone() * input.w.clone() * input.y.clone()));

                Ok(Output {
                    gdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)] + q[(3, 3)]).sqrt(),
                    pdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)]).sqrt(),
                    tdop: q[(3, 3)].sqrt(),
                    q,
                    state: FilterState::lsq(LSQState { p, x }),
                })
            },
            _ => {
                let g_prime = input.g.clone().transpose();
                let q = (g_prime.clone() * input.g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let p = (g_prime.clone() * input.w.clone() * input.g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let x = p * (g_prime.clone() * input.w.clone() * input.y.clone());
                if x[3].is_nan() {
                    return Err(Error::TimeIsNan);
                }

                Ok(Output {
                    gdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)] + q[(3, 3)]).sqrt(),
                    pdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)]).sqrt(),
                    tdop: q[(3, 3)].sqrt(),
                    q,
                    state: FilterState::lsq(LSQState { p, x }),
                })
            },
        }
    }
    fn kf_resolve(input: &Input, p_state: Option<FilterState>) -> Result<Output, Error> {
        match p_state {
            Some(FilterState::KF(p_state)) => {
                let p_phi = p_state.p.clone() * p_state.phi.clone().transpose();
                let pb_xn = p_state.phi.clone() * p_phi + p_state.q.clone();
                let xb_n = p_state.phi.clone() * p_state.x;

                let pb_xn_inv = pb_xn.try_inverse().ok_or(Error::MatrixInversionError)?;

                let q = input.g.clone().transpose() * input.w.clone() * input.g.clone();
                let p = q + pb_xn_inv.clone();
                let p = p.try_inverse().ok_or(Error::MatrixInversionError)?;

                let y_n = input.g.clone().transpose() * input.w.clone() * input.y.clone();
                let p_yn = pb_xn_inv.clone() * xb_n.clone();
                let x = p * (y_n + p_yn);

                Ok(Output {
                    gdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)] + q[(3, 3)]).sqrt(),
                    pdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)]).sqrt(),
                    tdop: q[(3, 3)].sqrt(),
                    q,
                    state: FilterState::kf(KFState {
                        p,
                        x,
                        q: Matrix4::from_diagonal(&Vector4::new(1.0, 1.0, 1.0, 0.0)),
                        phi: Matrix4::from_diagonal(&Vector4::new(1.0, 1.0, 1.0, 0.0)),
                    }),
                })
            },
            _ => {
                let g_prime = input.g.clone().transpose();
                let q = (g_prime.clone() * input.g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let p = (g_prime.clone() * input.w.clone() * input.g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let x = p * (g_prime.clone() * input.w.clone() * input.y.clone());
                if x[3].is_nan() {
                    return Err(Error::TimeIsNan);
                }

                Ok(Output {
                    gdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)] + q[(3, 3)]).sqrt(),
                    pdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)]).sqrt(),
                    tdop: q[(3, 3)].sqrt(),
                    q,
                    state: FilterState::kf(KFState {
                        p,
                        x,
                        q: Matrix4::from_diagonal(&Vector4::new(1.0, 1.0, 1.0, 0.0)),
                        phi: Matrix4::from_diagonal(&Vector4::new(1.0, 1.0, 1.0, 0.0)),
                    }),
                })
            },
        }
    }
    fn resolve(&self, input: &Input, p_state: Option<FilterState>) -> Result<Output, Error> {
        match self {
            Filter::None => Self::lsq_resolve(input, None),
            Filter::LSQ => Self::lsq_resolve(input, p_state),
            Filter::Kalman => Self::kf_resolve(input, p_state),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct SVInput {
    pub azimuth: f64,
    pub elevation: f64,
    pub iono_bias: Bias,
    pub tropo_bias: Bias,
}

#[derive(Debug, Clone)]
pub struct Input {
    pub y: DVector<f64>,
    pub g: MatrixXx4<f64>,
    pub w: DMatrix<f64>,
    pub sv: HashMap<SV, SVInput>,
}

#[derive(Debug, Clone, Default)]
pub struct Output {
    pub tdop: f64,
    pub gdop: f64,
    pub pdop: f64,
    pub q: Matrix4<f64>,
    pub state: FilterState,
}

impl Input {
    pub fn new(
        apriori: (f64, f64, f64),
        apriori_geo: (f64, f64, f64),
        cfg: &Config,
        cd: &Vec<Candidate>,
        iono_bias: &IonosphereBias,
        tropo_bias: &TroposphereBias,
    ) -> Result<Self, Error> {
        let mut y = DVector::<f64>::zeros(cd.len());
        let mut g = MatrixXx4::<f64>::zeros(cd.len());
        let mut sv = HashMap::<SV, SVInput>::with_capacity(cd.len());

        if cd.len() < 4 {
            return Err(Error::NotEnoughFittingCandidates);
        }

        /*
         * Compensate for ARP (if possible)
         */
        let apriori = match cfg.arp_enu {
            Some(offset) => (
                apriori.0 + offset.0,
                apriori.1 + offset.1,
                apriori.2 + offset.2,
            ),
            None => apriori,
        };

        let (x0, y0, z0) = apriori;

        for (idx, cd) in cd.iter().enumerate() {
            let state = cd.state.ok_or(Error::UnresolvedState)?;
            let clock_corr = cd.clock_corr.to_seconds();
            let (azimuth, elevation) = (state.azimuth, state.elevation);

            let (sv_x, sv_y, sv_z) = (state.position[0], state.position[1], state.position[2]);

            let rho = ((sv_x - x0).powi(2) + (sv_y - y0).powi(2) + (sv_z - z0).powi(2)).sqrt();
            g[(idx, 0)] = (x0 - sv_x) / rho;
            g[(idx, 1)] = (y0 - sv_y) / rho;
            g[(idx, 2)] = (z0 - sv_z) / rho;
            g[(idx, 3)] = 1.0_f64;

            let mut sv_input = SVInput::default();
            sv_input.azimuth = azimuth;
            sv_input.elevation = elevation;

            let mut models = 0.0_f64;

            if cfg.modeling.sv_clock_bias {
                models -= clock_corr * SPEED_OF_LIGHT;
            }
            if let Some(delay) = cfg.externalref_delay {
                models -= delay * SPEED_OF_LIGHT;
            }

            //TODO add phase for true PPP
            let code = match cfg.method {
                Method::SPP => cd.prefered_pseudorange().ok_or(Error::MissingPseudoRange)?,
                Method::CodePPP => cd
                    .pseudorange_combination()
                    .ok_or(Error::PseudoRangeCombination)?,
            };

            let (pr, frequency) = (code.value, code.frequency);

            // frequency dependent delay
            for delay in &cfg.int_delay {
                if delay.frequency == frequency {
                    models += delay.delay * SPEED_OF_LIGHT;
                }
            }

            /*
             * IONO + TROPO biases
             */
            let rtm = BiasRuntimeParams {
                t: cd.t,
                elevation,
                azimuth,
                frequency,
                apriori_geo,
            };

            /*
             * TROPO
             */
            if cfg.modeling.tropo_delay {
                if tropo_bias.needs_modeling() {
                    let bias = TroposphereBias::model(TropoModel::Niel, &rtm);
                    debug!("{:?} : modeled tropo delay {:.3E}[m]", cd.t, bias);
                    models += bias;
                    sv_input.tropo_bias = Bias::modeled(bias);
                } else if let Some(bias) = tropo_bias.bias(&rtm) {
                    debug!("{:?} : measured tropo delay {:.3E}[m]", cd.t, bias);
                    models += bias;
                    sv_input.tropo_bias = Bias::measured(bias);
                }
            }
            /*
             * IONO
             */
            if cfg.method == Method::SPP {
                if cfg.modeling.iono_delay {
                    if let Some(bias) = iono_bias.bias(&rtm) {
                        debug!(
                            "{:?} : modeled iono delay (f={:.3E}Hz) {:.3E}[m]",
                            cd.t, rtm.frequency, bias
                        );
                        models += bias;
                        sv_input.iono_bias = Bias::modeled(bias);
                    }
                }
            }

            y[idx] = pr - rho - models;
            sv.insert(cd.sv, sv_input);
        }

        let w = cfg.solver.weight_matrix(
            4, //TODO
            sv.values().map(|sv| sv.elevation).collect(),
        );
        debug!("y: {} g: {}, w: {}", y, g, w);
        Ok(Self { y, g, w, sv })
    }
}

#[derive(Debug, Clone)]
pub struct Navigation {
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
