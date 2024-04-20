pub mod solutions;
pub use solutions::{PVTSolution, PVTSolutionType};
use std::collections::HashMap;

use log::debug;

use crate::bias::{
    Bias, IonosphereBias, RuntimeParam as BiasRuntimeParams, TropoModel, TroposphereBias,
};
use crate::candidate::Candidate;
use crate::cfg::Config;
use crate::prelude::{Method, SV};
use crate::Error;
use nyx::cosmic::SPEED_OF_LIGHT;

use nalgebra::{DVector, Matrix3, Matrix4, Matrix4x1, MatrixXx4, Vector3};

#[derive(Debug, Clone, Default)]
pub enum Filter {
    NoFilter,
    #[default]
    LSQ,
}

#[derive(Debug, Clone, Default)]
struct FilterState {
    pub p: Matrix4<f64>,
    pub x: Matrix4x1<f64>,
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
    pub sv: HashMap<SV, SVInput>,
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
                //t: cd.t,
                t: cd.t_tx, //TODO
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
        debug!("y: {} g: {}", y, g);
        Ok(Self { y, g, sv })
    }
}

#[derive(Debug, Clone)]
pub struct Navigation {
    filter: Filter,
    filter_state: FilterState,
}

impl Navigation {}
