use log::{
    debug,
    //error,
    warn,
};

use std::collections::HashMap;

use crate::{
    ambiguity::Ambiguities,
    candidate::Candidate,
    cfg::Config,
    navigation::SVInput,
    // constants::Constants,
    prelude::{Error, PVTSolutionType, SV},
};

use nalgebra::{
    base::dimension::{U4, U8},
    DVector, MatrixXx4, OMatrix, OVector,
};

// use nyx::cosmic::SPEED_OF_LIGHT_M_S;

/// Navigation Input
#[derive(Debug, Clone)]
pub struct Input {
    /// Measurement vector
    pub y: DVector<f64>,
    /// NAV Matrix
    pub g: OMatrix<f64, U8, U8>,
    /// SV dependent data
    pub sv: HashMap<SV, SVInput>,
}

impl Input {
    /// Forms new Navigation Input
    pub fn new(
        apriori: (f64, f64, f64),
        cfg: &Config,
        cd: &[Candidate],
        w: MatrixXx4<f64>,
        _: &Ambiguities,
    ) -> Result<Self, Error> {
        const MIN_SIZE: usize = 4;

        let mut y = DVector::<f64>::zeros(MIN_SIZE);
        let mut g = MatrixXx4::<f64>::zeros(MIN_SIZE);
        let mut sv = HashMap::<SV, SVInput>::with_capacity(MIN_SIZE);

        // ARP compensation
        let apriori = match cfg.arp_enu {
            Some(_) => {
                //apriori.0 + offset.0,
                //apriori.1 + offset.1,
                //apriori.2 + offset.2,
                warn!("ARP compensation is not feasible yet");
                apriori
            },
            None => apriori,
        };

        let mut j = 0;
        let mut max = match cfg.sol_type {
            PVTSolutionType::TimeOnly => 1,
            _ => 4,
        };
        if cfg.fixed_altitude.is_some() {
            max -= 1;
        }

        for i in 0..cd.len() {
            match cd[i].matrix_contribution(cfg, j, &mut y, &mut g, apriori) {
                Ok(input) => {
                    sv.insert(cd[i].sv, input);
                    g[(4 + j, 4 + j)] = 1.0_f64;
                    y[4 + j] = y[j];

                    j += 1;
                    if j == cd.len() {
                        break;
                    }
                },
                Err(e) => {
                    debug!("{}({}) cannot contribute: {}", cd[i].t, cd[i].sv, e);
                    continue;
                },
            }

            // TODO reestablish phase contribution
            //if j > 3 {
            //    g[(j, j)] = 1.0_f64;

            //    if cfg.method == Method::PPP {
            //        let cmb = cd[i]
            //            .phase_if_combination()
            //            .ok_or(Error::PhaseRangeCombination)?;

            //        let f_1 = cmb.rhs.frequency();
            //        let lambda_j = cmb.lhs.wavelength();
            //        let f_j = cmb.lhs.frequency();

            //        let (lambda_n, lambda_w) = (
            //            SPEED_OF_LIGHT_M_S / (f_1 + f_j),
            //            SPEED_OF_LIGHT_M_S / (f_1 - f_j),
            //        );

            //        let bias = if let Some(ambiguity) = ambiguities.get(&(cd[i].sv, cmb.rhs)) {
            //            let (n_1, n_w) = (ambiguity.n_1, ambiguity.n_w);
            //            let b_c = lambda_n * (n_1 + (lambda_w / lambda_j) * n_w);
            //            debug!(
            //                "{} ({}/{}) b_c: {}",
            //                cd[i].t, cd[i].sv, cmb.rhs, b_c
            //            );
            //            b_c
            //        } else {
            //            error!(
            //                "{} ({}/{}): unresolved ambiguity",
            //                cd[i].t, cd[i].sv, cmb.rhs
            //            );
            //            return Err(Error::UnresolvedAmbiguity);
            //        };

            //        // TODO: conclude windup
            //        let windup = 0.0_f64;

            //        y[i] = cmb.value - rho - models - windup + bias;
            //    }
            //}
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
