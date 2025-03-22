use log::{debug, error};

pub mod solutions;
pub use solutions::{PVTSolution, PVTSolutionType};

#[cfg(feature = "serde")]
use serde::Serialize;

pub(crate) mod apriori;
pub(crate) mod dop;
pub(crate) mod state;

use nalgebra::{base::dimension::U4, DVector, DimName, MatrixXx4};

use anise::prelude::Epoch;

use crate::{
    navigation::{apriori::Apriori, dop::DilutionOfPrecision, state::State},
    prelude::{Bias, Candidate, Config, Duration, Error, IonosphereBias, Signal, SV},
};

/// SV Navigation information
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct SVContribution {
    /// Identitity
    pub sv: SV,
    /// [Signal] being used
    pub signal: Signal,
    /// Orbital state
    pub sv_pos_km: (f64, f64, f64),
    /// Orbital velocity
    pub sv_vel_km_s: (f64, f64, f64),
    /// Elevation from RX position
    pub elevation: f64,
    /// Azimuth from RX position
    pub azimuth: f64,
    /// Relativistic path range is evaluated for each contributor (only once)
    pub relativistic_path_range_m: f64,
    /// Troposphere bias in meters of delay
    pub tropo_bias: Option<f64>,
    /// Ionosphere bias
    pub iono_bias: Option<IonosphereBias>,
    /// Correction to said constellation, expressed as [Duration]
    pub clock_correction: Option<Duration>,
}

/// [Navigation] Filter
pub(crate) struct Navigation {
    /// [Config] preset
    cfg: Config,
    /// Iteration counter
    pub iter: usize,
    /// Filter [State]
    pub state: State,
    /// Measurement vector
    pub b: DVector<f64>,
    /// [SVContribution]s
    pub sv: Vec<SVContribution>,
    /// [DilutionOfPrecision]
    pub dop: DilutionOfPrecision,
}

impl Navigation {
    /// Creates new [Navigation] filter
    /// ## Input
    /// - cfg: [Config] preset
    /// - apriori: [Apriori] input
    /// - candidates: selected [Candidate]s
    /// - size: number of proposal
    /// - bias: [Bias] model implementation
    /// ## Returns
    /// - [Navigation], [Error]
    pub fn new<B: Bias>(
        t: Epoch,
        cfg: &Config,
        apriori: Apriori,
        candidates: &[Candidate],
        size: usize,
        bias: &B,
    ) -> Result<Self, Error> {
        let mut sv = Vec::with_capacity(size);

        let min_size = match cfg.solution {
            PVTSolutionType::PositionVelocityTime => 4,
            PVTSolutionType::TimeOnly => 1,
        };

        if size < min_size {
            return Err(Error::MatrixMinimalDimension);
        }

        let mut j = 0;

        let mut b = match cfg.solution {
            PVTSolutionType::TimeOnly => DVector::<f64>::zeros(std::cmp::max(4, size)),
            _ => DVector::<f64>::zeros(size),
        };

        for i in 0..size {
            let mut contrib = SVContribution::default();
            contrib.sv = candidates[i].sv;

            match candidates[i].vector_contribution(
                t,
                cfg,
                apriori.pos_m,
                apriori.lat_long_alt_deg_deg_km,
                &mut contrib,
                bias,
            ) {
                Ok((b_i, dr_i)) => {
                    b[j] = b_i;

                    if cfg.modeling.relativistic_path_range {
                        debug!(
                            "{}({}) - relativisitic path range: {:.3}m",
                            t, candidates[i].sv, dr_i
                        );
                    }

                    sv.push(contrib);
                    j += 1;
                },
                Err(e) => {
                    error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                },
            }
        }

        if size == 1 && cfg.solution == PVTSolutionType::TimeOnly {
            for r in 1..=3 {
                b[r] = b[0];
            }
        }

        if b.len() < U4::USIZE {
            Err(Error::MatrixMinimalDimension)
        } else {
            let state = State::from_apriori(&apriori).or(Err(Error::NavigationFilterInitError))?;

            debug!("initial state: {:?}", state.pos_m);

            Ok(Self {
                b,
                sv,
                state,
                iter: 0,
                cfg: cfg.clone(),
                dop: DilutionOfPrecision::default(),
            })
        }
    }

    /// Iterates mutable [Navigation] filter.
    pub fn iterate<B: Bias>(
        &mut self,
        t: Epoch,
        cfg: &Config,
        candidates: &[Candidate],
        size: usize,
        bias: &B,
    ) -> Result<bool, Error> {
        let mut j = 0;
        let mut converged = false;

        let mut h = match cfg.solution {
            PVTSolutionType::TimeOnly => MatrixXx4::<f64>::zeros(std::cmp::max(4, size)),
            _ => MatrixXx4::<f64>::zeros(size),
        };

        let min_size = match cfg.solution {
            PVTSolutionType::PositionVelocityTime => 4,
            PVTSolutionType::TimeOnly => 1,
        };

        if size < min_size {
            return Err(Error::MatrixMinimalDimension);
        }

        // form matrix
        for i in 0..size {
            let dr_i = self.sv[i].relativistic_path_range_m;

            match candidates[i].matrix_contribution(&self.cfg, dr_i, self.state.pos_m) {
                Ok((dx, dy, dz)) => {
                    h[(j, 0)] = dx;
                    h[(j, 1)] = dy;
                    h[(j, 2)] = dz;
                    h[(j, 3)] = 1.0;
                    j += 1;
                },
                Err(e) => {
                    error!("({}) does not contribute: {}", candidates[i].sv, e);
                },
            }
        }

        if j < min_size {
            return Err(Error::MatrixMinimalDimension);
        }

        if j == 1 && cfg.solution == PVTSolutionType::TimeOnly {
            for r in 1..=3 {
                h[(r, 0)] = h[(0, 0)];
                h[(r, 1)] = h[(0, 1)];
                h[(r, 2)] = h[(0, 2)];
                h[(r, 3)] = h[(0, 3)];
            }
        }

        // run
        let ht = h.transpose();
        let ht_h = ht.clone() * h.clone();
        let ht_h_inv = ht_h.try_inverse().ok_or(Error::MatrixInversion)?;
        let ht_b = ht * self.b.clone();

        let dx = ht_h_inv * ht_b;

        self.iter += 1;

        self.state.temporal_update(t, dx).map_err(|e| {
            error!("{} - physical error: {}", t, e);
            Error::StateUpdate
        })?;

        self.dop = DilutionOfPrecision::new(&self.state, ht_h_inv);

        let norm = (dx[0].powi(2) + dx[1].powi(2) + dx[2].powi(2)).sqrt();

        // let norm = match cfg.solution {
        //     PVTSolutionType::TimeOnly => {
        //         dx[3]
        //     },
        //     _ => {
        //         (dx[0].powi(2) + dx[1].powi(2) + dx[2].powi(2)).sqrt()
        //     },
        // };

        // update models (if desired)
        if !converged {
            let mut j = 0;
            // does not update the contribution
            let mut dummy = SVContribution::default();
            if self.cfg.solver.filter.model_update {
                for i in 0..size {
                    match candidates[i].vector_contribution(
                        t,
                        &self.cfg,
                        self.state.pos_m,
                        self.state.lat_long_alt_deg_deg_km,
                        &mut dummy,
                        bias,
                    ) {
                        Ok((b_i, _)) => {
                            self.b[j] = b_i;
                            j += 1;
                        },
                        Err(e) => {
                            error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                        },
                    }
                }
            }
        }

        Ok(self.iter == 8)
    }
}

#[cfg(test)]
mod test {
    use super::{DilutionOfPrecision, State};
    use crate::prelude::{Almanac, EARTH_J2000};
    use nalgebra::Matrix4;

    #[test]
    fn test_dop() {
        let almanac = Almanac::until_2035().unwrap();
        let frame = almanac.frame_from_uid(EARTH_J2000).unwrap();

        let state = State {
            frame,
            t: Default::default(),
            clock_dt: Default::default(),
            clock_drift_s_s: 0.0,
            pos_m: (1.0, 2.0, 3.0),
            lat_long_alt_deg_deg_km: (0.0, 0.0, 0.0),
            vel_m_s: (4.0, 5.0, 6.0),
        };

        let matrix = Matrix4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );

        let dop = DilutionOfPrecision::new(&state, matrix);

        assert_eq!(dop.gdop, (1.0_f64 + 6.0_f64 + 11.0_f64 + 16.0_f64).sqrt());
        assert_eq!(dop.tdop, 16.0_f64.sqrt());
    }
}
