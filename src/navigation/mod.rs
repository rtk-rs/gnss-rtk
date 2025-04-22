use log::{debug, error};

pub mod solutions;
pub use solutions::PVTSolution;

#[cfg(feature = "serde")]
use serde::Serialize;

pub(crate) mod apriori;
pub(crate) mod dop;
pub(crate) mod kalman;
pub(crate) mod postfit;
pub(crate) mod state;

use nalgebra::{DVector, Matrix4, MatrixXx4, Vector4, U4};

use anise::prelude::Epoch;

use crate::{
    navigation::{apriori::Apriori, dop::DilutionOfPrecision, kalman::Kalman, state::State},
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

/// [Navigation] Solver
pub(crate) struct Navigation {
    /// [Config] preset
    cfg: Config,
    /// [Kalman]
    kalman: Kalman<U4>,
    /// Postfit [Kalman]
    postfit: Option<PostfitKf>,
    /// True if this filter has been initialized
    pub initialized: bool,
    /// Navigation [State]
    pub state: State,
    /// Previous Epoch. Null on first attempt.
    prev_epoch: Option<Epoch>,
    /// [SVContribution]s
    pub sv: Vec<SVContribution>,
    /// [DilutionOfPrecision]
    pub dop: DilutionOfPrecision,
}

impl Navigation {
    /// Creates new [Navigation] solver.
    pub fn new(cfg: &Config, apriori: &Apriori) -> Self {
        Self {
            postfit: None,
            cfg: cfg.clone(),
            prev_epoch: None,
            initialized: false,
            sv: Vec::with_capacity(8),
            kalman: Kalman::<U4>::default(),
            state: State::from_apriori(state),
            dop: DilutionOfPrecision::default(),
        }
    }

    /// Reset [Navigation] filter
    pub fn reset(&mut self) {
        self.initialized = false;
        self.sv.clear();
        self.prev_epoch = None;
        self.dop = DilutionOfPrecision::default();

        self.kalman.reset();

        if let Some(postfit) = &mut self.postfit {
            postfit.reset();
        }
    }

    /// Iterates mutable [Navigation] filter.
    pub fn solving<B: Bias>(
        &mut self,
        t: Epoch,
        past_state: &State,
        candidates: &[Candidate],
        size: usize,
        bias: &B,
    ) -> Result<(), Error> {
        if !self.kalman.initialized {
            self.kf_initialization(t, past_state, candidates, size, bias)?;
        } else {
            self.kf_iteration(t, past_state, candidates, size, bias)?;
        }

        if let Some(denoising) = &self.cfg.solver.postfit_denoising {
            if let Some(postfit) = &mut self.postfit {
                let prev_epoch = self
                    .prev_epoch
                    .expect("internal error: undetermined past epoch");

                let dt = t - prev_epoch;
                let dx = postfit_kf.run(dt, &self.state)?;

                // update state
                state.temporal_update(t, dx).map_err(|e| {
                    error!("{} - state update failed with physical error: {}", t, e);
                    Error::StateUpdate
                })?;
            } else {
                self.postfit = Some(PostfitKf::new(
                    &self.state,
                    1.0 / denoising,
                    1.0 / denoising,
                    1.0,
                    1.0,
                ));
            }
        }

        self.prev_epoch = Some(t);

        Ok(())
    }

    /// [Kalman] initialization.
    pub fn kf_initialization<B: Bias>(
        &mut self,
        t: Epoch,
        past_state: &State,
        candidates: &[Candidate],
        size: usize,
        bias: &B,
    ) -> Result<(), Error> {
        let nb_iter = 10; // TODO improve

        self.sv.clear();

        let mut sv = Vec::with_capacity(size);
        let mut b = Vec::<f64>::with_capacity(size);
        let mut h = MatrixXx4::<f64>::zeros(len);

        // initial measurement vector
        for i in 0..size {
            let mut contrib = SVContribution::default();

            contrib.sv = candidates[i].sv;

            match candidates[i].vector_contribution(
                t,
                cfg,
                past_state.pos_m,
                past_state.lat_long_alt_deg_deg_km,
                &mut contrib,
                bias,
            ) {
                Ok((b_i, dr_i)) => {
                    b.push(b_i);

                    if cfg.modeling.relativistic_path_range {
                        debug!(
                            "{}({}) - relativistic path range: {:.3}m",
                            t, candidates[i].sv, dr_i
                        );
                    }

                    sv.push(contrib);
                },
                Err(e) => {
                    error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                },
            }
        }

        let len = b.len();

        if len < 4 {
            return Err(Error::MatrixMinimalDimension);
        }

        // TODO improve this
        // form measurement vector
        let mut b_vec = DVector::<f64>::zeros(len);
        for i in 0..len {
            b_vec[i] = b[i];
        }

        // TODO improve this
        let q_diag = Vector4::<f64>::new(0.0, 0.0, 0.0, 100E-6_f64.powi(2));
        let q_mat = Matrix4::from_diagonal(&q_diag);

        // run
        let mut state = past_state;

        for _ in 0..nb_iter {
            // form H tile
            for i in 0..b.len() {
                let dr_i = sv[i].relativistic_path_range_m;

                let (dx, dy, dz) = candidates[i].matrix_contribution(&self.cfg, dr_i, state.pos_m);

                h[(i, 0)] = dx;
                h[(i, 1)] = dy;
                h[(i, 2)] = dz;
                h[(i, 3)] = 1.0;
            }

            // verify correctness
            if h.nrows() != b.len() {
                return Err(Error::MatrixDimension);
            }

            // run
            let ht = h.transpose();
            let ht_h = ht.clone() * h.clone();
            let ht_h_inv = ht_h.try_inverse().ok_or(Error::MatrixInversion)?;
            let ht_b = ht * b_vec.clone();

            let dx = ht_h_inv * ht_b;

            // update latest DoP
            self.dop = DilutionOfPrecision::new(&state, ht_h_inv);

            state.temporal_update(t, dx).map_err(|e| {
                error!("{} - state update failed with physical error: {}", t, e);
                Error::StateUpdate
            })?;

            // TODO improve convergence determination
            // let norm = (dx[0].powi(2) + dx[1].powi(2) + dx[2].powi(2)).sqrt();

            // update of the measurement models
            for i in 0..b.len() {
                let mut unused = SVContribution::default();

                match candidates[i].vector_contribution(
                    t,
                    &self.cfg,
                    state.pos_m,
                    state.lat_long_alt_deg_deg_km,
                    &mut dummy,
                    bias,
                ) {
                    Ok((b_i, _)) => {
                        b_vec[i] = b_i;
                    },
                    Err(e) => {
                        error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                    },
                }
            }
        }

        self.kalman.initialize(&state.to_vector4(), ht_h_inv);
        Ok(())
    }

    /// [Kalman] filter iteration
    pub fn kf_iteration<B: Bias>(
        &mut self,
        t: Epoch,
        past_state: &State,
        candidates: &[Candidate],
        size: usize,
        bias: &B,
    ) -> Result<(), Error> {
        let z_k = state_state.vector();

        let f_diag = Vector4::new(1.0, 1.0, 1.0, 1.0);
        let f_mat = Matrix4::from_diagonal(&f_diag);

        let dx = self.kalman.run(z_k, f_mat, h_mat, r_mat)?;

        self.state.temporal_update(t, dx);

        Ok(())
    }

    /// Reset [Navigation] filter
    pub fn reset(&mut self) {
        self.prev_epoch = None;
        self.sv.clear();
        self.kalman.reset();
        if let Some(postfit) = &mut self.postfit {
            postfit.reset();
        }
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
