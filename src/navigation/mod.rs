use log::{debug, error};

#[cfg(feature = "serde")]
use serde::Serialize;

pub(crate) mod apriori;
pub(crate) mod solutions;

mod dop;
mod kalman;
mod postfit;

pub(crate) mod state;

use nalgebra::{DVector, Matrix4, MatrixXx4, Vector4, U4};

use crate::{
    navigation::{
        apriori::Apriori, dop::DilutionOfPrecision, kalman::Kalman, postfit::PostfitKf,
        state::State,
    },
    prelude::{Bias, Candidate, Config, Duration, Epoch, Error, Frame, IonosphereBias, Signal, SV},
};

pub use solutions::PVTSolution;

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
    /// [Frame]
    frame: Frame,
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
    pub fn new(cfg: &Config, frame: Frame) -> Self {
        Self {
            frame,
            postfit: None,
            cfg: cfg.clone(),
            prev_epoch: None,
            initialized: false,
            sv: Vec::with_capacity(8),
            kalman: Kalman::<U4>::new(),
            state: State::default(),
            dop: DilutionOfPrecision::default(),
        }
    }

    /// Reset [Navigation] filter
    pub fn reset(&mut self) {
        self.sv.clear();
        self.kalman.reset();

        if let Some(postfit) = &mut self.postfit {
            postfit.reset();
        }

        self.prev_epoch = None;
        self.initialized = false;
        self.dop = DilutionOfPrecision::default();
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
            self.kf_iteration(t, candidates, size, bias)?;
        }

        if let Some(denoising) = &self.cfg.solver.postfit_denoising {
            if let Some(postfit) = &mut self.postfit {
                let prev_epoch = self
                    .prev_epoch
                    .expect("internal error: undetermined past epoch");

                let dt = t - prev_epoch;
                let dx = postfit.run(dt, &self.state)?;

                // update state
                self.state
                    .temporal_postfit_update(self.frame, dx)
                    .map_err(|e| {
                        error!(
                            "{} - postfit state update failed with physical error: {}",
                            t, e
                        );
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

    /// Filter first iteration.
    pub fn kf_initialization<B: Bias>(
        &mut self,
        t: Epoch,
        state: &State,
        candidates: &[Candidate],
        size: usize,
        bias: &B,
    ) -> Result<(), Error> {
        let nb_iter = 10; // TODO improve

        self.sv.clear();

        let mut sv = Vec::with_capacity(size);
        let mut b = Vec::<f64>::with_capacity(size);
        let mut h = MatrixXx4::<f64>::zeros(size);

        let mut pending = state.clone();

        // initial measurement vector
        for i in 0..size {
            let mut contrib = SVContribution::default();

            contrib.sv = candidates[i].sv;

            match candidates[i].vector_contribution(
                t,
                &self.cfg,
                pending.pos_m,
                pending.lat_long_alt_deg_deg_km,
                &mut contrib,
                bias,
            ) {
                Ok((b_i, dr_i)) => {
                    b.push(b_i);

                    if self.cfg.modeling.relativistic_path_range {
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

        let b_len = b.len();

        if b_len < 4 {
            return Err(Error::MatrixMinimalDimension);
        }

        // TODO improve this: form vector
        let mut b_vec = DVector::<f64>::from_row_slice(&b);

        // TODO improve this: programmable Q
        let q_diag = Vector4::<f64>::new(0.0, 0.0, 0.0, 100E-6_f64.powi(2)); //TODO
        let q_mat = Matrix4::from_diagonal(&q_diag);

        let mut ht_h_inv = Matrix4::zeros();

        // run
        for _ in 0..nb_iter {
            // form H tile
            for i in 0..b.len() {
                let dr_i = sv[i].relativistic_path_range_m;

                let (dx, dy, dz) =
                    candidates[i].matrix_contribution(&self.cfg, dr_i, pending.pos_m);

                h[(i, 0)] = dx;
                h[(i, 1)] = dy;
                h[(i, 2)] = dz;
                h[(i, 3)] = 1.0;
            }

            // verify correctness
            if h.nrows() != b_len {
                return Err(Error::MatrixDimension);
            }

            // run
            let ht = h.transpose();
            let ht_h = ht.clone() * h.clone();
            ht_h_inv = ht_h.try_inverse().ok_or(Error::MatrixInversion)?;

            let ht_b = ht * b_vec.clone();

            let dx = ht_h_inv * ht_b;

            // update latest DoP
            self.dop = DilutionOfPrecision::new(&pending, ht_h_inv);

            pending.temporal_update(t, self.frame, dx).map_err(|e| {
                error!("{} - state update failed with physical error: {}", t, e);
                Error::StateUpdate
            })?;

            debug!("{} - pending state {}", t, pending);

            // models update
            for i in 0..b_len {
                let mut unused = SVContribution::default();

                match candidates[i].vector_contribution(
                    t,
                    &self.cfg,
                    pending.pos_m,
                    pending.lat_long_alt_deg_deg_km,
                    &mut unused,
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

        // validation
        self.state_validation(t, &pending)?;

        // arm kalman
        self.kalman.initialize(pending.to_vector4(), ht_h_inv);

        self.state = pending;
        self.prev_epoch = Some(t);

        debug!("{} - new state {}", t, self.state);
        debug!("{} - gdop={} tdop={}", t, self.dop.gdop, self.dop.tdop);

        Ok(())
    }

    /// [Kalman] filter iteration
    pub fn kf_iteration<B: Bias>(
        &mut self,
        t: Epoch,
        candidates: &[Candidate],
        size: usize,
        bias: &B,
    ) -> Result<(), Error> {
        panic!("kf run: not yet");

        // let mut pending = self.state;

        // let z_k = pending.to_vector4();

        // // TODO improve: dynamics
        // let f_diag = Vector4::new(1.0, 1.0, 1.0, 1.0);
        // let f_mat = Matrix4::from_diagonal(&f_diag);

        // let (dx, ht_h_inv) = self.kalman.run(z_k, f_mat, h_mat, r_mat)?;

        // pending.temporal_update(t, dx).map_err(|e| {
        //     error!("{} - state update failed with physical error: {}", t, e);
        //     Error::StateUpdate
        // })?;

        // // update
        // self.dop = DilutionOfPrecision::new(&pending, ht_h_inv);

        // // validation
        // self.state_validation(&pending);

        // self.state = pending;
        // self.past_epoch = Some(t);

        // debug!("{} - new state {}", t, self.state);
        // debug!("{} - gdop={} tdop={}", t, self.dop.gdop, self.dop.tdop);
        // Ok(())
    }

    /// Validate pending [State]
    fn state_validation(&self, t: Epoch, pending: &State) -> Result<(), Error> {
        // const n: usize = 4; // x, y, z, dt

        if self.dop.gdop > self.cfg.solver.max_gdop {
            return Err(Error::MaxGdopExceeded);
        }

        // let m = pres.len();

        // let pres = pres.transpose().dot(pres);
        // let denom = pres.len() as f64 - 4.0 - 1.0; /// x, y, z ,dt
        //
        // let chisqr = chisqr(0.001, m-n-1);
        //
        // if pres >= chisqr {
        //     error!("{} - measurement residual test failed! setup is too noisy ({}/{})", t, pres, chisqr);
        // }

        Ok(())
    }
}

#[cfg(test)]
mod test {
    use super::{DilutionOfPrecision, State};
    use crate::prelude::{Almanac, EARTH_J2000};
    use nalgebra::Matrix4;

    #[test]
    fn test_dop() {
        let state = State {
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
