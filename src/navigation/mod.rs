use log::{debug, error};

#[cfg(feature = "serde")]
use serde::Serialize;

pub(crate) mod apriori;
pub(crate) mod solutions;

mod dop;
mod kalman;
// mod postfit;

pub(crate) mod state;

use nalgebra::{
    DVector, DefaultAllocator, DimName, Matrix4, Matrix4xX, MatrixXx4, OVector, Vector4, U4,
};

use crate::{
    navigation::{
        apriori::Apriori,
        dop::DilutionOfPrecision,
        kalman::{Kalman, KfEstimate},
        // postfit::PostfitKf,
        state::State,
    },
    prelude::{Bias, Candidate, Config, Duration, Epoch, Error, Frame, IonosphereBias, Signal, SV},
    time::{AbsoluteTime, Time},
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

    // /// Postfit [Kalman]
    // postfit: Option<PostfitKf>,
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
    /// ## Input
    /// - cfg: [Config] preset
    /// - apriori: [Apriori] input
    /// - candidates: selected [Candidate]s
    /// - size: number of proposal
    /// - bias: [Bias] model implementation
    /// ## Returns
    /// - [Navigation], [Error]
    pub fn new(cfg: &Config, frame: Frame) -> Self {
        Self {
            frame,
            // postfit: None,
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

        // if let Some(postfit) = &mut self.postfit {
        //     postfit.reset();
        // }

        self.prev_epoch = None;
        self.initialized = false;
        self.dop = DilutionOfPrecision::default();
    }

    /// Iterates mutable [Navigation] filter.
    pub fn solving<B: Bias, T: Time>(
        &mut self,
        t: Epoch,
        past_state: &State,
        candidates: &[Candidate],
        size: usize,
        bias: &B,
        absolute_time: &AbsoluteTime<T>,
    ) -> Result<(), Error> {
        if !self.kalman.initialized {
            self.kf_initialization(t, past_state, candidates, size, bias, absolute_time)?;
        } else {
            self.kf_run(t, candidates, size, bias, absolute_time)?;
        }

        if let Some(denoising) = &self.cfg.solver.postfit_denoising {
            // if let Some(postfit) = &mut self.postfit {
            //     let prev_epoch = self
            //         .prev_epoch
            //         .expect("internal error: undetermined past epoch");

            //     let dt = t - prev_epoch;
            //     let dx = postfit.run(&self.state, dt)?;

            //     // update state
            //     self.state.postfit_update(self.frame, dx).map_err(|e| {
            //         error!(
            //             "{} - postfit state update failed with physical error: {}",
            //             t, e
            //         );
            //         Error::StateUpdate
            //     })?;
            // } else {
            //     self.postfit = Some(PostfitKf::new(
            //         &self.state,
            //         1.0 / denoising,
            //         1.0 / denoising,
            //         1.0,
            //         1.0,
            //     ));
            // }
        }

        self.prev_epoch = Some(t);

        Ok(())
    }

    /// Filter first iteration.
    pub fn kf_initialization<B: Bias, T: Time>(
        &mut self,
        t: Epoch,
        state: &State,
        candidates: &[Candidate],
        size: usize,
        bias: &B,
        absolute_time: &AbsoluteTime<T>,
    ) -> Result<(), Error> {
        let nb_iter = 10;

        self.sv.clear();

        let mut pending = state.clone();
        let mut indexes = Vec::<usize>::with_capacity(size);

        let mut y_k = Vec::<f64>::with_capacity(size);
        let mut sv = Vec::with_capacity(size);

        // measurement
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
                absolute_time,
            ) {
                Ok((y_i, dr_i)) => {
                    y_k.push(y_i);
                    indexes.push(i);

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

        let y_len = y_k.len();
        if y_len < 4 {
            return Err(Error::MatrixMinimalDimension);
        }

        let mut y_k = DVector::<f64>::from_row_slice(&y_k);
        let mut g_k = MatrixXx4::<f64>::zeros(y_len);

        let r_k = MatrixXx4::<f64>::identity(y_len); // TODO
        let w_k = MatrixXx4::<f64>::identity(y_len); // TODO

        let mut x_k = Vector4::zeros();
        let mut p_k = Matrix4::zeros();

        let mut gt_w_g_inv = Matrix4::zeros();
        let mut gt_g_inv = Matrix4::zeros();

        // run
        for _ in 0..nb_iter {
            // form g_k
            for (i, index) in indexes.iter().enumerate() {
                let dr_i = sv[*index].relativistic_path_range_m;

                let (dx, dy, dz) =
                    candidates[*index].matrix_contribution(&self.cfg, dr_i, pending.pos_m);

                g_k[(i, 0)] = dx;
                g_k[(i, 1)] = dy;
                g_k[(i, 2)] = dz;
                g_k[(i, 3)] = 1.0;
            }

            // verify correctness
            if g_k.nrows() != y_len {
                return Err(Error::MatrixDimension);
            }

            // run
            let gt = g_k.transpose();

            let gt_g = gt.clone() * g_k.clone();
            let gt_w_g = gt_g * w_k.clone();

            gt_g_inv = gt_g.try_inverse().ok_or(Error::MatrixInversion)?;

            let gt_w_b = gt * w_k.clone() * y_k.clone();

            x_k = gt_w_g_inv.clone() * gt_w_b;
            p_k = gt_w_g.try_inverse().ok_or(Error::MatrixInversion)?;

            pending.update(t, self.frame, x_k).map_err(|e| {
                error!("{} - state update failed with physical error: {}", t, e);
                Error::StateUpdate
            })?;

            // update latest DoP
            self.dop = DilutionOfPrecision::new(&pending, gt_g_inv);

            debug!("{} - pending state {}", t, pending);

            // models update
            for (i, index) in indexes.iter().enumerate() {
                let mut unused = SVContribution::default();

                match candidates[*index].vector_contribution(
                    t,
                    &self.cfg,
                    pending.pos_m,
                    pending.lat_long_alt_deg_deg_km,
                    &mut unused,
                    bias,
                    absolute_time,
                ) {
                    Ok((y_i, _)) => {
                        y_k[i] = y_i;
                    },
                    Err(e) => {
                        error!(
                            "{}({}) - cannot contribute: {}",
                            t, candidates[*index].sv, e
                        );
                    },
                }
            }
        }

        // validation
        self.state_validation(t, &pending)?;

        // arm kalman
        let initial_state = KfEstimate::new(x_k, p_k);

        // TODO only for static positioning
        let f_diag = Vector4::new(1.0, 1.0, 1.0, 1.0);
        let f_k = Matrix4::from_diagonal(&f_diag);

        // TODO
        let q_diag = Vector4::new(0.0, 0.0, 0.0, 0.0);
        let q_k = Matrix4::from_diagonal(&q_diag);

        self.kalman.initialize(initial_state, f_k, q_k);

        self.state = pending;
        self.prev_epoch = Some(t);

        debug!("{} - new state {}", t, self.state);
        debug!("{} - gdop={} tdop={}", t, self.dop.gdop, self.dop.tdop);

        Ok(())
    }

    /// [Kalman] filter run
    pub fn kf_run<B: Bias, T: Time>(
        &mut self,
        t: Epoch,
        candidates: &[Candidate],
        size: usize,
        bias: &B,
        absolute_time: &AbsoluteTime<T>,
    ) -> Result<(), Error> {
        self.sv.clear();

        let mut pending = self.state.clone();
        let mut indexes = Vec::<usize>::with_capacity(size);

        let mut y_k = Vec::<f64>::with_capacity(size);
        let mut sv = Vec::with_capacity(size);

        // measurement
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
                absolute_time,
            ) {
                Ok((y_i, dr_i)) => {
                    y_k.push(y_i);
                    indexes.push(i);

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

        let y_len = y_k.len();
        if y_len < 4 {
            return Err(Error::MatrixMinimalDimension);
        }

        let mut y_k = DVector::<f64>::from_row_slice(&y_k);
        let mut g_k = MatrixXx4::<f64>::zeros(y_len);

        let r_k = MatrixXx4::<f64>::identity(y_len); // TODO
        let w_k = Matrix4xX::<f64>::identity(y_len); // TODO

        // form g_k
        for (i, index) in indexes.iter().enumerate() {
            let dr_i = sv[*index].relativistic_path_range_m;

            let (dx, dy, dz) =
                candidates[*index].matrix_contribution(&self.cfg, dr_i, pending.pos_m);

            g_k[(i, 0)] = dx;
            g_k[(i, 1)] = dy;
            g_k[(i, 2)] = dz;
            g_k[(i, 3)] = 1.0;
        }

        // verify correctness
        if g_k.nrows() != y_len {
            return Err(Error::MatrixDimension);
        }

        // TODO only for static positioning
        let f_diag = Vector4::new(1.0, 1.0, 1.0, 1.0);
        let f_k = Matrix4::from_diagonal(&f_diag);

        // TODO
        let q_diag = Vector4::new(0.0, 0.0, 0.0, 0.0);
        let q_k = Matrix4::from_diagonal(&q_diag);

        let g_k = g_k.to_owned();
        let (dx, ht_h_inv) = self.kalman.run(f_k, g_k, q_k, y_k)?;

        pending.update(t, self.frame, dx).map_err(|e| {
            error!("{} - state update failed with physical error: {}", t, e);
            Error::StateUpdate
        })?;

        // update
        self.dop = DilutionOfPrecision::new(&pending, ht_h_inv);

        // validation
        self.state_validation(t, &pending);

        self.state = pending;
        self.prev_epoch = Some(t);

        debug!("{} - new state {}", t, self.state);
        debug!("{} - gdop={} tdop={}", t, self.dop.gdop, self.dop.tdop);
        Ok(())
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
#[cfg(feature = "embed_ephem")]
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
