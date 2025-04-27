use log::{debug, error};

#[cfg(feature = "serde")]
use serde::Serialize;

pub(crate) mod apriori;
pub(crate) mod solutions;

mod dop;
mod kalman;
mod postfit;

pub(crate) mod state;

use nalgebra::{allocator::Allocator, DMatrix, DVector, DefaultAllocator, DimName, OMatrix, U4};

use crate::{
    navigation::{
        apriori::Apriori,
        dop::DilutionOfPrecision,
        kalman::{Kalman, KfEstimate},
        postfit::PostfitKf,
        state::{correction::StateCorrection, State},
    },
    prelude::{
        Bias, Candidate, Config, Duration, Epoch, Error, Frame, IonosphereBias, Signal,
        SPEED_OF_LIGHT_M_S, SV,
    },
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
pub(crate) struct Navigation<S: DimName>
where
    DefaultAllocator: Allocator<S> + Allocator<S, S>,
    <DefaultAllocator as Allocator<S>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<S, S>>::Buffer<f64>: Copy,
{
    /// [Config] preset
    cfg: Config,

    /// [Frame]
    frame: Frame,

    /// [Kalman]
    kalman: Kalman<S>,

    /// Postfit [Kalman]
    postfit: Option<PostfitKf>,

    /// True if this filter has been initialized
    pub initialized: bool,

    /// Current [State]
    pub state: State<S>,

    /// Previous Epoch. Null on first attempt.
    prev_epoch: Option<Epoch>,

    /// [SVContribution]s
    pub sv: Vec<SVContribution>,

    /// [DilutionOfPrecision]
    pub dop: DilutionOfPrecision,
}

impl<S: DimName> Navigation<S>
where
    DefaultAllocator: Allocator<S> + Allocator<S, S>,
    <DefaultAllocator as Allocator<S>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<S, S>>::Buffer<f64>: Copy,
{
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
            postfit: None,
            cfg: cfg.clone(),
            prev_epoch: None,
            initialized: false,
            sv: Vec::with_capacity(8),
            kalman: Kalman::<S>::new(),
            state: State::default(),
            dop: DilutionOfPrecision::default(),
        }
    }

    /// Reset [Navigation] filter.
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
    pub fn solving<B: Bias, T: Time>(
        &mut self,
        t: Epoch,
        past_state: &State<S>,
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

        if self.cfg.solver.postfit_denoising > 0.0 {
            if let Some(postfit) = &mut self.postfit {
                let prev_epoch = self
                    .prev_epoch
                    .expect("internal error: undetermined past epoch");

                let dt = t - prev_epoch;
                let dx = postfit.run(&self.state, dt)?;

                // update state
                self.state
                    .postfit_update_mut(self.frame, dx.x)
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
                    1.0 / self.cfg.solver.postfit_denoising,
                    1.0 / self.cfg.solver.postfit_denoising,
                    1.0,
                    1.0,
                ));
            }
        }

        self.prev_epoch = Some(t);
        self.initialized = true;

        Ok(())
    }

    /// Filter first iteration.
    pub fn kf_initialization<B: Bias, T: Time>(
        &mut self,
        t: Epoch,
        state: &State<S>,
        candidates: &[Candidate],
        size: usize,
        bias: &B,
        absolute_time: &AbsoluteTime<T>,
    ) -> Result<(), Error> {
        assert!(S::USIZE >= U4::USIZE, "minimal dimensions!");

        let nb_iter = 10;

        self.sv.clear();

        let mut pending = state.clone();
        let mut indexes = Vec::<usize>::with_capacity(size);

        let mut y_k = Vec::<f64>::with_capacity(size);

        // measurement
        for i in 0..size {
            let mut contrib = SVContribution::default();

            contrib.sv = candidates[i].sv;
            let position_m = pending.position_ecef_m();

            match candidates[i].vector_contribution(
                t,
                &self.cfg,
                position_m,
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

                    self.sv.push(contrib);
                },
                Err(e) => {
                    error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                },
            }
        }

        let y_len = indexes.len();
        if y_len < S::USIZE {
            return Err(Error::MatrixMinimalDimension);
        }

        let mut y_k = DVector::<f64>::from_row_slice(&y_k);
        let mut g_k = DMatrix::<f64>::zeros(y_len, S::USIZE);

        let mut x_k = DVector::<f64>::zeros(S::USIZE);
        let mut correction = StateCorrection::default();

        let mut p_k = DMatrix::<f64>::zeros(S::USIZE, S::USIZE);

        // run
        for _ in 0..nb_iter {
            // form g_k
            for (i, index) in indexes.iter().enumerate() {
                let dr_i = self.sv[i].relativistic_path_range_m;

                let position_m = pending.position_ecef_m();

                let (dx, dy, dz) =
                    candidates[*index].matrix_contribution(&self.cfg, dr_i, position_m);

                g_k[(i, 0)] = dx;
                g_k[(i, 1)] = dy;
                g_k[(i, 2)] = dz;
                g_k[(i, 3)] = 1.0;
            }

            // verify correctness
            if g_k.nrows() != y_len {
                return Err(Error::MatrixDimension);
            }

            // TODO: r_k tuning
            let mut r_k = DMatrix::<f64>::identity(g_k.nrows(), g_k.nrows());

            for i in 0..g_k.nrows() {
                r_k[(i, i)] = 5.0; // TODO
            }

            let w_k = r_k.try_inverse().ok_or(Error::MatrixInversion)?;

            // run
            let gt = g_k.transpose();
            let gt_g = gt.clone() * g_k.clone();
            let gt_w = gt.clone() * w_k.clone();
            let gt_w_g = gt_w * g_k.clone();
            let gt_w_g_inv = gt_w_g.try_inverse().ok_or(Error::MatrixInversion)?;
            let gt_w_g_inv_gt = gt_w_g_inv.clone() * gt.clone();
            let gt_w_g_inv_gt_w = gt_w_g_inv_gt * w_k.clone();

            x_k = gt_w_g_inv_gt_w * y_k.clone();
            p_k = gt_w_g_inv.clone();

            for i in 0..S::USIZE {
                correction.dx[i] = x_k[i];
            }

            pending
                .correct_mut(self.frame, t, correction)
                .map_err(|e| {
                    error!("{} - state update failed with physical error: {}", t, e);
                    Error::StateUpdate
                })?;

            let gt_g_inv = gt_g.try_inverse().ok_or(Error::MatrixInversion)?;

            // update latest DoP
            self.dop = DilutionOfPrecision::new(&pending, gt_g_inv);

            debug!("{} - pending state {}", t, pending);

            // models update
            for (i, index) in indexes.iter().enumerate() {
                let mut unused = SVContribution::default();

                let position_m = pending.position_ecef_m();

                match candidates[*index].vector_contribution(
                    t,
                    &self.cfg,
                    position_m,
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
        self.state_validation()?;

        let mut f_k = OMatrix::<f64, S, S>::zeros();

        if self.cfg.profile.is_static() {
            for i in 0..3 {
                f_k[(i, i)] = 1.0;
            }
        }

        let mut q_k = OMatrix::<f64, S, S>::zeros();

        for i in 0..S::USIZE {
            if i == 3 {
                q_k[(i, i)] = (self.cfg.user_clock_sigma * SPEED_OF_LIGHT_M_S).powi(2);
            } else if i == 2 {
                q_k[(i, i)] = 5.0; // z bias
            } else {
                q_k[(i, i)] = 2.5;
            }
        }

        let initial_estimate = KfEstimate::from_dynamic(x_k, p_k);
        self.kalman.initialize(f_k, q_k, initial_estimate);

        self.state = pending;
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
        assert!(S::USIZE >= U4::USIZE, "minimal dimensions!");

        self.sv.clear();

        let mut pending = self.state.clone();
        let mut indexes = Vec::<usize>::with_capacity(size);

        let mut y_k = Vec::<f64>::with_capacity(size);

        // measurement
        for i in 0..size {
            let mut contrib = SVContribution::default();

            contrib.sv = candidates[i].sv;

            let pos_m = pending.position_ecef_m();

            match candidates[i].vector_contribution(
                t,
                &self.cfg,
                pos_m,
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

                    self.sv.push(contrib);
                },
                Err(e) => {
                    error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                },
            }
        }

        let y_len = y_k.len();
        if y_len < S::USIZE {
            return Err(Error::MatrixMinimalDimension);
        }

        let y_k = DVector::<f64>::from_row_slice(&y_k);
        let mut g_k = DMatrix::<f64>::zeros(y_len, S::USIZE);

        // TODO: r_k tuning
        let mut r_k = DMatrix::<f64>::identity(g_k.nrows(), g_k.nrows());

        for i in 0..g_k.nrows() {
            r_k[(i, i)] = 5.0; // TODO
        }

        let w_k = r_k.try_inverse().ok_or(Error::MatrixInversion)?;

        // form g_k
        for (i, index) in indexes.iter().enumerate() {
            let dr_i = self.sv[i].relativistic_path_range_m;

            let position_m = pending.position_ecef_m();

            let (dx, dy, dz) = candidates[*index].matrix_contribution(&self.cfg, dr_i, position_m);

            g_k[(i, 0)] = dx;
            g_k[(i, 1)] = dy;
            g_k[(i, 2)] = dz;
            g_k[(i, 3)] = 1.0;
        }

        if g_k.nrows() != y_len {
            // incorrect dimensions
            return Err(Error::MatrixDimension);
        }

        let mut f_k = OMatrix::<f64, S, S>::zeros();

        if self.cfg.profile.is_static() {
            for i in 0..3 {
                f_k[(i, i)] = 1.0;
            }
        }

        let mut q_k = OMatrix::<f64, S, S>::zeros();

        for i in 0..S::USIZE {
            if i == 3 {
                q_k[(i, i)] = (self.cfg.user_clock_sigma * SPEED_OF_LIGHT_M_S).powi(2);
            } else if i == 2 {
                q_k[(i, i)] = 5.0; // z bias
            } else {
                q_k[(i, i)] = 2.5;
            }
        }

        let estimate = self.kalman.run(f_k, &g_k, w_k, q_k, y_k)?;

        let correction = estimate.to_state_correction();
        debug!("state correction: dx={}", correction.dx);

        pending
            .correct_mut(self.frame, t, correction)
            .map_err(|e| {
                error!("{} - state update failed with physical error: {}", t, e);
                Error::StateUpdate
            })?;

        let gt_g_inv = (g_k.transpose() * g_k)
            .try_inverse()
            .ok_or(Error::MatrixInversion)?;

        // update
        self.dop = DilutionOfPrecision::new(&pending, gt_g_inv);

        self.state_validation()?;

        self.state = pending;
        debug!("{} - new state {}", t, self.state);
        debug!("{} - gdop={} tdop={}", t, self.dop.gdop, self.dop.tdop);

        Ok(())
    }

    /// Validate pending [State]
    fn state_validation(&self) -> Result<(), Error> {
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

// #[cfg(test)]
// #[cfg(feature = "embed_ephem")]
// mod test {
//     use super::{DilutionOfPrecision, State};
//     use crate::prelude::{Almanac, EARTH_J2000};
//     use nalgebra::Matrix4;
//
//     #[test]
//     fn test_dop() {
//         let state = State {
//             t: Default::default(),
//             clock_dt: Default::default(),
//             clock_drift_s_s: 0.0,
//             pos_m: (1.0, 2.0, 3.0),
//             lat_long_alt_deg_deg_km: (0.0, 0.0, 0.0),
//             vel_m_s: (4.0, 5.0, 6.0),
//         };
//
//         let matrix = Matrix4::new(
//             1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
//         );
//
//         let dop = DilutionOfPrecision::new(&state, matrix);
//
//         assert_eq!(dop.gdop, (1.0_f64 + 6.0_f64 + 11.0_f64 + 16.0_f64).sqrt());
//         assert_eq!(dop.tdop, 16.0_f64.sqrt());
//     }
// }
