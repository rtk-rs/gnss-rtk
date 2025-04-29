use log::{debug, error};

#[cfg(feature = "serde")]
use serde::Serialize;

#[cfg(doc)]
use crate::prelude::TimeScale;

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
        state::State,
    },
    prelude::{
        Bias, Candidate, Config, Duration, Epoch, Error, Frame, IonosphereBias, Signal,
        SPEED_OF_LIGHT_M_S, SV,
    },
    rtk::RTKBase,
};

pub(crate) const CLOCK_INDEX: usize = 3;

pub use solutions::PVTSolution;

/// SV Navigation information
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct SVContribution {
    /// [SV] identity
    pub sv: SV,

    /// [Signal] being used
    pub signal: Signal,

    /// Orbital state in kilometers ECEF
    pub sv_pos_km: (f64, f64, f64),

    /// Orbital velocity in km/s ECEF
    pub sv_vel_km_s: (f64, f64, f64),

    /// Elevation angle from RX position
    pub elevation_deg: f64,

    /// Azimuth angle from RX position
    pub azimuth_deg: f64,

    /// Relativistic path range
    pub relativistic_path_range_m: f64,

    /// Troposphere bias as meters of propagation delay.
    pub tropo_bias: Option<f64>,

    /// Ionosphere bias as meters of propagation delay.
    pub iono_bias: Option<IonosphereBias>,

    /// Offset to selected [TimeScale], expressed as [Duration] within that [TimeScale].
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

    /// Y allocation
    y_k_vec: Vec<f64>,

    /// W allocation
    w_k_vec: Vec<f64>,

    /// W
    w_k: DMatrix<f64>,

    /// G
    g_k: DMatrix<f64>,

    /// F
    f_k: OMatrix<f64, S, S>,

    /// Q
    q_k: OMatrix<f64, S, S>,

    /// X
    x_k: DVector<f64>,

    /// P
    p_k: DMatrix<f64>,

    /// [Kalman]
    kalman: Kalman<S>,

    /// contribution allocation
    indexes: Vec<usize>,

    /// [SVContribution]s
    pub sv: Vec<SVContribution>,

    /// Postfit [Kalman]
    postfit: Option<PostfitKf>,

    /// True if this filter has been initialized
    pub initialized: bool,

    /// Current [State]
    pub state: State<S>,

    /// [DilutionOfPrecision]
    pub dop: DilutionOfPrecision,

    /// Null of first iter
    prev_epoch: Option<Epoch>,
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
        let mut f_k = OMatrix::<f64, S, S>::zeros();
        let mut q_k = OMatrix::<f64, S, S>::zeros();

        if cfg.user.profile.is_static() {
            for i in 0..=2 {
                f_k[(i, i)] = 1.0;
            }
        }

        const X_BIAS_METERS: f64 = 1.0; // [m]
        const Y_BIAS_METERS: f64 = 1.0; // [m]
        const Z_BIAS_METERS: f64 = 1.0; // [m]

        for i in 0..S::USIZE {
            if i == CLOCK_INDEX {
                q_k[(i, i)] = (cfg.user.clock_sigma_s * SPEED_OF_LIGHT_M_S).powi(2);
            } else if i == 0 {
                q_k[(i, i)] = X_BIAS_METERS;
            } else if i == 1 {
                q_k[(i, i)] = Y_BIAS_METERS;
            } else if i == 2 {
                q_k[(i, i)] = Z_BIAS_METERS;
            }
        }

        Self {
            f_k,
            q_k,
            frame,
            postfit: None,
            cfg: cfg.clone(),
            prev_epoch: None,
            initialized: false,
            state: State::default(),
            sv: Vec::with_capacity(8),
            kalman: Kalman::<S>::new(),
            y_k_vec: Vec::with_capacity(8),
            w_k_vec: Vec::with_capacity(8),
            indexes: Vec::with_capacity(8),
            x_k: DVector::zeros(S::USIZE),
            dop: DilutionOfPrecision::default(),
            g_k: DMatrix::<f64>::zeros(4, 4),
            w_k: DMatrix::<f64>::zeros(4, 4),
            p_k: DMatrix::<f64>::zeros(S::USIZE, S::USIZE),
        }
    }

    /// Reset [Navigation] filter.
    pub fn reset(&mut self) {
        self.clear();
        self.kalman.reset();

        if let Some(postfit) = &mut self.postfit {
            postfit.reset();
        }

        self.prev_epoch = None;
        self.initialized = false;
        self.dop = DilutionOfPrecision::default();
    }

    /// Iterates mutable [Navigation] filter.
    pub fn solving<B: Bias, RTK: RTKBase>(
        &mut self,
        t: Epoch,
        past_state: &State<S>,
        candidates: &[Candidate],
        size: usize,
        rtk_bases: &[RTK],
        num_bases: usize,
        bias: &B,
    ) -> Result<(), Error> {
        assert!(num_bases < 2, "n-RTK* is not supported yet");

        self.clear();

        if !self.kalman.initialized {
            self.kf_initialization(t, past_state, candidates, size, rtk_bases, num_bases, bias)?;
        } else {
            self.kf_run(t, candidates, size, rtk_bases, num_bases, bias)?;
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
                let sigma_state_pos_std_dev_m = 1.0 / self.cfg.solver.postfit_denoising;

                let mut sigma_state_vel_std_dev_m = 1.0 / self.cfg.solver.postfit_denoising;

                if self.cfg.user.profile.is_static() {
                    sigma_state_vel_std_dev_m /= 100.0;
                }

                self.postfit = Some(PostfitKf::new(
                    &self.state,
                    sigma_state_pos_std_dev_m,
                    sigma_state_vel_std_dev_m,
                    1.0,
                    1.0,
                ));
            }
        }

        self.prev_epoch = Some(t);
        self.initialized = true;

        Ok(())
    }

    fn clear(&mut self) {
        self.sv.clear();
        self.indexes.clear();
        self.y_k_vec.clear();
        self.w_k_vec.clear();
    }

    /// Filter first iteration.
    pub fn kf_initialization<B: Bias, RTK: RTKBase>(
        &mut self,
        t: Epoch,
        state: &State<S>,
        candidates: &[Candidate],
        size: usize,
        rtk_bases: &[RTK],
        num_bases: usize,
        bias: &B,
    ) -> Result<(), Error> {
        assert!(S::USIZE >= U4::USIZE, "minimal dimensions!");

        let nb_iter = 10;

        let mut pending = state.clone();

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
            ) {
                Ok((y_i, r_i, dr_i)) => {
                    self.y_k_vec.push(y_i);
                    self.w_k_vec.push(1.0 / r_i);
                    self.indexes.push(i);
                    self.sv.push(contrib);

                    if self.cfg.modeling.relativistic_path_range {
                        debug!(
                            "{}({}) - relativistic path range: {:.3}m",
                            t, candidates[i].sv, dr_i
                        );
                    }
                },
                Err(e) => {
                    error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                },
            }
        }

        let y_len = self.indexes.len();
        if y_len < S::USIZE {
            return Err(Error::MatrixMinimalDimension);
        }

        self.g_k.resize_mut(y_len, S::USIZE, 0.0);

        // run
        for _ in 0..nb_iter {
            let y_len = self.y_k_vec.len();

            //self.y_k.resize_mut(y_len, 0.0); // TODO: vector
            self.w_k.resize_mut(y_len, y_len, 0.0);

            for i in 0..y_len {
                self.w_k[(i, i)] = 1.0 / self.w_k_vec[i];
            }

            let y_k = DVector::from_row_slice(&self.y_k_vec);

            // form g_k
            for (i, index) in self.indexes.iter().enumerate() {
                let dr_i = self.sv[i].relativistic_path_range_m;

                let position_m = pending.position_ecef_m();

                let (dx, dy, dz) =
                    candidates[*index].matrix_contribution(&self.cfg, dr_i, position_m);

                self.g_k[(i, 0)] = dx;
                self.g_k[(i, 1)] = dy;
                self.g_k[(i, 2)] = dz;
                self.g_k[(i, CLOCK_INDEX)] = 1.0;
            }

            // run
            let gt = self.g_k.transpose();
            let gt_g = gt.clone() * self.g_k.clone();
            let gt_w = gt.clone() * self.w_k.clone();
            let gt_w_g = gt_w * self.g_k.clone();
            let gt_w_g_inv = gt_w_g.try_inverse().ok_or(Error::MatrixInversion)?;
            let gt_w_g_inv_gt = gt_w_g_inv.clone() * gt.clone();
            let gt_w_g_inv_gt_w = gt_w_g_inv_gt * self.w_k.clone();

            self.x_k = gt_w_g_inv_gt_w * y_k.clone();
            self.p_k = gt_w_g_inv.clone();

            debug!("state correction: dx={}", self.x_k);

            pending.correct_mut(self.frame, t, &self.x_k).map_err(|e| {
                error!("{} - state update failed with physical error: {}", t, e);
                Error::StateUpdate
            })?;

            let gt_g_inv = gt_g.try_inverse().ok_or(Error::MatrixInversion)?;

            // update latest DoP
            self.dop = DilutionOfPrecision::new(&pending, gt_g_inv);

            debug!("{} - pending state {}", t, pending);

            // models update
            self.y_k_vec.clear();
            self.w_k_vec.clear();

            self.indexes.retain(|i| {
                let mut unused = SVContribution::default();

                let position_m = pending.position_ecef_m();

                match candidates[*i].vector_contribution(
                    t,
                    &self.cfg,
                    position_m,
                    pending.lat_long_alt_deg_deg_km,
                    &mut unused,
                    bias,
                ) {
                    Ok((y_i, r_i, _)) => {
                        self.y_k_vec.push(y_i);
                        self.w_k_vec.push(1.0 / r_i);
                        true
                    },
                    Err(e) => {
                        error!("{}({}) - cannot contribute: {}", t, candidates[*i].sv, e);
                        false
                    },
                }
            });
        }

        // validation
        self.state_validation()?;

        let initial_estimate = KfEstimate::from_dynamic(self.x_k.clone(), self.p_k.clone());

        self.kalman.initialize(self.f_k, self.q_k, initial_estimate);

        self.state = pending;
        debug!("{} - new state {}", t, self.state);
        debug!("{} - gdop={} tdop={}", t, self.dop.gdop, self.dop.tdop);

        Ok(())
    }

    /// [Kalman] filter run
    pub fn kf_run<B: Bias, RTK: RTKBase>(
        &mut self,
        t: Epoch,
        candidates: &[Candidate],
        size: usize,
        rtk_bases: &[RTK],
        num_bases: usize,
        bias: &B,
    ) -> Result<(), Error> {
        assert!(S::USIZE >= U4::USIZE, "minimal dimensions!");

        let mut pending = self.state.clone();

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
            ) {
                Ok((y_i, r_i, dr_i)) => {
                    self.y_k_vec.push(y_i);
                    self.w_k_vec.push(1.0 / r_i);
                    self.indexes.push(i);
                    self.sv.push(contrib);

                    if self.cfg.modeling.relativistic_path_range {
                        debug!(
                            "{}({}) - relativistic path range: {:.3}m",
                            t, candidates[i].sv, dr_i
                        );
                    }
                },
                Err(e) => {
                    error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                },
            }
        }

        let y_len = self.y_k_vec.len();

        if y_len < S::USIZE {
            return Err(Error::MatrixMinimalDimension);
        }

        let y_k = DVector::from_row_slice(&self.y_k_vec); // TODO vector

        self.w_k.resize_mut(y_len, y_len, 0.0);
        self.g_k.resize_mut(y_len, S::USIZE, 0.0);

        for i in 0..y_len {
            self.w_k[(i, i)] = 1.0 / self.w_k_vec[i];
        }

        // form g_k
        for (i, index) in self.indexes.iter().enumerate() {
            let dr_i = self.sv[i].relativistic_path_range_m;

            let position_m = pending.position_ecef_m();

            let (dx, dy, dz) = candidates[*index].matrix_contribution(&self.cfg, dr_i, position_m);

            self.g_k[(i, 0)] = dx;
            self.g_k[(i, 1)] = dy;
            self.g_k[(i, 2)] = dz;
            self.g_k[(i, CLOCK_INDEX)] = 1.0;
        }

        let estimate = self
            .kalman
            .run(&self.f_k, &self.g_k, &self.w_k, &self.q_k, &y_k)?;

        debug!("state correction: dx={}", estimate.x);

        for i in 0..estimate.x.nrows() {
            self.x_k[i] = estimate.x[i];
        }

        pending.correct_mut(self.frame, t, &self.x_k).map_err(|e| {
            error!("{} - state update failed with physical error: {}", t, e);
            Error::StateUpdate
        })?;

        let gt_g_inv = (self.g_k.transpose() * self.g_k.clone())
            .try_inverse()
            .ok_or(Error::MatrixInversion)?;

        // update
        self.dop = DilutionOfPrecision::new(&pending, gt_g_inv);

        debug!("{} - new state {}", t, pending);
        debug!("{} - gdop={} tdop={}", t, self.dop.gdop, self.dop.tdop);

        self.state_validation()?;
        self.state = pending;

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
