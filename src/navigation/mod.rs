use log::{debug, error, info};

#[cfg(feature = "serde")]
use serde::Serialize;

#[cfg(doc)]
use crate::prelude::TimeScale;

pub(crate) mod apriori;
pub(crate) mod solutions;

mod dop;
mod kalman;
mod postfit;
mod sv;

pub(crate) mod state;

use nalgebra::{allocator::Allocator, DMatrix, DVector, DefaultAllocator, DimName, OMatrix, U4};

use crate::{
    cfg::User,
    navigation::{
        apriori::Apriori,
        dop::DilutionOfPrecision,
        kalman::{Kalman, KfEstimate},
        postfit::PostfitKf,
        state::State,
    },
    prelude::{Bias, Candidate, Config, Epoch, Error, Frame, SPEED_OF_LIGHT_M_S},
    rtk::RTKBase,
};

pub use solutions::PVTSolution;
pub use sv::SVContribution;

/// [Navigation] Solver
pub(crate) struct Navigation<D: DimName>
where
    DefaultAllocator: Allocator<D> + Allocator<D, D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
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
    f_k: OMatrix<f64, D, D>,

    /// Q
    q_k: OMatrix<f64, D, D>,

    /// X
    x_k: DVector<f64>,

    /// P
    p_k: DMatrix<f64>,

    /// [Kalman]
    kalman: Kalman<D>,

    /// contribution allocation
    indexes: Vec<usize>,

    /// [SVContribution]s
    pub sv: Vec<SVContribution>,

    /// Postfit [Kalman]
    postfit: Option<PostfitKf>,

    /// True if this filter has been initialized
    pub initialized: bool,

    /// Current [State]
    pub state: State<D>,

    /// [DilutionOfPrecision]
    pub dop: DilutionOfPrecision,

    /// Null of first iter
    prev_epoch: Option<Epoch>,
}

impl<D: DimName> Navigation<D>
where
    DefaultAllocator: Allocator<D> + Allocator<D, D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
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
        match D::USIZE {
            4 | 7 => {},
            u => panic!("non supported dimensions: {}", u),
        }

        let mut f_k = OMatrix::<f64, D, D>::zeros();
        let mut q_k = OMatrix::<f64, D, D>::zeros();

        if D::USIZE == 4 {
            for i in 0..=2 {
                f_k[(i, i)] = 1.0;
            }
        }

        q_k[(0, 0)] = 1.0;
        q_k[(1, 1)] = 1.0;
        q_k[(2, 2)] = 1.0;

        q_k[(Self::clock_index(), Self::clock_index())] =
            (cfg.user.clock_sigma_s * SPEED_OF_LIGHT_M_S).powi(2);

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
            kalman: Kalman::<D>::new(),
            y_k_vec: Vec::with_capacity(8),
            w_k_vec: Vec::with_capacity(8),
            indexes: Vec::with_capacity(8),
            x_k: DVector::zeros(D::USIZE),
            dop: DilutionOfPrecision::default(),
            g_k: DMatrix::<f64>::zeros(4, 4),
            w_k: DMatrix::<f64>::zeros(4, 4),
            p_k: DMatrix::<f64>::zeros(D::USIZE, D::USIZE),
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

    /// Returns clock index
    pub(crate) fn clock_index() -> usize {
        D::USIZE - 1
    }

    /// Iterates mutable [Navigation] filter.
    /// ## Input
    /// - t: sampling [Epoch]
    /// - user: [User]
    /// - past_state: past [State]
    /// - candidates: proposed [Candidate]s
    /// - size: number of proposed [Cadndidate]s
    /// - rtk_base: possible [RTKBase] implementation]
    /// - bias: external [Bias] implementation
    pub fn solving<B: Bias, R: RTKBase>(
        &mut self,
        t: Epoch,
        user: User,
        past_state: &State<D>,
        candidates: &[Candidate],
        size: usize,
        rtk_base: &R,
        bias: &B,
    ) -> Result<(), Error> {
        self.clear();

        if self.cfg.user != user {
            // profile update
            if user.profile != self.cfg.user.profile {
                if let Some(profile) = user.profile {
                    info!("{}: switching to {} profile", t, profile);
                    self.cfg.user.profile = Some(profile);
                }
            }

            self.q_k[(Self::clock_index(), Self::clock_index())] =
                (user.clock_sigma_s * SPEED_OF_LIGHT_M_S).powi(2);
        }

        if !self.kalman.initialized {
            self.kf_initialization(t, past_state, candidates, size, rtk_base, bias)?;
        } else {
            self.kf_run(t, candidates, size, rtk_base, bias)?;
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
        state: &State<D>,
        candidates: &[Candidate],
        size: usize,
        rtk_base: &RTK,
        bias: &B,
    ) -> Result<(), Error> {
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

        if y_len < D::USIZE {
            return Err(Error::MatrixMinimalDimension);
        }

        self.g_k.resize_mut(y_len, D::USIZE, 0.0);

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

                self.g_k[(i, Self::clock_index())] = 1.0;
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
        rtk_base: &RTK,
        bias: &B,
    ) -> Result<(), Error> {
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

        if y_len < D::USIZE {
            return Err(Error::MatrixMinimalDimension);
        }

        let y_k = DVector::from_row_slice(&self.y_k_vec); // TODO vector

        self.w_k.resize_mut(y_len, y_len, 0.0);
        self.g_k.resize_mut(y_len, D::USIZE, 0.0);

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

            self.g_k[(i, Self::clock_index())] = 1.0;
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

#[cfg(test)]
mod test {
    use crate::navigation::Navigation;
    use nalgebra::{U4, U7, U9};

    #[test]
    fn navigation_dimensions_clock_index() {
        assert_eq!(Navigation::<U4>::clock_index(), 3);
        assert_eq!(Navigation::<U7>::clock_index(), 6);
        assert_eq!(Navigation::<U9>::clock_index(), 8);
    }
}
