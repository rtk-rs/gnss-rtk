use nalgebra::{
    allocator::Allocator, DMatrix, DVector, DefaultAllocator, DimName, Matrix6, OMatrix, Vector6,
    U6,
};

use crate::{
    error::Error,
    navigation::{
        kalman::{Kalman, KfEstimate},
        state::State,
    },
    prelude::Duration,
};

#[derive(Clone)]
pub struct PostfitKf {
    /// F [OMatrix]
    f_k: OMatrix<f64, U6, U6>,

    /// G [OMatrix]
    g_k: OMatrix<f64, U6, U6>,

    /// W [OMatrix]
    w_k: OMatrix<f64, U6, U6>,

    /// Q [OMatrix]
    q_k: OMatrix<f64, U6, U6>,

    /// [Kalman] filter
    kalman: Kalman,
}

impl PostfitKf {
    /// Builds new [PostfitKf] from initial [State]
    pub fn new(
        state: &State,
        state_pos_std_dev_m: f64,
        state_vel_std_dev_m_s: f64,
        meas_pos_std_dev_m: f64,
        meas_vel_std_dev_m_s: f64,
    ) -> Self {
        let r_diag = [
            meas_pos_std_dev_m.powi(2),
            meas_pos_std_dev_m.powi(2),
            meas_pos_std_dev_m.powi(2),
            meas_vel_std_dev_m_s.powi(2),
            meas_vel_std_dev_m_s.powi(2),
            meas_vel_std_dev_m_s.powi(2),
        ];

        let x_0 = state.position_velocity_ecef_m();

        let q_diag = Vector6::new(
            state_pos_std_dev_m.powi(2),
            state_pos_std_dev_m.powi(2),
            state_pos_std_dev_m.powi(2),
            state_vel_std_dev_m_s.powi(2),
            state_vel_std_dev_m_s.powi(2),
            state_vel_std_dev_m_s.powi(2),
        );

        let q_k = Matrix6::from_diagonal(&q_diag);
        let p_0 = Matrix6::from_diagonal(&q_diag);
        let f_k = Matrix6::identity();
        let g_k = Matrix6::identity();
        let w_k = Matrix6::identity();

        let mut kalman = Kalman::new(U6::USIZE);

        kalman.initialize_from_static(f_k, q_k, KfEstimate::from_static(x_0, p_0));

        Self {
            f_k,
            q_k,
            g_k,
            w_k,
            kalman,
        }
    }

    /// Run [PostfitKf] filter.
    ///
    /// ## Input
    /// - state: new [State]
    /// - sampling_interval: [Duration]
    ///
    /// ## Output
    /// - estimate: [KfEstimate]
    pub fn run(&mut self, state: &State, sampling_interval: Duration) -> Result<KfEstimate, Error> {
        let dt_s = sampling_interval.to_seconds();

        self.f_k[(0, 3)] = dt_s;
        self.f_k[(1, 4)] = dt_s;
        self.f_k[(2, 5)] = dt_s;

        let y_k = state.position_velocity_ecef_m();

        self.kalman
            .run_static(&self.f_k, &self.g_k, &self.w_k, &self.q_k, &y_k)
    }

    /// Reset this [PostfitKf]
    pub fn reset(&mut self) {
        self.kalman.reset();
    }
}
