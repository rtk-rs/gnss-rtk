use nalgebra::{Matrix6, Vector6, U6};

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
    /// Dynamics matrix
    f_k: Matrix6<f64>,
    /// R matrix
    r_mat: Matrix6<f64>,
    /// H Matrix
    h_mat: Matrix6<f64>,
    /// [Kalman] filter
    kalman: Kalman<U6>,
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
        let r_diag = Vector6::new(
            meas_pos_std_dev_m.powi(2),
            meas_pos_std_dev_m.powi(2),
            meas_pos_std_dev_m.powi(2),
            meas_vel_std_dev_m_s.powi(2),
            meas_vel_std_dev_m_s.powi(2),
            meas_vel_std_dev_m_s.powi(2),
        );

        let r_k = Matrix6::from_diagonal(&r_diag);

        let x_0 = state.to_pos_vel_vector6();

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

        let mut kalman = Kalman::new();
        let initial_state = KfEstimate::new(x_0, p_0);

        kalman.initialize(initial_state, f_k, q_k);

        Self {
            f_k,
            r_mat: r_k,
            h_mat,
            kalman,
        }
    }

    /// Run [PostfitKf] filter
    /// - sampling_interval: [Duration]
    /// - state: new [State]
    pub fn run(
        &mut self,
        state: &State,
        sampling_interval: Duration,
    ) -> Result<KfEstimate<U6>, Error> {
        let dt_s = sampling_interval.to_seconds();

        self.f_k[(0, 3)] = dt_s;
        self.f_k[(1, 4)] = dt_s;
        self.f_k[(2, 5)] = dt_s;

        let y_k = state.to_pos_vel_vector6();

        self.kalman.run(self.f_k, g_k, r_k, q_k, y_k)
    }

    /// Reset this [PostfitKf]
    pub fn reset(&mut self) {
        self.kalman.reset();
    }
}
