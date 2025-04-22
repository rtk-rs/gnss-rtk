use nyx::linalg::{Matrix6, Vector6, U6};

use crate::{
    error::Error,
    navigation::{kalman::Kalman, state::State},
    prelude::Duration,
};

#[derive(Debug, Clone)]
pub struct PostfitKf {
    kalman: Kalman<U6>,
    r_mat: Matrix6<f64>,
    h_mat: Matrix6<f64>,
    meas_pos_std_dev_m: f64,
    meas_vel_std_dev_m_s: f64,
    state_pos_std_dev_m: f64,
    state_vel_std_dev_m_s: f64,
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
        let q_diag = Vector6::new(
            state_pos_std_dev_m.powi(2),
            state_pos_std_dev_m.powi(2),
            state_pos_std_dev_m.powi(2),
            state_vel_std_dev_m_s.powi(2),
            state_vel_std_dev_m_s.powi(2),
            state_vel_std_dev_m_s.powi(2),
        );

        let q_mat = Matrix6::from_diagonal(&q_diag);

        let r_diag = Vector6::new(
            meas_pos_std_dev_m.powi(2),
            meas_pos_std_dev_m.powi(2),
            meas_pos_std_dev_m.powi(2),
            meas_vel_std_dev_m_s.powi(2),
            meas_vel_std_dev_m_s.powi(2),
            meas_vel_std_dev_m_s.powi(2),
        );

        let r_mat = Matrix6::from_diagonal(&r_diag);

        let x_0 = Vector6::new(
            state.pos_m.0,
            state.pos_m.1,
            state.pos_m.2,
            state.vel_m_s.0,
            state.vel_m_s.1,
            state.vel_m_s.2,
        );

        let p_0 = Matrix6::from_diagonal(&q_diag);

        let h_mat = Matrix6::identity();

        let kalman = Kalman::new_initialized(x_0, p_0, q_mat);

        Self {
            r_mat,
            h_mat,
            kalman,
            meas_pos_std_dev_m,
            meas_vel_std_dev_m_s,
            state_pos_std_dev_m,
            state_vel_std_dev_m_s,
        }
    }

    /// Run [PostfitKf] filter
    /// - sampling_interval: [Duration]
    /// - state: new [State]
    pub fn run(
        &mut self,
        sampling_interval: Duration,
        state: &State,
    ) -> Result<Vector6<f64>, Error> {
        let dt_s = sampling_interval.to_seconds();

        let f_diag = Vector6::<f64>::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
        let mut f_mat = Matrix6::from_diagonal(&f_diag);

        f_mat[(0, 3)] = dt_s;
        f_mat[(1, 4)] = dt_s;
        f_mat[(2, 5)] = dt_s;

        let z_k = Vector6::new(
            state.pos_m.0,
            state.pos_m.1,
            state.pos_m.2,
            state.vel_m_s.0,
            state.vel_m_s.1,
            state.vel_m_s.2,
        );

        let x_k = self.kalman.run(z_k, f_mat, self.h_mat, self.r_mat)?;

        Ok(x_k)
    }

    /// Reset this [PostfitKf]
    pub fn reset(&mut self) {
        self.kalman.reset();
    }
}
