use nalgebra::{
    allocator::Allocator, DVector, DefaultAllocator, DimName, Matrix6, OMatrix, Vector6, U6,
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
    /// F [Matrix6]
    f_k: OMatrix<f64, U6, U6>,

    /// G [Matrix6]
    g_k: OMatrix<f64, U6, U6>,

    /// W [Matrix6]
    w_k: OMatrix<f64, U6, U6>,

    /// Q [OMatrix6]
    q_k: OMatrix<f64, U6, U6>,

    /// [Kalman] filter
    kalman: Kalman<U6>,
}

impl PostfitKf {
    /// Builds new [PostfitKf] from initial [State]
    pub fn new<D: DimName>(
        state: &State<D>,
        state_pos_std_dev_m: f64,
        state_vel_std_dev_m_s: f64,
        meas_pos_std_dev_m: f64,
        meas_vel_std_dev_m_s: f64,
    ) -> Self
    where
        DefaultAllocator: Allocator<D> + Allocator<D, D> + Allocator<U6> + Allocator<U6, U6>,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
    {
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

        let mut kalman = Kalman::new();

        let initial_estimate = KfEstimate::from_static(x_0, p_0);

        kalman.initialize(f_k, q_k, initial_estimate);

        Self {
            f_k,
            q_k,
            kalman,
            w_k: OMatrix::identity(),
            g_k: OMatrix::identity(),
        }
    }

    /// Run [PostfitKf] filter
    /// - sampling_interval: [Duration]
    /// - state: new [State]
    pub fn run<D: DimName>(
        &mut self,
        state: &State<D>,
        sampling_interval: Duration,
    ) -> Result<KfEstimate<U6>, Error>
    where
        DefaultAllocator: Allocator<D> + Allocator<D, D> + Allocator<U6> + Allocator<U6, U6>,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
    {
        let dt_s = sampling_interval.to_seconds();

        self.f_k[(0, 3)] = dt_s;
        self.f_k[(1, 4)] = dt_s;
        self.f_k[(2, 5)] = dt_s;

        let y_k = DVector::from_row_slice((&state.position_velocity_ecef_m()).into());

        self.kalman
            .run(&self.f_k, &self.g_k, &self.w_k, &self.q_k, &y_k)
    }

    /// Reset this [PostfitKf]
    pub fn reset(&mut self) {
        self.kalman.reset();
    }
}
