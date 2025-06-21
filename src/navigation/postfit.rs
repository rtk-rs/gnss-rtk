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
    f_mat: OMatrix<f64, U6, U6>,

    /// G [DMatrix]
    g_mat: DMatrix<f64>,

    /// W [DMatrix]
    w_mat: DMatrix<f64>,

    /// Q [OMatrix]
    q_mat: OMatrix<f64, U6, U6>,

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

        let q_mat = Matrix6::from_diagonal(&q_diag);
        let p_0 = Matrix6::from_diagonal(&q_diag);
        let f_mat = Matrix6::identity();
        let g_mat = Matrix6::identity();
        let w_mat = Matrix6::identity();

        let mut kalman = Kalman::new();

        let initial_estimate = KfEstimate::from_static(x_0, p_0);

        kalman.initialize(f_mat, q_mat, initial_estimate);

        Self {
            f_mat,
            q_mat,
            g_mat: DMatrix::from_column_slice(U6::USIZE, U6::USIZE, g_mat.as_slice()),
            w_mat: DMatrix::from_column_slice(U6::USIZE, U6::USIZE, w_mat.as_slice()),
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

        self.f_mat[(0, 3)] = dt_s;
        self.f_mat[(1, 4)] = dt_s;
        self.f_mat[(2, 5)] = dt_s;

        let y_vec = DVector::from_row_slice(&state.position_velocity_ecef_m().as_slice());

        self.kalman
            .run(&self.f_mat, &self.g_mat, &self.w_mat, &self.q_mat, &y_vec)
    }

    /// Reset this [PostfitKf]
    pub fn reset(&mut self) {
        self.kalman.reset();
    }
}
