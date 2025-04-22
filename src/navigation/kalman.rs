use crate::error::Error;
use nyx::linalg::{DefaultAllocator, DimName, OMatrix, OVector};

#[derive(Debug, Clone)]
pub struct Kalman<S>
where
    S: DimName,
    DefaultAllocator: nalgebra::allocator::Allocator<S, S>,
    DefaultAllocator: nalgebra::allocator::Allocator<S>,
{
    /// P_k Matrix
    p_k: OMatrix<f64, S, S>,
    /// x_k Vector
    x_k: OVector<f64, S>,
    /// Q Covar Matrix
    q_mat: OMatrix<f64, S, S>,
    /// True if this [Kalman] filter is initialized
    pub initialized: bool,
}

impl<S> Kalman<S>
where
    S: DimName,
    DefaultAllocator: nalgebra::allocator::Allocator<S, S>,
    DefaultAllocator: nalgebra::allocator::Allocator<S>,
{
    /// Create a new [Kalman] filter without initialization.
    pub fn new() -> Self {
        let x_k = OVector::<f64, S>::zeros();
        let p_k = OMatrix::<f64, S, S>::zeros();
        let q_mat = OMatrix::<f64, S, S>::zeros();
        Self {
            p_k,
            x_k,
            q_mat,
            initialized: false,
        }
    }

    /// Create a new [Kalman] filter with initial state x_0, p_0.
    /// ## Input
    /// - x_0: Initial vectorized state
    /// - P_0: Initial P Matrix
    /// - Q: covar matrix
    pub fn new_initialized(
        x_0: OVector<f64, S>,
        p_0: OMatrix<f64, S, S>,
        q_mat: OMatrix<f64, S, S>,
    ) -> Self {
        Self {
            p_k: p_0,
            x_k: x_0,
            q_mat,
            initialized: true,
        }
    }

    /// Update Q [OMatrix]
    pub fn update_q_mat(&mut self, q_mat: OMatrix<f64, S, S>) {
        self.q_mat = q_mat;
    }

    /// Initialize this [Kalman] filter
    pub fn initialize(&mut self, x_0: OVector<f64, S>, p_0: OMatrix<f64, S, S>) {
        self.x_k = x_0;
        self.p_k = p_0;
        self.initialized = true;
    }

    /// Execute this [Kalman] filter.
    pub fn run(
        &mut self,
        z_k: OVector<f64, S>,
        f_mat: OMatrix<f64, S, S>,
        h_mat: OMatrix<f64, S, S>,
        r_mat: OMatrix<f64, S, S>,
    ) -> Result<OVector<f64, S>, Error> {
        if !self.initialized {
            return Err(Error::UninitializedFilter);
        }

        let f_mat_t = f_mat.transpose();

        let xk_k_1 = f_mat.clone() * self.x_k.clone();
        let pk_k_1 = f_mat.clone() * self.p_k.clone() * f_mat_t + self.q_mat.clone();

        let h_mat_t = h_mat.transpose();
        let h_p_ht = h_mat.clone() * pk_k_1.clone() * h_mat_t.clone();
        let h_p_ht_r = h_p_ht + r_mat;

        let k_mat = pk_k_1.clone()
            * h_mat_t.clone()
            * h_p_ht_r.try_inverse().ok_or(Error::MatrixInversion)?;

        let x_k = xk_k_1.clone() + k_mat.clone() * (z_k - h_mat.clone() * xk_k_1);
        let p_k = (OMatrix::<f64, S, S>::identity() - k_mat * h_mat) * pk_k_1;

        self.x_k = x_k;
        self.p_k = p_k;

        Ok(self.x_k.clone())
    }

    /// Reset this [Kalman] filter
    pub fn reset(&mut self) {
        self.initialized = false;
        self.x_k = OVector::<f64, S>::zeros();
        self.p_k = OMatrix::<f64, S, S>::zeros();
    }
}
