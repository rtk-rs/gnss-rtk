use crate::error::Error;
use nalgebra::{DefaultAllocator, DimName, OMatrix, OVector};

#[derive(Debug, Clone)]
pub struct Kalman<S>
where
    S: DimName,
    DefaultAllocator: nalgebra::allocator::Allocator<S, S>,
    DefaultAllocator: nalgebra::allocator::Allocator<S>,
{
    p_k: OMatrix<f64, S, S>,
    x_k: OVector<f64, S>,
    q_mat: OMatrix<f64, S, S>,
}

impl<S> Kalman<S>
where
    S: DimName,
    DefaultAllocator: nalgebra::allocator::Allocator<S, S>,
    DefaultAllocator: nalgebra::allocator::Allocator<S>,
{
    /// Create a new [Kalman] filter with null initial state
    pub fn new(q_mat: OMatrix<f64, S, S>) -> Self {
        let x_k = OVector::<f64, S>::zeros();
        let p_k = OMatrix::<f64, S, S>::zeros();
        Self::new_initialized(x_k, p_k, q_mat)
    }

    /// Create a new [Kalman] filter with initial state x_k, z_k
    pub fn new_initialized(
        x_0: OVector<f64, S>,
        p_0: OMatrix<f64, S, S>,
        q_mat: OMatrix<f64, S, S>,
    ) -> Self {
        Self {
            p_k: p_0,
            x_k: x_0,
            q_mat,
        }
    }

    pub fn run(
        &mut self,
        z_k: OVector<f64, S>,
        f_mat: OMatrix<f64, S, S>,
        h_mat: OMatrix<f64, S, S>,
        r_mat: OMatrix<f64, S, S>,
    ) -> Result<OVector<f64, S>, Error> {
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
}
