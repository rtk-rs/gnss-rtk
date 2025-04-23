use crate::error::Error;
use nalgebra::{
    allocator::Allocator,
    constraint::{DimEq, SameNumberOfColumns, SameNumberOfRows, ShapeConstraint},
    Const, DVector, DefaultAllocator, DimName, Matrix4, MatrixMN, MatrixXx4, OMatrix, OVector,
    Vector4, U4,
};

#[derive(Clone)]
pub struct KfEstimate<S>
where
    S: DimName,
    DefaultAllocator: nalgebra::allocator::Allocator<S, S>,
    DefaultAllocator: nalgebra::allocator::Allocator<S>,
{
    /// P Matrix
    pub p: OMatrix<f64, S, S>,

    /// x Vector
    pub x: OVector<f64, S>,
}

impl<S> KfEstimate<S>
where
    S: DimName,
    DefaultAllocator: nalgebra::allocator::Allocator<S, S>,
    DefaultAllocator: nalgebra::allocator::Allocator<S>,
{
    /// Create a zero [KfEstimate]
    pub fn zero() -> Self {
        let x = OVector::<f64, S>::zeros();
        let p = OMatrix::<f64, S, S>::zeros();
        Self { p, x }
    }

    /// Create new [KfEstimate]
    pub fn new(x: OVector<f64, S>, p: OMatrix<f64, S, S>) -> Self {
        Self { p, x }
    }
}

#[derive(Clone)]
pub struct Kalman<S>
where
    S: DimName,
    DefaultAllocator: nalgebra::allocator::Allocator<S, S>,
    DefaultAllocator: nalgebra::allocator::Allocator<S>,
{
    /// True if this [Kalman] filter has been initialized
    pub initialized: bool,

    /// Latest predicted [KfEstimate]
    prediction: KfEstimate<S>,
}

impl<S> Kalman<S>
where
    S: DimName,
    DefaultAllocator: nalgebra::allocator::Allocator<S, S>,
    DefaultAllocator: nalgebra::allocator::Allocator<S>,
{
    /// Create a new [Kalman] filter
    pub fn new() -> Self {
        Self {
            initialized: false,
            prediction: KfEstimate::zero(),
        }
    }

    /// Initialize this [Kalman]filter
    pub fn initialize(
        &mut self,
        initial_estimate: KfEstimate<S>,
        f_k: OMatrix<f64, S, S>,
        q_k: OMatrix<f64, S, S>,
    ) {
        // prediction
        let x_k1 = f_k.clone() * initial_estimate.x.clone();
        let p_k1 = f_k.clone() * initial_estimate.p.clone() * f_k.transpose() + q_k;

        self.prediction = KfEstimate { x: x_k1, p: p_k1 };

        self.initialized = true;
    }

    /// Reset this [Kalman] filter
    pub fn reset(&mut self) {
        self.initialized = false;
        self.prediction = KfEstimate::zero();
    }

    /// Run this [Kalman] filter, returning new [KfEstimate].
    ///
    /// ## Input
    /// - f_k: Dynamics [OMatrix]
    /// - g_k: G [OMatrix]
    /// - q_k: Q [OMatrix]
    /// - y_k: Measurement [OVector]
    pub fn run(
        &mut self,
        f_k: Matrix4<f64>,
        g_k: MatrixXx4<f64>,
        q_k: OMatrix<f64, U4, U4>,
        y_k: DVector<f64>,
    ) -> Result<KfEstimate<U4>, Error> {
        if !self.initialized {
            panic!("internal error: filter not initialized!");
        }

        let gt = g_k.transpose();

        let gt_w_y = gt.clone() * y_k; // vec<4>

        let p_inv = self
            .prediction
            .p
            .clone()
            .try_inverse()
            .ok_or(Error::MatrixInversion)?;

        let p_inv_x = p_inv.clone() * self.prediction.x.clone(); // vec<4> (2)

        let mut gt_w_y_p_inv_x = Vector4::zeros();

        let mut gt_w_g = Matrix4::zeros();

        for i in 0.. {
            gt_w_y_p_inv_x[i] = gt_w_y[i] + p_inv_x[i];
        }

        let x_k = self.prediction.p.clone() * gt_w_y_p_inv_x;

        let gt_g = gt.clone() * g_k.clone();
        let gt_g_p_inv = gt_g + p_inv.clone();

        let p_k = (gt.clone() * g_k.clone() + p_inv)
            .try_inverse()
            .ok_or(Error::MatrixInversion)?;

        // prediction
        let x_k1 = f_k.clone() * x_k.clone();
        let p_k1 = f_k.clone() * p_k.clone() * f_k.transpose() + q_k;

        let g_g_t = g_k.clone() * gt.clone();

        self.prediction = KfEstimate { x: x_k1, p: p_k1 };

        Ok(KfEstimate { x: x_k, p: p_k })
    }
}
