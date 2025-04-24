use crate::error::Error;
use nalgebra::{
    allocator::Allocator,
    constraint::{SameNumberOfRows, ShapeConstraint},
    DefaultAllocator, DimName, OMatrix, OVector,
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
    DefaultAllocator: Allocator<S> + Allocator<S, S>,
    <DefaultAllocator as Allocator<S>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<S, S>>::Buffer<f64>: Copy,
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
    DefaultAllocator: Allocator<S> + Allocator<S, S>,
    <DefaultAllocator as Allocator<S>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<S, S>>::Buffer<f64>: Copy,
{
    /// True if this [Kalman] filter has been initialized
    pub initialized: bool,

    /// Latest predicted [KfEstimate]
    prediction: KfEstimate<S>,
}

impl<S> Kalman<S>
where
    S: DimName,
    DefaultAllocator: Allocator<S> + Allocator<S, S>,
    <DefaultAllocator as Allocator<S>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<S, S>>::Buffer<f64>: Copy,
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
    /// - w_k: W [OMatrix]
    /// - q_k: Q [OMatrix]
    /// - y_k: Measurement [OVector]
    pub fn run<N: DimName>(
        &mut self,
        f_k: OMatrix<f64, S, S>,
        g_k: OMatrix<f64, N, S>,
        w_k: OMatrix<f64, N, S>,
        q_k: OMatrix<f64, S, S>,
        y_k: OVector<f64, N>,
    ) -> Result<KfEstimate<S>, Error>
    where
        DefaultAllocator: Allocator<N> + Allocator<N, S> + Allocator<S, N>,
        ShapeConstraint: SameNumberOfRows<S, N>,
        <ShapeConstraint as SameNumberOfRows<S, N>>::Representative: DimName,
    {
        if !self.initialized {
            panic!("internal error: filter not initialized!");
        }

        let gt = g_k.transpose();
        let gt_g = gt.clone() * g_k;
        let gt_g_w = gt_g.clone() * w_k.clone();
        let gt_g_w_y = gt_g_w.clone() * y_k.clone();

        let p_inv = self
            .prediction
            .p
            .clone()
            .try_inverse()
            .ok_or(Error::MatrixInversion)?;

        let p_inv_x = p_inv.clone() * self.prediction.x.clone();

        let gt_g_w_y_p_inv_x = gt_g_w_y + p_inv_x;
        let gt_w_g_p_inv = gt_g_w.clone() + p_inv;

        let x_k = self.prediction.p.clone() * gt_g_w_y_p_inv_x;

        let p_k = gt_w_g_p_inv.try_inverse().ok_or(Error::MatrixInversion)?;

        // prediction
        let x_k1 = f_k.clone() * x_k.clone();
        let p_k1 = f_k.clone() * p_k.clone() * f_k.transpose() + q_k;

        self.prediction = KfEstimate { x: x_k1, p: p_k1 };

        Ok(KfEstimate { x: x_k, p: p_k })
    }
}
