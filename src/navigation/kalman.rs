use crate::error::Error;
use nalgebra::{
    allocator::Allocator, DMatrix, DVector, DefaultAllocator, DimName, OMatrix, OVector,
};

#[derive(Clone)]
pub struct KfEstimate<S>
where
    S: DimName,
    DefaultAllocator: nalgebra::allocator::Allocator<S, S>,
    DefaultAllocator: nalgebra::allocator::Allocator<S>,
{
    /// P [OMatrix]
    pub p: OMatrix<f64, S, S>,

    /// x [OVector]
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
    pub fn new(x: DVector<f64>, p: DMatrix<f64>) -> Self {
        assert_eq!(
            p.nrows(),
            S::USIZE,
            "internal error: invalid initialization dimensions!"
        );
        assert_eq!(
            p.ncols(),
            S::USIZE,
            "internal error: invalid initialization dimensions!"
        );

        let mut x_stored = OVector::<f64, S>::zeros();
        let mut p_stored = OMatrix::<f64, S, S>::zeros();

        for i in 0..S::USIZE {
            x_stored[i] = x[i];

            for j in 0..S::USIZE {
                p_stored[(i, j)] = p[(i, j)];
            }
        }

        Self {
            p: p_stored,
            x: x_stored,
        }
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
    /// - w_k: W [OVector]
    /// - q_k: Q [OMatrix]
    /// - y_k: Measurement [OVector]
    pub fn run(
        &mut self,
        f_k: OMatrix<f64, S, S>,
        g_k: &DMatrix<f64>,
        w_k: DMatrix<f64>,
        q_k: OMatrix<f64, S, S>,
        y_k: DVector<f64>,
    ) -> Result<KfEstimate<S>, Error> {
        if !self.initialized {
            panic!("internal error: filter not initialized!");
        }

        assert_eq!(
            w_k.nrows(),
            w_k.ncols(),
            "internal error: w is not squared matrix!"
        );

        assert_eq!(
            w_k.nrows(),
            y_k.nrows(),
            "internal error: invalid dimensions!"
        );

        assert_eq!(g_k.ncols(), 4, "internal error: invalid G dimensions!");

        assert_eq!(
            g_k.nrows(),
            y_k.nrows(),
            "internal error: invalid G dimensions!"
        );

        let gt = g_k.transpose();
        let gt_w = gt.clone() * w_k;
        let gt_w_y = gt_w.clone() * y_k;
        let gt_w_g = gt_w.clone() * g_k.clone();

        let p_inv = self
            .prediction
            .p
            .clone()
            .try_inverse()
            .ok_or(Error::MatrixInversion)?;

        let p_inv_x = p_inv.clone() * self.prediction.x.clone();

        let x_k = gt_w_y + p_inv_x;
        let x_k = self.prediction.p.clone() * x_k;

        let p_k = gt_w_g + p_inv;
        let p_k = p_k.try_inverse().ok_or(Error::MatrixInversion)?;

        // prediction
        let x_k1 = f_k.clone() * x_k.clone();
        let p_k1 = f_k.clone() * p_k.clone() * f_k.transpose() + q_k;

        self.prediction = KfEstimate { x: x_k1, p: p_k1 };

        Ok(KfEstimate { x: x_k, p: p_k })
    }
}
