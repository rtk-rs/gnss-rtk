use crate::error::Error;

use nalgebra::{
    allocator::Allocator, DMatrix, DVector, DefaultAllocator, DimName, Dyn, Matrix, OMatrix,
    OVector, VecStorage, U1,
};

#[derive(Clone)]
pub struct KfEstimate<D>
where
    D: DimName,
    DefaultAllocator: Allocator<D, D>,
    DefaultAllocator: Allocator<D>,
{
    /// P [DMatrix]
    pub p: OMatrix<f64, D, D>,

    /// x [OVector]
    pub x: OVector<f64, D>,
}

impl<D> KfEstimate<D>
where
    D: DimName,
    DefaultAllocator: Allocator<D> + Allocator<D, D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
{
    /// Create a zero [KfEstimate]
    pub fn zero() -> Self {
        let x = OVector::<f64, D>::zeros();
        let p = OMatrix::<f64, D, D>::zeros();
        Self { p, x }
    }

    /// Reset this [KfEstimate]
    pub fn reset(&mut self) {
        for i in 0..self.x.nrows() {
            self.x[i] = 0.0;

            for j in 0..self.p.ncols() {
                self.p[(i, j)] = 0.0;
            }
        }
    }

    /// Create new [KfEstimate]
    pub fn from_dynamic(x: DVector<f64>, p: DMatrix<f64>) -> Self {
        let x_rows = x.nrows();
        let (p_rows, p_cols) = (p.nrows(), p.ncols());

        assert_eq!(x_rows, D::USIZE, "internal error: invalid dimension setup!");
        assert_eq!(p_rows, D::USIZE, "internal error: invalid dimension setup!");
        assert_eq!(p_cols, D::USIZE, "internal error: invalid dimension setup!");
        assert_eq!(p_rows, p_cols, "P is not square!");

        let mut x_stored = OVector::<f64, D>::zeros();
        let mut p_stored = OMatrix::<f64, D, D>::zeros();

        for i in 0..D::USIZE {
            x_stored[i] = x[i];

            for j in 0..D::USIZE {
                p_stored[(i, j)] = p[(i, j)];
            }
        }

        Self {
            p: p_stored,
            x: x_stored,
        }
    }

    /// Initializes [KfEstimate] from static matrices
    pub fn from_static(x: OVector<f64, D>, p: OMatrix<f64, D, D>) -> Self {
        Self { p, x }
    }
}

#[derive(Clone)]
pub struct Kalman<D>
where
    D: DimName,
    DefaultAllocator: Allocator<D> + Allocator<D, D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
{
    /// True if this [Kalman] filter has been initialized
    pub initialized: bool,

    /// Prediction as [KfEstimate].
    pub predicted: KfEstimate<D>,
}

impl<D> Kalman<D>
where
    D: DimName,
    DefaultAllocator: Allocator<D> + Allocator<D, D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
{
    /// Create a new [Kalman] filter
    pub fn new() -> Self {
        Self {
            initialized: false,
            predicted: KfEstimate::zero(),
        }
    }

    /// Initialize this [Kalman] filter
    pub fn initialize(
        &mut self,
        f_k: OMatrix<f64, D, D>,
        q_k: OMatrix<f64, D, D>,
        estimate: KfEstimate<D>,
    ) {
        // prediction
        let x_k = f_k.clone() * estimate.x;
        let f_k_t = f_k.transpose();
        let p_k = f_k * estimate.p * f_k_t + q_k;

        self.predicted = KfEstimate { x: x_k, p: p_k };
        self.initialized = true;
    }

    /// Reset this [Kalman] filter
    pub fn reset(&mut self) {
        self.initialized = false;
        self.predicted = KfEstimate::zero();
    }

    /// Run this [Kalman] filter, returning new [KfEstimate].
    ///
    /// ## Input
    /// - f_k: Dynamics [OMatrix]
    /// - g_k: G [DMatrix]
    /// - w_k: W [DMatrix]
    /// - q_k: Q [DMatrix]
    /// - y_k: Measurement [DVector]
    pub fn run(
        &mut self,
        f_k: &OMatrix<f64, D, D>,
        g_k: &DMatrix<f64>,
        w_k: &DMatrix<f64>,
        q_k: &OMatrix<f64, D, D>,
        y_k: &DVector<f64>,
    ) -> Result<KfEstimate<D>, Error> {
        let (w_rows, w_cols) = (w_k.nrows(), w_k.ncols());
        let (g_rows, g_cols) = (g_k.nrows(), g_k.ncols());
        let (_, q_cols) = (q_k.nrows(), q_k.ncols());

        let y_rows = y_k.nrows();

        if !self.initialized {
            panic!("internal error: filter not initialized!");
        }

        assert_eq!(w_rows, w_cols, "W is not square");

        assert_eq!(q_cols, g_cols, "invalid Q/G dimensions!");
        assert_eq!(g_cols, D::USIZE, "invalid G setup!");
        assert_eq!(y_rows, g_rows, "invalid Y/G dimensions!");
        assert_eq!(y_rows, w_rows, "invalid Y/W dimensions!");

        let gt = g_k.transpose();

        let p_inv = self
            .predicted
            .p
            .clone()
            .try_inverse()
            .ok_or(Error::MatrixInversion)?;

        let p_k = gt.clone() * w_k.clone();
        let p_k = p_k * g_k;
        let p_k = p_k + p_inv.clone();
        let p_k = p_k.try_inverse().ok_or(Error::MatrixInversion)?;

        let p_inv_x = p_inv.clone() * self.predicted.x.clone();

        let x_k = gt * w_k;
        let x_k = x_k * y_k;
        let x_k = x_k + p_inv_x;
        let x_k = p_k.clone() * x_k;

        // prediction
        let x_k1 = f_k.clone() * x_k.clone();

        let p_k1 = f_k.clone() * p_k.clone() * f_k.transpose() + q_k;

        self.predicted = KfEstimate { x: x_k1, p: p_k1 };

        Ok(KfEstimate { x: x_k, p: p_k })
    }
}

#[cfg(test)]
mod test {
    use super::KfEstimate;

    use nalgebra::{DimName, OMatrix, OVector, U4};

    #[test]
    fn static_kf_estimate() {
        let x = OVector::<f64, U4>::from_column_slice(&[1.0, 2.0, 3.0, 4.0]);

        let p = OMatrix::<f64, U4, U4>::from_column_slice(&[
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        ]);

        let kfe = KfEstimate::from_static(x, p);

        for i in 0..U4::USIZE {
            let x = kfe.x[i] as usize;
            assert_eq!(x, i + 1);

            for j in 0..U4::USIZE {
                let p = kfe.p[(i, j)] as usize;
                assert_eq!(p, i + 1 + j * U4::USIZE);
            }
        }
    }
}
