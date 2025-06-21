use crate::error::Error;

use nalgebra::{
    allocator::Allocator, DMatrix, DVector, DefaultAllocator, DimName, Dyn, Matrix, OMatrix,
    OVector, VecStorage, U1,
};

#[derive(Clone)]
pub struct KfEstimate {
    /// P [DMatrix]
    pub p: DMatrix<f64>,

    /// x [DVector]
    pub x: DVector<f64>,
}

impl KfEstimate {
    /// Create a zero [KfEstimate]
    pub fn zero(size: usize) -> Self {
        let x = DVector::<f64>::zeros(size);
        let p = DMatrix::<f64>::zeros(size, size);
        Self { p, x }
    }

    pub fn resize_mut(&mut self, r: usize, c: usize) {
        self.p.resize_mut(r, c, 0.0);
        let resized = self.x.clone().resize(r, 1, 0.0);
        self.x = DVector::from_row_slice(resized.as_slice());
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

    /// Initializes new [KfEstimate]
    pub fn new(x: &DVector<f64>, p: &DMatrix<f64>) -> Self {
        let x_rows = x.nrows();
        let (p_rows, p_cols) = (p.nrows(), p.ncols());

        assert_eq!(x_rows, p_cols, "P/x dimension issue!");
        assert_eq!(p_rows, p_cols, "P is not square!");

        Self {
            p: p.clone(),
            x: x.clone(),
        }
    }

    // /// Initializes [KfEstimate] from dyn/static matrices
    // pub fn from_dyn_static<D: DimName>(
    //     x: Matrix<f64, D, Dyn, VecStorage<f64, D, Dyn>>,
    //     p: OMatrix<f64, D, D>,
    // ) -> Self
    // where
    //     D: DimName,
    //     DefaultAllocator: nalgebra::allocator::Allocator<D>,
    //     DefaultAllocator: nalgebra::allocator::Allocator<D, D>,
    // {
    //     let (x_rows, x_cols) = (x.nrows(), x.ncols());
    //     let (p_rows, p_cols) = (p.nrows(), p.ncols());

    //     assert_eq!(x_cols, 1, "x is not a column vector!");
    //     assert_eq!(x_rows, p_cols, "P/x dimension issue!");
    //     assert_eq!(p_rows, p_cols, "P is not square!");

    //     Self {
    //         x: DMatrix::from_column_slice(D::USIZE, U1::USIZE, x.as_slice()),
    //         p: DMatrix::from_column_slice(D::USIZE, D::USIZE, p.as_slice()),
    //     }
    // }

    /// Initializes [KfEstimate] from static matrices
    pub fn from_static<D: DimName>(x: OVector<f64, D>, p: OMatrix<f64, D, D>) -> Self
    where
        D: DimName,
        DefaultAllocator: nalgebra::allocator::Allocator<D>,
        DefaultAllocator: nalgebra::allocator::Allocator<D, D>,
    {
        let (x_rows, _) = (x.nrows(), x.ncols());
        let (p_rows, p_cols) = (p.nrows(), p.ncols());

        assert_eq!(x_rows, p_cols, "P/x dimension issue!");
        assert_eq!(p_rows, p_cols, "P is not square!");

        Self {
            x: DVector::from_column_slice(x.as_slice()),
            p: DMatrix::from_column_slice(D::USIZE, D::USIZE, p.as_slice()),
        }
    }
}
