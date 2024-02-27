pub mod pvt;
pub mod filter;

use nalgebra::{DVector, DMatrix, Matrix3, Matrix4, Matrix4x1, MatrixXx4, Vector3};

pub struct Input {
    /// G matrix
    pub g: MatrixXx4<f64>,
    /// W matrix
    pub w: DMatrix<f64>,
    /// Y vector
    pub y: DVector<f64>,
}

pub struct Output {
    /// r { x, y, z, dt } vector
    pub r: Vector4<f64>,
    /// Q matrix
    pub q: Matrix4<f64>,
}
