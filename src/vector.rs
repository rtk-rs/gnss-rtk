#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use nalgebra::Vector3;

#[derive(Debug, Default, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Vector3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl From<(f64, f64, f64)> for Vector3D {
    fn from(v: (f64, f64, f64)) -> Self {
        Self {
            x: v.0,
            y: v.1,
            z: v.2,
        }
    }
}

impl From<Vector3<f64>> for Vector3D {
    fn from(vec: Vector3<f64>) -> Self {
        Self {
            x: vec[0],
            y: vec[1],
            z: vec[2],
        }
    }
}

impl Vector3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
    pub(crate) fn to_vec3(&self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.z)
    }
}
