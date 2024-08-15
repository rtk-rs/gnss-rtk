//! Brancroft solver
use crate::{prelude::Candidate, solver::Error};

use nalgebra::{Matrix4, Vector4};
use nyx_space::cosmic::SPEED_OF_LIGHT_M_S;

pub struct Bancroft {
    a: Vector4<f64>,
    b: Matrix4<f64>,
    m: Matrix4<f64>,
    ones: Vector4<f64>,
}

fn lorentz_4_4(a: Vector4<f64>, b: Vector4<f64>, m: &Matrix4<f64>) -> f64 {
    let scalar = a.transpose() * m * b;
    scalar[(0, 0)]
}

impl Bancroft {
    const EARTHRADIUS_M: f64 = 6_371.0E3;
    fn m_matrix() -> Matrix4<f64> {
        let mut m = Matrix4::<f64>::identity();
        m[(3, 3)] = -1.0;
        m
    }
    fn one_vector() -> Vector4<f64> {
        Vector4::<f64>::new(1.0_f64, 1.0_f64, 1.0_f64, 1.0_f64)
    }
    /// Builds new Bancroft solver
    pub fn new(cd: &[Candidate]) -> Result<Self, Error> {
        let m = Self::m_matrix();
        let mut a = Vector4::<f64>::default();
        let mut b = Matrix4::<f64>::default();
        assert!(
            cd.len() > 3,
            "can't resolve Bancroft equation: invalid input"
        );

        for i in 0..4 {
            let state = cd[i].orbit.ok_or(Error::UnresolvedState)?;
            let (x_i, y_i, z_i) = (
                state.radius_km.x * 1.0E3,
                state.radius_km.y * 1.0E3,
                state.radius_km.z * 1.0E3,
            );
            let r_i = cd[i]
                .prefered_pseudorange()
                .ok_or(Error::MissingPseudoRange)?
                .pseudo
                .unwrap();

            let clock_corr = cd[i].clock_corr.ok_or(Error::UnknownClockCorrection)?;
            let dt_i = clock_corr.duration.to_seconds();
            let tgd_i = cd[i].tgd.unwrap_or_default().to_seconds();

            b[(i, 0)] = x_i;
            b[(i, 1)] = y_i;
            b[(i, 2)] = z_i;
            let pr_i = r_i + dt_i * SPEED_OF_LIGHT_M_S - tgd_i;
            b[(i, 3)] = pr_i;
            a[i] = 0.5 * (x_i.powi(2) + y_i.powi(2) + z_i.powi(2) - pr_i.powi(2));
        }
        Ok(Self {
            a,
            b,
            m,
            ones: Self::one_vector(),
        })
    }
    pub fn resolve(&self) -> Result<Vector4<f64>, Error> {
        let _output = Vector4::<f64>::default();
        let b_inv = self.b.try_inverse().ok_or(Error::MatrixInversionError)?;
        let b_1 = b_inv * self.ones;
        let b_a = b_inv * self.a;
        let a = lorentz_4_4(b_1, b_1, &self.m);
        let b = 2.0 * (lorentz_4_4(b_1, b_a, &self.m) - 1.0);
        let c = lorentz_4_4(b_a, b_a, &self.m);
        let delta = b.powi(2) - 4.0 * a * c;
        if delta > 0.0 {
            let delta_sqrt = delta.sqrt();
            let x = ((-b + delta_sqrt) / 2.0 / a, (-b - delta_sqrt) / 2.0 / a);
            let solutions = (
                self.m * b_inv * (x.0 * self.ones + self.a),
                self.m * b_inv * (x.1 * self.ones + self.a),
            );
            let rho = (
                (solutions.0[0].powi(2) + solutions.0[1].powi(2) + solutions.0[2].powi(2)).sqrt(),
                (solutions.1[0].powi(2) + solutions.1[1].powi(2) + solutions.1[2].powi(2)).sqrt(),
            );

            let err = (
                (rho.0 - Self::EARTHRADIUS_M).abs(),
                (rho.1 - Self::EARTHRADIUS_M).abs(),
            );
            if err.0 < err.1 {
                Ok(solutions.0)
            } else {
                Ok(solutions.1)
            }
        } else if delta < 0.0 {
            Err(Error::BancroftImaginarySolution)
        } else {
            let x = -b / a / 2.0;
            Ok(self.m * b_inv * (x * self.ones + self.a))
        }
    }
}

#[cfg(test)]
mod test {
    use super::{lorentz_4_4, Bancroft};
    use nalgebra::Vector4;
    #[test]
    fn lorentz_product() {
        let a = Vector4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let b = Vector4::<f64>::new(5.0, 6.0, 7.0, 8.0);
        let m = Bancroft::m_matrix();
        assert_eq!(lorentz_4_4(a, b, &m), 6.0);
        assert_eq!(
            lorentz_4_4(a, b, &m),
            a[0] * b[0] + a[1] * b[1] + a[2] * b[2] - a[3] * b[3]
        );
        assert_eq!(
            lorentz_4_4(a, a, &m),
            a[0].powi(2) + a[1].powi(2) + a[2].powi(2) - a[3].powi(2)
        );
    }
}
