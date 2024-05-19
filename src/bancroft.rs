//! Brancroft solver
use crate::{prelude::Candidate, solver::Error};

use nalgebra::{Matrix4, Vector4};
use nyx_space::cosmic::SPEED_OF_LIGHT;

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
            cd.len() == 4,
            "can't resolve Bancroft equation: invalid input"
        );

        for i in 0..4 {
            let state = cd[i].state.ok_or(Error::UnresolvedState)?;
            let (x_i, y_i, z_i) = (state.position[0], state.position[1], state.position[2]);
            let r_i = cd[i]
                .prefered_pseudorange()
                .ok_or(Error::MissingPseudoRange)?
                .value;
            let dt_i = cd[i].clock_corr.to_seconds();
            let tgd_i = cd[i].tgd.unwrap_or_default().to_seconds();
            b[(i, 0)] = x_i;
            b[(i, 1)] = y_i;
            b[(i, 2)] = z_i;
            let pr_i = r_i + dt_i * SPEED_OF_LIGHT - tgd_i;
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
            if solutions.0[3] < 0.0 {
                Ok(solutions.1)
            } else if solutions.1[3] < 0.0 {
                Ok(solutions.0)
            } else {
                let rho = (
                    (solutions.0[0].powi(2) + solutions.0[1].powi(2) + solutions.0[2].powi(2))
                        .sqrt(),
                    (solutions.1[0].powi(2) + solutions.1[1].powi(2) + solutions.1[2].powi(2))
                        .sqrt(),
                );
                if rho.0 < rho.1 {
                    Ok(solutions.0)
                } else {
                    Ok(solutions.1)
                }
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
    use crate::prelude::{
        Candidate, Carrier, Constellation, Duration, Epoch, InterpolationResult, Observation, SV,
    };
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
    #[test]
    fn bancroft_solver() {
        let (x0, y0, z0) = (3582105.291, 532589.7313, 5232754.8054);

        let pr = Observation {
            snr: None,
            value: 28776032.260,
            carrier: Carrier::E1,
        };

        let mut cd0 = Candidate::new(
            SV::new(Constellation::default(), 2),
            Epoch::default(),
            Duration::from_seconds(142.784E-6),
            None, // TGD
            vec![pr],
            vec![],
            vec![],
        );
        let st = InterpolationResult::from_position((
            24170352.34904016,
            -16029029.85873581,
            -5905924.153143198,
        ));
        cd0.set_state(st);

        let pr = Observation {
            snr: None,
            value: 24090441.364,
            carrier: Carrier::E1,
        };

        let mut cd1 = Candidate::new(
            SV::new(Constellation::default(), 3),
            Epoch::default(),
            Duration::from_seconds(-313.533E-6),
            None, // TGD
            vec![pr],
            vec![],
            vec![],
        );
        let st = InterpolationResult::from_position((
            16069642.946692571,
            -8992001.827692423,
            23184746.654093638,
        ));
        cd1.set_state(st);

        let pr = Observation {
            snr: None,
            value: 24762903.616,
            carrier: Carrier::E1,
        };

        let mut cd2 = Candidate::new(
            SV::new(Constellation::default(), 5),
            Epoch::default(),
            Duration::from_seconds(-368.749E-6),
            None, // TGD
            vec![pr],
            vec![],
            vec![],
        );
        let st = InterpolationResult::from_position((
            26119621.94656989,
            7791422.617964384,
            11558902.718228433,
        ));
        cd2.set_state(st);

        let pr = Observation {
            snr: None,
            value: 25537644.454,
            carrier: Carrier::E1,
        };

        let mut cd3 = Candidate::new(
            SV::new(Constellation::default(), 8),
            Epoch::default(),
            Duration::from_seconds(6.158955E-3),
            None, // TGD
            vec![pr],
            vec![],
            vec![],
        );
        let st = InterpolationResult::from_position((
            -3601205.0295727667,
            -20311399.087870672,
            21230831.216778148,
        ));
        cd3.set_state(st);

        let pool = vec![cd0, cd1, cd2, cd3];
        let solver = Bancroft::new(&pool);
        assert!(
            solver.is_ok(),
            "failed to create bancroft solver: {}",
            solver.err().unwrap()
        );
        let solver = solver.unwrap();
        let output = solver.resolve();
        assert!(
            output.is_ok(),
            "bancroft solver failure: {}",
            output.err().unwrap()
        );
        let output = output.unwrap();

        let x_err = (output[0] - x0).abs();
        let y_err = (output[1] - y0).abs();
        let z_err = (output[2] - z0).abs();
        assert!(x_err < 100.0, "bancroft solver error: x error too large");
        assert!(y_err < 100.0, "bancroft solver error: x error too large");
        assert!(z_err < 100.0, "bancroft solver error: x error too large");
    }
} //
  //
