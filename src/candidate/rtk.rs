use crate::prelude::{Bias, Candidate, Config, Epoch, Error, Method, Vector3};

impl Candidate {
    /// Measurement vector contribution.
    pub(crate) fn rtk_vector_contribution<B: Bias>(
        &self,
        t: Epoch,
        cfg: &Config,
        remote: &Self,
    ) -> Result<(f64, f64, f64), Error> {
        let mut rho_m = match cfg.method {
            Method::SPP => self.best_snr_range_m().ok_or(Error::MissingPseudoRange)?,
            Method::PPP | Method::CPP => self
                .code_if_combination()
                .ok_or(Error::PseudoRangeCombination)?,
        };

        match cfg.method {
            Method::SPP => {
                rho_m -= remote.best_snr_range_m().ok_or(Error::MissingPseudoRange)?;
            },
            Method::PPP | Method::CPP => {
                rho_m -= remote
                    .code_if_combination()
                    .ok_or(Error::PseudoRangeCombination)?;
            },
        }

        Ok((rho_m, 1.0, 0.0))
    }

    /// RTK Matrix contribution.
    /// ## Input
    ///  - i: matrix row
    ///  - cfg: [Config] preset
    ///  - x0_y0_z0: position coordinates as ECEF (m)
    ///  - remote_x0y0z0_m
    pub(crate) fn rtk_matrix_contribution(
        &self,
        cfg: &Config,
        dr: f64,
        x0_y0_z0_m: Vector3<f64>,
        base_r0: Vector3<f64>,
    ) -> (f64, f64, f64) {
        let (x0_m, y0_m, z0_m) = (x0_y0_z0_m[0], x0_y0_z0_m[1], x0_y0_z0_m[2]);

        let orbit = self.orbit.unwrap_or_else(|| {
            panic!("internal error: matrix contribution prior vector contribution")
        });

        let pos_vel_m = orbit.to_cartesian_pos_vel() * 1.0E3;

        let sv_r = Vector3::new(pos_vel_m[0], pos_vel_m[1], pos_vel_m[2]);

        let dr = (sv_r - base_r0);
        let rho = dr.norm();
        dr / rho
    }
}
