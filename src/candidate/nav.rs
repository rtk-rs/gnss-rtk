//! Position solving candidate
use log::debug;

use nyx::{
    cosmic::SPEED_OF_LIGHT_M_S,
    linalg::{DimName, Matrix1x4},
};

use crate::{
    constants::Constants,
    navigation::MatrixContribution,
    prelude::{Candidate, Config, Error, Method},
};

impl Candidate {
    /// Fills state [OMatrix] and [OVector] vector from [Candidate] state for (SPP/CPP) Navigation.
    /// ## Input
    ///  - i: matrix row
    ///  - cfg: [Config] preset
    ///  - x0_y0_z0: apriori triplet (m ECEF)
    pub(crate) fn spp_matrix_contribution(
        &self,
        cfg: &Config,
        x0_y0_z0_m: (f64, f64, f64),
    ) -> Result<MatrixContribution, Error> {
        let mu = Constants::EARTH_GRAVITATION;

        let (x0_m, y0_m, z0_m) = x0_y0_z0_m;

        let orbit = self.orbit.ok_or(Error::UnresolvedState)?;

        let pos_vel_m = orbit.to_cartesian_pos_vel() * 1.0E3;
        let (sv_x_m, sv_y_m, sv_z_m) = (pos_vel_m[0], pos_vel_m[1], pos_vel_m[2]);

        let mut rho =
            ((sv_x_m - x0_m).powi(2) + (sv_y_m - y0_m).powi(2) + (sv_z_m - z0_m).powi(2)).sqrt();

        let mut h = Matrix1x4::zeros();

        if cfg.modeling.relativistic_path_range {
            let r_sat = (sv_x_m.powi(2) + sv_y_m.powi(2) + sv_z_m.powi(2)).sqrt();
            let r_0 = (x0_m.powi(2) + y0_m.powi(2) + z0_m.powi(2)).sqrt();

            let r_sat_0 = r_0 - r_sat;

            let dr = 2.0 * mu / SPEED_OF_LIGHT_M_S / SPEED_OF_LIGHT_M_S
                * ((r_sat + r_0 + r_sat_0) / (r_sat + r_0 - r_sat_0)).ln();

            debug!(
                "{}({}) relativistic path range {:.3E}m",
                self.t, self.sv, dr
            );

            rho += dr;
        }

        let (dx_m, dy_m, dz_m) = (
            (x0_m - sv_x_m) / rho,
            (y0_m - sv_y_m) / rho,
            (z0_m - sv_z_m) / rho,
        );

        h[(0, 0)] = dx_m;
        h[(0, 1)] = dy_m;
        h[(0, 2)] = dz_m;
        h[(0, 3)] = SPEED_OF_LIGHT_M_S;

        let iono_compensated = false;

        let range_m = match cfg.method {
            Method::SPP => self
                .best_snr_pseudo_range_m()
                .ok_or(Error::MissingPseudoRange)?,
            Method::CPP => {
                // TODO
                self.best_snr_pseudo_range_m()
                    .ok_or(Error::MissingPseudoRange)?
            },
            Method::PPP => {
                // TODO
                self.best_snr_pseudo_range_m()
                    .ok_or(Error::MissingPseudoRange)?
            },
        };

        let mut bias_m = 0.0;

        if cfg.modeling.sv_clock_bias {
            let dt = self.clock_corr.ok_or(Error::UnknownClockCorrection)?;

            bias_m -= dt.duration.to_seconds() * SPEED_OF_LIGHT_M_S;
        }

        if cfg.modeling.sv_total_group_delay {
            // TODO c * dt ?
            bias_m -= self.tgd.unwrap_or_default().to_seconds() * SPEED_OF_LIGHT_M_S;
        }

        if cfg.modeling.cable_delay {
            if let Some(delay) = cfg.externalref_delay {
                bias_m -= delay * SPEED_OF_LIGHT_M_S;
            }

            for _ in cfg.int_delay.iter() {
                // TODO frequency dependent delays
            }
        }

        if cfg.modeling.iono_delay && !iono_compensated {
            bias_m += self.iono_bias_m;
        }

        if cfg.modeling.tropo_delay {
            bias_m += self.tropo_bias_m;
        }

        Ok(MatrixContribution {
            h,
            b: range_m - rho - bias_m,
        })
    }

    /// Fills state [OMatrix] and [OVector] vector from [Candidate] state for RTK Navigation.
    /// ## Input
    ///  - i: matrix row
    ///  - cfg: [Config] preset
    ///  - x0_y0_z0: apriori triplet (m ECEF)
    pub(crate) fn rtk_matrix_contribution<N: DimName>(
        &self,
        _: usize,
        _: &Config,
        _: (f64, f64, f64),
    ) -> MatrixContribution {
        panic!("rtk not available yet");
    }
}
