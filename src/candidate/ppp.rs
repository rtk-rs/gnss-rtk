//! Position solving candidate
use log::debug;

use crate::{
    constants::SPEED_OF_LIGHT_M_S,
    navigation::{sv::SVContribution, vector::VectorContribution},
    prelude::{Candidate, Config, Duration, Epoch, Error, Method, Signal, Vector3},
};

impl Candidate {
    /// Measurement vector contribution.
    /// This will pass if
    /// - State has been previously resolved
    /// - Range estimate is available
    /// - Preset modeling are matched
    ///
    /// ## Input
    /// - t: [Epoch] of computation
    /// - cfg: [Config] preset
    /// - x0_y0_z0: current state (metric)
    /// - rx_lat_long_alt_ddeg_km: state as geodetic lat, long both
    /// in decimal degrees, and altitude above mean sea level (km)
    /// - contribution: mutable [SVContribution]
    /// ## Returns
    /// - b_i contribution, r_i contribution, dr: relativistic path range
    pub(crate) fn ppp_vector_contribution(
        &self,
        t: Epoch,
        cfg: &Config,
        two_rows: bool,
        amb: Option<u64>,
        x0_y0_z0_m: Vector3<f64>,
        rx_lat_long_alt_deg_deg_km: (f64, f64, f64),
        contribution: &mut SVContribution,
    ) -> Result<VectorContribution, Error> {
        let mut bias_m = 0.0;
        let mut vec = VectorContribution::default();

        let (x0_m, y0_m, z0_m) = (x0_y0_z0_m[0], x0_y0_z0_m[1], x0_y0_z0_m[2]);

        let orbit = self.orbit.ok_or(Error::UnresolvedState)?;
        let pos_vel_m = orbit.to_cartesian_pos_vel() * 1.0E3;

        let (elev_deg, azim_deg) = self.attitude().ok_or(Error::UnresolvedState)?;

        contribution.elevation_deg = elev_deg;
        contribution.azimuth_deg = azim_deg;

        let (sv_x_m, sv_y_m, sv_z_m) = (pos_vel_m[0], pos_vel_m[1], pos_vel_m[2]);

        let mut rho =
            ((sv_x_m - x0_m).powi(2) + (sv_y_m - y0_m).powi(2) + (sv_z_m - z0_m).powi(2)).sqrt();

        rho += self.relativistic_path_range;
        contribution.relativistic_path_range_m = self.relativistic_path_range;

        let (lambda, range_m) = match cfg.method {
            Method::SPP => {
                let (carrier, pr) = self.best_snr_range_m().ok_or(Error::MissingPseudoRange)?;
                contribution.signal = Signal::Single(carrier);

                (carrier.wavelength(), pr)
            },
            _ => {
                let comb = self
                    .code_if_combination()
                    .ok_or(Error::PseudoRangeCombination)?;

                contribution.signal = Signal::Dual((comb.lhs, comb.rhs));

                let (f1, f2) = (
                    comb.rhs.frequency_hz().powi(2),
                    comb.lhs.frequency_hz().powi(2),
                );

                (SPEED_OF_LIGHT_M_S * (f1 - f2) / f1 / f2, comb.value)
            },
        };

        let frequency_hz = SPEED_OF_LIGHT_M_S / lambda;

        let mut cp = match cfg.method {
            Method::PPP => {
                let comb = self
                    .phase_if_combination()
                    .ok_or(Error::PhaseRangeCombination)?;

                Some(comb.value)
            },
            _ => None,
        };

        if let Some(amb) = amb {
            if let Some(cp) = &mut cp {
                let amb = amb as f64;
                debug!("{}({}) n_amb={}", self.epoch, self.sv, amb.round() as u64);
                *cp -= amb * lambda;
            }
        }

        bias_m -= self.clock_corr.duration.to_seconds() * SPEED_OF_LIGHT_M_S;

        let sys_t = self.system_correction.unwrap_or(Duration::ZERO);

        bias_m += sys_t.to_seconds() * SPEED_OF_LIGHT_M_S;

        if sys_t >= Duration::MIN_POSITIVE {
            debug!(
                "{}({}) - system correction : {}",
                self.epoch, self.sv, sys_t
            );
        }

        bias_m -= self.tgd.to_seconds() * SPEED_OF_LIGHT_M_S;

        if let Some(delay_s) = cfg.externalref_delay_s {
            bias_m -= delay_s * SPEED_OF_LIGHT_M_S;
        }

        for _ in cfg.int_delay.iter() {
            // TODO
        }

        bias_m += self.ionod;
        bias_m += self.tropod;
        vec.sigma = 1.0; // TODO

        let pr = range_m - rho - bias_m;

        let cp = if let Some(cp) = cp {
            // TODO: lambda_n * windup
            Some(cp - rho - bias_m)
        } else {
            None
        };

        if two_rows || cfg.method == Method::PPP {
            if cp.is_none() {
                return Err(Error::MissingPhaseRangeMeasurements)?;
            }
        }

        if two_rows {
            vec.row_1 = pr;
            vec.row_2 = cp.unwrap_or_default();
        } else {
            if cfg.method == Method::PPP {
                vec.row_1 = cp.unwrap_or_default();
            } else {
                vec.row_1 = pr;
            }
        }

        Ok(vec)
    }

    /// Matrix contribution.
    ///
    /// ## Input
    ///  - i: matrix row
    ///  - cfg: [Config] preset
    ///  - x0_y0_z0: position coordinates as ECEF (m)
    pub(crate) fn ppp_matrix_contribution(
        &self,
        cfg: &Config,
        x0_y0_z0_m: Vector3<f64>,
    ) -> (f64, f64, f64) {
        let (x0_m, y0_m, z0_m) = (x0_y0_z0_m[0], x0_y0_z0_m[1], x0_y0_z0_m[2]);

        let orbit = self.orbit.unwrap_or_else(|| {
            panic!("internal error: matrix contribution prior vector contribution")
        });

        let pos_vel_m = orbit.to_cartesian_pos_vel() * 1.0E3;
        let (sv_x_m, sv_y_m, sv_z_m) = (pos_vel_m[0], pos_vel_m[1], pos_vel_m[2]);

        let mut rho =
            ((sv_x_m - x0_m).powi(2) + (sv_y_m - y0_m).powi(2) + (sv_z_m - z0_m).powi(2)).sqrt();

        if cfg.modeling.relativistic_path_range {
            rho += self.relativistic_path_range;
        }

        let (dx_m, dy_m, dz_m) = (
            (x0_m - sv_x_m) / rho,
            (y0_m - sv_y_m) / rho,
            (z0_m - sv_z_m) / rho,
        );

        (dx_m, dy_m, dz_m)
    }
}
