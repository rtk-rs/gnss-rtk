//! Position solving candidate
use log::debug;

use crate::{
    bias::IonosphereBias,
    constants::{EARTH_GRAVITATION, SPEED_OF_LIGHT_M_S},
    navigation::SVContribution,
    prelude::{Bias, Candidate, Config, Duration, Epoch, Error, Method, Signal, Vector3},
};

#[derive(Debug, Copy, Clone, Default)]
pub(crate) struct VectorContribution {
    /// Pseudo range contribution, always
    pub pr: f64,

    /// Carrier phase contribution, only in PPP
    pub cp: Option<f64>,

    /// relativistic path range
    pub dr: f64,

    /// sigma
    pub sigma: f64,
}

impl Candidate {
    /// Measurement vector contribution.
    /// This will pass if
    /// - State has been previously resolved
    /// - Range estimate is available
    /// - Preset modeling are matched
    /// ## Input
    /// - t: [Epoch] of computation
    /// - cfg: [Config] preset
    /// - x0_y0_z0: current state (metric)
    /// - rx_lat_long_alt_ddeg_km: state as geodetic lat, long both
    /// in decimal degrees, and altitude above mean sea level (km)
    /// - contribution: mutable [SVContribution]
    /// - bias: [Bias] model
    /// ## Returns
    /// - b_i contribution, r_i contribution, dr: relativistic path range
    pub(crate) fn vector_contribution<B: Bias>(
        &self,
        t: Epoch,
        cfg: &Config,
        x0_y0_z0_m: Vector3<f64>,
        rx_lat_long_alt_deg_deg_km: (f64, f64, f64),
        contribution: &mut SVContribution,
        bias: &B,
    ) -> Result<VectorContribution, Error> {
        let mu = EARTH_GRAVITATION;

        let mut dr = 0.0;
        let mut bias_m = 0.0;

        let (x0_m, y0_m, z0_m) = (x0_y0_z0_m[0], x0_y0_z0_m[1], x0_y0_z0_m[2]);

        let orbit = self.orbit.ok_or(Error::UnresolvedState)?;
        let pos_vel_m = orbit.to_cartesian_pos_vel() * 1.0E3;

        let (elev_deg, azim_deg) = self.attitude().ok_or(Error::UnresolvedState)?;

        contribution.elevation_deg = elev_deg;
        contribution.azimuth_deg = azim_deg;

        let (sv_x_m, sv_y_m, sv_z_m) = (pos_vel_m[0], pos_vel_m[1], pos_vel_m[2]);

        let mut rho =
            ((sv_x_m - x0_m).powi(2) + (sv_y_m - y0_m).powi(2) + (sv_z_m - z0_m).powi(2)).sqrt();

        if cfg.modeling.relativistic_path_range {
            let r_sat = (sv_x_m.powi(2) + sv_y_m.powi(2) + sv_z_m.powi(2)).sqrt();
            let r_0 = (x0_m.powi(2) + y0_m.powi(2) + z0_m.powi(2)).sqrt();

            let r_sat_0 = r_0 - r_sat;

            dr = 2.0 * mu / SPEED_OF_LIGHT_M_S / SPEED_OF_LIGHT_M_S
                * ((r_sat + r_0 + r_sat_0) / (r_sat + r_0 - r_sat_0)).ln();

            rho += dr;
            contribution.relativistic_path_range_m = dr;
        } else {
            contribution.relativistic_path_range_m = 0.0_f64;
        }

        let (frequency_hz, range_m) = match cfg.method {
            Method::SPP => {
                let (carrier, pr) = self.best_snr_range_m().ok_or(Error::MissingPseudoRange)?;
                contribution.signal = Signal::Single(carrier);
                (carrier.frequency_hz(), pr)
            },
            Method::CPP | Method::PPP => {
                let comb = self
                    .code_if_combination()
                    .ok_or(Error::PseudoRangeCombination)?;

                contribution.signal = Signal::Dual((comb.lhs, comb.rhs));
                (comb.rhs.frequency_hz(), comb.value)
            },
        };

        let cp = match cfg.method {
            Method::PPP => {
                let comb = self
                    .phase_if_combination()
                    .ok_or(Error::PhaseRangeCombination)?;

                Some(comb.value)
            },
            _ => None,
        };

        if cfg.modeling.sv_clock_bias {
            let dt = self.clock_corr.ok_or(Error::UnknownClockCorrection)?;
            contribution.clock_correction = Some(dt.duration);
            bias_m -= dt.duration.to_seconds() * SPEED_OF_LIGHT_M_S;

            let correction = self.system_correction.unwrap_or(Duration::ZERO);
            bias_m += correction.to_seconds() * SPEED_OF_LIGHT_M_S;

            debug!(
                "{}({}) - system correction : {}",
                self.t, self.sv, correction
            );
        }

        if cfg.modeling.sv_total_group_delay {
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

        let rtm = self
            .to_bias_runtime(
                t,
                frequency_hz,
                Vector3::new(x0_m, y0_m, z0_m),
                rx_lat_long_alt_deg_deg_km,
            )
            .expect("internal error: unresolved attitude (bias runtime)");

        if cfg.modeling.iono_delay && cfg.method == Method::SPP {
            let iono_bias_m = bias.ionosphere_bias_m(&rtm);

            // TODO: model verification (iono)
            // if iono_bias_m < cfg.max_tropo_bias {

            debug!("{}({}) - iono delay {:.3E}[m]", t, self.sv, iono_bias_m);

            bias_m += iono_bias_m;
            contribution.iono_bias = Some(IonosphereBias::modeled(iono_bias_m));
        }

        if cfg.modeling.tropo_delay {
            let tropo_bias_m = bias.troposphere_bias_m(&rtm);

            if tropo_bias_m < cfg.max_tropo_bias {
                debug!("{}({}) - tropo delay {:.3E}[m]", t, self.sv, tropo_bias_m);

                bias_m += tropo_bias_m;
                contribution.tropo_bias = Some(tropo_bias_m);
            } else {
                return Err(Error::RejectedTropoDelay);
            }
        }

        let pr = range_m - rho - bias_m;

        let cp = if let Some(cp) = cp {
            Some(cp - rho - bias_m) // TODO lambda_n * windup
        } else {
            None
        };

        Ok(VectorContribution {
            cp,
            pr,
            sigma: 1.0, // TODO
            dr,
        })
    }

    /// Matrix contribution.
    /// ## Input
    ///  - i: matrix row
    ///  - cfg: [Config] preset
    ///  - x0_y0_z0: position coordinates as ECEF (m)
    pub(crate) fn matrix_contribution(
        &self,
        cfg: &Config,
        dr: f64,
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
            rho += dr;
        }

        let (dx_m, dy_m, dz_m) = (
            (x0_m - sv_x_m) / rho,
            (y0_m - sv_y_m) / rho,
            (z0_m - sv_z_m) / rho,
        );

        (dx_m, dy_m, dz_m)
    }
}
