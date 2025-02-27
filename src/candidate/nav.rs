//! Position solving candidate
use hifitime::Unit;
use itertools::Itertools;
use log::debug;
use map_3d::{ecef2aer, ecef2geodetic, Ellipsoid};

use nyx::{
    cosmic::SPEED_OF_LIGHT_M_S,
    linalg::{DVector, OMatrix, OVector, U8},
};

use crate::{
    bias::RuntimeParams as BiasRuntimeParams,
    constants::Constants,
    navigation::SVInput,
    prelude::{
        Candidate, Carrier, Config, Duration, Epoch, Error, IonoComponents, IonosphereBias, Method,
        Orbit, TropoComponents, TropoModel, Vector3, SV,
    },
};

impl Candidate {
    /// Fills the Matrix and prepare for resolution.
    /// The matrix contribution is highly dependent on the
    /// configuration setup.
    /// This contribution's accuracy depends on the current
    /// conditions and data quality.
    pub(crate) fn matrix_contribution(
        &self,
        cfg: &Config,
        row: usize,
        y: &mut DVector<f64>,
        g: &mut OMatrixXx4<f64>,
        apriori: (f64, f64, f64),
    ) -> Result<SVInput, Error> {
        // When RTK is feasible, it is always prefered,
        // because it is much easier and has immediate accuracy.
        if self.is_rtk_compatible() {
            self.rtk_matrix_contribution(cfg, row, y, g)
        } else {
            self.ppp_matrix_contribution(cfg, row, y, g, apriori)
        }
    }

    /// Matrix conribution, in case of PPP resolution.
    /// The only requirement being that orbital state needs
    /// to be fully resolved. This contribution's accuracy
    /// (to the general process) depends on the current setup
    /// and provided data (condition and quality).
    fn ppp_matrix_contribution(
        &self,
        cfg: &Config,
        row: usize,
        y: &mut OVector<f64, U8>,
        g: &mut OMatrix<f64, U8, U8>,
        apriori: (f64, f64, f64),
    ) -> Result<SVInput, Error> {
        let mut sv_input = SVInput::default();
        let orbit = self.orbit.ok_or(Error::UnresolvedState)?;
        let state = orbit.to_cartesian_pos_vel() * 1.0E3;

        let (x0_m, y0_m, z0_m) = apriori;
        let (sv_x_m, sv_y_m, sv_z_m) = (state[0], state[1], state[2]);

        let (lat0, lon0, alt0) = ecef2geodetic(x0_m, y0_m, z0_m, Ellipsoid::WGS84);
        let (azimuth, elevation, _) =
            ecef2aer(sv_x_m, sv_y_m, sv_z_m, lat0, lon0, alt0, Ellipsoid::WGS84);
        sv_input.elevation = elevation.to_degrees();
        sv_input.azimuth = azimuth.to_degrees();

        let mut rho =
            ((sv_x_m - x0_m).powi(2) + (sv_y_m - y0_m).powi(2) + (sv_z_m - z0_m).powi(2)).sqrt();

        if cfg.modeling.relativistic_path_range {
            let mu = Constants::EARTH_GRAVITATION;
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

        let (x_i, y_i, z_i) = (
            (x0_m - sv_x_m) / rho,
            (y0_m - sv_y_m) / rho,
            (z0_m - sv_z_m) / rho,
        );

        g[(row, 0)] = x_i;
        g[(row, 1)] = y_i;
        g[(row, 2)] = z_i;
        g[(row, 3)] = 1.0_f64;

        let mut models = 0.0_f64;

        if cfg.modeling.sv_clock_bias {
            let corr = self.clock_corr.ok_or(Error::UnknownClockCorrection)?;
            sv_input.clock_correction = Some(corr.duration);
            models -= corr.duration.to_seconds() * SPEED_OF_LIGHT_M_S;
        }

        if cfg.modeling.sv_total_group_delay {
            models -= self.tgd.unwrap_or_default().to_seconds();
        }

        let (pr, frequency) = match cfg.method {
            Method::SPP => {
                let pr = self
                    .prefered_pseudorange()
                    .ok_or(Error::MissingPseudoRange)?;
                (pr.pseudo.unwrap(), pr.carrier.frequency())
            },
            Method::CPP | Method::PPP => {
                let pr = self
                    .code_if_combination()
                    .ok_or(Error::PseudoRangeCombination)?;
                (pr.value, pr.rhs.frequency())
            },
        };

        // cable delays
        if cfg.modeling.cable_delay {
            if let Some(delay) = cfg.externalref_delay {
                models -= delay * SPEED_OF_LIGHT_M_S;
            }
            // TODO: frequency dependent delays
            for delay in &cfg.int_delay {
                if delay.frequency == frequency {
                    models += delay.delay * SPEED_OF_LIGHT_M_S;
                }
            }
        }

        // tropo
        if cfg.modeling.tropo_delay {
            let bias = self.tropo_bias;
            models += bias;
            sv_input.tropo_bias = Some(bias);
        }

        // iono
        if cfg.modeling.iono_delay {
            let bias = self.iono_bias;
            models += bias;
            if cfg.method == Method::SPP {
                sv_input.iono_bias = Some(IonosphereBias::modeled(bias));
            } else {
                sv_input.iono_bias = Some(IonosphereBias::measured(bias));
            }
        }

        y[row] = pr - rho - models;
        Ok(sv_input)
    }

    /// Matrix contribution, in case of RTK resolution.
    fn rtk_matrix_contribution(
        &self,
        _: &Config,
        _: usize,
        _: &mut OVector<f64, U8>,
        _: &mut OMatrix<f64, U8, U8>,
    ) -> Result<SVInput, Error> {
        Err(Error::MissingRemoteRTKObservation(self.t, self.sv))
    }
}
