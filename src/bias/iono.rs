use crate::bias::Bias;

/// Ionospheric Components to attach
/// any resolution attempt. Fill as much as you can.
/// If this structure is empty, you should then provide observations
/// on at least two carrier signals so the solver can estimate this bias.
pub struct IonosphericBias {
    /// Klobuchar Model to apply
    pub kb_model: Option<KbModel>,
    /// Slan Total Electron Density estimate
    pub stec_meas: Option<f64>,
}

impl Bias for IonosphericBias {
    fn needs_modeling(&self) -> bool {
        self.kb_model.is_none() && self.stec_meas.is_none()
    }
}

/// Klobuchar Model
#[derive(Clone, Copy, Default, Debug)]
pub struct KbModel {
    /// alpha coefficients
    pub alpha: (f64, f64, f64, f64),
    /// beta coefficients
    pub beta: (f64, f64, f64, f64),
    /// ionosphere layer in [km] for this model
    pub h_km: f64,
}

use crate::prelude::{Epoch, TimeScale};
use map_3d::deg2rad;
use std::f64::consts::PI;

impl KbModel {
    pub(crate) fn meters_delay(
        &self,
        t: Epoch,
        e: f64,
        a: f64,
        user_lat_ddeg: f64,
        user_lon_ddeg: f64,
        frequency: f64,
    ) -> f64 {
        const PHI_P: f64 = 78.3;
        const R_EARTH: f64 = 6378.0;
        const LAMBDA_P: f64 = 291.0;
        const L1_F: f64 = 1575.42E6;

        let fract = R_EARTH / (R_EARTH + self.h_km);
        let phi_u = deg2rad(user_lat_ddeg);
        let lambda_u = deg2rad(user_lon_ddeg);

        let t_gps = t.to_duration_in_time_scale(TimeScale::GPST).to_seconds();
        let psi = PI / 2.0 - e - (fract * e.cos()).asin();
        let phi_i = (phi_u.sin() * psi.cos() + phi_u.cos() * psi.sin() * a.cos()).asin();
        let lambda_i = lambda_u + a.sin() * psi / phi_i.cos();
        let phi_m = (phi_i.sin() * PHI_P.sin()
            + phi_i.cos() * PHI_P.cos() * (lambda_i - LAMBDA_P).cos())
        .asin();

        let mut t_s = 43.2E3 * lambda_i / PI + t_gps;
        if t_s > 86.4E3 {
            t_s -= 86.4E3;
        } else if t_s < 0.0 {
            t_s += 86.4E3;
        }

        let mut a_i = self.alpha.0 * (phi_m / PI).powi(0)
            + self.alpha.1 * (phi_m / PI).powi(1)
            + self.alpha.2 * (phi_m / PI).powi(2)
            + self.alpha.3 * (phi_m / PI).powi(3);
        if a_i < 0.0 {
            a_i = 0.0_f64;
        }
        let mut p_i = self.beta.0 * (phi_m / PI).powi(0)
            + self.beta.1 * (phi_m / PI).powi(1)
            + self.beta.2 * (phi_m / PI).powi(2)
            + self.beta.3 * (phi_m / PI).powi(3);
        if p_i < 72.0E3 {
            p_i = 72.0E3;
        }

        let x_i = 2.0 * PI * (t_s - 50400.0) / p_i;
        let f = 1.0 / ((1.0 - fract * e.cos()).powi(2)).sqrt();
        let i_1 = match x_i < PI / 2.0 {
            true => 5.0 * 10E-9 + a_i * x_i.cos(),
            false => f * 5.0 * 10E-9,
        };

        i_1 * (L1_F / frequency).powi(2)
    }
}
