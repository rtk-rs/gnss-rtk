use crate::bias::RuntimeParam;
use crate::prelude::{Epoch, TimeScale};
use map_3d::deg2rad;
use std::f64::consts::PI;

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

impl IonosphericBias {
    pub(crate) fn bias(&self, rtm: &RuntimeParam) -> Option<f64> {
        if let Some(stec) = self.stec_meas {
            // TODO
            // let alpha = 40.3 * 10E16 / frequency / frequency;
            None
        } else if let Some(kb) = self.kb_model {
            const PHI_P: f64 = 78.3;
            const R_EARTH: f64 = 6378.0;
            const LAMBDA_P: f64 = 291.0;
            const L1_F: f64 = 1575.42E6;

            let (lat_ddeg, lon_ddeg, _) = rtm.apriori_geo;

            let fract = R_EARTH / (R_EARTH + kb.h_km);
            let (elev, azim) = (deg2rad(rtm.elevation), deg2rad(rtm.azimuth));
            let (phi_u, lambda_u) = (deg2rad(lat_ddeg), deg2rad(lon_ddeg));

            let t_gps = rtm
                .t
                .to_duration_in_time_scale(TimeScale::GPST)
                .to_seconds();
            let psi = PI / 2.0 - elev - (fract * elev.cos()).asin();
            let phi_i = (phi_u.sin() * psi.cos() + phi_u.cos() * psi.sin() * azim.cos()).asin();
            let lambda_i = lambda_u + azim.sin() * psi / phi_i.cos();
            let phi_m = (phi_i.sin() * PHI_P.sin()
                + phi_i.cos() * PHI_P.cos() * (lambda_i - LAMBDA_P).cos())
            .asin();

            let mut t_s = 43.2E3 * lambda_i / PI + t_gps;
            if t_s > 86.4E3 {
                t_s -= 86.4E3;
            } else if t_s < 0.0 {
                t_s += 86.4E3;
            }

            let mut a_i = kb.alpha.0 * (phi_m / PI).powi(0)
                + kb.alpha.1 * (phi_m / PI).powi(1)
                + kb.alpha.2 * (phi_m / PI).powi(2)
                + kb.alpha.3 * (phi_m / PI).powi(3);
            if a_i < 0.0 {
                a_i = 0.0_f64;
            }

            let mut p_i = kb.beta.0 * (phi_m / PI).powi(0)
                + kb.beta.1 * (phi_m / PI).powi(1)
                + kb.beta.2 * (phi_m / PI).powi(2)
                + kb.beta.3 * (phi_m / PI).powi(3);
            if p_i < 72.0E3 {
                p_i = 72.0E3;
            }

            let x_i = 2.0 * PI * (t_s - 50400.0) / p_i;
            let f = 1.0 / ((1.0 - fract * elev.cos()).powi(2)).sqrt();
            let i_1 = match x_i < PI / 2.0 {
                true => 5.0 * 10E-9 + a_i * x_i.cos(),
                false => f * 5.0 * 10E-9,
            };

            Some(i_1 * (L1_F / rtm.frequency).powi(2))
        } else {
            None
        }
    }
}
