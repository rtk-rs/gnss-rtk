use crate::{
    bias::{Bias, BiasRuntime},
    prelude::TimeScale,
};

use std::f64::consts::PI;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Ionopheric delay components that we propose.
#[derive(Default, Clone, Copy)]
pub enum IonosphereModel {
    /// Unknown
    #[default]
    Unknown,
    /// Provide a [KbModel]
    KbModel(KbModel),
    // /// Provide a [NgModel]
    // NgModel(NgModel),
    // /// Provide a [BdModel]
    // BdModel(BdModel),
}

impl Bias for IonosphereModel {
    fn bias_m(&self, rtm: &BiasRuntime) -> f64 {
        match self {
            Self::Unknown => 0.0,
            Self::KbModel(kb) => kb.bias_m(rtm),
            // Self::BdModel(bd) => bd.bias_m(rtm),
            // Self::NgModel(ng) => ng.bias_m(rtm),
        }
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

impl Bias for KbModel {
    fn bias_m(&self, rtm: &BiasRuntime) -> f64 {
        const PHI_P: f64 = 78.3;
        const R_EARTH: f64 = 6378.0;
        const LAMBDA_P: f64 = 291.0;
        const L1_F: f64 = 1575.42E6;

        let (phi_u, lambda_u) = (
            rtm.rx_lat_long_alt_deg_deg_km.0.to_radians(),
            rtm.rx_lat_long_alt_deg_deg_km.1.to_radians(),
        );

        let fract = R_EARTH / (R_EARTH + self.h_km);

        let (elev_rad, azim_rad) = (
            rtm.sv_elevation_azimuth_deg_deg.0.to_radians(),
            rtm.sv_elevation_azimuth_deg_deg.1.to_radians(),
        );

        let t_gpst = rtm
            .t
            .to_duration_in_time_scale(TimeScale::GPST)
            .to_seconds();

        let psi = PI / 2.0 - elev_rad - (fract * elev_rad.cos()).asin();
        let phi_i = (phi_u.sin() * psi.cos() + phi_u.cos() * psi.sin() * azim_rad.cos()).asin();
        let lambda_i = lambda_u + azim_rad.sin() * psi / phi_i.cos();
        let phi_m = (phi_i.sin() * PHI_P.sin()
            + phi_i.cos() * PHI_P.cos() * (lambda_i - LAMBDA_P).cos())
        .asin();

        let mut t_s = 43.2E3 * lambda_i / PI + t_gpst;
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
        let f = 1.0 / ((1.0 - fract * elev_rad.cos()).powi(2)).sqrt();
        let i_1 = match x_i < PI / 2.0 {
            true => 5.0 * 10E-9 + a_i * x_i.cos(),
            false => f * 5.0 * 10E-9,
        };

        i_1 * (L1_F / rtm.frequency_hz).powi(2)
    }
}

// /// Nequick-G Model: is not supported yet.
// #[derive(Clone, Copy, Default, Debug)]
// pub struct NgModel {
//     /// alpha coefficients
//     pub a: (f64, f64, f64),
// }

// impl NgModel {
//     pub(crate) fn bias_m(&self, _rtm: &RuntimeParams) -> f64 {
//         //let phi = deg2rad(rtm.apriori_geo.0);
//         //let mu = inclination / phi.cos().sqrt();
//         0.0
//     }
// }

// /// BDGIM Model: is not supported yet.
// #[derive(Clone, Copy, Default, Debug)]
// pub struct BdModel {
//     /// Alpha coefficients in TECu
//     pub alpha: (f64, f64, f64, f64, f64, f64, f64, f64, f64),
// }

// impl BdModel {
//     pub(crate) fn value(&self, _rtm: &RuntimeParams) -> f64 {
//         //let phi = deg2rad(rtm.apriori_geo.0);
//         //let mu = inclination / phi.cos().sqrt();
//         0.0
//     }
// }

/// Modeled (estimated) or measured bias
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum IonosphereBias {
    /// Measured bias in [m]
    Measured(f64),
    /// Modelled bias in [m]
    Modeled(f64),
}

impl Default for IonosphereBias {
    /// Builds a Default "Modeled" Bias with 0 value
    fn default() -> Self {
        Self::Modeled(0.0)
    }
}

impl IonosphereBias {
    /// Returns Bias value in [m]
    pub fn value(&self) -> f64 {
        match self {
            Self::Measured(bias) => *bias,
            Self::Modeled(bias) => *bias,
        }
    }
    /// Builds a measured bias in [m]
    pub(crate) fn measured(meas_m: f64) -> Self {
        Self::Measured(meas_m)
    }
    /// Builds a modeled bias in [m]
    pub(crate) fn modeled(model_m: f64) -> Self {
        Self::Modeled(model_m)
    }
}
