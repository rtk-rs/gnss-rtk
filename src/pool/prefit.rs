use crate::{
    pool::Pool,
    prelude::{Config, Method},
};

use log::{debug, error};

impl Pool {
    /// Apply Pre fit criterias
    pub fn pre_fit(&mut self, cfg: &Config) {
        if let Some(min_snr) = cfg.min_snr {
            self.pre_fit_min_snr(min_snr);
        }

        self.pre_fit_navi_compatible(&cfg.method);
    }

    /// Pre fit criterias
    fn pre_fit_min_snr(&mut self, min_snr: f64) {
        self.inner.retain_mut(|cd| {
            cd.min_snr_mask(min_snr);
            !cd.observations.is_empty()
        });
    }

    fn pre_fit_navi_compatible(&mut self, method: &Method) {
        self.inner.retain(|cd| match method {
            Method::SPP => {
                if cd.l1_pseudo_range().is_some() {
                    true
                } else {
                    error!("{} ({}) missing pseudo range observation", cd.t, cd.sv);
                    false
                }
            },
            Method::CPP => {
                if cd.cpp_compatible() {
                    true
                } else {
                    debug!("{} ({}) missing secondary frequency", cd.t, cd.sv);
                    false
                }
            },
            Method::PPP => {
                if cd.ppp_compatible() {
                    true
                } else {
                    debug!("{} ({}) missing phase or phase combination", cd.t, cd.sv);
                    false
                }
            },
        });
    }
}
