use crate::{
    pool::Pool,
    prelude::{Config, Method, TimeScale},
    time::{AbsoluteTime, Time},
};

use log::{debug, error};

impl Pool {
    /// Apply Pre fit criterias
    pub fn pre_fit<T: Time>(&mut self, cfg: &Config, absolute_time: &AbsoluteTime<T>) {
        if let Some(min_snr) = cfg.min_snr {
            self.pre_fit_min_snr(min_snr);
        }

        self.time_corrections(cfg.timescale, &absolute_time);
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
        for cd in self.candidates().iter() {
            for observation in cd.observations.iter() {
                debug!(
                    "{}({}) - {} observation: c_n={:?} l_n={:?}",
                    cd.t,
                    cd.sv,
                    observation.carrier,
                    observation.pseudo_range_m,
                    observation.phase_range_m
                );
            }
        }

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

    fn time_corrections<T: Time>(&mut self, target: TimeScale, absolute_time: &AbsoluteTime<T>) {
        self.retain_mut(|cd| {
            if cd.t.time_scale == target {
                true
            } else {
                if let Ok(t) = absolute_time.epoch_time_correction(cd.t, target) {
                    let correction = t - cd.t;
                    debug!(
                        "{} - {}: |{}-{}| {} correction",
                        cd.t, cd.sv, cd.t.time_scale, target, correction
                    );
                    cd.t = t;
                    true
                } else {
                    error!(
                        "{} - {} is dropped: unknown |{}-{}| correction",
                        cd.t, cd.sv, cd.t.time_scale, target
                    );
                    false
                }
            }
        });
    }
}
