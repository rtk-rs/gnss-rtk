use crate::{
    pool::Pool,
    prelude::{EnvironmentalBias, EphemerisSource, Method, OrbitSource, SpacebornBias},
    time::AbsoluteTime,
};

use log::{debug, error};

impl<EPH: EphemerisSource, ORB: OrbitSource, EB: EnvironmentalBias, SB: SpacebornBias>
    Pool<EPH, ORB, EB, SB>
{
    /// Apply Pre fit criterias
    pub fn pre_fit<T: AbsoluteTime>(&mut self, name: &str, absolute_time: &T) {
        // debug signals
        for cd in self.candidates().iter() {
            for observation in cd.observations.iter() {
                debug!(
                    "{}({}) - {} {} observation: c_n={:?} l_n={:?}",
                    cd.epoch,
                    cd.sv,
                    name,
                    observation.carrier,
                    observation.pseudo_range_m,
                    observation.phase_range_m,
                );
            }
        }

        self.pre_fit_ephemeris_update();

        // apply signal prefit min C/N0
        if let Some(min_snr) = self.cfg.min_snr {
            self.pre_fit_min_c_n0(min_snr);
        }

        // navigation mode compliance
        self.pre_fit_navigation_mode(name);

        // temporal corrections if needed
        self.pre_fit_time_corrections(absolute_time);

        // prefit biases
        self.pre_fit_biases();
    }

    /// Ephemeris update attempt
    fn pre_fit_ephemeris_update(&mut self) {
        for cd in self.inner.iter() {
            // update attempt
            if let Some(data) = self.eph_source.ephemeris_data(cd.epoch, cd.sv) {
                self.eph_buffer.insert(cd.sv, data);
            }
        }
    }

    /// Apply min C/N0 mask
    fn pre_fit_min_c_n0(&mut self, min_snr: f64) {
        self.inner.retain_mut(|cd| {
            cd.min_c_n0_mask(min_snr);
            !cd.observations.is_empty()
        });
    }

    /// Verify navigation mode compatibility.
    fn pre_fit_navigation_mode(&mut self, name: &str) {
        self.inner.retain(|cd| match self.cfg.method {
            Method::SPP => {
                if cd.l1_pseudo_range().is_some() {
                    true
                } else {
                    error!(
                        "{}({}) {} missing pseudo range observation",
                        cd.epoch, cd.sv, name
                    );
                    false
                }
            },
            Method::CPP => {
                if cd.cpp_compatible() {
                    true
                } else {
                    debug!(
                        "{}({}) {} missing secondary frequency",
                        cd.epoch, cd.sv, name
                    );
                    false
                }
            },
            Method::PPP | Method::PPP_AR => {
                if cd.ppp_compatible() {
                    true
                } else {
                    debug!(
                        "{}({}) {} missing phase or phase combination",
                        cd.epoch, cd.sv, name
                    );
                    false
                }
            },
        });
    }

    /// Apply temporal correction if needed
    fn pre_fit_time_corrections<T: AbsoluteTime>(&mut self, absolute_time: &T) {
        for cd in self.inner.iter_mut() {
            if cd.epoch.time_scale != self.cfg.timescale {
                let corrected = absolute_time.epoch_correction(cd.epoch, self.cfg.timescale);
                cd.system_correction = Some(cd.epoch - corrected);
                cd.epoch = corrected;
            }
        }
    }

    /// Calculates the pre fit biases to compensate
    fn pre_fit_biases(&mut self) {
        for cd in self.inner.iter_mut() {
            let rtm = cd.to_partial_bias_runtime();

            if self.cfg.modeling.sv_clock_bias {
                cd.clock_corr = self.space_bias.clock_bias(&rtm);
                debug!(
                    "{}({}) - sat clock correction = {}",
                    cd.epoch, cd.sv, cd.clock_corr.duration
                );
            }

            if self.cfg.modeling.sv_total_group_delay {
                let tgd = self.space_bias.group_delay(&rtm);
                debug!("{}({}) - tgd = {}", cd.epoch, cd.sv, tgd);
                cd.tgd = tgd;
            }
        }
    }
}
