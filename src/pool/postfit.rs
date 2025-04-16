use crate::{
    ambiguity::Solver as AmbiguitySolver,
    cfg::{Config, Method},
    constants::Constants,
    navigation::apriori::Apriori,
    pool::Pool,
    prelude::{Almanac, Vector3, SUN_J2000},
};

use log::{debug, error, info};

use hifitime::Unit;

impl Pool {
    /// Apply Post fit criterias
    pub fn post_fit(&mut self, almanac: &Almanac, cfg: &Config, apriori: &Apriori) {
        self.post_fit_attitudes(almanac, cfg, apriori);
        self.post_fit_velocities(cfg.modeling.relativistic_clock_bias);

        if let Some(max_occultation) = cfg.max_sv_occultation_percent {
            self.post_fit_eclipse(almanac, max_occultation);
        }

        if cfg.code_smoothing > 0 || cfg.method == Method::PPP {
            self.post_fit_ambiguity_solving();
        }

        if cfg.code_smoothing > 0 {
            self.post_fit_code_smoothing();
        }
    }

    /// Apply Attitudes Post fit
    fn post_fit_attitudes(&mut self, almanac: &Almanac, cfg: &Config, apriori: &Apriori) {
        let rx_orbit = apriori.to_orbit();
        self.inner
            .retain_mut(|cd| match cd.orbital_attitude_fixup(almanac, rx_orbit) {
                Ok(_) => true,
                Err(e) => {
                    error!("{}({}) - orbital fixup: {}", apriori.t, cd.sv, e);
                    false
                },
            });

        let min_elev_deg = cfg.min_sv_elev.unwrap_or(0.0);
        let min_azim_deg = cfg.min_sv_azim.unwrap_or(0.0);
        let max_azim_deg = cfg.max_sv_azim.unwrap_or(360.0);

        self.inner.retain(|cd| {
            if let Some((elev, azim)) = cd.attitude() {
                if elev < min_elev_deg {
                    info!("{}({}) - rejected (below elevation mask)", cd.t, cd.sv);
                    false
                } else if azim < min_azim_deg {
                    info!("{}({}) - rejected (below azimuth mask)", cd.t, cd.sv);
                    false
                } else if azim > max_azim_deg {
                    info!("{}({}) - rejected (above azimuth mask)", cd.t, cd.sv);
                    false
                } else {
                    debug!("{}({}) - elev={:.3}° azim={:.3}°", cd.t, cd.sv, elev, azim);
                    true
                }
            } else {
                true
            }
        });
    }

    /// Velocities fit
    fn post_fit_velocities(&mut self, relativistic_clock_bias: bool) {
        let mu = Constants::EARTH_GRAVITATION;
        let w_e = Constants::EARTH_SEMI_MAJOR_AXIS_WGS84;

        for cd in self.inner.iter_mut() {
            if let Some(sv_orbit) = &mut cd.orbit {
                if let Some(past_elected) = self.past.iter().find(|elected| elected.sv == cd.sv) {
                    let pos_vel_km_s = sv_orbit.to_cartesian_pos_vel();

                    let dt_s = (cd.t_tx - past_elected.t_tx).to_seconds();
                    let past_orbit = past_elected.orbit.unwrap();
                    let pos_vel_z1_km_s = past_orbit.to_cartesian_pos_vel();

                    let vel_km_s = (
                        (pos_vel_km_s[0] - pos_vel_z1_km_s[0]) / dt_s,
                        (pos_vel_km_s[1] - pos_vel_z1_km_s[1]) / dt_s,
                        (pos_vel_km_s[2] - pos_vel_z1_km_s[2]) / dt_s,
                    );

                    debug!("{} ({}) : vel {:?} km/s", cd.t, cd.sv, vel_km_s);

                    *sv_orbit = sv_orbit
                        .with_velocity_km_s(Vector3::new(vel_km_s.0, vel_km_s.1, vel_km_s.2));

                    if let Some(clock_corr) = &mut cd.clock_corr {
                        if relativistic_clock_bias && clock_corr.needs_relativistic_correction {
                            let pos_m = (
                                pos_vel_km_s[0] * 1.0E3,
                                pos_vel_km_s[1] * 1.0E3,
                                pos_vel_km_s[2] * 1.0E3,
                            );

                            let vel_m_s =
                                (vel_km_s.0 * 1.0E3, vel_km_s.1 * 1.0E3, vel_km_s.2 * 1.0E3);

                            let r_v_sat =
                                pos_m.0 * vel_m_s.0 + pos_m.1 * vel_m_s.1 + pos_m.2 * vel_m_s.2;

                            let bias = -2.0 * r_v_sat
                                / Constants::SPEED_OF_LIGHT_M_S
                                / Constants::SPEED_OF_LIGHT_M_S
                                * Unit::Second;

                            // let ea_deg = sv_orbit.ea_deg().map_err(Error::Physics)?;

                            // let ea_rad = ea_deg.to_radians();
                            // let gm = (w_e * mu).sqrt();
                            // let ecc = sv_orbit.ecc().map_err(Error::Physics)?;

                            // let bias = -2.0_f64 * ecc * ea_rad.sin() * gm
                            //     / SPEED_OF_LIGHT_M_S
                            //     / SPEED_OF_LIGHT_M_S
                            //     * Unit::Second;

                            debug!("{} ({}) : relativistic clock bias: {}", cd.t, cd.sv, bias);
                            // clock_corr.duration += bias;
                        }
                    } //clockbias
                } //velocity
            }
        }
    }

    fn post_fit_eclipse(&mut self, almanac: &Almanac, max_occultation: f64) {
        self.inner.retain(|cd| {
            let orbit = cd.orbit.unwrap();
            match almanac.occultation(SUN_J2000, self.earth_cef, orbit, None) {
                Ok(occultation) => occultation.percentage < max_occultation,
                Err(e) => {
                    error!("(anise) eclipse error: {}", e);
                    false
                },
            }
        });
    }

    fn post_fit_ambiguity_solving(&mut self) {
        self.inner.retain_mut(|cd| {
            if let Some(input) = cd.ambiguity_input() {
                let output = if let Some(solver) = self.solver.get_mut(&cd.sv) {
                    let out = solver.solve(input);
                    out
                } else {
                    let mut solver = AmbiguitySolver::new();
                    let out = solver.solve(input);

                    self.solver.insert(cd.sv, solver);
                    out
                };

                if let Some(output) = output {
                    debug!(
                        "{} ({}) - n_1={}(\u{03c3}={}) n_2={}(\u{03c3}w={})",
                        cd.t, cd.sv, output.n1, 0.0, output.n2, 0.0,
                    );

                    cd.update_ambiguities(output);
                    true
                } else {
                    debug!("{}({}) - phase tracking", cd.t, cd.sv);
                    false
                }
            } else {
                error!(
                    "{}({}) - phase tracking is not unfeasible (missing observations)",
                    cd.t, cd.sv
                );
                false
            }
        });
    }

    fn post_fit_code_smoothing(&mut self) {
        for cd in self.inner.iter_mut() {
            for sv_observ in cd.observations.iter_mut() {
                if sv_observ.carrier.is_l1() {
                    let lambda_1 = sv_observ.carrier.wavelength();
                    let n_1 = sv_observ.ambiguity.unwrap_or_default();
                    if n_1 != 0.0 {
                        if let Some(c_n) = &mut sv_observ.pseudo_range_m {
                            if let Some(l_n) = sv_observ.phase_range_m {
                                *c_n = self.smoother.smoothing(
                                    sv_observ.carrier,
                                    cd.sv,
                                    *c_n,
                                    n_1,
                                    lambda_1,
                                    l_n,
                                );
                            }
                        }
                    }
                } else {
                    let lambda_j = sv_observ.carrier.wavelength();
                    let n_j = sv_observ.ambiguity.unwrap_or_default();
                    if n_j != 0.0 {
                        if let Some(c_n) = &mut sv_observ.pseudo_range_m {
                            if let Some(l_n) = sv_observ.phase_range_m {
                                *c_n = self.smoother.smoothing(
                                    sv_observ.carrier,
                                    cd.sv,
                                    *c_n,
                                    n_j,
                                    lambda_j,
                                    l_n,
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}
