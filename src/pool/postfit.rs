use crate::{
    ambiguity::Solver as AmbiguitySolver,
    constants::{EARTH_GRAVITATION, EARTH_SEMI_MAJOR_AXIS_WGS84, SPEED_OF_LIGHT_M_S},
    navigation::state::State,
    pool::Pool,
    prelude::{
        Almanac, EnvironmentalBias, EphemerisSource, OrbitSource, SpacebornBias, Vector3,
        EARTH_J2000, SUN_J2000,
    },
};

use nalgebra::{allocator::Allocator, DefaultAllocator, DimName};

use log::{debug, error, info};

use hifitime::Unit;

use anise::errors::AlmanacResult;

impl<EPH: EphemerisSource, ORB: OrbitSource, EB: EnvironmentalBias, SB: SpacebornBias>
    Pool<EPH, ORB, EB, SB>
{
    /// Apply Post fit algorithms
    pub fn post_fit<D: DimName>(&mut self, name: &str, state: &State<D>) -> AlmanacResult<()>
    where
        DefaultAllocator: Allocator<D> + Allocator<D, D>,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
    {
        // fix attitudes & derivatives
        self.post_fit_attitudes(name, state);
        self.post_fit_velocities(name);
        self.post_fit_eclipse(name);

        self.post_fit_biases(name, state);

        // PPP workflow specific postfit
        if self.cfg.method.is_ppp() {
            // TODO
            // self.post_fit_phase_windup(almanac, state)?;
            // self.post_fit_ambiguity_solving();
        }

        // if self.cfg.code_smoothing > 0 {
        //     self.post_fit_code_smoothing();
        // }

        Ok(())
    }

    /// Post fit phase windup correction
    fn post_fit_phase_windup<D: DimName>(
        &mut self,
        almanac: &Almanac,
        state: &State<D>,
    ) -> AlmanacResult<()>
    where
        DefaultAllocator: Allocator<D> + Allocator<D, D>,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
    {
        let epoch = self.inner[0].epoch;

        let earth_sun = almanac.transform(SUN_J2000, EARTH_J2000, epoch, None)?;

        let r_sun = Vector3::new(
            earth_sun.radius_km.x * 1.0E3,
            earth_sun.radius_km.y * 1.0E3,
            earth_sun.radius_km.z * 1.0E3,
        );

        // for cd in self.inner.iter_mut() {
        // let prev_correction = self
        //     .past
        //     .iter()
        //     .filter_map(|past| {
        //         if past.sv == cd.sv {
        //             Some(past.windup)
        //         } else {
        //             None
        //         }
        //     })
        //     .reduce(|k, _| k);

        // cd.phase_windup_correction(state, r_sun, prev_correction);
        // }

        Ok(())
    }

    /// Apply Attitudes Post fit
    fn post_fit_attitudes<D: DimName>(&mut self, name: &str, state: &State<D>)
    where
        DefaultAllocator: Allocator<D> + Allocator<D, D>,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
    {
        let rx_orbit = state.to_orbit(self.earth_cef);

        self.inner.retain_mut(
            |cd| match cd.orbital_attitude_fixup(&self.almanac, rx_orbit) {
                Ok(_) => {
                    let elevation_deg = cd.elevation_deg.unwrap();
                    if elevation_deg < 0.0 {
                        error!(
                            "{}({}) {} - invalid negative elevation. Invalid input data!",
                            cd.epoch, cd.sv, name,
                        );
                        false
                    } else {
                        true
                    }
                },
                Err(e) => {
                    error!("{}({}) {} orbital fixup: {}", state.t, cd.sv, name, e);
                    false
                },
            },
        );

        let min_elev_deg = self.cfg.min_sv_elev.unwrap_or(0.0);
        let min_azim_deg = self.cfg.min_sv_azim.unwrap_or(0.0);
        let max_azim_deg = self.cfg.max_sv_azim.unwrap_or(360.0);

        self.inner.retain(|cd| {
            if let Some((elev, azim)) = cd.attitude() {
                if elev < min_elev_deg {
                    info!("{}({}) - rejected (below elevation mask)", cd.epoch, cd.sv);
                    false
                } else if azim < min_azim_deg {
                    info!("{}({}) - rejected (below azimuth mask)", cd.epoch, cd.sv);
                    false
                } else if azim > max_azim_deg {
                    info!("{}({}) - rejected (above azimuth mask)", cd.epoch, cd.sv);
                    false
                } else {
                    debug!(
                        "{}({}) - elev={:.3}° azim={:.3}°",
                        cd.epoch, cd.sv, elev, azim
                    );
                    true
                }
            } else {
                true
            }
        });
    }

    /// Velocities fit
    fn post_fit_velocities(&mut self, name: &str) {
        let mu = EARTH_GRAVITATION;
        let w_e = EARTH_SEMI_MAJOR_AXIS_WGS84;

        for cd in self.inner.iter_mut() {
            let time_of_flight = cd.signal_time_of_flight();

            if let Some(sv_orbit) = &mut cd.orbit {
                if let Some(past_elected) = self.past.iter().find(|elected| elected.sv == cd.sv) {
                    let pos_vel_km_s = sv_orbit.to_cartesian_pos_vel();
                    let past_time_of_flight = past_elected.signal_time_of_flight();

                    let dt_s = (time_of_flight - past_time_of_flight).to_seconds();
                    let past_orbit = past_elected.orbit.unwrap();
                    let pos_vel_z1_km_s = past_orbit.to_cartesian_pos_vel();

                    let vel_km_s = (
                        (pos_vel_km_s[0] - pos_vel_z1_km_s[0]) / dt_s,
                        (pos_vel_km_s[1] - pos_vel_z1_km_s[1]) / dt_s,
                        (pos_vel_km_s[2] - pos_vel_z1_km_s[2]) / dt_s,
                    );

                    debug!(
                        "{}({}) {} inst. velocity {:?} km/s",
                        cd.epoch, cd.sv, name, vel_km_s
                    );

                    *sv_orbit = sv_orbit
                        .with_velocity_km_s(Vector3::new(vel_km_s.0, vel_km_s.1, vel_km_s.2));

                    if self.cfg.modeling.relativistic_clock_bias
                        && cd.clock_corr.needs_relativistic_correction
                    {
                        let pos_m = (
                            pos_vel_km_s[0] * 1.0E3,
                            pos_vel_km_s[1] * 1.0E3,
                            pos_vel_km_s[2] * 1.0E3,
                        );

                        let vel_m_s = (vel_km_s.0 * 1.0E3, vel_km_s.1 * 1.0E3, vel_km_s.2 * 1.0E3);

                        let r_v_sat =
                            pos_m.0 * vel_m_s.0 + pos_m.1 * vel_m_s.1 + pos_m.2 * vel_m_s.2;

                        let bias =
                            -2.0 * r_v_sat / SPEED_OF_LIGHT_M_S / SPEED_OF_LIGHT_M_S * Unit::Second;

                        // let ea_deg = sv_orbit.ea_deg().map_err(Error::Physics)?;

                        // let ea_rad = ea_deg.to_radians();
                        // let gm = (w_e * mu).sqrt();
                        // let ecc = sv_orbit.ecc().map_err(Error::Physics)?;

                        // let bias = -2.0_f64 * ecc * ea_rad.sin() * gm
                        //     / SPEED_OF_LIGHT_M_S
                        //     / SPEED_OF_LIGHT_M_S
                        //     * Unit::Second;

                        debug!(
                            "{}({}) {} : relativistic clock bias: {}",
                            cd.epoch, cd.sv, name, bias
                        );
                        // clock_corr.duration += bias;
                    } //clockbias
                } //velocity
            }
        }
    }

    fn post_fit_eclipse(&mut self, name: &str) {
        self.inner.retain(|cd| {
            let orbit = cd.orbit.unwrap();
            match self
                .almanac
                .occultation(SUN_J2000, self.earth_cef, orbit, None)
            {
                Ok(occultation) => {
                    let retained = occultation.percentage < self.cfg.max_eclipse_rate_percent;

                    if !retained {
                        debug!("{}({}) {} : dropped due to eclipse", cd.epoch, cd.sv, name,);
                    }

                    retained
                },
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
                let mut retain = false;

                let output = if let Some(solver) = self.amb_solver.get_mut(&cd.sv) {
                    let out = solver.solve(&input);
                    out
                } else {
                    let mut solver = AmbiguitySolver::new();
                    let out = solver.solve(&input);

                    self.amb_solver.insert(cd.sv, solver);
                    out
                };

                if let Some(output) = output {
                    debug!(
                        "{}({}) - n_1={}(\u{03c3}={}) n_2={}(\u{03c3}w={})",
                        cd.epoch, cd.sv, output.n1, 0.0, output.n2, 0.0,
                    );

                    // cd.update_ambiguities(output);
                    retain = true;
                } else {
                    debug!("{}({}) - phase tracking", cd.epoch, cd.sv);
                }

                retain
            } else {
                error!(
                    "{}({}) - phase bias tracking not feasible (missing measurements)",
                    cd.epoch, cd.sv
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

    fn post_fit_biases<D: DimName>(&mut self, name: &str, state: &State<D>)
    where
        DefaultAllocator: Allocator<D> + Allocator<D, D>,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
    {
        let rcvr_position_ecef_m = state.to_position_ecef_m();

        for cd in self.inner.iter_mut() {
            let rtm = cd
                .to_bias_runtime(rcvr_position_ecef_m, state.lat_long_alt_deg_deg_km)
                .expect("internal error: post-fit while state is still not resolved!");

            let sat_orbit = cd
                .orbit
                .expect("internal error: post-fit while state is still not resolved!");

            let r_sat_m = sat_orbit.to_cartesian_pos_vel() * 1.0E3;

            if self.cfg.modeling.tropospheric_bias {
                let tropo = self.env_bias.troposphere_bias_m(&rtm);
                debug!("{}({}) {} - tropod={:.3}m", cd.epoch, cd.sv, name, tropo);
                cd.tropod = tropo;
            }

            if self.cfg.modeling.ionospheric_bias {
                let iono = self.env_bias.ionosphere_bias_m(&rtm);
                debug!("{}({}) {} - ionod={:.3}m", cd.epoch, cd.sv, name, iono);
                cd.ionod = iono;
            }

            let pos_ecef_m = state.to_position_ecef_m();
            let (x0_m, y0_m, z0_m) = (pos_ecef_m[0], pos_ecef_m[1], pos_ecef_m[2]);

            let r_0 = (x0_m.powi(2) + y0_m.powi(2) + z0_m.powi(2)).sqrt();
            let r_sat = (r_sat_m[0].powi(2) + r_sat_m[1].powi(2) + r_sat_m[2].powi(2)).sqrt();

            let r_sat_0 = r_0 - r_sat;

            if self.cfg.modeling.relativistic_path_range {
                let dr = 2.0 * EARTH_GRAVITATION / SPEED_OF_LIGHT_M_S / SPEED_OF_LIGHT_M_S
                    * ((r_sat + r_0 + r_sat_0) / (r_sat + r_0 - r_sat_0)).ln();

                debug!("{}({}) {} - rel. path range={}m", cd.epoch, cd.sv, name, dr);

                cd.relativistic_path_range = dr;
            }

            if self.cfg.modeling.phase_windup {
                // TODO
            }
        }
    }

    /// Runs a postfit SD algorithm, between seld and rhs.
    /// NB: only shared measurements are preserved
    pub fn post_fit_sd<D: DimName>(&mut self, rhs: &Self, state: &State<D>) -> Result<(), Error>
    where
        DefaultAllocator: Allocator<D> + Allocator<D, D>,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
    {
        // select pivot measurement
        let pivot_meas = self.post_fit_sd_pivot();

        if pivot_meas.is_none() {
            error!("failed to select a pivot measurement (empty set?)");
            return Err(Error::SdNoPivotMeasurement);
        }

        let pivot_meas = pivot_meas.unwrap();

        // TODO verify pivot has 1 or two frequencies ?

        self.retain_mut(|meas| {
            let mut retained = false;

            for freqz in meas.frequencies_iter() {
                for rhs_freqz in pivot_meas.frequencies_iter() {
                    if freqz == rhs_freqz {
                        meas.sd_mut(&pivot_meas);
                        retained = true;
                    }
                }
            }

            retained
        });

        // TODO: verify the content is still compliant with
        // navigation method and preferences
    }

    /// Select a pivot measurement
    pub fn post_fit_sd_pivot(&self) -> Option<&Candidate> {
        self.inner
            .iter()
            .max_by(|meas_a, meas_b| meas_a.elevation_deg > meas_b.elevation_deg)
            .reduce(|k, _| k)
    }
}
