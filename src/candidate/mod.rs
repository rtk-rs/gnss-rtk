//! Position solving candidate
use hifitime::Unit;
use log::debug;

use nyx::cosmic::SPEED_OF_LIGHT_M_S;

use crate::prelude::{Config, Duration, Epoch, Error, Method, Orbit, Vector3, SV};

mod nav;
mod signal;

pub mod clock;
// pub(crate) mod combination;

pub use crate::candidate::{clock::ClockCorrection, signal::Observation};

/// Position solving candidate
#[derive(Clone)]
pub struct Candidate {
    /// [SV]
    pub sv: SV,
    /// Sampling [Epoch]
    pub t: Epoch,
    /// [Orbit], which needs to be resolved for PPP
    pub(crate) orbit: Option<Orbit>,
    /// SV group delay expressed as a [Duration]
    pub(crate) tgd: Option<Duration>,
    /// Windup term in signal cycles
    pub(crate) wind_up: f64,
    /// [ClockCorrection]
    pub(crate) clock_corr: Option<ClockCorrection>,
    /// Local [Observation]s
    pub(crate) observations: Vec<Observation>,
    /// Remote [Observation]s
    pub(crate) remote_obs: Vec<Observation>,
    /// elevation at reception time
    pub(crate) elevation_deg: Option<f64>,
    /// azimuth at reception time
    pub(crate) azimuth_deg: Option<f64>,
    /// Ionosphere bias in meters.
    pub(crate) iono_bias_m: f64,
    /// Tropospheric bias in meters.
    pub(crate) tropo_bias_m: f64,
}

impl Candidate {
    /// Basic candidate definition. Each candidate
    /// is to be proposed to the navigation solver.
    /// This is the most simplistic definition (bare minimum).
    /// It is certainly not enough for PPP navigation and will require
    /// you provide more information (see other customization methods),
    /// especially if you want to achieve accurate results.
    /// Pure RTK navigation being the easiest scenario, you will only have to attach remote observations to this definition.
    /// ## Input
    /// - sv: [SV] Identity
    /// - t: sampling [Epoch]
    /// - observations: provide signals observations.
    ///   You have to provide observations that match your navigation method.
    pub fn new(sv: SV, t: Epoch, observations: Vec<Observation>) -> Self {
        Self {
            sv,
            t,
            observations,
            tgd: Default::default(),
            orbit: Default::default(),
            wind_up: Default::default(),
            remote_obs: Default::default(),
            azimuth_deg: Default::default(),
            elevation_deg: Default::default(),
            clock_corr: Default::default(),
            tropo_bias_m: Default::default(),
            iono_bias_m: Default::default(),
        }
    }

    /// Define Total Group Delay [TDG] if you know it.
    /// This will increase your accuracy in PPP opmode for up to 10m.
    /// If you know the [TGD] value, you should specifiy especially on first iteration,
    /// because it also impacts the [Solver] initialization process and any bias here also impacts
    /// negatively.
    pub fn set_group_delay(&mut self, tgd: Duration) {
        self.tgd = Some(tgd);
    }

    /// Define on board Clock Correction if you know it.
    /// This is mandatory for PPP and will increase your accuracy by hundreds of km.
    pub fn set_clock_correction(&mut self, corr: ClockCorrection) {
        self.clock_corr = Some(corr);
    }

    /// Provide remote [Observation]s observed by remote reference site. Not required if you intend to navigate in PPP mode.
    pub fn set_remote_observations(&mut self, remote: Vec<Observation>) {
        self.remote_obs = remote.clone();
    }

    /// Provide one remote [Observation] realized on remote reference site. Not required if you intend to navigate in PPP mode.
    pub fn add_remote_observation(&mut self, remote: Observation) {
        self.remote_obs.push(remote);
    }

    pub(crate) fn is_navi_compatible(&self) -> bool {
        self.is_rtk_compatible() || self.is_ppp_compatible()
    }

    /// Returns true if self is compatible with RTK positioning
    pub(crate) fn is_rtk_compatible(&self) -> bool {
        self.remote_obs.len() > 3 && self.observations.len() > 3
    }

    /// Returns true if self is compatible with PPP positioning
    pub(crate) fn is_ppp_compatible(&self) -> bool {
        self.orbit.is_some()
    }
}

// private
impl Candidate {
    /// Applies all perturbation models to [Self].
    /// This will panic if Orbit has not been resolved!
    pub(crate) fn apply_models(
        &mut self,
        method: Method,
        tropo_modeling: bool,
        iono_modeling: bool,
        azimuth_deg: f64,
        elevation_deg: f64,
        rx_geo: (f64, f64, f64),
        rx_rad: (f64, f64),
    ) -> Result<(), Error> {
        let pr = self
            .prefered_pseudorange()
            .ok_or(Error::MissingPseudoRange)?;

        let rtm = BiasRuntimeParams {
            t: self.t,
            rx_geo,
            rx_rad,
            elevation_deg,
            frequency: pr.carrier.frequency(),
            azimuth_rad: azimuth_deg.to_radians(),
            elevation_rad: elevation_deg.to_radians(),
        };
        if tropo_modeling {
            self.tropo_bias = self.tropo_components.value(TropoModel::Niel, &rtm);
        }
        if iono_modeling {
            if method == Method::SPP {
                self.iono_bias = self.iono_components.value(&rtm);
            }
        }
        Ok(())
    }

    /// Computes phase windup term. Self should be fully resolved, otherwse
    /// will panic.
    pub(crate) fn windup_correction(&mut self, _: Vector3<f64>, _: Vector3<f64>) -> f64 {
        0.0
        // let state = self.state.unwrap();
        // let r_sv = state.to_ecef();

        // let norm = (
        //     (sun[0] - r_sv[0]).powi(2)
        //     + (sun[1] - r_sv[1]).powi(2)
        //     + (sun[2] - r_sv[2]).powi(2)
        // ).sqrt();

        // let e = (r_sun - r_sv_mc ) / norm;
        // let j = k.cross(e);
        // let i = j.cross(k);

        // let d_prime_norm = d_prime.norm();
        // let d_norm = d.norm();
        // let psi = pho * (d_prime.cross(d));
        // let dphi = d_prime.dot(d) / d_prime.norm() / d.norm();

        // let n = (self.delta_phi.unwrap_or(0.0) / 2.0 / PI).round();
        // self.delta_phi = dphi + 2.0 * n;

        // self.delta_phi
        // self.wind_up =
    }

    /// Computes signal transmission time, expressed as [Epoch]
    /// and used in precise orbital state resolution (ppp workflow).
    /// - returns (t_tx, dt_ttx)
    /// - "t_tx": TX [Epoch] in required timescale
    /// - "dt_ttx" elapsed [Duration] in said timescale
    //TODO: remove dt_ttx and simply use t_tx.duration (newly available)
    pub(crate) fn transmission_time(&self, cfg: &Config) -> Result<(Epoch, Duration), Error> {
        let (t, ts) = (self.t, self.t.time_scale);
        let seconds_ts = t.to_duration_in_time_scale(t.time_scale).to_seconds();

        let dt_tx = seconds_ts
            - self
                .prefered_pseudorange()
                .ok_or(Error::MissingPseudoRange)?
                / SPEED_OF_LIGHT_M_S;

        let mut e_tx = Epoch::from_duration(dt_tx * Unit::Second, ts);

        if cfg.modeling.sv_clock_bias {
            let clock_corr = self.clock_corr.ok_or(Error::UnknownClockCorrection)?;
            debug!(
                "{} ({}) clock correction: {}",
                t, self.sv, clock_corr.duration
            );
            e_tx -= clock_corr.duration;
        }

        if cfg.modeling.sv_total_group_delay {
            if let Some(tgd) = self.tgd {
                debug!("{} ({}) {} tgd", t, self.sv, tgd);
                e_tx -= tgd;
            }
        }

        let dt_secs = (t - e_tx).to_seconds();
        let dt = Duration::from_seconds(dt_secs);
        assert!(
            dt_secs.is_sign_positive(),
            "Physical non sense - RX {:?} prior TX {:?}",
            t,
            e_tx
        );
        assert!(
            dt_secs <= 0.2,
            "{}({}): {} Space/Earth propagation delay is unrealistic: invalid input",
            t,
            self.sv,
            dt
        );
        Ok((e_tx, dt))
    }

    /// Creates [Candidate] with desired [Orbit]
    pub(crate) fn with_orbit(&self, orbit: Orbit) -> Self {
        let mut s = self.clone();
        s.orbit = Some(orbit);
        s
    }

    /// Returns (elevation, azimuth) in decimal degrees.
    pub(crate) fn attitude(&self) -> Option<(f64, f64)> {
        let el = self.elevation_deg?;
        let az = self.azimuth_deg?;
        Some((el, az))
    }

    /// Creates [Candidate] with elevation degree
    pub(crate) fn with_elevation_deg(&self, el: f64) -> Self {
        let mut s = self.clone();
        s.elevation_deg = Some(el);
        s
    }

    /// Creates [Candidate] with azimuth degree
    pub(crate) fn with_azimuth_deg(&self, az: f64) -> Self {
        let mut s = self.clone();
        s.azimuth_deg = Some(az);
        s
    }

    #[cfg(test)]
    pub fn set_orbit(&mut self, orbit: Orbit) {
        self.orbit = Some(orbit);
    }
}

#[cfg(test)]
mod test {
    use crate::prelude::{Candidate, Carrier, Epoch, Observation, SV};
    #[test]
    fn cpp_compatibility() {
        for (observations, cpp_compatible) in [(
            vec![
                Observation {
                    snr: Some(1.0),
                    pseudo: Some(1.0),
                    phase: Some(2.0),
                    ambiguity: None,
                    doppler: None,
                    carrier: Carrier::L1,
                },
                Observation {
                    snr: Some(1.0),
                    pseudo: Some(2.0),
                    phase: Some(2.0),
                    ambiguity: None,
                    doppler: None,
                    carrier: Carrier::L5,
                },
            ],
            true,
        )] {
            let cd = Candidate::new(SV::default(), Epoch::default(), observations);
            assert_eq!(cd.cpp_compatible(), cpp_compatible);
        }
    }
}
