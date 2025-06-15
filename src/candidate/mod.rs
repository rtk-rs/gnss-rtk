//! Position solving candidate
use hifitime::Unit;
use log::debug;

use crate::{
    ambiguity::{Input as AmbiguityInput, Output as Ambiguities},
    constants::SPEED_OF_LIGHT_M_S,
    navigation::state::State,
    prelude::{Almanac, Config, Constellation, Duration, Epoch, Error, Orbit, Vector3, SV},
};

use anise::errors::AlmanacResult;

use nalgebra::{allocator::Allocator, DefaultAllocator, DimName};

mod bias;
mod ppp;
mod rtk;
mod signal;

pub mod clock;
pub(crate) mod combination;

pub use crate::{
    candidate::{clock::ClockCorrection, signal::Observation},
    prelude::Carrier,
};

/// Position solving candidate
#[derive(Clone, Debug)]
pub struct Candidate {
    /// [SV]
    pub sv: SV,

    /// Sampling [Epoch]
    pub t: Epoch,

    /// TX [Epoch]
    pub(crate) t_tx: Epoch,

    /// dt TX [Duration]
    pub(crate) dt_tx: Duration,

    /// [Orbit]al state
    pub(crate) orbit: Option<Orbit>,

    /// SV group delay expressed as a [Duration]
    pub(crate) tgd: Option<Duration>,

    /// Windup term in signal cycles
    pub(crate) windup: f64,

    /// [ClockCorrection]
    pub(crate) clock_corr: Option<ClockCorrection>,

    /// Local [Observation]s
    pub(crate) observations: Vec<Observation>,

    /// elevation at reception time
    pub(crate) elevation_deg: Option<f64>,

    /// azimuth at reception time
    pub(crate) azimuth_deg: Option<f64>,

    /// Possible time system correction
    pub(crate) system_correction: Option<Duration>,
}

impl Candidate {
    /// Basic candidate definition, to propose to the navigation solver. Each candidate is to be
    /// This is the most simplistic definition (bare minimum).
    /// It is certainly not enough for PPP navigation and will require
    /// you provide more information (see other customization methods),
    /// especially if you want to achieve accurate results.
    /// ## Input
    /// - sv: [SV] Identity
    /// - t: sampling [Epoch]
    /// - observations: provide signals observations.
    ///   You have to provide observations that match your navigation method.
    pub fn new(sv: SV, t: Epoch, observations: Vec<Observation>) -> Self {
        Self {
            sv,
            t,
            t_tx: t,
            observations,
            system_correction: None,
            tgd: Default::default(),
            dt_tx: Default::default(),
            orbit: Default::default(),
            windup: Default::default(),
            azimuth_deg: Default::default(),
            elevation_deg: Default::default(),
            clock_corr: Default::default(),
        }
    }

    pub(crate) fn weight_perturbations(&self) -> f64 {
        let fact = match self.sv.constellation {
            Constellation::GPS => 1.0,
            Constellation::Galileo => 1.0,
            _ => 10.0,
        };

        let r_ratio = 100.0;

        let tropod = 3.0;
        let code_bias = 0.3;

        fact * r_ratio + tropod + code_bias
    }

    /// Define Total Group Delay [TDG] if you know it.
    /// This will increase your accuracy in PPP opmode for up to 10m.
    /// If you know the [TGD] value, you should specifiy especially on first iteration,
    /// because it also impacts the [Solver] initialization process and any bias here also impacts
    /// negatively.
    pub fn set_group_delay(&mut self, tgd: Duration) {
        self.tgd = Some(tgd);
    }

    /// Define on board [ClockCorrection] if you know it.
    /// This is mandatory for PPP and will increase your accuracy by hundreds of km.
    pub fn set_clock_correction(&mut self, corr: ClockCorrection) {
        self.clock_corr = Some(corr);
    }

    /// Copy and return updated [Candidate] with desired [ClockCorrection]
    pub fn with_clock_correction(&self, corr: ClockCorrection) -> Self {
        let mut s = self.clone();
        s.clock_corr = Some(corr);
        s
    }

    /// Update pseudo range observation (in meters) for this frequency.
    pub fn set_pseudo_range_m(&mut self, carrier: Carrier, pr_m: f64) {
        if let Some(observation) = self
            .observations
            .iter_mut()
            .filter_map(|obs| {
                if obs.carrier == carrier && obs.pseudo_range_m.is_some() {
                    Some(obs)
                } else {
                    None
                }
            })
            .reduce(|k, _| k)
        {
            observation.pseudo_range_m = Some(pr_m);
        } else {
            self.observations
                .push(Observation::pseudo_range(carrier, pr_m, None));
        }
    }

    /// Update with ambiguous range observation (in meters) for this frequency.
    pub fn set_ambiguous_phase_range_m(&mut self, carrier: Carrier, pr_m: f64) {
        if let Some(observation) = self
            .observations
            .iter_mut()
            .filter_map(|obs| {
                if obs.carrier == carrier && obs.phase_range_m.is_some() {
                    Some(obs)
                } else {
                    None
                }
            })
            .reduce(|k, _| k)
        {
            observation.phase_range_m = Some(pr_m);
        } else {
            self.observations
                .push(Observation::ambiguous_phase_range(carrier, pr_m, None));
        }
    }

    pub(crate) fn ambiguity_input(&self) -> Option<AmbiguityInput> {
        let l1 = self.l1_phase_range()?;
        let (f1_hz, l1) = (l1.0.frequency_hz(), l1.1);
        let c1 = self.l1_pseudo_range()?.1;

        let l2 = self.subsidary_phase_range()?;
        let (f2_hz, l2) = (l2.0.frequency_hz(), l2.1);
        let c2 = self.subsidary_pseudo_range()?.1;

        Some(AmbiguityInput {
            f1_hz,
            c1,
            l1,
            f2_hz,
            c2,
            l2,
            // f5_hz: None,
            // c5: None,
            // l5: None,
        })
    }

    pub(crate) fn update_ambiguities(&mut self, output: Ambiguities) {
        for obs in self.observations.iter_mut() {
            if obs.carrier.is_l1() {
                obs.ambiguity = Some(output.n1 as f64);
            } else {
                // TODO : improve
                // this will not work for triple frequency scenarios
                obs.ambiguity = Some(output.n2 as f64);
            }
        }
    }

    /// Computes phase windup correction term.
    pub(crate) fn phase_windup_correction(
        &mut self,
        rx_state: &State,
        r_sun: Vector3<f64>,
        past_correction: Option<f64>,
    ) {
        let sv_state = self.orbit.expect("phase windup - unresolved state");

        let r_sv = sv_state.to_cartesian_pos_vel() * 1.0E3;

        // todo self.yaw_attitude();

        let r_rx = rx_state.position_ecef_m();
        let r_sv = Vector3::new(r_sv[0], r_sv[1], r_sv[2]);
        let r_rr_rs = r_rx - r_sv;
        let e_r_rs = r_rr_rs.norm();

        self.windup = 0.0;
    }

    /// Computes signal transmission instant, as [Epoch]
    pub(crate) fn tx_epoch(&mut self, cfg: &Config) -> Result<(), Error> {
        let mut t_tx = self.t;

        let (_, pr) = self.best_snr_range_m().ok_or(Error::MissingPseudoRange)?;

        t_tx -= pr / SPEED_OF_LIGHT_M_S * Unit::Second;

        if cfg.modeling.sv_total_group_delay {
            if let Some(tgd) = self.tgd {
                debug!("{} ({}) - group delay {}", self.t, self.sv, tgd);
                t_tx -= tgd;
            }
        }

        if cfg.modeling.sv_clock_bias {
            let clock_corr = self.clock_corr.ok_or(Error::UnknownClockCorrection)?;

            debug!(
                "{} ({}) clock correction: {}",
                self.t, self.sv, clock_corr.duration,
            );

            t_tx -= clock_corr.duration;
        }

        assert!(
            t_tx < self.t,
            "Physical non sense - rx={} prior tx={}",
            self.t,
            t_tx
        );

        self.t_tx = t_tx;
        self.dt_tx = self.t - self.t_tx;

        Ok(())
    }

    /// Fix [Orbit]al attitude
    pub(crate) fn orbital_attitude_fixup(
        &mut self,
        almanac: &Almanac,
        rx_orbit: Orbit,
    ) -> AlmanacResult<()> {
        let orbit = self
            .orbit
            .expect("internal error: undefined orbital state (badop)");

        let elazrg = almanac.azimuth_elevation_range_sez(orbit, rx_orbit, None, None)?;

        self.azimuth_deg = Some(elazrg.azimuth_deg);
        self.elevation_deg = Some(elazrg.elevation_deg);
        Ok(())
    }

    /// Returns (elevation, azimuth) in decimal degrees.
    pub(crate) fn attitude(&self) -> Option<(f64, f64)> {
        let el = self.elevation_deg?;
        let az = self.azimuth_deg?;
        Some((el, az))
    }

    // /// Creates [Candidate] with elevation degree
    // pub(crate) fn with_elevation_deg(&self, el: f64) -> Self {
    //     let mut s = self.clone();
    //     s.elevation_deg = Some(el);
    //     s
    // }

    // /// Creates [Candidate] with azimuth degree
    // pub(crate) fn with_azimuth_deg(&self, az: f64) -> Self {
    //     let mut s = self.clone();
    //     s.azimuth_deg = Some(az);
    //     s
    // }

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
                    snr_dbhz: Some(1.0),
                    pseudo_range_m: Some(1.0),
                    phase_range_m: Some(2.0),
                    ambiguity: None,
                    doppler: None,
                    carrier: Carrier::L1,
                },
                Observation {
                    snr_dbhz: Some(1.0),
                    pseudo_range_m: Some(2.0),
                    phase_range_m: Some(2.0),
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
