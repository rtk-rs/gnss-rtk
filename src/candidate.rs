//! Position solving candidate

use gnss::prelude::SV;
use hifitime::Unit;
use itertools::Itertools;
use log::debug;

use nyx::cosmic::SPEED_OF_LIGHT;
use nyx::linalg::{DVector, MatrixXx4};

use crate::prelude::{Config, Duration, Epoch, InterpolationResult, Mode, Vector3};
use crate::solutions::{PVTBias, PVTSVData};
use crate::{
    bias,
    bias::{IonosphericBias, TropoModel, TroposphericBias},
    Error,
};

/// Signal observation to attach to each candidate
#[derive(Debug, Default, Clone)]
pub struct Observation {
    /// carrier frequency [Hz]
    pub frequency: f64,
    /// actual observation
    pub value: f64,
    /// optional (but recommended) SNR in [dB]
    pub snr: Option<f64>,
}

/// Position solving candidate
#[derive(Debug, Clone)]
pub struct Candidate {
    /// SV
    pub sv: SV,
    /// Signal sampling Epoch
    pub t: Epoch,
    /// state that needs to be resolved
    pub state: Option<InterpolationResult>,
    // SV group delay
    pub(crate) tgd: Option<Duration>,
    // SV clock state (compared to GNSS timescale)
    pub(crate) clock_state: Vector3<f64>,
    // SV clock correction
    pub(crate) clock_corr: Duration,
    // Code observations
    pub(crate) code: Vec<Observation>,
    // Phase observations
    pub(crate) phase: Vec<Observation>,
    // Doppler observations
    pub(crate) doppler: Vec<Observation>,
}

impl Candidate {
    /// Creates a new candidate, to inject in the solver pool.
    /// SV : satellite vehicle (identity).
    /// t: Epoch at which the signals were sampled.
    /// clock_state: SV clock state.
    /// clock_corr: current clock correction (mandatory).
    /// "code": provide as many observations as you can
    /// "phase": provide as many observations as you can
    /// "doppler": provide as many observations as you can
    pub fn new(
        sv: SV,
        t: Epoch,
        clock_state: Vector3<f64>,
        clock_corr: Duration,
        code: Vec<Observation>,
        phase: Vec<Observation>,
        doppler: Vec<Observation>,
    ) -> Result<Self, Error> {
        if code.is_empty() {
            Err(Error::NeedsAtLeastOnePseudoRange)
        } else {
            Ok(Self {
                sv,
                t,
                clock_state,
                clock_corr,
                code,
                phase,
                doppler,
                tgd: None,
                state: None,
            })
        }
    }
    /*
     * Returns one pseudo range observation [m], disregarding its frequency.
     * Infaillible, because we don't allow to build Self without at least
     * 1 PR observation
     */
    pub(crate) fn prefered_pseudorange(&self) -> Option<&Observation> {
        self.code
            .iter()
            // .map(|pr| pr.value)
            .reduce(|k, _| k)
    }
    /*
     * Returns true if we have pseudo range observ on two carriers
     */
    pub(crate) fn dual_freq_pseudorange(&self) -> bool {
        self.code
            .iter()
            .map(|c| (c.frequency * 10.0) as u16)
            .unique()
            .count()
            > 0
    }
    /*
     * Returns true if we have phase observ on two carriers
     */
    pub(crate) fn dual_freq_phase(&self) -> bool {
        self.phase
            .iter()
            .map(|c| (c.frequency * 10.0) as u16)
            .unique()
            .count()
            > 0
    }
    /*
     * apply min SNR mask
     */
    pub(crate) fn min_snr_mask(&mut self, min_snr: f64) {
        self.code = self
            .code
            .iter()
            .filter_map(|c| {
                let snr = c.snr?;
                if snr < min_snr {
                    None
                } else {
                    Some(c.clone())
                }
            })
            .collect();
        self.doppler = self
            .doppler
            .iter()
            .filter_map(|c| {
                let snr = c.snr?;
                if snr < min_snr {
                    None
                } else {
                    Some(c.clone())
                }
            })
            .collect();
        self.phase = self
            .phase
            .iter()
            .filter_map(|c| {
                let snr = c.snr?;
                if snr < min_snr {
                    None
                } else {
                    Some(c.clone())
                }
            })
            .collect();
    }
    /*
     * Try to form signal combination
    fn combination(&self) -> Option<Combination> {
        if self.pseudo_range.len() == 1 {
            return None;
        }
        let pr_a = &self.pseudo_range[0];
        let pr_b = &self.pseudo_range[1];
        let (pr_a, fr_a) = (pr_a.value, pr_a.frequency);
        let (pr_b, fr_b) = (pr_b.value, pr_b.frequency);
        //NB: pour phase c'est pareil
        let value = fr_a.powi(2) * pr_a - fr_b.powi(2) * pr_b / (fr_a.powi(2) - fr_b.powi(2));
        Some(Combination {
            value,
            f1: fr_a,
            f2: fr_b,
        })
    }
     */
    /*
     * Computes signal transmission Epoch
     * returns (t_tx, dt_ttx)
     * "t_tx": Epoch in given timescale
     * "dt_ttx": elapsed duration in seconds in given timescale
     * "frame": Solid body reference Frame
     */
    pub(crate) fn transmission_time(&self, cfg: &Config) -> Result<(Epoch, f64), Error> {
        let (t, ts) = (self.t, self.t.time_scale);
        let seconds_ts = t.to_duration().to_seconds();

        let dt_tx = seconds_ts
            - self
                .prefered_pseudorange()
                .ok_or(Error::MissingPseudoRange)?
                .value
                / SPEED_OF_LIGHT;

        let mut e_tx = Epoch::from_duration(dt_tx * Unit::Second, ts);

        if cfg.modeling.sv_clock_bias {
            debug!("{:?} ({}) clock_corr: {}", t, self.sv, self.clock_corr);
            e_tx -= self.clock_corr;
        }

        if cfg.modeling.sv_total_group_delay {
            if let Some(tgd) = self.tgd {
                debug!("{:?} ({}) tgd   : {}", t, self.sv, tgd);
                e_tx -= tgd;
            }
        }

        let dt = (t - e_tx).to_seconds();
        if dt < 0.0 {
            Err(Error::PhysicalNonSenseRxPriorTx)
        } else if dt > 0.1 {
            Err(Error::PhysicalNonSenseRxTooLate)
        } else {
            Ok((e_tx, dt))
        }
    }
    /*
     * Resolves Self
     */
    pub(crate) fn resolve(
        &self,
        t: Epoch,
        cfg: &Config,
        mode: Mode,
        apriori: (f64, f64, f64),
        apriori_geo: (f64, f64, f64),
        iono_bias: &IonosphericBias,
        tropo_bias: &TroposphericBias,
        row_index: usize,
        y: &mut DVector<f64>,
        g: &mut MatrixXx4<f64>,
    ) -> Result<PVTSVData, Error> {
        // state
        let state = self.state.ok_or(Error::UnresolvedState)?;
        let (x0, y0, z0) = apriori;
        let clock_corr = self.clock_corr.to_seconds();
        let (azimuth, elevation) = (state.azimuth, state.elevation);
        let sv_p = state.position();
        let (sv_x, sv_y, sv_z) = (sv_p[0], sv_p[1], sv_p[2]);

        let mut sv_data = PVTSVData::default();
        sv_data.azimuth = azimuth;
        sv_data.elevation = elevation;

        let rho = ((sv_x - x0).powi(2) + (sv_y - y0).powi(2) + (sv_z - z0).powi(2)).sqrt();
        g[(row_index, 0)] = (x0 - sv_x) / rho;
        g[(row_index, 1)] = (y0 - sv_y) / rho;
        g[(row_index, 2)] = (z0 - sv_z) / rho;
        g[(row_index, 3)] = 1.0_f64;

        let mut models = 0.0_f64;
        if cfg.modeling.sv_clock_bias {
            models -= clock_corr * SPEED_OF_LIGHT;
        }

        /*
         * Possible delay compensations
         */
        if let Some(delay) = cfg.externalref_delay {
            y[row_index] -= delay * SPEED_OF_LIGHT;
        }

        let pr = self
            .prefered_pseudorange()
            .ok_or(Error::MissingPseudoRange)?;

        let (pr, frequency) = (pr.value, pr.frequency);

        /*
         * TODO: frequency dependent delays
         */

        /*
         * IONO + TROPO biases
         */
        let rtm = bias::RuntimeParam {
            t,
            elevation,
            azimuth,
            apriori_geo,
            frequency,
        };

        /*
         * TROPO
         */
        if cfg.modeling.tropo_delay {
            if tropo_bias.needs_modeling() {
                let bias = TroposphericBias::model(TropoModel::Niel, &rtm);
                debug!("{:?} : modeled tropo delay {:.3E}[m]", t, bias);
                models += bias;
                sv_data.tropo_bias = PVTBias::modeled(bias);
            } else if let Some(bias) = tropo_bias.bias(&rtm) {
                debug!("{:?} : measured tropo delay {:.3E}[m]", t, bias);
                models += bias;
                sv_data.tropo_bias = PVTBias::measured(bias);
            }
        }

        /*
         * IONO
         */
        if cfg.modeling.iono_delay {
            if let Some(bias) = iono_bias.bias(&rtm) {
                debug!(
                    "{:?} : modeled iono delay (f={:.3E}Hz) {:.3E}[m]",
                    t, rtm.frequency, bias
                );
                models += bias;
                sv_data.iono_bias = PVTBias::modeled(bias);
            }
        }

        /*
         * Possible frequency dependent delays
         */
        for delay in &cfg.int_delay {
            if delay.frequency == frequency {
                y[row_index] += delay.delay * SPEED_OF_LIGHT;
            }
        }

        y[row_index] = pr - rho - models;

        Ok(sv_data)
    }
}
