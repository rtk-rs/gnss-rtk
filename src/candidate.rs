//! Position solving candidate

use gnss::prelude::SV;
use hifitime::Unit;
use log::debug;
use nyx_space::cosmic::SPEED_OF_LIGHT;
use nyx_space::linalg::{DVector, MatrixXx4};

use crate::prelude::{Config, Duration, Epoch, InterpolationResult, Mode, TropoComponents};
use crate::solutions::{PVTSVData, PVTSVTimeDelay};
use crate::{iono::KbModel, tropo::tropo_bias, Error, Vector3D};

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
    pub(crate) clock_state: Vector3D,
    // SV clock correction
    pub(crate) clock_corr: Duration,
    // Code observations
    pub(crate) code: Vec<Observation>,
    // Phase observations
    pub(crate) phase: Vec<Observation>,
}

impl Candidate {
    /// Creates a new candidate, to inject in the solver pool.
    /// SV : satellite vehicle (identity).
    /// t: Epoch at which the signals were sampled.
    /// clock_state: SV clock state.
    /// clock_corr: current clock correction (mandatory).
    /// code: provide as many observations as you can
    /// phase: provide as many observations as you can
    pub fn new(
        sv: SV,
        t: Epoch,
        clock_state: Vector3D,
        clock_corr: Duration,
        code: Vec<Observation>,
        phase: Vec<Observation>,
    ) -> Result<Self, Error> {
        if code.len() == 0 {
            Err(Error::NeedsAtLeastOnePseudoRange)
        } else {
            Ok(Self {
                sv,
                t,
                clock_state,
                clock_corr,
                code,
                phase,
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
    pub(crate) fn prefered_pseudorange(&self) -> &Observation {
        self.code
            .iter()
            // .map(|pr| pr.value)
            .reduce(|k, _| k)
            .unwrap()
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
     * Returns IONOD possibly impacting
    pub(crate) fn ionod_model(&self, frequency: f64) -> Option<Duration> {
        self.modeled_ionod
            .iter()
            .filter_map(|ionod| {
                if ionod.frequency == frequency {
                    Some(ionod.time_delay)
                } else {
                    None
                }
            })
            .reduce(|k, _| k)
    }
     */
    /*
     * Compute and return signal transmission Epoch
     */
    pub(crate) fn transmission_time(&self, cfg: &Config) -> Result<Epoch, Error> {
        let (t, ts) = (self.t, self.t.time_scale);
        let seconds_ts = t.to_duration().to_seconds();
        let dt_tx = seconds_ts - self.prefered_pseudorange().value / SPEED_OF_LIGHT;
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

        /* run physical verification */
        let dt = (t - e_tx).to_seconds();
        assert!(dt > 0.0, "resolved t_tx is physically impossible");
        assert!(dt < 1.0, "resolved t_tx is physically impossible");
        Ok(e_tx)
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
        row_index: usize,
        y: &mut DVector<f64>,
        g: &mut MatrixXx4<f64>,
        kb_model: Option<KbModel>,
        stec_meas: Option<f64>,
        tropod_model: Option<f64>,
        tropod_meas: Option<f64>,
    ) -> Result<PVTSVData, Error> {
        // state
        let sv = self.sv;
        let state = self.state.ok_or(Error::UnresolvedState)?;
        let (x0, y0, z0) = apriori;
        let (lat_ddeg, lon_ddeg, _) = apriori_geo;
        let clock_corr = self.clock_corr.to_seconds();
        let (azi, elev) = (state.azimuth, state.elevation);
        let (sv_x, sv_y, sv_z) = (state.sky_pos.x, state.sky_pos.y, state.sky_pos.z);

        let mut sv_data = PVTSVData::default();
        sv_data.azimuth = azi;
        sv_data.elevation = elev;

        let rho = ((sv_x - x0).powi(2) + (sv_y - y0).powi(2) + (sv_z - z0).powi(2)).sqrt();
        g[(row_index, 0)] = (x0 - sv_x) / rho;
        g[(row_index, 1)] = (y0 - sv_y) / rho;
        g[(row_index, 2)] = (z0 - sv_z) / rho;
        g[(row_index, 3)] = 1.0_f64;

        let mut models = -clock_corr * SPEED_OF_LIGHT;

        /*
         * Possible delay compensations
         */
        if let Some(delay) = cfg.externalref_delay {
            y[row_index] -= delay * SPEED_OF_LIGHT;
        }

        /*
         * TROPO
         * this is null if cfg.tropod is disabled
         */
        if cfg.modeling.tropo_delay {
            if let Some(bias) = tropod_meas {
                debug!("{:?} : measured tropo delay {:.3E}[m]", t, bias);
                models += bias;
                sv_data.tropo = PVTSVTimeDelay::measured(bias);
            } else if let Some(bias) = tropod_model {
                debug!("{:?} : modeled tropo delay {:.3E}[m]", t, bias);
                models += bias;
                sv_data.tropo = PVTSVTimeDelay::modeled(bias);
            }
        }

        if mode == Mode::SPP {
            let pr = self.prefered_pseudorange();
            let (pr, frequency) = (pr.value, pr.frequency);
            /*
             * IONO
             */
            if cfg.modeling.iono_delay {
                if let Some(model) = kb_model {
                    let meters_delay =
                        model.meters_delay(t, elev, azi, lat_ddeg, lon_ddeg, frequency);
                    debug!(
                        "{:?} : brdc ionod model (freq={:.3E}Hz) {:.3}[m]",
                        t, frequency, meters_delay
                    );

                    models += meters_delay;
                    sv_data.iono = PVTSVTimeDelay::modeled(meters_delay);
                }
                /*
                 * apply possibly passed STEC estimate
                 * TODO: this might be prefered ?
                 */
                else if let Some(stec) = stec_meas {
                    debug!("{:?} : ionod stec {} TECu", t, stec);
                    // TODO: compensate all pseudo range correctly
                    // let alpha = 40.3 * 10E16 / frequency / frequency;
                    // models += alpha * stec;
                    // sv_data.iono = PVTSVTimeDelay::measured(meters_delay);
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
        } else {
            // let comb = self.combination().ok_or(Error::SignalRecombination)?;

            // /*
            //  * Possibly frequency dependent delays
            //  */
            // for delay in &cfg.int_delay {
            //     //TODO <o
            //     if delay.frequency == comb.f1 {
            //         y[row_index] += delay.delay * SPEED_OF_LIGHT;
            //     }
            // }

            // y[row_index] = comb.value - rho - models;
        }

        Ok(sv_data)
    }
}
