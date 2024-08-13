//! Position solving candidate
use hifitime::Unit;
use itertools::Itertools;
use log::debug;
use std::cmp::Ordering;

use nyx::{
    cosmic::SPEED_OF_LIGHT_M_S,
    linalg::{OMatrix, OVector, U8},
};

use crate::{
    bias::RuntimeParams as BiasRuntimeParams,
    constants::Constants,
    navigation::SVInput,
    prelude::{
        Carrier, Config, Duration, Epoch, Error, IonoComponents, IonosphereBias, Method,
        OrbitalState, TropoComponents, TropoModel, Vector3, SV,
    },
};

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Observation {
    /// [Carrier]
    pub carrier: Carrier,
    /// Pseudo Range observation in [m]
    pub pseudo: Option<f64>,
    /// Phase Range observation in [m]
    pub phase: Option<f64>,
    /// Possible doppler observation
    pub doppler: Option<f64>,
    /// SNR
    pub snr: Option<f64>,
    /// For navigation methods that use phase range like [Method::PPP], phase range ambiguities
    /// need to be fixed at some point, otherwise, you end up with similar performances as
    /// [PseudoRange] based navigation methods.
    /// If you resolved the ambiguities yourself, set this value ahead of time, otherwise we will take care of it.
    pub ambiguity: Option<f64>,
}

impl Observation {
    /// Set [Carrier]
    pub fn set_carrier(&mut self, c: Carrier) {
        self.carrier = c;
    }
    /// Creates [Self] with given phase range [m] observation
    pub fn with_phase_range(&self, ph: f64) -> Self {
        let mut s = self.clone();
        s.phase = Some(ph);
        s
    }
    /// Creates [Self] with given pseudo range [m] observation
    pub fn with_pseudo_range(&self, pr: f64) -> Self {
        let mut s = self.clone();
        s.pseudo = Some(pr);
        s
    }
    /// Creates [Self] with doppler observation
    pub fn with_doppler(&self, dop: f64) -> Self {
        let mut s = self.clone();
        s.doppler = Some(dop);
        s
    }
}

/// Signal combination
#[derive(Debug, Copy, Clone, Default)]
pub(crate) struct Combination {
    /// Lhs signal
    pub lhs: Carrier,
    /// Rhs reference signal
    pub rhs: Carrier,
    /// Value
    pub value: f64,
    /// Phase combination ambiguity
    pub ambiguity: Option<f64>,
}

impl Combination {
    pub fn new(lhs: Carrier, rhs: Carrier, value: f64) -> Self {
        Self {
            lhs,
            rhs,
            value,
            ambiguity: None,
        }
    }
    pub fn with_ambiguity(&self, ambiguity: f64) -> Self {
        let mut s = self.clone();
        s.ambiguity = Some(ambiguity);
        s
    }
}

/// Position solving candidate
#[derive(Clone)]
pub struct Candidate {
    /// [SV]
    pub sv: SV,
    /// Sampling [Epoch]
    pub t: Epoch,
    /// State that needs to be resolved
    pub state: Option<OrbitalState>,
    /// t_tx Epoch
    pub(crate) t_tx: Epoch,
    // SV group delay expressed as a [Duration]
    pub(crate) tgd: Option<Duration>,
    // Windup term in cycles
    pub(crate) wind_up: f64,
    /// [ClockCorrection]
    pub(crate) clock_corr: Option<ClockCorrection>,
    /// Local [Observation]s
    pub(crate) observations: Vec<Observation>,
    /// Remote [Observation]s
    pub(crate) remote_obs: Vec<Observation>,
    /// Resolved bias
    pub(crate) iono_bias: f64,
    /// Resolved bias
    pub(crate) tropo_bias: f64,
    /// [IonoComponents]
    pub(crate) iono_components: IonoComponents,
    /// [TropoComponents]
    pub(crate) tropo_components: TropoComponents,
}

#[derive(Default, Debug, Copy, Clone)]
pub struct ClockCorrection {
    /// Correction to associated timescale, expressed as [Duration]
    pub duration: Duration,
    pub(crate) needs_relativistic_correction: bool,
}

impl ClockCorrection {
    /// Define a new [ClockCorrection] that already integrates relativistic corrections
    pub fn with_relativistic_correction(duration: Duration) -> Self {
        Self {
            duration,
            needs_relativistic_correction: false,
        }
    }
    /// Define a new [ClockCorrection] that does not integrate relativistic corrections
    pub fn without_relativistic_correction(duration: Duration) -> Self {
        Self {
            duration,
            needs_relativistic_correction: true,
        }
    }
}

// public
impl Candidate {
    /// Basic candidate definition. Each candidate
    /// is to be proposed to the navigation solver.
    /// This is the most simplistic definition (bare minimum).
    /// It is certainly not enough for PPP navigation and will require
    /// you provide more information (see other customization methods),
    /// especially if you want to achieve accurate results.
    /// Pure RTK navigation being the easiest scenario, you will only have to attach remote observations to this definition.
    /// ## Inputs
    /// - sv: [SV] Identity
    /// - t: sampling [Epoch]
    /// - observations: provide signals observations.
    ///   You have to provide observations that match your navigation method.
    /// - iono_components: provide [IonoComponents] if you're navigating
    ///  with a single signal and modeling.iono_delay is activated.
    /// - tropo_components: provide [TropoComponents], if you can,
    ///   to improve internal model. Note that this has no impact if modeling.tropo_delay
    ///   is turned off.
    pub fn new(sv: SV, t: Epoch, observations: Vec<Observation>) -> Self {
        Self {
            sv,
            t,
            t_tx: t,
            observations,
            iono_bias: 0.0,
            tropo_bias: 0.0,
            wind_up: 0.0_f64,
            remote_obs: Vec::new(),
            state: None,
            tgd: None,
            clock_corr: None,
            iono_components: IonoComponents::Unknown,
            tropo_components: TropoComponents::Unknown,
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
    /// Define [TropoComponents] that should apply to self and bypass our
    /// internal Meteorological table model. Accurate Tropospheric perturbation compensation
    /// will increase your PPP accuracy by tens of meters. This has no effect
    /// when its compensation is turned off.
    pub fn set_tropo_components(&mut self, tropo: TropoComponents) {
        self.tropo_components = tropo;
    }
    /// Define [IonoComponents] that should apply to self and bypass our
    /// internal model. Accurate Ionospheric perturbation compensation
    /// will increase your PPP accuracy by a few meters. This has no effect
    /// when its compensation is turned off. This has no effect when selected
    /// navigation mode allows direct compensation of this effect, which is always prefered.
    pub fn set_iono_components(&mut self, iono: IonoComponents) {
        self.iono_components = iono;
    }
    /// Provide remote [Observation]s observed by [BaseStation] when operating
    /// in RTK opmode.
    pub fn set_remote_observations(&mut self, remote: Vec<Observation>) {
        self.remote_obs = remote.clone();
    }
    /// Returns true if self is compatible with navigation
    /// and may contribute to navigation process
    pub(crate) fn is_navi_compatible(&self) -> bool {
        self.is_rtk_compatible() || self.is_ppp_compatible()
    }
    /// Returns true if self is compatible with RTK positioning
    pub(crate) fn is_rtk_compatible(&self) -> bool {
        self.remote_obs.len() > 3 && self.observations.len() > 3
    }
    /// Returns true if self is compatible with PPP positioning
    pub(crate) fn is_ppp_compatible(&self) -> bool {
        self.state.is_some()
    }
    /// Fills the Matrix and prepare for resolution.
    /// The matrix contribution is highly dependent on the
    /// configuration setup.
    /// This contribution's accuracy depends on the current
    /// conditions and data quality.
    pub(crate) fn matrix_contribution(
        &self,
        cfg: &Config,
        row: usize,
        y: &mut OVector<f64, U8>,
        g: &mut OMatrix<f64, U8, U8>,
        apriori_ecef_m: (f64, f64, f64),
    ) -> Result<SVInput, Error> {
        // When RTK is feasible, it is always prefered,
        // because it is much easier and has immediate accuracy.
        if self.is_rtk_compatible() {
            debug!("{}({}): rtk resolution attempt", self.t, self.sv);
            self.rtk_matrix_contribution(cfg, row, y, g)
        } else {
            debug!("{}({}): ppp resolution attempt", self.t, self.sv);
            self.ppp_matrix_contribution(cfg, row, y, g, apriori_ecef_m)
        }
    }
    /// Matrix conribution, in case of PPP resolution.
    /// The only requirement being that orbital state needs
    /// to be fully resolved. This contribution's accuracy
    /// (to the general process) depends on the current setup
    /// and provided data (condition and quality).
    fn ppp_matrix_contribution(
        &self,
        cfg: &Config,
        row: usize,
        y: &mut OVector<f64, U8>,
        g: &mut OMatrix<f64, U8, U8>,
        apriori_ecef_m: (f64, f64, f64),
    ) -> Result<SVInput, Error> {
        let mut sv_input = SVInput::default();
        let state = self.state.ok_or(Error::UnresolvedState)?;
        let (azimuth, elevation) = (state.azimuth, state.elevation);
        sv_input.azimuth = azimuth;
        sv_input.elevation = elevation;

        let (x0_ecef_m, y0_ecef_m, z0_ecef_m) = apriori_ecef_m;

        let (sv_x, sv_y, sv_z) = (state.position[0], state.position[1], state.position[2]);

        let mut rho =
            ((sv_x - x0_ecef_m).powi(2) + (sv_y - y0_ecef_m).powi(2) + (sv_z - z0_ecef_m).powi(2))
                .sqrt();

        if cfg.modeling.relativistic_path_range {
            let mu = Constants::EARTH_GRAVITATION;
            let r_sat =
                (state.position[0].powi(2) + state.position[1].powi(2) + state.position[2].powi(2))
                    .sqrt();
            let r_0 = (x0_ecef_m.powi(2) + y0_ecef_m.powi(2) + z0_ecef_m.powi(2)).sqrt();
            let r_sat_0 = r_0 - r_sat;
            let dr = 2.0 * mu / SPEED_OF_LIGHT_M_S / SPEED_OF_LIGHT_M_S
                * ((r_sat + r_0 + r_sat_0) / (r_sat + r_0 - r_sat_0)).ln();
            debug!(
                "{}({}) relativistic path range {:.3E}m",
                self.t, self.sv, dr
            );
            rho += dr;
        }

        let (x_i, y_i, z_i) = (
            (x0_ecef_m - sv_x) / rho,
            (y0_ecef_m - sv_y) / rho,
            (z0_ecef_m - sv_z) / rho,
        );

        g[(row, 0)] = x_i;
        g[(row, 1)] = y_i;
        g[(row, 2)] = z_i;
        g[(row, 3)] = 1.0_f64;

        let mut models = 0.0_f64;

        if cfg.modeling.sv_clock_bias {
            let corr = self.clock_corr.ok_or(Error::UnknownClockCorrection)?;
            sv_input.clock_correction = Some(corr.duration);
            models -= corr.duration.to_seconds() * SPEED_OF_LIGHT_M_S;
        }

        let (pr, frequency) = match cfg.method {
            Method::SPP => {
                let pr = self
                    .prefered_pseudorange()
                    .ok_or(Error::MissingPseudoRange)?;
                (pr.pseudo.unwrap(), pr.carrier.frequency())
            },
            Method::CPP | Method::PPP => {
                let pr = self
                    .code_if_combination()
                    .ok_or(Error::PseudoRangeCombination)?;
                (pr.value, pr.rhs.frequency())
            },
        };

        // cable delays
        if cfg.modeling.cable_delay {
            if let Some(delay) = cfg.externalref_delay {
                models -= delay * SPEED_OF_LIGHT_M_S;
            }
            // TODO: frequency dependent delays
            for delay in &cfg.int_delay {
                if delay.frequency == frequency {
                    models += delay.delay * SPEED_OF_LIGHT_M_S;
                }
            }
        }

        // tropo
        if cfg.modeling.tropo_delay {
            let bias = self.tropo_bias;
            models += bias;
            sv_input.tropo_bias = Some(bias);
        }

        // iono
        if cfg.modeling.iono_delay {
            let bias = self.iono_bias;
            models += bias;
            if cfg.method == Method::SPP {
                sv_input.iono_bias = Some(IonosphereBias::modeled(bias));
            } else {
                sv_input.iono_bias = Some(IonosphereBias::measured(bias));
            }
        }

        y[row] = pr - rho - models;
        Ok(sv_input)
    }
    /// Matrix contribution, in case of RTK resolution.
    fn rtk_matrix_contribution(
        &self,
        cfg: &Config,
        row: usize,
        y: &mut OVector<f64, U8>,
        g: &mut OMatrix<f64, U8, U8>,
    ) -> Result<SVInput, Error> {
        Err(Error::MissingRemoteRTKObservation(self.t, self.sv))
    }
}

// private
impl Candidate {
    /// Applies all perturbation models to [Self]
    pub(crate) fn apply_models(
        &mut self,
        method: Method,
        tropo_modeling: bool,
        iono_modeling: bool,
        apriori_geo_ddeg: (f64, f64, f64),
    ) {
        if let Some(state) = self.state {
            if let Some(obs) = self.prefered_pseudorange() {
                let rtm = BiasRuntimeParams {
                    t: self.t,
                    elevation_deg: state.elevation,
                    elevation_rad: state.elevation.to_radians(),
                    azimuth_deg: state.azimuth,
                    azimuth_rad: state.azimuth.to_radians(),
                    frequency: obs.carrier.frequency(),
                    apriori_geo: apriori_geo_ddeg,
                    apriori_rad: (
                        apriori_geo_ddeg.0.to_radians(),
                        apriori_geo_ddeg.1.to_radians(),
                    ),
                };
                if tropo_modeling {
                    self.tropo_bias = self.tropo_components.value(TropoModel::Niel, &rtm);
                }
                if iono_modeling {
                    if method == Method::SPP {
                        self.iono_bias = self.iono_components.value(&rtm);
                    }
                }
            }
        }
    }
    // Pseudo range iterator
    fn pseudo_range_iter(&self) -> Box<dyn Iterator<Item = (Carrier, f64)> + '_> {
        Box::new(self.observations.iter().filter_map(|ob| {
            let pseudo = ob.pseudo?;
            Some((ob.carrier, pseudo))
        }))
    }
    // Phase Range iterator
    fn phase_range_iter(&self) -> Box<dyn Iterator<Item = (Carrier, f64)> + '_> {
        Box::new(self.observations.iter().filter_map(|ob| {
            let phase = ob.phase?;
            Some((ob.carrier, phase))
        }))
    }
    /*
     * Returns best observed SNR, whatever the signal
     */
    pub(crate) fn pseudorange_best_snr(&self) -> Option<f64> {
        self.observations
            .iter()
            .max_by(|ob_a, ob_b| {
                if let Some(snr_a) = ob_a.snr {
                    if let Some(snr_b) = ob_b.snr {
                        snr_a.partial_cmp(&snr_b).unwrap()
                    } else {
                        Ordering::Greater
                    }
                } else {
                    Ordering::Less
                }
            })
            .map(|c| c.snr)?
    }
    /// Returns one pseudo range observation [m], whatever the frequency.
    pub(crate) fn prefered_pseudorange(&self) -> Option<Observation> {
        if let Some(c1) = self
            .observations
            .iter()
            .filter(|ob| {
                matches!(
                    ob.carrier,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                ) && ob.pseudo.is_some()
            })
            .reduce(|k, _| k)
        {
            Some(c1.clone())
        } else {
            self.observations
                .iter()
                .filter(|ob| {
                    ob.pseudo.is_some()
                        && !matches!(
                            ob.carrier,
                            Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                        )
                })
                .reduce(|k, _| k)
                .cloned()
        }
    }
    // True if Self is Method::CPP compatible
    pub(crate) fn cpp_compatible(&self) -> bool {
        self.dual_pseudorange()
    }
    // True if Self is Method::PPP compatible
    pub(crate) fn ppp_compatible(&self) -> bool {
        self.dual_pseudorange() && self.dual_phase()
    }
    // True if dual PR is present
    pub(crate) fn dual_pseudorange(&self) -> bool {
        self.pseudo_range_iter()
            .map(|(signal, _)| signal)
            .unique()
            .count()
            > 1
    }
    // True if dual phase is present
    pub(crate) fn dual_phase(&self) -> bool {
        self.phase_range_iter()
            .map(|(signal, _)| signal)
            .unique()
            .count()
            > 1
    }
    // Returns the L1 Pseudo Range observation [m] if it exists
    pub(crate) fn l1_pseudorange(&self) -> Option<(Carrier, f64)> {
        self.pseudo_range_iter()
            .filter(|(signal, _)| {
                matches!(
                    signal,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)
    }
    // Returns the L1 Phase Range observation [m] if it exists
    pub(crate) fn l1_phaserange(&self) -> Option<(Carrier, f64)> {
        self.phase_range_iter()
            .filter(|(signal, _)| {
                matches!(
                    signal,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)
    }
    // Returns the Lj Pseudo Range observation [m] if it exists
    pub(crate) fn lj_pseudorange(&self) -> Option<(Carrier, f64)> {
        self.pseudo_range_iter()
            .filter(|(signal, _)| {
                !matches!(
                    signal,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)
    }
    // Returns the Lj Phase Range observation [m] if it exists
    pub(crate) fn lj_phaserange(&self) -> Option<(Carrier, f64)> {
        self.phase_range_iter()
            .filter(|(signal, _)| {
                !matches!(
                    signal,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)
    }
    /// Returns IF code range combination
    pub(crate) fn code_if_combination(&self) -> Option<Combination> {
        let (c_l1, l1_pr) = self.l1_pseudorange()?;
        let freq_l1 = c_l1.frequency();

        let (c_lx, lx_pr) = self
            .pseudo_range_iter()
            .filter(|(c, _)| *c != c_l1)
            .reduce(|k, _| k)?;

        let freq_lx = c_lx.frequency();

        let alpha = 1.0 / (freq_l1.powi(2) - freq_lx.powi(2));
        let beta = freq_l1.powi(2);
        let gamma = freq_lx.powi(2);
        Some(Combination::new(
            c_lx,
            c_l1,
            alpha * (beta * l1_pr - gamma * lx_pr),
        ))
    }
    /// Returns IF phase range combination
    pub(crate) fn phase_if_combination(&self) -> Option<Combination> {
        let (c_1, l1_ph) = self.l1_phaserange()?;
        let f_l1 = c_1.frequency();

        let (c_lx, lx_ph) = self
            .phase_range_iter()
            .filter(|(c, _)| *c != c_1)
            .reduce(|k, _| k)?;

        let f_lx = c_lx.frequency();

        let alpha = 1.0 / (f_l1.powi(2) - f_lx.powi(2));
        let beta = f_l1.powi(2);
        let gamma = f_lx.powi(2);
        Some(Combination::new(
            c_lx,
            c_1,
            alpha * (beta * l1_ph - gamma * lx_ph),
        ))
    }
    /// Returns phase wide lane combination
    pub(crate) fn phase_wl_combination(&self) -> Option<Combination> {
        let (c_1, l_1) = self.l1_phaserange()?;
        let (c_j, l_j) = self
            .phase_range_iter()
            .filter(|(c, _)| *c != c_1)
            .reduce(|k, _| k)?;

        let (f_1, f_j) = (c_1.frequency(), c_j.frequency());
        Some(Combination::new(
            c_j,
            c_1,
            (f_1 * l_1 - f_j * l_j) / (f_1 - f_j),
        ))
    }
    /// Returns code narrow lane combination
    pub(crate) fn code_nl_combination(&self) -> Option<Combination> {
        let (c_1, l_1) = self.l1_pseudorange()?;
        let (c_j, l_j) = self
            .pseudo_range_iter()
            .filter(|(c, _)| *c != c_1)
            .reduce(|k, _| k)?;

        let (f_1, f_j) = (c_1.frequency(), c_j.frequency());

        Some(Combination::new(
            c_j,
            c_1,
            (f_1 * l_1 + f_j * l_j) / (f_1 + f_j),
        ))
    }
    pub(crate) fn mw_combination(&self) -> Option<Combination> {
        let ph_w = self.phase_wl_combination()?;
        let pr_n = self.code_nl_combination()?;
        Some(Combination::new(
            ph_w.lhs,
            ph_w.rhs,
            ph_w.value - pr_n.value,
        ))
    }
    // Form GF combination
    pub(crate) fn phase_gf_combination(&self) -> Option<Combination> {
        let (c_1, l_1) = self
            .phase_range_iter()
            .filter(|(c, _)| matches!(c, Carrier::L1 | Carrier::E1 | Carrier::B1aB1c))
            .reduce(|k, _| k)?;

        let (c_j, l_j) = self
            .phase_range_iter()
            .filter(|(c, _)| *c != c_1)
            .reduce(|k, _| k)?;

        Some(Combination::new(c_j, c_1, l_1 - l_j))
    }
    // Form GF combination
    pub(crate) fn code_gf_combination(&self) -> Option<Combination> {
        let (c_1, pr_1) = self
            .pseudo_range_iter()
            .filter(|(c, _)| matches!(c, Carrier::L1 | Carrier::E1 | Carrier::B1aB1c))
            .reduce(|k, _| k)?;

        let (c_j, pr_j) = self
            .phase_range_iter()
            .filter(|(c, _)| *c != c_1)
            .reduce(|k, _| k)?;

        Some(Combination::new(c_j, c_1, pr_j - pr_1))
    }
    // Computes phase windup term. Self should be fully resolved, otherwse
    // will panic.
    pub(crate) fn windup_correction(&mut self, ref_enu: Vector3<f64>, sun: Vector3<f64>) -> f64 {
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
    // Retains only observations with SNR >= min_snr
    pub(crate) fn min_snr_mask(&mut self, min_snr: f64) {
        self.observations.retain(|ob| {
            if let Some(snr) = ob.snr {
                snr >= min_snr
            } else {
                // no SNR information: we decide to still retain
                // because old or exotic software might not provide SNR information
                // and this would prohibit using the solver
                true
            }
        })
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

        // TODO: prefere (when possible) resolved phase range here ?
        let dt_tx = seconds_ts
            - self
                .prefered_pseudorange()
                .ok_or(Error::MissingPseudoRange)?
                .pseudo
                .unwrap()
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
                debug!("{} ({}) tgd   : {}", t, self.sv, tgd);
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
    #[cfg(test)]
    pub fn set_state(&mut self, state: OrbitalState) {
        self.state = Some(state);
    }
}

#[cfg(test)]
mod test {
    use crate::prelude::{
        Candidate, Carrier, ClockCorrection, Epoch, IonoComponents, Observation, TropoComponents,
        SV,
    };
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
            let cd = Candidate::new(
                SV::default(),
                Epoch::default(),
                ClockCorrection::default(),
                None,
                observations,
                IonoComponents::default(),
                TropoComponents::default(),
            );
            assert_eq!(cd.cpp_compatible(), cpp_compatible);
        }
    }
}
