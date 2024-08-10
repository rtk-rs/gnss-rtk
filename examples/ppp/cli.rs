// The Cli is only there to help us modify the initial conditions, in CI/CD.
pub struct Cli {
    matches: ArgMatches,
}

use crate::tester::Tester;

use gnss_rtk::prelude::{Config as RTKConfig, Method as RTKMethod};

impl Cli {
    fn new() -> Self {
        let cmd =

        Self {
            matches: cmd.getmatches(),
        }
    }
    pub fn config_preset(&self) -> RTKConfig {
        let mut cfg = self.matches.get_flag("static") {
            // Static preset, dedicated to stable / ultra stable targets
            RTKConfig::static_preset(self.navi_method())
        } else {
            // Dynamic presets: dedicated to moving rovers
            panic!("kinematics not yet supported!");
        };
        // Possible customizations
        if let Some(fixed_alt) = self.fixed_altitude() {
            cfg.with_fixed_altitude(fixed_alt);
        }
        if let Some(min_sv_elev) = self.min_sv_elev() {
            cfg.min_sv_evel = min_sv_elev;
        }
        if let Some(min_sv_azim) = self.min_sv_azim() {
            cfg.min_sv_azim = min_sv_azim;
        }
        if let Some(max_sv_azim) = self.max_sv_azim() {
            cfg.max_sv_azim = max_sv_azim;
        }
        if let Some(timescale) = self.timescale() {
            cfg.timescale = timescale;
        }
    }
    pub fn navi_method(&self) -> RTKMethod {
        if self.matches.get_flag("spp") {
            // Standard or Simple Precise Positioning
            // Signal signal Pseudo range
            RTKMethod::SPP
        } else if self.matches.get_flag("cpp") {
            // Dual signal Pseudo range
            RTKMethod::CPP
        } else if self.matches.get_flag("ppp") {
            // Dual signal Pseudo + Phase range
            RTKMethod::PPP
        }
    }
    pub fn fixed_altitude(&self) -> Option<&f64> {
        self.matches.get_one::<f64>("fixed-altitude")
    }
    pub fn apriori(&self) -> Option<(f64, f64, f64)> {
        self.parse_vec3d_f64("apriori")
    }
    pub fn timescale(&self) -> TimeScale {
        TimeScale::GPST
    }
    // Test Helper for CI/CD thorough verification purposes
    pub fn test_helper(&self) -> Tester {
        Tester {
            kinematics: !self.matches.get_flag("static"),
            max_velocity_m_s: self.max_velocity_m_s(),
            max_absolute_error_m: self.max_absolute_error_m(),
            vdop_var_threshold: self.vdop_var_threshold(),
            gdop_var_threshold: self.gdop_var_threshold(),
            max_gdop: self.max_gdop(),
            timescale: self.timescale(),
            min_sv_elev: self.min_sv_elev(),
            max_sv_elev: self.max_sv_elev(),
            min_sv_azim: self.min_sv_azim(),
            max_sv_azim: self.max_sv_azim(),
            max_tropod: self.max_tropod(),
            max_ionod: self.max_ionod(),
        }
    }
    fn parse_vec3d_f64(&self, key: &str) -> Option<&(f64, f64, f64)> {
        self.matches.get_one::<(f64, f64, f64)>(key)
    }
}
