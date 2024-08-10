pub struct Tester {
    pub kinematics: bool,
    pub max_velocity_m_s: (f64, f64, f64),
    pub max_absolute_error_m: (f64, f64, f64),
    pub vdop_var_threshold: f64,
    pub gdop_var_threshold: f64,
    pub max_gdop: f64,
    pub timescale: TimeScale,
    pub min_sv_elev: Option<f64>,
    pub min_sv_azim: Option<f64>,
    pub max_sv_azim: Option<f64>,
    pub max_tropod: Option<f64>,
    pub max_ionod: Option<f64>,
}
