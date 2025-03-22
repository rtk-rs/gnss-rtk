use crate::prelude::{Orbit, TimeScale};

#[derive(Default, Debug, Copy, Clone)]
pub struct TestConfig {
    pub kinematic: bool,
    pub timescale: TimeScale,
    pub max_gdop: Option<f64>,
    pub max_tdop: Option<f64>,
    pub rx_orbit: Option<Orbit>,
    pub max_xyz_err_m: (f64, f64, f64),
    pub max_velocity_m_s: (f64, f64, f64),
}

impl TestConfig {
    /// Builds static survey [TestConfig]
    pub fn static_survey(
        timescale: TimeScale,
        rx_orbit: Orbit,
        max_xyz_err_m: (f64, f64, f64),
    ) -> Self {
        let mut s = Self::default();
        s.timescale = timescale;
        s.rx_orbit = Some(rx_orbit);
        s.kinematic = false;
        s.max_xyz_err_m = max_xyz_err_m.clone();
        s.max_velocity_m_s = (1.0E-5, 1.0E-5, 1.0E-5);
        s
    }

    /// Set max tdop criteria
    pub fn with_max_tdop(&self, tdop: f64) -> Self {
        let mut s = self.clone();
        s.max_tdop = Some(tdop);
        s
    }

    /// Set max gdop criteria
    pub fn with_max_gdop(&self, gdop: f64) -> Self {
        let mut s = self.clone();
        s.max_gdop = Some(gdop);
        s
    }
}
