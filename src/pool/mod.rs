use log::error;

use std::collections::HashMap;

use crate::{
    // ambiguity::Solver as AmbiguitySolver,
    constants::EARTH_ANGULAR_VEL_RAD,
    prelude::{
        Candidate, Config, Duration, Ephemeris, EphemerisSource, Epoch, Frame, Orbit, OrbitSource,
        Rc, SV,
    },
    smoothing::Smoother,
};

use nalgebra::{Matrix3, Vector3};

pub mod postfit;
pub mod prefit;

pub struct Pool<EPH: EphemerisSource, ORB: OrbitSource> {
    /// ECEF [Frame]
    earth_cef: Frame,

    /// Current [Candidate]s pool
    inner: Vec<Candidate>,

    /// Previous [Candidate]s pool
    past: Vec<Candidate>,

    /// Measurements [Smoother]
    smoother: Smoother,

    /// [OrbitSource]
    orb_source: Rc<ORB>,

    /// [EphemerisSource]
    eph_source: Rc<EPH>,

    /// [Ephemeris] Buffer
    eph_buffer: HashMap<SV, Ephemeris>,

    /// Cycle slips
    cycle_slips: Vec<SV>,
}

fn orbit_rotation(t: Epoch, dt: Duration, orbit: &Orbit, modeling: bool, frame: Frame) -> Orbit {
    let we = EARTH_ANGULAR_VEL_RAD * dt.to_seconds();
    let (we_sin, we_cos) = we.sin_cos();
    let dcm3 = if modeling {
        Matrix3::new(we_cos, we_sin, 0.0, -we_sin, we_cos, 0.0, 0.0, 0.0, 1.0)
    } else {
        Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    };

    let state_m = orbit.to_cartesian_pos_vel() * 1.0E3;
    let position_m = Vector3::new(state_m[0], state_m[1], state_m[2]);
    let position = dcm3 * position_m;

    Orbit::from_position(
        position[0] / 1.0E3,
        position[1] / 1.0E3,
        position[2] / 1.0E3,
        t,
        frame,
    )
}

impl<EPH: EphemerisSource, ORB: OrbitSource> Pool<EPH, ORB> {
    /// Allocate new [Pool]
    pub fn allocate(
        smoothing_win_len: usize,
        earth_cef: Frame,
        eph_source: Rc<EPH>,
        orb_source: Rc<ORB>,
    ) -> Self {
        Self {
            earth_cef,
            eph_source,
            orb_source,
            past: Vec::with_capacity(8),
            inner: Vec::with_capacity(8),
            cycle_slips: Vec::with_capacity(8),
            eph_buffer: HashMap::with_capacity(8),
            smoother: Smoother::new(smoothing_win_len),
        }
    }

    /// Prepare for new epoch
    pub fn new_epoch(&mut self, candidates: &[Candidate]) {
        self.inner = candidates.to_vec();
    }

    // pub fn retain<F>(&mut self, f: F)
    // where
    //     F: FnMut(&Candidate) -> bool,
    // {
    //     self.inner.retain(f)
    // }

    // pub fn retain_mut<F>(&mut self, f: F)
    // where
    //     F: FnMut(&mut Candidate) -> bool,
    // {
    //     self.inner.retain_mut(f)
    // }

    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn candidates(&self) -> &[Candidate] {
        &self.inner
    }

    /// Ephemeris update attempt
    fn ephemeris_update(&mut self) {
        for cd in self.inner.iter() {
            // update attempt
            if let Some(data) = self.eph_source.ephemeris_data(cd.t, cd.sv) {
                self.eph_buffer.insert(cd.sv, data);
            }
        }
    }

    /// Determine orbital states
    pub fn orbital_states(&mut self, cfg: &Config) {
        self.ephemeris_update();

        self.inner.retain_mut(|cd| match cd.tx_epoch(cfg) {
            Ok(_) => {
                // direct state determination
                let mut determined = false;

                if let Some(orbit) = &self.orb_source.state_at(cd.t_tx, cd.sv, self.earth_cef) {
                    let orbit = orbit_rotation(
                        cd.t,
                        cd.dt_tx,
                        orbit,
                        cfg.modeling.earth_rotation,
                        self.earth_cef,
                    );

                    cd.orbit = Some(orbit);
                    determined = true;
                }

                // indirect state determination
                if !determined {
                    if let Some(eph) = &self.eph_buffer.get(&cd.sv) {
                        if let Some(state) = eph.resolve_state(cd.t_tx, self.earth_cef) {
                            let state = orbit_rotation(
                                cd.t,
                                cd.dt_tx,
                                &state,
                                cfg.modeling.earth_rotation,
                                self.earth_cef,
                            );

                            cd.orbit = Some(state);
                            determined = true;
                        }
                    }
                }

                determined
            },
            Err(e) => {
                error!("{}({}) - tx time error: {}", cd.t, cd.sv, e);
                false
            },
        });
    }
}
