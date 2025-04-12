use log::{debug, error};

use std::collections::HashMap;

use crate::{
    ambiguity::Solver as AmbiguitySolver,
    averager::Averager,
    constants::Constants,
    prelude::{Candidate, Config, Duration, Epoch, Frame, Orbit, OrbitSource, SV},
    smoothing::Smoother,
};

use nalgebra::{Matrix3, Vector3};

pub mod postfit;
pub mod prefit;

pub struct Pool {
    /// ECEF [Frame]
    earth_cef: Frame,
    /// Current [Candidate]s pool
    inner: Vec<Candidate>,
    /// Previous [Candidate]s pool
    past: Vec<Candidate>,
    /// [Smoother]
    smoother: Smoother,
    /// [AmbiguitySolver]
    solver: HashMap<SV, AmbiguitySolver>,
}

fn orbit_rotation(t: Epoch, dt: Duration, orbit: Orbit, modeling: bool, frame: Frame) -> Orbit {
    let we = Constants::EARTH_ANGULAR_VEL_RAD * dt.to_seconds();
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

impl Pool {
    /// Allocate new [Pool]
    pub fn allocate(smoothing_win_len: usize, earth_cef: Frame) -> Self {
        Self {
            earth_cef,
            past: Vec::with_capacity(8),
            inner: Vec::with_capacity(8),
            solver: HashMap::with_capacity(8),
            smoother: Smoother::new(smoothing_win_len),
        }
    }

    /// Prepare for new epoch
    pub fn new_epoch(&mut self, candidates: &[Candidate]) {
        self.inner = candidates.to_vec();
    }

    /// Conclude ongoing epoch.
    pub fn end_of_epoch(&mut self) {
        self.past = self.inner.clone();
    }

    pub fn retain<F>(&mut self, f: F)
    where
        F: FnMut(&Candidate) -> bool,
    {
        self.inner.retain(f)
    }

    pub fn retain_mut<F>(&mut self, f: F)
    where
        F: FnMut(&mut Candidate) -> bool,
    {
        self.inner.retain_mut(f)
    }

    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn candidates(&self) -> &[Candidate] {
        &self.inner
    }

    /// Determine orbital states
    pub fn orbital_states<O: OrbitSource>(&mut self, cfg: &Config, orbits: &mut O) {
        self.inner.retain_mut(|cd| match cd.tx_epoch(cfg) {
            Ok(_) => {
                if let Some(orbit) = orbits.next_at(cd.t_tx, cd.sv, self.earth_cef) {
                    debug!("{} - sv dt_tx={} sv t={}", cd.t, cd.t_tx, cd.t);

                    let orbit = orbit_rotation(
                        cd.t,
                        cd.dt_tx,
                        orbit,
                        cfg.modeling.earth_rotation,
                        self.earth_cef,
                    );

                    cd.orbit = Some(orbit);

                    true
                } else {
                    false
                }
            },
            Err(e) => {
                error!("{} ({}) - tx time error: {}", cd.t, cd.sv, e);
                false
            },
        });
    }
}
