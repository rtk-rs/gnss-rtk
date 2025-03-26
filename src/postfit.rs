use nyx::{
    cosmic::State as NyxState,
    dynamics::DynamicsError,
    linalg::{Matrix3, Matrix6, OMatrix, OVector, Vector3, Vector6, U3, U6},
    od::{
        filter::kalman::{KfEstimate, KF},
        Filter, ODError,
    },
};

use crate::{
    navigation::state::State,
    prelude::{Duration, Epoch, Frame, Orbit, EARTH_J2000},
};

#[derive(Clone, Copy, Default, PartialEq)]
pub struct PostfitState {
    t: Epoch,
    dt: Duration,
    pos_m: (f64, f64, f64),
    vel_m_s: (f64, f64, f64),
}

impl PostfitState {
    fn from_state(state: &State) -> Self {
        Self {
            t: state.t,
            dt: state.clock_dt,
            pos_m: state.pos_m,
            vel_m_s: state.vel_m_s,
        }
    }

    fn to_state(&self, frame: Frame) -> State {
        State {
            t: self.t,
            frame,
            clock_dt: self.dt,
            clock_drift_s_s: 0.0, //TODO
            pos_m: self.pos_m,
            lat_long_alt_deg_deg_km: (0.0, 0.0, 0.0), //TODO
            vel_m_s: self.vel_m_s,
        }
    }
}

impl std::fmt::Display for PostfitState {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "({}) - dt={} | x={}(m) y={}(m) z={}(m) | vel_x={}(m/s) vel_y={}[m/s] vel_z={}[m/s]",
            self.t,
            self.dt,
            self.pos_m.0,
            self.pos_m.1,
            self.pos_m.2,
            self.vel_m_s.0,
            self.vel_m_s.1,
            self.vel_m_s.2,
        )
    }
}

impl std::fmt::LowerExp for PostfitState {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "({}) - dt={} | x={}(m) y={}(m) z={}(m) | vel_x={}(m/s) vel_y={}[m/s] vel_z={}[m/s]",
            self.t,
            self.dt,
            self.pos_m.0,
            self.pos_m.1,
            self.pos_m.2,
            self.vel_m_s.0,
            self.vel_m_s.1,
            self.vel_m_s.2,
        )
    }
}

impl NyxState for PostfitState {
    type Size = U6;
    type VecLength = U6;

    fn epoch(&self) -> Epoch {
        self.t
    }

    fn set_epoch(&mut self, t: Epoch) {
        self.t = t;
    }

    fn to_vector(&self) -> OVector<f64, Self::VecLength> {
        Vector6::new(
            self.pos_m.0,
            self.pos_m.1,
            self.pos_m.2,
            self.vel_m_s.0,
            self.vel_m_s.1,
            self.vel_m_s.2,
        )
    }

    fn orbit(&self) -> Orbit {
        Orbit::from_cartesian_pos_vel(self.to_vector(), self.t, EARTH_J2000)
    }

    fn set(&mut self, epoch: Epoch, vector: &OVector<f64, Self::VecLength>) {
        self.t = epoch;
        self.pos_m = (vector[0], vector[1], vector[2]);
        self.vel_m_s = (vector[3], vector[4], vector[5]);
    }

    fn stm(&self) -> Result<OMatrix<f64, Self::Size, Self::Size>, DynamicsError> {
        let diag = Vector6::<f64>::identity();
        Ok(Matrix6::<f64>::from_diagonal(&diag))
    }

    fn unset_stm(&mut self) {}
}

pub struct PostfitKf {
    frame: Frame,
    kf: KF<PostfitState, U3, U6>,
}

impl PostfitKf {
    /// Builds new [PostfitKf] from initial [State]
    pub fn new(frame: Frame, state: &State) -> Self {
        let state = PostfitState::from_state(state);

        let q_diag = Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let q = Matrix6::from_diagonal(&q_diag);

        let kfe = KfEstimate::from_covar(state, q);

        Self {
            frame,
            kf: KF::no_snc(kfe),
        }
    }

    /// Run [PostfitKf] filter
    pub fn run(&mut self, state: &State) -> Result<State, ODError> {
        // time update / predict
        let nominal_state = PostfitState::from_state(state);
        let kfe = self.kf.time_update(nominal_state)?;

        let state = kfe.nominal_state;

        // self.kf.measurement_update(nominal_state, real_obs, computed_obs, measurement_covar, resid_rejection);
        Ok(state.to_state(self.frame))
    }
}
