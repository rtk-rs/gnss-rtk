use nyx::{
    cosmic::State as NyxState,
    dynamics::DynamicsError,
    linalg::{Matrix6, OMatrix, OVector, Vector6, U3, U6},
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
    null_computed_obs: Vector6<f64>,
    measurement_covar: Matrix6<f64>,
    kf: KF<PostfitState, U3, U6>,
}

impl PostfitKf {
    /// Builds new [PostfitKf] from initial [State]
    pub fn new(
        frame: Frame,
        state: &State,
        dt: Duration,
        sigma_sol_pos: f64,
        sigma_sol_vel: f64,
        sigma_meas_pos: f64,
        sigma_meas_vel: f64,
    ) -> Self {
        let state = PostfitState::from_state(state);

        let mut stm = Vector6::identity();

        let dt_s = dt.to_seconds();

        stm[(0, 4)] = dt_s;
        stm[(1, 5)] = dt_s;
        stm[(2, 6)] = dt_s;

        let q_sol = Vector6::new(
            sigma_sol_pos,
            sigma_sol_pos,
            sigma_sol_pos,
            sigma_sol_vel,
            sigma_sol_vel,
            sigma_sol_vel,
        );

        let q_sol = Matrix6::from_diagonal(&q_sol);
        let kfe = KfEstimate::from_covar(state, q_sol);
        let ckf = KF::no_snc(kfe);

        // Denoiser without system model
        let null_computed_obs = Vector6::zeros();

        // Measurement variances
        let q_meas = Vector6::new(
            sigma_meas_pos,
            sigma_meas_pos,
            sigma_meas_pos,
            sigma_meas_vel,
            sigma_meas_vel,
            sigma_meas_vel,
        );

        let measurement_covar = Matrix6::from_diagonal(&q_meas);

        Self {
            frame,
            kf: ckf,
            null_computed_obs,
            measurement_covar,
        }
    }

    /// Run [PostfitKf] filter
    pub fn run(
        &mut self,
        state: &State,
        sigma_m_pos_m: f64,
        sigma_m_vel_m_s: f64,
    ) -> Result<State, ODError> {
        // real observation is the new state we have possibly just improved
        let real_observations = PostfitState::from_state(state);

        let r = Vector6::new(
            sigma_m_pos_m,
            sigma_m_pos_m,
            sigma_m_pos_m,
            sigma_m_vel_m_s,
            sigma_m_vel_m_s,
            sigma_m_vel_m_s,
        );

        let kfe = self.kf.measurement_update(
            self.nominal_state,
            real_observations,
            &self.null_computed_obs,
            self.measurement_covar,
            None,
        )?;

        panic!("oops");
    }
}
