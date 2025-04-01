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
pub struct NominalState {
    t: Epoch,
    stm: Matrix6<f64>,
    pos_m: (f64, f64, f64),
    vel_m_s: (f64, f64, f64),
    clock_dt: Duration,
}

impl NominalState {
    fn from_state(state: &State, sampling_interval: Duration) -> Self {
        let dt_s = sampling_interval.to_seconds();
        let mut stm = Matrix6::<f64>::identity();
        stm[(0, 3)] = dt_s;
        stm[(1, 4)] = dt_s;
        stm[(2, 5)] = dt_s;

        Self {
            stm,
            t: state.t,
            pos_m: state.pos_m,
            vel_m_s: state.vel_m_s,
            clock_dt: state.clock_dt,
        }
    }

    fn to_state(&self, frame: Frame) -> State {
        State {
            t: self.t,
            frame,
            pos_m: self.pos_m,
            clock_dt: self.clock_dt,
            clock_drift_s_s: 0.0,
            vel_m_s: self.vel_m_s,
            lat_long_alt_deg_deg_km: (0.0, 0.0, 0.0), // update, using Orbital calcs
        }
    }
}

impl std::fmt::Display for NominalState {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "({}) - dt={} | x={}(m) y={}(m) z={}(m) | vel_x={}(m/s) vel_y={}[m/s] vel_z={}[m/s]",
            self.t,
            self.clock_dt,
            self.pos_m.0,
            self.pos_m.1,
            self.pos_m.2,
            self.vel_m_s.0,
            self.vel_m_s.1,
            self.vel_m_s.2,
        )
    }
}

impl std::fmt::LowerExp for NominalState {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "({}) - dt={} | x={}(m) y={}(m) z={}(m) | vel_x={}(m/s) vel_y={}[m/s] vel_z={}[m/s]",
            self.t,
            self.clock_dt,
            self.pos_m.0,
            self.pos_m.1,
            self.pos_m.2,
            self.vel_m_s.0,
            self.vel_m_s.1,
            self.vel_m_s.2,
        )
    }
}

impl NyxState for NominalState {
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
        Ok(self.stm)
    }

    fn unset_stm(&mut self) {}
}

pub struct PostfitKf {
    frame: Frame,
    nominal_state: NominalState,
    null_computed_obs: Vector6<f64>,
    measurement_covar: Matrix6<f64>,
    kf: KF<NominalState, U3, U6>,
}

impl PostfitKf {
    /// Builds new [PostfitKf] from initial [State]
    pub fn new(
        frame: Frame,
        state: &State,
        sigma_sol_pos: f64,
        sigma_sol_vel: f64,
        sigma_meas_pos: f64,
        sigma_meas_vel: f64,
        sampling_interval: Duration,
    ) -> Self {
        let nominal_state = NominalState::from_state(state, sampling_interval);

        let q_sol = Vector6::new(
            sigma_sol_pos.powi(2),
            sigma_sol_pos.powi(2),
            sigma_sol_pos.powi(2),
            sigma_sol_vel.powi(2),
            sigma_sol_vel.powi(2),
            sigma_sol_vel.powi(2),
        );

        let q_sol = Matrix6::from_diagonal(&q_sol);
        let kfe = KfEstimate::from_covar(nominal_state, q_sol);
        let ckf = KF::no_snc(kfe);

        // Denoiser without system model
        let null_computed_obs = Vector6::zeros();

        // Measurement variances
        let q_meas = Vector6::new(
            sigma_meas_pos.powi(2),
            sigma_meas_pos.powi(2),
            sigma_meas_pos.powi(2),
            sigma_meas_vel.powi(2),
            sigma_meas_vel.powi(2),
            sigma_meas_vel.powi(2),
        );

        let measurement_covar = Matrix6::from_diagonal(&q_meas);

        Self {
            frame,
            kf: ckf,
            nominal_state,
            null_computed_obs,
            measurement_covar,
        }
    }

    /// Upgrade to EKF
    pub fn ekf(&mut self) {
        self.kf.ekf = true;
    }

    /// Run [PostfitKf] filter
    pub fn run(&mut self, state: &State) -> Result<State, ODError> {
        let real_observations = Vector6::new(
            state.pos_m.0,
            state.pos_m.1,
            state.pos_m.2,
            state.vel_m_s.0,
            state.vel_m_s.1,
            state.vel_m_s.2,
        );

        // measurement
        let (kfe, residual) = self.kf.measurement_update(
            self.nominal_state,
            &real_observations,
            &self.null_computed_obs,
            self.measurement_covar,
            None,
        )?;

        let state = kfe.nominal_state.to_state(self.frame);
        Ok(state)
    }
}
