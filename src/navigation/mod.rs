use hifitime::Unit;
use log::{debug, error};
use nyx::cosmic::SPEED_OF_LIGHT_M_S;

pub mod solutions;
pub use solutions::{
    //InvalidationCause,
    PVTSolution,
    PVTSolutionType,
};

// mod filter;
// mod input;
// mod output;

use nalgebra::{
    base::dimension::U4, ArrayStorage, DVector, DimName, Dynamic, Matrix, Matrix1x4, Matrix3,
    Matrix4, MatrixXx4, Vector4,
};

use anise::{
    astro::PhysicsResult,
    math::{Vector3, Vector6},
    prelude::{Epoch, Frame},
};

// pub(crate) use input::Input;
// pub(crate) use output::Output;

// pub use filter::Filter;
// pub(crate) use filter::FilterState;

use crate::{
    cfg::{LoopExitCriteria, SolverOpts},
    prelude::{
        Candidate,
        Config,
        Duration,
        Error,
        IonosphereBias, //Method,
        Orbit,
        SV,
    },
};

/// SV Navigation information
#[derive(Debug, Clone, Default)]
pub struct SVInput {
    /// Identitity
    pub sv: SV,
    /// Orbital state
    pub sv_pos_km: (f64, f64, f64),
    /// Orbital velocity
    pub sv_vel_km_s: (f64, f64, f64),
    /// Elevation from RX position
    pub elevation: f64,
    /// Azimuth from RX position
    pub azimuth: f64,
    /// Relativistic path range is evaluated for each contributor (only once)
    pub relativistic_path_range_m: f64,
    /// Troposphere bias in meters of delay
    pub tropo_bias: Option<f64>,
    /// Ionosphere bias
    pub iono_bias: Option<IonosphereBias>,
    /// Correction to said constellation, expressed as [Duration]
    pub clock_correction: Option<Duration>,
}

/// [Navigation] filter [DilutionOfPrecision]
#[derive(Clone, Default, Copy)]
pub(crate) struct DilutionOfPrecision {
    /// Geometric DOP
    pub gdop: f64,
    /// Horizontal DOP
    pub hdop: f64,
    /// Vertical DOP
    pub vdop: f64,
    /// Temporal DOP
    pub tdop: f64,
}

impl DilutionOfPrecision {
    pub(crate) fn q_enu(h: Matrix4<f64>, lat_rad: f64, lon_rad: f64) -> Matrix3<f64> {
        let r = Matrix3::<f64>::new(
            -lon_rad.sin(),
            -lon_rad.cos() * lat_rad.sin(),
            lat_rad.cos() * lon_rad.cos(),
            lon_rad.cos(),
            -lat_rad.sin() * lon_rad.sin(),
            lat_rad.cos() * lon_rad.sin(),
            0.0_f64,
            lat_rad.cos(),
            lon_rad.sin(),
        );

        let q_3 = Matrix3::<f64>::new(
            h[(0, 0)],
            h[(0, 1)],
            h[(0, 2)],
            h[(1, 0)],
            h[(1, 1)],
            h[(1, 2)],
            h[(2, 0)],
            h[(2, 1)],
            h[(2, 2)],
        );

        r.clone().transpose() * q_3 * r
    }

    /// Creates new [DillutionOfPrecision] from matrix
    pub fn new(state: &State, g: Matrix<f64, U4, U4, ArrayStorage<f64, 4, 4>>) -> Self {
        let (lat_rad, long_rad) = (state.lat_ddeg.to_radians(), state.long_ddeg.to_radians());
        let q_enu = Self::q_enu(g, lat_rad, long_rad);

        Self {
            gdop: g.trace().sqrt(),
            tdop: g[(3, 3)].sqrt(),
            vdop: q_enu[(2, 2)].sqrt(),
            hdop: (q_enu[(0, 0)] + q_enu[(1, 1)]).sqrt(),
        }
    }
}

#[derive(Clone, Default, Copy)]
pub struct State {
    /// [Epoch]
    pub t: Epoch,
    /// Clock state as [Duration]
    pub clock: Duration,
    /// Altitude above mean sea level (km)
    pub alt_km: f64,
    /// Latitude (ddeg)
    pub lat_ddeg: f64,
    /// Longitude (ddeg)
    pub long_ddeg: f64,
    /// ECEF position (m)
    pub pos_m: (f64, f64, f64),
    /// Velocity vector (ECEF m.s⁻¹)
    pub vel_m_s: (f64, f64, f64),
}

impl State {
    /// Create new [State] from ECEF coordinates.
    pub fn from_ecef_m(pos_m: Vector3, t: Epoch, frame: Frame) -> PhysicsResult<Self> {
        let pos_vel = Vector6::new(pos_m[0], pos_m[1], pos_m[2], 0.0, 0.0, 0.0);
        let orbit = Orbit::from_cartesian_pos_vel(pos_vel / 1.0E3, t, frame);
        Self::from_orbit(&orbit)
    }

    /// Create new [State] from [Orbit]al solution.
    pub fn from_orbit(orbit: &Orbit) -> PhysicsResult<Self> {
        let pos_vel_m = orbit.to_cartesian_pos_vel() * 1.0E3;
        let latlongalt = orbit.latlongalt()?;
        Ok(Self {
            t: orbit.epoch,
            alt_km: latlongalt.2,
            lat_ddeg: latlongalt.0,
            long_ddeg: latlongalt.1,
            clock: Default::default(),
            pos_m: (pos_vel_m[0], pos_vel_m[1], pos_vel_m[2]),
            vel_m_s: (pos_vel_m[3], pos_vel_m[4], pos_vel_m[5]),
        })
    }

    /// Converts [State] to [Orbit]
    pub fn to_orbit(&self, frame: Frame) -> Orbit {
        let pos_vel_km = Vector6::new(
            self.pos_m.0 / 1.0E3,
            self.pos_m.1 / 1.0E3,
            self.pos_m.2 / 1.0E3,
            self.vel_m_s.0 / 1.0E3,
            self.vel_m_s.1 / 1.0E3,
            self.vel_m_s.2 / 1.0E3,
        );
        Orbit::from_cartesian_pos_vel(pos_vel_km, self.t, frame)
    }

    /// Update [State]
    pub fn update(&mut self, dx: Vector4<f64>) {
        self.pos_m.0 += dx[0];
        self.pos_m.1 += dx[1];
        self.pos_m.2 += dx[2];

        self.clock += (dx[3] / SPEED_OF_LIGHT_M_S) * Unit::Second;
    }
}

/// [Navigation] Filter
pub(crate) struct Navigation {
    /// [Config] preset
    cfg: Config,
    pub iter: usize,
    /// Filter [State]
    pub state: State,
    /// Measurement vector
    pub b: DVector<f64>,
    /// [SV] information
    pub sv: Vec<SVInput>,
    /// [DilutionOfPrecision]
    pub(crate) dop: DilutionOfPrecision,
}

impl Navigation {
    /// Creates new [Navigation] filter
    /// ## Input
    /// - cfg: [Config] preset
    /// - state: initial state
    /// - candidates: selected [Candidate]s
    /// - size: number of proposal
    /// ## Returns
    /// - [Navigation], [Error]
    pub fn new(
        t: Epoch,
        cfg: &Config,
        state: State,
        candidates: &[Candidate],
        size: usize,
    ) -> Result<Self, Error> {
        let mut sv = Vec::with_capacity(size);

        let size = candidates.len();

        match cfg.solution {
            PVTSolutionType::PositionVelocityTime => {
                if size < U4::USIZE {
                    return Err(Error::MatrixMinimalDimension);
                }
                if size < U4::USIZE {
                    return Err(Error::MatrixDimension);
                }
            },
            PVTSolutionType::TimeOnly => {
                if size < 1 {
                    return Err(Error::MatrixMinimalDimension);
                }
            },
        }

        let mut j = 0;
        let mut b = DVector::<f64>::zeros(size);

        for i in 0..size {
            match candidates[i].vector_contribution(t, cfg, state.pos_m) {
                Ok((b_i, dr_i)) => {
                    b[j] = b_i;

                    let mut input = SVInput::default();

                    input.sv = candidates[i].sv;
                    input.relativistic_path_range_m = dr_i;

                    if cfg.modeling.relativistic_path_range {
                        debug!(
                            "{}({}) - relativisitic path range: {:.3}m",
                            t, candidates[i].sv, dr_i
                        );
                    }

                    sv.push(input);
                    j += 1;
                },
                Err(e) => {
                    error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                },
            }
        }

        if j < U4::USIZE {
            Err(Error::MatrixMinimalDimension)
        } else {
            Ok(Self {
                b,
                sv,
                state,
                iter: 0,
                cfg: cfg.clone(),
                dop: DilutionOfPrecision::default(),
            })
        }
    }

    /// Iterates [Navigation] filter, updates initial state internally.
    pub fn iterate(
        &mut self,
        t: Epoch,
        candidates: &[Candidate],
        size: usize,
    ) -> Result<bool, Error> {
        let mut j = 0;
        let mut converged = false;
        let mut h = MatrixXx4::<f64>::zeros(size);

        // form matrix
        for i in 0..size {
            let dr_j = self.sv[j].relativistic_path_range_m;

            match candidates[i].matrix_contribution(&self.cfg, dr_j, self.state.pos_m) {
                Ok((dx, dy, dz, dt)) => {
                    h[(j, 0)] = dx;
                    h[(j, 1)] = dy;
                    h[(j, 2)] = dz;
                    h[(j, 3)] = dt;
                    j += 1;
                },
                Err(e) => {
                    error!("({}) does not contribute: {}", candidates[i].sv, e);
                },
            }
        }

        // verify matrix formation validity
        if j < U4::USIZE {
            return Err(Error::MatrixMinimalDimension);
        }

        // run
        let ht = h.transpose();
        let ht_h = ht.clone() * h.clone();
        let ht_h_inv = ht_h.try_inverse().ok_or(Error::MatrixInversion)?;
        let ht_b = ht * self.b.clone();
        let dx = ht_h_inv * ht_b;

        self.iter += 1;
        self.state.update(dx);

        self.dop = DilutionOfPrecision::new(&self.state, ht_h_inv);

        match self.cfg.solver.filter.loop_exit {
            LoopExitCriteria::Iteration(max_iter) => {
                if self.iter == max_iter {
                    converged = true;
                    debug!(
                        "{} - {}/{} - filter convergence x={}, y={}, z={}, dt={}",
                        t,
                        self.iter,
                        max_iter,
                        self.state.pos_m.0,
                        self.state.pos_m.1,
                        self.state.pos_m.2,
                        self.state.clock,
                    );
                }
            },
        }

        // update models (if desired)
        if !converged {
            let mut j = 0;
            if self.cfg.solver.filter.model_update {
                for i in 0..size {
                    match candidates[i].vector_contribution(t, &self.cfg, self.state.pos_m) {
                        Ok((b_i, _)) => {
                            self.b[j] = b_i;
                            j += 1;
                        },
                        Err(e) => {
                            error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                        },
                    }
                }
            }
        }

        Ok(converged)
    }
}

#[cfg(test)]
mod test {
    use super::{DilutionOfPrecision, State};
    use nalgebra::Matrix4;

    #[test]
    fn test_dop() {
        let state = State::default();

        let matrix = Matrix4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );

        let dop = DilutionOfPrecision::new(&state, matrix);

        assert_eq!(dop.gdop, (1.0_f64 + 6.0_f64 + 11.0_f64 + 16.0_f64).sqrt());
        assert_eq!(dop.tdop, 16.0_f64.sqrt());
    }
}
