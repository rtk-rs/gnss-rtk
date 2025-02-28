use hifitime::Unit;
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

use nalgebra::{base::dimension::U4, DVector, DimName, Matrix1x4, MatrixXx4, Vector4};

use anise::{
    astro::PhysicsResult,
    math::{Vector3, Vector6},
    prelude::{Epoch, Frame},
};

// pub(crate) use input::Input;
// pub(crate) use output::Output;

// pub use filter::Filter;
// pub(crate) use filter::FilterState;

use crate::prelude::{
    Candidate,
    Config,
    Duration,
    Error,
    IonosphereBias, //Method,
    Orbit,
};

/// SV Navigation information
#[derive(Debug, Clone, Default)]
pub struct SVInput {
    /// Possible [Orbit] state
    pub orbit: Option<Orbit>,
    /// Elevation from RX position
    pub elevation: f64,
    /// Azimuth from RX position
    pub azimuth: f64,
    /// Troposphere bias in meters of delay
    pub tropo_bias: Option<f64>,
    /// Ionosphere bias
    pub iono_bias: Option<IonosphereBias>,
    /// Correction to said constellation, expressed as [Duration]
    pub clock_correction: Option<Duration>,
}

pub(crate) struct State {
    pub t: Epoch,
    pub dt: Duration,
    pub pos_m: (f64, f64, f64),
    pub vel_m_s: (f64, f64, f64),
    pub lat_ddeg: f64,
    pub long_ddeg: f64,
    pub alt_km: f64,
}

impl State {
    /// Create new [State] from ECEF coordinates.
    pub fn from_ecef_m(coords: Vector3, t: Epoch, frame: Frame) -> Self {
        let pos_vel = Vector6::new(coords[0], coords[1], coords[2], 0.0, 0.0, 0.0);
        let orbit = Orbit::from_cartesian_pos_vel(pos_vel, t, frame);
        let s =
            Self::from_orbit(&orbit).unwrap_or_else(|e| panic!("from_orbit on 1st Iter: {}", e));
        s
    }

    /// Create new [State] from [Orbit]al solution.
    pub fn from_orbit(orbit: &Orbit) -> PhysicsResult<Self> {
        let pos_vel_m = orbit.to_cartesian_pos_vel() * 1.0E3;
        let latlongalt = orbit.latlongalt()?;
        Ok(Self {
            t: orbit.epoch,
            alt_km: latlongalt.2,
            dt: Default::default(),
            lat_ddeg: latlongalt.0,
            long_ddeg: latlongalt.1,
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
        self.dt += (dx[3] / SPEED_OF_LIGHT_M_S) * Unit::Second;
    }
}

pub(crate) struct MatrixContribution {
    pub h: Matrix1x4<f64>,
    pub b: f64,
}

#[derive(Debug, Clone)]
pub(crate) struct Navigation {
    b: DVector<f64>,
    h: MatrixXx4<f64>,
    pub iter: usize,
    pub dx: Vector4<f64>,
}

impl Navigation {
    /// Create new [Navigation] filter
    /// ## Input
    /// - cfg: [Config] preset
    /// - apriori: [Orbit]al state
    /// - candidates: [Candidate]s proposal
    /// ## Returns
    /// - [Navigation], [Error]
    pub fn new(cfg: &Config, state: &State, candidates: &[Candidate]) -> Result<Self, Error> {
        const MIN_SIZE: usize = 4;
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

        let mut b = DVector::<f64>::zeros(size);
        let mut h = MatrixXx4::<f64>::zeros(size);

        for i in 0..candidates.len() {
            let contribution = candidates[i].spp_matrix_contribution(i, cfg, state.pos_m);
            h[(i, 0)] = contribution.h[(0, 0)];
            h[(i, 1)] = contribution.h[(0, 1)];
            h[(i, 2)] = contribution.h[(0, 2)];
            h[(i, 3)] = contribution.h[(0, 3)];
            b[i] = contribution.b;
        }

        Ok(Self {
            b,
            h,
            iter: 0,
            dx: Vector4::zeros(),
        })
    }

    /// Iterates this [Navigation] filter, updating provided state
    pub fn iter(&mut self) -> Result<(), Error> {
        let ht = self.h.transpose();
        let ht_h = ht.clone() * self.h.clone();
        let ht_h_inv = ht_h.try_inverse().ok_or(Error::MatrixInversion)?;

        let ht_b = ht * self.b.clone();

        // TODO: only if validated ?
        self.dx = ht_h_inv * ht_b;
        self.iter += 1;
        Ok(())
    }
}
