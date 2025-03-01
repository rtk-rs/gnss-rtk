use hifitime::Unit;
use log::error;
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

        let mut j = 0;

        for i in 0..candidates.len() {
            match candidates[i].spp_matrix_contribution(cfg, state.pos_m) {
                Ok(contribution) => {
                    h[(j, 0)] = contribution.h[(0, 0)];
                    h[(j, 1)] = contribution.h[(0, 1)];
                    h[(j, 2)] = contribution.h[(0, 2)];
                    h[(j, 3)] = contribution.h[(0, 3)];
                    b[j] = contribution.b;
                    j += 1;
                },
                Err(e) => {
                    error!("({}) does not contribute: {}", candidates[i].sv, e);
                },
            }
        }

        Ok(Self {
            b,
            h,
            iter: 0,
            dx: Vector4::zeros(),
        })
    }

    /// Iterates this [Navigation] filter, updating provided state
    pub fn iterate(&mut self) -> Result<(), Error> {
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

#[cfg(test)]
mod test {
    use std::str::FromStr;

    use anise::{constants::frames::EARTH_J2000, math::Vector6};

    use crate::{
        navigation::{Navigation, State},
        prelude::{
            Almanac, Candidate, Carrier, Config, Epoch, Error, Observation, Orbit, Vector3, SV,
        },
    };

    use nalgebra::{Matrix4, Vector4};

    #[test]
    fn pvt_matrix_failures() {
        let cfg = Config::default();
        let (x0_m, y0_m, z0_m) = (0.0_f64, 0.0_f64, 0.0_f64);

        let almanac = Almanac::until_2035().unwrap();
        let frame = almanac.frame_from_uid(EARTH_J2000).unwrap();

        let state = &State::from_ecef_m(Vector3::new(x0_m, y0_m, z0_m), Default::default(), frame);

        let t = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

        let g01 = SV::from_str("G01").unwrap();
        let g02 = SV::from_str("G02").unwrap();
        let g03 = SV::from_str("G03").unwrap();
        let g04 = SV::from_str("G04").unwrap();

        let mut candidates = vec![Candidate::new(
            g01,
            t,
            vec![Observation {
                snr_dbhz: None,
                carrier: Carrier::L1,
                pseudo_range_m: Some(1.0),
                phase_range_m: None,
                doppler: None,
                ambiguity: None,
            }],
        )];

        match Navigation::new(&cfg, state, &candidates) {
            Err(e) => match e {
                Error::MatrixMinimalDimension => {},
                e => panic!("failed with invalid error: {}", e),
            },
            _ => panic!("should have failed 1x4"),
        }

        candidates.push(Candidate::new(
            g02,
            t,
            vec![Observation {
                snr_dbhz: None,
                carrier: Carrier::L1,
                pseudo_range_m: Some(1.0),
                phase_range_m: None,
                doppler: None,
                ambiguity: None,
            }],
        ));

        match Navigation::new(&cfg, state, &candidates) {
            Err(e) => match e {
                Error::MatrixMinimalDimension => {},
                e => panic!("failed with invalid error: {}", e),
            },
            _ => panic!("should have failed 2x4"),
        }

        candidates.push(Candidate::new(
            g03,
            t,
            vec![Observation {
                snr_dbhz: None,
                carrier: Carrier::L1,
                pseudo_range_m: Some(1.0),
                phase_range_m: None,
                doppler: None,
                ambiguity: None,
            }],
        ));

        match Navigation::new(&cfg, state, &candidates) {
            Err(e) => match e {
                Error::MatrixMinimalDimension => {},
                e => panic!("failed with invalid error: {}", e),
            },
            _ => panic!("should have failed 3x4"),
        }

        let candidates = vec![
            Candidate::new(
                g01,
                t,
                vec![Observation {
                    snr_dbhz: None,
                    carrier: Carrier::L1,
                    pseudo_range_m: Some(1.0),
                    phase_range_m: None,
                    doppler: None,
                    ambiguity: None,
                }],
            ),
            Candidate::new(
                g02,
                t,
                vec![Observation {
                    snr_dbhz: None,
                    carrier: Carrier::L1,
                    pseudo_range_m: Some(1.0),
                    phase_range_m: None,
                    doppler: None,
                    ambiguity: None,
                }],
            ),
            Candidate::new(
                g03,
                t,
                vec![Observation {
                    snr_dbhz: None,
                    carrier: Carrier::L1,
                    pseudo_range_m: Some(1.0),
                    phase_range_m: None,
                    doppler: None,
                    ambiguity: None,
                }],
            ),
            Candidate::new(
                g04,
                t,
                vec![Observation {
                    snr_dbhz: None,
                    carrier: Carrier::L1,
                    pseudo_range_m: Some(1.0),
                    phase_range_m: None,
                    doppler: None,
                    ambiguity: None,
                }],
            ),
        ];

        let mut nav = Navigation::new(&cfg, &state, &candidates).unwrap();

        assert_eq!(nav.b, Vector4::zeros(), "Only unresolved states!");
        assert_eq!(nav.h, Matrix4::zeros(), "Only unresolved states!");

        match nav.iterate() {
            Ok(_) => panic!("should have failed (due to unresolved state"),
            Err(e) => match e {
                Error::MatrixInversion => {},
                e => panic!("failed with unexpected error: {}", e),
            },
        }
    }

    #[test]
    fn pvt_matrix_noclock_nobias() {
        let mut cfg = Config::default();

        cfg.modeling.sv_clock_bias = false;
        cfg.modeling.iono_delay = false;
        cfg.modeling.sv_total_group_delay = false;

        let (x0_m, y0_m, z0_m) = (1.0_f64, 2.0_f64, 3.0_f64);

        let almanac = Almanac::until_2035().unwrap();
        let frame = almanac.frame_from_uid(EARTH_J2000).unwrap();

        let state = &State::from_ecef_m(Vector3::new(x0_m, y0_m, z0_m), Default::default(), frame);

        let t = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

        let g01 = SV::from_str("G01").unwrap();
        let g02 = SV::from_str("G02").unwrap();
        let g03 = SV::from_str("G03").unwrap();
        let g04 = SV::from_str("G04").unwrap();

        let mut candidates = vec![
            Candidate::new(
                g01,
                t,
                vec![Observation {
                    snr_dbhz: None,
                    carrier: Carrier::L1,
                    pseudo_range_m: Some(0.1),
                    phase_range_m: None,
                    doppler: None,
                    ambiguity: None,
                }],
            ),
            Candidate::new(
                g02,
                t,
                vec![Observation {
                    snr_dbhz: None,
                    carrier: Carrier::L1,
                    pseudo_range_m: Some(0.2),
                    phase_range_m: None,
                    doppler: None,
                    ambiguity: None,
                }],
            ),
            Candidate::new(
                g03,
                t,
                vec![Observation {
                    snr_dbhz: None,
                    carrier: Carrier::L1,
                    pseudo_range_m: Some(0.3),
                    phase_range_m: None,
                    doppler: None,
                    ambiguity: None,
                }],
            ),
            Candidate::new(
                g04,
                t,
                vec![Observation {
                    snr_dbhz: None,
                    carrier: Carrier::L1,
                    pseudo_range_m: Some(0.4),
                    phase_range_m: None,
                    doppler: None,
                    ambiguity: None,
                }],
            ),
        ];

        let sv_coords_m = vec![
            (10.0, 20.0, 30.0),
            (11.0, 21.0, 31.0),
            (12.0, 22.0, 32.0),
            (13.0, 23.0, 33.0),
        ];

        for (nth, coords) in sv_coords_m.iter().enumerate() {
            let pos_vel_m = Vector6::new(coords.0, coords.1, coords.2, 0.0, 0.0, 0.0);
            let orbit = Orbit::from_cartesian_pos_vel(pos_vel_m / 1.0E3, t, frame);
            candidates[nth].set_orbit(orbit);
        }

        let mut nav = Navigation::new(&cfg, state, &candidates).unwrap();

        let r_i = vec![
            candidates[0].l1_pseudo_range().unwrap(),
            candidates[1].l1_pseudo_range().unwrap(),
            candidates[2].l1_pseudo_range().unwrap(),
            candidates[3].l1_pseudo_range().unwrap(),
        ];

        let rho = vec![
            ((x0_m - sv_coords_m[0].0).powi(2)
                + (y0_m - sv_coords_m[0].1).powi(2)
                + (z0_m - sv_coords_m[0].2).powi(2))
            .sqrt(),
            ((x0_m - sv_coords_m[1].0).powi(2)
                + (y0_m - sv_coords_m[1].1).powi(2)
                + (z0_m - sv_coords_m[1].2).powi(2))
            .sqrt(),
            ((x0_m - sv_coords_m[2].0).powi(2)
                + (y0_m - sv_coords_m[2].1).powi(2)
                + (z0_m - sv_coords_m[2].2).powi(2))
            .sqrt(),
            ((x0_m - sv_coords_m[3].0).powi(2)
                + (y0_m - sv_coords_m[3].1).powi(2)
                + (z0_m - sv_coords_m[3].2).powi(2))
            .sqrt(),
        ];

        for i in 0..4 {
            let (dx_m, dy_m, dz_m) = (
                (x0_m - sv_coords_m[i].0) / rho[i],
                (y0_m - sv_coords_m[i].1) / rho[i],
                (z0_m - sv_coords_m[i].2) / rho[i],
            );

            assert_eq!(nav.h[(i, 0)], dx_m, "test failed [({},{})]", i, 0);
            assert_eq!(nav.h[(i, 1)], dy_m, "test failed [({},{})]", i, 1);
            assert_eq!(nav.h[(i, 2)], dz_m, "test failed [({},{})]", i, 2);
            assert_eq!(nav.h[(i, 3)], dx_m, "test failed [({},{})]", i, 3);
        }

        assert_eq!(nav.b, Vector4::zeros());
        nav.iterate().unwrap();
    }
}
