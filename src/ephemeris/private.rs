use log::{error, debug};
use nalgebra::{Rotation3, Vector3};

use crate::prelude::{Epoch, Frame, Orbit, Ephemeris, Duration};

impl Ephemeris {
    /// Returns True if this [Ephemeris] frame is still valid
    pub fn is_valid(&self, now: Epoch, max_dtoe: Duration) -> bool {
        (now - self.toe).abs() < max_dtoe
    }

    /// Returns ToE in seconds of week
    pub fn weekly_toe_seconds(&self) -> f64 {
        (self.toe.to_time_of_week().1 as f64)/1.0E9
    }
    
    /// Returns ToC in seconds of week
    pub fn weekly_toc_seconds(&self) -> f64 {
        (self.toc.to_time_of_week().1 as f64)/1.0E9
    }

    /// Resolves Kepler equations from [Ephemeris]
    pub fn resolve_state(&self, epoch: Epoch, frame: Frame) -> Option<Orbit> {

        // constants
        const GM_M3_S2: f64 = 3.986004418E14;
        const OMEGA_EARTH: f64 = 7.2921151467E-5;

        let e = self.eccentricity;
        let e_2 = e.powi(2);
        let a = self.semi_major_axis_m;
        let a_3 = a.powi(3);

        let (cus, cuc) = self.cus_cuc_rad;
        let (cis, cic) = self.cis_cic_rad;
        let (crs, crc) = self.crs_crc_m;
        let (i0, idot) = (self.i0_rad, self.idot_rad_s);
        let (omega0, omega, omega_dot) = (self.omega0_rad, self.omega_rad, self.omega_dot_rad_s);

        let timescale = self.sv.constellation.timescale()?;
        let epoch = epoch.to_time_scale(timescale);

        let t_k = (epoch - self.toe).to_seconds();

        let n0 = (GM_M3_S2 / a_3).sqrt();
        let n = n0 + self.dn_rad;
        let m = self.m0_rad + n * t_k;

        let mut i = 0;
        let mut e_k = 0.0_f64;
        let mut e_k_lst = 0.0_f64;

        loop {
            e_k = m + e * e_k_lst.sin();
            if (e_k - e_k_lst).abs() < 1E-10 {
                break;
            }

            i += 1;
            if i == 10 {
                error!("{}({}) - kepler solver in failure", epoch, self.sv);
                return None;
            }
        }

        let (sin_e_k, cos_e_k) = e_k.sin_cos();
        let v_k = ((1.0 - e_2).sqrt() * sin_e_k).atan2(cos_e_k - e);
        let (sin_v_k, cos_v_k) = v_k.sin_cos();

        let phi = v_k + omega;
        let (sin_2phi, cos_2phi) = (2.0 * phi).sin_cos();

        let u_k = phi + cuc * cos_2phi + cus * sin_2phi;
        let r_k = a * (1.0 - e * cos_e_k) + crc * cos_2phi + crs * sin_2phi;
        let i_k = i0 + idot * t_k + cic * cos_2phi + cis * sin_2phi;
        let omega_k = omega0 + (omega_dot - OMEGA_EARTH) * t_k - OMEGA_EARTH * self.weekly_toe_seconds();

        let (x, y, z) = (r_k * u_k.cos(), r_k * u_k.sin(), 0.0);

        // MEO orbit to ECEF rotation matrix
        let rot_x3 = Rotation3::from_axis_angle(&Vector3::x_axis(), i_k);
        let rot_z3 = Rotation3::from_axis_angle(&Vector3::z_axis(), omega_k);
        let rot3 = rot_z3 * rot_x3;

        let xyz_vec3 = Vector3::new(x, y, z);
        let xyz_ecef = rot3 * xyz_vec3;

        let (x_km, y_km, z_km) = (
            xyz_ecef[0] / 1000.0,
            xyz_ecef[1] / 1000.0,
            xyz_ecef[2] / 1000.0,
        );

        debug!("{}({}) - kepler solving x_km={}, y_km={} z_km={} t_k={}", epoch, self.sv, x_km, y_km, z_km, t_k);
        
        Some(Orbit::from_position(x_km, y_km, z_km, epoch, frame))
    }
}
