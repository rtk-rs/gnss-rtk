use log::error;

use crate::{
    navigation::{sv::SVContribution, vector::VectorContribution},
    prelude::{Candidate, Config, Epoch, Error, Method, Vector3},
    rtk::DoubleDifferences,
};

impl Candidate {
    /// Measurement vector contribution.
    pub(crate) fn rtk_vector_contribution(
        &self,
        epoch: Epoch,
        cfg: &Config,
        double_diffs: &DoubleDifferences,
        contribution: &mut SVContribution,
    ) -> Result<VectorContribution, Error> {
        let carrier = match cfg.method {
            Method::SPP | Method::CPP => match self.l1_pseudo_range() {
                Some((c_1, _)) => Some(c_1),
                _ => {
                    error!("{}({}) - missing pseudo range data", epoch, self.sv);
                    None
                },
            },
            Method::PPP | Method::PPP_AR => match self.l1_phase_range() {
                Some((c_1, _)) => Some(c_1),
                _ => {
                    error!("{}({}) - missing phase data", epoch, self.sv);
                    None
                },
            },
        };

        if carrier.is_none() {
            return Err(Error::UnknownCarrierFrequency);
        }

        let carrier = carrier.unwrap();

        let dd = double_diffs
            .double_difference(self.sv, carrier)
            .ok_or(Error::RtkDDPostfitMissing)?;

        Ok(VectorContribution {
            row_1: *dd,
            row_2: 0.0,
            sigma: 0.0,
        })
    }

    /// Matrix contribution.
    ///
    /// ## Input
    ///  - i: matrix row
    ///  - cfg: [Config] preset
    ///  - x0_y0_z0: position coordinates as ECEF (m)
    pub(crate) fn rtk_matrix_contribution(
        &self,
        x0_y0_z0_m: Vector3<f64>,
        pivot_position_ecef_m: (f64, f64, f64),
    ) -> (f64, f64, f64) {
        let (x0_m, y0_m, z0_m) = (x0_y0_z0_m[0], x0_y0_z0_m[1], x0_y0_z0_m[2]);
        let (pivot_x_m, pivot_y_m, pivot_z_m) = pivot_position_ecef_m;

        let orbit = self.orbit.unwrap_or_else(|| {
            panic!(
                "internal error: {}({}) state not fully resolved!",
                self.epoch, self.sv
            );
        });

        let pos_vel_m = orbit.to_cartesian_pos_vel() * 1.0E3;
        let (sv_x_m, sv_y_m, sv_z_m) = (pos_vel_m[0], pos_vel_m[1], pos_vel_m[2]);

        let rho_i =
            ((sv_x_m - x0_m).powi(2) + (sv_y_m - y0_m).powi(2) + (sv_z_m - z0_m).powi(2)).sqrt();

        let e_i = (
            (sv_x_m - x0_m) / rho_i,
            (sv_y_m - y0_m) / rho_i,
            (sv_z_m - z0_m) / rho_i,
        );

        let rho_j =
            ((pivot_x_m - x0_m).powi(2) + (pivot_y_m - y0_m).powi(2) + (pivot_z_m - z0_m).powi(2))
                .sqrt();

        let e_j = (
            (pivot_x_m - x0_m) / rho_j,
            (pivot_y_m - y0_m) / rho_j,
            (pivot_z_m - z0_m) / rho_j,
        );

        (e_j.0 - e_i.0, e_j.1 - e_i.1, e_j.2 - e_i.2)
    }
}

#[cfg(test)]
mod test {
    use crate::{
        prelude::{Config, Epoch, Frame, Method, Orbit},
        tests::{CandidatesBuilder, E05, ROVER_REFERENCE_COORDS_ECEF_M},
    };

    use nalgebra::Vector3;
    use rstest::*;

    use std::str::FromStr;

    #[fixture]
    fn build_earth_frame() -> Frame {
        use crate::tests::earth_frame;
        earth_frame()
    }

    #[test]
    fn rtk_spp_matrix_contribution() {
        let earth_frame = build_earth_frame();
        let t0 = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

        let pivot_position_ecef_km = (16577.017768, -4619.539763, 24092.494804);

        let pivot_position_ecef_m = (
            pivot_position_ecef_km.0 * 1000.0,
            pivot_position_ecef_km.1 * 1000.0,
            pivot_position_ecef_km.2 * 1000.0,
        );

        let e01_position_ecef_km = (-11562.163582, 14053.114306, 23345.128269);

        let e01_position_ecef_m = (
            e01_position_ecef_km.0 * 1000.0,
            e01_position_ecef_km.1 * 1000.0,
            e01_position_ecef_km.2 * 1000.0,
        );

        let mut rover = CandidatesBuilder::build_rover_sv_at(E05, t0);

        rover.orbit = Some(Orbit::from_position(
            e01_position_ecef_km.0,
            e01_position_ecef_km.1,
            e01_position_ecef_km.2,
            t0,
            earth_frame,
        ));

        let cfg = Config::default().with_navigation_method(Method::SPP);

        let x0_y0_z0_m = Vector3::new(
            ROVER_REFERENCE_COORDS_ECEF_M.0,
            ROVER_REFERENCE_COORDS_ECEF_M.1,
            ROVER_REFERENCE_COORDS_ECEF_M.2,
        );

        let (dx, dy, dz) = rover.rtk_matrix_contribution(x0_y0_z0_m, pivot_position_ecef_m);

        let rho_i = ((e01_position_ecef_m.0 - ROVER_REFERENCE_COORDS_ECEF_M.0).powi(2)
            + (e01_position_ecef_m.1 - ROVER_REFERENCE_COORDS_ECEF_M.1).powi(2)
            + (e01_position_ecef_m.2 - ROVER_REFERENCE_COORDS_ECEF_M.2).powi(2))
        .sqrt();

        let rho_j = ((pivot_position_ecef_m.0 - ROVER_REFERENCE_COORDS_ECEF_M.0).powi(2)
            + (pivot_position_ecef_m.1 - ROVER_REFERENCE_COORDS_ECEF_M.1).powi(2)
            + (pivot_position_ecef_m.2 - ROVER_REFERENCE_COORDS_ECEF_M.2).powi(2))
        .sqrt();

        let e_i = (
            (e01_position_ecef_m.0 - ROVER_REFERENCE_COORDS_ECEF_M.0) / rho_i,
            (e01_position_ecef_m.1 - ROVER_REFERENCE_COORDS_ECEF_M.1) / rho_i,
            (e01_position_ecef_m.2 - ROVER_REFERENCE_COORDS_ECEF_M.2) / rho_i,
        );

        let e_j = (
            (pivot_position_ecef_m.0 - ROVER_REFERENCE_COORDS_ECEF_M.0) / rho_j,
            (pivot_position_ecef_m.1 - ROVER_REFERENCE_COORDS_ECEF_M.1) / rho_j,
            (pivot_position_ecef_m.2 - ROVER_REFERENCE_COORDS_ECEF_M.2) / rho_j,
        );

        assert!((dx - (e_j.0 - e_i.0)).abs() < 1E-6,);

        assert!((dy - (e_j.1 - e_i.1)).abs() < 1E-6,);

        assert!((dz - (e_j.2 - e_i.2)).abs() < 1E-6,);
    }

    #[test]
    fn rtk_cpp_matrix_contribution() {
        let earth_frame = build_earth_frame();
        let t0 = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

        let pivot_position_ecef_km = (16577.017768, -4619.539763, 24092.494804);

        let pivot_position_ecef_m = (
            pivot_position_ecef_km.0 * 1000.0,
            pivot_position_ecef_km.1 * 1000.0,
            pivot_position_ecef_km.2 * 1000.0,
        );

        let e01_position_ecef_km = (-11562.163582, 14053.114306, 23345.128269);

        let e01_position_ecef_m = (
            e01_position_ecef_km.0 * 1000.0,
            e01_position_ecef_km.1 * 1000.0,
            e01_position_ecef_km.2 * 1000.0,
        );

        let mut rover = CandidatesBuilder::build_rover_sv_at(E05, t0);

        rover.orbit = Some(Orbit::from_position(
            e01_position_ecef_km.0,
            e01_position_ecef_km.1,
            e01_position_ecef_km.2,
            t0,
            earth_frame,
        ));

        let cfg = Config::default().with_navigation_method(Method::CPP);

        let x0_y0_z0_m = Vector3::new(
            ROVER_REFERENCE_COORDS_ECEF_M.0,
            ROVER_REFERENCE_COORDS_ECEF_M.1,
            ROVER_REFERENCE_COORDS_ECEF_M.2,
        );

        let (dx, dy, dz) = rover.rtk_matrix_contribution(x0_y0_z0_m, pivot_position_ecef_m);

        let rho_i = ((e01_position_ecef_m.0 - ROVER_REFERENCE_COORDS_ECEF_M.0).powi(2)
            + (e01_position_ecef_m.1 - ROVER_REFERENCE_COORDS_ECEF_M.1).powi(2)
            + (e01_position_ecef_m.2 - ROVER_REFERENCE_COORDS_ECEF_M.2).powi(2))
        .sqrt();

        let rho_j = ((pivot_position_ecef_m.0 - ROVER_REFERENCE_COORDS_ECEF_M.0).powi(2)
            + (pivot_position_ecef_m.1 - ROVER_REFERENCE_COORDS_ECEF_M.1).powi(2)
            + (pivot_position_ecef_m.2 - ROVER_REFERENCE_COORDS_ECEF_M.2).powi(2))
        .sqrt();

        let e_i = (
            (e01_position_ecef_m.0 - ROVER_REFERENCE_COORDS_ECEF_M.0) / rho_i,
            (e01_position_ecef_m.1 - ROVER_REFERENCE_COORDS_ECEF_M.1) / rho_i,
            (e01_position_ecef_m.2 - ROVER_REFERENCE_COORDS_ECEF_M.2) / rho_i,
        );

        let e_j = (
            (pivot_position_ecef_m.0 - ROVER_REFERENCE_COORDS_ECEF_M.0) / rho_j,
            (pivot_position_ecef_m.1 - ROVER_REFERENCE_COORDS_ECEF_M.1) / rho_j,
            (pivot_position_ecef_m.2 - ROVER_REFERENCE_COORDS_ECEF_M.2) / rho_j,
        );

        assert!((dx - (e_j.0 - e_i.0)).abs() < 1E-6,);

        assert!((dy - (e_j.1 - e_i.1)).abs() < 1E-6,);

        assert!((dz - (e_j.2 - e_i.2)).abs() < 1E-6,);
    }

    #[test]
    fn rtk_ppp_matrix_contribution() {
        let earth_frame = build_earth_frame();
        let t0 = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

        let pivot_position_ecef_km = (16577.017768, -4619.539763, 24092.494804);

        let pivot_position_ecef_m = (
            pivot_position_ecef_km.0 * 1000.0,
            pivot_position_ecef_km.1 * 1000.0,
            pivot_position_ecef_km.2 * 1000.0,
        );

        let e01_position_ecef_km = (-11562.163582, 14053.114306, 23345.128269);

        let e01_position_ecef_m = (
            e01_position_ecef_km.0 * 1000.0,
            e01_position_ecef_km.1 * 1000.0,
            e01_position_ecef_km.2 * 1000.0,
        );

        let mut rover = CandidatesBuilder::build_rover_sv_at(E05, t0);

        rover.orbit = Some(Orbit::from_position(
            e01_position_ecef_km.0,
            e01_position_ecef_km.1,
            e01_position_ecef_km.2,
            t0,
            earth_frame,
        ));

        let cfg = Config::default().with_navigation_method(Method::PPP);

        let x0_y0_z0_m = Vector3::new(
            ROVER_REFERENCE_COORDS_ECEF_M.0,
            ROVER_REFERENCE_COORDS_ECEF_M.1,
            ROVER_REFERENCE_COORDS_ECEF_M.2,
        );

        let (dx, dy, dz) = rover.rtk_matrix_contribution(x0_y0_z0_m, pivot_position_ecef_m);

        let rho_i = ((e01_position_ecef_m.0 - ROVER_REFERENCE_COORDS_ECEF_M.0).powi(2)
            + (e01_position_ecef_m.1 - ROVER_REFERENCE_COORDS_ECEF_M.1).powi(2)
            + (e01_position_ecef_m.2 - ROVER_REFERENCE_COORDS_ECEF_M.2).powi(2))
        .sqrt();

        let rho_j = ((pivot_position_ecef_m.0 - ROVER_REFERENCE_COORDS_ECEF_M.0).powi(2)
            + (pivot_position_ecef_m.1 - ROVER_REFERENCE_COORDS_ECEF_M.1).powi(2)
            + (pivot_position_ecef_m.2 - ROVER_REFERENCE_COORDS_ECEF_M.2).powi(2))
        .sqrt();

        let e_i = (
            (e01_position_ecef_m.0 - ROVER_REFERENCE_COORDS_ECEF_M.0) / rho_i,
            (e01_position_ecef_m.1 - ROVER_REFERENCE_COORDS_ECEF_M.1) / rho_i,
            (e01_position_ecef_m.2 - ROVER_REFERENCE_COORDS_ECEF_M.2) / rho_i,
        );

        let e_j = (
            (pivot_position_ecef_m.0 - ROVER_REFERENCE_COORDS_ECEF_M.0) / rho_j,
            (pivot_position_ecef_m.1 - ROVER_REFERENCE_COORDS_ECEF_M.1) / rho_j,
            (pivot_position_ecef_m.2 - ROVER_REFERENCE_COORDS_ECEF_M.2) / rho_j,
        );

        assert!((dx - (e_j.0 - e_i.0)).abs() < 1E-6,);

        assert!((dy - (e_j.1 - e_i.1)).abs() < 1E-6,);

        assert!((dz - (e_j.2 - e_i.2)).abs() < 1E-6,);
    }
}
