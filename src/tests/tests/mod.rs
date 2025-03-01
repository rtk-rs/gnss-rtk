use crate::prelude::*;

mod bancroft;
mod data;
mod pseudo_range;
mod solver;

//pub mod cpp;
//pub mod ppp;
pub mod spp;

use data::interp::interp_data;

fn fake_interpolator(t: Epoch, sv: SV, _size: usize) -> Option<InterpolationResult> {
    let database = interp_data();

    let state = database
        .iter()
        .filter(|k| k.1 == sv)
        .min_by_key(|k| (k.0 - t).abs())?;

    Some(state.2)
}

struct SolverInput {
    t_rx: Epoch,
    pool: Vec<Candidate>,
    iono_bias: IonosphereBias,
    tropo_bias: TroposphereBias,
}

struct Tester {
    pub absolute: bool,
    pub kinematic: bool,
    pub max_velocity: f64,
    pub max_gdop: f64,
    pub max_tdop: f64,
    pub timescale: TimeScale,
}

impl Tester {
    pub fn run(&self, _cfg: &Config, solutions: Vec<PVTSolution>) {
        for sol in solutions {
            let (_x, _y, _z) = (sol.position[0], sol.position[1], sol.position[2]);
            assert!(sol.gdop.abs() < self.max_gdop, "gdop limit exceeded");
            assert!(sol.tdop.abs() < self.max_tdop, "tdop limit exceeded");
            assert_eq!(
                sol.timescale, self.timescale,
                "solution expressed in wrong timescale"
            );
            if self.absolute {}
            let (_v_x, _v_y, _v_z) = (sol.velocity[0], sol.velocity[1], sol.velocity[2]);
            if self.kinematic {}
        }
    }
}
