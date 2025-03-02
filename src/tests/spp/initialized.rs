// use crate::{
//     prelude::{Almanac, Config, Epoch, Method, Solver, EARTH_J2000},
//     tests::{init_logger, orbits::OrbitDataSet, signals::SignalSource, REFERENCE_COORDS_ECEF_M},
// };

// use log::info;

// use std::str::FromStr;

// use rinex::prelude::Rinex;
// use sp3::prelude::SP3;

// #[test]
// fn gps_l1() {
//     init_logger();

//     let almanac = Almanac::until_2035().unwrap();
//     let frame = almanac.frame_from_uid(EARTH_J2000).unwrap();

//     let mut cfg = Config::default();

//     cfg.method = Method::SPP;
//     cfg.min_sv_elev = None;
//     cfg.min_snr = None;

//     let rinex = Rinex::from_gzip_file("data/ESBC00DNK_R_20201770000_01D_30S_MO.crx.gz").unwrap();

//     let mut source = SignalSource::from_rinex_gps(&rinex);

//     let sp3 = SP3::from_gzip_file("data/GRG0MGXFIN_20201770000_01D_15M_ORB.SP3.gz").unwrap();

//     let gps_orbits =
//         OrbitDataSet::from_sp3("data/GRG0MGXFIN_20201770000_01D_15M_ORB.SP3.gz", &almanac);

//     let mut solver = Solver::new(cfg, gps_orbits, Some(REFERENCE_COORDS_ECEF_M))
//         .unwrap_or_else(|e| panic!("Failed to deploy with default setup: {}", e));

//     for t_gpst_str in ["2020-06-25T00:00:00 GPST"] {
//         let t_gpst = Epoch::from_str(t_gpst_str).unwrap();

//         let pool = source.next().unwrap();

//         // verify data correctness
//         for cd in pool.iter() {
//             assert_eq!(cd.t, t_gpst, "invalid test setup!");
//         }

//         let sv = pool.iter().map(|cd| cd.sv).collect::<Vec<_>>();
//         info!("Testing: {} ({}x SV)", t_gpst, sv.len());

//         match solver.resolve(t_gpst, &pool) {
//             Ok((t, pvt)) => {
//                 panic!("t={} | pvt: {:?}", t, pvt);
//             },
//             Err(e) => {
//                 panic!("err={}", e);
//             },
//         }
//     }
// }
