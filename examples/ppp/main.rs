// ppp.rs, as opposed to the rtk.rs example, is only dedicated to 1D (autonomous) navigation,
// by opposition to RTK. Refer to the other example for that other technique.
// Because the solver supports all navigation technique, the Rust language requires us to still define the RTK sides of the API even in this very case. But since we know this is totally disabled in this example, all RTK side is tied to Null.

#[macro_use]
extern crate log;
extern crate serde;

use env_logger::{Builder, Target};

mod cli;
use cli::Cli;

mod source;
use source::DataSource;

mod orbit;
use orbit::Orbits;

pub mod setup;

use gnss_rtk::prelude::{
    BaseStation as RTKBaseStation, Carrier, Epoch, Error, InvalidationCause, Observation, Solver,
    SV,
};

// This example is direct positioning (not RTK), therefore
// the BaseStation returns Null all the time (== non existant)
struct BaseStation {}

impl RTKBaseStation for BaseStation {
    fn observe(&mut self, _: Epoch, _: SV, _: Carrier) -> Option<Observation> {
        None // no differential positioning
    }
}

pub fn main() {
    // Take advantage of generated logs
    let mut builder = Builder::from_default_env();
    builder
        .target(Target::Stdout)
        .format_timestamp_secs()
        .format_module_path(false)
        .init();

    // Cli helps customizing the initial conditions
    let cli = Cli::new();

    // Deployment setup
    let setup = cli.setup();
    let rtk_config = setup.rtk_config();
    let test_conditions = setup.test_conditions();
    let test_validation = setup.test_validation();
    let apriori = test_conditions.apriori;

    if let Some(ref apriori) = apriori {
        info!("Initial apriori: {:?}", apriori);
    } else {
        info!("Survey mode: no apriori knowledge");
    }

    info!("{:#?}", rtk_config);

    // Tests
    let mut errors = 0;
    let mut iterations = 0;
    let mut nb_solutions = 0;

    // Deploy
    let orbits = Orbits::new(); // Build the Orbit source
    let mut source = DataSource::new(test_conditions.max_candidates); // Build Data source

    // API is simple and straightforward
    //  [+] pass configuration setup
    //  [+] tie Orbit source
    //  [+] tie possible Base Station
    //  [+] (iterative) process requires mutable access
    let mut solver: Solver<Orbits, BaseStation> =
        Solver::ppp(&rtk_config, apriori, orbits).expect("failed to deploy solver");

    info!("PPP example deployed");

    // Browse data source and try solving
    while let Some((epoch, candidates)) = source.next() {
        iterations += 1;

        match solver.resolve(epoch, &candidates) {
            Ok((_epoch, solution)) => {
                // Success.
                nb_solutions += 1;
                // Position is always absolute, expressed in ECEF frame in [m].
                // It is sometimes convenient to display those in [km]
                let pos = solution.position;
                let (x_km, y_km, z_km) = (pos[0] / 1.0E3, pos[1] / 1.0E3, pos[2] / 1.0E3);
                info!(
                    "{}: new solution x={}km, y={}km, z={}km",
                    epoch, x_km, y_km, z_km
                );

                // Velocity is always resolved as well,
                // we discard the first valid solution for which we would not be
                // able to attach velocity.
                // Velocity is instantaneous, always absolute, in same frame [m]
                let vel = solution.velocity;
                let (vel_x, vel_y, vel_z) = (vel[0], vel[1], vel[2]);
                info!(
                    "{}: velocity x={}m/s, y={}m/s, z={}m/s",
                    epoch, vel_x, vel_y, vel_z
                );

                // Local clock state
                let (dt, timescale) = (solution.dt, solution.timescale);
                info!("{}: clock offset {} [{}]", epoch, dt, timescale);

                // Solutions contributor
                for info in solution.sv.values() {
                    // attitude
                    let (_el, _az) = (info.azimuth, info.elevation);
                    // Modeled (in this example) or simply copied ionosphere bias
                    // impacting selected signal from this spacecraft
                    let _bias_m = info.iono_bias;
                    // Modeled (in this example) or simply copied troposphere bias
                    // impacting selected signal from this spacecraft
                    let _bias_m = info.tropo_bias;
                    // Dilution of Precision informs on geometric performances
                    let (_tdop, _gdop, _pdop) = (solution.tdop, solution.gdop, solution.pdop);
                    // Determine the Vertical DoP for these lat,lon coordinates
                    let (lat_ddeg, lon_ddeg) = (45.0, 13.0);
                    let _vdop = solution.vdop(lat_ddeg, lon_ddeg);
                }
            },
            Err(e) => {
                errors += 1;
                println!("{}: error: {} ({})", epoch, e, errors);
                match e {
                    Error::NotEnoughCandidates => {},
                    Error::NotEnoughMatchingPreFit => {},
                    Error::NotEnoughMatchingPostFit => {},
                    Error::MatrixError => {},
                    Error::NavigationError => {},
                    Error::MatrixInversionError => {},
                    Error::FirstGuess => {},
                    Error::TimeIsNan => {},
                    Error::MissingPseudoRange => {},
                    Error::PseudoRangeCombination => {},
                    Error::PhaseRangeCombination => {},
                    Error::UnresolvedState => {},
                    Error::UnresolvedAmbiguity => {},
                    Error::PhysicalNonSenseRxPriorTx | Error::PhysicalNonSenseRxTooLate => {},
                    Error::BancroftError => {},
                    Error::InvalidStrategy => panic!("invalid strategy! should not happen here!"),
                    Error::BancroftImaginarySolution => {},
                    Error::EarthFrame => panic!("anise:: frame determination error"),
                    Error::Almanac(e) => panic!("anise:: almanac retrieval error: {}", e),
                    Error::Physics(e) => panic!("anise:: physical error: {}", e),
                    Error::InvalidatedSolution(cause) => match cause {
                        InvalidationCause::FirstSolution => {
                            // The current behavior will always discard the first solution.
                            // We will always wind up here on the first iteration
                            println!(
                                "{}: first solution invalidated, solutions are pending",
                                epoch
                            );
                        },
                        _other => {
                            // Conditions are either degrading and no longer fit preset criteria,
                            // or we need more iterations to finally meet preset criteria.
                        },
                    },
                }
            },
        }

        if nb_solutions == 0 {
            if iterations >= test_validation.max_init_iterations {
                panic!(
                    "TEST FAILED: consumed {} epochs without producing anything.",
                    iterations
                );
            }
        }
    }
}
