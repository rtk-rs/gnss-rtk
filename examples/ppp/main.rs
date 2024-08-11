// ppp.rs, as opposed to the rtk.rs example, is only dedicated to 1D (autonomous) navigation,
// by opposition to RTK. Refer to the other example for that other technique.
// Because the solver supports all navigation technique, the Rust language requires us to still define the RTK sides of the API even in this very case. But since we know this is totally disabled in this example, all RTK side is tied to Null.

extern crate serde;

mod cli;
use cli::Cli;

mod source;
use source::DataSource;

mod orbit;
use orbit::Orbits;

pub mod setup;

use gnss_rtk::prelude::{
    BaseStation as RTKBaseStation, Candidate, Carrier, ClockCorrection, Config, Duration, Epoch,
    Error, InvalidationCause, IonoComponents, Method, Observation, OrbitalState,
    OrbitalStateProvider, Solver, TropoComponents, SV,
};

// This example is direct positioning (not RTK), therefore
// the BaseStation returns Null all the time (== non existant)
struct BaseStation {}

impl RTKBaseStation for BaseStation {
    fn observe(&mut self, t: Epoch, sv: SV, carrier: Carrier) -> Option<Observation> {
        None // no differential positioning
    }
}

pub fn main() {
    // The Cli is only there to help us modify the initial conditions, in CI/CD.
    let cli = Cli::new();

    let orbits = Orbits::new(); // Build the Orbit source
    println!("Orbit source created: orbits");
    let mut source = DataSource::new(); // Build Data source

    // Deployment and test setup
    let setup = cli.setup();
    let rtk_config = setup.rtk_config();
    let test_conditions = setup.test_conditions();
    let apriori = test_conditions.apriori;

    // The API is pretty straightforward
    //  [+] pass configuration setup
    //  [+] tie Orbit source
    //  [+] tie possible Base Station
    //  [+] needs mutable access ue to iteration process
    let mut solver: Solver<Orbits, BaseStation> =
        Solver::ppp(&rtk_config, apriori, orbits).expect("failed to deploy solver");

    println!("PPP example deployed");

    // Browse your data and try resolve solutions
    while let Some((epoch, candidates)) = source.next() {
        match solver.resolve(epoch, &candidates) {
            Ok((_epoch, solution)) => {
                // A solution was successfully resolved for this Epoch.
                // The position is expressed as absolute ECEF [m].
                let _position = solution.position;
                // The velocity vector is expressed as variations of absolute ECEF positions [m/s]
                let _velocity = solution.velocity;
                // Receiver offset to preset timescale
                let (_clock_offset, _timescale) = (solution.dt, solution.timescale);
                // More infos on SVs that contributed to this solution
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
            Err(Error::InvalidatedSolution(cause)) => match cause {
                InvalidationCause::FirstSolution => {
                    // The current behavior will always discard the first solution.
                    // We will always wind up here on the first iteration
                },
                _other => {
                    // Conditions are either degrading and no longer fit preset criteria,
                    // or we need more iterations to finally meet preset criteria.
                },
            },
            Err(_e) => {
                // Something went wrong, use "e" to get more info on what is wrong.
                // The most plausible cause is the lack of observations, at this point in time.
                // But that should not last for too long, otherwise something's wrong in your setup
            },
        }
    }
}
