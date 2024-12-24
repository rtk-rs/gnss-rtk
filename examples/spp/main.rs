// SPP example (pseudo range based direct positioning).
// This is simply here to demonstrate how to operate the API, and does not generate actual results.
use gnss_rtk::prelude::{
    Candidate, Carrier, Config, Epoch, Error, Frame, InvalidationCause, Method, Observation, Orbit,
    OrbitSource, Solver, EARTH_J2000, SV,
};

// Orbit source example
struct Orbits {}

impl OrbitSource for &mut Orbits {
    // For each requested "t" and "sv",
    // if we can, we should resolve the SV [Orbit].
    // If interpolation is to be used (depending on your apps), you can
    // use the interpolation order that we recommend here, or decide to ignore it.
    // If you're not in position to determine [Orbit], simply return None.
    // If None is returned for too long, this [Epoch] will eventually be dropped out,
    // and we will move on to the next
    fn next_at(&mut self, t: Epoch, _sv: SV, _fr: Frame, _order: usize) -> Option<Orbit> {
        let (x_km, y_km, z_km) = (0.0_f64, 0.0_f64, 0.0_f64);
        Some(Orbit::from_position(x_km, y_km, z_km, t, EARTH_J2000))
    }
}

// Data source example
struct MyDataSource {}

impl MyDataSource {
    // Data source example
    fn new() -> Self {
        Self {}
    }
    // The objective here is to propose enough SV observations to resolve a solution.
    // Since our example only requires PseudoRange on a single frequency
    // we will limit ourselves to that.
    fn next(&mut self) -> Option<(Epoch, Vec<Candidate>)> {
        Some((
            // This must be the Epoch of observation,
            // ie, Sampling Instant
            Epoch::default(),
            vec![
                // Create a candidate from your Pseudo Range observation
                Candidate::new(
                    // Candidate Identity
                    SV::default(),
                    // Sampling Epoch. Must be identical for this grouping of candidates
                    Epoch::default(),
                    // List of observations
                    vec![Observation {
                        carrier: Carrier::L1, // example
                        pseudo: Some(3.0E6),  // example, this is raw observation
                        // Note that if you apply a min_snr preset,
                        // we might drop candidates that do not have this info
                        snr: None, // unknown
                        phase: None,
                        doppler: None,
                        ambiguity: None,
                    }],
                ),
                // Create all as many candidates as possible.
                // It's better to have more than needed, it leaves us more possibility in the election process.
            ],
        ))
    }
}

pub fn main() {
    // Build the Orbit source
    let mut orbits = Orbits {};

    // The preset API is useful to quickly deploy depending on your application.
    // Static presets target static positioning.
    let cfg = Config::static_ppp_preset(Method::SPP); // Single Freq. Pseudo Range based

    // The API is pretty straightforward, it requires the Configuration preset to be
    // built ahead of time. The only difficulty is the design your data source and SV state provider. We interact with the SV state provider by means of a function pointer.
    let solver = Solver::new(
        &cfg,
        // We deploy without apriori knowledge.
        // The solver will initialize itself.
        None, // Tie the Orbit source
    );

    // The solver needs to be mutable, due to the iteration process.
    let mut solver = solver.unwrap();

    let mut source = MyDataSource::new();

    // Browse your data source (This is an Example)
    while let Some((epoch, candidates)) = source.next() {
        match solver.resolve(epoch, &candidates, &mut orbits) {
            Ok((_epoch, solution)) => {
                // Receiver offset to preset timescale
                let (_clock_offset, _timescale) = (solution.dt, solution.timescale);
                // More infos on SVs that contributed to this solution
                for info in solution.sv.values() {
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
