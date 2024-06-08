// SPP example (pseudo range based direct positioning).
// This is simply here to demonstrate how to operate the API, and does not generate actual results.
use gnss_rtk::prelude::{
    Candidate, Carrier, Config, Duration, Epoch, Error, InterpolationResult, IonosphereBias,
    Method, PseudoRange, Solver, TroposphereBias, SV,
};

// Define your SV position provider.
fn position_provider(t: Epoch, sv: SV, order: usize) -> Option<InterpolationResult> {
    // For each requested "t" and "sv",
    // you should design an InterpolationResult,
    // ideally using recommended "order" in case interpolation is involved in your workflow,
    // For example by browsing a database.
    // When you're not in position to provide such information, simply return None.
    // If the minimum required of SV is not gathered at this point in time,
    // no solutions will be generated for this epoch, and the solver will move on
    // to the next.

    // dummy example
    let x = 0.0_f64;
    let y = 0.0_f64;
    let z = 0.0_f64;
    Some(InterpolationResult::from_position((x, y, z)))
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
                    // For each Candidate, you must provide the ongoing clock correction we should apply
                    Duration::default(),
                    // If you know the total group day for this Candidate, specify it here
                    None,
                    // List of Pseudo Range observations, we only need one in this scenario
                    vec![PseudoRange {
                        carrier: Carrier::L1, // example
                        value: 3.0E6,         // example, this is raw observation
                        // Note that if you apply a min_snr preset,
                        // we might drop candidates that do not have this info
                        snr: None, // unknown
                    }],
                    // List of Phase Range observations: not needed in this scenario
                    vec![],
                ),
                // Create all as many candidates as possible.
                // It's better to have more than needed, it leaves us more possibility in the election process.
            ],
        ))
    }
}

pub fn main() {
    // The preset API is useful to quickly deploy depending on your application.
    // Static presets target static positioning.
    let cfg = Config::static_preset(Method::SPP); // Single Freq. Pseudo Range based

    // The API is pretty straightforward, it requires the Configuration preset to be
    // built ahead of time. The only difficulty is the design your data source and SV state provider. We interact with the SV state provider by means of a function pointer.
    let solver = Solver::new(
        &cfg,
        // We deploy without apriori knowledge.
        // The solver will initialize itself.
        None,
        // connect the position provider
        |t, sv, order| position_provider(t, sv, order),
    );

    // The solver needs to be mutable, due to the iteration process.
    let mut solver = solver.unwrap();

    let mut source = MyDataSource::new();

    // Browse your data source (This is an Example)
    while let Some((epoch, candidates)) = source.next() {
        // External bias sources are not very well supported yet.
        // We recommend you use the internal modeling.
        // This is done by specifying you simply have no knowledge
        // of the external biases:
        let ionod = IonosphereBias::default(); // Unknown
        let tropod = TroposphereBias::default(); // Unknown

        match solver.resolve(epoch, &candidates, &ionod, &tropod) {
            Ok((epoch, solution)) => {
                // A solution was successfully resolved for this Epoch.
                // The position is expressed as absolute ECEF [m].
                let position = solution.position;
                // The velocity vector is expressed as variations of absolute ECEF positions [m/s]
                let velocity = solution.velocity;
                // Receiver offset to preset timescale
                let (clock_offset, timescale) = (solution.dt, solution.timescale);
                // More infos on SVs that contributed to this solution
                for (sv, info) in &solution.sv {
                    // attitude
                    let (el, az) = (info.azimuth, info.elevation);
                    // Modeled (in this example) or simply copied ionosphere bias
                    // impacting selected signal from this spacecraft
                    let bias_m = info.iono_bias;
                    // Modeled (in this example) or simply copied troposphere bias
                    // impacting selected signal from this spacecraft
                    let bias_m = info.tropo_bias;
                    // Dilution of Precision informs on geometric performances
                    let (tdop, gdop, pdop) = (solution.tdop, solution.gdop, solution.pdop);
                    // Determine the Vertical DoP for these lat,lon coordinates
                    let (lat_ddeg, lon_ddeg) = (45.0, 13.0);
                    let vdop = solution.vdop(lat_ddeg, lon_ddeg);
                }
            },
            Err(Error::InvalidatedSolution) => {
                // The current behavior will always discard the first solution,
                // so it's OK to have one first cycle here.
                // Otherwise, it means the conditions are degrading and
                // do not fit the preset criteria.
                // Or the preset criteria are stringent and we need more iterations
                // to achieve one solution.
            },
            Err(e) => {
                // Something went wrong, use "e" to get more info on what is wrong.
                // The most plausible cause is the lack of observations, at this point in time.
                // But that should not last for too long, otherwise something's wrong in your setup
            },
        }
    }
}
