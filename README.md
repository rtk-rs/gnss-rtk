GNSS-RTK
========

[![crates.io](https://img.shields.io/crates/v/gnss-rtk.svg)](https://crates.io/crates/gnss-rtk)
[![Rust](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml/badge.svg)](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml)
[![crates.io](https://docs.rs/gnss-rtk/badge.svg)](https://docs.rs/gnss-rtk)
[![rustc](https://img.shields.io/badge/rustc-1.64%2B-blue.svg)](https://img.shields.io/badge/rustc-1.64%2B-blue.svg)

Precise positioning calculations, in Rust

Notes on this ecosystem
=======================

- `Nalgebra` is used for anything related to linear algebra
- `Nyx-space` provides required features in orbital calculations
- `Hifitime` provides timing and timescale definitions
- `Gnss-rs` provides basic GNSS definitions

[The RINEX Wiki](https://github.com/georust/rinex/wiki) describes extensive application of this framework (at a high level).  

Solver
======

We only support 1D direct positioning, a subsidary data source required for RTK is not supported and will be developed very soon.   
That means the only differential technique you can currently used is SBAS (Ground/Space differential technique). 

PVT Solutions
=============

The objective is to resolve PVT solutions, which requires a minimum of SV in sights:
- 4 SV required, in default mode
- 3 SV required, in fixed altitude mode
- 1 SV required, in time only mode

The preset criteria are manually set in the configuration file (or config script).
At the moment, refer to the RINEX Wiki or RINEX scripts database, for meaningful examples.

Depending on the preset configuration, other requirements will apply to the previous list, most importantly:

- `CPP` strategy will required pseudo range observation on a secondary frequency
- `PPP` strategy will required pseudo range and phase observations on two frequencies
- SNR, Elevation and Azimuth mask will require to gather the required amount of SV within those conditions

Each PVT solution contains the Dilution of Precision (DOP) and other meaningful information, like which SV
contributed to the solution. We have the capability to express the clock offset in all supported Timescale.

Strategy and other settings
===========================

The solver's behavior and output results are highly dependent on the [selected strategy](https://docs.rs/gnss-rtk/latest/gnss_rtk/prelude/enum.Method.html).

Advanced strategies require deeper knowledge and most likely more tuning of the solver configuration. 
The Rust/JSON infrastructure is powerful enough though, to allow to only define the [config parts](https://docs.rs/gnss-rtk/latest/gnss_rtk/prelude/struct.Config.html)
you are interested in: others will simply default.

[PVTSolutionType](https://docs.rs/gnss-rtk/latest/gnss_rtk/prelude/enum.PVTSolutionType.html) defines the type of solutions we want to form and therefore,
the minimum amount of SV we need to gather. As previously stated, other criteria like `min_sv_elev` or `max_sv_azim` will restrict the condition on those vehicles that they must fit in
to be considered. 

When `fixed_altitude` is set to a certain value, the quantity of required SV is reduced by 1.  
This has no impact when `PVTSolutionType` is set to `TimeOnly`.

The `SolverOpts` configuration gives more advanced options on how to tweak the solver. Briefly, this allows to

- select one of our Navigation Filters, like Kalman filter or LSQ
- define the PVT solutions confirmation criteria

`Modeling` defines what physical and environmental phenomena we compensate for.   
Modeling are closely tied to the selected solver strategy. For example, 
models that impact at the centimetric level like the sunlight rate, are not meaningful in strategies other than advanced PPP.
On the other hand, you will not reach metric solutions, whatever the strategy might be, if a minimum of physical phenomena are not accounted for.

Atmosphere and Environmental biases
===================================

This solver is always capable of modeling all conditions and form a solution.   
It is important to understand how our API is designed and operate it the best you can to get the best results.

Troposphere Delay
==================

Troposphere bias always needs to be estimated.  
By default, the solver will use a model implemented in the [model::tropo API].  
If you're in position to determine yourself the Tropospherical Delay components (TropoComponents structure)
at the required latitude and _Epoch_, you are highly encouraged to provide your data.
To do so, we use a function pointer that can serve as a TropoComponents source.
TropoComponents evaluation parameters (function pointer arguments) should be :

- _Epoch_
- altitude (above sea level) expressed in meters
- latitude expressed as decimal degrees

For _Epochs_ where the data source is not capable to supply data, that is not a problem, we will rely on the internal model.

Example of handmade TropoComponents provider :  

TODO

:warning: When using the internal model, we recommend applying a >= 5° elevation mask to the candidates.

Ionosphere Delay
================

TODO

A priori position
=================

The solver can be armed with a priori knowledge (rough idea of the final position),
or can operate in complete autonomy. In this case, the solver will initialize itself
very accurately, this requires one extra step.

Solver deployment
=================

The solver API is pretty straightforward, it requires the Configuration preset to be
built ahead of time. The only difficulty is you need to design your source of SV position provider, and this is currently performed by means of a function pointer.

```rust
use gnss_rtk::prelude::{
    Config, Method, Solver,
    Carrier,
    InterpolationResult, Epoch, SV,
    IonosphereBias, TroposphereBias,
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
    Some(Interpolation::from_position((x, y, z)))
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
    fn next(&mut self) -> Option<(epoch, Vec<Candidate>)> {
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
                    vec![
                        PseudoRange {
                            carrier: Carrier::L1, // example
                            value: 3.0E6, // example, this is raw observation
                            // Note that if you apply a min_snr preset,
                            // we might drop candidates that do not have this info
                            snr: None, // unknown
                        },
                    ],
                    // List of Phase Range observations: not needed in this scenario
                    vec![],
                ),
                // Create all as many candidates as possible.
                // It's better to have more than needed, it leaves us more possibility in the election process.
            ],
        ))
    }
}

# The preset API is useful to quickly deploy depending on your application.
# Static presets target static positioning.
let cfg = Config::static_preset(Method::SPP); // Single Freq. Pseudo Range based

let solver = Solver::new(
    &cfg, 
    // Deploy without apriori knowledge (auto initialization)
    None, 
    // connect the position provider
    |t, sv, order| position_provider(t, sv, order)
);

let mut source = MyDataSource::new();

// Browse your data source (This is an Example) 
while let Some((epoch, candidates)) = source.next() {
    // External bias sources are not very well supported yet.
    // We recommend you use the internal modeling.
    // This is done by specifying you simply have no knowledge
    // of the external biases:
    let ionod = Option::<IonosphereBias>::None;
    let tropod = Option::<TroposphereBias>::None;

    match solver.resolve(epoch, &candidates, ionod, tropod) {
        Ok((epoch, solution)) => {
            // A solution was successfully resolved for this Epoch.
            // The position is expressed as absolute ECEF [m].
            let position = solution.position;
            // The velocity vector is expressed as variations of absolute ECEF positions [m/s]
            let velocity = solution.velocity;
            // Receiver offset to preset timescale
            let (clock_offset, timescale) = (solution.dt, solution.timescale);
            // More infos on SVs that contributed to this solution
            for (sv, info) in solution.sv {
                // attitude
                let (el, az) = (info.azimuth, info.elevation);
                // Modeled (in this example) or simply copied ionosphere bias
                // impacting selected signal from this spacecraft
                let bias_m = info.iono_bias;
                // Modeled (in this example) or simply copied troposphere bias
                // impacting selected signal from this spacecraft
                let bias_m = info.tropo_bias;
                // Dilution of Precision informs on geometric performances
                let (tdop, gdop, pdop) = (
                    solution.tdop,
                    solution.gdop,
                    solution.pdop,
                );
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
```
