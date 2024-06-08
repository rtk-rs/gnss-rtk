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

Getting started
===============

Refer to our example applications to understand how to operate our API in more detail.
