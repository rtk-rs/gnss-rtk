GNSS-RTK
========

[![crates.io](https://img.shields.io/crates/v/gnss-rtk.svg)](https://crates.io/crates/gnss-rtk)
[![Rust](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml/badge.svg)](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml)
[![crates.io](https://docs.rs/gnss-rtk/badge.svg)](https://docs.rs/gnss-rtk)
[![rustc](https://img.shields.io/badge/rustc-1.64%2B-blue.svg)](https://img.shields.io/badge/rustc-1.64%2B-blue.svg)

Precise position solver :crab:

Solving method
==============

The solver uses linear algebra implemented in the `nalgebra` library.

The current solver is straightforward and does not require initialization cycles.  
We can resolve a PVT for every input as long as the criterias for the current setup are met.

Some criterias are fixed by physics, others are customized and adjusted in the `Cfg` structure.

The minimum requirements to form a solution :

- the minimal number of SV within the criterias are in sight: 
  - 4 SV in default mode
  - 3 SV in fixed altitude mode
  - 1 SV in time only mode
- user was able to provide observations that satisfy the resolution strategy
- user was able to interpolate the SV state vector at the required _Epoch_

PVT Solutions
=============

The solver tries to resolve a Position Velocity Time (PVT) solution of the receiver.  
Currently the Velocity term is not evaluated, therefore we only output Positions and Time errors.
Dilution of precisions are estimated and attached to each solution.

Single Point Position (SPP)
===========================

The solver supports the SPP resolution method.

SPP can be deployed in degraded conditions where only one carrier signal and/or a unique constellation is in sight.
In such conditions:

- you can only hope for a precision of a few meters.    
- an interpolation order of 9 makes sense, going above will increase the computation load without any benefit.
- the solver will have to estimate the total bias due to Ionospheric delay. If you can define
these components accurately yourself, you are encouranged to do it (see related API section and following paragraphs)

## Atmospherical and Environmental conditions modeling

This solver is always capable of modelizing all conditions and form a solution.   
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

Solver Configuration
====================

Refer to the online documentation (API) for detailed information on all existing fields.
The RTKConfiguration structure, describes all configuration and customization
the solver supports. 

It is important to understand how, when and what to customize depending on your goals.

When working with the "cli" application, you can provide an RTKConfiguration
in the form of JSON, with `--rtk-cfg`.

RINEX Post Processing
=====================

The [rinex-cli application](https://github.com/georust/rinex) is there
to post process RINEX data and resolve PVT solutions from it.

CGGTTS: PVT and advanced Timing
===============================

The [rnx2cggtts application](https://github.com/georust/rinex) uses this
solver to estimate local clock performances and allow the comparison
of two remote clocks.
