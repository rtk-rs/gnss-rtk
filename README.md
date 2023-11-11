GNSS-RTK
========

[![crates.io](https://img.shields.io/crates/v/gnss-rtk.svg)](https://crates.io/crates/gnss-rtk)
[![Rust](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml/badge.svg)](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml)
[![crates.io](https://docs.rs/gnss-rtk/badge.svg)](https://docs.rs/gnss-rtk/badge.svg)
[![rustc](https://img.shields.io/badge/rustc-1.64%2B-blue.svg)](https://img.shields.io/badge/rustc-1.64%2B-blue.svg)

Precise position solver :crab:

Solving method
==============

The solver uses linear algebra implemented in the `nalgebra` library,
we do not use a Kalman filter.
This straightforward technique does not require initialization cycles and 
we can form a Navigation Solution for every input, as long as all criterias were met.

Some criterias are fixed by physics, others are customized and adjusted in the Solver Configuration.

The minimum requirements to form a solution :

- at least 4 SV candidates passed all customized requirements
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

## RINEX Post Processing

If you're working with RINEX data, the [rinex-cli application](https://github.com/georust/rinex) is there
to post process RINEX contexts and resolve a Precise Position from it. 

## CGGTTS and Advanced Timing

A [rnx2cggtts application](https://github.com/georust/rinex) is built to operate this solver differently
in an operation dedicated to high precision time transfer.
