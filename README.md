GNSS-RTK
========

[![crates.io](https://img.shields.io/crates/v/gnss-rtk.svg)](https://crates.io/crates/gnss-rtk)
[![rustc](https://img.shields.io/badge/rustc-1.64%2B-blue.svg)](https://img.shields.io/badge/rustc-1.64%2B-blue.svg)
[![crates.io](https://docs.rs/gnss-rtk/badge.svg)](https://docs.rs/gnss-rtk/badge.svg)

Precise position solver, in rust.

Solving method
==============

Only a straightforward Matrix based resolution method is implemented.  
Other solutions, like Kalman filter, exist and could potentially improve performances
at the expense of more complexity.

The matrix resolution technique gives the best result for every single epoch

- there are no initialization iterations
- there is no iteration or recursive behavior

Behavior and Output 
===================

The solver will try to resolve a position for every single existing Epoch.

When working with RINEX, preprocessing operations may apply. 
If you're working with the attached "cli" application, this is done with `-P`.  
For example, if the input context is huge, a smoothing or decimation

The solver will output a SolverEstimate object on each resolved Epoch.  
Refer to this structure's documentation for more information.

Timing DOP and Position DOP are estimated and attached to every single result.

SPP 
===

The solver supports the spp strategy. This strategy is the only strategy we can deploy
on single carrier context. It is most likely the unique strategy you can deploy if you're working
with old RINEX (like GPS only V2), or single frequency RINEX data.

When using SPP : 

- you can only hope for residual errors of a few meters
- an interpolation order above 9 makes no sense
- Ionospheric delay must be considered and modeled. Refer to the related section.

If you're operating this library from the "cli" application integrated to this toolsuite,
a forced `--spp` mode exists. It is a convenient way to restrict this library to SPP solving
and compare it to PPP.

PPP
===

The solver will adapt to PPP strategy if the context is sufficient (more than one carrier).   
PPP simplifies the solving process greatly, ionospheric delay is cancelled and does not have to be taken into account.  

PPP is deployed if you're typically working with modern RINEX data.

We allow the possibility to deploy a PPP strategy without SP3 data. This is not a typical use case.
Other tools like glab or rtklib probably do not allow this.
You need to understand that in this case, you want good navigation data quality in order to reduce
the error their interpolation will introduce.

When working with PPP, we recommend the interpolation order to be set to 11 (or above).

Tropospheric Delay
==================

TODO

Ionospheric Delay
=================

TODO

RTK Configuration
=================

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
