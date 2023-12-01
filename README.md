GNSS-RTK
========

[![crates.io](https://img.shields.io/crates/v/gnss-rtk.svg)](https://crates.io/crates/gnss-rtk)
[![Rust](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml/badge.svg)](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml)
[![crates.io](https://docs.rs/gnss-rtk/badge.svg)](https://docs.rs/gnss-rtk)
[![rustc](https://img.shields.io/badge/rustc-1.64%2B-blue.svg)](https://img.shields.io/badge/rustc-1.64%2B-blue.svg)

Precise position solver :crab:

Notes on this ecosystem
=======================

We use `nalgebra` in the solving process (Matrix, Vectors..).  
We use `nyx` for ephemerides, orbital calculations, navigation filters, etc..  
`Hifitime` is at the very basis of this work and allows pretty much everything here.  
[The RINEX Wiki](https://github.com/georust/rinex/wiki) describes extensive application of this framework.  
[The RNX2CGGTTS and its Wiki](https://github.com/georust/rinex/wiki) is another interesting application of this framework.

PVT Solutions
=============

The objective is to resolve a PVT solution, implemented in the form of a `prelude::PVTSolution`,
ideally the most precise as possible, at a given _Epoch_.

Resolving a PVT requires a minimum number of SV (actual measurements)
in sight and within the predefined criterias:

- 4 SV in default mode
- 3 SV in fixed altitude mode
- 1 SV in time only mode

The predefined criterias are manually set in the configuration file, 
refer to its [own documentation](./doc/config.md). This means the criterias can be
loosened or tightened, depending on what you want to achieve.

Some constraints on the measurements may apply too, as a rule of thumb:

- `SPP` strategies will only require Pseudo Range and will resolve without Phase observations.
This makes these strategies capapble to deploy on constrained and degraged environment.
For example: old receivers, single constellation in sight.. etc..

- `PPP` strategies require not only Pseudo Range but also Phase observations
and the sampling of two different radio frequencies. No solutions will be formed
if this predicate (on top of all the others explained here) do not stand.

We support several methods (also sometimes refered to as _strategies_) which
will have the solver behave differently, due to their very different nature.   
The method is set by the `solver::Mode`.

Dilution of precisions along other meaningful information are attached to each PVT solution.

This solver will eventually be able to resolve along any known GNSS constellation,
including a mixed pool of spacecrafts (for example a combination of GAL + GPS), 
and also express the PVT solution against any supported GNSS Time system (for example GST).

Strategies and behavior
=======================

We support a couple of strategies, only advanced strategies will give the best results
(most precise solutions):

- [spp](./doc/spp.md)
- [lsqspp](./doc/lsqspp.md)
- [ppp](./doc/ppp.md)

As a rule of thumb: 

- Non recursive strategies (like [spp](./doc/spp.md)) will generate
a solution as along as enough signal measurements were provided.
- Any physical phenomena can be accounted for in this framework,
on any strategy, even though some will not be meaningful depending on the
configuration setup.  
- The GDOP criteria can still be used to reject PVT solutions of non recursive strategies
- Only recursive strategies will take truly use other solver options of the configuration file.

The current API allows changing the strategy from one Epoch to another and this framework will most likely behave fine.   
But note that this has not been tested and is not the typical use of such tool.

## Atmospherical and Environmental biases

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
