GNSS-RTK
========

[![Rust](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml/badge.svg)](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml)
[![Rust](https://github.com/rtk-rs/gnss-rtk/actions/workflows/daily.yml/badge.svg)](https://github.com/rtk-rs/gnss-rtk/actions/workflows/daily.yml)
[![crates.io](https://img.shields.io/crates/v/gnss-rtk.svg)](https://crates.io/crates/gnss-rtk)
[![crates.io](https://docs.rs/gnss-rtk/badge.svg)](https://docs.rs/gnss-rtk)

[![License](https://img.shields.io/badge/license-MPL_2.0-orange?style=for-the-badge&logo=mozilla)](https://github.com/rtk-rs/gnss-rtk/blob/main/LICENSE)

The `GNSS-RTK` library provides a Position Velocity Time (PVT) solutions solver,
with an efficient abstract interface, suitable for almost any type of applications,
whether it is real-time or post-processing oriented. 

The objective of this library is to provide a powerful and accurate ecosystem
to answer this demanding requirement. Current, the library requires the `std`
library though. 

Licensing
=========

This library is part of the [RTK-rs framework](https://github.com/rtk-rs) which
is delivered under the [Mozilla V2 Public](https://www.mozilla.org/en-US/MPL/2.0) license.

Solver API
==========

This API demands you provide the minimum required to obtain valid PVT solutions.
One key element is that we are physics driven, not data format. You may deploy this solver
with your own custom data source, we are not tied to a specific format (CSV, RINEX...).

Because Navigation is a complex task, providing an abstract interface for the end user is not easy.
Therefore, we rely on somewhat "advanced" interfacing, mainly:

* One function pointer to provide the state of your SV
* One function pointer to provide possible environmental perturbations

GNSS-RTK Illustrations
======================

If you are inquiring for more details and results, you should take a look
at our [`rinex-cli`](https://github.com/rtk-rs/rinex-cli) Examples & Demos,
that illustrate what this solver has to offer.

Context
=======

This library is oriented towards precise navigation on Earth ground (whether it is
static or roaming navigation). It currently implements all the specificy that it internally means,
and it is not designed to operate on other planets. Although this modification would not require
a lot of effort, it is not on our scheduled todo list. If you are interested in seeing this happen,
feel free to join the effort online.

Configuration simplicity
========================

Although the taks is challenging, one objective is to keep things simple.   
The main task to do so is to provide [a consice and comprehensible Parametrization interface](./documentation/Config.md)

Illustrate, Teach & Learn
=========================

Because it is easy to operate and tweak this solver, it might appear as a good
"teaching" tool. In other words, it is easy to use this toolkit to illustrate one specific
aspect of GNSS navigation.

Signal and Measurement Flexibility
==================================

One objective is to make this solver flexible and capable to adapt to most exploitation scenarios.  
Answering 100% of usecases is impossible, for the simple reason that GNSS applications
cover a very wide spectrum and vary a lot. Yet we provide central key elements that are very interesting:

* Possibility to navigate using a single signal. Obviously, the samples you provide
should match your navigation technique at all times. Yet, we have several navigation techniques,
some are compatible with single frequency observation. This makes our solver compatible with
degraded or low-cost environments

* Military frequencies: our library treats the L1 (main signal) as either the L1 or L6
frequencies. That means you can replace L1 with L6 naturally, if you are able to decode and
sample that frequency. It also means you can operate the solver in single frequency L6 mode.

* Signals _almost_ do not matter. The current version of the library prioritizes signals this way:
  - The L1 is the main frequency and is always required. L6 might replace L1 in case it is missing
  - CPP and PPP navigation techniques require a subsidary signal, which needs to be either L2 or L5,
  they are not prioritized and both may work. That means if L2 is missing from time to time, we can use L5
  for that purpose
  - If you're in position to provide the main signal, and two subsidary frequencies (L1+L2+L5) we take
  advantage of that. In other words, we are compatible with modern precise GNSS receivers and what they have to offer

PPP / RTK
=========

This library is not particularly oriented towards PPP nor RTK, which are the most "popular" techniques
for people aiming at ultra precise results. Instead, we try to support most navigations techniques, these two just happen to be one of them.

Supported Techniques and Methods
================================

We differentiate absolute and differential navigation techniques. The differential technique (RTK) is work in progress.

Absolute or differential is selected by the deployment function. The 
[navigation technique](https://docs.rs/gnss-rtk/latest/gnss_rtk/prelude/enum.Method.html) is defined
by the configuration preset.

1. Absolute navigation
 
  - `SPP`: is a Pseudo Range navigation technique for single signal / degraded / limited setups.
  Phase range observations are disregarded, unless you intend to use the L1+C1 enhancing smoothing technique.
  You can enhance the SPP technique with an external model for environment perturbations. You can use the models we provide 
  to answer that requirement. If you do not provide an accurate model (which is a complex thing to do), you cannot obtain
  accurate solutions. We recommend using a different technique for any setups that allow to do so.

  - `CPP`: starting at CPP, we use physical cancellation of the ionosphere impact.
  Any ionosphere model is disregarded.
  You need to provide two separate frequencies for this mode to operate.
  CPP is like SPP and purely based on Pseudo Range navigation, phase range are disregarded,
  unless you intend to use the L1+C1+L2+C2 smoothing technique.

  - `PPP`: has the same requirements and objectives, but phase observations
  are now also expected. It can be further enhanced by deploying the code smoothing technique.
  Since PPP is very heavy and requires all signals to be available, our PPP presets activate the 
  smoothing technique by default.

2. Differential navigation

  - :warning: Work in Progress

Enhanced Navigation techniques
==============================

We provide a few options to enhance the accuracy of the solutions, depending of the context and input scenario.

- When using `SPP` technique, you can provide L1 phase observations and deploy the L1+C1 code smoothing technique,
to improve the accuracy of your results. This is very interesting for degraded / low-cost use cases.

- When using the `CPP` technique, you can provide L1+L2 phase observations and deploy the L1+C1+L2+C2 code smoothing
technique, which does not really make sense, because you are providing all the necessary input to switch to PPP technique.
You should switch to `PPP` technique any time L1 + L2 phase observations are feasible

- Enhancing the `PPP` technique with code smoothing will improve the accuracy of the solutions.

Notes on Timescales
====================

* Our Rust ecosystem offers great Timescale support. This library should be able to 
operate in any supported timescale.

Surveying and Apriori Knowledge
===============================

This solver is a true surveying tool and can operate without apriori knowledge. In other words,
it is compatible with obtaining an absolute position without any knowledge at starting point.
This tool is therefore suited for the challenging task of setting up an RTK reference point.

Other options
=============

- We provide the possibility to limit the SV contributors to a conic Azimutal + Elevation region.
If you use azimutal criterias, only vehicles that are located within those angles will contribute to the solutions.

Framework
=========

GNSS-RTK includes itself within and is closely tied to the following libraries:

* [ANISE](https://github.com/nyx-space/anise) for orbital calculations
* [Nyx-space](https://github.com/nyx-space/nyx) for advanced navigation
* [Hifitime](https://github.com/nyx-space/hifitime) for timing
* [Our GNSS library](https://github.com/rtk-rs/gnss) for basic GNSS definitions
* Nalgebra for all calculations
