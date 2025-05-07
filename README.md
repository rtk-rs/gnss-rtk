GNSS-RTK
========

[![Rust](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml/badge.svg)](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml)
[![Rust](https://github.com/rtk-rs/gnss-rtk/actions/workflows/daily.yml/badge.svg)](https://github.com/rtk-rs/gnss-rtk/actions/workflows/daily.yml)
[![crates.io](https://img.shields.io/crates/v/gnss-rtk.svg)](https://crates.io/crates/gnss-rtk)
[![crates.io](https://docs.rs/gnss-rtk/badge.svg)](https://docs.rs/gnss-rtk)

[![MRSV](https://img.shields.io/badge/MSRV-1.82.0-orange?style=for-the-badge)](https://github.com/rust-lang/rust/releases/tag/1.82.0)
[![License](https://img.shields.io/badge/license-MPL_2.0-orange?style=for-the-badge&logo=mozilla)](https://github.com/rtk-rs/gnss-rtk/blob/main/LICENSE)

The `GNSS-RTK` library provides a Position Velocity Time (PVT) solutions solver,
with an efficient abstract interface, suitable for almost any type of applications,
whether it is real-time or post-processing oriented. 

The objective of this library is to provide a powerful API that is easy to operate
and gives correct results at at all times. Due to the challenge in the task,
the library requires `std` support and it is not scheduled to make a `no-std` version of this library.

Licensing
=========

This library is part of the [RTK-rs framework](https://github.com/rtk-rs) which
is delivered under the [Mozilla V2 Public](https://www.mozilla.org/en-US/MPL/2.0) license.

Solver API
==========

This API demands you provide the minimum required to obtain valid PVT solutions.
One key element is that we are physics driven and are not tied to a specific data format (CSV, RINEX..). 
You may deploy this solver with your own data source.

Because Navigation is a complex task, providing an abstract interface for the end user is not easy.
Therefore, we rely on somewhat "advanced" interfacing, mainly:

* One function pointer to provide the state of the observed SV (also referred to as orbital source)
* One function pointer to provide possible environmental perturbations
* One function pointer to collect the latest time corrections

Deployment
==========

:warning: ANISE requires internet access either

1. at built time, when built with `embed_ephem` option 
2. at first deployment ever, when using low precision models and built without `embed-ephem` option
3. regularly, when using highest precision models, regardless of the compilation options.

This library provides an `embed_ephem` compilation option to reduce the requirements on low precision systems.

GNSS-RTK Illustrations
======================

If you are inquiring for more details and results, you should take a look
at our [`rinex-cli`](https://github.com/rtk-rs/rinex-cli) Examples & Demos,
that illustrate what this solver has to offer.

Context
=======

This library is oriented towards precise navigation on Earth ground (whether it is
static or roaming navigation does not impact the process at first). We implement all the
requirements to do so on planet Earth, not other planets.
Although this modification would not require a lot of effort, it is not planed as of today
to make `gnss_rtk` generic about the rover planet. If you are interested in seeing this happen,
feel free to join the effort online.

Configuration simplicity
========================

Although the taks is challenging, one objective is to keep things simple.   
To do so, we rely on the `serde` ecosystem and are able to provide [a consice and comprehensible Parametrization interface](./documentation/Config.md)

Teaching & learning toolkit
===========================

Because it is easy to operate and tweak `gnss_rtk`, it should be a good candidate
for teaching, learning or prototyping. 

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

Surveying and Apriori Knowledge
===============================

This solver is a true surveying tool and can operate without apriori knowledge. In other words,
it is compatible with obtaining an absolute position without any knowledge at starting point.
This tool is therefore suited for the challenging task of setting up an RTK reference point.

Signal and Measurement Flexibility
==================================

One objective is to make this solver flexible and capable to adapt to most exploitation scenarios.  
Answering 100% of usecases is impossible, for the simple reason that GNSS applications
cover a very wide spectrum and vary a lot. Yet we provide central key elements that are very interesting:

* Possibility to navigate using a single signal. Obviously, the samples you provide
should match your navigation technique at all times. Yet, we have several navigation techniques,
some are compatible with single frequency observation. This makes our solver compatible with
degraded or low-cost environments

* Military codes: our library only cares about frequencies.
Whether you arrive from a decoded military or civilian signal does not matter.
L1 is treated as main reference as always.

* High precision frequencies: E6 (Galileo) and LEX (QZSS) are both known

* :warning: This library requires L1 frequency to be present for advanced CPP or PPP techniques
(as reference signal). We allow L2 or L5 as subsidary signal, without prioritizing them.

Constellations, Timecales & Absolute time
=========================================

Like many other topics, `gnss_rtk` tries to be flexible and convenient to operate
regarding the Constellations and Timescales it supports. Rust comes with
a great Timescale support, thanks to the `Hifitime` library. Yet timescales are one thing,
solving "absolute time" is another.

Like signals exploitation, `gnss_rtk` offers a high level of flexibility for the timecales and constellations
you can use:

- you can navigate with a single constellation in sight
- you can mix constellations, for a modern and advanced setup
- you can navigate under changing conditions
- you can observe and express your signals in a different timescale than the prefered timescale

In order to resolve absolute time at all times (...), your time correction source needs to implement the `Time` trait.  
For simplicity, you should always implement it and provide the latest corrections available.  
Although, it is true that, if you navigate in GPS Only and Prefered GPST, the implementation is not needed and the absolute time will remain correct.

`gnss_rtk` is smart enough to combine your time corrections to obtain the one needed. In other words, you don't have to provide
the exactly needed time correction, if you provide many, we can recombine them to obtain the one we need. 

*In any case*:

1. Any SV for which the time correction is not available or outdated will not contribute to the solution.
So we garantee correct absolute time, at all times

2. We consider that any time correction may apply for a whole week. Which is physically correct and the maximal value.
In most cases, you should provide regular updates (at least daily) to obtain better results

3. You should provide (through the `Time` trait implementation) the latest time correction that may apply at that instant correctly.

UTC Timescale support
=====================

Like other timescales, UTC (Universal Time Coordinates) is supported. It is possible to express your temporal solutions
in UTC. 

Timescale & signal sampling
===========================

`gnss_rtk` allows you to express your signal observations (data points) in a different timescale than the prefered timescale.
This should offer yet another level of flexibility. 

For example, you can observe in `UTC` timescale and resolve `GPST`, but for that, your `Time` source needs to provide directly or help us
resolve `|UTC-GPST|`.

Custom Azimutal condition
=========================

It is possible to restrict the contributors to a conic Azimutal + Elevation area (angle ranges),
using the configuration preset.

Framework
=========

GNSS-RTK includes itself within and is closely tied to the following libraries:

* [ANISE](https://github.com/nyx-space/anise) for orbital calculations
* [Nyx-space](https://github.com/nyx-space/nyx) for advanced navigation
* [Hifitime](https://github.com/nyx-space/hifitime) for timing
* [Our GNSS library](https://github.com/rtk-rs/gnss) for basic GNSS definitions
* Nalgebra for all calculations
