GNSS-RTK
========

[![Rust](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml/badge.svg)](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml)
[![Rust](https://github.com/rtk-rs/gnss-rtk/actions/workflows/daily.yml/badge.svg)](https://github.com/rtk-rs/gnss-rtk/actions/workflows/daily.yml)
[![crates.io](https://img.shields.io/crates/v/gnss-rtk.svg)](https://crates.io/crates/gnss-rtk)
[![crates.io](https://docs.rs/gnss-rtk/badge.svg)](https://docs.rs/gnss-rtk)

[![MRSV](https://img.shields.io/badge/MSRV-1.81.0-orange?style=for-the-badge)](https://github.com/rust-lang/rust/releases/tag/1.81.0)
[![License](https://img.shields.io/badge/license-MPL_2.0-orange?style=for-the-badge&logo=mozilla)](https://github.com/rtk-rs/gnss-rtk/blob/main/LICENSE)

The `GNSS-RTK` library provides several Position Velocity Time (PVT) solution solvers,
with abstract and flexible interfaces, which makes it suitable for most navigation applications,
whether they are real-time or post-processing oriented.

<div align="center">
    <p>
        `PPP` versus a professional geodetic marker
    </p>
    <a href=https://github.com/rtk-rs/rinex-cli/blob/main/plots/front-page/map.png>
        <img src=https://github.com/rtk-rs/rinex-cli/blob/main/plots/front-page/map.png alt="Plot">
    </a>
</div>

<div align="center">
    <p>
        Errors from the geodetic marker (CPP, Galileo E1+E5)
    </p>
    <a href=https://github.com/rtk-rs/rinex-cli/blob/main/plots/front-page/coordinates.png>
        <img src=https://github.com/rtk-rs/rinex-cli/blob/main/plots/front-page/coordinates.png alt="Plot">
    </a>
</div>

<div align="center">
    <p>
        Roaming with `PPP` in the arctic (CPP, Galileo + GPS E1/L1+E5/L5)
    </p>
    <a href=https://github.com/rtk-rs/rinex-cli/blob/main/plots/front-page/coordinates.png>
        <img src=https://github.com/rtk-rs/rinex-cli/blob/main/plots/front-page/coordinates.png alt="Plot">
    </a>
</div>

Licensing
=========

This library is part of the [RTK-rs framework](https://github.com/rtk-rs) which
is delivered under the [Mozilla V2 Public](https://www.mozilla.org/en-US/MPL/2.0) license.

P.V.T Solutions
===============

The objective of each solver is to resolve the state of the target device: the GNSS receiver,
in space time, with high accuracy. The solutions are called P. V. T. for Position Velocity Time
solution, that emphasize the space & time coordinates dual coordinates. 

Whether the receiver is moving or not is application dependent. You should select the solver
that suites your application best.

PVT Solvers
===========

This library proposes the `PPP` solver and the `RTK` solver, which you can select
depending on your application. Currently we do not offer a hybrid solver that can do both,
but that will be easily created in near future.

- `PPP` is used when you cannot access an RTK network (or reference sites).
- `RTK` is used for differential navigation, with at least one RTK reference site.

`RTK` is currently limited to a single reference, but we will augment this to N base in very near future.  
`RTK` should always be prefered over `PPP` to obtain higher accuracy more easily.  
Both solvers can operate without initial preset and can make a very good first guess to initialize themselves.

In static applications, it is important you keep the user `Profile` up to date, if the range of velocity varies.  
`Profile` also contains the perturbations of the measurement system, this applies to all cases, either
static or dynamic.

Application Programming Interface (API)
=======================================

This API demands you provide the minimum required to obtain valid PVT solutions.
One key element is that we are physics driven and are not tied to a specific data format (CSV, RINEX..). 
You may deploy this solver with your own data source.

Because Navigation is a complex task, providing an abstract interface for the end user is not easy.
Therefore, we rely on somewhat "advanced" interfacing, mainly:

* When building the solver, it obtains mutable access to the orbit provider.
We wrapp it in a `Rc` (Reference Counter) so it can still live own in this current thread,
to be updated, as required by real time applications. For post-processing applications,
you are fine at this point, because you deploy the solver once your data set is ready.

* The bias (environmental perturbations) provider is much simpler.
* One final external interface to provide your own epoch conversion method, that the solver
may use.

When solving in RTK (differential navigation), you must propose one reference station
that implements the `RTKBase` trait, at each solving attempt.
This means we naturally adapt to changes over your RTK network.

This API is physics driven and does not depend on the input data source. You should
be able to deploy this solver from any valid data source you have at your disposal.

GNSS-RTK is limited to ground based (=low altitude, within atmosphere) navigation on planet Earth. 
Although it would be possible to make GNSS-RTK more abstract and compatible with other Planets, it
is not planned to this day. You can reach out to us and join forces, if you want to see this happen !

Logs
====

This library uses `$RUST_LOG` to report the current state, what it is doing, analyze your preset
and give recommendations, and finally, help debug potential use cases.

For correct debugging, you should set `$RUST_LOG=debug`. For ulimtate debugging, `$RUST_LOG=trace`
will give all the information we output.

Summary
=======

Select a navigation method:

| Method        | Physics                                  | Accuracy      |  Application                                            |
|---------------|------------------------------------------|---------------|---------------------------------------------------------|
| `SPP`         | Single Frequency Pseudo Range navigation |  2/5          | Low cost devices                                        |
| `CPP`         | Dual Frequency Pseudo Range navigation   |  4/5          | Mid cost devices, Timing applications                   |
| `PPP`         | Dual Frequency Pseudo Range + Phase      |  5/5          | High cost devices, Very precise applications, Profesionnal surveying and calibrations |

Select your solver:

| Solver        | Method        | Accuracy      |  Context                 |
|---------------|---------------|---------------|--------------------------|
| `PPP`         | `SPP`         | 3/5           | Low cost / hobbyist      |
|               | `CPP`         | 4/5           | Mid cost / lab           |
|               | `PPP`         | 4.5/5         | High cost / profesionnal |
| `RTK`         | `SPP`         | 4/5           | Low cost / hobbyist      |
|               | `CPP`         | 4.5/5         | Mid cost / lab           |
|               | `PPP`         | 5/5           | High cost / profesionnal |


Deployment
==========

:warning: ANISE requires internet access either

1. at built time, when built with `embed_ephem` option 
2. at first deployment ever, when using low precision models and built without `embed-ephem` option
3. regularly, when using highest precision models, regardless of the compilation options.

This library exposes the `embed_ephem` compilation option. By using it, it is possible to run up to
mid-range and precise application without internet access. Ultra precise applications require
regular updates from the Internet.

Configuration simplicity
========================

Although the taks is challenging, one objective is to keep things simple.   
To do so, we rely on the `serde` ecosystem and are able to provide [a consice and comprehensible Parametrization interface](./documentation/Config.md)

Notes on Navigation Method
==========================

Absolute or differential is selected at deployment. The 
[navigation method](https://docs.rs/gnss-rtk/latest/gnss_rtk/prelude/enum.Method.html) is defined
by the configuration preset.

- `SPP`: is a Pseudo Range navigation method for single signal / degraded / limited setups.
Phase range observations are disregarded, unless you intend to use the L1+C1 enhancing smoothing technique.
You can enhance the `SPP` with an external model for environment perturbations. You can use the models we provide 
to answer that requirement. If you do not provide an accurate model (which is a complex thing to do), you cannot obtain
accurate solutions. We recommend using a different technique for any setups that allow you to do so.
It is often times interesting to degrade a CPP or PPP run back to SPP to actually see the huge difference
these two made.

- `CPP`: starting at CPP, we use physical cancellation of the ionosphere bias.
You need to provide two separate frequencies for this mode to operate.
CPP is like SPP and purely based on Pseudo Range navigation, phase range are disregarded,
unless you intend to use the L1+C1+L2+C2 smoothing technique.

- `PPP`: has the same requirements and objectives, but phase observations
are now also expected. It can be further enhanced by deploying the code smoothing technique.
Since PPP is very heavy and requires all signals to be available, our PPP presets activate the 
smoothing technique by default.

All of these are independent of the rover's profile AND the navigation technique:

- you can use any signal strategy along any rover profile
- you an use any signal strategy with either static or dynamic applications
- you can use any signal strategy along RTK differential technique

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

`GNSS-RTK` is a complete surveying tool that can operate without apriori knowledge.  
In otherwords, it is possible to obtain a precise solution from scratch.  
[Refer to this page that demonstrates this capacity](https://github.com/rtk-rs/rinex-cli/blob/main/demos/SURVEY.md). 

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

Like other topics, `GNSS-RTK` tries to be convenient to operate
and flexible, regarding the temporal solution.

We support all timescales provided by `Nyx-Space/Hifitime`.   
The `Time` trait should be implemented for applications that require very precise temporal solutions.  
It allows following the timescale states precisely (using external corrections).  

This library is flexible enough to let you express your measurements in the timescale you want,
and express the solution in the timescale you want. UTC timescale is fully supported.

Other features
==============

Non exhaustive list of other interesting features

- It is possible to apply a custom conic Azimutal + Elevation mask

Framework
=========

GNSS-RTK includes itself within and is closely tied to the following libraries:

* [ANISE](https://github.com/nyx-space/anise) for orbital calculations
* [Nyx-space](https://github.com/nyx-space/nyx) for advanced navigation
* [Hifitime](https://github.com/nyx-space/hifitime) for timing
* [Our GNSS library](https://github.com/rtk-rs/gnss) for basic GNSS definitions
* Nalgebra for all calculations
