GNSS-RTK
========

[![Rust](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml/badge.svg)](https://github.com/rtk-rs/gnss-rtk/actions/workflows/rust.yml)
[![Rust](https://github.com/rtk-rs/gnss-rtk/actions/workflows/daily.yml/badge.svg)](https://github.com/rtk-rs/gnss-rtk/actions/workflows/daily.yml)
[![crates.io](https://img.shields.io/crates/v/gnss-rtk.svg)](https://crates.io/crates/gnss-rtk)
[![crates.io](https://docs.rs/gnss-rtk/badge.svg)](https://docs.rs/gnss-rtk)

[![MRSV](https://img.shields.io/badge/MSRV-1.82.0-orange?style=for-the-badge)](https://github.com/rust-lang/rust/releases/tag/1.82.0)
[![License](https://img.shields.io/badge/license-MPL_2.0-orange?style=for-the-badge&logo=mozilla)](https://github.com/rtk-rs/gnss-rtk/blob/main/LICENSE)

The `GNSS-RTK` library provides Position Velocity Time (PVT) solution solvers,
with abstract and flexible interfaces that may apply to most navigation scenarios
and applications. Whether the application is real-time or post-processing oriented does not matter.

This library is thoroughly validated, including performance of the accuracy of the solutions
validated for each release.

<div align="center">
    <p>
        <br>PPP solver</br> in static application, surveying a professional geodetic marker
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
        Roaming session (pedestrian profile), post-processed with <br>PPP solver</br> (no ground reference), 
        CPP, GPS L1/L5 + Galileo L1/L5
    </p>
    <a href=https://github.com/rtk-rs/rinex-cli/blob/main/plots/front-page/roaming-ppp1.png>
        <img src=https://github.com/rtk-rs/rinex-cli/blob/main/plots/front-page/roaming-ppp1.png alt="Plot">
    </a>
</div>

Licensing
=========

This library is part of the [RTK-rs framework](https://github.com/rtk-rs) which
is delivered under the [Mozilla V2 Public](https://www.mozilla.org/en-US/MPL/2.0) license.

GNSS-RTK / P.V.T solutions
==========================

The main objective of this library is to propose efficient GNSS solvers, that
perform the rather complex task of solving Position, Velocity and Timing (P.V.T) solutions.
Indeed, GNSS navigation allows to obtain the position of the receiver in 3D (in space) 
and in time as well, and may include the instantaneous velocity of the target as well. 

The timing solutions (4th component) is the state of the receiver in the navigation timescale. 
It is possible to express the solutions in any of the supported Timescales. 

For the spatial coordinates, whether the receiver is moving or not is application dependent and the
solver should be able to determine the target state in any case. 

The proposed API is compatible with both real-time and post-processed navigation, read the dedicated chapter.

GNSS-RTK is currently limited to ground navigation on planet Earth. It should be rather easy to modify the current
API to propose ground-based navigation on any solid body supported by the `Nyx/ANISE` library. Mainly
by modifying the `EnvironmentalBias` trait, and possibly the `SpacebornBias` trait as well. But this is unscheduled work
as of today.

Solver, strategies and API
==========================

The current version proposes a unique object called `Solver` that is compatible to both absolute
and differential (1 reference) navigation. Once your measurements have been collected,
you select the navigation technique with want to deploy with either `ppp_solving` and `rtk_solving`.
:warning: RTK navigation is currently under development and should not be used until further notice.

This means, once RTK becomes available, if your network goes down you can always switch back to PPP (absolute navigation)
temporarily. Eventually, RTK (differential) should always be prefered because it gives the highest accuracy.

This of course applies to any use case, whether it is static or dynamic (roaming).

Tied to the navigation technique, you need to select a mode via the `Method` object. This determines which
signal strategy you will be using. We propose 4 modes, each one of them applies to either RTK or absolute navigation:

- `SPP` : single frequency pseudo-range based navigation. This is dedicated to low-cost degraded setups.
- `CPP` : dual frequency pseudo-range based navigation. It is about 20x more accurate than the previous mode
and gives 0.1m accuracy with high quality data. The secondary frequency is used to cancelled the ionosphere phsically.
- `PPP` : dual frequency phase based navigation (raw signal). Although the navigation uses raw signal, you still have
to pass on the pseudo-range on both frequencies, which makes this strategy the most demanding in terms of measurement.
- `PPP+AR` : same as PPP but a secondary prefit kalman filter is deployed. :warning: this is under development
and should not be used as of right now. 

For each mode, you can either deploy using absolute (`ppp_solving` attempt) or differential (`rtk_solving` attempt).
`rtk_solving` is not fully available yet and still under development. When it becomes feasible, that means the
mode defines your measurement strategy, but differential navigation is being used. Starting from `PPP` and `PPP+AR`,
it means the navigation algorithm becomes `RTK-PPP` as defined in the "litterature".

Recommendations: because this solver should not fail but report errors instead, you should always prefer
the highest / most advanced technique, and adapt/degrade that choice according to the returned message.
If such a strategy fails to apply, please fill an issue online and we will patch the library.
For example: always prefer `PPP` mode, and the library will let you know when it failed to deploy the mode
(for example, L5 phase is lost). In this case, you can always switch back to `CPP` mode temporarily.
That is particularly true if PPP convergence has been achieved.
Of course, this strategy only applies to people targetting highest accuracy. If you know your measurement
setup and context is not compatible with one strategy, it does not make sense to attempt it.

Application Programming Interface (API)
=======================================

The API requires you implement of several `Traits` to provide information
that the solving process demands:

- `OrbitSource` must be implemented to provide the orbital states of each satellites
at a specifing point in time. It is the only mandatory traits, all the others can be tied to zero
and you will still obtain a (poor) solution

- `EnvironmentalBias` must be implemented to describe the environmental perturbations. 
On earth ground navigation, it comprises the ionospheric and tropospheric perturbation.
Depending on the selected mode, some of it may be disregarded. Starting from `CPP` mode,
the ionospheric delay is physically canceled: if you tie this bias to zero, you will still obtain
a solution of good quality. For the tropospheric delay, you can either implement one of the models we propose
or implement your own. It is mandatory to do so to obtain a high quality solution;

- `SpacebornBias` must be implemented to obtain a basic solution. Indeed, if you don't provide the
`SatelliteClockCorrection` at the requested point in time, the solution will be off by thousands of kilometers.

Each traits are wrapped inside an `Rc<>` (Reference Counter) which makes this API compatible with real-time navigation.
This allows your structure(s) to live elsewhere, with so called "internal multability" (read on this pseudo concept)
that the data collection process demands.

Orbit Provider
==============

When measurements have been proposed, the solver will issue a request to this function pointer,
soon after. That you must answer, otherwise this candidate proposal will be dropped and will not contribute
to the solution. That is not a big deal, assuming you are in position to provide many measurements.
But we will always need at least 4 proposal to pass this step and future "post-fit" attitude filters
that you may have selected.

Other function pointers are much easier to implement and are not mandatory.

Environment models & perturbations
==================================

We provide a function pointer that allows you to apply the Ionosphere and Troposphere perturbations
compensation. If you are in position to apply your own database and/or model, you then have an interface
to do so. Returning no compensation will simply degrade the quality of your solution.

If you don't have a custom model, simply pick one of our internal models and apply it as the answer
to the request. For example, `TroposphereModel` can serve as a simple answer to the troposphere perturbation request.

Time transposition function
===========================

We require a function pointer to perform any time transposition. This is very useful if you have
access to correction tables for finer temporal corrections. In case you don't, simply return
the transposition using `Epoch.to_time_scale()`.

This API allows us to support all timescales supported by Hifitime and obtain higher
precision for people who have access to such data.

Examples
========

This library is no longer shipped with examples, for the simple reason that they are complicated to maintain,
and our framework now offers the two kind of applications you can think of to this library:

- [RT-Navi](https://github.com/rtk-rs/rt-navi) for real-time navigation, which is obviously hardware dependent,
and involves multi-threading.
- [GNSS-Qc](https://github.com/rtk-rs/gnss-qc) for post-processed navigation, which is more complex

Logs
====

This library uses `$RUST_LOG` to report the current state, what it is doing, analyze your preset
and give recommendations, and finally, help debug potential use cases.

For correct debugging, you should set `$RUST_LOG=debug`. For ulimtate debugging, `$RUST_LOG=trace`
will give all the information we output.

Summary
=======

Select a navigation method, depending on your equiment and targeted accuracy:

| Method        | Physics                                  | Accuracy      |  Application                                            |
|---------------|------------------------------------------|---------------|---------------------------------------------------------|
| `SPP`         | Single Frequency Pseudo Range navigation |  2/5          | Low cost devices                                        |
| `CPP`         | Dual Frequency Pseudo Range navigation   |  4/5          | Mid cost devices, Timing applications                   |
| `PPP`         | Dual Frequency Pseudo Range + Phase      |  5/5          | High cost devices, Very precise applications, Profesionnal surveying and calibrations |

Select your solver, depending on your use case, type of application and targeted accuracy:

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
