Single Point Positioning
========================

SPP can be deployed in degraded conditions where only one carrier signal and/or a unique constellation is in sight.

Measurements requirements
=========================

`Mode::SPP` has less constraint on the measurements:
you only need to gather the approriate amount of pseudo range measurements. 

Several "bonus" may be unlocked if you provide extra measurements

* Providing dopplers has the benefit to validate the interpolated SV state vector
* Providing phase measurements will unlock fractional pseudo range estimate *in the future*
* Providing measurements on another signal will unlock the true ionosphere delay to be estimated.

Expected SPP configration
=========================

In `SPP` an interpolation of 9 makes sense and high orders will not be meaningful.  
You can use that to reduce the computation load.

Usually, in this mode the user provides measurements from a unique carrier signal.  
In this context, only a modeling of the ionosphere impact is feasible.  
As in other 

- you can only hope for a precision of a few meters.    
- an interpolation order of 9 makes sense, going above will increase the computation load without any benefit.
- the solver will have to estimate the total bias due to Ionospheric delay. If you can define
these components accurately yourself, you are encouranged to do it (see related API section and following paragraphs)

