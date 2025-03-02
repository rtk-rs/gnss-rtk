use crate::prelude::{Candidate, Carrier, Constellation, Epoch, Observation, SV};

use rinex::prelude::{obs::SignalObservation, Carrier as RinexCarrier, Observable, Rinex};

pub struct SignalSource<'a> {
    pub pending: Option<Candidate>,
    iter: Box<dyn Iterator<Item = (Epoch, &'a SignalObservation)> + 'a>,
}

impl<'a> SignalSource<'a> {
    pub fn from_rinex_gps(rinex: &'a Rinex) -> Self {
        Self {
            iter: Box::new(
                rinex
                    .signal_observations_sampling_ok_iter()
                    .filter_map(|(t, sig)| {
                        if sig.sv.constellation == Constellation::GPS {
                            Some((t, sig))
                        } else {
                            None
                        }
                    }),
            ),
            pending: None,
        }
    }

    pub fn from_rinex(rinex: &'a Rinex) -> Self {
        Self {
            iter: rinex.signal_observations_sampling_ok_iter(),
            pending: None,
        }
    }

    pub fn from_iter(iter: Box<dyn Iterator<Item = (Epoch, &'a SignalObservation)>>) -> Self {
        Self {
            iter,
            pending: None,
        }
    }
}

fn interesting_signal(observable: &Observable) -> bool {
    observable.is_doppler_observable()
        || observable.is_phase_range_observable()
        || observable.is_pseudo_range_observable()
}

fn carrier_cast(carrier: RinexCarrier) -> Carrier {
    match carrier {
        RinexCarrier::L1 => Carrier::L1,
        RinexCarrier::L2 => Carrier::L2,
        RinexCarrier::L5 => Carrier::L5,
        RinexCarrier::L6 => Carrier::L6,
        RinexCarrier::B2 => Carrier::B2,
        RinexCarrier::B1A => Carrier::B1aB1c,
        RinexCarrier::B1C => Carrier::B1aB1c,
        RinexCarrier::B3 => Carrier::B3,
        RinexCarrier::E1 => Carrier::E1,
        RinexCarrier::B1I => Carrier::B1I,
        RinexCarrier::B2A => Carrier::B2A,
        RinexCarrier::B2B => Carrier::B2iB2b,
        RinexCarrier::B2I => Carrier::B2iB2b,
        RinexCarrier::B3A => Carrier::B3,
        RinexCarrier::E5 => Carrier::E5,
        RinexCarrier::E5a => Carrier::E5A,
        RinexCarrier::E5b => Carrier::E5B,
        RinexCarrier::E6 => Carrier::E6,
        c => panic!("Non supported frequency: {}", c),
    }
}

fn form_candidate(t: Epoch, sig: &SignalObservation) -> Candidate {
    let sv = sig.sv;
    let constellation = sig.sv.constellation;

    let carrier = sig.observable.carrier(constellation).unwrap_or_else(|e| {
        panic!(
            "failed to determine {}({}) frequency: {:?}",
            sv, sig.observable, e
        )
    });

    let carrier = carrier_cast(carrier);

    let observation = match &sig.observable {
        Observable::PseudoRange(_) => Observation::pseudo_range(carrier, sig.value, None),
        Observable::Doppler(_) => Observation::doppler(carrier, sig.value, None),
        Observable::PhaseRange(_) => Observation::ambiguous_phase_range(carrier, sig.value, None),
        observable => panic!("invalid observable: {}", observable),
    };

    Candidate::new(sig.sv, t, vec![observation])
}

impl<'a> Iterator for SignalSource<'a> {
    type Item = Vec<Candidate>;

    /// Packs all [Candidate]s for every single Epoch entry, ready to be used
    /// in the solving process.
    fn next(&mut self) -> Option<Self::Item> {
        let mut t = Epoch::default();
        let mut ret = Vec::<Candidate>::with_capacity(8);
        let mut known_sv = Vec::<SV>::with_capacity(8);

        // possible residues
        if let Some(pending) = &self.pending {
            t = pending.t;
            known_sv.push(pending.sv);
            ret.push(pending.clone());
        }

        self.pending = None; // discard past run
        let mut len = 0;

        loop {
            let iter_next = self.iter.next();
            if iter_next.is_none() {
                break;
            }

            let (new_t, new_sig) = iter_next.unwrap();

            if !interesting_signal(&new_sig.observable) {
                continue;
            }

            if ret.len() == 0 {
                // first addition
                t = new_t;
                known_sv.push(new_sig.sv);
                ret.push(form_candidate(new_t, new_sig));
            } else {
                // expanding
                if new_t > t {
                    // new epoch: store for later & exit
                    self.pending = Some(form_candidate(new_t, new_sig));
                    break;
                } else {
                    if known_sv.contains(&new_sig.sv) {
                        for cd in ret.iter_mut() {
                            if cd.sv == new_sig.sv {
                                for observation in cd.observations.iter_mut() {
                                    match &new_sig.observable {
                                        Observable::Doppler(_) => {
                                            observation.doppler = Some(new_sig.value);
                                        },
                                        Observable::PhaseRange(_) => {
                                            observation.phase_range_m = Some(new_sig.value);
                                            observation.ambiguity = None;
                                        },
                                        Observable::PseudoRange(_) => {
                                            observation.pseudo_range_m = Some(new_sig.value);
                                        },
                                        observable => panic!("invalid observable: {}", observable),
                                    }
                                }
                            }
                        }
                    } else {
                        // new sv
                        ret.push(form_candidate(new_t, new_sig));
                    }

                    known_sv.push(new_sig.sv);
                }
            }
        }

        if ret.len() == 0 {
            None
        } else {
            Some(ret)
        }
    }
}
