use std::str::FromStr;

use crate::{
    prelude::{Config, Duration, Epoch},
    tests::CandidatesBuilder,
};

#[test]
fn transmission_time() {
    let default_cfg = Config::default();
    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

    let mut dataset = CandidatesBuilder::build_rover_at(t0_gpst);

    for cd in dataset.iter_mut() {
        cd.transmission_time("rover", &default_cfg)
            .unwrap_or_else(|e| {
                panic!(
                    "{}({}) - transmission time failed with {}",
                    cd.epoch, cd.sv, e
                )
            });

        let t_rx = cd.epoch;
        let t_tx = cd.tx_epoch;
        let dt = t_rx - t_tx;

        assert!(t_rx != t_tx, "invalid results!");

        assert!(t_rx > t_tx, "physical non sense!");

        assert!(
            dt < Duration::from_milliseconds(100.0),
            "unrealistic results for Earth MEO/LEO NAV"
        );
    }
}
