use std::str::FromStr;

use crate::{
    prelude::{Config, Duration, Epoch},
    tests::CandidatesBuilder,
};

#[test]
fn transmission_time() {
    let default_cfg = Config::default();

    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

    let mut dataset = CandidatesBuilder::build_at(t0_gpst);

    for cd in dataset.iter_mut() {
        cd.tx_epoch(&default_cfg).unwrap_or_else(|e| {
            panic!(
                "{}({}) - transmission time should be doable! {}",
                cd.t, cd.sv, e
            )
        });

        let t_rx = cd.t;
        let t_tx = cd.t_tx;
        let dt = t_rx - t_rx;

        assert!(t_rx != t_tx, "invalid results!");

        assert!(t_rx > t_tx, "physical non sense!");

        assert!(
            dt < Duration::from_milliseconds(100.0),
            "unrealistic results for Earth MEO/LEO NAV"
        );
    }
}
