// The Cli is only there to help us modify the initial conditions, in CI/CD.
use clap::{
    //value_parser,
    Arg,
    ArgAction,
    //ArgMatches,
    ColorChoice,
    Command,
};
use std::fs::read_to_string;

use crate::setup::Setup;

pub struct Cli {
    setup: Setup,
}

impl Cli {
    pub fn new() -> Self {
        let cmd = Command::new("ppp")
            .author("Guillaume W. Bres <guillaume.bressaix@gmail.com>")
            .version(env!("CARGO_PKG_VERSION"))
            .about("GNSS-RTK PPP Demo and Test program")
            .arg_required_else_help(false)
            .color(ColorChoice::Always)
            .arg(
                Arg::new("cfg")
                    .short('c')
                    .action(ArgAction::Set)
                    .required(false)
                    .help("Load Test Setup script (Optional)"),
            );
        let matches = cmd.get_matches();
        let setup = match matches.get_one::<String>("cfg") {
            Some(path) => {
                let content = read_to_string(path)
                    .unwrap_or_else(|e| panic!("failed to read test setup: {}", e));
                let setup: Setup = serde_json::from_str(&content)
                    .unwrap_or_else(|e| panic!("failed to parse test setup: {}", e));
                setup
            },
            None => Setup::default(),
        };
        Self { setup }
    }
    pub fn setup(&self) -> Setup {
        self.setup.clone()
    }
    //fn parse_vec3d_f64(&self, key: &str) -> Option<&(f64, f64, f64)> {
    //    self.matches.get_one::<(f64, f64, f64)>(key)
    //}
}
