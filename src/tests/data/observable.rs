use crate::tests::ParsingError;
use std::str::FromStr;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Observable {
    PseudoRange,
    AmbiguousPhaseRange,
    UnambiguousPhaseRange,
    Doppler,
}

impl std::fmt::Display for Observable {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::PseudoRange => write!(f, "pr"),
            Self::AmbiguousPhaseRange => write!(f, "cp"),
            Self::UnambiguousPhaseRange => write!(f, "rcp"),
            Self::Doppler => write!(f, "dop"),
        }
    }
}

impl std::str::FromStr for Observable {
    type Err = ParsingError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let trimmed = s.trim();
        if trimmed.eq("pr") {
            Ok(Self::PseudoRange)
        } else if trimmed.eq("cp") {
            Ok(Self::AmbiguousPhaseRange)
        } else if trimmed.eq("rcp") {
            Ok(Self::UnambiguousPhaseRange)
        } else if trimmed.eq("dop") {
            Ok(Self::Doppler)
        } else {
            Err(ParsingError::BadObservable)
        }
    }
}

#[test]
fn test_observable_parsing() {
    for obs_str in ["cp", "pr", "rcp", "dop"] {
        let obs = Observable::from_str(obs_str).unwrap();
        let formatted = obs.to_string();
        assert_eq!(formatted, obs_str);
    }
}
