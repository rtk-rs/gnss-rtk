use thiserror::Error;

#[derive(Debug, Copy, Clone, PartialEq, Error)]
pub enum ParsingError {
    #[error("bad observable")]
    BadObservable,
}
