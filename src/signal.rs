#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum Signal {
    L1,
    L2,
    L5,
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum UsedSignal {
    Single(Signal),
    Dual(Signal),
    Triple(Signal),
}
