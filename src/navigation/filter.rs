use nalgebra::{base::dimension::U8, OMatrix, OVector, Vector3, U3};

use nyx::cosmic::State as NyxState;
use nyx::NyxError;

#[cfg(feature = "serde")]
use serde::Deserialize;

use super::{Input, Output};
use crate::prelude::{Epoch, Error};

/// Navigation Filter.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub enum Filter {
    /// None: solver filter completely bypassed. Lighter calculations, no iterative behavior.
    None,
    #[default]
    /// LSQ Filter. Heavy computation.
    LSQ,
    /// Kalman Filter. Heavy+ computations. Compared to LSQ, the Kalman filter
    /// converges faster and has the ability to "improve" models
    Kalman,
}

#[derive(Debug, Clone, Default)]
struct LSQState {
    pub p: OMatrix<f64, U8, U8>,
    pub x: OVector<f64, U8>,
}

#[derive(Debug, Clone, Default)]
struct KFState {
    pub q: OMatrix<f64, U8, U8>,
    pub p: OMatrix<f64, U8, U8>,
    pub x: OVector<f64, U8>,
    pub phi: OMatrix<f64, U8, U8>,
}

#[derive(Debug, Clone)]
pub enum FilterState {
    LSQ(LSQState),
    KF(KFState),
}

impl Default for FilterState {
    fn default() -> Self {
        Self::LSQ(Default::default())
    }
}

impl FilterState {
    fn lsq(state: LSQState) -> Self {
        Self::LSQ(state)
    }
    fn as_lsq(&self) -> Option<&LSQState> {
        match self {
            Self::LSQ(state) => Some(state),
            _ => None,
        }
    }
    fn kf(state: KFState) -> Self {
        Self::KF(state)
    }
    fn as_kf(&self) -> Option<&KFState> {
        match self {
            Self::KF(state) => Some(state),
            _ => None,
        }
    }
    pub(crate) fn estimate(&self) -> OVector<f64, U8> {
        match self {
            Self::LSQ(state) => state.x,
            Self::KF(state) => state.x,
        }
    }
}

impl Filter {
    fn lsq_resolve(input: &Input, p_state: Option<FilterState>) -> Result<Output, Error> {
        match p_state {
            Some(FilterState::LSQ(p_state)) => {
                let p_1 = p_state.p.try_inverse().ok_or(Error::MatrixInversionError)?;

                let g_prime = input.g.clone().transpose();
                let q = (g_prime.clone() * input.g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let p = g_prime.clone() * input.w.clone() * input.g.clone();
                let p = (p_1 + p).try_inverse().ok_or(Error::MatrixInversionError)?;

                let x =
                    p * (p_1 * p_state.x + (g_prime.clone() * input.w.clone() * input.y.clone()));

                Ok(Output {
                    gdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)] + q[(3, 3)]).sqrt(),
                    pdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)]).sqrt(),
                    tdop: q[(4, 3)].sqrt(),
                    q,
                    state: FilterState::lsq(LSQState { p, x }),
                })
            },
            _ => {
                let g_prime = input.g.clone().transpose();

                let q = (g_prime.clone() * input.g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let p = (g_prime.clone() * input.w.clone() * input.g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let x = p * (g_prime.clone() * input.w.clone() * input.y.clone());
                if x[3].is_nan() {
                    return Err(Error::TimeIsNan);
                }

                Ok(Output {
                    gdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)] + q[(3, 3)]).sqrt(),
                    pdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)]).sqrt(),
                    tdop: q[(4, 3)].sqrt(),
                    q,
                    state: FilterState::lsq(LSQState { p, x }),
                })
            },
        }
    }
    fn kf_resolve(input: &Input, p_state: Option<FilterState>) -> Result<Output, Error> {
        match p_state {
            Some(FilterState::KF(p_state)) => {
                let x_bn = p_state.phi * p_state.x;
                let p_bn = p_state.phi * p_state.p * p_state.phi.transpose() + p_state.q;

                let p_bn_inv = p_bn.try_inverse().ok_or(Error::MatrixInversionError)?;
                let p_n = (input.g.transpose() * input.w * input.g + p_bn_inv)
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let w_g = input.g.transpose() * input.w * input.y;
                let w_gy_pbn = w_g + (p_bn_inv * x_bn);
                let x_n = p_n * w_gy_pbn;

                let q_n = input.g.transpose() * input.g;
                let phi_diag = OVector::<f64, U8>::from([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]);
                let q_diag = OVector::<f64, U8>::from([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]);

                Ok(Output {
                    gdop: (q_n[(0, 0)] + q_n[(1, 1)] + q_n[(2, 2)] + q_n[(3, 3)]).sqrt(),
                    pdop: (q_n[(0, 0)] + q_n[(1, 1)] + q_n[(2, 2)]).sqrt(),
                    tdop: q_n[(4, 3)].sqrt(),
                    q: q_n,
                    state: FilterState::kf(KFState {
                        p: p_n,
                        x: x_n,
                        q: OMatrix::<f64, U8, U8>::from_diagonal(&q_diag),
                        phi: OMatrix::<f64, U8, U8>::from_diagonal(&phi_diag),
                    }),
                })
            },
            _ => {
                let g_prime = input.g.clone().transpose();
                let q = (g_prime.clone() * input.g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let p = (g_prime.clone() * input.w.clone() * input.g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let x = p * (g_prime.clone() * input.w.clone() * input.y.clone());
                if x[3].is_nan() {
                    return Err(Error::TimeIsNan);
                }

                let phi_diag = OVector::<f64, U8>::from([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]);
                let q_diag = OVector::<f64, U8>::from([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]);

                Ok(Output {
                    gdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)] + q[(3, 3)]).sqrt(),
                    pdop: (q[(0, 0)] + q[(1, 1)] + q[(2, 2)]).sqrt(),
                    tdop: q[(4, 3)].sqrt(),
                    q,
                    state: FilterState::kf(KFState {
                        p,
                        x,
                        q: OMatrix::<f64, U8, U8>::from_diagonal(&q_diag),
                        phi: OMatrix::<f64, U8, U8>::from_diagonal(&phi_diag),
                    }),
                })
            },
        }
    }
    pub fn resolve(&self, input: &Input, p_state: Option<FilterState>) -> Result<Output, Error> {
        match self {
            Filter::None => Self::lsq_resolve(input, None),
            Filter::LSQ => Self::lsq_resolve(input, p_state),
            Filter::Kalman => Self::kf_resolve(input, p_state),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Copy, Default)]
pub(crate) struct State3D {
    pub t: Epoch,
    pub inner: Vector3<f64>,
}

impl std::fmt::LowerExp for State3D {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> Result<(), std::fmt::Error> {
        let (x, y, z) = (self.inner[0], self.inner[1], self.inner[2]);
        write!(f, "({:.6E},{:.6E},{:.6E})", x, y, z)
    }
}

impl std::fmt::Display for State3D {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> Result<(), std::fmt::Error> {
        let (x, y, z) = (self.inner[0], self.inner[1], self.inner[2]);
        write!(f, "({:.6E},{:.6E},{:.6E})", x, y, z)
    }
}

impl NyxState for State3D {
    type Size = U3;
    type VecLength = U3;
    fn as_vector(&self) -> Result<OVector<f64, U3>, NyxError> {
        Ok(self.inner.into())
    }
    fn unset_stm(&mut self) {}
    fn set(&mut self, t: Epoch, vector: &OVector<f64, U3>) -> Result<(), NyxError> {
        self.t = t;
        self.inner = vector.clone();
        Ok(())
    }
    fn epoch(&self) -> Epoch {
        self.t
    }
    fn set_epoch(&mut self, t: Epoch) {
        self.t = t;
    }
}
