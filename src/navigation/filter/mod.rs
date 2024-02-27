mod lsq;
use lsq::State as LSQState;

use thiserror::Error;

use super::Input as NavInput;
use super::Output as NavOutput;

use crate::cfg::Filter as FilterTypeEnum;

use nalgebra::Vector4;

#[derive(Error, Debug)]
pub enum Error {
    #[error("failed to invert matrix")]
    MatrixInversionError,
    #[error("failed to invert covar matrix")]
    CoverMatrixInversionError,
}

pub struct FilterState {
    /// r { x, y, z, dt } vector
    r: Vector4<f64>, 
    /// P = G' * w * g
    p: Matrix4<f64>,
}

/// Navigation filter
pub struct Filter {
    filter: FilterTypeEnum,
    state: Option<FilterState>,
}

impl Filter {
    pub fn resolve(&mut self, nav: NavInput) -> Result<NavOutput, Error> {
        let new_state = if let Some(state) = self.state {
            match self.filter {
                FilterTypeEnum::LSQ => {
                    let p_1 = state
                        .p
                        .try_inverse()
                        .ok_or(Error::MatrixInversionError)?;
                    
                    let g_prime = nav.g.clone().transpose();
                    let q = (g_prime.clone() * nav.g.clone())
                        .try_inverse()
                        .ok_or(Error::MatrixInversionError)?;
                    
                    let p = g_prime.clone() * nav.w.clone() * nav.g.clone();
                    let p = (p_1 + p)
                        .try_inverse()
                        .ok_or(Error::MatrixInversionError)?;

                    let r = p * (p_1 * p_state.x + (g_prime.clone() * nav.w.clone() * nav.y));
                    FilterState { r, p }
                },
                FilterTypeEnum::Kalman => todo!("kalman filter is not available yet"),
                _ => unreachable!("filter::resolve"),
            }
        } else {
            let g_prime = nav.g.clone().transpose();
            let q = (g_prime.clone() * nav.g.clone())
                .try_inverse()
                .ok_or(Error::MatrixInversionError)?;

            let p = (g_prime.clone() * nav.w.clone() * nav.g.clone())
                .try_inverse()
                .ok_or(Error::MatrixInversionError)?;
            
            let r = p * (g_prime.clone() * nav.w.clone() * nav.y);
            FilterState { r, p }
        };
        if self.filter != Filter::NoFilter {
            self.state = Some(new_state);
        }
        Ok(NavOutput {
            q,
            r: new_state.r.clone(),
        })
    }   
}

// // Filter state
// #[derive(Debug, Clone)]
// pub(crate) enum FilterState {
//     /// LSQ state
//     LSQState(LSQState),
//     // /// KF state
//     // KfState(KfState),
// }

// impl FilterState {
//     fn lsq(&self) -> Option<LSQState> {
//         match self {
//             Self::LSQState(state) => Some(state),
//             _ => None,
//         }
//     }
//     // fn kf(&self) -> Option<KfState> {
//     //     match self {
//     //         Self::KfState(state) => Some(state),
//     //         _ => None,
//     //     }
//     // }
// }
