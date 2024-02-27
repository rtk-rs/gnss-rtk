//! LSQ filter

#[derive(Debug, Clone)]
/// LSQ filter state
pub(crate) struct LSQState {
    /* p matrix */
    pub p: Matrix4<f64>,
    /* x estimate */
    pub x: Matrix4x1<f64>,
    /* output */
    pub m: MatrixOutput,
}

impl FilterState for LSQState {
    fn initialized(&self) -> bool {
        false
    }
    fn first_iteration(&self) -> bool {
        true
    }
    fn has_converged(&self) -> bool {
        false
    }
    fn eval(&self, m: MatrixInput, prev: Option<&Self>) -> Result<Self, Error> {
        if let Some(p_state) = prev {
            let p_1 = p_state
                .m
                .p
                .try_inverse()
                .ok_or(Error::)?;
            
            let g_prime = m.g.clone().transpose();
            let q = (g_prime.clone() * g.clone())
                    .try_inverse()
                    .ok_or(Error::MatrixInversionError)?;

                let p = g_prime.clone() * w.clone() * g.clone();
                let p = (p_1 + p)
                    .try_inverse()
                    .ok_or(Error::CovarMatrixInversionError)?;

                let x = p * (p_1 * p_state.x + (g_prime.clone() * w.clone() * y));
                (q, x, Some(FilterState::LSQState(LSQState { x, p })))
        } else {
            let g_prime = m.g.clone().transpose();
            let q = (g_prime.clone() * m.g.clone())
                .try_inverse()
                .ok_or(Error::MatrixInversionError)?;
            
            let p = (g_prime.clone() * m.w.clone() * g.clone())
                .try_inverse()
                .ok_or(Error::CovarMatrixInversionError)?;
                   
            let x = (p * g_prime.clone()) * m.w.clone() * m.y;
            Ok(Self {
                q,
                x,
                p
            })
        }
    }
}
