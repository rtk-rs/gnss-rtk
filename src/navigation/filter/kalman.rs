#[derive(Debug, Clone)]
/// KF state
pub(crate) struct State {
    /* p matrix */
    pub(crate) p: Matrix4<f64>,
    /* x estimate */
    pub(crate) x: Matrix4x1<f64>,
}


impl State {
    pub fn eval(&mut self,
        phi: &Matrix4<f64>,
        q: &Matrix4<f64>,
        x: &Vector4<f64>,
        p: &Matrix4x1<f64>,
    ) -> Result<Self, Error> {
        let x = phi * x;
        let p = (phi * p * phi.transpose()) + q;
        Ok(Self {
            x,
            p,
        })
    }
}

