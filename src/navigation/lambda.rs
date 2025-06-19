use nalgebra::{
    allocator::Allocator, DMatrix, DVector, DefaultAllocator, DimName, OMatrix, OVector,
};

use crate::prelude::{Error, SV};

use log::{debug, error};

#[derive(Debug, Default, Clone, PartialEq)]
struct FixedAmbiguity {
    pub sv: SV,
    pub n_int: u64,
}

#[derive(Default)]
pub struct LambdaAR {
    fixed: DVector<FixedAmbiguity>,
}

impl LambdaAR {
    const MAX_SEARCH: usize = 1_000;

    /// Reset this [LambdaAR]
    pub fn reset(&mut self) {
        // let ndf = self.n_int.nrows();
        // self.n_int.set(0.0);
    }

    /// Reset this particular [SV]
    pub fn reset_sv(&mut self, sv: SV) {}

    fn signum(value: f64) -> f64 {
        if value <= 0.0 {
            -1.0
        } else {
            1.0
        }
    }

    fn round(value: f64) -> f64 {
        (value + 0.5).floor()
    }

    /// Calculates L and D such as Q=L' Diag(D) L
    fn ld_factorization<D: DimName>(
        ndf: usize,
        q_mat: &OMatrix<f64, D, D>,
        l_mat: &mut DMatrix<f64>,
        d_vec: &mut DVector<f64>,
    ) -> Result<(), Error>
    where
        DefaultAllocator: Allocator<D>,
        DefaultAllocator: Allocator<D, D>,
    {
        let mut a = 0.0f64;
        let mut a_mat = q_mat.clone();

        Ok(())
    }

    fn reduction(
        ndf: usize,
        l_mat: &mut DMatrix<f64>,
        d_diag: &mut DMatrix<f64>,
        z_mat: &mut DMatrix<f64>,
    ) {
        let mut j = ndf - 2;
        let mut k = ndf - 2;

        loop {
            if j == 0 {
                break;
            }

            if j <= k {
                for i in j + 1..ndf {
                    Self::gauss_transform(i, j, ndf, l_mat, z_mat);
                }
            }

            let delta =
                d_diag[(j, j)] + l_mat[(j + 1, j)] * l_mat[(j + 1, j)] + d_diag[(j + 1, j + 1)];

            if delta + 1E-6 < d_diag[(j + 1, j + 1)] {
                Self::permutations(ndf, l_mat, d_diag, j, delta, z_mat);

                k = j;
                j = ndf - 2;
            } else {
                j -= 1;
            }
        }
    }

    fn gauss_transform(
        i: usize,
        j: usize,
        ndf: usize,
        l_mat: &mut DMatrix<f64>,
        z_mat: &mut DMatrix<f64>,
    ) {
        let mu = Self::round(l_mat[(i, j)]);

        if mu != 0.0 {
            for k in i..ndf {
                l_mat[(k, j)] -= mu * l_mat[(k, i)];
            }

            for k in 0..ndf {
                z_mat[(k, j)] -= mu * z_mat[(k, i)];
            }
        }
    }

    fn permutations(
        ndf: usize,
        l_mat: &mut DMatrix<f64>,
        d_diag: &mut DMatrix<f64>,
        j: usize,
        delta: f64,
        z_mat: &mut DMatrix<f64>,
    ) {
        let eta = d_diag[(j, j)] / delta;
        let lambda = d_diag[(j + 1, j + 1)] * l_mat[(j + 1, j)] / delta;

        d_diag[(j, j)] = eta * d_diag[(j + 1, j + 1)];
        d_diag[(j + 1, j + 1)] = delta;

        for k in 0..=j - 1 {
            let a0 = l_mat[(j, k)];
            let a1 = l_mat[(j + 1, k)];

            l_mat[(j, k)] -= l_mat[(j + 1, j)] * a0 + a1;
            l_mat[(j + 1, k)] = eta * a0 + lambda * a1;
        }

        l_mat[(j + 1, j)] = lambda;

        for k in j + 2..ndf {
            l_mat.swap((k, j), (k, j + 1));
        }

        for k in 0..ndf {
            z_mat.swap((k, j), (k, j + 1));
        }
    }

    fn search(
        ndf: usize,
        nfixed: usize,
        l_mat: DMatrix<f64>,
        d_diag: DMatrix<f64>,
        zs_vec: DVector<f64>,
        zn_mat: &mut DMatrix<f64>,
        s_vec: &mut DVector<f64>,
    ) {
        let mut maxdist = 1E99_f64;

        let mut nn = 0usize;
        let mut imax = 0usize;

        let mut s_mat = DMatrix::<f64>::zeros(ndf, ndf);
        let mut dist_vec = DVector::<f64>::zeros(ndf);

        let mut k = ndf - 1;

        let mut zb_vec = DVector::<f64>::zeros(ndf);
        let mut z_vec = DVector::<f64>::zeros(ndf);
        let mut step = DVector::<f64>::zeros(ndf);

        zb_vec[k] = zs_vec[k];
        z_vec[k] = Self::round(zb_vec[k]);

        let mut y = zb_vec[k] - z_vec[k];

        step[k] = Self::signum(y);

        for _ in 0..Self::MAX_SEARCH {
            let newdist = dist_vec[k] + y + y / d_diag[(k, k)];

            if newdist < maxdist {
                if k != 0 {
                    // Case 1: move down
                    k -= 1;
                    dist_vec[k] = newdist;

                    for i in 0..=k {
                        s_mat[(k, i)] =
                            s_mat[(k + 1, i)] + (z_vec[k + 1] - zb_vec[k + 1]) * l_mat[(k + 1, i)];
                    }

                    zb_vec[k] = zs_vec[k] + s_mat[(k, k)];
                    z_vec[k] = Self::round(zb_vec[k]);

                    y = zb_vec[k] - z_vec[k];
                    step[k] = Self::signum(y);
                } else {
                    // Case 2: store the candidate and try next valid integer

                    if nn < nfixed {
                        if nn == 0 || newdist > s_vec[imax] {
                            imax = nn;
                        }

                        for i in 0..ndf {
                            zn_mat[(i, nn)] = z_vec[i];
                        }

                        s_vec[nn] = newdist;
                        nn += 1;
                    } else {
                        if newdist < s_vec[imax] {
                            for i in 0..ndf {
                                zn_mat[(i, imax)] = z_vec[i];
                            }

                            s_vec[imax] = newdist;

                            for i in 0..nfixed {
                                imax = i;
                                if s_vec[imax] < s_vec[i] {
                                    imax = i;
                                }
                            }
                        }

                        maxdist = s_vec[imax];
                    }

                    z_vec[0] += step[0]; // next valid integer
                    y = zb_vec[0] - z_vec[0];
                    step[0] = -step[0] - Self::signum(step[0]);
                }
            } else {
                // case 3: exit or move up

                if k == ndf - 1 {
                    break;
                } else {
                    k += 1; // move up
                    z_vec[k] += step[k]; // next valid integer
                    y = zb_vec[k] - z_vec[k];
                    step[k] = -step[k] - Self::signum(step[k]);
                }
            }
        }

        // sort by s
        for i in 0..nfixed - 1 {
            for j in i + 1..nfixed {
                if s_vec[i] < s_vec[j] {
                    continue;
                }

                s_vec.swap_rows(i, j);

                for k in 0..ndf {
                    zn_mat.swap((k, i), (k, j));
                }
            }
        }
    }

    /// Runs modified LAMBDA ILS
    pub fn run(
        &mut self,
        ndf: usize,
        nfixed: usize,
        x_vec: DVector<f64>,
        q_mat: DMatrix<f64>,
        // sv_indexes: &DVector<(SV, usize)>,
    ) -> Result<(), Error> {
        const MAX_DIST: f64 = 1.0E99;

        let mut z_mat = DMatrix::<f64>::identity(ndf, ndf);
        let mut s_vec = DVector::<f64>::zeros(nfixed);

        let mut e_mat = DMatrix::<f64>::zeros(ndf, nfixed);

        let ldl = q_mat.clone().udu().ok_or(Error::AmbiguityFactorization)?;

        let mut d_diag = ldl.d_matrix();
        let mut l_mat = ldl.u.transpose();

        debug!(
            "search - ndf={} - X={} Q={} L={} D={}",
            ndf, x_vec, q_mat, l_mat, d_diag
        );

        Self::reduction(ndf, &mut l_mat, &mut d_diag, &mut z_mat);

        let zs_vec = z_mat.clone() * x_vec;

        debug!("search - z={}", zs_vec);

        Self::search(ndf, nfixed, l_mat, d_diag, zs_vec, &mut e_mat, &mut s_vec);

        debug!("search - E={} S={}", e_mat, s_vec);

        let z_inv = z_mat.try_inverse().ok_or(Error::AmbiguityInverse)?;

        debug!("search - Z'={}", z_inv);

        let f_mat = z_inv * e_mat;

        debug!("search - F={}", f_mat);

        Ok(())
    }
}

#[cfg(test)]
mod test {

    use super::LambdaAR;
    use crate::prelude::{Constellation, SV};
    use crate::tests::init_logger;

    use nalgebra::{DMatrix, DVector, DimName, OMatrix, OVector, U4, U6};

    #[test]
    fn gauss_transform() {
        let mut l_mat = DMatrix::<f64>::identity(U6::USIZE, U6::USIZE);
        let mut z_mat = l_mat.clone();
        // let a = OVector::<f64, U6>::from_row_slice(&[
        //     1585184.171,
        //     -6716599.430,
        //     3915742.905,
        //     7627233.455,
        //     9565990.879,
        //     989457273.200,
        // ]);

        for i in 0..U6::USIZE {
            for j in 0..U6::USIZE {
                LambdaAR::gauss_transform(i, j, U6::USIZE, &mut l_mat, &mut z_mat);
            }
        }
    }

    #[test]
    fn mlambda_ils_1() {
        init_logger();

        let mut lambda = LambdaAR::default();

        let g01 = SV::new(Constellation::GPS, 1);
        let g02 = SV::new(Constellation::GPS, 2);
        let g03 = SV::new(Constellation::GPS, 3);
        let g04 = SV::new(Constellation::GPS, 4);
        let g05 = SV::new(Constellation::GPS, 5);
        let g06 = SV::new(Constellation::GPS, 6);

        let sv_indexes = DVector::<(SV, usize)>::from_row_slice(&[
            (g01, 0),
            (g02, 1),
            (g03, 2),
            (g04, 3),
            (g05, 4),
            (g06, 5),
        ]);

        let x = DVector::<f64>::from_row_slice(&[
            1585184.171,
            -6716599.430,
            3915742.905,
            7627233.455,
            9565990.879,
            989457273.200,
        ]);

        let q = DMatrix::<f64>::from_row_slice(
            U6::USIZE,
            U6::USIZE,
            &[
                0.227134, 0.112202, 0.112202, 0.112202, 0.112202, 0.103473, 0.112202, 0.227134,
                0.112202, 0.112202, 0.112202, 0.103473, 0.112202, 0.112202, 0.227134, 0.112202,
                0.112202, 0.103473, 0.112202, 0.112202, 0.112202, 0.227134, 0.112202, 0.103473,
                0.112202, 0.112202, 0.112202, 0.112202, 0.227134, 0.103473, 0.103473, 0.103473,
                0.103473, 0.103473, 0.103473, 0.434339,
            ],
        );

        let f_mat = DMatrix::<f64>::from_row_slice(
            U6::USIZE,
            2,
            &[
                1585184.000000,
                1585184.000000,
                -6716599.000000,
                -6716600.000000,
                3915743.000000,
                3915743.000000,
                7627234.000000,
                7627233.000000,
                9565991.000000,
                9565991.000000,
                989457273.000000,
                989457273.000000,
            ],
        );

        let s_1 = DVector::<f64>::from_row_slice(&[3.507984, 3.708456]);

        let ndf = 6;
        let nfixed = 2;

        lambda.run(ndf, nfixed, x, q).unwrap_or_else(|e| {
            panic!("mlabmda search failed with {}", e);
        });

        // for i in 0..ndf {
        //     for j in 0..ndf {
        //         assert!(
        //             (f[(i, j)] - f1[(i, j)].abs()) < 1.0E-4,
        //             "lambda-search test#1 failed"
        //         );
        //     }

        //     assert!((s[i] - s1[i]).abs() < 1.0E-4, "lambda-search test#1 failed");
        // }
    }

    // #[test]
    // fn mlambda_search_2() {

    //     let a = DVector::<f64>::from_row_slice(&[
    //         -13324172.755747,
    //         -10668894.713608,
    //          -7157225.010770,
    //          -6149367.974367,
    //          -7454133.571066,
    //          -5969200.494550,
    //           8336734.058423,
    //           6186974.084502,
    //         -17549093.883655,
    //         -13970158.922370
    //     ]);

    //     let q = DMatrix::<f64>::from(&[
    //                 0.446320,        0.223160,        0.223160,        0.223160,        0.223160,        0.572775,        0.286388,        0.286388,        0.286388,        0.286388,
    //         0.223160,        0.446320,        0.223160,        0.223160,        0.223160,        0.286388,        0.572775,        0.286388,        0.286388,        0.286388,
    //         0.223160,        0.223160,        0.446320,        0.223160,        0.223160,        0.286388,        0.286388,        0.572775,        0.286388,        0.286388,
    //         0.223160,        0.223160,        0.223160,        0.446320,        0.223160,        0.286388,        0.286388,        0.286388,        0.572775,        0.286388,
    //         0.223160,        0.223160,        0.223160,        0.223160,        0.446320,        0.286388,        0.286388,        0.286388,        0.286388,        0.572775,
    //         0.572775,        0.286388,        0.286388,        0.286388,        0.286388,        0.735063,        0.367531,        0.367531,        0.367531,        0.367531,
    //         0.286388,        0.572775,        0.286388,        0.286388,        0.286388,        0.367531,        0.735063,        0.367531,        0.367531,        0.367531,
    //         0.286388,        0.286388,        0.572775,        0.286388,        0.286388,        0.367531,        0.367531,        0.735063,        0.367531,        0.367531,
    //         0.286388,        0.286388,        0.286388,        0.572775,        0.286388,        0.367531,        0.367531,        0.367531,        0.735063,        0.367531,
    //         0.286388,        0.286388,        0.286388,        0.286388,        0.572775,        0.367531,        0.367531,        0.367531,        0.367531,        0.735063
    //     ]);

    //     // static double F1[]={
    // // -13324188.000000,-13324188.000000,
    // // -10668901.000000,-10668908.000000,
    // //  -7157236.000000, -7157236.000000,
    // //  -6149379.000000, -6149379.000000,
    // //  -7454143.000000, -7454143.000000,
    // //  -5969220.000000, -5969220.000000,
    // //   8336726.000000,  8336717.000000,
    // //   6186960.000000,  6186960.000000,
    // // -17549108.000000,-17549108.000000,
    // // -13970171.000000,-13970171.000000
    //     //   };

    //     let s_2 = DVector::<f64>::from_row_slice(&[
    //          1506.435789,     1612.811795
    //     ]);

    //     let ndf = 10;
    //     let m = 2;

    //     lambda_search(ndf, m, a, q, f, s);

    //     for i in 0..ndf {
    //         for j in 0..ndf {
    //             assert!((f[(i, j)] - f2[(i, j)].abs()) < 1.0E-4, "lambda-search test#2 failed");
    //         }

    //         assert!((s[i] - s2[i]).abs() < 1.0E-4, "lambda-search test#2 failed");
    //     }
    // }
}
