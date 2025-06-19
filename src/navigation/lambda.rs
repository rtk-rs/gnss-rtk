use nalgebra::{allocator::Allocator, DVector, DefaultAllocator, DimName, OMatrix, OVector};

use crate::prelude::SV;

#[derive(Debug, Default, Clone, PartialEq)]
struct Ambiguity {
    pub sv: SV,
    pub n_int: u64,
}

#[derive(Default)]
pub struct LambdaAR {
    pub n_int: DVector<Ambiguity>,
}

impl LambdaAR {
    const MAX_SEARCH: usize = 1_000;

    /// Reset this [LambdaAR]
    pub fn reset(&mut self) {
        let ndf = self.n_int.nrows();
        // self.n_int.set(0.0);
    }

    /// Reset this particular [SV]
    pub fn reset_sv(&mut self, sv: SV) {}

    // pub fn resolve(&mut self, x: &DVector<f64>, q: &DMatrix<f64>, candidates: &[Candidate], indexes: &Vec<usize>) {

    //     let mut id = Vec::new();
    //     for i in indexes.iter() {
    //         id.push(candidates[*i].sv);
    //     }
    //
    //     const offset : usize = Navigation::clock_index() +1;
    //     let ndf = x.nrows() - offset;

    //     let mut a = DVector::<f64>::zeros(ndf);
    //     let mut q = DMatrix::<f64>::zeros(ndf, ndf);
    //     let mut q_diag = DVector::<f64>::zeros(ndf);

    //     for i in 0..ndf {
    //         a[i] = x[i + offset];
    //         for j in 0..ndf {
    //             // q[(i, j)] = self.p_mat[(i + Self::clock_index(), i + Self::clock_index() +j)];
    //         }
    //     }
    //
    //     debug!("search - ndf={} - A={} Q={}", ndf, a, q);

    //     let l = q.lower_triangle();
    //     let lt = l.transpose();
    //     let q_a = lt * q_diag * l;

    // }

    fn gauss_transform<D: DimName>(
        i: usize,
        j: usize,
        ndf: usize,
        l_mat: &mut OMatrix<f64, D, D>,
        z_mat: &mut OMatrix<f64, D, D>,
        a_vec: &mut OVector<f64, D>,
    ) where
        DefaultAllocator: Allocator<D>,
        DefaultAllocator: Allocator<D, D>,
    {
        let mu = l_mat[(i, j)].floor() + 0.5;

        if mu != 0.0 {
            for k in i..ndf {
                l_mat[(k, j)] -= mu * l_mat[(k, i)];
            }

            for k in 0..ndf {
                z_mat[(k, j)] -= mu * z_mat[(k, i)];
            }

            a_vec[j] -= mu * a_vec[i];
        }
    }

    // fn permutations<D: DimName>(
    //     i: usize
    //     l_mat: &mut OMatrix<f64, D, D>,
    //     d_mat: &mut OMatrix<f64, D, D>,
    //     z_mat: &mut OMatrix<f64, D, D>,
    //     delta: f64,
    // ) where
    //     DefaultAllocator: Allocator<D>,
    //     DefaultAllocator: Allocator<D, D>,
    // {
    //     let mut a0;

    //     let eta = d_mat[(i, i)];
    //     let lambda = d_mat[(i +1, i+1)] * l_mat[(i +1, i)] / delta;

    //     d[(i, i)] = eta * d[(i +1, i+1)];
    //     d[(i +1, i+1)] = delta;

    //     for j in j.. {
    //         // a0 = l_mat[j + k*n];
    //         // a1 = l_mat[j +1+k*n];
    //         // l_mat[j + k*n] -= a0 * l_mat[j +1 +j *n] + a1;
    //         // l_mat[j +1 + k*n] = a0 * eta + lambda * a1;
    //     }

    //     l_mat[j +1 +j*n] = lamda;

    //     // swap_columns(&mut l_mat, j+2..n);
    //     // for k in j+2..n {
    //     //     // swap l_mat[k+j*n], l_mat[k+(j+1]*n)
    //     //     // swap z_mat[k+j*n], z_mat[k+(j+1]*n)
    //     // }

    // }

    // pub fn search<D: DimName>(ndf: usize, d_mat: OMatrix<f64, D, D>)
    // where
    //     DefaultAllocator: Allocator<D>,
    //     DefaultAllocator: Allocator<D, D>,

    // {

    //     const MAX_DIST : f64 = 1.0E99;
    //     let mut i = 0;

    //     let k = ndf - 1;

    //     loop {

    //         if i == Self::MAX_SEARCH {
    //             error!("decorrelation failed (nth={}}", i);
    //             break;
    //         }

    //         for i in 0..Self::MAX_SEARCH {
    //
    //             let newdist = dist[k] + y * y/ d_mat[k];

    //             if newdist < MAX_DIST {
    //                 // case (1): move down
    //                 if k != 0 {
    //                     k -= 1;
    //                     dist[k] = newdist;
    //
    //                     for i in 0..k {
    //                         s[k +i*n] = s[k +1 +i *ndf] + (z[k+1] - zb[k+1]) * l_mat[k +1 +i*ndf];
    //                     }

    //                     zb[k] = zs[k] + s[k + k*ndf];
    //                     z[k] = zb[k].floor() + 0.5;
    //                     y = zb[k] - z[k];
    //                     step[k] = sign(y);
    //                 }
    //             } else {
    //                 // case (2): exit or move up

    //             }

    //         }

    //         // sort by s
    //         for i in 0..m-1 {
    //             for j in i+1..m {
    //                 if s[i] < s[j] {
    //                     continue;
    //                 }

    //                 // SWAP s[i], s[j]
    //                 for k in 0..n {
    //                     // SWAP(zn[k+1*n], zn[k +j*n])
    //                 }
    //             }
    //         }
    //     }
    // }
}

#[cfg(test)]
mod test {

    use super::LambdaAR;

    use nalgebra::{DVector, DimName, OMatrix, OVector, U4};

    #[test]
    fn gauss_transform() {
        let mut l_mat = OMatrix::<f64, U4, U4>::identity();

        let mut z_mat = l_mat.clone();

        let mut a = OVector::<f64, U4>::from_row_slice(&[
            1585184.171,
            -6716599.430,
            3915742.905,
            7627233.455,
            9565990.879,
            989457273.200,
        ]);

        LambdaAR::gauss_transform::<U4>(0, 0, U4::USIZE, &mut l_mat, &mut z_mat, &mut a);
    }

    // #[test]
    // fn mlambda_search_1() {
    //     let a = DVector::<f64>::from_row_slice(&[
    //         1585184.171,
    //        -6716599.430,
    //         3915742.905,
    //         7627233.455,
    //         9565990.879,
    //       989457273.200
    //     ]);

    //     let q = DMatrix::<f64>::from(&[
    //         0.227134,   0.112202,   0.112202,   0.112202,   0.112202,   0.103473,
    //         0.112202,   0.227134,   0.112202,   0.112202,   0.112202,   0.103473,
    //         0.112202,   0.112202,   0.227134,   0.112202,   0.112202,   0.103473,
    //         0.112202,   0.112202,   0.112202,   0.227134,   0.112202,   0.103473,
    //         0.112202,   0.112202,   0.112202,   0.112202,   0.227134,   0.103473,
    //         0.103473,   0.103473,   0.103473,   0.103473,   0.103473,   0.434339
    //     ]);

    //     // static double F1[]={
    //     //     1585184.000000,  1585184.000000,
    //     //    -6716599.000000, -6716600.000000,
    //     //     3915743.000000,  3915743.000000,
    //     //     7627234.000000,  7627233.000000,
    //     //     9565991.000000,  9565991.000000,
    //     //   989457273.000000,989457273.000000
    //     //   };

    //     let s_1 = DVector::<f64>::from_row_slice(&[
    //         3.507984,        3.708456,
    //     ]);

    //     let ndf = 6;
    //     let m = 2;

    //     lambda_search(ndf, m, a, q, f, s);

    //     for i in 0..ndf {
    //         for j in 0..ndf {
    //             assert!((f[(i, j)] - f1[(i, j)].abs()) < 1.0E-4, "lambda-search test#1 failed");
    //         }

    //         assert!((s[i] - s1[i]).abs() < 1.0E-4, "lambda-search test#1 failed");
    //     }
    // }

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
