pub mod solutions;
pub use solutions::{
    //InvalidationCause,
    PVTSolution,
    PVTSolutionType,
};

// mod filter;
// mod input;
// mod output;

use nalgebra::{
    allocator::Allocator, base::dimension::U4, Const, DefaultAllocator, DimName, OMatrix, OVector,
};

// pub(crate) use input::Input;
// pub(crate) use output::Output;

// pub use filter::Filter;
// pub(crate) use filter::FilterState;

use log::error;

use crate::prelude::{
    Candidate,
    Config,
    Duration,
    Error,
    IonosphereBias, //Method,
    Orbit,
};

/// SV Navigation information
#[derive(Debug, Clone, Default)]
pub struct SVInput {
    /// Possible [Orbit] state
    pub orbit: Option<Orbit>,
    /// Elevation from RX position
    pub elevation: f64,
    /// Azimuth from RX position
    pub azimuth: f64,
    /// Troposphere bias in meters of delay
    pub tropo_bias: Option<f64>,
    /// Ionosphere bias
    pub iono_bias: Option<IonosphereBias>,
    /// Correction to said constellation, expressed as [Duration]
    pub clock_correction: Option<Duration>,
}

#[derive(Debug, Clone)]
pub(crate) struct Navigation<N>
where
    N: DimName,
    DefaultAllocator: Allocator<N>,
    DefaultAllocator: Allocator<Const<4>, N>,
{
    b: OVector<f64, N>,
    h: OMatrix<f64, U4, N>,
    pub iter: usize,
    pub state: OMatrix<f64, U4, U4>,
}

impl<N> Navigation<N>
where
    DefaultAllocator: Allocator<N>,
    DefaultAllocator: Allocator<Const<4>, N>,
    N: DimName,
{
    /// Create new [Navigation] filter
    /// ## Input
    ///
    pub fn new(cfg: &Config, appriori: &Orbit, candidates: &[Candidate]) -> Result<Self, Error> {
        const MIN_SIZE: usize = 4;

        let size = candidates.len();

        let pos_vel_m = appriori.to_cartesian_pos_vel() * 1.0E3;
        let apriori_m = (pos_vel_m[0], pos_vel_m[1], pos_vel_m[2]);

        let mut b = OVector::<f64, N>::zeros();
        let mut h = OMatrix::<f64, U4, N>::zeros();

        match cfg.solution {
            PVTSolutionType::PositionVelocityTime => {
                if size < U4::USIZE {
                    return Err(Error::MatrixMinimalDimension);
                }
                if size < N::USIZE {
                    return Err(Error::MatrixDimension);
                }
            },
            PVTSolutionType::TimeOnly => {
                if size < 1 {
                    return Err(Error::MatrixMinimalDimension);
                }
            },
        }

        let mut j = 0;

        for cd in candidates.iter() {
            match cd.spp_matrix_contribution(j, cfg, &mut b, &mut h, apriori_m) {
                Ok(_) => {
                    j += 1;
                },
                Err(e) => {
                    error!("{}({}): spp_matrix_contribution: {}", cd.t, cd.sv, e);
                },
            }
        }
        Ok(Self {
            b,
            h,
            iter: 0,
            state: OMatrix::<f64, U4, U4>::zeros(),
        })
    }

    /// Iterates this [Navigation] filter, updating provided state
    pub fn iter(&mut self) -> Result<(), Error> {
        self.iter += 1;
        Ok(())
    }
}
