use nalgebra::{allocator::Allocator, DefaultAllocator, DimName, OVector};

#[derive(Clone, Copy)]
pub struct StateCorrection<D: DimName>
where
    DefaultAllocator: Allocator<D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
{
    pub dx: OVector<f64, D>,
}

impl<D: DimName> Default for StateCorrection<D>
where
    DefaultAllocator: Allocator<D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
{
    fn default() -> Self {
        Self {
            dx: OVector::<f64, D>::zeros(),
        }
    }
}
