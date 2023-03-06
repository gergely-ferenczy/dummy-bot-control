use core::fmt::Debug;
use crate::math::{ Vector3, FloatType as float };

pub trait SequenceFn: Debug {

    /// TODO
    fn advance(&mut self, x: float);

    /// TODO
    fn dist(&self) -> float;

    /// TODO
    fn get(&self) -> Vector3;

    /// TODO
    fn has_finished(&self) -> bool;
}
