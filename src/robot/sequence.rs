use core::fmt::Debug;
use crate::math::{ FloatType as float, Vector3 };

pub trait SequenceFn: Debug + Clone {

    /// TODO
    fn dist(&self) -> float;

    /// TODO
    fn get(&self, pos: float) -> Vector3;

    /// TODO
    fn has_finished(&self, pos: float) -> bool;
}

pub trait Sequence: Debug {

    /// Advances the sequence based on the provided parameters.
    ///
    /// `speed` must be given in m/s and `time` must be given in ms.
    fn advance(&mut self, speed: float, time: u32);

    /// TODO
    fn pos(&self, leg_id: usize) -> Vector3;

    /// TODO
    fn has_finished(&self) -> bool {
        false
    }
}