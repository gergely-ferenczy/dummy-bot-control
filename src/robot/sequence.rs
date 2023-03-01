use core::fmt::Debug;
use crate::math::{ FloatType as float, Vector3 };

pub trait SequenceFn: Debug {

    /// TODO
    fn advance(&mut self, speed: float, time: u32);

    /// TODO
    fn dist(&self) -> float;

    /// TODO
    fn get(&self) -> Vector3;

    /// TODO
    fn has_finished(&self) -> bool;
}

#[derive(Debug)]
pub struct Sequence {
    seqs: Vec<Box<dyn SequenceFn>>
}

impl Sequence {

    pub fn new<T: SequenceFn + 'static>(sequence_fns: [T; 6]) -> Self {
        let mut result = Sequence{ seqs: vec![] };
        for seq_fn in sequence_fns {
            result.seqs.push(Box::new(seq_fn));
        }

        result
    }

    /// Advances the sequence based on the provided parameters.
    ///
    /// `speed` must be given in m/s and `time` must be given in ms.
    pub fn advance(&mut self, speed: float, time: u32) {
        for seq_fn in self.seqs.iter_mut() {
            seq_fn.advance(speed, time);
        }
    }

    /// TODO
    pub fn get_leg_pos(&self, leg_id: usize) -> Vector3 {
        self.seqs[leg_id].get()
    }

    /// TODO
    pub fn has_finished(&self) -> bool {
        for i in 0..6 {
            if !self.seqs[i].has_finished() {
                return false
            }
        }
        return true
    }
}
