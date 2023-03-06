use core::fmt::Debug;
use crate::math::{ FloatType as float, Vector2, Vector3 };
use super::{ SequenceFn, WalkSequenceFn };

#[derive(Debug)]
pub struct Sequence {
    x: float,
    sequence_fns: [WalkSequenceFn; 6]
}

impl Sequence {

    pub fn new(sequence_fns: [WalkSequenceFn; 6]) -> Self {
        Sequence{ x: 0.0, sequence_fns: sequence_fns}
    }

    pub fn update(&mut self, step: &Vector2, step_height: float) {
        for i in 0..6 {
            self.sequence_fns[i].update(step, step_height);
        }
    }

    /// Advances the sequence based on the provided parameters.
    ///
    /// `speed` must be given in m/s and `time` must be given in ms.
    pub fn advance(&mut self, speed: float, time: u32) {

        self.x += speed * (time as float) / 1000.0;
        if self.x > 3.5 {
            self.x -=  2.0;
        }

        for seq_fn in self.sequence_fns.iter_mut() {
            seq_fn.advance(self.x);
        }
    }

    /// TODO
    pub fn get_leg_pos(&self, leg_id: usize) -> Vector3 {
        self.sequence_fns[leg_id].get()
    }

    /// TODO
    pub fn has_finished(&self) -> bool {
        for i in 0..6 {
            if !self.sequence_fns[i].has_finished() {
                return false
            }
        }
        return true
    }
}
