use core::fmt::Debug;

use crate::math::{ FloatType as float, Vector2, Vector3 };
use super::{ WalkSequenceFn };
use log::{ info };

#[derive(Debug)]
pub struct Sequence {
    x: float,
    step: Vector2,
    step_height: float,
    sequence_fns: [WalkSequenceFn; 6]
}

impl Sequence {

    pub fn new(step: &Vector2, step_height: float) -> Self {
        // let seq_fn1 = WalkSequenceFn::new(step, step_height, 0.0);
        // let seq_fn2 = WalkSequenceFn::new(step, step_height, 1.0);
        // let seq_fn3 = WalkSequenceFn::new(step, step_height, 0.0);
        // let seq_fn4 = WalkSequenceFn::new(step, step_height, 1.0);
        // let seq_fn5 = WalkSequenceFn::new(step, step_height, 0.0);
        // let seq_fn6 = WalkSequenceFn::new(step, step_height, 1.0);
        let seq_fn1 = WalkSequenceFn::new(step, step_height, (1.0 / 3.0) * 0.0);
        let seq_fn2 = WalkSequenceFn::new(step, step_height, (1.0 / 3.0) * 4.0);
        let seq_fn3 = WalkSequenceFn::new(step, step_height, (1.0 / 3.0) * 2.0);
        let seq_fn4 = WalkSequenceFn::new(step, step_height, (1.0 / 3.0) * 3.0);
        let seq_fn5 = WalkSequenceFn::new(step, step_height, (1.0 / 3.0) * 1.0);
        let seq_fn6 = WalkSequenceFn::new(step, step_height, (1.0 / 3.0) * 5.0);
        
        Sequence{ x: 0.0, step: step.clone(), step_height: step_height, 
            sequence_fns: [seq_fn1, seq_fn2, seq_fn3, seq_fn4, seq_fn5, seq_fn6]}
    }

    pub fn update(&mut self, step: &Vector2, step_height: float) {
        self.step = step.clone();
        self.step_height = step_height;
        for i in 0..6 {
            self.sequence_fns[i].update(step, step_height);
        }
    }

    /// Advances the sequence based on the provided parameters.
    ///
    /// `speed` must be given in m/s and `time` must be given in ms.
    pub fn advance(&mut self, speed: float, time: u32) {
        let distance = speed * (time as float) / 1000.0;
        self.x += 2.0 * distance / self.step.len();

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
}
