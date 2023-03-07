use core::fmt::Debug;

use crate::math::{ FloatType as float, Vector2, Vector3 };
use super::{ WalkSequenceFn };

#[derive(Debug)]
pub struct WalkSequence {
    x: float,
    sequence_fns: [WalkSequenceFn; 6]
}

impl WalkSequence {

    pub fn new(step: &Vector2, step_height_weight: float) -> Self {
        // let seq_fn1 = WalkSequenceFn::new(step, step_height, 0.0);
        // let seq_fn2 = WalkSequenceFn::new(step, step_height, 1.0);
        // let seq_fn3 = WalkSequenceFn::new(step, step_height, 0.0);
        // let seq_fn4 = WalkSequenceFn::new(step, step_height, 1.0);
        // let seq_fn5 = WalkSequenceFn::new(step, step_height, 0.0);
        // let seq_fn6 = WalkSequenceFn::new(step, step_height, 1.0);
        let seq_fn1 = WalkSequenceFn::new(step, step_height_weight, (1.0 / 3.0) * 0.0);
        let seq_fn2 = WalkSequenceFn::new(step, step_height_weight, (1.0 / 3.0) * 4.0);
        let seq_fn3 = WalkSequenceFn::new(step, step_height_weight, (1.0 / 3.0) * 2.0);
        let seq_fn4 = WalkSequenceFn::new(step, step_height_weight, (1.0 / 3.0) * 3.0);
        let seq_fn5 = WalkSequenceFn::new(step, step_height_weight, (1.0 / 3.0) * 1.0);
        let seq_fn6 = WalkSequenceFn::new(step, step_height_weight, (1.0 / 3.0) * 5.0);

        WalkSequence{ x: 0.0, sequence_fns: [seq_fn1, seq_fn2, seq_fn3, seq_fn4, seq_fn5, seq_fn6]}
    }

    pub fn update(&mut self, step: &Vector2, step_height_weight: float) {
        for i in 0..6 {
            self.sequence_fns[i].update(step, step_height_weight);
        }
    }

    /// Advances the sequence based on the provided parameters.
    ///
    /// `speed` must be given in m/s and `time` must be given in ms.
    pub fn advance(&mut self, speed: float, time: u32) {
        let distance = speed * (time as float) / 1000.0;
        let step_len = self.sequence_fns.iter()
            .map(|x| x.dist())
            .max_by(|x, y| x.partial_cmp(y).unwrap()).unwrap();

        self.x += 2.0 * distance / step_len;

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
