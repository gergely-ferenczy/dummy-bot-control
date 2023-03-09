use core::fmt::Debug;

use crate::math::{ FloatType as float, Vector2, Vector3 };
use super::{ WalkSequenceFn };

#[derive(Debug, Clone)]
struct StepUpdate {
    step: Vector2,
    step_height_weight: float
}

#[derive(Debug, Clone)]
pub struct WalkSequence {
    x: float,
    sequence_fns: [WalkSequenceFn; 6],
    step_update: Option<StepUpdate>
}

impl WalkSequence {

    pub fn new(step: &Vector2, step_height_weight: float) -> Self {
        // let seq_fn1 = WalkSequenceFn::new(step, step_height, 0.0);
        // let seq_fn2 = WalkSequenceFn::new(step, step_height, 1.0);
        // let seq_fn3 = WalkSequenceFn::new(step, step_height, 0.0);
        // let seq_fn4 = WalkSequenceFn::new(step, step_height, 1.0);
        // let seq_fn5 = WalkSequenceFn::new(step, step_height, 0.0);
        // let seq_fn6 = WalkSequenceFn::new(step, step_height, 1.0);
        let seq_fn1 = WalkSequenceFn::new(1, step, step_height_weight, (1.0 / 3.0) * 0.0);
        let seq_fn2 = WalkSequenceFn::new(2, step, step_height_weight, (1.0 / 3.0) * 4.0);
        let seq_fn3 = WalkSequenceFn::new(3, step, step_height_weight, (1.0 / 3.0) * 2.0);
        let seq_fn4 = WalkSequenceFn::new(4, step, step_height_weight, (1.0 / 3.0) * 3.0);
        let seq_fn5 = WalkSequenceFn::new(5, step, step_height_weight, (1.0 / 3.0) * 1.0);
        let seq_fn6 = WalkSequenceFn::new(6, step, step_height_weight, (1.0 / 3.0) * 5.0);

        WalkSequence{ x: 0.0, sequence_fns: [seq_fn1, seq_fn2, seq_fn3, seq_fn4, seq_fn5, seq_fn6], step_update: None }
    }

    pub fn update(&mut self, step: &Vector2, step_height_weight: float) {
        let mut scaling_required = false;
        let mut min_scale = 1.0;
        for i in 0..6 {
            if let Err(scale) = self.sequence_fns[i].update(step, step_height_weight) {
                if min_scale > scale {
                    min_scale = scale;
                    scaling_required = true;
                }
            }
        }
        if scaling_required {
            let smaller_step = step * min_scale;
            for i in 0..6 {
                if let Err(scale) = self.sequence_fns[i].update(&smaller_step, step_height_weight) {
                    panic!("Step downscaling failed: scale={} min_scale={}", scale, min_scale);
                }
            }
            self.step_update = Some(StepUpdate { step: step.clone(), step_height_weight });
        }
    }

    /// Advances the sequence based on the provided parameters.
    ///
    /// `speed` must be given in m/s and `time` must be given in ms.
    pub fn advance(&mut self, speed: float, time: u32) {
        let distance = speed * (time as float) / 1000.0;
        let step_len = self.sequence_fns.iter()
            .map(|x| x.dist())
            .min_by(|x, y| x.partial_cmp(y).unwrap()).unwrap();

        self.x += 2.0 * distance / step_len;

        if self.x > 3.5 {
            self.x -=  2.0;
        }

        if self.step_update.is_some() {
            let step_update = self.step_update.as_ref().unwrap();
            self.update(&step_update.step.clone(), step_update.step_height_weight);
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
