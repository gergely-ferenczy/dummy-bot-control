use core::fmt::Debug;
use crate::math::{ FloatType as float, Vector3 };
use super::{ Sequence, SequenceFn, functions };


#[derive(Debug, Clone)]
pub struct WalkSequenceFn {
    pos: Vector3,
    step_height: float,
    offset: float
}

impl WalkSequenceFn {
    pub fn new(pos: Vector3, step_height: float, offset: float) -> Self {
        let step_height = if pos[2] - step_height > 0.0 { pos[2] + step_height } else { step_height };
        WalkSequenceFn{ pos, step_height, offset }
    }
}

impl SequenceFn for WalkSequenceFn {
    fn dist(&self) -> float {
        self.pos.len()
    }

    fn get(&self, x: float) -> Vector3 {
         // Map x so that [0, step_len] -> [0, 1]
        let x = if x > 0.0 { x / self.pos.len() } else { x };

        debug_assert!(self.offset >= 0.0 && self.offset < 2.0);
        debug_assert!(x >= 0.0);

        if self.offset >= 0.0 && self.offset < 1.0 {
            let c1 = self.offset / 2.0;
            let c2 = 0.5 + self.offset;

            if x >= 0.0 && x < c1 {
                let step_end = -c1 * &self.pos;
                functions::linear_step(x / c1, &step_end)
            }
            else if x >= c1 && x < c2 {
                let step_start = -c1 * &self.pos;
                let step_end = (c2 - c1) * &self.pos;
                step_start + functions::quad_step((x - c1) / (c2 - c1), &step_end, self.step_height)
            }
            else {
                let x = (x - c2) % 2.0;
                if x < 1.0 {
                    let step_start = 0.5 * &self.pos;
                    let step_end = -&self.pos;
                    step_start + functions::linear_step(x, &step_end)
                }
                else {
                    let step_start = -0.5 * &self.pos;
                    let step_end = &self.pos;
                    step_start + functions::quad_step(x - 1.0, step_end, self.step_height)
                }
            }
        }
        else {
            let c1 = -0.5 + self.offset / 2.0;
            let c2 = -0.5 + self.offset;

            if x >= 0.0 && x < c1 {
                let step_end = c1 * &self.pos;
                functions::quad_step(x / c1, &step_end, self.step_height)
            }
            else if x >= c1 && x < c2 {
                let step_start = c1 * &self.pos;
                let step_end = -(c2 - c1) * &self.pos;
                step_start + functions::linear_step((x - c1) / (c2 - c1), &step_end)
            }
            else {
                let x = (x - c2) % 2.0;
                if x < 1.0 {
                    let step_start = -0.5 * &self.pos;
                    let step_end = &self.pos;
                    step_start + functions::quad_step(x, step_end, self.step_height)
                }
                else {
                    let step_start = 0.5 * &self.pos;
                    let step_end = -&self.pos;
                    step_start + functions::linear_step(x - 1.0, &step_end)
                }
            }
        }
    }

    fn has_finished(&self, pos: float) -> bool {
        let x = if pos != 0.0 { pos / self.pos.len() - self.offset } else { pos };
        x >= 1.0
    }
}

#[derive(Debug, Clone)]
pub struct WalkSequence {
    x: float,
    seqs: [WalkSequenceFn; 6]
}

impl WalkSequence {

    /// Creates a new `WalkSequence` with an independent sequence for each leg.
    pub fn new(seqs: [WalkSequenceFn; 6]) -> Self {
        WalkSequence{
            x: 0.0,
            seqs: seqs
        }
    }
}

impl Sequence for WalkSequence {
    fn advance(&mut self, speed: float, time: u32) {
        self.x += speed * (time as f32) / 1000.0;
    }

    fn pos(&self, leg_id: usize) -> Vector3 {
        self.seqs[leg_id].get(self.x)
    }
}
