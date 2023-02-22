use core::fmt::Debug;
use crate::math::{ FloatType as float, Vector3 };
use super::{ Sequence, SequenceFn };


#[derive(Debug, Clone)]
pub struct LegPosMoveFn {
    pos: Vector3,
    step_height: float,
    offset: float
}

impl LegPosMoveFn {
    pub fn new(pos: Vector3, step_height: float, offset: float) -> Self {
        let step_height = if pos[2] - step_height > 0.0 { pos[2] + step_height } else { step_height };
        LegPosMoveFn{ pos, step_height, offset }
    }
}

impl SequenceFn for LegPosMoveFn {
    fn dist(&self) -> float {
        self.pos.len()
    }

    fn get(&self, pos: float) -> Vector3 {
         // Map x to a relative position between 0.0 and 1.0
        let x = if pos != 0.0 { pos / self.pos.len() - self.offset } else { pos };

        if x < 0.0 {
            Vector3::new(0.0, 0.0, 0.0)
        }
        else if x > 1.0 {
            self.pos.clone()
        }
        else {
            let h = self.step_height;
            let s = self.pos[2];
            let b = 2.0*h + (4.0*h*h - 4.0*h*s).sqrt();
            let a = s - b;
            let z = a*x*x + b*x;
            Vector3::new(self.pos[0] * x, self.pos[1] * x, z)
        }
    }

    fn has_finished(&self, pos: float) -> bool {
        let x = if pos != 0.0 { pos / self.pos.len() - self.offset } else { pos };
        x >= 1.0
    }
}

#[derive(Debug, Clone)]
pub struct LegPosMove {
    x: float,
    seqs: [LegPosMoveFn; 6]
}

impl LegPosMove {

    /// Creates a new `LegPosMove` with an independent sequence for each leg.
    pub fn new(seqs: [LegPosMoveFn; 6]) -> Self {
        LegPosMove{
            x: 0.0,
            seqs: seqs
        }
    }
}

impl Sequence for LegPosMove {
    fn advance(&mut self, speed: float, time: u32) {
        self.x += speed * (time as f32) / 1000.0;
    }

    fn pos(&self, leg_id: usize) -> Vector3 {
        self.seqs[leg_id].get(self.x)
    }

    fn has_finished(&self) -> bool {
        for i in 0..6 {
            if !self.seqs[i].has_finished(self.x) {
                return false
            }
        }
        return true
    }
}
