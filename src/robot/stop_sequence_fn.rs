use core::fmt::Debug;
use crate::math::{ FloatType as float, Vector3 };
use super::{ functions };


#[derive(Debug, Clone)]
pub struct StopSequenceFn {
    x: float,
    target_pos: Vector3,
    step_height: float
}

impl StopSequenceFn {
    pub fn new(target_pos: &Vector3, step_height: float) -> Self {
        let step_height = if target_pos[2] > 0.0 { target_pos[2] + step_height } else { step_height };
        StopSequenceFn{ x: 0.0, target_pos: target_pos.clone(), step_height }
    }

    pub fn dist(&self) -> float {
        self.target_pos.len()
    }

    pub fn advance(&mut self, x: float) {
        self.x = x / self.target_pos.len();
    }

    pub fn get(&self) -> Vector3 {
        if self.x < 0.0 {
            Vector3::new(0.0, 0.0, 0.0)
        }
        else if self.x > 1.0 {
            self.target_pos.clone()
        }
        else {
            let vertical_pos = functions::quad_step_height(self.x, self.target_pos[2], self.step_height);
            Vector3::new(self.target_pos[0] * self.x, self.target_pos[1] * self.x, vertical_pos)
        }
    }

    pub fn has_finished(&self) -> bool {
        self.x >= 1.0
    }
}
