use core::fmt::Debug;
use crate::math::{ FloatType as float, Vector3 };
use super::{ functions };


#[derive(Debug, Clone)]
pub struct StopSequenceFn {
    x: float,
    position: Vector3,
    step_height: float
}

impl StopSequenceFn {
    pub fn new(position: &Vector3, step_height: float) -> Self {
        let step_height = if position[2] > 0.0 { position[2] + step_height } else { step_height };
        StopSequenceFn{ x: 0.0, position: position.clone(), step_height }
    }

    pub fn dist(&self) -> float {
        self.position.len()
    }

    pub fn advance(&mut self, x: float) {
        self.x = x / self.position.len();
    }

    pub fn get(&self) -> Vector3 {
        if self.x < 0.0 {
            Vector3::new(0.0, 0.0, 0.0)
        }
        else if self.x > 1.0 {
            self.position.clone()
        }
        else {
            functions::quad_step(self.x, &self.position, self.step_height)
        }
    }

    pub fn has_finished(&self) -> bool {
        self.x >= 1.0
    }
}
