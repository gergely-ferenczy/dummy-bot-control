use core::fmt::Debug;
use crate::math::{ FloatType as float, Vector2, Vector3 };
use super::{ SequenceFn, functions };
use log::{ info };


#[derive(Debug, Clone)]
pub struct WalkSequenceFn {
    x: float,
    step: Vector3,
    step_height: float,
    offset: float,

    step_update: Vector3,
    step_height_update: float
}

impl WalkSequenceFn {
    pub fn new(step: &Vector2, step_height: float, offset: float) -> Self {
        let x = 0.0;
        let step = Vector3::new(step[0], step[1], 0.0);

        let step_update = step.clone();
        let step_height_update = step_height;

        WalkSequenceFn{ x, step, step_height, offset, step_update, step_height_update }
    }

    pub fn update(&mut self, step: &Vector2, step_height: float) {
        self.step_update = Vector3::new(step[0], step[1], 0.0);
        self.step_height_update = step_height;
    }
}

impl SequenceFn for WalkSequenceFn {

    fn advance(&mut self, x: float) {
        if self.step != self.step_update || self.step_height != self.step_height_update {
            let c2 = if self.offset >= 0.0 && self.offset < 1.0 {
                0.5 + self.offset
            }
            else {
                -0.5 + self.offset
            };
            info!("{} {} {}", self.x, x, c2);

            if self.x <= c2 && x >= c2 {
                self.step = self.step_update.clone();
                self.step_height = self.step_height_update;
            }
        }

        self.x = x;
    }

    fn dist(&self) -> float {
        self.step.len()
    }

    fn get(&self) -> Vector3 {
        // Map x so that [0, step_len] -> [0, 2]
        let mut x: float = if self.x > 0.0 { 2.0 * self.x / self.step.len() } else { 0.0 };

        debug_assert!(self.offset >= 0.0 && self.offset < 2.0);

        if self.offset >= 0.0 && self.offset < 1.0 {
            let c1 = self.offset / 2.0;
            let c2 = 0.5 + self.offset;

            if x >= 0.0 && x < c1 {
                let step_end = -c1 * &self.step;
                functions::linear_step(x / c1, &step_end)
            }
            else if x >= c1 && x < c2 {
                let step_start = -c1 * &self.step;
                let step_end = (c2 - c1) * &self.step;
                step_start + functions::quad_step((x - c1) / (c2 - c1), &step_end, self.step_height)
            }
            else {
                x = (x - c2) % 2.0;
                if x < 1.0 {
                    let step_start = 0.5 * &self.step;
                    let step_end = -&self.step;
                    step_start + functions::linear_step(x, &step_end)
                }
                else {
                    let step_start = -0.5 * &self.step;
                    let step_end = &self.step;
                    step_start + functions::quad_step(x - 1.0, step_end, self.step_height)
                }
            }
        }
        else {
            let c1 = -0.5 + self.offset / 2.0;
            let c2 = -0.5 + self.offset;

            if x >= 0.0 && x < c1 {
                let step_end = c1 * &self.step;
                functions::quad_step(x / c1, &step_end, self.step_height)
            }
            else if x >= c1 && x < c2 {
                let step_start = c1 * &self.step;
                let step_end = -(c2 - c1) * &self.step;
                step_start + functions::linear_step((x - c1) / (c2 - c1), &step_end)
            }
            else {
                let x = (x - c2) % 2.0;
                if x < 1.0 {
                    let step_start = -0.5 * &self.step;
                    let step_end = &self.step;
                    step_start + functions::quad_step(x, step_end, self.step_height)
                }
                else {
                    let step_start = 0.5 * &self.step;
                    let step_end = -&self.step;
                    step_start + functions::linear_step(x - 1.0, &step_end)
                }
            }
        }
    }

    fn has_finished(&self) -> bool {
        false
    }
}
