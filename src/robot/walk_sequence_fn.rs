use core::fmt::Debug;
use crate::math::{ FloatType as float, Vector2, Vector3 };
use super::{ functions };
use log::{ info };

#[derive(Debug, Clone)]
struct StepConfig {
    step: Vector3,
    step_height_weight: float
}

#[derive(Debug, Clone)]
pub struct WalkSequenceFn {
    x: float,
    step_start: Vector3,
    step_cfg: StepConfig,
    step_update1: Option<StepConfig>,
    step_update2: Option<StepConfig>,
    offset: float,
    state: u32
}

impl WalkSequenceFn {
    pub fn new(step: &Vector2, step_height_weight: float, offset: float) -> Self {
        let x = 0.0;
        let step_start = Vector3::new(-step[0]/2.0, -step[1]/2.0, 0.0);
        let step_cfg = StepConfig{ step: Vector3::new(step[0], step[1], 0.0), step_height_weight };
        let state = 0;

        debug_assert!(offset >= 0.0 && offset < 2.0);

        WalkSequenceFn{ x, step_start, step_cfg, step_update1: Option::None, step_update2: Option::None, offset, state }
    }

    pub fn update(&mut self, step: &Vector2, step_height_weight: float) {
        let step_update = StepConfig{ step: Vector3::new(step[0], step[1], 0.0), step_height_weight };
        let step_pos = self.get();

        let mut cycle_pos = (self.x + 0.5 - self.offset) % 2.0;
        if cycle_pos > 1.0 {
            cycle_pos -= 1.0;
            let a = &self.step_cfg.step;
            let b = &step_update.step;
            self.step_start = -b/2.0 + (a-b)/2.0 * (cycle_pos - 0.5);
            self.step_cfg.step = step_update.step.clone();
            self.state = 1;
        }
        else {
            self.state = 2;
        }

        self.step_update1 = Option::Some(step_update);
    }

    pub fn advance(&mut self, x: float) {
        let cycle_pos_prev = (self.x + 0.5 - self.offset) % 2.0;
        let cycle_pos_curr = (x + 0.5 - self.offset) % 2.0;

        if self.state == 1 {
            let step_update = self.step_update1.as_ref().unwrap();

            if cycle_pos_prev > cycle_pos_curr {
                self.step_cfg.step = &step_update.step - &self.step_start;
                self.state = 0;
            }
        }
        else if self.state == 2 {
            if cycle_pos_prev <= 1.0 && cycle_pos_curr > 1.0 {
                let step_update = self.step_update1.as_ref().unwrap(); // TODO: remove this mess
                let step = &step_update.step;
                self.step_start = Vector3::new(-step[0]/2.0, -step[1]/2.0, 0.0);
                self.step_cfg.step = step.clone();
                self.state = 0;
            }
        }

        self.x = x;
    }

    pub fn dist(&self) -> float {
        self.step_cfg.step.len()
    }

    pub fn get(&self) -> Vector3 {
        let mut x = self.x;
        let step_height = self.step_cfg.step.len() * self.step_cfg.step_height_weight;

        if self.offset < 1.0 {
            let c1 = self.offset / 2.0;
            let c2 = 0.5 + self.offset;

            if x >= 0.0 && x < c1 {
                let step_end = -c1 * &self.step_cfg.step;
                functions::linear_step(x / c1, &step_end)
            }
            else if x >= c1 && x < c2 {
                let step_start = -c1 * &self.step_cfg.step;
                let step_end = (c2 - c1) * &self.step_cfg.step;
                step_start + functions::quad_step((x - c1) / (c2 - c1), &step_end, step_height)
            }
            else {
                x = (x - c2) % 2.0;
                if x < 1.0 {
                    let step_end = -&self.step_cfg.step;
                    -&self.step_start + functions::linear_step(x, &step_end)
                }
                else {
                    let step_end = &self.step_cfg.step;
                    &self.step_start + functions::quad_step(x - 1.0, step_end, step_height)
                }
            }
        }
        else {
            let c1 = -0.5 + self.offset / 2.0;
            let c2 = -0.5 + self.offset;

            if x >= 0.0 && x < c1 {
                let step_end = c1 * &self.step_cfg.step;
                functions::quad_step(x / c1, &step_end, step_height)
            }
            else if x >= c1 && x < c2 {
                let step_start = c1 * &self.step_cfg.step;
                let step_end = -(c2 - c1) * &self.step_cfg.step;
                step_start + functions::linear_step((x - c1) / (c2 - c1), &step_end)
            }
            else {
                let x = (x - c2) % 2.0;
                if x < 1.0 {
                    let step_end = &self.step_cfg.step;
                    &self.step_start + functions::quad_step(x, step_end, step_height)
                }
                else {
                    let step_end = -&self.step_cfg.step;
                    -&self.step_start + functions::linear_step(x - 1.0, &step_end)
                }
            }
        }
    }
}
