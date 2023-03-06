use core::fmt::Debug;
use crate::math::{ FloatType as float, Vector2, Vector3 };
use super::{ functions };
use log::{ info };

#[derive(Debug, Clone)]
struct StepConfig {
    step: Vector3,
    step_height: float
}

#[derive(Debug, Clone)]
pub struct WalkSequenceFn {
    x: float,
    step_start: Vector3,
    step_cfg: StepConfig,
    step_update1: StepConfig,
    step_update2: StepConfig,
    offset: float,
    state: u32
}

impl WalkSequenceFn {
    pub fn new(step: &Vector2, step_height: float, offset: float) -> Self {
        let x = 0.0;
        let step_start = Vector3::new(-step[0]/2.0, -step[1]/2.0, 0.0);
        let step_cfg = StepConfig{ step: Vector3::new(step[0], step[1], 0.0), step_height };
        let step_update1 = StepConfig{ step: Vector3::zero(), step_height: 0.0 };
        let step_update2 = StepConfig{ step: Vector3::zero(), step_height: 0.0 };
        let state = 0;

        debug_assert!(offset >= 0.0 && offset < 2.0);

        WalkSequenceFn{ x, step_start, step_cfg, step_update1, step_update2, offset, state }
    }

    pub fn update(&mut self, step: &Vector2, step_height: float) {
        let step_update = StepConfig{ step: Vector3::new(step[0], step[1], 0.0), step_height };
        if self.state == 0 {
            self.step_update1 = step_update;
            self.state = 1;
        }
        else {
            self.step_update2 = step_update;
        }
    }

    pub fn advance(&mut self, x: float) {
        if self.state == 1 {
            let switch_point = 1.5 + self.offset;

            if (self.x <= switch_point && x >= switch_point) || (self.x > x && self.offset == 0.0) {
                self.step_cfg.step = &self.step_update1.step / 2.0 - &self.step_start;
                self.step_cfg.step_height = self.step_update1.step_height;
                self.state = 2;
            }
        }
        else if self.state == 2 {
            let switch_point = if self.offset < 1.0 {
                2.5 + self.offset
            }
            else {
                0.5 + self.offset
            };


            if (self.x <= switch_point && x >= switch_point) || (self.x > x && self.offset == 1.0) {
                let step = self.step_update1.step.clone();
                self.step_start = Vector3::new(-step[0]/2.0, -step[1]/2.0, 0.0);
                self.step_cfg.step = step;

                if self.step_update2.step_height > 0.0 { // TODO
                    self.step_update1 = self.step_update2.clone();
                    self.step_update2 = StepConfig{ step: Vector3::zero(), step_height: 0.0 };
                    self.state = 1;
                }
                else {
                    self.state = 0;
                }
            }
        }

        self.x = x;
    }

    pub fn dist(&self) -> float {
        self.step_cfg.step.len()
    }

    pub fn get(&self) -> Vector3 {
        // Map x so that [0, step_len] -> [0, 2]
        let mut x = self.x;


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
                step_start + functions::quad_step((x - c1) / (c2 - c1), &step_end, self.step_cfg.step_height)
            }
            else {
                x = (x - c2) % 2.0;
                if x < 1.0 {
                    let step_end = -&self.step_cfg.step;
                    -&self.step_start + functions::linear_step(x, &step_end)
                }
                else {
                    let step_end = &self.step_cfg.step;
                    &self.step_start + functions::quad_step(x - 1.0, step_end, self.step_cfg.step_height)
                }
            }
        }
        else {
            let c1 = -0.5 + self.offset / 2.0;
            let c2 = -0.5 + self.offset;

            if x >= 0.0 && x < c1 {
                let step_end = c1 * &self.step_cfg.step;
                functions::quad_step(x / c1, &step_end, self.step_cfg.step_height)
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
                    &self.step_start + functions::quad_step(x, step_end, self.step_cfg.step_height)
                }
                else {
                    let step_end = -&self.step_cfg.step;
                    -&self.step_start + functions::linear_step(x - 1.0, &step_end)
                }
            }
        }
    }
}
