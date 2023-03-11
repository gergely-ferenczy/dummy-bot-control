use core::fmt::Debug;
use crate::{math::{ FloatType as float, FloatEq, Vector2, Vector3 }, float_eq};
use super::{ functions };

#[derive(Debug, Clone)]
struct StepConfig {
    step: Vector3,
    step_height_weight: float,
    max_step_len: float
}

#[derive(Debug, Clone)]
pub struct WalkSequenceFn {
    x: float,
    id: u32,
    step_start: Vector3,
    step_cfg: StepConfig,
    step_update1: Option<StepConfig>,
    step_update2: Option<StepConfig>,
    offset: float,
    state: u32 // TODO: replace this with an enum.
}

impl WalkSequenceFn {
    pub fn new(id: u32, step: &Vector2, step_height_weight: float, max_step_len: float, offset: float) -> Self {
        let x = 0.0;
        let step_start = Vector3::new(-step[0]/2.0, -step[1]/2.0, 0.0);
        let step_cfg = StepConfig{ step: Vector3::new(step[0], step[1], 0.0), step_height_weight, max_step_len };
        let state = 0;

        debug_assert!(offset >= 0.0 && offset < 2.0);

        WalkSequenceFn{ id, x, step_start, step_cfg, step_update1: Option::None, step_update2: Option::None, offset, state }
    }

    pub fn update(&mut self, step: &Vector2, step_height_weight: float, max_step_len: float) -> Result<(), float> {
        let step_update = StepConfig{ step: Vector3::new(step[0], step[1], 0.0), step_height_weight, max_step_len };
        let a = &self.step_cfg.step;
        let a_end = &self.step_start + a;
        let b = &step_update.step;
        let qa = b[0]*b[0] + b[1]*b[1];
        let qb = -2.0 * (a_end[0] * b[0] + a_end[1] * b[1]);
        let qc = a_end[0]*a_end[0] + a_end[1]*a_end[1] - (max_step_len*max_step_len) / 4.0;
        let scale = (-qb + (qb*qb - 4.0*qa*qc).sqrt()) / (2.0 * qa);

        if scale > 1.0 || float_eq!(scale, 1.0, 1e-5, abs) {
            let mut cycle_pos = (self.x + 0.5 - self.offset) % 2.0;
            if cycle_pos > 1.0 {
                cycle_pos -= 1.0;

                let step_start = &self.step_start + (a-b) * (1.0 - cycle_pos);
                self.step_start = step_start;
                self.step_cfg = step_update.clone();
                self.state = 1;
            }
            else {
                self.state = 3;
            }
            self.step_update1 = Option::Some(step_update);
            Ok(())
        }
        else {
            Err(scale)
        }
    }

    pub fn advance(&mut self, x: float) {
        let cycle_pos_prev = (self.x + 0.5 - self.offset) % 2.0;
        let cycle_pos_curr = (x + 0.5 - self.offset) % 2.0;

        if self.state == 1 {
            if cycle_pos_prev > cycle_pos_curr {
                let step_update = self.step_update1.as_ref().unwrap();
                self.step_start = self.get();
                self.step_cfg.step = &step_update.step / 2.0 - &self.step_start;
                self.step_cfg.step_height_weight = step_update.step_height_weight;
                self.state = 2;
            }
        }
        else if self.state == 2 {
            if cycle_pos_prev <= 1.0 && cycle_pos_curr > 1.0 {
                self.step_cfg = self.step_update1.take().unwrap();
                self.step_start = &self.step_cfg.step / -2.0;
                self.state = 0;
            }
        }
        else if self.state == 3 {
            if cycle_pos_prev <= 1.0 && cycle_pos_curr > 1.0 {
                let step_update = self.step_update1.as_ref().unwrap();
                self.step_start = &self.step_start + &self.step_cfg.step - &step_update.step;
                self.step_cfg = step_update.clone();
                self.state = 1;
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
                    &self.step_start + functions::linear_step(1.0 - x, &self.step_cfg.step)
                }
                else {
                    &self.step_start + functions::quad_step(x - 1.0, &self.step_cfg.step, step_height)
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
                    &self.step_start + functions::quad_step(x, &self.step_cfg.step, step_height)
                }
                else {
                    &self.step_start + functions::linear_step(1.0 - (x - 1.0), &self.step_cfg.step)
                }
            }
        }
    }
}
