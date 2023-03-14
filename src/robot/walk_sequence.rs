use core::fmt::Debug;

use crate::math::{ FloatType as float, Vector2, Vector3 };
use super::{ WalkSequenceFn };

#[derive(Debug, Clone)]
struct StepUpdate {
    step: Vector2,
    turn_origin: [Vector2; 6],
    turn_angle: float,
    step_height_weight: float,
    max_step_radius: float
}

#[derive(Debug, Clone)]
pub struct WalkSequence {
    x: float,
    sequence_fns: [WalkSequenceFn; 6],
    step_update: Option<StepUpdate>,
    lift_ratio: float
}

impl WalkSequence {

    pub fn new(step: &Vector2, turn_origin: &[Vector2; 6], turn_angle: float, step_height_weight: float, lift_ratio: float) -> Self {
        let seq_fn1 = WalkSequenceFn::new(1, &step, &turn_origin[0], turn_angle, step_height_weight, (1.0 / 6.0) * 0.0, lift_ratio);
        let seq_fn2 = WalkSequenceFn::new(2, &step, &turn_origin[1], turn_angle, step_height_weight, (1.0 / 6.0) * 4.0, lift_ratio);
        let seq_fn3 = WalkSequenceFn::new(3, &step, &turn_origin[2], turn_angle, step_height_weight, (1.0 / 6.0) * 2.0, lift_ratio);
        let seq_fn4 = WalkSequenceFn::new(4, &step, &turn_origin[3], turn_angle, step_height_weight, (1.0 / 6.0) * 3.0, lift_ratio);
        let seq_fn5 = WalkSequenceFn::new(5, &step, &turn_origin[4], turn_angle, step_height_weight, (1.0 / 6.0) * 1.0, lift_ratio);
        let seq_fn6 = WalkSequenceFn::new(6, &step, &turn_origin[5], turn_angle, step_height_weight, (1.0 / 6.0) * 5.0, lift_ratio);
        // let seq_fn1 = WalkSequenceFn::new(1, &step, &turn_origin[0], turn_angle, step_height_weight, 0.0, lift_ratio);
        // let seq_fn2 = WalkSequenceFn::new(2, &step, &turn_origin[1], turn_angle, step_height_weight, 0.5, lift_ratio);
        // let seq_fn3 = WalkSequenceFn::new(3, &step, &turn_origin[2], turn_angle, step_height_weight, 0.0, lift_ratio);
        // let seq_fn4 = WalkSequenceFn::new(4, &step, &turn_origin[3], turn_angle, step_height_weight, 0.5, lift_ratio);
        // let seq_fn5 = WalkSequenceFn::new(5, &step, &turn_origin[4], turn_angle, step_height_weight, 0.0, lift_ratio);
        // let seq_fn6 = WalkSequenceFn::new(6, &step, &turn_origin[5], turn_angle, step_height_weight, 0.5, lift_ratio);

        WalkSequence{
            x: 0.0,
            sequence_fns: [seq_fn1, seq_fn2, seq_fn3, seq_fn4, seq_fn5, seq_fn6],
            step_update: None,
            lift_ratio
        }
    }

    pub fn update(&mut self, step: &Vector2, turn_origin: &[Vector2; 6], turn_angle: float, step_height_weight: float, max_step_radius: float) {
        let mut scaling_required = false;
        let mut min_scale = 1.0;

        for i in 0..6 {
            if let Err(scale) = self.sequence_fns[i].update(&step, &turn_origin[i], turn_angle, step_height_weight, max_step_radius) {
                if min_scale > scale {
                    min_scale = scale;
                    scaling_required = true;
                }
            }
        }

        if scaling_required {
            let step_scaled = step * min_scale;
            let turn_angle_scaled = turn_angle * min_scale;

            for i in 0..6 {
                if let Err(scale) = self.sequence_fns[i].update(&step_scaled, &turn_origin[i], turn_angle_scaled, step_height_weight, max_step_radius) {
                    println!("{:?}", self.sequence_fns[i]);
                    panic!("Step downscaling failed: scale={} min_scale={} {}", scale, min_scale, i);
                }
            }
            self.step_update = Some(StepUpdate{
                step: step.clone(),
                turn_origin: turn_origin.clone(),
                turn_angle,
                step_height_weight,
                max_step_radius
            });
        }
        else {
            self.step_update = None;
        }
    }

    /// Advances the sequence based on the provided parameters.
    ///
    /// `speed` must be given in m/s and `time` must be given in ms.
    pub fn advance(&mut self, speed: float, max_speed: float, time: u32) {
        let time = (time as float) / 1000.0;
        let distance = speed * time;
        let max_distance = max_speed * time;

        let step_len_max = self.sequence_fns.iter()
            .map(|x| x.dist())
            .max_by(|x, y| x.partial_cmp(y).unwrap()).unwrap();
        let step_len_avg =  self.sequence_fns.iter()
            .map(|x| x.dist())
            .fold(0.0, |acc, x| acc + x ) / 6.0;

        let seq_dist_avg = distance / step_len_avg;
        let seq_dist_max = max_distance / step_len_max;
        if seq_dist_avg > seq_dist_max {
            self.x += seq_dist_max;
        }
        else {
            self.x += seq_dist_avg;
        }


        if self.x > 3.0 {
            self.x -=  1.0;
        }

        for seq_fn in self.sequence_fns.iter_mut() {
            seq_fn.advance(self.x);
        }

        if let Some(step_update) = &self.step_update {
            self.update(&step_update.step.clone(), &step_update.turn_origin.clone(), step_update.turn_angle,
                step_update.step_height_weight, step_update.max_step_radius);
        }
    }

    /// TODO
    pub fn get_leg_pos(&self, leg_id: usize) -> Vector3 {
        self.sequence_fns[leg_id].get()
    }
}
