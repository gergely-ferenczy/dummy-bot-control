use core::fmt::Debug;

use crate::math::{ FloatType as float, Vector2, Vector3 };
use super::{ WalkSequenceFn, WalkSequencePhase };

#[derive(Debug, Clone)]
pub struct WalkSequenceConfig {
    pub leg_static_pos: [Vector3; 6],
    pub step: Vector2,
    pub turn_origin: [Vector2; 6],
    pub turn_angle: float,
    pub step_height_weight: float,
    pub max_step_radius: float,
    pub max_move_radius: float,
    pub lift_ratio: float
}

#[derive(Debug, Clone)]
pub struct WalkSequence {
    x: float,
    sequence_fns: [WalkSequenceFn; 6],
    config_active: Option<WalkSequenceConfig>, // TODO: Revisit this, Option may not be necessary.
    config_update: Option<WalkSequenceConfig>
}

impl WalkSequence {

    pub fn new(config: &WalkSequenceConfig) -> Self {

        let seq_fn1 = WalkSequenceFn::new(1, (1.0 / 6.0) * 0.0, config.lift_ratio);
        let seq_fn2 = WalkSequenceFn::new(2, (1.0 / 6.0) * 4.0, config.lift_ratio);
        let seq_fn3 = WalkSequenceFn::new(3, (1.0 / 6.0) * 2.0, config.lift_ratio);
        let seq_fn4 = WalkSequenceFn::new(4, (1.0 / 6.0) * 3.0, config.lift_ratio);
        let seq_fn5 = WalkSequenceFn::new(5, (1.0 / 6.0) * 1.0, config.lift_ratio);
        let seq_fn6 = WalkSequenceFn::new(6, (1.0 / 6.0) * 5.0, config.lift_ratio);

        let mut walk_sequence = WalkSequence{
            x: 0.0,
            sequence_fns: [seq_fn1, seq_fn2, seq_fn3, seq_fn4, seq_fn5, seq_fn6],
            config_active: None,
            config_update: None
        };

        walk_sequence.update(config);

        return walk_sequence;
    }

    pub fn update(&mut self, config: &WalkSequenceConfig) {
        let mut scaling_required = false;
        let mut min_scale = 1.0;

        let (leg_static_pos, step, turn_origin, turn_angle, step_height_weight, max_step_radius, max_move_radius) = (
            &config.leg_static_pos,
            &config.step,
            &config.turn_origin,
            config.turn_angle,
            config.step_height_weight,
            config.max_step_radius,
            config.max_move_radius
        );

        for i in 0..6 {
            if let Err(scale) = self.sequence_fns[i].update(step, &turn_origin[i], turn_angle, step_height_weight,
                    max_step_radius, max_move_radius, &leg_static_pos[i], false) {
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
                let _ = self.sequence_fns[i].update(&step_scaled, &turn_origin[i], turn_angle_scaled,
                    step_height_weight, max_step_radius, max_move_radius, &leg_static_pos[i], true);
            }
            let mut config_active = config.clone();
            config_active.step = step_scaled;
            config_active.turn_angle = turn_angle_scaled;
            self.config_active = Some(config_active);
            self.config_update = Some(config.clone());
        }
        else {
            self.config_active = Some(config.clone());
            self.config_update = None;
        }
    }

    pub fn update_leg_static_pos(&mut self, leg_static_pos: [Vector3; 6]) {
        if let Some(config_update) = &mut self.config_update {
            config_update.leg_static_pos = leg_static_pos;
        }
        else if let Some(config_active) = &self.config_active {
            let mut config_update = config_active.clone();
            config_update.leg_static_pos = leg_static_pos;
            self.config_update = Some(config_update);
        }
    }

    /// Advances the sequence based on the provided parameters.
    ///
    /// `speed` must be given in m/s and `time` must be given in ms.
    pub fn advance(&mut self, speed: float, time: u32) {
        let time = (time as float) / 1000.0;
        let distance = speed * time;

        let first_push_step = self.sequence_fns.iter().find(|s| s.phase() == WalkSequencePhase::Push);
        let step_dist = match first_push_step {
            Some(s) => s.dist(),
            None => 0.0,
        };
        let lr = self.config_active.as_ref().unwrap().lift_ratio;
        let sequence_dist = (distance / step_dist) * (1.0 - lr);

        self.x += sequence_dist;

        if self.x > 3.0 {
            self.x -=  1.0;
        }

        for seq_fn in self.sequence_fns.iter_mut() {
            seq_fn.advance(self.x);
        }

        if let Some(config_update) = self.config_update.take() {
            self.update(&config_update);
        }
    }

    /// TODO
    pub fn get_leg_pos(&self, leg_id: usize) -> Vector3 {
        self.sequence_fns[leg_id].get()
    }
}
