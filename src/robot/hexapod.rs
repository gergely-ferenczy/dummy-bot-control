use crate::math::{ transform, FloatType as float, Vector2, Vector3, Matrix3 };
use super::{ Leg, WalkSequence, StopSequence, WalkSequenceConfig };


#[derive(Debug)]
pub struct HexapodConfig {
    pub leg_len1: float,
    pub leg_len2: float,
    pub joint_offset: [Vector3; 6],
    pub legs_origin: [Vector3; 6],
    pub legs_end_pos: [Vector3; 6],
    pub max_speed: float,
    pub max_step_radius: float,
    pub max_move_radius: float,
    pub max_step_len: float,
    pub max_turn_angle: float,
    pub max_body_offset: Vector3,
    pub max_body_rotation: float
}

#[derive(Debug)]
struct BodyRotation {
    angle: float,
    axis: Vector3,
    origin: Vector3,
    matrix: Matrix3
}

#[derive(Debug)]
struct BodyPosition {
    offset: Vector3,
    rotation: BodyRotation
}

impl BodyPosition {
    pub fn new() -> Self {
        Self {
            offset: Vector3::zero(),
            rotation: BodyRotation {
                angle: 0.0,
                axis: Vector3::new(0.0, 0.0, 1.0),
                origin: Vector3::zero(),
                matrix: Matrix3::identity(),
            }
        }
    }
}


#[derive(Debug)]
pub struct Hexapod {
    config: HexapodConfig,
    legs: [Leg; 6],
    legs_origin: [Vector3; 6],
    legs_end_pos: [Vector3; 6],
    legs_seq_pos: [Vector3; 6],
    body_pos: BodyPosition,
    body_pos_target: BodyPosition,
    walk_sequence: Option<WalkSequence>,
    stop_sequence: Option<StopSequence>,
    speed: float
}

impl Hexapod {
    fn update_legs(&mut self) {
        for i in 0..6 {
            let pos = &self.calc_leg_static_pos(i) + &self.legs_seq_pos[i];
            let plane_normal = &self.body_pos.rotation.matrix * Vector3::new(0.0, 0.0, 1.0);
            self.legs[i].set_position(&pos, &plane_normal);
        }
    }

    pub fn new(config: HexapodConfig) -> Self {
        let legs = [
            Leg::new(config.leg_len1, config.leg_len2, config.joint_offset[0].clone()),
            Leg::new(config.leg_len1, config.leg_len2, config.joint_offset[1].clone()),
            Leg::new(config.leg_len1, config.leg_len2, config.joint_offset[2].clone()),
            Leg::new(config.leg_len1, config.leg_len2, config.joint_offset[3].clone()),
            Leg::new(config.leg_len1, config.leg_len2, config.joint_offset[4].clone()),
            Leg::new(config.leg_len1, config.leg_len2, config.joint_offset[5].clone())
        ];

        let leg_origin_default = config.legs_origin.clone();
        let leg_end_pos_default = config.legs_end_pos.clone();

        let mut res = Self{
            config: config,
            legs: legs,
            legs_origin: leg_origin_default,
            legs_end_pos: leg_end_pos_default,
            legs_seq_pos: [Vector3::zero(), Vector3::zero(), Vector3::zero(), Vector3::zero(), Vector3::zero(), Vector3::zero()],
            body_pos: BodyPosition::new(),
            body_pos_target: BodyPosition::new(),
            walk_sequence: None,
            stop_sequence: None,
            speed: 0.0
        };
        res.update_legs();

        return res
    }

    fn calc_leg_origin(&self, id: usize) -> Vector3 {
        &self.body_pos.rotation.matrix * (&self.config.legs_origin[id] - &self.body_pos.rotation.origin)  + &self.body_pos.rotation.origin + &self.body_pos.offset
    }

    fn calc_leg_static_pos(&self, id: usize) -> Vector3 {
        &self.legs_end_pos[id] - &self.legs_origin[id]
    }

    fn move_vector_towards(current: &mut Vector3, target: &Vector3, distance: float) {
        let target_distance = target - (current as &_);
        if target_distance.len() > 0.0 {
            *current += target_distance.norm() * distance;
            let new_target_distance = target - (current as &_);

            if target_distance[0] >= 0.0 && new_target_distance[0] < 0.0 || target_distance[0] < 0.0 && new_target_distance[0] >= 0.0 ||
                target_distance[1] >= 0.0 && new_target_distance[1] < 0.0 || target_distance[1] < 0.0 && new_target_distance[1] >= 0.0 ||
                target_distance[2] >= 0.0 && new_target_distance[2] < 0.0 || target_distance[2] < 0.0 && new_target_distance[2] >= 0.0 {
                    *current = target.clone();
            }
        }
    }

    pub fn set_body_offset(&mut self, v: &Vector3) {
        self.body_pos_target.offset = Vector3::new(
            v[0] * &self.config.max_body_offset[0],
            v[1] * &self.config.max_body_offset[1],
            v[2] * &self.config.max_body_offset[2]
        );
    }

    pub fn set_body_rotation(&mut self, angle: float, axis: &Vector3, origin: &Vector3) {
        if axis.len() > 0.0 {
            self.body_pos_target.rotation.axis = axis.clone();
        }
        else {
            self.body_pos_target.rotation.axis = Vector3::new(0.0, 0.0, 1.0);
        }

        self.body_pos_target.rotation.angle = angle * self.config.max_body_rotation;
        self.body_pos_target.rotation.origin = origin.clone();
    }

    pub fn update(&mut self, time: u32) {
        for i in 0..6 {
            self.legs_seq_pos[i] = Vector3::zero();
        }

        if let Some(stop_sequence) = &mut self.stop_sequence {
            stop_sequence.advance(self.speed, time);
            if stop_sequence.has_finished() {
                for i in 0..6 {
                    self.legs_end_pos[i] = self.config.legs_end_pos[i].clone();
                }
                self.stop_sequence = Option::None;
            }
            else {
                for i in 0..6 {
                    self.legs_seq_pos[i] += stop_sequence.get_leg_pos(i);
                }
            }
        }
        else if let Some(walk_sequence) = &mut self.walk_sequence {
            walk_sequence.advance(self.speed, time);
            for i in 0..6 {
                self.legs_seq_pos[i] += walk_sequence.get_leg_pos(i);
            }
        }

        // TODO: a touch more KISS and DRY and all good stuff would be great
        // here. Also calculation documentation before I forget what this thing does.
        let mut static_position_updated = false;
        let time = (time as float) / 1000.0;
        let move_increment = self.config.max_speed * time;

        if self.body_pos.offset != self.body_pos_target.offset {
            Self::move_vector_towards(&mut self.body_pos.offset, &self.body_pos_target.offset, move_increment);
            static_position_updated = true;
        }

        let current_rotation = &mut self.body_pos.rotation;
        let target_rotation = &self.body_pos_target.rotation;
        if current_rotation.origin != target_rotation.origin {
            Self::move_vector_towards(&mut current_rotation.origin, &target_rotation.origin, move_increment);
            current_rotation.matrix = transform::rotate_matrix3(current_rotation.angle, &current_rotation.axis);
            static_position_updated = true;
        }

        if current_rotation.angle != target_rotation.angle ||
            (target_rotation.angle > 0.0 && current_rotation.axis != target_rotation.axis)
        {
            let angle_increment = move_increment / (self.config.max_move_radius / 2.0);
            let rm1 = transform::rotate_matrix3(current_rotation.angle, &current_rotation.axis);
            let rm2 = transform::rotate_matrix3(target_rotation.angle, &target_rotation.axis);
            let v0 = current_rotation.axis.cross(&target_rotation.axis);
            let v1 = if v0.len() == 0.0 {
                Vector3::new(current_rotation.axis[2], current_rotation.axis[0], current_rotation.axis[1])
            } else {
                v0
            };
            let v2 = &rm1 * &v1;
            let v3 = &rm2 * &v1;
            let axis_between = v2.cross(&v3);
            let angle_between = v2.angle(&v3);

            if angle_between < angle_increment {
                current_rotation.angle = target_rotation.angle;
                current_rotation.axis = target_rotation.axis.clone();
                current_rotation.matrix = transform::rotate_matrix3(current_rotation.angle, &current_rotation.axis);
            } else {
                let rm3 = transform::rotate_matrix3(angle_increment, &axis_between);
                let rm4 = rm3 * rm1;
                let v4 = &rm4 * &v1;
                let angle = v1.angle(&v4);
                let axis = v1.cross(&v4);
                current_rotation.angle = angle;
                current_rotation.axis = axis.norm();
                current_rotation.matrix = rm4.clone();
            }

            static_position_updated = true;
        }

        if static_position_updated {
            for i in 0..6 {
                self.legs_origin[i] = self.calc_leg_origin(i);
            }

            let leg_static_pos = [
                self.calc_leg_static_pos(0), self.calc_leg_static_pos(1), self.calc_leg_static_pos(2),
                self.calc_leg_static_pos(3), self.calc_leg_static_pos(4), self.calc_leg_static_pos(5)
            ];

            if let Some(walk_sequence) = &mut self.walk_sequence {
                walk_sequence.update_leg_static_pos(leg_static_pos);
            }
        }

        self.update_legs();
    }

    pub fn set_speed(&mut self, speed: float) {
        self.speed = speed.min(1.0).max(0.0) * self.config.max_speed;
    }

    pub fn set_step(&mut self, step: &Vector2, turn: float, step_height_weight: float) {
        // TODO: handle out-of-range values.
        let step = if step.len() > 1.0 { step.norm() } else { step.clone() };
        let step_len = step.len();
        let step_scaled = if step.len() > 0.0 { step.norm() * (0.5 + 0.5 * step.len()) * self.config.max_step_len } else { step };
        let turn = turn.clamp(-1.0, 1.0);
        let turn_angle = if turn != 0.0 { turn / turn.abs() * (0.5 + 0.5 * turn.abs()) * self.config.max_turn_angle } else { 0.0 };

        let mut turn_origin = [Vector2::zero(), Vector2::zero(), Vector2::zero(), Vector2::zero(), Vector2::zero(), Vector2::zero()];

        for i in 0..6 {
            turn_origin[i] = -Vector2::from(&self.config.legs_end_pos[i]);
        }

        let step_height_weight = step_height_weight.clamp(0.0, 2.0) / 2.0;

        let leg_static_pos = [
            self.calc_leg_static_pos(0), self.calc_leg_static_pos(1), self.calc_leg_static_pos(2),
            self.calc_leg_static_pos(3), self.calc_leg_static_pos(4), self.calc_leg_static_pos(5)
        ];

        if step_len > 0.0 || turn_angle != 0.0 {

            let config = WalkSequenceConfig{
                leg_static_pos,
                step: step_scaled,
                turn_origin,
                turn_angle,
                step_height_weight,
                max_step_radius: self.config.max_step_radius,
                max_move_radius: self.config.max_move_radius,
                lift_ratio: 0.3
            };

            if let Some(walk_sequence) = &mut self.walk_sequence {
                walk_sequence.update(&config);
            }
            else {
                let seq = WalkSequence::new(&config);
                self.walk_sequence = Some(seq);
            }

            self.set_speed(Vector2::new(step_len, turn).len().min(1.0));
        }
        else if let Some(walk_sequence) = &self.walk_sequence {
            let mut delays = [false; 6];
            for i in 0..6 {
                let seq_pos = walk_sequence.get_leg_pos(i);
                self.legs_end_pos[i] += &seq_pos;

                // TODO: maybe there is a better way to decide if the leg is in the air or not.
                // If the leg is currently touching the ground, delay the move until the other
                // legs are finished moving.
                if seq_pos[2] == 0.0 {
                    delays [i] = true;
                }
            }
            let positions = [
                &self.config.legs_end_pos[0] - &self.legs_end_pos[0],
                &self.config.legs_end_pos[1] - &self.legs_end_pos[1],
                &self.config.legs_end_pos[2] - &self.legs_end_pos[2],
                &self.config.legs_end_pos[3] - &self.legs_end_pos[3],
                &self.config.legs_end_pos[4] - &self.legs_end_pos[4],
                &self.config.legs_end_pos[5] - &self.legs_end_pos[5]
            ];

            self.stop_sequence = Option::Some(StopSequence::new(positions, step_height_weight, delays));
            self.walk_sequence = Option::None;

            self.set_speed(0.5);
        }
    }

    pub fn leg(&self, leg_id: usize) -> &Leg {
        &self.legs[leg_id]
    }

    pub fn leg_origin(&self, leg_id: usize) -> &Vector3 {
        &self.legs_origin[leg_id]
    }
}
