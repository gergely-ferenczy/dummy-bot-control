use crate::math::{ transform, FloatType as float, Vector3, Matrix3 };
use super::{ Leg, Sequence };

#[derive(Debug)]
pub struct Hexapod {
    legs: [Leg; 6],
    legs_origin: [Vector3; 6],
    legs_end_pos: [Vector3; 6],
    legs_seq_pos: [Vector3; 6],
    body_offset: Vector3,
    body_rot: Matrix3,
    body_rot_origin: Vector3,
    seqs: Vec<Sequence>,
    speed: float
}

impl Hexapod {
    fn update_legs(&mut self) {
        for i in 0..6 {
            /* 1) Rotate the leg by the body rotation around the body rotation origin point.
             *  a) Translate to the rotation origin.
             *  b) Apply the rotation matrix.
             *  c) Translate back to original position.
             * 2) Translate by the body offset.
             * 3) Translate by the leg origin position. */
            let mut pos = &self.body_rot * (&self.legs_end_pos[i] - &self.body_rot_origin) + &self.body_rot_origin;
            pos = pos - &self.body_offset - &self.legs_origin[i] + &self.legs_seq_pos[i];
            self.legs[i].set_position(&pos);
        }
    }

    pub fn new(legs: [Leg; 6], legs_origin: [Vector3; 6], legs_end_pos: [Vector3; 6]) -> Self {
        let mut res = Self{
            legs: legs,
            legs_origin: legs_origin,
            legs_end_pos: legs_end_pos,
            legs_seq_pos: [Vector3::zero(), Vector3::zero(), Vector3::zero(), Vector3::zero(), Vector3::zero(), Vector3::zero()],
            body_offset: Vector3::zero(),
            body_rot: Matrix3::identity(),
            body_rot_origin: Vector3::zero(),
            seqs: vec![],
            speed: 1.0
        };
        res.update_legs();

        res
    }

    pub fn set_body_offset(&mut self, v: &Vector3) {
        self.body_offset = v.clone();
        self.update_legs();
    }

    pub fn set_body_rotation(&mut self, angle: float, axis: &Vector3, origin: &Vector3) {
        self.body_rot = transform::rotate_matrix(angle, axis);
        self.body_rot_origin = origin.clone();
        self.update_legs();
    }

    pub fn advance_sequences(&mut self, time: u32) {
        for i in 0..6 {
            self.legs_seq_pos[i] = Vector3::zero();
        }
        self.seqs.retain_mut(|seq| {
            seq.advance(self.speed, time);
            if seq.has_finished() {
                for i in 0..6 {
                    self.legs_end_pos[i] += seq.get_leg_pos(i);
                }
                false
            }
            else {
                for i in 0..6 {
                    self.legs_seq_pos[i] += seq.get_leg_pos(i);
                }
                true
            }
        });
        self.update_legs();
    }

    pub fn start_seq(&mut self, seq: Sequence) {
        self.seqs.push(seq);
    }

    pub fn stop_seq(&mut self, seq: Sequence) {
        self.seqs.push(seq);
    }

    pub fn set_speed(&mut self, speed: float) {
        self.speed = speed;
    }

    pub fn leg(&self, leg_id: usize) -> &Leg {
        &self.legs[leg_id]
    }

    pub fn leg_origin(&self, leg_id: usize) -> &Vector3 {
        &self.legs_origin[leg_id]
    }
}
