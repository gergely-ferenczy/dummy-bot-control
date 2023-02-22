use std::fmt::Debug;
use crate::math::{ Matrix3 };
use crate::math::{ transform, Circle, FloatType as float, Vector3 };

#[derive(Debug)]
pub struct Leg {
    len_a: float,
    len_b: float,
    joint_offset: Vector3,
    position: Vector3
}

impl Leg {
    pub fn new(len_a: float, len_b: float, joint_offset: Vector3) -> Self {
        Leg{ len_a: len_a, len_b: len_b, joint_offset: joint_offset, position: Vector3::zero() }
    }

    pub fn calc_joint_angles(&self) -> (float, float, float) {
        let pos_zproj = Vector3::new(self.position[0], self.position[1], 0.0);
        let mut a0 = pos_zproj.angle(&Vector3::new(1.0, 0.0, 0.0));
        if pos_zproj[1] < 0.0 {
            a0 *= -1.0;
        }
        let rel_pos = &self.position - &self.joint_offset;
        let rel_pos_zproj = Vector3::new(rel_pos[0], rel_pos[1], 0.0);
        let mut a0_rel = rel_pos_zproj.angle(&Vector3::new(1.0, 0.0, 0.0));
        if rel_pos_zproj[1] < 0.0 {
            a0_rel *= -1.0;
        }
        let rm = transform::rotate_matrix(-a0_rel, &Vector3::new(0.0, 0.0, 1.0));
        let pos_turned = &rm * &rel_pos;

        let c1 = Circle::new(0.0, 0.0, self.len_a);
        let c2 = Circle::new(pos_turned[0], pos_turned[2], self.len_b);
        let (_, intp) = c1.intersection_points(&c2);

        let (px, py) = (intp.x, intp.y);
        let v1 = Vector3::new(px, 0.0, py);
        let v1neg = -&v1;
        let v2 = Vector3::new(pos_turned[0]-px, 0.0, pos_turned[2]-py);
        let mut a1 = v1.angle(&Vector3::new(1.0, 0.0, 0.0));
        let a2 = v2.angle(&v1neg);

        if intp.y < 0.0 {
            a1 *= -1.0;
        }

        (a0, a1, a2)
    }

    pub fn set_position(&mut self, pos: &Vector3) {
        self.position = pos.clone();
    }

    pub fn position(&self) -> &Vector3 {
        &self.position
    }
}

trait SequenceFn: Debug + Clone {

    /// TODO
    fn dist(&self) -> float;

    /// TODO
    fn get(&self, pos: float) -> Vector3;

    /// TODO
    fn has_finished(&self, pos: float) -> bool;
}

#[derive(Debug, Clone)]
pub struct LegPosMoveFn {
    pos: Vector3,
    step_height: float,
    offset: float
}

impl LegPosMoveFn {
    pub fn new(pos: Vector3, step_height: float, offset: float) -> Self {
        let step_height = if pos[2] - step_height > 0.0 { pos[2] + step_height } else { step_height };
        LegPosMoveFn{ pos, step_height, offset }
    }
}

impl SequenceFn for LegPosMoveFn {
    fn dist(&self) -> float {
        self.pos.len()
    }

    fn get(&self, pos: float) -> Vector3 {
         // Map x to a relative position between 0.0 and 1.0
        let x = if pos != 0.0 { pos / self.pos.len() - self.offset } else { pos };

        if x < 0.0 {
            Vector3::new(0.0, 0.0, 0.0)
        }
        else if x > 1.0 {
            self.pos.clone()
        }
        else {
            let h = self.step_height;
            let s = self.pos[2];
            let b = 2.0*h + (4.0*h*h - 4.0*h*s).sqrt();
            let a = s - b;
            let z = a*x*x + b*x;
            Vector3::new(self.pos[0] * x, self.pos[1] * x, z)
        }
    }

    fn has_finished(&self, pos: float) -> bool {
        let x = if pos != 0.0 { pos / self.pos.len() - self.offset } else { pos };
        x >= 1.0
    }
}

pub trait Sequence: Debug {
    /// Advances the sequence based on the provided parameters.
    ///
    /// `speed` must be given in m/s and `time` must be given in ms.
    fn advance(&mut self, speed: float, time: u32);

    /// TODO
    fn pos(&self, leg_id: usize) -> Vector3;

    /// TODO
    fn has_finished(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone)]
pub struct LegPosMove {
    x: float,
    seqs: [LegPosMoveFn; 6]
}

impl LegPosMove {

    /// Creates a new `LegPosMove` with an independent sequence for each leg.
    pub fn new(seqs: [LegPosMoveFn; 6]) -> Self {
        LegPosMove{
            x: 0.0,
            seqs: seqs
        }
    }
}

impl Sequence for LegPosMove {
    fn advance(&mut self, speed: float, time: u32) {
        self.x += speed * (time as f32) / 1000.0;
    }

    fn pos(&self, leg_id: usize) -> Vector3 {
        self.seqs[leg_id].get(self.x)
    }

    fn has_finished(&self) -> bool {
        for i in 0..6 {
            if !self.seqs[i].has_finished(self.x) {
                return false
            }
        }
        return true
    }
}

#[derive(Debug)]
pub struct Hexapod {
    legs: [Leg; 6],
    legs_origin: [Vector3; 6],
    legs_end_pos: [Vector3; 6],
    legs_seq_pos: [Vector3; 6],
    body_offset: Vector3,
    body_rot: Matrix3,
    body_rot_origin: Vector3,
    seqs: Vec<Box<dyn Sequence>>,
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
                    self.legs_end_pos[i] += seq.pos(i);
                }
                false
            }
            else {
                for i in 0..6 {
                    self.legs_seq_pos[i] += seq.pos(i);
                }
                true
            }
        });
        self.update_legs();
    }

    pub fn start_seq(&mut self, seq: impl Sequence + 'static) {
        self.seqs.push(Box::new(seq));
    }

    pub fn stop_seq(&mut self, seq: impl Sequence + 'static) {
        self.seqs.push(Box::new(seq));
    }

    pub fn set_speed(&mut self, speed: float) {
        self.speed = speed;
    }

    pub fn leg(&self, leg_id: usize) -> &Leg {
        &self.legs[leg_id]
    }
}