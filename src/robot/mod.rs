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
    fn get(&self, x: float) -> Vector3;
}

#[derive(Debug, Clone)]
pub struct LegPosMoveSeqFn {
    pos: Vector3,
    step_height: float
}

impl LegPosMoveSeqFn {
    pub fn new(pos: Vector3, step_height: float) -> Self {
        LegPosMoveSeqFn{ pos, step_height }
    }
}

impl SequenceFn for LegPosMoveSeqFn {
    fn get(&self, x: float) -> Vector3 {
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
}

trait Sequence: Debug {
    fn advance(&mut self, speed: float, time: u32);
    fn pos(&self, leg_id: usize) -> Vector3;
    fn has_finished(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone)]
pub struct LegPosMove {
    last_time: u32,
    x: float,
    x_max: float,
    seqs: [LegPosMoveSeqFn; 6],
    gait_offsets: [float; 6]
}

impl LegPosMove {
    pub fn new(seqs: [LegPosMoveSeqFn; 6], gait_offsets: [float; 6]) -> Self {
        let max_gait_offset = gait_offsets.iter().max_by(|a, b| a.total_cmp(b)).unwrap();
        LegPosMove{
            last_time: 0,
            x: 0.0,
            x_max: 1.0 + max_gait_offset,
            seqs: seqs,
            gait_offsets: gait_offsets
        }
    }
}

impl Sequence for LegPosMove {
    fn advance(&mut self, speed: float, time: u32) {
        let delta_time = time - self.last_time;
        self.x += speed * (delta_time as f32) / 1000.0;
        if self.x > self.x_max {
            self.x = self.x_max
        };
    }

    fn pos(&self, leg_id: usize) -> Vector3 {
        self.seqs[leg_id].get(self.x - self.gait_offsets[leg_id])
    }

    fn has_finished(&self) -> bool {
        /* This float equality comparison is valid because of how the advance() function handles self.x. */
        self.x == self.x_max
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
            let mut pos = (&self.body_rot * (&self.legs_end_pos[i] - &self.body_rot_origin) + &self.body_rot_origin);
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