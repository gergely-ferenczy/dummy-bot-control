use crate::math::{ transform, FloatType as float, Circle, Vector3 };

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

    // TODO redundant calculations
    pub fn intersection_pos(&self) -> Vector3 {
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
        let rm_rev = transform::rotate_matrix(a0_rel, &Vector3::new(0.0, 0.0, 1.0));
        &rm_rev * &v1
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