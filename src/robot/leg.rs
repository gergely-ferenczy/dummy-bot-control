use crate::math::{ transform, FloatType as float, Circle, Vector2, Vector3 };

#[derive(Debug, Clone)]
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

    fn z_angle(&self) -> float {
        let pos_zproj = Vector2::new(self.position[0], self.position[1]);
        let mut z_angle = pos_zproj.angle(&Vector2::new(1.0, 0.0));
        if pos_zproj[1] < 0.0 {
            z_angle *= -1.0;
        }
        z_angle
    }

    // TODO redundant calculations
    pub fn intersection_pos(&self) -> Vector3 {
        let z_angle = self.z_angle();
        let rm = transform::rotate_matrix(-z_angle, &Vector3::new(0.0, 0.0, 1.0));
        let pos_turned = &rm * &self.position;
        let rel_pos_turned = &pos_turned - &self.joint_offset;

        let c1 = Circle::new(0.0, 0.0, self.len_a);
        let c2 = Circle::new(rel_pos_turned[0], rel_pos_turned[2], self.len_b);
        let (_, intp) = c1.intersection_points(&c2);

        let (px, py) = (intp.x, intp.y);
        let v1 = Vector3::new(px, 0.0, py);
        let rm_rev = transform::rotate_matrix(z_angle, &Vector3::new(0.0, 0.0, 1.0));
        &rm_rev * &v1
    }

    pub fn calc_joint_angles(&self) -> (float, float, float) {
        let z_angle = self.z_angle();
        let rm = transform::rotate_matrix(z_angle, &Vector3::new(0.0, 0.0, 1.0));
        let pos_turned = &rm * &self.position;
        let joint_offset_turned = &rm * &self.joint_offset;
        let rel_pos_turned = pos_turned - joint_offset_turned;

        let c1 = Circle::new(0.0, 0.0, self.len_a);
        let c2 = Circle::new(rel_pos_turned[0], rel_pos_turned[2], self.len_b);
        let (_, intp) = c1.intersection_points(&c2);

        let (px, py) = (intp.x, intp.y);
        let v1 = Vector3::new(px, 0.0, py);
        let v1neg = -&v1;
        let v2 = Vector3::new(rel_pos_turned[0]-px, 0.0, rel_pos_turned[2]-py);
        let mut a1 = v1.angle(&Vector3::new(1.0, 0.0, 0.0));
        let a2 = v2.angle(&v1neg);

        if intp.y < 0.0 {
            a1 *= -1.0;
        }

        (z_angle, a1, a2)
    }

    pub fn set_position(&mut self, pos: &Vector3) {
        self.position = pos.clone();
    }

    pub fn position(&self) -> &Vector3 {
        &self.position
    }

    pub fn joint_offset(&self) -> Vector3 {
        let z_angle = self.z_angle();
        let rm = transform::rotate_matrix(z_angle, &Vector3::new(0.0, 0.0, 1.0));
        &rm * &self.joint_offset
    }
}