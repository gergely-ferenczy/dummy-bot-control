use crate::math::{ transform, FloatType as float, Circle, Vector2, Vector3 };

#[derive(Debug, Clone)]
pub struct Leg {
    len_a: float,
    len_b: float,
    joint_offset: Vector3,
    position: Vector3,
    plane_normal: Vector3
}

impl Leg {
    pub fn new(len_a: float, len_b: float, joint_offset: Vector3) -> Self {
        Leg { len_a: len_a, len_b: len_b, joint_offset: joint_offset, position: Vector3::zero(),
            plane_normal: Vector3::zero() }
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
        let rm = transform::rotate_matrix3(-z_angle, &Vector3::new(0.0, 0.0, 1.0));
        let pos_turned = &rm * &self.position;
        let rel_pos_turned = &pos_turned - &self.joint_offset;

        let c1 = Circle::new(0.0, 0.0, self.len_a);
        let c2 = Circle::new(rel_pos_turned[0], rel_pos_turned[2], self.len_b);
        let (_, intp) = c1.intersection_points(&c2);

        let (px, py) = (intp.x, intp.y);
        let v1 = Vector3::new(px, 0.0, py);
        let rm_rev = transform::rotate_matrix3(z_angle, &Vector3::new(0.0, 0.0, 1.0));
        let v2 = &rm_rev * v1;
        let v3 = Vector3::new(rel_pos_turned[0],   0.0, rel_pos_turned[2]);
        let v_il = &rm_rev * v3;
        let v_p = Vector3::new(0.0, 0.0, 1.0);

        let v_normal1 = v_il.cross(&self.plane_normal);
        let v_normal2 = v_il.cross(&v_p);
        let v_normal3 = v_normal1.cross(&v_normal2);

        let mut plane_angle = v_normal1.angle(&v_normal2);
        if v_normal3.dot(&v_il) > 0.0 {
            plane_angle *= -1.0;
        }

        let rm_plane = transform::rotate_matrix3(plane_angle, &v_il);

        rm_plane * v2
    }

    pub fn calc_joint_angles(&self) -> (float, float, float) {
        // TODO: Calculate intersection pos when end position is set. Use intersection pos to calculate angles.

        unimplemented!();
    }

    pub fn set_position(&mut self, position: &Vector3, plane_normal: &Vector3) {
        self.position = position.clone();
        self.plane_normal = plane_normal.clone();
    }

    pub fn position(&self) -> &Vector3 {
        &self.position
    }

    pub fn joint_offset(&self) -> Vector3 {
        let z_angle = self.z_angle();
        let rm = transform::rotate_matrix3(z_angle, &Vector3::new(0.0, 0.0, 1.0));
        &rm * &self.joint_offset
    }
}