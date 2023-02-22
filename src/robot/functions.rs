use crate::math::{ FloatType as float, Vector3 };

pub fn quad_step(x: float, pos: &Vector3, step_height: float) -> Vector3 {
    let h = step_height;
    let s = pos[2];
    let b = 2.0*h + (4.0*h*h - 4.0*h*s).sqrt();
    let a = s - b;
    let z = a*x*x + b*x;
    Vector3::new(pos[0] * x, pos[1] * x, z)
}