use crate::math::{ FloatType as float, Vector3 };

pub fn quad_step(x: float, step: &Vector3, step_height: float) -> Vector3 {
    let h = step_height;
    let s = step[2];
    let b = 2.0*h + (4.0*h*h - 4.0*h*s).sqrt();
    let a = s - b;
    let z = a*x*x + b*x;
    Vector3::new(step[0] * x, step[1] * x, z)
}

pub fn linear_step(x: float, step: &Vector3) -> Vector3 {
    x * step
}
