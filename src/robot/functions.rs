use crate::math::{ FloatType as float };

pub fn quad_step_height(x: float, end_height: float, step_height: float) -> float {
    let h = step_height;
    let s = end_height;
    let b = 2.0*h + (4.0*h*h - 4.0*h*s).sqrt();
    let a = s - b;
    let z = a*x*x + b*x;

    return z;
}
