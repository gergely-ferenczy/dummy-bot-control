use super::{ Matrix2, Matrix3, Vector3, FloatType as float };

pub fn rotate_matrix2(angle: float) -> Matrix2 {
    let c = angle.cos();
    let s = angle.sin();
    Matrix2::new(
        c, -s,
        s,  c
    )
}

pub fn rotate_matrix3(angle: float, axis: &Vector3) -> Matrix3 {
    let axis = axis.norm();
    let x = axis[0];
    let y = axis[1];
    let z = axis[2];
    let c = angle.cos();
    let s = angle.sin();
    let one_c = 1.0 - c;
    let xone_c = x * one_c;
    let yone_c = y * one_c;
    let zone_c = z * one_c;
    let xs = x * s;
    let ys = y * s;
    let zs = z * s;
    Matrix3::new(
        x*xone_c+c,  x*yone_c-zs, x*zone_c+ys,
        y*xone_c+zs, y*yone_c+c,  y*zone_c-xs,
        z*xone_c-ys, z*yone_c+xs, z*zone_c+c
    )
}

pub fn scale_matrix(v: &Vector3) -> Matrix3 {
    Matrix3::new(
        v[0], 0.0, 0.0,
        0.0, v[1], 0.0,
        0.0, 0.0, v[2]
    )
}
