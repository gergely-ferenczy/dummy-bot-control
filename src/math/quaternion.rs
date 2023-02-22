use std::{fmt::Display, ops::{Add, Sub, AddAssign, SubAssign, Mul, MulAssign } };
use super::{Vector3, Matrix3, FloatEq, FloatType as float};

#[derive(Debug, Clone, PartialEq)]
pub struct Quaternion {
    s: float,
    v: Vector3
}

impl Quaternion {
    pub fn new(s: float, x: float, y: float, z: float ) -> Self {
        Quaternion{ s: s, v: Vector3::new(x, y, z) }
    }

    pub fn from_rotation(angle: float, axis: Vector3 ) -> Self {
        let angle = angle / 2.0;
        let axis = axis.norm();
        let angle_sin = float::sin(angle);
        let angle_cos = float::cos(angle);
        Quaternion{ s: angle_cos, v: axis * angle_sin }
    }

    pub fn len(&self) -> float {
        float::sqrt(self.s*self.s + self.v[0]*self.v[0] + self.v[1]*self.v[1] + self.v[2]*self.v[2])
    }

    pub fn norm(&self) -> Self {
        let len = self.len();
        assert_ne!(len, 0.0, "A quaternion with 0 length cannot be normalized.");
        self * (1.0 / len)
    }

    pub fn conjugate(&self) -> Self {
        Quaternion::new(self.s, -self.v[0], -self.v[1], -self.v[2])
    }

    pub fn inverse(&self) -> Self {
        let sq_len = self.s*self.s + self.v[0]*self.v[0] + self.v[1]*self.v[1] + self.v[2]*self.v[2];
        assert_ne!(sq_len, 0.0, "A quaternion with 0 length cannot be inverted.");
        self.conjugate() * (1.0 / sq_len)
    }

    pub fn rotation_matrix(&self) -> Matrix3 {
        let a = self.s;
        let b = self.v[0];
        let c = self.v[1];
        let d = self.v[2];
        let s = 2.0 / (a*a + b*b + c*c + d*d);
        let bs = b * s; let cs = c * s; let ds = d * s;
        let ab = a * bs; let ac = a * cs; let ad = a * ds;
        let bb = b * bs; let bc = b * cs; let bd = b * ds;
        let cc = c * cs; let cd = c * ds; let dd = d * ds;

        Matrix3::new(
            1.0-cc-dd,     bc-ad,     bd+ac,
                bc+ad, 1.0-bb-dd,     cd-ab,
                bd-ac,     cd-ab, 1.0-bb-cc
        )
    }
}

impl Display for Quaternion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(p) = f.precision() {
            writeln!(f, "|{:.p$}, {:.p$}, {:.p$}, {:.p$}|", self.s, self.v[0], self.v[1], self.v[2])
        }
        else {
            writeln!(f, "|{}, {}, {}, {}|", self.s, self.v[0], self.v[1], self.v[2])
        }
    }
}

impl Add<&Quaternion> for &Quaternion {
    type Output = Quaternion;
    fn add(self, rhs: &Quaternion) -> Self::Output {
        Quaternion::new(self.s+rhs.s, self.v[0]+rhs.v[0], self.v[1]+rhs.v[1], self.v[2]+rhs.v[2])
    }
}
impl_op_variants!(Add, add, +, Quaternion, Quaternion, Quaternion);
impl_op_assign_variants!(AddAssign, add_assign, +, Quaternion, Quaternion, Quaternion);

impl Sub<&Quaternion> for &Quaternion {
    type Output = Quaternion;
    fn sub(self, rhs: &Quaternion) -> Self::Output {
        Quaternion::new(self.s-rhs.s, self.v[0]-rhs.v[0], self.v[1]-rhs.v[1], self.v[2]-rhs.v[2])
    }
}
impl_op_variants!(Sub, sub, -, Quaternion, Quaternion, Quaternion);
impl_op_assign_variants!(SubAssign, sub_assign, -, Quaternion, Quaternion, Quaternion);

impl Mul<&Quaternion> for &Quaternion {
    type Output = Quaternion;
    fn mul(self, rhs: &Quaternion) -> Self::Output {
        let a = self;
        let b = rhs;
        let dot = a.v.dot(&b.v);
        let cross = a.v.cross(&b.v);
        Quaternion{ s: a.s * b.s - dot, v: cross + (a.s * &b.v) + (b.s * &a.v) }
    }
}
impl_op_variants!(Mul, mul, *, Quaternion, Quaternion, Quaternion);
impl_op_assign_variants!(MulAssign, mul_assign, *, Quaternion, Quaternion, Quaternion);

impl Mul<&float> for &Quaternion {
    type Output = Quaternion;
    fn mul(self, rhs: &float) -> Self::Output {
        Quaternion::new(rhs*self.s, rhs*self.v[0], rhs*self.v[1], rhs*self.v[2])
    }
}
impl_op_variants!(Mul, mul, *, Quaternion, float, Quaternion);
impl_op_assign_variants!(MulAssign, mul_assign, *, Quaternion, float, Quaternion);

impl Mul<&Quaternion> for &float {
    type Output = Quaternion;
    fn mul(self, rhs: &Quaternion) -> Self::Output {
        rhs * self
    }
}
impl_op_variants!(Mul, mul, *, float, Quaternion, Quaternion);


impl FloatEq for Quaternion {
    type Tol = float;

    fn near_eq_rel(&self, other: &Self, tol: &Self::Tol) -> bool {
        float_eq!(self.s, other.s, tol, rel) &&
        float_eq!(self.v[0], other.v[0], tol, rel) &&
        float_eq!(self.v[1], other.v[1], tol, rel) &&
        float_eq!(self.v[2], other.v[2], tol, rel)
    }

    fn near_eq_abs(&self, other: &Self, tol: &Self::Tol) -> bool {
        float_eq!(self.s, other.s, tol, abs) &&
        float_eq!(self.v[0], other.v[0], tol, abs) &&
        float_eq!(self.v[1], other.v[1], tol, abs) &&
        float_eq!(self.v[2], other.v[2], tol, abs)
    }
}


#[cfg(test)]
mod tests {
    use crate::math::Matrix3;

    use super::super::{ FloatType as float, FloatEq, AssertFloatEq, Vector3 };
    use super::{ Quaternion };

    const TOL: float = 1e-5;

    #[test]
    fn from_rotation() {
        let q1 = Quaternion::from_rotation(0.576, Vector3::new(1.0, 2.0, 3.0));
        let q2 = Quaternion::new(0.95881385, 0.07591158, 0.15182316, 0.22773474);
        println!("{}{}", q1, q2);
        assert!(Quaternion::near_eq_rel(&q1, &q2, &TOL));
    }

    #[test]
    fn len() {
        let q = Quaternion::new(0.0, 0.0, 0.0, 0.0);
        assert_float_eq!(q.len(), 0.0, TOL);
        let q = Quaternion::new(1.0, 1.0, 1.0, 1.0);
        assert_float_eq!(q.len(), 2.0, TOL);
        let q = Quaternion::new(2.0, 3.0, 6.0, 0.0);
        assert_float_eq!(q.len(), 7.0, TOL);
        let q = Quaternion::new(0.0, 2.0, 3.0, 6.0);
        assert_float_eq!(q.len(), 7.0, TOL);
        let q = Quaternion::new(2.0, 0.0, 3.0, 6.0);
        assert_float_eq!(q.len(), 7.0, TOL);
        let q = Quaternion::new(2.0, 3.0, 0.0, 6.0);
        assert_float_eq!(q.len(), 7.0, TOL);
    }

    #[test]
    #[should_panic]
    fn norm_panic() {
        Quaternion::new(0.0, 0.0, 0.0, 0.0).norm();
    }

    #[test]
    fn norm() {
        // Already normalized vectors
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(1.0, 0.0, 0.0, 0.0).norm(),
            &Quaternion::new(1.0, 0.0, 0.0, 0.0),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(0.0, 1.0, 0.0, 0.0).norm(),
            &Quaternion::new(0.0, 1.0, 0.0, 0.0),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(0.0, 0.0, 1.0, 0.0).norm(),
            &Quaternion::new(0.0, 0.0, 1.0, 0.0),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(123456.0, 0.0, 0.0, 0.0).norm(),
            &Quaternion::new(1.0, 0.0, 0.0, 0.0),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(0.0, 1.23456, 0.0, 0.0).norm(),
            &Quaternion::new(0.0, 1.0, 0.0, 0.0),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(0.0, 0.0, 0.123456, 0.0).norm(),
            &Quaternion::new(0.0, 0.0, 1.0, 0.0),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(0.5, 0.5, 0.5, 0.5).norm(),
            &Quaternion::new(0.5, 0.5, 0.5, 0.5),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(0.777936, 0.0331544, 0.419299, 0.466803).norm(),
            &Quaternion::new(0.777936, 0.0331544, 0.419299, 0.466803),
            &TOL
        ));

        // Random vectors
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(5478.0, 89278.0, 55588.0, 22.0).norm(),
            &Quaternion::new(0.0520169, 0.847748, 0.527842, 0.000208903),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(5.478, 89.278, 55.588, 0.022).norm(),
            &Quaternion::new(0.0520169, 0.847748, 0.527842, 0.000208903),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(547800000.0, 8927800000.0, 5558800000.0, 2200000.0).norm(),
            &Quaternion::new(0.0520169, 0.847748, 0.527842, 0.000208903),
            &TOL
        ));

        // Quaternion really close to 0 length
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(float::EPSILON, float::EPSILON, float::EPSILON, float::EPSILON).norm(),
            &Quaternion::new(0.5, 0.5, 0.5, 0.5),
            &TOL
        ));
    }

    #[test]
    fn conjugate() {
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(1.0,  1.0,  1.0,  1.0).conjugate(),
            &Quaternion::new(1.0, -1.0, -1.0, -1.0),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(2.0,  3.0,  4.0,  5.0).conjugate(),
            &Quaternion::new(2.0, -3.0, -4.0, -5.0),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(0.0, 0.0, 0.0, 0.0).conjugate(),
            &Quaternion::new(0.0, 0.0, 0.0, 0.0),
            &TOL
        ));
    }

    #[test]
    #[should_panic]
    fn inverse_panic() {
        Quaternion::new(0.0, 0.0, 0.0, 0.0).inverse();
    }

    #[test]
    fn inverse() {
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(1.0, 0.0, 0.0, 0.0).inverse(),
            &Quaternion::new(1.0, 0.0, 0.0, 0.0),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(0.0, 1.0, 0.0, 0.0).inverse(),
            &Quaternion::new(0.0, -1.0, 0.0, 0.0),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(0.0, 0.0, 1.0, 0.0).inverse(),
            &Quaternion::new(0.0, 0.0, -1.0, 0.0),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(0.0, 0.0, 0.0, 1.0).inverse(),
            &Quaternion::new(0.0, 0.0, 0.0, -1.0),
            &TOL
        ));
        assert!(Quaternion::near_eq_rel(
            &Quaternion::new(1.0, 2.0, 3.0, 4.0).inverse(),
            &Quaternion::new(0.0333333, -0.0666667, -0.1, -0.133333),
            &TOL
        ));
    }

    #[test]
    fn add() {
        let v1 = Quaternion::new(1.0, 2.0,  3.0,  4.0);
        let v2 = Quaternion::new(6.0, 7.0,  8.0,  9.0);
        let v3 = Quaternion::new(7.0, 9.0, 11.0, 13.0);

        assert!(Quaternion::near_eq_rel(&( v1.clone() +  v2.clone()), &v3, &TOL));
        assert!(Quaternion::near_eq_rel(&( v1.clone() + &v2.clone()), &v3, &TOL));
        assert!(Quaternion::near_eq_rel(&(&v1.clone() +  v2.clone()), &v3, &TOL));
        assert!(Quaternion::near_eq_rel(&(&v1.clone() + &v2.clone()), &v3, &TOL));
    }

    #[test]
    fn add_assign() {
        let a1 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let mut a2 = Quaternion::new(6.0, 7.0, 8.0, 9.0);
        a2 += &a1;
        let mut a3 = Quaternion::new(6.0, 7.0, 8.0, 9.0);
        a3 += a1;
        let a4 = Quaternion::new(7.0, 9.0, 11.0, 13.0);

        assert!(Quaternion::near_eq_rel(&a2, &a4, &TOL));
        assert!(Quaternion::near_eq_rel(&a3, &a4, &TOL));
    }

    #[test]
    fn sub() {
        let v1 = Quaternion::new( 1.0,  2.0,  3.0,  4.0);
        let v2 = Quaternion::new( 6.0,  7.0,  8.0,  9.0);
        let v3 = Quaternion::new(-5.0, -5.0, -5.0, -5.0);

        assert!(Quaternion::near_eq_rel(&( v1.clone() -  v2.clone()), &v3, &TOL));
        assert!(Quaternion::near_eq_rel(&( v1.clone() - &v2.clone()), &v3, &TOL));
        assert!(Quaternion::near_eq_rel(&(&v1.clone() -  v2.clone()), &v3, &TOL));
        assert!(Quaternion::near_eq_rel(&(&v1.clone() - &v2.clone()), &v3, &TOL));
    }

    #[test]
    fn sub_assign() {
        let a1 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let mut a2 = Quaternion::new(6.0, 7.0, 8.0, 9.0);
        a2 -= &a1;
        let mut a3 = Quaternion::new(6.0, 7.0, 8.0, 9.0);
        a3 -= a1;
        let a4 = Quaternion::new(5.0, 5.0, 5.0, 5.0);

        assert!(Quaternion::near_eq_rel(&a2, &a4, &TOL));
        assert!(Quaternion::near_eq_rel(&a3, &a4, &TOL));
    }

    #[test]
    fn mul_self() {
        let q1 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let q2 = Quaternion::new(5.0, 6.0, 7.0, 8.0);
        let q3 = Quaternion::new(-60.0, 12.0, 30.0, 24.0);
        assert!(Quaternion::near_eq_rel(&( q1.clone() *  q2.clone()), &q3, &TOL));
        assert!(Quaternion::near_eq_rel(&( q1.clone() * &q2.clone()), &q3, &TOL));
        assert!(Quaternion::near_eq_rel(&(&q1.clone() *  q2.clone()), &q3, &TOL));
        assert!(Quaternion::near_eq_rel(&(&q1.clone() * &q2.clone()), &q3, &TOL));

        let q1 = Quaternion::new(5.0, 6.0, 7.0, 8.0);
        let q2 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let q3 = Quaternion::new(-60.0, 20.0, 14.0, 32.0);
        assert!(Quaternion::near_eq_rel(&( q1.clone() *  q2.clone()), &q3, &TOL));
        assert!(Quaternion::near_eq_rel(&( q1.clone() * &q2.clone()), &q3, &TOL));
        assert!(Quaternion::near_eq_rel(&(&q1.clone() *  q2.clone()), &q3, &TOL));
        assert!(Quaternion::near_eq_rel(&(&q1.clone() * &q2.clone()), &q3, &TOL));
    }

    #[test]
    fn mul_assign_self() {
        let q1 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let mut q2 = Quaternion::new(5.0, 6.0, 7.0, 8.0);
        q2 *= &q1;
        let mut q3 = Quaternion::new(5.0, 6.0, 7.0, 8.0);
        q3 *= &q1;

        let q4 = Quaternion::new(-60.0, 20.0, 14.0, 32.0);
        assert!(Quaternion::near_eq_rel(&q2, &q4, &TOL));
        assert!(Quaternion::near_eq_rel(&q3, &q4, &TOL));
    }

    #[test]
    fn mul_float() {
        let q1 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let q2 = Quaternion::new(2.0, 4.0, 6.0, 8.0);
        let f = 2.0;

        assert!(Quaternion::near_eq_rel(&( q1.clone() *  f.clone()), &q2, &TOL));
        assert!(Quaternion::near_eq_rel(&( q1.clone() * &f.clone()), &q2, &TOL));
        assert!(Quaternion::near_eq_rel(&(&q1.clone() *  f.clone()), &q2, &TOL));
        assert!(Quaternion::near_eq_rel(&(&q1.clone() * &f.clone()), &q2, &TOL));
        assert!(Quaternion::near_eq_rel(&( f.clone() *  q1.clone()), &q2, &TOL));
        assert!(Quaternion::near_eq_rel(&( f.clone() * &q1.clone()), &q2, &TOL));
        assert!(Quaternion::near_eq_rel(&(&f.clone() *  q1.clone()), &q2, &TOL));
        assert!(Quaternion::near_eq_rel(&(&f.clone() * &q1.clone()), &q2, &TOL));
    }

    #[test]
    fn mul_assign_float() {
        let f = 2.0;
        let mut q1 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        q1 *= &f;
        let mut q2 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        q2 *= f;

        let q3 = Quaternion::new(2.0, 4.0, 6.0, 8.0);
        assert!(Quaternion::near_eq_rel(&q1, &q3, &TOL));
        assert!(Quaternion::near_eq_rel(&q2, &q3, &TOL));
    }

    #[test]
    fn rotation_matrix() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let m = Matrix3::new(
            -0.66666675, 0.13333336, 0.73333335,
            0.66666675, -0.33333337, 0.66666675,
            0.33333334, 0.66666675, 0.13333333);

        assert!(Matrix3::near_eq_rel(&m, &q.rotation_matrix(), &TOL));
    }
}