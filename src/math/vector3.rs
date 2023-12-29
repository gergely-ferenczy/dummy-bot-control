use std::{fmt::Display, ops::{Add, Sub, Neg, AddAssign, SubAssign, Index, IndexMut, Mul, MulAssign, Div, DivAssign} };
use super::{FloatEq, FloatType as float, Vector2};

/// | 0 |
/// | 1 |
/// | 2 |
#[derive(Debug, Clone, PartialEq)]
pub struct Vector3 {
    v: [float; 3]
}

impl Vector3 {
    pub fn new(x: float, y: float, z: float) -> Self {
        Self { v: [x, y, z] }
    }

    pub fn len(&self) -> float {
        let v = self.v;
        float::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    }

    pub fn dist(&self, other: &Self) -> float {
        let a = self.v;
        let b = other.v;
        float::sqrt((b[0]-a[0]).powi(2) + (b[1]-a[1]).powi(2) + (b[2]-a[2]).powi(2))
    }

    pub fn angle(&self, other: &Self) -> float {
        let l1 = self.len();
        let l2 = other.len();

        if l1 == 0.0 || l2 == 0.0 {
            0.0
        }
        else {
            let angle_cos = self.dot(other) / (l1 * l2);
            if float_eq!(angle_cos, 1.0, float::EPSILON, abs) {
                0.0
            }
            else if float_eq!(angle_cos, -1.0, float::EPSILON, abs) {
                std::f64::consts::PI as float
            }
            else {
                angle_cos.acos()
            }
        }
    }

    pub fn norm(&self) -> Self {
        let len = self.len();
        assert_ne!(len, 0.0, "A vector with 0 length cannot be normalized.");
        self / len
    }

    pub fn dot(&self, other: &Self)-> float {
        self.v[0]*other.v[0] + self.v[1]*other.v[1] + self.v[2]*other.v[2]
    }

    pub fn cross(&self, other: &Self) -> Self {
        Self::new(
            self.v[1]*other.v[2] - self.v[2]*other.v[1],
            self.v[2]*other.v[0] - self.v[0]*other.v[2],
            self.v[0]*other.v[1] - self.v[1]*other.v[0]
        )
    }

    pub fn zero() -> Self {
        Vector3::new(0.0, 0.0, 0.0)
    }
}

impl Display for Vector3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "|")?;
        self.v[0].fmt(f)?;
        write!(f, ", ")?;
        self.v[1].fmt(f)?;
        write!(f, ", ")?;
        self.v[2].fmt(f)?;
        write!(f, "|")
    }
}

impl From<Vector2> for Vector3 {
    fn from(value: Vector2) -> Self {
        Vector3::new(value[0], value[1], 0.0)
    }
}

impl From<&Vector2> for Vector3 {
    fn from(value: &Vector2) -> Self {
        Vector3::new(value[0], value[1], 0.0)
    }
}

impl Add<&Vector3> for &Vector3 {
    type Output = Vector3;

    fn add(self, rhs: &Vector3) -> Self::Output {
        let a = &self.v;
        let b = &rhs.v;

        Vector3::new(a[0]+b[0], a[1]+b[1], a[2]+b[2])
    }
}
impl_op_variants!(Add, add, +, Vector3, Vector3, Vector3);
impl_op_assign_variants!(AddAssign, add_assign, +, Vector3, Vector3, Vector3);

impl Sub<&Vector3> for &Vector3 {
    type Output = Vector3;

    fn sub(self, rhs: &Vector3) -> Self::Output {
        let a = &self.v;
        let b = &rhs.v;

        Vector3::new(a[0]-b[0], a[1]-b[1], a[2]-b[2])
    }
}
impl_op_variants!(Sub, sub, -, Vector3, Vector3, Vector3);
impl_op_assign_variants!(SubAssign, sub_assign, -, Vector3, Vector3, Vector3);

impl Mul<&float> for &Vector3 {
    type Output = Vector3;

    fn mul(self, rhs: &float) -> Self::Output {
        Vector3::new(self.v[0]*rhs, self.v[1]*rhs, self.v[2]*rhs)
    }
}
impl_op_variants!(Mul, mul, *, Vector3, float, Vector3);
impl_op_assign_variants!(MulAssign, mul_assign, *, Vector3, float, Vector3);

impl Mul<&Vector3> for &float {
    type Output = Vector3;

    fn mul(self, rhs: &Vector3) -> Self::Output {
        rhs * self
    }
}
impl_op_variants!(Mul, mul, *, float, Vector3, Vector3);

impl Div<&float> for &Vector3 {
    type Output = Vector3;

    fn div(self, rhs: &float) -> Self::Output {
        Vector3::new(self.v[0]/rhs, self.v[1]/rhs, self.v[2]/rhs)
    }
}
impl_op_variants!(Div, div, /, Vector3, float, Vector3);
impl_op_assign_variants!(DivAssign, div_assign, /, Vector3, float, Vector3);

impl Div<&Vector3> for &float {
    type Output = Vector3;

    fn div(self, rhs: &Vector3) -> Self::Output {
        rhs / self
    }
}
impl_op_variants!(Div, div, /, float, Vector3, Vector3);

impl Neg for Vector3 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Vector3::new(-self.v[0], -self.v[1], -self.v[2])
    }
}

impl Neg for &Vector3 {
    type Output = Vector3;

    fn neg(self) -> Self::Output {
        Vector3::new(-self.v[0], -self.v[1], -self.v[2])
    }
}

impl Index<usize> for Vector3 {
    type Output = float;

    fn index(&self, i: usize) -> &Self::Output {
        &self.v[i]
    }
}

impl IndexMut<usize> for Vector3 {
    fn index_mut(&mut self, i: usize) -> &mut Self::Output {
        &mut self.v[i]
    }
}

impl FloatEq for Vector3 {
    type Tol = float;

    fn near_eq_rel(&self, other: &Self, tol: &Self::Tol) -> bool {
        self.v.iter()
            .zip(other.v.iter())
            .all(|(a, b)| float_eq!(a, b, tol, rel))
    }

    fn near_eq_abs(&self, other: &Self, tol: &Self::Tol) -> bool {
        self.v.iter()
            .zip(other.v.iter())
            .all(|(a, b)| float_eq!(a, b, tol, abs))
    }
}

#[cfg(test)]
mod tests {
    use super::super::{ FloatType as float, FloatEq, AssertFloatEq };
    use super::Vector3;

    const TOL: float = 1e-5;

    #[test]
    fn len() {
        let v = Vector3::new(2.0, 3.0, 6.0);
        assert_float_eq!(v.len(), 7.0, 1e-5);
    }

    #[test]
    fn dist() {
        let v1 = Vector3::new(4.0, 6.0, 12.0);
        let v2 = Vector3::new(2.0, 3.0, 6.0);
        assert_float_eq!(Vector3::dist(&v1, &v2), 7.0, 1e-5);
    }

    #[test]
    fn angle() {
        // 0° angles
        let v1 = Vector3::new(0.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 0.0, 0.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), 0.0, TOL);
        let v1 = Vector3::new(0.0, 0.0, 0.0);
        let v2 = Vector3::new(1.0, 0.0, 0.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), 0.0, TOL);
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(6.0, 0.0, 0.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), 0.0, TOL);
        let v1 = Vector3::new(0.0, 12.0, 0.0);
        let v2 = Vector3::new(0.0, 5.0, 0.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), 0.0, TOL);
        let v1 = Vector3::new(0.0, 0.0, 3.0);
        let v2 = Vector3::new(0.0, 0.0, 8.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), 0.0, TOL);
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(2.0, 4.0, 6.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), 0.0, TOL);
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(1000.0, 2000.0, 3000.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), 0.0, TOL);

        // 180° angles
        let a = std::f64::consts::PI as float;
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(-6.0, 0.0, 0.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), a, TOL);
        let v1 = Vector3::new(0.0, 12.0, 0.0);
        let v2 = Vector3::new(0.0, -5.0, 0.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), a, TOL);
        let v1 = Vector3::new(0.0, 0.0, 3.0);
        let v2 = Vector3::new(0.0, 0.0, -8.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), a, TOL);
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(-2.0, -4.0, -6.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), a, TOL);
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(-1000.0, -2000.0, -3000.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), a, TOL);

        // 90° angles
        let a = std::f64::consts::FRAC_PI_2 as float;
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), a, TOL);
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 0.0, 1.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), a, TOL);
        let v1 = Vector3::new(0.0, 1.0, 0.0);
        let v2 = Vector3::new(0.0, 0.0, 1.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), a, TOL);
        let v1 = Vector3::new(1.0, 1.0, 0.0);
        let v2 = Vector3::new(0.0, 0.0, 1.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), a, TOL);
        let v1 = Vector3::new(0.0, 1.0, 1.0);
        let v2 = Vector3::new(1.0, 0.0, 0.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), a, TOL);
        let v1 = Vector3::new(1.0, 0.0, 1.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), a, TOL);

        // And a few random angles for good measure

        let v1 = Vector3::new(5478.0, 89278.0, 55588.0);
        let v2 = Vector3::new(48899.0, 2084.0, 26356.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), 1.236819, TOL);
        let v1 = Vector3::new(5.478, 89.278, 55.588);
        let v2 = Vector3::new(48899.0, 2084.0, 26356.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), 1.236819, TOL);
        let v1 = Vector3::new(5.478, 89.278, 55.588);
        let v2 = Vector3::new(4.8899, 0.2084, 2.6356);
        assert_float_eq!(Vector3::angle(&v1, &v2), 1.236819, TOL);
        let v1 = Vector3::new(69796.0, 41047.0, 24709.0);
        let v2 = Vector3::new(88143.0, 73765.0, 27657.0);
        assert_float_eq!(Vector3::angle(&v1, &v2), 0.1702819, TOL);
        let v1 = Vector3::new(697960000.0, 410470000.0, 247090000.0);
        let v2 = Vector3::new(0.88143, 0.73765, 0.27657);
        assert_float_eq!(Vector3::angle(&v1, &v2), 0.1702819, TOL);
    }

    #[test]
    #[should_panic]
    fn norm_panic() {
        Vector3::new(0.0, 0.0, 0.0).norm();
    }

    #[test]
    fn norm() {
        // Already normalized vectors
        assert!(Vector3::near_eq_rel(
            &Vector3::new(1.0, 0.0, 0.0).norm(),
            &Vector3::new(1.0, 0.0, 0.0),
            &TOL
        ));
        assert!(Vector3::near_eq_rel(
            &Vector3::new(0.0, 1.0, 0.0).norm(),
            &Vector3::new(0.0, 1.0, 0.0),
            &TOL
        ));
        assert!(Vector3::near_eq_rel(
            &Vector3::new(0.0, 0.0, 1.0).norm(),
            &Vector3::new(0.0, 0.0, 1.0),
            &TOL
        ));
        assert!(Vector3::near_eq_rel(
            &Vector3::new(123456.0, 0.0, 0.0).norm(),
            &Vector3::new(1.0, 0.0, 0.0),
            &TOL
        ));
        assert!(Vector3::near_eq_rel(
            &Vector3::new(0.0, 1.23456, 0.0).norm(),
            &Vector3::new(0.0, 1.0, 0.0),
            &TOL
        ));
        assert!(Vector3::near_eq_rel(
            &Vector3::new(0.0, 0.0, 0.123456).norm(),
            &Vector3::new(0.0, 0.0, 1.0),
            &TOL
        ));
        assert!(Vector3::near_eq_rel(
            &Vector3::new(0.0520169, 0.847748, 0.527842).norm(),
            &Vector3::new(0.0520169, 0.847748, 0.527842),
            &TOL
        ));
        assert!(Vector3::near_eq_rel(
            &Vector3::new(0.879659, 0.0374897, 0.474126).norm(),
            &Vector3::new(0.879659, 0.0374897, 0.474126),
            &TOL
        ));

        // Random vectors
        assert!(Vector3::near_eq_rel(
            &Vector3::new(5478.0, 89278.0, 55588.0).norm(),
            &Vector3::new(0.0520169, 0.847748, 0.527842),
            &TOL
        ));
        assert!(Vector3::near_eq_rel(
            &Vector3::new(5.478, 89.278, 55.588).norm(),
            &Vector3::new(0.0520169, 0.847748, 0.527842),
            &TOL
        ));
        assert!(Vector3::near_eq_rel(
            &Vector3::new(48899.0, 2084.0, 26356.0).norm(),
            &Vector3::new(0.879659, 0.0374897, 0.474126),
            &TOL
        ));
        assert!(Vector3::near_eq_rel(
            &Vector3::new(4889900000.0, 2084000000.0, 26356000000.0).norm(),
            &Vector3::new(0.18187080702, 0.07751053433, 0.9802627845),
            &TOL
        ));

        // Vector really close to 0 length
        assert!(Vector3::near_eq_rel(
            &Vector3::new(float::EPSILON, float::EPSILON, float::EPSILON).norm(),
            &Vector3::new(0.57735, 0.57735, 0.57735),
            &TOL
        ));
    }

    #[test]
    fn dot() {
        let dot = Vector3::dot(&Vector3::new(0.0, 0.0, 0.0), &Vector3::new(0.0, 0.0, 0.0));
        assert_float_eq!(dot, 0.0, TOL);
        let dot = Vector3::dot(&Vector3::new(1.0, 0.0, 0.0), &Vector3::new(0.0, 1.0, 0.0));
        assert_float_eq!(dot, 0.0, TOL);
        let dot = Vector3::dot(&Vector3::new(1.0, 0.0, 0.0), &Vector3::new(0.0, 0.0, 1.0));
        assert_float_eq!(dot, 0.0, TOL);
        let dot = Vector3::dot(&Vector3::new(0.0, 1.0, 0.0), &Vector3::new(0.0, 0.0, 1.0));
        assert_float_eq!(dot, 0.0, TOL);

        let dot = Vector3::dot(&Vector3::new(1.0, 0.0, 0.0), &Vector3::new(1.0, 0.0, 0.0));
        assert_float_eq!(dot, 1.0, TOL);
        let dot = Vector3::dot(&Vector3::new(0.0, 1.0, 0.0), &Vector3::new(0.0, 1.0, 0.0));
        assert_float_eq!(dot, 1.0, TOL);
        let dot = Vector3::dot(&Vector3::new(0.0, 0.0, 1.0), &Vector3::new(0.0, 0.0, 1.0));
        assert_float_eq!(dot, 1.0, TOL);

        let dot = Vector3::dot(&Vector3::new(5.478, 0.0, 0.0), &Vector3::new(89.278, 0.0, 0.0));
        assert_float_eq!(dot, 489.065, TOL);
        let dot = Vector3::dot(&Vector3::new(0.0, 5.478, 0.0), &Vector3::new(0.0, 89.278, 0.0));
        assert_float_eq!(dot, 489.065, TOL);
        let dot = Vector3::dot(&Vector3::new(0.0, 0.0, 5.478), &Vector3::new(0.0, 0.0, 89.278));
        assert_float_eq!(dot, 489.065, TOL);

        let dot = Vector3::dot(&Vector3::new(1.0, 2.0, 3.0), &Vector3::new(3.0, 4.0, 5.0));
        assert_float_eq!(dot, 26.0, TOL);
    }

    #[test]
    fn cross() {
        // Vectors with 1 length
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);
        let v3 = Vector3::new(0.0, 0.0, 1.0);
        assert!(Vector3::near_eq_rel(&Vector3::cross(&v1, &v2), &v3, &TOL));
        let v1 = Vector3::new(0.0, 1.0, 0.0);
        let v2 = Vector3::new(1.0, 0.0, 0.0);
        let v3 = Vector3::new(0.0, 0.0, -1.0);
        assert!(Vector3::near_eq_rel(&Vector3::cross(&v1, &v2), &v3, &TOL));
        let v1 = Vector3::new(0.0, 0.0, 1.0);
        let v2 = Vector3::new(1.0, 0.0, 0.0);
        let v3 = Vector3::new(0.0, 1.0, 0.0);
        assert!(Vector3::near_eq_rel(&Vector3::cross(&v1, &v2), &v3, &TOL));
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 0.0, 1.0);
        let v3 = Vector3::new(0.0, -1.0, 0.0);
        assert!(Vector3::near_eq_rel(&Vector3::cross(&v1, &v2), &v3, &TOL));
        let v1 = Vector3::new(0.0, 1.0, 0.0);
        let v2 = Vector3::new(0.0, 0.0, 1.0);
        let v3 = Vector3::new(1.0, 0.0, 0.0);
        assert!(Vector3::near_eq_rel(&Vector3::cross(&v1, &v2), &v3, &TOL));
        let v1 = Vector3::new(0.0, 0.0, 1.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);
        let v3 = Vector3::new(-1.0, 0.0, 0.0);
        assert!(Vector3::near_eq_rel(&Vector3::cross(&v1, &v2), &v3, &TOL));

        // Vectors really close to 0 length
        let v1 = Vector3::new(float::EPSILON, 0.0, 0.0);
        let v2 = Vector3::new(0.0, float::EPSILON, 0.0);
        let v3 = Vector3::new(0.0, 0.0, float::EPSILON.powi(2));
        assert!(Vector3::near_eq_rel(&Vector3::cross(&v1, &v2), &v3, &TOL));

        // Vectors with 0 length
        let v1 = Vector3::new(0.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 0.0, 0.0);
        let v3 = Vector3::new(0.0, 0.0, 0.0);
        assert!(Vector3::near_eq_rel(&Vector3::cross(&v1, &v2), &v3, &TOL));

        // Random vectors
        let v1 = Vector3::new(5478.0, 89278.0, 55588.0);
        let v2 = Vector3::new(48899.0, 2084.0, 26356.0);
        let v3 = Vector3::new(2237165576.0, 2573819444.0, -4354188770.0);
        assert!(Vector3::near_eq_rel(&Vector3::cross(&v1, &v2), &v3, &TOL));
        let v1 = Vector3::new(5.478, 89.278, 55.588);
        let v2 = Vector3::new(48.899, 2.084, 26.356);
        let v3 = Vector3::new(2237.17, 2573.82, -4354.19);
        assert!(Vector3::near_eq_rel(&Vector3::cross(&v1, &v2), &v3, &TOL));
        let v1 = Vector3::new(697960000.0, 410470000.0, 247090000.0);
        let v2 = Vector3::new(0.88143, 0.73765, 0.27657);
        let v3 = Vector3::new(-6.87423e7, 2.47577e7, 1.5305e8);
        assert!(Vector3::near_eq_rel(&Vector3::cross(&v1, &v2), &v3, &TOL));
    }

    #[test]
    fn add() {
        let v1 = Vector3::new(1.0, 2.0,  3.0);
        let v2 = Vector3::new(6.0, 7.0,  8.0);
        let v3 = Vector3::new(7.0, 9.0, 11.0);

        assert!(Vector3::near_eq_rel(&( v1.clone() +  v2.clone()), &v3, &TOL));
        assert!(Vector3::near_eq_rel(&( v1.clone() + &v2.clone()), &v3, &TOL));
        assert!(Vector3::near_eq_rel(&(&v1.clone() +  v2.clone()), &v3, &TOL));
        assert!(Vector3::near_eq_rel(&(&v1.clone() + &v2.clone()), &v3, &TOL));
    }

    #[test]
    fn add_assign() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let mut v2 = Vector3::new(6.0, 7.0, 8.0);
        v2 += &v1;
        let mut v3 = Vector3::new(6.0, 7.0, 8.0);
        v3 += v1;
        let v4 = Vector3::new(7.0, 9.0, 11.0);

        assert!(Vector3::near_eq_rel(&v2, &v4, &TOL));
        assert!(Vector3::near_eq_rel(&v3, &v4, &TOL));
    }

    #[test]
    fn sub() {
        let v1 = Vector3::new( 1.0,  2.0,  3.0);
        let v2 = Vector3::new( 6.0,  7.0,  8.0);
        let v3 = Vector3::new(-5.0, -5.0, -5.0);

        assert!(Vector3::near_eq_rel(&( v1.clone() -  v2.clone()), &v3, &TOL));
        assert!(Vector3::near_eq_rel(&( v1.clone() - &v2.clone()), &v3, &TOL));
        assert!(Vector3::near_eq_rel(&(&v1.clone() -  v2.clone()), &v3, &TOL));
        assert!(Vector3::near_eq_rel(&(&v1.clone() - &v2.clone()), &v3, &TOL));
    }

    #[test]
    fn sub_assign() {
        let v1 = Vector3::new(6.0, 7.0,  8.0);
        let mut v2 = Vector3::new(1.0, 2.0, 3.0);
        v2 -= &v1;
        let mut v3 = Vector3::new(1.0, 2.0, 3.0);
        v3 -= v1;
        let v4 = Vector3::new(-5.0, -5.0, -5.0);

        assert!(Vector3::near_eq_rel(&v2, &v4, &TOL));
        assert!(Vector3::near_eq_rel(&v3, &v4, &TOL));
    }

    #[test]
    fn mul() {
        let v1 = Vector3::new( 1.0,  2.0,  3.0);
        let v2 = Vector3::new( 2.0,  4.0,  6.0);

        assert!(Vector3::near_eq_rel(&( v1.clone() *  2.0), &v2, &TOL));
        assert!(Vector3::near_eq_rel(&( v1.clone() * &2.0), &v2, &TOL));
        assert!(Vector3::near_eq_rel(&(&v1.clone() *  2.0), &v2, &TOL));
        assert!(Vector3::near_eq_rel(&(&v1.clone() * &2.0), &v2, &TOL));
        assert!(Vector3::near_eq_rel(&( 2.0 *  v1.clone()), &v2, &TOL));
        assert!(Vector3::near_eq_rel(&(&2.0 *  v1.clone()), &v2, &TOL));
        assert!(Vector3::near_eq_rel(&( 2.0 * &v1.clone()), &v2, &TOL));
        assert!(Vector3::near_eq_rel(&(&2.0 * &v1.clone()), &v2, &TOL));
    }

    #[test]
    fn mul_assign() {
        let mut v1 = Vector3::new( 1.0,  2.0,  3.0);
        let v2 = Vector3::new( 2.0,  4.0,  6.0);
        v1 *= 2.0;
        assert!(Vector3::near_eq_rel(&v1, &v2, &TOL));

        v1 = Vector3::new( 1.0,  2.0,  3.0);
        v1 *= &2.0;
        assert!(Vector3::near_eq_rel(&v1, &v2, &TOL));
    }

    #[test]
    fn index() {
        let a: [float; 3] = [1.0, 2.0, 3.0];
        let v = Vector3::new(1.0, 2.0, 3.0);

        for i in 0..3 {
            assert_eq!(v[i], a[i]);
        }
    }

    #[test]
    fn index_mut() {
        let a: [float; 3] = [1.0, 2.0, 3.0];
        let mut v = Vector3::new(0.0, 0.0, 0.0);

        for i in 0..3 {
            v[i] = (i+1) as float;
        }

        for i in 0..3 {
            assert_eq!(v[i], a[i]);
        }
    }
}