use std::{fmt::Display, ops::{Add, Sub, Neg, AddAssign, SubAssign, Index, IndexMut, Mul, MulAssign, Div, DivAssign} };
use super::{FloatEq, FloatType as float};


#[derive(Debug, Clone, PartialEq)]
pub struct Vector2 {
    v: [float; 2]
}

impl Vector2 {
    pub fn new(x: float, y: float) -> Self {
        Self { v: [x, y] }
    }

    pub fn len(&self) -> float {
        let v = self.v;
        float::sqrt(v[0]*v[0] + v[1]*v[1])
    }

    pub fn dist(&self, other: &Self) -> float {
        let a = self.v;
        let b = other.v;
        float::sqrt((b[0]-a[0]).powi(2) + (b[1]-a[1]).powi(2))
    }

    pub fn angle(&self, other: &Self) -> float {
        let l1 = self.len();
        let l2 = other.len();

        if l1 == 0.0 && l2 == 0.0 {
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
        self.v[0]*other.v[0] + self.v[1]*other.v[1]
    }

    pub fn zero() -> Self {
        Vector2::new(0.0, 0.0)
    }
}

impl Display for Vector2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(p) = f.precision() {
            write!(f, "|{:.p$}, {:.p$}|", self.v[0], self.v[1])
        }
        else {
            write!(f, "|{}, {}|", self.v[0], self.v[1])
        }
    }
}

impl Add<&Vector2> for &Vector2 {
    type Output = Vector2;

    fn add(self, rhs: &Vector2) -> Self::Output {
        let a = &self.v;
        let b = &rhs.v;

        Vector2::new(a[0]+b[0], a[1]+b[1])
    }
}
impl_op_variants!(Add, add, +, Vector2, Vector2, Vector2);
impl_op_assign_variants!(AddAssign, add_assign, +, Vector2, Vector2, Vector2);

impl Sub<&Vector2> for &Vector2 {
    type Output = Vector2;

    fn sub(self, rhs: &Vector2) -> Self::Output {
        let a = &self.v;
        let b = &rhs.v;

        Vector2::new(a[0]-b[0], a[1]-b[1])
    }
}
impl_op_variants!(Sub, sub, -, Vector2, Vector2, Vector2);
impl_op_assign_variants!(SubAssign, sub_assign, -, Vector2, Vector2, Vector2);

impl Mul<&float> for &Vector2 {
    type Output = Vector2;

    fn mul(self, rhs: &float) -> Self::Output {
        Vector2::new(self.v[0]*rhs, self.v[1]*rhs)
    }
}
impl_op_variants!(Mul, mul, *, Vector2, float, Vector2);
impl_op_assign_variants!(MulAssign, mul_assign, *, Vector2, float, Vector2);

impl Mul<&Vector2> for &float {
    type Output = Vector2;

    fn mul(self, rhs: &Vector2) -> Self::Output {
        rhs * self
    }
}
impl_op_variants!(Mul, mul, *, float, Vector2, Vector2);

impl Div<&float> for &Vector2 {
    type Output = Vector2;

    fn div(self, rhs: &float) -> Self::Output {
        Vector2::new(self.v[0]/rhs, self.v[1]/rhs)
    }
}
impl_op_variants!(Div, div, /, Vector2, float, Vector2);
impl_op_assign_variants!(DivAssign, div_assign, /, Vector2, float, Vector2);

impl Div<&Vector2> for &float {
    type Output = Vector2;

    fn div(self, rhs: &Vector2) -> Self::Output {
        rhs / self
    }
}
impl_op_variants!(Div, div, /, float, Vector2, Vector2);

impl Neg for Vector2 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Vector2::new(-self.v[0], -self.v[1])
    }
}

impl Neg for &Vector2 {
    type Output = Vector2;

    fn neg(self) -> Self::Output {
        Vector2::new(-self.v[0], -self.v[1])
    }
}

impl Index<usize> for Vector2 {
    type Output = float;

    fn index(&self, i: usize) -> &Self::Output {
        &self.v[i]
    }
}

impl IndexMut<usize> for Vector2 {
    fn index_mut(&mut self, i: usize) -> &mut Self::Output {
        &mut self.v[i]
    }
}

impl FloatEq for Vector2 {
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
    use super::Vector2;

    const TOL: float = 1e-5;

    #[test]
    fn len() {
        let v = Vector2::new(3.0, 4.0);
        assert_float_eq!(v.len(), 5.0, 1e-5);
    }

    #[test]
    fn dist() {
        let v1 = Vector2::new(6.0, 8.0);
        let v2 = Vector2::new(3.0, 4.0);
        assert_float_eq!(Vector2::dist(&v1, &v2), 5.0, 1e-5);
    }

    #[test]
    fn angle() {
        // 0° angles
        let v1 = Vector2::new(0.0, 0.0);
        let v2 = Vector2::new(0.0, 0.0);
        assert_float_eq!(Vector2::angle(&v1, &v2), 0.0, TOL);
        let v1 = Vector2::new(1.0, 0.0);
        let v2 = Vector2::new(6.0, 0.0);
        assert_float_eq!(Vector2::angle(&v1, &v2), 0.0, TOL);
        let v1 = Vector2::new(0.0, 12.0);
        let v2 = Vector2::new(0.0, 5.0);
        assert_float_eq!(Vector2::angle(&v1, &v2), 0.0, TOL);
        let v1 = Vector2::new(1.0, 2.0);
        let v2 = Vector2::new(2.0, 4.0);
        assert_float_eq!(Vector2::angle(&v1, &v2), 0.0, TOL);
        let v1 = Vector2::new(1.0, 2.0);
        let v2 = Vector2::new(1000.0, 2000.0);
        assert_float_eq!(Vector2::angle(&v1, &v2), 0.0, TOL);

        // 180° angles
        let a = std::f64::consts::PI as float;
        let v1 = Vector2::new(1.0, 0.0);
        let v2 = Vector2::new(-6.0, 0.0);
        assert_float_eq!(Vector2::angle(&v1, &v2), a, TOL);
        let v1 = Vector2::new(0.0, 12.0);
        let v2 = Vector2::new(0.0, -5.0);
        assert_float_eq!(Vector2::angle(&v1, &v2), a, TOL);
        let v1 = Vector2::new(1.0, 2.0);
        let v2 = Vector2::new(-2.0, -4.0);
        assert_float_eq!(Vector2::angle(&v1, &v2), a, TOL);
        let v1 = Vector2::new(1.0, 2.0);
        let v2 = Vector2::new(-1000.0, -2000.0);
        assert_float_eq!(Vector2::angle(&v1, &v2), a, TOL);

        // 90° angles
        let a = std::f64::consts::FRAC_PI_2 as float;
        let v1 = Vector2::new(1.0, 0.0);
        let v2 = Vector2::new(0.0, 1.0);
        assert_float_eq!(Vector2::angle(&v1, &v2), a, TOL);
        let v1 = Vector2::new(0.0, 1.0);
        let v2 = Vector2::new(1.0, 0.0);
        assert_float_eq!(Vector2::angle(&v1, &v2), a, TOL);

        // And a few random angles for good measure

        // TODO
    }

    #[test]
    #[should_panic]
    fn norm_panic() {
        Vector2::new(0.0, 0.0).norm();
    }

    #[test]
    fn norm() {

        println!("{}", Vector2::new(10.0, 0.0).norm());

        // Already normalized vectors
        assert!(Vector2::near_eq_rel(
            &Vector2::new(1.0, 0.0).norm(),
            &Vector2::new(1.0, 0.0),
            &TOL
        ));
        assert!(Vector2::near_eq_rel(
            &Vector2::new(0.0, 1.0).norm(),
            &Vector2::new(0.0, 1.0),
            &TOL
        ));
        assert!(Vector2::near_eq_rel(
            &Vector2::new(123456.0, 0.0).norm(),
            &Vector2::new(1.0, 0.0),
            &TOL
        ));
        assert!(Vector2::near_eq_rel(
            &Vector2::new(0.0, 1.23456).norm(),
            &Vector2::new(0.0, 1.0),
            &TOL
        ));
        assert!(Vector2::near_eq_rel(
            &Vector2::new(0.573576, 0.819152).norm(),
            &Vector2::new(0.573576, 0.819152),
            &TOL
        ));

        // Random vectors
        assert!(Vector2::near_eq_rel(
            &Vector2::new(573576.0, 819152.0).norm(),
            &Vector2::new(0.573576, 0.819152),
            &TOL
        ));

        // Vector really close to 0 length
        assert!(Vector2::near_eq_rel(
            &Vector2::new(float::EPSILON, float::EPSILON).norm(),
            &Vector2::new(0.707107, 0.707107),
            &TOL
        ));
    }

    #[test]
    fn dot() {
        let dot = Vector2::dot(&Vector2::new(0.0, 0.0), &Vector2::new(0.0, 0.0));
        assert_float_eq!(dot, 0.0, TOL);
        let dot = Vector2::dot(&Vector2::new(1.0, 0.0), &Vector2::new(0.0, 1.0));
        assert_float_eq!(dot, 0.0, TOL);

        let dot = Vector2::dot(&Vector2::new(1.0, 0.0), &Vector2::new(1.0, 0.0));
        assert_float_eq!(dot, 1.0, TOL);
        let dot = Vector2::dot(&Vector2::new(0.0, 1.0), &Vector2::new(0.0, 1.0));
        assert_float_eq!(dot, 1.0, TOL);

        let dot = Vector2::dot(&Vector2::new(5.478, 0.0), &Vector2::new(89.278, 0.0));
        assert_float_eq!(dot, 489.065, TOL);
        let dot = Vector2::dot(&Vector2::new(0.0, 5.478), &Vector2::new(0.0, 89.278));
        assert_float_eq!(dot, 489.065, TOL);

        let dot = Vector2::dot(&Vector2::new(1.0, 2.0), &Vector2::new(3.0, 4.0));
        assert_float_eq!(dot, 11.0, TOL);
    }

    #[test]
    fn add() {
        let v1 = Vector2::new(1.0, 2.0);
        let v2 = Vector2::new(6.0, 7.0);
        let v3 = Vector2::new(7.0, 9.0);

        assert!(Vector2::near_eq_rel(&( v1.clone() +  v2.clone()), &v3, &TOL));
        assert!(Vector2::near_eq_rel(&( v1.clone() + &v2.clone()), &v3, &TOL));
        assert!(Vector2::near_eq_rel(&(&v1.clone() +  v2.clone()), &v3, &TOL));
        assert!(Vector2::near_eq_rel(&(&v1.clone() + &v2.clone()), &v3, &TOL));
    }

    #[test]
    fn add_assign() {
        let v1 = Vector2::new(1.0, 2.0);
        let mut v2 = Vector2::new(6.0, 7.0);
        v2 += &v1;
        let mut v3 = Vector2::new(6.0, 7.0);
        v3 += v1;
        let v4 = Vector2::new(7.0, 9.0);

        assert!(Vector2::near_eq_rel(&v2, &v4, &TOL));
        assert!(Vector2::near_eq_rel(&v3, &v4, &TOL));
    }

    #[test]
    fn sub() {
        let v1 = Vector2::new( 1.0,  2.0);
        let v2 = Vector2::new( 6.0,  7.0);
        let v3 = Vector2::new(-5.0, -5.0);

        assert!(Vector2::near_eq_rel(&( v1.clone() -  v2.clone()), &v3, &TOL));
        assert!(Vector2::near_eq_rel(&( v1.clone() - &v2.clone()), &v3, &TOL));
        assert!(Vector2::near_eq_rel(&(&v1.clone() -  v2.clone()), &v3, &TOL));
        assert!(Vector2::near_eq_rel(&(&v1.clone() - &v2.clone()), &v3, &TOL));
    }

    #[test]
    fn sub_assign() {
        let v1 = Vector2::new(6.0, 7.0);
        let mut v2 = Vector2::new(1.0, 2.0);
        v2 -= &v1;
        let mut v3 = Vector2::new(1.0, 2.0);
        v3 -= v1;
        let v4 = Vector2::new(-5.0, -5.0);

        assert!(Vector2::near_eq_rel(&v2, &v4, &TOL));
        assert!(Vector2::near_eq_rel(&v3, &v4, &TOL));
    }

    #[test]
    fn mul() {
        let v1 = Vector2::new( 1.0,  2.0);
        let v2 = Vector2::new( 2.0,  4.0);

        assert!(Vector2::near_eq_rel(&( v1.clone() *  2.0), &v2, &TOL));
        assert!(Vector2::near_eq_rel(&( v1.clone() * &2.0), &v2, &TOL));
        assert!(Vector2::near_eq_rel(&(&v1.clone() *  2.0), &v2, &TOL));
        assert!(Vector2::near_eq_rel(&(&v1.clone() * &2.0), &v2, &TOL));
        assert!(Vector2::near_eq_rel(&( 2.0 *  v1.clone()), &v2, &TOL));
        assert!(Vector2::near_eq_rel(&(&2.0 *  v1.clone()), &v2, &TOL));
        assert!(Vector2::near_eq_rel(&( 2.0 * &v1.clone()), &v2, &TOL));
        assert!(Vector2::near_eq_rel(&(&2.0 * &v1.clone()), &v2, &TOL));
    }

    #[test]
    fn mul_assign() {
        let mut v1 = Vector2::new( 1.0,  2.0);
        let v2 = Vector2::new( 2.0,  4.0);
        v1 *= 2.0;
        assert!(Vector2::near_eq_rel(&v1, &v2, &TOL));

        v1 = Vector2::new( 1.0,  2.0);
        v1 *= &2.0;
        assert!(Vector2::near_eq_rel(&v1, &v2, &TOL));
    }

    #[test]
    fn index() {
        let a: [float; 2] = [1.0, 2.0];
        let v = Vector2::new(1.0, 2.0);

        for i in 0..2 {
            assert_eq!(v[i], a[i]);
        }
    }

    #[test]
    fn index_mut() {
        let a: [float; 2] = [1.0, 2.0];
        let mut v = Vector2::new(0.0, 0.0);

        for i in 0..2 {
            v[i] = (i+1) as float;
        }

        for i in 0..2 {
            assert_eq!(v[i], a[i]);
        }
    }
}