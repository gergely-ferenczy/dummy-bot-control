use std::{fmt::Display, ops::{Add, Sub, AddAssign, SubAssign, Mul, MulAssign, Index, IndexMut} };
use super::{Vector2, FloatEq, FloatType as float};

/// | 0 1 2 |
/// | 3 4 5 |
/// | 6 7 8 |
#[derive(Debug, Clone, PartialEq)]
pub struct Matrix2 {
    m: [float; 4]
}

impl Matrix2 {

    pub fn new(m0: float, m1: float, m2: float, m3: float) -> Self {
        Self { m: [m0, m1, m2, m3] }
    }

    pub fn determinant(&self) -> float {
        let m = &self.m;

        m[0] * m[3] - m[1] * m[2]
    }

    pub fn transpose(&self) -> Self {
        let m = &self.m;
        Self::new(m[0], m[2], m[1], m[3])
    }

    pub fn inverse(&self) -> Self {
        1.0 / self.determinant() * Matrix2::new(self.m[3], -self.m[1], -self.m[2], self.m[0])
    }

    pub fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 1.0)
    }

    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0)
    }

}

impl Display for Matrix2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "|{}, {}|", self.m[0], self.m[1])?;
        writeln!(f, "|{}, {}|", self.m[2], self.m[3])
    }
}

impl From<&[float]> for Matrix2 {
    fn from(m: &[float]) -> Self {
        Self{ m: m.try_into().unwrap() }
    }
}

impl Add for &Matrix2 {
    type Output = Matrix2;

    fn add(self, rhs: Self) -> Self::Output {
        let a = self.m;
        let b = rhs.m;

        Matrix2::new(a[0]+b[0], a[1]+b[1], a[2]+b[2], a[3]+b[3])
    }
}
impl_op_variants!(Add, add, +, Matrix2, Matrix2, Matrix2);
impl_op_assign_variants!(AddAssign, add_assign, +, Matrix2, Matrix2, Matrix2);

impl Sub for &Matrix2 {
    type Output = Matrix2;

    fn sub(self, rhs: Self) -> Self::Output {
        let a = self.m;
        let b = rhs.m;

        Matrix2::new(a[0]-b[0], a[1]-b[1], a[2]-b[2], a[3]-b[3])
    }
}
impl_op_variants!(Sub, sub, -, Matrix2, Matrix2, Matrix2);
impl_op_assign_variants!(SubAssign, sub_assign, -, Matrix2, Matrix2, Matrix2);

impl Mul for &Matrix2 {
    type Output = Matrix2;

    fn mul(self, rhs: Self) -> Self::Output {
        let a = &self.m;
        let b = &rhs.m;

        Matrix2::new(
            a[0]*b[0] + a[1]*b[2], a[0]*b[1] + a[1]*b[3],
            a[2]*b[0] + a[3]*b[2], a[2]*b[1] + a[3]*b[3]
        )
    }
}
impl_op_variants!(Mul, mul, *, Matrix2, Matrix2, Matrix2);
impl_op_assign_variants!(MulAssign, mul_assign, *, Matrix2, Matrix2, Matrix2);

impl Mul<&Vector2> for &Matrix2 {
    type Output = Vector2;

    fn mul(self, rhs: &Vector2) -> Self::Output {
        let m = self.m;
        let v = rhs;

        Vector2::new(
            m[0] * v[0] + m[1] * v[1],
            m[2] * v[0] + m[3] * v[1]
        )
    }
}
impl_op_variants!(Mul, mul, *, Matrix2, Vector2, Vector2);

impl Mul<&float> for &Matrix2 {
    type Output = Matrix2;

    fn mul(self, rhs: &float) -> Self::Output {
        let m = self.m;

        Matrix2::new(
            m[0] * rhs, m[1] * rhs,
            m[2] * rhs, m[3] * rhs
        )
    }
}
impl_op_variants!(Mul, mul, *, Matrix2, float, Matrix2);

impl Mul<&Matrix2> for &float {
    type Output = Matrix2;

    fn mul(self, rhs: &Matrix2) -> Self::Output {
        let m = rhs.m;

        Matrix2::new(
            m[0] * self, m[1] * self,
            m[2] * self, m[3] * self
        )
    }
}
impl_op_variants!(Mul, mul, *, float, Matrix2, Matrix2);

impl Index<usize> for Matrix2 {
    type Output = [float];

    fn index(&self, i: usize) -> &Self::Output {
        let i = i * 2;
        &self.m[i..i+2]
    }
}

impl IndexMut<usize> for Matrix2 {
    fn index_mut(&mut self, i: usize) -> &mut Self::Output {
        let i = i * 2;
        &mut self.m[i..i+2]
    }
}

impl FloatEq for Matrix2 {
    type Tol = float;

    fn near_eq_rel(&self, other: &Self, tol: &Self::Tol) -> bool {
        self.m.iter()
            .zip(other.m.iter())
            .all(|(a, b)| float_eq!(a, b, tol, rel) )
    }

    fn near_eq_abs(&self, other: &Self, tol: &Self::Tol) -> bool {
        self.m.iter()
            .zip(other.m.iter())
            .all(|(a, b)| float_eq!(a, b, tol, abs) )
    }
}


#[cfg(test)]
mod tests {
    use super::super::{ FloatType as float, FloatEq };
    use super::Matrix2;

    const TOL: float = 1e-5;

    #[test]
    fn determinant() {
        let m = Matrix2::new(
            1.0, 2.0,
            3.0, 4.0
        );
        assert_eq!(m.determinant(), -2.0);

        let m = Matrix2::new(
            1.0, 0.0,
            0.0, 1.0
        );
        assert_eq!(m.determinant(), 1.0);

        let m = Matrix2::new(
            1.0, 2.0,
            2.0, 3.0
        );
        assert_eq!(m.determinant(), -1.0);

        let m = Matrix2::new(
            -1.0, 2.0,
             2.0, 3.0
        );
        assert_eq!(m.determinant(), -7.0);
    }

    #[test]
    fn transpose() {
        assert_eq!(
            Matrix2::new(
                1.0, 2.0,
                3.0, 4.0
            ).transpose(),
            Matrix2::new(
                1.0, 3.0,
                2.0, 4.0
            )
        );
    }

    #[test]
    fn inverse() {
        assert!(
            Matrix2::near_eq_rel(
                &Matrix2::new(
                    1.0, 0.0,
                    0.0, 1.0
                ).inverse(),
                &Matrix2::new(
                    1.0, 0.0,
                    0.0, 1.0
                ),
                &TOL
            )
        );

        assert!(
            Matrix2::near_eq_rel(
                &Matrix2::new(
                    1.0, 2.0,
                    2.0, 3.0
                ).inverse(),
                &Matrix2::new(
                    -3.0, 2.0,
                    2.0, -1.0
                ),
                &TOL
            )
        );
    }

    #[test]
    fn identity() {
        assert_eq!(
            Matrix2::identity(),
            Matrix2::new(
                1.0, 0.0,
                0.0, 1.0
            )
        );
    }

    #[test]
    fn near_eq_rel() {
        let default: [float; 4] = [1.0, 2.0, 3.0, 4.0];

        assert!(
            Matrix2::near_eq_rel(
                &Matrix2::from(&default[..]),
                &Matrix2::from(&default[..]),
                &TOL
            )
        );

        for i in 0..4 {
            let m = [&default[0..i], &[0.0], &default[i+1..4]].concat();
            assert!(
                !Matrix2::near_eq_rel(
                    &Matrix2::from(&default[..]),
                    &Matrix2::from(&m[..]),
                    &TOL
                )
            );
        }
    }

    #[test]
    fn add() {

        let m1 = Matrix2::new(1.0, 2.0, 2.0, 1.0);
        let m2 = Matrix2::new(6.0, 7.0, 7.0, 6.0);
        let m3 = Matrix2::new(7.0, 9.0, 9.0, 7.0);

        assert!(Matrix2::near_eq_rel(&( m1.clone() +  m2.clone()), &m3, &TOL));
        assert!(Matrix2::near_eq_rel(&( m1.clone() + &m2.clone()), &m3, &TOL));
        assert!(Matrix2::near_eq_rel(&(&m1.clone() +  m2.clone()), &m3, &TOL));
        assert!(Matrix2::near_eq_rel(&(&m1.clone() + &m2.clone()), &m3, &TOL));
    }

    #[test]
    fn add_assign() {
        let m1 = Matrix2::new(1.0, 2.0, 2.0, 1.0);
        let mut m2 = Matrix2::new(6.0, 7.0, 7.0, 6.0);
        m2 += &m1;
        let mut m3 = Matrix2::new(6.0, 7.0, 7.0, 6.0);
        m3 += m1;
        let m4 = Matrix2::new(7.0, 9.0, 9.0, 7.0);

        assert!(Matrix2::near_eq_rel(&m2, &m4, &TOL));
        assert!(Matrix2::near_eq_rel(&m3, &m4, &TOL));
    }

    #[test]
    fn sub() {
        let m1 = Matrix2::new( 1.0,  2.0,  2.0,  1.0);
        let m2 = Matrix2::new( 6.0,  7.0,  7.0,  6.0);
        let m3 = Matrix2::new(-5.0, -5.0, -5.0, -5.0);

        assert!(Matrix2::near_eq_rel(&( m1.clone() -  m2.clone()), &m3, &TOL));
        assert!(Matrix2::near_eq_rel(&( m1.clone() - &m2.clone()), &m3, &TOL));
        assert!(Matrix2::near_eq_rel(&(&m1.clone() -  m2.clone()), &m3, &TOL));
        assert!(Matrix2::near_eq_rel(&(&m1.clone() - &m2.clone()), &m3, &TOL));
    }

    #[test]
    fn sub_assign() {
        let m1 = Matrix2::new(6.0, 7.0, 7.0, 6.0);
        let mut m2 = Matrix2::new(1.0, 2.0, 2.0, 1.0);
        m2 -= &m1;
        let mut m3 = Matrix2::new(1.0, 2.0, 2.0, 1.0);
        m3 -= m1;
        let m4 = Matrix2::new(-5.0, -5.0, -5.0, -5.0);

        assert!(Matrix2::near_eq_rel(&m2, &m4, &TOL));
        assert!(Matrix2::near_eq_rel(&m3, &m4, &TOL));
    }

    #[test]
    fn mul() {
        let m1 = Matrix2::new( 1.0,  2.0,  2.0,  1.0);
        let m2 = Matrix2::new( 6.0,  7.0,  7.0,  6.0);
        let m3 = Matrix2::new(20.0, 19.0, 19.0, 20.0);

        assert!(Matrix2::near_eq_rel(&( m1.clone() *  m2.clone()), &m3, &TOL));
        assert!(Matrix2::near_eq_rel(&( m1.clone() * &m2.clone()), &m3, &TOL));
        assert!(Matrix2::near_eq_rel(&(&m1.clone() *  m2.clone()), &m3, &TOL));
        assert!(Matrix2::near_eq_rel(&(&m1.clone() * &m2.clone()), &m3, &TOL));
    }

    #[test]
    fn mul_assign() {
        let m1 = Matrix2::new(6.0, 7.0, 7.0, 6.0);
        let mut m2 = Matrix2::new(1.0, 2.0, 2.0, 1.0);
        m2 *= &m1;
        let mut m3 = Matrix2::new(1.0, 2.0, 2.0, 1.0);
        m3 *= m1;
        let m4 = Matrix2::new(20.0, 19.0, 19.0, 20.0);

        assert!(Matrix2::near_eq_rel(&m2, &m4, &TOL));
        assert!(Matrix2::near_eq_rel(&m3, &m4, &TOL));
    }

    #[test]
    fn index() {
        let v: [float; 4] = [1.0, 2.0, 3.0, 4.0];
        let m = Matrix2::new(1.0, 2.0, 3.0, 4.0);

        for i in 0..4 {
            assert_eq!(m[i/2][i%2], v[i]);
        }
    }

    #[test]
    fn index_mut() {
        let v: [float; 4] = [1.0, 2.0, 3.0, 4.0];
        let mut m = Matrix2::new(0.0, 0.0, 0.0, 0.0);

        for i in 0..4 {
            m[i/2][i%2] = (i+1) as float;
        }

        for i in 0..4 {
            assert_eq!(m[i/2][i%2], v[i]);
        }
    }
}