use std::{fmt::Display, ops::{Add, Sub, AddAssign, SubAssign, Mul, MulAssign, Index, IndexMut} };
use super::{Vector3, FloatEq, FloatType as float};

/// | 0 1 2 |
/// | 3 4 5 |
/// | 6 7 8 |
#[derive(Debug, Clone, PartialEq)]
pub struct Matrix3 {
    m: [float; 9]
}

impl Matrix3 {

    pub fn new(m0: float, m1: float, m2: float, m3: float, m4: float, m5: float, m6: float, m7: float, m8: float) -> Self {
        Self { m: [m0, m1, m2, m3, m4, m5, m6, m7, m8] }
    }

    pub fn determinant(&self) -> float {
        let m = &self.m;

        m[0] * (m[4] * m[8] - m[5] * m[7]) -
        m[1] * (m[3] * m[8] - m[5] * m[6]) +
        m[2] * (m[3] * m[7] - m[4] * m[6])
    }

    pub fn transpose(&self) -> Self {
        let m = &self.m;
        Self::new(m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8])
    }

    pub fn inverse(&self) -> Self {
        let mut m1 = self.clone();
        let mut m2 = Self::identity();

        let mut row = 0;
        let mut col = 0;

        while row < 3 && col < 3 {

            /* Search for the biggest pivot value in the current column.  */
            let mut i = row;
            for j in (row+1)..3 {
                if m1[i][col] < m1[j][col] {
                    i = j;
                }
            }

            /* If all possible pivot values are 0, jump to the next column. */
            if m1[i][col] == 0.0 {
                col += 1;
                continue;
            }

            /* The biggest pivot value is not in the current row, so swap them. */
            if i != row {
                let swap = |m: &mut Matrix3, x1: usize, y1: usize, x2: usize, y2: usize| {
                    let t = m[x1][y1];
                    m[x1][y1] = m[x2][y2];
                    m[x2][y2] = t;
                };

                for k in 0..3 {
                    swap(&mut m1, row, k, i, k);
                    swap(&mut m2, row, k, i, k);
                }
            }

            /* Divide all elements in the current row by the pivot value. */
            let div = m1[row][col];
            for m in col..3 {
                m1[row][m] /= div;
            }
            for m in 0..3 {
                m2[row][m] /= div;
            }

            /* Subtract the current row times a multiplier value from all other rows, so that the
            values above and below the pivot are all 0. */
            for i in 0..3 {
                if i != row {
                    let mul = m1[i][col];
                    for k in col..3 {
                        m1[i][k] -= m1[row][k] * mul;
                    }
                    for k in 0..3 {
                        m2[i][k] -= m2[row][k] * mul;
                    }
                }
            }

            /* Jump to the next pivot. */
            row += 1;
            col += 1;
        }

        m2
    }

    pub fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    }

    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    }

}

impl Display for Matrix3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "|{}, {}, {}|", self.m[0], self.m[1], self.m[2])?;
        writeln!(f, "|{}, {}, {}|", self.m[3], self.m[4], self.m[5])?;
        write!  (f, "|{}, {}, {}|", self.m[6], self.m[7], self.m[8])
    }
}

impl From<&[float]> for Matrix3 {
    fn from(m: &[float]) -> Self {
        Self{ m: m.try_into().unwrap() }
    }
}

impl Add for &Matrix3 {
    type Output = Matrix3;

    fn add(self, rhs: Self) -> Self::Output {
        let a = self.m;
        let b = rhs.m;

        Matrix3::new(a[0]+b[0], a[1]+b[1], a[2]+b[2], a[3]+b[3], a[4]+b[4], a[5]+b[5], a[6]+b[6], a[7]+b[7], a[8]+b[8])
    }
}
impl_op_variants!(Add, add, +, Matrix3, Matrix3, Matrix3);
impl_op_assign_variants!(AddAssign, add_assign, +, Matrix3, Matrix3, Matrix3);

impl Sub for &Matrix3 {
    type Output = Matrix3;

    fn sub(self, rhs: Self) -> Self::Output {
        let a = self.m;
        let b = rhs.m;

        Matrix3::new(a[0]-b[0], a[1]-b[1], a[2]-b[2], a[3]-b[3], a[4]-b[4], a[5]-b[5], a[6]-b[6], a[7]-b[7], a[8]-b[8])
    }
}
impl_op_variants!(Sub, sub, -, Matrix3, Matrix3, Matrix3);
impl_op_assign_variants!(SubAssign, sub_assign, -, Matrix3, Matrix3, Matrix3);

impl Mul for &Matrix3 {
    type Output = Matrix3;

    fn mul(self, rhs: Self) -> Self::Output {
        let a = &self.m;
        let b = &rhs.m;

        Matrix3::new(
            a[0]*b[0] + a[1]*b[3] + a[2]*b[6],  a[0]*b[1] + a[1]*b[4] + a[2]*b[7],  a[0]*b[2] + a[1]*b[5] + a[2]*b[8],
            a[3]*b[0] + a[4]*b[3] + a[5]*b[6],  a[3]*b[1] + a[4]*b[4] + a[5]*b[7],  a[3]*b[2] + a[4]*b[5] + a[5]*b[8],
            a[6]*b[0] + a[7]*b[3] + a[8]*b[6],  a[6]*b[1] + a[7]*b[4] + a[8]*b[7],  a[6]*b[2] + a[7]*b[5] + a[8]*b[8]
        )
    }
}
impl_op_variants!(Mul, mul, *, Matrix3, Matrix3, Matrix3);
impl_op_assign_variants!(MulAssign, mul_assign, *, Matrix3, Matrix3, Matrix3);

impl Mul<&Vector3> for &Matrix3 {
    type Output = Vector3;

    fn mul(self, rhs: &Vector3) -> Self::Output {
        let m = self;
        let v = rhs;

        Vector3::new(
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]
        )
    }
}
impl_op_variants!(Mul, mul, *, Matrix3, Vector3, Vector3);


impl Index<usize> for Matrix3 {
    type Output = [float];

    fn index(&self, mut i: usize) -> &Self::Output {
        i *= 3;
        &self.m[i..i+3]
    }
}

impl IndexMut<usize> for Matrix3 {
    fn index_mut(&mut self, mut i: usize) -> &mut Self::Output {
        i *= 3;
        &mut self.m[i..i+3]
    }
}

impl FloatEq for Matrix3 {
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
    use super::Matrix3;

    const TOL: float = 1e-5;

    #[test]
    fn determinant() {
        let m = Matrix3::new(
            1.0, 2.0, 3.0,
            4.0, 5.0, 6.0,
            7.0, 8.0, 9.0
        );
        assert_eq!(m.determinant(), 0.0);

        let m = Matrix3::new(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        );
        assert_eq!(m.determinant(), 1.0);

        let m = Matrix3::new(
            1.0, 2.0, 3.0,
            3.0, 2.0, 1.0,
            2.0, 1.0, 3.0
        );
        assert_eq!(m.determinant(), -12.0);

        let m = Matrix3::new(
            -1.0, 2.0,  3.0,
            3.0,  2.0, -1.0,
            2.0, -1.0,  3.0
        );
        assert_eq!(m.determinant(), -48.0);
    }

    #[test]
    fn transpose() {
        assert_eq!(
            Matrix3::new(
                1.0, 2.0, 3.0,
                4.0, 5.0, 6.0,
                7.0, 8.0, 9.0
            ).transpose(),
            Matrix3::new(
                1.0, 4.0, 7.0,
                2.0, 5.0, 8.0,
                3.0, 6.0, 9.0
            )
        );
    }

    #[test]
    fn inverse() {
        assert!(
            Matrix3::near_eq_rel(
                &Matrix3::new(
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0
                ).inverse(),
                &Matrix3::new(
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0
                ),
                &1e-5
            )
        );

        assert!(
            Matrix3::near_eq_rel(
                &Matrix3::new(
                    1.0, 2.0, 3.0,
                    3.0, 2.0, 1.0,
                    2.0, 1.0, 3.0
                ).inverse(),
                &Matrix3::new(
                    -5.0/12.0, 3.0/12.0, 4.0/12.0,
                    7.0/12.0,  3.0/12.0, -8.0/12.0,
                    1.0/12.0, -3.0/12.0, 4.0/12.0
                ),
                &1e-5
            )
        );
    }

    #[test]
    fn identity() {
        assert_eq!(
            Matrix3::identity(),
            Matrix3::new(
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            )
        );
    }

    #[test]
    fn near_eq_rel() {
        let default: [float; 9] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];

        assert!(
            Matrix3::near_eq_rel(
                &Matrix3::from(&default[..]),
                &Matrix3::from(&default[..]),
                &TOL
            )
        );

        for i in 0..9 {
            let m = [&default[0..i], &[0.0], &default[i+1..9]].concat();
            assert!(
                !Matrix3::near_eq_rel(
                    &Matrix3::from(&default[..]),
                    &Matrix3::from(&m[..]),
                    &TOL
                )
            );
        }
    }

    #[test]
    fn add() {

        let m1 = Matrix3::new(1.0, 2.0,  3.0,  3.0, 2.0, 1.0, 2.0, 1.0,  3.0);
        let m2 = Matrix3::new(6.0, 7.0,  8.0,  8.0, 7.0, 6.0, 7.0, 6.0,  8.0);
        let m3 = Matrix3::new(7.0, 9.0, 11.0, 11.0, 9.0, 7.0, 9.0, 7.0, 11.0);

        assert!(Matrix3::near_eq_rel(&( m1.clone() +  m2.clone()), &m3, &TOL));
        assert!(Matrix3::near_eq_rel(&( m1.clone() + &m2.clone()), &m3, &TOL));
        assert!(Matrix3::near_eq_rel(&(&m1.clone() +  m2.clone()), &m3, &TOL));
        assert!(Matrix3::near_eq_rel(&(&m1.clone() + &m2.clone()), &m3, &TOL));
    }

    #[test]
    fn add_assign() {
        let m1 = Matrix3::new(1.0, 2.0, 3.0, 3.0, 2.0, 1.0, 2.0, 1.0, 3.0);
        let mut m2 = Matrix3::new(6.0, 7.0, 8.0,  8.0, 7.0, 6.0, 7.0, 6.0, 8.0);
        m2 += &m1;
        let mut m3 = Matrix3::new(6.0, 7.0, 8.0, 8.0, 7.0, 6.0, 7.0, 6.0, 8.0);
        m3 += m1;
        let m4 = Matrix3::new(7.0, 9.0, 11.0, 11.0, 9.0, 7.0, 9.0, 7.0, 11.0);

        assert!(Matrix3::near_eq_rel(&m2, &m4, &TOL));
        assert!(Matrix3::near_eq_rel(&m3, &m4, &TOL));
    }

    #[test]
    fn sub() {
        let m1 = Matrix3::new( 1.0,  2.0,  3.0,  3.0,  2.0,  1.0,  2.0,  1.0,  3.0);
        let m2 = Matrix3::new( 6.0,  7.0,  8.0,  8.0,  7.0,  6.0,  7.0,  6.0,  8.0);
        let m3 = Matrix3::new(-5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0);

        assert!(Matrix3::near_eq_rel(&( m1.clone() -  m2.clone()), &m3, &TOL));
        assert!(Matrix3::near_eq_rel(&( m1.clone() - &m2.clone()), &m3, &TOL));
        assert!(Matrix3::near_eq_rel(&(&m1.clone() -  m2.clone()), &m3, &TOL));
        assert!(Matrix3::near_eq_rel(&(&m1.clone() - &m2.clone()), &m3, &TOL));
    }

    #[test]
    fn sub_assign() {
        let m1 = Matrix3::new(6.0, 7.0,  8.0, 8.0, 7.0, 6.0, 7.0, 6.0, 8.0);
        let mut m2 = Matrix3::new(1.0, 2.0, 3.0,  3.0, 2.0, 1.0, 2.0, 1.0, 3.0);
        m2 -= &m1;
        let mut m3 = Matrix3::new(1.0, 2.0, 3.0, 3.0, 2.0, 1.0, 2.0, 1.0, 3.0);
        m3 -= m1;
        let m4 = Matrix3::new(-5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0);

        assert!(Matrix3::near_eq_rel(&m2, &m4, &TOL));
        assert!(Matrix3::near_eq_rel(&m3, &m4, &TOL));
    }

    #[test]
    fn mul() {
        let m1 = Matrix3::new( 1.0,  2.0,  3.0,  3.0,  2.0,  1.0,  2.0,  1.0,  3.0);
        let m2 = Matrix3::new( 6.0,  7.0,  8.0,  8.0,  7.0,  6.0,  7.0,  6.0,  8.0);
        let m3 = Matrix3::new(43.0, 39.0, 44.0, 41.0, 41.0, 44.0, 41.0, 39.0, 46.0);

        assert!(Matrix3::near_eq_rel(&( m1.clone() *  m2.clone()), &m3, &TOL));
        assert!(Matrix3::near_eq_rel(&( m1.clone() * &m2.clone()), &m3, &TOL));
        assert!(Matrix3::near_eq_rel(&(&m1.clone() *  m2.clone()), &m3, &TOL));
        assert!(Matrix3::near_eq_rel(&(&m1.clone() * &m2.clone()), &m3, &TOL));
    }

    #[test]
    fn mul_assign() {
        let m1 = Matrix3::new(6.0, 7.0, 8.0,  8.0, 7.0, 6.0, 7.0, 6.0, 8.0);
        let mut m2 = Matrix3::new(1.0, 2.0,  3.0, 3.0, 2.0, 1.0, 2.0, 1.0, 3.0);
        m2 *= &m1;
        let mut m3 = Matrix3::new(1.0, 2.0,  3.0, 3.0, 2.0, 1.0, 2.0, 1.0, 3.0);
        m3 *= m1;
        let m4 = Matrix3::new(43.0, 39.0, 44.0, 41.0, 41.0, 44.0, 41.0, 39.0, 46.0);

        assert!(Matrix3::near_eq_rel(&m2, &m4, &TOL));
        assert!(Matrix3::near_eq_rel(&m3, &m4, &TOL));
    }

    #[test]
    fn index() {
        let v: [float; 9] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let m = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

        for i in 0..9 {
            assert_eq!(m[i/3][i%3], v[i]);
        }
    }

    #[test]
    fn index_mut() {
        let v: [float; 9] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let mut m = Matrix3::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        for i in 0..9 {
            m[i/3][i%3] = (i+1) as float;
        }

        for i in 0..9 {
            assert_eq!(m[i/3][i%3], v[i]);
        }
    }
}