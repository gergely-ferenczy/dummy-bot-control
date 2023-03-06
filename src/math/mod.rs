
macro_rules! impl_op_variants {
    ($op_class:ident, $method:ident, $op:tt, $lhs_type:ty, $rhs_type:ty, $output_type:ty) => {
        impl $op_class<$rhs_type> for $lhs_type {
            type Output = $output_type;
            fn $method(self, rhs: $rhs_type) -> Self::Output { &self $op &rhs }
        }
        impl $op_class<&$rhs_type> for $lhs_type {
            type Output = $output_type;
            fn $method(self, rhs: &$rhs_type) -> Self::Output { &self $op rhs }
        }
        impl $op_class<$rhs_type> for &$lhs_type {
            type Output = $output_type;
            fn $method(self, rhs: $rhs_type) -> Self::Output { self $op &rhs }
        }
    };
}

macro_rules! impl_op_assign_variants {
    ($op_class:ident, $method:ident, $op:tt, $lhs_type:ty, $rhs_type:ty, $output_type:ty) => {
        impl $op_class<$rhs_type> for $lhs_type {
            fn $method(&mut self, rhs: $rhs_type) {
                let lhs: &Self = self;
                *self = lhs $op &rhs;
            }
        }
        impl $op_class<&$rhs_type> for $lhs_type {
            fn $method(&mut self, rhs: &$rhs_type) {
                let lhs: &Self = self;
                *self = lhs $op rhs;
            }
        }
    };
}

pub trait FloatEq {
    type Tol;

    fn near_eq_rel(&self, other: &Self, tol: &Self::Tol) -> bool;
    fn near_eq_abs(&self, other: &Self, tol: &Self::Tol) -> bool;
}

macro_rules! impl_float_eq_trait {
    ($float_type:ty) => {
        impl FloatEq for $float_type {
            type Tol = Self;

            fn near_eq_rel(&self, other: &Self, tol: &Self::Tol) -> bool {
                let a = *self;
                let b = *other;
                let abs_sum = a.abs() + b.abs();
                //println!("{} {} {}", a, b, tol);
                // A simple equality check covers cases for exact match, inf and different but
                // equal 0 representations like "-0.0 == 0.0".
                a == b || {
                    let diff = Self::abs(a - b);
                    if a == 0.0 || b == 0.0 || abs_sum < <$float_type>::MIN_POSITIVE {
                        // When a or b is zero or both are extremely close to it relative error is less meaningful.
                        diff <= *tol * <$float_type>::MIN_POSITIVE
                    }
                    else {
                        diff <= *tol * Self::min(abs_sum, Self::MAX)
                    }
                }
            }

            fn near_eq_abs(&self, other: &Self, tol: &Self::Tol) -> bool {
                Self::abs(*self - *other) <= *tol
            }
        }
    };
}

pub trait AssertFloatEq {
    type Tol;

    fn debug_near_eq_tol(&self, other: &Self, tol: &Self::Tol) -> Self;
}

macro_rules! impl_assert_float_eq_trait {
    ($float_type:ty) => {
        impl AssertFloatEq for $float_type {
            type Tol = Self;

            fn debug_near_eq_tol(&self, other: &Self, tol: &Self::Tol) -> Self {
                let a = *self;
                let b = *other;
                let abs_sum = a.abs() + b.abs();
                if a == 0.0 || b == 0.0 || abs_sum < <$float_type>::MIN_POSITIVE {
                    // When a or b is zero or both are extremely close to it relative error is less meaningful.
                    *tol * <$float_type>::MIN_POSITIVE
                }
                else {
                    *tol * Self::min(abs_sum, Self::MAX)
                }
            }
        }
    };
}

impl_float_eq_trait!(f32);
impl_float_eq_trait!(f64);
impl_assert_float_eq_trait!(f32);
impl_assert_float_eq_trait!(f64);

#[macro_export]
macro_rules! float_eq {
    ($left:expr, $right:expr, $tol:expr) => {
        float_eq!($left, $right, $tol, rel)
    };

    ($left:expr, $right:expr, $tol:expr, rel) => {
        match (&$left, &$right, &$tol) {
            (left_val, right_val, tol_val) => {
                left_val.near_eq_rel(right_val, tol_val)
            }
        }
    };

    ($left:expr, $right:expr, $tol:expr, abs) => {
        match (&$left, &$right, &$tol) {
            (left_val, right_val, tol_val) => {
                left_val.near_eq_abs(right_val, tol_val)
            }
        }
    };
}

#[macro_export]
macro_rules! float_ne {
    ($left:expr, $right:expr, $tol:expr) => {
        !float_eq!($left, $right, $tol)
    };

    ($left:expr, $right:expr, $tol:expr, rel) => {
        !float_eq!($left, $right, $tol, rel)
    };

    ($left:expr, $right:expr, $tol:expr, abs) => {
        !float_eq!($left, $right, $tol, abs)
    };
}

#[macro_export]
macro_rules! assert_float_eq {
    ($left:expr, $right:expr, $tol:expr) => {
        assert_float_eq!($left, $right, $tol, rel);
    };
    ($left:expr, $right:expr, $tol:expr, rel) => {
        match (&$left, &$right, &$tol) {
            (left_val, right_val, tol_val) => {
                if !float_eq!(left_val, right_val, tol_val, rel) {
                    panic!(
                        "assertion failed: `float_eq!({}, {}, {}, rel)`, calculated tolerance: {}, diff: {}",
                        *left_val,
                        *right_val,
                        *tol_val,
                        AssertFloatEq::debug_near_eq_tol(&*left_val, &*right_val, &*tol_val),
                        (*left_val - *right_val).abs()
                    )
                }
            }
        }
    };
    ($left:expr, $right:expr, $tol:expr, abs) => {
        match (&$left, &$right, &$tol) {
            (left_val, right_val, tol_val) => {
                if !float_eq!(left_val, right_val, tol_val, abs) {
                    panic!(
                        "assertion failed: `float_eq!({}, {}, {}, abs)`, diff: {}",
                        *left_val,
                        *right_val,
                        *tol_val,
                        (*left_val - *right_val).abs()
                    )
                }
            }
        }
    };
}

#[macro_export]
macro_rules! assert_float_ne {
    ($left:expr, $right:expr, $tol:expr) => {
        assert_float_ne!($left, $right, $tol, rel);
    };
    ($left:expr, $right:expr, $tol:expr, rel) => {
        match (&$left, &$right, &$tol) {
            (left_val, right_val, tol_val) => {
                if !float_ne!(left_val, right_val, tol_val, rel) {
                    panic!(
                        "assertion failed: `float_ne!({}, {}, {})`, calculated tolerance: {}, diff: {}",
                        *left_val,
                        *right_val,
                        *tol_val,
                        AssertFloatEq::debug_near_eq_tol(&*left_val, &*right_val, &*tol_val),
                        (*left_val - *right_val).abs()
                    )
                }
            }
        }
    };
    ($left:expr, $right:expr, $tol:expr, abs) => {
        match (&$left, &$right, &$tol) {
            (left_val, right_val, tol_val) => {
                if !float_ne!(left_val, right_val, tol_val, abs) {
                    panic!(
                        "assertion failed: `float_ne!({}, {}, {}, abs)`, diff: {}",
                        *left_val,
                        *right_val,
                        *tol_val,
                        (*left_val - *right_val).abs()
                    )
                }
            }
        }
    };
}

#[cfg(math_float_precision="64")]
pub type FloatType = f64;
#[cfg(not(math_float_precision="64"))]
pub type FloatType = f32;

mod circle;
mod matrices;
mod point;
pub mod transform;
mod vector2;
mod vector3;
mod quaternion;

pub use circle::*;
pub use matrices::*;
pub use point::*;
pub use vector2::*;
pub use vector3::*;
pub use quaternion::*;


#[cfg(test)]
mod tests {
    use super::FloatEq;
    use super::AssertFloatEq;

    macro_rules! assert_float_eq_with_type {
        ($left:expr, $right:expr, $tol:expr, $float_type:ty) => {
            assert_float_eq!(($left as $float_type), ($right as $float_type), $tol);
        };
    }

    macro_rules! assert_float_ne_with_type {
        ($left:expr, $right:expr, $tol:expr, $float_type:ty) => {
            assert_float_ne!(($left as $float_type), ($right as $float_type), $tol);
        };
    }

    macro_rules! test_float_eq {
        ($float_type: ty, $test_fn_name:ident, $tol: literal) => {
            #[test]
            fn $test_fn_name() {
                let tol: $float_type = $tol;

                // Different 0 representations
                assert_float_eq_with_type!( 0.0,  0.0, tol, $float_type);
                assert_float_eq_with_type!( 0.0, -0.0, tol, $float_type);
                assert_float_eq_with_type!(-0.0,  0.0, tol, $float_type);
                assert_float_eq_with_type!(-0.0, -0.0, tol, $float_type);

                // Numbers around 1
                assert_float_eq_with_type!(1.0, 1.0+(tol/2.0), tol, $float_type);
                assert_float_eq_with_type!(1.0, 1.0-(tol/2.0), tol, $float_type);
                assert_float_eq_with_type!(1.0+(tol/2.0), 1.0, tol, $float_type);
                assert_float_eq_with_type!(1.0-(tol/2.0), 1.0, tol, $float_type);

                // Numbers around -1
                assert_float_eq_with_type!(-1.0, -1.0+(tol/2.0), tol, $float_type);
                assert_float_eq_with_type!(-1.0, -1.0-(tol/2.0), tol, $float_type);
                assert_float_eq_with_type!(-1.0+(tol/2.0), -1.0, tol, $float_type);
                assert_float_eq_with_type!(-1.0-(tol/2.0), -1.0, tol, $float_type);

                // Numbers between 0 and 1
                assert_float_eq_with_type!(0.000001, 0.000001+(tol/2.0*0.000001), tol, $float_type);
                assert_float_eq_with_type!(0.000001, 0.000001-(tol/2.0*0.000001), tol, $float_type);
                assert_float_eq_with_type!(0.000001+(tol/2.0*0.000001), 0.000001, tol, $float_type);
                assert_float_eq_with_type!(0.000001-(tol/2.0*0.000001), 0.000001, tol, $float_type);

                // Numbers between -1 and 0
                assert_float_eq_with_type!(-0.000001, -0.000001+(tol/2.0*0.000001), tol, $float_type);
                assert_float_eq_with_type!(-0.000001, -0.000001-(tol/2.0*0.000001), tol, $float_type);
                assert_float_eq_with_type!(-0.000001+(tol/2.0*0.000001), -0.000001, tol, $float_type);
                assert_float_eq_with_type!(-0.000001-(tol/2.0*0.000001), -0.000001, tol, $float_type);

                // Regular large numbers
                assert_float_eq_with_type!((1.0+tol/2.0)/tol, 1.0/tol, tol, $float_type);
                assert_float_eq_with_type!(1.0/tol, (1.0+tol/2.0)/tol, tol, $float_type);
                assert_float_eq_with_type!((1.0-tol/2.0)/tol, 1.0/tol, tol, $float_type);
                assert_float_eq_with_type!(1.0/tol, (1.0-tol/2.0)/tol, tol, $float_type);
                assert_float_eq_with_type!(-(1.0+tol/2.0)/tol, -1.0/tol, tol, $float_type);
                assert_float_eq_with_type!(-1.0/tol, -(1.0+tol/2.0)/tol, tol, $float_type);
                assert_float_eq_with_type!(-(1.0-tol/2.0)/tol, -1.0/tol, tol, $float_type);
                assert_float_eq_with_type!(-1.0/tol, -(1.0-tol/2.0)/tol, tol, $float_type);

                // Sum rounding error
                assert_float_eq_with_type!(0.1 + 0.2, 0.3, tol, $float_type);
                assert_float_eq_with_type!(-0.1 + -0.2, -0.3, tol, $float_type);

                // Sub rounding error
                assert_float_eq_with_type!(0.6 - 0.2, 0.4, tol, $float_type);
                assert_float_eq_with_type!(-0.6 - -0.2, -0.4, tol, $float_type);

                // NaN values
                assert_float_ne_with_type!(<$float_type>::NAN, <$float_type>::NAN, tol, $float_type);
                assert_float_ne_with_type!(<$float_type>::NAN, 0.0, tol, $float_type);
                assert_float_ne_with_type!(0.0, <$float_type>::NAN, tol, $float_type);
                assert_float_ne_with_type!(<$float_type>::NAN, -0.0, tol, $float_type);
                assert_float_ne_with_type!(-0.0, <$float_type>::NAN, tol, $float_type);
                assert_float_ne_with_type!(<$float_type>::NAN, <$float_type>::INFINITY, tol, $float_type);
                assert_float_ne_with_type!(<$float_type>::INFINITY, <$float_type>::NAN, tol, $float_type);
                assert_float_ne_with_type!(<$float_type>::NAN, <$float_type>::NEG_INFINITY, tol, $float_type);
                assert_float_ne_with_type!(<$float_type>::NEG_INFINITY, <$float_type>::NAN, tol, $float_type);
                assert_float_ne_with_type!(<$float_type>::NAN, <$float_type>::MAX, tol, $float_type);
                assert_float_ne_with_type!(<$float_type>::MAX, <$float_type>::NAN, tol, $float_type);
                assert_float_ne_with_type!(<$float_type>::NAN, <$float_type>::MIN, tol, $float_type);
                assert_float_ne_with_type!(<$float_type>::MIN, <$float_type>::NAN, tol, $float_type);

                // Infinities
                assert_float_eq_with_type!(<$float_type>::INFINITY, <$float_type>::INFINITY, tol, $float_type);
                assert_float_eq_with_type!(<$float_type>::NEG_INFINITY, <$float_type>::NEG_INFINITY, tol, $float_type);

                // Extremely large values (overflow potential)
                assert_float_eq_with_type!(<$float_type>::MAX, <$float_type>::MAX, tol, $float_type);
                assert_float_ne_with_type!(<$float_type>::MAX, -<$float_type>::MAX, tol, $float_type);
                assert_float_ne_with_type!(-<$float_type>::MAX, <$float_type>::MAX, tol, $float_type);
                assert_float_ne_with_type!(<$float_type>::MAX, <$float_type>::MAX / 2.0, tol, $float_type);
                assert_float_ne_with_type!(-<$float_type>::MAX, <$float_type>::MAX / 2.0, tol, $float_type);
                assert_float_ne_with_type!(<$float_type>::MAX, -<$float_type>::MAX / 2.0, tol, $float_type);

                // Numbers on opposite sides of 0
                assert_float_eq_with_type!(-tol/3.0*<$float_type>::MIN_POSITIVE,  tol/3.0*<$float_type>::MIN_POSITIVE, tol, $float_type);
                assert_float_eq_with_type!( tol/3.0*<$float_type>::MIN_POSITIVE, -tol/3.0*<$float_type>::MIN_POSITIVE, tol, $float_type);
            }
        };
    }

    test_float_eq!(f32, test_float_eq_f32, 1e-7);
    test_float_eq!(f64, test_float_eq_f64, 1e-15);
}