use super::{FloatType as float, FloatEq};

#[derive(Debug, Clone, PartialEq)]
pub struct Point2 {
    pub x: float,
    pub y: float
}

impl Point2 {
    pub fn new(x: float, y: float) -> Self {
        Point2{ x, y }
    }

    pub fn dist(&self, other: &Self) -> float {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
}

impl FloatEq for Point2 {
    type Tol = float;
    
    fn near_eq_abs(&self, other: &Self, tol: &Self::Tol) -> bool {
        float_eq!(self.x, other.x, tol, rel) &&
        float_eq!(self.y, other.y, tol, rel)
    }

    fn near_eq_rel(&self, other: &Self, tol: &Self::Tol) -> bool {
        float_eq!(self.x, other.x, tol, abs) &&
        float_eq!(self.y, other.y, tol, abs)
    }
}


#[derive(Debug, Clone, PartialEq)]
pub struct Point3 {
    pub x: float,
    pub y: float,
    pub z: float
}

impl Point3 {
    pub fn new(x: float, y: float, z: float) -> Self {
        Point3{ x, y, z }
    }

    pub fn dist(&self, other: &Self) -> float {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2) + (self.z - other.z).powi(2)).sqrt()
    }
}

impl FloatEq for Point3 {
    type Tol = float;
    
    fn near_eq_abs(&self, other: &Self, tol: &Self::Tol) -> bool {
        float_eq!(self.x, other.x, tol, rel) &&
        float_eq!(self.y, other.y, tol, rel) &&
        float_eq!(self.z, other.z, tol, rel)
    }

    fn near_eq_rel(&self, other: &Self, tol: &Self::Tol) -> bool {
        float_eq!(self.x, other.x, tol, abs) &&
        float_eq!(self.y, other.y, tol, abs) &&
        float_eq!(self.z, other.z, tol, rel)
    }
}

impl From<Point2> for Point3 {
    fn from(p: Point2) -> Self {
        Point3 { x: p.x, y: p.y, z: 0.0 }
    }
}