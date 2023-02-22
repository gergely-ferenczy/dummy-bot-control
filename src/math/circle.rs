use super::{Point2, FloatType as float };

#[derive(Debug, Clone, PartialEq)]
pub struct Circle {
    x: float,
    y: float,
    r: float
}

impl Circle {
    pub fn new(x: float, y: float, r: float) -> Self {
        Circle{ x, y, r }
    }

    pub fn intersection_points(&self, other: &Self) -> (Point2, Point2) {
        let a = self;
        let b = other;
        let dd = (a.x - b.x).powi(2) + (a.y - b.y).powi(2);
        let d = dd.sqrt();
        let l = (a.r.powi(2) - b.r.powi(2) + dd) / (2.0 * d);
        let h = (a.r.powi(2) - l.powi(2)).sqrt();

        let lperd = l / d;
        let hperd = h / d;
        let x2mx1 = b.x - a.x;
        let y2my1 = b.y - a.y;
        let lperd_x2mx1 = lperd * x2mx1;
        let lperd_y2my1 = lperd * y2my1;
        let hperd_x2mx1 = hperd * x2mx1;
        let hperd_y2my1 = hperd * y2my1;

        (
            Point2::new(lperd_x2mx1 + hperd_y2my1 + a.x, lperd_y2my1 - hperd_x2mx1 + a.y),
            Point2::new(lperd_x2mx1 - hperd_y2my1 + a.x, lperd_y2my1 + hperd_x2mx1 + a.y)
        )
    }
}

#[cfg(test)]
mod tests {
    use crate::math::FloatEq;

    use super::{ Circle };
    use super::super::{ Point2, FloatType as float};

    const TOL: float = 1e-5;

    #[test]
    fn intersection_points() {
        // No intersection points
        let (p1_act, p2_act) = Circle::intersection_points(
            &Circle { x: 0.0, y: 0.0, r: 4.0 },
            &Circle { x: 1.0, y: 1.0, r: 6.0 }
        );
        assert!(p1_act.x.is_nan());
        assert!(p1_act.y.is_nan());
        assert!(p2_act.x.is_nan());
        assert!(p2_act.y.is_nan());

        // 1 intersection point
        let p_exp = Point2::new(6.0, 0.0);
        let (p1_act, p2_act) = Circle::intersection_points(
            &Circle { x: 0.0, y: 0.0, r: 6.0 },
            &Circle { x: 9.0, y: 0.0, r: 3.0 }
        );
        assert!(p_exp.near_eq_rel(&p1_act, &TOL));
        assert!(p_exp.near_eq_rel(&p2_act, &TOL));

        // 2 intersection points
        let p1_exp = Point2::new(-0.8902364, 0.2347636);
        let p2_exp = Point2::new( 4.7652364, 5.8902364);
        let (p1_act, p2_act) = Circle::intersection_points(
            &Circle { x: 2.0, y: 3.0, r: 4.0 },
            &Circle { x: 6.0, y: -1.0, r: 7.0 }
        );
        assert!(p1_exp.near_eq_rel(&p1_act, &TOL));
        assert!(p2_exp.near_eq_rel(&p2_act, &TOL));

        let p1_exp = Point2::new(5.8631544,  3.351309);
        let p2_exp = Point2::new(3.2368455, -1.901309);
        let (p1_act, p2_act) = Circle::intersection_points(
            &Circle { x: 4.0, y: 1.0, r: 3.0 },
            &Circle { x: -4.0, y: 5.0, r: 10.0 }
        );
        assert!(p1_exp.near_eq_rel(&p1_act, &TOL));
        assert!(p2_exp.near_eq_rel(&p2_act, &TOL));

        // Infinite intersection points
        let (p1_act, p2_act) = Circle::intersection_points(
            &Circle { x: 2.0, y: 3.0, r: 4.0 },
            &Circle { x: 2.0, y: 3.0, r: 4.0 }
        );
        assert!(p1_act.x.is_nan());
        assert!(p1_act.y.is_nan());
        assert!(p2_act.x.is_nan());
        assert!(p2_act.y.is_nan());
    }
}