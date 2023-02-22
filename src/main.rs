mod math;
mod robot;

use math::{ Vector3 };
use robot::{ Leg, Hexapod, LegPosMoveFn, LegPosMove };

fn main() {
    let legs = [
        Leg::new(0.06, 0.08, Vector3::zero()),
        Leg::new(0.06, 0.08, Vector3::zero()),
        Leg::new(0.06, 0.08, Vector3::zero()),
        Leg::new(0.06, 0.08, Vector3::zero()),
        Leg::new(0.06, 0.08, Vector3::zero()),
        Leg::new(0.06, 0.08, Vector3::zero())
    ];
    let legs_origin = [
        Vector3::new(-0.05,  0.05, 0.0),
        Vector3::new(-0.06,   0.0, 0.0),
        Vector3::new(-0.05, -0.05, 0.0),
        Vector3::new( 0.05,  0.05, 0.0),
        Vector3::new( 0.06,   0.0, 0.0),
        Vector3::new( 0.05, -0.05, 0.0)
    ];
    let legs_end_pos = [
        Vector3::new(-0.1,  0.1, 0.01),
        Vector3::new(-0.12, 0.0, 0.01),
        Vector3::new(-0.1, -0.1, 0.01),
        Vector3::new( 0.1,  0.1, 0.01),
        Vector3::new( 0.12, 0.0, 0.01),
        Vector3::new( 0.1, -0.1, 0.01)
    ];

    let mut h = Hexapod::new(legs, legs_origin, legs_end_pos);
    let seq_fn1 = LegPosMoveFn::new(Vector3::new(0.1, 0.0, -0.05), 0.01, 0.0);
    let seq_fn2 = LegPosMoveFn::new(Vector3::new(0.1, 0.0, -0.05), 0.01, 1.0);
    let seq_fn3 = LegPosMoveFn::new(Vector3::new(0.1, 0.0, -0.05), 0.01, 0.0);
    let seq_fn4 = LegPosMoveFn::new(Vector3::new(0.1, 0.0, -0.05), 0.01, 1.0);
    let seq_fn5 = LegPosMoveFn::new(Vector3::new(0.1, 0.0, -0.05), 0.01, 0.0);
    let seq_fn6 = LegPosMoveFn::new(Vector3::new(0.1, 0.0, -0.05), 0.01, 1.0);
    let seq = LegPosMove::new([seq_fn1, seq_fn2, seq_fn3, seq_fn4, seq_fn5, seq_fn6]);

    h.set_speed(0.2);
    h.start_seq(seq);

    for i in 0..=40 {
        print!("{:4}ms ", i*50);
        for j in 0..6 {
            let pos = h.leg(j).position();
            print!("|{:6.3}, {:6.3}| ", pos[0], pos[2]);
        }
        println!();
        h.advance_sequences(50);
    }
}
