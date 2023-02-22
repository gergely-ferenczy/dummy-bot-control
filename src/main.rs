mod math;
mod robot;

use std::time::Instant;

use math::{ Vector3 };
use robot::{ Leg, Hexapod, LegPosMoveSeqFn, LegPosMove };

fn main() {
    let legs = [
        Leg::new(60.0, 80.0, Vector3::zero()),
        Leg::new(60.0, 80.0, Vector3::zero()),
        Leg::new(60.0, 80.0, Vector3::zero()),
        Leg::new(60.0, 80.0, Vector3::zero()),
        Leg::new(60.0, 80.0, Vector3::zero()),
        Leg::new(60.0, 80.0, Vector3::zero())
    ];
    let legs_origin = [
        Vector3::new(-50.0,  50.0, 0.0),
        Vector3::new(-60.0,   0.0, 0.0),
        Vector3::new(-50.0, -50.0, 0.0),
        Vector3::new( 50.0,  50.0, 0.0),
        Vector3::new( 60.0,   0.0, 0.0),
        Vector3::new( 50.0, -50.0, 0.0)
    ];
    let legs_end_pos = [
        Vector3::new(-100.0,  100.0, 10.0),
        Vector3::new(-120.0,    0.0, 10.0),
        Vector3::new(-100.0, -100.0, 10.0),
        Vector3::new( 100.0,  100.0, 10.0),
        Vector3::new( 120.0,    0.0, 10.0),
        Vector3::new( 100.0, -100.0, 10.0)
    ];

    let mut h = Hexapod::new(legs, legs_origin, legs_end_pos);
    let seq_fn = LegPosMoveSeqFn::new(Vector3::new(20.0, 0.0, 0.0), 30.0);
    let gait_offsets = [0.0, 1.0, 0.0, 1.0, 0.0, 1.0];
    let seq = LegPosMove::new([seq_fn.clone(), seq_fn.clone(), seq_fn.clone(), seq_fn.clone(), seq_fn.clone(), seq_fn.clone()], gait_offsets);

    h.set_speed(2.0);
    h.start_seq(seq);

    for _ in 0..20 {
        for i in 0..6 {
            let pos = h.leg(i).position();
            let sign = if pos[2] == 10.0 { '*' } else { ' ' };
            print!("| {:.1}, {:.1}{}| ", pos[0], pos[2], sign);
        }
        println!();
        h.advance_sequences(100);
    }
}
