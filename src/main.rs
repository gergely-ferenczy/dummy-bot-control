#![allow(dead_code)] // TODO: remove this after prototyping phase is done

use std::f32::consts::PI;
use std::time::{ Duration, Instant };
use std::io::Write;
use log::{ info };

use std::net::TcpListener;
use tungstenite::{ accept, Message };
use json::{ JsonValue };

mod math;
mod robot;

use math::{ Vector2, Vector3, FloatType as float };
use robot::{ Hexapod, Leg };

#[derive(Debug)]
struct ControlPacket {
    speed: float,
    step: Vector2,
    step_height_weight: float
}


fn get_msg_text(msg: &Message) -> Option<&str> {
    match msg {
        Message::Text(s) => Some(s),
        Message::Binary(v) => Some(std::str::from_utf8(v).expect("Invalid UTF8")),
        _ => None
    }
}


fn create_pos_info_msg(h: &Hexapod) -> JsonValue {
    let mut legs = Vec::new();


    for i in 0..6 {
        let mut leg_pos = Vec::new();
        let start_pos = h.leg_origin(i);
        let mid_pos = start_pos + h.leg(i).intersection_pos();
        let end_pos = start_pos + h.leg(i).position();

        leg_pos.push(json::array![-start_pos[0], start_pos[2], start_pos[1]]);
        leg_pos.push(json::array![-mid_pos[0], mid_pos[2], mid_pos[1]]);
        leg_pos.push(json::array![-end_pos[0], end_pos[2], end_pos[1]]);
        legs.push(leg_pos);
    }

    json::object! {
        "legs": legs
    }
}


fn monitor_listener(rx: std::sync::mpsc::Receiver<JsonValue>) {
    let addr = "127.0.0.1:8080";
    let listener = TcpListener::bind(&addr).expect("Can't listen");
    info!("Listening on: {}", addr);

    // Handle connections
    while let Ok((stream, _)) = listener.accept() {
        let peer = stream.peer_addr().expect("connected streams should have a peer address");
        let mut ws_stream = accept(stream).expect("Failed to accept");
        info!("New WebSocket connection: {}", peer);

        loop {
            let msg = rx.recv().unwrap();
            let msg_str = json::stringify(msg);
            let res = ws_stream.write_message(Message::Text(msg_str));
            if res.is_err() {
                break;
            }
        }
    }
}

fn control_listener(tx: std::sync::mpsc::Sender<ControlPacket>) {
    let addr = "127.0.0.1:8081";
    let listener = TcpListener::bind(&addr).expect("Can't listen");
    info!("Listening on: {}", addr);

    // Handle connections
    while let Ok((stream, _)) = listener.accept() {
        let peer = stream.peer_addr().expect("connected streams should have a peer address");
        let mut ws_stream = accept(stream).expect("Failed to accept");
        info!("New WebSocket connection: {}", peer);

        loop {
            if let Ok(msg) = ws_stream.read_message() {
                if let Ok(msg_text) = msg.into_text() {
                    if let Ok(json_data) = json::parse(msg_text.as_str()) {
                        let cp = ControlPacket{
                            speed: json_data["speed"].as_f32().unwrap(),
                            step: Vector2::new(json_data["step"]["x"].as_f32().unwrap(), json_data["step"]["y"].as_f32().unwrap()),
                            step_height_weight: json_data["step_height_weight"].as_f32().unwrap()
                        };

                        tx.send(cp);
                    }
                }
            }
            else {
                break;
            }
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {

    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .format(|buf, record| { writeln!(buf, "{}: {}", record.level(), record.args()) })
        .init();

    let (monitor_tx, monitor_rx) = std::sync::mpsc::channel();
    let (control_tx, control_rx) = std::sync::mpsc::channel::<ControlPacket>();

    let robot_control_thread = std::thread::spawn(move || {
        let legs = [
            Leg::new(0.06, 0.08, Vector3::zero()),
            Leg::new(0.06, 0.08, Vector3::zero()),
            Leg::new(0.06, 0.08, Vector3::zero()),
            Leg::new(0.06, 0.08, Vector3::zero()),
            Leg::new(0.06, 0.08, Vector3::zero()),
            Leg::new(0.06, 0.08, Vector3::zero())
        ];
        let legs_origin = [
            Vector3::new(-0.03,  0.05, 0.02),
            Vector3::new(-0.04,   0.0, 0.02),
            Vector3::new(-0.03, -0.05, 0.02),
            Vector3::new( 0.03,  0.05, 0.02),
            Vector3::new( 0.04,   0.0, 0.02),
            Vector3::new( 0.03, -0.05, 0.02)
        ];
        let legs_end_pos = [
            Vector3::new(-0.1,  0.1, 0.00),
            Vector3::new(-0.11, 0.0, 0.00),
            Vector3::new(-0.1, -0.1, 0.00),
            Vector3::new( 0.1,  0.1, 0.00),
            Vector3::new( 0.11, 0.0, 0.00),
            Vector3::new( 0.1, -0.1, 0.00)
        ];

        let mut h = Hexapod::new(legs, legs_origin, legs_end_pos);
        h.set_speed(0.02);

        let period :u64 = 10;
        let start = Instant::now();

        let mut cntr = 0;
        loop {
            h.update(period as u32);

            monitor_tx.send(create_pos_info_msg(&h)).unwrap();

            while let Ok(cp) = control_rx.try_recv() {
                let step = cp.step * 0.1; // Vector2::new(0.0, 0.08);
                let speed = cp.speed * 0.2; // 0.02;

                h.set_speed(speed);
                h.set_step(&step, cp.step_height_weight);
                info!("{:?} {:?}", &step, speed);
            }

            std::thread::sleep(Duration::from_millis(cntr * period).saturating_sub(start.elapsed()));
            cntr += 1;
        }
    });

    let monitor_listener_thread = std::thread::spawn(move || {
        monitor_listener(monitor_rx);
    });

    let control_listener_thread = std::thread::spawn(move || {
        control_listener(control_tx);
    });

    monitor_listener_thread.join().expect("Couldn't join on the associated thread");
    control_listener_thread.join().expect("Couldn't join on the associated thread");
    robot_control_thread.join().expect("Couldn't join on the associated thread");

    Ok(())
}
