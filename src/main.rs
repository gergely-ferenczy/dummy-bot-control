#![allow(dead_code)] // TODO: remove this after prototyping phase is done

use std::time::{ Duration, Instant };
use std::io::Write;
use log::{ info };

use std::net::TcpListener;
use tungstenite::{ accept, Message };
use json::{ JsonValue };

mod math;
mod robot;

use math::{ Vector2, Vector3, FloatType as float, FloatModule };
use robot::{ Hexapod, HexapodConfig };

#[derive(Debug)]
struct ControlPacket {
    step: Vector2,
    step_height_weight: float,
    turn_angle: float,
    body_offset: Vector3,
    body_rotation_angle: float,
    body_rotation_axis: Vector3
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
        let joint_offset = start_pos + h.leg(i).joint_offset();

        leg_pos.push(json::array![-start_pos[0], start_pos[2], start_pos[1]]);
        leg_pos.push(json::array![-joint_offset[0], joint_offset[2], joint_offset[1]]);
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
                            step: Vector2::new(
                                json_data["step"]["x"].as_f32().unwrap(),
                                json_data["step"]["y"].as_f32().unwrap()
                            ),
                            step_height_weight: json_data["step_height_weight"].as_f32().unwrap(),
                            turn_angle: json_data["turn_angle"].as_f32().unwrap(),
                            body_offset: Vector3::new(
                                json_data["body_offset"]["x"].as_f32().unwrap(),
                                json_data["body_offset"]["y"].as_f32().unwrap(),
                                json_data["body_offset"]["z"].as_f32().unwrap()
                            ),
                            body_rotation_angle: json_data["body_rotation_angle"].as_f32().unwrap(),
                            body_rotation_axis: Vector3::new(
                                json_data["body_rotation_axis"]["x"].as_f32().unwrap(),
                                json_data["body_rotation_axis"]["y"].as_f32().unwrap(),
                                json_data["body_rotation_axis"]["z"].as_f32().unwrap()
                            )
                        };

                        tx.send(cp).expect("blah");
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
        let leg_joint_offset = [
            Vector3::new(0.01, 0.0, -0.005),
            Vector3::new(0.01, 0.0, -0.005),
            Vector3::new(0.01, 0.0, -0.005),
            Vector3::new(0.01, 0.0, -0.005),
            Vector3::new(0.01, 0.0, -0.005),
            Vector3::new(0.01, 0.0, -0.005)
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
            Vector3::new(-0.09,  0.085, 0.00),
            Vector3::new(-0.11,  0.0,   0.00),
            Vector3::new(-0.09, -0.085, 0.00),
            Vector3::new( 0.09,  0.085, 0.00),
            Vector3::new( 0.11,  0.0,   0.00),
            Vector3::new( 0.09, -0.085, 0.00)
        ];

        let config = HexapodConfig {
            leg_len1: 0.06,
            leg_len2: 0.06,
            joint_offset: leg_joint_offset,
            legs_origin: legs_origin,
            legs_end_pos: legs_end_pos,
            max_speed: 0.16,
            max_step_radius: 0.05,
            max_move_radius: 0.12,
            max_step_len: 0.08,
            max_turn_angle: FloatModule::consts::FRAC_PI_4,
            max_body_offset: Vector3::new(0.03, 0.03, 0.03),
            max_body_rotation: FloatModule::consts::FRAC_PI_8
        };

        let mut h = Hexapod::new(config);

        let period :u64 = 10;
        let start = Instant::now();

        let mut cntr = 0;
        loop {
            h.update(period as u32);

            monitor_tx.send(create_pos_info_msg(&h)).unwrap();

            while let Ok(cp) = control_rx.try_recv() {
                h.set_step(&cp.step, cp.turn_angle, cp.step_height_weight);
                h.set_body_offset(&cp.body_offset);
                h.set_body_rotation(cp.body_rotation_angle, &cp.body_rotation_axis, &Vector3::zero());
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
