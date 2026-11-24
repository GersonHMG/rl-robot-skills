//! A pretend robot for trying small_engine without hardware: listens for
//! grSim commands (output.kind = "grsim") and publishes SSL-Vision packets
//! of where it is. The robot follows commands with a first-order lag.
//!
//!   cargo run --example fake_robot -- [robot_id] [vision_port]

#[allow(clippy::all, non_camel_case_types, non_snake_case, dead_code)]
mod proto {
    include!(concat!(env!("OUT_DIR"), "/protos/mod.rs"));
}

use proto::grSim_Packet::GrSim_Packet;
use proto::ssl_vision_detection::{SSL_DetectionFrame, SSL_DetectionRobot};
use proto::ssl_vision_wrapper::SSL_WrapperPacket;
use protobuf::Message;
use std::net::UdpSocket;
use std::time::{Duration, Instant};

fn main() -> std::io::Result<()> {
    let mut args = std::env::args().skip(1);
    let id: u32 = args.next().and_then(|a| a.parse().ok()).unwrap_or(0);
    let port: u16 = args.next().and_then(|a| a.parse().ok()).unwrap_or(10020);

    let commands = UdpSocket::bind("127.0.0.1:20011")?;
    commands.set_nonblocking(true)?;
    let vision = UdpSocket::bind("0.0.0.0:0")?;
    vision.set_multicast_loop_v4(true)?;
    let dest = format!("224.5.23.2:{port}");
    println!("fake robot {id}: commands on 127.0.0.1:20011, vision to {dest}");

    let dt = 1.0 / 60.0;
    let tau = 0.1; // response time constant, s
    let (mut x, mut y, mut th) = (0.0f64, 0.0f64, 0.0f64);
    let (mut vx, mut vy, mut w) = (0.0f64, 0.0f64, 0.0f64); // robot frame
    let mut cmd = (0.0, 0.0, 0.0);
    let start = Instant::now();
    let mut buf = [0u8; 2048];

    for frame in 0u32.. {
        while let Ok(len) = commands.recv(&mut buf) {
            if let Ok(p) = GrSim_Packet::parse_from_bytes(&buf[..len]) {
                for rc in p.commands.robot_commands.iter().filter(|rc| rc.id() == id) {
                    cmd = (rc.veltangent() as f64, rc.velnormal() as f64, rc.velangular() as f64);
                }
            }
        }
        let k = dt / tau;
        vx += (cmd.0 - vx) * k;
        vy += (cmd.1 - vy) * k;
        w += (cmd.2 - w) * k;
        let (s, c) = th.sin_cos();
        x += (c * vx - s * vy) * dt;
        y += (s * vx + c * vy) * dt;
        th = (th + w * dt + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;

        let mut r = SSL_DetectionRobot::new();
        r.set_robot_id(id);
        r.set_confidence(0.95);
        r.set_x((x * 1000.0) as f32);
        r.set_y((y * 1000.0) as f32);
        r.set_orientation(th as f32);
        r.set_pixel_x(0.0);
        r.set_pixel_y(0.0);
        let mut det = SSL_DetectionFrame::new();
        let t = start.elapsed().as_secs_f64();
        det.set_frame_number(frame);
        det.set_t_capture(t);
        det.set_t_sent(t);
        det.set_camera_id(0);
        det.robots_blue.push(r);
        let mut packet = SSL_WrapperPacket::new();
        packet.detection = protobuf::MessageField::some(det);
        vision.send_to(&packet.write_to_bytes().unwrap(), &dest)?;

        std::thread::sleep(Duration::from_secs_f64(dt));
    }
    Ok(())
}
