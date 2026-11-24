//! Sends the robot-frame command out: serial radio (real robot), grSim, or
//! nowhere (dry run).

use crate::config::{OutputConfig, OutputKind, Team};
use protobuf::Message;
use std::io::Write;
use std::net::UdpSocket;

/// What the robot is told to do this tick. Velocities in the robot frame:
/// `vx` forward, `vy` to the left (m/s), `omega` counter-clockwise (rad/s).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Command {
    pub vx: f64,
    pub vy: f64,
    pub omega: f64,
    pub kick: bool,
    /// Dribbler level 0..7.
    pub dribbler: u8,
}

pub enum Radio {
    Serial { port: Box<dyn serialport::SerialPort>, slots: u32 },
    Grsim { socket: UdpSocket, addr: String },
    None,
}

impl Radio {
    pub fn open(cfg: &OutputConfig) -> Result<Self, String> {
        match cfg.kind {
            OutputKind::Serial => {
                let port = serialport::new(&cfg.serial_port, cfg.baud_rate)
                    .data_bits(serialport::DataBits::Eight)
                    .parity(serialport::Parity::None)
                    .stop_bits(serialport::StopBits::One)
                    .flow_control(serialport::FlowControl::None)
                    .timeout(std::time::Duration::from_millis(20))
                    .open()
                    .map_err(|e| format!("cannot open {}: {e}", cfg.serial_port))?;
                Ok(Self::Serial { port, slots: cfg.radio_slots })
            }
            OutputKind::Grsim => {
                let socket = UdpSocket::bind("0.0.0.0:0").map_err(|e| e.to_string())?;
                Ok(Self::Grsim { socket, addr: cfg.grsim_addr.clone() })
            }
            OutputKind::None => Ok(Self::None),
        }
    }

    pub fn send(&mut self, id: u32, team: Team, cmd: &Command) {
        match self {
            Self::Serial { port, slots } => {
                let frame = serialize(id, cmd, *slots);
                if let Err(e) = port.write_all(&frame).and_then(|_| port.flush()) {
                    eprintln!("radio: serial write failed: {e}");
                }
            }
            Self::Grsim { socket, addr } => {
                let bytes = grsim_packet(id, team, cmd);
                if let Err(e) = socket.send_to(&bytes, addr.as_str()) {
                    eprintln!("radio: grSim send failed: {e}");
                }
            }
            Self::None => {}
        }
    }
}

/// Applies `radio none | grsim [addr] | serial <port> [baud]` to `base`.
pub fn configure(base: &OutputConfig, args: &[String]) -> Result<OutputConfig, String> {
    let mut o = base.clone();
    let args: Vec<&str> = args.iter().map(String::as_str).collect();
    match args[..] {
        ["none"] => o.kind = OutputKind::None,
        ["grsim"] => o.kind = OutputKind::Grsim,
        ["grsim", addr] => {
            o.kind = OutputKind::Grsim;
            o.grsim_addr = addr.to_string();
        }
        ["serial", port] | ["serial", port, _] => {
            o.kind = OutputKind::Serial;
            o.serial_port = port.to_string();
            if let Some(b) = args.get(2) {
                o.baud_rate = b.parse().map_err(|_| format!("`{b}` is not a baud rate"))?;
            }
        }
        _ => return Err("needs none | grsim [addr] | serial <port> [baud]".into()),
    }
    Ok(o)
}

pub fn describe(o: &OutputConfig) -> String {
    match o.kind {
        OutputKind::Serial => format!("serial {} @ {}", o.serial_port, o.baud_rate),
        OutputKind::Grsim => format!("grsim {}", o.grsim_addr),
        OutputKind::None => "none".into(),
    }
}

/// The engine's radio frame: `slots` robots x 5 bytes, every slot but ours
/// zero. Velocities are sent in cm/s (crad/s), clamped to +-511.
///
/// Byte 0: id(3) | dribbler(3) | kick(1) | callback(1)
/// Bytes 1-3: sign(1) | low 7 bits of |vx|, |vy|, |omega|
/// Byte 4: high bits of |vx|(2) | |vy|(2) | |omega|(4)
pub fn serialize(id: u32, cmd: &Command, slots: u32) -> Vec<u8> {
    let mut frame = vec![0u8; slots as usize * 5];
    if id >= slots {
        return frame;
    }
    let scale = |v: f64| ((v * 100.0) as i32).clamp(-511, 511);
    let (vx, vy, w) = (scale(cmd.vx), scale(cmd.vy), scale(cmd.omega));
    let sign = |v: i32| if v < 0 { 0x80u8 } else { 0 };
    let low = |v: i32| (v.unsigned_abs() & 0x7F) as u8;
    let high = |v: i32| (v.unsigned_abs() >> 7) as u8;

    let b = &mut frame[id as usize * 5..id as usize * 5 + 5];
    b[0] = ((id as u8 & 0x07) << 5) | ((cmd.dribbler & 0x07) << 2) | ((cmd.kick as u8) << 1);
    b[1] = sign(vx) | low(vx);
    b[2] = sign(vy) | low(vy);
    b[3] = sign(w) | low(w);
    b[4] = ((high(vx) & 0x03) << 6) | ((high(vy) & 0x03) << 4) | (high(w) & 0x0F);
    frame
}

fn grsim_packet(id: u32, team: Team, cmd: &Command) -> Vec<u8> {
    use crate::proto::{grSim_Commands::*, grSim_Packet::GrSim_Packet};
    let mut rc = GrSim_Robot_Command::new();
    rc.set_id(id);
    rc.set_veltangent(cmd.vx as f32);
    rc.set_velnormal(cmd.vy as f32);
    rc.set_velangular(cmd.omega as f32);
    rc.set_kickspeedx(if cmd.kick { 3.0 } else { 0.0 });
    rc.set_kickspeedz(0.0);
    rc.set_spinner(cmd.dribbler > 0);
    rc.set_wheelsspeed(false);
    let mut cmds = GrSim_Commands::new();
    cmds.set_timestamp(0.0);
    cmds.set_isteamyellow(team == Team::Yellow);
    cmds.robot_commands.push(rc);
    let mut packet = GrSim_Packet::new();
    packet.commands = protobuf::MessageField::some(cmds);
    packet.write_to_bytes().unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn configure_parses_radio_commands() {
        let base = OutputConfig::default();
        let args = |s: &str| s.split_whitespace().map(String::from).collect::<Vec<_>>();
        let o = configure(&base, &args("serial COM5 57600")).unwrap();
        assert_eq!((o.kind, o.serial_port.as_str(), o.baud_rate), (OutputKind::Serial, "COM5", 57600));
        assert_eq!(configure(&base, &args("grsim")).unwrap().grsim_addr, base.grsim_addr);
        assert_eq!(configure(&base, &args("none")).unwrap().kind, OutputKind::None);
        assert!(configure(&base, &args("serial")).is_err());
        assert!(configure(&base, &args("serial COM5 fast")).is_err());
    }

    #[test]
    fn frame_has_only_our_slot() {
        let cmd = Command { vx: 1.5, vy: -0.5, omega: 0.0, kick: true, dribbler: 3 };
        let f = serialize(2, &cmd, 6);
        assert_eq!(f.len(), 30);
        assert!(f[..10].iter().chain(&f[15..]).all(|&b| b == 0));
        let b = &f[10..15];
        assert_eq!(b[0], (2 << 5) | (3 << 2) | (1 << 1));
        // 150 = 0b1_0010110: low 7 bits 22, high bit 1
        assert_eq!(b[1], 22);
        assert_eq!(b[2], 0x80 | 50);
        assert_eq!(b[3], 0);
        assert_eq!(b[4], 1 << 6);
    }

    #[test]
    fn speeds_are_clamped() {
        let cmd = Command { vx: 9.0, vy: -9.0, omega: 20.0, ..Default::default() };
        let b = serialize(0, &cmd, 6);
        // 511 = high 3, low 127
        assert_eq!(b[1], 127);
        assert_eq!(b[2], 0x80 | 127);
        assert_eq!(b[3], 127);
        assert_eq!(b[4], (3 << 6) | (3 << 4) | 3);
    }

    #[test]
    fn id_outside_the_frame_sends_nothing() {
        let cmd = Command { vx: 1.0, ..Default::default() };
        assert!(serialize(7, &cmd, 6).iter().all(|&b| b == 0));
    }
}
