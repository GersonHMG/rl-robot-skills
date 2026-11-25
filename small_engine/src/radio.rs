//! Sends the robot-frame command out: serial radio (real robot), grSim, or
//! nowhere (dry run). The serial side speaks the Sysmic base station protocol
//! (Base-Station-SSL, robot-firmware `main`): one 30-byte command per tick,
//! 32-byte telemetry back from each robot that hears it.

use crate::config::{OutputConfig, OutputKind, Team};
use protobuf::Message;
use std::collections::BTreeMap;
use std::io::{Read, Write};
use std::net::UdpSocket;
use std::time::Instant;

/// Robots per command frame (ids 0-5), 5 bytes each.
pub const SLOTS: u32 = 6;
const SLOT_BYTES: usize = 5;
const TELEMETRY_SIZE: usize = 32;
/// Telemetry v2 (wheel speeds) / v3 (adds the gyro): marker in byte 3, XOR
/// of bytes 0-30 in byte 31.
const TELEMETRY_V2: u8 = 0xA5;
const TELEMETRY_V3: u8 = 0xA6;

/// What one robot last sent back over the radio.
#[derive(Debug, Clone, Copy)]
pub struct Reply {
    pub count: u64,
    pub at: Instant,
    pub ball: bool,
    pub range_mm: u16,
    /// v2/v3 telemetry: the robot reports a valid command and its motors active.
    pub v2: Option<(bool, bool)>,
    /// v3 telemetry: gyro z rate, rad/s (counter-clockwise), if the gyro is ok.
    pub gyro: Option<f64>,
}

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
    Serial {
        port: Box<dyn serialport::SerialPort>,
        /// Telemetry bytes not parsed yet, and the replies by robot id.
        rx: Vec<u8>,
        replies: BTreeMap<u8, Reply>,
    },
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
                // Same line state as pyserial (bsctl, pilotar): DTR and RTS on, input cleared.
                let mut port = port;
                let _ = port.write_data_terminal_ready(true);
                let _ = port.write_request_to_send(true);
                let _ = port.clear(serialport::ClearBuffer::Input);
                Ok(Self::Serial { port, rx: Vec::new(), replies: BTreeMap::new() })
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
            Self::Serial { port, rx, replies } => {
                // One write per frame: the base station splits frames on line idle.
                let frame = serialize(id, cmd);
                if let Err(e) = port.write_all(&frame).and_then(|_| port.flush()) {
                    eprintln!("radio: serial write failed: {e}");
                }
                // Replies to the previous frame.
                let n = port.bytes_to_read().unwrap_or(0) as usize;
                if n > 0 {
                    let mut buf = vec![0u8; n];
                    if let Ok(n) = port.read(&mut buf) {
                        rx.extend_from_slice(&buf[..n]);
                        parse_telemetry(rx, replies);
                    }
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

    /// Robots that answered over the radio, by id (serial only).
    pub fn replies(&self) -> Vec<(u8, Reply)> {
        match self {
            Self::Serial { replies, .. } => replies.iter().map(|(k, v)| (*k, *v)).collect(),
            _ => Vec::new(),
        }
    }
}

/// One line per robot that answered: "robot 0: 120 replies, 0.0 s ago, ball no".
pub fn describe_replies(replies: &[(u8, Reply)]) -> Vec<String> {
    replies
        .iter()
        .map(|(id, r)| {
            let mut s = format!(
                "robot {id}: {} replies, {:.1} s ago, ball {} ({} mm)",
                r.count,
                r.at.elapsed().as_secs_f64(),
                if r.ball { "yes" } else { "no" },
                r.range_mm
            );
            if let Some((valid, active)) = r.v2 {
                s += &format!(", cmd {}, motors {}", if valid { "ok" } else { "invalid" }, if active { "on" } else { "off" });
            }
            if let Some(g) = r.gyro {
                s += &format!(", gyro {g:+.2} rad/s");
            }
            s
        })
        .collect()
}

/// Same signature check as the base station's bsctl: byte 0 = id<<5 | ball,
/// range < 256 mm, then all zero (v1) or the v2 / v3 marker with its XOR.
fn is_telemetry(p: &[u8]) -> bool {
    if p.len() != TELEMETRY_SIZE || p[0] & 0x1E != 0 || p[2] != 0 {
        return false;
    }
    p[3..].iter().all(|&b| b == 0)
        || (matches!(p[3], TELEMETRY_V2 | TELEMETRY_V3) && p[..31].iter().fold(0, |x, b| x ^ b) == p[31])
}

/// Splits the stream into 32-byte replies; skips a byte at a time to realign.
fn parse_telemetry(rx: &mut Vec<u8>, replies: &mut BTreeMap<u8, Reply>) {
    let mut i = 0;
    while rx.len() - i >= TELEMETRY_SIZE {
        let p = &rx[i..i + TELEMETRY_SIZE];
        if !is_telemetry(p) {
            i += 1;
            continue;
        }
        let id = p[0] >> 5;
        let count = replies.get(&id).map_or(0, |r| r.count) + 1;
        let v2 = matches!(p[3], TELEMETRY_V2 | TELEMETRY_V3).then(|| (p[4] & 1 != 0, p[4] & 2 != 0));
        let gyro = (p[3] == TELEMETRY_V3 && p[4] & 4 != 0).then(|| i16::from_le_bytes([p[5], p[6]]) as f64 / 1000.0);
        let range_mm = u16::from_le_bytes([p[1], p[2]]);
        replies.insert(id, Reply { count, at: Instant::now(), ball: p[0] & 1 != 0, range_mm, v2, gyro });
        i += TELEMETRY_SIZE;
    }
    rx.drain(..i);
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

/// The 30-byte command: 6 slots of 5 bytes in id order; the others get
/// stop (only their id). Velocities x100, rounded, sign bit apart:
/// vx, vy up to 5.11 m/s (9 bits), omega up to 20.47 rad/s (11 bits).
///
/// Byte 0: id(3) | dribbler 0-6 (3) | kick(1) | 0
/// Bytes 1-3: sign(1) | low 7 bits of |vx|, |vy|, |omega|
/// Byte 4: high bits of |vx|(2) | |vy|(2) | |omega|(4)
pub fn serialize(id: u32, cmd: &Command) -> Vec<u8> {
    let mut frame = vec![0u8; SLOTS as usize * SLOT_BYTES];
    for (slot, b) in frame.chunks_mut(SLOT_BYTES).enumerate() {
        b[0] = (slot as u8) << 5;
    }
    if id >= SLOTS {
        return frame;
    }
    let scale = |v: f64, max: u32| {
        let m = ((v.abs() * 100.0).round() as u32).min(max);
        (m, if v < 0.0 && m > 0 { 0x80u8 } else { 0 })
    };
    let ((vx, sx), (vy, sy), (w, sw)) = (scale(cmd.vx, 511), scale(cmd.vy, 511), scale(cmd.omega, 2047));

    let b = &mut frame[id as usize * SLOT_BYTES..][..SLOT_BYTES];
    b[0] = ((id as u8) << 5) | (cmd.dribbler.min(6) << 2) | ((cmd.kick as u8) << 1);
    b[1] = sx | (vx & 0x7F) as u8;
    b[2] = sy | (vy & 0x7F) as u8;
    b[3] = sw | (w & 0x7F) as u8;
    b[4] = (((vx >> 7) & 0x03) << 6 | ((vy >> 7) & 0x03) << 4 | ((w >> 7) & 0x0F)) as u8;
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

    /// Stop slot for each id, as bsctl's encode_frame({}) sends it.
    fn stop_frame() -> Vec<u8> {
        (0..6u8).flat_map(|i| [i << 5, 0, 0, 0, 0]).collect()
    }

    #[test]
    fn frame_matches_bsctl() {
        // bsctl: encode_frame({2: RobotCommand(1.5, -0.5, 0, 3, True)})
        let cmd = Command { vx: 1.5, vy: -0.5, omega: 0.0, kick: true, dribbler: 3 };
        let f = serialize(2, &cmd);
        let mut want = stop_frame();
        want[10..15].copy_from_slice(&[0x4E, 0x16, 0xB2, 0x00, 0x40]);
        assert_eq!(f, want);
        // omega has 11 bits: 7.5 rad/s = 750 = high 5, low 110; 0.006 rounds to 1
        let f = serialize(0, &Command { omega: -7.5, vx: 0.006, ..Default::default() });
        assert_eq!(&f[..5], &[0x00, 0x01, 0x00, 0x80 | 110, 5]);
    }

    #[test]
    fn speeds_and_dribbler_are_clamped() {
        let cmd = Command { vx: 9.0, vy: -9.0, omega: 30.0, dribbler: 7, ..Default::default() };
        let b = serialize(0, &cmd);
        assert_eq!(&b[..5], &[6 << 2, 127, 0x80 | 127, 127, (3 << 6) | (3 << 4) | 15]);
        // a tiny negative rounds to 0 without a sign bit
        assert_eq!(serialize(0, &Command { vx: -0.001, ..Default::default() }), stop_frame());
    }

    #[test]
    fn id_outside_the_frame_sends_stop() {
        let cmd = Command { vx: 1.0, ..Default::default() };
        assert_eq!(serialize(7, &cmd), stop_frame());
    }

    #[test]
    fn telemetry_is_parsed_and_realigned() {
        let mut v1 = [0u8; 32];
        v1[0] = (2 << 5) | 1;
        v1[1] = 40;
        let mut v2 = [0u8; 32];
        v2[0] = 0 << 5;
        v2[3] = TELEMETRY_V2;
        v2[4] = 0b11;
        v2[31] = v2[..31].iter().fold(0, |x, b| x ^ b);
        // v3 from robot 5: gyro ok, -1.5 rad/s = -1500 = 0xFA24
        let mut v3 = [0u8; 32];
        v3[0] = 5 << 5;
        v3[3] = TELEMETRY_V3;
        v3[4] = 0b100 | (7 << 3);
        v3[5..7].copy_from_slice(&(-1500i16).to_le_bytes());
        v3[31] = v3[..31].iter().fold(0, |x, b| x ^ b);
        let mut rx = vec![0xFF, 0x13]; // stray bytes
        rx.extend_from_slice(&v1);
        rx.extend_from_slice(&v2);
        rx.extend_from_slice(&v3);
        rx.extend_from_slice(&v1[..10]); // half a packet stays for next time
        let mut replies = BTreeMap::new();
        parse_telemetry(&mut rx, &mut replies);
        assert_eq!(rx.len(), 10);
        let r2 = replies[&2];
        assert!(r2.ball && r2.range_mm == 40 && r2.v2.is_none() && r2.count == 1);
        assert_eq!(replies[&0].v2, Some((true, true)));
        assert_eq!(replies[&0].gyro, None);
        let r5 = replies[&5];
        assert_eq!((r5.v2, r5.gyro), (Some((false, false)), Some(-1.5)));
        // bsctl's README v2 example: robot 2 with the ball at 42 mm
        let hex = "41 2a 00 a5 03 10 71 02 6a ff 00 00 32 00 8a 02 6a ff 00 00 32 00 5e 01 88 ff 00 00 28 00 07 21";
        let mut rx: Vec<u8> = hex.split(' ').map(|h| u8::from_str_radix(h, 16).unwrap()).collect();
        parse_telemetry(&mut rx, &mut replies);
        let r2 = replies[&2];
        assert!(r2.ball && r2.range_mm == 42 && r2.v2 == Some((true, true)) && r2.count == 2);
    }
}
