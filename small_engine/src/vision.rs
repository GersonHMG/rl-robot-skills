//! SSL-Vision receiver: listens to the multicast feed on its own thread and
//! keeps the latest pose of the one robot we drive (plus the ball). It also
//! lists every robot it sees, to check the feed is arriving, and can switch
//! to another address / port while running (`vision` command).

use crate::config::{KalmanConfig, Team, VisionConfig};
use crate::proto::ssl_vision_wrapper::SSL_WrapperPacket;
use protobuf::Message;
use std::collections::BTreeMap;
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Latest estimate of the robot, world frame, metres / radians.
#[derive(Debug, Clone, Copy, Default)]
pub struct RobotObs {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    /// Velocities, world frame: Kalman estimate (or raw finite differences
    /// with the filter off). The pose above is the raw measurement.
    pub vx: f64,
    pub vy: f64,
    pub omega: f64,
    /// Vision timestamp of the detection, s.
    pub t_capture: f64,
    pub camera: u32,
    pub confidence: f32,
    pub frame: u32,
}

/// Where the feed is received: multicast group, port, network card.
#[derive(Debug, Clone, PartialEq)]
pub struct Net {
    pub addr: String,
    pub port: u16,
    pub iface: String,
}

impl std::fmt::Display for Net {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}:{} on {}", self.addr, self.port, self.iface)
    }
}

/// A robot in the last detections, any team.
#[derive(Debug, Clone, Copy)]
pub struct Seen {
    pub x: f64,
    pub y: f64,
    pub camera: u32,
    pub at: Instant,
}

#[derive(Debug, Clone, Default)]
pub struct VisionState {
    pub robot: Option<RobotObs>,
    /// When `robot` was last updated (local clock).
    pub robot_seen: Option<Instant>,
    pub ball: Option<(f64, f64)>,
    /// Detection packets received so far.
    pub packets: u64,
    /// Detections of our robot accepted so far.
    pub detections: u64,
    /// The robot we drive as vision sees it, changed live by the `id` command.
    pub robot_id: u32,
    pub team: Team,
    /// Velocity filter settings, changed live by the `kf` command.
    pub kalman: KalmanConfig,
    /// Current socket, a switch asked for by `vision`, and why the last one failed.
    pub net: Option<Net>,
    pub reopen: Option<Net>,
    pub error: Option<String>,
    /// When the last detection packet arrived.
    pub last_packet: Option<Instant>,
    /// Every robot seen, by (team, id): "blue" / "yellow".
    pub seen: BTreeMap<(&'static str, u32), Seen>,
}

impl VisionState {
    /// The robot, if it was seen within `timeout` seconds.
    pub fn fresh_robot(&self, timeout: f64) -> Option<RobotObs> {
        match (self.robot, self.robot_seen) {
            (Some(r), Some(at)) if at.elapsed().as_secs_f64() <= timeout => Some(r),
            _ => None,
        }
    }
}

pub type SharedVision = Arc<Mutex<VisionState>>;

pub fn open_socket(net: &Net) -> std::io::Result<UdpSocket> {
    use socket2::{Domain, Protocol, SockAddr, Socket, Type};
    let group: Ipv4Addr = net.addr.parse().map_err(invalid)?;
    if !group.is_multicast() {
        return Err(invalid(format!("{group} is not a multicast address")));
    }
    let iface: Ipv4Addr = net.iface.parse().map_err(invalid)?;

    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    socket.set_reuse_address(true)?;
    socket.set_multicast_loop_v4(true)?;
    socket.bind(&SockAddr::from(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, net.port)))?;
    socket.join_multicast_v4(&group, &iface)?;
    // Also on loopback, for a vision feed or simulator on this machine.
    let _ = socket.join_multicast_v4(&group, &Ipv4Addr::LOCALHOST);
    let socket: UdpSocket = socket.into();
    socket.set_read_timeout(Some(Duration::from_millis(200)))?;
    Ok(socket)
}

fn invalid<E: std::fmt::Display>(e: E) -> std::io::Error {
    std::io::Error::new(std::io::ErrorKind::InvalidInput, e.to_string())
}

/// Starts the receiver thread. It runs for the life of the program.
pub fn spawn(cfg: VisionConfig, robot_id: u32, team: Team) -> std::io::Result<SharedVision> {
    let net = Net { addr: cfg.multicast_addr.clone(), port: cfg.port, iface: cfg.interface.clone() };
    let mut socket = open_socket(&net)?;
    let shared: SharedVision = Arc::default();
    {
        let mut s = shared.lock().unwrap();
        s.kalman = cfg.kalman;
        s.net = Some(net);
        s.robot_id = robot_id;
        s.team = team;
    }
    let state = shared.clone();
    std::thread::Builder::new().name("vision".into()).spawn(move || {
        let mut buf = vec![0u8; 65536];
        let mut tracker = Tracker::new(cfg.clone());
        loop {
            // Switch socket if asked; on failure keep the current one.
            let reopen = state.lock().unwrap().reopen.take();
            if let Some(net) = reopen {
                let opened = open_socket(&net);
                let mut s = state.lock().unwrap();
                match opened {
                    Ok(new) => {
                        socket = new;
                        println!("vision: listening on {net}");
                        s.net = Some(net);
                        s.error = None;
                        s.seen.clear();
                        s.last_packet = None;
                    }
                    Err(e) => {
                        eprintln!("vision {net}: {e}");
                        s.error = Some(format!("{net}: {e}"));
                    }
                }
            }
            let len = match socket.recv_from(&mut buf) {
                Ok((len, _)) => len,
                Err(e) if matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => continue,
                Err(e) => {
                    eprintln!("vision: socket error: {e}");
                    std::thread::sleep(Duration::from_millis(100));
                    continue;
                }
            };
            let Ok(packet) = SSL_WrapperPacket::parse_from_bytes(&buf[..len]) else {
                continue;
            };
            let mut s = state.lock().unwrap();
            let (id, team) = (s.robot_id, s.team);
            tracker.handle(&packet, id, team, &mut s);
        }
    })?;
    Ok(shared)
}

/// Constant-velocity Kalman filter on one axis: state (position, velocity).
#[derive(Debug, Clone, Copy)]
struct Axis {
    p: f64,
    v: f64,
    cov: [[f64; 2]; 2],
}

impl Axis {
    fn new(p: f64, v: f64, r: f64) -> Self {
        Self { p, v, cov: [[r * r, 0.0], [0.0, 1.0]] }
    }

    /// Predict `dt` ahead, then correct with measurement `z`. `q`: acceleration
    /// noise std, `r`: measurement noise std, `angle`: wrap differences to (-pi, pi].
    fn step(&mut self, z: f64, dt: f64, q: f64, r: f64, angle: bool) {
        let [[a, b], [_, d]] = self.cov;
        let q2 = q * q;
        self.p += self.v * dt;
        let a = a + 2.0 * dt * b + dt * dt * d + q2 * dt.powi(4) / 4.0;
        let b = b + dt * d + q2 * dt.powi(3) / 2.0;
        let d = d + q2 * dt * dt;

        let y = if angle { wrap_angle(z - self.p) } else { z - self.p };
        let s = a + r * r;
        let (k0, k1) = (a / s, b / s);
        self.p += k0 * y;
        if angle {
            self.p = wrap_angle(self.p);
        }
        self.v += k1 * y;
        self.cov = [[(1.0 - k0) * a, (1.0 - k0) * b], [(1.0 - k0) * b, d - k1 * b]];
    }
}

/// Turns raw detections into the robot estimate.
pub struct Tracker {
    cfg: VisionConfig,
    /// x, y, theta filters; None until two detections in a row.
    filter: Option<[Axis; 3]>,
}

impl Tracker {
    pub fn new(cfg: VisionConfig) -> Self {
        Self { cfg, filter: None }
    }

    pub fn handle(&mut self, packet: &SSL_WrapperPacket, robot_id: u32, team: Team, s: &mut VisionState) {
        let Some(det) = packet.detection.as_ref() else { return };
        s.packets += 1;
        let now = Instant::now();
        s.last_packet = Some(now);
        for (name, list) in [("blue", &det.robots_blue), ("yellow", &det.robots_yellow)] {
            for r in list {
                let seen = Seen { x: r.x() as f64 / 1000.0, y: r.y() as f64 / 1000.0, camera: det.camera_id(), at: now };
                s.seen.insert((name, r.robot_id()), seen);
            }
        }

        if let Some(ball) = det.balls.iter().max_by(|a, b| a.confidence().total_cmp(&b.confidence())) {
            s.ball = Some((ball.x() as f64 / 1000.0, ball.y() as f64 / 1000.0));
        }

        let robots = match team {
            Team::Blue => &det.robots_blue,
            Team::Yellow => &det.robots_yellow,
        };
        let Some(r) = robots
            .iter()
            .filter(|r| r.robot_id() == robot_id)
            .max_by(|a, b| a.confidence().total_cmp(&b.confidence()))
        else {
            return;
        };

        let x = r.x() as f64 / 1000.0;
        let y = r.y() as f64 / 1000.0;
        if self.cfg.max_x > self.cfg.min_x && (x < self.cfg.min_x || x > self.cfg.max_x) {
            return;
        }
        let obs = RobotObs {
            x,
            y,
            theta: r.orientation() as f64,
            t_capture: det.t_capture(),
            camera: det.camera_id(),
            confidence: r.confidence(),
            frame: det.frame_number(),
            ..Default::default()
        };
        self.update(s, obs);
    }

    fn update(&mut self, s: &mut VisionState, mut obs: RobotObs) {
        if let Some(prev) = s.robot {
            let dt = obs.t_capture - prev.t_capture;
            // Same frame seen by a second camera, or an out-of-order packet.
            if dt < 1e-3 {
                return;
            }
            let k = s.kalman;
            if dt >= 0.5 {
                // A gap longer than half a second: restart the estimate.
                self.filter = None;
            } else if k.enabled {
                let f = self.filter.get_or_insert_with(|| {
                    [
                        Axis::new(prev.x, prev.vx, k.r_pos),
                        Axis::new(prev.y, prev.vy, k.r_pos),
                        Axis::new(prev.theta, prev.omega, k.r_theta),
                    ]
                });
                f[0].step(obs.x, dt, k.q_pos, k.r_pos, false);
                f[1].step(obs.y, dt, k.q_pos, k.r_pos, false);
                f[2].step(obs.theta, dt, k.q_theta, k.r_theta, true);
                (obs.vx, obs.vy, obs.omega) = (f[0].v, f[1].v, f[2].v);
            } else {
                self.filter = None;
                obs.vx = (obs.x - prev.x) / dt;
                obs.vy = (obs.y - prev.y) / dt;
                obs.omega = wrap_angle(obs.theta - prev.theta) / dt;
            }
        } else {
            self.filter = None;
        }
        s.robot = Some(obs);
        s.robot_seen = Some(Instant::now());
        s.detections += 1;
    }
}

/// Angle in (-pi, pi].
pub fn wrap_angle(a: f64) -> f64 {
    let mut a = a % std::f64::consts::TAU;
    if a > std::f64::consts::PI {
        a -= std::f64::consts::TAU;
    } else if a <= -std::f64::consts::PI {
        a += std::f64::consts::TAU;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::ssl_vision_detection::{SSL_DetectionFrame, SSL_DetectionRobot};

    pub fn packet(t: f64, id: u32, x_mm: f32, y_mm: f32, theta: f32) -> SSL_WrapperPacket {
        let mut robot = SSL_DetectionRobot::new();
        robot.set_robot_id(id);
        robot.set_x(x_mm);
        robot.set_y(y_mm);
        robot.set_orientation(theta);
        robot.set_confidence(0.9);
        robot.set_pixel_x(0.0);
        robot.set_pixel_y(0.0);
        let mut det = SSL_DetectionFrame::new();
        det.set_t_capture(t);
        det.set_t_sent(t);
        det.set_frame_number(1);
        det.set_camera_id(0);
        det.robots_blue.push(robot);
        let mut p = SSL_WrapperPacket::new();
        p.detection = protobuf::MessageField::some(det);
        p
    }

    #[test]
    fn tracks_only_our_robot_and_estimates_velocity() {
        let mut t = Tracker::new(VisionConfig::default());
        let mut s = VisionState::default();
        s.kalman.enabled = false; // raw finite differences
        t.handle(&packet(0.0, 3, 0.0, 0.0, 0.0), 3, Team::Blue, &mut s);
        t.handle(&packet(0.1, 3, 100.0, -50.0, 0.2), 3, Team::Blue, &mut s);
        t.handle(&packet(0.2, 4, 900.0, 900.0, 0.0), 3, Team::Blue, &mut s);
        t.handle(&packet(0.2, 3, 100.0, 0.0, 0.0), 3, Team::Yellow, &mut s);
        let r = s.robot.unwrap();
        assert!((r.x - 0.1).abs() < 1e-6 && (r.y + 0.05).abs() < 1e-6);
        assert!((r.vx - 1.0).abs() < 1e-4 && (r.vy + 0.5).abs() < 1e-4);
        assert!((r.omega - 2.0).abs() < 1e-4);
        assert_eq!(s.detections, 2);
        assert_eq!(s.seen.keys().copied().collect::<Vec<_>>(), [("blue", 3), ("blue", 4)]);
    }

    #[test]
    fn duplicate_camera_frames_are_ignored() {
        let mut t = Tracker::new(VisionConfig::default());
        let mut s = VisionState::default();
        t.handle(&packet(1.0, 0, 0.0, 0.0, 0.0), 0, Team::Blue, &mut s);
        t.handle(&packet(1.0, 0, 10.0, 0.0, 0.0), 0, Team::Blue, &mut s);
        assert_eq!(s.robot.unwrap().x, 0.0);
    }

    #[test]
    fn wraps_angles() {
        assert!((wrap_angle(3.5) - (3.5 - std::f64::consts::TAU)).abs() < 1e-12);
        assert!((wrap_angle(-3.5) - (-3.5 + std::f64::consts::TAU)).abs() < 1e-12);
    }
}
