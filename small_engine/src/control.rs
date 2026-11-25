//! What the robot is doing (the mode) and the controller that turns it into a
//! robot-frame velocity command each tick.

use crate::config::ControlConfig;
use crate::radio::Command;
use crate::vision::{wrap_angle, RobotObs};
use std::collections::VecDeque;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    pub x: f64,
    pub y: f64,
    /// Heading to hold there; `None` keeps the current one.
    pub theta: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Mode {
    /// Zero velocity.
    Idle,
    /// Open loop: constant velocity, robot frame (`local`) or world frame,
    /// optionally until a time (s since start).
    Velocity { vx: f64, vy: f64, omega: f64, local: bool, until: Option<f64> },
    /// Closed loop through waypoints; idles after the last one.
    Path(VecDeque<Waypoint>),
    /// Manual driving (keyboard): robot frame, ends at `until` unless renewed.
    /// The only mode that keeps going without vision, like pilotar.
    Manual { vx: f64, vy: f64, omega: f64, until: f64 },
}

/// How long one `drive` command lasts. The window renews it every frame.
pub const MANUAL_LEASE: f64 = 0.25;

impl Mode {
    pub fn name(&self) -> &'static str {
        match self {
            Mode::Idle => "idle",
            Mode::Velocity { local: true, .. } => "local_vel",
            Mode::Velocity { .. } => "world_vel",
            Mode::Path(_) => "path",
            Mode::Manual { .. } => "manual",
        }
    }
}

pub struct Controller {
    pub cfg: ControlConfig,
    pub mode: Mode,
    /// Command sent last tick, robot frame (for the acceleration limit).
    last: Command,
    /// Kick on the next tick only; dribbler level held until changed.
    pub kick_pending: bool,
    pub dribbler: u8,
}

impl Controller {
    pub fn new(cfg: ControlConfig) -> Self {
        Self { cfg, mode: Mode::Idle, last: Command::default(), kick_pending: false, dribbler: 0 }
    }

    /// Current target, if following a path.
    pub fn target(&self) -> Option<Waypoint> {
        match &self.mode {
            Mode::Path(p) => p.front().copied(),
            _ => None,
        }
    }

    pub fn inside_field(&self, x: f64, y: f64) -> bool {
        let (l, w) = (self.cfg.field_half_length, self.cfg.field_half_width);
        (l <= 0.0 || x.abs() <= l) && (w <= 0.0 || y.abs() <= w)
    }

    /// One control tick. `robot` is `None` when vision lost the robot: then
    /// the robot is stopped at once (no ramp) and the mode is kept, so it
    /// resumes when it is seen again. Manual driving is the exception: it
    /// goes on blind (robot frame, short lease, same limits). Returns the
    /// command to send.
    pub fn step(&mut self, robot: Option<RobotObs>, now: f64, dt: f64) -> Command {
        let r = match robot {
            Some(r) => r,
            None if matches!(self.mode, Mode::Manual { .. }) => RobotObs::default(), // only its frame is used
            None => {
                self.last = Command::default();
                return Command { dribbler: self.dribbler, ..Default::default() };
            }
        };

        if robot.is_some() && !self.inside_field(r.x, r.y) && self.mode != Mode::Idle {
            eprintln!("safety: robot at ({:.2}, {:.2}) left the field limits, stopping", r.x, r.y);
            self.mode = Mode::Idle;
        }

        // Desired velocity: world frame (wx, wy) or robot frame (lx, ly).
        let (local, vx, vy, omega) = match &mut self.mode {
            Mode::Idle => (true, 0.0, 0.0, 0.0),
            Mode::Velocity { vx, vy, omega, local, until } => {
                if until.is_some_and(|t| now >= t) {
                    self.mode = Mode::Idle;
                    (true, 0.0, 0.0, 0.0)
                } else {
                    (*local, *vx, *vy, *omega)
                }
            }
            Mode::Manual { vx, vy, omega, until } => {
                if now >= *until {
                    self.mode = Mode::Idle;
                    (true, 0.0, 0.0, 0.0)
                } else {
                    (true, *vx, *vy, *omega)
                }
            }
            Mode::Path(points) => {
                // Drop every waypoint already reached.
                while let Some(p) = points.front() {
                    if points.len() > 1 && reached(&self.cfg, &r, p) {
                        points.pop_front();
                    } else {
                        break;
                    }
                }
                match points.front().copied() {
                    Some(p) if points.len() == 1 && reached(&self.cfg, &r, &p) => {
                        println!("reached ({:.3}, {:.3})", p.x, p.y);
                        self.mode = Mode::Idle;
                        (true, 0.0, 0.0, 0.0)
                    }
                    Some(p) => {
                        let (wx, wy, w) = go_to(&self.cfg, &r, &p);
                        (false, wx, wy, w)
                    }
                    None => {
                        self.mode = Mode::Idle;
                        (true, 0.0, 0.0, 0.0)
                    }
                }
            }
        };

        let (lx, ly) = if local { (vx, vy) } else { world_to_robot(vx, vy, r.theta) };
        let mut cmd = Command { vx: lx, vy: ly, omega, kick: false, dribbler: self.dribbler };
        self.limit(&mut cmd, dt);
        cmd.kick = std::mem::take(&mut self.kick_pending);
        self.last = cmd;
        cmd
    }

    /// Speed and acceleration limits, applied in the robot frame.
    fn limit(&self, cmd: &mut Command, dt: f64) {
        let c = &self.cfg;
        let speed = cmd.vx.hypot(cmd.vy);
        if speed > c.max_speed {
            cmd.vx *= c.max_speed / speed;
            cmd.vy *= c.max_speed / speed;
        }
        cmd.omega = cmd.omega.clamp(-c.max_omega, c.max_omega);

        let (dx, dy) = (cmd.vx - self.last.vx, cmd.vy - self.last.vy);
        let dv = dx.hypot(dy);
        let max_dv = c.max_accel * dt;
        if dv > max_dv {
            cmd.vx = self.last.vx + dx * max_dv / dv;
            cmd.vy = self.last.vy + dy * max_dv / dv;
        }
        let max_dw = c.max_alpha * dt;
        cmd.omega = self.last.omega + (cmd.omega - self.last.omega).clamp(-max_dw, max_dw);
    }
}

fn reached(cfg: &ControlConfig, r: &RobotObs, p: &Waypoint) -> bool {
    let close = (p.x - r.x).hypot(p.y - r.y) <= cfg.pos_tolerance;
    let facing = p.theta.is_none_or(|t| wrap_angle(t - r.theta).abs() <= cfg.angle_tolerance);
    close && facing
}

/// World-frame velocity toward `p`: the fastest speed from which the robot
/// can still brake at `max_accel` within the distance left, capped by the
/// max speed and by a proportional term near the target. Same for heading.
fn go_to(cfg: &ControlConfig, r: &RobotObs, p: &Waypoint) -> (f64, f64, f64) {
    let (dx, dy) = (p.x - r.x, p.y - r.y);
    let dist = dx.hypot(dy);
    let speed = if dist > 1e-6 {
        (2.0 * cfg.max_accel * dist).sqrt().min(cfg.max_speed).min(cfg.kp_pos * dist)
    } else {
        0.0
    };
    let (wx, wy) = if dist > 1e-6 { (dx / dist * speed, dy / dist * speed) } else { (0.0, 0.0) };

    let omega = match p.theta {
        Some(t) => {
            let e = wrap_angle(t - r.theta);
            let w = (2.0 * cfg.max_alpha * e.abs()).sqrt().min(cfg.max_omega).min(cfg.kp_angle * e.abs());
            w.copysign(e)
        }
        None => 0.0,
    };
    (wx, wy, omega)
}

/// World-frame vector seen from a robot facing `theta`.
pub fn world_to_robot(x: f64, y: f64, theta: f64) -> (f64, f64) {
    let (s, c) = theta.sin_cos();
    (c * x + s * y, -s * x + c * y)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn robot(x: f64, y: f64, theta: f64) -> Option<RobotObs> {
        Some(RobotObs { x, y, theta, ..Default::default() })
    }

    #[test]
    fn world_to_robot_rotates() {
        let (x, y) = world_to_robot(1.0, 0.0, std::f64::consts::FRAC_PI_2);
        assert!(x.abs() < 1e-12 && (y + 1.0).abs() < 1e-12);
    }

    #[test]
    fn stops_without_vision() {
        let mut c = Controller::new(ControlConfig::default());
        c.mode = Mode::Velocity { vx: 1.0, vy: 0.0, omega: 0.0, local: true, until: None };
        assert_eq!(c.step(None, 0.0, 0.016).vx, 0.0);
        assert!(matches!(c.mode, Mode::Velocity { .. }), "mode resumes when seen again");
    }

    #[test]
    fn manual_drives_blind_until_its_lease_ends() {
        let mut c = Controller::new(ControlConfig::default());
        c.mode = Mode::Manual { vx: 0.5, vy: 0.0, omega: 1.0, until: MANUAL_LEASE };
        let dt = 0.016;
        let mut cmd = Command::default();
        for i in 0..10 {
            cmd = c.step(None, i as f64 * dt, dt);
        }
        assert!(cmd.vx > 0.0 && cmd.omega > 0.0, "drives without vision: {cmd:?}");
        assert!(cmd.vx <= c.cfg.max_accel * 10.0 * dt + 1e-9, "acceleration still limited");
        c.step(None, MANUAL_LEASE, dt);
        assert_eq!(c.mode, Mode::Idle);
        assert_eq!(c.step(None, MANUAL_LEASE + dt, dt).vx, 0.0);
    }

    #[test]
    fn acceleration_is_limited() {
        let mut c = Controller::new(ControlConfig::default());
        c.mode = Mode::Velocity { vx: 1.0, vy: 0.0, omega: 0.0, local: true, until: None };
        let cmd = c.step(robot(0.0, 0.0, 0.0), 0.0, 0.01);
        assert!((cmd.vx - c.cfg.max_accel * 0.01).abs() < 1e-9);
    }

    #[test]
    fn timed_velocity_ends() {
        let mut c = Controller::new(ControlConfig::default());
        c.mode = Mode::Velocity { vx: 0.5, vy: 0.0, omega: 0.0, local: true, until: Some(1.0) };
        c.step(robot(0.0, 0.0, 0.0), 0.5, 0.016);
        c.step(robot(0.0, 0.0, 0.0), 1.0, 0.016);
        assert_eq!(c.mode, Mode::Idle);
    }

    #[test]
    fn leaving_the_field_stops() {
        let mut c = Controller::new(ControlConfig::default());
        c.mode = Mode::Velocity { vx: 0.5, vy: 0.0, omega: 0.0, local: true, until: None };
        c.step(robot(5.0, 0.0, 0.0), 0.0, 0.016);
        assert_eq!(c.mode, Mode::Idle);
    }

    /// Integrates the robot perfectly following the commands: it must reach
    /// the waypoint, facing the requested heading, and idle.
    #[test]
    fn path_converges() {
        let mut c = Controller::new(ControlConfig::default());
        c.mode = Mode::Path(VecDeque::from([
            Waypoint { x: 0.5, y: 0.0, theta: None },
            Waypoint { x: 0.5, y: 0.5, theta: Some(1.0) },
        ]));
        let (mut x, mut y, mut th) = (0.0, 0.0, 0.0);
        let dt = 1.0 / 60.0;
        for i in 0..600 {
            let cmd = c.step(robot(x, y, th), i as f64 * dt, dt);
            let (s, co) = th.sin_cos();
            x += (co * cmd.vx - s * cmd.vy) * dt;
            y += (s * cmd.vx + co * cmd.vy) * dt;
            th += cmd.omega * dt;
            if c.mode == Mode::Idle {
                break;
            }
        }
        assert_eq!(c.mode, Mode::Idle);
        assert!((x - 0.5).abs() < 0.03 && (y - 0.5).abs() < 0.03, "ended at ({x}, {y})");
        assert!((th - 1.0).abs() < 0.06, "heading {th}");
    }
}
