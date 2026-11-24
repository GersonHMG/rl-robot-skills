//! Text commands, typed at the prompt or read from a script file. Scripts
//! can `wait` and `waitidle`, so a test run (drive, record, repeat) can be
//! written once and replayed exactly.

use crate::control::{Controller, Mode, Waypoint};
use crate::recorder::Recorder;
use crate::vision::{RobotObs, SharedVision};
use std::collections::VecDeque;
use std::sync::mpsc;

pub const HELP: &str = "\
Units: metres, seconds, radians. Velocities in m/s and rad/s.
  stop | s                    stop now (also clears a running script)
  vel <vx> <vy> <w> [secs]    open-loop velocity, robot frame (vx forward)
  wvel <vx> <vy> <w> [secs]   open-loop velocity, world frame
  goto <x> <y> [theta]        drive to a point (and heading)
  path <x> <y> [<x> <y> ...]  drive through points
  square <side>               square path starting at the current position
  turn <theta>                turn in place to a heading
  kick                        kick once
  dribble <0-7>               dribbler level
  set <key> <value>           max_speed max_accel max_omega max_alpha kp_pos kp_angle
  rec [name] | rec stop       start / stop CSV recording
  status                      pose, velocity, mode, vision counters
  run <file>                  run a script (same commands, plus `wait <secs>`, `waitidle`, # comments)
  radio [none | grsim [addr] | serial <port> [baud]]   show / change the output
  rhai <file>                 run a Rhai script: vel() wvel() stop() wait() pose() time()
  help | quit";

pub enum Effect {
    None,
    Quit,
    StartRecording(Option<String>),
    StopRecording,
    RunScript(String),
    /// `radio` arguments (empty = show the current output).
    Radio(Vec<String>),
}

pub struct Context<'a> {
    pub now: f64,
    pub robot: Option<RobotObs>,
    pub ctrl: &'a mut Controller,
    pub vision: &'a SharedVision,
    pub recording: Option<&'a Recorder>,
}

enum Wait {
    None,
    Until(f64),
    Idle,
}

pub struct Console {
    pending: VecDeque<String>,
    wait: Wait,
}

/// Forwards stdin lines to `tx`.
pub fn spawn_stdin(tx: mpsc::Sender<String>) {
    std::thread::spawn(move || {
        for line in std::io::stdin().lines() {
            let Ok(line) = line else { break };
            if tx.send(line).is_err() {
                break;
            }
        }
    });
}

impl Console {
    pub fn new() -> Self {
        Self { pending: VecDeque::new(), wait: Wait::None }
    }

    /// Typed lines: `stop` acts at once, even while a script waits.
    pub fn queue(&mut self, line: &str) {
        let first = line.split_whitespace().next().unwrap_or("");
        if matches!(first, "stop" | "s" | "quit" | "q") {
            if !self.pending.is_empty() || !matches!(self.wait, Wait::None) {
                println!("script aborted");
            }
            self.pending.clear();
            self.wait = Wait::None;
            self.pending.push_front(line.to_string());
        } else {
            self.pending.push_back(line.to_string());
        }
    }

    /// Lines from a running Rhai script: plain steps, in order.
    pub fn push(&mut self, line: String) {
        self.pending.push_back(line);
    }

    /// Runs queued commands until one has to wait or one has an effect.
    pub fn step(&mut self, ctx: Context) -> Effect {
        match self.wait {
            Wait::Until(t) if ctx.now < t => return Effect::None,
            Wait::Idle if ctx.ctrl.mode != Mode::Idle => return Effect::None,
            _ => self.wait = Wait::None,
        }
        let Context { now, robot, ctrl, vision, recording } = ctx;
        while let Some(line) = self.pending.pop_front() {
            match self.exec(&line, now, robot, ctrl, vision, recording) {
                Ok(Effect::None) => {}
                Ok(effect) => return effect,
                Err(e) => {
                    eprintln!("`{}`: {e}", line.trim());
                    if !self.pending.is_empty() {
                        eprintln!("script aborted");
                        self.pending.clear();
                    }
                }
            }
            if !matches!(self.wait, Wait::None) {
                break;
            }
        }
        Effect::None
    }

    fn exec(
        &mut self,
        line: &str,
        now: f64,
        robot: Option<RobotObs>,
        ctrl: &mut Controller,
        vision: &SharedVision,
        recording: Option<&Recorder>,
    ) -> Result<Effect, String> {
        let line = line.split('#').next().unwrap_or("").trim();
        let mut words = line.split_whitespace();
        let Some(cmd) = words.next() else { return Ok(Effect::None) };
        let args: Vec<&str> = words.collect();
        let nums = || -> Result<Vec<f64>, String> {
            args.iter().map(|a| a.parse::<f64>().map_err(|_| format!("`{a}` is not a number"))).collect()
        };
        let need_robot = || robot.ok_or_else(|| "robot not seen by vision".to_string());

        match cmd {
            "help" | "h" | "?" => println!("{HELP}"),
            "quit" | "q" | "exit" => return Ok(Effect::Quit),
            "stop" | "s" => ctrl.mode = Mode::Idle,
            "vel" | "wvel" => {
                let n = nums()?;
                if !(3..=4).contains(&n.len()) {
                    return Err("needs <vx> <vy> <w> [secs]".into());
                }
                ctrl.mode = Mode::Velocity {
                    vx: n[0],
                    vy: n[1],
                    omega: n[2],
                    local: cmd == "vel",
                    until: n.get(3).map(|s| now + s),
                };
            }
            "goto" => {
                let n = nums()?;
                if !(2..=3).contains(&n.len()) {
                    return Err("needs <x> <y> [theta]".into());
                }
                set_path(ctrl, vec![Waypoint { x: n[0], y: n[1], theta: n.get(2).copied() }])?;
            }
            "path" => {
                let n = nums()?;
                if n.is_empty() || n.len() % 2 != 0 {
                    return Err("needs pairs of <x> <y>".into());
                }
                set_path(ctrl, n.chunks(2).map(|p| Waypoint { x: p[0], y: p[1], theta: None }).collect())?;
            }
            "square" => {
                let n = nums()?;
                let side = *n.first().ok_or("needs <side>")?;
                let r = need_robot()?;
                let corners = [(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)];
                set_path(ctrl, corners.iter().map(|(dx, dy)| Waypoint { x: r.x + dx, y: r.y + dy, theta: None }).collect())?;
            }
            "turn" => {
                let n = nums()?;
                let theta = *n.first().ok_or("needs <theta>")?;
                let r = need_robot()?;
                set_path(ctrl, vec![Waypoint { x: r.x, y: r.y, theta: Some(theta) }])?;
            }
            "kick" => ctrl.kick_pending = true,
            "dribble" => {
                let n = nums()?;
                ctrl.dribbler = n.first().ok_or("needs <0-7>")?.clamp(0.0, 7.0) as u8;
            }
            "set" => {
                let [key, value] = args[..] else { return Err("needs <key> <value>".into()) };
                let v: f64 = value.parse().map_err(|_| format!("`{value}` is not a number"))?;
                let c = &mut ctrl.cfg;
                let slot = match key {
                    "max_speed" => &mut c.max_speed,
                    "max_accel" => &mut c.max_accel,
                    "max_omega" => &mut c.max_omega,
                    "max_alpha" => &mut c.max_alpha,
                    "kp_pos" => &mut c.kp_pos,
                    "kp_angle" => &mut c.kp_angle,
                    "pos_tolerance" => &mut c.pos_tolerance,
                    "angle_tolerance" => &mut c.angle_tolerance,
                    _ => return Err(format!("unknown key `{key}`")),
                };
                *slot = v;
                println!("{key} = {v}");
            }
            "rec" | "record" => {
                return Ok(match args.first() {
                    Some(&"stop") => Effect::StopRecording,
                    name => Effect::StartRecording(name.map(|s| s.to_string())),
                });
            }
            "status" | "st" => {
                let v = vision.lock().unwrap();
                match robot {
                    Some(r) => println!(
                        "pose ({:.3}, {:.3}, {:.3} rad)  vel ({:.2}, {:.2}, {:.2})  cam {}",
                        r.x, r.y, r.theta, r.vx, r.vy, r.omega, r.camera
                    ),
                    None => println!("robot not seen"),
                }
                println!(
                    "mode {}  target {:?}  vision packets {} detections {}  recording {}",
                    ctrl.mode.name(),
                    ctrl.target().map(|t| (t.x, t.y, t.theta)),
                    v.packets,
                    v.detections,
                    recording.map(|r| r.path.display().to_string()).unwrap_or_else(|| "off".into())
                );
            }
            "wait" => {
                let n = nums()?;
                self.wait = Wait::Until(now + n.first().ok_or("needs <secs>")?);
            }
            "waitidle" => self.wait = Wait::Idle,
            "radio" => return Ok(Effect::Radio(args.iter().map(|a| a.to_string()).collect())),
            "rhai" => return Ok(Effect::RunScript(args.first().ok_or("needs <file>")?.to_string())),
            "run" => {
                let path = args.first().ok_or("needs <file>")?;
                let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
                for l in text.lines().rev() {
                    self.pending.push_front(l.to_string());
                }
                println!("running {path}");
            }
            _ => return Err("unknown command (try `help`)".into()),
        }
        Ok(Effect::None)
    }
}

fn set_path(ctrl: &mut Controller, points: Vec<Waypoint>) -> Result<(), String> {
    if let Some(p) = points.iter().find(|p| !ctrl.inside_field(p.x, p.y)) {
        return Err(format!("({}, {}) is outside the field limits", p.x, p.y));
    }
    ctrl.mode = Mode::Path(points.into());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::ControlConfig;

    fn step(c: &mut Console, ctrl: &mut Controller, now: f64) -> Effect {
        let vision = SharedVision::default();
        let robot = Some(RobotObs::default());
        c.step(Context { now, robot, ctrl, vision: &vision, recording: None })
    }

    #[test]
    fn script_waits_then_continues() {
        let mut c = Console::new();
        let mut ctrl = Controller::new(ControlConfig::default());
        // As loaded by `run`: a `stop` inside a script is just a step.
        for l in ["vel 0.5 0 0", "wait 1.0", "stop  # comment", "goto 1 1"] {
            c.pending.push_back(l.to_string());
        }
        step(&mut c, &mut ctrl, 0.0);
        assert_eq!(ctrl.mode.name(), "local_vel");
        step(&mut c, &mut ctrl, 0.5);
        assert_eq!(ctrl.mode.name(), "local_vel");
        step(&mut c, &mut ctrl, 1.0);
        assert_eq!(ctrl.mode.name(), "path");
    }

    #[test]
    fn typed_stop_aborts_a_script() {
        let mut c = Console::new();
        let mut ctrl = Controller::new(ControlConfig::default());
        for l in ["vel 0.5 0 0", "wait 10", "goto 1 1"] {
            c.queue(l);
        }
        step(&mut c, &mut ctrl, 0.0);
        c.queue("stop");
        step(&mut c, &mut ctrl, 0.1);
        assert_eq!(ctrl.mode, Mode::Idle);
        step(&mut c, &mut ctrl, 20.0);
        assert_eq!(ctrl.mode, Mode::Idle);
    }

    #[test]
    fn refuses_points_outside_the_field() {
        let mut c = Console::new();
        let mut ctrl = Controller::new(ControlConfig::default());
        c.queue("goto 9 0");
        step(&mut c, &mut ctrl, 0.0);
        assert_eq!(ctrl.mode, Mode::Idle);
    }
}
