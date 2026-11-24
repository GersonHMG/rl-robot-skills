//! Rhai scripts: `rhai <file>` opens a `.rhai` file and runs it in its own
//! thread. The script drives the robot by sending the same console commands
//! as typed ones, so every limit and safety check still applies.
//!
//!   vel(vx, vy, w)    robot-frame velocity (m/s, rad/s), held until changed
//!   wvel(vx, vy, w)   world-frame velocity
//!   stop()            zero velocity
//!   wait(secs)        sleep
//!   pose()            #{seen, x, y, theta, vx, vy, omega}
//!   time()            seconds since the script started
//!
//! A typed `stop` (or a new `rhai`) aborts the script. When it ends, the
//! robot is stopped.

use crate::vision::SharedVision;
use rhai::{Dynamic, Engine, EvalAltResult, Map};
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::time::{Duration, Instant};

type Res<T> = Result<T, Box<EvalAltResult>>;

/// A running script. Dropping it aborts the script.
pub struct Script {
    cancel: Arc<AtomicBool>,
    /// Console commands sent by the script.
    lines: mpsc::Receiver<String>,
}

impl Drop for Script {
    fn drop(&mut self) {
        self.cancel.store(true, Ordering::SeqCst);
    }
}

impl Script {
    pub fn spawn(path: String, vision: SharedVision, timeout: f64) -> Self {
        let cancel = Arc::new(AtomicBool::new(false));
        let (tx, lines) = mpsc::channel();
        let flag = cancel.clone();
        std::thread::spawn(move || {
            let result = run(&path, &tx, vision, timeout, &flag);
            if flag.load(Ordering::SeqCst) {
                println!("{path}: aborted");
                return;
            }
            match result {
                Ok(()) => println!("{path}: done"),
                Err(e) => eprintln!("{path}: {e}"),
            }
            let _ = tx.send("stop".into());
        });
        Self { cancel, lines }
    }

    /// Commands sent since the last call, and whether the script has ended.
    pub fn poll(&self) -> (Vec<String>, bool) {
        let mut out = Vec::new();
        loop {
            match self.lines.try_recv() {
                Ok(l) => out.push(l),
                Err(mpsc::TryRecvError::Empty) => return (out, false),
                Err(mpsc::TryRecvError::Disconnected) => return (out, true),
            }
        }
    }
}

fn num(v: &Dynamic) -> Res<f64> {
    v.as_float()
        .or_else(|_| v.as_int().map(|i| i as f64))
        .map_err(|t| format!("expected a number, got {t}").into())
}

fn run(path: &str, tx: &mpsc::Sender<String>, vision: SharedVision, timeout: f64, cancel: &Arc<AtomicBool>) -> Result<(), String> {
    let mut engine = Engine::new();
    let send = {
        let (tx, cancel) = (tx.clone(), cancel.clone());
        Rc::new(move |line: String| -> Res<()> {
            if cancel.load(Ordering::SeqCst) || tx.send(line).is_err() {
                return Err("aborted".into());
            }
            Ok(())
        })
    };

    for name in ["vel", "wvel"] {
        let send = send.clone();
        engine.register_fn(name, move |vx: Dynamic, vy: Dynamic, w: Dynamic| -> Res<()> {
            send(format!("{name} {} {} {}", num(&vx)?, num(&vy)?, num(&w)?))
        });
    }
    let s = send.clone();
    engine.register_fn("stop", move || s("stop".into()));

    let c = cancel.clone();
    engine.register_fn("wait", move |secs: Dynamic| -> Res<()> {
        let end = Instant::now() + Duration::from_secs_f64(num(&secs)?.clamp(0.0, 1e6));
        while let Some(left) = end.checked_duration_since(Instant::now()) {
            if c.load(Ordering::SeqCst) {
                return Err("aborted".into());
            }
            std::thread::sleep(left.min(Duration::from_millis(10)));
        }
        Ok(())
    });

    engine.register_fn("pose", move || -> Map {
        let r = vision.lock().unwrap().fresh_robot(timeout);
        let mut m = Map::new();
        m.insert("seen".into(), r.is_some().into());
        let r = r.unwrap_or_default();
        for (k, v) in [("x", r.x), ("y", r.y), ("theta", r.theta), ("vx", r.vx), ("vy", r.vy), ("omega", r.omega)] {
            m.insert(k.into(), v.into());
        }
        m
    });

    let start = Instant::now();
    engine.register_fn("time", move || start.elapsed().as_secs_f64());

    let c = cancel.clone();
    engine.on_progress(move |_| c.load(Ordering::SeqCst).then(|| "aborted".into()));

    engine.run_file(path.into()).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn script_sends_commands_then_stops() {
        let path = std::env::temp_dir().join(format!("small_engine_{}.rhai", std::process::id()));
        std::fs::write(&path, "vel(0.5, 0, 1); wait(0.01); if pose().seen { wvel(1, 2, 3) } stop();").unwrap();
        let script = Script::spawn(path.display().to_string(), SharedVision::default(), 0.3);
        let mut lines = Vec::new();
        loop {
            let (l, done) = script.poll();
            lines.extend(l);
            if done {
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(lines, ["vel 0.5 0 1", "stop", "stop"]);
        let _ = std::fs::remove_file(path);
    }
}
