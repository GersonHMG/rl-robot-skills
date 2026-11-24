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
//!
//! Parameters: a script may define `fn params()` returning an array of maps,
//! one per parameter:
//!
//!   #{ name: "radius", type: "float", value: 0.5, min: 0.1, max: 3.0, help: "m" }
//!
//! `type` is "float" (the default), "int", "bool" or "choice" (with
//! `options: ["a", "b"]`); `value` (the default; `default` itself is a
//! reserved word in Rhai), `min`, `max` and `help` are optional.
//! Values are given as `rhai <file> -- name=value ...` (the GUI builds this
//! line), checked against the declaration, and the script reads them from
//! the constant map `args` (`args.radius`). A parameter without a default
//! must be given.

use crate::vision::SharedVision;
use rhai::{Array, CallFnOptions, Dynamic, Engine, EvalAltResult, Map, Scope, AST};
use std::collections::HashMap;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::time::{Duration, Instant};

type Res<T> = Result<T, Box<EvalAltResult>>;

/// A running script. Dropping it aborts the script.
pub struct Script {
    pub path: String,
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
    /// `args`: `name=value` pairs for the script's declared parameters.
    pub fn spawn(path: String, args: Vec<String>, vision: SharedVision, timeout: f64) -> Self {
        let cancel = Arc::new(AtomicBool::new(false));
        let (tx, lines) = mpsc::channel();
        let flag = cancel.clone();
        let file = path.clone();
        std::thread::spawn(move || {
            let path = file;
            let result = run(&path, &args, &tx, vision, timeout, &flag);
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
        Self { path, cancel, lines }
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

/// Kind of a declared parameter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Kind {
    Float,
    Int,
    Bool,
    Choice,
}

/// One entry of a script's `params()`.
#[derive(Debug, Clone)]
pub struct Param {
    pub name: String,
    pub kind: Kind,
    /// Default value as text ("0.5", "true", "ccw"); None = must be given.
    pub default: Option<String>,
    pub min: Option<f64>,
    pub max: Option<f64>,
    /// Allowed values of a choice.
    pub options: Vec<String>,
    pub help: String,
}

impl Param {
    /// "[min, max]" for numbers, "" otherwise.
    pub fn range(&self) -> String {
        let f = |v: Option<f64>| v.map(|v| v.to_string()).unwrap_or_default();
        match (self.kind, self.min, self.max) {
            (Kind::Float | Kind::Int, None, None) => String::new(),
            (Kind::Float | Kind::Int, min, max) => format!("[{}, {}]", f(min), f(max)),
            _ => String::new(),
        }
    }
}

/// Reads the parameters a script declares, without running it.
pub fn read_params(path: &str) -> Result<Vec<Param>, String> {
    let engine = Engine::new();
    let ast = engine.compile_file(path.into()).map_err(|e| format!("{path}: {e}"))?;
    declared(&engine, &ast)
}

/// Calls `params()` (if the script defines it) without running the top level.
fn declared(engine: &Engine, ast: &AST) -> Result<Vec<Param>, String> {
    if !ast.iter_functions().any(|f| f.name == "params" && f.params.is_empty()) {
        return Ok(Vec::new());
    }
    let options = CallFnOptions::new().eval_ast(false);
    let list: Array = engine
        .call_fn_with_options(options, &mut Scope::new(), ast, "params", ())
        .map_err(|e| format!("params(): {e}"))?;
    list.into_iter().map(param).collect()
}

fn as_num(v: &Dynamic) -> Option<f64> {
    v.as_float().ok().or_else(|| v.as_int().ok().map(|i| i as f64))
}

fn param(entry: Dynamic) -> Result<Param, String> {
    let m = entry.try_cast::<Map>().ok_or("params(): each entry must be a map like #{name: \"x\"}")?;
    let text = |k: &str| m.get(k).filter(|v| !v.is_unit()).map(|v| v.to_string());
    let name = text("name").ok_or("params(): an entry has no `name`")?;
    if name.is_empty() || name.contains(|c: char| c.is_whitespace() || c == '=') {
        return Err(format!("params(): bad name `{name}`"));
    }
    let kind = match text("type").as_deref().unwrap_or("float") {
        "float" | "number" => Kind::Float,
        "int" => Kind::Int,
        "bool" => Kind::Bool,
        "choice" => Kind::Choice,
        t => return Err(format!("params(): {name}: unknown type `{t}` (float, int, bool, choice)")),
    };
    let options: Vec<String> = match m.get("options").and_then(|v| v.clone().try_cast::<Array>()) {
        Some(a) => a.iter().map(|o| o.to_string()).collect(),
        None => Vec::new(),
    };
    if kind == Kind::Choice && (options.is_empty() || options.iter().any(|o| o.is_empty() || o.contains(char::is_whitespace))) {
        return Err(format!("params(): {name}: a choice needs `options`, without spaces"));
    }
    Ok(Param {
        kind,
        default: text("value").or_else(|| text("default")),
        min: m.get("min").and_then(as_num),
        max: m.get("max").and_then(as_num),
        options,
        help: text("help").unwrap_or_default(),
        name,
    })
}

/// Parses and checks one value given as text.
pub fn check(p: &Param, value: &str) -> Result<Dynamic, String> {
    let s = value.trim();
    let name = &p.name;
    if s.is_empty() {
        return Err(format!("{name}: missing value"));
    }
    Ok(match p.kind {
        Kind::Float | Kind::Int => {
            let x: f64 = s.parse().ok().filter(|x: &f64| x.is_finite()).ok_or(format!("{name}: `{s}` is not a number"))?;
            if p.min.is_some_and(|m| x < m) || p.max.is_some_and(|m| x > m) {
                return Err(format!("{name}: {x} is outside {}", p.range()));
            }
            if p.kind == Kind::Int {
                if x.fract() != 0.0 {
                    return Err(format!("{name}: `{s}` is not a whole number"));
                }
                (x as i64).into()
            } else {
                x.into()
            }
        }
        Kind::Bool => match s {
            "true" | "1" | "on" | "yes" => true.into(),
            "false" | "0" | "off" | "no" => false.into(),
            _ => return Err(format!("{name}: `{s}` is not true / false")),
        },
        Kind::Choice if p.options.iter().any(|o| o == s) => s.into(),
        Kind::Choice => return Err(format!("{name}: `{s}` is not one of {}", p.options.join(", "))),
    })
}

/// Builds the `args` map from `name=value` pairs: unknown names are
/// refused, missing ones take their default.
pub fn build_args(params: &[Param], given: &[String]) -> Result<Map, String> {
    let mut values = HashMap::new();
    for g in given {
        let (k, v) = g.split_once('=').ok_or(format!("`{g}`: expected name=value"))?;
        if !params.iter().any(|p| p.name == k) {
            return Err(format!("unknown parameter `{k}`"));
        }
        values.insert(k, v);
    }
    let mut args = Map::new();
    for p in params {
        let v = values.get(p.name.as_str()).copied().or(p.default.as_deref()).unwrap_or("");
        args.insert(p.name.as_str().into(), check(p, v)?);
    }
    Ok(args)
}

fn num(v: &Dynamic) -> Res<f64> {
    v.as_float()
        .or_else(|_| v.as_int().map(|i| i as f64))
        .map_err(|t| format!("expected a number, got {t}").into())
}

fn run(
    path: &str,
    given: &[String],
    tx: &mpsc::Sender<String>,
    vision: SharedVision,
    timeout: f64,
    cancel: &Arc<AtomicBool>,
) -> Result<(), String> {
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

    let ast = engine.compile_file(path.into()).map_err(|e| e.to_string())?;
    let args = build_args(&declared(&engine, &ast)?, given)?;
    let mut scope = Scope::new();
    scope.push_constant("args", args);
    engine.run_ast_with_scope(&mut scope, &ast).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn script_sends_commands_then_stops() {
        let path = std::env::temp_dir().join(format!("small_engine_{}.rhai", std::process::id()));
        std::fs::write(&path, "vel(0.5, 0, 1); wait(0.01); if pose().seen { wvel(1, 2, 3) } stop();").unwrap();
        let script = Script::spawn(path.display().to_string(), Vec::new(), SharedVision::default(), 0.3);
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
