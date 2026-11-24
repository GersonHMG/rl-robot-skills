//! small_engine: receive SSL-Vision, control and record one real robot.
//!
//! Usage: small_engine [config.toml] [--run script.txt]
//! Type `help` at the prompt for the commands.

mod config;
mod console;
mod control;
mod radio;
mod recorder;
mod screen;
mod script;
mod vision;

#[allow(clippy::all, non_camel_case_types, non_snake_case, dead_code)]
mod proto {
    include!(concat!(env!("OUT_DIR"), "/protos/mod.rs"));
}

use config::Config;
use console::{Console, Effect};
use control::Controller;
use radio::{Command, Radio};
use recorder::{Recorder, Row};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let mut config_path = PathBuf::from("small_engine.toml");
    let mut script = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--run" => script = Some(args.next().ok_or("--run needs a file")?),
            "-h" | "--help" => {
                println!("usage: small_engine [config.toml] [--run script.txt]\n\n{}", console::HELP);
                return Ok(());
            }
            _ => config_path = PathBuf::from(a),
        }
    }

    let cfg = Config::load(&config_path)?;
    let vision = vision::spawn(cfg.vision.clone(), cfg.robot_id, cfg.team)
        .map_err(|e| format!("vision {}:{}: {e}", cfg.vision.multicast_addr, cfg.vision.port))?;
    let radio = Radio::open(&cfg.output)?;
    let (commands, lines) = std::sync::mpsc::channel();

    println!(
        "small_engine: robot {} ({:?}), vision {}:{}, output {:?}, {} Hz. Type `help`.",
        cfg.robot_id, cfg.team, cfg.vision.multicast_addr, cfg.vision.port, cfg.output.kind, cfg.rate_hz
    );

    let running = Arc::new(AtomicBool::new(true));
    {
        let running = running.clone();
        ctrlc::set_handler(move || running.store(false, Ordering::SeqCst)).map_err(|e| e.to_string())?;
    }
    console::spawn_stdin(commands.clone());

    // The control loop runs in its own thread; the window needs the main one.
    let screen_state = screen::SharedState::default();
    let vision_view = vision.clone();
    let engine = {
        let (cfg, running, screen_state) = (cfg.clone(), running.clone(), screen_state.clone());
        std::thread::spawn(move || {
            engine_loop(&cfg, vision, radio, lines, &running, &screen_state, script);
            running.store(false, Ordering::SeqCst); // closes the window
        })
    };
    if cfg.screen {
        match screen::run(&cfg, screen_state, vision_view, commands, running.clone()) {
            Ok(()) => running.store(false, Ordering::SeqCst), // window closed: stop the engine
            Err(e) => eprintln!("screen: {e} (continuing without it)"),
        }
    }
    engine.join().map_err(|_| "control loop panicked".to_string())
}

/// Runs until `quit`, Ctrl+C or the window closes, then leaves the robot still.
/// Drops (aborts) the running script and stops the recording it started.
fn end_script(rhai: &mut Option<script::Script>, script_rec: &mut bool, console: &mut console::Console) {
    *rhai = None;
    if std::mem::take(script_rec) {
        console.push("rec stop".into());
    }
}

fn engine_loop(
    cfg: &Config,
    vision: vision::SharedVision,
    mut radio: Radio,
    lines: std::sync::mpsc::Receiver<String>,
    running: &AtomicBool,
    screen_state: &screen::SharedState,
    script: Option<String>,
) {
    let mut ctrl = Controller::new(cfg.control.clone());
    let mut console = Console::new();
    let mut recorder: Option<Recorder> = None;
    let mut rhai: Option<script::Script> = None;
    let mut script_rec = false; // the running script started the recording
    let mut output = cfg.output.clone();
    if let Some(path) = script {
        console.queue(&format!("run {path}"));
    }

    let dt = cfg.tick();
    let start = Instant::now();
    let mut next = start;
    let mut was_seen = true;

    while running.load(Ordering::SeqCst) {
        let now = start.elapsed().as_secs_f64();
        let (robot, ball) = {
            let v = vision.lock().unwrap();
            (v.fresh_robot(cfg.vision.timeout), v.ball)
        };

        while let Ok(line) = lines.try_recv() {
            let first = line.split_whitespace().next();
            if matches!(first, Some("rec" | "record")) {
                script_rec = false; // the user took over the recording
            }
            console.queue(&line);
            if matches!(first, Some("stop" | "s" | "quit" | "q" | "exit")) {
                end_script(&mut rhai, &mut script_rec, &mut console); // aborts it
            }
        }
        if let Some(s) = &rhai {
            let (cmds, done) = s.poll();
            for l in cmds {
                if l.starts_with("rec") {
                    script_rec = l != "rec stop";
                }
                console.push(l);
            }
            if done {
                end_script(&mut rhai, &mut script_rec, &mut console);
            }
        }
        let ctx = console::Context { now, robot, ctrl: &mut ctrl, vision: &vision, recording: recorder.as_ref() };
        match console.step(ctx) {
            Effect::None => {}
            Effect::Quit => break,
            Effect::StartRecording(name) => {
                if let Some(r) = recorder.take() {
                    report_recording(r);
                }
                match Recorder::start(cfg.record_dir.as_ref(), name.as_deref(), &meta(cfg, &vision.lock().unwrap().kalman)) {
                    Ok(r) => {
                        println!("recording to {}", r.path.display());
                        recorder = Some(r);
                    }
                    Err(e) => eprintln!("cannot record: {e}"),
                }
            }
            Effect::RunScript(path, args) => {
                println!("running {path} (type `stop` to abort)");
                end_script(&mut rhai, &mut script_rec, &mut console);
                rhai = Some(script::Script::spawn(path, args, vision.clone(), cfg.vision.timeout));
            }
            Effect::Radio(args) if args.is_empty() => println!("radio: {}", radio::describe(&output)),
            Effect::Radio(args) => match radio::configure(&output, &args) {
                Ok(new) => {
                    radio.send(cfg.robot_id, cfg.team, &Command::default());
                    radio = Radio::None; // release the port before reopening it
                    match Radio::open(&new) {
                        Ok(r) => {
                            radio = r;
                            output = new;
                            println!("radio: {}", radio::describe(&output));
                        }
                        Err(e) => {
                            output.kind = config::OutputKind::None;
                            eprintln!("radio: {e}; output is now none");
                        }
                    }
                }
                Err(e) => eprintln!("radio: {e}"),
            },
            Effect::StopRecording => match recorder.take() {
                Some(r) => report_recording(r),
                None => println!("not recording"),
            },
        }

        if robot.is_none() && was_seen && ctrl.mode != control::Mode::Idle {
            eprintln!("vision: robot {} not seen, holding still", cfg.robot_id);
        }
        was_seen = robot.is_some();

        // Mode and target as they were during this tick.
        let (mode, target) = (ctrl.mode.name(), ctrl.target());
        let cmd = ctrl.step(robot, now, dt);
        radio.send(cfg.robot_id, cfg.team, &cmd);
        *screen_state.lock().unwrap() = screen::State {
            robot,
            ball,
            target,
            mode,
            radio: radio::describe(&output),
            script: rhai.as_ref().map(|s| s.path.clone()),
            recording: recorder.as_ref().map(|r| r.path.file_name().unwrap_or_default().to_string_lossy().into_owned()),
        };

        if let Some(rec) = recorder.as_mut() {
            let row = Row { time: now, robot, cmd, mode, ball };
            if let Err(e) = rec.write(&row) {
                eprintln!("recording failed, stopped: {e}");
                recorder = None;
            }
        }

        next += Duration::from_secs_f64(dt);
        let now = Instant::now();
        if next > now {
            std::thread::sleep(next - now);
        } else if now - next > Duration::from_secs_f64(dt * 5.0) {
            next = now; // fell far behind: don't try to catch up
        }
    }

    // Leave the robot standing still.
    for _ in 0..5 {
        radio.send(cfg.robot_id, cfg.team, &Command::default());
        std::thread::sleep(Duration::from_secs_f64(dt));
    }
    if let Some(r) = recorder.take() {
        report_recording(r);
    }
    println!("bye");
}

fn report_recording(r: Recorder) {
    match r.finish() {
        Ok((path, rows)) => println!("saved {rows} rows to {}", path.display()),
        Err(e) => eprintln!("recording not saved: {e}"),
    }
}

fn meta(cfg: &Config, k: &config::KalmanConfig) -> String {
    let c = &cfg.control;
    format!(
        "robot_id = {}\nteam = \"{:?}\"\nrate_hz = {}\noutput = \"{:?}\"\n\
         max_speed = {}\nmax_accel = {}\nmax_omega = {}\nmax_alpha = {}\n\
         kf_enabled = {}\nkf_q_pos = {}\nkf_r_pos = {}\nkf_q_theta = {}\nkf_r_theta = {}\n",
        cfg.robot_id,
        cfg.team,
        cfg.rate_hz,
        cfg.output.kind,
        c.max_speed,
        c.max_accel,
        c.max_omega,
        c.max_alpha,
        k.enabled,
        k.q_pos,
        k.r_pos,
        k.q_theta,
        k.r_theta
    )
}
