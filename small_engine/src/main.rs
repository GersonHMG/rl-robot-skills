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
    let mut radio = Radio::open(&cfg.output)?;
    let mut ctrl = Controller::new(cfg.control.clone());
    let mut console = Console::new();
    let mut recorder: Option<Recorder> = None;
    let mut rhai: Option<script::Script> = None;
    let (commands, lines) = std::sync::mpsc::channel();
    let screen_state = screen::SharedState::default();
    if !cfg.screen.is_empty() {
        screen::spawn(&cfg.screen, screen_state.clone(), commands.clone()).map_err(|e| format!("screen {}: {e}", cfg.screen))?;
        println!("debug screen: http://{}", cfg.screen);
    }

    println!(
        "small_engine: robot {} ({:?}), vision {}:{}, output {:?}, {} Hz. Type `help`.",
        cfg.robot_id, cfg.team, cfg.vision.multicast_addr, cfg.vision.port, cfg.output.kind, cfg.rate_hz
    );

    let running = Arc::new(AtomicBool::new(true));
    {
        let running = running.clone();
        ctrlc::set_handler(move || running.store(false, Ordering::SeqCst)).map_err(|e| e.to_string())?;
    }
    console::spawn_stdin(commands);
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
            if matches!(line.split_whitespace().next(), Some("stop" | "s" | "quit" | "q" | "exit")) {
                rhai = None; // aborts it
            }
            console.queue(&line);
        }
        if let Some(s) = &rhai {
            let (cmds, done) = s.poll();
            for l in cmds {
                console.push(l);
            }
            if done {
                rhai = None;
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
                match Recorder::start(cfg.record_dir.as_ref(), name.as_deref(), &meta(&cfg)) {
                    Ok(r) => {
                        println!("recording to {}", r.path.display());
                        recorder = Some(r);
                    }
                    Err(e) => eprintln!("cannot record: {e}"),
                }
            }
            Effect::RunScript(path) => {
                println!("running {path} (type `stop` to abort)");
                rhai = Some(script::Script::spawn(path, vision.clone(), cfg.vision.timeout));
            }
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
        *screen_state.lock().unwrap() = screen::snapshot(&cfg, robot, ball, mode, target, recorder.as_ref());

        if let Some(rec) = recorder.as_mut() {
            let row = Row { time: now, robot, cmd, mode, target, ball };
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
    Ok(())
}

fn report_recording(r: Recorder) {
    match r.finish() {
        Ok((path, rows)) => println!("saved {rows} rows to {}", path.display()),
        Err(e) => eprintln!("recording not saved: {e}"),
    }
}

fn meta(cfg: &Config) -> String {
    let c = &cfg.control;
    format!(
        "robot_id = {}\nteam = \"{:?}\"\nrate_hz = {}\noutput = \"{:?}\"\n\
         max_speed = {}\nmax_accel = {}\nmax_omega = {}\nmax_alpha = {}\n",
        cfg.robot_id, cfg.team, cfg.rate_hz, cfg.output.kind, c.max_speed, c.max_accel, c.max_omega, c.max_alpha
    )
}
