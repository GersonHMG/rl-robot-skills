//! Debug screen: a tiny web page (default http://127.0.0.1:8080) that draws
//! the field and where the robot is. No dependencies: std TcpListener plus
//! one embedded HTML file that polls `/state` (a JSON snapshot). The record
//! button posts to `/rec/start` or `/rec/stop`, which queue the console's
//! `rec` / `rec stop` commands.

use crate::config::{Config, Team};
use crate::control::Waypoint;
use crate::recorder::Recorder;
use crate::vision::RobotObs;
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{mpsc, Arc, Mutex};
use std::time::Duration;

const PAGE: &str = include_str!("screen.html");

/// Latest JSON snapshot, rewritten by the control loop every tick.
pub type SharedState = Arc<Mutex<String>>;

/// `commands` is the console's input: lines sent there run like typed ones.
pub fn spawn(addr: &str, state: SharedState, commands: mpsc::Sender<String>) -> std::io::Result<()> {
    let listener = TcpListener::bind(addr)?;
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            let (state, commands) = (state.clone(), commands.clone());
            std::thread::spawn(move || handle(stream, &state, &commands));
        }
    });
    Ok(())
}

fn handle(stream: TcpStream, state: &SharedState, commands: &mpsc::Sender<String>) -> std::io::Result<()> {
    stream.set_read_timeout(Some(Duration::from_secs(2)))?;
    let mut reader = BufReader::new(&stream);
    let mut request = String::new();
    reader.read_line(&mut request)?;
    loop {
        let mut header = String::new();
        if reader.read_line(&mut header)? == 0 || header.trim().is_empty() {
            break;
        }
    }
    let mut parts = request.split_whitespace();
    let (method, path) = (parts.next().unwrap_or(""), parts.next().unwrap_or(""));
    let (status, ctype, body) = match (method, path) {
        ("GET", "/") => ("200 OK", "text/html", PAGE.to_string()),
        ("GET", "/state") => ("200 OK", "application/json", state.lock().unwrap().clone()),
        ("POST", "/rec/start" | "/rec/stop") => {
            let cmd = if path == "/rec/start" { "rec" } else { "rec stop" };
            let _ = commands.send(cmd.to_string());
            ("200 OK", "text/plain", "ok".to_string())
        }
        _ => ("404 Not Found", "text/plain", "not found".to_string()),
    };
    let mut out = &stream;
    write!(
        out,
        "HTTP/1.1 {status}\r\nContent-Type: {ctype}; charset=utf-8\r\nContent-Length: {}\r\n\
         Cache-Control: no-store\r\nConnection: close\r\n\r\n{body}",
        body.len()
    )
}

/// JSON for the page. Hand-written to avoid a serde_json dependency.
pub fn snapshot(
    cfg: &Config,
    robot: Option<RobotObs>,
    ball: Option<(f64, f64)>,
    mode: &str,
    target: Option<Waypoint>,
    recording: Option<&Recorder>,
) -> String {
    let robot = robot.map_or("null".into(), |r| {
        format!(r#"{{"x":{:.4},"y":{:.4},"theta":{:.4},"vx":{:.3},"vy":{:.3},"omega":{:.3}}}"#, r.x, r.y, r.theta, r.vx, r.vy, r.omega)
    });
    let ball = ball.map_or("null".into(), |(x, y)| format!(r#"{{"x":{x:.4},"y":{y:.4}}}"#));
    let target = target.map_or("null".into(), |t| format!(r#"{{"x":{:.4},"y":{:.4}}}"#, t.x, t.y));
    let recording = recording.map_or("null".into(), |r| {
        format!(r#""{}""#, r.path.file_name().unwrap_or_default().to_string_lossy().replace(['"', '\\'], ""))
    });
    let team = if cfg.team == Team::Blue { "blue" } else { "yellow" };
    format!(
        r#"{{"id":{},"team":"{team}","hl":{},"hw":{},"mode":"{mode}","robot":{robot},"ball":{ball},"target":{target},"recording":{recording}}}"#,
        cfg.robot_id, cfg.control.field_half_length, cfg.control.field_half_width
    )
}
