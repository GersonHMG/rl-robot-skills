//! Debug screen: a tiny web page (default http://127.0.0.1:8080) that draws
//! the field and where the robot is. No dependencies: std TcpListener plus
//! one embedded HTML file that polls `/state` (a JSON snapshot).

use crate::config::{Config, Team};
use crate::control::Waypoint;
use crate::vision::RobotObs;
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::time::Duration;

const PAGE: &str = include_str!("screen.html");

/// Latest JSON snapshot, rewritten by the control loop every tick.
pub type SharedState = Arc<Mutex<String>>;

pub fn spawn(addr: &str, state: SharedState) -> std::io::Result<()> {
    let listener = TcpListener::bind(addr)?;
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            let state = state.clone();
            std::thread::spawn(move || handle(stream, &state));
        }
    });
    Ok(())
}

fn handle(stream: TcpStream, state: &SharedState) -> std::io::Result<()> {
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
) -> String {
    let robot = robot.map_or("null".into(), |r| {
        format!(r#"{{"x":{:.4},"y":{:.4},"theta":{:.4},"vx":{:.3},"vy":{:.3},"omega":{:.3}}}"#, r.x, r.y, r.theta, r.vx, r.vy, r.omega)
    });
    let ball = ball.map_or("null".into(), |(x, y)| format!(r#"{{"x":{x:.4},"y":{y:.4}}}"#));
    let target = target.map_or("null".into(), |t| format!(r#"{{"x":{:.4},"y":{:.4}}}"#, t.x, t.y));
    let team = if cfg.team == Team::Blue { "blue" } else { "yellow" };
    format!(
        r#"{{"id":{},"team":"{team}","hl":{},"hw":{},"mode":"{mode}","robot":{robot},"ball":{ball},"target":{target}}}"#,
        cfg.robot_id, cfg.control.field_half_length, cfg.control.field_half_width
    )
}
