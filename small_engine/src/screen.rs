//! Debug screen: a native window (egui) that opens with the engine. It draws
//! the field with the robot, its heading, the ball and the current target,
//! and has a record button that sends the console's `rec` / `rec stop`.

use crate::config::{Config, OutputKind, Team};
use crate::control::Waypoint;
use crate::vision::RobotObs;
use eframe::egui::{self, Color32, Pos2, Stroke, Vec2};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::Duration;

/// What the control loop shows, rewritten every tick.
#[derive(Clone, Default)]
pub struct State {
    pub robot: Option<RobotObs>,
    pub ball: Option<(f64, f64)>,
    pub target: Option<Waypoint>,
    pub mode: &'static str,
    /// Current output, e.g. "serial COM3 @ 115200".
    pub radio: String,
    /// Path of the running Rhai script.
    pub script: Option<String>,
    /// File name of the active recording.
    pub recording: Option<String>,
}

pub type SharedState = Arc<Mutex<State>>;

struct Screen {
    state: SharedState,
    /// The console's input: lines sent here run like typed ones.
    commands: mpsc::Sender<String>,
    /// Cleared when the engine stops; the window then closes.
    running: Arc<AtomicBool>,
    robot_id: u32,
    color: Color32,
    half_length: f32,
    half_width: f32,
    /// Radio window and its fields.
    show_radio: bool,
    kind: OutputKind,
    port: String,
    baud: String,
    grsim_addr: String,
    ports: Vec<String>,
    /// Rhai script picked with "Open script".
    script_path: Option<String>,
}

fn serial_ports() -> Vec<String> {
    serialport::available_ports().unwrap_or_default().into_iter().map(|p| p.port_name).collect()
}

/// Opens the window; returns when it is closed.
pub fn run(cfg: &Config, state: SharedState, commands: mpsc::Sender<String>, running: Arc<AtomicBool>) -> Result<(), String> {
    let (l, w) = (cfg.control.field_half_length, cfg.control.field_half_width);
    let screen = Screen {
        state,
        commands,
        running,
        robot_id: cfg.robot_id,
        color: if cfg.team == Team::Blue { Color32::from_rgb(58, 123, 255) } else { Color32::from_rgb(245, 208, 0) },
        half_length: if l > 0.0 { l as f32 } else { 4.5 },
        half_width: if w > 0.0 { w as f32 } else { 3.0 },
        show_radio: false,
        kind: cfg.output.kind,
        port: cfg.output.serial_port.clone(),
        baud: cfg.output.baud_rate.to_string(),
        grsim_addr: cfg.output.grsim_addr.clone(),
        ports: Vec::new(),
        script_path: None,
    };
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_title("small_engine").with_inner_size([900.0, 650.0]),
        ..Default::default()
    };
    eframe::run_native("small_engine", options, Box::new(|_| Ok(Box::new(screen)))).map_err(|e| e.to_string())
}

impl eframe::App for Screen {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        if !self.running.load(Ordering::SeqCst) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        let s = self.state.lock().unwrap().clone();

        egui::Panel::top("bar").show(ui, |ui| {
            ui.horizontal(|ui| {
                let recording = s.recording.is_some();
                let button = if recording {
                    egui::Button::new(egui::RichText::new("Stop recording").color(Color32::WHITE)).fill(Color32::from_rgb(200, 40, 40))
                } else {
                    egui::Button::new("Record")
                };
                if ui.add(button).clicked() {
                    let _ = self.commands.send(if recording { "rec stop" } else { "rec" }.into());
                }
                let mut info = match s.robot {
                    Some(r) => format!(
                        "robot {}  x {:.3}  y {:.3}  θ {:.3}  v ({:.2}, {:.2}, {:.2})",
                        self.robot_id, r.x, r.y, r.theta, r.vx, r.vy, r.omega
                    ),
                    None => format!("robot {} not seen", self.robot_id),
                };
                info += &format!("  |  {}", s.mode);
                if let Some(name) = &s.recording {
                    info += &format!("  |  recording {name}");
                }
                ui.separator();
                if ui.button("Open script").clicked() {
                    if let Some(p) = rfd::FileDialog::new().add_filter("Rhai script", &["rhai"]).pick_file() {
                        self.script_path = Some(p.display().to_string());
                    }
                }
                if let Some(path) = self.script_path.clone() {
                    let name = std::path::Path::new(&path).file_name().unwrap_or_default().to_string_lossy().into_owned();
                    let running = s.script.as_deref() == Some(path.as_str());
                    ui.label(name).on_hover_text(&path);
                    if running {
                        if ui.button("Stop").clicked() {
                            let _ = self.commands.send("stop".into());
                        }
                    } else if ui.button("Play").clicked() {
                        let _ = self.commands.send(format!("rhai {path}"));
                    }
                    // Re-reads the file from disk and starts it over.
                    if ui.button("Reload").on_hover_text("re-read the file and restart it").clicked() {
                        let _ = self.commands.send(format!("rhai {path}"));
                    }
                }
                ui.separator();
                if ui.button("Radio").clicked() {
                    self.show_radio = !self.show_radio;
                    self.ports = serial_ports();
                }
                ui.monospace(info);
            });
        });

        let mut open = self.show_radio;
        egui::Window::new("Radio").open(&mut open).resizable(false).show(&ctx, |ui| {
            ui.label(format!("current: {}", s.radio));
            ui.horizontal(|ui| {
                ui.radio_value(&mut self.kind, OutputKind::Serial, "serial");
                ui.radio_value(&mut self.kind, OutputKind::Grsim, "grSim");
                ui.radio_value(&mut self.kind, OutputKind::None, "none");
            });
            match self.kind {
                OutputKind::Serial => {
                    ui.horizontal(|ui| {
                        ui.label("port");
                        ui.add(egui::TextEdit::singleline(&mut self.port).desired_width(90.0));
                        egui::ComboBox::from_id_salt("ports").selected_text("detected").show_ui(ui, |ui| {
                            for p in &self.ports {
                                ui.selectable_value(&mut self.port, p.clone(), p);
                            }
                        });
                        if ui.button("Refresh").clicked() {
                            self.ports = serial_ports();
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("baud");
                        ui.add(egui::TextEdit::singleline(&mut self.baud).desired_width(90.0));
                    });
                }
                OutputKind::Grsim => {
                    ui.horizontal(|ui| {
                        ui.label("address");
                        ui.text_edit_singleline(&mut self.grsim_addr);
                    });
                }
                OutputKind::None => {}
            }
            if ui.button("Apply").clicked() {
                let cmd = match self.kind {
                    OutputKind::Serial => format!("radio serial {} {}", self.port.trim(), self.baud.trim()),
                    OutputKind::Grsim => format!("radio grsim {}", self.grsim_addr.trim()),
                    OutputKind::None => "radio none".into(),
                };
                let _ = self.commands.send(cmd);
            }
        });
        self.show_radio = open;

        let field = egui::Frame::default().fill(Color32::from_rgb(30, 90, 40));
        egui::CentralPanel::default().frame(field).show(ui, |ui| {
            let rect = ui.max_rect();
            let (hl, hw, margin) = (self.half_length, self.half_width, 0.3);
            let k = (rect.width() / (2.0 * (hl + margin))).min(rect.height() / (2.0 * (hw + margin))); // px per metre
            let at = |x: f64, y: f64| rect.center() + Vec2::new(x as f32 * k, -(y as f32) * k); // world y up
            let p = ui.painter();
            let white = Stroke::new(2.0, Color32::WHITE);

            let corners = [at(-hl as f64, -hw as f64), at(hl as f64, -hw as f64), at(hl as f64, hw as f64), at(-hl as f64, hw as f64)];
            for i in 0..4 {
                p.line_segment([corners[i], corners[(i + 1) % 4]], white);
            }
            p.line_segment([at(0.0, -hw as f64), at(0.0, hw as f64)], white);
            p.circle_stroke(at(0.0, 0.0), 0.5 * k, white);

            if let Some(t) = s.target {
                let (c, d) = (at(t.x, t.y), 0.08 * k);
                let red = Stroke::new(2.0, Color32::from_rgb(255, 70, 70));
                p.line_segment([c + Vec2::splat(-d), c + Vec2::splat(d)], red);
                p.line_segment([c + Vec2::new(-d, d), c + Vec2::new(d, -d)], red);
            }
            if let Some((x, y)) = s.ball {
                p.circle_filled(at(x, y), (0.0215 * k).max(3.0), Color32::from_rgb(255, 140, 0));
            }
            if let Some(r) = s.robot {
                let c = at(r.x, r.y);
                p.circle_filled(c, (0.09 * k).max(4.0), self.color);
                let nose: Pos2 = at(r.x + 0.12 * r.theta.cos(), r.y + 0.12 * r.theta.sin());
                p.line_segment([c, nose], Stroke::new(3.0, Color32::BLACK));
            }
        });

        ctx.request_repaint_after(Duration::from_millis(50));
    }
}
