//! Settings read from `small_engine.toml`. Every value has a default, so a
//! missing file or a missing key is fine.

use serde::Deserialize;
use std::path::Path;

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    /// The robot being driven, as vision sees it: pattern id 0-15 and team.
    pub robot_id: u32,
    /// "blue" or "yellow".
    pub team: Team,
    /// Its radio id 0-5 (the switches on the robot board), when it differs
    /// from `robot_id`.
    pub radio_id: Option<u32>,
    /// Control loop rate, Hz.
    pub rate_hz: f64,
    pub vision: VisionConfig,
    pub output: OutputConfig,
    pub control: ControlConfig,
    /// Folder where recordings are written.
    pub record_dir: String,
    /// Open the debug screen window.
    pub screen: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Team {
    #[default]
    Blue,
    Yellow,
}

impl Team {
    pub fn name(self) -> &'static str {
        match self {
            Team::Blue => "blue",
            Team::Yellow => "yellow",
        }
    }

    pub fn parse(s: &str) -> Option<Team> {
        match s {
            "blue" | "b" => Some(Team::Blue),
            "yellow" | "y" => Some(Team::Yellow),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct VisionConfig {
    pub multicast_addr: String,
    pub port: u16,
    /// Interface to join the multicast group on ("0.0.0.0" = default).
    pub interface: String,
    /// Seconds without seeing the robot before it is stopped.
    pub timeout: f64,
    /// Ignore detections whose x is outside [min_x, max_x] (camera overlap); 0/0 = off.
    pub min_x: f64,
    pub max_x: f64,
    /// Velocity estimate, `[vision.kalman]`.
    pub kalman: KalmanConfig,
}

/// Kalman filter for vx, vy, omega: a constant-velocity model per axis
/// (x, y, theta). Tunable live with the `kf` command / the Filter panel.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct KalmanConfig {
    /// Off = raw finite differences.
    pub enabled: bool,
    /// Process noise: acceleration std, m/s^2 and rad/s^2.
    /// Higher = follows changes faster, noisier.
    pub q_pos: f64,
    pub q_theta: f64,
    /// Vision measurement noise std, m and rad. Higher = smoother, more lag.
    pub r_pos: f64,
    pub r_theta: f64,
}

impl Default for KalmanConfig {
    fn default() -> Self {
        Self { enabled: true, q_pos: 3.0, q_theta: 20.0, r_pos: 0.003, r_theta: 0.02 }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct OutputConfig {
    /// "serial" (real radio), "grsim" (simulator) or "none" (dry run).
    pub kind: OutputKind,
    pub serial_port: String,
    pub baud_rate: u32,
    pub grsim_addr: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum OutputKind {
    Serial,
    Grsim,
    None,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ControlConfig {
    /// Translation limits, m/s and m/s^2.
    pub max_speed: f64,
    pub max_accel: f64,
    /// Rotation limits, rad/s and rad/s^2.
    pub max_omega: f64,
    pub max_alpha: f64,
    /// Distance / angle at which a go-to counts as reached.
    pub pos_tolerance: f64,
    pub angle_tolerance: f64,
    /// Proportional gain used in the last centimetres (1/s).
    pub kp_pos: f64,
    pub kp_angle: f64,
    /// Commands outside this field rectangle (half sizes, m) are refused
    /// and a robot leaving it is stopped. 0 = no limit.
    pub field_half_length: f64,
    pub field_half_width: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            robot_id: 0,
            radio_id: None,
            team: Team::Blue,
            rate_hz: 60.0,
            vision: VisionConfig::default(),
            output: OutputConfig::default(),
            control: ControlConfig::default(),
            record_dir: "recordings".into(),
            screen: true,
        }
    }
}

impl Default for VisionConfig {
    fn default() -> Self {
        Self {
            multicast_addr: "224.5.23.2".into(),
            port: 10020,
            interface: "0.0.0.0".into(),
            timeout: 0.3,
            min_x: 0.0,
            max_x: 0.0,
            kalman: KalmanConfig::default(),
        }
    }
}

impl Default for OutputConfig {
    fn default() -> Self {
        Self {
            kind: OutputKind::None,
            serial_port: if cfg!(windows) { "COM3".into() } else { "/dev/ttyUSB0".into() },
            baud_rate: 460800,
            grsim_addr: "127.0.0.1:20011".into(),
        }
    }
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            max_speed: 1.0,
            max_accel: 2.0,
            max_omega: 4.0,
            max_alpha: 15.0,
            pos_tolerance: 0.02,
            angle_tolerance: 0.05,
            kp_pos: 3.0,
            kp_angle: 4.0,
            field_half_length: 4.5,
            field_half_width: 3.0,
        }
    }
}

impl Config {
    pub fn load(path: &Path) -> Result<Self, String> {
        match std::fs::read_to_string(path) {
            Ok(text) => toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display())),
            Err(_) => {
                eprintln!("{} not found, using defaults", path.display());
                Ok(Self::default())
            }
        }
    }

    pub fn tick(&self) -> f64 {
        1.0 / self.rate_hz
    }
}
