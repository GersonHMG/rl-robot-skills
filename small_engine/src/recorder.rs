//! CSV recording of every control tick: what vision saw and what was sent.

use crate::radio::Command;
use crate::vision::RobotObs;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

pub const HEADER: &str = "time,seen,t_capture,frame,confidence,x,y,theta,vx,vy,omega,\
cmd_vx,cmd_vy,cmd_omega,mode,ball_x,ball_y";

pub struct Recorder {
    writer: BufWriter<File>,
    pub path: PathBuf,
    pub rows: u64,
}

/// One tick's worth of data.
pub struct Row<'a> {
    /// Seconds since the engine started.
    pub time: f64,
    /// Fresh vision of the robot this tick (None = not seen / timed out).
    pub robot: Option<RobotObs>,
    /// Robot frame, as sent.
    pub cmd: Command,
    /// `Mode::name()` during the tick.
    pub mode: &'a str,
    pub ball: Option<(f64, f64)>,
}

impl Recorder {
    /// Opens `<dir>/<name>.csv` (name defaults to a timestamp) and writes a
    /// `<name>.meta.toml` next to it with `meta`.
    pub fn start(dir: &Path, name: Option<&str>, meta: &str) -> std::io::Result<Self> {
        std::fs::create_dir_all(dir)?;
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let name = name.map(str::to_owned).unwrap_or_else(|| format!("rec_{stamp}"));
        let path = dir.join(format!("{name}.csv"));
        std::fs::write(dir.join(format!("{name}.meta.toml")), format!("unix_start = {stamp}\n{meta}"))?;
        let mut writer = BufWriter::new(File::create(&path)?);
        writeln!(writer, "{HEADER}")?;
        Ok(Self { writer, path, rows: 0 })
    }

    pub fn write(&mut self, row: &Row) -> std::io::Result<()> {
        let w = &mut self.writer;
        write!(w, "{:.4},", row.time)?;
        match row.robot {
            Some(r) => write!(
                w,
                "1,{:.4},{},{:.3},{:.5},{:.5},{:.5},{:.4},{:.4},{:.4},",
                r.t_capture, r.frame, r.confidence, r.x, r.y, r.theta, r.vx, r.vy, r.omega
            )?,
            None => write!(w, "0,,,,,,,,,,")?,
        }
        let c = &row.cmd;
        write!(w, "{:.4},{:.4},{:.4},{},", c.vx, c.vy, c.omega, row.mode)?;
        match row.ball {
            Some((x, y)) => writeln!(w, "{x:.4},{y:.4}")?,
            None => writeln!(w, ",")?,
        }
        self.rows += 1;
        Ok(())
    }

    pub fn finish(mut self) -> std::io::Result<(PathBuf, u64)> {
        self.writer.flush()?;
        Ok((self.path, self.rows))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rows_have_as_many_columns_as_the_header() {
        let dir = std::env::temp_dir().join(format!("small_engine_test_{}", std::process::id()));
        let mut rec = Recorder::start(&dir, Some("t"), "").unwrap();
        let mode = "idle";
        let seen = Some(RobotObs { x: 1.0, ..Default::default() });
        for (robot, ball) in [(seen, Some((0.1, 0.2))), (None, None)] {
            rec.write(&Row { time: 0.0, robot, cmd: Command::default(), mode, ball }).unwrap();
        }
        let (path, rows) = rec.finish().unwrap();
        assert_eq!(rows, 2);
        let text = std::fs::read_to_string(&path).unwrap();
        let n = HEADER.split(',').count();
        for line in text.lines() {
            assert_eq!(line.split(',').count(), n, "{line}");
        }
        let _ = std::fs::remove_dir_all(dir);
    }
}
