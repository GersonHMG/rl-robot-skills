# small_engine

A minimal version of the Sysmic engine for working with **one real robot**:
it receives SSL-Vision, drives the robot through the radio, and records
everything to CSV. No AI. About 1000 lines of Rust.

```
SSL-Vision (multicast UDP) ──> vision.rs ──> control.rs ──> radio.rs ──> serial radio / grSim
                                    │             │
                                    └──────> recorder.rs (CSV, one row per tick)
console.rs: commands typed at the prompt or replayed from a script file
```

## Running

```powershell
cargo run --release                       # uses small_engine.toml in this folder
cargo run --release -- other.toml --run scripts/square.txt
```

Set `[output] kind = "serial"` and `serial_port` (e.g. `COM3`) for the real
robot, `"grsim"` for a simulator, `"none"` to only watch and record.

Without hardware, start the pretend robot in another terminal; it takes the
grSim commands and publishes vision of itself:

```powershell
cargo run --example fake_robot
```

## Debug screen

Starting the executable opens a window (egui) showing the field with the
robot, its heading, the ball and the current target, plus the pose and mode.
The **Record** button starts / stops a recording, same as `rec` / `rec stop`.
**Open script** picks a `.rhai` file; **Play** / **Stop** run and abort it, and
**Reload** re-reads the file from disk and restarts it. **Manual** (or the M key) drives the robot from the keyboard: W/S forward/back,
A/D left/right, Q/E turn left/right, at the speeds set next to it; releasing the
keys stops it, and it stops by itself within 0.25 s if the window stalls. **Radio** opens a small window to pick the output (serial port and baud,
grSim address, or none) and apply it live. Closing the window stops the engine (robot left still). Set `screen = false`
to run without it; the console works either way.

## Commands

Units are metres, seconds and radians.

| Command | Does |
|---|---|
| `stop` / `s` | stop now; also aborts a running script |
| `vel vx vy w [secs]` | open-loop velocity, robot frame (vx forward, vy left) |
| `wvel vx vy w [secs]` | open-loop velocity, world frame |
| `goto x y [theta]` | drive to a point (and heading) |
| `path x1 y1 x2 y2 ...` | drive through points |
| `square side` / `turn theta` | square from the current position / turn in place |
| `kick`, `dribble 0-7` | kicker and dribbler |
| `set key value` | change a `[control]` limit or gain live |
| `rec [name]`, `rec stop` | start / stop recording |
| `status` | pose, velocity, mode, vision counters |
| `run file` | run a script; scripts can also `wait secs` and `waitidle` |
| `radio [none \| grsim [addr] \| serial port [baud]]` | show / change the output live (robot is stopped first) |
| `rhai file` | run a [Rhai](https://rhai.rs) script (below) |

Scripts make test runs repeatable: see [scripts/square.txt](scripts/square.txt).

## Rhai scripts

`rhai scripts/square.rhai` opens the file and runs it in its own thread. The
script sends the same commands as the console, so all limits and safety
checks apply. A typed `stop` aborts it; the robot is stopped when it ends.

| Function | Does |
|---|---|
| `vel(vx, vy, w)` | velocity, robot frame (m/s, rad/s), held until changed |
| `wvel(vx, vy, w)` | velocity, world frame |
| `stop()` | zero velocity |
| `wait(secs)` | sleep |
| `pose()` | `#{seen, x, y, theta, vx, vy, omega}` from vision |
| `time()` | seconds since the script started |

See [scripts/square.rhai](scripts/square.rhai).

## Safety

- The robot is commanded to zero when vision has not seen it for
  `vision.timeout` seconds (it resumes when seen again).
- Targets outside the field limits are refused; a robot that leaves them is
  stopped.
- Every command passes through the `max_speed` / `max_accel` / `max_omega` /
  `max_alpha` limits, in the robot frame.
- Ctrl+C and `quit` send zero velocity before exiting.

## Recordings

`recordings/<name>.csv` plus `<name>.meta.toml` (robot, rate, limits). One row
per control tick:

Empty cells mean "not available this tick". Units: m, s, rad.

| Column | Unit | Meaning |
|---|---|---|
| `time` | s | time since small_engine started (local clock) |
| `seen` | 0/1 | 1 if vision saw our robot within `[vision] timeout`; when 0 the next 9 columns are empty |
| `t_capture` | s | SSL-Vision capture timestamp of the detection |
| `frame` | - | SSL-Vision frame number |
| `confidence` | 0-1 | SSL-Vision detection confidence |
| `x`, `y` | m | measured position, world frame |
| `theta` | rad | measured heading, world frame (0 = +x, CCW positive) |
| `vx`, `vy` | m/s | velocity, world frame: finite difference of the pose, smoothed with `velocity_alpha` |
| `omega` | rad/s | angular velocity, same smoothing |
| `cmd_vx` | m/s | command sent this tick, **robot frame**, forward (after limits) |
| `cmd_vy` | m/s | command sent this tick, **robot frame**, left (after limits) |
| `cmd_omega` | rad/s | command sent this tick, CCW positive (after limits) |
| `mode` | - | controller mode: `idle`, `local_vel` (`vel`), `world_vel` (`wvel`), `path` (`goto`/`path`/`square`/`turn`) |
| `ball_x`, `ball_y` | m | ball position, world frame (most confident detection); empty if not seen |

The `.meta.toml` holds `unix_start`, `robot_id`, `team`, `rate_hz`, `output`
and the limits `max_speed`, `max_accel`, `max_omega`, `max_alpha` at the start
of the recording.

## Radio frame

Same 5-byte-per-robot protocol as the engine (`radio_slots` slots, all zero
except ours): byte 0 = id(3) | dribbler(3) | kick(1) | 0; bytes 1-3 = sign bit +
low 7 bits of |vx|, |vy|, |w| in cm/s (crad/s); byte 4 = high bits
vx(2) | vy(2) | w(4). Values are clamped to ±511, so ±5.11 m/s.
