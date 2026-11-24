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
**Reload** re-reads the file from disk and restarts it. If the script declares parameters, a
**Script parameters** window (also the **Params** button) shows one field per
parameter: a number box (with its range), a checkbox or a dropdown. Bad or
missing values are marked in red and the script is not started until they are
fixed. **Manual** (or the M key) drives the robot from the keyboard: W/S forward/back,
A/D left/right, Q/E turn left/right, at the speeds set next to it; releasing the
keys stops it, and it stops by itself within 0.25 s if the window stalls. **Radio** opens a small window to pick the output (serial port and baud,
grSim address, or none) and apply it live. **Filter** turns the velocity
Kalman filter on/off and tunes it. Closing the window stops the engine (robot left still). Set `screen = false`
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
| `kf [on \| off \| q_pos r_pos q_theta r_theta]` | show / tune the velocity filter |
| `rhai file [-- name=value ...]` | run a [Rhai](https://rhai.rs) script with parameters (below) |

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

### Parameters

A script declares its parameters in `fn params()`, an array of maps. The
debug screen turns them into input fields; on the console they are given after
`--`. The script reads them from the constant map `args`:

```rust
fn params() {
    [
        #{ name: "radius", type: "float", value: 0.5, min: 0.15, max: 3.0, help: "m" },
        #{ name: "direction", type: "choice", options: ["ccw", "cw"], value: "ccw" },
        #{ name: "face_center", type: "bool", value: true },
    ]
}
wvel(args.radius, 0, 0);
```

| Key | |
|---|---|
| `name` | required, no spaces |
| `type` | `float` (default), `int`, `bool`, `choice` |
| `options` | allowed values of a `choice` |
| `value` | optional default (`default` is a reserved word in Rhai); without it the value must be given |
| `min`, `max` | optional range of a number |
| `help` | optional tooltip |

Values are checked before the script starts (missing, not a number, out of
range, not an option, unknown name); a bad value is reported and the script
does not run.

### Example: orbit

[scripts/orbit.rhai](scripts/orbit.rhai) circles a point while facing it.
It first drives to the nearest point of the circle, then orbits for `turns`
revolutions (0 = until Stop). The comments at the top explain the math.

- Screen: **Open script** -> `orbit.rhai`, fill in the fields, **Play** (or **Run**).
- Console: `rhai scripts/orbit.rhai -- center_x=1 center_y=0 radius=0.6 direction=cw speed=0.4 turns=3`

| Parameter | Default | |
|---|---|---|
| `center_x`, `center_y` | 0, 0 | centre, m (world) |
| `radius` | 0.5 | m, 0.15-3 |
| `direction` | `ccw` | `ccw` or `cw` |
| `speed` | 0.5 | m/s along the circle |
| `turns` | 2 | revolutions, 0 = until Stop |
| `face_center` | true | false keeps the start heading |

### System identification tests

[scripts/sysid/](scripts/sysid/) holds one script per test. Press **Record**
first, and for tests 1-3 raise the limits so the steps reach the robot:
`set max_accel 50`, `set max_speed 2` (set them back after). Tests 1-3 drive
back to the start pose between steps, so the robot stays in a small area.

| Script | Commands | Gives |
|---|---|---|
| `0_still.rhai` | zero for `duration` (10 s) | vision noise, creep (printed at the end) |
| `1_delay.rhai` | steps 0 -> +-`amp` (1 m/s) -> 0, `reps` (10) per axis, vx then vy | delay tau per axis |
| `2_stair_vx.rhai` | vx at +-0.1 0.2 0.3 0.5 0.8 1.0 1.5 `max` (2), 1 s each | vx gain, saturation, coupling |
| `3_stair_vy.rhai` | same on vy | vy gain, saturation, coupling |

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
| `vx`, `vy` | m/s | velocity, world frame: Kalman filter estimate (raw finite differences when the filter is off) |
| `omega` | rad/s | angular velocity, same estimate |
| `cmd_vx` | m/s | command sent this tick, **robot frame**, forward (after limits) |
| `cmd_vy` | m/s | command sent this tick, **robot frame**, left (after limits) |
| `cmd_omega` | rad/s | command sent this tick, CCW positive (after limits) |
| `mode` | - | controller mode: `idle`, `local_vel` (`vel`), `world_vel` (`wvel`), `path` (`goto`/`path`/`square`/`turn`) |
| `ball_x`, `ball_y` | m | ball position, world frame (most confident detection); empty if not seen |

The `.meta.toml` holds `unix_start`, `robot_id`, `team`, `rate_hz`, `output`
the limits `max_speed`, `max_accel`, `max_omega`, `max_alpha` and the Kalman
settings `kf_*` at the start of the recording.

## Velocity filter

`vx`, `vy`, `omega` come from a small Kalman filter: a constant-velocity model
on each of x, y, theta, corrected by every vision detection. The pose itself
stays the raw measurement. Settings are in `[vision.kalman]` and can be changed
live with the **Filter** panel or `kf`:

| Key | Unit | Effect |
|---|---|---|
| `enabled` | | off = raw finite differences |
| `q_pos`, `q_theta` | m/s², rad/s² | how hard the robot can accelerate: higher follows changes faster, noisier |
| `r_pos`, `r_theta` | m, rad | vision noise: higher is smoother, more lag |

## Radio frame

Same 5-byte-per-robot protocol as the engine (`radio_slots` slots, all zero
except ours): byte 0 = id(3) | dribbler(3) | kick(1) | 0; bytes 1-3 = sign bit +
low 7 bits of |vx|, |vy|, |w| in cm/s (crad/s); byte 4 = high bits
vx(2) | vy(2) | w(4). Values are clamped to ±511, so ±5.11 m/s.
