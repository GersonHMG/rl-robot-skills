# small_engine

A minimal version of the Sysmic engine for working with **one real robot**:
it receives SSL-Vision, drives the robot through the radio, and records
everything to CSV. No GUI, no Lua, no AI. About 1000 lines of Rust.

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

Scripts make test runs repeatable: see [scripts/square.txt](scripts/square.txt).

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

| Column | Meaning |
|---|---|
| `time` | s since small_engine started |
| `seen` | 1 if vision saw the robot within the timeout, else the vision columns are empty |
| `t_capture`, `frame`, `camera`, `confidence` | from the vision packet |
| `x`, `y`, `theta` | measured pose, world frame |
| `vx`, `vy`, `omega` | smoothed finite-difference velocity, world frame |
| `cmd_vx`, `cmd_vy`, `cmd_omega` | command sent this tick, **robot frame** |
| `kick`, `dribbler` | actuator commands |
| `mode`, `target_*` | what the controller was doing |
| `ball_x`, `ball_y` | ball, if seen |

## Radio frame

Same 5-byte-per-robot protocol as the engine (`radio_slots` slots, all zero
except ours): byte 0 = id(3) | dribbler(3) | kick(1) | 0; bytes 1-3 = sign bit +
low 7 bits of |vx|, |vy|, |w| in cm/s (crad/s); byte 4 = high bits
vx(2) | vy(2) | w(4). Values are clamped to ±511, so ±5.11 m/s.
