# rl-robot-skills

Tools for learning skills on a single real SSL (RoboCup Small Size) robot.

## small_engine/

Minimal Rust engine: SSL-Vision in (multicast UDP), velocity commands out
(serial radio or grSim), CSV recording of every control tick. See
`small_engine/README.md` for commands, recording columns and the radio frame.

- Build / run: `cd small_engine; cargo run --release` (reads `small_engine.toml`).
- No hardware: `cargo run --example fake_robot` in a second terminal, with
  `[output] kind = "grsim"`.
- Tests: `cargo test`.

Layout: `main.rs` (control loop) → `vision.rs`, `console.rs` (text commands
and scripts), `control.rs` (modes + limits), `radio.rs`, `recorder.rs`.

## Conventions

- Keep it minimal: few dependencies, small files, no frameworks.
- Units are metres, seconds, radians. Commands are robot frame unless named world.
- Every command goes through the controller limits; never send to the radio directly.
- Safety first: the robot stops on vision loss, on leaving the field, and on exit.
- Atomic commits, one feature each.
