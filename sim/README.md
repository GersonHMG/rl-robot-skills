# sim

MuJoCo simulation of a single SSL robot. It speaks grSim, so `small_engine`
drives it like the real robot: velocity commands in over UDP, SSL-Vision
packets out over multicast. No field and no ball: just the robot.

```
uv run sim.py              # viewer window, camera follows the robot
uv run sim.py --headless   # no window
uv run sim.py --config bad_robot.toml   # the robot with bad wheels
uv run python -m unittest  # tests
```

Then in `small_engine/small_engine.toml` set `[output] kind = "grsim"` and run
`cargo run --release` (the sim takes the place of `fake_robot`).

## Files

| File | Does |
|---|---|
| `config.toml` | robot, motor and network parameters (typical SSL estimates) |
| `bad_robot.toml` | the bad-wheels robot: only the keys that differ from `config.toml` |
| `robot.py` | MuJoCo model and the omni-wheel model |
| `wire.py` | minimal protobuf encode/decode for grSim and SSL-Vision |
| `sim.py` | real-time loop, network, viewer |

## Robot model

The body is a 180 x 150 mm, 2.5 kg cylinder on x/y slide joints and a z hinge
(no contacts, it cannot tip). Wheels are drawn but not simulated. Every 2 ms
physics step:

1. The robot-frame command `(vx, vy, omega)` becomes four wheel speeds from
   the wheel angles (57, 135, 225, 303 degrees from the front), wheel radius
   and centre distance.
2. Motor limits: wheel speeds are capped at `max_wheel_speed`, and their
   change at the acceleration `max_wheel_torque` allows. All wheels scale
   together so the direction is kept.
3. The wheel speeds go back to a body velocity, and velocity actuators push
   the body toward it. Their force is capped by `friction * m * g`, so hard
   pushes saturate like slipping wheels.

With the defaults: about 3.9 m/s top forward speed and 5 m/s^2 acceleration.
The motor numbers are estimates; fit them to recordings of the real robot.

Safety like the real robot: no command for `command_timeout` stops the
wheels. `vision_noise_pos` / `vision_noise_theta` add Gaussian noise to the
published pose.

## Two robots: good and bad

`config.toml` is the good robot: perfect wheels. `bad_robot.toml` overrides
only the `[wheels]` faults (and a reddish shell). The robot still computes
wheel speeds as if its wheels were perfect, so the faults show up as errors
the controller has to fix:

| `[wheels]` key | Fault | Bad robot |
|---|---|---|
| `gain` | each wheel's real speed / asked speed (worn motor, slipping hub) | `[1.0, 0.75, 1.0, 0.9]`: drives straight -> curves and drifts |
| `deadband` | asked slower than this (rad/s), the wheel does not turn (stiction) | `1.5`: very slow moves stall |
| `noise`, `noise_time` | slow random speed error per wheel (bad rollers, loose wheel) | 15 %, over 0.15 s |
| `seed` | random seed for the noise (omit for a new one each run) | |

Open loop, `vel 1 0 0` for 1 s, the bad robot ends about 8 cm to the side,
4 degrees turned and 15 cm short. Closed loop (`goto`, `square`) small_engine
still gets there. A deadband above about 2 rad/s makes `goto` stall short of
the 2 cm tolerance with the default `kp_pos = 3`.

Make other robots the same way: a small toml with only the keys that change.
Only one sim can run at a time on the default ports; give a second one its
own `command_addr` and `robot_id` (and point small_engine at it).
