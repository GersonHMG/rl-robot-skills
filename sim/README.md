# sim

MuJoCo simulation of a single SSL robot. It speaks grSim, so `small_engine`
drives it like the real robot: velocity commands in over UDP, SSL-Vision
packets out over multicast. No field and no ball: just the robot.

```
uv run sim.py              # viewer window, camera follows the robot
uv run sim.py --headless   # no window
uv run sim.py --config bad_robot.toml   # the robot with bad wheels
uv run sim.py --config slip_robot.toml  # torque problems: spins when moving sideways
uv run python -m unittest  # tests
```

Then in `small_engine/small_engine.toml` set `[output] kind = "grsim"` and run
`cargo run --release` (the sim takes the place of `fake_robot`).

## Files

| File | Does |
|---|---|
| `config.toml` | robot, motor and network parameters (typical SSL estimates) |
| `bad_robot.toml` | the bad-wheels robot: only the keys that differ from `config.toml` |
| `slip_robot.toml` | the robot with torque problems: spins when moving sideways |
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
   Wheel speeds are capped at `max_wheel_speed`, all scaled together so the
   direction is kept. Then the `[wheels]` faults apply.
2. Each wheel runs its own speed loop, like the motor firmware: its force
   comes from the gap between its target and how fast its rim moves over the
   floor (`response` sets how hard). Nothing corrects the robot's heading as
   a whole; that is small_engine's job.
3. The motor torque `max_wheel_torque` caps the forces, all wheels scaled
   together. Then a weak motor (`[wheels] torque` < 1) caps its own wheel
   (orange in the viewer).
4. Each wheel's grip (`friction * m * g / 4`, times its `[wheels] grip`) caps
   its own force. Past it the wheel slips: it only pushes with `kinetic`
   times its grip until the force asked of it drops below that. Slipping
   wheels are red in the viewer.
5. The wheel forces add up to the body force and torque, against the rolling
   drag `drag` (0 for the good robot).

With the defaults: about 3.9 m/s top forward speed, about 5.5 m/s^2 forward
and 4.4 m/s^2 sideways acceleration, and the motors can never out-push the
grip, so the good robot never slips. Because the wheels are not placed
symmetrically front to back, it turns about 0.04 rad (2 degrees) while it
speeds up sideways, then holds that heading.
The motor numbers are estimates; fit them to recordings of the real robot.

Safety like the real robot: no command for `command_timeout` stops the
wheels. `vision_noise_pos` / `vision_noise_theta` add Gaussian noise to the
published pose.

## Three robots: good, bad wheels, torque problems

`config.toml` is the good robot: perfect wheels. `bad_robot.toml` overrides
only the `[wheels]` faults (and a reddish shell). The robot still computes
wheel speeds as if its wheels were perfect, so the faults show up as errors
the controller has to fix:

| `[wheels]` key | Fault | Bad robot |
|---|---|---|
| `gain` | each wheel's real speed / asked speed (worn motor, slipping hub) | `[1.0, 0.2, 1.0, 0.9]`: back-left wheel nearly dead, drives straight -> spins away |
| `deadband` | asked slower than this (rad/s), the wheel does not turn (stiction) | `1.5`: very slow moves stall |
| `noise`, `noise_time` | slow random speed error per wheel (bad rollers, loose wheel) | 15 %, over 0.15 s |
| `grip` | tyre grip per wheel, times `[motor] friction` (worn tyre < 1) | |
| `torque` | motor strength per wheel, times `[motor] max_wheel_torque` (weak motor < 1) | |
| `seed` | random seed for the noise (omit for a new one each run) | |

Open loop, `vel 1 0 0` for 1 s, the bad robot turns about 1.2 rad/s (it
ends up 0.6-1.3 rad turned, depending on the wobble) and 25-35 % short.
Closed loop (`goto`, `square`) small_engine still gets there. A deadband above about 2 rad/s makes `goto` stall short of
the 2 cm tolerance with the default `kp_pos = 3`.

### Torque-problem robot: spins sideways

`slip_robot.toml`: weak motors (`max_wheel_torque = 0.06`, half the good
robot), the two back motors much weaker (`torque` 20 %), rolling drag
(`drag = 3` N per m/s) and worn tyres (`friction = 0.5`).

Moving sideways loads the back wheels most (they point most sideways), and
the drag needs that push the whole time, not only while speeding up. The
weak back motors max out and lag (orange in the viewer), the front ones do
not, so the robot turns as soon as it slides, and keeps turning: it ends up
spinning at about 2 rad/s and barely moves. Forward moves load the wheels
evenly and stay straight.

Open loop, sideways at 1 m/s (small_engine's 2 m/s^2 ramp), heading:

| After | 0.5 s | 1 s | 1.5 s | 2 s | 3 s |
|---|---|---|---|---|---|
| Good robot | 0.04 | 0.04 | 0.04 | 0.04 | 0.04 rad |
| Torque-problem robot | 0.48 | 1.49 | 2.50 | 3.53 | 5.62 rad |

Forward at 1 m/s it stays on line (0.00 rad). Closed loop small_engine
holds the heading and still reaches sideways `goto` targets, slower.

How bad it is comes from the back motors' `torque`: 0.4 gives a gentle
banana (about 0.5 rad/s), 0.25 a steady spin (1.6 rad/s), 0.1 spinning in
place (3 rad/s). Wheels can still slip (red) on harsh pushes; for more slip
lower `friction` and raise `max_wheel_torque`.

Make other robots the same way: a small toml with only the keys that change.
Only one sim can run at a time on the default ports; give a second one its
own `command_addr` and `robot_id` (and point small_engine at it).
