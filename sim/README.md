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

Delays like the real robot: a command reaches the wheels `command_delay`
(20 ms) after it arrives, like the radio and motor firmware. A vision
packet goes out `vision_delay` (40 ms) after its camera frame, like the
cameras, SSL-Vision and network, so small_engine sees where the robot was.
Together that is about 60 ms in the control loop: a controller tuned with no
delay can overshoot here. Set both to 0 for the ideal robot.

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

### Torque-problem robot: spins sideways, nothing smooth

`slip_robot.toml` stacks the faults:

| Setting | Value | Effect |
|---|---|---|
| `max_wheel_torque` | 0.07 (good: 0.12) | weak motors |
| `[wheels] torque` | back motors 18 % | sideways moves spin it (below) |
| `drag` | 3 N per m/s | the motors work all the time, not only speeding up |
| `friction` | 0.35 | worn tyres slip (red) on most pushes, even forward |
| `noise`, `noise_time` | 20 %, 0.04 s | fast wheel wobble: the robot shakes |
| `deadband` | 1.0 rad/s | slow wheel speeds stick: jerky starts and stops |

Moving sideways loads the back wheels most (they point most sideways). The
weak back motors max out and lag (orange in the viewer), the front ones do
not, so the robot turns as soon as it slides and keeps turning at about
2 rad/s while barely moving. Forward it does not spin but slips, shakes and
wanders (about +-0.2 rad over 3 s) and is slower.

Open loop, after 3 s at 1 m/s (small_engine's 2 m/s^2 ramp):

| | Good robot | Torque-problem robot |
|---|---|---|
| forward | 2.7 m, on line, smooth | ~1.8 m, wanders +-0.2 rad, shakes ~0.3 m/s^2, slipping most of the time |
| sideways | 2.7 m, 0.04 rad turned | spins ~6 rad (about one turn), ends within 0.2 m of the start |

Closed loop small_engine still finishes `goto` and `square`, fighting it all
the way. Tuning: the back `torque` sets the spin (0.4 gentle banana, 0.1
spin in place); `noise` the shake; `friction` the slipping. A `deadband`
above about 2 rad/s makes small_engine stall just short of targets.

Make other robots the same way: a small toml with only the keys that change.
Only one sim can run at a time on the default ports; give a second one its
own `command_addr` and `robot_id` (and point small_engine at it).
