"""The robot: a MuJoCo model plus a simple omni-wheel model.

The body slides on the floor (x, y slide joints and a z hinge). Wheels are
not simulated as bodies. Each physics step:

1. Wheel speeds: the robot-frame command becomes four wheel speeds, capped
   at the motor top speed, then the wheel faults below are applied.
2. Wheel forces: each wheel runs its own speed loop, like the robot's motor
   firmware: force from the gap between its target and how fast its rim
   moves over the floor. Nothing corrects the robot's heading as a whole.
   The nominal motor torque caps the forces (all scaled together), then a
   weak motor caps its own wheel ([wheels] torque), then each wheel's grip
   on the floor caps it. Past the grip the wheel slips and only pushes with
   sliding friction (lower) until the demand drops again.
3. The wheel forces add up to the body force and torque MuJoCo applies,
   against the rolling drag ([motor] drag): moving takes force all the time.

Wheel forces are not equal: a sideways push needs more from each wheel than
a forward one (the wheels point mostly forward), and most from the back
wheels. So with weak grip lateral and sudden moves slip first, and with weak
back motors the back lags only when moving sideways: the robot yaws the whole
time and its path bends like a banana.

Bad wheels ([wheels] in the config): each wheel can turn slower than asked
(gain), not turn at all below a speed (deadband, stiction), wobble (noise, a
slow random speed error), grip less (grip, worn tyre) and push less (torque,
weak motor). The robot does not know: it computes wheel speeds for perfect
wheels, so a bad robot drifts and curves.
"""

import math

import mujoco
import numpy as np

G = 9.81
SLIP_RGBA = (1.0, 0.1, 0.1, 1.0)
LIMIT_RGBA = (1.0, 0.6, 0.0, 1.0)
WHEEL_RGBA = (0.2, 0.2, 0.2, 1.0)


class Robot:
    def __init__(self, cfg: dict):
        r, m = cfg["robot"], cfg["motor"]
        n = len(r["wheel_angles"])
        angles = np.radians(r["wheel_angles"])
        self.wheel_radius = r["wheel_radius"]
        self.max_wheel_speed = m["max_wheel_speed"]
        # A: robot-frame (vx, vy, omega) -> wheel rim speed. Transposed, it maps
        # wheel forces to the body force and torque (fx, fy, tau).
        self.A = np.column_stack([-np.sin(angles), np.cos(angles), np.full(n, r["wheel_distance"])])
        self.J = self.A / self.wheel_radius          # -> wheel speed, rad/s
        self.J_inv = np.linalg.pinv(self.J)

        mass = r["mass"]
        self.loop_gain = mass / (2 * m["response"])  # N per m/s of rim speed error
        self.max_force = m["max_wheel_torque"] / self.wheel_radius
        w = cfg.get("wheels", {})
        self.motor_force = self.max_force * np.array(w.get("torque", [1.0] * n), dtype=float)
        tyres = np.array(w.get("grip", [1.0] * n), dtype=float)  # worn tyres < 1
        self.grip = tyres * m["friction"] * mass * G / n     # static, per wheel
        self.slide = self.grip * m.get("kinetic", 0.7)       # once slipping
        self.slipping = np.zeros(n, dtype=bool)
        self.limited = np.zeros(n, dtype=bool)  # weak motor at its torque limit

        self.model = mujoco.MjModel.from_xml_string(_xml(cfg))
        self.data = mujoco.MjData(self.model)
        self.wheel_geoms = [self.model.geom(f"wheel{i}").id for i in range(n)]
        self.wheels = np.zeros(n)  # wheel speed targets, rad/s

        self.gain = np.array(w.get("gain", [1.0] * n), dtype=float)
        self.deadband = w.get("deadband", 0.0)
        self.noise = w.get("noise", 0.0)
        self.noise_time = w.get("noise_time", 0.1)
        self.wobble = np.zeros(n)
        self.rng = np.random.default_rng(w.get("seed"))

    @property
    def dt(self) -> float:
        return self.model.opt.timestep

    def pose(self) -> tuple[float, float, float]:
        x, y, th = self.data.qpos
        return x, y, math.remainder(th, math.tau)

    def velocity(self) -> tuple[float, float, float]:
        """Robot-frame (vx, vy, omega)."""
        vxw, vyw, w = self.data.qvel
        c, s = math.cos(self.data.qpos[2]), math.sin(self.data.qpos[2])
        return c * vxw + s * vyw, -s * vxw + c * vyw, w

    def step(self, vx: float, vy: float, omega: float):
        target = self.J @ (vx, vy, omega)
        # motor speed limit: scale all wheels together to keep the direction
        peak = np.max(np.abs(target))
        if peak > self.max_wheel_speed:
            target *= self.max_wheel_speed / peak
        self.wheels = target

        rim = self.A @ self.velocity()  # how fast each rim moves over the floor
        demand = self.loop_gain * (self.actual_wheels() * self.wheel_radius - rim)
        # motor torque: the firmware scales all wheels together to keep the
        # direction; a weak motor and the grip are per wheel and bend it
        peak = np.max(np.abs(demand))
        force = demand * (self.max_force / peak) if peak > self.max_force else demand
        self.limited = np.abs(force) > self.motor_force
        force = np.clip(force, -self.motor_force, self.motor_force)
        mag = np.abs(force)
        self.slipping = (mag > self.grip) | (self.slipping & (mag > self.slide))
        force = np.where(self.slipping, np.sign(force) * self.slide, force)

        fx, fy, tau = self.A.T @ force
        c, s = math.cos(self.data.qpos[2]), math.sin(self.data.qpos[2])
        self.data.qfrc_applied[:] = (c * fx - s * fy, s * fx + c * fy, tau)
        mujoco.mj_step(self.model, self.data)

    def actual_wheels(self) -> np.ndarray:
        """Wheel speeds the floor sees, after the wheel faults."""
        if self.noise:  # Ornstein-Uhlenbeck: std `noise`, correlated over `noise_time`
            k = self.dt / self.noise_time
            self.wobble += -self.wobble * k + self.noise * math.sqrt(2 * k) * self.rng.standard_normal(len(self.wobble))
        actual = self.wheels * self.gain * (1 + self.wobble)
        actual[np.abs(self.wheels) < self.deadband] = 0.0
        return actual

    def show_slip(self):
        """Paint slipping wheels red, weak motors at their limit orange (viewer)."""
        for g, slip, limit in zip(self.wheel_geoms, self.slipping, self.limited):
            self.model.geom_rgba[g] = SLIP_RGBA if slip else LIMIT_RGBA if limit else WHEEL_RGBA


def _xml(cfg: dict) -> str:
    r, m = cfg["robot"], cfg["motor"]
    radius, h, mass = r["radius"], r["height"], r["mass"]
    drag = m.get("drag", 0.0)
    turn_drag = drag * r["wheel_distance"] ** 2
    wheels = "".join(
        f'<geom name="wheel{i}" type="cylinder" size="{r["wheel_radius"]} 0.006" rgba="{" ".join(map(str, WHEEL_RGBA))}"'
        f' pos="{r["wheel_distance"] * math.cos(a):.4f} {r["wheel_distance"] * math.sin(a):.4f} {r["wheel_radius"] - h / 2:.4f}"'
        f' zaxis="{math.cos(a):.4f} {math.sin(a):.4f} 0" mass="0"/>'
        for i, a in enumerate(np.radians(r["wheel_angles"]))
    )
    return f"""
<mujoco model="ssl_robot">
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -{G}"/>
  <default><geom contype="0" conaffinity="0"/></default>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".25 .3 .25" rgb2=".2 .25 .2" width="512" height="512"/>
    <material name="floor" texture="grid" texrepeat="10 10"/>
  </asset>
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="5 5 .1" material="floor"/>
    <body name="robot" pos="0 0 {h / 2}">
      <joint name="x" type="slide" axis="1 0 0" damping="{drag}"/>
      <joint name="y" type="slide" axis="0 1 0" damping="{drag}"/>
      <joint name="theta" type="hinge" axis="0 0 1" damping="{turn_drag}"/>
      <geom name="shell" type="cylinder" size="{radius} {h / 2}" mass="{mass}" rgba="{r.get("color", ".1 .1 .1")} .6"/>
      <geom name="front" type="box" size=".01 .03 .005" pos="{radius - 0.012} 0 {h / 2}" mass="0" rgba="1 .5 0 1"/>
      <geom name="patch" type="cylinder" size=".025 .001" pos="0 0 {h / 2}" mass="0" rgba=".1 .3 1 1"/>
      {wheels}
    </body>
  </worldbody>
</mujoco>"""
