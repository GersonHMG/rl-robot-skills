"""The robot: a MuJoCo model plus a simple omni-wheel model.

The body slides on the floor (x, y slide joints and a z hinge). Wheels are
not simulated as bodies. Each physics step:

1. Wheel speeds: the robot-frame command becomes four wheel speeds, capped
   at the motor top speed, then the wheel faults below are applied.
2. Wheel forces: the force that would bring the body to those wheel speeds
   is split over the wheels. The motor torque caps them (all scaled
   together, like the robot's motor controller), then each wheel's grip on
   the floor caps it. Past the grip the wheel slips and
   only pushes with sliding friction (lower) until the demand drops again.
3. The wheel forces add up to the body force and torque MuJoCo applies.

Wheel forces are not equal: a sideways push needs more from each wheel than
a forward one (the wheels point mostly forward), so with weak grip lateral
and sudden moves slip first, and a slipping wheel pushes the body off line.

Bad wheels ([wheels] in the config): each wheel can turn slower than asked
(gain), not turn at all below a speed (deadband, stiction), wobble (noise, a
slow random speed error), and grip less than the others (grip, worn tyre). The robot does not know: it computes
wheel speeds for perfect wheels, so a bad robot drifts and curves.
"""

import math

import mujoco
import numpy as np

G = 9.81
SLIP_RGBA = (1.0, 0.1, 0.1, 1.0)
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
        self.split = np.linalg.pinv(self.A.T)        # body force -> least wheel forces

        mass = r["mass"]
        inertia = 0.5 * mass * r["radius"] ** 2
        self.gain_force = np.array([mass, mass, inertia]) / m["response"]
        self.max_force = m["max_wheel_torque"] / self.wheel_radius
        w = cfg.get("wheels", {})
        tyres = np.array(w.get("grip", [1.0] * n), dtype=float)  # worn tyres < 1
        self.grip = tyres * m["friction"] * mass * G / n     # static, per wheel
        self.slide = self.grip * m.get("kinetic", 0.7)       # once slipping
        self.slipping = np.zeros(n, dtype=bool)

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

        want = self.J_inv @ self.actual_wheels()
        demand = self.split @ (self.gain_force * (want - self.velocity()))
        # motor torque: the robot's controller scales all wheels together (keeps
        # the direction); grip is the floor's, per wheel, and bends it
        peak = np.max(np.abs(demand))
        force = demand * (self.max_force / peak) if peak > self.max_force else demand
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
        """Paint slipping wheels red (viewer)."""
        for g, slip in zip(self.wheel_geoms, self.slipping):
            self.model.geom_rgba[g] = SLIP_RGBA if slip else WHEEL_RGBA


def _xml(cfg: dict) -> str:
    r = cfg["robot"]
    radius, h, mass = r["radius"], r["height"], r["mass"]
    wheels = "".join(
        f'<geom name="wheel{i}" type="cylinder" size="{r["wheel_radius"]} 0.006" rgba="{" ".join(map(str, WHEEL_RGBA))}"'
        f' pos="{r["wheel_distance"] * math.cos(a):.4f} {r["wheel_distance"] * math.sin(a):.4f} {r["wheel_radius"] - h / 2:.4f}"'
        f' zaxis="{math.cos(a):.4f} {math.sin(a):.4f} 0" mass="0"/>'
        for i, a in enumerate(np.radians(r["wheel_angles"]))
    )
    return f"""
<mujoco model="ssl_robot">
  <option timestep="0.002" gravity="0 0 -{G}"/>
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
      <joint name="x" type="slide" axis="1 0 0"/>
      <joint name="y" type="slide" axis="0 1 0"/>
      <joint name="theta" type="hinge" axis="0 0 1"/>
      <geom name="shell" type="cylinder" size="{radius} {h / 2}" mass="{mass}" rgba="{r.get("color", ".1 .1 .1")} .6"/>
      <geom name="front" type="box" size=".01 .03 .005" pos="{radius - 0.012} 0 {h / 2}" mass="0" rgba="1 .5 0 1"/>
      <geom name="patch" type="cylinder" size=".025 .001" pos="0 0 {h / 2}" mass="0" rgba=".1 .3 1 1"/>
      {wheels}
    </body>
  </worldbody>
</mujoco>"""
