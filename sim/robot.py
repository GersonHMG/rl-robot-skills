"""The robot: a MuJoCo model plus a simple omni-wheel model.

The body slides on the floor (x, y slide joints and a z hinge). Wheels are
not simulated as bodies. Each step the robot-frame command is turned into
wheel speeds, limited by the motors (speed and torque), turned back into a
body velocity, and velocity actuators push the body toward it. The actuator
force is capped by floor friction, so hard pushes saturate like slipping.

Bad wheels ([wheels] in the config): each wheel can turn slower than asked
(gain), not turn at all below a speed (deadband, stiction), and wobble
(noise, a slow random speed error). The robot does not know: it computes
wheel speeds for perfect wheels, so a bad robot drifts and curves.
"""

import math

import mujoco
import numpy as np

G = 9.81


class Robot:
    def __init__(self, cfg: dict):
        r, m = cfg["robot"], cfg["motor"]
        self.wheel_radius = r["wheel_radius"]
        self.max_wheel_speed = m["max_wheel_speed"]
        angles = np.radians(r["wheel_angles"])
        # wheel speed (rad/s) = J @ (vx, vy, omega), robot frame
        self.J = np.column_stack(
            [-np.sin(angles), np.cos(angles), np.full(len(angles), r["wheel_distance"])]
        ) / self.wheel_radius
        self.J_inv = np.linalg.pinv(self.J)
        # torque -> per-wheel acceleration, each wheel carrying a share of the mass
        wheel_force = m["max_wheel_torque"] / self.wheel_radius
        self.max_wheel_accel = wheel_force / (r["mass"] / len(angles)) / self.wheel_radius

        self.model = mujoco.MjModel.from_xml_string(_xml(cfg))
        self.data = mujoco.MjData(self.model)
        self.wheels = np.zeros(len(angles))  # current wheel speed targets, rad/s

        w = cfg.get("wheels", {})
        self.gain = np.array(w.get("gain", [1.0] * len(angles)), dtype=float)
        self.deadband = w.get("deadband", 0.0)
        self.noise = w.get("noise", 0.0)
        self.noise_time = w.get("noise_time", 0.1)
        self.wobble = np.zeros(len(angles))
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
        # motor torque limit: same, on the change
        change = target - self.wheels
        peak = np.max(np.abs(change))
        limit = self.max_wheel_accel * self.dt
        if peak > limit:
            change *= limit / peak
        self.wheels += change

        bvx, bvy, w = self.J_inv @ self.actual_wheels()
        c, s = math.cos(self.data.qpos[2]), math.sin(self.data.qpos[2])
        self.data.ctrl[:] = (c * bvx - s * bvy, s * bvx + c * bvy, w)
        mujoco.mj_step(self.model, self.data)

    def actual_wheels(self) -> np.ndarray:
        """Wheel speeds the floor sees, after the wheel faults."""
        if self.noise:  # Ornstein-Uhlenbeck: std `noise`, correlated over `noise_time`
            k = self.dt / self.noise_time
            self.wobble += -self.wobble * k + self.noise * math.sqrt(2 * k) * self.rng.standard_normal(len(self.wobble))
        actual = self.wheels * self.gain * (1 + self.wobble)
        actual[np.abs(self.wheels) < self.deadband] = 0.0
        return actual


def _xml(cfg: dict) -> str:
    r, m = cfg["robot"], cfg["motor"]
    radius, h, mass = r["radius"], r["height"], r["mass"]
    inertia = 0.5 * mass * radius**2
    push = m["friction"] * mass * G  # most force the floor can take
    turn = push * r["wheel_distance"]
    kv, kw = mass / m["response"], inertia / m["response"]
    wheels = "".join(
        f'<geom type="cylinder" size="{r["wheel_radius"]} 0.004" rgba=".2 .2 .2 1"'
        f' pos="{r["wheel_distance"] * math.cos(a):.4f} {r["wheel_distance"] * math.sin(a):.4f} {r["wheel_radius"] - h / 2:.4f}"'
        f' zaxis="{math.cos(a):.4f} {math.sin(a):.4f} 0" mass="0"/>'
        for a in np.radians(r["wheel_angles"])
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
      <joint name="x" type="slide" axis="1 0 0" damping="0"/>
      <joint name="y" type="slide" axis="0 1 0"/>
      <joint name="theta" type="hinge" axis="0 0 1"/>
      <geom name="shell" type="cylinder" size="{radius} {h / 2}" mass="{mass}" rgba="{r.get("color", ".1 .1 .1")} .6"/>
      <geom name="front" type="box" size=".01 .03 .005" pos="{radius - 0.012} 0 {h / 2}" mass="0" rgba="1 .5 0 1"/>
      <geom name="patch" type="cylinder" size=".025 .001" pos="0 0 {h / 2}" mass="0" rgba=".1 .3 1 1"/>
      {wheels}
    </body>
  </worldbody>
  <actuator>
    <velocity joint="x" kv="{kv}" forcelimited="true" forcerange="-{push} {push}"/>
    <velocity joint="y" kv="{kv}" forcelimited="true" forcerange="-{push} {push}"/>
    <velocity joint="theta" kv="{kw}" forcelimited="true" forcerange="-{turn} {turn}"/>
  </actuator>
</mujoco>"""
