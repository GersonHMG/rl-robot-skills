"""uv run python -m unittest"""

import math
import socket
import unittest
from pathlib import Path

import numpy as np

import wire
from robot import Robot
from sim import Sim, load_config

CFG = load_config(Path(__file__).parent / "config.toml")


def run(robot: Robot, cmd, secs: float):
    for _ in range(round(secs / robot.dt)):
        robot.step(*cmd)


class WireTest(unittest.TestCase):
    def test_grsim_command_round_trip(self):
        yellow, robots = wire.grsim_commands(wire.grsim_packet(3, True, 1.5, -0.5, 2.0))
        self.assertTrue(yellow)
        self.assertEqual(robots, [{"id": 3, "vx": 1.5, "vy": -0.5, "omega": 2.0}])

    def test_vision_packet_fields(self):
        det = wire.parse(wire.parse(wire.vision_packet(7, 1.0, 2, False, 0.5, -1.0, 0.25))[1][0])
        self.assertEqual(det[1], [7])
        self.assertNotIn(6, det)  # no yellow robots
        robot = wire.parse(det[7][0])
        self.assertEqual(robot[2], [2])
        self.assertAlmostEqual(wire._float(robot, 3), 500.0)
        self.assertAlmostEqual(wire._float(robot, 4), -1000.0)
        self.assertAlmostEqual(wire._float(robot, 5), 0.25)


class RobotTest(unittest.TestCase):
    def test_wheel_model_round_trip(self):
        r = Robot(CFG)
        v = np.array([0.7, -0.3, 1.2])
        np.testing.assert_allclose(r.J_inv @ (r.J @ v), v, atol=1e-9)

    def test_drives_forward_in_robot_frame(self):
        r = Robot(CFG)
        r.data.qpos[2] = math.pi / 2  # facing +y
        run(r, (1.0, 0.0, 0.0), 1.0)
        x, y, th = r.pose()
        vx, vy, w = r.velocity()
        self.assertAlmostEqual(vx, 1.0, delta=0.02)
        self.assertAlmostEqual(vy, 0.0, delta=0.02)
        self.assertGreater(y, 0.7)
        self.assertAlmostEqual(x, 0.0, delta=0.02)

    def test_turns(self):
        r = Robot(CFG)
        run(r, (0.0, 0.0, 3.0), 0.5)
        self.assertAlmostEqual(r.velocity()[2], 3.0, delta=0.05)

    def test_motor_limits(self):
        r = Robot(CFG)
        run(r, (10.0, 0.0, 0.0), 0.05)
        self.assertLess(r.velocity()[0], 0.5)  # torque limits the acceleration
        run(r, (10.0, 0.0, 0.0), 3.0)
        top = r.velocity()[0]
        self.assertLess(top, 10.0)  # wheel speed limits the top speed
        self.assertGreater(top, 3.0)
        self.assertLessEqual(np.max(np.abs(r.wheels)), r.max_wheel_speed + 1e-9)


class BadRobotTest(unittest.TestCase):
    def test_config_overrides_base(self):
        bad = load_config(Path(__file__).parent / "bad_robot.toml")
        self.assertEqual(bad["wheels"]["deadband"], 1.5)
        self.assertEqual(bad["robot"]["mass"], CFG["robot"]["mass"])  # kept from config.toml
        self.assertEqual(CFG["wheels"]["gain"], [1.0] * 4)

    def test_weak_wheel_curves(self):
        cfg = dict(CFG, wheels={"gain": [1.0, 0.75, 1.0, 1.0]})
        r = Robot(cfg)
        run(r, (1.0, 0.0, 0.0), 1.0)
        self.assertGreater(abs(r.pose()[2]), 0.05)  # asked straight, it turns

    def test_deadband_stalls_slow_moves(self):
        r = Robot(dict(CFG, wheels={"deadband": 5.0}))
        run(r, (0.1, 0.0, 0.0), 1.0)
        self.assertAlmostEqual(r.pose()[0], 0.0, delta=1e-6)
        run(r, (0.5, 0.0, 0.0), 1.0)
        self.assertGreater(r.pose()[0], 0.3)

    def test_noise_is_seeded(self):
        cfg = dict(CFG, wheels={"noise": 0.2, "seed": 1})
        a, b = Robot(cfg), Robot(cfg)
        run(a, (1.0, 0.0, 0.0), 0.5)
        run(b, (1.0, 0.0, 0.0), 0.5)
        self.assertEqual(a.pose(), b.pose())
        self.assertNotAlmostEqual(a.pose()[1], 0.0, places=4)


def ramp(robot: Robot, cmd, accel: float = 2.0, secs: float = 1.5):
    """Speed up like small_engine (accel m/s^2), then hold. Returns slip steps."""
    cmd = np.array(cmd, float)
    top = np.linalg.norm(cmd[:2])
    slips = 0
    for i in range(round(secs / robot.dt)):
        robot.step(*(cmd * min(1.0, i * robot.dt * accel / top)))
        slips += robot.slipping.any()
    return slips


class SlipRobotTest(unittest.TestCase):
    SLIP = load_config(Path(__file__).parent / "slip_robot.toml")
    SLIP["wheels"]["seed"] = 1

    def test_good_robot_never_slips(self):
        for cmd in [(1, 0, 0), (0, 1, 0), (0.7, 0.7, 0)]:
            r = Robot(CFG)
            run(r, cmd, 1.0)  # even a sudden start
            self.assertFalse(r.slipping.any() or r.limited.any())
            np.testing.assert_allclose(r.velocity(), cmd, atol=0.01)

    def test_good_robot_slides_nearly_straight(self):
        r = Robot(CFG)
        ramp(r, (0, 1, 0), secs=2.0)
        self.assertLess(abs(r.pose()[2]), 0.06)

    def test_lateral_spins_forward_roughly_straight(self):
        forward, lateral = Robot(self.SLIP), Robot(self.SLIP)
        ramp(forward, (1, 0, 0), secs=2.0)
        self.assertLess(abs(forward.pose()[2]), 0.5)  # wanders, does not spin
        ramp(lateral, (0, 1, 0), secs=2.0)
        self.assertGreater(lateral.data.qpos[2], 2.0)      # turned more than 2 rad,
        self.assertGreater(lateral.velocity()[2], 1.5)     # and still spinning
        self.assertTrue(lateral.limited[1] and lateral.limited[2])  # back motors maxed

    def test_shakes(self):
        good, slip = Robot(CFG), Robot(self.SLIP)
        shake = {}
        for name, r in (("good", good), ("slip", slip)):
            ramp(r, (1, 0, 0), secs=1.0)
            v = []
            for _ in range(60):
                run(r, (1, 0, 0), 1 / 60)
                v.append(r.velocity()[0])
            shake[name] = np.std(np.diff(v)) * 60  # m/s^2
        self.assertLess(shake["good"], 0.01)
        self.assertGreater(shake["slip"], 0.1)

    def test_low_grip_sudden_start_slips_then_recovers(self):
        cfg = dict(CFG, motor=dict(CFG["motor"], friction=0.3, max_wheel_torque=0.3))
        r = Robot(cfg)
        run(r, (1, 0, 0), 0.1)
        self.assertTrue(r.slipping.any())
        run(r, (1, 0, 0), 2.0)
        self.assertFalse(r.slipping.any())


class SimTest(unittest.TestCase):
    def test_command_drives_robot_and_times_out(self):
        cfg = dict(CFG, network=dict(CFG["network"], command_addr="127.0.0.1:0"))
        sim = Sim(cfg)
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tx.sendto(wire.grsim_packet(9, False, 1.0, 0, 0), sim.commands.getsockname())  # other robot
        tx.sendto(wire.grsim_packet(sim.id, False, 0.5, 0, 0), sim.commands.getsockname())
        for _ in range(250):
            sim.step()
        self.assertAlmostEqual(sim.robot.velocity()[0], 0.5, delta=0.02)
        for _ in range(500):  # past command_timeout: stops
            sim.step()
        self.assertAlmostEqual(sim.robot.velocity()[0], 0.0, delta=0.01)
        self.assertGreater(sim.frame, 0)
        tx.close(); sim.commands.close(); sim.vision.close()


if __name__ == "__main__":
    unittest.main()
