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
