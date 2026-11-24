"""Single SSL robot in MuJoCo, speaking grSim: velocity commands in over UDP,
SSL-Vision packets of the robot out over multicast. small_engine drives it
with `[output] kind = "grsim"`, same as the real robot through the radio.

    uv run sim.py [--config config.toml] [--headless]
    uv run sim.py --config bad_robot.toml      # robot with bad wheels
    uv run sim.py --config slip_robot.toml     # robot whose wheels slip
"""

import argparse
import collections
import random
import socket
import time
import tomllib
from pathlib import Path

import wire
from robot import Robot


BASE = Path(__file__).parent / "config.toml"


def load_config(path: Path = BASE) -> dict:
    """config.toml, with the keys of `path` (e.g. bad_robot.toml) on top."""
    with open(BASE, "rb") as f:
        cfg = tomllib.load(f)
    if Path(path).resolve() != BASE.resolve():
        with open(path, "rb") as f:
            _merge(cfg, tomllib.load(f))
    return cfg


def _merge(base: dict, over: dict):
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v


def addr(s: str) -> tuple[str, int]:
    host, port = s.rsplit(":", 1)
    return host, int(port)


class Sim:
    """Robot + network. step() advances one physics step in sim time."""

    def __init__(self, cfg: dict):
        self.cfg = cfg
        net = cfg["network"]
        self.robot = Robot(cfg)
        self.id = cfg["robot_id"]
        self.yellow = cfg["team"] == "yellow"
        self.timeout = net["command_timeout"]
        self.command_delay = net.get("command_delay", 0.0)
        self.vision_period = 1.0 / net["vision_hz"]
        self.noise = (net["vision_noise_pos"], net["vision_noise_theta"])

        self.commands = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.commands.bind(addr(net["command_addr"]))
        self.commands.setblocking(False)
        self.vision = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vision.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self.vision_dest = addr(net["vision_addr"])

        self.cmd = (0.0, 0.0, 0.0)
        self.pending = collections.deque()  # (sim time it reaches the wheels, command)
        self.last_cmd = -1e9  # sim time of the last command
        self.next_vision = 0.0
        self.frame = 0

    @property
    def time(self) -> float:
        return self.robot.data.time

    def receive(self):
        while True:
            try:
                buf = self.commands.recv(2048)
            except (BlockingIOError, ConnectionResetError):
                return
            try:
                yellow, robots = wire.grsim_commands(buf)
            except ValueError:
                continue
            for rc in robots:
                if yellow == self.yellow and rc["id"] == self.id:
                    cmd = (rc["vx"], rc["vy"], rc["omega"])
                    self.pending.append((self.time + self.command_delay, cmd))

    def apply_commands(self):
        """Commands reach the wheels `command_delay` after they arrive (radio + firmware)."""
        while self.pending and self.pending[0][0] <= self.time:
            self.cmd = self.pending.popleft()[1]
            self.last_cmd = self.time

    def publish(self):
        x, y, th = self.robot.pose()
        npos, nth = self.noise
        if npos:
            x, y = x + random.gauss(0, npos), y + random.gauss(0, npos)
        if nth:
            th += random.gauss(0, nth)
        packet = wire.vision_packet(self.frame, time.time(), self.id, self.yellow, x, y, th)
        self.vision.sendto(packet, self.vision_dest)
        self.frame += 1

    def step(self):
        self.receive()
        self.apply_commands()
        stale = self.time - self.last_cmd > self.timeout
        self.robot.step(*((0.0, 0.0, 0.0) if stale else self.cmd))
        if self.time >= self.next_vision:
            self.publish()
            self.next_vision += self.vision_period


def catch_up(sim: Sim, start: float):
    """Run physics in real time: step until sim time reaches the wall clock.
    After a stall (window dragged, laptop asleep) skip ahead instead."""
    behind = time.perf_counter() - start - sim.time
    if behind > 0.2:
        sim.robot.data.time += behind
        sim.next_vision = sim.time
        sim.last_cmd = -1e9
    while sim.time < time.perf_counter() - start:
        sim.step()


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--config", type=Path, default=BASE, help="config.toml or a robot file that overrides it")
    p.add_argument("--headless", action="store_true", help="no viewer window")
    args = p.parse_args()

    sim = Sim(load_config(args.config))
    net = sim.cfg["network"]
    team = "yellow" if sim.yellow else "blue"
    print(f"sim: {team} robot {sim.id}, commands on {net['command_addr']}, vision to {net['vision_addr']}")

    start = time.perf_counter()
    if args.headless:
        try:
            while True:
                catch_up(sim, start)
                time.sleep(0.001)
        except KeyboardInterrupt:
            return

    import mujoco.viewer

    with mujoco.viewer.launch_passive(sim.robot.model, sim.robot.data) as viewer:
        viewer.cam.distance, viewer.cam.elevation = 2.0, -50
        while viewer.is_running():
            with viewer.lock():
                catch_up(sim, start)
                viewer.cam.lookat[:2] = sim.robot.pose()[:2]
                sim.robot.show_slip()
            viewer.sync()
            time.sleep(1 / 120)


if __name__ == "__main__":
    main()
