"""Just enough protobuf to read grSim commands and write SSL-Vision packets,
so the sim needs no protoc and no protobuf package. Field numbers follow
small_engine/proto/*.proto.
"""

import struct

VARINT, FIXED64, LEN, FIXED32 = 0, 1, 2, 5


def _varint(n: int) -> bytes:
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def _key(field: int, kind: int) -> bytes:
    return _varint(field << 3 | kind)


def uint(field: int, v: int) -> bytes:
    return _key(field, VARINT) + _varint(v)


def f32(field: int, v: float) -> bytes:
    return _key(field, FIXED32) + struct.pack("<f", v)


def f64(field: int, v: float) -> bytes:
    return _key(field, FIXED64) + struct.pack("<d", v)


def msg(field: int, body: bytes) -> bytes:
    return _key(field, LEN) + _varint(len(body)) + body


def parse(buf: bytes) -> dict[int, list]:
    """Fields -> raw values: int (varint), bytes (len), or the 4/8 raw bytes."""
    fields: dict[int, list] = {}
    i = 0
    while i < len(buf):
        key, i = _read_varint(buf, i)
        field, kind = key >> 3, key & 7
        if kind == VARINT:
            v, i = _read_varint(buf, i)
        elif kind == FIXED64:
            v, i = buf[i : i + 8], i + 8
        elif kind == FIXED32:
            v, i = buf[i : i + 4], i + 4
        elif kind == LEN:
            n, i = _read_varint(buf, i)
            v, i = buf[i : i + n], i + n
        else:
            raise ValueError(f"unsupported wire type {kind}")
        fields.setdefault(field, []).append(v)
    return fields


def _read_varint(buf: bytes, i: int) -> tuple[int, int]:
    n = shift = 0
    while True:
        if i >= len(buf):
            raise ValueError("truncated varint")
        b = buf[i]
        i += 1
        n |= (b & 0x7F) << shift
        shift += 7
        if not b & 0x80:
            return n, i


def _float(f: dict, field: int) -> float:
    return struct.unpack("<f", f[field][-1])[0] if field in f else 0.0


def grsim_commands(buf: bytes) -> tuple[bool, list[dict]]:
    """grSim_Packet -> (is_team_yellow, [{id, vx, vy, omega}]).

    veltangent is the robot's forward speed, velnormal its left speed.
    """
    packet = parse(buf)
    if 1 not in packet:
        return False, []
    cmds = parse(packet[1][-1])
    yellow = bool(cmds.get(2, [0])[-1])
    robots = []
    for raw in cmds.get(3, []):
        rc = parse(raw)
        robots.append(
            {"id": rc.get(1, [0])[-1], "vx": _float(rc, 4), "vy": _float(rc, 5), "omega": _float(rc, 6)}
        )
    return yellow, robots


def grsim_packet(robot_id: int, yellow: bool, vx: float, vy: float, omega: float) -> bytes:
    """A grSim_Packet with one velocity command (used by the tests)."""
    rc = (uint(1, robot_id) + f32(2, 0) + f32(3, 0) + f32(4, vx) + f32(5, vy)
          + f32(6, omega) + uint(7, 0) + uint(8, 0))
    return msg(1, f64(1, 0.0) + uint(2, int(yellow)) + msg(3, rc))


def vision_packet(frame: int, t: float, robot_id: int, yellow: bool,
                  x: float, y: float, theta: float) -> bytes:
    """SSL_WrapperPacket with one detected robot; x, y in metres."""
    robot = (f32(1, 0.95) + uint(2, robot_id) + f32(3, x * 1000) + f32(4, y * 1000)
             + f32(5, theta) + f32(6, 0) + f32(7, 0))
    det = (uint(1, frame) + f64(2, t) + f64(3, t) + uint(4, 0)
           + msg(6 if yellow else 7, robot))
    return msg(1, det)
