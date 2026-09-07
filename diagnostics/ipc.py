"""Decode the fixed-size, private local IPC emitted by Telemetry/diagnostics.cpp."""

import math
import struct
from dataclasses import dataclass

HEADER = struct.Struct("<4sI16siIQQQQ")
MOTOR = struct.Struct("<7diiIIQ")
TAIL = struct.Struct("<33dIQQ")
PACKET_BYTES = HEADER.size + 4 * MOTOR.size + TAIL.size


def finite(value):
    return round(value, 6) if math.isfinite(value) else None


@dataclass(frozen=True)
class Snapshot:
    producer_id: str
    robot_id: int
    cycle: int
    acquired_us: int
    offered: int
    dropped: int
    wheels: list
    values: tuple
    flags: int
    kick_requested: int
    kick_sent: int


def decode(packet: bytes) -> Snapshot:
    if len(packet) != PACKET_BYTES:
        raise ValueError("incorrect local snapshot length")
    magic, version, producer, robot, reserved, cycle, stamp, offered, dropped = (
        HEADER.unpack_from(packet)
    )
    if magic != b"PHI1" or version != 1 or reserved != 0:
        raise ValueError("unsupported local snapshot")
    if not 0 <= robot <= 15 or stamp == 0 or dropped > offered:
        raise ValueError("invalid local snapshot identity or counters")
    offset = HEADER.size
    wheels = []
    for wheel_id in range(1, 5):
        fields = MOTOR.unpack_from(packet, offset)
        offset += MOTOR.size
        requested, sent, velocity, position, current, voltage, temperature = fields[:7]
        mode, fault, replied, energized, age_us = fields[7:]
        if replied not in (0, 1) or energized not in (0, 1):
            raise ValueError("invalid local motor flags")
        measured = [
            finite(value) if replied else None
            for value in (velocity, position, current, voltage, temperature)
        ]
        wheels.append(
            [
                wheel_id,
                finite(requested),
                finite(sent) if energized else None,
                *measured,
                mode if replied else None,
                fault if replied else None,
                bool(replied),
                age_us if replied else None,
                bool(energized),
            ]
        )
    tail = TAIL.unpack_from(packet, offset)
    return Snapshot(
        producer.hex(),
        robot,
        cycle,
        stamp,
        offered,
        dropped,
        wheels,
        tuple(finite(value) for value in tail[:33]),
        tail[33],
        tail[34],
        tail[35],
    )
