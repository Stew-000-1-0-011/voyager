"""Pinion and racks (module 1, PETG print)."""

from __future__ import annotations

import math

from build123d import Align, Box, Cylinder, Face, Part, Pos, Rot, Wire, extrude

from params import GEAR_FACE, MODULE, MOTOR_SHAFT_D, MOTOR_SHAFT_FLAT, PINION_TEETH, PRESSURE_ANGLE
from parts.involute import gear_outline, rack_outline

PINION_OUTER_D = (PINION_TEETH + 2) * MODULE
RACK_BASE = 6.0  # rack body thickness behind the tooth roots
M3_CLEAR = 3.2
M3_NUT_AF = 5.6
M3_NUT_T = 2.6


def _prism(points: list[tuple[float, float]], height: float) -> Part:
    face = Face(Wire.make_polygon([(x, y, 0) for x, y in points], close=True))
    return extrude(face, height)


def pinion_blank() -> Part:
    return _prism(gear_outline(PINION_TEETH, MODULE, PRESSURE_ANGLE), GEAR_FACE)


def pinion() -> Part:
    """Gear centred on the Z axis, z 0..GEAR_FACE, D bore with an M3 set screw + nut trap on +X."""
    r = MOTOR_SHAFT_D / 2
    flat = MOTOR_SHAFT_FLAT - r  # centre -> flat
    bore = Cylinder(r, GEAR_FACE, align=(Align.CENTER, Align.CENTER, Align.MIN))
    bore -= Pos(flat + r, 0, GEAR_FACE / 2) * Box(2 * r, 2 * r + 1, GEAR_FACE + 1)
    screw = Pos(0, 0, GEAR_FACE / 2) * Rot(0, 90, 0) * Cylinder(M3_CLEAR / 2, 12,
                                                                align=(Align.CENTER, Align.CENTER, Align.MIN))
    nut = Pos(5.0, 0, GEAR_FACE / 2) * Box(M3_NUT_T, M3_NUT_AF, GEAR_FACE,
                                          align=(Align.CENTER, Align.CENTER, Align.MIN))
    return pinion_blank() - bore - screw - nut


def rack_teeth(length: float) -> int:
    return math.ceil(length / (math.pi * MODULE))


def rack(length: float) -> Part:
    """Rack along +X from x=0; pitch line y=0 with teeth toward +Y; z 0..GEAR_FACE.
    Two M3 holes through the base (along Y) to fix it to its slider."""
    n = rack_teeth(length)
    body = _prism(rack_outline(n, MODULE, PRESSURE_ANGLE, RACK_BASE), GEAR_FACE)
    y_mid = -1.25 * MODULE - RACK_BASE / 2
    for x in (15.0, n * math.pi * MODULE - 15.0):
        body -= Pos(x, y_mid, GEAR_FACE / 2) * Rot(90, 0, 0) * Cylinder(M3_CLEAR / 2, RACK_BASE + 2)
    return body


def _rack_len() -> float:
    # imported lazily to avoid a cycle: the assembly decides where the arms are
    from assembly import ARM_PLATE_Y_OPEN

    # at full open the rack must still cover the pinion (y = 0) with a full tooth width beyond it
    return ARM_PLATE_Y_OPEN + PINION_OUTER_D / 2 + 5.0


RACK_LEN = _rack_len()
