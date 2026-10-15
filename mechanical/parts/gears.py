"""Pinion and racks (module 1, PETG print)."""

from __future__ import annotations

import math

from build123d import Align, Box, Cylinder, Face, Part, Pos, Rot, Wire, extrude

from params import (ARM_T, GEAR_FACE, MODULE, MOTOR_SHAFT_D, MOTOR_SHAFT_FLAT, OPEN_INNER, PAD_T, PINION_TEETH,
                    PRESSURE_ANGLE)
from parts.involute import gear_outline, rack_outline

PINION_OUTER_D = (PINION_TEETH + 2) * MODULE
RACK_BASE = 6.0  # rack body thickness behind the tooth roots
M3_CLEAR = 3.2
M3_NUT_AF = 5.6
M3_NUT_T = 2.6
HUB_D = 14.0
HUB_H = 5.0        # passes through the deck so the 10mm motor shaft engages ~8mm
SET_SCREW_Z = -HUB_H / 2


def _prism(points: list[tuple[float, float]], height: float) -> Part:
    face = Face(Wire.make_polygon([(x, y, 0) for x, y in points], close=True))
    return extrude(face, height, dir=(0, 0, 1))


def pinion_blank() -> Part:
    return _prism(gear_outline(PINION_TEETH, MODULE, PRESSURE_ANGLE), GEAR_FACE)


def pinion() -> Part:
    """Gear centred on the Z axis (gear z 0..GEAR_FACE) with a hub below (z -HUB_H..0) that reaches down
    through the deck to the motor shaft. D bore all through; M3 set screw + nut trap in the hub on +X."""
    r = MOTOR_SHAFT_D / 2
    flat = MOTOR_SHAFT_FLAT - r  # centre -> flat
    body = pinion_blank() + Pos(0, 0, -HUB_H) * Cylinder(HUB_D / 2, HUB_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
    total = GEAR_FACE + HUB_H
    bore = Pos(0, 0, -HUB_H) * Cylinder(r, total, align=(Align.CENTER, Align.CENTER, Align.MIN))
    bore -= Pos(flat + r, 0, -HUB_H + total / 2) * Box(2 * r, 2 * r + 1, total + 1)
    screw = Pos(0, 0, SET_SCREW_Z) * Rot(0, 90, 0) * Cylinder(M3_CLEAR / 2, 12,
                                                              align=(Align.CENTER, Align.CENTER, Align.MIN))
    nut = Pos(4.5, 0, SET_SCREW_Z) * Box(M3_NUT_T, M3_NUT_AF, M3_NUT_AF + 1)
    return body - bore - screw - nut


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


# At full open the rack must still cover the pinion (y = 0) plus a full tooth beyond it.
# Arm plate centre-line at full open = OPEN_INNER / 2 + PAD_T + ARM_T / 2 (see assembly.arm_plate_y).
RACK_LEN = OPEN_INNER / 2 + PAD_T + ARM_T / 2 + PINION_OUTER_D / 2 + 5.0
