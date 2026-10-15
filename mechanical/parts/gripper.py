"""Gripper parts other than the gears (PETG print unless noted)."""

from __future__ import annotations

from build123d import Align, Box, Cylinder, Part, Pos, Rot

from params import ARM_T, BOX, GEAR_FACE, MODULE, MOTOR_HOLE_D, MOTOR_HOLE_PITCH, PAD_T, PAD_Z_MAX, PAD_Z_MIN
from parts.gears import RACK_BASE

CLEAR = 0.3
WALL = 3.0
ARM_Z_MIN = PAD_Z_MIN - 10
ARM_Z_MAX = PAD_Z_MAX + 10
GRIP_PLATE_T = 3.0
PINION_GAP = 1.0   # pinion underside above the motor plate


RACK_DEPTH = 2.25 * MODULE + RACK_BASE  # tooth tip to back face


def guide_block() -> Part:
    """Fixed to the frame; the rack (along local X) slides through its slot. Teeth side (+Y) stays open
    so the pinion can mesh."""
    slot_y = RACK_DEPTH + 2 * CLEAR
    slot_z = GEAR_FACE + 2 * CLEAR
    block = Box(20, slot_y + WALL, slot_z + 2 * WALL, align=(Align.MIN, Align.MAX, Align.CENTER))
    return block - Pos(-1, 0.01, 0) * Box(22, slot_y, slot_z, align=(Align.MIN, Align.MAX, Align.CENTER))


def arm(length: float) -> Part:
    """Vertical plate in the XZ plane (thickness along Y), x 0..length, z ARM_Z_MIN..ARM_Z_MAX."""
    return Pos(0, 0, ARM_Z_MIN) * Box(length, ARM_T, ARM_Z_MAX - ARM_Z_MIN, align=(Align.MIN, Align.CENTER, Align.MIN))


def pad() -> Part:
    """Rubber-faced pad covering the box side: x 0..BOX, z PAD_Z_MIN..PAD_Z_MAX, thickness along Y."""
    return Pos(0, 0, PAD_Z_MIN) * Box(BOX, PAD_T, PAD_Z_MAX - PAD_Z_MIN, align=(Align.MIN, Align.CENTER, Align.MIN))


def arm_bracket(height: float) -> Part:
    """Connects the rack (above) to the arm plate (below): a post of the given height."""
    return Box(20, ARM_T, height, align=(Align.MIN, Align.CENTER, Align.MIN))


def grip_motor_mount() -> Part:
    """Plate holding the JGA25-370 vertically (shaft up) under the pinion. Plate top at z = 0; the gearbox
    face screws to its underside (z = -GRIP_PLATE_T) so the shaft reaches into the pinion."""
    plate = Box(40, 40, GRIP_PLATE_T, align=(Align.CENTER, Align.CENTER, Align.MAX))
    plate -= Cylinder(7.5, GRIP_PLATE_T + 1, align=(Align.CENTER, Align.CENTER, Align.MAX))  # shaft boss
    for dx in (-MOTOR_HOLE_PITCH / 2, MOTOR_HOLE_PITCH / 2):
        plate -= Pos(dx, 0, 0) * Cylinder(MOTOR_HOLE_D / 2, GRIP_PLATE_T + 1,
                                           align=(Align.CENTER, Align.CENTER, Align.MAX))
    return plate


def rack_stop() -> Part:
    """Block bolted on the rack; hits a guide block at the end of travel."""
    return Box(8, RACK_DEPTH + 4, GEAR_FACE + 2 * WALL, align=(Align.MIN, Align.MAX, Align.CENTER))
