"""Gripper parts other than the gears (PETG print unless noted)."""

from __future__ import annotations

from build123d import Align, Box, Cylinder, Part, Pos, Rot

from params import ARM_T, BOX, GEAR_FACE, MODULE, MOTOR_HOLE_D, MOTOR_HOLE_PITCH, PAD_T, PAD_Z_MAX, PAD_Z_MIN
from parts.gears import RACK_BASE

CLEAR = 0.3
WALL = 3.0
ARM_Z_MIN = PAD_Z_MIN - 10
ARM_Z_MAX = PAD_Z_MAX + 10
DECK_T = 3.0


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


def deck(x0: float, x1: float, half_y: float, hole_x: float) -> Part:
    """Plate under the guide blocks (top at z = 0); the gripper motor screws to its underside and the
    pinion hub passes through the centre hole at x = hole_x."""
    plate = Pos(x0, -half_y, -DECK_T) * Box(x1 - x0, 2 * half_y, DECK_T, align=(Align.MIN, Align.MIN, Align.MIN))
    plate -= Pos(hole_x, 0, 0) * Cylinder(8.0, 2 * DECK_T + 1)
    for dx in (-MOTOR_HOLE_PITCH / 2, MOTOR_HOLE_PITCH / 2):
        plate -= Pos(hole_x + dx, 0, 0) * Cylinder(MOTOR_HOLE_D / 2, 2 * DECK_T + 1)
    return plate


def rack_stop() -> Part:
    """Block bolted to the rack back face (local -Y side, x 0..8). At full open it hits the end face of a
    guide block. Local frame = rack frame: back face at y = -(1.25m + RACK_BASE)."""
    back = -(1.25 * MODULE + RACK_BASE)
    return Pos(0, back, 0) * Box(8, 6, GEAR_FACE, align=(Align.MIN, Align.MAX, Align.MIN))
