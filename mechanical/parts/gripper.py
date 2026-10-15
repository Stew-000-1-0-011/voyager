"""Gripper parts other than the gears (PETG print unless noted)."""

from __future__ import annotations

from build123d import Align, Box, Cylinder, Part, Pos, Rot

from params import ARM_T, BOX, MOTOR_HOLE_D, MOTOR_HOLE_PITCH, PAD_T, PAD_Z_MAX, PAD_Z_MIN, RAIL_SECTION

CLEAR = 0.3
WALL = 3.0
ARM_Z_MIN = PAD_Z_MIN - 10
ARM_Z_MAX = PAD_Z_MAX + 10


def slider() -> Part:
    """U channel riding on the square rail (rail along local X); the rack bolts to its top."""
    outer = RAIL_SECTION + 2 * (CLEAR + WALL)
    s = Box(30, outer, outer, align=(Align.MIN, Align.CENTER, Align.CENTER))
    s -= Box(32, RAIL_SECTION + 2 * CLEAR, RAIL_SECTION + 2 * CLEAR, align=(Align.MIN, Align.CENTER, Align.CENTER))
    return s - Pos(-1, 0, -outer / 2) * Box(32, RAIL_SECTION, WALL + CLEAR, align=(Align.MIN, Align.CENTER, Align.MIN))


def arm(length: float) -> Part:
    """Vertical plate in the XZ plane (thickness along Y), x 0..length, z ARM_Z_MIN..ARM_Z_MAX."""
    return Box(length, ARM_T, ARM_Z_MAX - ARM_Z_MIN, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
        Pos(0, 0, ARM_Z_MIN).location)


def pad() -> Part:
    """Rubber-faced pad covering the box side: x 0..BOX, z PAD_Z_MIN..PAD_Z_MAX, thickness along Y."""
    return Pos(0, 0, PAD_Z_MIN) * Box(BOX, PAD_T, PAD_Z_MAX - PAD_Z_MIN, align=(Align.MIN, Align.CENTER, Align.MIN))


def arm_bracket(height: float) -> Part:
    """Connects the rack (above) to the arm plate (below): a post of the given height."""
    return Box(20, ARM_T, height, align=(Align.MIN, Align.CENTER, Align.MIN))


def grip_motor_mount() -> Part:
    """Plate holding the JGA25-370 vertically (shaft up) under the pinion; motor face at z=0."""
    plate = Box(40, 40, 5, align=(Align.CENTER, Align.CENTER, Align.MAX))
    plate -= Cylinder(7.5, 6, align=(Align.CENTER, Align.CENTER, Align.MAX))  # gearbox boss clearance
    for dx in (-MOTOR_HOLE_PITCH / 2, MOTOR_HOLE_PITCH / 2):
        plate -= Pos(dx, 0, 0) * Cylinder(MOTOR_HOLE_D / 2, 6, align=(Align.CENTER, Align.CENTER, Align.MAX))
    return plate


def stopper() -> Part:
    """Block clamped to the rail; the slider hits it at the stroke ends."""
    outer = RAIL_SECTION + 2 * (CLEAR + WALL)
    s = Box(10, outer, outer, align=(Align.MIN, Align.CENTER, Align.CENTER))
    return s - Box(12, RAIL_SECTION + 2 * CLEAR, RAIL_SECTION + 2 * CLEAR, align=(Align.MIN, Align.CENTER, Align.CENTER))

