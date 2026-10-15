"""Drive base: 20mm extrusion frame, drive motor mounts, caster mount, board mount."""

from __future__ import annotations

from build123d import Align, Box, Cylinder, Part, Pos, Rot

from params import (BOARD, CASTER_H, EXTRUSION, FRAME_D, FRAME_W, FRAME_Z, MOTOR_DIA, MOTOR_HOLE_D, MOTOR_HOLE_PITCH,
                    WHEEL_D, WHEEL_X)

E = EXTRUSION
CROSS_X = WHEEL_X + WHEEL_D / 2 + E / 2 + 3  # motor cross member just ahead of the wheels
MOUNT_T = 5.0


def _bar_y(length: float, x: float) -> Part:
    return Pos(x, 0, FRAME_Z) * Box(E, length, E, align=(Align.CENTER, Align.CENTER, Align.MIN))


def _bar_x(length: float, y: float) -> Part:
    return Pos(0, y, FRAME_Z) * Box(length, E, E, align=(Align.CENTER, Align.CENTER, Align.MIN))


def frame_members() -> list[tuple[str, float, Part]]:
    """(name, cut length, placed solid) for each 20mm extrusion."""
    side = FRAME_D - 2 * E
    return [
        ("front", FRAME_W, _bar_y(FRAME_W, FRAME_D / 2 - E / 2)),
        ("rear", FRAME_W, _bar_y(FRAME_W, -FRAME_D / 2 + E / 2)),
        ("left", side, _bar_x(side, FRAME_W / 2 - E / 2)),
        ("right", side, _bar_x(side, -FRAME_W / 2 + E / 2)),
        ("motor_cross", FRAME_W - 2 * E, _bar_y(FRAME_W - 2 * E, CROSS_X)),
    ]


def drive_motor_mount() -> Part:
    """Hangs from the cross member: top flange (M5 to T-nut) + vertical plate bolted to the gearbox face.
    Local frame: gearbox face plane y = 0 (motor body toward -y), motor axis at x = 0, z = 0."""
    reach = CROSS_X - WHEEL_X  # cross member centre is this far ahead of the motor axis
    top = FRAME_Z + E - WHEEL_D / 2  # cross member top above the axis
    plate = Pos(0, 0, top + MOUNT_T) * Box(MOTOR_DIA + 10, MOUNT_T, top + MOUNT_T + MOTOR_DIA / 2 + 4,
                                           align=(Align.CENTER, Align.MIN, Align.MAX))
    plate -= Rot(90, 0, 0) * Cylinder(4.0, 20)  # shaft boss clearance
    for dx in (-MOTOR_HOLE_PITCH / 2, MOTOR_HOLE_PITCH / 2):
        plate -= Pos(dx, 0, 0) * Rot(90, 0, 0) * Cylinder(MOTOR_HOLE_D / 2, 20)
    flange = Pos(-MOTOR_DIA / 2, 0, top) * Box(reach + E / 2 + MOTOR_DIA / 2, MOUNT_T, MOUNT_T,
                                                align=(Align.MIN, Align.MIN, Align.MIN))
    return plate + flange


def caster_mount() -> Part:
    """Spacer between the rear member underside and the ball caster flange."""
    return Box(40, 40, max(FRAME_Z - CASTER_H, 1.0), align=(Align.CENTER, Align.CENTER, Align.MAX))


def board_mount() -> Part:
    """Four M3 standoffs for the perfboard, on a plate bridging two members."""
    plate = Box(BOARD[0] + 10, BOARD[1] + 10, 3, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for sx in (-1, 1):
        for sy in (-1, 1):
            plate += Pos(sx * (BOARD[0] / 2 - 4), sy * (BOARD[1] / 2 - 4), 3) * Cylinder(3, 8,
                                                                                      align=(Align.CENTER, Align.CENTER, Align.MIN))
    return plate
