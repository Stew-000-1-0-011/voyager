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


CLAMP_X0 = -25.0   # clamp extends this far behind the motor axis
CLAMP_BOLT_X = 19.0
CLAMP_BOTTOM = 17.5  # cap bottom below the axis (ground clearance = axis height - this)
BORE_R = MOTOR_DIA / 2  # split clamp: the bolts squeeze the halves onto the gearbox


def _clamp_y0() -> float:
    from params import GEARBOX_CLAMP_LEN

    return -GEARBOX_CLAMP_LEN


def motor_clamp() -> Part:
    """Upper half of the split clamp around the JGA25 gearbox. Local frame: motor axis along Y with the
    gearbox face at y = 0 (body toward -y), axis at x = z = 0. Sits on the cross member top (M5 + T-nut)
    and bears on its rear face (M5 + T-nut); the cap closes it from below with two M3."""
    y0 = _clamp_y0()
    member_front = CROSS_X + E / 2 - WHEEL_X   # cross member faces, relative to the motor axis
    member_rear = CROSS_X - E / 2 - WHEEL_X
    top = FRAME_Z + E - WHEEL_D / 2 + MOUNT_T  # 5mm over the member top
    width = -y0
    body = Pos(CLAMP_X0, y0, 0) * Box(member_rear - CLAMP_X0, width, top, align=(Align.MIN, Align.MIN, Align.MIN))
    body += Pos(member_rear, y0, top - MOUNT_T) * Box(member_front - member_rear, width, MOUNT_T,
                                                      align=(Align.MIN, Align.MIN, Align.MIN))
    body -= Pos(0, y0 / 2, 0) * Rot(90, 0, 0) * Cylinder(BORE_R, width + 2)
    for dx in (-CLAMP_BOLT_X, CLAMP_BOLT_X):
        body -= Pos(dx, y0 / 2, 0) * Cylinder(1.7, 2 * top, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return body


def motor_clamp_cap() -> Part:
    """Lower half of the clamp (same local frame), z -CLAMP_BOTTOM..0."""
    y0 = _clamp_y0()
    width = -y0
    cap = Pos(CLAMP_X0, y0, -CLAMP_BOTTOM) * Box(-2 * CLAMP_X0, width, CLAMP_BOTTOM,
                                                 align=(Align.MIN, Align.MIN, Align.MIN))
    cap -= Pos(0, y0 / 2, 0) * Rot(90, 0, 0) * Cylinder(BORE_R, width + 2)
    for dx in (-CLAMP_BOLT_X, CLAMP_BOLT_X):
        cap -= Pos(dx, y0 / 2, -CLAMP_BOTTOM - 1) * Cylinder(1.7, CLAMP_BOTTOM + 2,
                                                            align=(Align.CENTER, Align.CENTER, Align.MIN))
    return cap


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


UPRIGHT_Y = 40.0   # clear of the gripper motor (r 12.5) and of the right bracket (|y| >= 70)


def upright_members() -> list[tuple[str, float, Part]]:
    """Two vertical 20mm extrusions on the front member carrying the gripper deck (top at DECK_BOTTOM)."""
    from params import DECK_T, RACK_Z

    deck_bottom = RACK_Z - 0.3 - 3.0 - DECK_T  # rack clearance, guide wall, deck
    length = deck_bottom - (FRAME_Z + E)
    x = FRAME_D / 2 - E / 2
    return [(f"upright_{n}", length,
             Pos(x, s * UPRIGHT_Y, FRAME_Z + E) * Box(E, E, length, align=(Align.CENTER, Align.CENTER, Align.MIN)))
            for n, s in (("l", 1), ("r", -1))]
