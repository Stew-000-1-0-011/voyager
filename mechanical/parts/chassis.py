"""Drive base: 20mm extrusion frame, drive motor mounts, caster mount, board mount."""

from __future__ import annotations

from build123d import Align, Box, Cylinder, Part, Pos, Rot

from params import (BATTERY, BOARD, CASTER_H, EXTRUSION, FRAME_D, FRAME_W, FRAME_Z, MID_CROSS_X, MOTOR_DIA,
                    MOTOR_HOLE_D, MOTOR_HOLE_PITCH, TRAY_Y, WHEEL_D, WHEEL_X)

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
        ("mid_cross", FRAME_W - 2 * E, _bar_y(FRAME_W - 2 * E, MID_CROSS_X)),
    ]


# the tray and the battery box bridge the rear member and the mid cross member
BAY_X = (-FRAME_D / 2 + 2.0, MID_CROSS_X + E / 2 - 2.0)
TRAY_T = 3.0
BOARD_X = BAY_X[0] + 2.0 + BOARD[1] / 2   # perfboard centre (rear part of the tray, front = +X)
BOARD_Y = -14.0


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
    """Electronics tray bridging the rear member and the mid cross member (M5 + T-nuts), with four M3
    standoffs for the perfboard. The drivers, the buck module and the power splice sit on the rest of
    the tray (double-sided tape / cable ties). Placed in the robot frame, bottom on the member tops."""
    x0, x1 = BAY_X
    plate = Pos(x0, TRAY_Y[0], 0) * Box(x1 - x0, TRAY_Y[1] - TRAY_Y[0], TRAY_T,
                                        align=(Align.MIN, Align.MIN, Align.MIN))
    for sx in (-1, 1):
        for sy in (-1, 1):
            plate += Pos(BOARD_X + sx * (BOARD[1] / 2 - 4), BOARD_Y + sy * (BOARD[0] / 2 - 4), TRAY_T) * Cylinder(
                3, 8, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for x in (-FRAME_D / 2 + E / 2, MID_CROSS_X):  # M5 holes into the member slots
        for y in (TRAY_Y[0] + 10, TRAY_Y[1] - 10):
            plate -= Pos(x, y, 0) * Cylinder(2.7, 2 * TRAY_T)
    return plate


BATT_WALL = 2.0
BATT_CLEAR = 1.0


def battery_box() -> Part:
    """Open-top printed box for the LiPo pack, bridging the rear member and the mid cross member.
    Two slots in the floor take a hook-and-loop strap over the pack; the rear wall is cut for the leads.
    Local frame: centred on the pack in X / Y, bottom at z = 0."""
    L, W, H = BATTERY
    il, iw = L + 2 * BATT_CLEAR, W + 2 * BATT_CLEAR
    ol, ow, oh = il + 2 * BATT_WALL, iw + 2 * BATT_WALL, BATT_WALL + H * 0.75
    box = Box(ol, ow, oh, align=(Align.CENTER, Align.CENTER, Align.MIN))
    box -= Pos(0, 0, BATT_WALL) * Box(il, iw, oh, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for dx in (-L / 4, L / 4):  # strap slots
        box -= Pos(dx, 0, 0) * Box(4, ow + 2, BATT_WALL * 2 + 0.1, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    box -= Pos(-ol / 2, 0, BATT_WALL + H / 2) * Box(BATT_WALL * 3, iw * 0.6, oh, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for x in (-ol / 2 + 9, ol / 2 - 9):  # M5 into the members
        box -= Pos(x, 0, 0) * Cylinder(2.7, 2 * BATT_WALL)
    return box


def battery_box_size() -> tuple[float, float]:
    L, W, _ = BATTERY
    return L + 2 * BATT_CLEAR + 2 * BATT_WALL, W + 2 * BATT_CLEAR + 2 * BATT_WALL


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
