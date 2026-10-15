"""HC-SR04 holder."""

from __future__ import annotations

from build123d import Align, Box, Cylinder, Part, Pos, Rot

from params import SONAR_BOARD, SONAR_CAN_D, SONAR_CAN_H, SONAR_CAN_PITCH

HOLDER_T = 3.0


def sonar_mount() -> Part:
    """Plate behind the sensor PCB plus a clamp to the side member. Local frame: sensor faces +Y,
    can centres at z = 0, plate back face at y = 0."""
    w, h = SONAR_BOARD
    plate = Box(w + 6, HOLDER_T, h + 6, align=(Align.CENTER, Align.MIN, Align.CENTER))
    for dx in (-(w / 2 - 2), w / 2 - 2):  # M2 holes matching the HC-SR04 corner holes
        plate -= Pos(dx, 0, h / 2 - 2) * Rot(90, 0, 0) * Cylinder(1.1, 20)
        plate -= Pos(dx, 0, -(h / 2 - 2)) * Rot(90, 0, 0) * Cylinder(1.1, 20)
    return plate


def sonar_body() -> Part:
    """Purchased HC-SR04 (for clearance checks): PCB + two cans facing +Y, can centres at z = 0."""
    w, h = SONAR_BOARD
    body = Box(w, 1.6, h, align=(Align.CENTER, Align.MIN, Align.CENTER))
    for dx in (-SONAR_CAN_PITCH / 2, SONAR_CAN_PITCH / 2):
        body += Pos(dx, 1.6, 0) * Rot(-90, 0, 0) * Cylinder(SONAR_CAN_D / 2, SONAR_CAN_H,
                                                             align=(Align.CENTER, Align.CENTER, Align.MIN))
    return body
