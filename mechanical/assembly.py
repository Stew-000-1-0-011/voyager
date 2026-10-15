"""Placement of all parts (open / closed) and exports."""

from __future__ import annotations

from params import ARM_T, FRAME_D, MODULE, OPEN_INNER, PAD_T, PINION_TEETH

PINION_R = PINION_TEETH * MODULE / 2
PINION_X = FRAME_D / 2 - 10.0          # pinion axis just behind the frame front edge
# arm plate centre-line y at full open (pad inner face at OPEN_INNER / 2)
ARM_PLATE_Y_OPEN = OPEN_INNER / 2 + PAD_T + ARM_T / 2


def rack_pitch_lines() -> tuple[float, float]:
    """X of the left (front) and right (rear) rack pitch lines; both racks run along Y."""
    return PINION_X + PINION_R, PINION_X - PINION_R


# ---------------------------------------------------------------------------
# chassis

from build123d import Align, Box, Cylinder, Part, Pos, Rot  # noqa: E402

from params import (CASTER_D, CASTER_H, EXTRUSION, FRAME_W, FRAME_Z, MOTOR_DIA, MOTOR_LEN,  # noqa: E402
                    SONAR_SPACING, SONAR_Z, WHEEL_D, WHEEL_W, WHEEL_X)
from parts.chassis import MOUNT_T, board_mount, caster_mount, drive_motor_mount, frame_members  # noqa: E402
from parts.sensors import HOLDER_T, sonar_body, sonar_mount  # noqa: E402

WHEEL_Y = FRAME_W / 2 - EXTRUSION - 3 - WHEEL_W / 2   # wheel centre |y|
GEARBOX_FACE_Y = WHEEL_Y - WHEEL_W / 2 - 2 - MOUNT_T  # motor gearbox face |y|
SONAR_X = SONAR_SPACING / 2
ESTOP = (70.0, 70.0, 70.0)


def _side(part: Part, left: bool) -> Part:
    """Parts are modelled for the left (+Y) side; mirror by rotating 180 degrees about Z."""
    return part if left else Rot(0, 0, 180) * part


def chassis() -> dict[str, Part]:
    parts: dict[str, Part] = {}
    frame = None
    for _, _, solid in frame_members():
        frame = solid if frame is None else frame + solid
    parts["frame"] = frame
    axis_z = WHEEL_D / 2
    for side, left in (("l", True), ("r", False)):
        sgn = 1 if left else -1
        wheel = Pos(WHEEL_X, sgn * WHEEL_Y, axis_z) * Rot(90, 0, 0) * Cylinder(WHEEL_D / 2, WHEEL_W)
        motor = Pos(WHEEL_X, 0, axis_z) * _side(
            Pos(0, GEARBOX_FACE_Y, 0) * Rot(90, 0, 0) * Cylinder(MOTOR_DIA / 2, MOTOR_LEN,
                                                                 align=(Align.CENTER, Align.CENTER, Align.MIN)), left)
        mount = Pos(WHEEL_X, 0, axis_z) * _side(Pos(0, GEARBOX_FACE_Y, 0) * drive_motor_mount(), left)
        parts[f"wheel_{side}"] = wheel
        parts[f"motor_{side}"] = motor
        parts[f"motor_mount_{side}"] = mount
    rear_x = -(FRAME_D / 2 - EXTRUSION / 2)
    parts["caster_mount"] = Pos(rear_x, 0, FRAME_Z) * caster_mount()
    parts["caster"] = Pos(rear_x, 0, 0) * Cylinder(CASTER_D / 2, CASTER_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
    parts["board"] = Pos(-40, 0, FRAME_Z + EXTRUSION) * board_mount()
    parts["estop"] = Pos(-FRAME_D / 2 + ESTOP[0] / 2, FRAME_W / 2 - ESTOP[1] / 2, FRAME_Z + EXTRUSION) * Box(
        *ESTOP, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for name, x, left in (("sonar_lf", SONAR_X, True), ("sonar_lr", -SONAR_X, True),
                          ("sonar_rf", SONAR_X, False), ("sonar_rr", -SONAR_X, False)):
        holder = Pos(0, FRAME_W / 2, 0) * sonar_mount()
        body = Pos(0, FRAME_W / 2 + HOLDER_T, 0) * sonar_body()
        parts[name] = Pos(x, 0, SONAR_Z) * _side(holder + body, left)
    return parts
