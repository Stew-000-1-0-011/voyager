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

from build123d import Align, Box, Cylinder, Part, Plane, Pos, Rot, mirror  # noqa: E402

from params import (CASTER_D, CASTER_H, EXTRUSION, FRAME_W, FRAME_Z, MOTOR_DIA, MOTOR_LEN,  # noqa: E402
                    SONAR_SPACING, SONAR_Z, WHEEL_D, WHEEL_W, WHEEL_X)
from parts.chassis import (BAY_X, BATT_WALL, TRAY_T, battery_box, board_mount, caster_mount,  # noqa: E402
                           frame_members, motor_clamp, motor_clamp_cap, upright_members)
from params import BATTERY, BATTERY_Y, ELECTRONICS_H, TRAY_Y  # noqa: E402
from parts.sensors import HOLDER_T, sonar_body, sonar_mount  # noqa: E402

WHEEL_Y = FRAME_W / 2 - EXTRUSION - 3 - WHEEL_W / 2   # wheel centre |y|
GEARBOX_FACE_Y = WHEEL_Y - WHEEL_W / 2 - 3       # motor gearbox face |y| (shaft reaches into the wheel hub)
SONAR_X = SONAR_SPACING / 2
ESTOP = (70.0, 70.0, 70.0)
MAIN_SWITCH = (22.0, 20.0, 30.0)   # rocker switch in a small printed stand (measure)
POWER_SPLICE = (35.0, 20.0, 17.0)  # two 5-way lever connectors (+ and -) for the battery branch
ELECTRONICS_Y1 = 40.0              # electronics envelope ends here; the switch and the splice sit beyond


def _side(part: Part, left: bool) -> Part:
    """Parts are modelled for the left (+Y) side; the right side is their mirror image across y = 0."""
    return part if left else mirror(part, Plane.XZ)


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
        parts[f"wheel_{side}"] = wheel
        parts[f"motor_{side}"] = motor
        parts[f"clamp_{side}"] = Pos(WHEEL_X, 0, axis_z) * _side(Pos(0, GEARBOX_FACE_Y, 0) * motor_clamp(), left)
        parts[f"clamp_cap_{side}"] = Pos(WHEEL_X, 0, axis_z) * _side(
            Pos(0, GEARBOX_FACE_Y, 0) * motor_clamp_cap(), left)
    rear_x = -(FRAME_D / 2 - EXTRUSION / 2)
    parts["caster_mount"] = Pos(rear_x, 0, FRAME_Z) * caster_mount()
    parts["caster"] = Pos(rear_x, 0, 0) * Cylinder(CASTER_D / 2, CASTER_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
    top = FRAME_Z + EXTRUSION
    parts["board"] = Pos(0, 0, top) * board_mount()
    # perfboard, DRV8871 x3, buck module and their wiring: kept as one envelope above the tray
    parts["electronics"] = _box(BAY_X[0] + 1, BAY_X[1] - 1, TRAY_Y[0] + 1, ELECTRONICS_Y1,
                                top + TRAY_T, top + TRAY_T + ELECTRONICS_H)
    parts["main_switch"] = _box(BAY_X[0] + 3, BAY_X[0] + 3 + MAIN_SWITCH[0], TRAY_Y[1] - 2 - MAIN_SWITCH[1],
                                TRAY_Y[1] - 2, top + TRAY_T, top + TRAY_T + MAIN_SWITCH[2])
    parts["power_splice"] = _box(BAY_X[0] + 35, BAY_X[0] + 35 + POWER_SPLICE[0], TRAY_Y[1] - 2 - POWER_SPLICE[1],
                                 TRAY_Y[1] - 2, top + TRAY_T, top + TRAY_T + POWER_SPLICE[2])
    bx = (BAY_X[0] + BAY_X[1]) / 2
    parts["battery_box"] = Pos(bx, BATTERY_Y, top) * battery_box()
    parts["battery"] = Pos(bx, BATTERY_Y, top + BATT_WALL) * Box(*BATTERY, align=(Align.CENTER, Align.CENTER, Align.MIN))
    parts["estop"] = Pos(-FRAME_D / 2 + ESTOP[0] / 2, FRAME_W / 2 - ESTOP[1] / 2, FRAME_Z + EXTRUSION) * Box(
        *ESTOP, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for name, x, left in (("sonar_lf", SONAR_X, True), ("sonar_lr", -SONAR_X, True),
                          ("sonar_rf", SONAR_X, False), ("sonar_rr", -SONAR_X, False)):
        holder = Pos(0, FRAME_W / 2, 0) * sonar_mount()
        body = Pos(0, FRAME_W / 2 + HOLDER_T, 0) * sonar_body()
        parts[name] = Pos(x, 0, SONAR_Z) * _side(holder + body, left)
    return parts


# ---------------------------------------------------------------------------
# gripper

from params import ARM_LEN, BOX, CLOSED_INNER, GEAR_FACE, RACK_Z  # noqa: E402
import math  # noqa: E402

from parts.gears import RACK_LEN, pinion, rack, rack_teeth  # noqa: E402
from parts.gripper import (ARM_Z_MAX, CLEAR, DECK_T, WALL, arm, deck, guide_block, pad,  # noqa: E402
                           rack_stop)

ARM_X0 = FRAME_D / 2 + 2.0            # arm plates start just ahead of the frame
ARM_X1 = FRAME_D / 2 + ARM_LEN        # arm tip
PAD_X0 = ARM_X1 - BOX
POST = 10.0                           # bracket post size
GUIDE_SPANS = ((25.0, 45.0), (50.0, 70.0))  # |y| ranges of the guide blocks (always inside the rack overlap)
DECK_TOP = RACK_Z - CLEAR - WALL      # guide blocks sit on the deck
DECK_HALF_Y = 68.0                    # stays clear of the right bracket post at full close


def arm_plate_y(open_: bool) -> float:
    inner = OPEN_INNER if open_ else CLOSED_INNER
    return inner / 2 + PAD_T + ARM_T / 2


def rack_offsets(open_: bool) -> tuple[float, float]:
    """Y of the outer (arm) end of the left and right racks."""
    y = arm_plate_y(open_)
    return y, -y


def _box(x0, x1, y0, y1, z0, z1) -> Part:
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=(Align.MIN, Align.MIN, Align.MIN))


def gripper(open_: bool) -> dict[str, Part]:
    parts: dict[str, Part] = {}
    yl, yr = rack_offsets(open_)
    front, rear = rack_pitch_lines()
    # racks end flush against their bracket (bolted face to face)
    # left rack: in front of the pinion, teeth toward -X, ends at the inner face of the left bracket
    length = rack_teeth(RACK_LEN) * math.pi * MODULE  # actual printed length (whole teeth)
    end_l = yl - ARM_T / 2
    parts["rack_l"] = Pos(front, end_l - length, RACK_Z) * Rot(0, 0, 90) * rack(RACK_LEN)
    # right rack: behind the pinion, teeth toward +X
    end_r = yr + ARM_T / 2
    parts["rack_r"] = Pos(rear, end_r + length, RACK_Z) * Rot(0, 0, -90) * rack(RACK_LEN)
    parts["pinion"] = Pos(PINION_X, 0, RACK_Z) * pinion()
    # stops ride on the rack backs; at full open they hit the outer end of the inner guide block
    stop_s = GUIDE_SPANS[0][0] - 8.0 - (arm_plate_y(True) - ARM_T / 2) + length
    parts["stop_l"] = Pos(front, end_l - length, RACK_Z) * Rot(0, 0, 90) * Pos(stop_s, 0, 0) * rack_stop()
    parts["stop_r"] = Pos(rear, end_r + length, RACK_Z) * Rot(0, 0, -90) * Pos(stop_s, 0, 0) * rack_stop()

    guides = None
    for y0, _y1 in GUIDE_SPANS:
        for sgn, pitch, rot in ((1, front, 90), (-1, rear, -90)):  # noqa: B007
            # guide local x runs along the rack; local y = 0 at the tooth tips (pitch line + module)
            g = Pos(pitch, sgn * y0, RACK_Z + GEAR_FACE / 2) * Rot(0, 0, rot) * Pos(0, 1.0, 0) * guide_block()
            guides = g if guides is None else guides + g
    parts["guides"] = guides

    # deck under the guide blocks, carried by two 20mm uprights on the front frame member
    gb = guides.bounding_box()
    parts["deck"] = Pos(0, 0, DECK_TOP) * deck(gb.min.X, gb.max.X, DECK_HALF_Y, PINION_X)
    parts["grip_motor"] = Pos(PINION_X, 0, DECK_TOP - DECK_T - MOTOR_LEN) * Cylinder(
        MOTOR_DIA / 2, MOTOR_LEN, align=(Align.CENTER, Align.CENTER, Align.MIN))
    parts["uprights"] = upright_members()[0][2] + upright_members()[1][2]

    rack_top = RACK_Z + GEAR_FACE
    for side, y in (("l", yl), ("r", yr)):
        sgn = 1 if side == "l" else -1
        parts[f"arm_{side}"] = Pos(ARM_X0, y, 0) * arm(ARM_X1 - ARM_X0)
        parts[f"pad_{side}"] = Pos(PAD_X0, sgn * (abs(y) - ARM_T / 2 - PAD_T / 2), 0) * pad()
        ya, yb = y - ARM_T / 2, y + ARM_T / 2
        if side == "l":
            # front rack sits right above the arm start: one post from the arm top to the rack top
            parts["bracket_l"] = _box(ARM_X0, ARM_X0 + POST, ya, yb, ARM_Z_MAX, rack_top)
        else:
            # rear rack is behind the front rack: drop down, then run forward under the front rack band
            x_rack = rear - 1.25 - 6.0
            post = _box(x_rack, x_rack + POST, ya, yb, ARM_Z_MAX - POST, rack_top)
            link = _box(x_rack + POST, ARM_X0, ya, yb, ARM_Z_MAX - POST, ARM_Z_MAX)  # butts against the arm end
            parts["bracket_r"] = post + link
    return parts


def build(open_: bool) -> dict[str, Part]:
    return {**chassis(), **gripper(open_)}


# ---------------------------------------------------------------------------
# exports

from pathlib import Path  # noqa: E402

from build123d import Compound, export_step, export_stl  # noqa: E402

from parts.gripper import rack_stop  # noqa: E402


def printed_parts() -> dict[str, tuple[Part, int]]:
    """3D-printed parts: name -> (part in print-friendly local frame, quantity)."""
    g = gripper(True)
    return {
        "pinion": (pinion(), 1),
        "rack": (rack(RACK_LEN), 2),
        "guide_block": (guide_block(), 4),
        "rack_stop": (rack_stop(), 4),
        "arm": (arm(ARM_X1 - ARM_X0), 2),
        "pad": (pad(), 2),
        "bracket_l": (g["bracket_l"], 1),
        "bracket_r": (g["bracket_r"], 1),
        "deck": (gripper(True)["deck"], 1),
        "motor_clamp": (motor_clamp(), 2),
        "motor_clamp_cap": (motor_clamp_cap(), 2),
        "caster_mount": (caster_mount(), 1),
        "board_mount": (board_mount(), 1),
        "battery_box": (battery_box(), 1),
        "sonar_mount": (sonar_mount(), 4),
    }


def export(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for open_, name in ((True, "open"), (False, "closed")):
        export_step(Compound(children=list(build(open_).values())), str(out_dir / f"assembly_{name}.step"))
    for name, (part, qty) in printed_parts().items():
        export_stl(part, str(out_dir / f"{name}_x{qty}.stl"))


if __name__ == "__main__":
    export(Path(__file__).parent / "build")
