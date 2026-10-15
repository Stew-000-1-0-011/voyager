import pytest

from assembly import chassis
from params import FRAME_D, FRAME_W
from parts.chassis import frame_members


def test_frame_outer_size():
    bb = None
    for _, _, solid in frame_members():
        bb = solid.bounding_box() if bb is None else bb.add(solid.bounding_box())
    assert bb.size.X == pytest.approx(FRAME_D)
    assert bb.size.Y == pytest.approx(FRAME_W)


def test_wheel_contacts_floor():
    parts = chassis()
    for name in ("wheel_l", "wheel_r"):
        assert parts[name].bounding_box().min.Z == pytest.approx(0.0, abs=0.5)


def test_caster_contacts_floor():
    assert chassis()["caster"].bounding_box().min.Z == pytest.approx(0.0, abs=1.0)


# --- whole assembly (open / closed) ---------------------------------------

import itertools  # noqa: E402

from build123d import Align, Cylinder, Pos, Rot  # noqa: E402

from assembly import build  # noqa: E402
from params import BOX, CLOSED_INNER, ENVELOPE_MAX, OPEN_INNER, PAD_Z_MAX, PAD_Z_MIN, SONAR_Z  # noqa: E402

OPEN = build(True)
CLOSED = build(False)
# pinion teeth are not phased with the rack teeth; mesh overlap is expected there
MESH_PAIRS = {frozenset(("pinion", "rack_l")), frozenset(("pinion", "rack_r"))}


def _bbox(parts):
    bb = None
    for p in parts.values():
        bb = p.bounding_box() if bb is None else bb.add(p.bounding_box())
    return bb


def _overlap(a, b) -> float:
    if not a.bounding_box().overlaps(b.bounding_box()):
        return 0.0
    return (a & b).volume


def test_open_envelope_within_500():
    bb = _bbox(OPEN)
    assert bb.size.X <= ENVELOPE_MAX and bb.size.Y <= ENVELOPE_MAX


def test_closed_envelope():
    bb = _bbox(CLOSED)
    assert bb.size.Y <= 320
    assert bb.size.X <= ENVELOPE_MAX


def _inner(parts) -> float:
    return parts["pad_l"].bounding_box().min.Y - parts["pad_r"].bounding_box().max.Y


def test_open_inner_width():
    assert _inner(OPEN) == pytest.approx(OPEN_INNER, abs=1)


def test_closed_inner_width():
    assert _inner(CLOSED) == pytest.approx(CLOSED_INNER, abs=1)


def test_closed_racks_do_not_collide():
    for a, b in itertools.combinations(("rack_l", "rack_r", "motor_l", "motor_r"), 2):
        assert _overlap(CLOSED[a], CLOSED[b]) < 1e-3, (a, b)


def test_arms_clear_wheels():
    for parts in (OPEN, CLOSED):
        for a in ("arm_l", "arm_r", "pad_l", "pad_r"):
            for w in ("wheel_l", "wheel_r"):
                assert _overlap(parts[a], parts[w]) < 1e-3, (a, w)


def test_no_part_collides_with_another():
    for parts in (OPEN, CLOSED):
        for a, b in itertools.combinations(parts, 2):
            if frozenset((a, b)) in MESH_PAIRS:
                continue
            assert _overlap(parts[a], parts[b]) < 1e-2, (a, b)


def test_sonar_line_of_sight_clear():
    for parts in (OPEN, CLOSED):
        for name in ("sonar_lf", "sonar_lr", "sonar_rf", "sonar_rr"):
            bb = parts[name].bounding_box()
            left = name[6] == "l"
            x = (bb.min.X + bb.max.X) / 2
            y0 = bb.max.Y if left else bb.min.Y
            ray = Pos(x, y0, SONAR_Z) * Rot(-90 if left else 90, 0, 0) * Cylinder(
                10, 50, align=(Align.CENTER, Align.CENTER, Align.MIN))
            for other, p in parts.items():
                if other != name:
                    assert _overlap(ray, p) < 1e-3, (name, other)


def test_pad_height_hits_box():
    for side in ("pad_l", "pad_r"):
        bb = OPEN[side].bounding_box()
        assert bb.min.Z == pytest.approx(PAD_Z_MIN) and bb.max.Z == pytest.approx(PAD_Z_MAX)
        assert 0 <= bb.min.Z and bb.max.Z <= BOX
