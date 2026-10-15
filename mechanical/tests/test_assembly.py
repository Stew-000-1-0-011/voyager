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
