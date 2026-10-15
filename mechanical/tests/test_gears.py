import math

import pytest
from assembly import PINION_X, rack_pitch_lines
from params import GEAR_FACE, PINION_TEETH, MODULE, STROKE
from parts.gears import HUB_H, RACK_LEN, pinion, pinion_blank, rack


def test_pinion_outer_diameter():
    bb = pinion().bounding_box()
    assert bb.size.X == pytest.approx(22.0, abs=0.1)
    assert bb.size.Y == pytest.approx(22.0, abs=0.1)
    assert bb.size.Z == pytest.approx(GEAR_FACE + HUB_H, abs=1e-6)


def test_pinion_has_d_bore():
    # D bore through the 12mm gear section alone removes at least 80% of a 4mm cylinder
    gear_only = pinion() & pinion_blank()
    removed = pinion_blank().volume - gear_only.volume
    assert removed >= math.pi * 2**2 * GEAR_FACE * 0.8


def test_rack_covers_stroke():
    bb = rack(RACK_LEN).bounding_box()
    assert bb.size.X >= STROKE + 40


def test_pinion_meshes_with_racks():
    front, rear = rack_pitch_lines()
    r = PINION_TEETH * MODULE / 2
    assert front == pytest.approx(PINION_X + r)
    assert rear == pytest.approx(PINION_X - r)
