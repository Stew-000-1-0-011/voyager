import math

import pytest

from parts.involute import gear_outline, pitch_radius, rack_outline


def test_pitch_radius():
    assert pitch_radius(20, 1.0) == 10.0


def test_gear_outline_radii():
    pts = gear_outline(20, 1.0)
    radii = [math.hypot(x, y) for x, y in pts]
    assert max(radii) == pytest.approx(11.0, abs=0.05)
    assert min(radii) == pytest.approx(8.75, abs=0.05)


def test_gear_outline_has_20_teeth():
    pts = gear_outline(20, 1.0)
    radii = [math.hypot(x, y) for x, y in pts]
    tips = sum(1 for i in range(len(radii)) if radii[i] > 10.9 and radii[i - 1] <= 10.9)
    assert tips == 20


def test_rack_pitch_is_pi_m():
    pts = rack_outline(10, 1.0)
    # start x of each tip land (y == +m entered from below)
    starts = sorted(x for (x, y), (_, py) in zip(pts, [pts[-1]] + pts[:-1])
                    if y == pytest.approx(1.0) and py < 1.0 - 1e-9)
    assert len(starts) == 10
    gaps = [b - a for a, b in zip(starts, starts[1:])]
    assert all(g == pytest.approx(math.pi, abs=1e-6) for g in gaps)
