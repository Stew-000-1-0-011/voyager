"""Involute spur gear and rack outlines (pure geometry, mm)."""

from __future__ import annotations

import math

Point = tuple[float, float]


def pitch_radius(teeth: int, module: float) -> float:
    return teeth * module / 2


def _inv(a: float) -> float:
    return math.tan(a) - a


def gear_outline(teeth: int, module: float, pressure_angle_deg: float = 20.0,
                 points_per_flank: int = 12) -> list[Point]:
    """Closed CCW outline: addendum m, dedendum 1.25m, tooth thickness pi*m/2 on the pitch circle."""
    a = math.radians(pressure_angle_deg)
    r = pitch_radius(teeth, module)
    rb = r * math.cos(a)
    ra = r + module
    rf = r - 1.25 * module
    half = math.pi / (2 * teeth) + _inv(a)  # half tooth angle on the base circle (polar)

    def psi(rho: float) -> float:
        """Polar half-angle of the tooth at radius rho (>= rb)."""
        return half - _inv(math.acos(rb / rho))

    r0 = max(rb, rf)
    radii = [r0 + (ra - r0) * i / (points_per_flank - 1) for i in range(points_per_flank)]
    pts: list[Point] = []
    for k in range(teeth):
        c = 2 * math.pi * k / teeth
        left = [(rho, c - psi(rho)) for rho in radii]
        right = [(rho, c + psi(rho)) for rho in reversed(radii)]
        if rf < rb:  # radial segment from the root circle up to the base circle
            left.insert(0, (rf, c - psi(rb)))
            right.append((rf, c + psi(rb)))
        tip = [(ra, c - psi(ra) + (2 * psi(ra)) * t / 3) for t in (1, 2)]
        for rho, th in left + tip + right:
            pts.append((rho * math.cos(th), rho * math.sin(th)))
    return pts


def rack_outline(n_teeth: int, module: float, pressure_angle_deg: float = 20.0,
                 base_height: float = 6.0) -> list[Point]:
    """Closed outline along +x; pitch line y = 0, tips at y = +m, roots at y = -1.25m."""
    p = math.pi * module
    t = math.tan(math.radians(pressure_angle_deg))
    y_tip, y_root = module, -1.25 * module
    y_base = y_root - base_height
    pts: list[Point] = [(0.0, y_base), (0.0, y_root)]
    for k in range(n_teeth):
        xc = p * (k + 0.5)
        pts += [
            (xc - (p / 4 + 1.25 * module * t), y_root),
            (xc - (p / 4 - module * t), y_tip),
            (xc + (p / 4 - module * t), y_tip),
            (xc + (p / 4 + 1.25 * module * t), y_root),
        ]
    pts += [(n_teeth * p, y_root), (n_teeth * p, y_base)]
    return pts
