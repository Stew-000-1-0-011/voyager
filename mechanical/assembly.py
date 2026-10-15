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
