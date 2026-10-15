"""Carrier board: the single source of truth for parts, pins and nets.

The schematic (gen_schematic.py) and the perfboard documents
(gen_perfboard.py) are generated from this file. Pin assignment follows
spec section 5.4.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Part:
    ref: str
    value: str
    symbol: str
    footprint: str
    pins: dict[str, str]  # pin number -> net name (unconnected pins omitted)


# Socket pin 1..20; pin 1 is the end opposite the USB connector. On a standard Blue Pill the two
# edges run in opposite directions: PB12 faces GND at the USB end, 3.3V faces VB at the far end.
BLUEPILL_ROW_A = ["3V3", "G", "5V", "PB9", "PB8", "PB7", "PB6", "PB5", "PB4", "PB3",
                  "PA15", "PA12", "PA11", "PA10", "PA9", "PA8", "PB15", "PB14", "PB13", "PB12"]
BLUEPILL_ROW_B = ["VB", "PC13", "PC14", "PC15", "PA0", "PA1", "PA2", "PA3", "PA4", "PA5",
                  "PA6", "PA7", "PB0", "PB1", "PB10", "PB11", "R", "3V3", "G", "G"]

# 5V-tolerant pins of STM32F103C8 (datasheet "FT").
FT_PINS = frozenset(
    [f"PA{i}" for i in range(8, 16)] + [f"PB{i}" for i in (2, 3, 4)] + [f"PB{i}" for i in range(6, 16)]
)

MCU_PIN_NET: dict[str, str] = {
    # Pins are grouped so each row of the Blue Pill serves its own side of the board:
    # row B (PA0..PB11) the left side, row A (PB9..PB12) the right side and the gripper.
    # Motor drivers (PWM x2 per motor)
    "PA0": "L_IN1", "PA1": "L_IN2",
    "PB7": "R_IN1", "PB6": "R_IN2", "PB9": "G_IN1", "PB8": "G_IN2",
    # Encoders
    "PA6": "ENC_L_A", "PA7": "ENC_L_B", "PA8": "ENC_R_A", "PA9": "ENC_R_B",
    "PB4": "ENC_G_A", "PB3": "ENC_G_B",
    # Ultrasonic: one trigger per sensor; front pair and rear pair are fired together
    "PB0": "US_TRIG_LF", "PB1": "US_TRIG_LR", "PB15": "US_TRIG_RF", "PB13": "US_TRIG_RR",
    "PB10": "US_ECHO_LF", "PB11": "US_ECHO_LR", "PB14": "US_ECHO_RF", "PB12": "US_ECHO_RR",
    # Misc
    "PA4": "VBAT_SENSE", "PA2": "UART_TX", "PA3": "UART_RX", "PA12": "USB_DP",
    # Power
    "5V": "+5V_BP", "3V3": "+3V3", "G": "GND",
}

_CONN = "Connector_Generic:Conn_01x{:02d}"
_XH = "Connector_JST:JST_XH_B{n}B-XH-A_1x{n:02d}_P2.50mm_Vertical"
_TB2 = "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal"
_R = "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal"


def _row(names: list[str]) -> dict[str, str]:
    return {str(i + 1): MCU_PIN_NET[n] for i, n in enumerate(names) if n in MCU_PIN_NET}


def _conn(ref: str, value: str, nets_in_order: list[str], footprint: str) -> Part:
    return Part(ref, value, _CONN.format(len(nets_in_order)), footprint,
                {str(i + 1): n for i, n in enumerate(nets_in_order)})


def _two(ref: str, value: str, symbol: str, footprint: str, n1: str, n2: str) -> Part:
    return Part(ref, value, symbol, footprint, {"1": n1, "2": n2})


def _build_parts() -> list[Part]:
    socket = "Connector_PinSocket_2.54mm:PinSocket_1x20_P2.54mm_Vertical"
    parts = [
        Part("J_BP_A", "Bluepill row A", _CONN.format(20), socket, _row(BLUEPILL_ROW_A)),
        Part("J_BP_B", "Bluepill row B", _CONN.format(20), socket, _row(BLUEPILL_ROW_B)),
        _conn("J_PWR", "VBAT in", ["GND", "VBAT"], _TB2),
    ]
    for m in ("L", "R", "G"):
        # same order as the DRV8871 module header (GND, VM, IN1, IN2) so the cable runs straight through;
        # VM is fed from the battery harness, not from the board
        parts.append(Part(f"J_DRV_{m}", "Driver IN", _CONN.format(4), _XH.format(n=4),
                          {"1": "GND", "3": f"{m}_IN1", "4": f"{m}_IN2"}))
    for m in ("L", "R", "G"):
        parts.append(_conn(f"J_ENC_{m}", "Encoder", ["GND", "+3V3", f"ENC_{m}_A", f"ENC_{m}_B"], _XH.format(n=4)))
    for pos in ("LF", "RF", "LR", "RR"):
        # same order as the HC-SR04 header (Vcc, Trig, Echo, Gnd)
        parts.append(_conn(f"J_US_{pos}", "HC-SR04", ["+5V", f"US_TRIG_{pos}", f"US_ECHO_{pos}_RAW", "GND"],
                           _XH.format(n=4)))
        parts.append(_two(f"R_ECHO_{pos}", "1k", "Device:R", _R, f"US_ECHO_{pos}_RAW", f"US_ECHO_{pos}"))
    parts += [
        # MP1584 IN- and OUT- are the same node on the module: one GND wire
        _conn("J_BUCK", "Buck module", ["GND", "VBAT", "+5V"], _XH.format(n=3)),
        _two("D_5V", "1N5819", "Device:D_Schottky", "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal",
             "+5V_BP", "+5V"),
        _two("R_DIV_H", "47k", "Device:R", _R, "VBAT", "VBAT_SENSE"),
        _two("R_DIV_L", "10k", "Device:R", _R, "VBAT_SENSE", "GND"),
        _two("C_DIV", "100n", "Device:C", "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm",
             "VBAT_SENSE", "GND"),
        _two("C_BULK", "470u 25V", "Device:C_Polarized", "Capacitor_THT:CP_Radial_D8.0mm_P3.50mm",
             "VBAT", "GND"),
        _two("R_USB", "1k8 DNP", "Device:R", _R, "USB_DP", "+3V3"),
        _conn("J_UART", "UART", ["GND", "+3V3", "UART_TX", "UART_RX"], _XH.format(n=4)),
    ]
    return parts


PARTS: list[Part] = _build_parts()


def pin_count(p: Part) -> int:
    """Physical pins, including ones left unconnected."""
    if p.symbol.startswith("Connector_Generic:Conn_01x"):
        return int(p.symbol.rsplit("x", 1)[1])
    return len(p.pins)


def ground_pin(p: Part) -> str | None:
    """The GND pin of a cable connector: its first or last pin, the one that sits on the board edge."""
    for pin in ("1", str(pin_count(p))):
        if p.pins.get(pin) == "GND":
            return pin
    return None

# Pins tied together on the Blue Pill module itself: the board only has to reach one of each group.
MODULE_BRIDGES: dict[str, list[tuple[str, str]]] = {
    "+3V3": [("J_BP_A", "1"), ("J_BP_B", "18")],
    "GND": [("J_BP_A", "2"), ("J_BP_B", "19"), ("J_BP_B", "20")],
}


def nets() -> dict[str, list[tuple[str, str]]]:
    out: dict[str, list[tuple[str, str]]] = {}
    for p in PARTS:
        for pin, net in p.pins.items():
            out.setdefault(net, []).append((p.ref, pin))
    return out


# ---------------------------------------------------------------------------
# Perfboard placement (2.54mm grid). Column 0 = robot left edge, row 0 = front
# edge, viewed from the component side. Rotation: direction in which pin
# numbers advance (0: +col, 90: +row, 180: -col, 270: -row).

BOARD_HOLES = (33, 28)  # ~84 x 71mm, fits a 90 x 75mm board

PLACEMENT: dict[str, tuple[int, int, int]] = {
    # Cable connectors stand at right angles to the board edge with their GND pin (first or last) on the perimeter,
    # where a bare ring wire on the solder side joins them all. Each connector sits on the side of the
    # Blue Pill row that drives it, in the same order as its pins, so the signal wires hardly cross.
    # Bluepill: pin 1 toward the front, USB toward the rear edge
    "J_BP_B": (13, 4, 90),
    "J_BP_A": (19, 4, 90),
    # front edge (pins run toward the rear)
    "J_UART": (5, 0, 90),
    "J_DRV_L": (11, 0, 90),
    "J_DRV_G": (21, 0, 90),
    "J_DRV_R": (24, 0, 90),
    # left edge (pins run inward)
    "J_ENC_L": (0, 7, 0),
    "J_US_LF": (3, 16, 180),
    "J_US_LR": (3, 20, 180),
    # right edge
    "J_ENC_G": (32, 4, 180),
    "J_ENC_R": (32, 9, 180),
    "J_US_RF": (29, 20, 0),
    "J_US_RR": (29, 25, 0),
    # rear edge (pins run toward the front): power in, buck module
    "J_PWR": (1, 27, 270),
    "J_BUCK": (7, 27, 270),
    # interior
    "C_BULK": (4, 25, 90),
    "R_DIV_H": (10, 25, 0),
    "R_DIV_L": (15, 25, 270),
    "C_DIV": (10, 22, 0),
    "D_5V": (13, 26, 0),
    "R_ECHO_LF": (2, 18, 0),
    "R_ECHO_LR": (4, 23, 0),
    "R_ECHO_RF": (29, 17, 270),
    "R_ECHO_RR": (27, 25, 270),
    "R_USB": (17, 19, 270),
}

# Hole step between consecutive pin numbers.
_PIN_STEP = {"J_PWR": 2,
             "R_": 4, "D_5V": 4, "C_DIV": 2, "C_BULK": 1}

_DIRS = {0: (1, 0), 90: (0, 1), 180: (-1, 0), 270: (0, -1)}


def _pin_step(ref: str) -> int:
    for key, step in _PIN_STEP.items():
        if ref == key or (key.endswith("_") and ref.startswith(key)):
            return step
    return 1


def pin_hole(ref: str, pin: str) -> tuple[int, int]:
    c, r, rot = PLACEMENT[ref]
    dc, dr = _DIRS[rot]
    k = (int(pin) - 1) * _pin_step(ref)
    return c + dc * k, r + dr * k


def part_outline(ref: str) -> set[tuple[int, int]]:
    """Holes covered on the component side by a part's leads and body (approximate outline).
    XH housings and terminal blocks reach one hole past their end pins and one hole to either side."""
    p = next(q for q in PARTS if q.ref == ref)
    pins = [pin_hole(ref, str(k)) for k in range(1, pin_count(p) + 1)]
    (c1, r1), (c2, r2) = pins[0], pins[-1]
    n = max(abs(c2 - c1), abs(r2 - r1)) or 1
    dc, dr = _DIRS[PLACEMENT[ref][2]]
    line = [(c1 + dc * k, r1 + dr * k) for k in range(n + 1)]
    if ref == "C_BULK":  # 8mm can
        c, r = pins[0]
        return {(c + x * abs(dr) + y * abs(dc), r + x * abs(dc) + y * abs(dr))
                for x in range(-1, 2) for y in range(-1, 3)}
    if ref.startswith(("J_ENC", "J_US", "J_DRV", "J_PWR", "J_BUCK", "J_UART")):
        ends = [(c1 - dc, r1 - dr)] + line + [(c2 + dc, r2 + dr)]
        return {(c + dr * k, r + dc * k) for c, r in ends for k in (-1, 0, 1)}
    return set(line)
