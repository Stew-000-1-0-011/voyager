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
    # Motor drivers (PWM x2 per motor)
    "PB6": "L_IN1", "PB7": "L_IN2", "PB8": "R_IN1", "PB9": "R_IN2",
    "PA8": "G_IN1", "PA9": "G_IN2",
    # Encoders
    "PA0": "ENC_L_A", "PA1": "ENC_L_B", "PA6": "ENC_R_A", "PA7": "ENC_R_B",
    "PB3": "ENC_G_A", "PB4": "ENC_G_B",
    # Ultrasonic
    "PB0": "US_TRIG_F", "PB1": "US_TRIG_R",
    "PB12": "US_ECHO_LF", "PB13": "US_ECHO_RF", "PB14": "US_ECHO_LR", "PB15": "US_ECHO_RR",
    # Misc
    "PA4": "VBAT_SENSE", "PA2": "UART_TX", "PA3": "UART_RX",
    "PB10": "I2C_SCL", "PB11": "I2C_SDA", "PA12": "USB_DP",
    "PA5": "SPARE_PA5", "PA10": "SPARE_PA10", "PA15": "SPARE_PA15", "PB5": "SPARE_PB5",
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
        parts.append(_conn(f"J_VM_{m}", "Driver VM", ["GND", "VBAT"], _TB2))
    for m in ("L", "R", "G"):
        parts.append(_conn(f"J_DRV_{m}", "Driver IN", ["GND", f"{m}_IN1", f"{m}_IN2"], _XH.format(n=3)))
    for m in ("L", "R", "G"):
        parts.append(_conn(f"J_ENC_{m}", "Encoder", ["GND", "+3V3", f"ENC_{m}_A", f"ENC_{m}_B"], _XH.format(n=4)))
    for pos in ("LF", "RF", "LR", "RR"):
        trig = "US_TRIG_F" if pos.endswith("F") else "US_TRIG_R"
        parts.append(_conn(f"J_US_{pos}", "HC-SR04", ["GND", "+5V", trig, f"US_ECHO_{pos}_RAW"], _XH.format(n=4)))
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
        _conn("J_I2C", "I2C", ["GND", "+3V3", "I2C_SCL", "I2C_SDA"], _XH.format(n=4)),
        _conn("J_SPARE", "Spare GPIO", ["SPARE_PA5", "SPARE_PA10", "SPARE_PA15", "SPARE_PB5", "+3V3", "GND"],
              "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical"),
    ]
    return parts


PARTS: list[Part] = _build_parts()


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

BOARD_HOLES = (44, 36)  # ~112 x 91mm, fits a 115 x 95mm board

PLACEMENT: dict[str, tuple[int, int, int]] = {
    # Cable connectors stand at right angles to the board edge with pin 1 (always GND) on the perimeter,
    # where a bare ring wire on the solder side joins them all.
    # Bluepill: pin 1 toward the front, USB toward the rear edge
    "J_BP_B": (19, 8, 90),
    "J_BP_A": (25, 8, 90),
    # front edge (pins run toward the rear): drivers, driver power, gripper encoder
    "J_VM_L": (1, 0, 90),
    "J_DRV_L": (6, 0, 90),
    "J_ENC_G": (10, 0, 90),
    "J_DRV_G": (14, 0, 90),
    "J_VM_G": (24, 0, 90),
    "J_DRV_R": (30, 0, 90),
    "J_VM_R": (38, 0, 90),
    # left / right edges (pins run inward): wheel encoders and ultrasonic sensors
    "J_US_LF": (0, 4, 0),
    "J_ENC_L": (0, 14, 0),
    "J_US_LR": (0, 26, 0),
    "J_US_RF": (43, 4, 180),
    "J_ENC_R": (43, 14, 180),
    "J_US_RR": (43, 26, 180),
    # rear edge (pins run toward the front): power in, buck module, expansion
    "J_PWR": (2, 35, 270),
    "J_BUCK": (6, 35, 270),
    "J_UART": (28, 35, 270),
    "J_I2C": (35, 35, 270),
    # interior
    "C_BULK": (4, 30, 0),
    "R_DIV_H": (8, 31, 0),
    "R_DIV_L": (12, 29, 180),
    "C_DIV": (13, 26, 0),
    "D_5V": (11, 33, 0),
    "R_ECHO_LF": (5, 7, 0),
    "R_ECHO_LR": (5, 29, 0),
    "R_ECHO_RF": (38, 7, 180),
    "R_ECHO_RR": (38, 29, 180),
    "R_USB": (28, 30, 0),
    "J_SPARE": (13, 13, 90),
}

# Hole step between consecutive pin numbers.
_PIN_STEP = {"J_PWR": 2, "J_VM_L": 2, "J_VM_R": 2, "J_VM_G": 2,
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
