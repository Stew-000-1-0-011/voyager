"""Generate carrier.kicad_sch from netlist.py.

Every pin gets a short wire stub ending in a local net label ("label
connection" style), so connectivity comes only from net names.
"""

from __future__ import annotations

import copy
import json
import math
import uuid
from pathlib import Path

from kiutils.items.common import Effects, Font, Justify, Position, Property
from kiutils.items.schitems import (
    Connection,
    Text,
    LocalLabel,
    NoConnect,
    SchematicSymbol,
    SymbolProjectInstance,
    SymbolProjectPath,
)
from kiutils.schematic import Schematic
from kiutils.symbol import SymbolLib

from netlist import BLUEPILL_ROW_A, BLUEPILL_ROW_B, PARTS, Part

SYMBOL_DIR = Path("/usr/share/kicad/symbols")
PROJECT = "carrier"
GRID = 1.27
STUB = 2.54
COL_PITCH = 50.8
ROW_PITCH = 35.56
ORIGIN = (25.4, 25.4)
COLS = 7


def _snap(v: float) -> float:
    return round(round(v / GRID) * GRID, 4)


_UID_NS = uuid.UUID("6f1c2a52-7d0e-4c1e-9a3b-2f6c0b1d9e41")
_uid_counter = 0


def _uid() -> str:
    """Deterministic UUIDs so regenerating the schematic gives an identical file."""
    global _uid_counter
    _uid_counter += 1
    return str(uuid.uuid5(_UID_NS, f"carrier-{_uid_counter}"))


def _load_symbols(parts: list[Part]):
    libs: dict[str, SymbolLib] = {}
    out = {}
    for p in parts:
        if p.symbol in out:
            continue
        lib, name = p.symbol.split(":")
        if lib not in libs:
            libs[lib] = SymbolLib.from_file(str(SYMBOL_DIR / f"{lib}.kicad_sym"))
        sym = copy.deepcopy(next(s for s in libs[lib].symbols if s.entryName == name))
        sym.libraryNickname = lib
        out[p.symbol] = sym
    return out


def _pins(sym):
    return [pin for unit in sym.units for pin in unit.pins]


def _place(index: int, sym) -> tuple[float, float]:
    col, row = index % COLS, index // COLS
    x = ORIGIN[0] + col * COL_PITCH
    y = ORIGIN[1] + row * ROW_PITCH
    if any(pin.position.Y > 10 for pin in _pins(sym)):  # tall connectors need more room
        y += 15.24
    return _snap(x + 12.7), _snap(y)


def _text_positions(sym, sx: float, sy: float) -> tuple[Position, Position, str]:
    """Reference/value positions that do not overlap pins or labels."""
    pins = _pins(sym)
    if all(p.position.angle in (90, 270) for p in pins):  # vertical two-pin parts: text on the right
        return (Position(X=_snap(sx + 2.54), Y=_snap(sy - 1.27), angle=0),
                Position(X=_snap(sx + 2.54), Y=_snap(sy + 1.27), angle=0), "left")
    top = max(p.position.Y for p in pins)
    bottom = min(p.position.Y for p in pins)
    return (Position(X=sx, Y=_snap(sy - top - 3.81), angle=0),
            Position(X=sx, Y=_snap(sy - bottom + 3.81), angle=0), "left")


def _label_effects() -> Effects:
    return Effects(font=Font(width=1.27, height=1.27), justify=Justify(horizontally="left"))


def generate(out_dir: Path) -> Path:
    global _uid_counter
    _uid_counter = 0
    sch = Schematic.create_new()
    sch.version = 20231120
    sch.generator = "carrier_gen"
    sch.uuid = _uid()
    sch.paper.paperSize = "A3"
    symbols = _load_symbols(PARTS)
    sch.libSymbols = list(symbols.values())

    # big connectors (Bluepill rows) first so they get their own row
    order = sorted(PARTS, key=lambda p: (0 if p.ref.startswith("J_BP") else 1, p.ref))
    tall_row_used = 0
    small_index = 0
    for part in order:
        sym = symbols[part.symbol]
        if part.ref.startswith("J_BP"):
            sx, sy = _snap(ORIGIN[0] + 38.1 + tall_row_used * 101.6), _snap(ORIGIN[1] + 25.4)
            tall_row_used += 1
        else:
            sx, sy = _place(small_index + 2 * COLS, sym)
            small_index += 1

        inst = SchematicSymbol(
            libraryNickname=sym.libraryNickname,
            entryName=sym.entryName,
            position=Position(X=sx, Y=sy, angle=0),
            unit=1,
            inBom=True,
            onBoard=True,
            dnp="DNP" in part.value,
            uuid=_uid(),
        )
        ref_pos, val_pos, just = _text_positions(sym, sx, sy)
        inst.properties = [
            Property(key="Reference", value=part.ref, id=0, position=ref_pos,
                     effects=Effects(font=Font(), justify=Justify(horizontally=just))),
            Property(key="Value", value=part.value, id=1, position=val_pos,
                     effects=Effects(font=Font(), justify=Justify(horizontally=just))),
            Property(key="Footprint", value=part.footprint, id=2,
                     position=Position(X=sx, Y=sy, angle=0), effects=Effects(font=Font(), hide=True)),
            Property(key="Datasheet", value="", id=3,
                     position=Position(X=sx, Y=sy, angle=0), effects=Effects(font=Font(), hide=True)),
        ]
        inst.pins = {pin.number: _uid() for pin in _pins(sym)}
        inst.instances = [SymbolProjectInstance(
            name=PROJECT,
            paths=[SymbolProjectPath(sheetInstancePath=f"/{sch.uuid}", reference=part.ref, unit=1)],
        )]
        sch.schematicSymbols.append(inst)

        if part.ref.startswith("J_BP"):
            names = BLUEPILL_ROW_A if part.ref == "J_BP_A" else BLUEPILL_ROW_B
            for pin in _pins(sym):
                sch.texts.append(Text(text=names[int(pin.number) - 1],
                                      position=Position(X=_snap(sx + 3.81), Y=_snap(sy - pin.position.Y), angle=0),
                                      effects=Effects(font=Font(width=1.27, height=1.27),
                                                      justify=Justify(horizontally="left")),
                                      uuid=_uid()))

        for pin in _pins(sym):
            # library coordinates have Y up; schematic has Y down
            px, py = _snap(sx + pin.position.X), _snap(sy - pin.position.Y)
            ang = math.radians(pin.position.angle)
            # the pin points from its connection end toward the body; go the other way
            dx, dy = -math.cos(ang), math.sin(ang)
            ex, ey = _snap(px + dx * STUB), _snap(py + dy * STUB)
            net = part.pins.get(pin.number)
            if net is None:
                sch.noConnects.append(NoConnect(position=Position(X=px, Y=py), uuid=_uid()))
                continue
            sch.graphicalItems.append(Connection(type="wire", points=[Position(X=px, Y=py), Position(X=ex, Y=ey)],
                                                 uuid=_uid()))
            label_angle = {(-1, 0): 180, (1, 0): 0, (0, -1): 90, (0, 1): 270}[(round(dx), round(dy))]
            effects = _label_effects()
            if label_angle in (180, 270):
                effects.justify = Justify(horizontally="right")
            sch.labels.append(LocalLabel(text=net, position=Position(X=ex, Y=ey, angle=label_angle),
                                         effects=effects, uuid=_uid()))

    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{PROJECT}.kicad_sch"
    sch.to_file(str(path))
    (out_dir / f"{PROJECT}.kicad_pro").write_text(
        json.dumps({"meta": {"filename": f"{PROJECT}.kicad_pro", "version": 1}}, indent=2) + "\n"
    )
    return path


if __name__ == "__main__":
    generate(Path(__file__).parent)
