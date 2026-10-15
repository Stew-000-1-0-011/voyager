"""Generate perfboard.md (placement, wiring table, checks) from netlist.py."""

from __future__ import annotations

from pathlib import Path

from netlist import BLUEPILL_ROW_A, BLUEPILL_ROW_B, BOARD_HOLES, PARTS, PLACEMENT, nets, pin_hole

HEAVY = ("VBAT", "GND")


def _hole(h: tuple[int, int]) -> str:
    return f"({h[0]},{h[1]})"


def wire_list() -> list[tuple[str, tuple[str, str], tuple[str, str]]]:
    """Point-to-point wires per net, ordered as a nearest-neighbour chain."""
    out = []
    for net, conns in sorted(nets().items()):
        remaining = sorted(conns, key=lambda c: pin_hole(*c))
        chain = [remaining.pop(0)]
        while remaining:
            last = pin_hole(*chain[-1])
            nxt = min(remaining, key=lambda c: abs(pin_hole(*c)[0] - last[0]) + abs(pin_hole(*c)[1] - last[1]))
            remaining.remove(nxt)
            chain.append(nxt)
        out += [(net, a, b) for a, b in zip(chain, chain[1:])]
    return out


def _placement_section() -> list[str]:
    lines = ["## 部品配置", "",
             f"基板は {BOARD_HOLES[0]} × {BOARD_HOLES[1]} 穴(2.54mm 格子、約 100 × 80mm)。部品面から見て、"
             "列 0 がロボットの左端、行 0 が前端。回転はピン番号が進む向き(0: 列+、90: 行+、180: 列−、270: 行−)。", "",
             "| ref | 値 | ピン1の穴 | 回転 |", "|---|---|---|---|"]
    by_ref = {p.ref: p for p in PARTS}
    for ref, (c, r, rot) in sorted(PLACEMENT.items()):
        lines.append(f"| {ref} | {by_ref[ref].value} | ({c},{r}) | {rot} |")
    return lines + [""]


def _wiring_section() -> list[str]:
    lines = ["## 結線表", "",
             "ネットごとに、近い順につなぐ。太線は AWG18、細線は AWG26 またはすずめっき線。", "",
             "| ✓ | ネット | 始点 | 終点 | 線材 |", "|---|---|---|---|---|"]
    for net, a, b in wire_list():
        gauge = "太線" if net in HEAVY else "細線"
        lines.append(f"| ☐ | {net} | {a[0]}.{a[1]} @ {_hole(pin_hole(*a))} | "
                     f"{b[0]}.{b[1]} @ {_hole(pin_hole(*b))} | {gauge} |")
    return lines + [""]


def _orientation_section() -> list[str]:
    return [
        "## Bluepill の向きの確認", "",
        f"- ソケット J_BP_A のピン1({_hole(pin_hole('J_BP_A', '1'))})に挿さる端子が **{BLUEPILL_ROW_A[0]}**、"
        f"J_BP_B のピン1({_hole(pin_hole('J_BP_B', '1'))})が **{BLUEPILL_ROW_B[0]}** であること。"
        "USB コネクタは基板の後端(行の大きい側)を向く。",
        "- 実物のシルク印刷で確認する。違う場合は、ハンダ付けの前に `netlist.py` の "
        "`BLUEPILL_ROW_A` / `BLUEPILL_ROW_B` と `PLACEMENT` を直して再生成する。", "",
    ]


def _check_section() -> list[str]:
    return [
        "## 導通確認", "",
        "- 結線表の各行を、テスターの導通モードで確認して ✓ を付ける。",
        "- 次の組み合わせが **短絡していない** ことを確認する(電源を入れる前に必ず行う)。",
        "  - ☐ VBAT と GND",
        "  - ☐ +5V と GND",
        "  - ☐ +5V_BP と GND",
        "  - ☐ +3V3 と GND",
        "  - ☐ VBAT と +5V",
        "",
    ]


def generate_md(path: Path) -> None:
    lines = ["# ユニバーサル基板 配置・結線", "",
             "このファイルは `gen_perfboard.py` が `netlist.py` から生成する。手で編集しない。", ""]
    lines += _orientation_section() + _placement_section() + _wiring_section() + _check_section()
    path.write_text("\n".join(lines))


if __name__ == "__main__":
    generate_md(Path(__file__).parent / "perfboard.md")
