"""Generate perfboard.md (placement, wiring table, checks) from netlist.py."""

from __future__ import annotations

from pathlib import Path

import colorsys
import hashlib

from netlist import BLUEPILL_ROW_A, BLUEPILL_ROW_B, BOARD_HOLES, PARTS, PLACEMENT, nets, pin_count, pin_hole
from route import BARE, WIRE, Routing, route



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
             f"基板は {BOARD_HOLES[0]} × {BOARD_HOLES[1]} 穴(2.54mm 格子、約 {BOARD_HOLES[0] * 2.54:.0f} × {BOARD_HOLES[1] * 2.54:.0f}mm)。部品面から見て、"
             "列 0 がロボットの左端、行 0 が前端。回転はピン番号が進む向き(0: 列+、90: 行+、180: 列−、270: 行−)。", "",
             "| ref | 値 | ピン1の穴 | 回転 |", "|---|---|---|---|"]
    by_ref = {p.ref: p for p in PARTS}
    for ref, (c, r, rot) in sorted(PLACEMENT.items()):
        lines.append(f"| {ref} | {by_ref[ref].value} | ({c},{r}) | {rot} |")
    return lines + [""]


def _wiring_section() -> list[str]:
    lines = ["## 結線表", "",
             "ネットごとに、近い順につなぐ。", "",
             "| ✓ | ネット | 始点 | 終点 |", "|---|---|---|---|"]
    for net, a, b in wire_list():
        lines.append(f"| ☐ | {net} | {a[0]}.{a[1]} @ {_hole(pin_hole(*a))} | "
                     f"{b[0]}.{b[1]} @ {_hole(pin_hole(*b))} |")
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


def _cable_section() -> list[str]:
    """Connector pin order as soldered on the board: crimp the cable housings to match."""
    lines = ["## コネクタのピン順(ケーブル製作用)", "",
             "ピン順は相手のモジュールの端子と同じなので、ケーブルはまっすぐ(1番どうし)つなぐ。GND のピンは基板の外周"
             "(GND の母線)側にある。図の□はピン1。空欄は基板側で何もつながないピン(ドライバの VM はバッテリーから直接)。", "",
             "| コネクタ | 1 | 2 | 3 | 4 |", "|---|---|---|---|---|"]
    for p in sorted(PARTS, key=lambda p: p.ref):
        if p.ref.startswith(("J_ENC", "J_US", "J_DRV", "J_PWR", "J_BUCK", "J_UART")):
            cells = [p.pins.get(str(i), "未接続" if i <= pin_count(p) else "") for i in range(1, 5)]
            lines.append(f"| {p.ref}({p.value}) | " + " | ".join(cells) + " |")
    return lines + [""]


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


def _segments(holes: tuple[tuple[int, int], ...]) -> list[tuple[int, int]]:
    """Collapse a hole path into its corner points."""
    pts = [holes[0]]
    for prev, cur, nxt in zip(holes, holes[1:], holes[2:]):
        if (cur[0] - prev[0], cur[1] - prev[1]) != (nxt[0] - cur[0], nxt[1] - cur[1]):
            pts.append(cur)
    pts.append(holes[-1])
    return pts


def _routing_section(r: Routing) -> list[str]:
    thin = list(r.jumpers)
    bare = sorted((t for t in r.traces if t.layer == BARE), key=lambda t: (t.net, t.holes))
    wire = sorted((t for t in r.traces if t.layer == WIRE), key=lambda t: (t.net, t.holes))
    lines = ["## 配線経路", "",
             "配線はすべてはんだ面で行う。電源(VBAT、GND)も基板上の電流は小さい(降圧モジュールの入力と電圧測定だけ)"
             "ので、信号と同じ線で引く。作業順:(1)はんだ面の裸配線(すずめっき線を穴に沿って這わせ、通る穴ごとにはんだ付け)、"
             "(2)はんだ面の被覆線(耐熱の被覆線、たとえば AWG26〜30 の ETFE ラッピング線を、裸配線の上から穴の列に沿って"
             "這わせる。はんだ付けは両端だけで、端はピンのはんだの上か、裸配線の上に付ける。裸配線や他の被覆線とは"
             "直角に交差してよいが、途中で他のピンのはんだの上は通さない)。ポリウレタン線ははんだごての熱で被覆が溶けるので、"
             "交差に使わない。図は `images/perfboard_top.png`(部品面)、`images/perfboard_bottom.png`(はんだ面の裸配線)、"
             "`images/perfboard_wires.png`(はんだ面の被覆線)。はんだ面の図は左右反転。", "",
             f"裸配線 {len(bare)} 本、被覆線 {len(wire)} 本。", "",
             "### (1) はんだ面の裸配線", "", "| ✓ | ネット | 経路(角の穴) |", "|---|---|---|"]
    lines += [f"| ☐ | {t.net} | {' → '.join(_hole(h) for h in _segments(t.holes))} |" for t in bare]
    lines += ["", "### (2) はんだ面の被覆線", "", "| ✓ | ネット | 経路(角の穴、はんだ付けは両端だけ) |", "|---|---|---|"]
    lines += [f"| ☐ | {t.net} | {' → '.join(_hole(h) for h in _segments(t.holes))} |" for t in wire]
    if thin:
        lines += ["", "### (3) 直結の被覆線(格子に乗らなかったもの)", "", "| ✓ | ネット | 始点 | 終点 |", "|---|---|---|---|"]
        lines += [f"| ☐ | {j.net} | {_hole(j.a)} | {_hole(j.b)} |" for j in sorted(thin, key=lambda j: (j.net, j.a))]
    return lines + [""]


# --- SVG -------------------------------------------------------------------

PX = 22
MARGIN = 70


def _color(net: str) -> str:
    fixed = {"VBAT": "#d62728", "GND": "#222222", "+5V": "#ff7f0e", "+5V_BP": "#e377c2", "+3V3": "#9467bd"}
    if net in fixed:
        return fixed[net]
    h = int(hashlib.md5(net.encode()).hexdigest()[:4], 16) / 0xFFFF
    r, g, b = colorsys.hls_to_rgb(h, 0.38, 0.75)
    return f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}"


def _svg(r: Routing, view: str) -> str:
    """view: "top" (component side), "bare" or "wire" (solder side, mirrored)."""
    mirror = view != "top"
    cols, rows = BOARD_HOLES
    w, h = MARGIN * 2 + (cols - 1) * PX, MARGIN * 2 + (rows - 1) * PX + 40

    def xy(hole: tuple[int, int]) -> tuple[float, float]:
        c = cols - 1 - hole[0] if mirror else hole[0]
        return MARGIN + c * PX, MARGIN + 40 + hole[1] * PX

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" '
           'font-family="Noto Sans CJK JP, IPAGothic, DejaVu Sans, sans-serif">',
           f'<rect width="{w}" height="{h}" fill="white"/>']
    title = {"top": "部品面(部品の配置)", "bare": "はんだ面 (1) 裸配線(左右反転:実物を裏返した向き)",
             "wire": "はんだ面 (2) 被覆線(左右反転、灰色は先に付けた裸配線)"}[view] + "  上=ロボット前方"
    out.append(f'<text x="{MARGIN}" y="34" font-size="22" font-weight="bold">{title}</text>')
    x0, y0 = xy((0, 0))
    x1, y1 = xy((cols - 1, rows - 1))
    out.append(f'<rect x="{min(x0, x1) - PX / 2}" y="{y0 - PX / 2}" width="{abs(x1 - x0) + PX}" '
               f'height="{y1 - y0 + PX}" fill="#f3ead7" stroke="#8a7a5a"/>')
    for c in range(cols):
        for rr in range(rows):
            x, y = xy((c, rr))
            out.append(f'<circle cx="{x}" cy="{y}" r="2.6" fill="#b9a988"/>')
    for c in range(0, cols, 5):
        x, _ = xy((c, 0))
        out.append(f'<text x="{x}" y="{y0 - PX}" font-size="12" text-anchor="middle">{c}</text>')
    for rr in range(0, rows, 5):
        x = min(x0, x1) - PX * 1.4
        _, y = xy((0, rr))
        out.append(f'<text x="{x}" y="{y + 4}" font-size="12" text-anchor="middle">{rr}</text>')

    shown = [t for t in r.traces if (view, t.layer) in (("bare", BARE), ("wire", WIRE))]
    if view == "wire":  # the bare wiring underneath, for orientation
        for t in r.traces:
            if t.layer == BARE:
                pts = " ".join(f"{xy(p)[0]},{xy(p)[1]}" for p in t.holes)
                out.append(f'<polyline points="{pts}" fill="none" stroke="#c9c2b4" stroke-width="5" '
                           'stroke-linecap="round" stroke-linejoin="round"/>')
    if True:
        for t in shown:
            pts = " ".join(f"{xy(p)[0]},{xy(p)[1]}" for p in t.holes)
            width = 5 if view == "bare" else 3
            out.append(f'<polyline points="{pts}" fill="none" stroke="{_color(t.net)}" stroke-width="{width}" '
                       'stroke-linecap="round" stroke-linejoin="round"/>')
            if view == "wire":  # soldered ends
                for end in (t.holes[0], t.holes[-1]):
                    ex, ey = xy(end)
                    out.append(f'<circle cx="{ex}" cy="{ey}" r="5" fill="none" stroke="{_color(t.net)}" '
                               'stroke-width="2"/>')
        labelled = set()
        for t in sorted(shown, key=lambda t: -len(t.holes)):
            if t.net in labelled:
                continue
            labelled.add(t.net)
            mid = t.holes[len(t.holes) // 2]
            x, y = xy(mid)
            width = 6.2 * len(t.net) + 4
            out.append(f'<rect x="{x + 2}" y="{y - 16}" width="{width}" height="13" fill="white" '
                       'fill-opacity="0.85" rx="2"/>')
            out.append(f'<text x="{x + 4}" y="{y - 6}" font-size="10" font-weight="bold" '
                       f'fill="{_color(t.net)}">{t.net}</text>')
    # parts
    for p in PARTS:
        holes = [pin_hole(p.ref, str(i)) for i in range(1, pin_count(p) + 1)]
        xs = [xy(hh)[0] for hh in holes]
        ys = [xy(hh)[1] for hh in holes]
        if not mirror:
            out.append(f'<rect x="{min(xs) - PX * 0.45}" y="{min(ys) - PX * 0.45}" width="{max(xs) - min(xs) + PX * 0.9}" '
                       f'height="{max(ys) - min(ys) + PX * 0.9}" fill="#ffffff" fill-opacity="0.7" stroke="#335" '
                       'stroke-width="1.5" rx="3"/>')
        for i, hh in enumerate(holes):
            x, y = xy(hh)
            shape = "rect" if i == 0 else "circle"
            if shape == "rect":
                out.append(f'<rect x="{x - 5}" y="{y - 5}" width="10" height="10" fill="#335"/>')
            else:
                out.append(f'<circle cx="{x}" cy="{y}" r="4.5" fill="#335"/>')
        tx, ty = (min(xs) + max(xs)) / 2, min(ys) - PX * 0.6
        if p.ref.startswith("J_BP"):
            ty = min(ys) - PX * 0.8
        out.append(f'<text x="{tx}" y="{ty}" font-size="11" text-anchor="middle" fill="#113">{p.ref}</text>')
    if not mirror:
        for j in r.jumpers:
            (ax, ay), (bx, by) = xy(j.a), xy(j.b)
            width = 2.2
            out.append(f'<line x1="{ax}" y1="{ay}" x2="{bx}" y2="{by}" stroke="{_color(j.net)}" '
                       f'stroke-width="{width}" stroke-opacity="0.85" stroke-linecap="round"/>')
    # Bluepill pin names
    for ref, names in (("J_BP_A", BLUEPILL_ROW_A), ("J_BP_B", BLUEPILL_ROW_B)):
        for i, name in enumerate(names):
            x, y = xy(pin_hole(ref, str(i + 1)))
            side = 1 if (ref == "J_BP_A") != mirror else -1
            anchor = "start" if side > 0 else "end"
            out.append(f'<text x="{x + side * 9}" y="{y + 4}" font-size="10" text-anchor="{anchor}" '
                       f'fill="#a33">{name}</text>')
    out.append("</svg>")
    return "\n".join(out)


def generate_md(path: Path, r: Routing | None = None) -> None:
    r = r or route()
    lines = ["# ユニバーサル基板 配置・結線", "",
             "このファイルは `gen_perfboard.py` が `netlist.py` から生成する。手で編集しない。", ""]
    lines += (_orientation_section() + _cable_section() + _placement_section() + _routing_section(r)
              + _wiring_section() + _check_section())
    path.write_text("\n".join(lines))


if __name__ == "__main__":
    here = Path(__file__).parent
    routing = route()
    generate_md(here / "perfboard.md", routing)
    (here / "perfboard_top.svg").write_text(_svg(routing, "top"))
    (here / "perfboard_bottom.svg").write_text(_svg(routing, "bare"))
    (here / "perfboard_wires.svg").write_text(_svg(routing, "wire"))
