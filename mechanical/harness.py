"""Off-board wiring: where every cable runs, how long to cut it -> ../docs/harness.md, images/harness.png.

Routes are polylines in the top view (robot frame: +X forward, +Y left, mm). Cables run along the
extrusions and are tied to them; inside the electronics bay they run loose above the tray.
Board connector positions come from electrical/netlist.py, so they follow the perfboard layout.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

from assembly import (BAY_X, ELECTRONICS_Y1, ESTOP, GEARBOX_FACE_Y, MAIN_SWITCH, PINION_X, POWER_SPLICE, SONAR_X,
                      WHEEL_Y)
from params import (BATTERY_Y, BOARD, EXTRUSION, FRAME_D, FRAME_W, MID_CROSS_X, MOTOR_LEN, TRAY_Y, WHEEL_D,
                    WHEEL_W, WHEEL_X)
from parts.chassis import BOARD_X, BOARD_Y, CROSS_X, battery_box_size

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "electrical"))
from netlist import BOARD_HOLES, PARTS, pin_hole  # noqa: E402

E = EXTRUSION
PITCH = 2.54
SLACK = 60.0  # per cable: both ends plus strain relief

Pt = tuple[float, float]


def board_xy(ref: str, pin: str) -> Pt:
    """Robot-frame position of a perfboard pin (board front = +X, column 0 = robot left)."""
    c, r = pin_hole(ref, pin)
    cols, rows = BOARD_HOLES
    return (BOARD_X + ((rows - 1) / 2 - r) * PITCH, BOARD_Y + ((cols - 1) / 2 - c) * PITCH)


def conn_xy(ref: str) -> Pt:
    p = next(q for q in PARTS if q.ref == ref)
    pts = [board_xy(ref, k) for k in p.pins]
    return (sum(x for x, _ in pts) / len(pts), sum(y for _, y in pts) / len(pts))


# --- where the modules sit (measure on the real parts and adjust) ---------------------------------
FRONT_ROW_X = (BOARD_X + BOARD[1] / 2 + BAY_X[1]) / 2       # strip of tray in front of the perfboard
DRV = {"L": (FRONT_ROW_X, 26.0), "G": (FRONT_ROW_X, 0.0), "R": (FRONT_ROW_X, -26.0)}
BUCK = (FRONT_ROW_X, -54.0)
SWITCH = (BAY_X[0] + 3 + MAIN_SWITCH[0] / 2, TRAY_Y[1] - 2 - MAIN_SWITCH[1] / 2)
SPLICE = (BAY_X[0] + 35 + POWER_SPLICE[0] / 2, TRAY_Y[1] - 2 - POWER_SPLICE[1] / 2)
ESTOP_C = (-FRAME_D / 2 + ESTOP[0] / 2, FRAME_W / 2 - ESTOP[1] / 2)
BATT_C = ((BAY_X[0] + BAY_X[1]) / 2, BATTERY_Y)
BATT_LEAD = (BATT_C[0] - battery_box_size()[0] / 2, BATTERY_Y)  # leads leave through the rear wall
MOTOR_END = {"L": (WHEEL_X, GEARBOX_FACE_Y - MOTOR_LEN), "R": (WHEEL_X, -(GEARBOX_FACE_Y - MOTOR_LEN))}
GRIP_MOTOR = (PINION_X, 0.0)
SONAR = {"LF": (SONAR_X, FRAME_W / 2), "LR": (-SONAR_X, FRAME_W / 2),
         "RF": (SONAR_X, -FRAME_W / 2), "RR": (-SONAR_X, -FRAME_W / 2)}
REAR_OUT = -FRAME_D / 2 - 4          # along the outer face of the rear member
FRONT_OF_MID = MID_CROSS_X + E / 2 + 4
SIDE_L, SIDE_R = FRAME_W / 2 - E / 2, -(FRAME_W / 2 - E / 2)  # on top of the side members
BAY_FRONT = BAY_X[1] - 4
LEFT_LANE = BOARD_Y + BOARD[0] / 2 + 4       # strip left of the perfboard
DRV_LANE = FRONT_ROW_X + 16                  # in front of the drivers


@dataclass
class Cable:
    name: str
    what: str
    wire: str
    ends: str
    route: list[Pt]
    note: str = ""
    fixed: str = ""  # length given as text instead of measured along the route

    @property
    def length_text(self) -> str:
        return self.fixed or f"{self.length:.0f}mm"

    @property
    def length(self) -> float:
        run = sum(math.dist(a, b) for a, b in zip(self.route, self.route[1:]))
        return math.ceil((run + SLACK) / 50.0) * 50.0


def cables() -> list[Cable]:
    out: list[Cable] = []
    # power: battery -> (fuse) -> main switch -> e-stop -> splice; minus straight to the splice
    out.append(Cable("P1", "電池 → メインスイッチ(+)/ 電源分岐(−)", "AWG16 赤黒", "XT60 オス ↔ 圧着端子",
                     [BATT_LEAD, (REAR_OUT, BATTERY_Y), (REAR_OUT, SWITCH[1]), SWITCH],
                     "平形ヒューズ 10A を + 側の途中に入れる。− は分岐まで同じ経路で並走させる"))
    out.append(Cable("P2", "メインスイッチ → 非常停止", "AWG16", "圧着端子",
                     [SWITCH, (SWITCH[0], ESTOP_C[1]), ESTOP_C]))
    out.append(Cable("P3", "非常停止 → 電源分岐(+)", "AWG16", "圧着端子 ↔ レバー式コネクタ",
                     [ESTOP_C, (SPLICE[0], ESTOP_C[1]), SPLICE]))
    for m in ("L", "G", "R"):
        out.append(Cable(f"P{4 + 'LGR'.index(m)}", f"電源分岐 → DRV_{m} の電源端子(VM, GND)", "AWG18 赤黒",
                         "レバー式コネクタ ↔ ネジ端子",
                         [SPLICE, (SPLICE[0], LEFT_LANE), (DRV_LANE, LEFT_LANE), (DRV_LANE, DRV[m][1]), DRV[m]],
                         "基板の左脇から前へ回し、ドライバの前を通す"))
    out.append(Cable("P7", "電源分岐 → 基板 J_PWR", "AWG20 赤黒", "レバー式コネクタ ↔ ネジ端子",
                     [SPLICE, (SPLICE[0], LEFT_LANE), (conn_xy("J_PWR")[0], LEFT_LANE), conn_xy("J_PWR")]))
    out.append(Cable("P8", "基板 J_BUCK ↔ 降圧モジュール", "AWG22 3本", "XH 3P ↔ はんだ付け",
                     [conn_xy("J_BUCK"), (conn_xy("J_BUCK")[0], LEFT_LANE), (DRV_LANE, LEFT_LANE),
                      (DRV_LANE, BUCK[1]), BUCK],
                     "基板の左脇から前へ回す。基板を詰め直すときに J_BUCK を右前へ寄せれば短くなる"))
    for m in ("L", "G", "R"):
        ref = f"J_DRV_{m}"
        out.append(Cable(f"D{'LGR'.index(m) + 1}", f"基板 {ref} → DRV_{m} の信号ピン", "AWG26 4芯",
                         "XH 4P ↔ ピンヘッダ用ソケット 4P(両端同じ順:GND, VM, IN1, IN2)",
                         [conn_xy(ref), (conn_xy(ref)[0] + 8, conn_xy(ref)[1]), (DRV[m][0], DRV[m][1] - 8),
                          DRV[m]], "VM の線は基板側で何もつながないので、抜いておいてもよい"))
    # motors: the JGA25 6P lead carries M1/M2 and the encoder; split it near the bay
    for m, enc in (("L", "J_ENC_L"), ("R", "J_ENC_R")):
        end = MOTOR_END[m]
        split = (FRONT_OF_MID, end[1])
        out.append(Cable(f"M{'LR'.index(m) + 1}", f"{'左' if m == 'L' else '右'}駆動モーター → DRV_{m} 出力",
                         "モーター付属 6P の M1, M2", "PH 6P ↔ ネジ端子",
                         [end, split, (split[0], DRV[m][1]), DRV[m]],
                         "6P ケーブルを中間クロス材の前で M1/M2 とエンコーダ4本に分ける"))
        out.append(Cable(f"E{'LR'.index(m) + 1}", f"{'左' if m == 'L' else '右'}エンコーダ → 基板 {enc}",
                         "モーター付属 6P の GND, VCC, A, B", "PH 6P ↔ XH 4P",
                         [end, split, (BAY_FRONT, split[1]), (BAY_FRONT, conn_xy(enc)[1]), conn_xy(enc)]))
    out.append(Cable("M3", "把持モーター → DRV_G 出力", "モーター付属 6P の M1, M2(延長)", "PH 6P ↔ ネジ端子",
                     [GRIP_MOTOR, (CROSS_X, 0.0), (FRONT_OF_MID, 0.0), DRV["G"]],
                     "モーター下端から中心線に沿って後ろへ。把持の上下動はないので固定でよい"))
    out.append(Cable("E3", "把持エンコーダ → 基板 J_ENC_G", "モーター付属 6P の GND, VCC, A, B(延長)",
                     "PH 6P ↔ XH 4P",
                     [GRIP_MOTOR, (CROSS_X, 0.0), (FRONT_OF_MID, 0.0), (BAY_FRONT, 0.0),
                      (BAY_FRONT, conn_xy("J_ENC_G")[1]), conn_xy("J_ENC_G")]))
    # ultrasonic sensors: along the side members, into the bay from the front or the rear
    for pos in ("LF", "RF"):
        s = SONAR[pos]
        side = SIDE_L if pos[0] == "L" else SIDE_R
        ref = f"J_US_{pos}"
        edge = ELECTRONICS_Y1 if pos[0] == "L" else TRAY_Y[0] + 2
        out.append(Cable(f"S{('LF', 'RF', 'LR', 'RR').index(pos) + 1}", f"超音波 {pos} → 基板 {ref}", "AWG26 4芯",
                         "XH 4P ↔ ピンヘッダ用ソケット 4P(両端同じ順:Vcc, Trig, Echo, Gnd)",
                         [s, (s[0], side), (FRONT_OF_MID, side), (FRONT_OF_MID, edge), (conn_xy(ref)[0], edge),
                          conn_xy(ref)]))
    lr = SONAR["LR"]
    out.append(Cable("S3", "超音波 LR → 基板 J_US_LR", "AWG26 4芯", "XH 4P ↔ ピンヘッダ用ソケット 4P(両端同じ順:Vcc, Trig, Echo, Gnd)",
                     [lr, (lr[0], SIDE_L), (ESTOP_C[0] + ESTOP[0] / 2 + 5, SIDE_L),
                      (ESTOP_C[0] + ESTOP[0] / 2 + 5, ELECTRONICS_Y1 + 2), (conn_xy("J_US_LR")[0], ELECTRONICS_Y1 + 2),
                      conn_xy("J_US_LR")], "非常停止の前を回る"))
    rr = SONAR["RR"]
    out.append(Cable("S4", "超音波 RR → 基板 J_US_RR", "AWG26 4芯", "XH 4P ↔ ピンヘッダ用ソケット 4P(両端同じ順:Vcc, Trig, Echo, Gnd)",
                     [rr, (rr[0], SIDE_R), (REAR_OUT, SIDE_R), (REAR_OUT, conn_xy("J_US_RR")[1]),
                      conn_xy("J_US_RR")], "電池箱の後ろ、後ろフレームの外面を通る"))
    usb = board_xy("J_BP_A", "20")
    usb = (usb[0] - 3 * PITCH, (board_xy("J_BP_A", "20")[1] + board_xy("J_BP_B", "20")[1]) / 2)
    out.append(Cable("U1", "Bluepill USB → PC", "USB ケーブル 3〜5m", "Micro-B ↔ A",
                     [usb, (REAR_OUT - 30, usb[1])],
                     "後ろへ出し、後ろフレームに結束して引っ張りを逃がす。USB アイソレータは PC 側に置く",
                     fixed="3〜5m"))
    return out


# --- drawing --------------------------------------------------------------------------------------

COLORS = {"P": "#d62728", "M": "#ff7f0e", "D": "#8c564b", "E": "#2ca02c", "S": "#1f77b4", "U": "#7f3fbf"}
SCALE = 2.4
MARGIN = 60


def _svg(cs: list[Cable]) -> str:
    x_min, x_max = -FRAME_D / 2 - 50, FRAME_D / 2 + 30
    y_min, y_max = -FRAME_W / 2 - 30, FRAME_W / 2 + 30
    w = (y_max - y_min) * SCALE + 2 * MARGIN + 420
    h = (x_max - x_min) * SCALE + 2 * MARGIN + 30

    def p(pt: Pt) -> tuple[float, float]:  # top view, front up, robot left on the left
        x, y = pt
        return MARGIN + (y_max - y) * SCALE, MARGIN + 30 + (x_max - x) * SCALE

    def rect(x0, x1, y0, y1, fill, stroke="#666", label="", dash=False):
        (ax, ay), (bx, by) = p((max(x0, x1), max(y0, y1))), p((min(x0, x1), min(y0, y1)))
        d = ' stroke-dasharray="6,4"' if dash else ""
        s = (f'<rect x="{ax}" y="{ay}" width="{bx - ax}" height="{by - ay}" fill="{fill}" stroke="{stroke}"'
             f' stroke-width="1.2"{d}/>')
        if label:
            s += (f'<text x="{(ax + bx) / 2}" y="{(ay + by) / 2 + 4}" font-size="12" text-anchor="middle"'
                  f' fill="#333">{label}</text>')
        return s

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" '
           'font-family="Noto Sans CJK JP, IPAGothic, DejaVu Sans, sans-serif">',
           f'<rect width="{w}" height="{h}" fill="white"/>',
           f'<text x="{MARGIN}" y="34" font-size="22" font-weight="bold">基板外の配線(上面図、上 = 前方)</text>']
    fd, fw = FRAME_D / 2, FRAME_W / 2
    alu = "#d9d9d9"
    out += [rect(fd - E, fd, -fw, fw, alu), rect(-fd, -fd + E, -fw, fw, alu),
            rect(-fd + E, fd - E, fw - E, fw, alu), rect(-fd + E, fd - E, -fw, -fw + E, alu),
            rect(CROSS_X - E / 2, CROSS_X + E / 2, -fw + E, fw - E, alu),
            rect(MID_CROSS_X - E / 2, MID_CROSS_X + E / 2, -fw + E, fw - E, alu, label="中間クロス材(追加)")]
    for s in (1, -1):
        out.append(rect(WHEEL_X - WHEEL_D / 2, WHEEL_X + WHEEL_D / 2, s * (WHEEL_Y - WHEEL_W / 2),
                        s * (WHEEL_Y + WHEEL_W / 2), "#eee", label="車輪"))
        out.append(rect(WHEEL_X - 12.5, WHEEL_X + 12.5, s * GEARBOX_FACE_Y, s * (GEARBOX_FACE_Y - MOTOR_LEN),
                        "#f6e7d0", label="モーター"))
    out.append(rect(BAY_X[0], BAY_X[1], TRAY_Y[0], TRAY_Y[1], "#f7f3e8", label=""))
    out.append(rect(BOARD_X - BOARD[1] / 2, BOARD_X + BOARD[1] / 2, BOARD_Y - BOARD[0] / 2, BOARD_Y + BOARD[0] / 2,
                    "#fff8dc", label="基板"))
    for m, (x, y) in DRV.items():
        out.append(rect(x - 14, x + 14, y - 11, y + 11, "#e8f0ff", label=f"DRV_{m}"))
    out.append(rect(BUCK[0] - 9, BUCK[0] + 9, BUCK[1] - 11, BUCK[1] + 11, "#e8ffe8", label="降圧"))
    out.append(rect(SWITCH[0] - MAIN_SWITCH[0] / 2, SWITCH[0] + MAIN_SWITCH[0] / 2, SWITCH[1] - MAIN_SWITCH[1] / 2,
                    SWITCH[1] + MAIN_SWITCH[1] / 2, "#ffe0e0", label="SW"))
    out.append(rect(SPLICE[0] - POWER_SPLICE[0] / 2, SPLICE[0] + POWER_SPLICE[0] / 2,
                    SPLICE[1] - POWER_SPLICE[1] / 2, SPLICE[1] + POWER_SPLICE[1] / 2, "#ffe0e0", label="分岐"))
    out.append(rect(ESTOP_C[0] - ESTOP[0] / 2, ESTOP_C[0] + ESTOP[0] / 2, ESTOP_C[1] - ESTOP[1] / 2,
                    ESTOP_C[1] + ESTOP[1] / 2, "#fff3b0", label="非常停止"))
    bl, bw = battery_box_size()
    out.append(rect(BATT_C[0] - bl / 2, BATT_C[0] + bl / 2, BATT_C[1] - bw / 2, BATT_C[1] + bw / 2, "#e0e0ff",
                    label="電池箱"))
    out.append(rect(GRIP_MOTOR[0] - 12.5, GRIP_MOTOR[0] + 12.5, -12.5, 12.5, "#f6e7d0", label="把持M"))
    for name, (x, y) in SONAR.items():
        out.append(rect(x - 22, x + 22, y, y + (8 if y > 0 else -8), "#cde", label=""))
        tx, ty = p((x, y + (16 if y > 0 else -16)))
        out.append(f'<text x="{tx}" y="{ty + 4}" font-size="12" text-anchor="middle">{name}</text>')
    for ref in ("J_PWR", "J_BUCK", "J_ENC_L", "J_ENC_R", "J_ENC_G", "J_US_LF", "J_US_LR", "J_US_RF", "J_US_RR",
                "J_DRV_L", "J_DRV_G", "J_DRV_R"):
        cx, cy = p(conn_xy(ref))
        out.append(f'<circle cx="{cx}" cy="{cy}" r="3.5" fill="#333"/>')
    for k, c in enumerate(cs):
        pts = " ".join(f"{p(q)[0]:.1f},{p(q)[1]:.1f}" for q in c.route)
        off = (k % 3 - 1) * 2.0
        out.append(f'<polyline points="{pts}" fill="none" stroke="{COLORS[c.name[0]]}" stroke-width="3" '
                   f'stroke-opacity="0.85" stroke-linejoin="round" transform="translate({off},{off})"/>')
        mx, my = p(c.route[len(c.route) // 2])
        out.append(f'<rect x="{mx - 13}" y="{my - 9}" width="26" height="16" rx="3" fill="white" '
                   f'stroke="{COLORS[c.name[0]]}"/>')
        out.append(f'<text x="{mx}" y="{my + 4}" font-size="11" font-weight="bold" text-anchor="middle" '
                   f'fill="{COLORS[c.name[0]]}">{c.name}</text>')
    lx = MARGIN + (y_max - y_min) * SCALE + 30
    ly = MARGIN + 60
    for key, label in (("P", "電源(AWG16〜20)"), ("M", "モーター線"), ("D", "ドライバ信号線"), ("E", "エンコーダ線"), ("S", "超音波センサー線"),
                       ("U", "USB")):
        out.append(f'<line x1="{lx}" y1="{ly}" x2="{lx + 30}" y2="{ly}" stroke="{COLORS[key]}" stroke-width="4"/>')
        out.append(f'<text x="{lx + 40}" y="{ly + 5}" font-size="14">{label}</text>')
        ly += 26
    ly += 10
    for c in cs:
        out.append(f'<text x="{lx}" y="{ly}" font-size="12"><tspan font-weight="bold" fill="{COLORS[c.name[0]]}">'
                   f'{c.name}</tspan> {c.what}  {c.length_text}</text>')
        ly += 19
    out.append("</svg>")
    return "\n".join(out)


def _md(cs: list[Cable]) -> str:
    lines = ["# 基板外の配線", "",
             "このファイルは `mechanical/harness.py` が生成する。手で編集しない。図は `mechanical/images/harness.png`。", "",
             "- ケーブルはアルミフレームの上面か外面に沿わせ、結束バンドで 50〜80mm おきに留める。"
             "電装トレイの上では基板とモジュールの間をゆるく回す。",
             "- 長さは図の経路に両端の余裕 60mm を足して 50mm 単位に切り上げた目安。実物に当てて切る。",
             "- 電源の流れ:電池 → XT60 → ヒューズ 10A → メインスイッチ → 非常停止 → 電源分岐(レバー式コネクタ)"
             " → DRV8871 ×3 の電源端子と基板の J_PWR。電池の − は電源分岐の − へ直接つなぐ。",
             "- モーターの 6P ケーブルは中間クロス材の前で、M1/M2(ドライバへ)とエンコーダ4本(基板へ)に分ける。",
             "- モジュールの置き場所(トレイ上、基板の前):"
             f" DRV_L / DRV_G / DRV_R を左から順に、降圧モジュールを右端。メインスイッチと電源分岐はトレイの左奥。", "",
             "| 記号 | 区間 | 線材 | 端末 | 長さの目安 | 備考 |", "|---|---|---|---|---|---|"]
    lines += [f"| {c.name} | {c.what} | {c.wire} | {c.ends} | {c.length_text} | {c.note} |" for c in cs]
    return "\n".join(lines) + "\n"


def write() -> None:
    import cairosvg

    cs = cables()
    (ROOT / "docs" / "harness.md").write_text(_md(cs))
    svg = Path(__file__).parent / "build" / "harness.svg"
    svg.parent.mkdir(exist_ok=True)
    svg.write_text(_svg(cs))
    cairosvg.svg2png(url=str(svg), write_to=str(Path(__file__).parent / "images" / "harness.png"),
                     output_width=2400, background_color="white")


if __name__ == "__main__":
    write()
