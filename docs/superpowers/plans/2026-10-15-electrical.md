# 電気系(回路図とユニバーサル基板)実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 1つの Python 定義(`electrical/netlist.py`)から KiCad 8 の回路図とユニバーサル基板の結線表を生成し、ERC エラー 0、両者の結線の一致を自動で確認できるようにする。

**Architecture:** 部品・ピン・ネットは `netlist.py` だけで定義する。`gen_schematic.py` が kiutils で `.kicad_sch` を書き出す。回路図は、各ピンに短い配線とネットラベルを付ける「ラベル接続方式」にする。`gen_perfboard.py` は同じ定義から `perfboard.md`(部品配置と結線表)を書き出す。テストでは、kicad-cli で回路図からネットリストを出力し、`netlist.py` と照合する。

**Tech Stack:** Python 3.13、uv、kiutils 1.4.8、pytest 9、KiCad 8(PPA `kicad/kicad-8.0-releases`。`kicad-cli sch erc` は KiCad 8 以降にしかない)

**Spec:** `docs/superpowers/specs/2026-10-15-carrier-robot-design.md`(5章)

**実行順:** 3つの計画のうち最初に実行する(ユニバーサル基板の製作が 10/18〜20 のため)。

## Global Constraints

- マイコンは STM32F103C8(Bluepill)。ピン割り当ては設計書 5.4 の表をそのまま使う。
- 電源:LiPo 3S。非常停止より後ろの電池電圧を VBAT と呼ぶ。5V は外付け降圧モジュール、3.3V は Bluepill のオンボードレギュレータで作る。
- DRV8871 モジュールと降圧モジュールは基板に載せず、コネクタ経由でつなぐ。
- 生成物(`.kicad_sch` 以外の ERC レポートやネットリスト)はリポジトリに入れない。生成した `.kicad_sch` はコミットする(KiCad で開いて見られるようにするため)。
- ネット名は英大文字とアンダースコアのみ(例:`L_IN1`)。電源ネットは `VBAT`、`GND`、`+5V`、`+5V_BP`、`+3V3`。

## Review Focus

1. **Bluepill の物理的なピン順の誤り。** ソケットの向きを間違えると全結線がずれる。`perfboard.md` に USB 端の向きを明記し、実物と照合する手順を Task 3 に入れる。
2. **5V 非耐性ピンへの 5V 入力。** ECHO は必ず 1kΩ を経由して PB12〜15 に入り、エンコーダの電源は +3V3 であること。Task 1 のテスト `test_5v_signals_only_on_ft_pins` で確認する。
3. **ネットの片側だけの接続(配線の付け忘れ)。** 端子が1つしかないネットを禁止する。Task 1 のテスト `test_no_single_pin_nets` で確認する。
4. **同じ MCU ピンを2つの機能に使う重複。** Task 1 のテスト `test_mcu_pins_unique` で確認する。
5. **生成した回路図と結線表の食い違い。** Task 2 のテスト `test_schematic_netlist_matches_definition` で、kicad-cli が出力したネットリストと照合する。

---

## ファイル構成

- `electrical/pyproject.toml`:uv のプロジェクト。依存は kiutils と pytest。
- `electrical/netlist.py`:部品・ピン・ネットの唯一の定義。データとチェック関数のみを置く。
- `electrical/gen_schematic.py`:`netlist.py` から `electrical/carrier.kicad_sch` と `electrical/carrier.kicad_pro` を生成する。
- `electrical/gen_perfboard.py`:`netlist.py` から `electrical/perfboard.md` を生成する。
- `electrical/scripts/erc.sh`:kicad-cli で ERC とネットリスト出力を行う。
- `electrical/tests/test_netlist.py`、`electrical/tests/test_schematic.py`、`electrical/tests/test_perfboard.py`

---

### Task 1: ネット定義とルールチェック

**Files:**
- Create: `electrical/pyproject.toml`, `electrical/netlist.py`, `electrical/tests/test_netlist.py`, `.gitignore`(`electrical/build/`、`__pycache__/`、`.venv/` を除外)

**Interfaces:**
- Produces:
  - `@dataclass(frozen=True) class Part: ref: str; value: str; symbol: str; footprint: str; pins: dict[str, str]`。`pins` は「ピン番号 → ネット名」。未接続ピンは含めない。
  - `PARTS: list[Part]`
  - `BLUEPILL_ROW_A: list[str]`、`BLUEPILL_ROW_B: list[str]`:ソケットのピン1〜20に対応する Bluepill の端子名。
  - `MCU_PIN_NET: dict[str, str]`:Bluepill 端子名(例 `"PB6"`)→ ネット名。
  - `def nets() -> dict[str, list[tuple[str, str]]]`:ネット名 → (部品番号, ピン番号) の一覧。
  - `FT_PINS: frozenset[str]`:5V 耐性ピン(PA8〜PA15、PB2〜PB4、PB6〜PB15)。

- [ ] **Step 1: 環境を準備する**

Run: `add-apt-repository -y ppa:kicad/kicad-8.0-releases && apt-get update && apt-get install -y --no-install-recommends kicad kicad-symbols && cd electrical && uv init --no-workspace --name carrier-electrical && uv add kiutils==1.4.8 && uv add --dev pytest`
Expected: `kicad-cli version` が 8.x を表示する。

- [ ] **Step 2: 失敗するテストを書く**(`electrical/tests/test_netlist.py`)

```python
from netlist import PARTS, MCU_PIN_NET, FT_PINS, nets

def test_spec_pin_assignment():
    expected = {
        "PB6": "L_IN1", "PB7": "L_IN2", "PB8": "R_IN1", "PB9": "R_IN2",
        "PA8": "G_IN1", "PA9": "G_IN2",
        "PA0": "ENC_L_A", "PA1": "ENC_L_B", "PA6": "ENC_R_A", "PA7": "ENC_R_B",
        "PB3": "ENC_G_A", "PB4": "ENC_G_B",
        "PB0": "US_TRIG_F", "PB1": "US_TRIG_R",
        "PB12": "US_ECHO_LF", "PB13": "US_ECHO_RF", "PB14": "US_ECHO_LR", "PB15": "US_ECHO_RR",
        "PA4": "VBAT_SENSE", "PA2": "UART_TX", "PA3": "UART_RX",
        "PB10": "I2C_SCL", "PB11": "I2C_SDA", "PA12": "USB_DP",
        "PA5": "SPARE_PA5", "PA10": "SPARE_PA10", "PA15": "SPARE_PA15", "PB5": "SPARE_PB5",
        "5V": "+5V_BP", "3V3": "+3V3", "G": "GND",
    }
    for pin, net in expected.items():
        assert MCU_PIN_NET[pin] == net, pin

def test_no_single_pin_nets():
    for net, conns in nets().items():
        assert len(conns) >= 2, net

def test_mcu_pins_unique():
    signal = {p: n for p, n in MCU_PIN_NET.items() if n not in ("GND", "+3V3")}
    assert len(set(signal.values())) == len(signal)

def test_5v_signals_only_on_ft_pins():
    # ECHO は 1kΩ 経由。抵抗の MCU 側ネットが FT ピンにだけつながること
    for name in ("US_ECHO_LF", "US_ECHO_RF", "US_ECHO_LR", "US_ECHO_RR"):
        mcu = [p for p, n in MCU_PIN_NET.items() if n == name]
        assert mcu and all(p in FT_PINS for p in mcu)
    for p in PARTS:
        if p.ref.startswith("J_ENC"):
            assert p.pins["1"] == "+3V3"
            assert "+5V" not in p.pins.values()

def test_divider_and_diode():
    by_ref = {p.ref: p for p in PARTS}
    assert set(by_ref["R_DIV_H"].pins.values()) == {"VBAT", "VBAT_SENSE"}
    assert set(by_ref["R_DIV_L"].pins.values()) == {"VBAT_SENSE", "GND"}
    assert by_ref["D_5V"].pins == {"1": "+5V_BP", "2": "+5V"}  # 1=K, 2=A
```

- [ ] **Step 3: テストを実行して失敗を確認する**

Run: `cd electrical && uv run pytest tests/test_netlist.py -v`
Expected: FAIL(`ModuleNotFoundError: netlist`)

- [ ] **Step 4: `electrical/netlist.py` を実装する**

部品とピン(シンボルは KiCad 8 の標準ライブラリ、フットプリントは回路図の情報としてのみ使う):

| ref | value | symbol | ピン → ネット |
|---|---|---|---|
| J_BP_A | Bluepill row A | Connector_Generic:Conn_01x20 | `BLUEPILL_ROW_A` の順(下記) |
| J_BP_B | Bluepill row B | Connector_Generic:Conn_01x20 | `BLUEPILL_ROW_B` の順(下記) |
| J_PWR | VBAT in | Connector_Generic:Conn_01x02 | 1 VBAT, 2 GND |
| J_VM_L, J_VM_R, J_VM_G | Driver VM | Conn_01x02 | 1 VBAT, 2 GND |
| J_DRV_L / J_DRV_R / J_DRV_G | Driver IN | Conn_01x03 | 1 x_IN1, 2 x_IN2, 3 GND(x は L / R / G) |
| J_ENC_L / J_ENC_R / J_ENC_G | Encoder | Conn_01x04 | 1 +3V3, 2 GND, 3 ENC_x_A, 4 ENC_x_B |
| J_US_LF / RF / LR / RR | HC-SR04 | Conn_01x04 | 1 +5V, 2 US_TRIG_F(LF, RF)または US_TRIG_R(LR, RR), 3 US_ECHO_xx_RAW, 4 GND |
| R_ECHO_LF / RF / LR / RR | 1k | Device:R | 1 US_ECHO_xx_RAW, 2 US_ECHO_xx |
| J_BUCK | Buck module | Conn_01x04 | 1 VBAT, 2 GND, 3 +5V, 4 GND |
| D_5V | 1N5819 | Device:D_Schottky | 1(K) +5V_BP, 2(A) +5V |
| R_DIV_H | 47k | Device:R | 1 VBAT, 2 VBAT_SENSE |
| R_DIV_L | 10k | Device:R | 1 VBAT_SENSE, 2 GND |
| C_DIV | 100n | Device:C | 1 VBAT_SENSE, 2 GND |
| C_BULK | 470u 25V | Device:C_Polarized | 1 VBAT, 2 GND |
| R_USB | 1k8 DNP | Device:R | 1 USB_DP, 2 +3V3 |
| J_UART | UART | Conn_01x04 | 1 +3V3, 2 UART_TX, 3 UART_RX, 4 GND |
| J_I2C | I2C | Conn_01x04 | 1 +3V3, 2 I2C_SCL, 3 I2C_SDA, 4 GND |
| J_SPARE | Spare GPIO | Conn_01x06 | 1 SPARE_PA5, 2 SPARE_PA10, 3 SPARE_PA15, 4 SPARE_PB5, 5 +3V3, 6 GND |

```python
# ソケットのピン1→20。Bluepill の一般的なピン配置(ピン1は USB コネクタと反対側の端)
BLUEPILL_ROW_A = ["PB12","PB13","PB14","PB15","PA8","PA9","PA10","PA11","PA12","PA15",
                  "PB3","PB4","PB5","PB6","PB7","PB8","PB9","5V","G","3V3"]
BLUEPILL_ROW_B = ["VB","PC13","PC14","PC15","PA0","PA1","PA2","PA3","PA4","PA5",
                  "PA6","PA7","PB0","PB1","PB10","PB11","R","3V3","G","G"]
```

ソケットのピンは `MCU_PIN_NET` に従ってネットに接続する。`MCU_PIN_NET` にない端子(VB、PC13〜15、R、PA11)は未接続とする。

- [ ] **Step 5: テストを実行して成功を確認する**

Run: `cd electrical && uv run pytest tests/test_netlist.py -v`
Expected: 5 passed

- [ ] **Step 6: コミットする**

```bash
git add .gitignore electrical/pyproject.toml electrical/uv.lock electrical/netlist.py electrical/tests/test_netlist.py
git commit -m "feat(electrical): single-source netlist with rule checks"
```

---

### Task 2: 回路図の生成と ERC

**Files:**
- Create: `electrical/gen_schematic.py`, `electrical/scripts/erc.sh`, `electrical/tests/test_schematic.py`
- Generated (commit): `electrical/carrier.kicad_sch`, `electrical/carrier.kicad_pro`

**Interfaces:**
- Consumes: `PARTS`, `nets()`(Task 1)
- Produces: `def generate(out_dir: Path) -> Path`(`.kicad_sch` のパスを返す)。`erc.sh` は終了コードで成否を返し、`electrical/build/carrier.net`(KiCad XML ネットリスト)と `electrical/build/erc.rpt` を出力する。

- [ ] **Step 1: 失敗するテストを書く**(`electrical/tests/test_schematic.py`)

```python
import subprocess, xml.etree.ElementTree as ET
from pathlib import Path
from netlist import nets

ROOT = Path(__file__).resolve().parents[1]

def run_erc():
    return subprocess.run(["bash", str(ROOT / "scripts/erc.sh")], capture_output=True, text=True)

def test_erc_has_no_errors():
    r = run_erc()
    assert r.returncode == 0, r.stdout + r.stderr

def test_schematic_netlist_matches_definition():
    run_erc()
    tree = ET.parse(ROOT / "build/carrier.net")
    got = {}
    for net in tree.iter("net"):
        name = net.get("name").lstrip("/")
        nodes = sorted((n.get("ref"), n.get("pin")) for n in net.iter("node"))
        if len(nodes) > 1 or not name.startswith("unconnected"):
            got[name] = nodes
    want = {k: sorted(v) for k, v in nets().items()}
    assert got == want
```

- [ ] **Step 2: テストを実行して失敗を確認する**

Run: `cd electrical && uv run pytest tests/test_schematic.py -v`
Expected: FAIL(`erc.sh` が存在しない)

- [ ] **Step 3: `gen_schematic.py` を実装する**

- `/usr/share/kicad/symbols/<lib>.kicad_sym` から kiutils の `SymbolLib` で使用するシンボルを読み込み、回路図の `libSymbols` に埋め込む。
- 部品は A4 用紙上に 50.8mm 間隔の格子で並べる。座標は 1.27mm 格子に合わせる。
- 各ピンの先端から外向きに 2.54mm の配線を引き、その端にネット名の `LocalLabel` を置く(ラベル接続方式)。ピン先端の座標は、シンボルのピン定義(位置、長さ、角度)から求める。
- 未接続ピン(Bluepill の VB、PC13〜15、R、PA11)には `NoConnect` を置く。
- R_USB は `dnp` 属性を付け、値に「DNP」を含める。
- `carrier.kicad_pro` は最小限の JSON(`{"meta": {"filename": "carrier.kicad_pro", "version": 1}}`)とする。
- `if __name__ == "__main__": generate(Path(__file__).parent)`

- [ ] **Step 4: `scripts/erc.sh` を実装する**

```bash
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build
uv run python gen_schematic.py
kicad-cli sch export netlist --format kicadxml -o build/carrier.net carrier.kicad_sch
kicad-cli sch erc --severity-error --exit-code-violations -o build/erc.rpt carrier.kicad_sch
```

- [ ] **Step 5: テストを実行して成功を確認する**

Run: `cd electrical && uv run pytest -v`
Expected: 7 passed。ERC で「入力ピンが駆動されていない」というエラーが出た場合は、汎用コネクタ(すべて passive)とラベルだけで構成しているかを確認する。電源シンボルは使わない。

- [ ] **Step 6: KiCad で開けることを確認する**

Run: `kicad-cli sch export pdf -o build/carrier.pdf electrical/carrier.kicad_sch`
Expected: PDF が生成される。PDF を開き、ラベルが重なっていないことを目視で確認する。

- [ ] **Step 7: コミットする**

```bash
git add electrical/gen_schematic.py electrical/scripts/erc.sh electrical/tests/test_schematic.py electrical/carrier.kicad_sch electrical/carrier.kicad_pro
git commit -m "feat(electrical): generate KiCad schematic and run ERC"
```

---

### Task 3: ユニバーサル基板の配置と結線表

**Files:**
- Modify: `electrical/netlist.py`(部品ごとの配置座標を追加)
- Create: `electrical/gen_perfboard.py`, `electrical/tests/test_perfboard.py`
- Generated (commit): `electrical/perfboard.md`

**Interfaces:**
- Consumes: `PARTS`, `nets()`
- Produces: `PLACEMENT: dict[str, tuple[int, int, int]]`(ref → (列, 行, 回転角 0/90/180/270)。2.54mm 格子、穴番号は 0 始まり)。`BOARD_HOLES = (40, 32)`(約 100 × 80mm)。`def pin_hole(ref: str, pin: str) -> tuple[int, int]`。`def generate_md(path: Path) -> None`。

- [ ] **Step 1: 失敗するテストを書く**(`electrical/tests/test_perfboard.py`)

```python
from netlist import PARTS, PLACEMENT, BOARD_HOLES, pin_hole

def test_every_part_placed_on_board():
    for p in PARTS:
        for pin in p.pins:
            c, r = pin_hole(p.ref, pin)
            assert 0 <= c < BOARD_HOLES[0] and 0 <= r < BOARD_HOLES[1], (p.ref, pin)

def test_no_two_pins_share_a_hole():
    holes = [pin_hole(p.ref, pin) for p in PARTS for pin in p.pins]
    assert len(holes) == len(set(holes))

def test_bluepill_rows_are_15_24mm_apart():
    a1, b1 = pin_hole("J_BP_A", "1"), pin_hole("J_BP_B", "1")
    assert abs(a1[0] - b1[0]) + abs(a1[1] - b1[1]) == 6  # 6 穴 = 15.24mm

def test_cable_connectors_on_board_edge():
    for p in PARTS:
        if p.ref.startswith(("J_ENC", "J_US", "J_DRV", "J_VM", "J_PWR", "J_BUCK")):
            holes = [pin_hole(p.ref, pin) for pin in p.pins]
            assert any(c in (0, BOARD_HOLES[0]-1) or r in (0, BOARD_HOLES[1]-1) for c, r in holes), p.ref
```

`pin_hole` での穴間隔は次のとおり。2.5mm ピッチの XH は 2.54mm 格子の連続穴に挿す(脚を少し曲げる)。5.08mm のネジ端子は穴2つおき、抵抗とダイオード(1N5819、DO-41)はリード間 10.16mm で穴4つ分、470µF は 3.5mm ピッチのため隣接2穴(脚を曲げる)とする。

- [ ] **Step 2: テストを実行して失敗を確認する**

Run: `cd electrical && uv run pytest tests/test_perfboard.py -v`
Expected: FAIL(`ImportError: PLACEMENT`)

- [ ] **Step 3: `PLACEMENT` と `pin_hole` を実装する**

配置の条件:
- Bluepill は中央に置く。USB コネクタ側を基板の後方の辺(USB ケーブルを後ろから出すため)に向ける。
- ケーブル用コネクタは基板の辺に置く。左側の辺にはエンコーダ L、超音波 LF と LR、右側の辺には同じく R 側、前方の辺には J_DRV と J_VM、後方の辺には J_PWR と J_BUCK を置く。
- 分圧抵抗、ダイオード、470µF は J_PWR の近くにまとめる。

- [ ] **Step 4: `gen_perfboard.py` を実装する**

`perfboard.md` には次を書く。
1. 部品配置表(ref、値、ピン1の穴座標、回転)。
2. 結線表:ネットごとに、つなぐ穴座標を最短順(最近傍順)に並べる。各行は「ネット名、始点(ref.pin @ 列,行)、終点、推奨線材」の形にする。VBAT と GND は「太線」、それ以外は「細線」とする。
3. Bluepill の向きの確認手順:「ソケットの J_BP_A ピン1に挿さる端子が PB12、J_BP_B ピン1が VB であることを、実物のシルク印刷で確認する。違う場合は Bluepill を 180° 回して挿す前に `BLUEPILL_ROW_A/B` を修正して再生成する」。
4. 導通確認のチェックリスト:結線表の各行にチェック欄を付ける。VBAT と GND、+5V と GND、+3V3 と GND が短絡していないことも確認項目に入れる。

- [ ] **Step 5: テストを実行して成功を確認する**

Run: `cd electrical && uv run pytest -v && uv run python gen_perfboard.py`
Expected: 11 passed。`perfboard.md` が生成される。

- [ ] **Step 6: コミットする**

```bash
git add electrical/netlist.py electrical/gen_perfboard.py electrical/tests/test_perfboard.py electrical/perfboard.md
git commit -m "feat(electrical): perfboard placement and wiring table"
```
