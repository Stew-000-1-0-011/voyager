# 機構(build123d)実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** build123d で機体の全部品と組立てをコードで作る。外形(500mm 以内)、把持のストローク、干渉の有無を pytest で確認できるようにし、3Dプリント用の STL とアルミ材の切断寸法表を出力する。

**Architecture:** 寸法はすべて `mechanical/params.py` に置き、部品のスクリプトはそこからだけ値を読む。歯形(インボリュート)の計算は純粋な関数として `mechanical/parts/involute.py` に分け、数値でテストする。組立てでは開いた状態と閉じた状態の両方を作り、テストで寸法を確かめる。

**Tech Stack:** Python 3.13、uv、build123d 0.13、pytest 9

**Spec:** `docs/superpowers/specs/2026-10-15-carrier-robot-design.md`(3.1、4章)

**実行順:** 電気系の計画の後。ファームウェアの計画とは独立していて、並行して進められる。

## Global Constraints

- 加工はアルミ材と3Dプリンタのみ。CNC 前提の部品は作らない。
- 単位は mm。座標系は、機体中心の床面を原点とし、+X を前、+Y を左、+Z を上とする。
- 外形の上限:開いた状態・閉じた状態とも 500 × 500mm 以内(高さは 800mm 以内だが、実際は約150mm)。
- 歯車:モジュール 1、圧力角 20°、ピニオン 20歯、歯幅 12mm、PETG。
- 生成物(`mechanical/build/` の STEP と STL)はリポジトリに入れない。
- 実物を測るまで確定できない寸法(モーターの取付穴など)は、`params.py` に「要実測」とコメントを付けて置く。

## Review Focus

1. **アームを閉じたときに、ラック同士やラックと駆動モーターがぶつかる。** ラックは閉じると中央側に伸びる。Task 4 のテスト `test_closed_racks_do_not_collide` で確認する。
2. **開いたときの外形が 500mm を超える。** パッドやストッパーの厚みを足し忘れやすい。Task 4 のテスト `test_open_envelope_within_500` で、全部品の合成バウンディングボックスで確認する。
3. **超音波センサーの視野を車輪やアームがふさぐ。** センサーの前方(±Y 方向)50mm の範囲に他の部品がないこと。Task 4 のテスト `test_sonar_line_of_sight_clear` で確認する。
4. **ピニオンとラックの噛み合い位置のずれ。** ピニオンのピッチ円とラックのピッチ線が接していないと、歯が噛まない、または干渉する。Task 2 のテスト `test_pinion_meshes_with_racks` で確認する。
5. **パッドの高さが箱に当たらない。** パッドが床から 30〜120mm、箱(150mm)の側面に当たる範囲にあること。Task 4 のテスト `test_pad_height_hits_box` で確認する。

---

## ファイル構成

```
mechanical/
  pyproject.toml
  params.py            全寸法
  parts/
    involute.py        歯形の計算(純粋関数)
    gears.py           pinion(), rack()
    gripper.py         slider(), arm(), grip_motor_mount(), stopper()
    chassis.py         frame_members(), drive_motor_mount(), caster_mount(), board_mount()
    sensors.py         sonar_mount()
  assembly.py          build(open: bool) -> Compound、export()
  cutlist.py           アルミ材の切断寸法表を出力
  tests/
    test_involute.py, test_gears.py, test_assembly.py
```

---

### Task 1: 寸法定義と歯形計算

**Files:**
- Create: `mechanical/pyproject.toml`, `mechanical/params.py`, `mechanical/parts/__init__.py`, `mechanical/parts/involute.py`, `mechanical/tests/test_involute.py`
- Modify: `.gitignore`(`mechanical/build/` を追加)

**Interfaces:**
- Produces:
  - `params.py` の定数(下表)
  - `def pitch_radius(teeth: int, module: float) -> float`
  - `def gear_outline(teeth: int, module: float, pressure_angle_deg: float = 20.0, points_per_flank: int = 12) -> list[tuple[float, float]]`:閉じた外形の2D座標(反時計回り)。歯先円 = r + m、歯底円 = r − 1.25m。
  - `def rack_outline(n_teeth: int, module: float, pressure_angle_deg: float = 20.0, base_height: float = 6.0) -> list[tuple[float, float]]`:ピッチ線を y = 0、歯先を +y 側とする2D外形。

`params.py` の主な値:

| 名前 | 値 | 根拠 |
|---|---|---|
| `FRAME_W`, `FRAME_D`, `EXTRUSION` | 300, 250, 20 | 足回りの枠 |
| `WHEEL_D`, `WHEEL_W` | 65, 26 | 車輪 |
| `WHEEL_X` | 30 | 車輪中心は枠中心より 30mm 前 |
| `MOTOR_DIA`, `MOTOR_LEN`, `MOTOR_SHAFT_D` | 25, 52, 4 | JGA25-370(要実測) |
| `MOTOR_HOLE_PITCH`, `MOTOR_HOLE_D` | 17, 3.2 | ギアボックス端面の取付穴(要実測) |
| `MODULE`, `PINION_TEETH`, `GEAR_FACE` | 1.0, 20, 12 | 歯車 |
| `OPEN_INNER`, `CLOSED_INNER` | 400, 130 | 把持の内幅 |
| `STROKE` | `(OPEN_INNER - CLOSED_INNER) / 2` = 135 | 片側のストローク |
| `ARM_LEN` | 170 | 枠前端からアーム先端まで |
| `PAD_Z_MIN`, `PAD_Z_MAX`, `PAD_T` | 30, 120, 5 | パッドの高さ範囲と厚み(ゴム込み) |
| `RAIL_SECTION` | 10 | レールの角材 |
| `SONAR_Z`, `SONAR_SPACING` | 45, 200 | 超音波センサーの高さと前後の間隔 |
| `BOX` | 150 | ノーツ |
| `ENVELOPE_MAX` | 500 | 外形の上限 |

- [ ] **Step 1: 環境を用意する**

Run: `cd mechanical && uv init --no-workspace --name carrier-mechanical && uv add build123d==0.13.0 && uv add --dev pytest`
Expected: `uv run python -c "import build123d"` が成功する。

- [ ] **Step 2: 失敗するテストを書く**(`tests/test_involute.py`)

```python
import math
from parts.involute import pitch_radius, gear_outline, rack_outline

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
    tips = sum(1 for i in range(len(radii)) if radii[i] > 10.9 and radii[i-1] <= 10.9)
    assert tips == 20

def test_rack_pitch_is_pi_m():
    pts = rack_outline(10, 1.0)
    # 歯先(y == 1.0)の区間の始点 x を集め、隣り合う間隔がすべて π であること
    ...
```

(`test_rack_pitch_is_pi_m` の検証内容:歯先の始点の間隔がすべて π ± 1e-6、歯先の数が 10)

(ファイル先頭で `import pytest` も行う)

- [ ] **Step 3: 失敗を確認する** — Run: `cd mechanical && uv run pytest tests/test_involute.py -v` / Expected: FAIL(モジュールがない)

- [ ] **Step 4: `params.py` と `involute.py` を実装する**

インボリュート曲線は、基礎円半径 `rb = r * cos(α)` から、パラメータ t で `(rb(cos t + t sin t), rb(sin t − t cos t))` を描く。歯先円で切り、歯底円までは半径方向の直線でつなぐ。歯厚はピッチ円上で πm/2。ラックの歯は、高さ m(歯先)と 1.25m(歯底)、側面の傾きが圧力角の台形とする。

- [ ] **Step 5: 成功を確認する** — Expected: 4 passed
- [ ] **Step 6: コミットする** — `git commit -m "feat(mechanical): parameters and involute gear/rack geometry"`

---

### Task 2: 歯車と把持部品

**Files:**
- Create: `mechanical/parts/gears.py`, `mechanical/parts/gripper.py`, `mechanical/tests/test_gears.py`

**Interfaces:**
- Consumes:`gear_outline`, `rack_outline`, `params`
- Produces:
  - `def pinion() -> Part`:20歯、歯幅 12mm。中心に直径 4mm の D カット穴(平面までの距離 3.5mm)。側面から M3 ナットの埋め込み穴(二面幅 5.5mm、厚み 2.5mm)と、D カットの平面に向かう M3 の穴。
  - `def rack(length: float) -> Part`:ラックの歯の数は `ceil(length / (π·m))`。歯幅 12mm。裏面にスライダーを固定する M3 穴 2個。
  - `RACK_LEN = STROKE + 2 * PINION_OUTER_D + 20`(ストローク全域で噛み合いを保つ長さ)
  - `def slider() -> Part`:10mm 角のレールを囲むコの字(クリアランス 0.3mm)。
  - `def arm() -> Part`:ラック外端から前方へ `ARM_LEN` 伸びる板(厚み 6mm、高さ `PAD_Z_MAX - PAD_Z_MIN + 20`)。内側面にパッド取付面。
  - `def grip_motor_mount() -> Part`:JGA25-370 を縦向き(出力軸が上)に固定し、ピニオンがラックのピッチ線に接する高さに置くブラケット。
  - `def stopper() -> Part`:レールに固定する当て板(開き側、閉じ側で共用)。

- [ ] **Step 1: 失敗するテストを書く**(`tests/test_gears.py`)

- `test_pinion_outer_diameter`:`pinion().bounding_box()` の X と Y の幅が 22.0 ± 0.1。Z の幅が 12.0。
- `test_pinion_has_d_bore`:`pinion().volume` が、同じ外形で穴なしの体積より `π·2²·12 × 0.8` 以上小さい。
- `test_rack_covers_stroke`:`rack(RACK_LEN)` の長さが `STROKE + 40` 以上。
- `test_pinion_meshes_with_racks`:組立ての `rack_pitch_lines() -> tuple[float, float]` で得た左右ラックのピッチ線の X 座標が、ピニオン中心の X ± 10.0(ピッチ半径)に一致する(ラックは Y 方向に動き、ピニオンの前後に1本ずつ噛む)。

- [ ] **Step 2〜4: 失敗を確認 → 実装 → 成功を確認** — Run: `cd mechanical && uv run pytest tests/test_gears.py -v` / Expected: 4 passed
- [ ] **Step 5: コミットする** — `git commit -m "feat(mechanical): pinion, racks and gripper parts"`

---

### Task 3: 足回りとセンサーの部品

**Files:**
- Create: `mechanical/parts/chassis.py`, `mechanical/parts/sensors.py`, `mechanical/cutlist.py`

**Interfaces:**
- Produces:
  - `def frame_members() -> list[tuple[str, float, Location]]`:20mm 角アルミ材の(名前、長さ、配置)。外周 4本(前後は `FRAME_W`、左右は `FRAME_D - 2*EXTRUSION`)と、駆動モーター用の横材1本。
  - `def drive_motor_mount() -> Part`:JGA25-370 を横向き(軸が ±Y)に固定し、アルミ材に M5 と T ナットで留めるブラケット。車輪の中心高さ = `WHEEL_D / 2`。
  - `def caster_mount() -> Part`:後部の横材の中央に付け、ボールキャスターの接地面を車輪の接地面と同じ高さにする(キャスター本体の高さは `CASTER_H` として `params.py` に追加。要実測)。
  - `def board_mount() -> Part`:ユニバーサル基板(100 × 80mm)用の4点スタンドオフ(M3)。
  - `def sonar_mount() -> Part`:HC-SR04(基板 45 × 20mm、送受信器 φ16 が中心間 26mm)を外向きに保持し、送受信器の中心を `SONAR_Z` にする。
  - `cutlist.py`:`frame_members()` から「名前、長さ、本数」の表を `mechanical/build/cutlist.md` に書く。

- [ ] **Step 1: 失敗するテストを書く**(`tests/test_assembly.py` の先頭部分)

- `test_frame_outer_size`:`frame_members()` の外形の幅と奥行きが `FRAME_W`、`FRAME_D` に一致する。
- `test_wheel_contacts_floor`:駆動モーター取付を配置したとき、車輪の最下点が Z = 0 ± 0.5。
- `test_caster_contacts_floor`:キャスターの最下点が Z = 0 ± 1.0。

- [ ] **Step 2〜4: 失敗を確認 → 実装 → 成功を確認** — Run: `cd mechanical && uv run pytest tests/test_assembly.py -v -k "frame or floor"` / Expected: 3 passed
- [ ] **Step 5: コミットする** — `git commit -m "feat(mechanical): chassis, motor/caster mounts, sonar mount, cutlist"`

---

### Task 4: 組立て・寸法の確認・出力

**Files:**
- Create: `mechanical/assembly.py`
- Modify: `mechanical/tests/test_assembly.py`

**Interfaces:**
- Consumes:Task 2〜3 の全部品
- Produces:
  - `def rack_offsets(open: bool) -> tuple[float, float]`(左右ラックの Y 方向の位置)
  - `def rack_pitch_lines() -> tuple[float, float]`(左右ラックのピッチ線の X 座標。Task 2 のテストで使う)
  - `def build(open: bool) -> dict[str, Part]`(部品名 → 配置済みの Part。名前は `"frame"`, `"wheel_l"`, `"wheel_r"`, `"motor_l"`, `"motor_r"`, `"rack_l"`, `"rack_r"`, `"arm_l"`, `"arm_r"`, `"pad_l"`, `"pad_r"`, `"pinion"`, `"sonar_lf"`, `"sonar_lr"`, `"sonar_rf"`, `"sonar_rr"`, `"board"`, `"caster"`)
  - `def export(out_dir: Path) -> None`:両状態の STEP と、3Dプリント部品ごとの STL を `mechanical/build/` に書く。

- [ ] **Step 1: 失敗するテストを書く**(`tests/test_assembly.py` に追加)

- `test_open_envelope_within_500`:`build(True)` の全部品の合成バウンディングボックスの X と Y の幅が 500 以下。
- `test_closed_envelope`:`build(False)` の Y の幅が 320 以下(スタート時に並べやすい幅)。
- `test_open_inner_width`:開いたとき、左右パッドの内面の間隔が `OPEN_INNER` ± 1。
- `test_closed_inner_width`:閉じたとき `CLOSED_INNER` ± 1。
- `test_closed_racks_do_not_collide`:閉じたとき、`rack_l`、`rack_r`、`motor_l`、`motor_r` のどの2つの組み合わせでも、共通部分の体積が 0。
- `test_arms_clear_wheels`:開いた状態と閉じた状態の両方で、`arm_*`、`pad_*` と `wheel_*` の共通部分の体積が 0。
- `test_sonar_line_of_sight_clear`:各センサーの前面から外向きに 50mm 伸ばした φ20 の円柱と、他の部品の共通部分の体積が 0。
- `test_pad_height_hits_box`:パッドの Z 範囲が `PAD_Z_MIN`〜`PAD_Z_MAX` で、その範囲が 0〜`BOX` に含まれる。

- [ ] **Step 2: 失敗を確認する** — Run: `cd mechanical && uv run pytest -v` / Expected: 新しい 8 件が FAIL
- [ ] **Step 3: `assembly.py` を実装する**

ピニオンは枠前端の中央、レールの上に置く(軸は Z 方向)。左ラックはピニオンの前側(+X)でピッチ線を接し、閉じる向きで −Y に動く。右ラックは後ろ側(−X)で噛み、閉じる向きで +Y に動く。ラックの位置 = 開いたときの位置 −(開いた量 − 現在の開き量)/ 2 とする。

- [ ] **Step 4: 成功を確認する** — Run: `cd mechanical && uv run pytest -v` / Expected: 19 passed
- [ ] **Step 5: 出力する** — Run: `cd mechanical && uv run python assembly.py && uv run python cutlist.py && ls build/` / Expected: `assembly_open.step`、`assembly_closed.step`、部品ごとの `.stl`、`cutlist.md` がある。
- [ ] **Step 6: コミットする** — `git commit -m "feat(mechanical): assembly with envelope/interference checks and exports"`
