# ファームウェアと PC 側操縦プログラム 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** USB 有線の手動操縦で、ロボットを走らせて把持できるファームウェア(STM32F103、RTICX)と PC 側プログラムを作る。ハードウェアに依存しない処理は `robot-core` にまとめ、PC 上でテストする。

**Architecture:** `robot-core`(no_std)には、通信、駆動の配分、PWM の極性、指令タイムアウト、非常停止の保持、電池の判定、把持の状態管理、エンコーダのデコード、超音波の距離換算を入れる。`firmware` は周辺機能を設定し、RTICX のタスクから `robot-core` を呼ぶだけにする。`host` は `robot-core` の通信の型をそのまま使う。

**Tech Stack:** Rust 1.97 / edition 2024。robot-core:`corncobs 0.1`、`crc 3`。firmware:`rticx-cortex-m 0.2.1`、`stm32f1xx-hal 0.11`(feature `stm32f103`, `medium`)、`usb-device 0.3`、`usbd-serial 0.2`、`defmt 1`、`defmt-rtt 1`、`panic-probe 1`、`cortex-m 0.7`、`cortex-m-rt 0.7`。host:`gilrs 0.11`、`serialport 4`、`serde` + `toml`、`anyhow`。

**Spec:** `docs/superpowers/specs/2026-10-15-carrier-robot-design.md`(6章、7章、5.3、5.4)

**実行順:** 電気系の計画の次に実行する。Task 1〜6 はハードウェアなしで完了できる。

## Global Constraints

- 制御周期 100Hz(10ms)、PC からの指令 50Hz、テレメトリ 20Hz。
- 指令タイムアウト 200ms。タイムアウトしたら駆動はブレーキ、把持は保持を続ける。
- 電池:10.5V 未満で出力上限 50%、9.9V 未満で停止(停止は再起動まで解除しない)。非常停止の判定は 3.0V 未満。
- 前進量・旋回量・モーター指令はすべて -1000〜1000 の整数。
- PWM は 20kHz。オフ区間がブレーキになる極性(片側を High 固定、もう片側に反転した PWM)。
- フレーミングは COBS(区切り 0x00)、誤り検出は CRC-8/SMBUS(`crc::CRC_8_SMBUS`)。マルチバイトの値はリトルエンディアン。
- USB の VID/PID は 0x16C0 / 0x27DD(USB CDC 試験用の共有 ID)、製品名は `voyager-carrier`。
- `robot-core` は `#![no_std]`、ヒープ不使用、`f32` 不使用(整数演算のみ)。

## Review Focus

1. **非常停止解除の瞬間の急発進。** スティックを倒したまま解除しても、スティックが一度中立に戻るまで駆動しないのが自然。Task 3 のテスト `estop_release_requires_neutral_stick` で確認する。
2. **電池電圧の一時的な落ち込み(モーター起動時)。** 一瞬 9.9V を下回っただけで停止が保持されると、試合中に止まる。平滑化してから判定する。Task 3 のテスト `brief_sag_does_not_trip_critical` で確認する。
3. **USB の抜き差し・PC 側プログラムの再起動。** 再接続後、古い連番でも新しい指令を受け付けること。連番は表示用で、受け付けの判定には使わない。Task 2 のテスト `seq_wraparound_is_accepted` で確認する。
4. **把持のエンコーダ符号が逆の配線。** 閉じる指令でカウントが減ると、止まった判定が狂う。止まる位置で判定するのではなく、移動量の絶対値で判定し、設定 `grip_dir` で符号を反転できるようにする。Task 4 のテスト `reversed_encoder_with_dir_flag` で確認する。
5. **フレームの途中で切れた USB のデータや不正なバイト列。** デコーダが詰まらず、次の 0x00 から復帰すること。Task 2 のテスト `decoder_recovers_after_garbage` で確認する。

---

## ファイル構成

```
firmware/
  Cargo.toml                 ワークスペース(members: robot-core, host)
  robot-core/src/lib.rs      モジュールの公開のみ
  robot-core/src/protocol.rs Command / Telemetry / FrameDecoder
  robot-core/src/drive.rs    mix / SlewLimiter / BridgeOutput
  robot-core/src/safety.rs   Watchdog / EstopLatch / BatteryMonitor / adc_to_mv
  robot-core/src/gripper.rs  Gripper / GripperConfig / GripState
  robot-core/src/quad.rs     QuadDecoder(把持エンコーダのソフトデコード)
  robot-core/src/ultrasonic.rs echo_to_mm
  robot-core/src/control.rs  Controller(100Hz の1ステップを組み立てる)
  host/src/main.rs           ループと引数
  host/src/mapping.rs        PadState → Command
  host/src/link.rs           シリアルの接続・再接続・送受信
  host/src/log.rs            画面表示と CSV
  host/config.toml           ボタン割り当ての既定値
  firmware/                  別ワークスペース(thumbv7m-none-eabi)
    Cargo.toml, .cargo/config.toml, memory.x, build.rs
    src/main.rs              RTICX の app
    src/board.rs             クロック・ピン・タイマー・USB の初期化
    src/motors.rs            BridgeOutput を PWM に反映
    src/sonar.rs             トリガーと EXTI のタイムスタンプ
```

---

### Task 1: ワークスペースと通信の型

**Files:**
- Create: `firmware/Cargo.toml`, `firmware/robot-core/Cargo.toml`, `firmware/robot-core/src/lib.rs`, `firmware/robot-core/src/protocol.rs`
- Modify: `.gitignore`(`target/` を追加)

**Interfaces:**
- Produces(`robot_core::protocol`):
  - `#[repr(u8)] enum GripCmd { Keep = 0, Open = 1, Close = 2, Stop = 3 }`
  - `bitflags` 相当の定数 `FLAG_SLOW: u8 = 0x01`, `FLAG_ESTOP_RELEASE: u8 = 0x02`
  - `struct Command { seq: u8, throttle: i16, turn: i16, grip: GripCmd, flags: u8 }`
  - `#[repr(u8)] enum GripState { Unhomed = 0, Idle = 1, Opening = 2, Open = 3, Closing = 4, Holding = 5, Fault = 6 }`
  - `struct Telemetry { seq: u8, battery_mv: u16, enc_left: i32, enc_right: i32, enc_grip: i32, grip_state: GripState, box_count: u8, sonar_mm: [u16; 4], estop_latched: bool, faults: u8 }`(`sonar_mm` は左前・右前・左後・右後の順、0 は無効)
  - 故障ビット `FAULT_GRIP_TIMEOUT = 0x01`, `FAULT_BATTERY_LOW = 0x02`, `FAULT_BATTERY_CRITICAL = 0x04`, `FAULT_LINK_TIMEOUT = 0x08`
  - `const MAX_FRAME: usize = 48`
  - `impl Command { fn encode(&self, out: &mut [u8; MAX_FRAME]) -> usize; fn decode(payload: &[u8]) -> Result<Command, DecodeError> }`(Telemetry も同じ)
  - `enum DecodeError { Cobs, Length, Crc, Type, Value }`
  - `struct FrameDecoder` / `fn push(&mut self, byte: u8) -> Option<&[u8]>`(0x00 を受けたら COBS を解いた中身を返す。バッファが溢れたら次の 0x00 まで破棄する)

ペイロードの並び(CRC は最後の1バイトで、それより前の全バイトに対して計算する):
- Command:`0x01, seq, throttle(i16), turn(i16), grip(u8), flags(u8), crc` = 9 バイト
- Telemetry:`0x02, seq, battery_mv(u16), enc_left(i32), enc_right(i32), enc_grip(i32), grip_state(u8), box_count(u8), sonar_mm[4](u16×4), status(u8: bit0=estop_latched), faults(u8), crc` = 29 バイト

`decode` は範囲外の値を `DecodeError::Value` にする(throttle と turn は -1000〜1000、grip は 0〜3、grip_state は 0〜6)。

- [ ] **Step 1: 失敗するテストを書く**(`protocol.rs` の `#[cfg(test)] mod tests`)

テスト名と主な検証内容:
- `command_roundtrip`:`Command { seq: 7, throttle: -1000, turn: 999, grip: GripCmd::Close, flags: FLAG_SLOW }` を encode → FrameDecoder に1バイトずつ push → decode して一致する。encode の結果は最後の1バイトだけが 0x00 で、途中に 0x00 を含まない。
- `telemetry_roundtrip`:`enc_left: i32::MIN`、`sonar_mm: [0, 1500, 20, 65535]`、`estop_latched: true` が往復で一致する。
- `crc_error_detected`:CRC の前のバイトを1ビット反転すると `Err(DecodeError::Crc)`。
- `out_of_range_rejected`:throttle = 1001 を手で組んだペイロードは `Err(DecodeError::Value)`。
- `seq_wraparound_is_accepted`:seq = 255 の次に seq = 0 を decode しても両方 `Ok`。
- `decoder_recovers_after_garbage`:`[0x13; 100]` を push したあとに正しいフレームを push すると、そのフレームが取り出せる。

- [ ] **Step 2: 失敗を確認する**

Run: `cd firmware && cargo test -p robot-core protocol`
Expected: コンパイルエラー(型が未定義)

- [ ] **Step 3: 実装する**

`robot-core/Cargo.toml`:`corncobs = "0.1"`, `crc = "3"`。`firmware/Cargo.toml` は `[workspace] members = ["robot-core", "host"]`, `resolver = "3"`。

- [ ] **Step 4: 成功を確認する**

Run: `cd firmware && cargo test -p robot-core protocol`
Expected: 6 passed

- [ ] **Step 5: コミットする**

```bash
git add .gitignore firmware/Cargo.toml firmware/Cargo.lock firmware/robot-core
git commit -m "feat(core): command/telemetry protocol with COBS and CRC-8"
```

---

### Task 2: 駆動の配分と PWM 極性

**Files:**
- Create: `firmware/robot-core/src/drive.rs`

**Interfaces:**
- Produces(`robot_core::drive`):
  - `fn mix(throttle: i16, turn: i16, limit: i16) -> (i16, i16)`:左 = throttle + turn、右 = throttle − turn。どちらかの絶対値が 1000 を超えたら、比を保って両方を縮める。最後に ±`limit` で切る。
  - `const SLOW_LIMIT: i16 = 400`
  - `struct SlewLimiter { current: i16 }` / `fn step(&mut self, target: i16, max_step: i16) -> i16`
  - `const ACCEL_STEP: i16 = 40`(100Hz で 0→1000 が 0.25 秒)
  - `struct BridgeOutput { in1: u16, in2: u16 }` / `fn bridge(cmd: i16, period: u16) -> BridgeOutput`:`cmd > 0` なら `in1 = period, in2 = period - period*|cmd|/1000`。`cmd < 0` なら `in1` と `in2` を入れ替える。`cmd == 0` なら両方 `period`(ブレーキ)。

- [ ] **Step 1: 失敗するテストを書く**

- `mix_straight`:`mix(500, 0, 1000) == (500, 500)`
- `mix_spin`:`mix(0, 300, 1000) == (300, -300)`
- `mix_saturates_keeping_ratio`:`mix(1000, 500, 1000) == (1000, 333)`
- `mix_slow_limit`:`mix(1000, 0, SLOW_LIMIT) == (400, 400)`
- `slew_limits_step`:`current=0` から `step(1000, 40)` を3回で 120。`step(-1000, 40)` も同様に減る。
- `bridge_forward_has_brake_off_phase`:`bridge(250, 3600) == BridgeOutput { in1: 3600, in2: 2700 }`
- `bridge_reverse_mirrors_forward`:`bridge(-250, 3600) == BridgeOutput { in1: 2700, in2: 3600 }`
- `bridge_zero_is_brake`:`bridge(0, 3600) == BridgeOutput { in1: 3600, in2: 3600 }`
- `bridge_full`:`bridge(1000, 3600).in2 == 0`

- [ ] **Step 2: 失敗を確認する** — Run: `cd firmware && cargo test -p robot-core drive` / Expected: コンパイルエラー
- [ ] **Step 3: 実装する**
- [ ] **Step 4: 成功を確認する** — Expected: 9 passed
- [ ] **Step 5: コミットする** — `git commit -m "feat(core): arcade mixing, slew limiter, brake-decay bridge output"`

---

### Task 3: 安全機能(タイムアウト・非常停止・電池)

**Files:**
- Create: `firmware/robot-core/src/safety.rs`

**Interfaces:**
- Produces(`robot_core::safety`):
  - `struct Watchdog { last_ms: Option<u32> }` / `fn feed(&mut self, now_ms: u32)` / `fn expired(&self, now_ms: u32) -> bool`(一度も feed されていなければ true。`now_ms` の wrapping 減算で判定)。`const LINK_TIMEOUT_MS: u32 = 200`
  - `fn adc_to_mv(raw: u16) -> u16`:`raw * 3300 * 57 / (4095 * 10)`(47kΩ / 10kΩ 分圧、Vref 3.3V、12 ビット)。u32 で計算する。
  - `enum BatteryLevel { Ok, Low, Critical, Absent }`
  - `struct BatteryMonitor { filtered_mv: u32, critical: bool, low: bool }` / `fn update(&mut self, raw_mv: u16) -> BatteryLevel`:指数移動平均(新値の重み 1/16、初回は生値で初期化)。`Absent` は生値 < 3000mV のとき(平滑化しない。非常停止の即時検知のため)。`Critical` は平滑値 < 9900 で保持(以後ずっと `Critical`)。`Low` は平滑値 < 10500 で入り、> 10700 で抜ける。
  - `struct EstopLatch { latched: bool, need_neutral: bool }` / `fn update(&mut self, absent: bool, release: bool, stick_neutral: bool) -> bool`(true = 駆動禁止)。`absent` で保持。保持中は `release && !absent` で解除要求を受け付け、その後 `stick_neutral` になるまで駆動禁止を続ける。

- [ ] **Step 1: 失敗するテストを書く**

- `watchdog_expires_after_200ms`:feed(1000) → expired(1199) は false、expired(1200) は true。
- `watchdog_unfed_is_expired`
- `watchdog_handles_wraparound`:feed(u32::MAX - 50) → expired(100) は false。
- `adc_full_scale`:`adc_to_mv(4095) == 18810`、`adc_to_mv(0) == 0`。
- `battery_low_hysteresis`:10400 を 100 回 → `Low`、10600 を 100 回 → まだ `Low`、10800 を 100 回 → `Ok`。
- `brief_sag_does_not_trip_critical`:12000 で初期化後、9000 を 3 回(30ms)入れても `Critical` にならない。
- `critical_latches`:9500 を 100 回 → `Critical`。その後 12000 を 100 回でも `Critical`。
- `absent_is_immediate`:12000 の直後に 0 → `Absent`。
- `estop_latches_until_release`:absent → true。absent が消えても release が来るまで true。
- `estop_release_requires_neutral_stick`:release を受けた時点でスティックが倒れていれば true のまま。neutral になった時点で false。

- [ ] **Step 2〜4: 失敗を確認 → 実装 → 成功を確認** — Run: `cd firmware && cargo test -p robot-core safety` / Expected: 10 passed
- [ ] **Step 5: コミットする** — `git commit -m "feat(core): link watchdog, battery monitor, estop latch"`

---

### Task 4: 把持の状態管理とエンコーダのデコード

**Files:**
- Create: `firmware/robot-core/src/gripper.rs`, `firmware/robot-core/src/quad.rs`

**Interfaces:**
- Produces(`robot_core::quad`):`struct QuadDecoder { count: i32, last: u8 }` / `fn update(&mut self, a: bool, b: bool)`(4逓倍。不正な遷移(2ビット同時変化)は無視)/ `fn count(&self) -> i32`
- Produces(`robot_core::gripper`):
  - `struct GripperConfig { dir: i8, open_cmd: i16, close_cmd: i16, hold_cmd: i16, stall_eps: i32, stall_ticks: u16, timeout_ticks: u16, two_box_max: i32, one_box_max: i32 }`
  - `const DEFAULT_GRIPPER: GripperConfig = GripperConfig { dir: 1, open_cmd: -600, close_cmd: 500, hold_cmd: 200, stall_eps: 2, stall_ticks: 10, timeout_ticks: 300, two_box_max: 63 * 80, one_box_max: 63 * 130 }`(63 カウント/mm は JGA25-370 の 11 パルス × 4逓倍 × 減速比約 90 ÷ ピニオン周長 62.8mm の概算。`docs/tuning.md` で実測値に置き換える)
  - `struct Gripper { state: GripState, homed: bool, home: i32, last_enc: i32, still: u16, ticks: u16 }` / `fn new() -> Self` / `fn update(&mut self, cmd: GripCmd, enc: i32, cfg: &GripperConfig) -> i16`(モーター指令。正 = 閉じる向き、`cfg.dir` を掛けて返す)/ `fn state(&self) -> GripState` / `fn box_count(&self, enc: i32, cfg: &GripperConfig) -> u8`

状態遷移(`still` は |enc − last_enc| ≤ stall_eps が続いた周期数、`ticks` は動作開始からの周期数):

| 現在 | 入力 | 次 | 出力 |
|---|---|---|---|
| Unhomed / Idle / Open / Holding / Fault | Open | Opening | open_cmd |
| Unhomed / Idle / Open / Fault | Close | 未原点なら Opening(原点合わせを先に行う)、原点済みなら Closing | close_cmd |
| Opening | still ≥ stall_ticks | Open(原点合わせ中なら home = enc、homed = true) | 0 |
| Closing | still ≥ stall_ticks | Holding | hold_cmd |
| Opening / Closing | ticks ≥ timeout_ticks | Fault | 0 |
| 任意 | Stop | Idle(原点情報は保持) | 0 |
| 任意 | Keep | 状態維持 | 状態に応じた出力 |

表にない組み合わせ(例:Holding 中の Close、Closing 中の Close)は状態を維持する。

箱数:Holding のときだけ判定する。閉じた量 `d = (enc - home) * dir` が `d ≤ two_box_max` なら 2、`d ≤ one_box_max` なら 1、それ以外は 0。

- [ ] **Step 1: 失敗するテストを書く**

- `quad_counts_forward_and_back`:A/B を 00→10→11→01→00 で +4、逆順で −4。
- `quad_ignores_invalid_transition`:00→11 でカウントが変わらない。
- `first_close_homes_then_closes`:Unhomed で Close → Opening(出力 −600)。enc を 11 周期止める → Open。続けて Close → Closing(出力 500)。
- `close_until_stall_then_hold`:Closing で enc を増やし続けてから 11 周期止める → Holding(出力 200)。
- `box_count_two`:home = 0、閉じた量 63*50 で Holding → 2。
- `box_count_one_and_zero`:63*125 → 1、63*135 → 0。
- `timeout_goes_fault`:Closing で enc を毎周期 +10 し続けて 300 周期 → Fault(出力 0)。
- `stop_keeps_home`:Holding で Stop → Idle。その後 Close で Opening を経由せずに Closing へ。
- `reversed_encoder_with_dir_flag`:`dir: -1` で enc を減らしながら閉じても Holding と箱数の判定が正しく、出力の符号が反転する。

- [ ] **Step 2〜4: 失敗を確認 → 実装 → 成功を確認** — Run: `cd firmware && cargo test -p robot-core gripper quad` / Expected: 9 passed
- [ ] **Step 5: コミットする** — `git commit -m "feat(core): stall-based gripper state machine and quadrature decoder"`

---

### Task 5: 超音波の換算と 100Hz 制御ステップ

**Files:**
- Create: `firmware/robot-core/src/ultrasonic.rs`, `firmware/robot-core/src/control.rs`

**Interfaces:**
- Consumes:Task 1〜4 のすべて
- Produces:
  - `fn echo_to_mm(cycles: u32, cpu_hz: u32) -> u16`:`cycles * 343_000 / cpu_hz / 2` を u64 で計算する。20〜1500mm の範囲外は 0 を返す。
  - `struct Inputs { now_ms: u32, cmd: Option<Command>, battery_raw_mv: u16, enc_grip: i32 }`(`cmd` はこの周期に新しく届いた指令)
  - `struct Outputs { left: i16, right: i16, grip: i16, faults: u8, estop_latched: bool, grip_state: GripState, box_count: u8, battery_mv: u16 }`
  - `struct Controller { .. }` / `fn new(cfg: GripperConfig) -> Self` / `fn step(&mut self, i: &Inputs) -> Outputs`

`step` の手順:
1. 新しい指令があれば watchdog を feed し、最新指令として保持する。
2. 電池の判定を行う。`Absent` を非常停止の検知に使う。
3. 非常停止の保持を更新する(stick_neutral は throttle == 0 かつ turn == 0)。
4. 駆動禁止(非常停止の保持中、Critical、またはタイムアウト)なら、左右の目標を 0 にし、SlewLimiter をリセットして即 0 にする。そうでなければ `mix`(低速フラグで limit = SLOW_LIMIT、Low なら limit を 500 以下に)→ SlewLimiter。
5. 把持:非常停止の保持中か Critical なら `GripCmd::Stop`。それ以外は、タイムアウト中なら `GripCmd::Keep`、そうでなければ指令の grip を使う。
6. faults を組み立てる。

- [ ] **Step 1: 失敗するテストを書く**

- `echo_1m`:`echo_to_mm(419_825, 72_000_000) == 1000`(±1 を許容)
- `echo_out_of_range_is_zero`:10mm 相当と 2000mm 相当で 0。
- `no_command_means_brake`:起動直後の step は left = right = 0、faults に `FAULT_LINK_TIMEOUT`。
- `command_drives_with_slew`:throttle 1000 の指令を毎周期送ると、左右が 40, 80, 120 と増える。
- `timeout_brakes_but_keeps_grip`:Holding にしたあと指令を止めて 21 周期 → left = right = 0、grip は hold_cmd のまま。
- `estop_cuts_everything`:battery_raw_mv = 0 → left = right = grip = 0、`estop_latched == true`。
- `low_battery_limits_output`:平滑値が 10.4V のとき throttle 1000 で最終的に left ≤ 500。

- [ ] **Step 2〜4: 失敗を確認 → 実装 → 成功を確認** — Run: `cd firmware && cargo test -p robot-core` / Expected: 全テスト(41 件)passed
- [ ] **Step 5: コミットする** — `git commit -m "feat(core): ultrasonic conversion and 100Hz controller step"`

---

### Task 6: PC 側操縦プログラム

**Files:**
- Create: `firmware/host/Cargo.toml`, `firmware/host/src/main.rs`, `firmware/host/src/mapping.rs`, `firmware/host/src/link.rs`, `firmware/host/src/log.rs`, `firmware/host/config.toml`

**Interfaces:**
- Consumes:`robot_core::protocol::{Command, Telemetry, FrameDecoder, GripCmd, FLAG_*}`
- Produces:
  - `struct PadState { left_y: f32, right_x: f32, close: bool, open: bool, slow_toggle: bool, estop_release: bool }`(ボタンは押されている間 true)
  - `#[derive(Deserialize)] struct Mapping { deadzone: f32, close: String, open: String, slow_toggle: String, estop_release: String, invert_left_y: bool }`(ボタン名は gilrs の `Button` の Debug 名。例 `"South"`)
  - `struct Mapper { slow: bool, prev: PadState, seq: u8 }` / `fn command(&mut self, pad: &PadState, m: &Mapping) -> Command`
  - `fn find_port() -> Option<String>`(VID 0x16C0 / PID 0x27DD の USB シリアルを探す)

`config.toml` の既定値:`deadzone = 0.08`、`close = "South"`、`open = "East"`、`slow_toggle = "North"`、`estop_release = "Start"`、`invert_left_y = true`。

`command` の仕様:スティック値がデッドゾーン内なら 0。外側はデッドゾーン分を詰めて -1000〜1000 に線形変換する。前進量は left_y、旋回量は right_x。slow はボタンが押された瞬間(立ち上がり)に反転する。close と open が同時なら Keep。どちらでもなければ Keep。estop_release は押されている間フラグを立てる。seq は呼ぶたびに +1(wrapping)。

`main` の動作:50Hz ループで gilrs のイベントを処理 → `command` → encode して送信。受信バイトを FrameDecoder に入れ、Telemetry を画面に1行で表示(10Hz、`\r` で上書き)し、CSV に追記する(`logs/telemetry-<起動時刻>.csv`、ヘッダは Telemetry のフィールド名 + `host_ms`)。ポートが開けない・切れたときは 0.5 秒ごとに `find_port` で再接続する。引数 `--port <path>` で自動検出を上書き、`--config <path>` で設定ファイルを指定。

- [ ] **Step 1: 失敗するテストを書く**(`mapping.rs`)

- `deadzone_maps_to_zero`:left_y = 0.05 → throttle 0。
- `full_stick_is_1000`:left_y = −1.0(invert_left_y = true)→ throttle 1000。right_x = −1.0 → turn −1000。
- `edge_after_deadzone_is_small`:left_y = −0.09 → throttle は 1〜20。
- `slow_toggles_on_rising_edge`:slow_toggle を true のまま 3 回呼んでも1回だけ反転。離して押すと戻る。
- `open_and_close_together_is_keep`
- `seq_increments_and_wraps`

- [ ] **Step 2〜4: 失敗を確認 → 実装 → 成功を確認** — Run: `cd firmware && cargo test -p host` / Expected: 6 passed。`cargo build -p host` が通る(Linux では `libudev-dev` が必要。無ければ `apt-get install -y libudev-dev`)。
- [ ] **Step 5: コミットする** — `git commit -m "feat(host): gamepad teleop over USB serial with telemetry log"`

---

### Task 7: ファームウェア(RTICX)

**Files:**
- Create: `firmware/firmware/Cargo.toml`(独自の `[workspace]`)、`firmware/firmware/.cargo/config.toml`、`firmware/firmware/memory.x`、`firmware/firmware/build.rs`、`firmware/firmware/src/main.rs`、`firmware/firmware/src/board.rs`、`firmware/firmware/src/motors.rs`、`firmware/firmware/src/sonar.rs`
- Create: `docs/bringup.md`

**Interfaces:**
- Consumes:`robot_core::control::{Controller, Inputs, Outputs}`、`robot_core::drive::bridge`、`robot_core::quad::QuadDecoder`、`robot_core::ultrasonic::echo_to_mm`、`robot_core::protocol::*`、`robot_core::safety::adc_to_mv`、`robot_core::gripper::DEFAULT_GRIPPER`
- Produces:`firmware.elf`(ST-Link で書き込み)

設定値:
- `.cargo/config.toml`:`target = "thumbv7m-none-eabi"`、`runner = "probe-rs run --chip STM32F103C8"`、`rustflags = ["-C", "link-arg=-Tlink.x", "-C", "link-arg=-Tdefmt.x"]`
- `memory.x`:FLASH 64K @ 0x08000000、RAM 20K @ 0x20000000
- クロック:HSE 8MHz → SYSCLK 72MHz、PCLK1 36MHz、USB 48MHz。
- PWM:TIM4 CH1〜4(PB6〜9)、TIM1 CH1〜2(PA8、PA9)を 20kHz。周期の値(ARR + 1)を `bridge` の `period` に渡す。
- エンコーダ:TIM2(PA0、PA1)、TIM3(PA6、PA7)を Qei モード。16 ビットのカウンタは、毎周期の差分を i32 に積算する。
- 把持エンコーダ:PB3、PB4 を両エッジの EXTI3、EXTI4 にし、割り込みで `QuadDecoder::update`。JTAG を無効化して SWD のみ残す(`afio.mapr.disable_jtag`)。
- 超音波:PB0、PB1 を出力(10µs のパルス)。PB12〜15 を両エッジの EXTI(EXTI15_10)にし、DWT の CYCCNT で立ち上がりから立ち下がりまでを測る。
- ADC:ADC1 で PA4 を読む。
- USB:`usb-device` + `usbd-serial`。VID/PID 0x16C0/0x27DD、製品名 `voyager-carrier`。

RTICX のタスク(書き方は `rticx-cortex-m` の examples の `hello_rtic.rs` と `async_ping_pong.rs` に従う):

| タスク | 種類 | 優先度 | 内容 |
|---|---|---|---|
| `UsbIrq` | ハードウェアタスク(`USB_LP_CAN_RX0`) | 3 | USB を poll し、受信バイトを FrameDecoder に入れて、デコードできた Command を共有の `latest_cmd: Option<Command>` に置く |
| `GripEnc` | ハードウェアタスク(`EXTI3`、`EXTI4`) | 4 | QuadDecoder を更新する |
| `SonarEcho` | ハードウェアタスク(`EXTI15_10`) | 4 | エッジ時刻を記録し、立ち下がりで距離を共有の `sonar_mm` に置く |
| `Control` | 非同期タスク、10ms 周期 | 2 | ADC・エンコーダ・`latest_cmd.take()` を集めて `Controller::step`、`bridge` で PWM を設定 |
| `Telemetry` | 非同期タスク、50ms 周期 | 1 | Telemetry を encode して USB に書く(書けなければ捨てる) |
| `SonarTrigger` | 非同期タスク、33ms 周期 | 1 | 前寄りペアと後ろ寄りペアを交互にトリガー |

- [ ] **Step 1: ビルド環境を用意する**

Run: `rustup target add thumbv7m-none-eabi && cargo install flip-link probe-rs-tools --locked`(probe-rs は書き込み用。コンテナ内では書き込みしない)
Expected: `rustup target list --installed` に `thumbv7m-none-eabi` が出る。

- [ ] **Step 2: 最小構成で RTICX の起動を確認する**

`main.rs` に LED(PC13)を 500ms 周期で点滅させる非同期タスクだけを書く。
Run: `cd firmware/firmware && cargo build --release`
Expected: ビルドが成功する。

- [ ] **Step 3: 周辺機能とタスクを実装する**(上の表のとおり)

- [ ] **Step 4: ビルドとサイズを確認する**

Run: `cd firmware/firmware && cargo build --release && cargo size --release -- -A | grep -E "^\.(text|data|bss)"`
Expected: ビルドが成功する。`.text` + `.data` が 65536 未満、`.data` + `.bss` が 20480 未満。

- [ ] **Step 5: 実機の立ち上げ手順書 `docs/bringup.md` を書く**

順番と各項目の合格条件:
1. Bluepill 単体:LED 点滅、PC に `voyager-carrier` として認識される(`R10` の不良なら 1.8kΩ を付ける)。
2. host を起動し、テレメトリが 20Hz で表示される。host を止めて 200ms 以内に `FAULT_LINK_TIMEOUT` が立つ(テレメトリで確認)。
3. モーターを外したまま、テスターかロジックアナライザで PWM を確認する(前進で IN1 が High 固定、IN2 が反転 PWM)。
4. 車輪を浮かせて駆動モーターを接続する。前進で左右とも前に回ること、エンコーダが前進で増えることを確認する。違えば配線を入れ替える。
5. 把持モーター:Close で閉じる向きに動くこと。逆なら配線を入れ替える。エンコーダが閉じる向きで増えなければ `dir = -1` にする。
6. 非常停止を押すと全モーターが止まり、解除して release ボタンを押し、スティックを中立にするまで動かないこと。
7. 超音波:壁から 300mm に置いて 280〜320mm と表示されること。

- [ ] **Step 6: コミットする**

```bash
git add firmware/firmware docs/bringup.md
git commit -m "feat(firmware): RTICX firmware wiring robot-core to STM32F103 peripherals"
```

---

### Task 8: 調整手順書と当日チェックリスト

**Files:**
- Create: `docs/tuning.md`, `docs/checklist.md`, `docs/rules.md`

- [ ] **Step 1: `docs/tuning.md` を書く**

項目:(1)カウント/mm の実測(把持を開ききりから閉じきりまで動かし、カウント差 ÷ 実際の移動量)。(2)把持力の調整:ばねばかりでアームの間を引き、片側 10N になるよう `close_cmd` と `hold_cmd` を決める。(3)`two_box_max` と `one_box_max` を実測の閉じた位置の中間値にする。(4)電池のしきい値の確認(安定化電源で 10.4V と 9.8V を与える)。各値は `robot-core/src/gripper.rs` の `DEFAULT_GRIPPER` に反映し、`cargo test` が通ることを確認してからコミットする。

- [ ] **Step 2: `docs/checklist.md` を書く**

大会当日の確認:電池の電圧(1セル 4.0V 以上)、ヒューズ、非常停止の動作、USB ケーブルとアイソレータ、host の起動とテレメトリ、把持の原点合わせ、予備部品(Bluepill、DRV8871、ヒューズ、電池)、運営への申請(バッテリー)。

- [ ] **Step 3: `docs/rules.md` を書く**

設計書 2章の表を元に、このロボットに関係するルール(保持の定義 2.4.2、スラローム内の保持上限 2.4.3、木枠側面の接触禁止 3.3 と Q21、ゴールゾーンの黒い領域、サイズ、電源、非常停止、申請)を条文番号付きで箇条書きにする。

- [ ] **Step 3: コミットする** — `git commit -m "docs: tuning procedure and match-day checklist"`
