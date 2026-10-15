# voyager — ノーツ運搬ロボット

千葉大学ロボットコンテスト 2026 用の差動2輪ロボット。ノーツ(15cm 立方体の箱)を2個まとめて横から挟み、ノーツゾーンからゴールゾーンまで運ぶ。

- 設計書:`docs/superpowers/specs/2026-10-15-carrier-robot-design.md`
- 実装計画:`docs/superpowers/plans/`
- ルールの要点:`docs/rules.md` / 立ち上げ手順:`docs/bringup.md` / 調整:`docs/tuning.md` / 当日の確認:`docs/checklist.md`

## 構成

| ディレクトリ | 内容 | 確認方法 |
|---|---|---|
| `electrical/` | 回路の定義(`netlist.py`)から KiCad 8 回路図とユニバーサル基板の配置・配線を生成 | `cd electrical && uv run pytest`(ERC、回路図と定義の一致、配線の短絡なし) |
| `firmware/robot-core` | ハードウェアに依存しない制御(通信、駆動、安全、把持) | `cd firmware && cargo test -p robot-core` |
| `firmware/host` | PC 側の操縦プログラム(ゲームパッド → USB) | `cd firmware && cargo run -p host --release` |
| `firmware/firmware` | STM32F103(Bluepill)用ファームウェア、Rust no-std + RTICX | `cd firmware/firmware && cargo build --release` |
| `mechanical/` | build123d による機体の CAD(部品、組立て、干渉・外形チェック) | `cd mechanical && uv run pytest` |

## 図の再生成

- 回路図・配線図:`cd electrical && bash scripts/erc.sh && uv run python gen_perfboard.py && bash scripts/export_images.sh` → `electrical/images/`
- 機構:`cd mechanical && uv run python assembly.py && uv run python render.py && uv run python cutlist.py` → STEP / STL / 切断表は `mechanical/build/`、図は `mechanical/images/`

KiCad 8(`kicad-cli`)と日本語フォント(Noto Sans CJK)が必要。
