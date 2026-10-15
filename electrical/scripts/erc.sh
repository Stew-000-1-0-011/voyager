#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build
uv run --quiet python gen_schematic.py
kicad-cli sch export netlist --format kicadxml -o build/carrier.net carrier.kicad_sch
kicad-cli sch erc --severity-error --exit-code-violations -o build/erc.rpt carrier.kicad_sch
