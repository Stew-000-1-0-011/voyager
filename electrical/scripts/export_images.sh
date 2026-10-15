#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build/sch images
kicad-cli sch export svg --exclude-drawing-sheet -o build/sch carrier.kicad_sch >/dev/null
uv run --quiet python - <<'PY'
from pathlib import Path
import cairosvg

def to_png(src: Path, dst: Path) -> None:
    cairosvg.svg2png(url=str(src), write_to=str(dst), output_width=2400, background_color="white")

to_png(next(Path("build/sch").glob("*.svg")), Path("images/schematic.png"))
for side in ("top", "bottom", "wires"):
    svg = Path(f"perfboard_{side}.svg")
    if svg.exists():
        to_png(svg, Path(f"images/perfboard_{side}.png"))
PY
