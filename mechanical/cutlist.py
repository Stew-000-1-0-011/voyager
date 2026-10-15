"""Aluminium extrusion cut list -> build/cutlist.md."""

from __future__ import annotations

from collections import Counter
from pathlib import Path

from params import EXTRUSION
from parts.chassis import frame_members, upright_members


def cutlist() -> list[tuple[float, int, list[str]]]:
    counts: Counter[float] = Counter()
    names: dict[float, list[str]] = {}
    for name, length, _ in frame_members() + upright_members():
        counts[length] += 1
        names.setdefault(length, []).append(name)
    return [(length, counts[length], names[length]) for length in sorted(counts, reverse=True)]


def write(path: Path) -> None:
    lines = [f"# アルミフレーム切断表({EXTRUSION:.0f}mm 角)", "", "| 長さ (mm) | 本数 | 部位 |", "|---|---|---|"]
    lines += [f"| {length:.1f} | {n} | {', '.join(names)} |" for length, n, names in cutlist()]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    write(Path(__file__).parent / "build" / "cutlist.md")
