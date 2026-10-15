"""Line drawings (hidden lines dotted) of the assembly and printed parts -> images/*.png."""

from __future__ import annotations

import io
import tempfile
from pathlib import Path

import cairosvg
from build123d import Compound, ExportSVG, LineType, Part
from PIL import Image, ImageDraw, ImageFont

from assembly import build, printed_parts
from params import CLOSED_INNER, OPEN_INNER

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
OUT = Path(__file__).parent / "images"

# (eye direction, up vector)
VIEWS = {
    "iso": ((1.0, -1.2, 0.9), (0, 0, 1)),
    "top": ((0, 0, 1), (1, 0, 0)),       # front of the robot points up
    "front": ((1, 0, 0), (0, 0, 1)),     # looking at the gripper from ahead
    "side": ((0, -1, 0), (0, 0, 1)),     # from the robot's right side, front to the right
}


def _png(shape: Part | Compound, view: str, width: int) -> Image.Image:
    eye, up = VIEWS[view]
    origin = tuple(1000 * c for c in eye)
    visible, hidden = shape.project_to_viewport(origin, viewport_up=up)
    svg = ExportSVG(margin=5, line_weight=0.8)
    svg.add_layer("hidden", line_color=0x969696, line_type=LineType.ISO_DOT, line_weight=0.3)
    svg.add_layer("visible", line_color=0x141E3C)
    svg.add_shape(hidden, layer="hidden")
    svg.add_shape(visible, layer="visible")
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "view.svg"
        svg.write(str(path))
        png = cairosvg.svg2png(url=str(path), output_width=width, background_color="white")
    return Image.open(io.BytesIO(png)).convert("RGB")


def _font(size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(FONT, size)


def _titled(img: Image.Image, title: str, size: int = 28) -> Image.Image:
    out = Image.new("RGB", (img.width, img.height + size + 20), "white")
    out.paste(img, (0, size + 20))
    ImageDraw.Draw(out).text((10, 6), title, fill=(0, 0, 0), font=_font(size))
    return out


def _grid(images: list[Image.Image], cols: int, pad: int = 20) -> Image.Image:
    rows = (len(images) + cols - 1) // cols
    cw = max(i.width for i in images)
    rh = max(i.height for i in images)
    out = Image.new("RGB", (cols * cw + (cols + 1) * pad, rows * rh + (rows + 1) * pad), "white")
    for k, im in enumerate(images):
        r, c = divmod(k, cols)
        out.paste(im, (pad + c * (cw + pad), pad + r * (rh + pad)))
    return out


def _summary(parts: dict[str, Part], open_: bool) -> str:
    bb = None
    for p in parts.values():
        bb = p.bounding_box() if bb is None else bb.add(p.bounding_box())
    inner = OPEN_INNER if open_ else CLOSED_INNER
    state = "開いた状態" if open_ else "閉じた状態"
    return (f"{state}  全長 {bb.size.X:.0f} mm × 幅 {bb.size.Y:.0f} mm × 高さ {bb.size.Z:.0f} mm"
            f"  /  パッド内幅 {inner:.0f} mm")


def render_assembly(open_: bool) -> None:
    parts = build(open_)
    comp = Compound(children=list(parts.values()))
    name = "open" if open_ else "closed"
    _titled(_png(comp, "iso", 1600), _summary(parts, open_)).save(OUT / f"assembly_{name}_iso.png")
    views = [_titled(_png(comp, v, 900), label) for v, label in
             (("top", "上面図(上 = 前方)"), ("front", "正面図(前から)"), ("side", "側面図(右から、右 = 前方)"))]
    sheet = _grid(views, 3)
    _titled(sheet, _summary(parts, open_), 34).save(OUT / f"assembly_{name}_views.png")


def render_parts() -> None:
    tiles = [_titled(_png(part, "iso", 500), f"{name} ×{qty}", 24) for name, (part, qty) in printed_parts().items()]
    _titled(_grid(tiles, 4), "3Dプリント部品(PETG)", 34).save(OUT / "parts_printed.png")


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    render_assembly(True)
    render_assembly(False)
    render_parts()
