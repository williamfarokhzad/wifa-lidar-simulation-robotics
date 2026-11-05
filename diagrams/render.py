"""Render editable diagrams from the example scenes. Run from project root."""
import json
import math
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lidar_scan import scan

HERE = Path(__file__).resolve().parent
INK, TEAL, ORANGE = "#19333f", "#06756e", "#9a451d"


def label(x, y, value, size=23, color=INK, weight=500):
    return (f'<text x="{x}" y="{y}" font-family="DejaVu Sans,sans-serif" '
            f'font-size="{size}" font-weight="{weight}" fill="{color}">{value}</text>')


def svg(contents):
    return ('<svg xmlns="http://www.w3.org/2000/svg" width="960" height="620" '
            'viewBox="0 0 960 620"><rect width="960" height="620" rx="24" '
            'fill="#f2f8f7"/>' + contents + '</svg>\n')


def scene(name):
    return json.loads((HERE.parent / "examples" / f"{name}.json").read_text())


def aisle():
    s = scene("aisle")
    out = label(40, 63, "Which surface returns first?", 37, weight=700)
    out += label(40, 103, "Top view · aisle.json · distances in metres")
    out += '<rect x="34" y="130" width="494" height="460" rx="14" fill="#fff"/>'
    origin_x, origin_y, scale = 114, 335, 64
    for i in range(6):
        x = origin_x + i * scale
        out += f'<path d="M{x} 146V570" stroke="#dbe6e7"/>'
        out += label(x - 5, 584, str(i), 16)
    for i in range(-3, 4):
        y = origin_y - i * scale
        out += f'<path d="M49 {y}H510" stroke="#dbe6e7"/>'
    out += label(477, 363, "+X", 19) + label(72, 168, "+Y", 19)
    for a, b, c, d in s["boxes"]:
        out += (f'<rect x="{origin_x+a*scale}" y="{origin_y-d*scale}" '
                f'width="{(c-a)*scale}" height="{(d-b)*scale}" '
                'fill="#abc8c8" stroke="#365e63" stroke-width="2.5"/>')
    for row in scan(s):
        hit = row["status"] == "hit"
        t = row["range_m"] if hit else s["limits"][1]
        rad = math.radians(row["angle_deg"])
        x, y = origin_x + t*scale*math.cos(rad), origin_y - t*scale*math.sin(rad)
        color = TEAL if hit else ORANGE
        dash = '' if hit else ' stroke-dasharray="11 8"'
        out += (f'<path d="M{origin_x} {origin_y}L{x:.3f} {y:.3f}" '
                f'stroke="{color}" stroke-width="4"{dash}/>')
        if hit:
            out += f'<circle cx="{x:.3f}" cy="{y:.3f}" r="6" fill="{color}"/>'
    out += f'<circle cx="{origin_x}" cy="{origin_y}" r="8" fill="{INK}"/>'
    for y, heading, detail, col in [
        (195, "+45° · 1.414 m", "Hits the side box.", TEAL),
        (305, "0° · 2.000 m", "Front box hides the wall.", TEAL),
        (415, "−45° · no return", "Dashed to the 5 m limit.", ORANGE),
    ]:
        out += label(555, y, heading, 29, col, 700) + label(555, y+35, detail)
    return svg(out)


def limits():
    s = scene("blind-zone")
    rows = scan(s)
    near, far = s["limits"]
    scale = 140
    x = lambda metres: 102 + metres * scale
    out = label(40, 63, "A missing range has two meanings", 36, weight=700)
    out += label(40, 104, "Three separate bearings · blind-zone.json")
    for i, (row, dist, extra) in enumerate(zip(rows, [s["boxes"][1][0], rows[1]["range_m"], -s["boxes"][3][2]],
                                               ["Near face blocks the 2 m box.",
                                                "Exactly at the 5 m limit.",
                                                "Face is outside the 5 m limit."])):
        y = 215 + 150*i
        out += f'<rect x="{x(0)}" y="{y-13}" width="{scale*near}" height="26" fill="#eccab3"/>'
        out += f'<path d="M{x(0)} {y}H{x(5.45)}" stroke="#9bb3ba" stroke-width="2"/>'
        for tick in (0, near, far):
            out += f'<path d="M{x(tick)} {y-19}v38" stroke="#678189" stroke-width="2"/>'
        col = TEAL if row["status"] == "hit" else ORANGE
        out += f'<circle cx="{x(dist)}" cy="{y}" r="8" fill="{col}"/>'
        if i == 0:
            ghost = x(s["boxes"][0][0])
            out += f'<circle cx="{ghost}" cy="{y}" r="7" fill="none" stroke="#678189" stroke-width="2"/>'
        out += label(102, y-29, f'{row["angle_deg"]:.0f}° · {row["status"].replace("_", " ")}', 28, col, 700)
        out += label(102, y+51, extra)
    out += label(756, 587, "5 m limit", 19)
    return svg(out)


def main():
    for name, source in [("lidar-first-return", aisle()), ("lidar-range-limits", limits())]:
        vector = HERE / f"{name}.svg"
        vector.write_text(source, encoding="utf-8")
        with TemporaryDirectory() as folder:
            png = Path(folder) / "image.png"
            subprocess.run(["inkscape", str(vector), f"--export-filename={png}"], check=True)
            subprocess.run(["convert", str(png), "-strip", "-quality", "88",
                            str(HERE / f"{name}.webp")], check=True)


if __name__ == "__main__":
    main()
