"""Regenerate dimensions.svg (1 drawing unit = 1 mm at the stated scale) and dimensions.json from the primitives in
task.py. Run: python fixture/draw.py (needs NumPy and the yam_arena package that task.py imports)."""

from __future__ import annotations

import itertools
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import task  # noqa: E402

C = np.array(list(itertools.product((-1, 1), repeat=3)))
COLOURS = {"horizontal_test_machine": "#7495b5", "type_i_specimen": "#e8d9b0", "extensometer": "#343a40",
           "open_storage_case": "#8a8f94"}


def hull(points):
    p = sorted(set(map(tuple, np.round(points, 3))))

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    lo, hi = [], []
    for q in p:
        while len(lo) > 1 and cross(lo[-2], lo[-1], q) <= 0:
            lo.pop()
        lo.append(q)
    for q in p[::-1]:
        while len(hi) > 1 and cross(hi[-2], hi[-1], q) <= 0:
            hi.pop()
        hi.append(q)
    return lo[:-1] + hi[:-1]


def boxes(spec, pos=None):
    s = spec.state(pos=pos)
    return spec.world_boxes({"base": (s.pos, task.parts._quat_to_matrix(s.quat_xyzw))})


def poly(points, fill, opacity=0.6):
    return ('<polygon points="' + " ".join(f"{x:.2f},{y:.2f}" for x, y in hull(points))
            + f'" fill="{fill}" fill-opacity="{opacity}" stroke="#263647" stroke-width=".5"/>')


out = ['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="1100" viewBox="0 0 1100 1100">',
       '<rect width="1100" height="1100" fill="white"/><g font-family="Arial" fill="#182737">',
       '<text x="30" y="34" font-size="22">Fit the extensometer: model geometry (mm)</text>',
       '<text x="30" y="58" font-size="13">Maker dimensions for the 2630-105 core, ESM303H frame and Type I specimen;'
       ' detail offsets are estimates. Not maker CAD.</text>',
       '<text x="30" y="92" font-size="15">Top view, layout A, 1 unit = 1 mm (world x, away from the robot, is up the page; y is to the left)</text>']
# top view: world x -> page up, world y -> page left; 1 mm = 1 unit
for spec in task.OBJECTS:
    for _, c, r, size in sorted(boxes(spec), key=lambda b: b[1][2]):
        pp = C * (size / 2) @ r.T + c
        out.append(poly(np.c_[880 - 1000 * pp[:, 1], 690 - 1000 * (pp[:, 0] - 0.05)], COLOURS[spec.name]))
out += ['<text x="900" y="360" font-size="13">Type I specimen</text>',
        '<text x="900" y="378" font-size="13">165 x 19 x 3.2 HDPE</text><text x="900" y="396" font-size="13">115 between jaw mouths</text>',
        '<text x="40" y="700" font-size="13">Open case 235 x 165; layout B: case and sensor 35 nearer the robot, 40 to the right</text>']
# side section through the fitted extensometer and the specimen (x across, z up), 1:1 at 4 px per mm
ox, oz = 420.0, 980.0
out.append('<text x="30" y="760" font-size="15">Section at the near arm, fitted (4 units = 1 mm): knife edge on the front'
           ' face, rigid clip over the upper edge</text>')
fit = np.array((task.XF - task.L_TIP, task.Y, task.AXIS))
for spec, pos in ((task.SPECIMEN, None), (task.SENSOR, fit)):
    for _, c, r, size in boxes(spec, pos):
        pp = C * (size / 2) @ r.T + c
        if spec is task.SPECIMEN and abs(c[1] - task.Y) > 0.03:
            continue
        if spec is task.SENSOR and (c[1] - task.Y) > 0.001:
            continue  # near arm only
        out.append(poly(np.c_[ox + 4000 * (pp[:, 0] - fit[0]), oz - 4000 * (pp[:, 2] - task.AXIS)], COLOURS[spec.name],
                        0.45))
out += [f'<text x="820" y="880" font-size="13">Knife edge 10 long, 78 ahead of housing centre</text>',
        f'<text x="820" y="900" font-size="13">Clip wire 1.6 dia, 2.2 above the 13 wide edge</text>',
        f'<text x="820" y="920" font-size="13">Hook 3.0 behind the back face (clip held open)</text>',
        f'<text x="820" y="940" font-size="13">Housing 44 x 52 x 39; arms 6 x 6; gauge 25</text>',
        '</g></svg>']
here = Path(__file__).resolve().parent
(here / "dimensions.svg").write_text("\n".join(out))


def shapes(spec):
    return [dict(kind=s.kind, size_m=[round(v, 6) for v in s.size], at_m=[round(v, 6) for v in s.at],
                 quat_xyzw=[round(v, 6) for v in s.quat]) for s in spec.shapes]


(here / "dimensions.json").write_text(json.dumps({
    "units": "m, part frames; see task.py for the frames",
    "layouts": {"A": {"extensometer": task.SENSOR_A, "case": task.CASE_A},
                "B": {"extensometer": task.SENSOR_B, "case": task.CASE_B}},
    "fitted_extensometer_origin": fit.tolist(),
    "parts": {spec.name: dict(pos=list(spec.pos), mass_kg=spec.mass, shapes=shapes(spec)) for spec in task.OBJECTS},
}, indent=1))
print("wrote", here / "dimensions.svg", here / "dimensions.json")
