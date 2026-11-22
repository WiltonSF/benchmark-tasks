"""Replaceable pivot-bore bushing for real trials (see fixture/README.md, "Real-trial finger").

Run ``python fixture/bushings.py`` to write ``bushing.stl`` (millimetres, axis along z, binary STL). The same
bushing goes in both ears. It is a tube 3.00 mm long, the thickness of a T42 knuckle ear, with a 0.3 mm x 45 degree
lead-in chamfer on the outside edge of one end (pressed in first) so it starts square. The bore is drawn at the
maker's 6.70 mm CAD pivot bore, so it prints tight in the same way as the finger's own ears; the outside is drawn
0.10 mm over the carrier finger's 9.30 mm sockets for a press fit. The simulation does not use this file: it models
each ear and its bushing as one piece with the same bore.
"""
from __future__ import annotations

import struct
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
LENGTH = 3.00  # mm, = ear thickness (STL z 4.50-7.50 and 24.25-27.25)
BORE = 6.70  # mm, drawn as the maker's CAD pivot bore; prints about 6.20-6.35 mm (enrolment window)
OD = 9.40  # mm, drawn outside diameter (carrier sockets 9.30 mm)
CHAMFER = 0.30  # mm x 45 degrees, outside edge of the z = 0 end
SEGMENTS = 96


def tube(od: float, bore: float, length: float, chamfer: float, n: int = SEGMENTS) -> np.ndarray:
    """Triangles (k, 3, 3) of a closed tube with a chamfered outside edge at z = 0, outward normals by vertex order."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    c, s = np.cos(a), np.sin(a)
    ro, ri, rc = od / 2, bore / 2, od / 2 - chamfer

    def p(r, i, z):
        return (r * c[i], r * s[i], z)

    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [(p(ro, i, chamfer), p(ro, j, chamfer), p(ro, j, length)),
                 (p(ro, i, chamfer), p(ro, j, length), p(ro, i, length))]  # outside wall
        tris += [(p(rc, i, 0.0), p(rc, j, 0.0), p(ro, j, chamfer)),
                 (p(rc, i, 0.0), p(ro, j, chamfer), p(ro, i, chamfer))]  # lead-in chamfer
        tris += [(p(ri, i, 0.0), p(ri, j, length), p(ri, j, 0.0)),
                 (p(ri, i, 0.0), p(ri, i, length), p(ri, j, length))]  # bore wall (faces the axis)
        tris += [(p(ro, i, length), p(ro, j, length), p(ri, j, length)),
                 (p(ro, i, length), p(ri, j, length), p(ri, i, length))]  # top face
        tris += [(p(rc, i, 0.0), p(ri, j, 0.0), p(rc, j, 0.0)),
                 (p(rc, i, 0.0), p(ri, i, 0.0), p(ri, j, 0.0))]  # bottom face (chamfered end)
    return np.asarray(tris, float)


def write_stl(path: Path, tris: np.ndarray, name: str) -> None:
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    with path.open('wb') as f:
        f.write(name.encode()[:80].ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        for k in range(len(tris)):
            f.write(struct.pack('<12fH', *n[k], *tris[k].ravel(), 0))


if __name__ == '__main__':
    t = tube(OD, BORE, LENGTH, CHAMFER)
    out = HERE / 'bushing.stl'
    write_stl(out, t, f'T42 pivot-bore bushing: OD {OD:.2f} bore {BORE:.2f} x {LENGTH:.2f} mm')
    v = t
    vol = np.einsum('ij,ij->i', v[:, 0], np.cross(v[:, 1], v[:, 2])).sum() / 6 / 1000  # cm3
    print(f'{out.name}: {len(t)} triangles, {vol:.3f} cm3, about {vol * 1.04:.2f} g of ABS')
