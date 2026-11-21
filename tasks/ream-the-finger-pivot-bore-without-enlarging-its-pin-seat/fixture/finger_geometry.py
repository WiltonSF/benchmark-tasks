"""Native finger geometry for the task: the original Yale OpenHand Model T42 file ``finger_pf_torsion_t42.STL``
(unmodified, CC BY-NC 3.0, see README) and a 0.5 mm solid-occupancy grid made from it.

Run ``python fixture/finger_geometry.py`` to rebuild ``occupancy.npz`` from the STL (pure NumPy, a few seconds).
All values are in the STL's own frame, in millimetres: x across the finger (pad side at high x), y along it (pivot
lug at y < 19.5), z through its 31.75 mm thickness (the pivot bore axis).
"""
from __future__ import annotations

import struct
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STL = HERE / 'finger_pf_torsion_t42.STL'
OCC = HERE / 'occupancy.npz'
STEP = 0.5  # mm, x and y
ZSTEP = 0.25  # mm


def triangles() -> np.ndarray:
    data = STL.read_bytes()
    n = struct.unpack('<I', data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return rec['v'].reshape(-1, 3, 3).astype(float)


def mesh():
    """(unique vertices (mm), triangle index rows)."""
    t = triangles().reshape(-1, 3)
    key = np.round(t, 4)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    return uniq, inv.reshape(-1, 3)


def build() -> dict:
    T = triangles()
    xs = np.arange(0, 31.0, STEP) + STEP / 2
    ys = np.arange(0, 119.0, STEP) + STEP / 2
    zc = np.arange(0, 31.75, ZSTEP) + ZSTEP / 2
    occ = np.zeros((len(xs), len(ys), len(zc)), bool)
    A, B, C = T[:, 0, :2], T[:, 1, :2], T[:, 2, :2]
    v0, v1 = B - A, C - A
    d00, d01, d11 = (v0 * v0).sum(1), (v0 * v1).sum(1), (v1 * v1).sum(1)
    den = d00 * d11 - d01 * d01
    ok = np.abs(den) > 1e-12
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            v2 = np.array([x, y]) - A
            d20, d21 = (v2 * v0).sum(1), (v2 * v1).sum(1)
            with np.errstate(all='ignore'):
                v = (d11 * d20 - d01 * d21) / den
                w = (d00 * d21 - d01 * d20) / den
            u = 1 - v - w
            m = ok & (u >= 0) & (v >= 0) & (w >= 0)
            if not m.any():
                continue
            z = u[m] * T[m, 0, 2] + v[m] * T[m, 1, 2] + w[m] * T[m, 2, 2]
            z = np.unique(np.round(np.sort(z), 4))
            for a, b in zip(z[0::2], z[1::2]):
                occ[i, j, (zc > a) & (zc < b)] = True
    return dict(occ=np.packbits(occ), shape=np.array(occ.shape), step=np.array([STEP, STEP, ZSTEP]))


_GRID = None


def grid():
    global _GRID
    if _GRID is None:
        d = np.load(OCC)
        shape = tuple(int(v) for v in d['shape'])
        occ = np.unpackbits(d['occ'])[:int(np.prod(shape))].reshape(shape).astype(bool)
        _GRID = (occ, d['step'].astype(float))
    return _GRID


def solid(points_mm) -> np.ndarray:
    """True where a point (STL frame, mm) lies inside the original solid."""
    occ, step = grid()
    p = np.asarray(points_mm, float).reshape(-1, 3)
    idx = np.floor(p / step).astype(int)
    inside = np.all((idx >= 0) & (idx < np.array(occ.shape)), axis=1)
    out = np.zeros(len(p), bool)
    i = idx[inside]
    out[inside] = occ[i[:, 0], i[:, 1], i[:, 2]]
    return out


if __name__ == '__main__':
    d = build()
    np.savez_compressed(OCC, **d)
    occ, step = grid()
    print('solid volume %.3f cm3 (mesh 30.187)' % (occ.sum() * step.prod() / 1000))
