"""Check the casting files against task.py: run with the simulator's Python (it imports task.py).

    python specimen/check_specimen.py

Checks: the skin constants in make_mould.py equal task.py's; pins.csv equals task.py's PIN_DIRS at the skin and
tip radii; every STL is a closed, consistently oriented surface; the master, the cast-skin reference and the
silicone-mould reference each carry exactly 532 pin tips (or pin holes), each on task.py's tip point; the skin's
key dimensions; MANIFEST.sha256. Prints one line per check and PASS or FAIL at the end.
"""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
TASK_DIR = HERE.parent
fails: list[str] = []


def ok(msg):
    print(f"  ok   {msg}")


def bad(msg):
    fails.append(msg)
    print(f"  FAIL {msg}")


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def read_stl(path):
    b = path.read_bytes()
    n = int(np.frombuffer(b[80:84], "<u4")[0])
    rec = np.frombuffer(b[84:84 + 50 * n], dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    tri = rec["v"].astype(np.float64)
    verts, idx = np.unique(tri.reshape(-1, 3), axis=0, return_inverse=True)
    return verts, idx.reshape(-1, 3)


def closed_and_oriented(f):
    """Every directed edge appears once and its reverse once: a closed 2-manifold with consistent winding."""
    e = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    if (e[:, 0] == e[:, 1]).any():
        return False, "degenerate edge"
    key = e[:, 0] * (f.max() + 1) + e[:, 1]
    rkey = e[:, 1] * (f.max() + 1) + e[:, 0]
    u, c = np.unique(key, return_counts=True)
    if (c != 1).any():
        return False, f"{int((c != 1).sum())} directed edges repeated"
    if not np.isin(rkey, u).all():
        return False, f"{int((~np.isin(rkey, u)).sum())} open edges"
    return True, ""


def volume(v, f):
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def tip_clusters(v, f, r_lo, r_hi):
    """Groups of triangles whose three corners all lie between r_lo and r_hi from the dome's centre, below the
    equator: the flat pin-tip faces (or pin-hole floors). Returns each group's centroid."""
    r = np.linalg.norm(v, axis=1)
    sel = (r >= r_lo) & (r <= r_hi) & (v[:, 2] < 0.5)
    faces = f[sel[f].all(axis=1)]
    parent = np.arange(len(v))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b, c in faces:
        ra, rb, rc = find(a), find(b), find(c)
        parent[rb] = ra
        parent[find(rc)] = ra
    used = np.unique(faces)
    roots = np.array([find(x) for x in used])
    return np.array([v[used[roots == g]].mean(axis=0) for g in np.unique(roots)])


def main():
    print("check_specimen: casting files vs task.py")
    task = load(TASK_DIR / "task.py", "task_under_check")
    mm = load(HERE / "make_mould.py", "make_mould_under_check")

    # 1. constants and layout
    for k in ("R_FORMER", "R_SKIN", "PIN_L", "PIN_D", "R_TIP", "N_PINS", "SKIRT_TOP"):
        (ok if getattr(task, k) == getattr(mm, k) else bad)(f"{k} = {getattr(task, k)} in task.py and make_mould.py")
    if np.array_equal(task.PIN_DIRS, mm.PIN_DIRS) and np.array_equal(task.PIN_RING, mm.PIN_RING):
        ok("make_mould.py pin directions and rings identical to task.py (bitwise)")
    else:
        bad("make_mould.py pin layout differs from task.py")
    counts = [int((task.PIN_RING == k).sum()) for k in range(len(task.RING_COUNT))]
    ok(f"ring counts {counts} (sum {sum(counts)}) at {task.RING_DEG} degrees from the apex") \
        if sum(counts) == 532 and counts == list(task.RING_COUNT) else bad(f"ring counts {counts}")
    scad = (TASK_DIR / "fixture" / "painting_former.scad").read_text()
    (ok if "R_FORMER = 19.0" in scad and "BODY_TOP = 25.0" in scad and task.R_FORMER == 0.019
     and task.SKIRT_TOP == 0.025 else bad)("painting former: r 19 mm dome and body to z 25 mm = the core's and "
                                           "the skin's inside")

    # 2. pins.csv
    rows = list(csv.DictReader((HERE / "pins.csv").open()))
    root = np.array([[float(r[f"root_{a}_mm"]) for a in "xyz"] for r in rows])
    tip = np.array([[float(r[f"tip_{a}_mm"]) for a in "xyz"] for r in rows])
    ring = np.array([int(r["ring"]) for r in rows])
    e_root = np.abs(root - task.PIN_DIRS * task.R_SKIN * 1e3).max()
    e_tip = np.abs(tip - task.PIN_DIRS * task.R_TIP * 1e3).max()
    (ok if len(rows) == 532 and e_root <= 1e-6 and e_tip <= 1e-6 and np.array_equal(ring, task.PIN_RING) else bad)(
        f"pins.csv: {len(rows)} pins, max error vs task.py root {e_root:.1e} mm, tip {e_tip:.1e} mm, rings equal")

    # 3. meshes
    geo = json.loads((HERE / "geometry.json").read_text())
    tips_mm = task.PIN_DIRS * task.R_TIP * 1e3
    rt = task.R_TIP * 1e3
    meshes = {}
    for name in ("master", "core", "box_tube", "box_base", "silicone_mould_reference", "skin_reference"):
        v, f = read_stl(HERE / f"{name}.stl")
        meshes[name] = (v, f)
        closed, why = closed_and_oriented(f)
        vol = volume(v, f)
        good = closed and vol > 0 and abs(vol - geo["volume"][name]) <= 1e-3 * geo["volume"][name]
        (ok if good else bad)(f"{name}.stl: {len(f)} triangles, closed and consistently oriented: {closed} {why}; "
                              f"volume {vol:.1f} mm3 (outward normals)")
    for name in ("master", "skin_reference", "silicone_mould_reference"):
        v, f = meshes[name]
        cen = tip_clusters(v, f, rt - 0.02, rt + 0.02)
        d = np.linalg.norm(cen[:, None, :] - tips_mm[None, :, :], axis=2)
        nearest = d.argmin(axis=1)
        worst = d.min(axis=1).max() if len(cen) else float("inf")
        good = len(cen) == 532 and len(set(nearest.tolist())) == 532 and worst <= 0.01
        (ok if good else bad)(f"{name}.stl: {len(cen)} flat pin-tip faces at r {rt:.1f} mm, each on its own "
                              f"task.py tip point (worst {worst:.4f} mm)")

    # 4. the cast skin's dimensions (skin_reference = mould cavity minus seated core)
    v, f = meshes["skin_reference"]
    rxy = np.hypot(v[:, 0], v[:, 1])
    r3 = np.linalg.norm(v, axis=1)
    skirt = (v[:, 2] > -0.01) & (v[:, 2] < 23.5)  # the skirt's facet rings lie at z 0 and the rim
    dome = (v[:, 2] < -1.0) & (r3 < 20.5)
    checks = [
        ("skirt outside radius", rxy[skirt].max(), task.R_SKIN * 1e3),
        ("skirt inside radius", rxy[skirt].min(), task.R_FORMER * 1e3),
        ("dome outside radius (between pins)", r3[dome].max(), task.R_SKIN * 1e3),
        ("dome inside radius", r3[dome].min(), task.R_FORMER * 1e3),
        ("rim height above the dome's centre", v[:, 2].max(), task.SKIRT_TOP * 1e3),
        ("apex pin tip below the dome's centre", -v[:, 2].min(), task.R_TIP * 1e3),
    ]
    for label, got, want in checks:
        (ok if abs(got - want) <= 0.01 else bad)(f"skin: {label} {got:.3f} mm (task.py {want:.3f})")
    k87 = task.PIN_RING == task.PIN_RING.max()
    p = tips_mm[k87]
    across = np.linalg.norm(p[:, None] - p[None], axis=2).max()
    ok(f"skin: across opposite tips of the {task.RING_DEG[-1]:g} degree ring {across:.2f} mm "
       f"(2 x {rt:.0f} x sin 87 = {2 * rt * math.sin(math.radians(87)):.2f})")
    mass = geo["volume"]["skin_reference"] * 1.04e-3
    ok(f"skin: volume {geo['volume']['skin_reference']:.0f} mm3, {mass:.2f} g at VytaFlex 60's 1.04 g/cm3 "
       f"(analytic {geo['skin_volume_analytic_no_notch']} mm3 without the rim notch)")
    vc, fc = meshes["core"]
    low = vc[:, 2] < task.SKIRT_TOP * 1e3 - 0.01
    (ok if abs(np.linalg.norm(vc[low & (vc[:, 2] < 0)], axis=1).max() - 19.0) < 0.01
     and abs(np.hypot(vc[low, 0], vc[low, 1]).max() - 19.0) < 0.01 else bad)(
        "core: r 19 mm dome and body below the rim, the painting former's size")

    # 5. manifest
    man = HERE / "MANIFEST.sha256"
    if man.exists():
        bad_lines = []
        for line in man.read_text().splitlines():
            h, name = line.split(maxsplit=1)
            if hashlib.sha256((HERE / name.strip()).read_bytes()).hexdigest() != h:
                bad_lines.append(name)
        (ok if not bad_lines else bad)(f"MANIFEST.sha256: {len(man.read_text().splitlines())} files, "
                                       f"mismatches {bad_lines}")
    else:
        bad("MANIFEST.sha256 missing")
    print("PASS" if not fails else f"FAIL ({len(fails)})")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
