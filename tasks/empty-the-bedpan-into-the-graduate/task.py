"""On the station table, on a blue absorbent pad, an adult plastic bedpan with urine in it: a wide, shallow, open
cream pan 27.9 cm front to back and 35.6 cm side to side, 10.2 cm tall, with rounded corners, straight walls and a
handle at one end. Which end the handle is at, and how much urine there is, changes between trials. In front of the
pad, nearer the robot, stands an empty stainless steel measuring graduate (a 1 quart measure, 13.1 cm tall, 11.7 cm
across its open top) with its handle toward the robot.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so the reset hooks run before lots move objects)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
DT = 0.05  # control step (s)
G = 9.81

CREAM, BLUE, STEEL, URINE = (0.90, 0.88, 0.80), (0.30, 0.55, 0.85), (0.72, 0.74, 0.76), (0.93, 0.78, 0.18)
DARK = (0.30, 0.31, 0.33)

# --- bedpan ------------------------------------------------------------------------------------------------------
# Part frame: origin at the centre of the underside, x front to back (27.9 cm), y side to side (35.6 cm), z up.
# The handle is at the +y end. The corners are rounded (65 mm radius, 4 flat facets each). The 6 mm walls flare in,
# 55 mm over their 96 mm height (30 degrees from upright), to a 16.9 x 24.6 cm floor.
PX, PY, PH = 0.279, 0.356, 0.102
WALL, FLOOR, CR = 0.006, 0.006, 0.065
FACETS = 4
BAND, INSET = 0.0, 0.055
HF = PH - BAND - FLOOR  # height of the flared walls
PAN_MASS = 0.145  # kg, empty
HANDLE = (0.070, 0.090, 0.024)
SEAT_T = 0.003  # the seat plate over the handle half
S2 = math.sqrt(2.0)


def _mat_quat(m):
    return parts._matrix_to_quat(np.asarray(m, float))


def _qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _outline(hx, hy, r):
    """A rounded rectangle, half sizes hx, hy, corner radius r (each corner FACETS flat facets), counter-clockwise."""
    pts = []
    for cx, cy, a0 in ((hx - r, -hy + r, -90.0), (hx - r, hy - r, 0.0), (-hx + r, hy - r, 90.0), (-hx + r, -hy + r, 180.0)):
        for k in range(FACETS + 1):
            a = math.radians(a0 + 90.0 * k / FACETS)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _inset(hx, hy, r, d):
    """The same outline moved d inward."""
    return hx - d, hy - d, max(r - d, 0.004)


def _prism_hull(poly, z0, z1, color):
    pts = [(x, y, z0) for x, y in poly] + [(x, y, z1) for x, y in poly]
    n = len(poly)
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    faces += [(k, (k + 1) % n, n + (k + 1) % n, n + k) for k in range(n)]
    return parts.hull(pts, faces, color=color)


# The inside of the pan at the rim: a chamfered rectangle WALL inside the outer one.
IN_HX, IN_HY = PX / 2 - WALL, PY / 2 - WALL
IN_R = CR - WALL


def _sides(poly):
    for k in range(len(poly)):
        a, b = np.array(poly[k]), np.array(poly[(k + 1) % len(poly)])
        e = b - a
        L = float(np.linalg.norm(e))
        yield a, b, e / L, np.array([e[1], -e[0]]) / L, L


def _pan_shapes():
    out = [_prism_hull(_outline(*_inset(IN_HX, IN_HY, IN_R, INSET - WALL)), 0.0, FLOOR, CREAM)]
    phi = math.atan(INSET / HF)
    slant = math.hypot(INSET, HF)
    for a, b, e, n, L in _sides(_outline(IN_HX, IN_HY, IN_R)):
        rot = parts._rot((0, 0, 1), math.atan2(n[1], n[0])) @ parts._rot((0, 1, 0), phi)
        mid = (a + b) / 2 - n * INSET / 2
        c = np.array([mid[0], mid[1], FLOOR + HF / 2]) + rot @ np.array([WALL / 2, 0.0, 0.0])
        out.append(parts.box((WALL, L + 0.006, slant + 0.004), at=tuple(c), color=CREAM, quat=tuple(_mat_quat(rot))))
        if BAND <= 0.0:
            continue
        bc = (a + b) / 2 + n * WALL / 2
        out.append(parts.box((WALL, L + 0.004, BAND), at=(bc[0], bc[1], PH - BAND / 2), color=CREAM,
                             quat=_qz(math.atan2(n[1], n[0]))))
    out.append(parts.box(HANDLE, at=(0.0, PY / 2 - 0.030 + HANDLE[1] / 2, PH - 0.006 - HANDLE[2] / 2), color=CREAM))
    # the seat: a 3 mm plate on the rim over the handle half (y >= 0), the outer outline's half, one convex piece
    half = [(x, y) for x, y in _outline(PX / 2, PY / 2, CR) if y >= 0.0]
    half = [(PX / 2, 0.0)] + sorted(half, key=lambda p: math.atan2(p[1], p[0])) + [(-PX / 2, 0.0)]
    out.append(_prism_hull(half, PH, PH + SEAT_T, CREAM))
    return out


# --- liquid model (pan frame) ------------------------------------------------------------------------------------
# The urine is a tracked volume with a plane free surface. The inside is cut into 4 mm columns; each column's
# bottom is the floor or, near the edge, the flared wall.
DEPTH = PH - FLOOR
CELL = 0.004
_gx = np.arange(-IN_HX + CELL / 2, IN_HX, CELL)
_gy = np.arange(-IN_HY + CELL / 2, IN_HY, CELL)
_GX, _GY = np.meshgrid(_gx, _gy, indexing="ij")
_EDGES = [(n, float(n @ a)) for a, b, e, n, L in _sides(_outline(IN_HX, IN_HY, IN_R))]


def _signed(x, y):
    """Signed distance (m, negative inside) from the inner rim outline, as the largest of its edges' half-planes."""
    return np.maximum.reduce([n[0] * x + n[1] * y - d for n, d in _EDGES])


_IN = _signed(_GX, _GY) <= 0.0
CX, CY = _GX[_IN], _GY[_IN]
_S = _signed(CX, CY)
ZB = np.clip((_S + INSET) / INSET, 0.0, 1.0) * HF  # column bottom above the floor
CELL_A = CELL * CELL
PAN_AREA = CELL_A * CX.size


def _depths(z0, gx, gy):
    return np.clip(z0 + gx * CX + gy * CY - ZB, 0.0, None)


def _vol(z0, gx, gy):
    return float(_depths(z0, gx, gy).sum() * CELL_A)


def _surface(v_m3, gx, gy):
    """The plane's offset above the floor at the pan's centre that holds v_m3 (walls taken as unbounded)."""
    span = abs(gx) * IN_HX + abs(gy) * IN_HY
    lo, hi = -span, span + HF + v_m3 / PAN_AREA + 1e-6
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if _vol(mid, gx, gy) < v_m3 else (lo, mid)
    return (lo + hi) / 2


def _rim_samples(step=0.004):
    pts, nrm, ds = [], [], []
    for a, b, e, n, L in _sides(_outline(IN_HX, IN_HY, IN_R)):
        k = int(max(1, round(L / step)))
        for i in range(k):
            pts.append(a + (b - a) * (i + 0.5) / k)
            nrm.append(n)
            ds.append(L / k)
    return np.array(pts), np.array(nrm), np.array(ds)


RIM_P, RIM_N, RIM_DS = _rim_samples()
_OPEN = RIM_P[:, 1] < 0.0  # the rim under the seat (the handle half) is closed: nothing flows out there
RIM_P, RIM_N, RIM_DS = RIM_P[_OPEN], RIM_N[_OPEN], RIM_DS[_OPEN]
CW = 0.62 * (2.0 / 3.0) * math.sqrt(2.0 * G)  # sharp-crested weir: Q = CW * h^1.5 per metre of wetted rim
ZETA = 0.15  # damping ratio of the slosh mode
SUBSTEPS = 10

# --- graduate (Vollrath 95320: 1 qt, 5.15 in tall, 4.59 in usable top, 3.90 in base, 0.68 lb) --------------------
GH, G_TOP_R, G_BASE_R = 0.131, 0.1166 / 2, 0.099 / 2
G_WALL, G_FLOOR, G_MASS = 0.003, 0.004, 0.31
G_SEGS = 18
G_TAPER = math.atan((G_TOP_R - G_BASE_R) / GH)
G_MOUTH_R = G_TOP_R - G_WALL  # inner radius at the rim
G_XY = (0.150, 0.0)
G_CAP_ML = 1000.0


def _g_inner_r(z):
    return G_BASE_R - G_WALL + (z - G_FLOOR) * math.tan(G_TAPER)


def _graduate_shapes():
    out = [parts.cylinder(G_BASE_R, G_FLOOR, at=(0.0, 0.0, G_FLOOR / 2), color=STEEL)]
    seg_w = 2 * math.pi * (G_BASE_R + G_TOP_R) / 2 / G_SEGS + 0.004
    slant = GH / math.cos(G_TAPER)
    for k in range(G_SEGS):
        a = 2 * math.pi * k / G_SEGS
        r = (G_BASE_R + G_TOP_R) / 2 - G_WALL / 2
        rot = parts._rot((0, 0, 1), a) @ parts._rot((0, 1, 0), G_TAPER)  # the segment leans outward
        out.append(parts.box((G_WALL, seg_w, slant), at=(r * math.cos(a), r * math.sin(a), GH / 2), color=STEEL,
                             quat=tuple(_mat_quat(rot))))
    hx = -(G_TOP_R + 0.030)  # handle toward the robot (-x): a bar and two arms
    out.append(parts.box((0.010, 0.016, 0.085), at=(hx, 0.0, 0.070), color=STEEL))
    out.append(parts.box((0.034, 0.016, 0.010), at=(hx + 0.017, 0.0, 0.110), color=STEEL))
    out.append(parts.box((0.034, 0.016, 0.010), at=(hx + 0.019, 0.0, 0.032), color=STEEL))
    for k in range(4):  # graduation marks on the robot's side: 250, 500, 750 and 1000 mL
        z = _level_of(250.0 * (k + 1))
        out.append(parts.box((0.002, 0.020, 0.002), at=(-(_g_inner_r(z) + G_WALL + 0.0012), 0.012, z), color=DARK,
                             collide=False))
    return out


def _vol_ml_below(h):
    """mL inside the graduate up to h above its inner floor."""
    r0, k = G_BASE_R - G_WALL, math.tan(G_TAPER)
    return 1e6 * math.pi * (r0 * r0 * h + r0 * k * h * h + k * k * h ** 3 / 3.0)


def _level_of(ml):
    lo, hi = 0.0, GH - G_FLOOR
    for _ in range(50):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if _vol_ml_below(mid) < ml else (lo, mid)
    return G_FLOOR + lo


def _disc_mesh(r, z, n=20, cx=0.0, cy=0.0):
    pts = [(cx, cy, z)] + [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n), z)
                           for i in range(n)]
    tris = tuple((0, 1 + i, 1 + (i + 1) % n) for i in range(n))
    return tuple(pts), tris


def _column_mesh(z_top, n=20):
    """The liquid in the graduate up to z_top: its surface and the side next to the wall."""
    r1, r0 = _g_inner_r(z_top) - 0.0005, _g_inner_r(G_FLOOR) - 0.0005
    pts = [(0.0, 0.0, z_top)]
    pts += [(r1 * math.cos(2 * math.pi * i / n), r1 * math.sin(2 * math.pi * i / n), z_top) for i in range(n)]
    pts += [(r0 * math.cos(2 * math.pi * i / n), r0 * math.sin(2 * math.pi * i / n), G_FLOOR + 0.0005) for i in range(n)]
    tris = [(0, 1 + i, 1 + (i + 1) % n) for i in range(n)]
    for i in range(n):
        a, b, c, d = 1 + i, 1 + (i + 1) % n, 1 + n + i, 1 + n + (i + 1) % n
        tris += [(a, c, b), (b, c, d)]
    return tuple(pts), tuple(tris)


G_LEVELS = [20.0 * (k + 1) for k in range(int(G_CAP_ML // 20))][:25]  # 20 .. 500 mL drawn
STREAM_XY = (0.010, 0.0)  # where a stream entering the mouth is drawn (graduate frame)


def _stream_mesh():
    x0, y0 = STREAM_XY
    w, d, z0, z1 = 0.012, 0.004, _level_of(10.0), GH + 0.035
    pts = [(x0 - d, y0 - w, z0), (x0 + d, y0 - w, z0), (x0 + d, y0 + w, z0), (x0 - d, y0 + w, z0),
           (x0 - d, y0 - w, z1), (x0 + d, y0 - w, z1), (x0 + d, y0 + w, z1), (x0 - d, y0 + w, z1)]
    tris = ((0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7), (4, 5, 6), (4, 6, 7))
    return tuple(pts), tris


GRAD_VIS = [parts.Visual(f"lvl{k}", *_column_mesh(_level_of(ml)), color=URINE) for k, ml in enumerate(G_LEVELS)]
GRAD_VIS.append(parts.Visual("stream", *_stream_mesh(), color=URINE))

# Liquid in the pan, drawn as a layer on the floor (level) at 12 volumes.
PAN_LEVELS = [25.0, 50.0, 75.0, 100.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0]


def _layer_mesh(ml):
    h = max(0.0015, _surface(ml * 1e-6, 0.0, 0.0))
    poly = _outline(*_inset(IN_HX, IN_HY, IN_R, INSET * max(0.0, 1.0 - h / HF) + 0.001))
    z = FLOOR + h
    pts = [(0.0, 0.0, z)] + [(x, y, z) for x, y in poly]
    n = len(poly)
    return tuple(pts), tuple((0, 1 + i, 1 + (i + 1) % n) for i in range(n))


PAN_VIS = [parts.Visual(f"pan{k}", *_layer_mesh(ml), color=URINE) for k, ml in enumerate(PAN_LEVELS)]

PAN_XY = (0.405, 0.0)
PAD = (0.39, 0.50, 0.002)
PAD_XY = (0.405, 0.0)

# Spilled urine is drawn as puddles on a 50 mm grid over the table in front of the pad and the pad itself.
PUDDLE_X = [0.03 + 0.05 * i for i in range(8)]
PUDDLE_Y = [-0.225 + 0.05 * j for j in range(10)]
PUDDLE_VIS = [parts.Visual(f"pud{i}_{j}", *_disc_mesh(0.022, 0.0035, cx=x - PAD_XY[0], cy=y - PAD_XY[1]),
                           color=URINE) for i, x in enumerate(PUDDLE_X) for j, y in enumerate(PUDDLE_Y)]

BEDPAN = parts.Part("bedpan", pos=(*PAN_XY, TOP + PAD[2]), mass=PAN_MASS, shapes=_pan_shapes(), visuals=PAN_VIS,
                    friction=(1.2, 1.0))
GRADUATE = parts.Part("graduate", pos=(*G_XY, TOP), mass=G_MASS, shapes=_graduate_shapes(), visuals=GRAD_VIS,
                      friction=(0.5, 0.4))
UNDERPAD = parts.Part("underpad", pos=(*PAD_XY, TOP), shapes=[parts.box(PAD, at=(0.0, 0.0, PAD[2] / 2), color=BLUE)],
                      visuals=PUDDLE_VIS)
OBJECTS = [UNDERPAD, BEDPAN, GRADUATE]

# Lots: the handle at the robot's left (A, B's mirror image) and the urine's volume.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", volume=250.0, handle="left", place={"bedpan": (*PAN_XY, TOP + PAD[2])}),
    lots.Lot("B", volume=150.0, handle="right", yaw={"bedpan": math.pi}),
    lots.Lot("C", volume=400.0, handle="right", yaw={"bedpan": math.pi}),
)

# --- pan geometry helpers ----------------------------------------------------------------------------------------


def _R(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def _tilt_deg(q) -> float:
    return tilt.tilt_of(q)


def _pan_com() -> np.ndarray:
    """The empty pan's centre of mass (part frame), from its shapes' volumes (as PhysX computes it)."""
    tot, acc = 0.0, np.zeros(3)
    for s in BEDPAN.shapes:
        v = float(np.prod(2 * s.half))
        tot += v
        acc += v * np.asarray(s.at)
    return acc / tot


PAN_COM = _pan_com()

# --- the liquid's state ------------------------------------------------------------------------------------------
# One record per process, shared by every copy of this module that gets imported (the simulator's and any checker's).
_L: dict = sys.modules.setdefault("_bedpan_liquid_record", type(sys)("_bedpan_liquid_record")).__dict__.setdefault(
    "state", {})


def reset_state(volume_ml: float | None = None) -> None:
    v = lots.current().volume if volume_ml is None else volume_ml
    _L.update(v0=float(v), pan=float(v), grad=0.0, spill=0.0, u=np.zeros(2), ud=np.zeros(2), vel=None,
              acc=np.zeros(3), inflow=0.0, spill_cells=set(), known=False, clear=False, hold=0, steps=0)


def graduate_mouth(grad):
    R = _R(grad.quat_xyzw)
    return np.asarray(grad.pos, float) + R @ np.array([0.0, 0.0, GH]), R


def _caught(points, out_dirs, speeds, grad):
    """Which outflow points' streams land inside the graduate's mouth. Each stream leaves the rim horizontally
    outward at `speed` and falls freely to the plane of the mouth."""
    mouth, R = graduate_mouth(grad)
    if _tilt_deg(grad.quat_xyzw) > 20.0:
        return np.zeros(len(points), bool), points[:, :2]
    dz = points[:, 2] - mouth[2]
    t = np.sqrt(np.clip(2.0 * dz / G, 0.0, None))
    land = points[:, :2] + out_dirs * (speeds * t)[:, None]
    inside = np.linalg.norm(land - mouth[:2], axis=1) <= G_MOUTH_R - 0.003
    return inside & (dz > 0.0), land


def _note_spill(ml, xy):
    for (x, y) in np.atleast_2d(xy):
        i = int(round((x - PUDDLE_X[0]) / 0.05))
        j = int(round((y - PUDDLE_Y[0]) / 0.05))
        if 0 <= i < len(PUDDLE_X) and 0 <= j < len(PUDDLE_Y):
            _L["spill_cells"].add((i, j))
    _L["spill"] += ml


def step_liquid(objects, dt: float = DT) -> None:
    """Advance the urine by one control step: slosh, outflow over the rim, where each stream lands, and what the
    graduate holds."""
    pan, grad = objects["bedpan"], objects["graduate"]
    R = _R(pan.quat_xyzw)
    pos = np.asarray(pan.pos, float)
    # pan acceleration (filtered), for the effective gravity the surface settles to
    vel = np.asarray(pan.lin_vel, float)
    if _L["vel"] is not None:
        a = np.clip((vel - _L["vel"]) / dt, -5.0, 5.0)
        _L["acc"] = _L["acc"] + (a - _L["acc"]) * min(1.0, dt / 0.1)
    _L["vel"] = vel
    up = np.array([0.0, 0.0, G]) + _L["acc"]
    n_eq = R.T @ (up / np.linalg.norm(up))  # the level surface's normal, pan frame
    _L["inflow"] = 0.0
    h = dt / SUBSTEPS
    for _ in range(SUBSTEPS):
        v = _L["pan"] * 1e-6
        if v <= 1e-9:
            _L["pan"] = 0.0
            break
        if n_eq[2] < 0.12:  # the pan is on its side or upside down: everything left runs off the whole rim at once
            pts = pos + (R @ np.c_[RIM_P, np.full(len(RIM_P), PH)].T).T
            ok, land = _caught(pts, np.zeros((len(pts), 2)), np.zeros(len(pts)), grad)
            share = RIM_DS / RIM_DS.sum() * _L["pan"]
            _L["grad"] += float(share[ok].sum())
            _L["inflow"] += float(share[ok].sum())
            if (~ok).any():
                _note_spill(float(share[~ok].sum()), land[~ok][::10])
            _L["pan"] = 0.0
            break
        # slosh: the surface's tilt relative to the pan (horizontal parts of its normal) is a damped oscillator
        # about the level surface, with the first sloshing mode's frequency for the liquid's current footprint
        gx0, gy0 = -_L["u"] / max(math.sqrt(max(1.0 - float(_L["u"] @ _L["u"]), 0.0)), 0.1)
        z0 = _surface(v, gx0, gy0)
        d = _depths(z0, gx0, gy0)
        wet = max(float((d > 0).sum()) * CELL_A, 4 * CELL_A)
        hbar = max(v / wet, 0.002)
        omega = math.pi * math.sqrt(G * hbar) / max(math.sqrt(wet), 0.05)
        u_eq = n_eq[:2]
        acc = -omega * omega * (_L["u"] - u_eq) - 2.0 * ZETA * omega * _L["ud"]
        _L["ud"] = _L["ud"] + acc * h
        _L["u"] = _L["u"] + _L["ud"] * h
        nrm = float(np.linalg.norm(_L["u"]))
        if nrm > 0.99:
            _L["u"] *= 0.99 / nrm
        nz = math.sqrt(1.0 - float(_L["u"] @ _L["u"]))
        gx, gy = -_L["u"] / nz
        z0 = _surface(v, gx, gy)
        head = z0 + gx * RIM_P[:, 0] + gy * RIM_P[:, 1] - DEPTH
        wet_rim = head > 0.0
        if not wet_rim.any():
            continue
        hv = head[wet_rim] * nz
        q = CW * hv ** 1.5 * RIM_DS[wet_rim]
        z_rim = DEPTH - float((gx * RIM_P[:, 0] + gy * RIM_P[:, 1]).max())
        excess = v - _vol(z_rim, gx, gy)
        dv = min(float(q.sum()) * h, max(excess, 0.0))
        if dv <= 0.0:
            continue
        share = q / q.sum() * dv * 1e6  # mL from each wetted rim point
        pts = pos + (R @ np.c_[RIM_P[wet_rim], np.full(wet_rim.sum(), PH)].T).T
        out = (R @ np.c_[RIM_N[wet_rim], np.zeros(wet_rim.sum())].T).T[:, :2]
        out /= np.maximum(np.linalg.norm(out, axis=1), 1e-9)[:, None]
        ok, land = _caught(pts, out, (2.0 / 3.0) * np.sqrt(2.0 * G * hv), grad)
        _L["pan"] -= float(share.sum())
        _L["grad"] += float(share[ok].sum())
        _L["inflow"] += float(share[ok].sum())
        if (~ok).any():
            _note_spill(float(share[~ok].sum()), land[~ok][share[~ok] > 0.05] if (share[~ok] > 0.05).any() else
                        land[~ok][:1])
    # the graduate keeps what fits below its lowest rim point when it leans
    gt = math.radians(_tilt_deg(grad.quat_xyzw))
    if gt > 1e-3 and _L["grad"] > 0.0:
        keep = _vol_ml_below(max(0.0, GH - G_FLOOR - G_MOUTH_R * math.tan(min(gt, 1.5)))) if gt < 1.5 else 0.0
        if _L["grad"] > keep:
            _note_spill(_L["grad"] - keep, np.asarray(grad.pos, float)[None, :2])
            _L["grad"] = keep
    _L["steps"] += 1


def pan_liquid_force(objects):
    """The urine's weight on the pan (N, pan frame) and its torque about the pan's centre of mass (N m)."""
    pan = objects["bedpan"]
    v = _L["pan"] * 1e-6
    if v <= 1e-9:
        return np.zeros(3), np.zeros(3)
    R = _R(pan.quat_xyzw)
    n = R.T @ np.array([0.0, 0.0, 1.0])
    gx, gy = (-n[0] / n[2], -n[1] / n[2]) if n[2] > 0.12 else (0.0, 0.0)
    z0 = _surface(v, gx, gy)
    d = _depths(z0, gx, gy)
    tot = max(float(d.sum()), 1e-12)
    c = np.array([float((d * CX).sum()) / tot, float((d * CY).sum()) / tot,
                  FLOOR + float((d * (ZB + d / 2)).sum()) / tot])
    f = R.T @ np.array([0.0, 0.0, -v * 1000.0 * G])
    return f, np.cross(c - PAN_COM, f)


# --- end state ---------------------------------------------------------------------------------------------------
UPRIGHT = 5.0  # degrees
ON_SURFACE = 0.003  # m: underside within this of the surface it stands on
SPOT = 0.040  # m: the pan's centre within this of where it started
SPOT_YAW = math.radians(15.0)
HOLD_STEPS = 20  # 1 s at 20 Hz
FULL_IN, HALF_IN, SPILL_MAX = 0.95, 0.50, 0.02


def _yaw(q):
    R = _R(q)
    return math.atan2(R[1, 0], R[0, 0])


def _graduate_standing(grad) -> bool:
    return bool(_tilt_deg(grad.quat_xyzw) <= UPRIGHT + EPS and abs(float(grad.pos[2]) - TOP) <= ON_SURFACE + EPS)


def _pan_on_spot(pan) -> bool:
    lot = lots.current()
    sx, sy, sz = lot.pos("bedpan")
    dyaw = (_yaw(pan.quat_xyzw) - lot.yaw_of("bedpan") + math.pi / 2) % math.pi - math.pi / 2  # either way round
    return bool(_tilt_deg(pan.quat_xyzw) <= UPRIGHT + EPS and abs(float(pan.pos[2]) - sz) <= ON_SURFACE + EPS
                and math.hypot(float(pan.pos[0]) - sx, float(pan.pos[1]) - sy) <= SPOT + EPS
                and abs(dyaw) <= SPOT_YAW + EPS)


def _transfer_ok(frac_in) -> bool:
    return bool(_L["known"] and _L["spill"] < SPILL_MAX * _L["v0"] - EPS and _L["grad"] >= frac_in * _L["v0"] - EPS)


def _done(objects) -> bool:
    pan, grad = objects["bedpan"], objects["graduate"]
    return bool(_transfer_ok(FULL_IN) and _L["clear"] and _graduate_standing(grad) and _pan_on_spot(pan)
                and pan.resting() and grad.resting())


def record(objects, clear: bool, dt: float = DT) -> None:
    """One control step: the urine, whether every finger and gripper is clear of the pan and the graduate, and how
    long the finished state has held."""
    _L["known"] = True
    step_liquid(objects, dt)
    _L["clear"] = bool(clear)
    _L["hold"] = _L["hold"] + 1 if _done(objects) else 0


if "v0" not in _L:
    reset_state()


def score(objects):
    """1.0 when at least 95% of the urine is in the graduate, less than 2% was ever spilled, the graduate stands
    upright on the table, the pan sits flat on its spot on the pad, nothing is held and all has been still for 1 s.
    0.5: the same transfer (95% in, under 2% spilled) without that end state. 0.25: at least 50% in the graduate with
    under 2% spilled. 0 once 2% has been spilled, or if the hands' state is unknown."""
    if not _L["known"]:
        return 0.0
    if _L["hold"] >= HOLD_STEPS and _done(objects):
        return 1.0
    if _transfer_ok(FULL_IN):
        return 0.5
    if _transfer_ok(HALF_IN):
        return 0.25
    return 0.0


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    return float(max(0.0, _L["v0"] - _L["grad"]) / max(_L["v0"], 1.0))


# --- simulator hooks ---------------------------------------------------------------------------------------------
HAND_LINKS = ("tip_left", "tip_right", "gripper")
CLEAR = 0.005


SAMPLE = 0.003  # m between surface sample points on the hand's collision faces
_HAND_PTS: dict = {}


def hand_points(link: str) -> np.ndarray:
    """Points on a hand link's collision surfaces (link frame): every vertex plus each triangle face sampled about every
    3 mm, so a flat face resting on an object is seen even when no vertex is near it. Built once and cached."""
    if link not in _HAND_PTS:
        import xml.etree.ElementTree as ET

        from yam_arena import ASSET_DIR, YAM_URDF, analysis
        from yam_arena.urdf import rpy_matrix

        pts = []
        for lk in ET.parse(YAM_URDF).getroot().iter("link"):
            if lk.get("name") != link:
                continue
            for col in lk.findall("collision"):
                origin = col.find("origin")
                rot, off = np.eye(3), np.zeros(3)
                if origin is not None:
                    rot = rpy_matrix(np.array([float(v) for v in origin.get("rpy", "0 0 0").split()]))
                    off = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
                tris = analysis.read_stl(ASSET_DIR / col.find("geometry/mesh").get("filename")) @ rot.T + off
                pts.append(sample_triangles(tris))
        _HAND_PTS[link] = np.concatenate(pts) if pts else np.zeros((0, 3))
    return _HAND_PTS[link]


def sample_triangles(tris) -> np.ndarray:
    """Each triangle's corners plus points across its face about every 3 mm, kept one per 1.5 mm cell."""
    pts = []
    for t in np.asarray(tris, float):
        edge = max(np.linalg.norm(t[1] - t[0]), np.linalg.norm(t[2] - t[1]), np.linalg.norm(t[0] - t[2]))
        n = max(1, int(math.ceil(edge / SAMPLE)))
        for i in range(n + 1):
            for j in range(n + 1 - i):
                pts.append(t[0] + (t[1] - t[0]) * i / n + (t[2] - t[0]) * j / n)
    return np.unique(np.round(np.array(pts) / 0.0015), axis=0) * 0.0015


def hands_clear_of(link_poses, st) -> bool:
    """True when every sampled point of every hand link (4x4 world poses by link name) is more than 5 mm from every
    shape of the pan and the graduate (their boxes; the floor counts as its bounding box)."""
    from yam_arena import analysis

    pts = [hand_points(n) @ m[:3, :3].T + m[:3, 3] for n, m in link_poses]
    pts = [p for p in pts if len(p)]
    if not pts:
        return False
    cloud = np.concatenate(pts)
    lo, hi = cloud.min(axis=0), cloud.max(axis=0)
    for name, spec in (("bedpan", BEDPAN), ("graduate", GRADUATE)):
        s = st[name]
        for _, c, r, size in spec.world_boxes({"base": (np.asarray(s.pos), _R(s.quat_xyzw))}):
            c = np.asarray(c)
            reach = 0.5 * float(np.linalg.norm(size)) + CLEAR + 0.001
            if np.any(c < lo - reach) or np.any(c > hi + reach):
                continue  # this shape is far from every hand point
            if analysis.box_signed_distance(cloud, c, np.asarray(r), np.asarray(size)).min() <= CLEAR + EPS:
                return False
    return True


def _hands_clear(sim, st) -> bool:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    poses_out = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in HAND_LINKS:
                m = np.eye(4)
                m[:3, :3] = _R(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                poses_out.append((n, m))
    if len(poses_out) < 2 * len(HAND_LINKS):
        return False
    return hands_clear_of(poses_out, st)


_SHOWN: dict = {}


def _show(sim, part, names_on) -> None:
    shown = _SHOWN.setdefault(part, set())
    for n in shown - names_on:
        sim.set_visible(part, n, False)
    for n in names_on - shown:
        sim.set_visible(part, n, True)
    _SHOWN[part] = set(names_on)


def _draw(sim) -> None:
    k = max((i for i, ml in enumerate(PAN_LEVELS) if _L["pan"] >= ml - 12.5), default=None)
    _show(sim, "bedpan", {f"pan{k}"} if k is not None and _L["pan"] > 5.0 else set())
    g = max((i for i, ml in enumerate(G_LEVELS) if _L["grad"] >= ml - 10.0), default=None)
    on = {f"lvl{g}"} if g is not None and _L["grad"] > 5.0 else set()
    if _L["inflow"] > 0.02:
        on.add("stream")
    _show(sim, "graduate", on)
    _show(sim, "underpad", {f"pud{i}_{j}" for i, j in _L["spill_cells"]})


@parts.on_reset
def _on_reset(sim) -> None:
    reset_state()
    _L["last_t"] = None
    _SHOWN.clear()
    for part, vis in (("bedpan", PAN_VIS), ("graduate", GRAD_VIS), ("underpad", PUDDLE_VIS)):
        for v in vis:
            sim.set_visible(part, v.name, False)
    _draw(sim)
    try:
        sim.set_wrench("bedpan", *pan_liquid_force(sim.states()))
    except Exception:
        pass


@parts.on_step
def _on_step(sim) -> None:
    if _L.get("last_t") == sim.t:  # a second copy of this module sharing the record: step it once
        return
    _L["last_t"] = sim.t
    st = sim.states()
    clear = False  # only measured when the rest of the finished state holds; unknown counts as not clear
    if _transfer_ok(FULL_IN) and _graduate_standing(st["graduate"]) and _pan_on_spot(st["bedpan"]):
        try:
            clear = _hands_clear(sim, st)
        except Exception:  # unknown hand state: no credit
            clear = False
    record(st, clear)
    if os.environ.get("TASK_DEBUG") and _L["steps"] % 20 == 0:
        print(f"[liquid] t={_L['steps'] * DT:.1f} pan={_L['pan']:.1f} grad={_L['grad']:.1f} spill={_L['spill']:.2f} "
              f"clear={clear} hold={_L['hold']} pan_tilt={_tilt_deg(st['bedpan'].quat_xyzw):.1f}", flush=True)
    _draw(sim)
    sim.set_wrench("bedpan", *pan_liquid_force(st))


# --- demo: both arms ---------------------------------------------------------------------------------------------
GRIP_A = 0.125  # m from the pouring corner's (square) apex along each wall to the grip
TIP = 0.013
POUR_TILT = 60.0  # degrees
D_POUR = np.array([-1.0, 0.0, 0.0])  # the pouring corner points toward the robot


def _corner(sx):
    """The pouring corner (part frame): the middle of the chamfer's inner rim edge, front corner of the open end."""
    c = np.array([sx * (IN_HX - IN_R), -(IN_HY - IN_R)])
    p = c + IN_R * np.array([sx, -1.0]) / S2
    return np.array([p[0], p[1], PH])


FLARE = math.atan(INSET / HF)  # the walls lean out this far from vertical
GRIP_DOWN = 0.017  # m from the rim down the wall to the TCP: the pads hold the top 30 mm of the wall


def _grips(sx):
    """The two grips (part frame): TCP point, the outward normal of the wall it pinches, and the yaw (part frame) of
    a straight-down grip closing across that wall. The grip leans with the wall so the pads lie flat on it."""
    out = []
    for n, along in ((np.array([sx, 0.0]), np.array([0.0, -PY / 2 + GRIP_A])),
                     (np.array([0.0, -1.0]), np.array([sx * (PX / 2 - GRIP_A), 0.0]))):
        rim = np.where(n != 0, n * (np.array([IN_HX, IN_HY]) + WALL / 2), along)  # the wall's mid-surface at the rim
        down = np.array([-n[0] * math.sin(FLARE), -n[1] * math.sin(FLARE), -math.cos(FLARE)])
        tcp = np.array([rim[0], rim[1], PH]) + down * GRIP_DOWN
        yl = math.pi / 2 if n[0] != 0 else 0.0
        out.append((tcp, n, yl, down))
    return out


def _pan_rot(psi, theta_deg):
    return parts._rot(np.cross([0.0, 0.0, 1.0], D_POUR), math.radians(theta_deg)) @ parts._rot((0, 0, 1), psi)


def _plan_poses():
    """Pan keyframes (duration s, origin, rotation, finger) from the start layout the cameras show: which end the
    handle is at sets the pouring corner."""
    lot = lots.current()
    psi0 = lot.yaw_of("bedpan")
    x0, y0, z0 = lot.pos("bedpan")
    sx = -1.0 if abs(psi0) < 1.0 else 1.0  # the front wall in the part frame
    c_loc = _corner(sx)
    n_c = np.array([sx, -1.0]) / S2
    psi_p = math.pi - math.atan2(n_c[1], n_c[0])
    psi_p = psi0 + ((psi_p - psi0 + math.pi) % (2 * math.pi) - math.pi)
    o0 = np.array([x0, y0, z0])
    frames = [(0.0, o0, _pan_rot(psi0, 0.0))]

    def at_corner(cw, psi, th):
        R = _pan_rot(psi, th)
        return np.asarray(cw, float) - R @ c_loc, R

    lift = o0 + np.array([0.0, 0.0, 0.035])
    frames.append((2.5, lift, _pan_rot(psi0, 0.0)))
    c_lift = lift + _pan_rot(psi0, 0.0) @ c_loc
    gx, gy = G_XY
    # carry and turn: the corner to just behind the graduate, high enough for the pan to pass over its rim
    pre = np.array([gx + 0.11, gy, TOP + 0.185])
    n = 6
    for k in range(1, n + 1):
        f = k / n
        cw = c_lift + (pre - c_lift) * f
        cw[2] = max(cw[2], c_lift[2])
        frames.append((1.2, *at_corner(cw, psi0 + (psi_p - psi0) * f, 0.0)))
    # tip to 24 degrees behind the graduate (nothing runs out yet), bring the corner over the mouth, then tip on
    # slowly about the corner while lowering it, keeping the pan clear of the graduate's rim
    tilt_pts = [(10.0, 0.11, 0.182, 1.5), (20.0, 0.10, 0.177, 1.5), (24.0, 0.07, 0.173, 1.5), (26.0, 0.018, 0.172, 1.5),
                (30.0, 0.018, 0.169, 2.0), (36.0, 0.018, 0.165, 2.5), (42.0, 0.018, 0.161, 2.5),
                (48.0, 0.018, 0.158, 2.5), (54.0, 0.018, 0.156, 2.5), (60.0, 0.018, 0.154, 2.5),
                (POUR_TILT, 0.018, 0.152, 3.0), (POUR_TILT, 0.018, 0.152, 5.0)]
    for th, cx, cz, du in tilt_pts:
        frames.append((du, *at_corner((gx + cx, gy, TOP + cz), psi_p, th)))
    back = [(56.0, 0.018, 0.157), (44.0, 0.04, 0.162), (32.0, 0.05, 0.170), (22.0, 0.08, 0.178), (10.0, 0.11, 0.185),
            (0.0, 0.11, 0.190)]
    for th, cx, cz in back:
        frames.append((1.5, *at_corner((gx + cx, gy, TOP + cz), psi_p, th)))
    c_pre = frames[-1][1] + frames[-1][2] @ c_loc
    for k in range(1, n + 1):
        f = k / n
        cw = c_pre + (c_lift - c_pre) * f
        frames.append((1.2, *at_corner(cw, psi_p + (psi0 - psi_p) * f, 0.0)))
    frames.append((2.0, o0 + np.array([0.0, 0.0, 0.006]), _pan_rot(psi0, 0.0)))
    return frames, sx


def demo():
    """Both arms: pinch the pan's front wall and its open end's wall near the front corner at the open end, lift it,
    turn it so that corner points toward the graduate, tip it slowly about that corner until it is empty, tip it
    back, set it down where it was and let go."""
    frames, sx = _plan_poses()
    psi0 = math.atan2(frames[0][2][1, 0], frames[0][2][0, 0])
    Rz0 = parts._rot((0, 0, 1), psi0)
    tracks = []
    for g, n, yl, down in _grips(sx):
        tracks.append(([(du, o + R @ g) for du, o, R in frames], [R for _, _, R in frames], n, yl, down))
    # the grip further to the robot's left goes to the left arm
    left_first = tracks[0][0][0][1][1] > tracks[1][0][0][1][1]
    arms = {"left": tracks[0] if left_first else tracks[1], "right": tracks[1] if left_first else tracks[0]}
    plans = {}
    for arm, (pts, rots, n, yl, down) in arms.items():
        yaw = psi0 + yl
        yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2  # the gripper is symmetric: keep the yaw within +-90 deg
        if arm == "right" and abs(yaw + math.pi / 2) < 1e-6:
            yaw = math.pi / 2  # the right arm reaches a crosswise grip left of the middle this way round
        inward = Rz0 @ np.array([-n[0], -n[1], 0.0])
        lean = parts._rot(np.cross([0.0, 0.0, -1.0], inward), FLARE) @ grip_rotation(yaw)  # at the start, world
        d0 = Rz0 @ down

        def wp(du, pos, R, finger, label):
            rot = R @ Rz0.T @ lean
            return Waypoint(du, tuple(float(v) for v in pos), finger, label, tilt._TiltedYaw(yaw, rot))

        p0, pe, Re = pts[0][1], pts[-1][1], rots[-1]
        back = -d0 * 0.035  # back up the wall, clear of the rim
        w = [wp(2.0, p0 + back + np.array([0.0, 0.0, 0.01]), rots[0], FINGER_OPEN, "over the wall"),
             wp(1.5, p0, rots[0], FINGER_OPEN, "down astride the wall"),
             wp(1.0, p0, rots[0], FINGER_CLOSED, "pinch")]
        for k, ((du, p), R) in enumerate(zip(pts[1:], rots[1:]), 1):
            w.append(wp(du, p, R, FINGER_CLOSED, f"pan pose {k}"))
        w.append(wp(1.0, pe, Re, FINGER_OPEN, "let go"))
        up = pe + back + np.array([0.0, 0.0, 0.07])
        # the front-wall hand moves off toward its own side, away from the graduate
        aside = up + (np.array([0.0, 0.12 if arm == "left" else -0.12, 0.0]) if n[0] != 0 else 0.0)
        w.append(wp(1.5, up, Re, FINGER_OPEN, "up"))
        w.append(wp(1.5, aside, Re, FINGER_OPEN, "aside"))
        w.append(wp(3.0, aside, Re, FINGER_OPEN, "wait"))
        plans[arm] = w
    return plans
