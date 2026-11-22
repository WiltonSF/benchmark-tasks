"""A four-light wooden window sash lies flat on a decorating bench, putty side up: four panes of glass held by
pale putty bevels inside a grey-primed pine frame. Its near rail runs left-right across the front, closest to the
arms. Along the rail, the strip beside one of the two near panes is already painted dark green; the strip beside the
other near pane is bare primer. On each side of the sash stand an open quart paint can three-quarters full of dark
green paint and a notched pine brush rest with a 2 in angled sash brush lying across it, bristles in the air.
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.5
EPS = 1e-9
MM = 0.001

# ---- sash: 20 x 25 in four-light pine sash, 8 x 10 in lights, 1-1/8 in thick, putty glazed ----------------------
XS = 0.120                      # near (bottom) rail outer edge, world x
RAIL = 0.0857                   # bottom rail, outer edge to sight line (3-3/8 in)
STILE = 0.0603                  # stiles and top rail (2-3/8 in)
MUNTIN = 0.0222                 # muntins between sight lines (7/8 in)
LIGHT_X, LIGHT_Y = 0.2334, 0.1826   # daylight opening of each light
THICK = 0.0286                  # sash thickness (1-1/8 in)
RAB, RAB_M = 0.0095, 0.00635    # putty bevel width at rails/stiles and at muntins
GLASS_T, GLASS_DOWN = 0.0024, 0.00395   # single-strength glass; its top sits 3.95 mm below the wood face
XG = XS + RAIL                  # the near rail's sight line: where its putty meets the glass
XE = XS + 0.635
HALF_W = 0.254
M2 = MUNTIN / 2
Z_WOOD = TOP + THICK
Z_GLASS = Z_WOOD - GLASS_DOWN
LIGHTS = {  # name: (x0, x1, y0, y1) daylight
    "near_left": (XG, XG + LIGHT_X, M2, M2 + LIGHT_Y),
    "near_right": (XG, XG + LIGHT_X, -M2 - LIGHT_Y, -M2),
    "far_left": (XG + LIGHT_X + MUNTIN, XG + 2 * LIGHT_X + MUNTIN, M2, M2 + LIGHT_Y),
    "far_right": (XG + LIGHT_X + MUNTIN, XG + 2 * LIGHT_X + MUNTIN, -M2 - LIGHT_Y, -M2),
}
PRIMER = (0.80, 0.80, 0.77)
PUTTY = (0.80, 0.70, 0.52)
GLASS = (0.64, 0.76, 0.80)
PAINT = (0.06, 0.26, 0.13)


def mat_to_quat(m):
    """Rotation matrix to (x, y, z, w), w >= 0 (Shepperd's method; safe at half turns)."""
    m = np.asarray(m, float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    q = -q if q[3] < 0 else q
    return q / np.linalg.norm(q)


def _quat_cols(ex, ey, ez):
    m = np.stack([ex, ey, ez], axis=1)
    q = mat_to_quat(m)
    assert np.allclose(parts._quat_to_matrix(q), m, atol=1e-9)
    return tuple(float(v) for v in q)


def _sash_shapes():
    s = []
    zc = THICK / 2
    x_far = XG + 2 * LIGHT_X + MUNTIN
    # Wood faces (full thickness), outside every putty rabbet.
    s.append(parts.box((XG - RAB - XS, 2 * HALF_W, THICK), ((XS + XG - RAB) / 2, 0, zc), PRIMER))       # bottom rail
    s.append(parts.box((XE - x_far - RAB, 2 * HALF_W, THICK), ((XE + x_far + RAB) / 2, 0, zc), PRIMER))  # top rail
    for sg in (1, -1):
        y0 = M2 + LIGHT_Y + RAB
        s.append(parts.box((x_far - XG + 2 * RAB, HALF_W - y0, THICK), ((XG + x_far) / 2, sg * (HALF_W + y0) / 2, zc), PRIMER))
    s.append(parts.box((x_far - XG, 2 * (M2 - RAB_M), THICK), ((XG + x_far) / 2, 0, zc), PRIMER))      # vertical muntin
    xm = XG + LIGHT_X + M2
    s.append(parts.box((MUNTIN - 2 * RAB_M, 2 * (M2 + LIGHT_Y), THICK), (xm, 0, zc), PRIMER))         # cross muntin
    # Glass: each pane runs 8 mm under its putty on every side.
    for x0, x1, y0, y1 in LIGHTS.values():
        s.append(parts.box((x1 - x0 + .016, y1 - y0 + .016, GLASS_T), ((x0 + x1) / 2, (y0 + y1) / 2, THICK - GLASS_DOWN - GLASS_T / 2), GLASS))
    # Putty bevels: 3 mm slabs sloping from the glass at the sight line up to the wood face.
    for name, (x0, x1, y0, y1) in LIGHTS.items():
        near = name.startswith("near")
        left = name.endswith("left")
        edges = [  # (edge point at sight line, edge direction, outward direction, bevel width, length)
            ((x0, (y0 + y1) / 2), (0, 1), (-1, 0), RAB if near else RAB_M, y1 - y0),
            ((x1, (y0 + y1) / 2), (0, 1), (1, 0), RAB_M if near else RAB, y1 - y0),
            (((x0 + x1) / 2, y1), (1, 0), (0, 1), RAB if left else RAB_M, x1 - x0),
            (((x0 + x1) / 2, y0), (1, 0), (0, -1), RAB_M if left else RAB, x1 - x0),
        ]
        for (px, py), (ex, ey), (ox, oy), w, length in edges:
            o = np.array([ox, oy, 0.0])
            up = np.array([0, 0, 1.0])
            d = (w * o + GLASS_DOWN * up) / math.hypot(w, GLASS_DOWN)   # up the slope, outward
            n = (-GLASS_DOWN * o + w * up) / math.hypot(w, GLASS_DOWN)  # surface normal
            e = np.array([ex, ey, 0.0])
            ex_ = np.cross(e, n)
            ex_ = ex_ if ex_ @ d > 0 else -ex_
            ey_ = np.cross(n, ex_)
            mid = np.array([px, py, THICK - GLASS_DOWN]) + d * math.hypot(w, GLASS_DOWN) / 2 - n * 0.0015
            s.append(parts.box((math.hypot(w, GLASS_DOWN), length + 2 * w, 0.003), tuple(mid), PUTTY,
                               quat=_quat_cols(ex_, ey_, n)))
    return s


SASH = parts.Part("sash", pos=(0.0, 0.0, TOP), shapes=_sash_shapes(), friction=(0.6, 0.5))

# ---- quart paint can, lid off, about 0.7 L of dark green paint ------------------------------------------------------
CAN_R, CAN_H, PAINT_LEVEL = 0.0540, 0.124, 0.090
CAN_POS_A = (0.170, 0.360, TOP)


def _can_shapes():
    s = [parts.cylinder(CAN_R, 0.003, (0, 0, 0.0015), (0.70, 0.70, 0.72))]
    n = 36
    for k in range(n):
        a = 2 * math.pi * k / n
        r = CAN_R - 0.001
        s.append(parts.box((0.002, 2 * math.pi * CAN_R / n + 0.0006, CAN_H), (r * math.cos(a), r * math.sin(a), CAN_H / 2),
                           (0.70, 0.70, 0.72), quat=(0, 0, math.sin(a / 2), math.cos(a / 2))))
    s.append(parts.cylinder(CAN_R - 0.0025, 0.002, (0, 0, PAINT_LEVEL), PAINT, collide=False))
    return s


SIDES = {"left": 1.0, "right": -1.0}   # a full tool set stands on each side of the sash


def _side(p, side):
    return (p[0], SIDES[side] * p[1], p[2])


CANS = {sd: parts.Part(f"paint_can_{sd}", pos=_side(CAN_POS_A, sd), mass=0.95, shapes=_can_shapes(), friction=(0.6, 0.5))
        for sd in SIDES}

# ---- brush rest: 38 x 60 x 165 mm pine block, 110 mm notch between a handle-end pillar and a ferrule pillar ----------
REST_LEN = 0.165
X_HANDLE_END_A = 0.500
REST_POS_A = (X_HANDLE_END_A - REST_LEN / 2, 0.350, TOP)
END_TOP, FERRULE_TOP = 0.0395, 0.038
WOOD = (0.86, 0.72, 0.50)
RESTS = {sd: parts.Part(f"brush_rest_{sd}", pos=_side(REST_POS_A, sd), mass=0.20, friction=(0.6, 0.5), shapes=[
    parts.box((REST_LEN, 0.060, 0.018), (0, 0, 0.009), WOOD),
    parts.box((0.030, 0.060, END_TOP - 0.018), (REST_LEN / 2 - 0.015, 0, (END_TOP + 0.018) / 2), WOOD),
    parts.box((0.025, 0.060, FERRULE_TOP - 0.018), (-REST_LEN / 2 + 0.0125, 0, (FERRULE_TOP + 0.018) / 2), WOOD),
]) for sd in SIDES}

# ---- Purdy XL Glide 2 in angular trim brush: rigid handle and ferrule, compliant bristle tips ------------------------
# Part frame: origin at the handle end on the brush axis, +X toward the bristle tips, Y across the width, Z the
# thickness. It lies on the rest turned to face the arms (yaw pi), broad face down.
HANDLE_L, FERRULE_L, ROOT_L, PAD_L = 0.140, 0.025, 0.040, 0.028
TIP_X = HANDLE_L + FERRULE_L + ROOT_L + PAD_L   # 0.233 m, bristle tips at rest
PAD_W, PAD_T = 0.0508, 0.012
GRIP_X = 0.095                                  # where the reference grips the handle
K_BRISTLE, BRISTLE_TRAVEL = 300.0, 0.010
BRUSH_POS_A = (X_HANDLE_END_A, 0.350, TOP + FERRULE_TOP + 0.009)
BRUSHES = {sd: parts.Part(
    f"brush_{sd}", pos=_side(BRUSH_POS_A, sd), yaw=math.pi, mass=0.075, friction=(0.9, 0.8), angular_damping=2.0,
    shapes=[parts.box((HANDLE_L, 0.022, 0.015), (HANDLE_L / 2, 0, 0), (0.62, 0.40, 0.22)),
            parts.box((FERRULE_L, 0.054, 0.018), (HANDLE_L + FERRULE_L / 2, 0, 0), (0.78, 0.78, 0.80)),
            parts.box((ROOT_L, PAD_W, 0.014), (HANDLE_L + FERRULE_L + ROOT_L / 2, 0, 0), (0.82, 0.70, 0.52))],
    joints=[parts.Slider("bristles", shapes=[parts.box((PAD_L, PAD_W, PAD_T), (TIP_X - PAD_L / 2, 0, 0), (0.84, 0.73, 0.55))],
                         axis=(-1, 0, 0), travel=BRISTLE_TRAVEL, mass=0.010, spring=K_BRISTLE, damping=2.0,
                         max_effort=20.0)],
    link_friction=(("bristles", (0.30, 0.25)),)) for sd in SIDES}
BRUSH = BRUSHES["left"]  # geometry reference for offline checks

OBJECTS = [SASH] + [d[sd] for sd in SIDES for d in (CANS, RESTS, BRUSHES)]

# Nothing jointed moves between layouts: only which near-rail strip is bare changes (a zero move of the left can
# forces the layout tool's settle and fresh first frame).
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", target="near_left", side="left", sign=1.0, place={"paint_can_left": CAN_POS_A}),
    lots.Lot("B", target="near_right", side="right", sign=-1.0, place={"paint_can_left": CAN_POS_A}),
)

# ---- paint field: 1 mm cells over the whole sash and 30 mm of bench around it ---------------------------------------
CELL = 0.001
GX0, NX = XG - 0.116, 696
GY0, NY = -0.284, 568
XC = GX0 + (np.arange(NX) + 0.5) * CELL
YC = GY0 + (np.arange(NY) + 0.5) * CELL
GXX, GYY = np.meshgrid(XC, YC, indexing="ij")
IN_SASH = (GXX >= XS) & (GXX <= XE) & (np.abs(GYY) <= HALF_W)
IS_GLASS = np.zeros_like(IN_SASH)
EDGE_DIST = np.full(GXX.shape, np.inf)   # glass cells: distance to the nearest sight line
PUTTY_RISE = np.full(GXX.shape, -1.0)
for _name, (_x0, _x1, _y0, _y1) in LIGHTS.items():
    _in = (GXX > _x0) & (GXX < _x1) & (GYY > _y0) & (GYY < _y1)
    IS_GLASS |= _in
    EDGE_DIST = np.where(_in, np.minimum.reduce([GXX - _x0, _x1 - GXX, GYY - _y0, _y1 - GYY]), EDGE_DIST)
    _near, _left = _name.startswith("near"), _name.endswith("left")
    for _dist, _w, _span in (((_x0 - GXX), RAB if _near else RAB_M, (GYY > _y0 - RAB) & (GYY < _y1 + RAB)),
                             ((GXX - _x1), RAB_M if _near else RAB, (GYY > _y0 - RAB) & (GYY < _y1 + RAB)),
                             ((GYY - _y1), RAB if _left else RAB_M, (GXX > _x0 - RAB) & (GXX < _x1 + RAB)),
                             ((_y0 - GYY), RAB_M if _left else RAB, (GXX > _x0 - RAB) & (GXX < _x1 + RAB))):
        _band = (_dist >= 0) & (_dist < _w) & _span
        PUTTY_RISE = np.where(_band, np.maximum(PUTTY_RISE, _dist / _w), PUTTY_RISE)
IS_PUTTY = IN_SASH & ~IS_GLASS & (PUTTY_RISE >= 0)
ZS = np.where(~IN_SASH, TOP, np.where(IS_GLASS, Z_GLASS, np.where(IS_PUTTY, Z_GLASS + GLASS_DOWN * np.clip(PUTTY_RISE, 0, 1), Z_WOOD)))
RAIL_ZONE = IN_SASH & (GXX < XG)                         # the near rail, its putty and its ends under the stiles
LAP = IS_GLASS & (EDGE_DIST <= 0.002 + EPS)              # up to 2 mm onto the glass from any putty line
FLOOD_ZONE = IS_GLASS & ~LAP
STRAY_ZONE = ~IN_SASH | (IN_SASH & ~IS_GLASS & ~RAIL_ZONE)


def band_mask(target):
    x0, x1, y0, y1 = LIGHTS[target]
    return (GXX >= XG - 0.025) & (GXX < XG) & (GYY > y0) & (GYY < y1)


def line_mask(target):
    return band_mask(target) & (GXX >= XG - CELL)


BAND = {k: band_mask(k) for k in ("near_left", "near_right")}
LINE = {k: line_mask(k) for k in ("near_left", "near_right")}

FILM, PAINTED = 0.10, 0.05          # mm: film a pass lays, film that counts as painted
D0 = 0.014                          # m: flattened bristle patch behind the heel line
SPLAY_ONSET, SPLAY_RATE, SPLAY_MAX = 0.003, 1.5, 0.006
C_MIN, C_CRUSH = 0.0007, 0.009      # m of bristle compression: paints from 0.7 mm (0.21 N); pushed in to the ferrule at 9 mm
REACH, RISE = 0.006, 0.0045         # m: painted surfaces lie from 6 mm below to 4.5 mm above the lowest tip corner
ON_SURFACE = 0.0015                 # m: the lowest tip corner within 1.5 mm of the sash surface under it (contact)
CAN_TILT, CAN_SHIFT = 15.0, 0.020   # degrees, m: the open can knocked over or shoved
LOAD_MAX, LOAD_RATE = 400.0, 400.0  # mm^3 per dip; mm^3/s while the tips are 5 mm or more under the paint
STRAY_MAX = 200.0                   # mm^2 (2 cm^2)
SLIDE_MIN = 0.00025                 # m per control step (5 mm/s)
BLOB_SPEED, BLOB_S = 0.002, 1.0     # loaded, pressed, under 2 mm/s for 1 s
HOLD_STEPS = 20                     # 1 s at 20 Hz

S = parts.__dict__.setdefault("sash_paint_state", {})


def splay(c):
    return min(SPLAY_MAX, SPLAY_RATE * max(0.0, c - SPLAY_ONSET))


def reset_state(lot=None):
    lot = lot or lots.current()
    film = np.zeros(GXX.shape)
    other = "near_right" if lot.target == "near_left" else "near_left"
    film[BAND[other]] = FILM  # the strip already painted at an earlier sitting
    S.clear()
    S.update(film=film, load={sd: 0.0 for sd in SIDES}, stray_out=0.0, blob=False, crushed=False, dropped=False, broken=False,
             known=False, clear=False, hold=0, dwell={sd: 0.0 for sd in SIDES}, prev_tip={sd: None for sd in SIDES}, t=-1.0,
             target=lot.target, maxc=0.0, changed=set(), can_knocked=False,
             can0={sd: np.asarray(CANS[sd].pos, float) for sd in SIDES},
             vis_reset=True, error=None)


def splay_footprint(corners, axis_w, c):
    """Plan-view patch laid by the bristle tips: the tip face projected down, deepened to D0 toward the tips,
    widened by the splay. Returns (inside(xy) function, lowest corner height)."""
    a = np.array(axis_w[:2], float)
    if np.linalg.norm(a) < 0.2:
        a = np.array([1.0, 0.0])
    a /= np.linalg.norm(a)
    b = np.array([-a[1], a[0]])
    xy = corners[:, :2]
    A, B = xy @ a, xy @ b
    s = splay(c)
    amin, amax = A.min() - s, max(A.max(), A.min() + D0) + s
    bmin, bmax = B.min() - s, B.max() + s
    return a, b, (amin, amax, bmin, bmax)


def on_sash(corners):
    """The lowest tip corner is over the sash and within ON_SURFACE of its surface there (in contact with it)."""
    k = int(np.argmin(corners[:, 2]))
    i = int(math.floor((corners[k, 0] - GX0) / CELL))
    j = int(math.floor((corners[k, 1] - GY0) / CELL))
    if not (0 <= i < NX and 0 <= j < NY) or not IN_SASH[i, j]:
        return False
    return bool(corners[k, 2] - ZS[i, j] <= ON_SURFACE + EPS)


def deposit(corners, axis_w, c, moved, in_can, side="left"):
    """One control step of the declared paint rule for one brush. corners: (4,3) world tip-face corners."""
    if c >= C_CRUSH - EPS:
        S["crushed"] = True
    if in_can or S["load"][side] <= 1e-9 or c < C_MIN - EPS or not on_sash(corners):
        return 0
    a, b, (amin, amax, bmin, bmax) = splay_footprint(corners, axis_w, c)
    zlow = float(corners[:, 2].min())
    # Cells inside the patch's bounding box, then the oriented test.
    pts = np.array([[amin, bmin], [amin, bmax], [amax, bmin], [amax, bmax]])
    world = pts[:, :1] * a + pts[:, 1:] * b
    i0 = max(0, int(math.floor((world[:, 0].min() - GX0) / CELL)))
    i1 = min(NX, int(math.ceil((world[:, 0].max() - GX0) / CELL)) + 1)
    j0 = max(0, int(math.floor((world[:, 1].min() - GY0) / CELL)))
    j1 = min(NY, int(math.ceil((world[:, 1].max() - GY0) / CELL)) + 1)
    area_mm2 = (amax - amin) * (bmax - bmin) * 1e6
    n_in = 0
    if i1 > i0 and j1 > j0:
        X, Y = GXX[i0:i1, j0:j1], GYY[i0:i1, j0:j1]
        A, B = X * a[0] + Y * a[1], X * b[0] + Y * b[1]
        inside = (A >= amin - EPS) & (A <= amax + EPS) & (B >= bmin - EPS) & (B <= bmax + EPS)
        n_in = int(inside.sum())
        zs = ZS[i0:i1, j0:j1]
        reach = inside & (zs >= zlow - REACH - EPS) & (zs <= zlow + RISE + EPS)
        if moved and reach.any():
            f = S["film"][i0:i1, j0:j1]
            need = np.where(reach, np.maximum(FILM - f, 0.0), 0.0)
            total = float(need.sum())  # mm^3 (1 mm^2 cells, mm of film)
            if total > 0:
                k = min(1.0, S["load"][side] / total)
                f += need * k
                S["load"][side] -= total * k
                ii, jj = np.nonzero(need > 0)
                S["changed"].update(zip((ii + i0) // 2, (jj + j0) // 2))
    # Patch area off the grid (beyond the bench margin) counts as stray paint.
    off = (world[:, 0].min() < GX0 or world[:, 0].max() > GX0 + NX * CELL or
           world[:, 1].min() < GY0 or world[:, 1].max() > GY0 + NY * CELL)
    if moved and off:
        S["stray_out"] += max(0.0, area_mm2 - n_in)
    return n_in


def paint_metrics(target=None):
    target = target or S["target"]
    painted = S["film"] >= PAINTED - 1e-12
    band, line = BAND[target], LINE[target]
    return dict(coverage=float(painted[band].mean()), line=float(painted[line].mean()),
                flood=int((painted & FLOOD_ZONE).sum()), stray=float((painted & STRAY_ZONE).sum() + S["stray_out"]),
                lap_max_mm=float(((GXX - XG)[painted & IS_GLASS]).max() * 1000) if (painted & IS_GLASS).any() else 0.0)


def brush_geometry(b):
    """World tip-face corners, tips direction and compression from the brush state (pad link pose)."""
    r = parts._quat_to_matrix(b.quat_xyzw)
    c = float(b.joints.get("bristles", 0.0))
    if "bristles" in b.links:
        lp, lq = b.links["bristles"]
        rl = parts._quat_to_matrix(lq)
        tip = np.asarray(lp, float) + rl @ np.array([PAD_L / 2, 0, 0])
    else:
        tip = np.asarray(b.pos, float) + r @ np.array([TIP_X - c, 0, 0])
        rl = r
    corners = np.array([tip + rl @ np.array([0, sy * PAD_W / 2, sz * PAD_T / 2]) for sy in (-1, 1) for sz in (-1, 1)])
    return corners, rl[:, 0], c, tip


def can_has(tip, can):
    """Tip centre inside the can: (inside the can, under the paint by 5 mm or more)."""
    d = tip - np.asarray(can.pos, float)
    inside = math.hypot(d[0], d[1]) < CAN_R - 0.008 and d[2] < CAN_H
    return inside, inside and d[2] <= PAINT_LEVEL - 0.005


BRUSH_BOXES = ((np.array([HANDLE_L / 2, 0, 0]), np.array([HANDLE_L, 0.022, 0.015]) / 2),
               (np.array([HANDLE_L + FERRULE_L / 2, 0, 0]), np.array([FERRULE_L, 0.054, 0.018]) / 2),
               (np.array([HANDLE_L + FERRULE_L + ROOT_L / 2, 0, 0]), np.array([ROOT_L, PAD_W, 0.014]) / 2),
               (np.array([TIP_X - PAD_L / 2, 0, 0]), np.array([PAD_L, PAD_W, PAD_T]) / 2))


def hands_clear(points, b, gap=0.030):
    r = parts._quat_to_matrix(b.quat_xyzw)
    local = (np.asarray(points, float) - np.asarray(b.pos, float)) @ r
    for centre, half in BRUSH_BOXES:
        d = np.linalg.norm(np.maximum(np.abs(local - centre) - half, 0.0), axis=1)
        if d.min() < gap - EPS:
            return False
    return True


def on_rest(b, rest):
    """Brush lying level across the rest: handle end on the end pillar, ferrule on the ferrule pillar, both seated."""
    rb = parts._quat_to_matrix(b.quat_xyzw)
    if abs(rb[2, 0]) > math.sin(math.radians(10)) or rb[2, 2] < math.cos(math.radians(15)):
        return False
    rr = parts._quat_to_matrix(rest.quat_xyzw)
    end = rr.T @ (np.asarray(b.pos, float) + rb @ np.array([0.015, 0, 0]) - np.asarray(rest.pos, float))
    fer = rr.T @ (np.asarray(b.pos, float) + rb @ np.array([HANDLE_L + FERRULE_L / 2, 0, 0]) - np.asarray(rest.pos, float))
    ok_end = abs(end[0] - (REST_LEN / 2 - 0.015)) <= 0.015 + 0.003 and abs(end[1]) <= 0.030 and abs(end[2] - (END_TOP + 0.0075)) <= 0.005
    ok_fer = abs(fer[0] - (-REST_LEN / 2 + 0.0125)) <= 0.0125 + 0.003 and abs(fer[1]) <= 0.030 and abs(fer[2] - (FERRULE_TOP + 0.009)) <= 0.005
    return bool(ok_end and ok_fer)


def all_on_rest(st):
    """Both brushes lying still across their own rests."""
    return all(on_rest(st[f"brush_{sd}"], st[f"brush_rest_{sd}"]) and st[f"brush_{sd}"].resting(0.01, 0.5) for sd in SIDES)


def score(st):
    try:
        if not S.get("known") or S["broken"] or S["blob"] or S["crushed"] or S["dropped"] or S["can_knocked"]:
            return 0.0
        m = paint_metrics()
        if m["flood"] > 0 or m["stray"] > STRAY_MAX + EPS:
            return 0.0
        if S["hold"] < HOLD_STEPS or not all_on_rest(st):
            return 0.0
        c, line = m["coverage"], m["line"]
        if c >= 0.95 - EPS and line >= 0.95 - EPS:
            return 1.0
        if c >= 0.80 - EPS and line >= 0.80 - EPS:
            return 0.75
        if c >= 0.50 - EPS:
            return 0.5
        if c >= 0.25 - EPS:
            return 0.25
        return 0.0
    except (KeyError, ValueError, TypeError, IndexError):
        return 0.0


def success(st):
    return score(st) == 1.0


if not S:
    reset_state()

# ---- simulator hooks ------------------------------------------------------------------------------------------------
HAND = ("gripper", "tip_left", "tip_right")
_CACHE = {}


def hand_points(base):
    from yam_arena import analysis
    from scipy.spatial import ConvexHull
    origin = parts._np(base.scene.env_origins[0])
    points = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        if not np.isfinite(poses).all():
            raise RuntimeError("unknown hand pose")
        for i, name in enumerate(asset.body_names):
            if name not in HAND:
                continue
            if name not in _CACHE:
                vv = analysis.world_hulls({name: np.eye(4)})
                if not vv:
                    raise RuntimeError("missing hand hull " + name)
                v = np.concatenate([h.vertices for _, h in vv])
                samples = [v]
                for tri in ConvexHull(v).simplices:
                    a, b, c = v[tri]
                    n = max(1, int(np.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / .003)))
                    samples.append(np.array([a + (b - a) * u / n + (c - a) * w / n for u in range(n + 1) for w in range(n + 1 - u)]))
                _CACHE[name] = np.unique(np.round(np.concatenate(samples) / .0025).astype(np.int32), axis=0) * .0025
            points.append(_CACHE[name] @ parts._quat_to_matrix(poses[i, 3:]).T + poses[i, :3] - origin)
    if len(points) != 6:
        raise RuntimeError("unknown hand state")
    return np.concatenate(points)


_VIS = {"stage": None, "tiles": {}}


def _tile_points(ti, tj, origin):
    i, j = 2 * ti, 2 * tj
    pts = []
    for di, dj in ((0, 0), (2, 0), (2, 2), (0, 2)):
        ii, jj = min(NX - 1, max(0, i + di - (1 if di else 0))), min(NY - 1, max(0, j + dj - (1 if dj else 0)))
        z = ZS[ii, jj] + 0.0006
        pts.append((GX0 + (i + di) * CELL + origin[0], GY0 + (j + dj) * CELL + origin[1], z + origin[2]))
    return pts


def update_visuals(base, everything=False):
    from isaaclab.sim.utils import get_current_stage
    from pxr import Gf, UsdGeom, Vt
    stage = get_current_stage()
    if stage is not _VIS["stage"]:
        _VIS["stage"], _VIS["tiles"] = stage, {}
    origin = parts._np(base.scene.env_origins[0])
    painted = S["film"] >= PAINTED - 1e-12
    if S.pop("vis_reset", False):
        for g in _VIS["tiles"].values():
            UsdGeom.Imageable(g).MakeInvisible()
        everything = True
    if everything:
        ii, jj = np.nonzero(painted)
        keys = set(zip(ii // 2, jj // 2))
    else:
        keys = S["changed"]
    for ti, tj in keys:
        block = painted[2 * ti:2 * ti + 2, 2 * tj:2 * tj + 2]
        show = int(block.sum()) >= 2
        g = _VIS["tiles"].get((ti, tj))
        if g is None:
            if not show:
                continue
            g = UsdGeom.Mesh.Define(stage, f"/World/SashPaint/t{ti}_{tj}")
            g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in _tile_points(ti, tj, origin)]))
            g.CreateFaceVertexCountsAttr([4])
            g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
            g.CreateDoubleSidedAttr(True)
            g.CreateDisplayColorAttr([Gf.Vec3f(*PAINT)])
            _VIS["tiles"][(ti, tj)] = g
        UsdGeom.Imageable(g).MakeVisible() if show else UsdGeom.Imageable(g).MakeInvisible()
    S["changed"] = set()


@parts.on_reset
def on_reset(sim):
    reset_state()
    try:
        update_visuals(sim.env.unwrapped)
    except Exception as exc:  # visuals only; scoring state is unaffected
        print("[sash] visual reset failed " + repr(exc), flush=True)


@parts.on_step
def on_step(sim):
    t = float(sim.t)
    if abs(S.get("t", -1.0) - t) < 1e-6:
        return  # the same control step seen through a second import of this file
    dt = t - S["t"] if S["t"] >= 0 else 0.05
    S["t"] = t
    try:
        base = sim.env.unwrapped
        st = sim.states()
        for sd in SIDES:
            b, can = st[f"brush_{sd}"], st[f"paint_can_{sd}"]
            if not np.isfinite(np.r_[b.pos, b.quat_xyzw, can.pos]).all():
                raise RuntimeError("non-finite object state")
            corners, axis_w, c, tip = brush_geometry(b)
            S["maxc"] = max(S["maxc"], c)
            inside, under = can_has(tip, can)
            if under:
                S["load"][sd] = min(LOAD_MAX, S["load"][sd] + LOAD_RATE * dt)
            prev = S["prev_tip"][sd]
            step = float(np.linalg.norm((tip - prev)[:2])) if prev is not None else 0.0
            S["prev_tip"][sd] = tip.copy()
            moved = step >= SLIDE_MIN - EPS and step <= 0.025
            deposit(corners, axis_w, c, moved, inside, sd)
            pressed = c >= C_MIN - EPS and S["load"][sd] > 5.0 and not inside and on_sash(corners)
            S["dwell"][sd] = S["dwell"][sd] + dt if pressed and step < BLOB_SPEED * dt else 0.0
            if S["dwell"][sd] >= BLOB_S - 1e-6:
                S["blob"] = True
            if float(np.min([b.low[2], b.pos[2]])) < TOP - 0.020:
                S["dropped"] = True
            rc = parts._quat_to_matrix(can.quat_xyzw)
            if rc[2, 2] < math.cos(math.radians(CAN_TILT)) or np.linalg.norm(np.asarray(can.pos[:2]) - S["can0"][sd][:2]) > CAN_SHIFT + EPS:
                S["can_knocked"] = True
            if sd == lots.current().side:
                trace = (b, can, corners, c, tip)
        hp = hand_points(base)
        clear = all(hands_clear(hp, st[f"brush_{sd}"]) for sd in SIDES)
        S["clear"] = clear
        S["known"] = True
        S["hold"] = S["hold"] + 1 if clear and all_on_rest(st) else 0
        try:
            update_visuals(base)
        except Exception as exc:
            print("[sash] visual update failed " + repr(exc), flush=True)
        if int(round(t * 20)) % 5 == 0 or os.environ.get("TRACE_OUT"):
            b, can, corners, c, tip = trace
            rest = st[f"brush_rest_{lots.current().side}"]
            m = paint_metrics()
            row = dict(t=round(t, 2), lot=lots.current().name, c_mm=round(c * 1000, 2), maxc_mm=round(S["maxc"] * 1000, 2),
                       load=round(S["load"][lots.current().side], 1), cover=round(m["coverage"], 4), line=round(m["line"], 4), flood=m["flood"],
                       stray=round(m["stray"], 1), lap_mm=round(m["lap_max_mm"], 2), blob=S["blob"], crushed=S["crushed"],
                       dropped=S["dropped"], knocked=S["can_knocked"], clear=clear, hold=S["hold"], on_rest=on_rest(b, rest), tip=[round(float(v), 4) for v in tip],
                       zlow=round(float(corners[:, 2].min() - TOP), 4), heel_x=round(float(corners[np.argmin(corners[:, 2]), 0]), 4),
                       brush=[round(float(v), 4) for v in b.pos], quat=[round(float(v), 4) for v in b.quat_xyzw],
                       can=[round(float(v), 4) for v in can.pos], rest=[round(float(v), 4) for v in rest.pos], score=score(st))
            print("[sash] " + json.dumps(row), flush=True)
    except Exception as exc:
        S["broken"] = True
        S["error"] = repr(exc)
        print("[sash] FAILED " + repr(exc), flush=True)


# ---- reference demo: open loop from the published layout --------------------------------------------------------------
LEAN = 45.0          # degrees the grip leans, tips toward the arms
C_PLAN = 0.0015      # planned bristle compression on the work
U_PASS = (-0.0010, 0.0125)   # heel line of each pass from the sight line (negative: onto the glass)
# Fixed trace corrections (TCP command minus the ideal), measured once; no runtime state is read.
CORR = {"A": {"dip": (0.0, 0.0, 0.0), "pass1": (0.0004, 0.0, 0.0), "pass2": (0.0015, 0.0, 0.0)},
        "B": {"dip": (0.0, 0.0, 0.0), "pass1": (0.0004, 0.0, 0.0), "pass2": (0.0015, 0.0, 0.0)}}
F_GRIP = -(0.022 - 0.007) / 2


def heel_offset(c=C_PLAN, lean=LEAN):
    """Heel line (lowest tip edge) minus TCP, for a brush gripped at GRIP_X, leaned `lean` degrees, tips toward -x."""
    th = math.radians(lean)
    axis = np.array([-math.cos(th), 0.0, -math.sin(th)])
    zed = np.array([-math.sin(th), 0.0, math.cos(th)])
    return (TIP_X - c - GRIP_X) * axis - (PAD_T / 2) * zed


def tip_offset(lean=LEAN):
    th = math.radians(lean)
    return (TIP_X - GRIP_X) * np.array([-math.cos(th), 0.0, -math.sin(th)])


def demo():
    lot = lots.current()
    sg = lot.sign
    arm = lot.side
    corr = CORR[lot.name]
    brush = np.asarray(BRUSHES[arm].pos, float)
    can = np.asarray(CANS[arm].pos, float)
    x0, x1, y0, y1 = LIGHTS[lot.target]
    grip = brush + np.array([-GRIP_X, 0.0, 0.0])          # brush yaw pi: part +X is world -x
    up = np.array([0, 0, 1.0])

    def w(dt, p, f, label, lean=0.0, fix=None):
        p = np.asarray(p, float) + (np.asarray(corr[fix]) if fix else 0.0)
        return tilt.Waypoint(float(dt), tuple(float(v) for v in p), float(f), label, yaw=0.0, tilt=float(lean), heading=0.0)

    plan = [w(2.0, grip + 0.075 * up, FINGER_OPEN, "over the brush handle"),
            w(1.5, grip, FINGER_OPEN, "fingers either side of the handle"),
            w(1.0, grip, F_GRIP, "grip the handle"),
            w(1.5, grip + 0.075 * up, F_GRIP, "lift the brush off the rest"),
            w(2.0, np.array([grip[0] - 0.045, grip[1], TOP + 0.23]), F_GRIP, "lean the brush, tips down", LEAN)]
    tip = tip_offset()

    def dip(label):
        over = can + np.array([-0.015, 0.0, CAN_H + 0.022])
        into = can + np.array([-0.015, 0.0, PAINT_LEVEL - 0.012])
        plan.extend([w(2.5, over - tip, F_GRIP, f"tips over the can ({label})", LEAN, "dip"),
                     w(1.5, into - tip, F_GRIP, "tips into the paint", LEAN, "dip"),
                     w(1.0, into - tip, F_GRIP, "load the bristles", LEAN, "dip"),
                     w(1.2, over - tip, F_GRIP, "lift out of the can", LEAN, "dip")])

    heel = heel_offset()
    y_hi, y_lo = (y1 - PAD_W / 2, y0 + PAD_W / 2)
    ys, ye = (y_hi, y_lo) if sg > 0 else (y_lo, y_hi)
    d = -1.0 if sg > 0 else 1.0   # stroke direction in y: toward the middle muntin

    high_z = can[2] + CAN_H + 0.022 - tip[2]  # TCP height that keeps the tips 22 mm above the can rim

    def stroke(u_heel, surface, label, fix):
        x_heel = XG - u_heel
        tcp = np.array([x_heel, 0.0, surface]) - heel
        start = np.array([tcp[0], ys, tcp[2]])
        end = np.array([tcp[0], ye, tcp[2]])
        above = start + np.array([0, -d * 0.012, 0.015])
        plan.extend([w(2.0, np.array([above[0], above[1], high_z]), F_GRIP, f"carry high to the rail ({label})", LEAN, fix),
                     w(1.5, above, F_GRIP, "lower over the rail", LEAN, fix),
                     w(0.6, start, F_GRIP, "touch down moving", LEAN, fix),
                     w(3.6, end, F_GRIP, f"cut in along the rail ({label})", LEAN, fix),
                     w(0.5, end + np.array([0, d * 0.010, 0.015]), F_GRIP, "lift off moving", LEAN, fix),
                     w(1.2, np.array([end[0], end[1] + d * 0.010, high_z]), F_GRIP, "rise clear of the sash", LEAN, fix)])

    dip("first load")
    stroke(U_PASS[0], Z_GLASS, "pass 1: edge 1 mm onto the glass", "pass1")
    dip("reload")
    stroke(U_PASS[1], Z_WOOD, "pass 2", "pass2")
    plan.extend([w(2.0, np.array([0.30, sg * 0.10, TOP + 0.22]), F_GRIP, "lift clear of the sash", LEAN),
                 w(3.0, grip + 0.085 * up, F_GRIP, "level the brush over the rest"),
                 w(1.5, grip + 0.002 * up, F_GRIP, "lay the brush on the rest"),
                 w(1.0, grip + 0.002 * up, FINGER_OPEN, "let go"),
                 w(1.5, grip + 0.08 * up, FINGER_OPEN, "hands clear"),
                 w(3.0, grip + 0.08 * up, FINGER_OPEN, "still")])
    return {arm: plan}
