"""On a dark blue placemat on the station table: a short, wide clear plastic tumbler holding 200 mL of apple juice
on a cork coaster; a plastic flour scoop with an open front and a long handle, holding a measured dose of a powdered
drink thickener; and a small wire whisk standing head down in a white ceramic utensil cup. Where the tumbler stands,
which side the whisk and the scoop are on, and how much thickener is in the scoop change between trials.
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
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
DT = 0.05  # control step (s)
G = 9.81

NAVY, CORK, CLEAR, WHITE, STEEL = (0.12, 0.17, 0.32), (0.70, 0.55, 0.38), (0.80, 0.84, 0.86), (0.93, 0.93, 0.91), \
    (0.72, 0.74, 0.76)
RED, POWDER = (0.78, 0.16, 0.14), (0.96, 0.93, 0.84)
# The juice's colour as it takes up thickener (share of the prescribed concentration): clear amber to cloudy.
JUICE = [(0.80, 0.47, 0.10), (0.84, 0.58, 0.24), (0.88, 0.69, 0.40), (0.92, 0.80, 0.58)]
LUMP = (1.0, 1.0, 1.0)


def _qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _R(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def _ngon(r, n, z0, z1, color, phase=0.0, cx=0.0, cy=0.0):
    """An n-sided upright prism (a round body PhysX keeps still), collided as its convex hull."""
    ring = [(cx + r * math.cos(phase + 2 * math.pi * k / n), cy + r * math.sin(phase + 2 * math.pi * k / n))
            for k in range(n)]
    pts = [(x, y, z0) for x, y in ring] + [(x, y, z1) for x, y in ring]
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    faces += [(k, (k + 1) % n, n + (k + 1) % n, n + k) for k in range(n)]
    return parts.hull(pts, faces, color=color)


def _ring_walls(r_in, r_out, z0, z1, n, color):
    """A round wall as n flat panels between radii r_in and r_out."""
    out = []
    t = r_out - r_in
    w = 2 * math.pi * r_out / n + 0.002
    for k in range(n):
        a = 2 * math.pi * k / n
        rc = (r_in + r_out) / 2
        out.append(parts.box((t, w, z1 - z0), at=(rc * math.cos(a), rc * math.sin(a), (z0 + z1) / 2), color=color,
                             quat=_qz(a)))
    return out


# --- placemat (fixed) -------------------------------------------------------------------------------------------
MAT_XY, MAT_SIZE = (0.30, 0.0), (0.45, 0.60, 0.002)
MAT_Z = TOP + MAT_SIZE[2]  # the mat's top
CATCH_SIZE = (0.60, 0.75)  # physical dry recovery liner, centered on MAT_XY

# --- tumbler: 300 mL clear polycarbonate, 8.6 cm across, 6.5 cm tall, 200 mL of apple juice ----------------------
T_RIN, T_WALL, T_FLOOR, T_H = 0.040, 0.003, 0.005, 0.065
T_ROUT = T_RIN + T_WALL
JUICE_ML = 200.0
DEPTH = JUICE_ML * 1e-6 / (math.pi * T_RIN ** 2)  # 39.8 mm of juice
SURF = T_FLOOR + DEPTH  # the juice's surface above the tumbler's underside
T_MASS = 0.091 + JUICE_ML * 1e-3  # estimated 91 g polycarbonate shell; fixed juice mass
SPILL_TILT = 25.0  # degrees: the juice reaches the rim (a level surface 20 mm below a rim 40 mm out) at 26.8
COASTER_R, COASTER_T = 0.045, 0.004

# The drink's surface is cut into 7 equal areas: a centre disc and 6 sectors around it.
N_REG = 7
R_C = T_RIN * math.sqrt(1.0 / N_REG)
V_REG = JUICE_ML / N_REG  # mL in each


def region_of(x, y) -> int:
    """The region (0: centre disc, 1-6: sectors from the tumbler's +x axis, counter-clockwise) at x, y (tumbler
    frame)."""
    if math.hypot(x, y) < R_C:
        return 0
    return 1 + int((math.atan2(y, x) % (2 * math.pi)) // (math.pi / 3)) % 6


def _region_centre(i):
    if i == 0:
        return 0.0, 0.0
    a = (i - 0.5) * math.pi / 3
    r = (R_C + T_RIN) / 2
    return r * math.cos(a), r * math.sin(a)


def _region_mesh(i, z, n=8):
    if i == 0:
        pts = [(0.0, 0.0, z)] + [(R_C * math.cos(2 * math.pi * k / 16), R_C * math.sin(2 * math.pi * k / 16), z)
                                 for k in range(16)]
        return tuple(pts), tuple((0, 1 + k, 1 + (k + 1) % 16) for k in range(16))
    a0 = (i - 1) * math.pi / 3
    ro = T_RIN - 0.0008
    pts = []
    for k in range(n + 1):
        a = a0 + (math.pi / 3) * k / n
        pts += [(R_C * math.cos(a), R_C * math.sin(a), z), (ro * math.cos(a), ro * math.sin(a), z)]
    tris = []
    for k in range(n):
        p, q, r, s = 2 * k, 2 * k + 1, 2 * k + 2, 2 * k + 3
        tris += [(p, q, s), (p, s, r)]
    return tuple(pts), tuple(tris)


def _disc(r, z, cx=0.0, cy=0.0, n=12):
    pts = [(cx, cy, z)] + [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n), z)
                           for k in range(n)]
    return tuple(pts), tuple((0, 1 + k, 1 + (k + 1) % n) for k in range(n))


def _blob(r, cx, cy, cz):
    """A small octahedron (a lump)."""
    pts = ((cx + r, cy, cz), (cx - r, cy, cz), (cx, cy + r, cz), (cx, cy - r, cz), (cx, cy, cz + r), (cx, cy, cz - r))
    tris = ((0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4), (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5))
    return pts, tris


def _column(cx, cy, z0, z1, w=0.0035):
    pts = [(cx - w, cy - w, z0), (cx + w, cy - w, z0), (cx + w, cy + w, z0), (cx - w, cy + w, z0),
           (cx - w, cy - w, z1), (cx + w, cy - w, z1), (cx + w, cy + w, z1), (cx - w, cy + w, z1)]
    tris = ((0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7), (4, 5, 6), (4, 6, 7))
    return tuple(pts), tris


T_VIS = []
for _i in range(N_REG):
    for _c, _col in enumerate(JUICE):
        T_VIS.append(parts.Visual(f"juice{_i}_{_c}", *_region_mesh(_i, SURF), color=_col))
    _x, _y = _region_centre(_i)
    T_VIS.append(parts.Visual(f"dry{_i}_0", *_disc(0.001, SURF + 0.0008, _x, _y), color=POWDER))
    T_VIS.append(parts.Visual(f"dry{_i}_1", *_disc(0.009, SURF + 0.0009, _x, _y), color=POWDER))
    for _k in range(3):
        _ox, _oy = 0.006 * math.cos(2.1 * _k + _i), 0.006 * math.sin(2.1 * _k + _i)
        T_VIS.append(parts.Visual(f"lump{_i}_{_k}", *_blob(0.0035, _x + _ox, _y + _oy, SURF + 0.001), color=LUMP))
    T_VIS.append(parts.Visual(f"stream{_i}", *_column(_x, _y, SURF, T_H + 0.012), color=POWDER))


def _tumbler_shapes():
    out = [_ngon(T_ROUT, 20, 0.0, T_FLOOR, CLEAR)]
    out += _ring_walls(T_RIN, T_ROUT, T_FLOOR, T_H, 20, CLEAR)
    return out


# --- utensil cup (whisk holder): white ceramic, 5.2 cm across, 3.6 cm bore, 6 cm tall, 200 g ---------------------
H_RIN, H_ROUT, H_FLOOR, H_H = 0.018, 0.026, 0.008, 0.060


def _holder_shapes():
    return [_ngon(H_ROUT, 16, 0.0, H_FLOOR, WHITE)] + _ring_walls(H_RIN, H_ROUT, H_FLOOR, H_H, 16, WHITE)


# --- whisk: 16 cm mini wire whisk, 2.6 cm head, 1.6 cm red handle, 30 g ----------------------------------------
# Part frame: origin at the tip of the head, z up the handle.
W_HEAD, W_LEN, W_HANDLE_R = 0.060, 0.160, 0.008
W_PROFILE = [(0.0, 0.006), (0.006, 0.011), (0.014, 0.013), (0.026, 0.0125), (0.040, 0.010), (0.052, 0.007),
             (0.060, 0.005)]
W_GRIP = 0.120  # the TCP on the handle, above the tip


def _w_radius(z):
    zs, rs = zip(*W_PROFILE)
    return float(np.interp(z, zs, rs))


def _whisk_shapes():
    n = 12
    pts = [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n), z) for z, r in W_PROFILE
           for k in range(n)]
    m = len(W_PROFILE)
    faces = [tuple(range(n - 1, -1, -1)), tuple(range((m - 1) * n, m * n))]
    faces += [(j * n + k, j * n + (k + 1) % n, (j + 1) * n + (k + 1) % n, (j + 1) * n + k) for j in range(m - 1)
              for k in range(n)]
    head = parts.hull(pts, faces, color=STEEL)
    return [head, _ngon(W_HANDLE_R, 12, W_HEAD, W_LEN, RED)]


# --- scoop: plastic flour scoop, open front, 8 x 6 cm bowl, hollow 10 cm handle, 30 g -------------------------------
# Part frame: origin under the middle of the floor, +x toward the open front, z up.
S_HALF_L, S_HALF_W, S_T = 0.040, 0.030, 0.002
S_BACK_H, S_FRONT_H = 0.045, 0.020
S_HANDLE_END = -0.140
S_HANDLE_W, S_HANDLE_H = 0.014, 0.014
S_HANDLE_TOP = 0.045
S_GRIP = np.array([-0.090, 0.0, S_HANDLE_TOP + 0.001])  # the TCP on the handle
S_LIP = np.array([S_HALF_L, 0.0, S_T])  # the middle of the open front's edge
S_MASS = 0.030


def _side_h(x):
    return S_BACK_H + (S_FRONT_H - S_BACK_H) * (x + S_HALF_L) / (2 * S_HALF_L)


def _scoop_shapes():
    out = [parts.box((2 * S_HALF_L, 2 * S_HALF_W, S_T), at=(0.0, 0.0, S_T / 2), color=RED),
           parts.box((S_T, 2 * S_HALF_W, S_BACK_H), at=(-S_HALF_L + S_T / 2, 0.0, S_BACK_H / 2), color=RED)]
    for sy in (-1.0, 1.0):
        y0, y1 = sy * (S_HALF_W - S_T), sy * S_HALF_W
        pts = [(-S_HALF_L, y0, 0.0), (S_HALF_L, y0, 0.0), (S_HALF_L, y0, S_FRONT_H), (-S_HALF_L, y0, S_BACK_H),
               (-S_HALF_L, y1, 0.0), (S_HALF_L, y1, 0.0), (S_HALF_L, y1, S_FRONT_H), (-S_HALF_L, y1, S_BACK_H)]
        out.append(parts.hull(pts, [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)],
                              color=RED))
    hl = -S_HALF_L - S_HANDLE_END
    hx = (-S_HALF_L + S_HANDLE_END) / 2
    out.append(parts.box((hl, S_HANDLE_W, S_T), at=(hx, 0.0, S_HANDLE_TOP - S_T / 2), color=RED))
    for sy in (-1.0, 1.0):
        out.append(parts.box((hl, S_T, S_HANDLE_H), at=(hx, sy * (S_HANDLE_W - S_T) / 2, S_HANDLE_TOP - S_HANDLE_H / 2),
                             color=RED))
    return out


# The scoop's inside as 2.5 mm voxels (part frame), for what it can hold.
VOX = 0.0025
_vx = np.arange(-S_HALF_L + S_T + VOX / 2, S_HALF_L, VOX)
_vy = np.arange(-S_HALF_W + S_T + VOX / 2, S_HALF_W - S_T, VOX)
_vz = np.arange(S_T + VOX / 2, S_BACK_H, VOX)
_VX, _VY, _VZ = np.meshgrid(_vx, _vy, _vz, indexing="ij")
_inside = _VZ <= _side_h(_VX)
S_VOX = np.c_[_VX[_inside], _VY[_inside], _VZ[_inside]]
VOX_ML = VOX ** 3 * 1e6


def _rim_points(step=0.003):
    """The scoop's rim (part frame): the open front's edge, the tops of the side walls and the back wall."""
    pts = []
    for y in np.arange(-S_HALF_W + S_T, S_HALF_W - S_T + 1e-9, step):
        pts.append((S_HALF_L, y, S_T))
        pts.append((-S_HALF_L, y, S_BACK_H))
    for x in np.arange(-S_HALF_L, S_HALF_L + 1e-9, step):
        for sy in (-1.0, 1.0):
            pts.append((x, sy * (S_HALF_W - S_T), _side_h(x)))
    for z in np.arange(S_T, S_FRONT_H + 1e-9, step):  # the front edges of the side walls
        for sy in (-1.0, 1.0):
            pts.append((S_HALF_L, sy * (S_HALF_W - S_T), z))
    return np.array(pts)


S_RIM = _rim_points()

# Powder in the scoop, drawn as a cream slab on its floor against the back wall, at 8 levels.
S_LEVELS = [2.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 37.5, 45.0]


def _heap_mesh(ml):
    h = 0.012
    L = min(2 * S_HALF_L - 2 * S_T, ml * 1e-6 / (h * (2 * S_HALF_W - 2 * S_T)))
    x0, x1 = -S_HALF_L + S_T + 0.0005, -S_HALF_L + S_T + L
    w = S_HALF_W - S_T - 0.0005
    z0, z1 = S_T + 0.0005, S_T + h
    pts = [(x0, -w, z0), (x1, -w, z0), (x1, w, z0), (x0, w, z0), (x0, -w, z1), (x1, -w, z1), (x1, w, z1), (x0, w, z1)]
    tris = ((0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7), (4, 5, 6), (4, 6, 7))
    return tuple(pts), tris


S_VIS = [parts.Visual(f"heap{k}", *_heap_mesh(ml), color=POWDER) for k, ml in enumerate(S_LEVELS)]

# --- layout (lot A as declared) ---------------------------------------------------------------------------------
D_A = (0.220, -0.030)  # the tumbler's centre
E_A = (1.0, 0.0)  # from the tumbler toward where the scoop pours from (the scoop's front faces back along it)
HOLDER_A = (0.300, -0.180)
LIP_A = (0.270, 0.100)  # where the scoop's lip starts


def _scoop_pose(lip, e):
    """Start pos and yaw of the scoop: lying flat on the mat, its open front facing back along E."""
    centre = np.array(lip) + S_HALF_L * np.array(e)
    return (float(centre[0]), float(centre[1]), MAT_Z), math.atan2(-e[1], -e[0])


_sp, _syaw = _scoop_pose(LIP_A, E_A)

SPILL_STEP = 0.04
SPILL_X = [0.10 + SPILL_STEP * i for i in range(10)]
SPILL_Y = [-0.26 + SPILL_STEP * j for j in range(14)]
MAT_VIS = [parts.Visual(f"sp{i}_{j}", *_disc(0.008, MAT_SIZE[2] + 0.0006, x - MAT_XY[0], y - MAT_XY[1]),
                        color=POWDER) for i, x in enumerate(SPILL_X) for j, y in enumerate(SPILL_Y)]
PLACEMAT = parts.Part("placemat", pos=(*MAT_XY, TOP), shapes=[parts.box(MAT_SIZE, at=(0.0, 0.0, MAT_SIZE[2] / 2),
                                                                         color=NAVY)], visuals=MAT_VIS)
COASTER = parts.Part("coaster", pos=(*D_A, MAT_Z), mass=0.006, shapes=[_ngon(COASTER_R, 20, 0.0, COASTER_T, CORK)],
                     friction=(1.0, 0.9))
TUMBLER = parts.Part("tumbler", pos=(*D_A, MAT_Z + COASTER_T), mass=T_MASS, shapes=_tumbler_shapes(), visuals=T_VIS,
                     friction=(0.6, 0.5))
HOLDER = parts.Part("holder", pos=(*HOLDER_A, MAT_Z), mass=0.200, shapes=_holder_shapes(), friction=(0.8, 0.7))
WHISK = parts.Part("whisk", pos=(*HOLDER_A, MAT_Z + H_FLOOR), mass=0.030, shapes=_whisk_shapes(), friction=(0.8, 0.7))
SCOOP = parts.Part("scoop", pos=_sp, yaw=_syaw, mass=S_MASS, shapes=_scoop_shapes(), visuals=S_VIS,
                   friction=(0.8, 0.7))
OBJECTS = [PLACEMAT, COASTER, TUMBLER, HOLDER, WHISK, SCOOP]


def _lot(name, d, e, holder, lip, dose):
    sp, syaw = _scoop_pose(lip, e)
    return lots.Lot(name, dose=dose, d=d, e=e,
                    place={"coaster": (*d, MAT_Z), "tumbler": (*d, MAT_Z + COASTER_T), "holder": (*holder, MAT_Z),
                           "whisk": (*holder, MAT_Z + H_FLOOR), "scoop": sp},
                    yaw={"scoop": syaw})


# Benchmark doses: two/three level 15 mL tablespoons, 30/45 mL.
# No clinical consistency is inferred from the dose or internal region state.
D_C = (0.190, 0.040)
LOTS = lots.declare(
    OBJECTS,
    _lot("A", D_A, E_A, HOLDER_A, LIP_A, 30.0),
    _lot("B", (D_A[0], -D_A[1]), E_A, (HOLDER_A[0], -HOLDER_A[1]), (LIP_A[0], -LIP_A[1]), 45.0),
    _lot("C", D_C, E_A, HOLDER_A, LIP_A, 45.0),
)

# --- the powder and the drink: rules ----------------------------------------------------------------------------
REPOSE = math.radians(40.0)  # the powder's angle of repose (and of sliding off the scoop's floor)
Q_MAX = 200.0  # mL/s: the most that leaves the scoop's lip (a scoop tipped over empties in a fraction of a second)
K_DISP = 40.0  # dry powder in a region is wetted at a rate K_DISP * the whisk's local stirring (in 1/s) ...
W_CAP = 5.0  # mL/s: ... but one region's surface takes up at most this much powder a second, however hard it is stirred
D_MAX = 1.5  # mL: more dry powder than this on one region (a layer over 2 mm deep) traps dry cores: lumps
M_STILL, T_SKIN = 0.02, 3.0  # dry powder on a region the whisk stirs less than 0.02/s for 3 s skins over: lumps
K_MIX = 2.0  # wetted powder evens out between regions at a rate K_MIX * (local stirring + swirl)
T_SWIRL = 1.5  # s: the swirl the whisk leaves behind dies away with this time constant (it evens, never wets)
V_CAP = 0.2  # m/s: whisk point speeds above this count as this
WINDOW = 5  # control steps (0.25 s): the whisk's travel is measured over this window ...
STROKE_MIN = 0.003  # m: ... and a point that moved less than this in it is not stirring (a tremor is not a stroke)
REACH = 0.030  # m: a whisk point stirs the regions whose centres are within this of it
W_SLICES = [(0.005 + 0.010 * k) for k in range(6)]  # heights up the whisk's head (m) at which it is sampled
W_DL = 0.010
_W_OFF = [(0.0, 0.0), (0.6, 0.0), (-0.6, 0.0), (0.0, 0.6), (0.0, -0.6)]
W_PTS = np.array([(fx * _w_radius(z), fy * _w_radius(z), z) for z in W_SLICES for fx, fy in _W_OFF])
W_AREA = np.array([2 * _w_radius(z) * W_DL / len(_W_OFF) for z in W_SLICES for _ in _W_OFF])  # m^2 per point

# --- the record (one per process, shared by every copy of this module) ------------------------------------------
_L: dict = sys.modules.setdefault("_thicken_drink_record", type(sys)("_thicken_drink_record")).__dict__.setdefault(
    "state", {})


def reset_state(dose_ml: float | None = None) -> None:
    dose = lots.current().dose if dose_ml is None else dose_ml
    _L.update(dose=float(dose), scoop=float(dose), dry=np.zeros(N_REG), hyd=np.zeros(N_REG), lumps=np.zeros(N_REG),
              still=np.zeros(N_REG), swirl=0.0, spill=0.0, juice_spilt=False, inflow=0.0, inflow_reg=-1,
              spill_cells=set(), w_hist=[], stir=0.0, known=False, clear=False, hold=0, partial_hold=0,
              released_hold=0, certificate=None, invalid=False, measurement_invalid=False,
              steps=0, max_tilt=0.0)


def scoop_capacity(pos, quat) -> tuple[float, np.ndarray, np.ndarray]:
    """What the scoop can hold in this pose (mL), the lowest point of its rim (world) and the retained voxels' mask.
    The powder keeps what lies below a plane through the rim's lowest point that rises at the angle of repose
    toward the scoop's inside; anything above it runs out over that point."""
    R = _R(quat)
    p = np.asarray(pos, float)
    rim = p + S_RIM @ R.T
    zmin = float(rim[:, 2].min())
    low = rim[rim[:, 2] <= zmin + 0.001].mean(axis=0)
    vox = p + S_VOX @ R.T
    c = vox.mean(axis=0)
    u = c[:2] - low[:2]
    nu = float(np.linalg.norm(u))
    u = u / nu if nu > 1e-6 else np.zeros(2)
    plane = low[2] + math.tan(REPOSE) * np.clip((vox[:, :2] - low[:2]) @ u, 0.0, None)
    keep = vox[:, 2] <= plane
    return float(keep.sum()) * VOX_ML, low, keep


def _land(point, tumbler):
    """Where powder falling plumb from `point` (world) ends up: a region of the drink, or None (spilled)."""
    R = _R(tumbler.quat_xyzw)
    if tilt.tilt_of(tumbler.quat_xyzw) > 20.0:
        return None
    q = R.T @ (np.asarray(point, float) - np.asarray(tumbler.pos, float))
    if q[2] < SURF or math.hypot(q[0], q[1]) > T_RIN - 0.003:
        return None
    return region_of(q[0], q[1])


def _note_spill(ml, xy):
    if np.any(np.abs(np.asarray(xy)-MAT_XY) > np.asarray(CATCH_SIZE)/2):
        _L["measurement_invalid"]=True
    i = int(round((xy[0] - SPILL_X[0]) / SPILL_STEP))
    j = int(round((xy[1] - SPILL_Y[0]) / SPILL_STEP))
    if 0 <= i < len(SPILL_X) and 0 <= j < len(SPILL_Y):
        _L["spill_cells"].add((i, j))
    _L["spill"] += ml


REG_C = np.array([_region_centre(i) for i in range(N_REG)])


def _stirring(whisk, tumbler, dt):
    """The whisk head's swept volume per second through the juice (m^3/s), in total and per region, from the travel
    of 30 points on its head relative to the tumbler over the last 0.25 s: a point's speed is the distance it travelled
    across the whisk's axis in that window over 0.25 s (at most 0.2 m/s), and it stirs only if it got at least 3 mm
    from where it was at the window's start (a tremor is not a stroke); its swept volume is shared among the regions whose centres are within 3 cm of it."""
    Rt, pt = _R(tumbler.quat_xyzw), np.asarray(tumbler.pos, float)
    Rw, pw = _R(whisk.quat_xyzw), np.asarray(whisk.pos, float)
    q = (pw + W_PTS @ Rw.T - pt) @ Rt  # tumbler frame
    axis = Rt.T @ Rw[:, 2]
    hist = _L["w_hist"]
    if hist and float(np.abs(q - hist[-1]).max()) > 0.05:  # a jump (scene reset): start again
        hist.clear()
    hist.append(q)
    del hist[:-(WINDOW + 1)]
    per = np.zeros(N_REG)
    if len(hist) < WINDOW + 1:
        return 0.0, per
    H = np.array(hist)  # (WINDOW + 1, points, 3)

    def perp(d):
        return d - (d @ axis)[..., None] * axis

    path = np.linalg.norm(perp(np.diff(H, axis=0)), axis=2).sum(axis=0)  # distance travelled in the window
    reach = np.linalg.norm(perp(H - H[0]), axis=2).max(axis=0)  # how far from where it was 0.25 s ago it got
    sp = np.where(reach >= STROKE_MIN - EPS, np.minimum(path / (WINDOW * dt), V_CAP), 0.0)
    wet = (np.hypot(q[:, 0], q[:, 1]) <= T_RIN) & (q[:, 2] >= T_FLOOR) & (q[:, 2] <= SURF)
    sw = W_AREA * sp * wet
    for k in np.nonzero(sw > 0.0)[0]:
        near = np.hypot(REG_C[:, 0] - q[k, 0], REG_C[:, 1] - q[k, 1]) <= REACH
        if not near.any():
            near[region_of(q[k, 0], q[k, 1])] = True
        per[near] += sw[k] / near.sum()
    return float(sw.sum()), per


def step_powder(objects, dt: float = DT) -> None:
    """One control step of the powder: out of the scoop, onto the drink or the mat, wetted by stirring, lumps."""
    tum, sc, wh = objects["tumbler"], objects["scoop"], objects["whisk"]
    # Wet scoop residue cannot be weighed as dry powder in the physical protocol.
    for _,c,r,size in SCOOP.world_boxes({"base":(np.asarray(sc.pos),_R(sc.quat_xyzw))}):
        half=np.abs(r)@(size/2)
        q=np.maximum(np.abs(c[:2]-tum.pos[:2])-half[:2],0.)
        if (np.linalg.norm(q) <= T_RIN and c[2]-half[2] <= tum.pos[2]+SURF
                and c[2]+half[2] >= tum.pos[2]+T_FLOOR):
            _L["measurement_invalid"]=True
    _L["max_tilt"] = max(_L["max_tilt"], tilt.tilt_of(tum.quat_xyzw))
    if _L["max_tilt"] > SPILL_TILT + EPS:
        _L["juice_spilt"] = True
    # out of the scoop
    _L["inflow"], _L["inflow_reg"] = 0.0, -1
    if _L["scoop"] > 1e-9:
        cap, low, _ = scoop_capacity(sc.pos, sc.quat_xyzw)
        out = min(max(_L["scoop"] - cap, 0.0), Q_MAX * dt)
        if out > 0.0:
            _L["scoop"] -= out
            i = _land(low, tum)
            if i is None:
                _note_spill(out, low[:2])
            else:
                _L["dry"][i] += out
                _L["inflow"], _L["inflow_reg"] = out / dt, i
    # stirring
    S, per = _stirring(wh, tum, dt)
    V = JUICE_ML * 1e-6
    _L["swirl"] += (S / V - _L["swirl"]) * min(1.0, dt / T_SWIRL)
    g = per / (V / N_REG)  # the whisk's own stirring in each region (1/s)
    m = g + _L["swirl"]  # what evens out wetted powder
    _L["stir"] = S / V
    # wetting (by the whisk itself, at most W_CAP per region), lumps
    dry, hyd = _L["dry"], _L["hyd"]
    wet = np.minimum(dry * (1.0 - np.exp(-K_DISP * g * dt)), W_CAP * dt)
    dry -= wet
    hyd += wet
    over = np.clip(dry - D_MAX, 0.0, None)
    _L["lumps"] += over
    dry -= over
    still = (dry > 0.02) & (g < M_STILL)
    _L["still"] = np.where(still, _L["still"] + dt, 0.0)
    skin = _L["still"] >= T_SKIN - EPS
    _L["lumps"] += np.where(skin, dry, 0.0)
    dry[skin] = 0.0
    _L["still"][skin] = 0.0
    # hydrated powder evens out between regions
    a = 1.0 - np.exp(-K_MIX * m * dt)
    cbar = hyd.sum() / N_REG
    flux = a * (cbar - hyd)
    flux -= flux.mean()
    # Limit the conservative exchange before it would make a region negative.
    # Clipping after exchange would create powder and corrupt delivery accounting.
    losing = flux < 0.0
    scale = min(1.0, float(np.min(hyd[losing] / -flux[losing]))) if losing.any() else 1.0
    hyd += max(0.0, scale) * flux
    _L["steps"] += 1


# --- end state ---------------------------------------------------------------------------------------------------
LUMP_MAX = 0.05  # display batching only; not a physical lump-volume threshold
LUMP_VISIBLE = 1e-6  # numerical inventory tolerance; any rendered agglomerate fails
# A modeled dry island is a 1 mm deep circular patch. At this inventory its
# diameter reaches the physical screen's 2 mm observable-defect threshold.
DRY_VISIBLE = math.pi * 0.001 ** 2 * 0.001 * 1e6  # mL, chosen visual proxy
ACCOUNT_TOL = 1e-4  # mL numerical conservation tolerance, not a scale uncertainty
POWDER_SPILL_MAX = 0.05  # share of the dose
HALF, FULL = 0.50, 0.95  # shares of the initial dose delivered into the drink
UNIFORM, DRY_MAX = 0.10, 0.01
T_UPRIGHT, T_SPOT, ON = 5.0, 0.015, 0.003
SUPPORT_EPS = 0.00025  # 0.25 mm numerical contact penetration allowance, not an airborne gap
S_UPRIGHT = 10.0
HOLD_STEPS = 20


def inventory_valid() -> bool:
    """Independent material accounting; unknown, negative or created powder fails closed."""
    try:
        values = np.r_[_L["scoop"], _L["spill"], _L["dry"], _L["hyd"], _L["lumps"]]
        return bool(np.isfinite(values).all() and np.isfinite(_L["dose"]) and _L["dose"] > 0
                    and values.min() >= -ACCOUNT_TOL and abs(values.sum() - _L["dose"]) <= ACCOUNT_TOL)
    except (KeyError, TypeError, ValueError):
        return False


def dry_defect() -> bool:
    """A rendered dry island at least 2 mm across; no hidden dry-mass percentage."""
    return bool(np.any(_L["dry"] >= DRY_VISIBLE))


def lump_defect() -> bool:
    """Rendered agglomerates stand in for material retained by the specified sieve."""
    return bool(np.any(_L["lumps"] > LUMP_VISIBLE))


def delivered_share() -> float:
    return float((_L["dose"] - _L["scoop"] - _L["spill"]) / _L["dose"])


def _gate_ok() -> bool:
    return bool(_L["known"] and not _L["invalid"] and not _L["measurement_invalid"]
                and inventory_valid() and not _L["juice_spilt"]
                and not lump_defect() and not dry_defect()
                and _L["spill"] < POWDER_SPILL_MAX * _L["dose"] - EPS)


def hydrated_share() -> float:
    return float(_L["hyd"].sum()) / max(_L["dose"], 1e-9)


def uniform() -> bool:
    hyd = _L["hyd"]
    mean = float(hyd.mean())
    if mean <= 0.0:
        return False
    return bool(float(np.abs(hyd - mean).max()) <= UNIFORM * mean + EPS and float(_L["dry"].sum())
                < DRY_MAX * _L["dose"] - EPS)


def _tumbler_on_spot(tum) -> bool:
    sx, sy, sz = lots.current().pos("tumbler")
    return bool(tilt.tilt_of(tum.quat_xyzw) <= T_UPRIGHT + EPS and abs(float(tum.pos[2]) - sz) <= ON + EPS
                and math.hypot(float(tum.pos[0]) - sx, float(tum.pos[1]) - sy) <= T_SPOT + EPS)


def _whisk_in_holder(wh, ho) -> bool:
    rh = _R(ho.quat_xyzw)
    q = rh.T @ (np.asarray(wh.pos, float) - np.asarray(ho.pos, float))
    # Test the actual lowest collision point in the holder's frame, including
    # a tilted head, rather than a symmetric tolerance around the tip origin.
    vertices = np.concatenate([np.asarray(s.points) for s in WHISK.shapes if s.collide])
    world = np.asarray(wh.pos) + vertices @ _R(wh.quat_xyzw).T
    bottom = float(((world-np.asarray(ho.pos)) @ rh)[:, 2].min())
    return bool(math.hypot(q[0], q[1]) <= H_RIN + EPS and H_FLOOR-SUPPORT_EPS <= bottom <= H_FLOOR+ON+EPS
                and tilt.tilt_of(wh.quat_xyzw) <= 15.0 + EPS)


def _scoop_down(sc) -> bool:
    return bool(tilt.tilt_of(sc.quat_xyzw) <= S_UPRIGHT + EPS and _on_surface(sc, MAT_XY, MAT_SIZE[:2], MAT_Z))


def _on_surface(obj, xy, size, height) -> bool:
    lo, hi = np.asarray(obj.low), np.asarray(obj.high)
    return bool(np.isfinite(np.r_[lo, hi]).all() and height-SUPPORT_EPS <= lo[2] <= height+ON+EPS
                and np.all(lo[:2] >= np.asarray(xy)-np.asarray(size)/2-EPS)
                and np.all(hi[:2] <= np.asarray(xy)+np.asarray(size)/2+EPS))


def _working_surface(obj) -> bool:
    from yam_arena import station
    # Bare-table support cannot certify an object embedded in the raised mat.
    lo, hi = np.asarray(obj.low), np.asarray(obj.high)
    mat_lo = np.r_[np.asarray(MAT_XY)-np.asarray(MAT_SIZE[:2])/2, TOP]
    mat_hi = np.r_[np.asarray(MAT_XY)+np.asarray(MAT_SIZE[:2])/2, MAT_Z]
    if np.all(np.minimum(hi, mat_hi)-np.maximum(lo, mat_lo) > SUPPORT_EPS):
        return False
    return bool(_on_surface(obj, MAT_XY, MAT_SIZE[:2], MAT_Z)
                or _on_surface(obj, station.TABLE_CENTER[:2], station.TABLE_SIZE[:2], TOP))


def _released_end(objects) -> bool:
    """All positive tiers need independently supported, still, unheld objects."""
    tum, wh, sc, ho, co = (objects[k] for k in ("tumbler", "whisk", "scoop", "holder", "coaster"))
    fixtures = (_on_surface(ho, MAT_XY, MAT_SIZE[:2], MAT_Z)
                and _on_surface(co, MAT_XY, MAT_SIZE[:2], MAT_Z))
    cup_supported = (abs(float(tum.low[2])-float(co.high[2])) <= ON+EPS
                     and float(tum.low[2]) >= float(co.high[2])-SUPPORT_EPS
                     and np.linalg.norm(tum.pos[:2]-co.pos[:2]) <= COASTER_R-.010+EPS)
    whisk_in_drink = (abs(float(wh.low[2])-(float(tum.pos[2])+T_FLOOR)) <= ON+EPS
                      and float(wh.low[2]) >= float(tum.pos[2])+T_FLOOR-SUPPORT_EPS
                      and np.linalg.norm(wh.pos[:2]-tum.pos[:2]) <= T_RIN-.006+EPS)
    whisk_supported = _whisk_in_holder(wh, ho) or whisk_in_drink or _working_surface(wh)
    return bool(_L["clear"] and fixtures and cup_supported and _tumbler_on_spot(tum)
                and whisk_supported and _working_surface(sc)
                and all(s.resting() for s in (tum, wh, sc, ho, co)))


def _end_ok(objects) -> bool:
    tum, wh, sc, ho = objects["tumbler"], objects["whisk"], objects["scoop"], objects["holder"]
    return bool(_released_end(objects) and _whisk_in_holder(wh, ho) and _scoop_down(sc))


def _drink_ok() -> bool:
    return bool(_gate_ok() and delivered_share() >= FULL - EPS)


def _one_tool_ok(objects) -> bool:
    tum, wh, sc, ho = (objects[k] for k in ("tumbler", "whisk", "scoop", "holder"))
    return bool(_released_end(objects) and (_whisk_in_holder(wh, ho) or _scoop_down(sc)))


def _stamp(objects):
    return tuple(float(v) for k in ("tumbler", "whisk", "scoop", "holder", "coaster")
                 for v in np.r_[objects[k].pos, objects[k].quat_xyzw, objects[k].lin_vel,
                                 objects[k].ang_vel, objects[k].low, objects[k].high])


def record(objects, clear, dt: float = DT) -> None:
    """One control step: the powder, whether every finger and gripper is clear of the tumbler, the whisk and the
    scoop and their holder/coaster supports (None: unknown, which scores 0), and the supported release holds."""
    _L["known"] = clear is not None
    try:
        values = np.asarray(_stamp(objects))
        if not np.isfinite(values).all() or not np.isfinite(dt) or dt <= 0 or dt > 0.100001:
            _L["invalid"] = True
            return
        if any(not .99 <= np.linalg.norm(objects[k].quat_xyzw) <= 1.01
               for k in ("tumbler", "whisk", "scoop", "holder", "coaster")):
            _L["invalid"]=True
            return
    except (KeyError, AttributeError, TypeError, ValueError):
        _L["invalid"] = True
        return
    step_powder(objects, dt)
    _L["clear"] = bool(clear)
    _L["hold"] = _L["hold"] + 1 if (_drink_ok() and _end_ok(objects)) else 0
    _L["partial_hold"] = _L["partial_hold"] + 1 if (_drink_ok() and _one_tool_ok(objects)) else 0
    _L["released_hold"] = _L["released_hold"] + 1 if (_gate_ok() and _released_end(objects)) else 0
    _L["certificate"] = _stamp(objects)


if "dose" not in _L:
    reset_state()


def score(objects):
    """Grade measured delivery and visible dispersion, then the existing tool put-away goals.
    The score does not establish an IDDSI level, molecular hydration or suitability to serve."""
    if not _gate_ok():
        return 0.0
    try:
        if _L["certificate"] != _stamp(objects):
            return 0.0
    except (KeyError, AttributeError, TypeError, ValueError):
        return 0.0
    if _L["released_hold"] < HOLD_STEPS or not _released_end(objects):
        return 0.0
    if _L["hold"] >= HOLD_STEPS and _drink_ok() and _end_ok(objects):
        return 1.0
    share = delivered_share()
    if share >= FULL - EPS:
        return 0.75 if _L["partial_hold"] >= HOLD_STEPS and _one_tool_ok(objects) else 0.5
    if share >= HALF - EPS:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    return float(max(0.0, 1.0 - delivered_share())) if inventory_valid() else 1.0


# --- simulator hooks: the hands ---------------------------------------------------------------------------------
HAND_LINKS = ("tip_left", "tip_right", "gripper")
CLEAR = 0.005
FACE_STEP = 0.003
_FACE_PTS: dict = {}


def _face_points(h) -> np.ndarray:
    """Points on every face of a convex hull (its link frame), about 3 mm apart, plus its vertices: a flat face lying
    on an object is then seen even when no vertex is near it."""
    out = [h.vertices]
    planes = np.unique(np.round(h.planes, 4), axis=0)  # an STL gives one plane per triangle: drop repeats
    for n, d in zip(planes[:, :3], planes[:, 3]):
        n = n / np.linalg.norm(n)
        on = h.vertices[np.abs(h.vertices @ n - d) < 2e-4]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        a = np.cross(n, [1.0, 0.0, 0.0] if abs(n[0]) < 0.9 else [0.0, 1.0, 0.0])
        a /= np.linalg.norm(a)
        b = np.cross(n, a)
        ang = np.arctan2((on - c) @ b, (on - c) @ a)
        ring = on[np.argsort(ang)]
        for k in range(len(ring)):
            p0, p1 = ring[k], ring[(k + 1) % len(ring)]
            e1, e2 = p0 - c, p1 - c
            m = max(1, int(math.ceil(max(np.linalg.norm(e1), np.linalg.norm(e2), np.linalg.norm(p1 - p0)) / FACE_STEP)))
            for i in range(m + 1):
                for j in range(m + 1 - i):
                    out.append((c + e1 * (i / m) + e2 * (j / m))[None, :])
    pts = np.concatenate(out)
    _, keep = np.unique(np.floor(pts / (FACE_STEP / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([h.vertices, pts[np.sort(keep)]])


def hand_points(link_poses) -> np.ndarray:
    """World points on the faces of the finger tips and the gripper bodies. link_poses: {link name: 4x4}."""
    from yam_arena import analysis

    pts = []
    for name, m in link_poses.items():
        for k, h in enumerate(analysis.link_hulls().get(name, [])):
            key = (name, k)
            if key not in _FACE_PTS:
                _FACE_PTS[key] = _face_points(h)
            pts.append(_FACE_PTS[key] @ m[:3, :3].T + m[:3, 3])
    return np.concatenate(pts) if pts else np.zeros((0, 3))


SCORED = ("tumbler", "whisk", "scoop", "holder", "coaster")


def points_clear(points, objects) -> bool:
    """True when no point is within CLEAR of any shape (its box) of the tumbler, whisk, scoop, holder or coaster."""
    from yam_arena import analysis

    if len(points) == 0 or not np.isfinite(points).all():
        return False
    spec = {"tumbler": TUMBLER, "whisk": WHISK, "scoop": SCOOP, "holder": HOLDER, "coaster": COASTER}
    for name in SCORED:
        s = objects[name]
        boxes = spec[name].world_boxes({"base": (np.asarray(s.pos, float), _R(s.quat_xyzw))})
        ext = [(c - np.abs(r) @ (size / 2), c + np.abs(r) @ (size / 2)) for _, c, r, size in boxes]
        lo = np.min([e[0] for e in ext], axis=0) - CLEAR - 0.01
        hi = np.max([e[1] for e in ext], axis=0) + CLEAR + 0.01
        near_obj = points[np.all((points >= lo) & (points <= hi), axis=1)]
        if not len(near_obj):
            continue
        for (_, c, r, size), (blo, bhi) in zip(boxes, ext):
            near = near_obj[np.all((near_obj >= blo - CLEAR - 0.01) & (near_obj <= bhi + CLEAR + 0.01), axis=1)]
            if len(near) and float(analysis.box_signed_distance(near, c, r, size).min()) <= CLEAR + EPS:
                return False
    return True


def _hands_clear(sim, st) -> bool:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    poses = {}
    for arm in ("left_robot", "right_robot"):
        try:
            asset = base.scene[arm]
            lp = parts._np(asset.data.body_link_pose_w)[0]
        except (KeyError, AttributeError, IndexError, TypeError, ValueError):
            return None
        if not set(HAND_LINKS).issubset(asset.body_names) or not np.isfinite(lp).all():
            return None
        for i, n in enumerate(asset.body_names):
            if n in HAND_LINKS:
                m = np.eye(4)
                m[:3, :3] = _R(lp[i, 3:])
                m[:3, 3] = lp[i, :3] - origin
                poses[f"{arm}:{n}"] = (n, m)
    if len(poses) != 2 * len(HAND_LINKS):
        return None
    from yam_arena import analysis

    pts = []
    for key, (n, m) in poses.items():
        hulls = analysis.link_hulls().get(n, [])
        if not hulls:
            return None
        for k, h in enumerate(hulls):
            if (n, k) not in _FACE_PTS:
                _FACE_PTS[(n, k)] = _face_points(h)
            pts.append(_FACE_PTS[(n, k)] @ m[:3, :3].T + m[:3, 3])
    if not pts:
        return None
    return points_clear(np.concatenate(pts), st)


# --- drawing ---------------------------------------------------------------------------------------------------
_SHOWN: dict = {}


def _show(sim, part, names_on) -> None:
    shown = _SHOWN.setdefault(part, set())
    for n in shown - names_on:
        sim.set_visible(part, n, False)
    for n in names_on - shown:
        sim.set_visible(part, n, True)
    _SHOWN[part] = set(names_on)


def _draw(sim) -> None:
    on = set()
    target = _L["dose"] / N_REG
    for i in range(N_REG):
        f = _L["hyd"][i] / max(target, 1e-9)
        on.add(f"juice{i}_{0 if f < 0.25 else 1 if f < 0.6 else 2 if f < 0.9 else 3}")
        if _L["dry"][i] > 0.6:
            on.add(f"dry{i}_1")
        elif _L["dry"][i] >= DRY_VISIBLE:
            on.add(f"dry{i}_0")
        n = int(min(3, math.ceil(_L["lumps"][i] / LUMP_MAX - 1e-9))) if _L["lumps"][i] > 1e-6 else 0
        on |= {f"lump{i}_{k}" for k in range(n)}
    if _L["inflow_reg"] >= 0:
        on.add(f"stream{_L['inflow_reg']}")
    _show(sim, "tumbler", on)
    k = max((i for i, ml in enumerate(S_LEVELS) if _L["scoop"] >= ml - 1.0), default=None)
    _show(sim, "scoop", {f"heap{k}"} if k is not None and _L["scoop"] > 0.5 else set())
    _show(sim, "placemat", {f"sp{i}_{j}" for i, j in _L["spill_cells"]})


@parts.on_reset
def _on_reset(sim) -> None:
    reset_state()
    _L["last_t"] = None
    _SHOWN.clear()
    for part, vis in (("tumbler", T_VIS), ("scoop", S_VIS), ("placemat", MAT_VIS)):
        for v in vis:
            sim.set_visible(part, v.name, False)
    _draw(sim)


@parts.on_step
def _on_step(sim) -> None:
    if _L.get("last_t") == sim.t:  # a second copy of this module sharing the record: step it once
        return
    _L["last_t"] = sim.t
    st = sim.states()
    try:
        clear = _hands_clear(sim, st)
    except Exception:  # unknown hand state: no credit at all
        clear = None
    record(st, clear)
    if os.environ.get("TASK_DEBUG") and _L["steps"] % 10 == 0:
        print(f"[powder] t={_L['steps'] * DT:.1f} scoop={_L['scoop']:.2f} dry={_L['dry'].round(2).tolist()} "
              f"hyd={_L['hyd'].round(2).tolist()} lumps={_L['lumps'].sum():.3f} spill={_L['spill']:.2f} "
              f"stir={_L['stir']:.3f} swirl={_L['swirl']:.3f} clear={clear} hold={_L['hold']} "
              f"invalid={_L['invalid']} measure_invalid={_L['measurement_invalid']} "
              f"released={_released_end(st)} whisk_stowed={_whisk_in_holder(st['whisk'],st['holder'])} "
              f"scoop_stowed={_scoop_down(st['scoop'])} "
              f"ttilt={tilt.tilt_of(st['tumbler'].quat_xyzw):.1f} stilt={tilt.tilt_of(st['scoop'].quat_xyzw):.1f}",
              flush=True)
    _draw(sim)


# --- demo --------------------------------------------------------------------------------------------------------
SQUEEZE = 0.006  # m: the fingers are driven this far past the faces they close on
OPEN = -0.030  # finger value: 6 cm apart
W_SHUT = -(2 * W_HANDLE_R - SQUEEZE) / 2
S_SHUT = -(S_HANDLE_W - SQUEEZE) / 2
LIP_R = 0.028  # the lip is held this far from the tumbler's centre, toward the scoop's side
LIP_UP = 0.010  # and this far above the rim
STIR_TILT = 20.0  # degrees: the whisk leans away from the scoop while it stirs
STIR_A, STIR_B, STIR_C = 0.012, 0.004, 0.004  # m: ellipse half-axes (across, along E) and its centre (along E)
STIR_REV, STIR_PTS = 1.0, 10  # s per turn, waypoints per turn
POUR_FROM, POUR_SLOW, POUR_TO = 18.0, 36.0, 42.0  # scoop tilt (degrees)
POUR_S, POUR_S_END = 34.0, 24.0  # s to tilt from POUR_FROM to POUR_SLOW, and on to POUR_TO
T_STIR_END = 84.0  # s: stirring stops


def pour_pose(d, e, syaw, gam):
    """The scoop's pose (origin, rotation) tipped `gam` degrees with its front edge held over the tumbler at d."""
    R0 = parts._rot((0, 0, 1), syaw)
    ev = np.array([e[0], e[1], 0.0])
    axis = np.cross([0.0, 0.0, -1.0], ev)
    lip_t = np.array([d[0], d[1], 0.0]) + LIP_R * ev
    lip_t[2] = MAT_Z + COASTER_T + T_H + LIP_UP
    R = parts._rot(axis, math.radians(gam)) @ R0
    return lip_t - R @ S_LIP, R


def _pour_frames(d, e, sp, syaw):
    """Scoop keyframes (duration s, origin, rotation) and the tilt of each (degrees, for the gripper)."""
    R0 = parts._rot((0, 0, 1), syaw)
    o0 = np.array(sp)
    lip_z = MAT_Z + COASTER_T + T_H + LIP_UP

    def at(gam):
        return pour_pose(d, e, syaw, gam)

    fr = [(0.0, o0, R0, 0.0)]
    lift = o0 + np.array([0.0, 0.0, lip_z - MAT_Z + 0.012])
    fr.append((2.0, lift, R0, 0.0))
    o, R = at(POUR_FROM)
    back = np.array([0.0, 0.0, 0.015])
    fr.append((3.0, o + back, R, POUR_FROM))
    fr.append((1.0, o, R, POUR_FROM))
    # a quarter degree per waypoint, for an even flow; slower past POUR_SLOW, where the last layer on the floor slides
    for g0, g1, du in ((POUR_FROM, POUR_SLOW, POUR_S), (POUR_SLOW, POUR_TO, POUR_S_END)):
        n = int(round((g1 - g0) * 4))
        for k in range(1, n + 1):
            g = g0 + (g1 - g0) * k / n
            o, R = at(g)
            fr.append((du / n, o, R, g))
    fr.append((4.0, o, R, POUR_TO))
    o, R = at(POUR_FROM)
    fr.append((3.0, o + back, R, POUR_FROM))
    fr.append((3.0, lift, R0, 0.0))
    fr.append((2.0, o0 + np.array([0.0, 0.0, 0.006]), R0, 0.0))
    return fr


def _arms(lot):
    """Which arm stirs: the one on the whisk's side (visible). The other pours."""
    return ("right", "left") if lot.pos("holder")[1] < 0 else ("left", "right")


def demo():
    """One hand takes the whisk from its cup and keeps it moving through the juice; the other lifts the scoop over
    the tumbler and tips it forward slowly, so the thickener runs off its front edge a little at a time. The whisk
    keeps going for a while after the scoop is empty. Then the scoop goes back down on the mat and the whisk back in
    its cup."""
    lot = lots.current()
    d, e = lot.d, lot.e
    stir_arm, pour_arm = _arms(lot)
    ev = np.array([e[0], e[1], 0.0])
    nv = np.array([-e[1], e[0], 0.0])
    heading = math.atan2(e[1], e[0])
    plans = {}

    # the scoop
    sp, syaw = lot.pos("scoop"), lot.yaw_of("scoop")
    yaw = (syaw + math.pi / 2) % math.pi - math.pi / 2  # fingers close across the handle
    frames = _pour_frames(d, e, sp, syaw)
    R0 = frames[0][2]
    g0 = np.array(sp) + R0 @ S_GRIP
    w = [tilt.Waypoint(2.5, tuple(float(v) for v in g0 + [0.0, 0.0, 0.04]), OPEN, "over the handle", yaw=yaw),
         tilt.Waypoint(1.5, tuple(float(v) for v in g0), OPEN, "down astride the handle", yaw=yaw),
         tilt.Waypoint(1.0, tuple(float(v) for v in g0), S_SHUT, "pinch the handle", yaw=yaw)]
    for k, (du, o, R, gam) in enumerate(frames[1:], 1):
        tcp = o + R @ S_GRIP
        w.append(tilt.Waypoint(du, tuple(float(v) for v in tcp), S_SHUT, f"scoop {k}", yaw=yaw, tilt=gam,
                               heading=heading))
    end = o0 = frames[-1][1] + frames[-1][2] @ S_GRIP
    w.append(tilt.Waypoint(1.0, tuple(float(v) for v in end), OPEN, "let go", yaw=yaw))
    w.append(tilt.Waypoint(1.5, tuple(float(v) for v in o0 + [0.0, 0.0, 0.05]), OPEN, "up", yaw=yaw))
    plans[pour_arm] = w

    # the whisk
    hp = np.array(lot.pos("whisk"))
    top = MAT_Z + 0.205
    in_cup = hp + np.array([0.0, 0.0, W_GRIP])
    down = np.array([math.sin(math.radians(STIR_TILT)) * e[0], math.sin(math.radians(STIR_TILT)) * e[1],
                     -math.cos(math.radians(STIR_TILT))])  # from the TCP toward the whisk's tip
    tip_z = MAT_Z + COASTER_T + T_FLOOR + 0.006
    centre = np.array([d[0], d[1], tip_z]) + STIR_C * ev
    lean = dict(tilt=STIR_TILT, heading=heading)

    def tcp_for_tip(tip):
        return tip - down * W_GRIP

    w = [tilt.Waypoint(2.0, tuple(float(v) for v in in_cup + [0.0, 0.0, 0.03]), OPEN, "over the whisk"),
         tilt.Waypoint(1.5, tuple(float(v) for v in in_cup), OPEN, "down to the handle"),
         tilt.Waypoint(0.8, tuple(float(v) for v in in_cup), W_SHUT, "grip the handle"),
         tilt.Waypoint(2.0, (float(in_cup[0]), float(in_cup[1]), top), W_SHUT, "lift it out")]
    over = tcp_for_tip(centre)
    w.append(tilt.Waypoint(2.0, (float(over[0]), float(over[1]), top), W_SHUT, "over the tumbler", **lean))
    t = sum(x.duration for x in w)
    first = centre + STIR_A * nv
    w.append(tilt.Waypoint(2.0, tuple(float(v) for v in tcp_for_tip(first)), W_SHUT, "into the juice", **lean))
    t += 2.0
    k = 0
    while t < T_STIR_END:
        a = 2 * math.pi * (k + 1) / STIR_PTS
        tip = centre + STIR_A * math.cos(a) * nv + STIR_B * math.sin(a) * ev
        du = STIR_REV / STIR_PTS
        w.append(tilt.Waypoint(du, tuple(float(v) for v in tcp_for_tip(tip)), W_SHUT, "stir", **lean))
        t += du
        k += 1
    w.append(tilt.Waypoint(2.0, (float(over[0]), float(over[1]), top), W_SHUT, "out of the juice", **lean))
    w.append(tilt.Waypoint(2.0, (float(in_cup[0]), float(in_cup[1]), top), W_SHUT, "over its cup"))
    put = in_cup + np.array([0.0, 0.0, 0.010])
    w.append(tilt.Waypoint(2.0, tuple(float(v) for v in put), W_SHUT, "into its cup"))
    w.append(tilt.Waypoint(0.8, tuple(float(v) for v in put), OPEN, "let go"))
    w.append(tilt.Waypoint(1.5, tuple(float(v) for v in put + [0.0, 0.0, 0.05]), OPEN, "up"))
    w.append(tilt.Waypoint(3.0, tuple(float(v) for v in put + [0.0, 0.0, 0.05]), OPEN, "wait"))
    plans[stir_arm] = w
    return plans
