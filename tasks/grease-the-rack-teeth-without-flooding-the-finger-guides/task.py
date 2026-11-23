"""On a grey anti-static bench mat: a shallow stainless steel instrument tray. In it lie the two steel racks of a
Robotiq Hand-E gripper that has been opened for its rack-and-pinion service: each rack is a 62 mm steel bar with a
cross-shaped section (a narrow stem above and below a wider guide flange). They lie side by side across the tray,
toward one of its long walls, each with the face that carries its tapped screw holes upward. A nylon tile-and-grout
brush whose bristles are loaded with red grease lies bristles up across the tray's two long walls, its head over one
wall and its handle reaching past the other. Beyond the tray, on the mat, stand the opened gripper body (its rack
housing removed, the pinion showing on top); its rack housing and its tool coupling lie beside it, and a grease
syringe lies near the right-hand corner.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402  (before lots, so parts' reset hooks run before the lots' moves)
import lots  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
EPS = 1e-6

STEEL, HOLE, TOOTH = (0.42, 0.43, 0.45), (0.08, 0.08, 0.09), (0.27, 0.28, 0.30)
MAT_GREY, TRAY_STEEL = (0.36, 0.42, 0.48), (0.74, 0.75, 0.77)
HANDLE_WHITE, BRISTLE_RED, GREASE_RED = (0.93, 0.93, 0.91), (0.78, 0.22, 0.16), (0.80, 0.06, 0.05)
HANDE_BLACK, HANDE_GREY, ALU = (0.10, 0.10, 0.11), (0.30, 0.31, 0.33), (0.62, 0.63, 0.65)
SYRINGE_CLEAR = (0.85, 0.80, 0.78)

# -- the bench: an anti-static mat and on it a stainless instrument tray (both fixed) ---------------------------------
MAT_SIZE = (0.457, 0.610, 0.002)  # 18 x 24 in anti-static bench mat
MAT_CENTER = (0.300, 0.0)
MAT_TOP = TOP + MAT_SIZE[2]
TRAY_SIZE = (0.254, 0.165, 0.018)  # Miltex non-perforated Mayo tray, 10 x 6-1/2 x 23/32 in
TRAY_CENTER = (0.300, 0.0)
TRAY_FLOOR_T, TRAY_WALL_T = 0.001, 0.0015
FLOOR = MAT_TOP + TRAY_FLOOR_T  # the tray's floor top
WALL_TOP = MAT_TOP + TRAY_SIZE[2]
TRAY_IN = (TRAY_SIZE[0] / 2 - TRAY_WALL_T, TRAY_SIZE[1] / 2 - TRAY_WALL_T)  # inner half-lengths (x, y)


def _tray_shapes():
    L, W, H = TRAY_SIZE
    t, f = TRAY_WALL_T, TRAY_FLOOR_T
    zc = H / 2 - TRAY_SIZE[2] / 2  # part frame: origin at the tray's centre, half height up
    return [
        parts.box((L, W, f), at=(0.0, 0.0, -H / 2 + f / 2), color=TRAY_STEEL),
        parts.box((L, t, H), at=(0.0, W / 2 - t / 2, zc), color=TRAY_STEEL),
        parts.box((L, t, H), at=(0.0, -W / 2 + t / 2, zc), color=TRAY_STEEL),
        parts.box((t, W - 2 * t, H), at=(L / 2 - t / 2, 0.0, zc), color=TRAY_STEEL),
        parts.box((t, W - 2 * t, H), at=(-L / 2 + t / 2, 0.0, zc), color=TRAY_STEEL),
    ]


MAT = parts.Part("bench_mat", pos=(MAT_CENTER[0], MAT_CENTER[1], TOP + MAT_SIZE[2] / 2),
                 shapes=[parts.box(MAT_SIZE, color=MAT_GREY)], friction=(0.8, 0.7))
TRAY = parts.Part("tray", pos=(TRAY_CENTER[0], TRAY_CENTER[1], MAT_TOP + TRAY_SIZE[2] / 2), shapes=_tray_shapes(),
                  friction=(0.4, 0.35))

# -- the opened Robotiq Hand-E and its parts, beyond the tray (fixed; Robotiq manual Fig. 5-1, sections 6.1 and 6.3) --
HANDE_BODY_XY = (0.490, 0.170)  # the gripper body standing on its base face, rack housing off
HOUSING_XY = (0.490, 0.068)  # the rack housing, lying channels up
COUPLING_XY = (0.490, -0.035)  # the tool coupling (GRP-ES-CPL-062), lying flat
SYRINGE_XY = (0.470, -0.150)  # the grease syringe (ACC-LUB-SHC1500), lying along y
# (kind, centre (world), radius or size, half height, colour): cylinders stand on their axis (z) unless noted
HANDE_CYLS = [
    ("base", (*HANDE_BODY_XY, MAT_TOP + 0.010), 0.0375, 0.010),  # 75 mm base
    ("body", (*HANDE_BODY_XY, MAT_TOP + 0.020 + 0.0315), 0.031, 0.0315),  # 62 mm waist to 83 mm (estimate)
    ("pinion", (*HANDE_BODY_XY, MAT_TOP + 0.083 + 0.004), 0.007, 0.004),  # pinion stub (estimate)
    ("housing", (*HOUSING_XY, MAT_TOP + 0.008), 0.0375, 0.008),  # rack housing, 16 mm (estimate)
    ("coupling", (*COUPLING_XY, MAT_TOP + 0.00845), 0.0375, 0.00845),  # coupling, 75 x 16.9 mm
]
SYRINGE = ((*SYRINGE_XY, MAT_TOP + 0.011), 0.011, 0.060)  # lying along y: centre, radius, half length (estimate)


def _hande_shapes():
    """The opened gripper, its housing, its coupling and the syringe, in a part frame at the body's base."""
    o = np.array([*HANDE_BODY_XY, MAT_TOP])
    colors = {"base": HANDE_GREY, "body": HANDE_BLACK, "pinion": ALU, "housing": HANDE_BLACK, "coupling": ALU}
    out = [parts.cylinder(r, 2 * h, at=tuple(np.asarray(c) - o), color=colors[k]) for k, c, r, h in HANDE_CYLS]
    # the housing's two rack channels and the coupling's contact block, drawn only
    for dy in (-0.01425, 0.01425):
        out.append(parts.box((0.070, 0.0062, 0.0008), at=(HOUSING_XY[0] - o[0], HOUSING_XY[1] + dy - o[1], 0.0164),
                             color=HOLE, collide=False))
    out.append(parts.box((0.012, 0.008, 0.0008), at=(COUPLING_XY[0] - o[0], COUPLING_XY[1] - o[1], 0.0173),
                         color=(0.80, 0.65, 0.20), collide=False))
    c, r, h = SYRINGE
    out.append(parts.cylinder(r, 2 * h, at=tuple(np.asarray(c) - o), color=SYRINGE_CLEAR, axis="y"))
    return out


HANDE = parts.Part("opened_hande", pos=(*HANDE_BODY_XY, MAT_TOP), shapes=_hande_shapes(), friction=(0.6, 0.5))

# -- the two racks (loose). Part frame: x along the rack, y across it, z up through it; origin at the middle of its
# bounding box. Section from the maker's Fig. 5-11 (Racks in rack housing), read at the drawing's scale: a lower stem
# 6.1 wide x 4.8 tall (the back face, the one opposite the tapped holes, is its underside), a guide flange 14.0 wide x
# 6.0 tall, and an upper stem 6.1 wide x 5.3 tall whose top is the tapped face (it stands 1.5 mm proud of the
# housing). Length 62 mm. ---------------------------------------------------------------------------------------------
RACK_L = 0.062
STEM_W, FLANGE_W = 0.0061, 0.014
H_LOW, H_FLANGE, H_UP = 0.0048, 0.0060, 0.0053
RACK_H = H_LOW + H_FLANGE + H_UP  # 16.1 mm
Z0 = -RACK_H / 2  # the back face, in the part frame
Z_FL0, Z_FL1, Z_TAP = Z0 + H_LOW, Z0 + H_LOW + H_FLANGE, RACK_H / 2
RACK_MASS = 0.071  # steel, 9.0 cm3 (estimate)
RACK_SIZE = (RACK_L, FLANGE_W, RACK_H)
HOLES_X = (0.009, 0.0135, 0.018, 0.0225, 0.027)  # M3, 2 mm dowel, M3, 2 mm dowel, M3 at 4.5 mm pitch (Fig. 5-11)


def _rack_shapes():
    out = [
        parts.box((RACK_L, STEM_W, H_LOW), at=(0.0, 0.0, Z0 + H_LOW / 2), color=STEEL),
        parts.box((RACK_L, FLANGE_W, H_FLANGE), at=(0.0, 0.0, Z_FL0 + H_FLANGE / 2), color=STEEL),
        parts.box((RACK_L, STEM_W, H_UP), at=(0.0, 0.0, Z_FL1 + H_UP / 2), color=STEEL),
    ]
    for k, x in enumerate(HOLES_X):  # tapped holes and dowel holes on the tapped face, drawn only
        out.append(parts.cylinder(0.0015 if k % 2 == 0 else 0.0010, 0.0004, at=(x, 0.0, Z_TAP + 0.0001),
                                  color=HOLE, collide=False))
    pitch = math.pi * 0.001  # module 1 teeth along one side of the lower stem (estimate), drawn only
    for k in range(-9, 10):
        out.append(parts.box((0.0007, 0.0004, H_LOW * 0.9), at=(k * pitch, STEM_W / 2 + 0.0001, Z0 + H_LOW / 2),
                             color=TOOTH, collide=False))
    return out


# -- grease strips: every face of a rack but its two end faces, in 16 rows along it. Zones: 0 = the back (back face,
# the lower stem's sides, the flange's undersides), 1 = the +y side (flange side, the step on top of the flange, the
# upper stem's side), 2 = the -y side, 3 = the tapped face (must stay dry). -------------------------------------------
N_ROWS = 16
ROW = RACK_L / N_ROWS
LEDGE_W = (FLANGE_W - STEM_W) / 2
STRIPS = []  # (zone, label, y, z, normal (part frame), half-size across the face (m), axis across ("y" or "z"))
for j in (-1, 0, 1):  # the back face in three strips across: bristles wrapped round from a side reach only its edges
    STRIPS.append((0, f"back face {'-+'[j > 0] if j else 'middle'}", j * STEM_W / 3, Z0, (0, 0, -1), STEM_W / 6, "y"))
for s, side in ((1, "+y"), (-1, "-y")):
    STRIPS.append((0, f"lower stem side {side}", s * STEM_W / 2, Z0 + H_LOW / 2, (0, s, 0), H_LOW / 2, "z"))
    STRIPS.append((0, f"flange underside {side}", s * (STEM_W / 2 + LEDGE_W / 2), Z_FL0, (0, 0, -1), LEDGE_W / 2, "y"))
for s, zone, side in ((1, 1, "+y"), (-1, 2, "-y")):
    STRIPS.append((zone, f"flange side {side}", s * FLANGE_W / 2, Z_FL0 + H_FLANGE / 2, (0, s, 0), H_FLANGE / 2, "z"))
    STRIPS.append((zone, f"flange top {side}", s * (STEM_W / 2 + LEDGE_W / 2), Z_FL1, (0, 0, 1), LEDGE_W / 2, "y"))
    STRIPS.append((zone, f"upper stem side {side}", s * STEM_W / 2, Z_FL1 + H_UP / 2, (0, s, 0), H_UP / 2, "z"))
for j in (-1, 0, 1):
    STRIPS.append((3, "tapped face", j * STEM_W / 3, Z_TAP, (0, 0, 1), STEM_W / 6, "y"))
LIFT = 0.0002  # grease drawn this far proud of its face


def _make_cells():
    pts, nrm, zone, quads = [], [], [], []
    for k in range(N_ROWS):
        x = -RACK_L / 2 + (k + 0.5) * ROW
        for z_, label, y, z, n, half, axis in STRIPS:
            p = np.array([x, y, z])
            nn = np.asarray(n, float)
            pts.append(p)
            nrm.append(nn)
            zone.append(z_)
            u = np.array([0.45 * ROW, 0.0, 0.0])
            v = np.array([0.0, 0.95 * half, 0.0]) if axis == "y" else np.array([0.0, 0.0, 0.95 * half])
            c = p + nn * LIFT
            quads.append([c - u - v, c + u - v, c + u + v, c - u + v])
    return np.array(pts), np.array(nrm), np.array(zone), quads


CELL_P, CELL_N, CELL_ZONE, CELL_QUADS = _make_cells()
N_CELLS = len(CELL_P)
N_ZONES = 3  # per rack, scored
ZONE_NAMES = ("back", "+y side", "-y side")
TAPPED = CELL_ZONE == 3
# for the finger rule: each strip's half length along the rack (kept 0.4 mm short of the end faces) and half width
# across its face, and which axis runs across it (1: y, 2: z)
CELL_HALF = np.array([[0.4 * ROW, half] for _ in range(N_ROWS) for *_, half, axis in STRIPS])
CELL_AX = np.array([1 if axis == "y" else 2 for _ in range(N_ROWS) for *_, axis in STRIPS])
# for the bristle rule: each strip's full extent, along the rack (unit vector, half length) and across its face
STRIP_ALONG = np.array([1.0, 0.0, 0.0])
STRIP_ACROSS = np.array([[0.0, 1.0, 0.0] if a == 1 else [0.0, 0.0, 1.0] for a in CELL_AX])
STRIP_HALF = np.array([[ROW / 2, h] for h in CELL_HALF[:, 1]])


def _grease_visuals():
    out = []
    for k, q in enumerate(CELL_QUADS):
        a, b, c = q[:3]
        outward = float(np.cross(b - a, c - a) @ CELL_N[k]) > 0
        tris = ((0, 1, 2), (0, 2, 3)) if outward else ((0, 2, 1), (0, 3, 2))
        out.append(parts.Visual(f"g{k}", tuple(tuple(float(v) for v in p) for p in q), tris, color=GREASE_RED,
                                visible=False))  # shown once greased
    return tuple(out)


RACK_NAMES = ("rack_1", "rack_2")
RACK_Y = 0.030  # the racks lie along y in the tray, side by side, on the +y half (lot A)
RACK_XS = (0.270, 0.210)
RACK_Z = FLOOR + RACK_H / 2
RACK_YAW = math.pi / 2  # the part's x (along the rack) is the world's y


def _rack(name, x):
    return parts.Part(name, pos=(x, RACK_Y, RACK_Z), yaw=RACK_YAW, shapes=_rack_shapes(), mass=RACK_MASS,
                      friction=(0.4, 0.35), report=("base",), visuals=_grease_visuals(), contact_offset=0.002)


RACKS = [_rack("rack_1", RACK_XS[0]), _rack("rack_2", RACK_XS[1])]

# -- the brush (loose): a Libman 18 tile and grout brush, 248 mm, nylon bristles 7/8 in (22 mm). Part frame: origin
# at the middle of the bristle base (the head's top face), bristles up (+z), the handle along -y. -------------------------
HEAD = (0.025, 0.065, 0.012)  # x, y, z (estimate)
TUFT = (0.020, 0.056, 0.022)  # the bristles' outline: a 20 x 56 mm block (estimate), 22 mm long
SOFT = 0.005  # the outer bristles give way this far before the packed core resists: a part sinks this far in
TUFT_CORE = (TUFT[0] - 2 * SOFT, TUFT[1] - 2 * SOFT, TUFT[2] - SOFT)  # the stiff core, what the simulator collides
TUFT_CORE_AT = np.array([0.0, 0.0, TUFT_CORE[2] / 2])
HANDLE = (0.022, 0.183, 0.012)
HANDLE_Y = -(HEAD[1] / 2 + HANDLE[1] / 2)
TUFT_AT = np.array([0.0, 0.0, TUFT[2] / 2])
BRUSH_MASS = 0.060  # estimate
GRIP_BACK = 0.180  # where a hand takes the handle: this far from the head's middle
BRUSH_X = 0.320  # the brush lies across the tray at this x
BRUSH_HEAD_Y = 0.060  # head over the +y wall in lot A (the -y wall in lot B)
BRUSH_Z = WALL_TOP + HEAD[2]  # its underside on the wall tops


def _brush_shapes():
    return [
        parts.box(HEAD, at=(0.0, 0.0, -HEAD[2] / 2), color=HANDLE_WHITE),
        parts.box(HANDLE, at=(0.0, HANDLE_Y, -HANDLE[2] / 2), color=HANDLE_WHITE),
        parts.box(TUFT, at=tuple(TUFT_AT), color=BRISTLE_RED, collide=False),  # the bristles as seen
        parts.box(TUFT_CORE, at=tuple(TUFT_CORE_AT), color=BRISTLE_RED),  # their packed core, inside them
    ]


BRUSH = parts.Part("brush", pos=(BRUSH_X, BRUSH_HEAD_Y, BRUSH_Z), shapes=_brush_shapes(), mass=BRUSH_MASS,
                   friction=(0.15, 0.12), report=("base",), contact_offset=0.002, angular_damping=2.0)  # greasy

OBJECTS = [MAT, TRAY, HANDE, *RACKS, BRUSH]

# Layout lots: trial k uses lot k mod 2. Lot A: the brush's head lies over the tray's +y wall, its handle toward the
# robot's right, and the racks lie on the tray's +y half. Lot B: all mirrored: the brush turned end for end (head over
# the -y wall, handle toward the left) and the racks on the -y half.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"brush": (BRUSH_X, BRUSH_HEAD_Y, BRUSH_Z)}),
    lots.Lot("B", place={"brush": (BRUSH_X, -BRUSH_HEAD_Y, BRUSH_Z), "rack_1": (RACK_XS[0], -RACK_Y, RACK_Z),
                         "rack_2": (RACK_XS[1], -RACK_Y, RACK_Z)}, yaw={"brush": math.pi}),
)

# -- the greasing rule and the trial record ----------------------------------------------------------------------------
F_MIN = 0.2  # N between the brush and that rack
WINDOW = 4  # control steps (0.2 s): the bristles must have moved MOVE_MIN against the rack over the last WINDOW steps
MOVE_MIN = 0.0006  # m, i.e. 3 mm/s: slower is a held press that only jitters
STEP_MAX = 0.012  # m a control step: a faster move is a jump, not brushing
GREASE_TRAVEL = 0.002  # m of bristles sliding past a strip, within reach and pressing, greases it
ZONE_FRACTION = 0.9  # a zone is greased when at least 90% of its strips are
HAND_CLEAR = 0.030  # m: hands this far from both racks and the brush at the end
GRIPPER_TOUCH = 0.001  # m: bristles this close to the opened gripper, its housing or coupling (or the syringe) count
FINGER_TOUCH = 0.001  # m: a finger or gripper surface this close outside a strip's face, over the strip, touches it
HOLD_STEPS = int(SUCCESS_HOLD_S * 20)
_S = parts.__dict__.setdefault("_rack_grease_state", {})


def reset_state():
    shown = set(_S.get("shown", ()))
    _S.clear()
    _S.update(travel={n: np.zeros(N_CELLS) for n in RACK_NAMES}, prev={n: None for n in RACK_NAMES}, known=False,
              broken=False, landed=False, gripper_hit=False, clear=False, hold=0, views=None, shown=shown,
              maxforce=0.0, log_t=-1.0, greasy={"left_robot": False, "right_robot": False},
              finger_on_tapped=False, wiped=0)


if not _S:
    reset_state()


def _frame(st):
    return np.asarray(st.pos, float), parts._quat_to_matrix(st.quat_xyzw)


def tuft_box(brush_state):
    """(centre, rotation, size) of the bristles' outline in the world."""
    p, r = _frame(brush_state)
    return p + r @ TUFT_AT, r, np.asarray(TUFT)


def _box_dist(local, half):
    """Distance from points (in a box's frame) to the box, and the box point nearest each."""
    nearest = np.clip(local, -half, half)
    return np.linalg.norm(local - nearest, axis=1), nearest


def record_brush(forces, brush_state, rack_states):
    """One control step of brushing. ``forces``: N between the brush and each rack; ``brush_state`` and
    ``rack_states`` give poses (``rack_states``: name -> state). The rack cannot enter the bristles' packed core (the
    simulator's collider), but it sinks into their soft outer SOFT mm, so the bristles' outline overlaps it. A strip
    gains the distance the bristles moved against the rack since the last step (in any direction: a stroke along the
    face or a dab into it) while (a) any part of the strip lies inside the bristles' outline, (b) the brush has pressed that rack
    with at least F_MIN on this step and the last, and (c) the bristles have moved at least MOVE_MIN against the rack
    over the last WINDOW steps (a held press that only jitters counts nothing). A step over STEP_MAX counts nothing."""
    tc, tr, ts = tuft_box(brush_state)
    half = ts / 2
    for name in RACK_NAMES:
        rp, rr = _frame(rack_states[name])
        c_local = rr.T @ (tc - rp)  # the outline's centre in the rack's frame
        r_local = rr.T @ tr
        loaded = forces.get(name, 0.0) >= F_MIN - EPS
        hist = _S["prev"][name] or []
        hist = (hist + [(c_local, loaded)])[-(WINDOW + 1):]
        _S["prev"][name] = hist
        if len(hist) < 2 or not loaded or not hist[-2][1]:
            continue
        step = float(np.linalg.norm(c_local - hist[-2][0]))
        moved = float(np.linalg.norm(c_local - hist[0][0])) if len(hist) == WINDOW + 1 else 0.0
        if step > STEP_MAX + EPS or moved < MOVE_MIN - EPS:
            continue
        q = (CELL_P - c_local) @ r_local  # strip middles in the outline's frame
        ext = (np.abs(STRIP_ALONG @ r_local)[None, :] * STRIP_HALF[:, :1]
               + np.abs(STRIP_ACROSS @ r_local) * STRIP_HALF[:, 1:])  # each strip's half extent in that frame
        touched = np.all(np.abs(q) <= half + ext - 1e-5, axis=1)  # any part of the strip inside the outline
        _S["travel"][name][touched] += step


def record_hands(hand_pts, rack_states):
    """Fingers and grippers on the racks. ``hand_pts``: arm -> world points on that arm's gripper and finger
    surfaces; ``rack_states``: name -> state. A hand point within FINGER_TOUCH outside a strip's face and over the
    strip (away from the rack's two end faces, where a rack is held) touches it: a greased strip it touches is wiped
    bare and the hand is greasy from then on; a greasy hand that touches a tapped face greases it."""
    for name in RACK_NAMES:
        rp, rr = _frame(rack_states[name])
        for arm, pts in hand_pts.items():
            local = (np.asarray(pts, float) - rp) @ rr
            box = np.asarray(RACK_SIZE) / 2 + FINGER_TOUCH + 0.0005
            near = local[np.all(np.abs(local) <= box, axis=1)]
            near = near[np.abs(near[:, 0]) <= RACK_L / 2 - 0.0005]
            if not len(near):
                continue
            rel = near[:, None, :] - CELL_P[None, :, :]
            dn = np.einsum("pkj,kj->pk", rel, CELL_N)
            along = np.abs(rel[:, :, 0]) <= CELL_HALF[None, :, 0] + EPS
            across = np.abs(np.take_along_axis(rel, CELL_AX[None, :, None], axis=2)[:, :, 0]) <= CELL_HALF[None, :, 1]
            touch = np.any((dn >= -0.0005) & (dn <= FINGER_TOUCH + EPS) & along & across, axis=0)
            if not touch.any():
                continue
            g = greased(name)
            if np.any(touch & ~TAPPED & g):
                _S["greasy"][arm] = True
                _S["wiped"] += int(np.sum(touch & ~TAPPED & g))
                _S["travel"][name][touch & ~TAPPED] = 0.0
            if _S["greasy"][arm] and np.any(touch & TAPPED):
                _S["finger_on_tapped"] = True


def no_brush():
    for name in RACK_NAMES:
        _S["prev"][name] = None


def greased(name) -> np.ndarray:
    return _S["travel"][name] >= GREASE_TRAVEL - EPS


def zones_greased(name) -> list[bool]:
    g = greased(name)
    return [bool(np.mean(g[CELL_ZONE == z]) >= ZONE_FRACTION - EPS) for z in range(N_ZONES)]


def tapped_greased() -> bool:
    """Any grease at all on a tapped face: one brushing step that reaches it is enough (a dab leaves grease)."""
    return any(bool(np.any(_S["travel"][n][TAPPED] > 0.0)) for n in RACK_NAMES)


def n_zones() -> int:
    return int(sum(sum(zones_greased(n)) for n in RACK_NAMES))


def racks_full() -> int:
    return int(sum(all(zones_greased(n)) for n in RACK_NAMES))


# -- end-state and whole-trial geometry --------------------------------------------------------------------------------
def _corners(st, size):
    c, r = _frame(st)
    h = np.asarray(size) / 2
    signs = np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)], float)
    return c + (signs * h) @ r.T


def rack_in_tray(st) -> bool:
    """Lying on the tray's floor: its lowest corner within 3 mm of the floor and all of it inside the walls."""
    k = _corners(st, RACK_SIZE)
    low = float(k[:, 2].min())
    inside = (np.all(np.abs(k[:, 0] - TRAY_CENTER[0]) <= TRAY_IN[0] + EPS)
              and np.all(np.abs(k[:, 1] - TRAY_CENTER[1]) <= TRAY_IN[1] + EPS))
    return bool(inside and FLOOR - 0.001 - EPS <= low <= FLOOR + 0.003 + EPS)


def rack_landed_outside(st) -> bool:
    """Down on the mat or the table outside the tray (or off the table)."""
    k = _corners(st, RACK_SIZE)
    c = np.asarray(st.pos, float)
    out_xy = (abs(c[0] - TRAY_CENTER[0]) > TRAY_SIZE[0] / 2 or abs(c[1] - TRAY_CENTER[1]) > TRAY_SIZE[1] / 2)
    return bool((out_xy and float(k[:, 2].min()) <= MAT_TOP + 0.004) or float(k[:, 2].min()) < TOP - 0.010)


def brush_ends(st):
    """World points of the brush's two ends (head end, handle end) on its centre line, at its underside."""
    c, r = _frame(st)
    head = c + r @ np.array([0.0, HEAD[1] / 2, -HEAD[2]])
    tail = c + r @ np.array([0.0, HANDLE_Y - HANDLE[1] / 2, -HANDLE[2]])
    return head, tail


def brush_back(st) -> bool:
    """Back across the tray: bristles up (within 10 degrees), lying across both long walls (its two ends beyond them,
    its underside within 3 mm of their tops) and over the tray along x."""
    c, r = _frame(st)
    if float(r[2, 2]) < math.cos(math.radians(10.0)) - EPS:
        return False
    head, tail = brush_ends(st)
    ys = sorted((float(head[1]), float(tail[1])))
    across = ys[0] <= TRAY_CENTER[1] - TRAY_SIZE[1] / 2 - EPS and ys[1] >= TRAY_CENTER[1] + TRAY_SIZE[1] / 2 + EPS
    level = all(WALL_TOP - 0.003 - EPS <= float(p[2]) <= WALL_TOP + 0.003 + EPS for p in (head, tail))
    over = all(abs(float(p[0]) - TRAY_CENTER[0]) <= TRAY_IN[0] - 0.015 + EPS for p in (head, tail))
    return bool(across and level and over)


def _tuft_points(st, step=0.003):
    c, r, s = tuft_box(st)
    h = s / 2
    g = [np.linspace(-h[i], h[i], int(math.ceil(2 * h[i] / step)) + 1) for i in range(3)]
    pts = np.array([[a, b, d] for a in g[0] for b in g[1] for d in g[2]
                    if abs(abs(a) - h[0]) < 1e-9 or abs(abs(b) - h[1]) < 1e-9 or abs(abs(d) - h[2]) < 1e-9])
    return c + pts @ r.T


def bristles_near_gripper(brush_state) -> bool:
    """The bristle block within GRIPPER_TOUCH of the opened gripper, its housing, its coupling or the syringe."""
    pts = _tuft_points(brush_state)
    for _, c, rad, hh in HANDE_CYLS:
        d_r = np.linalg.norm(pts[:, :2] - np.asarray(c[:2]), axis=1) - rad
        d_z = np.abs(pts[:, 2] - c[2]) - hh
        d = np.where((d_r > 0) & (d_z > 0), np.hypot(d_r, d_z), np.maximum(d_r, d_z))
        if float(d.min()) <= GRIPPER_TOUCH + EPS:
            return True
    c, rad, hh = SYRINGE
    d_r = np.linalg.norm(pts[:, [0, 2]] - np.asarray([c[0], c[2]]), axis=1) - rad
    d_y = np.abs(pts[:, 1] - c[1]) - hh
    d = np.where((d_r > 0) & (d_y > 0), np.hypot(d_r, d_y), np.maximum(d_r, d_y))
    return bool(float(d.min()) <= GRIPPER_TOUCH + EPS)


# -- scoring ---------------------------------------------------------------------------------------------------------
def _ruled_out() -> bool:
    return bool(not _S["known"] or _S["broken"] or _S["landed"] or _S["gripper_hit"] or tapped_greased()
                or _S["finger_on_tapped"])


def score(objects):
    """Greased zones, judged once both racks lie in the tray, both hands are 30 mm clear of the racks and the brush,
    and the racks and the brush have been still for 1 s."""
    if _ruled_out() or not _S["clear"] or _S["hold"] < HOLD_STEPS:
        return 0.0
    racks = [objects[n] for n in RACK_NAMES]
    if not all(r.resting() and rack_in_tray(r) for r in racks) or not objects["brush"].resting():
        return 0.0
    full = racks_full()
    if full == 2 and brush_back(objects["brush"]):
        return 1.0
    if full == 2:
        return 0.75
    if full == 1:
        return 0.5
    if n_zones() >= 2:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) == 1.0


# -- simulator side: contact views, hands, the grease's look ------------------------------------------------------------
HAND_LINKS = ("gripper", "tip_left", "tip_right")
_FACE_PTS: dict[str, np.ndarray] = {}


def _arr(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy())
    return np.asarray(x)


def _make_views():
    from isaaclab_physx.physics import PhysxManager

    sv = PhysxManager.get_physics_sim_view()
    brush = parts._PATHS["brush"]
    view = sv.create_rigid_contact_view(brush, filter_patterns=[parts._PATHS[n] for n in RACK_NAMES],
                                        max_contact_data_count=64)
    print(f"[grease] contact view: brush -> {len(RACK_NAMES)} racks", file=sys.stderr, flush=True)
    return {"brush": view}


def _hull_surface(h, spacing=0.003):
    from scipy.spatial import ConvexHull

    v = h.vertices
    out = [v]
    for tri in ConvexHull(v).simplices:
        a, b, c = v[tri]
        n = max(1, int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / spacing)))
        out.append(np.array([a + (b - a) * u / n + (c - a) * w / n for u in range(n + 1) for w in range(n + 1 - u)]))
    return np.unique(np.round(np.concatenate(out) / 0.0025).astype(np.int64), axis=0) * 0.0025


def hand_points(base) -> dict:
    """Points on the faces (every 3 mm) of each arm's gripper and finger collision hulls, in the world: arm -> array."""
    from yam_arena import analysis

    origin = parts._np(base.scene.env_origins[0])
    out, n_links = {}, 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        pts = []
        for i, n in enumerate(asset.body_names):
            if n not in HAND_LINKS:
                continue
            if n not in _FACE_PTS:
                _FACE_PTS[n] = np.concatenate([_hull_surface(h) for h in analysis.link_hulls().get(n, [])])
            pts.append(_FACE_PTS[n] @ parts._quat_to_matrix(poses[i, 3:]).T + (poses[i, :3] - origin))
            n_links += 1
        out[arm] = np.concatenate(pts) if pts else np.zeros((0, 3))
    if n_links != 6:
        raise RuntimeError("hand links not found")
    return out


def object_boxes(objects):
    """(centre, rotation, size) of every collider of both racks and the brush, where they are now."""
    out = []
    for name in (*RACK_NAMES, "brush"):
        part = RACKS[RACK_NAMES.index(name)] if name != "brush" else BRUSH
        c, r = _frame(objects[name])
        out += [(cc, rr, s) for _, cc, rr, s in part.world_boxes({"base": (c, r)})]
    return out


def hands_clear(points, objects) -> bool:
    from yam_arena import analysis

    return all(float(analysis.box_signed_distance(points, bc, br, bs).min()) >= HAND_CLEAR - EPS
               for bc, br, bs in object_boxes(objects))


@parts.on_reset
def _on_reset(sim) -> None:
    views = _S.get("views")
    shown = set(_S.get("shown", ()))
    reset_state()
    _S["views"] = views
    for name, vis in shown:  # the grease is authored hidden; hide what the last trial showed
        try:
            sim.set_visible(name, vis, False)
        except Exception as exc:  # noqa: BLE001
            print(f"[grease] could not hide {name}/{vis}: {exc}", file=sys.stderr)
            break
    _S["shown"] = set()


@parts.on_step
def _on_step(sim) -> None:
    try:
        base = sim.env.unwrapped
        if _S.get("views") is None:
            _S["views"] = _make_views()
        v = _S["views"]
        dt = base.physics_dt
        st = sim.states()
        m = _arr(v["brush"].get_contact_force_matrix(dt=dt)).reshape(-1, len(RACK_NAMES), 3).sum(axis=0)
        forces = {n: float(np.linalg.norm(m[i])) for i, n in enumerate(RACK_NAMES)}
        _S["maxforce"] = max(_S["maxforce"], *forces.values())
        if max(forces.values()) > 0.02:
            record_brush(forces, st["brush"], st)
        else:
            no_brush()
        if any(rack_landed_outside(st[n]) for n in RACK_NAMES):
            _S["landed"] = True
        if bristles_near_gripper(st["brush"]):
            _S["gripper_hit"] = True
        hp = hand_points(base)
        record_hands(hp, st)
        _S["clear"] = hands_clear(np.concatenate(list(hp.values())), st)
        _S["known"] = True
        still = all(st[n].resting() for n in (*RACK_NAMES, "brush"))
        _S["hold"] = _S["hold"] + 1 if (_S["clear"] and still) else 0
        for n in RACK_NAMES:
            g = greased(n)
            for k in range(N_CELLS):
                key = (n, f"g{k}")
                if g[k] and key not in _S["shown"]:
                    sim.set_visible(n, f"g{k}", True)
                    _S["shown"].add(key)
                elif not g[k] and key in _S["shown"]:  # wiped off by a finger
                    sim.set_visible(n, f"g{k}", False)
                    _S["shown"].discard(key)
        if sim.t - _S["log_t"] >= 1.0 - 1e-6:
            _S["log_t"] = sim.t
            print("[grease] " + json.dumps(dict(
                t=round(float(sim.t), 2), forces={k: round(f, 2) for k, f in forces.items()},
                maxforce=round(_S["maxforce"], 2),
                strips={n: [int(np.sum(greased(n)[CELL_ZONE == z])) for z in range(4)] for n in RACK_NAMES},
                zones=n_zones(), tapped=tapped_greased(), greasy=_S["greasy"], wiped=_S["wiped"],
                finger_on_tapped=_S["finger_on_tapped"],
                missing={n: [[int(k) for k in np.flatnonzero(~greased(n) & (CELL_ZONE == z))]
                             if np.sum(~greased(n) & (CELL_ZONE == z)) <= 12 else int(np.sum(~greased(n) & (CELL_ZONE == z)))
                             for z in range(N_ZONES)] for n in RACK_NAMES}, landed=_S["landed"], gripper=_S["gripper_hit"],
                clear=_S["clear"], hold=_S["hold"],
                racks={n: [round(float(x), 4) for x in (*st[n].pos, *st[n].quat_xyzw)] for n in RACK_NAMES},
                brush=[round(float(x), 4) for x in (*st["brush"].pos, *st["brush"].quat_xyzw)])), flush=True)
    except Exception as exc:  # unreadable contacts, hands or poses: no credit for this trial
        _S["broken"] = True
        print(f"[grease] step check failed: {exc!r}", file=sys.stderr, flush=True)


# -- the scripted demo: open loop from the visible layout (where the racks and the brush lie). One arm takes the brush's
# handle where it lies and keeps the brush on the tray's rims; the other takes each rack by its two end faces and works
# its back, then each side, over the loaded bristles, then lays it back where it lay. ------------------------------------
FINGER_RACK = -0.0295  # 59 mm between the pads: 1.5 mm into each of the rack's end faces (62 mm apart)
FINGER_HANDLE = -0.0095  # 19 mm: 1.5 mm into each side of the 22 mm handle
FINGER_AROUND = -0.035  # 70 mm: open round a rack's ends without reaching the tray's wall
GRIP_Z = 0.017  # the rack hand's TCP above the rack's back face: its fingertips 4 mm above that face
PRESS = 0.0015  # the back face is brought this far down into the bristle tips
PRESS_SIDE = 0.007  # a side is commanded this far into the bristles' core: the arm stops 3-5 mm short of a target
# there (measured in a trace), so this gives a firm press
WIGGLE_Y = 0.006  # back pass: the rack slides +/- this far along its length (the fingers ride above the bristles)
SIDE_Z = 0.0105  # side pass: the rack's back face this far above the brush head's top: the bristle tips then reach
# 0.7 mm up the upper stem's side and stay 4.6 mm below the tapped face (the held rack sags about 2 mm, measured)
SIDE_SLIDE = 0.003  # and slid +/- this far along its length (a vertical stroke would roll it in the end-face grip)
CLEAR = 0.012  # rack's back face this far above the bristle tips when moving round them
YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # the brush hand: fingers closing along x, across the handle
YAW_RACK = 0.0  # the rack hand: fingers closing along y, on the rack's end faces


def _wp(dur, pos, finger, label, yaw):
    return Waypoint(float(dur), tuple(float(v) for v in pos), float(finger), label, float(yaw))


def _park(arm):
    return np.array([0.18, 0.24 if arm == "left" else -0.24, TOP + 0.20])


def _brush_tcp(head, s):
    """The TCP for the brush's head middle at ``head`` (bristle base), the handle running toward s * y."""
    return np.asarray(head, float) + np.array([0.0, s * GRIP_BACK, -HEAD[2] / 2])


def demo():
    lot = lots.current()
    bx, by, bz = lot.pos("brush")
    s = -1.0 if by > 0 else 1.0  # the handle runs this way along y, toward the arm that keeps the brush
    keeper = "right" if s < 0 else "left"
    mover = "left" if keeper == "right" else "right"
    yk = YAW[keeper]
    K, M = [], []
    tips = bz + TUFT_CORE[2]  # the top of the bristles' packed core (the soft tips above it give way)

    def mv(dt, back_face_xyz, f, label):  # the rack hand, given where the rack's back face middle should be
        x, y, z = back_face_xyz
        M.append(_wp(dt, (x, y, z + GRIP_Z), f, label, YAW_RACK))

    def t(w):
        return sum(x.duration for x in w)

    def sync():  # the arm that is ahead waits for the other
        dt = t(M) - t(K)
        if dt > 1e-6:
            K.append(_wp(dt, K[-1].pos, K[-1].finger, "wait", K[-1].yaw))
        elif dt < -1e-6:
            M.append(_wp(-dt, M[-1].pos, M[-1].finger, "wait", M[-1].yaw))

    up = np.array([0.0, 0.0, 0.05])
    grip = _brush_tcp((bx, by, bz), s)
    park_k = _park(keeper)
    K.append(_wp(1.0, park_k, FINGER_OPEN, "stand clear", 0.0))

    def keep():  # the keeping arm takes the brush's handle where it lies across the rims and holds it there
        K.append(_wp(2.0, grip + up, FINGER_OPEN, "over the brush handle", yk))
        K.append(_wp(1.2, grip, FINGER_OPEN, "down to the handle", yk))
        K.append(_wp(0.8, grip, FINGER_HANDLE, "hold the brush on the rims", yk))

    def leave():  # lets go and stands clear while the other arm takes or lays down a rack
        K.append(_wp(0.8, grip, FINGER_OPEN, "let go of the brush", yk))
        K.append(_wp(1.0, grip + up, FINGER_OPEN, "up", yk))
        K.append(_wp(1.5, park_k, FINGER_OPEN, "stand clear", 0.0))

    def passes():
        # the back: down on the bristle tips and slid to and fro along the rack's length
        above = (bx, by, tips + CLEAR)
        mv(1.0, (bx, by, tips - PRESS), FINGER_RACK, "back face down on the bristles")
        for _ in range(2):
            mv(0.9, (bx, by + WIGGLE_Y, tips - PRESS), FINGER_RACK, "slide it along them")
            mv(1.8, (bx, by - WIGGLE_Y, tips - PRESS), FINGER_RACK, "and back")
            mv(0.9, (bx, by, tips - PRESS), FINGER_RACK, "and to the middle")
        mv(0.8, above, FINGER_RACK, "up off the bristles")
        # each side pressed against a long face of the bristle block, worked up and down
        for side in (1.0, -1.0):
            x_press = bx + side * (TUFT_CORE[0] / 2 + FLANGE_W / 2 - PRESS_SIDE)
            x_off = x_press + side * 0.010
            zs = bz + SIDE_Z
            mv(1.2, (x_off, by, tips + CLEAR), FINGER_RACK, "beside the bristles")
            mv(0.8, (x_off, by, zs), FINGER_RACK, "down beside them")
            mv(0.8, (x_press, by, zs), FINGER_RACK, "side pressed into the bristles")
            for _ in range(2):
                mv(0.6, (x_press, by + SIDE_SLIDE, zs), FINGER_RACK, "slide it along them")
                mv(1.2, (x_press, by - SIDE_SLIDE, zs), FINGER_RACK, "and back")
                mv(0.6, (x_press, by, zs), FINGER_RACK, "and to the middle")
            mv(0.8, (x_off, by, zs), FINGER_RACK, "off the bristles")
            mv(0.8, (x_off, by, tips + CLEAR), FINGER_RACK, "up")

    for k, name in enumerate(("rack_1", "rack_2")):
        rx, ry, rz = lot.pos(name)
        spot = np.array([rx, ry, rz - RACK_H / 2])  # its back face's middle where it lies
        mv(2.0 if k == 0 else 1.5, spot + up, FINGER_AROUND, f"over {name}")
        mv(1.5, spot, FINGER_AROUND, "down round its ends")
        mv(0.8, spot, FINGER_RACK, "grip its end faces")
        mv(1.0, spot + [0.0, 0.0, 0.03], FINGER_RACK, "lift")
        mv(2.0, (bx, by, tips + CLEAR), FINGER_RACK, "over the bristles")
        sync()
        keep()  # the rack is held up out of the way while the other arm takes the brush's handle
        sync()
        passes()
        sync()
        leave()
        sync()
        drop = spot + np.array([0.0, 0.0, 0.003])
        mv(2.0, drop + [0.0, 0.0, 0.03], FINGER_RACK, "carry it back to its place")
        mv(1.2, drop, FINGER_RACK, "lower it")
        mv(0.8, drop, FINGER_AROUND, "let go")
        mv(1.0, drop + up, FINGER_AROUND, "up")
    M.append(_wp(1.5, _park(mover), FINGER_OPEN, "clear", 0.0))
    sync()
    M.append(_wp(2.0, M[-1].pos, FINGER_OPEN, "wait", 0.0))
    K.append(_wp(2.0, K[-1].pos, FINGER_OPEN, "wait", 0.0))
    return {mover: M, keeper: K}
