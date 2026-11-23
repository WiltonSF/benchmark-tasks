"""A plumber's bench with two small bench vises. The left-hand vise (nearer the robot's right arm) holds a white
1-1/2 inch PVC pipe upright; a black line is drawn round the pipe 18 mm below its top end. The other vise holds a
short upright pipe stub with a white PVC coupling cemented on top, its upper socket open and facing up. A small can
of blue PVC cement stands on the bench with its lid resting loosely on the neck; a wool dauber on a wire hangs from
the lid into the cement. The can stands in a different place in some trials.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys
from functools import lru_cache

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
EPS = 1e-9

# ---------------------------------------------------------------------------------------------------------------
# Sizes (m). Pipe: 1-1/2 in Schedule 40 PVC, OD 1.900 in, minimum wall 0.145 in (maker's catalogue).
# Coupling: 1-1/2 in PVC DWV hub x hub coupling, 1-9/16 in long with a 1/8 in centre stop (maker's catalogue), so
# each hub is 18.26 mm deep. Socket and outside diameters are estimates (ASTM D2665 class).
# ---------------------------------------------------------------------------------------------------------------
R_PIPE = 0.02413  # pipe outside radius
R_BORE = 0.02045  # pipe inside radius (also the coupling stop's inside radius)
DEPTH = 0.01826  # socket depth = the band to coat on the pipe end
R_SOCK = 0.02415  # socket wall radius (mouth 48.6 / root 48.2 mm, estimate)
R_COUP = 0.0280  # coupling outside radius (estimate)
STOP = 0.003175  # centre stop height
COUPLING_LEN = 2 * DEPTH + STOP  # 39.69 mm

H_PIPE = TOP + 0.115  # top end of the pipe in its vise
H_COUP = TOP + 0.105  # top (mouth) of the coupling
Z_STOP = H_COUP - DEPTH  # top face of the coupling's stop (bottom of the socket)
PLATE_H = 0.015  # vise base plate

# Dauber (part frame: origin at the lid centre, z up when hanging). Lid 40 x 16 mm; wire 45 mm; wool swab 19 x 25 mm.
LID_R, LID_H = 0.020, 0.016
WIRE_TOP, WIRE_BOT = -0.008, -0.053
SWAB_R, SWAB_TOP, SWAB_TIP = 0.0095, -0.053, -0.078

# Can: 8 fl oz (237 mL) round can, 70 mm across, 87 mm to the shoulder, neck 36 mm across to 95 mm (estimates).
CAN_R, CAN_H, NECK_R, NECK_IN, NECK_H = 0.035, 0.087, 0.018, 0.015, 0.008
CEMENT_LEVEL = 0.040  # cement surface above the can's bottom (about 140 mL left)

# Declared coating rule (see README): enough pressure is 1.0 mm or more of swab squeeze; over 5.0 mm squeezes cement
# out (a run or pool); one dip coats about 3,200 mm2 of new surface.
PRESS_MIN, PRESS_MAX = 0.0010, 0.0050
LOAD_AREA = 3.2e-3  # m^2 of window area one dip can coat
DIP_DEPTH = 0.008  # swab tip at least this far below the cement surface reloads it
POOL_SHARE = 0.25  # a quarter or more of the socket wall coated again from another dip puddles at the stop

WHITE = (0.93, 0.93, 0.90)
GREY = (0.42, 0.44, 0.46)
STEEL = (0.30, 0.33, 0.36)
JAW = (0.20, 0.22, 0.24)
LID = (0.10, 0.20, 0.62)
CAN_COLOR = (0.70, 0.72, 0.74)
SWAB = (0.20, 0.36, 0.86)
SWAB_CORE = (0.18, 0.33, 0.82)
WIRE = (0.52, 0.52, 0.50)
MARK = (0.03, 0.03, 0.03)
CEMENT = (0.06, 0.25, 0.85)
LABEL = (0.05, 0.22, 0.70)
parts.VISUAL_ONLY.update({SWAB, WIRE, MARK, CEMENT, LABEL})  # the swab's soft outer 5 mm, the thin wire, paint marks


def ring(r_in, r_out, z0, z1, color, at=(0.0, 0.0)):
    """A tube wall from z0 to z1 (part frame) as overlapping vertical cylinders."""
    t = r_out - r_in
    mid = (r_in + r_out) / 2
    n = max(12, math.ceil(2 * math.pi * mid / t * 1.3))
    return [parts.cylinder(t / 2, z1 - z0, (at[0] + mid * math.cos(a), at[1] + mid * math.sin(a), (z0 + z1) / 2), color)
            for a in np.linspace(0, 2 * math.pi, n, endpoint=False)]


def vise(top_of_jaws, clamp_r):
    """A small bench vise: base plate, two jaws closed on a round part of radius clamp_r, screw and handle."""
    jt = top_of_jaws - TOP
    return [
        parts.box((0.12, 0.10, PLATE_H), (0.0, 0.0, PLATE_H / 2), GREY),
        parts.box((0.07, 0.018, jt - PLATE_H), (0.0, clamp_r + 0.009, (PLATE_H + jt) / 2), JAW),
        parts.box((0.07, 0.018, jt - PLATE_H), (0.0, -clamp_r - 0.009, (PLATE_H + jt) / 2), JAW),
        parts.cylinder(0.006, 0.040, (0.0, clamp_r + 0.038, PLATE_H + 0.020), STEEL, axis="y"),
        parts.cylinder(0.004, 0.090, (0.0, clamp_r + 0.060, PLATE_H + 0.020), STEEL, axis="x"),
    ]


# Fixed layout (lot A). x away from the arms, y to the robot's left; heights above the bench.
PIPE_XY = (0.290, -0.220)
COUP_XY = (0.200, -0.100)
CAN_A = (0.180, -0.270)
CAN_B = (0.245, -0.345)

_pipe_shapes = vise(TOP + 0.068, R_PIPE) + ring(R_BORE, R_PIPE, PLATE_H, H_PIPE - TOP, WHITE)
_pipe_shapes += [parts.cylinder(0.0009, 0.0025, ((R_PIPE + 0.0003) * math.cos(a), (R_PIPE + 0.0003) * math.sin(a),
                                                 H_PIPE - TOP - DEPTH - 0.00125), MARK)
                 for a in np.linspace(0, 2 * math.pi, 90, endpoint=False)]
PIPE_VISE = parts.Part("pipe_vise", pos=(PIPE_XY[0], PIPE_XY[1], TOP), shapes=_pipe_shapes, friction=(0.6, 0.5))

_cb = H_COUP - TOP - COUPLING_LEN  # coupling bottom (part frame)
_coup_shapes = vise(TOP + 0.058, R_PIPE)
_coup_shapes += ring(R_BORE, R_PIPE, PLATE_H, _cb + DEPTH, WHITE)  # stub cemented into the lower hub
_coup_shapes += ring(R_SOCK, R_COUP, _cb, _cb + DEPTH, WHITE)  # lower hub
_coup_shapes += ring(R_BORE, R_COUP, _cb + DEPTH, _cb + DEPTH + STOP, WHITE)  # centre stop
_coup_shapes += ring(R_SOCK, R_COUP, _cb + DEPTH + STOP, H_COUP - TOP, WHITE)  # upper hub (the socket)
COUPLING_VISE = parts.Part("coupling_vise", pos=(COUP_XY[0], COUP_XY[1], TOP), shapes=_coup_shapes, friction=(0.6, 0.5))

CAN = parts.Part("cement_can", pos=(CAN_A[0], CAN_A[1], TOP + CAN_H / 2), mass=0.25, friction=(0.6, 0.5), shapes=[
    parts.cylinder(CAN_R - 0.001, 0.002, (0, 0, -CAN_H / 2 + 0.001), CAN_COLOR),  # bottom
    *ring(CAN_R - 0.004, CAN_R, -CAN_H / 2 + 0.002, CAN_H / 2 - 0.002, CAN_COLOR),  # wall
    *ring(NECK_IN, CAN_R, CAN_H / 2 - 0.002, CAN_H / 2, CAN_COLOR),  # top, open in the middle
    *ring(NECK_IN, NECK_R, CAN_H / 2, CAN_H / 2 + NECK_H, CAN_COLOR),  # neck
    parts.cylinder(CAN_R + 0.0004, 0.040, (0, 0, -0.008), LABEL),  # label band (drawn only)
    parts.cylinder(CAN_R - 0.0045, 0.0008, (0, 0, -CAN_H / 2 + CEMENT_LEVEL), CEMENT),  # cement surface (drawn only)
])
_lid_z = TOP + CAN_H + NECK_H + LID_H / 2
DAUBER = parts.Part("dauber", pos=(CAN_A[0], CAN_A[1], _lid_z), mass=0.012, friction=(0.9, 0.8), shapes=[
    parts.cylinder(LID_R, LID_H, (0, 0, 0), LID),
    parts.cylinder(0.00125, WIRE_TOP - WIRE_BOT, (0, 0, (WIRE_TOP + WIRE_BOT) / 2), WIRE),
    parts.cylinder(SWAB_R, SWAB_TOP - SWAB_TIP, (0, 0, (SWAB_TOP + SWAB_TIP) / 2), SWAB),
    # the swab's stiff 9 mm core collides: squeezing the soft wool past 5 mm puts the core on the wall
    parts.cylinder(SWAB_R - PRESS_MAX, SWAB_TOP - SWAB_TIP, (0, 0, (SWAB_TOP + SWAB_TIP) / 2), SWAB_CORE),
])
OBJECTS = [PIPE_VISE, COUPLING_VISE, CAN, DAUBER]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"cement_can": CAN.pos, "dauber": DAUBER.pos}),
    lots.Lot("B", place={"cement_can": (CAN_B[0], CAN_B[1], CAN.pos[2]), "dauber": (CAN_B[0], CAN_B[1], _lid_z)}),
)

# ---------------------------------------------------------------------------------------------------------------
# Scoring windows: 6 rows x 48 columns on the pipe band (top end down to the line) and on the socket wall (mouth down
# to the stop), each about 3.0 x 3.2 mm, sampled at 3 x 3 points. A window is coated when any of its points is.
# ---------------------------------------------------------------------------------------------------------------
ROWS, COLS, SUB = 6, 48, 3  # SUB x SUB points per window at 10, 50 and 90 percent


def _windows(radius, z_low):
    dz, da = DEPTH / ROWS, 2 * math.pi / COLS
    pts, owner = [], []
    for r in range(ROWS):
        for c in range(COLS):
            for i in range(SUB):
                for j in range(SUB):
                    z = z_low + (r + (0.1 + 0.4 * i)) * dz  # 10%, 50% and 90% across the window
                    a = (c + (0.1 + 0.4 * j)) * da
                    pts.append((radius * math.cos(a), radius * math.sin(a), z))
                    owner.append(r * COLS + c)
    return np.array(pts), np.array(owner), dz * da * radius


P_PTS, P_OWN, P_WIN_AREA = _windows(R_PIPE, H_PIPE - DEPTH)  # relative to the pipe axis at bench x/y
S_PTS, S_OWN, S_WIN_AREA = _windows(R_SOCK, Z_STOP)
NWIN = ROWS * COLS

# Swab sample points (dauber frame), for the bore and bench rules.
_SW = [(r * math.cos(a), r * math.sin(a), z) for z in np.linspace(SWAB_TIP, SWAB_TOP, 6)
       for r in (0.0, SWAB_R / 2, SWAB_R) for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
SWAB_PTS = np.unique(np.round(np.array(_SW), 9), axis=0)
AXIS_PTS = np.array([(0.0, 0.0, z) for z in np.linspace(SWAB_TIP, SWAB_TOP, 11)])


def fresh():
    return dict(
        p_coats=np.zeros(NWIN, int), p_first=np.full(NWIN, -1), p_last=np.full(NWIN, -1),
        s_coats=np.zeros(NWIN, int), s_last=np.full(NWIN, -1),
        load=LOAD_AREA, load_id=0, wet=True, in_cement=True, fail=None, max_press=0.0, prev=None,
    )


M = fresh()
H: dict = {}


def mat(q):
    return parts._quat_to_matrix(q)


def swab_world(pos, rot, pts):
    return np.asarray(pts) @ rot.T + pos


def press_depths(pos, rot, pipe_xy, coup_xy):
    """Squeeze (m) of the swab against the pipe band (swab outside the pipe) and the socket wall (swab inside)."""
    ax = swab_world(pos, rot, AXIS_PTS)
    out = {"pipe": 0.0, "socket": 0.0, "outside": 0.0}
    rp = np.hypot(ax[:, 0] - pipe_xy[0], ax[:, 1] - pipe_xy[1])
    on = ax[:, 2] <= H_PIPE + 1e-9
    if on.any():
        out["pipe"] = float(max(0.0, np.max(R_PIPE + SWAB_R - rp[on])))
    rc = np.hypot(ax[:, 0] - coup_xy[0], ax[:, 1] - coup_xy[1])
    inside = (ax[:, 2] <= H_COUP + 1e-9) & (ax[:, 2] >= Z_STOP - 1e-9) & (rc < R_SOCK)
    if inside.any():
        out["socket"] = float(max(0.0, np.max(rc[inside] + SWAB_R - R_SOCK)))
    outside = (ax[:, 2] <= H_COUP + 1e-9) & (rc >= R_SOCK)
    if outside.any():
        out["outside"] = float(max(0.0, np.max(R_COUP + SWAB_R - rc[outside])))
    return out


def in_swab(pos, rot, pts):
    loc = (np.asarray(pts) - pos) @ rot
    return (loc[:, 2] >= SWAB_TIP - EPS) & (loc[:, 2] <= SWAB_TOP + EPS) & (np.hypot(loc[:, 0], loc[:, 1]) <= SWAB_R + EPS)


def violations(pos, rot, pipe_xy, coup_xy, can_pos):
    """Whole-trial faults for one swab pose: cement into a bore, onto the bench, or squeezed out."""
    sw = swab_world(pos, rot, SWAB_PTS)
    ax = swab_world(pos, rot, SWAB_PTS * (0.0, 0.0, 1.0))  # the swab's centre line at each point's height
    rp = np.hypot(sw[:, 0] - pipe_xy[0], sw[:, 1] - pipe_xy[1])
    rc = np.hypot(sw[:, 0] - coup_xy[0], sw[:, 1] - coup_xy[1])
    # A bore is entered from its open end: the swab's centre line is over the pipe end or inside the socket there.
    # (Squeezing against the outside of a wall is the separate pressed-too-hard rule.)
    ap = np.hypot(ax[:, 0] - pipe_xy[0], ax[:, 1] - pipe_xy[1])
    ac = np.hypot(ax[:, 0] - coup_xy[0], ax[:, 1] - coup_xy[1])
    if np.any((rp < R_BORE) & (sw[:, 2] < H_PIPE - EPS) & (ap < R_PIPE)):
        return "cement inside the pipe bore"
    if np.any((rc < R_BORE) & (sw[:, 2] < Z_STOP - EPS) & (ac < R_COUP)):
        return "cement past the stop into the bore"
    incan = np.hypot(sw[:, 0] - can_pos[0], sw[:, 1] - can_pos[1]) < NECK_IN
    if np.any((sw[:, 2] < TOP + 0.001) & ~incan):
        return "cement on the bench"
    d = press_depths(pos, rot, pipe_xy, coup_xy)
    if max(d.values()) > PRESS_MAX + EPS:
        return "pressed too hard: cement squeezed out"
    return None


def apply_pose(m, pos, rot, pipe_xy, coup_xy, can_pos):
    """Advance the declared coating state by one swab pose (pure NumPy; used on the GPU and offline)."""
    if m["fail"]:
        return
    tip = swab_world(pos, rot, [(0, 0, SWAB_TIP)])[0]
    in_cement = (np.hypot(tip[0] - can_pos[0], tip[1] - can_pos[1]) < NECK_IN
                 and tip[2] < can_pos[2] - CAN_H / 2 + CEMENT_LEVEL - DIP_DEPTH)
    if in_cement and not m["in_cement"]:
        m["load_id"] += 1
    if in_cement:
        m["load"], m["wet"] = LOAD_AREA, True
    m["in_cement"] = in_cement
    if m["wet"]:
        v = violations(pos, rot, pipe_xy, coup_xy, can_pos)
        if v:
            m["fail"] = v
            return
    d = press_depths(pos, rot, pipe_xy, coup_xy)
    m["max_press"] = max(m["max_press"], max(d.values()))
    ax = swab_world(pos, rot, AXIS_PTS)
    for name, pts, own, area, xy in (("p", P_PTS, P_OWN, P_WIN_AREA, pipe_xy), ("s", S_PTS, S_OWN, S_WIN_AREA, coup_xy)):
        r_ax = np.hypot(ax[:, 0] - xy[0], ax[:, 1] - xy[1])
        if name == "p":
            ok = d["pipe"] >= PRESS_MIN - EPS and np.all(r_ax > R_PIPE)  # wiping the outside of the pipe
        else:
            ok = d["socket"] >= PRESS_MIN - EPS and np.all(r_ax < R_SOCK)  # wiping inside the socket
        if not ok or m["load"] <= 0 or not m["wet"]:
            continue
        world = pts + (xy[0], xy[1], 0.0)
        hit = np.unique(own[in_swab(pos, rot, world)])
        new = hit[m[name + "_last"][hit] < m["load_id"]]
        if not len(new):
            continue
        take = min(len(new), int(math.floor(m["load"] / area + 1e-9)))
        if take <= 0:
            m["load"] = 0.0
            continue
        new = new[:take]
        if name == "p":
            m["p_first"][new[m["p_coats"][new] == 0]] = m["load_id"]
        m[name + "_coats"][new] += 1
        m[name + "_last"][new] = m["load_id"]
        m["load"] = max(0.0, m["load"] - take * area)
        if name == "s" and np.mean(m["s_coats"] >= 2) >= POOL_SHARE - EPS:
            m["fail"] = "cement pooled in the socket (a quarter of the wall coated again from another dip)"
            return


def sweep(m, pos, rot, pipe_xy, coup_xy, can_pos):
    """Apply the poses between the last control step and this one, at most 2 mm apart."""
    prev = m["prev"]
    m["prev"] = (np.asarray(pos, float).copy(), rot.copy())
    if prev is None:
        apply_pose(m, pos, rot, pipe_xy, coup_xy, can_pos)
        return
    n = int(min(20, max(1, math.ceil(np.linalg.norm(np.asarray(pos) - prev[0]) / 0.002))))
    for k in range(1, n + 1):
        apply_pose(m, prev[0] + (np.asarray(pos) - prev[0]) * k / n, rot, pipe_xy, coup_xy, can_pos)


def coverage(m):
    return dict(pipe1=float(np.mean(m["p_coats"] >= 1)), pipe2=float(np.mean(m["p_coats"] >= 2)),
                socket=float(np.mean(m["s_coats"] >= 1)))


def stowed(dauber, can):
    """Dauber back in the can: lid resting on the neck, centred within 6 mm, hanging within 10 degrees of upright."""
    r = mat(dauber.quat_xyzw)
    lid_bottom = dauber.pos[2] - LID_H / 2
    neck_top = can.pos[2] + CAN_H / 2 + NECK_H
    return (abs(lid_bottom - neck_top) <= 0.003 and np.hypot(*(dauber.pos[:2] - can.pos[:2])) <= 0.006
            and r[2, 2] >= math.cos(math.radians(10)))


def in_order(m):
    """The can's order: every band window first coated from a dip before the socket's, and again from one after it."""
    s = m["s_last"][m["s_coats"] > 0]
    return bool(len(s)) and bool(np.all(m["p_first"] < s.min())) and bool(np.all(m["p_last"] > s.max()))


def tier(cov, back_in_can, order=True):
    full_p1, full_p2, full_s = cov["pipe1"] >= 1 - EPS, cov["pipe2"] >= 1 - EPS, cov["socket"] >= 1 - EPS
    if full_p2 and full_s and order and back_in_can:
        return 1.0
    if full_p1 and full_s:
        return 0.75
    if (full_p1 and cov["socket"] >= 0.5 - EPS) or (full_s and cov["pipe1"] >= 0.5 - EPS):
        return 0.5
    if full_p1 or full_s:
        return 0.25
    return 0.0


# ---------------------------------------------------------------------------------------------------------------
# Hands-clear check: every face of both grippers' hulls sampled at about 3 mm.
# ---------------------------------------------------------------------------------------------------------------
@lru_cache(maxsize=1)
def face_clouds():
    from yam_arena import analysis

    result = {}
    for n in ("link4", "link5", "gripper", "tip_left", "tip_right"):
        points = []
        for hull in analysis.link_hulls()[n]:
            points.extend(hull.vertices)
            for plane in np.unique(hull.planes.round(7), axis=0):
                face = hull.vertices[np.abs(hull.vertices @ plane[:3] - plane[3]) < 2e-6]
                if len(face) < 3:
                    continue
                c = face.mean(0)
                u = face[0] - c
                u /= np.linalg.norm(u)
                v = np.cross(plane[:3], u)
                face = face[np.argsort(np.arctan2((face - c) @ v, (face - c) @ u))]
                for a, b in zip(face, np.roll(face, -1, axis=0)):
                    count = max(1, math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / 0.003))
                    for i in range(count + 1):
                        for j in range(count - i + 1):
                            points.append(c + (a - c) * i / count + (b - c) * j / count)
        cloud = np.array(points)
        _, ids = np.unique(np.floor(cloud / 0.002).astype(int), axis=0, return_index=True)
        result[n] = cloud[ids]
    return result


def hand_cloud(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    cloud = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        poses = parts._np(robot.data.body_link_pose_w)[0]
        if not np.isfinite(poses).all():
            raise ValueError("unknown hands")
        for n, p in face_clouds().items():
            q = poses[robot.body_names.index(n)]
            cloud.append(p @ mat(q[3:]).T + q[:3] - origin)
    return np.concatenate(cloud)


def gap(cloud, spec, state):
    p = (cloud - state.pos) @ mat(state.quat_xyzw)
    lo, hi = spec._local_box()
    return float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0), axis=1).min())


# ---------------------------------------------------------------------------------------------------------------
# Drawing the coat (blue dots on coated windows; darker after a second coat).
# ---------------------------------------------------------------------------------------------------------------
_stage, _dots = None, {}


def _win_centres(radius, z_low, outward):
    out = []
    for r in range(ROWS):
        for c in range(COLS):
            a = (c + 0.5) * 2 * math.pi / COLS
            rr = radius + outward
            out.append((rr * math.cos(a), rr * math.sin(a), z_low + (r + 0.5) * DEPTH / ROWS))
    return np.array(out)


P_DOTS = _win_centres(R_PIPE, H_PIPE - DEPTH, 0.0009)
S_DOTS = _win_centres(R_SOCK, Z_STOP, -0.0009)


def draw():
    global _stage, _dots
    from isaaclab.sim.utils import get_current_stage
    from pxr import Gf, UsdGeom

    stage = get_current_stage()
    if stage is not _stage:
        _stage, _dots = stage, {}
    for key, centres, coats, xy in (("p", P_DOTS, M["p_coats"], PIPE_XY), ("s", S_DOTS, M["s_coats"], COUP_XY)):
        for i in np.flatnonzero(coats > 0):
            k = (key, int(i))
            level = min(2, int(coats[i]))
            entry = _dots.get(k)
            if entry is not None and entry[1] == level:
                continue  # only write to the stage when a window changes
            if entry is None:
                g = UsdGeom.Sphere.Define(stage, f"/World/CementCoat/{key}{int(i)}")
                g.CreateRadiusAttr(0.0017)
                g.AddTranslateOp().Set(Gf.Vec3d(*(centres[i] + (xy[0], xy[1], 0.0))))
            else:
                g = entry[0]
            shade = (0.03, 0.16, 0.62) if level >= 2 else (0.10, 0.36, 0.95)
            g.CreateDisplayColorAttr().Set([Gf.Vec3f(*shade)])
            _dots[k] = (g, level)


# ---------------------------------------------------------------------------------------------------------------
# Simulator hooks
# ---------------------------------------------------------------------------------------------------------------
@parts.on_reset
def reset(sim):
    global M, H
    M = fresh()
    H = dict(known=False, bad=None, t=0.0, released=0.0, start=None)


@parts.on_step
def step(sim):
    states = sim.states()
    dt = float(sim.t) - H.get("t", 0.0)
    H["t"] = float(sim.t)
    try:
        d, can = states["dauber"], states["cement_can"]
        if not np.isfinite(np.r_[d.pos, d.quat_xyzw, can.pos, can.quat_xyzw]).all() or not 0 < dt <= 0.2:
            raise ValueError("unknown state")
        if H["start"] is None:
            H["start"] = can.pos.copy()
        if mat(can.quat_xyzw)[2, 2] < math.cos(math.radians(15)):
            H["bad"] = H["bad"] or "can tipped (spill)"
        if np.hypot(*(can.pos[:2] - H["start"][:2])) > 0.020:
            H["bad"] = H["bad"] or "can knocked across the bench"
        for n, spec in (("pipe_vise", PIPE_VISE), ("coupling_vise", COUPLING_VISE)):
            if np.linalg.norm(states[n].pos - np.asarray(spec.pos)) > 0.003:
                H["bad"] = H["bad"] or f"{n} moved"
        sweep(M, d.pos, mat(d.quat_xyzw), PIPE_XY, COUP_XY, can.pos)
        still = d.resting(0.003, 0.03) and can.resting(0.003, 0.03)
        clear = False
        if still:
            cloud = hand_cloud(sim)
            clear = min(gap(cloud, DAUBER, d), gap(cloud, CAN, can)) >= 0.030
        else:  # fail closed if the hands cannot be read
            for arm in ("left_robot", "right_robot"):
                if not np.isfinite(parts._np(sim.env.unwrapped.scene[arm].data.body_link_pose_w)).all():
                    raise ValueError("unknown hands")
        H["released"] = H["released"] + dt if (still and clear) else 0.0
        H["known"] = True
        draw()
    except (ValueError, KeyError, IndexError, AttributeError) as e:
        H["known"] = False
        H["bad"] = H["bad"] or f"unreadable: {e}"
    if os.environ.get("TRACE_OUT"):
        robot = sim.env.unwrapped.scene["right_robot"]
        poses = parts._np(robot.data.body_link_pose_w)[0]
        origin = parts._np(sim.env.unwrapped.scene.env_origins[0])
        g = poses[robot.body_names.index("gripper")]
        tcp = mat(g[3:]) @ np.array((0.0, 0.0, 0.134)) + g[:3] - origin
        dd = states["dauber"]
        tip = swab_world(dd.pos, mat(dd.quat_xyzw), [(0, 0, SWAB_TIP)])[0]
        row = dict(t=float(sim.t), lot=lots.current().name, tcp=tcp.tolist(), dauber=dd.pos.tolist(),
                   quat=dd.quat_xyzw.tolist(), tip=tip.tolist(), press=press_depths(dd.pos, mat(dd.quat_xyzw), PIPE_XY, COUP_XY),
                   cov=coverage(M), load=M["load"], load_id=M["load_id"], fail=M["fail"], bad=H["bad"],
                   released=H["released"], score=score(states))
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("coating.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


def score(objects):
    try:
        if not H.get("known") or H.get("bad") or M["fail"] or H["released"] < 1.0 - 1e-9:
            return 0.0
        return tier(coverage(M), stowed(objects["dauber"], objects["cement_can"]), in_order(M))
    except (KeyError, TypeError, ValueError):
        return 0.0


def success(objects):
    return score(objects) == 1.0


# ---------------------------------------------------------------------------------------------------------------
# Scripted demo: open loop from the published layout (pipe, coupling and can positions as a camera sees them).
# ---------------------------------------------------------------------------------------------------------------
GRIP_ABOVE_LID = 0.007  # TCP above the lid centre: pad tips 2 mm above the lid's lower edge
TCP_TO_TIP = GRIP_ABOVE_LID - SWAB_TIP  # 85 mm
FINGER_LID = -(LID_R - 0.003)  # close 3 mm past the lid's sides
# Fixed corrections measured once in a trace (arm sag with the dauber held; not runtime feedback).
SAG_Z = 0.0004  # trace 1: the swab end ran 0.4 mm low
PIPE_FIX = (-0.0005, 0.0005)  # traces 1-2: the swab's orbit round the pipe ran about 0.5 mm off plan
PIPE_FIX2 = (-0.0014, 0.0002)  # trace 2: after the third dip the held dauber leaned 1.1 degrees, moving the swab 1.1 mm
SOCK_FIX = (-0.0004, -0.0002)  # traces 2 (lot A) and 7 (lot B): 0.2-0.7 mm off plan
SQUEEZE = 0.0026  # planned squeeze of the swab on the pipe (pressing rule: 1.0 to 5.0 mm)
SQUEEZE_SOCK = 0.0027  # in the socket (the straight moves between orbit points pull it 0.15 mm off the wall)
HUB = (0.215, -0.290)  # a point clear of the pipe and coupling, used when travelling to and from the socket


def demo():
    lot = lots.current()
    can = np.asarray(lot.pos("cement_can"))
    lid = np.asarray(lot.pos("dauber"))
    neck_top = can[2] + CAN_H / 2 + NECK_H
    f = FINGER_LID
    W = []

    def wp(t, xy, tcp_z, finger=f, label="", fix=(0.0, 0.0)):
        x = float(xy[0] + fix[0])
        y = float(xy[1] + fix[1])
        W.append(Waypoint(float(t), (x, y, float(tcp_z + SAG_Z)), float(finger), label, 0.0))

    def tcp_for_tip(tip_z):
        return tip_z + TCP_TO_TIP

    carry = tcp_for_tip(neck_top + 0.006)  # swab tip 6 mm above the can's neck
    lid_tcp = lid[2] + GRIP_ABOVE_LID

    # 1. take the dauber by its lid and lift it straight out of the can
    wp(2.5, lid[:2], lid_tcp + 0.05, FINGER_OPEN, "above the lid")
    wp(2.0, lid[:2], lid_tcp, FINGER_OPEN, "fingers round the lid")
    wp(1.0, lid[:2], lid_tcp, f, "grip the lid")
    wp(2.5, lid[:2], carry, f, "lift the dauber out of the can")

    def dip(label):
        wp(2.0, can[:2], carry, f, f"over the can ({label})")
        wp(1.5, can[:2], lid_tcp + 0.003, f, f"dip ({label})")
        wp(1.0, can[:2], lid_tcp + 0.003, f, f"soak ({label})")
        wp(2.0, can[:2], carry, f, f"lift out ({label})")

    px, py = PIPE_XY
    band_tcp = tcp_for_tip(H_PIPE - DEPTH - 0.0034)  # swab spans the band with about 3 mm to spare each end
    r_pipe = R_PIPE + SWAB_R - SQUEEZE
    a0 = math.radians(200)  # start on the side facing the arm's base, away from the coupling

    def pipe_coat(label):
        start = (px + (r_pipe + 0.012) * math.cos(a0), py + (r_pipe + 0.012) * math.sin(a0))
        wp(2.5, start, carry, f, f"beside the pipe ({label})")
        wp(1.5, start, band_tcp, f, f"down beside the band ({label})")
        for k, a in enumerate(np.linspace(a0, a0 + math.radians(390), 27)):
            wp(1.2 if k == 0 else 0.35, (px + r_pipe * math.cos(a), py + r_pipe * math.sin(a)), band_tcp, f,
               f"round the band ({label})", PIPE_FIX if label == "first coat" else PIPE_FIX2)
        a1 = a0 + math.radians(390)
        out = (px + (r_pipe + 0.012) * math.cos(a1), py + (r_pipe + 0.012) * math.sin(a1))
        wp(0.8, out, band_tcp, f, f"off the band ({label})")
        wp(1.5, out, carry, f, f"up ({label})")

    cx, cy = COUP_XY
    rim_tcp = tcp_for_tip(H_COUP + 0.004)
    sock_tcp = tcp_for_tip(Z_STOP + 0.0024)  # swab end 2.4 mm above the stop as planned; traces ran 0.5-1 mm low
    r_sock = R_SOCK - SWAB_R + SQUEEZE_SOCK
    b0 = math.radians(270)

    def socket_coat():
        wp(1.2, can[:2], rim_tcp, f, "up to rim height")
        wp(1.5, HUB, rim_tcp, f, "round the pipe vise")
        wp(2.5, (cx, cy), rim_tcp, f, "over the socket")
        wp(1.5, (cx, cy), sock_tcp + 0.003, f, "down into the socket")  # 3 mm higher while over the bore
        for k, a in enumerate(np.linspace(b0, b0 + math.radians(390), 27)):
            wp(1.0 if k == 0 else 0.35, (cx + r_sock * math.cos(a), cy + r_sock * math.sin(a)), sock_tcp, f,
               "round the socket wall", SOCK_FIX)
        wp(1.0, (cx, cy), sock_tcp + 0.004, f, "back to the centre")
        wp(1.5, (cx, cy), rim_tcp, f, "up out of the socket")
        wp(2.5, HUB, rim_tcp, f, "back round the pipe vise")

    pipe_coat("first coat")
    dip("for the socket")
    socket_coat()
    dip("second coat")
    pipe_coat("second coat")
    # put the dauber back: lid on the neck, let go, hands clear
    wp(2.5, can[:2], carry, f, "over the can")
    wp(2.0, can[:2], lid_tcp + 0.002, f, "lid onto the neck")
    wp(1.0, can[:2], lid_tcp + 0.002, FINGER_OPEN, "let go")
    wp(1.5, can[:2], lid_tcp + 0.06, FINGER_OPEN, "hands clear")
    wp(4.0, can[:2], lid_tcp + 0.06, FINGER_OPEN, "still")
    return {"right": W}
