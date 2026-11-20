"""A kitchen counter (the station table). In front of the robot sits a bare aluminium quarter sheet pan with three
round cookies lying flat on it in a row along its far rim. To its right stands an empty wire cooling rack. Nearer
the robot a thin stainless cookie turner lies on two small posts under its handle, its blade flat on the counter and
pointing to the left. Where the cookies lie on the pan changes between trials.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
D = math.radians

ALU = (0.80, 0.81, 0.83)
WIRE = (0.70, 0.71, 0.73)
STEEL = (0.74, 0.75, 0.77)
BLACK = (0.08, 0.08, 0.09)
WOOD = (0.62, 0.45, 0.28)
COOKIE = (0.72, 0.52, 0.30)
CHIP = (0.25, 0.14, 0.08)


def _qz(a: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _qy(a: float) -> tuple[float, float, float, float]:
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# -- the cookies: non-perishable stand-ins ---------------------------------------------------------------------------
# Each is a 3 in (76.2 mm) x 1/2 in (12.7 mm) unfinished birch craft circle, its underside edge sanded to a 6 mm wide,
# 1.5 mm high bevel (the curled-up rim of a baked drop cookie that lets a turner in) and its top edge rounded 3 mm,
# sealed with food-safe mineral oil; about 37 g (estimate from birch at 0.65 g/cm3). Size and mass match the baked
# cookie the original task used (Otis Spunkmeyer item 58814: 72 to 82 mm across, 35 g). Drawing in fabrication/.
# Part frame: origin at the centre of the underside, z up.
CK_R, CK_H, CK_MASS = 0.0381, 0.0127, 0.037
BEVEL_W, BEVEL_H, TOP_R = 0.006, 0.0015, 0.003
SEG = 32


def _cookie_shapes():
    pts = []
    for k in range(SEG):
        a = 2 * math.pi * k / SEG
        c, s = math.cos(a), math.sin(a)
        for r, z in ((CK_R - BEVEL_W, 0.0), (CK_R, BEVEL_H), (CK_R, CK_H - TOP_R), (CK_R - TOP_R, CK_H)):
            pts.append((r * c, r * s, z))
    faces = []
    for k in range(SEG):
        a, b = 4 * k, 4 * ((k + 1) % SEG)
        for i in range(3):
            faces.append((a + i, b + i, b + i + 1, a + i + 1))
    faces.append(tuple(4 * k for k in reversed(range(SEG))))
    faces.append(tuple(4 * k + 3 for k in range(SEG)))
    out = [parts.hull(pts, faces, color=COOKIE)]
    # looks only: a few dark chips on top, so the stand-in reads as a cookie on camera
    for (x, y) in ((0.012, 0.008), (-0.015, 0.010), (0.002, -0.016), (-0.006, 0.022), (0.020, -0.012)):
        out.append(parts.box((0.007, 0.006, 0.0012), at=(x, y, CK_H + 0.0003), color=CHIP, collide=False))
    return out


# -- the sheet pan: Nordic Ware Naturals quarter sheet (aluminium) ----------------------------------------------------
# Listed about 13 x 9.6 x 1 in outside; modelled with a 216 x 305 mm flat floor (long side across), 1 mm thick, and rims
# 25 mm tall, 3 mm thick (estimates within the listing). Fixed: it stands on a non-slip silicone mat in real trials.
PAN_FX, PAN_FY, PAN_T, PAN_RIM, RIM_T = 0.216, 0.305, 0.001, 0.025, 0.003
PAN_C = (0.380, -0.1025)
FLOOR = TOP + PAN_T


def _pan_shapes():
    hx, hy = PAN_FX / 2, PAN_FY / 2
    return [
        parts.box((PAN_FX, PAN_FY, PAN_T), at=(0.0, 0.0, PAN_T / 2), color=ALU, friction=(0.90, 0.80)),
        parts.box((RIM_T, PAN_FY + 2 * RIM_T, PAN_RIM), at=(-hx - RIM_T / 2, 0.0, PAN_RIM / 2), color=ALU),
        parts.box((RIM_T, PAN_FY + 2 * RIM_T, PAN_RIM), at=(hx + RIM_T / 2, 0.0, PAN_RIM / 2), color=ALU),
        parts.box((PAN_FX, RIM_T, PAN_RIM), at=(0.0, -hy - RIM_T / 2, PAN_RIM / 2), color=ALU),
        parts.box((PAN_FX, RIM_T, PAN_RIM), at=(0.0, hy + RIM_T / 2, PAN_RIM / 2), color=ALU),
    ]


# -- the cooling rack: a quarter-sheet wire cooling grid ----------------------------------------------------------------
# 305 x 216 mm, wires 2 mm thick 10 mm apart running front to back on three cross wires and a frame, tops 17 mm above the
# counter (estimates for a quarter-sheet grid such as the Nordic Ware quarter sheet cooling grid). Fixed (non-slip mat).
RK_X, RK_Y, RK_H, WIRE_D = 0.305, 0.216, 0.017, 0.002
RACK_C = (0.370, -0.392)
WIRE_TOP = TOP + RK_H


def _rack_shapes():
    hx, hy = RK_X / 2, RK_Y / 2
    zt = RK_H - WIRE_D / 2
    out = []
    n = int(round(RK_Y / 0.010))
    for k in range(n + 1):
        y = -hy + k * RK_Y / n
        out.append(parts.box((RK_X, WIRE_D, WIRE_D), at=(0.0, y, zt), color=WIRE, friction=(0.40, 0.35)))
    for x in (-hx, -hx / 3, hx / 3, hx):
        out.append(parts.box((WIRE_D, RK_Y, WIRE_D), at=(x, 0.0, zt - WIRE_D), color=WIRE))
    for x in (-hx + 0.01, hx - 0.01):
        for y in (-hy + 0.01, hy - 0.01):
            out.append(parts.box((0.004, 0.004, RK_H - 2 * WIRE_D), at=(x, y, (RK_H - 2 * WIRE_D) / 2), color=WIRE))
    return out


# -- the turner: a small stainless offset cookie turner -----------------------------------------------------------------
# Blade 90 x 65 mm, 0.6 mm thick with its front 4 mm ground to a 0.15 mm edge; a neck dropping the blade 43 mm below the
# handle's middle; black polypropylene handle 110 x 22 x 14 mm; about 69 g (all estimates for a mini offset turner).
# Part frame: origin at the handle's middle, x toward the blade's tip, z up with the blade flat.
HD_L, HD_W, HD_T = 0.110, 0.022, 0.014
BL_L, BL_W, BL_T, BL_EDGE = 0.090, 0.065, 0.0006, 0.00015
DROP = 0.043  # handle middle above the blade's underside
BL_X0 = HD_L / 2 + 0.025  # the blade's back edge
TURNER_MASS = 0.069


def _blade_hull():
    x0, x1, hw = BL_X0, BL_X0 + BL_L, BL_W / 2
    zb = -DROP
    pts = [(x0, -hw, zb), (x0, hw, zb), (x1, -hw, zb), (x1, hw, zb),
           (x0, -hw, zb + BL_T), (x0, hw, zb + BL_T), (x1 - 0.004, -hw, zb + BL_T), (x1 - 0.004, hw, zb + BL_T),
           (x1, -hw, zb + BL_EDGE), (x1, hw, zb + BL_EDGE)]
    faces = [(0, 2, 3, 1), (4, 5, 7, 6), (6, 7, 9, 8), (2, 8, 9, 3), (0, 1, 5, 4), (0, 4, 6, 8, 2), (1, 3, 9, 7, 5)]
    return parts.hull(pts, faces, color=STEEL, friction=(0.05, 0.05))


def _turner_shapes():
    neck_len = math.hypot(BL_X0 - HD_L / 2 + 0.006, DROP - HD_T / 2)
    neck_ang = math.atan2(DROP - HD_T / 2, BL_X0 - HD_L / 2 + 0.006)
    return [
        parts.box((HD_L, HD_W, HD_T), at=(0.0, 0.0, 0.0), color=BLACK),
        parts.box((neck_len, 0.016, 0.0015), at=((HD_L / 2 - 0.006 + BL_X0) / 2, 0.0, -(DROP + HD_T / 2) / 2 + 0.0004),
                  color=STEEL, quat=_qy(neck_ang)),
        _blade_hull(),
    ]


# -- the turner's rest: two hardwood posts 16 x 8 mm, 36 mm tall, under the ends of the handle (drawing) ----------------
POST_H = DROP - HD_T / 2  # the blade lies flat on the counter with the handle on the posts


def _rest_shapes():
    return [parts.box((0.008, 0.016, POST_H), at=(x, 0.0, POST_H / 2), color=WOOD) for x in (-0.048, 0.048)]


# -- layout -------------------------------------------------------------------------------------------------------------
# The cookies lie in a row along the pan's far rim, 6 mm from it, where the turner's whole blade fits on the floor behind
# them. Lot B moves the row 12 mm left and 4 mm nearer the robot.
COOKIES_AT = {
    "A": ((0.444, -0.190), (0.444, -0.100), (0.444, -0.010)),
    "B": ((0.440, -0.178), (0.440, -0.088), (0.440, 0.002)),
}
NAMES = ("cookie_1", "cookie_2", "cookie_3")
TURNER_YAW = math.pi / 2  # pointing left (blade toward +y)
TURNER_AT = (0.165, -0.100)  # the handle's middle
TURNER_Z = TOP + POST_H + HD_T / 2
SPOTS = ((0.290, -0.350), (0.415, -0.350), (0.350, -0.435))  # where the demo aims the cookies on the rack

COOKIE_PARTS = tuple(parts.Part(n, pos=(*COOKIES_AT["A"][i], FLOOR), mass=CK_MASS, shapes=_cookie_shapes(),
                                friction=(0.50, 0.45), contact_offset=0.002) for i, n in enumerate(NAMES))
PAN = parts.Part("pan", pos=(*PAN_C, TOP), shapes=_pan_shapes(), friction=(0.40, 0.35), contact_offset=0.002)
RACK = parts.Part("rack", pos=(*RACK_C, TOP), shapes=_rack_shapes(), friction=(0.40, 0.35), contact_offset=0.002)
TURNER = parts.Part("turner", pos=(*TURNER_AT, TURNER_Z), yaw=TURNER_YAW, mass=TURNER_MASS, shapes=_turner_shapes(),
                    friction=(0.9, 0.8), contact_offset=0.002, com=(0.035, 0.0, -0.012))
REST = parts.Part("rest", pos=(*TURNER_AT, TOP), yaw=TURNER_YAW, shapes=_rest_shapes(), friction=(0.6, 0.5),
                  contact_offset=0.002)

OBJECTS = [*COOKIE_PARTS, PAN, RACK, TURNER, REST]
LOTS = lots.declare(OBJECTS, *(lots.Lot(n, place={nm: (*xy, FLOOR) for nm, xy in zip(NAMES, ps)})
                               for n, ps in COOKIES_AT.items()))
DEMO_TOUCHES = ("rest",)


# -- geometry for scoring -----------------------------------------------------------------------------------------------
def frame(st):
    return np.asarray(st.pos, float), _rot(st.quat_xyzw)


def cookie_tilt(st) -> float:
    """Degrees the cookie's top face leans off level (more than 90 when it lies upside down)."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(frame(st)[1][2, 2])))))


def cookie_gap(pts, st) -> float:
    """Closest distance from any point to the cookie (taken as its 76.2 x 12.7 mm cylinder), m."""
    p, r = frame(st)
    loc = (np.asarray(pts, float) - p) @ r
    if len(loc) == 0:
        return 1.0
    rad = np.linalg.norm(loc[:, :2], axis=1) - CK_R
    zz = np.abs(loc[:, 2] - CK_H / 2) - CK_H / 2
    q = np.stack([rad, zz], axis=1)
    out = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
    return float(out.min())


TURNER_LOCAL = np.array([[x, y, z] for x in (-HD_L / 2, HD_L / 2) for y in (-HD_W / 2, HD_W / 2)
                         for z in (-HD_T / 2, HD_T / 2)] +
                        [[x, y, -DROP + z] for x in (BL_X0, BL_X0 + BL_L) for y in (-BL_W / 2, BL_W / 2)
                         for z in (0.0, BL_T)])


def turner_gap(pts, st) -> float:
    """Closest distance from any point to the turner's handle or blade boxes, m."""
    p, r = frame(st)
    loc = (np.asarray(pts, float) - p) @ r
    if len(loc) == 0:
        return 1.0
    best = math.inf
    for c, half in ((np.zeros(3), np.array([HD_L / 2, HD_W / 2, HD_T / 2])),
                    (np.array([BL_X0 + BL_L / 2, 0.0, -DROP + BL_T / 2]), np.array([BL_L / 2, BL_W / 2, BL_T / 2]))):
        q = np.abs(loc - c) - half
        best = min(best, float((np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)).min()))
    return best


def on_rack(st) -> bool:
    """The cookie lies on the rack: its whole outline inside the wire grid seen from above, its underside on the wires
    (no more than 4 mm above their tops, 1 mm below) and its top face up within 8 degrees."""
    p, _ = frame(st)
    inside = (abs(p[0] - RACK_C[0]) <= RK_X / 2 - CK_R + EPS and abs(p[1] - RACK_C[1]) <= RK_Y / 2 - CK_R + EPS)
    return bool(inside and WIRE_TOP - 0.001 - EPS <= p[2] <= WIRE_TOP + 0.004 + EPS and cookie_tilt(st) <= 8.0 + EPS)


def on_counter(st) -> bool:
    """The cookie lies on the bare counter: its underside within 3 mm of the counter top, its middle off the pan's
    floor."""
    p, _ = frame(st)
    over_pan = abs(p[0] - PAN_C[0]) <= PAN_FX / 2 + EPS and abs(p[1] - PAN_C[1]) <= PAN_FY / 2 + EPS
    return bool(float(p[2]) < TOP + 0.003 and not over_pan)


def turner_home(st) -> bool:
    """The turner is back on its rest: handle's middle within 30 mm of its marked place seen from above and no more
    than 10 mm above or below its height on the posts, pointing within 15 degrees of its marked direction, blade flat
    within 10 degrees."""
    p, r = frame(st)
    d = math.hypot(p[0] - TURNER_AT[0], p[1] - TURNER_AT[1])
    heading = math.atan2(r[1, 0], r[0, 0])
    turn = abs((heading - TURNER_YAW + math.pi) % (2 * math.pi) - math.pi)
    flat = float(r[2, 2]) >= math.cos(D(10.0)) - EPS
    return bool(d <= 0.030 + EPS and abs(p[2] - TURNER_Z) <= 0.010 + EPS and turn <= D(15.0) + EPS and flat)


# -- the trial record ---------------------------------------------------------------------------------------------------
TOUCH = 0.002  # a hand within 2 mm of a cookie touches the food
CLEAR = 0.010  # let go: every sampled hand point at least 10 mm from the turner and every cookie
DROP_V = 0.60  # m/s downward: a free fall of about 18 mm; the cookie counts as dropped
HOLD_S = 1.0
STILL_M = 0.001
_rec = parts.__dict__.setdefault("cookie_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", zero=False, why="", spoiled={n: "" for n in NAMES}, gap=None,
                t=None, snap=None, still=None, free=None, at_rest=False, max_down={n: 0.0 for n in NAMES})


if not _rec:
    reset_record()


def _snapshot(objects) -> np.ndarray:
    return np.concatenate([np.asarray(objects[n].pos, float) for n in (*NAMES, "turner")] +
                          [frame(objects["turner"])[1][:, 0]])


def observe(objects, gaps, t) -> None:
    """One control step. ``gaps``: (closest sampled hand point to any cookie, to the turner or any cookie), m; None if
    the arms could not be read, which costs the trial its credit."""
    _rec["steps"] += 1
    try:
        snap = _snapshot(objects)
        if not np.all(np.isfinite(snap)):
            raise ValueError("scene state unknown")
        if gaps is None or not all(math.isfinite(g) for g in gaps):
            raise ValueError("arm state unknown")
        food, anything = (float(g) for g in gaps)
        if not _rec["zero"] and food < TOUCH - EPS:
            _rec.update(zero=True, why="a gripper touched a cookie")
        for n in NAMES:
            st = objects[n]
            vz = float(np.asarray(st.lin_vel, float)[2])
            _rec["max_down"][n] = max(_rec["max_down"][n], -vz)
            if _rec["spoiled"][n]:
                continue
            if -vz > DROP_V + EPS:
                _rec["spoiled"][n] = "dropped"
            elif on_counter(st):
                _rec["spoiled"][n] = "fell on the counter"
            elif cookie_tilt(st) > 60.0:
                _rec["spoiled"][n] = "turned over or on its edge"
        if _rec["snap"] is None or float(np.abs(snap - _rec["snap"]).max()) > STILL_M + EPS:
            _rec["snap"], _rec["still"] = snap, float(t)
        _rec["free"] = (float(t) if _rec["free"] is None else _rec["free"]) if anything >= CLEAR - EPS else None
        rest_since = None if _rec["free"] is None else max(_rec["free"], _rec["still"])
        _rec["at_rest"] = rest_since is not None and float(t) - rest_since >= HOLD_S - EPS
        _rec.update(gap=anything, t=float(t))
    except (KeyError, ValueError, TypeError, AttributeError, IndexError) as e:
        _rec.update(known=False, reason=str(e), free=None, at_rest=False)


def good(objects) -> list[str]:
    return [n for n in NAMES if not _rec["spoiled"][n] and on_rack(objects[n])]


# -- scoring --------------------------------------------------------------------------------------------------------------
def score(objects):
    """0 for the whole trial once a gripper has touched a cookie (and if the scene or the arms could not be read at any
    step). Otherwise, with every hand point 10 mm or more from the turner and the cookies and everything still for 1 s:
    0.25 for each cookie lying flat on the cooling rack (top up within 8 degrees, wholly over the wires) that was never
    dropped (downward speed over 0.6 m/s), never on the counter and never turned over; 1.0 when all three are and the
    turner is back on its rest."""
    try:
        if not _rec["known"] or _rec["zero"] or _rec["steps"] == 0 or not _rec["at_rest"]:
            return 0.0
        g = good(objects)
        if len(g) == len(NAMES) and turner_home(objects["turner"]):
            return 1.0
        return 0.25 * len(g)
    except (KeyError, ValueError, TypeError, AttributeError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - 1e-9


def distance(objects):
    try:
        return float(sum(math.hypot(objects[n].pos[0] - RACK_C[0], objects[n].pos[1] - RACK_C[1]) for n in NAMES))
    except Exception:  # noqa: BLE001
        return 3.0


# -- the arms as point clouds: hull faces sampled every 3 to 5 mm ------------------------------------------------------
HAND_LINKS = ("link5", "gripper", "tip_left", "tip_right")
_FACES: dict = {}


def _hull_face_points(hull, step) -> np.ndarray:
    pts = [hull.vertices]
    for n, d in zip(hull.planes[:, :3], hull.planes[:, 3]):
        on = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 = e1 / np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for i in range(len(ring)):
            a, b = ring[i], ring[(i + 1) % len(ring)]
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step)))
            ii, jj = np.meshgrid(np.arange(m + 1), np.arange(m + 1), indexing="ij")
            keep = (ii + jj) <= m
            s_, t_ = ii[keep][:, None] / m, jj[keep][:, None] / m
            pts.append(c + (a - c) * s_ + (b - c) * t_)
    pts = np.concatenate(pts)
    _, idx = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(idx)]])


def link_points(name, step=0.003) -> np.ndarray:
    key = (name, step)
    if key not in _FACES:
        from yam_arena import analysis

        hs = analysis.link_hulls().get(name, [])
        if not hs:
            raise ValueError(f"no collision hull for {name}")
        _FACES[key] = np.concatenate([_hull_face_points(h, step) for h in hs])
    return _FACES[key]


STEP = {"link5": 0.005, "gripper": 0.005, "tip_left": 0.003, "tip_right": 0.003}


def hand_points(link_poses: dict) -> np.ndarray:
    return np.concatenate([link_points(name, STEP[name]) @ m[:3, :3].T + m[:3, 3]
                           for (_, name), m in link_poses.items() if name in HAND_LINKS])


def _arm_poses(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = {}
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in HAND_LINKS:
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                out[(arm, n)] = m
    if len(out) != 2 * len(HAND_LINKS) or not all(np.isfinite(m).all() for m in out.values()):
        return None
    return out


def hand_gaps(objects, pts) -> tuple[float, float]:
    food = min(cookie_gap(pts, objects[n]) for n in NAMES)
    return food, min(food, turner_gap(pts, objects["turner"]))


@parts.on_reset
def _new_trial(sim) -> None:
    reset_record()
    _rec["last_t"] = None


_stock_apply = lots._apply


def _apply_lot(emb) -> bool:
    moved = _stock_apply(emb)
    reset_record()
    _rec["last_t"] = None
    return moved


lots._apply = _apply_lot


@parts.on_step
def _step(sim) -> None:
    if _rec.get("last_t") == sim.t:
        return
    _rec["last_t"] = sim.t
    try:
        objects = sim.states()
    except Exception:  # noqa: BLE001
        objects = {}
    g = None
    try:
        poses = _arm_poses(sim)
        if poses is not None and all(n in objects for n in (*NAMES, "turner")):
            g = hand_gaps(objects, hand_points(poses))
    except Exception:  # noqa: BLE001
        g = None
    observe(objects, g, float(sim.t))
    log = (str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("cookies.jsonl")) if os.environ.get("TRACE_OUT")
           else os.environ.get("COOKIE_LOG"))
    if log and objects and _rec["steps"] % 2 == 0:
        try:
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, score=score(objects), zero=_rec["zero"],
                       why=_rec["why"], spoiled=dict(_rec["spoiled"]), gap=None if _rec["gap"] is None else
                       round(_rec["gap"], 4), at_rest=_rec["at_rest"], known=_rec["known"], reason=_rec["reason"],
                       good=good(objects), home=turner_home(objects["turner"]),
                       turner=np.round(objects["turner"].pos, 4).tolist(),
                       tq=np.round(objects["turner"].quat_xyzw, 4).tolist(),
                       vmax={n: round(_rec["max_down"][n], 3) for n in NAMES})
            for n in NAMES:
                row[n] = np.round(objects[n].pos, 4).tolist() + [round(cookie_tilt(objects[n]), 1)]
            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            a = base.scene["right_robot"]
            poses = parts._np(a.data.body_link_pose_w)[0]
            row["grip"] = np.round(poses[list(a.body_names).index("gripper"), :3] - origin, 4).tolist()
            with open(log, "a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with open(log, "a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- demo: open loop from the layout the top camera shows ------------------------------------------------------------
# The right arm takes the turner by the middle of its handle and turns it to point forward. For each cookie it sets the
# blade flat on the pan's floor just behind the cookie and slides it forward: the thin front edge goes in under the
# cookie's bevelled rim (a cookie that is shoved instead stops against the far rim) and the cookie rides onto the
# blade. It lifts the cookie level over the pan's rim, carries it over the rack, lowers the blade almost onto the
# wires, rolls the turner 25 degrees about its handle so the cookie's rim comes down onto the wires, draws the blade out
# from under it sideways and rolls back for the next. Last it lays the turner back on its rest, lets go and goes back to
# rest. The left arm stays at rest. COOKIE_DEMO=hand picks the cookies up with the fingers instead (scores 0).
SQUEEZE = -(HD_W / 2 - 0.004)
SAG = 0.003  # the arm carries a light tool about this much below its target (measured on a trace); the demo aims high
PUSH_PRESS = 0.001  # and presses the blade this much into the pan's floor while it slides
ROLL = D(25.0)
BLADE_C = BL_X0 + BL_L / 2  # the blade's middle, ahead of the handle's middle


def _w(d, p, finger, label, yaw, rot):
    return Waypoint(d, tuple(float(v) for v in p), finger, label, tilt._TiltedYaw(float(yaw), rot))


def _rest_pose(arm):
    from yam_arena import ik, station
    from yam_arena.demo import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return tuple(float(v) for v in pos), tilt._TiltedYaw(0.0, rot)


def demo():
    seen = lots.current()
    mode = os.environ.get("COOKIE_DEMO", "")
    r_pick = tilt.rotation(TURNER_YAW, 0.0, TURNER_YAW)  # straight down, the fingers closing across the handle
    r_fwd = tilt.rotation(0.0, 0.0, 0.0)
    p0 = np.array([*TURNER_AT, TURNER_Z])

    def grip_z(blade_bottom):
        return blade_bottom + DROP + SAG

    plan = []
    if mode == "hand":
        for i, n in enumerate(NAMES):
            cx, cy, _ = seen.pos(n)
            sx, sy = SPOTS[i]
            z = FLOOR + CK_H / 2 + 0.002
            plan += [
                _w(2.0, (cx, cy, z + 0.08), FINGER_OPEN, "above the cookie", 0.0, r_fwd),
                _w(1.5, (cx, cy, z), FINGER_OPEN, "fingers round the cookie", 0.0, r_fwd),
                _w(0.8, (cx, cy, z), -(CK_R - 0.004), "pinch it", 0.0, r_fwd),
                _w(2.0, (sx, sy, z + 0.06), -(CK_R - 0.004), "carry it to the rack", 0.0, r_fwd),
                _w(1.0, (sx, sy, WIRE_TOP + CK_H / 2 + 0.003), -(CK_R - 0.004), "set it down", 0.0, r_fwd),
                _w(0.8, (sx, sy, WIRE_TOP + CK_H / 2 + 0.003), FINGER_OPEN, "let go", 0.0, r_fwd),
                _w(1.0, (sx, sy, WIRE_TOP + 0.10), FINGER_OPEN, "lift", 0.0, r_fwd),
            ]
    else:
        plan += [
            _w(2.5, p0 + [0.0, 0.0, 0.08], FINGER_OPEN, "above the turner's handle", TURNER_YAW, r_pick),
            _w(1.5, p0 + [0.0, 0.0, SAG], FINGER_OPEN, "fingers down round the handle", TURNER_YAW, r_pick),
            _w(0.8, p0 + [0.0, 0.0, SAG], SQUEEZE, "close on the handle", TURNER_YAW, r_pick),
            _w(1.2, p0 + [0.0, 0.0, 0.07], SQUEEZE, "lift the turner", TURNER_YAW, r_pick),
        ]
        above_rim = TOP + PAN_RIM + 0.030  # the blade's underside while it crosses the pan's rim
        for i, n in enumerate(NAMES):
            cx, cy, _ = seen.pos(n)
            sx, sy = SPOTS[i]
            tip0 = cx - CK_R - 0.010  # the blade's front edge 10 mm short of the cookie
            h0 = tip0 - (BL_X0 + BL_L)  # the handle's middle then
            h1 = cx - BLADE_C - 0.008  # the blade's middle 8 mm short of the cookie's middle at the end of the slide
            z_push = grip_z(FLOOR - PUSH_PRESS)
            z_lift = grip_z(above_rim)
            plan += [
                _w(2.2 if i == 0 else 2.0, (h0, cy, z_lift), SQUEEZE, f"{n}: over the pan behind it", 0.0, r_fwd),
                _w(1.2, (h0, cy, z_push + 0.004), SQUEEZE, f"{n}: blade down to the floor", 0.0, r_fwd),
                _w(0.5, (h0, cy, z_push), SQUEEZE, f"{n}: blade flat on the floor", 0.0, r_fwd),
                _w(3.0, (h1, cy, z_push), SQUEEZE, f"{n}: slide the blade under it", 0.0, r_fwd),
                _w(1.2, (h1, cy, z_lift), SQUEEZE, f"{n}: lift it level", 0.0, r_fwd),
            ]
            # over the rack: roll the blade's left side down until the cookie's left rim rests on the wires, then
            # draw the blade out to the right from under it, and roll back
            bx, by = sx - BLADE_C, sy - 0.011
            z_over = grip_z(WIRE_TOP + 0.003)
            r_roll = _rx(-ROLL) @ r_fwd
            plan += [
                _w(2.2, (bx, by, z_lift), SQUEEZE, f"{n}: carry it over the rack", 0.0, r_fwd),
                _w(1.0, (bx, by, z_over), SQUEEZE, f"{n}: blade just above the wires", 0.0, r_fwd),
                _w(1.2, (bx, by, z_over + 0.010), SQUEEZE, f"{n}: roll the blade, cookie's rim onto the wires", 0.0,
                   r_roll),
                _w(0.5, (bx, by, z_over + 0.010), SQUEEZE, f"{n}: let it settle", 0.0, r_roll),
                _w(1.2, (bx, by - 0.065, z_over + 0.010), SQUEEZE, f"{n}: draw the blade out from under it", 0.0,
                   r_roll),
                _w(1.0, (bx, by - 0.065, z_over + 0.040), SQUEEZE, f"{n}: roll back and lift", 0.0, r_fwd),
            ]
        plan += [
            _w(2.5, p0 + [0.0, 0.0, 0.07], SQUEEZE, "back over the rest", TURNER_YAW, r_pick),
            _w(1.5, p0 + [0.0, 0.0, SAG + 0.004], SQUEEZE, "lay the turner on its rest", TURNER_YAW, r_pick),
            _w(0.8, p0 + [0.0, 0.0, SAG + 0.004], FINGER_OPEN, "let go", TURNER_YAW, r_pick),
            _w(1.2, p0 + [0.0, 0.0, 0.10], FINGER_OPEN, "lift the fingers clear", TURNER_YAW, r_pick),
        ]
    rrest, ryaw = _rest_pose("right")
    plan.append(Waypoint(2.0, rrest, FINGER_OPEN, "back to the rest pose", ryaw))
    plan.append(Waypoint(3.0, rrest, FINGER_OPEN, "wait", ryaw))
    lrest, lyaw = _rest_pose("left")
    t_right = sum(w.duration for w in plan)
    plans = {"right": plan, "left": [Waypoint(t_right, lrest, FINGER_OPEN, "stay at rest", lyaw)]}
    for arm, pl in plans.items():
        r, y = _rest_pose(arm)
        f = pl[0].finger
        pl[:0] = [Waypoint(2.0, r, f, "to the rest pose", y), Waypoint(0.5, r, f, "settle at the rest pose", y)]
    return plans
