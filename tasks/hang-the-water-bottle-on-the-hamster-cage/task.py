"""An empty wire-top hamster cage stands on the bench across the table from the robot, its front wall facing the
robot. The wire top sits in a deep plastic base; the front wall has vertical bars, a horizontal wire just above the
base's rim, another high up, and a top frame wire, and a small door with a flat steel frame. A filled 8 oz small-pet
water bottle in its steel hanger stands spout-down in a drip stand on the bench to the robot's right. Where the cage
and the stand are changes between trials.
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
import tilt  # noqa: E402  (the hanging grip leans the bottle toward the cage)
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

WIRE_C = (0.80, 0.80, 0.78)  # zinc-plated steel wire
BASE_C = (0.20, 0.55, 0.75)  # plastic base
FRAME_C = (0.55, 0.56, 0.58)  # door frame strip
BOTTLE_C = (0.80, 0.90, 0.97)  # clear bottle with water (opaque in the simulator)
CAP_C = (0.15, 0.35, 0.80)
TUBE_C = (0.60, 0.61, 0.63)  # stainless sipper tube
HOLD_C = (0.08, 0.08, 0.09)  # black epoxy-coated steel hanger
STAND_C = (0.92, 0.92, 0.90)


def _f(v):
    """Plain Python floats: a position of NumPy floats crashes the simulator at reset."""
    return tuple(float(x) for x in v)


# -- the cage, in its own frame: x = 0 is the front wall's bar plane (+x into the cage), y along the front wall, z up
# from the bench. Wire 2.5 mm, 12 mm clear between vertical bars (a Syrian-hamster wire top).
WIRE_R = 0.00125
GAP = 0.012
PITCH = GAP + 2 * WIRE_R
CAGE_W, CAGE_D = 0.460, 0.290  # outside length along the front wall, depth
FLOOR_T = 0.008  # the base's floor: the cage floor is this far above the bench
FLOOR_Z = FLOOR_T
HX = -2 * WIRE_R  # horizontal wires are welded on the outside of the vertical bars
Z_LOW = 0.0706  # horizontal wire just above the base's rim (centre, from the bench)
Z_HOOK = 0.2205  # the high horizontal wire a hanger hooks over
Z_TOP = 0.2700  # top frame wire
BASE_TOP = Z_LOW - WIRE_R  # the base's rim
DOOR_Y = (0.040, 0.120)  # door opening along the front wall (cage frame), its frame strips just outside this
DOOR_Z = (0.095, 0.250)
STRIP = (0.006, 0.008)  # door frame strip: proud of the bars (x), wide (y or z)
HOOK_Y = ((-CAGE_W / 2 + 0.004, DOOR_Y[0] - STRIP[1]), (DOOR_Y[1] + STRIP[1], CAGE_W / 2 - 0.004))  # high wire runs


def _bar_ys():
    n = int(CAGE_W / 2 / PITCH)
    return [(k + 0.5) * PITCH for k in range(-n, n)]


def _cage_shapes():
    out = []
    # plastic base: floor, front, sides and back up to the rim
    t = 0.004
    out.append(parts.box((CAGE_D, CAGE_W, FLOOR_T), at=(CAGE_D / 2, 0.0, FLOOR_T / 2), color=BASE_C))
    out.append(parts.box((t, CAGE_W, BASE_TOP - 0.0005), at=(-0.002 - t / 2 + 0.002, 0.0, (BASE_TOP - 0.0005) / 2),
                         color=BASE_C))
    out.append(parts.box((t, CAGE_W, BASE_TOP), at=(CAGE_D - t / 2, 0.0, BASE_TOP / 2), color=BASE_C))
    for s in (-1, 1):
        out.append(parts.box((CAGE_D, t, BASE_TOP), at=(CAGE_D / 2, s * (CAGE_W / 2 - t / 2), BASE_TOP / 2),
                             color=BASE_C))
    # front wall: vertical bars from the rim to the top frame
    h = Z_TOP - BASE_TOP
    for y in _bar_ys():
        out.append(parts.cylinder(WIRE_R, h, at=(0.0, y, BASE_TOP + h / 2), color=WIRE_C))
    # front horizontal wires: low and top run the full length; the high one stops at the door frame
    for z in (Z_LOW, Z_TOP):
        out.append(parts.cylinder(WIRE_R, CAGE_W, at=(HX, 0.0, z), color=WIRE_C, axis="y"))
    for y0, y1 in HOOK_Y:
        out.append(parts.cylinder(WIRE_R, y1 - y0, at=(HX, (y0 + y1) / 2, Z_HOOK), color=WIRE_C, axis="y"))
    # door frame: four flat strips proud of the bars, and a latch tab
    sx, sw = STRIP
    zc, zh = (DOOR_Z[0] + DOOR_Z[1]) / 2, DOOR_Z[1] - DOOR_Z[0] + 2 * sw
    for y in (DOOR_Y[0] - sw / 2, DOOR_Y[1] + sw / 2):
        out.append(parts.box((sx, sw, zh), at=(-WIRE_R - sx / 2, y, zc), color=FRAME_C))
    for z in (DOOR_Z[0] - sw / 2, DOOR_Z[1] + sw / 2):
        out.append(parts.box((sx, DOOR_Y[1] - DOOR_Y[0], sw), at=(-WIRE_R - sx / 2, (DOOR_Y[0] + DOOR_Y[1]) / 2, z),
                             color=FRAME_C))
    out.append(parts.box((0.010, 0.012, 0.004), at=(-WIRE_R - sx - 0.005, (DOOR_Y[0] + DOOR_Y[1]) / 2, DOOR_Z[0] + 0.02),
                         color=FRAME_C))
    # sides, back and roof: the same wire at the same pitch (never touched; no collision)
    for x in np.arange(PITCH, CAGE_D, PITCH):
        for s in (-1, 1):
            out.append(parts.cylinder(WIRE_R, h, at=(float(x), s * CAGE_W / 2, BASE_TOP + h / 2), color=WIRE_C,
                                      collide=False))
    for y in _bar_ys():
        out.append(parts.cylinder(WIRE_R, h, at=(CAGE_D, y, BASE_TOP + h / 2), color=WIRE_C, collide=False))
        out.append(parts.cylinder(WIRE_R, CAGE_D, at=(CAGE_D / 2, y, Z_TOP), color=WIRE_C, axis="x", collide=False))
    for s in (-1, 1):
        out.append(parts.cylinder(WIRE_R, CAGE_D, at=(CAGE_D / 2, s * CAGE_W / 2, Z_TOP), color=WIRE_C, axis="x",
                                  collide=False))
    out.append(parts.cylinder(WIRE_R, CAGE_W, at=(CAGE_D, 0.0, Z_TOP), color=WIRE_C, axis="y", collide=False))
    return out


# -- the bottle (Kaytee Clear 8 oz) in its hanger, in the bottle's own frame: origin at the middle of the body's
# lower face (the spout end), z up the bottle's axis toward its closed end, x toward the cage when hung.
BODY_L, BODY_D, BODY_W = 0.112, 0.044, 0.057  # oval body, flat sides 57 mm wide
CAP_R, CAP_L = 0.015, 0.012
TUBE_R, TUBE_L = 0.004, 0.050  # 8 mm stainless sipper tube with a ball tip
BOTTLE_KG = 0.39  # 237 ml of water, the bottle and the steel hanger
HANG_DEG = 35.0  # the hanger holds the bottle leaning this far from vertical, spout toward the cage
TIP_TARGET = 0.050  # the hanger is made so the tip sits 50 mm above the cage floor when it is hung on the high wire
A_H = np.array([math.sin(math.radians(HANG_DEG)), 0.0, -math.cos(math.radians(HANG_DEG))])  # hung: top -> spout
N_H = np.array([math.cos(math.radians(HANG_DEG)), 0.0, math.sin(math.radians(HANG_DEG))])  # hung: toward the cage
R_HUNG = np.c_[N_H, [0.0, 1.0, 0.0], -A_H]  # bottle frame -> cage frame when hung
TIP_L = np.array([0.0, 0.0, -(CAP_L + TUBE_L)])  # tube tip, bottle frame
CORNER_L = np.array([BODY_D / 2, 0.0, 0.0])  # the body's lower edge on the cage side


def _hung_origin_xz():
    """Where the bottle's origin sits (cage frame x, z) when hung: lower edge just off the bars, tip at target."""
    cx = -(WIRE_R + 0.0005) - (R_HUNG @ CORNER_L)[0]
    cz = FLOOR_Z + TIP_TARGET - (R_HUNG @ TIP_L)[2]
    return cx, cz


HUNG_X, HUNG_Z = _hung_origin_xz()


def to_bottle(p_cage, y=0.0) -> np.ndarray:
    """A cage-frame point (hung bottle at y) in the bottle's frame."""
    return R_HUNG.T @ (np.asarray(p_cage, float) - np.array([HUNG_X, y, HUNG_Z]))


HOLD_T = 0.004  # hanger rod, square
SEAT_C = np.array([HX, 0.0, Z_HOOK + WIRE_R + HOLD_T / 2])  # hook bar centre over the wire, hung
SEAT_L = to_bottle(SEAT_C - np.array([0.0, 0.0, HOLD_T / 2]))  # the hook bar's underside over the wire, bottle frame
HOOK_OUT = to_bottle(SEAT_C + np.array([-0.010, 0.0, 0.0]))
HOOK_IN = to_bottle(SEAT_C + np.array([0.008, 0.0, 0.0]))
LIP_BOT = to_bottle(SEAT_C + np.array([0.008, 0.0, -0.012]))
BAND_Z = 0.030  # the hanger's band round the body, up from the lower face
STRUT_LO = np.array([BODY_D / 2 + HOLD_T / 2, 0.0, BAND_Z])


def _rod(p0, p1, t=HOLD_T, color=HOLD_C):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    L = float(np.linalg.norm(d))
    pitch = math.atan2(-d[2], d[0])
    return parts.box((L + t, t, t), at=_f((p0 + p1) / 2), color=color, pitch=pitch)


def _bottle_shapes():
    body = [parts.box((BODY_D, BODY_W - 0.016, BODY_L), at=(0.0, 0.0, BODY_L / 2), color=BOTTLE_C),
            parts.box((BODY_D - 0.014, BODY_W, BODY_L), at=(0.0, 0.0, BODY_L / 2), color=BOTTLE_C),
            parts.cylinder(CAP_R, CAP_L, at=(0.0, 0.0, -CAP_L / 2), color=CAP_C),
            parts.cylinder(TUBE_R, TUBE_L, at=(0.0, 0.0, -CAP_L - TUBE_L / 2), color=TUBE_C)]
    t = 0.003
    band = [parts.box((t, BODY_W + 2 * t, 0.008), at=(s * (BODY_D / 2 + t / 2), 0.0, BAND_Z), color=HOLD_C)
            for s in (-1, 1)]
    band += [parts.box((BODY_D, t, 0.008), at=(0.0, s * (BODY_W / 2 + t / 2), BAND_Z), color=HOLD_C) for s in (-1, 1)]
    hanger = [_rod(STRUT_LO, HOOK_OUT), _rod(HOOK_OUT, HOOK_IN), _rod(HOOK_IN, LIP_BOT)]
    return body + band + hanger


# -- the drip stand: a plate with a hole for the cap and spout, on two posts on a base
PLATE_Z = 0.075  # plate top above the bench
HOLE = (0.034, 0.036)


def _stand_shapes():
    out = [parts.box((0.100, 0.110, 0.008), at=(0.0, 0.0, 0.004), color=STAND_C)]
    pt = 0.006
    ox, oy = 0.070, 0.084
    hx, hy = HOLE
    zc = PLATE_Z - pt / 2
    out.append(parts.box(((ox - hx) / 2, oy, pt), at=(-(ox + hx) / 4, 0.0, zc), color=STAND_C))
    out.append(parts.box(((ox - hx) / 2, oy, pt), at=((ox + hx) / 4, 0.0, zc), color=STAND_C))
    out.append(parts.box((hx, (oy - hy) / 2, pt), at=(0.0, -(oy + hy) / 4, zc), color=STAND_C))
    out.append(parts.box((hx, (oy - hy) / 2, pt), at=(0.0, (oy + hy) / 4, zc), color=STAND_C))
    ph = PLATE_Z - pt - 0.008
    for s in (-1, 1):
        out.append(parts.box((0.012, 0.010, ph), at=(0.0, s * 0.035, 0.008 + ph / 2), color=STAND_C))
    return out


# -- layout (world: x away from the robot, y to its left, z up)
CAGE_X = 0.420  # the front wall's bar plane
CAGE_A = (CAGE_X, -0.120, TOP)
CAGE_B = (CAGE_X, -0.316, TOP)  # lot B: the cage 196 mm to the robot's right, its door where lot A's bottle hangs
STAND_A = (0.240, -0.200, TOP)
STAND_B = (0.240, -0.330, TOP)
CAGE = parts.Part("cage", pos=_f(CAGE_A), mass=2.0, shapes=_cage_shapes(), friction=(0.6, 0.5), contact_offset=0.001)
STAND = parts.Part("stand", pos=_f(STAND_A), mass=0.6, shapes=_stand_shapes(), friction=(0.7, 0.6),
                   contact_offset=0.001)
BOTTLE = parts.Part("bottle", pos=_f((STAND_A[0], STAND_A[1], TOP + PLATE_Z)), mass=BOTTLE_KG, shapes=_bottle_shapes(),
                    friction=(0.7, 0.6), contact_offset=0.001)

OBJECTS = [CAGE, STAND, BOTTLE]


def _bottle_on(stand):
    return (stand[0], stand[1], TOP + PLATE_Z)


LOTS = lots.declare(OBJECTS,
                    lots.Lot("A", place={"cage": CAGE_A, "stand": STAND_A, "bottle": _bottle_on(STAND_A)}),
                    lots.Lot("B", place={"cage": CAGE_B, "stand": STAND_B, "bottle": _bottle_on(STAND_B)}))
SUCCESS_HOLD_S = 1.5  # the 1 s hold is enforced in score() itself; this only keeps a run going past success


# -- scoring geometry --------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def in_cage(objects, p_bottle) -> np.ndarray:
    """Bottle-frame points in the cage's frame (as the cage stands now)."""
    b, c = objects["bottle"], objects["cage"]
    rb, rc = _rot(b.quat_xyzw), _rot(c.quat_xyzw)
    w = np.asarray(b.pos, float) + np.atleast_2d(p_bottle) @ rb.T
    return (w - np.asarray(c.pos, float)) @ rc


def measures(objects) -> dict:
    """Everything the score looks at, in the cage's frame (m, degrees)."""
    b, c = objects["bottle"], objects["cage"]
    rb, rc = _rot(b.quat_xyzw), _rot(c.quat_xyzw)
    tip, seat, corner, o = in_cage(objects, np.array([TIP_L, SEAT_L, CORNER_L, np.zeros(3)]))
    spout = rc.T @ (rb @ np.array([0.0, 0.0, -1.0]))  # the tube's direction, cage frame
    return dict(tip=tip, seat=seat, corner=corner, down=math.degrees(math.asin(max(-1.0, min(1.0, -spout[2])))),
                origin=o)


BAR_OUT, BAR_IN = 0.010, 0.008  # the hook's top bar runs this far out from / in to its seat point (lip end)
BAR_END = 0.002  # the wire lies under the bar at least this far in from either end
SEAT_DZ_LO, SEAT_DZ_HI = -0.0010, 0.0015  # bar underside to the wire's top
TIP_IN = 0.008  # tube tip at least this far inside the bar plane
TIP_LO, TIP_HI = 0.040, 0.060  # tip height above the cage floor
DOWN_MIN = 20.0
REST_X = -0.006  # the body's lower edge within this of the bar plane: resting against the wires


def hung(objects) -> bool:
    m = measures(objects)
    s, tp = m["seat"], m["tip"]
    w = HX - s[0]  # where the wire lies along the bar from the seat point (+: toward the lip)
    on_wire = (-(BAR_OUT - BAR_END) - EPS <= w <= BAR_IN - BAR_END + EPS and SEAT_DZ_LO - EPS <= s[2] - (Z_HOOK + WIRE_R) <= SEAT_DZ_HI + EPS
               and any(y0 + 0.002 - EPS <= s[1] <= y1 - 0.002 + EPS for y0, y1 in HOOK_Y))
    tip_ok = (tp[0] >= TIP_IN - EPS and TIP_LO - EPS <= tp[2] - FLOOR_Z <= TIP_HI + EPS
              and abs(tp[1]) <= CAGE_W / 2 - 0.010 + EPS and tp[0] <= CAGE_D - 0.010)
    return bool(on_wire and tip_ok and m["down"] >= DOWN_MIN - EPS and m["corner"][0] >= REST_X - EPS)


# -- trial record ----------------------------------------------------------------------------------------------
CLEAR = 0.005
STILL_M = 0.001
HOLD_S = 1.0
CAGE_SHOVE, CAGE_TILT = 0.010, 3.0  # the cage pushed more than 10 mm or tipped more than 3 degrees: no credit
_rec = parts.__dict__.setdefault("hamster_bottle_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", gap=None, since=None, held=False, snap=None, t=0.0, ok=False,
                cage0=None, shoved=False, worst_shove=0.0)


if not _rec:
    reset_record()


def _snap(objects) -> np.ndarray:
    b, c = objects["bottle"], objects["cage"]
    rb, rc = _rot(b.quat_xyzw), _rot(c.quat_xyzw)
    pts = [np.asarray(b.pos) + rb @ p for p in (np.zeros(3), TIP_L, SEAT_L, np.array([0.0, 0.0, BODY_L]))]
    pts += [np.asarray(c.pos) + rc @ p for p in (np.zeros(3), np.array([0.0, 0.2, 0.25]), np.array([0.3, -0.2, 0.0]))]
    return np.concatenate(pts)


def _cage_move(objects) -> tuple[float, float]:
    c = objects["cage"]
    p0, r0 = _rec["cage0"]
    rc = _rot(c.quat_xyzw)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(r0.T @ rc) - 1) / 2))))
    return float(np.linalg.norm(np.asarray(c.pos, float) - p0)), ang


def observe(objects, gap, t) -> None:
    """One control step. ``gap``: closest sampled hand point to the bottle and hanger (m), or None when the hands could
    not be read, which ends all credit for the trial."""
    _rec["steps"] += 1
    _rec["t"] = float(t)
    try:
        b, c = objects["bottle"], objects["cage"]
        vals = [*b.pos, *b.quat_xyzw, *c.pos, *c.quat_xyzw]
        if not all(math.isfinite(float(v)) for v in vals):
            raise ValueError("bottle or cage state unknown")
        if _rec["cage0"] is None:
            _rec["cage0"] = (np.asarray(c.pos, float).copy(), _rot(c.quat_xyzw))
        d, ang = _cage_move(objects)
        _rec["worst_shove"] = max(_rec["worst_shove"], d)
        if d > CAGE_SHOVE + EPS or ang > CAGE_TILT + EPS:
            _rec["shoved"] = True
        if gap is None or not math.isfinite(gap):
            raise ValueError("hand state unknown")
        ok = hung(objects)
        snap = _snap(objects)
    except (KeyError, ValueError, TypeError, AttributeError, IndexError) as e:
        _rec.update(known=False, reason=str(e), since=None, held=False)
        return
    _rec["gap"] = float(gap)
    if not ok or gap <= CLEAR + EPS:
        _rec.update(since=None, held=False, snap=None, ok=ok)
    elif _rec["since"] is None or float(np.abs(snap - _rec["snap"]).max()) > STILL_M + EPS:
        _rec.update(since=float(t), held=False, snap=snap, ok=True)
    else:
        _rec["held"] = float(t) - _rec["since"] >= HOLD_S - EPS


def score(objects):
    """0 for the whole trial if the hands, the bottle or the cage could not be read at any step, or if the cage was
    ever pushed more than 10 mm or tipped more than 3 degrees from where it started. Otherwise 1.0 when, with both
    hands more than 5 mm from the bottle and its hanger and nothing moving more than 1 mm for 1 s: the hanger's hook
    rests on top of the front wall's high horizontal wire (under its top bar, 2 mm or more from either end), the tube's tip is at least 8 mm inside the bars and 40 to
    60 mm above the cage floor, the tube points down at least 20 degrees, and the bottle's lower end rests against
    the wires (within 6 mm of the bar plane). 0 otherwise."""
    try:
        if not _rec["known"] or _rec["shoved"] or _rec["steps"] == 0 or not _rec["held"]:
            return 0.0
        if _rec["gap"] is None or _rec["gap"] <= CLEAR + EPS:
            return 0.0
        if not hung(objects):
            return 0.0
        if float(np.abs(_snap(objects) - _rec["snap"]).max()) > STILL_M + EPS:
            return 0.0
        return 1.0
    except (KeyError, ValueError, TypeError, AttributeError, IndexError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - 1e-9


def distance(objects):
    try:
        m = measures(objects)
        return (abs(m["seat"][0] - HX) + abs(m["seat"][2] - Z_HOOK - WIRE_R) + max(0.0, TIP_IN - m["tip"][0])
                + abs(m["tip"][2] - FLOOR_Z - TIP_TARGET))
    except (KeyError, ValueError, TypeError, AttributeError):
        return 1.0


# -- hands: hull faces sampled every 3 mm ------------------------------------------------------------------------
_HAND_LINKS = ("gripper", "tip_left", "tip_right")
_FACES: dict = {}


def _hull_face_points(hull, step=0.003) -> np.ndarray:
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


def link_points(name) -> np.ndarray:
    if name not in _FACES:
        from yam_arena import analysis

        hs = analysis.link_hulls().get(name, [])
        if not hs:
            raise ValueError(f"no collision hull for {name}")
        _FACES[name] = np.concatenate([_hull_face_points(h) for h in hs])
    return _FACES[name]


def bottle_boxes(b):
    return [(c, r, s) for _, c, r, s in BOTTLE.world_boxes({"base": (np.asarray(b.pos, float), _rot(b.quat_xyzw))})]


def hand_gap(objects, pts) -> float:
    """Closest hand point to the bottle or its hanger (m; negative inside). Round parts count as their boxes."""
    from yam_arena.analysis import box_signed_distance

    b = objects["bottle"]
    pts = pts[np.linalg.norm(pts - np.asarray(b.pos, float), axis=1) < 0.40]
    if len(pts) == 0:
        return 1.0
    return float(min(box_signed_distance(pts, c, r, s).min() for c, r, s in bottle_boxes(b)))


def _hand_points(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                out.append(link_points(n) @ _rot(poses[i, 3:]).T + (poses[i, :3] - origin))
                found += 1
    if found != 2 * len(_HAND_LINKS):
        return None
    pts = np.concatenate(out)
    return pts if np.isfinite(pts).all() else None


# -- simulator hooks ---------------------------------------------------------------------------------------------
@parts.on_reset
def _new_trial(sim) -> None:
    reset_record()
    _rec["last_t"] = None


@parts.on_step
def _step(sim) -> None:
    if _rec.get("last_t") == sim.t:
        return
    _rec["last_t"] = sim.t
    try:
        objects = sim.states()
    except Exception:  # noqa: BLE001  unknown state: no credit
        objects = {}
    g = None
    try:
        pts = _hand_points(sim)
        g = hand_gap(objects, pts) if pts is not None else None
    except Exception:  # noqa: BLE001
        g = None
    observe(objects, g, float(sim.t))
    log = str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("hang.jsonl")) if os.environ.get("TRACE_OUT") else None
    if log and _rec["steps"] % 5 == 0 and objects:
        try:
            m = measures(objects)
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, score=score(objects), hung=hung(objects),
                       tip=np.round(m["tip"], 4).tolist(), seat=np.round(m["seat"], 4).tolist(),
                       corner_x=round(float(m["corner"][0]), 4), down=round(m["down"], 1),
                       bottle=np.round(np.asarray(objects["bottle"].pos), 4).tolist(), gap=_rec["gap"],
                       held=_rec["held"], shove=round(_rec["worst_shove"], 4), known=_rec["known"],
                       reason=_rec["reason"])
            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            a = base.scene["right_robot"]
            poses = parts._np(a.data.body_link_pose_w)[0]
            row["right_g"] = np.round(poses[list(a.body_names).index("gripper"), :3] - origin, 4).tolist()
            with open(log, "a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with open(log, "a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- demo: open loop from the published layout (where the cage, its door and the stand are) ----------------------
G_L = np.array([0.0, 0.0, BODY_L - 0.035])  # grip point (TCP), bottle frame: 35 mm below the closed end
F_GRIP, F_WIDE, F_LET = -0.0190, -0.0400, -0.0320  # fingers across the body's 44 mm depth
FLIP = np.diag([1.0, -1.0, -1.0])  # bottle frame -> TCP frame (fingers point down the bottle toward the spout)
LIFT_HOOK = 0.016  # carried this far above the hung height while the tube goes in, so the hook clears the wire
BACK = 0.060  # start of the push along the tube, back from the hung pose
PREFER_Y = -0.270  # the hanging spot the right arm would choose in an open bay (world y)
PAST_DOOR = BODY_W / 2 + STRIP[1] + 0.006  # the bottle's middle stays this far from the door frame


def hang_y(cage_y) -> float:
    """The bar gap (world y) to hang at: the one nearest PREFER_Y whose hook and bottle clear the door frame."""
    best = None
    for k in range(-20, 21):
        y = k * PITCH  # gap centres in the cage frame
        if not any(y0 + BODY_W / 2 <= y <= y1 - BODY_W / 2 for y0, y1 in HOOK_Y):
            continue
        if DOOR_Y[0] - PAST_DOOR < y < DOOR_Y[1] + PAST_DOOR:
            continue
        if best is None or abs(cage_y + y - PREFER_Y) < abs(cage_y + best - PREFER_Y):
            best = y
    return float(cage_y + best)


# A fixed calibration for carrying the 0.39 kg bottle leaned out (measured once on the stock arm, GPU trace 1): the
# arm settles about 4.8 degrees less leaned than asked, and its TCP about 12 mm short, 2 mm right and 18 mm low.
# It is the same for every layout and reads nothing from the simulation.
LEAN_EXTRA = math.radians(4.8)
SAG = np.array([0.012, 0.002, 0.018])


def hung_tcp(cage, y, up=0.0, back=0.0, carry=True):
    """TCP position and rotation holding the bottle hung at world y, raised ``up`` and drawn back ``back`` along the
    tube; ``carry`` adds the fixed carrying calibration."""
    o = np.array([cage[0] + HUNG_X, y, cage[2] + HUNG_Z]) - back * A_H + up * np.array([0.0, 0.0, 1.0])
    p, r = o + R_HUNG @ G_L, R_HUNG @ FLIP
    if carry:
        p, r = p + SAG, parts._rot((0, 1, 0), -LEAN_EXTRA) @ r
    return p, r


def stand_tcp(stand, up=0.0):
    o = np.array([stand[0], stand[1], stand[2] + PLATE_Z + up])
    return o + G_L, FLIP


def _rest_pose(arm):
    from yam_arena import ik, station
    from yam_arena.demo import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return tuple(float(v) for v in pos), tilt._TiltedYaw(0.0, rot)


def W(d, p, f, lab, rot):
    return Waypoint(float(d), _f(p), float(f), lab, tilt._TiltedYaw(0.0, rot))


def demo():
    lot = lots.current()
    cage = lot.place.get("cage", CAGE_A)
    stand = lot.place.get("stand", STAND_A)
    y = hang_y(cage[1])
    gp, gr = stand_tcp(stand)
    up = np.array([0.0, 0.0, 1.0])
    p_back, r_h = hung_tcp(cage, y, LIFT_HOOK, BACK)
    p_in, _ = hung_tcp(cage, y, LIFT_HOOK)
    p_seat, _ = hung_tcp(cage, y, -0.002)
    p_off, _ = hung_tcp(cage, y, -0.002, 0.070)
    rest, ryaw = _rest_pose("right")
    right = [
        W(1.5, gp + 0.060 * up, F_WIDE, "over the bottle", gr),
        W(1.2, gp, F_WIDE, "fingers either side of the body", gr),
        W(0.8, gp, F_GRIP, "grip", gr),
        W(0.3, gp, F_GRIP, "settle", gr),
        W(1.2, gp + 0.068 * up, F_GRIP, "lift the spout out of the stand", gr),
        W(3.0, p_back, F_GRIP, "lean it spout-first toward the bars, hook above the wire", r_h),
        W(0.8, p_back, F_GRIP, "settle", r_h),
        W(2.5, p_in, F_GRIP, "tube in through the gap along its own line", r_h),
        W(0.5, p_in, F_GRIP, "settle", r_h),
        W(1.5, p_seat, F_GRIP, "lower the hook onto the wire", r_h),
        W(0.5, p_seat, F_GRIP, "settle", r_h),
        W(0.8, p_seat, F_LET, "let go", r_h),
        W(1.5, p_off, F_LET, "draw the fingers back off the bottle", r_h),
        W(1.2, p_off + 0.06 * up, F_LET, "up and away", r_h),
        Waypoint(2.0, rest, F_LET, "back to the rest pose", ryaw),
        Waypoint(4.0, rest, F_LET, "wait", ryaw),
    ]
    left = [Waypoint(sum(w.duration for w in right), (0.20, 0.25, TOP + 0.22), FINGER_OPEN, "wait")]
    plans = {"left": left, "right": right}
    for arm, plan in plans.items():
        rest, ryaw = _rest_pose(arm)
        plan[:0] = [Waypoint(2.0, rest, FINGER_OPEN, "to the rest pose", ryaw),
                    Waypoint(0.5, rest, FINGER_OPEN, "settle at the rest pose", ryaw)]
    return plans
