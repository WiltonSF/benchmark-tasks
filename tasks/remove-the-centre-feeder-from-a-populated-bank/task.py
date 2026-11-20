"""An electronics bench. A short black 20 x 20 mm V-slot aluminium rail stands on two feet, running straight away
from the robot. Three black 8 mm tape feeders sit on it side by side at the rail's 15.5 mm slot pitch, each hooked
over the rail and clipped to a yellow slot block on the rail's left face, with a white 7 inch component reel hanging
from it on the left. The three units are identical and touch-close: about 0.5 mm between feeder bodies and 2.5 mm
between reels. To the left of the bank lies a blue service mat. Nobody is in the scene.
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
import observation  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
D = math.radians

# -- colours ------------------------------------------------------------------------------------------------------------
RAIL_BLACK = (0.12, 0.12, 0.13)
FEEDER_BLACK = (0.07, 0.07, 0.08)
YELLOW = (0.95, 0.72, 0.08)
REEL_WHITE = (0.90, 0.91, 0.92)
TAPE = (0.20, 0.17, 0.14)
MAT_BLUE = (0.16, 0.33, 0.62)
LABEL = (0.95, 0.80, 0.25)


def _qy(a: float) -> tuple[float, float, float, float]:
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


# -- geometry, in a feeder's own frame (metres) ---------------------------------------------------------------------------
# u: inward, from the reel side to the pick end; w: across, along the rail (the slot pitch); z: up. Origin: the rail's
# centre line at the slot centre. Opulo LumenPnP 8 mm feeder: frame-8 envelope 164.8 x 82.8 x 15 mm (maker CAD, feeder
# repository commit c9869fe); 15 mm wide; spool arm carrying a standard 7 inch (178 mm) reel. Shapes below are a
# simplified copy of the hook, slot-block clip and lever; their exact dimensions are estimates from the maker's photos.
PITCH = 0.0155  # slot pitch (estimate: 15 mm feeder plus clearance)
W2 = 0.0075  # half the feeder width
Z_R = 0.112  # rail centre height above the bench (reels clear the bench by 8 mm)

RAIL_CORE = ((-0.010, 0.010), (-0.010, 0.006))  # (u range, z range)
RAIL_TOP = [((-0.010, -0.00205), (0.006, 0.010)), ((0.00205, 0.010), (0.006, 0.010))]  # the top-face slot, 4.1 x 4 mm
STRIP = ((-0.016, -0.010), (-0.016, 0.010), 0.00725)  # slot block on the rail's outer face, half width in w
PEG = ((-0.0215, -0.016), (-0.002, 0.004), 0.0025)  # alignment peg on the slot block

BODY = [((-0.018, -0.001), (0.01005, 0.052)), ((-0.001, 0.140), (0.0115, 0.052))]  # rests on the rail's outer half
TAB = ((-0.018, -0.006), (0.052, 0.064))
PRONG = ((-0.0245, -0.0175), (-0.012, 0.01005), (0.0032, 0.0075))  # two prongs either side of the peg
BEAR = ((-0.0175, -0.01605), (-0.006, -0.003), (0.0032, 0.0075))  # the pads that bear on the slot block
BRIDGE = ((-0.0245, -0.0175), (0.00405, 0.01005), 0.0032)
STEM = ((-0.0010, 0.0010), (0.0083, 0.0115))
NOSE = ((0.0, 0.0083), 0.0020)  # round hook nose in the rail's top slot; the feeder turns about it
NOSE_W = 0.014  # the nose runs along the slot for nearly the feeder's width
ARM = [((-0.060, -0.018), (0.016, 0.028)), ((-0.125, -0.055), (-0.020, 0.022))]  # spool arm (stepped)
HUB = (-0.125, -0.015)
REEL_R, REEL_W = 0.089, 0.013
PIVOT = (-0.0200, -0.0103)  # release lever pivot, outboard of the tooth so a lift on the tooth turns it
TOOTH = ((-0.0156, -0.01855), 0.0025, (0.0034, 0.0070))  # one tooth either side of the peg (w and -w)
LEVER_SIDE = ((-0.0215, -0.0175), (-0.018, -0.010))  # the lever's two side bars, beside the peg
LEVER_CROSS = [((-0.028, -0.0215), (-0.013, -0.010)), ((-0.028, -0.025), (-0.013, 0.0))]  # crossbar and thumb tab

FEEDER_MASS = 0.29  # kg, body with electronics and a loaded reel (estimate)
LEVER_MASS = 0.025
LEVER_SPRING, LEVER_PRELOAD = 0.10, -0.30  # N m/rad, rad (target beyond the closed stop: a preloaded spring)


def _box(ur, zr, wr, color, collide=True, quat=None, friction=None):
    if isinstance(wr, (int, float)):
        wr = (-wr, wr)
    size = (ur[1] - ur[0], wr[1] - wr[0], zr[1] - zr[0])
    at = ((ur[0] + ur[1]) / 2, (wr[0] + wr[1]) / 2, (zr[0] + zr[1]) / 2)
    return parts.box(size, at=at, color=color, quat=quat, collide=collide, friction=friction)


def _feeder_shapes():
    out = [_box(u, z, W2, FEEDER_BLACK) for u, z in BODY]
    out.append(_box(*TAB, W2, FEEDER_BLACK))
    for s in (1, -1):
        w = PRONG[2] if s > 0 else (-PRONG[2][1], -PRONG[2][0])
        out.append(_box(PRONG[0], PRONG[1], w, FEEDER_BLACK))
        out.append(_box(BEAR[0], BEAR[1], w, FEEDER_BLACK))
    out.append(_box(BRIDGE[0], BRIDGE[1], BRIDGE[2], FEEDER_BLACK))
    out.append(_box(STEM[0], STEM[1], NOSE_W / 2, FEEDER_BLACK))
    out.append(parts.cylinder(NOSE[1], NOSE_W, at=(NOSE[0][0], 0.0, NOSE[0][1]), color=FEEDER_BLACK, axis="y"))
    out.append(_box(ARM[0][0], ARM[0][1], 0.0025, FEEDER_BLACK))
    out.append(_box(ARM[1][0], ARM[1][1], 0.0025, FEEDER_BLACK, collide=False))
    out.append(parts.cylinder(REEL_R, REEL_W, at=(HUB[0], 0.0, HUB[1]), color=REEL_WHITE, axis="y"))
    # looks only: the wound tape seen through the reel's faces, the hub and a label strip on the feeder's side
    for s in (1, -1):
        out.append(parts.cylinder(0.070, 0.0004, at=(HUB[0], s * (REEL_W / 2 + 0.0002), HUB[1]), color=TAPE, axis="y",
                                  collide=False))
        out.append(parts.cylinder(0.018, 0.0006, at=(HUB[0], s * (REEL_W / 2 + 0.0005), HUB[1]), color=REEL_WHITE,
                                  axis="y", collide=False))
        out.append(parts.cylinder(0.006, 0.0008, at=(HUB[0], s * (REEL_W / 2 + 0.0008), HUB[1]), color=YELLOW, axis="y",
                                  collide=False))
        out.append(_box((0.030, 0.075), (0.030, 0.040), (s * W2 - 0.0002, s * W2 + 0.0002) if s > 0 else
                        (-W2 - 0.0002, -W2 + 0.0002), LABEL, collide=False))
    return out


def _lever_shapes():
    out = []
    for s in (1, -1):
        w = TOOTH[2] if s > 0 else (-TOOTH[2][1], -TOOTH[2][0])
        out.append(parts.cylinder(TOOTH[1], w[1] - w[0], at=(TOOTH[0][0], (w[0] + w[1]) / 2, TOOTH[0][1]), color=YELLOW,
                                  axis="y"))
        out.append(_box(*LEVER_SIDE, w, YELLOW))
    out += [_box(u, z, TOOTH[2][1], YELLOW) for u, z in LEVER_CROSS]
    return out


def _rail_shapes():
    L = 0.050  # half length of the rail section
    out = [_box(*RAIL_CORE, L, RAIL_BLACK)]
    for u, z in RAIL_TOP:
        out.append(_box(u, z, L, RAIL_BLACK))
    for s in (1, -1):  # looks only: the V grooves on the two side faces
        out.append(_box((s * 0.0101 - 0.0001, s * 0.0101 + 0.0001), (-0.003, 0.003), L, (0.05, 0.05, 0.05), collide=False))
    for k in (-1, 0, 1):
        w = k * PITCH
        out.append(_box(STRIP[0], STRIP[1], (w - STRIP[2], w + STRIP[2]), YELLOW))
        out.append(_box(PEG[0], PEG[1], (w - PEG[2], w + PEG[2]), YELLOW))
    for s in (1, -1):  # two feet under the rail ends
        w = (0.040, 0.050) if s > 0 else (-0.050, -0.040)
        out.append(_box((-0.010, 0.010), (-Z_R + 0.006, -0.010), w, RAIL_BLACK))
        wf = (0.032, 0.058) if s > 0 else (-0.058, -0.032)
        out.append(_box((-0.030, 0.030), (-Z_R, -Z_R + 0.006), wf, RAIL_BLACK))
    return out


# -- layout -------------------------------------------------------------------------------------------------------------
X0, Y_R = 0.240, -0.060  # the middle slot: rail centre line under it
YAW = -math.pi / 2  # feeder frame u -> world -y, w -> world +x
NAMES = ("feeder_front", "feeder_middle", "feeder_back")  # nearest the robot first
MIDDLE = "feeder_middle"
NEIGHBOURS = ("feeder_front", "feeder_back")


def _feeder(name, k):
    lever = parts.Hinge("lever", shapes=tuple(_lever_shapes()), pivot=(PIVOT[0], 0.0, PIVOT[1]), axis=(0.0, 1.0, 0.0),
                        lower=0.0, upper=0.6, mass=LEVER_MASS, spring=LEVER_SPRING, damping=0.002, max_effort=1.0,
                        target=LEVER_PRELOAD)
    return parts.Part(name, pos=(X0 + k * PITCH, Y_R, TOP + Z_R), yaw=YAW, mass=FEEDER_MASS,
                      shapes=tuple(_feeder_shapes()), joints=(lever,), friction=(0.9, 0.8), contact_offset=0.002,
                      link_friction=(("lever", (0.2, 0.15)),))


FEEDERS = {n: _feeder(n, k) for n, k in zip(NAMES, (-1, 0, 1))}
RAIL = parts.Part("feeder_rail", pos=(X0, Y_R, TOP + Z_R), yaw=YAW, shapes=tuple(_rail_shapes()), friction=(0.3, 0.25),
                  contact_offset=0.002)
MAT_SIZE = (0.300, 0.420, 0.003)
MAT_A = (0.350, 0.360)
MAT = parts.Part("service_mat", pos=(*MAT_A, TOP), mass=0.5,
                 shapes=(parts.box(MAT_SIZE, at=(0.0, 0.0, MAT_SIZE[2] / 2), color=MAT_BLUE),), friction=(0.9, 0.8),
                 contact_offset=0.002)
OBJECTS = [RAIL, *FEEDERS.values(), MAT]
LOTS = lots.declare(OBJECTS, lots.Lot("A", place={"service_mat": (*MAT_A, TOP)}),
                    lots.Lot("B", place={"service_mat": (MAT_A[0] - 0.030, MAT_A[1], TOP)}))

# -- scoring constants --------------------------------------------------------------------------------------------------
NB_MOVE, NB_TURN = 0.002, D(3.0)  # neighbour limits at the end of the trial
NB_LOOSE_MOVE, NB_LOOSE_TURN = 0.005, D(10.0)  # beyond either at any moment, a neighbour has been knocked loose
CLEAR = 0.005  # middle feeder clear of the rail, slot blocks and pegs
HAND_CLEAR = 0.005  # released: no gripper or fingertip within this of the middle feeder
FLAT = D(20.0)  # lying on a face: its across axis within this of vertical
STILL_V, STILL_W, HOLD_S = 0.005, 0.05, 1.0
DROP_V = 1.2  # m/s: any feeder this fast has been dropped (a free fall from about 7 cm)
SUCCESS_HOLD_S = 1.0


# -- geometry helpers ---------------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def _surface_points(shape, step=0.004) -> np.ndarray:
    """Points on a shape's surface in its part (or link) frame."""
    if shape.kind == "cylinder":  # all cylinders here lie along the frame's y axis
        r, h = shape.size[0], shape.size[1]
        n = max(12, int(2 * math.pi * r / step))
        a = np.linspace(0, 2 * math.pi, n, endpoint=False)
        ring = np.c_[r * np.cos(a), np.zeros(n), r * np.sin(a)]  # axis y
        pts = [ring + (0, y, 0) for y in (-h / 2, h / 2)]
        pts.append(np.array([[0.0, -h / 2, 0.0], [0.0, h / 2, 0.0]]))
        loc = np.vstack(pts)
    else:
        half = np.asarray(shape.size, float) / 2
        axes = [np.linspace(-hh, hh, max(2, int(2 * hh / step) + 1)) for hh in half]
        g = np.array(np.meshgrid(*axes, indexing="ij")).reshape(3, -1).T
        loc = g[np.any(np.isclose(np.abs(g), half, atol=1e-9), axis=1)]
    return loc @ shape.R.T + np.asarray(shape.at, float)


_PTS = {}


def feeder_points(st) -> np.ndarray:
    """World points on every collider of a feeder (body and lever), from its reported pose and lever link pose."""
    if "base" not in _PTS:
        _PTS["base"] = np.vstack([_surface_points(s) for s in _feeder_shapes() if s.collide])
        lever_pts = np.vstack([_surface_points(s) for s in _lever_shapes()])
        _PTS["lever"] = lever_pts - np.array([PIVOT[0], 0.0, PIVOT[1]])  # relative to the hinge (link frame origin)
    p, r = np.asarray(st.pos, float), _rot(st.quat_xyzw)
    out = [_PTS["base"] @ r.T + p]
    if "lever" not in getattr(st, "links", {}):
        raise ValueError("lever link pose unknown")
    lp, lq = (np.asarray(a, float) for a in st.links["lever"])
    out.append(_PTS["lever"] @ _rot(lq).T + lp)
    return np.vstack(out)


def _rail_boxes():
    if "rail" not in _PTS:
        st = RAIL.state()
        _PTS["rail"] = [(c, r, s) for _, c, r, s in RAIL.world_boxes({"base": (np.asarray(st.pos), _rot(st.quat_xyzw))})]
    return _PTS["rail"]


def rail_gap(pts) -> float:
    """Least distance (m) from any point to the rail, its slot blocks, pegs and feet."""
    best = math.inf
    for c, r, s in _rail_boxes():
        loc = (pts - c) @ r
        q = np.abs(loc) - np.asarray(s, float) / 2
        d = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
        best = min(best, float(d.min()))
    return best


def _shape_frames(st):
    """(shape, world centre, world rotation) for every collider of a feeder: body shapes and lever shapes."""
    p, r = np.asarray(st.pos, float), _rot(st.quat_xyzw)
    out = [(s, p + r @ np.asarray(s.at, float), r @ s.R) for s in _feeder_shapes() if s.collide]
    if "lever" not in getattr(st, "links", {}):
        raise ValueError("lever link pose unknown")
    lp, lq = (np.asarray(a, float) for a in st.links["lever"])
    lr = _rot(lq)
    piv = np.array([PIVOT[0], 0.0, PIVOT[1]])
    out += [(s, lp + lr @ (np.asarray(s.at, float) - piv), lr @ s.R) for s in _lever_shapes()]
    return out


def _shape_distance(pts, shape, c, r) -> np.ndarray:
    """Exact signed distance (m) from points to one box, or to one solid cylinder along its frame's y axis."""
    loc = (pts - c) @ r
    if shape.kind == "cylinder":
        rad, h = shape.size[0], shape.size[1]
        q = np.c_[np.hypot(loc[:, 0], loc[:, 2]) - rad, np.abs(loc[:, 1]) - h / 2]
    else:
        q = np.abs(loc) - np.asarray(shape.size, float) / 2
    return np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)


def hand_gap(clouds, st) -> float:
    """Least distance from any face-sampled gripper or fingertip surface point to any collider of the feeder, computed
    exactly against each box and cylinder (reel faces included). Capped at 30 mm."""
    if not clouds:
        raise ValueError("no hand geometry")
    frames = _shape_frames(st)
    best = 0.03
    for c in clouds:
        c = np.asarray(c, float)
        if c.ndim != 2 or c.shape[1] != 3 or not len(c) or not np.isfinite(c).all():
            raise ValueError("hand geometry malformed")
        for shape, cen, rot in frames:
            reach = float(np.linalg.norm(shape.half)) + 0.03
            near = c[np.linalg.norm(c - cen, axis=1) <= reach]
            if len(near):
                best = min(best, float(_shape_distance(near, shape, cen, rot).min()))
    return best


def flat(st) -> bool:
    across = _rot(st.quat_xyzw)[:, 1]  # the feeder's w axis
    return bool(abs(across[2]) >= math.cos(FLAT) - EPS)


def on_mat(st, mat) -> bool:
    """Every sampled point of the middle feeder lies over the mat, and its lowest point within 5 mm of the mat top."""
    pts = feeder_points(st)
    mp, mr = np.asarray(mat.pos, float), _rot(mat.quat_xyzw)
    loc = (pts - mp) @ mr
    inside = np.all(np.abs(loc[:, :2]) <= np.array(MAT_SIZE[:2]) / 2 + EPS)
    low = float(loc[:, 2].min())
    return bool(inside and -0.001 <= low <= MAT_SIZE[2] + 0.005 + EPS)


def on_bench(st, mat) -> bool:
    """Resting on the bench or the mat: its lowest point within 5 mm of the bench top or of the mat top."""
    pts = feeder_points(st)
    low = float(pts[:, 2].min())
    mp = np.asarray(mat.pos, float)
    return bool(TOP - 0.001 <= low <= TOP + 0.005 + EPS or
                mp[2] + MAT_SIZE[2] - 0.001 <= low <= mp[2] + MAT_SIZE[2] + 0.005 + EPS)


# -- the trial record ---------------------------------------------------------------------------------------------------
def _fresh():
    return dict(known=False, steps=0, t=None, start=None, zero=False, why="", gap=math.inf, hand=0.0, since=None,
                held=0.0, prev=None)


_rec = _fresh()


def _turn(qa, qb) -> float:
    r = _rot(qa).T @ _rot(qb)
    return math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))


def observe(objects, clouds, t) -> None:
    """One control step. Unknown or malformed observations fail closed for the rest of the trial."""
    try:
        sts = {n: objects[n] for n in (*NAMES, MAT.name)}
        for n, st in sts.items():
            arr = np.r_[st.pos, st.quat_xyzw, st.lin_vel, st.ang_vel]
            if arr.shape != (13,) or not np.isfinite(arr).all() or np.linalg.norm(st.quat_xyzw) <= EPS:
                raise ValueError("state unknown: " + n)
        if _rec["t"] is not None and not t > _rec["t"]:
            raise ValueError("time did not advance")
        if _rec["start"] is None:
            _rec["start"] = {n: (np.asarray(sts[n].pos, float).copy(), np.asarray(sts[n].quat_xyzw, float).copy())
                             for n in NAMES}
        m = sts[MIDDLE]
        pts = feeder_points(m)
        gap = rail_gap(pts)
        hand = hand_gap(clouds, m)
        if not _rec["zero"]:
            for n in NEIGHBOURS:
                p0, q0 = _rec["start"][n]
                if np.linalg.norm(np.asarray(sts[n].pos) - p0) > NB_LOOSE_MOVE + EPS or \
                        _turn(q0, sts[n].quat_xyzw) > NB_LOOSE_TURN + EPS:
                    _rec.update(zero=True, why=f"{n} was knocked loose")
            for n in NAMES:
                if np.linalg.norm(sts[n].lin_vel) > DROP_V:
                    _rec.update(zero=True, why=f"{n} was dropped")
                if float(np.asarray(sts[n].pos)[2]) < TOP - 0.02:
                    _rec.update(zero=True, why=f"{n} left the bench")
        rest = (gap >= CLEAR - EPS and hand >= HAND_CLEAR - EPS and flat(m) and on_bench(m, sts[MAT.name]) and
                m.resting(STILL_V, STILL_W) and neighbours_seated(sts))
        if rest and _rec["since"] is None:
            _rec["since"] = t
        if not rest:
            _rec["since"] = None
        _rec.update(known=True, steps=_rec["steps"] + 1, t=t, gap=gap, hand=hand,
                    held=0.0 if _rec["since"] is None else t - _rec["since"])
    except (KeyError, ValueError, TypeError, AttributeError, IndexError) as e:
        _rec.update(known=False, why="unknown observation: " + str(e))


def neighbours_seated(objects) -> bool:
    """Both outer feeders within 2 mm and 3 degrees of where they started."""
    for n in NEIGHBOURS:
        p0, q0 = _rec["start"][n]
        st = objects[n]
        if np.linalg.norm(np.asarray(st.pos, float) - p0) > NB_MOVE + EPS or _turn(q0, st.quat_xyzw) > NB_TURN + EPS:
            return False
    return True


def score(objects):
    """0: unknown observation; at any moment a neighbour knocked loose (moved more than 5 mm or turned more than 10
    degrees) or any feeder dropped or off the bench; at the end, a neighbour more than 2 mm or 3 degrees from its start,
    or the middle feeder within 5 mm of the rail, its feet, slot blocks or pegs.
    0.5: the middle feeder 5 mm or more clear of all of them (held, moving, or not lying on a side).
    0.75: also released (no hand within 5 mm), lying on a side on the bench or mat, still for 1 s.
    1.0: also entirely over the mat."""
    try:
        if not _rec["known"] or _rec["steps"] == 0 or _rec["zero"] or _rec["start"] is None:
            return 0.0
        if not neighbours_seated(objects):
            return 0.0
        m = objects[MIDDLE]
        pts = feeder_points(m)
        if rail_gap(pts) < CLEAR - EPS:
            return 0.0
        value = 0.5
        if _rec["held"] >= HOLD_S - EPS and _rec["hand"] >= HAND_CLEAR - EPS and flat(m) and \
                on_bench(m, objects[MAT.name]) and m.resting(STILL_V, STILL_W):
            value = 0.75
            if on_mat(m, objects[MAT.name]):
                value = 1.0
        return value
    except (KeyError, ValueError, TypeError, AttributeError, IndexError):
        return 0.0


def success(objects):
    return score(objects) == 1.0


@parts.on_reset
def _reset(sim):
    _rec.clear()
    _rec.update(_fresh())
    observation.reset()


@parts.on_step
def _step(sim):
    objects = sim.states()
    try:
        clouds, _ = observation.hand_state(sim)
    except (ValueError, AttributeError, KeyError, IndexError):
        _rec.update(known=False, why="hand state unknown")
        return
    observe(objects, clouds, float(sim.t))
    if os.environ.get("TRACE_OUT"):
        row = dict(t=float(sim.t), lot=lots.current().name, known=_rec["known"], zero=_rec["zero"], why=_rec["why"],
                   gap=_rec["gap"], hand=_rec["hand"], held=_rec["held"], score=score(objects))
        for n in (*NAMES, MAT.name):
            st = objects[n]
            row[n] = dict(pos=np.asarray(st.pos).round(5).tolist(), quat=np.asarray(st.quat_xyzw).round(5).tolist(),
                          v=float(np.linalg.norm(st.lin_vel)), joints={k: float(v) for k, v in st.joints.items()})
        try:  # debugging only: where each hand's TCP is
            from yam_arena.constants import TCP_OFFSET

            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            for arm in ("left_robot", "right_robot"):
                robot = base.scene[arm]
                pose = parts._np(robot.data.body_link_pose_w)[0, robot.body_names.index("gripper")]
                tcp = pose[:3] - origin + parts._quat_to_matrix(pose[3:]) @ np.array(TCP_OFFSET)
                q = parts._np(robot.data.joint_pos)[0]
                row[arm] = dict(tcp=np.round(tcp, 4).tolist(), finger=float(q[robot.joint_names.index("joint7")]))
        except (AttributeError, KeyError, IndexError, ValueError, ImportError):
            pass
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("feeder.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


# -- demo: open loop from the published layout --------------------------------------------------------------------------
def world(u, z, k=0):
    """World point of feeder-frame (u, z) in slot k (-1 front, 0 middle, 1 back)."""
    return (X0 + k * PITCH, Y_R - u, TOP + Z_R + z)


GRIP_YAW = math.pi / 2  # fingers close along world x, across the feeders
PINCH = -0.0020  # finger command: 4.5 mm past each face of the 13 mm reel


def _w(d, p, finger, label, tilt_deg=0.0, heading=0.0, yaw=GRIP_YAW):
    return tilt.Waypoint(d, tuple(float(v) for v in p), finger, label, yaw, tilt=tilt_deg, heading=heading)


def demo():
    lot = lots.current()
    mx, my, _ = lot.pos(MAT.name)
    top = TOP + Z_R + BODY[1][1][1]  # feeder top
    # right hand: one open finger presses the middle feeder's pick end down, the other finger is in free space
    px, py = X0 + 0.050, Y_R - 0.120
    right = [
        _w(2.5, (px, py, TOP + 0.215), FINGER_OPEN, "above the middle pick end"),
        _w(1.5, (px, py, top + 0.0129 + 0.004), FINGER_OPEN, "finger on the pick end"),
        _w(3.0, (px, py, top + 0.0129 - 0.040), FINGER_OPEN, "press it down"),
        _w(4.2, (px, py, top + 0.0129 - 0.040), FINGER_OPEN, "hold it tipped"),
        _w(1.5, (px, py, TOP + 0.215), FINGER_OPEN, "lift away"),
        _w(2.0, (0.20, -0.30, TOP + 0.20), FINGER_OPEN, "aside"),
        _w(20.0, (0.20, -0.30, TOP + 0.20), FINGER_OPEN, "wait"),
    ]
    # left hand: pinch the middle reel where it stands proud, lift the hook out, move it out a little, level it,
    # slide it out along its own plane past the neighbours' reels, then lay it on the mat
    gy = Y_R + 0.127
    level = dict(tilt_deg=20.0, heading=-math.pi / 2)
    out_y = gy + 0.370
    lay = (mx + 0.010, out_y)
    away = tilt.away("left", lay)

    def lw(*a, **k):
        return _w(*a, yaw=-GRIP_YAW, **k)

    left = [
        lw(2.5, (X0, gy, TOP + 0.212), FINGER_OPEN, "above the reels"),
        lw(4.5, (X0, gy, TOP + 0.212), FINGER_OPEN, "wait for the tip"),
        lw(1.5, (X0, gy, TOP + 0.203), FINGER_OPEN, "fingers either side of the proud reel"),
        lw(1.0, (X0, gy, TOP + 0.203), PINCH, "pinch the reel"),
        lw(0.7, (X0, gy, TOP + 0.203), PINCH, "settle"),
        lw(1.0, (X0, gy, TOP + 0.213), PINCH, "lift the hook out"),
        lw(1.5, (X0, gy, TOP + 0.213), PINCH, "hold while the other hand lifts"),
        lw(1.5, (X0, gy + 0.025, TOP + 0.212), PINCH, "out a little"),
        lw(2.0, (X0, gy + 0.025, TOP + 0.211), PINCH, "level it", **level),
        lw(3.0, (X0, gy + 0.170, TOP + 0.211), PINCH, "slide it out", **level),
        lw(3.0, (X0, out_y, TOP + 0.211), PINCH, "clear of the bank"),
        lw(1.5, (*lay, TOP + 0.190), PINCH, "over the mat"),
        lw(2.0, (*lay, TOP + 0.175), PINCH, "lower"),
        lw(2.0, (*lay, TOP + 0.175), PINCH, "lean it", tilt_deg=35.0, heading=away),
        lw(1.0, (*lay, TOP + 0.175), FINGER_OPEN, "let go", tilt_deg=35.0, heading=away),
        lw(1.5, (*lay, TOP + 0.195), FINGER_OPEN, "up", tilt_deg=20.0, heading=away),
        lw(4.0, (*lay, TOP + 0.195), FINGER_OPEN, "wait", tilt_deg=20.0, heading=away),
    ]
    return {"left": left, "right": right}
