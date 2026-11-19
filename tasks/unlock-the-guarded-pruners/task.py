"""On the station table: a pair of bypass pruning shears, locked shut by its stopper, lies flat in a weighted stand.
A rigid guard on the stand covers the blades; the two handles lie on the stand's raised rest, toward the robot.
Where the stand lies and which way it points changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

RED, STEEL, DARK_STEEL, BRASS, STAND, GUARD = ((0.72, 0.08, 0.07), (0.80, 0.81, 0.83), (0.45, 0.46, 0.48),
                                              (0.78, 0.62, 0.25), (0.20, 0.21, 0.23), (0.93, 0.62, 0.10))

# -- the pruners ------------------------------------------------------------------------------------------------------
# Part frame: origin at the pivot bolt, at the underside of the tool, z up; the blades point along +x, the handles
# back along -x. Modelled on the ARS VS-8Z: 200 mm long, 54 mm blade, 220 g. The handle and blade outlines, the
# spring and the stopper's shape are estimates (the maker publishes only length, blade length, mass and pack size).
A_LOCK = math.radians(10.0)  # each handle 10 degrees off the tool's axis when locked shut
R0, R1 = 0.060, 0.146  # the grips run from 60 mm to 146 mm behind the pivot
GRIP_W, T = 0.016, 0.020  # grip width in the tool's plane; thickness of the tool (under the 22 mm pack depth)
NECK_W, LAYER = 0.012, 0.009  # the necks and blades lie in two layers, lower (fixed) and upper (moving), 2 mm apart
BLADE_L, BLADE_W = 0.054, 0.016
BOLT_R, BOLT_PROUD = 0.006, 0.004  # the pivot nut stands 4 mm proud of the upper face
Q_SQUEEZE, Q_OPEN = math.radians(-4.0), math.radians(20.0)  # the moving handle: squeezed stop, natural opening stop
# The return spring: 0.6 N m when locked, 0.4 N m at the opening stop (estimates; about 7 N at the grip)
SPRING_K = 0.2 / Q_OPEN
SPRING_REST = Q_OPEN + 0.4 / SPRING_K
HANDLE_DAMPING = 0.01

R_PIN, PIN_R, PIN_H = 0.125, 0.0035, 0.0045  # the stopper's pin, on top of the moving grip near its end
_um = np.array([-math.cos(A_LOCK), math.sin(A_LOCK)])
_inner = np.array([-math.sin(A_LOCK), -math.cos(A_LOCK)])
PIN_XY = R_PIN * _um + 0.005 * _inner
# The stopper: a steel latch plate pivoted on top of the fixed grip. Its notch (3 mm deep, 0.3 mm play a side) sits
# over the pin from the outside, so the spring pulls the pin into the notch's floor and the plate cannot turn. A
# light spring turns the plate off the pin once the pin has been squeezed out of the notch.
NOTCH_DEPTH, NOTCH_PLAY, WALL, BAR = 0.003, 0.0003, 0.002, 0.004
STOP_PIVOT = (float(PIN_XY[0]), -0.018, T)
STOP_FREE = math.radians(25.0)  # the plate's free stop
STOP_K, STOP_REST = 0.01, math.radians(50.0)  # about 0.009 N m toward free (estimate)
STOP_MASS = 0.015
HOOK_Z0, HOOK_Z1, PLATE_Z0, PLATE_Z1 = T + 0.0005, T + 0.007, T + 0.007, T + 0.010


def _bar(r0, r1, ang, w, z0, z1, color):
    """A shape along a line from the pivot at angle ang (part frame), r0..r1 out, width w, z0..z1. Boxes are
    axis-aligned in the kit, so a sloping bar is laid as short boxes (5 mm steps)."""
    n = max(1, int(math.ceil((r1 - r0) / 0.005)))
    out = []
    u = np.array([math.cos(ang), math.sin(ang)])
    for i in range(n):
        a, b = r0 + (r1 - r0) * i / n, r0 + (r1 - r0) * (i + 1) / n
        c = (a + b) / 2 * u
        ext = np.abs(u) * (b - a) + np.abs(np.array([-u[1], u[0]])) * w
        out.append(parts.box((float(ext[0]), float(ext[1]), z1 - z0), at=(float(c[0]), float(c[1]), (z0 + z1) / 2),
                             color=color))
    return out


FIXED_SHAPES = [
    *_bar(R0, R1, math.pi + A_LOCK, GRIP_W, 0.0, T, RED),  # the fixed handle's grip
    *_bar(0.012, R0, math.pi + A_LOCK, NECK_W, 0.0, LAYER, DARK_STEEL),  # its neck, lower layer
    parts.cylinder(0.011, LAYER, at=(0.0, 0.0, LAYER / 2), color=DARK_STEEL),  # the boss round the pivot
    *_bar(0.008, BLADE_L, 0.0, BLADE_W, 0.0, LAYER, DARK_STEEL),  # the hook (counter) blade
    parts.cylinder(BOLT_R, T + BOLT_PROUD, at=(0.0, 0.0, (T + BOLT_PROUD) / 2), color=BRASS),  # pivot bolt and nut
]
MOVING_SHAPES = [
    *_bar(R0, R1, math.pi - A_LOCK, GRIP_W, 0.0, T, RED),  # the moving handle's grip
    *_bar(0.012, R0, math.pi - A_LOCK, NECK_W, T - LAYER, T, DARK_STEEL),  # its neck, upper layer
    *_bar(0.008, BLADE_L, 0.0, BLADE_W, T - LAYER, T, STEEL),  # the cutting blade
    parts.cylinder(PIN_R, PIN_H, at=(float(PIN_XY[0]), float(PIN_XY[1]), T + PIN_H / 2), color=BRASS),  # the pin
]
_hy = float(PIN_XY[1]) + PIN_R + NOTCH_PLAY  # the notch's floor
_xa, _xb = float(PIN_XY[0]) - PIN_R - NOTCH_PLAY, float(PIN_XY[0]) + PIN_R + NOTCH_PLAY
_hz = (HOOK_Z0 + HOOK_Z1) / 2
_py0, _py1 = STOP_PIVOT[1] - 0.004, _hy + BAR
STOPPER_SHAPES = [
    parts.box((_xb - _xa + 2 * WALL, BAR, HOOK_Z1 - HOOK_Z0), at=((_xa + _xb) / 2, _hy + BAR / 2, _hz), color=STEEL),
    parts.box((WALL, NOTCH_DEPTH, HOOK_Z1 - HOOK_Z0), at=(_xb + WALL / 2, _hy - NOTCH_DEPTH / 2, _hz), color=STEEL),
    parts.box((WALL, NOTCH_DEPTH, HOOK_Z1 - HOOK_Z0), at=(_xa - WALL / 2, _hy - NOTCH_DEPTH / 2, _hz), color=STEEL),
    parts.box((0.010, _py1 - _py0, PLATE_Z1 - PLATE_Z0), at=(STOP_PIVOT[0], (_py0 + _py1) / 2,
                                                             (PLATE_Z0 + PLATE_Z1) / 2), color=STEEL),
    parts.cylinder(0.004, PLATE_Z1 - T, at=(STOP_PIVOT[0], STOP_PIVOT[1], (T + PLATE_Z1) / 2), color=STEEL),
]
HANDLE = parts.Hinge("handle", shapes=MOVING_SHAPES, pivot=(0.0, 0.0, 0.0), axis=(0.0, 0.0, -1.0), lower=Q_SQUEEZE,
                     upper=Q_OPEN, mass=0.085, spring=SPRING_K, damping=HANDLE_DAMPING)
STOPPER = parts.Hinge("stopper", shapes=STOPPER_SHAPES, pivot=STOP_PIVOT, axis=(0.0, 0.0, 1.0), lower=0.0,
                      upper=STOP_FREE, mass=STOP_MASS, spring=STOP_K, damping=0.0005)

# -- the stand --------------------------------------------------------------------------------------------------------
# Stand frame: origin on the table under the pivot. A weighted base plate, a rest 30 mm high under the whole tool, and
# a rigid guard box (roof, both sides and the far end) over the blades' whole sweep, open toward the handles.
REST_H = 0.030
GUARD_X0, GUARD_X1, GUARD_Y, GUARD_WALL, ROOF_Z0, ROOF_T = -0.030, 0.085, 0.045, 0.004, REST_H + T + 0.006, 0.004
# Inside the guard the tool is located, with about 1 mm play: two keeper blocks hang from the roof either side of
# the pivot nut, and two low blocks on the rest flank the fixed blade under the moving blade's sweep.
KEEP_GAP, LOW_H, LOW_GAP = 0.001, 0.008, 0.0015
STAND_SHAPES = [
    parts.box((0.250, 0.150, REST_H), at=(-0.040, 0.020, REST_H / 2), color=STAND),  # the rest
    parts.box((0.115, 2 * GUARD_Y, ROOF_T), at=((GUARD_X0 + GUARD_X1) / 2, 0.0, ROOF_Z0 + ROOF_T / 2), color=GUARD),
    parts.box((0.115, GUARD_WALL, ROOF_Z0 - REST_H), at=((GUARD_X0 + GUARD_X1) / 2, GUARD_Y - GUARD_WALL / 2,
                                                          (ROOF_Z0 + REST_H) / 2), color=GUARD),
    parts.box((0.115, GUARD_WALL, ROOF_Z0 - REST_H), at=((GUARD_X0 + GUARD_X1) / 2, -GUARD_Y + GUARD_WALL / 2,
                                                          (ROOF_Z0 + REST_H) / 2), color=GUARD),
    parts.box((GUARD_WALL, 2 * GUARD_Y, ROOF_Z0 - REST_H), at=(GUARD_X1 - GUARD_WALL / 2, 0.0,
                                                                (ROOF_Z0 + REST_H) / 2), color=GUARD),
    *[parts.box((0.0045, 0.010, ROOF_Z0 - (REST_H + T + 0.0005)),
                at=(sx * (BOLT_R + KEEP_GAP + 0.00225), 0.0, (ROOF_Z0 + REST_H + T + 0.0005) / 2), color=GUARD)
      for sx in (-1, 1)],
    *[parts.box((0.036, 0.0105, LOW_H), at=(0.032, sy * (BLADE_W / 2 + LOW_GAP + 0.00525), REST_H + LOW_H / 2),
                color=GUARD) for sy in (-1, 1)],
]
STAND_MASS = 1.5

LOT_A = ((0.36, -0.03), math.radians(-20.0))
LOT_B = ((0.40, 0.02), math.radians(25.0))
STAND = parts.Part("stand", pos=(*LOT_A[0], TOP), yaw=LOT_A[1], mass=STAND_MASS, shapes=STAND_SHAPES)
PRUNER = parts.Part("pruner", pos=(*LOT_A[0], TOP + REST_H), yaw=LOT_A[1], mass=0.120, shapes=FIXED_SHAPES,
                    joints=[HANDLE, STOPPER], friction=(0.6, 0.5))
OBJECTS = [STAND, PRUNER]

LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"stand": (*LOT_A[0], TOP), "pruner": (*LOT_A[0], TOP + REST_H)},
             yaw={"stand": LOT_A[1], "pruner": LOT_A[1]}),
    lots.Lot("B", place={"stand": (*LOT_B[0], TOP), "pruner": (*LOT_B[0], TOP + REST_H)},
             yaw={"stand": LOT_B[1], "pruner": LOT_B[1]}),
)

# -- scoring constants --------------------------------------------------------------------------------------------------
STOP_OFF = math.radians(15.0)  # the stopper is off: turned 15 degrees or more (its notch then clears the pin)
STOP_ON = math.radians(10.0)  # still on its pin: turned less than 10 degrees
FORCED = math.radians(3.0)  # the handle more than 3 degrees open while the stopper is still on: forced through
OPEN_TOL = math.radians(5.0)  # full credit: within 5 degrees of the opening stop
WIDER = 0.010  # half credit: the handle ends at least 10 mm further apart than when locked
IN_REST_XY, IN_REST_YAW, IN_REST_Z = 0.008, math.radians(5.0), 0.002
STAND_XY, STAND_YAW = 0.008, math.radians(3.0)
LOST_XY, LOST_Z = 0.020, 0.015  # never: the pivot more than 20 mm from its start, or the tool 15 mm off its rest
STILL_V, STILL_W = 0.005, 0.05  # m/s and rad/s
CLEAR = 0.005
FACE_STEP = 0.003
HOLD_S = 2.0
_HAND_LINKS = ("gripper", "tip_left", "tip_right")


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _yaw_of(q) -> float:
    r = _rot(q)
    return math.atan2(r[1, 0], r[0, 0])


def _wrap(a) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def tip_gap(q) -> float:
    """Distance between the two grips' outer end corners (m) with the moving handle at q."""
    a = np.array([math.cos(math.pi + A_LOCK), math.sin(math.pi + A_LOCK)]) * R1
    b_ang = math.pi - A_LOCK - q
    b = np.array([math.cos(b_ang), math.sin(b_ang)]) * R1
    return float(np.linalg.norm(a - b))


LOCKED_GAP = tip_gap(0.0)


# -- the trial record ------------------------------------------------------------------------------------------------
_rec = parts.__dict__.setdefault("pruner_record", {})


def _fresh() -> None:
    _rec.update(seated=False, forced=False, lost=False, gap=None, t=None, since=None, held=False, prev=None, still=False)


if not _rec:
    _fresh()


def forced_through(p) -> bool:
    """Is the handle more than 3 degrees open while the stopper is still on its pin (turned under 10 degrees)?"""
    return bool(p.joints["handle"] > FORCED + EPS and p.joints["stopper"] < STOP_ON - EPS)


def lost(objects) -> bool:
    """Has the tool left its stand's guard: its pivot more than 20 mm from where it started, its underside more than
    15 mm above the rest or below it, or the stand moved more than 20 mm?"""
    lot = lots.current()
    p, s = objects["pruner"], objects["stand"]
    p0, s0 = np.asarray(lot.pos("pruner")), np.asarray(lot.pos("stand"))
    off = np.linalg.norm(np.asarray(p.pos)[:2] - p0[:2])
    low = float(p.low[2]) - (TOP + REST_H)
    return bool(off > LOST_XY + EPS or low > LOST_Z + EPS or low < -LOST_Z - EPS
                or np.linalg.norm(np.asarray(s.pos)[:2] - s0[:2]) > LOST_XY + EPS)


def in_rest(objects) -> bool:
    """The tool lies in its rest: pivot within 8 mm of its start, turned within 5 degrees of it, its underside within
    2 mm of the rest's top; the stand within 8 mm and 3 degrees of its start."""
    lot = lots.current()
    p, s = objects["pruner"], objects["stand"]
    p0, s0 = np.asarray(lot.pos("pruner")), np.asarray(lot.pos("stand"))
    return bool(np.linalg.norm(np.asarray(p.pos)[:2] - p0[:2]) <= IN_REST_XY + EPS
                and abs(_wrap(_yaw_of(p.quat_xyzw) - lot.yaw_of("pruner"))) <= IN_REST_YAW + EPS
                and abs(float(p.low[2]) - (TOP + REST_H)) <= IN_REST_Z + EPS
                and np.linalg.norm(np.asarray(s.pos)[:2] - s0[:2]) <= STAND_XY + EPS
                and abs(_wrap(_yaw_of(s.quat_xyzw) - lot.yaw_of("stand"))) <= STAND_YAW + EPS)


def tier(objects) -> float:
    """The end state's credit, before the release, stillness and whole-trial rules."""
    p = objects["pruner"]
    q, st = float(p.joints["handle"]), float(p.joints["stopper"])
    if st < STOP_OFF - EPS:
        return 0.0
    if q >= Q_OPEN - OPEN_TOL - EPS and in_rest(objects):
        return 1.0
    if tip_gap(q) >= LOCKED_GAP + WIDER - EPS:
        return 0.5
    return 0.0


# -- release distance (gripper hull faces, sampled) ---------------------------------------------------------------
def _face_points(hull, step=FACE_STEP) -> np.ndarray:
    """Points on every face of a convex hull, about `step` apart (corners included)."""
    pts = [hull.vertices]
    for n, d in zip(hull.planes[:, :3], hull.planes[:, 3]):
        on = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for i in range(len(ring)):
            a, b = ring[i], ring[(i + 1) % len(ring)]
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step)))
            for s_ in range(m + 1):
                for t_ in range(m + 1 - s_):
                    pts.append((c + (a - c) * (s_ / m) + (b - c) * (t_ / m))[None, :])
    pts = np.concatenate(pts)
    _, keep = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(keep)]])


_FACES: dict = {}


def _link_faces(name):
    if name not in _FACES:
        from yam_arena import analysis

        _FACES[name] = [(h, _face_points(h)) for h in analysis.link_hulls().get(name, [])]
    return _FACES[name]


def _boxes(objects):
    p, s = objects["pruner"], objects["stand"]
    poses = {"base": (np.asarray(p.pos, float), _rot(p.quat_xyzw))}
    for name, (lp, lq) in getattr(p, "links", {}).items():
        poses[name] = (np.asarray(lp, float), _rot(lq))
    return PRUNER.world_boxes(poses) + STAND.world_boxes({"base": (np.asarray(s.pos, float), _rot(s.quat_xyzw))})


def _gap(objects, hulls) -> float:
    """The closest any point on a gripper hull's faces (sampled about every 3 mm, corners included) comes to any shape
    of the tool or its stand (m; negative inside)."""
    from yam_arena import analysis

    pts = np.concatenate([getattr(h, "face_points", h.vertices) for h in hulls])
    c = np.asarray(objects["pruner"].pos, float)
    near = np.linalg.norm(pts[:, :2] - c[:2], axis=1) < 0.30
    if not near.any():
        return 0.05
    pts = pts[near]
    return float(min(analysis.box_signed_distance(pts, ctr, r, sz).min() for _, ctr, r, sz in _boxes(objects)))


class _WorldHull:
    def __init__(self, hull, faces, m):
        rot, pos = m[:3, :3], m[:3, 3]
        self.vertices = hull.vertices @ rot.T + pos
        n_w = hull.planes[:, :3] @ rot.T
        self.planes = np.c_[n_w, hull.planes[:, 3] + n_w @ pos]
        self.face_points = faces @ rot.T + pos


def _hands(sim):
    """Both arms' gripper and finger tip hulls in the world, or None unless all six hand links were found."""
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    hulls, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                link = [_WorldHull(h, f, m) for h, f in _link_faces(n)]
                found += 1 if link else 0
                hulls += link
    return hulls if found == 2 * len(_HAND_LINKS) else None


# -- stillness ----------------------------------------------------------------------------------------------------------
def _snapshot(objects, t):
    p, s = objects["pruner"], objects["stand"]
    pts = [np.asarray(p.pos, float), np.asarray(s.pos, float)] + [np.asarray(v[0], float) for v in p.links.values()]
    angs = [_yaw_of(p.quat_xyzw), _yaw_of(s.quat_xyzw), float(p.joints["handle"]), float(p.joints["stopper"])]
    tilt = [_rot(p.quat_xyzw)[:, 2], _rot(s.quat_xyzw)[:, 2]]
    return t, pts, angs, tilt


def still_between(a, b) -> bool:
    """Did every part move under 5 mm/s and turn under 0.05 rad/s between two snapshots?"""
    dt = b[0] - a[0]
    if dt <= 0:
        return False
    v = max(np.linalg.norm(x - y) for x, y in zip(a[1], b[1])) / dt
    w = max(abs(_wrap(x - y)) for x, y in zip(a[2], b[2])) / dt
    w = max(w, max(math.acos(float(np.clip(x @ y, -1.0, 1.0))) for x, y in zip(a[3], b[3])) / dt)
    return bool(v < STILL_V - EPS and w < STILL_W - EPS)


def _hold(objects, t) -> None:
    """Count the 2 s hold: credited end state, let go (every gripper part more than 5 mm away) and still at this
    control step, continuously since `since`."""
    ok = _rec["gap"] is not None and _rec["gap"] > CLEAR + EPS and _rec["still"] and tier(objects) > 0.0
    if not ok:
        _rec.update(since=None, held=False)
        return
    if _rec["since"] is None:
        _rec["since"] = t
    _rec["held"] = t - _rec["since"] >= HOLD_S - EPS


@parts.on_reset
def _new_trial(sim) -> None:
    _fresh()
    sim.set_target("pruner", "handle", SPRING_REST)
    sim.set_target("pruner", "stopper", STOP_REST)


def _seat(base) -> None:
    """Lay the stand and the tool exactly at the lot's pose, still."""
    import torch

    lot = lots.current()
    origin = base.scene.env_origins[0][:3]
    for name in ("stand", "pruner"):
        obj = base.scene[name]
        x, y, z = lot.pos(name)
        a = lot.yaw_of(name)
        pose = torch.tensor([[x, y, z, 0.0, 0.0, math.sin(a / 2), math.cos(a / 2)]], dtype=torch.float32,
                            device=base.device)
        pose[:, :3] += origin.to(pose.dtype)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=base.device))


_lots_apply = lots._apply


def _apply_and_seat(emb) -> bool:
    _lots_apply(emb)
    base = emb._base
    _seat(base)
    for _ in range(max(1, round(0.3 / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(lots.RENDERS):
        base.sim.render()
    return True


lots._apply = _apply_and_seat


@parts.on_step
def _step(sim) -> None:
    if not _rec["seated"]:  # a guard: re-lay both at the first step if the reset left either more than 1 mm off
        _rec["seated"] = True
        o = sim.states()
        lot = lots.current()
        if any(np.linalg.norm(np.asarray(o[n].pos)[:2] - np.asarray(lot.pos(n))[:2]) > 0.001 for n in ("stand", "pruner")):
            _seat(sim.env.unwrapped)
    objects = sim.states()
    t = float(sim.t)
    p = objects["pruner"]
    if forced_through(p):
        _rec["forced"] = True
    if lost(objects):
        _rec["lost"] = True
    snap = _snapshot(objects, t)
    _rec["still"] = _rec["prev"] is not None and still_between(_rec["prev"], snap)
    _rec["prev"] = snap
    _rec["t"] = t
    hulls = _hands(sim)
    if hulls is None:  # the arms could not be read: no credit at this step
        _rec.update(gap=None, since=None, held=False)
        return
    _rec["gap"] = _gap(objects, hulls)
    _hold(objects, t)


# -- scoring ---------------------------------------------------------------------------------------------------------


def score(objects):
    """1.0: the stopper off, the handles sprung open to within 5 degrees of their stop, the tool in its rest. 0.5: the
    stopper off and the handle ends at least 10 mm further apart than locked. Either needs both grippers more than
    5 mm from the tool and stand and everything still for 2 s; with no measurement, or if the tool was ever forced
    open past its stopper or taken out of its guard, the score is 0."""
    if _rec["forced"] or _rec["lost"] or not _rec["held"]:
        return 0.0
    if _rec["gap"] is None or _rec["gap"] <= CLEAR + EPS or not _rec["still"]:
        return 0.0
    return tier(objects)


def success(objects):
    """The stopper is off and the handles are sprung fully open, the tool in its rest, let go and still."""
    return score(objects) == 1.0


def distance(objects):
    return float(Q_OPEN - objects["pruner"].joints["handle"])


# -- demo --------------------------------------------------------------------------------------------------------------
# The right arm comes down over the two grips 88 mm behind the pivot, fingers wide, and closes across both grips,
# squeezing the handles together to their stop. It holds the squeeze for a moment, then opens its fingers slowly, so
# the spring opens the handles against the finger, slides over toward the pivot and to the moving handle's side so
# the handle reaches its stop clear of the finger, and lifts away. Uses only the stand's pose as the cameras show it.
GRIP_X = -0.088  # along the tool, behind the pivot
GRIP_Y = -0.00315  # midway between the grips' outer faces at the squeezed stop
F_SQUEEZE = -0.0185  # 2 mm inside the squeezed grips' outer faces on each side
TCP_Z = REST_H + 0.016  # TCP 46 mm up: finger tips 3 mm above the rest
AWAY = (0.015, 0.012)  # then over 15 mm toward the pivot and 12 mm to the moving handle's side


def _world(lot, local):
    (x, y), a = (lot.pos("stand")[0], lot.pos("stand")[1]), lot.yaw_of("stand")
    c, s = math.cos(a), math.sin(a)
    return (x + c * local[0] - s * local[1], y + s * local[0] + c * local[1])


def demo():
    """The right arm squeezes the locked handles together, then lets them spring open and moves away."""
    lot = lots.current()
    yaw = lot.yaw_of("stand")
    g = _world(lot, (GRIP_X, GRIP_Y))
    g2 = _world(lot, (GRIP_X + AWAY[0], GRIP_Y + AWAY[1]))
    z, hover = TOP + TCP_Z, TOP + 0.12
    right = [
        Waypoint(2.0, (*g, hover), FINGER_OPEN, "over the grips", yaw),
        Waypoint(1.5, (*g, z), FINGER_OPEN, "down beside the grips", yaw),
        Waypoint(1.2, (*g, z), F_SQUEEZE, "squeeze the handles", yaw),
        Waypoint(1.0, (*g, z), F_SQUEEZE, "hold the squeeze", yaw),
        Waypoint(3.0, (*g, z), FINGER_OPEN, "open slowly", yaw),
        Waypoint(1.5, (*g2, z), FINGER_OPEN, "let the handle reach its stop", yaw),
        Waypoint(1.5, (*g2, hover), FINGER_OPEN, "lift away", yaw),
        Waypoint(1.5, (0.15, -0.25, TOP + 0.15), FINGER_OPEN, "park", 0.0),
    ]
    left = [Waypoint(2.0, (0.15, 0.25, TOP + 0.15), FINGER_OPEN, "park", 0.0)]
    return {"right": right, "left": left}
