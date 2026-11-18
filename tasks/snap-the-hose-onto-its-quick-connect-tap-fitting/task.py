"""On the station table: an outdoor tap on a short grey post, its spout pointing toward the robot's left, with a
brass quick-connect coupler on the spout. The coupler has a round bore with a lead-in chamfer at its mouth, two sprung
steel locking pins inside the bore, and a black sprung sleeve (collar) round its front that slides back toward the
tap. On the table to the robot's left lies the free end of a green garden hose with a brass male hose connector on it.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
import tilt  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BRASS, CHROME, BLACK = (0.80, 0.62, 0.25), (0.72, 0.73, 0.75), (0.10, 0.10, 0.11)
STEEL, POST_GREY, HOSE_GREEN, RED = (0.58, 0.60, 0.63), (0.45, 0.46, 0.48), (0.10, 0.45, 0.20), (0.75, 0.12, 0.10)

# --- the tap and its coupler (fixed). Coupler frame: origin at the centre of the mouth, on the bore's axis; +x runs
# into the bore (toward the tap), z up. In the world the mouth faces the robot's left (+y). ---------------------------
MOUTH = (0.300, -0.010)  # world xy of the mouth's centre
AXIS_H = 0.070  # the bore's axis above the table
YAW_C = -math.pi / 2  # coupler +x -> world -y
BORE_R = 0.0085  # 17.0 mm bore: 0.5 mm all round the 16.0 mm shaft
BORE_X = (0.0027, 0.040)  # behind the lead-in chamfer, to the bore's floor
WALL = 0.004
CHAMFER_R = 0.011  # the chamfer opens the mouth to 22 mm
PIN_X = 0.009  # locking pins' centre line, 9 mm behind the mouth
PIN_R = 0.0025  # each pin's rounded nose: radius 2.5 mm (5 mm wide along the bore)
PIN_IN = 0.0065  # pin nose 6.5 mm from the axis at rest: 2.0 mm into the bore
PIN_OUT = 0.014  # pin's outer end from the axis at rest
PIN_TRAVEL = 0.0025
PIN_H = 0.008
LAND_IN = 0.0143  # the collar's land, 0.3 mm outside the pins' outer ends
LAND_FRONT = 0.0055  # land's front edge behind the mouth, collar forward
RELIEF_IN = 0.0170  # the collar's relief in front of the land: room for the pins to move out
COLLAR_OUT = 0.0195
COLLAR_X = (0.0, 0.032)
COLLAR_TRAVEL = 0.011
COLLAR_SPRING, COLLAR_PRELOAD = 273.0, 0.0044  # N/m; rest point 4.4 mm past the forward stop: 1.2 N to 4.2 N
PIN_SPRING, PIN_PRELOAD = 20.0, 0.002
UNCOVER = PIN_X + PIN_R - LAND_FRONT  # 6.0 mm of collar travel before the land is behind the pins


def _ring12(r_in, depth, x0, x1, color):
    """Twelve slabs round the x axis leaving a 12-sided bore of inscribed radius r_in, from x0 to x1."""
    w = 2 * math.tan(math.pi / 12) * (r_in + depth)
    out = []
    for i in range(12):
        a = i * math.pi / 6
        c = np.array([(x0 + x1) / 2, 0.0, r_in + depth / 2])
        c = parts._rot((1, 0, 0), a) @ c
        out.append(parts.box((x1 - x0, w, depth), at=tuple(float(v) for v in c), color=color, roll=a))
    return out


def _chamfer():
    """A 45-degree lead-in: twelve plates from 22 mm across at the mouth down to the bore."""
    t = 0.002
    run = CHAMFER_R - BORE_R
    length = run * math.sqrt(2)
    mid = np.array([run / 2, 0.0, (CHAMFER_R + BORE_R) / 2])
    centre = mid + np.array([math.sqrt(0.5), 0.0, math.sqrt(0.5)]) * (t / 2)
    w = 2 * math.tan(math.pi / 12) * (CHAMFER_R + t)
    out = []
    for i in range(12):
        a = i * math.pi / 6
        c = parts._rot((1, 0, 0), a) @ centre
        out.append(parts.box((length, w, t), at=tuple(float(v) for v in c), color=BRASS, roll=a, pitch=math.pi / 4))
    return out


def _slab(x0, x1, y0, y1, z0, z1, color, **kw):
    return parts.box((x1 - x0, y1 - y0, z1 - z0), at=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), color=color, **kw)


def _coupler_shapes():
    h = 0.014  # front face: a square plate 28 mm across round a 22 mm opening
    o = CHAMFER_R
    out = [_slab(0.0, 0.003, -h, h, o, h, BRASS), _slab(0.0, 0.003, -h, h, -h, -o, BRASS),
           _slab(0.0, 0.003, -h, -o, -o, o, BRASS), _slab(0.0, 0.003, o, h, -o, o, BRASS)]
    out += _chamfer()
    out += _ring12(BORE_R, WALL, BORE_X[0], BORE_X[1], BRASS)
    out.append(_slab(BORE_X[1], BORE_X[1] + 0.004, -0.013, 0.013, -0.013, 0.013, BRASS))  # the bore's floor
    out.append(parts.cylinder(0.0105, 0.090, at=(0.089, 0.0, 0.0), color=BRASS, axis="x"))  # spout and tap adaptor
    out.append(_slab(0.134, 0.170, -0.020, 0.020, -0.022, 0.026, CHROME))  # tap body
    out.append(parts.cylinder(0.006, 0.020, at=(0.152, 0.0, 0.036), color=CHROME, axis="z"))  # spindle
    out.append(_slab(0.140, 0.188, -0.005, 0.005, 0.044, 0.052, RED))  # tap lever, pointing away from the coupler
    out.append(_slab(0.136, 0.168, -0.016, 0.016, -AXIS_H, -0.022, POST_GREY))  # the post the tap is fixed to
    out.append(_slab(0.120, 0.184, -0.030, 0.030, -AXIS_H, -AXIS_H + 0.006, POST_GREY))  # its foot plate
    return out


def _pin(side):
    s = 1.0 if side > 0 else -1.0
    nose = parts.cylinder(PIN_R, PIN_H, at=(PIN_X, s * (PIN_IN + PIN_R), 0.0), color=STEEL, axis="z")
    body = parts.box((2 * PIN_R, PIN_OUT - PIN_IN - PIN_R, PIN_H),
                     at=(PIN_X, s * (PIN_IN + PIN_R + PIN_OUT) / 2, 0.0), color=STEEL)
    return parts.Slider(f"pin_{'l' if side > 0 else 'r'}", shapes=[nose, body], axis=(0.0, s, 0.0),
                        travel=PIN_TRAVEL, mass=0.003, spring=PIN_SPRING, damping=0.2, max_effort=50)


def _collar():
    """A round sleeve (12-sided): a relief ring in front, the land ring behind, the same outside all along."""
    x0, x1 = COLLAR_X
    out = _ring12(RELIEF_IN, COLLAR_OUT - RELIEF_IN, x0, LAND_FRONT, BLACK)
    out += _ring12(LAND_IN, COLLAR_OUT - LAND_IN, LAND_FRONT, x1, BLACK)
    for s in (1.0, -1.0):  # knurled bands on the two flats the fingers take (drawn only)
        out.append(_slab(x1 - 0.012, x1 - 0.004, *sorted((s * COLLAR_OUT, s * (COLLAR_OUT + 0.0005))),
                         -0.004, 0.004, CHROME, collide=False))
    return parts.Slider("collar", shapes=out, axis=(1.0, 0.0, 0.0), travel=COLLAR_TRAVEL, mass=0.05,
                        spring=COLLAR_SPRING, damping=0.5, max_effort=50, friction=(1.0, 0.9))  # knurled


COUPLER = parts.Part("coupler", pos=(*MOUTH, TOP + AXIS_H), yaw=YAW_C, shapes=_coupler_shapes(),
                     joints=[_collar(), _pin(+1), _pin(-1)], friction=(0.3, 0.25), contact_offset=0.001)

# --- the hose end (loose). Plug frame: origin at the centre of the connector's flange face, +x toward its nose. ----
SHAFT_R, GROOVE_R = 0.0080, 0.0060  # 16.0 mm shaft, 12.0 mm groove core
GROOVE_X = (0.0062, 0.0126)  # 6.4 mm groove from 6.2 to 12.6 mm in front of the flange face: the pins (6.5 to 11.5 mm in) drop in with the flange up to 1 mm short
NOSE_L = 0.024
FLANGE_R, FLANGE_T = 0.0125, 0.006
FERRULE_R, FERRULE_X = 0.0120, (-0.040, -FLANGE_T)
HOSE_R, SEG, N_SEG = 0.0125, 0.060, 8  # 25 mm hose, eight 60 mm sections: the last 48 cm


def _plug_shapes():
    def cyl(r, x0, x1, color=BRASS):
        return parts.cylinder(r, x1 - x0, at=((x0 + x1) / 2, 0.0, 0.0), color=color, axis="x")

    return [cyl(0.0070, 0.0225, NOSE_L), cyl(0.0075, 0.021, 0.0225), cyl(SHAFT_R, GROOVE_X[1], 0.021),
            cyl(GROOVE_R, *GROOVE_X), cyl(SHAFT_R, 0.0, GROOVE_X[0]), cyl(FLANGE_R, -FLANGE_T, 0.0),
            cyl(FERRULE_R, *FERRULE_X)]


def _hose():
    out, parent = [], "base"
    for k in range(N_SEG):
        x0 = FERRULE_X[0] - k * SEG
        out.append(parts.Hinge(
            f"hose{k + 1}", shapes=[parts.cylinder(HOSE_R, SEG - 0.001, at=(x0 - SEG / 2, 0.0, 0.0), color=HOSE_GREEN,
                                                   axis="x")],
            pivot=(x0, 0.0, 0.0), axis=(0.0, 0.0, 1.0) if k % 2 == 0 else (0.0, 1.0, 0.0), lower=-0.5, upper=0.5,
            mass=0.015, spring=0.02, damping=0.005, max_effort=5, parent=parent))
        parent = f"hose{k + 1}"
    return out


PLUG_START = (0.270, 0.150)  # world xy of the flange face's centre at the start
PLUG_YAW = -math.pi / 2 + 0.40  # nose pointing to the robot's right and a little away
PLUG = parts.Part("hose_end", pos=(*PLUG_START, TOP + HOSE_R), yaw=PLUG_YAW, shapes=_plug_shapes(), joints=_hose(),
                  mass=0.12, friction=(0.5, 0.4), contact_offset=0.001)


OBJECTS = [COUPLER, PLUG]
# check.py models the fingers as fully closed; on the 39 mm collar they stay open to its width, outside the bore's wall
DEMO_TOUCHES = ("coupler",)

# --- scoring ----------------------------------------------------------------------------------------------------------
EPS = 1e-6
PAST_PINS = PIN_X + PIN_R + 0.003  # nose 14.5 mm into the bore: past the pins' centre line by its own nose length
HOME = 0.002  # flange face within 2 mm of the mouth (the groove's play: a locked connector rests 0.7 to 1.4 mm out)
COLLAR_HOME = 0.001  # the collar back within 1 mm of its forward stop
HOME_TILT, HOME_OFF = 5.0, 0.0015  # the bore's play lets a seated connector sit up to about 3 degrees askew
PAST_TILT, PAST_OFF = 6.0, 0.002
CLEAR = 0.005
ARM_LINKS = ("link3", "link4", "link5", "gripper", "tip_left", "tip_right")  # every arm link that can reach the coupler
PULL_N = 10.0
RAMP_STEPS = 10  # the pull rises to 10 N over 0.5 s (1 N per control step) ...
HOLD_STEPS = 10  # ... and holds 10 N for 0.5 s
PULL_STEPS = RAMP_STEPS + HOLD_STEPS
SETTLE_STEPS = 10
GIVE_MAX = 0.0015  # the connector may come out at most 1.5 mm under the pull
GIVE_MIN = 0.00002  # and the pull must visibly act (0.02 mm)
ABORT = 0.003  # a connector coming out more than 3 mm is not locked: the pull stops there
STILL_STEPS = 20
DROPPED = 0.02

_C_ROT = parts._rot((0, 0, 1), YAW_C)
_C_POS = np.array([*MOUTH, TOP + AXIS_H])
_AXIS_OUT = _C_ROT @ np.array([-1.0, 0.0, 0.0])  # world direction out of the mouth

# In the simulator an on_step callback measures every step how close both arms (forearm, wrist, gripper and fingers)
# come to the hose end, the collar and the pins. Until it has measured (or when it cannot), "clear" is None and nothing gets full
# credit. Kept in the parts module so near_misses.py, which imports this file separately, can set it offline.
_grip = parts.__dict__.setdefault("_hose_task_grip", {"clear": None})
_pull = parts.__dict__.setdefault("_hose_task_pull", {"phase": "idle"})
_record = parts.__dict__.setdefault("_hose_task_record", {})


def _clear_record():
    _record.clear()
    _record.update(past=False, home=False, prev=None, still=0, still_pose=None)


_clear_record()


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _in_coupler(plug):
    """(flange depth f, radial offset, tilt in degrees) of the hose connector, in the coupler's frame. f is how far
    the flange face is into the mouth (0 home, negative in front of it)."""
    p = _C_ROT.T @ (np.asarray(plug.pos, float) - _C_POS)
    ax = _C_ROT.T @ (_rot(plug.quat_xyzw) @ np.array([1.0, 0.0, 0.0]))
    tilt_deg = math.degrees(math.acos(float(np.clip(ax[0], -1.0, 1.0))))
    return float(p[0]), float(math.hypot(p[1], p[2])), tilt_deg


def _is_home(plug) -> bool:
    f, off, t = _in_coupler(plug)
    return bool(f >= -HOME - EPS and off <= HOME_OFF + EPS and t <= HOME_TILT + EPS)


def _is_past(plug) -> bool:
    f, off, t = _in_coupler(plug)
    return bool(f + NOSE_L >= PAST_PINS - EPS and off <= PAST_OFF + EPS and t <= PAST_TILT + EPS)


def _dropped(objects) -> bool:
    return bool(float(objects["hose_end"].pos[2]) < TOP - DROPPED)


def _pose(plug):
    return np.asarray(plug.pos, float), np.asarray(plug.quat_xyzw, float)


def _same_pose(a, b) -> bool:
    """Within 0.2 mm and 0.2 degrees."""
    return bool(np.linalg.norm(a[0] - b[0]) <= 0.0002 + EPS
                and abs(float(np.dot(a[1], b[1]))) >= math.cos(math.radians(0.2) / 2) - 1e-9)


def _update(objects) -> None:
    """Once per control step: the whole-trial record. Cleared on a scene reset: the hose end back within 0.5 mm of its
    start, at rest, after a jump of over 20 mm in one control step (the simulator's reset also clears it)."""
    plug = objects["hose_end"]
    pos = np.asarray(plug.pos, float)
    prev = _record["prev"]
    if (prev is not None and np.linalg.norm(pos - np.asarray(PLUG.pos)) < 0.0005 and plug.resting()
            and np.linalg.norm(pos - prev) > 0.020):
        _clear_record()
        _idle()
    _record["prev"] = pos
    if _is_past(plug):
        _record["past"] = True
    if _is_home(plug):
        _record["home"] = True


def _count_still(objects) -> None:
    """Once per control step (from score()): consecutive steps with the hose end at rest and its pose unchanged."""
    plug = objects["hose_end"]
    now = _pose(plug)
    was = _record["still_pose"]
    same = was is not None and _same_pose(now, was)
    _record["still"] = _record["still"] + 1 if (same and plug.resting()) else 0
    _record["still_pose"] = now


def _clear(objects):
    """The arms' measured clearance (None until measured). near_misses.py stands in for the measurement offline."""
    if "offline_clear" in _grip:
        _grip["clear"] = _grip["offline_clear"](objects)
    return _grip["clear"]


def _collar_forward(objects) -> bool:
    return bool(objects["coupler"].joints.get("collar", 1.0) <= COLLAR_HOME + EPS)


def _ready(objects) -> bool:
    plug = objects["hose_end"]
    return bool(_grip["clear"] is True and _is_home(plug) and plug.resting() and _collar_forward(objects))


def _idle() -> None:
    """Back to no test (keeping only the offline stand-in, which near_misses.py installs)."""
    keep = {k: v for k, v in _pull.items() if k == "offline_applied"}
    _pull.clear()
    _pull.update(keep, phase="idle")


def _want(k: int) -> float:
    """The pull's force on pull step k (0-based): a ramp to PULL_N, then PULL_N."""
    return PULL_N * min(1.0, (k + 1) / RAMP_STEPS)


def _out(plug, ref) -> float:
    """How far the connector has come out of the bore since ref (m, along the bore)."""
    return float(np.dot(np.asarray(plug.pos, float) - ref[0], _AXIS_OUT))


def _pull_step(objects) -> None:
    """Once per control step: the pull test. It starts once the connector is home, let go and still for 1 s: a pull
    straight out of the bore rising to 10 N over 0.5 s and held at 10 N for 0.5 s, then none for 0.5 s. A connector
    that comes out more than 3 mm fails at once and the pull stops. Anything touching the hose end or the collar
    restarts it."""
    plug = objects["hose_end"]
    p = _pull
    if p["phase"] == "idle":
        if _ready(objects) and _record["still"] >= STILL_STEPS:
            p.update(phase="pull", k=0, ref=_pose(plug), give=0.0, pulled=0, passed=False)
        return
    if p["phase"] in ("pull", "settle"):
        if _grip["clear"] is not True:
            _idle()
            return
        p["give"] = max(p["give"], _out(plug, p["ref"]))
        if p["phase"] == "pull":  # fail closed: count the full-force steps on which 10 N was really applied
            if "offline_applied" in p:  # near_misses.py stands in for the simulator's force offline
                p["applied"] = p["offline_applied"](objects) * _want(p["k"]) / PULL_N
            full = _want(p["k"]) >= PULL_N - 1e-9
            p["pulled"] += int(full and abs(float(p.get("applied") or 0.0) - PULL_N) < 1e-6)
            if p["give"] > ABORT:
                p.update(phase="done", passed=False)
                return
        p["k"] += 1
        if p["phase"] == "pull" and p["k"] >= PULL_STEPS:
            p.update(phase="settle", k=0)
        elif p["phase"] == "settle" and p["k"] >= SETTLE_STEPS:
            ok = (p["pulled"] >= HOLD_STEPS - 1 and GIVE_MIN - EPS <= p["give"] <= GIVE_MAX + EPS
                  and _is_home(plug) and plug.resting() and _collar_forward(objects))
            p.update(phase="done", passed=bool(ok), after=_pose(plug))
        return
    if (not _same_pose(_pose(plug), p.get("after", p["ref"])) or not plug.resting()
            or _grip["clear"] is not True):  # done, then disturbed
        _idle()


def _value(objects) -> float:
    plug = objects["hose_end"]
    if _dropped(objects):
        return 0.0
    if (_pull["phase"] == "done" and _pull["passed"] and _grip["clear"] is True and _is_home(plug)
            and plug.resting() and _collar_forward(objects)):
        return 1.0
    if _record["home"]:
        return 0.5
    if _record["past"]:
        return 0.25
    return 0.0


def success(objects):
    """The hose connector home in the coupler, locked (it passed the pull test), let go and still."""
    _update(objects)
    _clear(objects)
    return bool(_value(objects) == 1.0)


def score(objects):
    _update(objects)
    _count_still(objects)
    _clear(objects)
    _pull_step(objects)
    return _value(objects)


# --- simulator callbacks --------------------------------------------------------------------------------------------
def _boxes(st):
    plug, coup = st["hose_end"], st["coupler"]
    poses = {"base": (np.asarray(plug.pos), _rot(plug.quat_xyzw))}
    poses.update({k: (np.asarray(p), _rot(q)) for k, (p, q) in plug.links.items()})
    out = [(c, r, s) for _, c, r, s in PLUG.world_boxes(poses)]
    cposes = {k: (np.asarray(p), _rot(q)) for k, (p, q) in coup.links.items()}
    out += [(c, r, s) for _, c, r, s in COUPLER.world_boxes(cposes)]
    return out


FACE_STEP = 0.003  # arm hull faces are sampled about every 3 mm


def _face_points(vertices, planes, step=FACE_STEP) -> np.ndarray:
    """Points on every face of a convex hull about `step` apart, corners included: a flat face lying on a part is
    caught even when every corner of it is far away (some YAM hull faces are over 100 mm across)."""
    pts = [np.asarray(vertices, float)]
    _, first = np.unique(np.round(planes / np.array([1e-4, 1e-4, 1e-4, 1e-5])).astype(np.int64), axis=0,
                         return_index=True)  # a flat face is many coplanar triangles: sample each plane once
    for n, d in zip(planes[np.sort(first), :3], planes[np.sort(first), 3]):
        on = vertices[np.abs(vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 = e1 / np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for a, b in zip(ring, np.roll(ring, -1, axis=0)):  # fan of triangles (c, a, b), each sampled on a grid
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c)) / step)))
            for i in range(m + 1):
                for j in range(m + 1 - i):
                    pts.append((c + (a - c) * (i / m) + (b - c) * (j / m))[None, :])
    pts = np.concatenate(pts)
    _, keep = np.unique(np.round(pts / (step / 3)).astype(np.int64), axis=0, return_index=True)  # drop near-repeats
    return np.concatenate([np.asarray(vertices, float), pts[np.sort(keep)]])


_LINK_POINTS: dict = {}


def _link_points(name) -> list:
    """(centre, radius, face sample points) of each collision hull of an arm link, in the link's frame (computed once
    per link)."""
    if name not in _LINK_POINTS:
        from yam_arena import analysis

        out = []
        for h in analysis.link_hulls().get(name, []):
            pts = _face_points(h.vertices, h.planes)
            c = pts.mean(axis=0)
            out.append((c, float(np.linalg.norm(pts - c, axis=1).max()), pts))
        _LINK_POINTS[name] = out
    return _LINK_POINTS[name]


def _arms_clear(points, boxes) -> bool:
    """True only if every given point of the arms' hulls is more than CLEAR from every box."""
    from yam_arena import analysis

    if not len(points):
        return True  # no hull point near any box (the caller checks that the arms were measured)
    pts = np.concatenate(points)
    for c, r, s in boxes:
        reach = float(np.linalg.norm(s)) / 2 + CLEAR + 1e-3  # only points inside the box's bounding sphere can be close
        near = pts[np.einsum("ij,ij->i", pts - c, pts - c) <= reach * reach]
        if len(near) and float(analysis.box_signed_distance(near, c, r, s).min()) <= CLEAR:
            return False
    return True


@parts.on_step
def _measure_grip(sim) -> None:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    boxes = _boxes(sim.states())
    lo = np.min([c - np.linalg.norm(sz) / 2 for c, _, sz in boxes], axis=0) - CLEAR
    hi = np.max([c + np.linalg.norm(sz) / 2 for c, _, sz in boxes], axis=0) + CLEAR
    points, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ARM_LINKS:
                rot, pos = _rot(poses[i, 3:]), poses[i, :3] - origin
                hulls = _link_points(n)
                found += 1 if hulls else 0
                for c, rad, p in hulls:  # a hull whose bounding sphere misses every box's region cannot be close
                    cw = rot @ c + pos
                    if np.all(cw + rad >= lo) and np.all(cw - rad <= hi):
                        points.append(p @ rot.T + pos)
    # fail closed: unless every listed link of both arms was found, the state stays unknown (None)
    _grip["clear"] = _arms_clear(points, boxes) if found == 2 * len(ARM_LINKS) else None


def _set_pull(sim, force: float) -> None:
    import torch

    base = sim.env.unwrapped
    asset = base.scene["hose_end"]
    plug = sim.states()["hose_end"]
    at = np.asarray(plug.pos) + parts._np(base.scene.env_origins[0])
    f = _AXIS_OUT * force
    dev = base.device
    asset.permanent_wrench_composer.set_forces_and_torques_index(
        forces=torch.tensor([[[*f]]], dtype=torch.float32, device=dev),
        positions=torch.tensor([[[*at]]], dtype=torch.float32, device=dev),
        body_ids=[list(asset.body_names).index("base")], env_ids=torch.tensor([0], dtype=torch.int32, device=dev),
        is_global=True)
    _pull["applied"] = force


@parts.on_step
def _apply_pull(sim) -> None:
    """The pull test's force: on while the test is in its pull phase (set each step at the connector), else off."""
    want = _want(int(_pull.get("k", 0))) if _pull.get("phase") == "pull" else 0.0
    if want or _pull.get("applied"):
        _set_pull(sim, want)


@parts.on_step
def _springs(sim) -> None:
    """The collar's and pins' springs are preloaded: their rest points lie past the forward and inner stops."""
    if not _record.get("preloaded"):
        sim.set_target("coupler", "collar", -COLLAR_PRELOAD)
        sim.set_target("coupler", "pin_l", -PIN_PRELOAD)
        sim.set_target("coupler", "pin_r", -PIN_PRELOAD)
        _record["preloaded"] = True


@parts.on_reset
def _reset(sim) -> None:
    _grip["clear"] = None
    _idle()
    _clear_record()
    _set_pull(sim, 0.0)
    _springs(sim)


parts.views((*MOUTH, TOP + AXIS_H))  # close cameras for the videos, off unless HOSE_VIEWS_DIR is set


# --- demo: open loop from the published start layout ------------------------------------------------------------------
Waypoint = tilt.Waypoint
TIP = 0.013  # finger tips below the TCP
PLUG_GRIP_X = -0.025  # grip the ferrule 25 mm behind the flange face
FERRULE_GRIP = -(FERRULE_R - 0.005)  # pads 5 mm inside the ferrule's faces: a firm pinch against the hose's weight
FERRULE_OPEN = -0.030
COLLAR_GRIP = -(COLLAR_OUT - 0.008)  # 8 mm past its faces: a firm pinch (the collar is guided, so it cannot shift)
COLLAR_OPEN = -0.032
COLLAR_GRIP_X = 0.016  # pinch the collar halfway along
SAG = 0.004  # the left arm settles about 4 mm below its target near x 0.30 with a light load (measured)


def _world(p_local, origin, yaw):
    return np.asarray(origin, float) + parts._rot((0, 0, 1), yaw) @ np.asarray(p_local, float)


LEAN = 30.0  # degrees: each arm's finger tips lean away from its own base, keeping the two elbows apart


def _plug_plan():
    # the lean turns about the fingers' closing line, so the connector keeps the same lean in the pads all the way:
    # at the pick the finger tips lean toward its nose, as they do at the bore
    lean = dict(tilt=LEAN, heading=YAW_C)
    pick = dict(tilt=LEAN, heading=PLUG_YAW)
    start = (*PLUG_START, TOP + HOSE_R)
    gx, gy, _ = _world((PLUG_GRIP_X, 0.0, 0.0), start, PLUG_YAW)
    pick_z = TOP + HOSE_R + 0.006
    carry = TOP + AXIS_H + 0.050
    line_z = TOP + AXIS_H + 0.006 + SAG  # plug axis on the bore's axis
    yaw_end = YAW_C  # plug +x into the bore
    # grip point when the flange is d in front of the mouth
    def at(d):
        return _world((-d + PLUG_GRIP_X, 0.0, 0.0), (*MOUTH, 0.0), YAW_C)[:2]
    ax, ay = at(0.040)
    bx, by = at(0.012)  # nose 12 mm into the bore: in the chamfer and past it
    hx, hy = at(-0.008)  # 8 mm past home: a firm push against the flange stop (the arm is soft)
    return [
        Waypoint(2.0, (gx, gy, carry), FERRULE_OPEN, "over the hose connector", PLUG_YAW, **pick),
        Waypoint(1.5, (gx, gy, pick_z), FERRULE_OPEN, "down round the ferrule", PLUG_YAW, **pick),
        Waypoint(0.8, (gx, gy, pick_z), FERRULE_GRIP, "grip the ferrule", PLUG_YAW, **pick),
        Waypoint(1.5, (gx, gy, carry), FERRULE_GRIP, "lift it", PLUG_YAW, **pick),
        Waypoint(2.5, (ax, ay, carry), FERRULE_GRIP, "over the line of the bore, turned to it", yaw_end, **lean),
        Waypoint(1.5, (ax, ay, line_z), FERRULE_GRIP, "down onto the line, 40 mm out", yaw_end, **lean),
        Waypoint(1.0, (ax, ay, line_z), FERRULE_GRIP, "settle", yaw_end, **lean),
        Waypoint(1.5, (bx, by, line_z), FERRULE_GRIP, "nose into the mouth", yaw_end, **lean),
        Waypoint(2.0, (hx, hy, line_z), FERRULE_GRIP, "push home", yaw_end, **lean),
        Waypoint(2.0, (hx, hy, line_z), FERRULE_GRIP, "hold home while the collar springs forward", yaw_end, **lean),
        Waypoint(0.6, (hx, hy, line_z), FERRULE_OPEN, "let go", yaw_end, **lean),
        Waypoint(1.0, (hx, hy, carry + 0.03), FERRULE_OPEN, "straight up", yaw_end, **lean),
        Waypoint(1.5, (hx, hy + 0.10, carry + 0.03), FERRULE_OPEN, "away", yaw_end),
        Waypoint(7.0, (hx, hy + 0.10, carry + 0.03), FERRULE_OPEN, "wait clear (the pull test runs)", yaw_end),
    ]


def _collar_plan(t_home):
    yaw = math.pi / 2  # fingers close along x, on the collar's side faces
    lean = dict(tilt=LEAN, heading=math.pi / 2)
    gx, gy, _ = _world((COLLAR_GRIP_X, 0.0, 0.0), (*MOUTH, 0.0), YAW_C)
    z = TOP + AXIS_H + 0.006
    above = TOP + AXIS_H + 0.070
    back = 0.034  # commanded 34 mm back for the collar's 11 mm stop: the arm is soft (about 190 N/m)
    px, py = _world((COLLAR_GRIP_X + back, 0.0, 0.0), (*MOUTH, 0.0), YAW_C)[:2]
    w = [
        Waypoint(2.0, (gx, gy, above), COLLAR_OPEN, "over the collar", yaw),
        Waypoint(1.5, (gx, gy, z), COLLAR_OPEN, "down round the collar", yaw, **lean),
        Waypoint(0.8, (gx, gy, z), COLLAR_GRIP, "grip the collar", yaw, **lean),
        Waypoint(1.5, (px, py, z), COLLAR_GRIP, "pull it back toward the tap", yaw, **lean),
    ]
    held = sum(x.duration for x in w)
    w.append(Waypoint(max(0.5, t_home - held), (px, py, z), COLLAR_GRIP, "hold it back", yaw, **lean))
    w += [
        Waypoint(0.6, (px, py, z), COLLAR_OPEN, "let it go", yaw, **lean),
        Waypoint(0.8, (gx, gy, z), COLLAR_OPEN, "back off the tap", yaw, **lean),
        Waypoint(1.0, (gx, gy, above), COLLAR_OPEN, "straight up", yaw, **lean),
        Waypoint(1.5, (gx, gy - 0.10, above), COLLAR_OPEN, "away", yaw),
        Waypoint(7.5, (gx, gy - 0.10, above), COLLAR_OPEN, "wait clear (the pull test runs)", yaw),
    ]
    return w


def demo():
    left = _plug_plan()
    t_home = sum(x.duration for x in left[:9]) + 0.5  # the connector pushed home
    return {"left": left, "right": _collar_plan(t_home)}

