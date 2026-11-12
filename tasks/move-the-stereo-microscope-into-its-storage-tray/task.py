"""On the station table, to one side of the robot: a student stereo microscope standing on its base, side-on to the
robot. A round pillar rises at the back of the base and carries a focus block with two black focus knobs; the white
binocular head sits forward of it over the stage, with the objective underneath. Two black eyepieces stand in the
two eyepiece tubes on top of the head and are the highest parts. Between the arms is a fixed grey storage tray with
a low rim. Which side the microscope stands on changes between trials.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run before the lots move objects)
import lots  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WHITE, DARK, BLACK = (0.92, 0.92, 0.90), (0.22, 0.23, 0.26), (0.06, 0.06, 0.07)
SILVER, STAGE, TRAY = (0.74, 0.75, 0.78), (0.97, 0.97, 0.97), (0.42, 0.47, 0.55)

# Microscope body. Part frame: origin at the centre of the base's underside, +x toward the microscope's front (where
# the user sits), y across, z up. Sizes in metres.
BASE = (0.135, 0.100, 0.025)
POST_X, POST_R, POST_TOP = -0.048, 0.0125, 0.190  # round pillar, 25 mm across, 25 to 190 mm
BLOCK_X, BLOCK_Z = (-0.066, -0.028), (0.100, 0.160)  # focus block on the pillar, 48 mm wide
HEAD_X, HEAD_Z, HEAD_W = (-0.028, 0.050), (0.115, 0.160), 0.090  # binocular head
EYE_X, EYE_Y = 0.020, 0.030  # eyepiece tube axes, 60 mm apart
TUBE_Z = (0.160, 0.177)  # eyepiece tubes on the head
BORE = 0.0131  # half-width of each tube's bore: 26.2 mm across
WALL = 0.0035
BODY_MASS = 0.56

# Eyepiece: own frame origin at the bottom of the barrel's axis. A round 23.2 mm barrel 16 mm long and a 30 mm eyecup
# 30 mm tall whose underside rests on the tube's rim; seated, the barrel's bottom is 161 mm up, 1 mm above the head.
BARREL_R, BARREL_L = 0.0116, 0.016
CUP_R, CUP_L = 0.015, 0.030
EYE_SEAT_Z = TUBE_Z[1] - BARREL_L  # 0.161
EYE_TOP = EYE_SEAT_Z + BARREL_L + CUP_L  # 0.207
EYE_MASS = 0.04


def _tube(cx, cy):
    """An eyepiece tube: four walls and four corner blocks, so the bore touches a round barrel at eight points on a
    circle 26.2 mm across."""
    h, zc = TUBE_Z[1] - TUBE_Z[0], sum(TUBE_Z) / 2
    out = []
    for s in (-1, 1):
        out.append(parts.box((WALL, 2 * (BORE + WALL), h), at=(cx + s * (BORE + WALL / 2), cy, zc), color=SILVER))
        out.append(parts.box((2 * BORE, WALL, h), at=(cx, cy + s * (BORE + WALL / 2), zc), color=SILVER))
    d = BORE / math.sqrt(2)
    c = BORE - d
    for sx in (-1, 1):
        for sy in (-1, 1):
            out.append(parts.box((c, c, h), at=(cx + sx * (d + c / 2), cy + sy * (d + c / 2), zc), color=SILVER))
    return out


BODY_SHAPES = [
    parts.box(BASE, at=(0.0, 0.0, BASE[2] / 2), color=DARK),  # base
    parts.cylinder(0.030, 0.002, at=(EYE_X, 0.0, BASE[2] + 0.001), color=STAGE, axis="z"),  # stage plate
    parts.cylinder(POST_R, POST_TOP - BASE[2], at=(POST_X, 0.0, (POST_TOP + BASE[2]) / 2), color=SILVER,
                   axis="z"),  # pillar
    parts.box((BLOCK_X[1] - BLOCK_X[0], 0.048, BLOCK_Z[1] - BLOCK_Z[0]),
              at=(sum(BLOCK_X) / 2, 0.0, sum(BLOCK_Z) / 2), color=WHITE),  # focus block
    parts.cylinder(0.016, 0.012, at=(-0.045, 0.030, 0.130), color=BLACK, axis="y"),  # focus knobs
    parts.cylinder(0.016, 0.012, at=(-0.045, -0.030, 0.130), color=BLACK, axis="y"),
    parts.box((HEAD_X[1] - HEAD_X[0], HEAD_W, HEAD_Z[1] - HEAD_Z[0]), at=(sum(HEAD_X) / 2, 0.0, sum(HEAD_Z) / 2),
              color=WHITE),  # binocular head
    parts.cylinder(0.016, 0.025, at=(EYE_X, 0.0, HEAD_Z[0] - 0.0125), color=BLACK, axis="z"),  # objective
    *_tube(EYE_X, EYE_Y),
    *_tube(EYE_X, -EYE_Y),
]
EYE_SHAPES = [
    parts.cylinder(BARREL_R, BARREL_L, at=(0.0, 0.0, BARREL_L / 2), color=BLACK, axis="z"),
    parts.cylinder(CUP_R, CUP_L, at=(0.0, 0.0, BARREL_L + CUP_L / 2), color=BLACK, axis="z"),
]

# Layout. Lot A (the screenshot): the microscope to the robot's left, its pillar toward the robot's left (part +x is
# world -y). Lot B mirrors it to the robot's right, pillar toward the robot's right.
SCOPE_A, YAW_A = (0.21, 0.23), -math.pi / 2
SCOPE_B, YAW_B = (0.21, -0.23), math.pi / 2
TRAY_XY = (0.21, 0.0)
EYES = ("eyepiece_1", "eyepiece_2")
EYE_LOCAL = {"eyepiece_1": (EYE_X, EYE_Y), "eyepiece_2": (EYE_X, -EYE_Y)}


def _rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def at_scope(xy, yaw, local):
    """World point of a microscope-frame point, for the microscope's origin at (x, y, TOP) turned `yaw`."""
    p = np.array([xy[0], xy[1], TOP]) + _rz(yaw) @ np.asarray(local, float)
    return tuple(float(v) for v in p)


def _eye_start(name, xy, yaw):
    return at_scope(xy, yaw, (*EYE_LOCAL[name], EYE_SEAT_Z))


SCOPE = parts.Part("microscope", pos=(*SCOPE_A, TOP), mass=BODY_MASS, yaw=YAW_A, shapes=BODY_SHAPES)
EYEPIECES = [parts.Part(n, pos=_eye_start(n, SCOPE_A, YAW_A), mass=EYE_MASS, shapes=EYE_SHAPES) for n in EYES]

# Storage tray (fixed): inner 150 mm (x) by 175 mm (y), floor 4 mm thick, rim 8 mm above the floor, walls 8 mm.
TRAY_IN = (0.150, 0.175)
FLOOR_T, RIM_H, TRAY_WALL = 0.004, 0.008, 0.008
FLOOR_TOP = TOP + FLOOR_T
_TX, _TY = TRAY_IN[0] + 2 * TRAY_WALL, TRAY_IN[1] + 2 * TRAY_WALL
TRAY_PART = parts.Part("storage_tray", pos=(*TRAY_XY, TOP), shapes=[
    parts.box((_TX, _TY, FLOOR_T), at=(0.0, 0.0, FLOOR_T / 2), color=TRAY),
    parts.box((TRAY_WALL, _TY, RIM_H), at=((TRAY_IN[0] + TRAY_WALL) / 2, 0.0, FLOOR_T + RIM_H / 2), color=TRAY),
    parts.box((TRAY_WALL, _TY, RIM_H), at=(-(TRAY_IN[0] + TRAY_WALL) / 2, 0.0, FLOOR_T + RIM_H / 2), color=TRAY),
    parts.box((TRAY_IN[0], TRAY_WALL, RIM_H), at=(0.0, (TRAY_IN[1] + TRAY_WALL) / 2, FLOOR_T + RIM_H / 2), color=TRAY),
    parts.box((TRAY_IN[0], TRAY_WALL, RIM_H), at=(0.0, -(TRAY_IN[1] + TRAY_WALL) / 2, FLOOR_T + RIM_H / 2), color=TRAY),
])

OBJECTS = [SCOPE, *EYEPIECES, TRAY_PART]

_ZERO = (0.0, 0.0, 0.0)
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"microscope": SCOPE.pos}),  # a zero move, so lot A also gets the settle and fresh frame
    lots.Lot("B", place={"microscope": (*SCOPE_B, TOP), **{n: _eye_start(n, SCOPE_B, YAW_B) for n in EYES}},
             yaw={"microscope": YAW_B}),
)

# -- scoring --------------------------------------------------------------------------------------------------------
EPS = 1e-6
EYE_OUT = 0.005  # an eyepiece may rise at most 5 mm out of its tube at any point
EYE_SEATED = 0.001  # at the end: within 1 mm of fully down
EYE_SIDE = 0.003  # an eyepiece's barrel axis more than 3 mm off its tube's axis is out of the tube
MAX_TILT = math.radians(10.0)  # the microscope, at any point
FLAT = math.radians(3.0)  # at the end
FALL_SPEED = 0.6  # m/s downward, at any point
ON_FLOOR = 0.001  # the base's lowest corner at most 1 mm above the tray floor (and at most 2 mm below: contact)
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the microscope or an eyepiece
HOLD = 1.0
SUCCESS_HOLD_S = HOLD


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(state) -> float:
    return math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0)))


def eye_offset(objects, name) -> tuple[float, float]:
    """(rise above seated, sideways distance from the tube's axis) of an eyepiece, in the microscope's frame (m)."""
    scope, eye = objects["microscope"], objects[name]
    local = _rot(scope.quat_xyzw).T @ (np.asarray(eye.pos, float) - np.asarray(scope.pos, float))
    ex, ey = EYE_LOCAL[name]
    return float(local[2] - EYE_SEAT_Z), float(math.hypot(local[0] - ex, local[1] - ey))


def _base_corners(scope) -> np.ndarray:
    r = _rot(scope.quat_xyzw)
    return np.array([scope.pos + r @ np.array([sx * BASE[0] / 2, sy * BASE[1] / 2, 0.0])
                     for sx in (-1, 1) for sy in (-1, 1)])


def in_tray(scope) -> bool:
    """The base flat on the tray floor, wholly inside the rim."""
    c = _base_corners(scope)
    inside = all(abs(p[0] - TRAY_XY[0]) <= TRAY_IN[0] / 2 + EPS and abs(p[1] - TRAY_XY[1]) <= TRAY_IN[1] / 2 + EPS
                 for p in c)
    bottom = float(c[:, 2].min())
    return bool(inside and _tilt(scope) <= FLAT + EPS and FLOOR_TOP - 0.002 <= bottom <= FLOOR_TOP + ON_FLOOR + EPS)


# Whole-trial record: did an eyepiece ever rise over 5 mm or leave its tube, the microscope tilt over 10 degrees, or
# drop faster than 0.6 m/s? Cleared at a scene reset (the on_reset hook in the simulator; offline, the microscope back
# within 0.5 mm of its start and at rest after a jump of over 20 mm in one control step, which a robot cannot do).
# Also recorded: was the microscope ever lifted with its base more than 15 mm clear of the table and both eyepieces
# seated (the 0.5 tier)?
_record = {"fault": False, "prev": None, "lifted": False}
# Release and hold, measured in the simulator every step. Until the grippers have been measured nothing is credited
# (fail closed). Offline (near_misses.py sets OFFLINE_FLAG) each case stands for a state held 1 s with both grippers
# more than 5 mm away. The offline path is never used once a simulator has run a callback in this process ("live").
_sim = {"measured": False, "clear": False, "since": None, "t": 0.0, "live": False}
OFFLINE_FLAG = "STEREO_MICROSCOPE_TASK_OFFLINE_CHECK"
LIFTED = 0.015  # the base's lowest corner more than 15 mm above the table (the tray's rim is 12 mm up)


def _offline() -> bool:
    return os.environ.get(OFFLINE_FLAG) == "1" and not _sim["live"] and not _sim["measured"]


def _faulted(objects) -> bool:
    scope = objects["microscope"]
    if _tilt(scope) > MAX_TILT + EPS or float(scope.lin_vel[2]) < -FALL_SPEED - EPS:
        return True
    for n in EYES:
        rise, side = eye_offset(objects, n)
        if rise > EYE_OUT + EPS or side > EYE_SIDE + EPS:
            return True
    return False


def _update(objects) -> None:
    scope = objects["microscope"]
    pos = np.asarray(scope.pos, float)
    prev = _record["prev"]
    start = np.asarray(lots.current().pos("microscope"), float)
    if prev is not None and np.linalg.norm(pos - start) < 0.0005 and scope.resting():
        if np.linalg.norm(pos - prev) > 0.020:
            _record["fault"] = False
            _record["lifted"] = False
    _record["prev"] = pos
    if _faulted(objects):
        _record["fault"] = True
    elif float(_base_corners(scope)[:, 2].min()) > TOP + LIFTED + EPS and _eyes_seated(objects):
        _record["lifted"] = True


def _eyes_seated(objects, still=False) -> bool:
    for n in EYES:
        rise, side = eye_offset(objects, n)
        if not (rise <= EYE_SEATED + EPS and side <= EYE_SIDE + EPS and (objects[n].resting() or not still)):
            return False
    return True


def placed(objects) -> bool:
    """In the tray, both eyepieces seated, everything still."""
    if not in_tray(objects["microscope"]) or not objects["microscope"].resting():
        return False
    return _eyes_seated(objects, still=True)


def _clear() -> bool:
    return True if _offline() else bool(_sim["measured"] and _sim["clear"])


def _done(objects) -> bool:
    _update(objects)
    return bool(not _record["fault"] and _clear() and placed(objects))


def _half(objects) -> bool:
    """0.5: lifted clear earlier with the eyepieces in, and now standing on the table (outside the tray), upright and
    still, let go, eyepieces seated."""
    scope = objects["microscope"]
    bottom = float(_base_corners(scope)[:, 2].min())
    on_surface = TOP - 0.002 <= bottom <= TOP + ON_FLOOR + EPS
    return bool(not _record["fault"] and _record["lifted"] and _clear() and on_surface and _tilt(scope) <= FLAT + EPS
                and scope.resting() and _eyes_seated(objects, still=True))


def success(objects):
    """The microscope stands flat in the tray with both eyepieces seated, let go and still; no eyepiece ever pulled
    out, and the microscope never tipped or dropped."""
    return _done(objects)


def distance(objects):
    return float(np.linalg.norm(np.asarray(objects["microscope"].pos[:2]) - np.array(TRAY_XY)))


def score(objects):
    """1.0 once the success state has held for 1 s; 0.5 for a microscope lifted clear with its eyepieces in and set
    down upright elsewhere, let go; otherwise 0."""
    if _done(objects):
        if _offline():
            return 1.0
        since = _sim["since"]
        if since is not None and _sim["t"] - since >= HOLD - 1e-6:
            return 1.0
    return 0.5 if _half(objects) else 0.0


def _boxes(st):
    out = []
    for name, spec in ((n, s) for n, s in zip(("microscope", *EYES), (SCOPE, *EYEPIECES))):
        s = st[name]
        out += [(c, r, size) for _, c, r, size in spec.world_boxes({"base": (np.asarray(s.pos), _rot(s.quat_xyzw))})]
    return out


@parts.on_reset
def _new_trial(sim) -> None:
    _sim.update(measured=False, clear=False, since=None, t=0.0, live=True)
    _record.update(fault=False, prev=None, lifted=False)
    _seen["start"] = None


@parts.on_step
def _measure(sim) -> None:
    from yam_arena import analysis

    st = sim.states()
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right", "gripper"):
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    _sim["clear"] = bool(hulls) and all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in _boxes(st))
    _sim["measured"] = True
    _sim["live"] = True
    _sim["t"] = float(sim.t)
    if _seen["start"] is None:
        _seen["start"] = _spot_of(st["microscope"])
    if _sim["clear"] and not _record["fault"] and placed(st):
        if _sim["since"] is None:
            _sim["since"] = float(sim.t)
    else:
        _sim["since"] = None


# -- demo: the arm on the microscope's side --------------------------------------------------------------------------
# The demo does not know the lot. At the trial's first control step a callback reads where the microscope stands (a
# coarse, camera-visible pose) and matches it to the nearest published start spot below; each waypoint's position is
# worked out from that spot when the plan reaches it (before that first reading, for one control step, the
# screenshot's spot is assumed). Offline (no simulator) the screenshot's spot is used.
START_SPOTS = ((SCOPE_A, YAW_A), (SCOPE_B, YAW_B))  # the published start spots and headings (task.json setup)
_seen = {"start": None}


def _spot_of(scope):
    xy = np.asarray(scope.pos[:2], float)
    return min(START_SPOTS, key=lambda spot: float(np.linalg.norm(xy - np.asarray(spot[0]))))


TIP = 0.013  # finger tips below the TCP
GRIP_Z = 0.166 + TIP  # TCP above the base's underside: tips 166 mm up, 6 mm above the focus block
LIFT = 0.030  # commanded: base 30 mm above the table while carried (the rim is 12 mm up; the loaded arm sags ~9 mm)
RELEASE = 0.007  # commanded: base 7 mm above the floor when the fingers open (the loaded arm sags a few mm)
PINCH = -0.004  # pads commanded 8 mm apart on the 25 mm pillar: a firm pinch against its tipping moment
WIDE = -0.021  # pads 42 mm apart
ABOVE = 0.215  # TCP height for the approach (tips 202 mm up, beside the pillar's top)
UP = 0.200  # first the open fingers rise straight up along the pillar (tips 187 mm up)
OUT = 0.210  # then back toward the robot, the pillar passing between the open pads
LAG = 0.012  # the loaded arm stops about 15 mm short of a sideways carry's end, so aim 12 mm further
BACK = -0.07


class SeenWaypoint(Waypoint):
    """A Waypoint whose position is worked out when the plan reaches it, from the microscope's seen start spot."""

    def __init__(self, duration, where, finger, label=""):
        object.__setattr__(self, "_where", where)
        super().__init__(duration, (0.0, 0.0, 0.0), finger, label)

    @property
    def pos(self):
        return self._where()

    @pos.setter
    def pos(self, value):
        pass


def _park(arm):
    """Where the arm that is not needed waits: on its own side, well clear of the microscope and the tray."""
    return (0.15, 0.22 if arm == "left" else -0.22, TOP + 0.18)


def _route(arm):
    """(x, y, z) points for each step of the carry, for `arm`, or its resting point if the other arm does the job."""
    (sx, sy), yaw = _seen["start"] or START_SPOTS[0]
    doer = "left" if sy > 0 else "right"
    if arm != doer:
        park = _park(arm)
        return {k: park for k in ("above", "grip", "lift", "carry", "put", "up", "out")}
    gx, gy, _ = at_scope((sx, sy), yaw, (POST_X, 0.0, 0.0))
    tx, ty, _ = at_scope(TRAY_XY, yaw, (POST_X, 0.0, 0.0))
    ty += math.copysign(LAG, ty - gy)  # further along the carry
    put = GRIP_Z + FLOOR_T + RELEASE
    return {
        "above": (gx, gy, TOP + ABOVE), "grip": (gx, gy, TOP + GRIP_Z), "lift": (gx, gy, TOP + GRIP_Z + LIFT),
        "carry": (tx, ty, TOP + GRIP_Z + LIFT), "put": (tx, ty, TOP + put), "up": (tx, ty, TOP + UP),
        "out": (tx + BACK, ty, TOP + OUT),
    }


def demo():
    """The arm on the microscope's side comes straight down on the pillar above the focus block, a finger on its front
    and back; lifts it, carries it sideways over the tray, lowers it and lets go, then rises. The other arm stays
    at its waiting point. Where the microscope stands is read from the scene at the first control step."""
    plans = {}
    for arm in ("left", "right"):
        def at(key, arm=arm):
            return lambda: _route(arm)[key]

        plans[arm] = [
            SeenWaypoint(1.5, at("above"), WIDE, "over the pillar"),
            SeenWaypoint(1.5, at("grip"), WIDE, "down beside the pillar, above the focus block"),
            SeenWaypoint(0.8, at("grip"), PINCH, "grip the pillar"),
            SeenWaypoint(1.5, at("lift"), PINCH, "lift"),
            SeenWaypoint(3.0, at("carry"), PINCH, "carry over the tray"),
            SeenWaypoint(1.5, at("put"), PINCH, "lower into the tray"),
            SeenWaypoint(0.8, at("put"), WIDE, "release"),
            SeenWaypoint(1.2, at("up"), WIDE, "straight up"),
            SeenWaypoint(1.0, at("out"), WIDE, "back toward the robot"),
            SeenWaypoint(2.0, at("out"), WIDE, "wait"),
        ]
    return plans
