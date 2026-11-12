"""On the station table, an orange silicone craft mat. On it lies a blue corded mini hot-glue gun (no cord shown), its nozzle
pointing to the robot's right: a handle, a housing over it with a glue stick in the back and a brass nozzle at the
front, and a silver wire stand hinged under the front of the housing, folded back under it. The gun has tipped
forward and rests on the front edge of its handle's base and on its nozzle tip.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BLUE, BRASS, STICK, SILVER, ORANGE, DARK = (
    (0.12, 0.30, 0.62), (0.80, 0.62, 0.28), (0.93, 0.92, 0.84), (0.78, 0.79, 0.82), (0.88, 0.45, 0.18), (0.10, 0.10, 0.11))

# Silicone mat, fixed: 200 x 300 x 3 mm.
MAT_T = 0.003
MAT_XY, MAT_SIZE = (0.30, 0.07), (0.20, 0.30)
MAT = Box("craft_mat", pos=(MAT_XY[0], MAT_XY[1], TOP + MAT_T / 2), size=(*MAT_SIZE, MAT_T), color=ORANGE)
MAT_TOP = TOP + MAT_T

# The gun, in its own frame (m): origin at the centre of the handle's base, x toward the nozzle, z up the handle,
# y across its thickness. It is placed turned so that x points to the robot's right (world -y).
HANDLE = (0.028, 0.026, 0.086)  # x 14 mm either side of the origin
HOUSING_X, HOUSING_Z = (-0.015, 0.105), (0.085, 0.115)  # 120 x 30 x 30 mm
NOZZLE_R, NOZZLE_X, NOZZLE_Z = 0.005, (0.105, 0.125), 0.098
TIP_R, TIP_X = 0.0025, (0.125, 0.133)
STICK_R, STICK_X = 0.0035, (-0.055, -0.015)  # 7 mm glue stick, 40 mm showing out of the back
HEEL_X = HANDLE[0] / 2  # the front edge of the handle's base
NOZZLE_TIP = np.array([TIP_X[1], 0.0, NOZZLE_Z])

# Stand: a U of 4 mm wire, 60 mm across (bars at y = +-28 mm), 70 mm long, on a pivot rod under the front of the
# housing. Folded (0) it lies back close under the housing; it swings down and forward to a stop at 135 degrees.
PIVOT = (0.095, 0.0, 0.082)
STAND_W, STAND_L, WIRE = 0.028, 0.070, 0.004
STAND_OPEN = math.radians(135.0)
DETENT_MID = math.radians(20.0)  # the detent's crest: below it the stand clicks folded, above it open
DETENT_K = 0.08  # N m/rad
STAND_MASS = 0.02
GUN_MASS = 0.23

_px, _, _pz = PIVOT
STAND_SHAPES = [
    parts.box((STAND_L, WIRE, WIRE), at=(_px - STAND_L / 2, STAND_W, _pz), color=SILVER),
    parts.box((STAND_L, WIRE, WIRE), at=(_px - STAND_L / 2, -STAND_W, _pz), color=SILVER),
    parts.box((WIRE, 2 * STAND_W + WIRE, WIRE), at=(_px - STAND_L + WIRE / 2, 0.0, _pz), color=SILVER),
]
BODY_SHAPES = [
    parts.box(HANDLE, at=(0.0, 0.0, HANDLE[2] / 2), color=BLUE),
    parts.box((HOUSING_X[1] - HOUSING_X[0], 0.030, HOUSING_Z[1] - HOUSING_Z[0]),
              at=(sum(HOUSING_X) / 2, 0.0, sum(HOUSING_Z) / 2), color=BLUE),
    parts.box((0.008, 0.012, 0.025), at=(0.018, 0.0, 0.0725), color=DARK),  # feed trigger
    parts.cylinder(NOZZLE_R, NOZZLE_X[1] - NOZZLE_X[0], at=(sum(NOZZLE_X) / 2, 0.0, NOZZLE_Z), color=BRASS, axis="x"),
    parts.cylinder(TIP_R, TIP_X[1] - TIP_X[0], at=(sum(TIP_X) / 2, 0.0, NOZZLE_Z), color=BRASS, axis="x"),
    parts.cylinder(STICK_R, STICK_X[1] - STICK_X[0], at=(sum(STICK_X) / 2, 0.0, 0.100), color=STICK, axis="x"),
    parts.cylinder(0.003, 2 * STAND_W + 0.006, at=PIVOT, color=SILVER, axis="y"),  # pivot rod
]

GUN_XY = (0.30, 0.13)
GUN_YAW = -math.pi / 2
GUN = parts.Part(
    "glue_gun",
    pos=(*GUN_XY, MAT_TOP),
    mass=GUN_MASS,
    yaw=GUN_YAW,
    shapes=BODY_SHAPES,
    joints=[parts.Hinge("stand", shapes=STAND_SHAPES, pivot=PIVOT, axis=(0.0, -1.0, 0.0), lower=0.0,
                        upper=STAND_OPEN, mass=STAND_MASS, spring=DETENT_K, damping=0.01)],
)
OBJECTS = [MAT, GUN]

EPS = 1e-6
NOZZLE_UP = 0.040  # the nozzle tip at least 40 mm above the mat
UPRIGHT = math.radians(30.0)  # the handle within 30 degrees of vertical
ON_MAT = 0.003  # the gun's lowest point within 3 mm of the mat
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the gun or its stand
HOLD = 1.0


# -- geometry -----------------------------------------------------------------------------------------------------


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _shape_volume(s) -> float:
    if s.kind == "box":
        return float(np.prod(s.size))
    r, h = s.size[0], s.size[1]
    return math.pi * r * r * h


def _boxes(gun):
    """(centre, rotation, size) of every shape of the gun and its stand, in the world."""
    poses = {"base": (np.asarray(gun.pos, float), _rot(gun.quat_xyzw))}
    if "stand" in gun.links:
        lp, lq = gun.links["stand"]
        poses["stand"] = (np.asarray(lp, float), _rot(lq))
    return GUN.world_boxes(poses)


def _corners(boxes) -> np.ndarray:
    signs = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], float)
    return np.concatenate([c + (signs * (s / 2)) @ r.T for _, c, r, s in boxes])


def centre_of_mass(gun) -> np.ndarray:
    """World centre of mass: the body's mass spread evenly over its shapes, plus the stand's."""
    boxes = _boxes(gun)
    body = [(c, _shape_volume(sh)) for (link, c, _, _), sh in zip(boxes, _shapes_in_order(gun)) if link == "base"]
    vol = sum(v for _, v in body)
    com = sum(GUN_MASS * v / vol * c for c, v in body)
    stand = [c for link, c, _, _ in boxes if link == "stand"]
    com = com + STAND_MASS * np.mean(stand, axis=0)
    return com / (GUN_MASS + STAND_MASS)


def _shapes_in_order(gun):
    out = list(BODY_SHAPES)
    if "stand" in gun.links:
        out += STAND_SHAPES
    return out


def _cross2(a, b) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _hull2(pts: np.ndarray) -> np.ndarray:
    pts = np.unique(np.round(pts, 6), axis=0)
    if len(pts) < 3:
        return pts
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def half(ps):
        h = []
        for p in ps:
            while len(h) >= 2 and _cross2(h[-1] - h[-2], p - h[-2]) <= 0:
                h.pop()
            h.append(p)
        return h

    lo, hi = half(pts), half(pts[::-1])
    return np.array(lo[:-1] + hi[:-1])


def _inside(p, hull) -> bool:
    if len(hull) < 3:
        return False
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        if _cross2(b - a, p - a) < -EPS:
            return False
    return True


def nozzle_height(gun) -> float:
    r = _rot(gun.quat_xyzw)
    return float((np.asarray(gun.pos) + r @ NOZZLE_TIP)[2] - MAT_TOP)


def handle_lean(gun) -> float:
    return math.acos(float(np.clip(_rot(gun.quat_xyzw)[2, 2], -1.0, 1.0)))


def standing(gun) -> bool:
    """Standing on its handle on the mat, nozzle up and clear, and able to stand there: its centre of mass is over
    the area spanned by whatever touches the mat (any part of the gun or the stand)."""
    pts = _corners(_boxes(gun))
    low = float(pts[:, 2].min())
    if low < MAT_TOP - ON_MAT - EPS or low > MAT_TOP + ON_MAT + EPS:
        return False
    feet = pts[pts[:, 2] <= low + 0.002][:, :2]
    mx, my = MAT_XY
    if np.any(np.abs(feet[:, 0] - mx) > MAT_SIZE[0] / 2 + EPS) or np.any(np.abs(feet[:, 1] - my) > MAT_SIZE[1] / 2 + EPS):
        return False
    return bool(
        handle_lean(gun) <= UPRIGHT + EPS
        and nozzle_height(gun) >= NOZZLE_UP - EPS
        and _inside(centre_of_mass(gun)[:2], _hull2(feet))
    )


# -- scoring --------------------------------------------------------------------------------------------------------
# In the simulator an on_step callback measures, every step, how close both grippers are to the gun and its stand,
# and how long the end state has held. Until it has measured, nothing is credited (fail closed). Offline
# (near_misses.py sets OFFLINE_FLAG; no simulator) each case stands for a state already held for 1 s with the grippers away.
_sim = {"measured": False, "clear": False, "since": None, "t": 0.0, "open": False}


# Offline mode is switched on only by near_misses.py, which sets this environment variable before its cases run.
OFFLINE_FLAG = "GLUE_GUN_TASK_OFFLINE_CHECK"


def _offline() -> bool:
    import os

    return os.environ.get(OFFLINE_FLAG) == "1"


def _done(objects) -> bool:
    gun = objects["glue_gun"]
    clear = True if _offline() else (_sim["measured"] and _sim["clear"])
    return bool(clear and gun.resting() and standing(gun))


def success(objects):
    """The gun stands on its handle on the mat, nozzle at least 40 mm up, let go and still."""
    return _done(objects)


def distance(objects):
    return max(0.0, NOZZLE_UP - nozzle_height(objects["glue_gun"]))


def score(objects):
    """Pass/fail: 1.0 once the success state has held for 1 s, otherwise 0."""
    if not _done(objects):
        return 0.0
    if _offline():
        return 1.0
    since = _sim["since"]
    return 1.0 if since is not None and _sim["t"] - since >= HOLD - 1e-6 else 0.0


@parts.on_reset
def _new_trial(sim) -> None:
    _sim.update(measured=False, clear=False, since=None, t=0.0, open=False)
    sim.set_target("glue_gun", "stand", -0.1)


@parts.on_step
def _physics(sim) -> None:
    from yam_arena import analysis

    st = sim.states()
    gun = st["glue_gun"]
    # The stand's detent: past its crest the spring clicks it open against its stop, short of it back folded.
    is_open = float(gun.joints["stand"]) > DETENT_MID
    if is_open != _sim["open"]:
        sim.set_target("glue_gun", "stand", STAND_OPEN + 0.1 if is_open else -0.1)
        _sim["open"] = is_open
    # Release and hold.
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
    _sim["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for _, c, r, s in _boxes(gun))
    _sim["measured"] = True
    _sim["t"] = float(sim.t)
    if _sim["clear"] and gun.resting() and standing(gun):
        if _sim["since"] is None:
            _sim["since"] = float(sim.t)
    else:
        _sim["since"] = None


# -- poses (for the demo and near_misses.py) ----------------------------------------------------------------------


def pose(lean: float, about_x: float, about_z: float, stand: float = 0.0, dz: float = 0.0, roll: float = 0.0):
    """The gun's state leaning `lean` rad nose-down about the gun-frame point (about_x, 0, about_z), which rests on
    the mat (then raised `dz`), with the stand at `stand` rad. `roll` turns it onto its side first."""
    ry = parts._rot((0.0, 1.0, 0.0), lean)
    rx = parts._rot((1.0, 0.0, 0.0), roll)
    r = parts._rot((0.0, 0.0, 1.0), GUN_YAW) @ ry @ rx
    contact = np.array([GUN_XY[0], GUN_XY[1], MAT_TOP + dz]) + parts._rot((0.0, 0.0, 1.0), GUN_YAW) @ np.array(
        [HEEL_X, 0.0, 0.0])
    p = contact - r @ np.array([about_x, 0.0, about_z])
    return GUN.state(joints={"stand": stand}, pos=p, quat_xyzw=parts._matrix_to_quat(r))


def tipped_lean() -> float:
    """Resting on the handle's front edge and the nozzle tip's underside."""
    dx, dz = NOZZLE_TIP[0] - HEEL_X, NOZZLE_Z - TIP_R
    return math.atan2(dz, dx)


START = pose(tipped_lean(), HEEL_X, 0.0)


def world(gun_state, point) -> np.ndarray:
    return np.asarray(gun_state.pos) + _rot(gun_state.quat_xyzw) @ np.asarray(point, float)


# -- demo -----------------------------------------------------------------------------------------------------------
# Open loop from the published start layout (the gun tipped on its nozzle, where it settles).
TIP = 0.013  # finger tips below the TCP
LEFT_GRIP = np.array([-0.004, 0.0, 0.115])  # top of the housing, over the centre of mass in the start pose
GRIP_DEPTH = 0.016  # finger tips this far below the housing's top there
PINCH = -0.012  # pads 24 mm apart on the 30 mm housing
WIDE = -0.025
PRESS_S = 0.040  # press the stand's near bar 40 mm from the pivot
PRESS_OUT = 0.008  # finger tips' centre 8 mm outside the bar, clear of the housing
PRESS_TILT = 30.0  # degrees: the right fingertips lean toward the gun, so the fingers' wider upper parts stay clear
PRESS_TO = 0.010  # finger tips' centre down to 10 mm above the mat (the leaning tips' outer edge 3 mm above it)
LIFT = 0.075
SET_DOWN = 0.060  # lowered back to 60 mm above where it lay: the open stand's foot touches at about 57 mm


def demo():
    """Right arm: press the stand's near bar down past its click so it springs out, then move away. Left arm: grip the
    housing over the centre of mass, lift the gun so the stand clicks fully open, lower it until the stand's foot is
    on the mat, let go so it rocks back onto its handle, and move away."""
    bar = np.array([_px - PRESS_S * math.cos(math.radians(5.0)), -STAND_W, _pz + PRESS_S * math.sin(math.radians(5.0))])
    press = world(START, bar)
    grip = world(START, LEFT_GRIP)
    gz = grip[2] - GRIP_DEPTH + TIP
    px, py = float(press[0]) - PRESS_OUT, float(press[1])
    gx, gy = float(grip[0]), float(grip[1])
    above = MAT_TOP + 0.10
    lean = dict(yaw=0.0, tilt=PRESS_TILT, heading=0.0)
    app = tilt.rotation(**lean) @ np.array([0.0, 0.0, 1.0])  # the approach axis: TCP to finger tips

    def tcp(z):  # the TCP that puts the finger tips' centre at (px, py, z)
        return tuple(float(v) for v in np.array([px, py, z]) - TIP * app)

    right = [
        tilt.Waypoint(1.5, tcp(above), 0.0, "over the stand's near bar", **lean),
        tilt.Waypoint(1.0, tcp(float(press[2]) + 0.010), 0.0, "down to the bar", **lean),
        tilt.Waypoint(2.0, tcp(MAT_TOP + PRESS_TO), 0.0, "press it down past the click", **lean),
        tilt.Waypoint(1.0, tcp(MAT_TOP + PRESS_TO), 0.0, "hold", **lean),
        tilt.Waypoint(1.5, tcp(above), 0.0, "up", **lean),
        Waypoint(1.5, (px - 0.05, py - 0.14, above), 0.0, "away to the right"),
        Waypoint(9.0, (px - 0.05, py - 0.14, above), 0.0, "wait"),
    ]
    yaw = -math.pi / 2  # fingers close along world x, across the housing
    left = [
        Waypoint(1.5, (gx + 0.08, gy + 0.12, gz + 0.06), WIDE, "wait aside", yaw),
        Waypoint(6.0, (gx + 0.08, gy + 0.12, gz + 0.06), WIDE, "wait aside", yaw),
        Waypoint(1.5, (gx, gy, gz + 0.05), WIDE, "over the housing", yaw),
        Waypoint(1.0, (gx, gy, gz), WIDE, "down", yaw),
        Waypoint(0.8, (gx, gy, gz), PINCH, "grip", yaw),
        Waypoint(1.5, (gx, gy, gz + LIFT), PINCH, "lift: the stand clicks open", yaw),
        Waypoint(1.0, (gx, gy, gz + LIFT), PINCH, "hold", yaw),
        Waypoint(1.5, (gx, gy, gz + SET_DOWN), PINCH, "lower onto the stand's foot", yaw),
        Waypoint(0.6, (gx, gy, gz + SET_DOWN), WIDE, "let go", yaw),
        Waypoint(1.2, (gx, gy, gz + SET_DOWN + 0.02), WIDE, "up", yaw),
        Waypoint(1.0, (gx, gy + 0.10, gz + SET_DOWN + 0.02), WIDE, "away", yaw),
        Waypoint(3.0, (gx, gy + 0.10, gz + SET_DOWN + 0.02), WIDE, "wait while it settles", yaw),
    ]
    return {"left": left, "right": right}
