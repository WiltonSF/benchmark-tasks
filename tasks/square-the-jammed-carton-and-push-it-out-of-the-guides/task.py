"""On the station table, in front of the right arm: the end of a conveyor lane, two fixed steel guide bars running
away from the robot with a gap between them, open at both ends. Their last stretch is a yellow section, thicker on the
inside, so the lane narrows at a square shoulder on each side. A brown cardboard carton, too wide to grip either way,
stands flat in the lane, turned askew, with one leading corner caught against a shoulder.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
CY = -0.12  # lane centre line
X0, XS, XE = 0.16, 0.34, 0.40  # guides' near end, the shoulder, the guides' far end
WIDE, NARROW = 0.059, 0.053  # half the gap between the guides before and after the shoulder (118 mm, 106 mm)
WALL = 0.012  # guide thickness before the shoulder
GH = 0.040  # guide height
CARTON = (0.12, 0.10, 0.05)  # 120 mm along the lane, 100 mm across, 50 mm tall
CARTON_MASS = 0.3
SKEW = math.radians(9.0)  # turned counter-clockwise seen from above: the front-left corner leads to the left guide
_FL_X = CARTON[0] / 2 * math.cos(SKEW) - CARTON[1] / 2 * math.sin(SKEW)  # front-left corner ahead of the centre
START_X = XS - 0.0005 - _FL_X  # front-left corner 0.5 mm short of the left shoulder

STEEL, YELLOW, CARD = (0.55, 0.57, 0.60), (0.88, 0.70, 0.10), (0.42, 0.26, 0.12)
_STEP = WALL + WIDE - NARROW  # thickness of the guide past the shoulder (18 mm)

GUIDES = []
for _side, _s in (("left", 1), ("right", -1)):
    GUIDES.append(Box(f"guide_{_side}", pos=((X0 + XS) / 2, CY + _s * (WIDE + WALL / 2), TOP + GH / 2),
                      size=(XS - X0, WALL, GH), color=STEEL))
    GUIDES.append(Box(f"guide_{_side}_narrow", pos=((XS + XE) / 2, CY + _s * (NARROW + _STEP / 2), TOP + GH / 2),
                      size=(XE - XS, _STEP, GH), color=YELLOW))
BOX = Box("carton", pos=(START_X, CY, TOP + CARTON[2] / 2), size=CARTON, mass=CARTON_MASS, yaw=SKEW, color=CARD)
OBJECTS = [*GUIDES, BOX]

EPS = 1e-6
FLAT_DEG = 5.0  # top face within 5 degrees of level
ON_TABLE = 0.001  # underside at most 1 mm above the table
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the carton
IN_NARROW = 0.030  # partial credit: the carton's leading end at least 30 mm past the shoulder
SQUARE_DEG = 3.0
OUTER = WIDE + WALL  # the guides' outer faces, either side of the centre line
END_OFF = 0.06  # out: centre within 6 cm of the lane's centre line


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# Released: an on_step callback measures, every step, how close both arms' fingers and gripper bodies are to the
# carton. Offline (near_misses.py) there is no robot, so "clear" stays True.
_grip = {"clear": True}


# Path: the carton must go through the lane, never around the guides. Judged over the whole trial: once its near end
# has gone back past the guides' near end, or its box has reached past a guide's outer face anywhere between the
# guides' two ends, the trial scores 0. The record clears only at a scene reset: the carton back within 0.5 mm of
# its start, at rest, having got there by a jump of over 20 mm in one control step.
_path = {"fault": False, "prev": None}


def _start_pose(st) -> bool:
    return bool(np.linalg.norm(np.asarray(st.pos, float) - np.asarray(BOX.pos, float)) <= 0.0005 and st.resting())


def _track(st) -> None:
    pos = np.asarray(st.pos, float)
    prev = _path["prev"]
    if prev is not None and _start_pose(st) and np.linalg.norm(pos - prev) > 0.020:
        _path["fault"] = False
    _path["prev"] = pos
    lo, hi = st.low, st.high
    beside = float(hi[0]) > X0 + EPS and float(lo[0]) < XE - EPS and (
        float(hi[1]) > CY + OUTER + EPS or float(lo[1]) < CY - OUTER - EPS)
    if float(lo[0]) < X0 - EPS or beside:
        _path["fault"] = True


@parts.on_reset
def _grip_reset(sim):
    _grip["clear"] = True
    _path["fault"] = False
    _path["prev"] = None


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    c = sim.states()["carton"]
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
    gap = analysis.hulls_to_box(hulls, np.asarray(c.pos, float), _rot(c.quat_xyzw), np.asarray(CARTON))[0]
    _grip["clear"] = bool(gap >= CLEAR - EPS)


def _tilt(st) -> float:
    return math.degrees(math.acos(float(np.clip(_rot(st.quat_xyzw)[2, 2], -1.0, 1.0))))


def _skew(st) -> float:
    r = _rot(st.quat_xyzw)
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _settled(st) -> bool:
    """Flat on the table, still, and let go."""
    return bool(
        _tilt(st) <= FLAT_DEG + EPS
        and abs(float(st.low[2]) - TOP) <= ON_TABLE + EPS
        and st.resting()
        and _grip["clear"]
    )


def _out(st) -> bool:
    """Wholly past the guides' far end, in line with the lane."""
    return bool(float(st.low[0]) >= XE - EPS and abs(float(st.pos[1]) - CY) <= END_OFF + EPS)


def _in_narrow(st) -> bool:
    """Square in the narrow part, its leading end at least 30 mm past the shoulder and not yet out."""
    return bool(
        float(st.high[0]) >= XS + IN_NARROW - EPS
        and float(st.low[0]) < XE - EPS
        and abs(_skew(st)) <= SQUARE_DEG + EPS
        and abs(float(st.pos[1]) - CY) <= NARROW - CARTON[1] / 2 + 0.001 + EPS
    )


def success(objects):
    """Through the lane and wholly past the far end of the guides, flat on the table, still and let go."""
    c = objects["carton"]
    _track(c)
    return bool(not _path["fault"] and _out(c) and _settled(c))


def distance(objects):
    return float(max(0.0, XE - float(objects["carton"].low[0])))


def score(objects):
    """1.0 as success(); 0.5 squared into the narrow part, its leading end at least 30 mm in, flat, still and let
    go; else 0. 0 for the rest of the trial once the carton has gone round the guides."""
    c = objects["carton"]
    _track(c)
    if _path["fault"] or not _settled(c):
        return 0.0
    if _out(c):
        return 1.0
    return 0.5 if _in_narrow(c) else 0.0


# -- scripted demo: fixed waypoints, no state read. The right arm backs the carton off the shoulder by pushing its
# front face back, near the caught corner's side so it slides back without turning, then re-feeds it with one
# finger pushed straight up the middle of the lane, which turns its rear face square as it goes. -----------------
TIP = 0.013  # pad tips below the TCP
Z = TOP + 0.020  # TCP height while pushing: tips 7 mm above the table
LIFT = 0.07
BACK_DY = 0.015  # back-off contact: 15 mm left of the carton's centre line
BACK_DIST = 0.06


def _front_x(dy: float) -> float:
    """x of the carton's front face at the start, dy to the left of its centre line."""
    c, s = math.cos(SKEW), math.sin(SKEW)
    hx, hy = CARTON[0] / 2, CARTON[1] / 2
    fl = (hx * c - hy * s, hx * s + hy * c)
    fr = (hx * c + hy * s, hx * s - hy * c)
    t = (fl[1] - dy) / (fl[1] - fr[1])
    return START_X + fl[0] + t * (fr[0] - fl[0])


PARK_L = (0.18, 0.24, TOP + 0.12)


def demo():
    x0 = _front_x(BACK_DY) + 0.027  # the finger's back edge (20 mm behind the TCP) 7 mm ahead of the face
    y0 = CY + BACK_DY
    x1 = x0 - 0.007 - BACK_DIST
    feed_x = 0.13
    end_x = XE + 0.012 - TIP  # pad tips 12 mm past the far end
    right = [
        Waypoint(1.5, (x0, y0, Z + LIFT), FINGER_CLOSED, "over the lane ahead of the carton"),
        Waypoint(1.0, (x0, y0, Z), FINGER_CLOSED, "down in front of it"),
        Waypoint(5.0, (x1, y0, Z), FINGER_CLOSED, "push it back off the shoulder"),
        Waypoint(0.8, (x1, y0, Z + LIFT), FINGER_CLOSED, "up"),
        Waypoint(1.2, (feed_x, CY, Z + LIFT), FINGER_CLOSED, "behind the carton"),
        Waypoint(1.0, (feed_x, CY, Z), FINGER_CLOSED, "down behind it"),
        Waypoint(8.0, (end_x, CY, Z), FINGER_CLOSED, "push it through and out"),
        Waypoint(1.0, (XE - 0.03, CY, Z + LIFT), FINGER_CLOSED, "back and up"),
    ]
    return {"left": [Waypoint(1.0, PARK_L, FINGER_OPEN, "park")], "right": right}
