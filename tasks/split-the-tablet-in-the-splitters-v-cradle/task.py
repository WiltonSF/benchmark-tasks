"""On the station table, between the arms: a tablet splitter. Its light blue base plate carries two low dark grey
walls that meet in a V pointing away from the robot, and a blue lid frame, hinged behind the V, stands open leaning
back on its stop. Under the lid runs a dark steel blade bar. A large round white tablet lies loose on the plate in
front of the V. Where the splitter stands, which way it is turned, and where the tablet lies change between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (parts before lots: its reset hooks run before the lots move objects)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

PLATE_BLUE, WHITE, LID_BLUE = (0.62, 0.78, 0.92), (0.96, 0.96, 0.94), (0.16, 0.36, 0.72)
STEEL, TABLET_WHITE, WALL_GREY = (0.20, 0.21, 0.23), (0.98, 0.97, 0.92), (0.30, 0.32, 0.35)
PLATE_FRICTION = (0.4, 0.35)  # smooth plastic
WALL_FRICTION = (1.2, 1.0)  # grips the plate, so a push on a wall does not slide it
TABLET_FRICTION = (0.05, 0.05)  # a coated tablet: with the plate about 0.2, with a wall about 0.6

# Splitter frame: origin at the lid's pivot line on the table (in lot A, 305 mm out on the midline), x away from the
# robot along the blade line (the V's bisector), y to the robot's left, z up. Lots move and turn the whole splitter.
PLATE_T = 0.008
PLATE_X, PLATE_Y = (-0.155, 0.010), 0.065  # x range, half-width
APEX_X = -0.015
WALL_H, WALL_T = 0.007, 0.005
MOUTH = 0.070  # the V's opening at its front corners
DEPTH = MOUTH / 2  # walls at 45 degrees: the mouth is 35 mm in front of the apex
MOUTH_X = APEX_X - DEPTH
PIVOT_Z = PLATE_T + 0.018  # above the table top
SPLIT_MASS, WALL_MASS = 2.0, 1.5  # loose but heavy only so that the lots can move them

TAB_R, TAB_H, TAB_MASS = 0.015, 0.010, 0.02
SEAT_X = APEX_X - TAB_R * math.sqrt(2.0)  # the centre of a tablet touching both walls: 36.2 mm in front of the pivot

# Each wall: (outward normal of its inner face in the splitter frame, direction from the apex to its mouth corner,
# and the sign that turns the wall box's own y axis into that normal).
_S = 1.0 / math.sqrt(2.0)
WALLS = {
    "wall_right": (np.array([_S, -_S]), np.array([-_S, -_S]), 1.0),
    "wall_left": (np.array([_S, _S]), np.array([-_S, _S]), -1.0),
}
WALL_LEN = DEPTH * math.sqrt(2.0)  # inner faces meet at the apex; the boxes must not overlap (they would push apart)


def _wall_local(name):
    n, d, _ = WALLS[name]
    mid = np.array([APEX_X, 0.0]) + d * WALL_LEN / 2 + n * WALL_T / 2
    return mid, float(math.atan2(d[1], d[0]))


def _r2(a):
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


# Lots: where the pivot stands (world x, y) and how far the splitter is turned (rad, about the vertical), and where the
# tablet starts in the splitter frame.
POSES = {"A": ((0.305, 0.0), 0.0), "B": ((0.305, -0.080), math.radians(20.0)), "C": ((0.305, 0.080), math.radians(-20.0))}
TAB_STARTS = {"A": (-0.115, 0.032), "B": (-0.115, -0.032), "C": (-0.065, 0.0)}


def to_world(lot, xy):
    (hx, hy), a = POSES[lot]
    return np.array([hx, hy]) + _r2(a) @ np.asarray(xy, float)


# The lid, given as it lies closed (flat, toward the robot): u is the distance from the pivot toward the robot, zc the
# height relative to the pivot. It is written standing up (joint 0, vertical): a closed point (u, zc) stands at
# x = zc (splitter frame), height PIVOT_Z + u. Joint position q turns it toward the robot; q = pi/2 is flat.
BAR_W, BAR_UNDER, BAR_TOPC = 0.004, -0.008, -0.002  # the bar's underside lies 10 mm above the plate when flat
BAR_U = (0.012, 0.075)
RAIL_Y = 0.036
LID_MASS = 0.04
LID_BACK, LID_UP = -0.15, math.pi / 2 + 0.12  # leans back 8.6 degrees on its stop; empty, the bar's tip reaches the plate
LID_DAMPING = 0.015


def _lid_box(u, y, zc, color):
    return parts.box((zc[1] - zc[0], y[1] - y[0], u[1] - u[0]),
                     at=((zc[0] + zc[1]) / 2, (y[0] + y[1]) / 2, PIVOT_Z + (u[0] + u[1]) / 2), color=color)


BAR = _lid_box(BAR_U, (-BAR_W / 2, BAR_W / 2), (BAR_UNDER, BAR_TOPC), STEEL)
LID_SHAPES = [
    BAR,
    _lid_box((0.005, 0.080), (RAIL_Y - 0.004, RAIL_Y + 0.004), (-0.002, 0.006), LID_BLUE),  # left rail
    _lid_box((0.005, 0.080), (-RAIL_Y - 0.004, -RAIL_Y + 0.004), (-0.002, 0.006), LID_BLUE),  # right rail
    _lid_box((0.005, 0.013), (-RAIL_Y - 0.004, RAIL_Y + 0.004), (-0.002, 0.006), LID_BLUE),  # back cross piece
    _lid_box((0.072, 0.082), (-RAIL_Y - 0.004, RAIL_Y + 0.004), (-0.002, 0.010), LID_BLUE),  # front grip tab
]
LID = parts.Hinge("lid", shapes=LID_SHAPES, pivot=(0.0, 0.0, PIVOT_Z), axis=(0.0, -1.0, 0.0), lower=LID_BACK,
                  upper=LID_UP, mass=LID_MASS, damping=LID_DAMPING)

SPLITTER = parts.Part(
    "splitter",
    pos=(POSES["A"][0][0], POSES["A"][0][1], TOP),
    mass=SPLIT_MASS,
    shapes=[
        parts.box((PLATE_X[1] - PLATE_X[0], 2 * PLATE_Y, PLATE_T),
                  at=((PLATE_X[0] + PLATE_X[1]) / 2, 0.0, PLATE_T / 2), color=PLATE_BLUE),
        parts.box((0.008, 0.010, PIVOT_Z - PLATE_T), at=(0.0, 0.045, (PIVOT_Z + PLATE_T) / 2), color=WHITE),  # hinge
        parts.box((0.008, 0.010, PIVOT_Z - PLATE_T), at=(0.0, -0.045, (PIVOT_Z + PLATE_T) / 2), color=WHITE),  # posts
        parts.cylinder(0.004, 0.100, at=(0.0, 0.0, PIVOT_Z), color=WHITE, axis="y"),  # hinge pin
    ],
    joints=[LID],
    friction=PLATE_FRICTION,
)


def _wall(name):
    mid, yaw = _wall_local(name)
    w = to_world("A", mid)
    return parts.Part(name, pos=(float(w[0]), float(w[1]), TOP + PLATE_T), yaw=yaw, mass=WALL_MASS,
                      friction=WALL_FRICTION,
                      shapes=[parts.box((WALL_LEN, WALL_T, WALL_H), at=(0.0, 0.0, WALL_H / 2), color=WALL_GREY)])


_TA = to_world("A", TAB_STARTS["A"])
TABLET = parts.Part("tablet", pos=(float(_TA[0]), float(_TA[1]), TOP + PLATE_T), mass=TAB_MASS,
                    friction=TABLET_FRICTION,
                    shapes=[parts.cylinder(TAB_R, TAB_H, at=(0.0, 0.0, TAB_H / 2), color=TABLET_WHITE, axis="z")])
WALL_R, WALL_L = _wall("wall_right"), _wall("wall_left")
OBJECTS = [SPLITTER, WALL_R, WALL_L, TABLET]

# lots.py moves a jointed part by writing its root body (the plate); the lid follows through its hinge and pulls the
# plate back by the lid's share of the mass. Asking for the plate's move scaled up by total over plate mass lands it
# where the lot says.
LOT_MOVE_GAIN = (SPLIT_MASS + LID_MASS) / SPLIT_MASS


def _lot(name):
    (hx, hy), a = POSES[name]
    d = np.array([hx, hy]) - np.array(POSES["A"][0])
    sp = np.array(POSES["A"][0]) + d * LOT_MOVE_GAIN
    place = {"splitter": (float(sp[0]), float(sp[1]), TOP)}
    yaw = {"splitter": a}
    for w in WALLS:
        mid, wy = _wall_local(w)
        p = to_world(name, mid)
        place[w], yaw[w] = (float(p[0]), float(p[1]), TOP + PLATE_T), wy + a
    t = to_world(name, TAB_STARTS[name])
    place["tablet"] = (float(t[0]), float(t[1]), TOP + PLATE_T)
    return lots.Lot(name, place=place, yaw=yaw)


# Lot A is the screenshot. Every lot moves its objects (lot A by zero), so each trial gets the same settle and frame.
LOTS = lots.declare(OBJECTS, _lot("A"), _lot("B"), _lot("C"))

EPS = 1e-6
WALL_GAP = 0.001  # seated: the tablet's edge within 1 mm of each wall's inner face
ON_PLATE = 0.001  # its lowest point within 1 mm of the plate
FLAT = math.radians(10.0)
BLADE_GAP = 0.001  # blade down: the bar's underside within 1 mm of the tablet's top
LID_STILL = 0.005  # rad per control step (0.1 rad/s)
CLEAR = 0.005  # released: no gripper or finger within 5 mm of the tablet or the lid
SUCCESS_HOLD_S = 1.0


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(state) -> float:
    return math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0)))


def _centre(tab) -> np.ndarray:
    """The tablet's centre (its body origin is the middle of its underside)."""
    return np.asarray(tab.pos, float) + _rot(tab.quat_xyzw) @ np.array([0.0, 0.0, TAB_H / 2])


def _yaw(state) -> float:
    r = _rot(state.quat_xyzw)
    return math.atan2(r[1, 0], r[0, 0])


def wall_gaps(objects, tab) -> list[float]:
    """Gap from the tablet's rim to each wall's inner face, seen from above (negative: pressed into it), read from
    where the walls actually are."""
    c = _centre(tab)[:2]
    out = []
    for name, (_, _, sign) in WALLS.items():
        w = objects[name]
        r = _rot(w.quat_xyzw)
        n = sign * r[:2, 1] / np.linalg.norm(r[:2, 1])
        inner = np.asarray(w.pos)[:2] - n * WALL_T / 2
        out.append(float(-((c - inner) @ n)) - TAB_R)
    return out


def _plate_top(objects) -> float:
    return float(objects["splitter"].pos[2]) + PLATE_T


def in_v(objects, tab, gap=None) -> bool:
    """Seen from above, touching both walls of the V (each gap at most 1 mm, or `gap`)."""
    g = WALL_GAP if gap is None else gap
    return all(x <= g + EPS for x in wall_gaps(objects, tab))


def seated(objects, tab) -> bool:
    """Flat on the plate and touching both walls of the V."""
    return bool(_tilt(tab) <= FLAT + EPS and abs(tab.low[2] - _plate_top(objects)) <= ON_PLATE + EPS
                and in_v(objects, tab))


def blade_gap(split, tab) -> float | None:
    """Height of the bar's underside above the tablet's top, at the point of the bar nearest the tablet's centre seen
    from above; None if the bar does not pass over the tablet."""
    pos, quat = split.links["lid"]
    r = _rot(quat)
    c = np.asarray(pos) + r @ (np.asarray(BAR.at) - LID.origin)
    half = np.asarray(BAR.size) / 2
    centre = _centre(tab)
    along = r[:, 2]  # the bar's long axis (part z when standing)
    flat = np.array([along[0], along[1], 0.0])
    if np.linalg.norm(flat) < 1e-6:
        return None
    t = float(np.clip((centre - c)[:2] @ along[:2] / (flat @ flat), -half[2], half[2]))
    under = c + along * t - r[:, 0] * half[0]  # the bar's underside: its -x face in the standing pose
    side = r[:, 1]
    lateral = abs(float((centre - under)[:2] @ side[:2]))
    if lateral > TAB_R + half[1] + EPS:
        return None
    top = centre[2] + TAB_H / 2
    lowest = min(under[2] + r[2, 1] * half[1] * k for k in (-1.0, 1.0))
    return float(lowest - top)


def blade_down(split, tab) -> bool:
    g = blade_gap(split, tab)
    return g is not None and g <= BLADE_GAP + EPS


# Whole-trial record. A fault: the blade comes down onto the tablet (bar within 1 mm of its top) while the tablet is not
# in the V (seen from above, a gap over 1.5 mm to either wall: an unequal cut), or the splitter is shifted more than
# 3 mm or turned more than 2 degrees, or a wall more than 2 mm, from its lot's place. Cleared only by a scene reset:
# the tablet back within 2 mm of its lot's start and at rest, after a jump of over 20 mm in one control step (a robot
# cannot move it back that way).
FAULT_GAP = 0.0015
SPLIT_MOVE, SPLIT_TURN, WALL_MOVE = 0.003, math.radians(2.0), 0.002
RESET_TOL = 0.002
_record = {"fault": False, "prev": None}
# Released: an on_step callback measures, every step, how close both arms' fingers and grippers come to the tablet and
# to the lid, and how far the lid turned since the last step. Offline (near_misses.py) there is no robot, so "clear"
# stays True and the lid counts as still.
_grip = {"clear": True, "q": None, "dq": 0.0}


def _displaced(objects) -> bool:
    name = lots.current().name
    (hx, hy), a = POSES[name]
    sp = objects["splitter"]
    if np.hypot(sp.pos[0] - hx, sp.pos[1] - hy) > SPLIT_MOVE + EPS:
        return True
    if abs((_yaw(sp) - a + math.pi) % (2 * math.pi) - math.pi) > SPLIT_TURN + EPS or _tilt(sp) > SPLIT_TURN + EPS:
        return True
    for w in WALLS:
        p = to_world(name, _wall_local(w)[0])
        if np.hypot(*(np.asarray(objects[w].pos)[:2] - p)) > WALL_MOVE + EPS:
            return True
    return False


def _update(objects) -> None:
    tab, split = objects["tablet"], objects["splitter"]
    pos = np.asarray(tab.pos, float)
    prev = _record["prev"]
    start = to_world(lots.current().name, TAB_STARTS[lots.current().name])
    if prev is not None and np.hypot(*(pos[:2] - start)) < RESET_TOL and tab.resting():
        if np.linalg.norm(pos - prev) > 0.020:
            _record["fault"] = False
    _record["prev"] = pos
    if (blade_down(split, tab) and not in_v(objects, tab, FAULT_GAP)) or _displaced(objects):
        _record["fault"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    tab, split = st["tablet"], st["splitter"]
    boxes = TABLET.world_boxes({"base": (np.asarray(tab.pos), _rot(tab.quat_xyzw))})
    lp, lq = split.links["lid"]
    boxes += SPLITTER.world_boxes({"lid": (np.asarray(lp), _rot(lq))})
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
    q = split.joints.get("lid", 0.0)
    _grip["dq"] = 0.0 if _grip["q"] is None else abs(q - _grip["q"])
    _grip["q"] = q
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for _, c, r, s in boxes)


def _settled(objects) -> bool:
    return bool(objects["tablet"].resting() and _grip["dq"] <= LID_STILL + EPS and _grip["clear"])


def success(objects):
    """The tablet sits in the V touching both walls and the blade rests on it, let go and still; the blade never came
    down on the tablet while it was out of the V."""
    _update(objects)
    tab, split = objects["tablet"], objects["splitter"]
    return bool(not _record["fault"] and _settled(objects) and seated(objects, tab) and blade_down(split, tab))


def distance(objects):
    c = _centre(objects["tablet"])[:2]
    return float(np.hypot(*(c - to_world(lots.current().name, (SEAT_X, 0.0)))))


def score(objects):
    """1.0 on success. 0.5: the tablet sits in the V touching both walls, let go and still, the blade not on it."""
    _update(objects)
    tab, split = objects["tablet"], objects["splitter"]
    if _record["fault"] or not _settled(objects) or not seated(objects, tab):
        return 0.0
    return 1.0 if blade_down(split, tab) else 0.5


# --- demo: one arm pushes the tablet into the V, then an arm tips the lid over -------------------------------------
TIP = 0.013  # finger tips below the TCP
TIP_HALF = 0.0132  # closed finger tips: 26 mm long along the grip's x, 13 mm across
PUSH_Z = TOP + PLATE_T + 0.003 + TIP  # tips 3 mm above the plate
OVERSHOOT = 0.015  # commanded past the stop; the arm's compliance takes it up
TRAVEL = TOP + 0.15
LID_TRAVEL = TOP + 0.13  # higher, an arm cannot point straight down behind the lid
LID_TOP_BACK = 0.010 * math.cos(LID_BACK) + 0.082 * math.sin(-LID_BACK)  # the tab's back face at the top (splitter x)
LID_TOP_Z = TOP + PIVOT_Z + 0.082 * math.cos(LID_BACK)
LID_PULL_TO = -0.005  # splitter x the finger centre is pulled to: the lid well past upright
LID_PUSH_Y = 0.020
# Which arm pushes the tablet and which tips the lid, per lot (the arm on the splitter's side does both in B and C).
ARMS = {"A": ("right", "left"), "B": ("right", "right"), "C": ("left", "left")}
PARK = {"left": (0.15, 0.22), "right": (0.15, -0.22)}


def _push_plan(lot, arm):
    s = to_world(lot, TAB_STARTS[lot])
    goal = to_world(lot, (SEAT_X, 0.0))
    d = (goal - s) / np.linalg.norm(goal - s)
    yaw = float(math.atan2(d[1], d[0]))
    behind = s - d * (TAB_R + TIP_HALF + 0.006)
    end = goal - d * (TAB_R + TIP_HALF - OVERSHOOT)
    back = end - d * 0.020
    return [
        Waypoint(1.5, (*behind, TRAVEL), FINGER_CLOSED, "over the tablet's far side", yaw),
        Waypoint(1.5, (*behind, PUSH_Z), FINGER_CLOSED, "down behind the tablet", yaw),
        Waypoint(3.0, (*end, PUSH_Z), FINGER_CLOSED, "push it into the V", yaw),
        Waypoint(1.0, (*end, PUSH_Z), FINGER_CLOSED, "hold", yaw),
        Waypoint(0.8, (*back, PUSH_Z), FINGER_CLOSED, "back off", yaw),
        Waypoint(1.0, (*back, TRAVEL), FINGER_CLOSED, "up", yaw),
        Waypoint(1.5, (*PARK[arm], TRAVEL), FINGER_CLOSED, "park", 0.0),
    ]


def _lid_plan(lot, arm):
    side = 1.0 if arm == "left" else -1.0
    a = POSES[lot][1]
    start = to_world(lot, (LID_TOP_BACK + TIP_HALF + 0.006, side * LID_PUSH_Y))
    end = to_world(lot, (LID_PULL_TO, side * LID_PUSH_Y))
    return [
        Waypoint(2.0, (*start, LID_TRAVEL), FINGER_CLOSED, "over the back of the lid", a),
        Waypoint(1.5, (*start, LID_TOP_Z), FINGER_CLOSED, "down behind the lid's top", a),
        Waypoint(2.5, (*end, LID_TOP_Z), FINGER_CLOSED, "pull the top over", a),
        Waypoint(0.8, (*end, LID_TRAVEL), FINGER_CLOSED, "up", a),
        Waypoint(1.5, (*PARK[arm], LID_TRAVEL), FINGER_CLOSED, "away", 0.0),
    ]


def plan(lot):
    """Waypoints per arm for `lot`, padded so both arms run the same length and end with 7 s of settling."""
    pusher, closer = ARMS[lot]
    out = {"left": [], "right": []}
    out[pusher] += _push_plan(lot, pusher)
    t = sum(w.duration for w in out[pusher])
    if closer != pusher:
        out[closer].append(Waypoint(t, (*PARK[closer], LID_TRAVEL), FINGER_CLOSED, "wait"))
    out[closer] += _lid_plan(lot, closer)
    end = max(sum(w.duration for w in v) for v in out.values()) + 7.0
    for arm, ws in out.items():
        rest = ws[-1].pos if ws else (*PARK[arm], TRAVEL)
        ws.append(Waypoint(end - sum(w.duration for w in ws), rest, FINGER_CLOSED, "wait while the lid settles"))
    return out


def demo():
    """Closed fingers behind the tablet push it along the line to the V's apex until both walls hold it; the arm backs
    off and parks. Then an arm comes down behind the open lid's top, pulls it toward the robot past upright and lets
    it fall onto the tablet. Where the splitter and tablet are is what the cameras show for the trial's lot."""
    return plan(lots.current().name)
