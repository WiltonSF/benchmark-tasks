"""On the station table: a small model wheelchair facing the robot, with its seat, backrest and two big rear wheels,
and in front of it, nearer the robot, a model mobile patient hoist. The hoist has a grey mast standing on a low base frame with a battery box,
a boom reaching from the mast's top toward the chair with a red spreader block hanging from its end, and two orange
legs hinged on vertical pins under the frame at the mast's line, 30 mm behind the frame's front edge, lying side by side and pointing at the chair. Where the pair stands changes
between trials.
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

GREY, ORANGE, RED = (0.70, 0.72, 0.75), (0.95, 0.55, 0.12), (0.85, 0.12, 0.10)
BLACK, SEAT_BLUE, CHROME = (0.08, 0.08, 0.09), (0.15, 0.25, 0.55), (0.55, 0.57, 0.60)

# Hoist, part frame: origin at the middle of the cross bar's underside, x toward the chair, z up.
BAR = (0.090, 0.100, 0.015)  # the base frame the mast stands on, from 60 mm behind the mast to 30 mm in front
BAR_X = -0.015
BATTERY = (0.030, 0.050, 0.030)  # battery box on the frame behind the mast
BATTERY_IN = 0.012  # set in from the frame's back edge, so a push on that edge meets only the frame
MAST = 0.025
MAST_TOP = 0.160
REACH = 0.100  # the spreader block's centre, in front of the mast's centre
SPREADER = 0.015
LEG = (0.200, 0.020, 0.0135)
LEG_CLEAR = 0.0015  # the legs ride 1.5 mm clear of the table; the frame's pads carry the hoist
LEG_Y = 0.045  # each leg's centre line when closed: 70 mm apart inside
LEG_OPEN = 0.55  # the legs' outer stop, rad
HOIST_MASS, LEG_MASS = 0.30, 0.02
FEET = (0.15, 0.15)  # low friction under the frame stands in for castors


def _leg(name, side):
    return parts.Hinge(
        name, pivot=(0.0, side * LEG_Y, LEG_CLEAR + LEG[2] / 2), axis=(0.0, 0.0, float(side)), lower=0.0,
        upper=LEG_OPEN, mass=LEG_MASS, damping=0.01,
        shapes=[parts.box(LEG, at=(LEG[0] / 2, side * LEG_Y, LEG_CLEAR + LEG[2] / 2), color=ORANGE)])


def _hoist_shapes():
    return [
        parts.box(BAR, at=(BAR_X, 0.0, BAR[2] / 2), color=GREY),
        parts.box(BATTERY, at=(BAR_X - BAR[0] / 2 + BATTERY_IN + BATTERY[0] / 2, 0.0, BAR[2] + BATTERY[2] / 2), color=CHROME),
        parts.box((MAST, MAST, MAST_TOP - BAR[2]), at=(0.0, 0.0, (MAST_TOP + BAR[2]) / 2), color=GREY),
        parts.box((REACH + MAST / 2 + SPREADER / 2, 0.020, 0.015),
                  at=((REACH - MAST / 2 + SPREADER / 2) / 2, 0.0, MAST_TOP + 0.0075), color=GREY),  # boom
        parts.box((SPREADER,) * 3, at=(REACH, 0.0, MAST_TOP - SPREADER / 2), color=RED),
    ]


# Wheelchair stand-in, part frame: origin at the middle of its footprint on the table, x away from the robot (the
# chair faces the robot). It is a loose but heavy part only so that the lots can move it.
CHAIR_LEG, CHAIR_SPAN, SEAT_Z = 0.015, 0.100, 0.090
LEG_AT = (CHAIR_SPAN - CHAIR_LEG) / 2  # 42.5 mm
WHEEL_R, WHEEL_Y = 0.045, 0.058
CHAIR_MASS = 3.0


def _chair_shapes():
    out = [parts.box((CHAIR_LEG, CHAIR_LEG, SEAT_Z - 0.010), at=(sx * LEG_AT, sy * LEG_AT, (SEAT_Z - 0.010) / 2),
                     color=CHROME) for sx in (-1, 1) for sy in (-1, 1)]
    out += [
        parts.box((CHAIR_SPAN, CHAIR_SPAN, 0.010), at=(0.0, 0.0, SEAT_Z - 0.005), color=SEAT_BLUE),  # seat
        parts.box((0.010, CHAIR_SPAN, 0.100), at=(CHAIR_SPAN / 2 - 0.005, 0.0, SEAT_Z + 0.050), color=SEAT_BLUE),
        parts.cylinder(WHEEL_R, 0.008, at=(LEG_AT, WHEEL_Y, WHEEL_R), color=BLACK, axis="y"),
        parts.cylinder(WHEEL_R, 0.008, at=(LEG_AT, -WHEEL_Y, WHEEL_R), color=BLACK, axis="y"),
    ]
    return out


HOIST_X0, HOIST_X1 = 0.125, 0.315  # the mast's centre (on the legs' pin line) at the start, and with the spreader
# over the seat in lots A and B
CHAIR_X = HOIST_X1 + REACH  # 0.415: the seat's centre; the chair's front legs are 40 mm in front of the leg tips
SHIFT_B = -0.070  # lot B: both 70 mm to the robot's right
FARTHER_C = 0.040  # lot C: the chair 40 mm farther out, the hoist where it is in lot A
# Each lot: (hoist mast x, y), (chair centre x, y), as the lot is meant to stand.
LOT_POS = {"A": ((HOIST_X0, 0.0), (CHAIR_X, 0.0)), "B": ((HOIST_X0, SHIFT_B), (CHAIR_X, SHIFT_B)),
           "C": ((HOIST_X0, 0.0), (CHAIR_X + FARTHER_C, 0.0))}

HOIST = parts.Part("hoist", pos=(HOIST_X0, 0.0, TOP), mass=HOIST_MASS, shapes=_hoist_shapes(),
                   joints=[_leg("left_leg", 1), _leg("right_leg", -1)], friction=FEET)
CHAIR = parts.Part("wheelchair", pos=(CHAIR_X, 0.0, TOP), mass=CHAIR_MASS, shapes=_chair_shapes())
OBJECTS = [HOIST, CHAIR]

# lots.py moves an articulated part by writing its root (the frame) only; the legs follow through their joints, which
# pulls the frame back by the legs' share of the mass (measured: 8.3 mm of 70). Asking for the move scaled up by the
# total over the frame's mass lands the hoist where the lot says.
LOT_MOVE_GAIN = (HOIST_MASS + 2 * LEG_MASS) / HOIST_MASS

LOTS = lots.declare(
    OBJECTS,
    *[lots.Lot(n, place={"hoist": (hx, hy * LOT_MOVE_GAIN, TOP), "wheelchair": (cx, cy, TOP)})
      for n, ((hx, hy), (cx, cy)) in LOT_POS.items()]
)

EPS = 1e-6
LEGS_OPEN = 0.45  # rad, each leg
OVER_SEAT = 0.030  # the spreader block's centre within 30 mm of the seat's centre, seen from above
TOUCH = 0.001  # any hoist shape within 1 mm of any chair shape
CHAIR_MOVE, CHAIR_TURN = 0.005, math.radians(3.0)
HOIST_TIP, HOIST_LIFT = math.radians(15.0), 0.010
CLEAR = 0.005
LEG_STILL = 0.005  # rad per control step
RESET_TOL = 0.002
FACING = math.radians(15.0)
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


def _turn(state) -> float:
    return 2.0 * math.acos(float(np.clip(abs(np.asarray(state.quat_xyzw, float)[3]), 0.0, 1.0)))


def _hoist_boxes(h):
    poses = {"base": (np.asarray(h.pos), _rot(h.quat_xyzw))}
    poses.update({k: (np.asarray(p), _rot(q)) for k, (p, q) in h.links.items()})
    return [(c, r, s) for _, c, r, s in HOIST.world_boxes(poses)]


def _chair_boxes(c):
    return [(cc, r, s) for _, cc, r, s in CHAIR.world_boxes({"base": (np.asarray(c.pos), _rot(c.quat_xyzw))})]


def _overlap(a, b, margin=0.0) -> bool:
    """Do two oriented boxes (centre, rotation, size) come within `margin` of each other? Separating-axis test."""
    (ca, ra, sa), (cb, rb, sb) = a, b
    ha, hb = np.asarray(sa) / 2 + margin / 2, np.asarray(sb) / 2 + margin / 2
    d = np.asarray(cb) - np.asarray(ca)
    axes = [ra[:, i] for i in range(3)] + [rb[:, i] for i in range(3)]
    axes += [np.cross(ra[:, i], rb[:, j]) for i in range(3) for j in range(3)]
    for ax in axes:
        n = np.linalg.norm(ax)
        if n < 1e-9:
            continue
        ax = ax / n
        if abs(ax @ d) > sum(ha[i] * abs(ax @ ra[:, i]) for i in range(3)) + sum(
                hb[i] * abs(ax @ rb[:, i]) for i in range(3)):
            return False
    return True


def touching(h, c) -> bool:
    return any(_overlap(a, b, TOUCH) for a in _hoist_boxes(h) for b in _chair_boxes(c))


def legs_open(h) -> bool:
    return all(h.joints[k] >= LEGS_OPEN - EPS for k in ("left_leg", "right_leg"))


def spreader_offset(h, c) -> float:
    s = np.asarray(h.pos) + _rot(h.quat_xyzw) @ np.array([REACH, 0.0, MAST_TOP - SPREADER / 2])
    return float(np.hypot(*(s[:2] - np.asarray(c.pos)[:2])))


# Whole-trial record: did the hoist ever touch the chair, move it over 5 mm or turn it over 3 degrees, tip over 15
# degrees or rise over 10 mm? Cleared only by a scene reset: the hoist back within 2 mm of its start and at rest,
# after a jump of over 20 mm in one control step (a robot cannot move it back that way).
_record = {"fault": False, "prev": None}
# Released: an on_step callback measures every step how close both arms' fingers and grippers come to the hoist, and
# how far its legs turned since the last step. Offline (near_misses.py) there is no robot: clear and still.
_grip = {"clear": True, "q": None, "dq": 0.0}


def _update(objects) -> None:
    h, c = objects["hoist"], objects["wheelchair"]
    lot = lots.current()
    pos = np.asarray(h.pos, float)
    prev = _record["prev"]
    start = np.array([*LOT_POS[lot.name][0], TOP])
    if prev is not None and np.linalg.norm(pos - start) < RESET_TOL and h.resting():
        if np.linalg.norm(pos - prev) > 0.020:
            _record["fault"] = False
    _record["prev"] = pos
    chair_moved = np.linalg.norm(np.asarray(c.pos)[:2] - np.asarray(lot.pos("wheelchair"))[:2]) > CHAIR_MOVE + EPS
    if (touching(h, c) or chair_moved or _turn(c) > CHAIR_TURN + EPS or _tilt(h) > HOIST_TIP + EPS
            or h.low[2] > TOP + HOIST_LIFT + EPS):
        _record["fault"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    h = sim.states()["hoist"]
    boxes = _hoist_boxes(h)
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
    q = np.array([h.joints["left_leg"], h.joints["right_leg"]])
    _grip["dq"] = 0.0 if _grip["q"] is None else float(np.abs(q - _grip["q"]).max())
    _grip["q"] = q
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in boxes)


def _settled(h) -> bool:
    return bool(h.resting() and _grip["dq"] <= LEG_STILL + EPS and _grip["clear"])


def success(objects):
    """Both legs open, the spreader block over the seat, the hoist let go and still; it never touched or moved the
    chair, tipped or was lifted."""
    _update(objects)
    h, c = objects["hoist"], objects["wheelchair"]
    return bool(not _record["fault"] and _settled(h) and legs_open(h)
                and spreader_offset(h, c) <= OVER_SEAT + EPS)


def distance(objects):
    return spreader_offset(objects["hoist"], objects["wheelchair"])


def score(objects):
    """1.0 on success. 0.5: both legs open, let go and still, the chair never touched, not yet in position, the hoist
    no farther from the chair than at the start and still facing it (turned at most 15 degrees)."""
    _update(objects)
    h, c = objects["hoist"], objects["wheelchair"]
    if _record["fault"] or not _settled(h) or not legs_open(h):
        return 0.0
    if spreader_offset(h, c) <= OVER_SEAT + EPS:
        return 1.0
    return 0.5 if not _moved_away(h, c) and _heading_off(h) <= FACING + EPS else 0.0


def _heading_off(h) -> float:
    """How far the hoist is turned from its start heading (straight at the chair), rad."""
    r = _rot(h.quat_xyzw)
    return abs(math.atan2(r[1, 0], r[0, 0]))


def _moved_away(h, c) -> bool:
    """The hoist's mast farther from the chair's centre than at the start of the trial."""
    (hx, hy), (cx, cy) = LOT_POS[lots.current().name]
    now = np.hypot(h.pos[0] - c.pos[0], h.pos[1] - c.pos[1])
    return bool(now > np.hypot(hx - cx, hy - cy) + 0.002 + EPS)


# --- demo -----------------------------------------------------------------------------------------------------
TIP = 0.013
LOW = TOP + 0.007 + TIP  # finger tips 7 mm above the table (the loaded arm sags a few mm)
TRAVEL = TOP + 0.12
LEG_PUSH_X = 0.150  # along the hoist from the cross bar: where the fingers push each leg
LEG_PUSH_TO = 0.136  # sideways from the hoist's centre line: 4 mm past the stop at 0.132 (more turns the hoist)
PUSH_PAST = 0.020  # the frame pushed this far past its mark; friction and the soft arm take it up
BEHIND = -BAR_X + BAR[0] / 2 + 0.0132 + 0.020  # finger centre behind the mast: comes down 20 mm clear of the frame
PUSH_FINGERS = -0.035  # fingers part open: two tips about 70 mm apart push the frame's back edge


def _open_leg(hx, hy, side, wait):
    x, y0, y1 = hx + LEG_PUSH_X, hy + side * 0.015, hy + side * LEG_PUSH_TO
    return [
        Waypoint(wait, (0.18, side * 0.20, TRAVEL), FINGER_CLOSED, "wait"),
        Waypoint(1.5, (x, y0, TRAVEL), FINGER_CLOSED, "over the gap between the legs"),
        Waypoint(1.2, (x, y0, LOW), FINGER_CLOSED, "down between them"),
        Waypoint(2.0, (x, y1, LOW), FINGER_CLOSED, "swing the leg out to its stop"),
        Waypoint(0.8, (x, y1, TRAVEL), FINGER_CLOSED, "up"),
        Waypoint(1.2, (0.18, side * 0.22, TRAVEL), FINGER_CLOSED, "park"),
    ]


def demo():
    """Right arm swings the right leg out from inside to its stop; then the left arm the left leg; then the right arm
    pushes the frame from behind with two finger tips until the spreader block is over the seat. Where the hoist stands is what the
    cameras show for the trial's lot."""
    (hx, hy), (cx, _) = LOT_POS[lots.current().name]
    right = _open_leg(hx, hy, -1, 0.5)
    t_right = sum(w.duration for w in right)
    left = _open_leg(hx, hy, 1, t_right)
    t_left = sum(w.duration for w in left)
    sx, ex = hx - BEHIND, cx - REACH - BEHIND + PUSH_PAST
    right += [
        Waypoint(t_left - t_right, (0.18, -0.22, TRAVEL), FINGER_CLOSED, "wait"),
        Waypoint(1.5, (sx, hy, TRAVEL), PUSH_FINGERS, "behind the frame"),
        Waypoint(1.2, (sx, hy, LOW), PUSH_FINGERS, "down"),
        Waypoint(5.0, (ex, hy, LOW), PUSH_FINGERS, "push the hoist in round the chair"),
        Waypoint(0.6, (ex - 0.020, hy, LOW), PUSH_FINGERS, "back off"),
        Waypoint(1.0, (ex - 0.020, hy, TRAVEL), PUSH_FINGERS, "up"),
        Waypoint(1.2, (0.18, -0.22, TRAVEL), FINGER_CLOSED, "park"),
        Waypoint(3.0, (0.18, -0.22, TRAVEL), FINGER_CLOSED, "wait while it settles"),
    ]
    left += [Waypoint(sum(w.duration for w in right) - t_left, (0.18, 0.22, TRAVEL), FINGER_CLOSED, "wait")]
    return {"left": left, "right": right}
