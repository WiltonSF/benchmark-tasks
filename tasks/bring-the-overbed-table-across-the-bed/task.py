"""On the station table: a small model hospital bed lying left to right across the far side of the table, its frame on
four short legs with an open space under it, a mattress on top, and a patient lying along it under a green blanket,
head to the robot's right, hands resting on the blanket. Nearer the robot stands a model overbed table: a low orange
base, a grey column and a cream table top. Where and which way the overbed table stands changes between trials.
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

STEEL, DARK, WHITE, BLUE = (0.62, 0.64, 0.67), (0.25, 0.26, 0.28), (0.92, 0.92, 0.90), (0.55, 0.70, 0.85)
BLANKET, SKIN, CREAM, ORANGE = (0.35, 0.62, 0.40), (0.88, 0.70, 0.58), (0.93, 0.88, 0.74), (0.95, 0.50, 0.10)

# Bed (fixed), world frame. Long axis left to right (y), its near side facing the robot.
BED_X0, BED_X1, BED_Y = 0.360, 0.520, 0.165  # frame outline: x from 0.36 to 0.52, y within +-0.165
FRAME_UNDER, FRAME_T = 0.040, 0.015  # the frame slab's underside 40 mm above the table
MATTRESS_TOP = 0.080
BED_LEG = 0.020
PATIENT_X = (0.410, 0.470)
BODY_Y = (-0.100, 0.125)  # the blanketed body, from the shoulders to the feet
PATIENT_TOP = 0.105
HEAD = (0.040, 0.040, 0.030)  # at the robot's right (head) end, y -0.145 to -0.105
HANDS = (0.018, 0.040, 0.018)  # hands resting on the blanket, centred at y +0.025
HANDS_Y = 0.025

_bx, _by = (BED_X0 + BED_X1) / 2, 0.0
_bed_shapes = [
    parts.box((BED_X1 - BED_X0, 2 * BED_Y, FRAME_T), at=(_bx, _by, FRAME_UNDER + FRAME_T / 2), color=STEEL),
    parts.box((BED_X1 - BED_X0, 2 * BED_Y, MATTRESS_TOP - FRAME_UNDER - FRAME_T),
              at=(_bx, _by, (MATTRESS_TOP + FRAME_UNDER + FRAME_T) / 2), color=BLUE),
    parts.box((PATIENT_X[1] - PATIENT_X[0], BODY_Y[1] - BODY_Y[0], PATIENT_TOP - MATTRESS_TOP),
              at=(sum(PATIENT_X) / 2, sum(BODY_Y) / 2, (PATIENT_TOP + MATTRESS_TOP) / 2), color=BLANKET),
    parts.box(HEAD, at=(sum(PATIENT_X) / 2, -0.125, MATTRESS_TOP + HEAD[2] / 2), color=SKIN),
    parts.box(HANDS, at=(sum(PATIENT_X) / 2, HANDS_Y, PATIENT_TOP + HANDS[2] / 2), color=SKIN),
]
for sx in (BED_X0 + BED_LEG / 2, BED_X1 - BED_LEG / 2):
    for sy in (BED_Y - BED_LEG / 2, -BED_Y + BED_LEG / 2):
        _bed_shapes.append(parts.box((BED_LEG, BED_LEG, FRAME_UNDER), at=(sx, sy, FRAME_UNDER / 2), color=DARK))
BED = parts.Part("bed", pos=(0.0, 0.0, TOP), shapes=_bed_shapes)
PATIENT_SHAPES = BED.shapes[2:5]  # blanketed body, head, hands

# Overbed table (loose). Part frame: origin on the floor under the column's centre, x along the feet (from the column
# toward the feet's free ends), z up. A C-shaped base (a bar under the column and two feet from it), the column, and
# the top cantilevered from the column's top over the feet.
BASE_H = 0.018
BAR_X = (-0.040, 0.0125)
BAR_W = 0.150
FOOT_X = (0.0125, 0.1625)
FOOT_W, FOOT_Y = 0.020, 0.065  # feet 20 mm wide, centre lines 65 mm either side (110 mm apart inside)
COLUMN = 0.025
TOP_UNDER, TOP_T = 0.142, 0.008
TOP_X, TOP_W = (-0.0125, 0.1275), 0.110
TABLE_MASS = 0.35
CASTORS = (0.15, 0.15)  # low friction under the base stands in for castors
GRIP_BLOCK = (0.025, 0.040, 0.035)  # a grip block on the back of the base, behind the column
GRIP_X = BAR_X[0] - GRIP_BLOCK[0] / 2  # its centre, 52.5 mm behind the column's centre

FOOT_SHAPES = tuple(
    parts.box((FOOT_X[1] - FOOT_X[0], FOOT_W, BASE_H), at=(sum(FOOT_X) / 2, s * FOOT_Y, BASE_H / 2), color=ORANGE)
    for s in (1, -1))
TABLE_TOP_SHAPE = parts.box((TOP_X[1] - TOP_X[0], TOP_W, TOP_T), at=(sum(TOP_X) / 2, 0.0, TOP_UNDER + TOP_T / 2),
                            color=CREAM)
TOP_CENTRE = np.array([sum(TOP_X) / 2, 0.0, TOP_UNDER + TOP_T / 2])


def _table(pos, yaw):
    return parts.Part(
        "overbed_table", pos=pos, yaw=yaw, mass=TABLE_MASS, friction=CASTORS,
        shapes=[
            parts.box((BAR_X[1] - BAR_X[0], BAR_W, BASE_H), at=(sum(BAR_X) / 2, 0.0, BASE_H / 2), color=ORANGE),
            *FOOT_SHAPES,
            parts.box(GRIP_BLOCK, at=(GRIP_X, 0.0, GRIP_BLOCK[2] / 2), color=DARK),
            parts.box((COLUMN, COLUMN, TOP_UNDER - BASE_H), at=(0.0, 0.0, (TOP_UNDER + BASE_H) / 2), color=STEEL),
            TABLE_TOP_SHAPE,
        ])


# In position: the column 10 mm short of the bed's near side, in line with the hands.
HOME = (BED_X0 - COLUMN / 2 - 0.010, HANDS_Y)
START_TURNED = (0.190, -0.050, TOP)  # lot A: to the right, turned so the feet point left along the bed's side
YAW_TURNED = math.pi / 2
START_STRAIGHT = (0.200, HANDS_Y, TOP)  # lot B: in front of the bed, in line with the hands, feet toward it
TABLE = _table(START_TURNED, YAW_TURNED)
OBJECTS = [BED, TABLE]

LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"overbed_table": START_TURNED}),  # a zero-distance move: the lot still settles and refreshes
    lots.Lot("B", place={"overbed_table": START_STRAIGHT}, yaw={"overbed_table": 0.0}),
)

EPS = 1e-6
UNDER_IN = 0.040  # each foot's centre at least 40 mm inside the bed's outline
REACH = 0.060  # the top's centre within 60 mm of the hands along the bed
LEVEL = math.radians(5.0)
ON_FLOOR = 0.001  # the base's lowest point within 1 mm of the table at the end
LIFT = 0.020  # fault: the base's lowest point more than 20 mm above the table
TIP = math.radians(10.0)
PATIENT_GAP = 0.005  # fault: any part of the table within 5 mm of the patient
CLEAR = 0.005  # released: no gripper or finger within 5 mm of the table
RESET_TOL = 0.002
STILL_S = 1.0  # scored states: the table still (and let go) for at least 1 s
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


def _table_boxes(t):
    return [(c, r, s) for _, c, r, s in TABLE.world_boxes({"base": (np.asarray(t.pos), _rot(t.quat_xyzw))})]


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


_PATIENT_BOXES = [(np.array([0.0, 0.0, TOP]) + np.asarray(s.at), np.eye(3), 2 * s.half) for s in PATIENT_SHAPES]


def near_patient(t) -> bool:
    return any(_overlap(a, b, PATIENT_GAP) for a in _table_boxes(t) for b in _PATIENT_BOXES)


def _world(t, local) -> np.ndarray:
    return np.asarray(t.pos) + _rot(t.quat_xyzw) @ np.asarray(local, float)


def feet_under(t) -> bool:
    """Both feet's centres inside the bed's frame outline by 40 mm or more."""
    for s in (1, -1):
        c = _world(t, (sum(FOOT_X) / 2, s * FOOT_Y, BASE_H / 2))
        inside = min(c[0] - BED_X0, BED_X1 - c[0], BED_Y - abs(c[1]))
        if inside < UNDER_IN - EPS:
            return False
    return True


def top_centre(t) -> np.ndarray:
    return _world(t, TOP_CENTRE)


def top_over_bed(t) -> bool:
    c = top_centre(t)
    return bool(BED_X0 - EPS <= c[0] <= BED_X1 + EPS and abs(c[1]) <= BED_Y + EPS)


def hands_offset(t) -> float:
    return float(abs(top_centre(t)[1] - HANDS_Y))


# Whole-trial record: was the table ever lifted (base over 20 mm up), tipped over 10 degrees, or brought within 5 mm of
# the patient? Cleared only by a scene reset: the table back within 2 mm of its start and at rest, after a jump of over
# 20 mm in one control step (a robot cannot move it back that way).
_record = {"fault": False, "prev": None}
# Released: an on_step callback measures every step how close both arms' fingers and grippers come to the table.
# Offline (near_misses.py) there is no robot, so "clear" stays True.
# The same callback times how long the table has been still and let go (seconds); offline it counts as long enough.
_grip = {"clear": True, "still_s": math.inf, "t": None}


def _update(objects) -> None:
    t = objects["overbed_table"]
    pos = np.asarray(t.pos, float)
    prev = _record["prev"]
    start = np.asarray(lots.current().pos("overbed_table"), float)
    if prev is not None and np.linalg.norm(pos - start) < RESET_TOL and t.resting():
        if np.linalg.norm(pos - prev) > 0.020:
            _record["fault"] = False
    _record["prev"] = pos
    if t.low[2] > TOP + LIFT + EPS or _tilt(t) > TIP + EPS or near_patient(t):
        _record["fault"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    boxes = _table_boxes(sim.states()["overbed_table"])
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
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in boxes)
    t_prev, _grip["t"] = _grip["t"], float(sim.t)
    if t_prev is None or sim.t < t_prev or not (_grip["clear"] and sim.states()["overbed_table"].resting()):
        _grip["still_s"] = 0.0  # a new episode, a moving table or a gripper on it
    else:
        _grip["still_s"] += float(sim.t) - t_prev


def _settled(t) -> bool:
    return bool(t.resting() and _grip["clear"] and _grip["still_s"] >= STILL_S - EPS and t.low[2] <= TOP + ON_FLOOR + EPS and _tilt(t) <= LEVEL + EPS)


def _across(t) -> bool:
    return feet_under(t) and top_over_bed(t)


def success(objects):
    """The table across the bed: both feet under the frame, the top over the bed within reach of the hands, level,
    let go and still; never lifted, tipped or brought against the patient."""
    _update(objects)
    t = objects["overbed_table"]
    return bool(not _record["fault"] and _settled(t) and _across(t) and hands_offset(t) <= REACH + EPS)


def distance(objects):
    t = objects["overbed_table"]
    return float(np.linalg.norm(np.asarray(t.pos)[:2] - np.asarray(HOME)))


def score(objects):
    """Pass/fail: 1.0 on success, otherwise 0."""
    return 1.0 if success(objects) else 0.0


# --- demo -----------------------------------------------------------------------------------------------------
TIPZ = 0.013  # finger tips below the TCP
GRIP_Z = TOP + GRIP_BLOCK[2] + 0.001  # TCP 1 mm above the grip block's top: pads over its upper 12 mm
TRAVEL = TOP + 0.14
BACK = 0.030  # the gripper comes down and goes up this far behind the grip block, clear of the top's edge
WIDE = -0.032  # pads 64 mm apart, either side of the 40 mm block
PINCH = -(GRIP_BLOCK[1] / 2 - 0.003)  # close 3 mm past the block's faces (a full close pulls it off centre)
TURN_AT = (0.140, HANDS_Y)  # lot A: where the grip block is when the turn ends, far enough back that the feet clear
PUSH_PAST = 0.006  # commanded past the column's stop 10 mm short of the bed; the soft arm lags a few mm
PARK = (0.20, -0.22, TRAVEL)


def _grip_xy(pos, yaw):
    return (pos[0] + math.cos(yaw) * GRIP_X, pos[1] + math.sin(yaw) * GRIP_X)


def demo():
    """Right arm: grip the block on the back of the base, (lot A) turn the table so its feet point at the bed while
    drawing it in front of the hands, then push it in until the column is just short of the bed, let go. Where the
    table stands is what the cameras show for the trial's lot."""
    lot = lots.current()
    pos, yaw = lot.pos("overbed_table"), lot.yaw_of("overbed_table")
    gx, gy = _grip_xy(pos, yaw)
    hx, hy = HOME[0] + GRIP_X + PUSH_PAST, HOME[1]
    bx, by = gx - math.cos(yaw) * BACK, gy - math.sin(yaw) * BACK
    w = [
        Waypoint(1.5, (bx, by, TRAVEL), WIDE, "behind the grip block", yaw=yaw),
        Waypoint(1.2, (bx, by, GRIP_Z), WIDE, "down", yaw=yaw),
        Waypoint(0.8, (gx, gy, GRIP_Z), WIDE, "in round the block", yaw=yaw),
        Waypoint(0.8, (gx, gy, GRIP_Z), PINCH, "grip", yaw=yaw),
    ]
    if abs(yaw) > 1e-6:
        w.append(Waypoint(4.0, (*TURN_AT, GRIP_Z), PINCH, "turn the feet toward the bed", yaw=0.0))
    w += [
        Waypoint(5.0, (hx, hy, GRIP_Z), PINCH, "push the table in across the bed"),
        Waypoint(0.8, (hx, hy, GRIP_Z), WIDE, "let go"),
        Waypoint(0.8, (hx - BACK, hy, GRIP_Z), WIDE, "back off"),
        Waypoint(1.0, (hx - BACK, hy, TRAVEL), WIDE, "up"),
        Waypoint(1.2, PARK, FINGER_CLOSED, "park"),
        Waypoint(3.0, PARK, FINGER_CLOSED, "wait while it settles"),
    ]
    return {"right": w}
