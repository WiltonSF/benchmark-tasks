"""On the station table, in front of the robot's left arm: a bench radial drill on a dark grey base plate. A green arm
turns about a vertical column at the far right corner of the plate; it starts pointing toward the robot and swings
only clockwise seen from above, round toward the robot's left, where a yellow line on the plate marks its stop. A
red drill head slides along a slot in the arm, from the column out to the arm's end; it has a white knob on top and a
black spindle that hangs below the arm, lower than the tops of the parts on the plate. The head cannot be lifted off
the arm and the arm's height is fixed. Two long aluminium parts stand on the plate inside the arm's swing, each
lying along a line from the column. Where they stand changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots: its reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

GREEN, DARK, YELLOW = (0.25, 0.45, 0.32), (0.22, 0.23, 0.25), (0.95, 0.80, 0.10)
RED, WHITE, BLACK, ALU = (0.80, 0.12, 0.10), (0.95, 0.95, 0.95), (0.06, 0.06, 0.07), (0.70, 0.78, 0.88)

# World frame: x away from the arm bases, y to the robot's left, z up (m). The column's axis is vertical through P.
PX, PY = 0.350, 0.070
PLATE_T = 0.006
# Arm angle q (the hinge): 0 points the arm at the robot (world direction 180 deg, measured from +x toward +y);
# q increases clockwise seen from above, so the arm's world direction is 180 deg - q. Its stop is at Q_PARK.
Q_PARK = 110.0

ARM_BOT, ARM_T = 0.080, 0.012  # the arm's underside above the table, its thickness
ARM_TOP = ARM_BOT + ARM_T
ARM_MID = ARM_BOT + ARM_T / 2
RAIL_W, SLOT_W = 0.012, 0.020  # two rails either side of a 20 mm slot: the arm is 44 mm wide
ARM_END, ARM_TAIL = 0.250, 0.030  # arm length from the column's axis, and its tail behind the axis
SLOT_IN, SLOT_OUT = 0.072, 0.228  # slot ends, from the axis
STEM_R = 0.008
R_IN, R_OUT = SLOT_IN + STEM_R, SLOT_OUT - STEM_R  # 80 and 220 mm: the head's travel (spindle axis from the column)
R_START = 0.150
SPINDLE_R, SPINDLE_LOW = 0.010, 0.030  # black spindle, 20 mm across, its tip 30 mm above the table
COLUMN_R, COLUMN_TOP = 0.022, 0.140


def direction(q_deg: float) -> np.ndarray:
    """Unit vector (world x, y) along the arm at arm angle q (deg)."""
    a = math.radians(180.0 - q_deg)
    return np.array([math.cos(a), math.sin(a)])


def at(q_deg: float, r: float) -> tuple[float, float]:
    """World (x, y) of a point r from the column's axis along the arm at angle q."""
    p = np.array((PX, PY)) + r * direction(q_deg)
    return float(p[0]), float(p[1])


ARM = parts.Hinge(
    "swing",
    shapes=[  # written at q = 0: the arm along -x from the axis
        parts.box((ARM_END + ARM_TAIL, RAIL_W, ARM_T), at=(-(ARM_END - ARM_TAIL) / 2, SLOT_W / 2 + RAIL_W / 2, ARM_MID),
                  color=GREEN),
        parts.box((ARM_END + ARM_TAIL, RAIL_W, ARM_T), at=(-(ARM_END - ARM_TAIL) / 2, -SLOT_W / 2 - RAIL_W / 2, ARM_MID),
                  color=GREEN),
        parts.box((SLOT_IN + ARM_TAIL, SLOT_W, ARM_T), at=(-(SLOT_IN - ARM_TAIL) / 2, 0.0, ARM_MID), color=GREEN),
        parts.box((ARM_END - SLOT_OUT, SLOT_W, ARM_T), at=(-(ARM_END + SLOT_OUT) / 2, 0.0, ARM_MID), color=GREEN),
    ],
    pivot=(0.0, 0.0, ARM_MID),
    axis=(0.0, 0.0, -1.0),  # positive: clockwise seen from above
    lower=0.0,
    upper=math.radians(Q_PARK),
    mass=0.30,
    spring=0.0,
    damping=0.05,
)

WAYS_FRICTION = (0.3, 0.25)  # oiled steel ways: the drill (arm, column, plate) and the head
PLATE_X, PLATE_Y = (-0.250, 0.060), (-0.040, 0.260)  # base plate extent relative to the axis
_park = direction(Q_PARK)
DRILL = parts.Part(
    "radial_drill",
    pos=(PX, PY, TOP),
    shapes=[
        parts.box((PLATE_X[1] - PLATE_X[0], PLATE_Y[1] - PLATE_Y[0], PLATE_T),
                  at=(sum(PLATE_X) / 2, sum(PLATE_Y) / 2, PLATE_T / 2), color=DARK),
        parts.cylinder(COLUMN_R, COLUMN_TOP - PLATE_T, at=(0.0, 0.0, (COLUMN_TOP + PLATE_T) / 2), color=GREEN,
                       axis="z"),
    ],
    joints=[ARM],
    friction=WAYS_FRICTION,
)
# The yellow stop line: painted on the plate under the parked arm's centre line, 10 mm wide, from 40 to 240 mm out.
STOP_LINE = Box("stop_line", pos=(*at(Q_PARK, 0.140), TOP + PLATE_T + 0.0005), size=(0.200, 0.010, 0.001),
                yaw=math.radians(180.0 - Q_PARK), color=YELLOW)

# The drill head: a loose part captive on the arm. Origin on the spindle's axis at the top of the rails. A round top
# flange rests on the rails, a stem runs down through the slot, a round lower flange sits 3 mm under the rails (it
# stops the head being lifted off), and the spindle hangs below it.
FLANGE_R = 0.020
GAP = 0.003
LOW_FLANGE_T = 0.006
_low_flange_top = -(ARM_T + GAP)
_spindle_top = _low_flange_top - LOW_FLANGE_T  # 71 mm above the table
_spindle_h = (ARM_TOP + _spindle_top) - SPINDLE_LOW
KNOB_R, KNOB_H = 0.011, 0.030
HEAD_MASS = 0.15
HEAD_SHAPES = [
    parts.cylinder(FLANGE_R, 0.008, at=(0.0, 0.0, 0.004), color=RED, axis="z"),
    parts.cylinder(KNOB_R, KNOB_H, at=(0.0, 0.0, 0.008 + KNOB_H / 2), color=WHITE, axis="z"),
    parts.cylinder(STEM_R, ARM_T + GAP, at=(0.0, 0.0, -(ARM_T + GAP) / 2), color=RED, axis="z"),
    parts.cylinder(FLANGE_R, LOW_FLANGE_T, at=(0.0, 0.0, _low_flange_top - LOW_FLANGE_T / 2), color=RED, axis="z"),
    parts.cylinder(SPINDLE_R, _spindle_h, at=(0.0, 0.0, _spindle_top - _spindle_h / 2), color=BLACK, axis="z"),
]
HEAD = parts.Part("drill_head", pos=(*at(0.0, R_START), TOP + ARM_TOP), mass=HEAD_MASS, shapes=HEAD_SHAPES,
                  friction=WAYS_FRICTION)

# Two aluminium parts, 100 mm long, 30 mm wide, 50 mm tall, each standing along a line from the column's axis, in
# the inner band (its ends 62.5 and 162.5 mm from the axis) or the outer band (137 and 237 mm).
WP_SIZE = (0.100, 0.030, 0.050)
WP_MASS = 0.40
INNER, OUTER = 0.1125, 0.187  # centre distances from the axis
Q_FIRST, Q_SECOND = 25.0, 75.0  # arm angles of the two parts' lines
WP_Z = TOP + PLATE_T + WP_SIZE[2] / 2


def wp_place(q_deg: float, r: float) -> tuple[float, float, float]:
    return (*at(q_deg, r), WP_Z)


def wp_yaw(q_deg: float) -> float:
    return math.radians(180.0 - q_deg)


PART_A = Box("part_a", pos=wp_place(Q_FIRST, INNER), size=WP_SIZE, yaw=wp_yaw(Q_FIRST), mass=WP_MASS, color=ALU)
PART_B = Box("part_b", pos=wp_place(Q_SECOND, OUTER), size=WP_SIZE, yaw=wp_yaw(Q_SECOND), mass=WP_MASS, color=ALU)
WORKPIECES = ("part_a", "part_b")

OBJECTS = [DRILL, STOP_LINE, HEAD, PART_A, PART_B]

# Lot A (the screenshot): the first part (25 deg into the swing) in the inner band, the second (75 deg) in the
# outer band. Lot B: both inner. Lot C: both outer. `bands` lists each part's (arm angle, band) for the demo.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"part_a": wp_place(Q_FIRST, INNER), "part_b": wp_place(Q_SECOND, OUTER)},
             bands=((Q_FIRST, "inner"), (Q_SECOND, "outer"))),
    lots.Lot("B", place={"part_a": wp_place(Q_FIRST, INNER), "part_b": wp_place(Q_SECOND, INNER)},
             bands=((Q_FIRST, "inner"), (Q_SECOND, "inner"))),
    lots.Lot("C", place={"part_a": wp_place(Q_FIRST, OUTER), "part_b": wp_place(Q_SECOND, OUTER)},
             bands=((Q_FIRST, "outer"), (Q_SECOND, "outer"))),
)

# -- scoring -------------------------------------------------------------------------------------------------------
EPS = 1e-6
PARK_TOL = 2.0  # deg: parked, the arm within 2 deg of its stop
IN_TOL = 0.005  # run in: the spindle's axis within 5 mm of its innermost position
MOVE_LIMIT, TILT_LIMIT = 0.003, 3.0  # a part moved more than 3 mm or tilted more than 3 deg was struck
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the head or the arm
SUCCESS_HOLD_S = 1.5


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(q) -> float:
    return math.degrees(math.acos(float(np.clip(_rot(q)[2, 2], -1.0, 1.0))))


def arm_deg(objects) -> float:
    return math.degrees(objects["radial_drill"].joints["swing"])


def head_radius(objects) -> float:
    """Horizontal distance of the spindle's axis from the column's axis (m)."""
    return float(np.linalg.norm(np.asarray(objects["drill_head"].pos[:2]) - np.array((PX, PY))))


def _struck(objects) -> bool:
    lot = lots.current()
    for n in WORKPIECES:
        s = objects[n]
        if np.linalg.norm(np.asarray(s.pos) - np.asarray(lot.pos(n))) > MOVE_LIMIT + EPS:
            return True
        if _tilt_deg(s.quat_xyzw) > TILT_LIMIT + EPS:
            return True
    return False


# Whole-trial record: was either part ever moved over 3 mm or tilted over 3 deg? Cleared only by a scene reset: the
# head jumps over 20 mm in one control step (a robot cannot move it that fast) and lands within 0.5 mm of its start,
# at rest, with the arm within 0.5 deg of its start and both parts within 0.5 mm of their starts.
_record = {"fault": False, "prev": None}
# Released: in the simulator an on_step callback measures, every step, how close both arms' fingers and grippers are
# to the head and the arm. Offline (near_misses.py) there is no robot, so "clear" stays True.
_grip = {"clear": True}


def _at_start(objects) -> bool:
    lot = lots.current()
    head = objects["drill_head"]
    if np.linalg.norm(np.asarray(head.pos) - np.asarray(HEAD.pos)) > 0.0005 or not head.resting():
        return False
    if abs(arm_deg(objects)) > 0.5:
        return False
    return all(np.linalg.norm(np.asarray(objects[n].pos) - np.asarray(lot.pos(n))) <= 0.0005 for n in WORKPIECES)


def _update(objects) -> None:
    pos = np.asarray(objects["drill_head"].pos, float)
    prev = _record["prev"]
    if prev is not None and np.linalg.norm(pos - prev) > 0.020 and _at_start(objects):
        _record["fault"] = False
    _record["prev"] = pos
    if _struck(objects):
        _record["fault"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    head, drill = st["drill_head"], st["radial_drill"]
    boxes = HEAD.world_boxes({"base": (np.asarray(head.pos), _rot(head.quat_xyzw))})
    lp, lq = drill.links["swing"]
    boxes += DRILL.world_boxes({"swing": (np.asarray(lp), _rot(lq))})
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
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for _, c, r, s in boxes)


def _parked(objects) -> bool:
    """The arm within 2 deg of its stop, let go (no gripper within 5 mm of the head or arm) and still."""
    return bool(arm_deg(objects) >= Q_PARK - PARK_TOL - EPS and _grip["clear"] and objects["drill_head"].resting())


def _run_in(objects) -> bool:
    return head_radius(objects) <= R_IN + IN_TOL + EPS


def success(objects):
    """The arm is at its stop, the head run in to the column, both let go and still; neither part was ever struck."""
    _update(objects)
    return bool(not _record["fault"] and _parked(objects) and _run_in(objects))


def distance(objects):
    return float(max(0.0, math.radians(Q_PARK - arm_deg(objects))) * R_IN + max(0.0, head_radius(objects) - R_IN))


def score(objects):
    """1.0 parked with the head run in; 0.5 parked with the head not run in; 0 if a part was ever struck."""
    _update(objects)
    if _record["fault"] or not _parked(objects):
        return 0.0
    return 1.0 if _run_in(objects) else 0.5


# -- demo: left arm only -------------------------------------------------------------------------------------------
# Fixed waypoints taken from the trial's lot (where the parts stand; the top camera shows the same layout): grip the head's
# knob, run the head out past any part in the inner band or in past any part in the outer band before the swing
# reaches it, change over between the parts if needed, swing to the stop, run the head in, let go.
TIP = 0.013  # finger tips below the TCP
GRIP_Z = TOP + ARM_TOP + 0.008 + 0.006 + TIP  # tips 6 mm up the 30 mm knob: a low grip tips the head less
TRAVEL_Z = TOP + 0.150
PINCH = -(KNOB_R - 0.003)  # pads 3 mm inside the knob's faces
STEP = 5.0  # deg per swing waypoint
PUSH_PAST = 4.0  # deg commanded past the stop (the arm is soft)
Q_RUN_IN = Q_PARK - 5.0
HOLD_IN = 0.025  # m commanded past the slot's inner end: the loaded arm lags by about 20 mm near the stop


def _wp(q, r, finger, label, dur, yaw=0.0):
    return Waypoint(dur, (*at(q, r), GRIP_Z), finger, label, yaw)


def _radial_yaw(q):
    """Grip yaw that closes the pads along the arm at angle q, so a radial slide pushes on a pad's face instead of
    relying on the pads' friction (yaw 0 closes them along world y). At the arm's start the wrist cannot turn that far,
    so there the pads stay across the arm and the head slides by their grip."""
    return 0.0 if q < 20.0 else math.radians(90.0 - q)


def _radial(q, r0, r1, label):
    """Turn the grip along the arm (the round head turns with it in its slot), slide the head from r0 to r1, hold
    it against the slot's end if that is where it goes, and turn the grip back."""
    y = _radial_yaw(q)
    out = [_wp(q, r0, PINCH, "turn the grip along the arm", 0.8, y),
           _wp(q, r1, PINCH, label, 1.8, y)]
    if r1 == R_IN:
        out.append(_wp(q, R_IN - HOLD_IN, PINCH, "hold in", 1.0, y))
    return out


def _swing(q0, q1, r):
    n = max(1, math.ceil(abs(q1 - q0) / STEP))
    return [_wp(q0 + (q1 - q0) * (k + 1) / n, r, PINCH, f"swing to {q0 + (q1 - q0) * (k + 1) / n:.0f}", 0.45)
            for k in range(n)]


def demo():
    """Left arm only: grip the knob, set the head's radius clear of each part before the swing reaches it, swing to
    the stop, run the head in, let go and lift away."""
    bands = lots.current().bands
    x0, y0 = at(0.0, R_START)
    plan = [
        Waypoint(1.5, (x0, y0, TRAVEL_Z), FINGER_OPEN, "over the knob"),
        Waypoint(1.5, (x0, y0, GRIP_Z), FINGER_OPEN, "down to the knob"),
        Waypoint(0.8, (x0, y0, GRIP_Z), PINCH, "grip"),
    ]
    q, r, prev = 0.0, R_START, 0.0
    for q_part, band in bands:
        need = R_OUT if band == "inner" else R_IN
        if need != r:
            q_change = prev + 0.56 * (q_part - prev) if prev > 0 else 0.0  # nearer the next part: the inner one has more to clear
            plan += _swing(q, q_change, r) if q_change > q else []
            plan += _radial(q_change, r, need, "run out" if need > r else "run in")
            plan.append(_wp(q_change, need, PINCH, "turn the grip back", 0.8))
            q, r = q_change, need
        prev = q_part
    if r != R_IN:  # past the last part: run in 5 deg short of the stop (30 deg past the part's line), then swing on
        plan += _swing(q, Q_RUN_IN, r)
        plan += _radial(Q_RUN_IN, r, R_IN, "run in")
        plan.append(_wp(Q_RUN_IN, R_IN, PINCH, "turn the grip back", 0.8))
        q, r = Q_RUN_IN, R_IN
    plan += _swing(q, Q_PARK + PUSH_PAST, r)
    q = Q_PARK + PUSH_PAST
    plan.append(_wp(q, R_IN - HOLD_IN, PINCH, "hold in", 0.6))
    xe, ye = at(q, R_IN)
    y = plan[-1].yaw
    plan += [
        Waypoint(0.6, (xe, ye, GRIP_Z), FINGER_OPEN, "release", y),
        Waypoint(1.2, (xe, ye, TRAVEL_Z), FINGER_OPEN, "up", y),
        Waypoint(1.0, (0.25, 0.26, TRAVEL_Z), FINGER_OPEN, "away"),
        Waypoint(2.5, (0.25, 0.26, TRAVEL_Z), FINGER_OPEN, "wait"),
    ]
    return {"left": plan}
