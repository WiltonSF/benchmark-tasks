"""On the station table, in front of the arms: a model dishwasher with its door folded down flat toward the robot
and no top, so its tub is open from above. Two white rails run front to back along the tub's side walls. A dark
heater bar lies across the tub floor between the rails, near the back wall. The lower rack, a grey wire basket on
four black wheels, stands in the front half of the tub.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so on_reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, TUB_FLOOR, WHITE = (0.78, 0.79, 0.80), (0.33, 0.34, 0.36), (0.97, 0.97, 0.95)
DARK, HEATER = (0.22, 0.23, 0.25), (0.55, 0.18, 0.10)
WIRE, BLACK = (0.62, 0.64, 0.67), (0.06, 0.06, 0.07)

# Tub, world frame (m): x away from the arm bases, y to the robot's left, z up.
FLOOR_T = 0.004
FLOOR = TOP + FLOOR_T  # the tub floor's top
TUB_X0, TUB_X1 = 0.14, 0.47  # open front, back wall's inner face
WALL_Y = 0.15  # side walls' inner faces at y = +-0.15
WALL_T, WALL_H = 0.010, 0.080
RAIL_IN, RAIL_H = 0.115, 0.030  # each rail runs from |y| 0.115 (inner edge) to the wall, its top 30 mm up
RAIL_TOP = FLOOR + RAIL_H
HEATER_X, HEATER_DX, HEATER_H = 0.400, 0.015, 0.020  # across the floor between the rails' inner edges
DOOR = (0.110, 0.32, 0.006)  # folded-down door in front of the tub
LIP_W, LIP_T = 0.015, 0.005  # each lip reaches 15 mm in from its wall
LIP_Z = RAIL_H + 0.030 + 0.003  # its underside 3 mm above the top of a wheel standing on the rail

TUB = parts.Part(
    "dishwasher",
    pos=(0.0, 0.0, TOP),
    shapes=[
        parts.box((TUB_X1 - TUB_X0, 2 * WALL_Y, FLOOR_T), at=((TUB_X0 + TUB_X1) / 2, 0.0, FLOOR_T / 2), color=TUB_FLOOR),
        *[parts.box((TUB_X1 + WALL_T - TUB_X0, WALL_T, WALL_H),
                    at=((TUB_X0 + TUB_X1 + WALL_T) / 2, s * (WALL_Y + WALL_T / 2), WALL_H / 2), color=STEEL)
          for s in (1, -1)],
        parts.box((WALL_T, 2 * (WALL_Y + WALL_T), WALL_H), at=(TUB_X1 + WALL_T / 2, 0.0, WALL_H / 2), color=STEEL),
        *[parts.box((TUB_X1 - TUB_X0, WALL_Y - RAIL_IN, RAIL_H),
                    at=((TUB_X0 + TUB_X1) / 2, s * (WALL_Y + RAIL_IN) / 2, FLOOR_T + RAIL_H / 2), color=WHITE)
          for s in (1, -1)],
        parts.box((HEATER_DX, 2 * RAIL_IN, HEATER_H), at=(HEATER_X, 0.0, FLOOR_T + HEATER_H / 2), color=HEATER),
        parts.box(DOOR, at=(TUB_X0 - DOOR[0] / 2, 0.0, DOOR[2] / 2), color=DARK),
        # Retaining lips along each wall over the rails: a wheel against its wall cannot be lifted out.
        *[parts.box((TUB_X1 - TUB_X0, LIP_W, LIP_T), at=((TUB_X0 + TUB_X1) / 2, s * (WALL_Y - LIP_W / 2),
                                                       FLOOR_T + LIP_Z + LIP_T / 2), color=STEEL)
          for s in (1, -1)],
    ],
)
RAILS = TUB.shapes[4:6]
HEATER_SHAPE = TUB.shapes[6]

# Rack. Part frame: origin at the rack's centre at wheel-axle height, x front to back, z up.
R_WHEEL, W_WHEEL = 0.015, 0.010
WHEEL_X, WHEEL_Y = 0.075, 0.125  # wheel centres; the wheels span |y| 0.12 to 0.13
SIDE_Y, SIDE_R = 0.105, 0.006  # round side bars along x
HALF_L = 0.090  # front and rear bars' outer faces at x -+0.09
RACK_MASS, WHEEL_MASS = 0.24, 0.015
ON_RAILS_Z = RAIL_TOP + R_WHEEL  # the origin's height with all four wheels on the rails

_frame = [
    *[parts.cylinder(SIDE_R, 2 * HALF_L, at=(0.0, s * SIDE_Y, 0.0), color=WIRE, axis="x") for s in (1, -1)],
    *[parts.box((0.010, 2 * SIDE_Y, 0.010), at=(s * (HALF_L - 0.005), 0.0, 0.0), color=WIRE) for s in (1, -1)],
    *[parts.box((2 * HALF_L - 0.02, 0.004, 0.004), at=(0.0, y, -0.002), color=WIRE) for y in (0.05, 0.0, -0.05)],
    *[parts.cylinder(0.003, 2 * (WHEEL_Y - W_WHEEL / 2 - SIDE_Y), at=(sx * WHEEL_X, sy * (WHEEL_Y + SIDE_Y - W_WHEEL / 2) / 2,
                                                                       0.0), color=WIRE, axis="y")
      for sx in (1, -1) for sy in (1, -1)],  # stub axles
    *[parts.cylinder(0.002, 0.040, at=(x, y, 0.020), color=WIRE, axis="z")
      for x in (-0.035, 0.035) for y in (-0.07, -0.035, 0.0, 0.035, 0.07)],  # tines
]
WHEELS = {f"wheel_{fb}_{lr}": (sx * WHEEL_X, sy * WHEEL_Y)
          for fb, sx in (("front", -1), ("rear", 1)) for lr, sy in (("left", 1), ("right", -1))}

START_Y = 0.0195  # the rack shoved this far against one wall (its wheels' outer faces 0.5 mm from it)
START_Z = ON_RAILS_Z + 0.005  # set down from 5 mm up
LOT_A_XY = (0.28, -START_Y)  # against the right wall: its left wheels miss their rail and drop
LOT_B_XY = (0.25, START_Y)  # against the left wall: its right wheels drop

RACK = parts.Part(
    "rack",
    pos=(LOT_A_XY[0], 0.0, START_Z),  # declared centred on the rails; each lot then shoves it against a wall
    mass=RACK_MASS,
    shapes=_frame,
    joints=[parts.Hinge(n, shapes=[parts.cylinder(R_WHEEL, W_WHEEL, at=(x, y, 0.0), color=BLACK, axis="y")],
                        pivot=(x, y, 0.0), axis=(0.0, 1.0, 0.0), lower=-40.0, upper=40.0, mass=WHEEL_MASS,
                        damping=0.0005)
            for n, (x, y) in WHEELS.items()],
)

OBJECTS = [TUB, RACK]

# Lot B (the first trial: the screenshot and videos): right side down, nearer the door. Lot A: left side down.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("B", place={"rack": (*LOT_B_XY, START_Z)}, down="right"),  # first: the screenshot and videos
    lots.Lot("A", place={"rack": (*LOT_A_XY, START_Z)}, down="left"),
)

EPS = 1e-6
ON_RAIL_TOL = 0.003  # a wheel's lowest point at most 3 mm above (or 1 mm into) its rail's top
LEVEL = math.radians(2.0)
HOME_GAP = 0.015  # the rack's back within 15 mm of the back wall
HEATER_TOUCH = 0.002
CLEAR = 0.005  # let go: no gripper part within 5 mm of the rack
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


def _wheel_centres(rack) -> dict[str, np.ndarray]:
    r = _rot(rack.quat_xyzw)
    return {n: np.asarray(rack.pos) + r @ np.array([x, y, 0.0]) for n, (x, y) in WHEELS.items()}


def wheel_on_rail(rack, name) -> bool:
    """The wheel's centre is over its own rail's top (between the rail's inner edge and the wall) within the rail's
    length, and its lowest point is at most 3 mm above the rail top."""
    c = _wheel_centres(rack)[name]
    side = 1.0 if WHEELS[name][1] > 0 else -1.0
    bottom = c[2] - R_WHEEL
    return bool(
        RAIL_IN - EPS <= side * c[1] <= WALL_Y + EPS
        and TUB_X0 - EPS <= c[0] <= TUB_X1 + EPS
        and RAIL_TOP - 0.001 - EPS <= bottom <= RAIL_TOP + ON_RAIL_TOL + EPS
    )


def back_gap(rack) -> float:
    """Distance from the rack's rear bar (middle of its back face) to the back wall's inner face."""
    p = np.asarray(rack.pos) + _rot(rack.quat_xyzw) @ np.array([HALF_L, 0.0, 0.0])
    return float(TUB_X1 - p[0])


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
        ra_ = sum(ha[i] * abs(ax @ ra[:, i]) for i in range(3))
        rb_ = sum(hb[i] * abs(ax @ rb[:, i]) for i in range(3))
        if abs(ax @ d) > ra_ + rb_:
            return False
    return True


def _rack_boxes(rack):
    """Every shape of the rack, wheels included, as (centre, rotation, size). Wheels are taken as their boxes, which
    is exact for the heater check: a wheel meets the heater's upright face with its rim."""
    r = _rot(rack.quat_xyzw)
    p = np.asarray(rack.pos)
    poses = {"base": (p, r)}
    for j in RACK.joints:
        poses[j.name] = (p + r @ j.origin, r)
    return [(c, rr, s) for _, c, rr, s in RACK.world_boxes(poses)]


_HEATER_BOX = (np.array([0.0, 0.0, TOP]) + np.asarray(HEATER_SHAPE.at), np.eye(3), 2 * HEATER_SHAPE.half)


def touches_heater(rack) -> bool:
    return any(_overlap(b, _HEATER_BOX, HEATER_TOUCH) for b in _rack_boxes(rack))


# Whole-trial record: did the rack ever come within 2 mm of the heater? Cleared by a scene reset only: the on_reset
# hook in the simulator, or offline the rack back within 0.5 mm of its declared start and at rest after a jump
# of over 20 mm in one control step (a robot cannot move it that way).
_record = {"fault": False, "prev": None}
# Let go: measured every step in the simulator by an on_step callback (gripper and finger hulls to the rack's shapes).
# Offline there is no robot, so it stays True.
_grip = {"clear": True}


@parts.on_reset
def _new_trial(sim) -> None:
    _record["fault"] = False
    _record["prev"] = None


def _update(objects) -> None:
    rack = objects["rack"]
    pos = np.asarray(rack.pos, float)
    prev = _record["prev"]
    start = np.asarray(RACK.pos, float)  # where a scene reset puts it, before the lot's shove
    if prev is not None and np.linalg.norm(pos - start) < 0.0005 and rack.resting():
        if np.linalg.norm(pos - prev) > 0.020:
            _record["fault"] = False
    _record["prev"] = pos
    if touches_heater(rack):
        _record["fault"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    rack = sim.states()["rack"]
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
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in _rack_boxes(rack))


def _seated(rack) -> bool:
    """All four wheels on their rails, the rack level within 2 degrees, let go and still."""
    return bool(all(wheel_on_rail(rack, n) for n in WHEELS) and _tilt(rack) <= LEVEL + EPS and rack.resting()
                and _grip["clear"])


def success(objects):
    """All four wheels on their rails, the rack level and home against the back wall, let go and still; the rack
    never touched the heater."""
    _update(objects)
    rack = objects["rack"]
    return bool(not _record["fault"] and _seated(rack) and back_gap(rack) <= HOME_GAP + EPS)


def distance(objects):
    return max(back_gap(objects["rack"]), 0.0)


def score(objects):
    """1.0 on success; 0.5 with all four wheels on their rails, level, let go and still but not home; else 0. 0 for
    the rest of the trial once the rack touches the heater."""
    _update(objects)
    rack = objects["rack"]
    if _record["fault"] or not _seated(rack):
        return 0.0
    return 1.0 if back_gap(rack) <= HOME_GAP + EPS else 0.5


# --- demo -------------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
GAP_OPEN = -0.012  # pads 24 mm apart around the 12 mm side bar
GAP_GRIP = -0.003  # closed 3 mm past each face of the bar
TRAVEL = TOP + 0.14
# The dropped side settles with its wheels on the floor and the rack turned about its other wheels.
DROP = math.asin(RAIL_H / (2 * WHEEL_Y))
LIFT = 0.042  # TCP rise: the dropped wheels' bottoms end about 15 mm above the rail tops
SHIFT = START_Y + 0.0005  # back to the centre: each wheel's centre over its rail by 10 mm
HOME_X = TUB_X1 - HALF_L  # the rack's centre when home
FINGER_FRONT = 0.0135  # the closed fingers' front faces ahead of the TCP


def _settled_bar(lot) -> tuple[float, float, float]:
    """Where the dropped side's bar settles: the rack turned by DROP about its other wheels' centres."""
    x, y, _ = lot.pos("rack")
    s = 1.0 if lot.down == "left" else -1.0
    pivot_y = y - s * WHEEL_Y
    lever = WHEEL_Y + SIDE_Y
    return (x, pivot_y + s * lever * math.cos(DROP), ON_RAILS_Z - lever * math.sin(DROP))


def demo():
    """The arm on the side that is down grips that side's bar, lifts it until its wheels clear the rail, moves it
    out over the rail, sets it down and lets go; then it pushes the rack's front bar with closed fingers until the
    rack stops at the back wall, and backs away. Which side is down, and where the rack is, are what the cameras
    show for the trial's lot."""
    lot = lots.current()
    arm = lot.down
    s = 1.0 if arm == "left" else -1.0
    bx, by, bz = _settled_bar(lot)
    grip_z = bz - 0.004 + TIP  # tips 4 mm below the bar's centre
    seat_y = by + s * SHIFT
    seat_z = ON_RAILS_Z - 0.004 + TIP + 0.006  # the wheels released 6 mm above the rails
    rx = lot.pos("rack")[0]
    front = rx - HALF_L  # front bar's front face
    push_z = ON_RAILS_Z + TIP
    py = 0.0
    w = [
        Waypoint(1.5, (bx, by, TRAVEL), GAP_OPEN, "over the dropped side's bar"),
        Waypoint(1.5, (bx, by, grip_z), GAP_OPEN, "down beside the bar"),
        Waypoint(0.8, (bx, by, grip_z), GAP_GRIP, "grip"),
        Waypoint(1.5, (bx, by, grip_z + LIFT), GAP_GRIP, "lift the side until its wheels clear the rail"),
        Waypoint(1.5, (bx, seat_y, grip_z + LIFT), GAP_GRIP, "out over the rail"),
        Waypoint(1.5, (bx, seat_y, seat_z), GAP_GRIP, "down to just above the rail"),
        Waypoint(0.6, (bx, seat_y, seat_z), GAP_OPEN, "let go"),
        Waypoint(1.0, (bx, seat_y, TRAVEL), GAP_OPEN, "up"),
        Waypoint(1.5, (front - FINGER_FRONT - 0.02, py, TRAVEL), FINGER_CLOSED, "in front of the rack"),
        Waypoint(1.5, (front - FINGER_FRONT - 0.02, py, push_z), FINGER_CLOSED, "down to the front bar"),
        Waypoint(5.0, (HOME_X - HALF_L - FINGER_FRONT + 0.006, py, push_z), FINGER_CLOSED, "push the rack home"),
        Waypoint(1.0, (HOME_X - HALF_L - FINGER_FRONT - 0.04, py, push_z), FINGER_CLOSED, "back off"),
        Waypoint(1.0, (HOME_X - HALF_L - FINGER_FRONT - 0.04, py, TRAVEL), FINGER_CLOSED, "up"),
        Waypoint(2.0, (HOME_X - HALF_L - FINGER_FRONT - 0.04, py, TRAVEL), FINGER_CLOSED, "wait"),
    ]
    return {arm: w}
