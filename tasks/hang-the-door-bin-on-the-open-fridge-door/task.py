"""On the station table: an open fridge door standing on its own, its inner liner toward the robot, with two grey
channels on the liner. Which way the door is turned changes between trials. A pale blue door bin, an open box with
two tabs on its back, stands on the table in front of one of the arms.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so on_reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WHITE, GASKET, CHANNEL, BIN = (0.94, 0.94, 0.92), (0.30, 0.31, 0.33), (0.55, 0.57, 0.60), (0.62, 0.80, 0.95)

# Door. Part frame: origin on the table under the middle of the liner face; x' out of the liner away from the robot
# (into the door), y' along the door to its left end (at yaw 0, the robot's left), z up. The liner face is x' = 0.
DOOR_W, DOOR_T, DOOR_H = 0.24, 0.030, 0.140
FOOT = (0.12, DOOR_W, 0.006)  # a base plate behind the liner, so the door stands
CH_Y = 0.080  # channel centres at y' = +-0.08
CH_Z0, CH_Z1 = 0.030, 0.090  # each channel block from 30 to 90 mm up
STOP_T = 0.005
STOP_Z = CH_Z0 + STOP_T  # the stops' tops, 35 mm up
LIP_X0, LIP_X1 = -0.018, -0.012  # front lips (6 mm thick) in front of the pocket
POCKET_HALF_W, NECK_HALF_W, CH_HALF_W = 0.0125, 0.0065, 0.016
DOOR_MASS = 4.0  # heavy so it stands; a loose part only so the lots can turn it


def _channel(c):
    h = CH_Z1 - STOP_Z
    zc = STOP_Z + h / 2
    out = [parts.box((-LIP_X0, 2 * CH_HALF_W, STOP_T), at=(LIP_X0 / 2, c, CH_Z0 + STOP_T / 2), color=CHANNEL)]
    for s in (1, -1):
        wall_w = CH_HALF_W - POCKET_HALF_W
        out.append(parts.box((-LIP_X1, wall_w, h), at=(LIP_X1 / 2, c + s * (POCKET_HALF_W + wall_w / 2), zc),
                             color=CHANNEL))
        lip_w = CH_HALF_W - NECK_HALF_W
        out.append(parts.box((LIP_X1 - LIP_X0, lip_w, h), at=((LIP_X0 + LIP_X1) / 2, c + s * (NECK_HALF_W + lip_w / 2),
                                                              zc), color=CHANNEL))
    return out


DOOR_SHAPES = [
    parts.box(FOOT, at=(FOOT[0] / 2, 0.0, FOOT[2] / 2), color=GASKET),
    parts.box((DOOR_T, DOOR_W, DOOR_H), at=(DOOR_T / 2, 0.0, FOOT[2] + DOOR_H / 2), color=WHITE),
    *[parts.box((0.004, DOOR_W, 0.008), at=(-0.002, 0.0, FOOT[2] + z), color=GASKET) for z in (0.004, DOOR_H - 0.004)],
    *_channel(CH_Y), *_channel(-CH_Y),
]

DOOR_XY = (0.36, 0.0)
DOOR = parts.Part("fridge_door", pos=(*DOOR_XY, TOP), mass=DOOR_MASS, shapes=DOOR_SHAPES)

# Bin. Part frame: origin at the middle of its underside; x toward its back (the tabs), y along it, z up.
BIN_L, BIN_D, BIN_H, BIN_WALL = 0.175, 0.060, 0.050, 0.006
NECK = (0.012, 0.005, BIN_H)  # each tab, the bin's full height: a neck 6 mm longer than the lips are thick...
HEAD = (0.006, 0.018, BIN_H)  # ...then a head
NOTCH_HALF_W, NOTCH_H = 0.019, 0.020  # the back wall is cut away 20 mm up around each tab, clear of the lips
BIN_MASS = 0.15
_back = BIN_D / 2
HEAD_X = _back + NECK[0] + HEAD[0] / 2  # head centres in the bin frame
HEAD_ZC = HEAD[2] / 2

BIN_SHAPES = [
    parts.box((BIN_D - 2 * BIN_WALL, BIN_L - 2 * BIN_WALL, BIN_WALL), at=(0.0, 0.0, BIN_WALL / 2), color=BIN),  # floor
    parts.box((BIN_WALL, BIN_L, BIN_H), at=(-_back + BIN_WALL / 2, 0.0, BIN_H / 2), color=BIN),  # front
    parts.box((BIN_WALL, 2 * (CH_Y - NOTCH_HALF_W), BIN_H), at=(_back - BIN_WALL / 2, 0.0, BIN_H / 2), color=BIN),
    *[parts.box((BIN_WALL, BIN_L / 2 - CH_Y + NOTCH_HALF_W, BIN_H - NOTCH_H),
                at=(_back - BIN_WALL / 2, s * (BIN_L / 2 + CH_Y - NOTCH_HALF_W) / 2, (BIN_H + NOTCH_H) / 2), color=BIN)
      for s in (1, -1)],  # the back wall's ends, above the notches
    *[parts.box((BIN_D - 2 * BIN_WALL, BIN_WALL, BIN_H), at=(0.0, s * (BIN_L - BIN_WALL) / 2, BIN_H / 2), color=BIN)
      for s in (1, -1)],
]
for _s in (1, -1):
    BIN_SHAPES += [
        parts.box(NECK, at=(_back + NECK[0] / 2, _s * CH_Y, NECK[2] / 2), color=WHITE),
        parts.box(HEAD, at=(HEAD_X, _s * CH_Y, HEAD_ZC), color=WHITE),
    ]

BIN_XY = (0.20, -0.12)
DOOR_BIN = parts.Part("door_bin", pos=(*BIN_XY, TOP), mass=BIN_MASS, yaw=math.radians(20.0), shapes=BIN_SHAPES)

OBJECTS = [DOOR, DOOR_BIN]

# Lot C (the first trial: the screenshot and videos): the door turned 40 degrees, its right end toward the robot.
# A: square to the arms. B: turned 30 degrees the other way. The bin's start place and yaw change too.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("C", yaw={"fridge_door": math.radians(-40.0), "door_bin": 0.0}, place={"door_bin": (0.20, 0.12, TOP)},
             arm="left"),  # first: the screenshot and videos
    lots.Lot("A", place={"fridge_door": (*DOOR_XY, TOP)}, arm="right"),
    lots.Lot("B", yaw={"fridge_door": math.radians(30.0), "door_bin": math.radians(-30.0)},
             place={"door_bin": (0.21, -0.14, TOP)}, arm="right"),
)

EPS = 1e-6
ON_STOP = 0.003  # a head's underside within 3 mm above its stop
UPRIGHT = math.radians(10.0)  # a hung bin leans forward on its tabs by up to about 7 degrees
IN_DEPTH = 0.010
SQUARE = math.radians(3.0)
CLEAR = 0.005
SUCCESS_HOLD_S = 1.0


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def heads_in_door(objects) -> list[np.ndarray]:
    """Each tab head's centre in the door frame (x' into the door, y' along it, z above the table)."""
    door, b = objects["fridge_door"], objects["door_bin"]
    rd, rb = _rot(door.quat_xyzw), _rot(b.quat_xyzw)
    out = []
    for s in (1, -1):
        w = np.asarray(b.pos) + rb @ np.array([HEAD_X, s * CH_Y, HEAD_ZC])
        out.append(rd.T @ (w - np.asarray(door.pos)))
    return out


def head_in_channel(local, s) -> bool:
    """The head is inside its channel's pocket (behind the lips, between the side walls), its underside at least
    10 mm below the channel's top and not below the stop."""
    return bool(
        LIP_X1 - EPS <= local[0] - HEAD[0] / 2 and local[0] + HEAD[0] / 2 <= 0.0 + EPS
        and abs(local[1] - s * CH_Y) <= POCKET_HALF_W - HEAD[1] / 2 + EPS
        and local[2] - HEAD[2] / 2 <= CH_Z1 - IN_DEPTH + EPS
        and local[2] - HEAD[2] / 2 >= STOP_Z - 0.001 - EPS
    )


def head_on_stop(local) -> bool:
    return bool(local[2] - HEAD[2] / 2 <= STOP_Z + ON_STOP + EPS)


def square_to_door(objects) -> bool:
    """The bin upright within 10 degrees and, seen from above, its back parallel to the door within 3 degrees."""
    rd, rb = _rot(objects["fridge_door"].quat_xyzw), _rot(objects["door_bin"].quat_xyzw)
    up = math.acos(float(np.clip(rb[2, 2], -1.0, 1.0)))
    dy, by = rd[:2, 1], rb[:2, 1]  # the door's and the bin's long axes, projected onto the table
    turn = abs(math.atan2(dy[0] * by[1] - dy[1] * by[0], float(dy @ by)))
    return bool(up <= UPRIGHT + EPS and turn <= SQUARE + EPS)


# Let go: measured every step in the simulator by an on_step callback. Offline there is no robot, so it stays True.
_grip = {"clear": True}


def _boxes(part, state):
    r = _rot(state.quat_xyzw)
    return [(c, rr, s) for _, c, rr, s in part.world_boxes({"base": (np.asarray(state.pos), r)})]


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    b = sim.states()["door_bin"]
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
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in _boxes(DOOR_BIN, b))


def _hung(objects) -> bool:
    b = objects["door_bin"]
    if not (b.resting() and _grip["clear"]):
        return False
    heads = heads_in_door(objects)
    return bool(all(head_in_channel(h, s) and head_on_stop(h) for h, s in zip(heads, (1, -1)))
                and square_to_door(objects))


def success(objects):
    """Both tabs down in their channels on the stops, the bin upright and square to the door, let go and still."""
    return _hung(objects)


def distance(objects):
    heads = heads_in_door(objects)
    goal = [np.array([(LIP_X1 + 0.0) / 2, s * CH_Y, STOP_Z + HEAD[2] / 2]) for s in (1, -1)]
    return float(max(np.linalg.norm(h - g) for h, g in zip(heads, goal)))


def score(objects):
    """Pass/fail: 1.0 hung, else 0."""
    return 1.0 if _hung(objects) else 0.0


# --- demo: one arm -----------------------------------------------------------------------------------------------
TIP = 0.013
FRONT_X = -_back + BIN_WALL / 2  # the front wall's centre in the bin frame
GRIP_DZ = BIN_H - 0.022 + TIP  # tips 22 mm below the bin's top
OPEN = -0.012  # pads 24 mm apart around the 6 mm front wall
SHUT = -0.0015  # closed 1.5 mm past each face
TRAVEL = 0.15
BACK_OFF = -0.0025  # aim the bin's back 2.5 mm behind the lips' front: the loaded arm lags about 5 mm toward the robot
HUNG_X = LIP_X0 - _back - BACK_OFF  # the bin origin's x' while it is lowered


def _grip_yaw(bin_yaw: float, arm: str) -> float:
    """A yaw that closes the fingers across the front wall (along the bin's x), kept within the arm's easy range
    (the fingers are symmetric, so a half turn grips the same way)."""
    lo = math.radians(-45 if arm == "right" else -135)
    y = bin_yaw + math.pi / 2
    while y > lo + math.pi:
        y -= math.pi
    while y < lo:
        y += math.pi
    return y


def _w(xy, z):
    return (float(xy[0]), float(xy[1]), TOP + z)


def demo():
    """Right arm: grip the bin's front wall across its thickness, lift it above the channels, turn it square to the
    door and carry it onto the channels' line, lower it with its back against the channel fronts until the tabs are
    6 mm above the stops, let go and back off. The door's angle and the bin's are what the cameras show."""
    lot = lots.current()
    a = lot.yaw_of("fridge_door")
    by = lot.yaw_of("door_bin")
    bx, byy, _ = lot.pos("door_bin")
    rb = np.array([[math.cos(by), -math.sin(by)], [math.sin(by), math.cos(by)]])
    pick = np.array([bx, byy]) + rb @ np.array([FRONT_X, 0.0])
    ra = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    dx, dy, _ = lot.pos("fridge_door")
    hang_origin = np.array([dx, dy]) + ra @ np.array([HUNG_X, 0.0])
    over = hang_origin + ra @ np.array([-0.012, 0.0])  # 12 mm in front of the hang line
    place = hang_origin + ra @ np.array([FRONT_X, 0.0])
    over_g = over + ra @ np.array([FRONT_X, 0.0])
    y0, y1 = _grip_yaw(by, lot.arm), _grip_yaw(a, lot.arm)
    lift_z = CH_Z1 + 0.012 - STOP_Z  # bin underside 12 mm above where the heads clear the channel tops
    hang_z = STOP_Z + 0.006  # bin underside (= heads' undersides) 6 mm above the stops
    return {lot.arm: [
        Waypoint(1.5, _w(pick, TRAVEL), OPEN, "over the bin's front wall", y0),
        Waypoint(1.5, _w(pick, GRIP_DZ), OPEN, "down", y0),
        Waypoint(0.8, _w(pick, GRIP_DZ), SHUT, "grip the front wall", y0),
        Waypoint(1.5, _w(pick, STOP_Z + lift_z + GRIP_DZ), SHUT, "lift", y0),
        Waypoint(2.5, _w(over_g, STOP_Z + lift_z + GRIP_DZ), SHUT, "turn square to the door, in front of it", y1),
        Waypoint(1.5, _w(place, STOP_Z + lift_z + GRIP_DZ), SHUT, "back against the channel fronts", y1),
        Waypoint(3.0, _w(place, hang_z + GRIP_DZ), SHUT, "lower the tabs into the channels", y1),
        Waypoint(0.6, _w(place, hang_z + GRIP_DZ), OPEN, "let go", y1),
        Waypoint(1.0, _w(place + ra @ np.array([-0.05, 0.0]), hang_z + GRIP_DZ + 0.02), OPEN, "back off", y1),
        Waypoint(1.0, _w(place + ra @ np.array([-0.05, 0.0]), TRAVEL), OPEN, "up", y1),
        Waypoint(2.0, _w(place + ra @ np.array([-0.05, 0.0]), TRAVEL), OPEN, "wait", y1),
    ]}
