"""On the station table: the upper and lower joints of a clarinet, black with silver keys. The lower joint stands
socket up on a grey stand base between the arms; a blue lever pad sticks out of its side below the socket, between
two silver key posts. The upper joint stands cork tenon down in a small holder in front of the left arm; a blue
lever hangs from its side below the shoulder. Which way each lever points changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so its hooks run first)
import lots  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BLACK, SILVER, CORK, GREY = (0.05, 0.05, 0.06), (0.80, 0.81, 0.84), (0.72, 0.56, 0.38), (0.45, 0.46, 0.48)
KEY_BLUE = (0.10, 0.35, 0.95)  # the two bridge-key levers: a visibility marker

# Lower joint. Part frame: origin at the centre of the stand base's underside, z up; its bridge-key lever points
# along +x. A 50 mm round body on a stand base, topped by a socket collar 40 mm square with a 32 mm square bore.
BODY_R = 0.025
PLATE = (0.100, 0.100, 0.006)
COLLAR_LOW, RIM = 0.036, 0.066  # the collar (and socket) runs from 36 to 66 mm; the rim is its top
BORE_HALF, COLLAR_HALF = 0.016, 0.020
WALL = COLLAR_HALF - BORE_HALF
LOW_PAD_TOP = RIM - 0.014  # the lower lever pad's top, 14 mm under the rim
PAD_W = 0.020  # both pads 20 mm wide (across the lever)
POST_Y0, POST_Y1 = 0.020, 0.035  # key posts: 20 to 35 mm either side of the lever's centre line
POST_X0, POST_X1 = 0.020, 0.044
LOWER_MASS = 3.0
LOWER_XY = (0.32, 0.07)

_collar_h = RIM - COLLAR_LOW
_collar_z = (COLLAR_LOW + RIM) / 2
LOWER_SHAPES = [
    parts.box(PLATE, at=(0.0, 0.0, PLATE[2] / 2), color=GREY),
    parts.cylinder(BODY_R, COLLAR_LOW - PLATE[2], at=(0.0, 0.0, (COLLAR_LOW + PLATE[2]) / 2), color=BLACK),
    parts.box((WALL, 2 * COLLAR_HALF, _collar_h), at=(BORE_HALF + WALL / 2, 0.0, _collar_z), color=BLACK),
    parts.box((WALL, 2 * COLLAR_HALF, _collar_h), at=(-BORE_HALF - WALL / 2, 0.0, _collar_z), color=BLACK),
    parts.box((2 * BORE_HALF, WALL, _collar_h), at=(0.0, BORE_HALF + WALL / 2, _collar_z), color=BLACK),
    parts.box((2 * BORE_HALF, WALL, _collar_h), at=(0.0, -BORE_HALF - WALL / 2, _collar_z), color=BLACK),
    # bridge-key lever pad, 22 mm out from the collar face, top 14 mm under the rim
    parts.box((0.022, PAD_W, 0.015), at=(0.031, 0.0, LOW_PAD_TOP - 0.0075), color=KEY_BLUE),
]
for _s in (1, -1):  # two key posts, tops flush with the rim
    LOWER_SHAPES.append(parts.box((POST_X1 - POST_X0, POST_Y1 - POST_Y0, _collar_h),
                                  at=((POST_X0 + POST_X1) / 2, _s * (POST_Y0 + POST_Y1) / 2, _collar_z), color=SILVER))

# Upper joint. Part frame: origin at the centre of the shoulder (the body's underside), z up; its lever points +x.
UP_BODY_H = 0.055
TENON_R, TENON_L = 0.0125, 0.025
DROP = 0.012  # the upper lever pad hangs 12 mm below the shoulder
UP_PAD_X0, UP_PAD_X1 = 0.030, 0.044
UPPER_MASS = 0.12
UPPER_SHAPES = [
    parts.cylinder(BODY_R, UP_BODY_H, at=(0.0, 0.0, UP_BODY_H / 2), color=BLACK),
    parts.cylinder(TENON_R, TENON_L, at=(0.0, 0.0, -TENON_L / 2), color=CORK),
    parts.box((UP_PAD_X1 - 0.022, PAD_W, 0.010), at=((0.022 + UP_PAD_X1) / 2, 0.0, 0.005), color=KEY_BLUE),  # arm
    parts.box((UP_PAD_X1 - UP_PAD_X0, PAD_W, DROP), at=((UP_PAD_X0 + UP_PAD_X1) / 2, 0.0, -DROP / 2),
              color=KEY_BLUE),  # pad
]

# Holder: a fixed square collar 15 mm tall with the same bore as the socket; the tenon stands on the table in it.
HOLDER_XY = (0.20, 0.20)
HOLDER_H = 0.015
HOLDER = parts.Part(
    "holder",
    pos=(*HOLDER_XY, TOP),
    shapes=[
        parts.box((WALL, 2 * COLLAR_HALF, HOLDER_H), at=(BORE_HALF + WALL / 2, 0.0, HOLDER_H / 2), color=GREY),
        parts.box((WALL, 2 * COLLAR_HALF, HOLDER_H), at=(-BORE_HALF - WALL / 2, 0.0, HOLDER_H / 2), color=GREY),
        parts.box((2 * BORE_HALF, WALL, HOLDER_H), at=(0.0, BORE_HALF + WALL / 2, HOLDER_H / 2), color=GREY),
        parts.box((2 * BORE_HALF, WALL, HOLDER_H), at=(0.0, -BORE_HALF - WALL / 2, HOLDER_H / 2), color=GREY),
    ],
)

D = math.radians
LOWER = parts.Part("lower_joint", pos=(*LOWER_XY, TOP), mass=LOWER_MASS, yaw=0.0, shapes=LOWER_SHAPES)
UPPER = parts.Part("upper_joint", pos=(*HOLDER_XY, TOP + TENON_L), mass=UPPER_MASS, yaw=D(90), shapes=UPPER_SHAPES)
OBJECTS = [LOWER, UPPER, HOLDER]

# Lot A (the screenshot): lower lever points away from the robot, upper lever to the robot's left.
# Lot B: lower lever to the left, upper lever away. Lot C: lower lever to the right, upper lever back and to the right.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"lower_joint": LOWER.pos}),  # zero move: forces the settle and a fresh first frame
    lots.Lot("B", yaw={"lower_joint": D(90), "upper_joint": 0.0}),
    lots.Lot("C", yaw={"lower_joint": D(-90), "upper_joint": D(-150)}),
)

EPS = 1e-6
HOME_DZ = 0.002  # shoulder within 2 mm of the rim
CENTRED = 0.005  # upper joint's axis within 5 mm of the socket's axis
UPRIGHT = D(5.0)
LINED_UP = D(12.0)  # the two levers within 12 degrees of each other, seen from above
LOWER_MOVE, LOWER_TILT = 0.010, D(5.0)
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the upper joint
SUCCESS_HOLD_S = 2.0  # success must hold 2 s, so the arm is clear of the top camera's view when a trial ends


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(state) -> float:
    return math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0)))


def lever_angle(objects) -> float:
    """Angle (rad, 0..pi) between the two levers' directions seen from above."""
    a = _rot(objects["upper_joint"].quat_xyzw)[:2, 0]
    b = _rot(objects["lower_joint"].quat_xyzw)[:2, 0]
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-6 or nb < 1e-6:
        return math.pi
    return math.acos(float(np.clip(a @ b / (na * nb), -1.0, 1.0)))


def home_offset(objects):
    """(shoulder height above the rim, horizontal distance between the axes), m."""
    up, low = objects["upper_joint"], objects["lower_joint"]
    rim = np.asarray(low.pos) + _rot(low.quat_xyzw) @ np.array([0.0, 0.0, RIM])
    d = np.asarray(up.pos) - rim
    return float(d[2]), float(np.hypot(d[0], d[1]))


# Whole-trial record: was the lower joint ever moved more than 10 mm or tilted more than 5 degrees from its start?
# Cleared only by a scene reset: the upper joint back within 0.5 mm of its start and at rest, after a jump of over
# 20 mm in one control step (a robot cannot move it back that way).
_record = {"fault": False, "prev": None}
# Released: an on_step callback measures every step how close both arms' fingers and grippers are to the upper
# joint. Offline (near_misses.py) there is no robot, so "clear" stays True.
_grip = {"clear": True}


def _update(objects) -> None:
    up = objects["upper_joint"]
    pos = np.asarray(up.pos, float)
    prev = _record["prev"]
    start = np.asarray(lots.current().pos("upper_joint"), float)
    if prev is not None and np.linalg.norm(pos - start) < 0.0005 and up.resting():
        if np.linalg.norm(pos - prev) > 0.020:
            _record["fault"] = False
    _record["prev"] = pos
    low = objects["lower_joint"]
    moved = np.linalg.norm(np.asarray(low.pos) - np.asarray(lots.current().pos("lower_joint"), float))
    if moved > LOWER_MOVE + EPS or _tilt(low) > LOWER_TILT + EPS:
        _record["fault"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    up = sim.states()["upper_joint"]
    boxes = UPPER.world_boxes({"base": (np.asarray(up.pos), _rot(up.quat_xyzw))})
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


def _joined(objects) -> bool:
    up, low = objects["upper_joint"], objects["lower_joint"]
    dz, dxy = home_offset(objects)
    return bool(
        abs(dz) <= HOME_DZ + EPS
        and dxy <= CENTRED + EPS
        and _tilt(up) <= UPRIGHT + EPS
        and lever_angle(objects) <= LINED_UP + EPS
        and up.resting()
        and low.resting()
        and _grip["clear"]
    )


def success(objects):
    """The upper joint is home in the socket with its lever over the lower lever, let go and still; the lower joint
    was never shoved or tipped."""
    _update(objects)
    return bool(not _record["fault"] and _joined(objects))


def distance(objects):
    dz, dxy = home_offset(objects)
    return float(math.hypot(dz, dxy))


def score(objects):
    """Pass/fail: 1.0 on success, otherwise 0."""
    _update(objects)
    return 1.0 if not _record["fault"] and _joined(objects) else 0.0


# --- demo: left arm only ----------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
GRIP_DZ = UP_BODY_H + 0.002  # TCP above the shoulder: tips 11 mm down the 55 mm body
OPEN, SHUT = -0.035, -0.022  # pads 70 mm apart; closed 3 mm past the 50 mm body's faces
CARRY = RIM + 0.034  # shoulder height while carried: the tenon's end 9 mm over the rim
SEAT = RIM + 0.004  # commanded 4 mm above the rim; the loaded arm sags about 4 mm
TRAVEL = 0.15
COMP = (0.0025, 0.0)  # traced: lowering, the held joint lags about 3 mm toward the robot


def _w(x, y, z):
    return (x, y, TOP + z)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def demo():
    """Left arm: grip the upper joint round its body, lift it out of the holder, carry it over the socket while
    turning it so its lever points the way the lower joint's lever points (both seen in the cameras for the trial's
    lot), lower the tenon into the socket, let go and back off."""
    lot = lots.current()
    turn = _wrap(lot.yaw_of("lower_joint") - lot.yaw_of("upper_joint"))
    y0, y1 = -turn / 2, turn / 2
    hx, hy, hz = lot.pos("upper_joint")
    lx, ly, _ = lot.pos("lower_joint")
    sx, sy = lx + COMP[0], ly + COMP[1]
    pick = hz - TOP + GRIP_DZ
    return {"left": [
        Waypoint(1.5, _w(hx, hy, TRAVEL), OPEN, "over the upper joint", y0),
        Waypoint(1.5, _w(hx, hy, pick), OPEN, "down round its body", y0),
        Waypoint(0.8, _w(hx, hy, pick), SHUT, "grip", y0),
        Waypoint(1.5, _w(hx, hy, CARRY + GRIP_DZ), SHUT, "lift out of the holder", y0),
        Waypoint(2.5, _w(sx, sy, CARRY + GRIP_DZ), SHUT, "carry over the socket, turning the lever into line", y1),
        Waypoint(1.0, _w(sx, sy, CARRY + GRIP_DZ), SHUT, "settle", y1),
        Waypoint(3.0, _w(sx, sy, SEAT + GRIP_DZ), SHUT, "lower the tenon into the socket", y1),
        Waypoint(0.6, _w(sx, sy, SEAT + GRIP_DZ), OPEN, "release", y1),
        Waypoint(0.6, _w(sx, sy, SEAT + GRIP_DZ + 0.03), OPEN, "up off the joint", y1),
        Waypoint(1.0, _w(sx - 0.16, sy + 0.18, TRAVEL), OPEN, "back off, clear of the top camera's view"),
        Waypoint(3.0, _w(sx - 0.16, sy + 0.18, TRAVEL), OPEN, "wait while it settles"),
    ]}
