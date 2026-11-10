"""On the station table: a small white freestanding cooker (a box 15 cm deep, 20 cm wide and 21 cm tall with a dark
oven door on the side facing the robot) stands on four short feet, a little way out from a grey wall at the far side
of the table. It is too wide to grip. At the foot of the wall stands a black anti-tip bracket: a low hood 3 cm deep
over a tunnel open toward the robot, and a plate up the wall behind it. Where the cooker and the bracket stand
changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WHITE, DARK, BLACK, GREY, STEEL = (0.92, 0.92, 0.90), (0.12, 0.12, 0.14), (0.05, 0.05, 0.05), (0.60, 0.61, 0.63), \
    (0.45, 0.46, 0.48)

# Wall: fixed, its face toward the robot at x 0.48, 12 cm tall.
WALL_FACE = 0.48
WALL = Box("wall", pos=(WALL_FACE + 0.005, 0.0, TOP + 0.06), size=(0.010, 0.40, 0.12), color=GREY)

# Cooker. Part frame: origin at the floor under the body's centre, x away from the robot. Body 15 (x) x 20 (y) x
# 21 cm, underside 2.5 cm up, on four feet centred 1.5 cm in from each corner: a pad 2.4 cm across and 6 mm tall on
# a post 8 mm across.
DEPTH, WIDTH, HEIGHT = 0.150, 0.200, 0.210
UNDER = 0.025
PAD_R, PAD_H, POST_R = 0.012, 0.006, 0.004
FOOT_X, FOOT_Y = DEPTH / 2 - 0.015, WIDTH / 2 - 0.015  # 0.060, 0.085
COOKER_MASS = 1.2
COOKER_FRICTION = (0.3, 0.3)


def _foot(sx, sy):
    x, y = sx * FOOT_X, sy * FOOT_Y
    return [parts.cylinder(PAD_R, PAD_H, at=(x, y, PAD_H / 2), color=STEEL, axis="z"),
            parts.cylinder(POST_R, UNDER - PAD_H, at=(x, y, (UNDER + PAD_H) / 2), color=STEEL, axis="z")]


COOKER_SHAPES = [
    parts.box((DEPTH, WIDTH, HEIGHT), at=(0.0, 0.0, UNDER + HEIGHT / 2), color=WHITE),
    parts.box((0.004, 0.160, 0.120), at=(-DEPTH / 2 - 0.002, 0.0, UNDER + 0.080), color=DARK),  # oven door
    parts.box((0.004, 0.160, 0.030), at=(-DEPTH / 2 - 0.002, 0.0, UNDER + 0.180), color=STEEL),  # control panel
    *_foot(1, 1), *_foot(1, -1), *_foot(-1, 1), *_foot(-1, -1),
]
BODY_SHAPES = COOKER_SHAPES[:3]

# Bracket: a heavy loose part (so the lots can move it; nothing in the task should). Part frame: origin at the floor
# under the tunnel's centre. Two side walls 1 cm wide and 1 cm tall, inner faces 17 mm from the centre line; two hood
# plates 4 mm thick at 10-14 mm up, leaving a slot 18 mm wide over the centre line for the foot's post. 3 cm deep.
# Behind the hood, a plate 3 mm thick, as wide as the bracket and 11 cm tall, stands against the wall (the leg a real
# bracket is screwed to the wall by; here it also shows the bracket in the top camera above the cooker).
BR_DEPTH = 0.030
TUNNEL_HALF = 0.017
SLOT_HALF = 0.009
WALL_W, WALL_H = 0.010, 0.010
HOOD_T = 0.004
BR_OUT = TUNNEL_HALF + WALL_W  # 27 mm
BRACKET_MASS = 3.0
LEG_T, LEG_H = 0.003, 0.110
BRACKET_SHAPES = [
    parts.box((BR_DEPTH, WALL_W, WALL_H), at=(0.0, sy * (TUNNEL_HALF + WALL_W / 2), WALL_H / 2), color=BLACK)
    for sy in (1, -1)
] + [
    parts.box((BR_DEPTH, BR_OUT - SLOT_HALF, HOOD_T), at=(0.0, sy * (SLOT_HALF + BR_OUT) / 2, WALL_H + HOOD_T / 2),
              color=BLACK)
    for sy in (1, -1)
] + [parts.box((LEG_T, 2 * BR_OUT, LEG_H), at=(BR_DEPTH / 2 + LEG_T / 2, 0.0, LEG_H / 2), color=BLACK)]
FRONT0 = 0.21
BR_X = WALL_FACE - 0.001 - LEG_T - BR_DEPTH / 2  # bracket's back 1 mm off the wall
STOP_FACE = WALL_FACE - 0.001 - LEG_T  # the cooker's back stops against the bracket's wall plate
HOME_X = STOP_FACE - DEPTH / 2  # cooker centre with its back at the plate; its rear feet are then over BR_X
START_X = FRONT0 + DEPTH / 2  # front face at x 0.21


def _bracket_y(target_y, foot):
    return target_y + (FOOT_Y if foot == "left" else -FOOT_Y)


# Lot A (the screenshot): bracket under the rear-left foot of a cooker centred at y +4 cm; cooker starts at y -1 cm.
# Lot B: bracket under the rear-right foot of a cooker centred at y +2 cm; cooker starts at y -2.5 cm.
# Lot C: bracket under the rear-left foot of a cooker centred at y -3 cm; cooker starts at y -0.5 cm, turned 5 deg.
# Lot D: bracket under the rear-right foot of a cooker centred at y -4 cm; cooker starts at y +0.5 cm, turned 3 deg.
A_START, B_START, C_START, D_START = (START_X, -0.010, TOP), (START_X, -0.025, TOP), (START_X, -0.005, TOP), \
    (START_X, 0.005, TOP)
A_BR, B_BR, C_BR = (BR_X, _bracket_y(0.04, "left"), TOP), (BR_X, _bracket_y(0.02, "right"), TOP), \
    (BR_X, _bracket_y(-0.03, "left"), TOP)
D_BR = (BR_X, _bracket_y(-0.04, "right"), TOP)
C_YAW, D_YAW = math.radians(5.0), math.radians(3.0)

COOKER = parts.Part("cooker", pos=A_START, mass=COOKER_MASS, friction=COOKER_FRICTION, shapes=COOKER_SHAPES)
BRACKET = parts.Part("bracket", pos=A_BR, mass=BRACKET_MASS, friction=(1.2, 1.0), shapes=BRACKET_SHAPES)
OBJECTS = [COOKER, BRACKET, WALL]

LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"cooker": A_START, "bracket": A_BR}, foot="left"),
    lots.Lot("B", place={"cooker": B_START, "bracket": B_BR}, foot="right"),
    lots.Lot("C", place={"cooker": C_START, "bracket": C_BR}, yaw={"cooker": C_YAW}, foot="left"),
    lots.Lot("D", place={"cooker": D_START, "bracket": D_BR}, yaw={"cooker": D_YAW}, foot="right"),
)

EPS = 1e-6
HOME_GAP = 0.010  # back within 10 mm of the wall
SQUARE = math.radians(3.0)  # yaw and lean
FOOT_IN_Y = 0.006  # foot centre within 6 mm of the tunnel's centre line (the pad fits with 5 mm a side)
LINED_Y = 0.005  # lined up: within 5 mm of the centre line
MOVED_BACK = 0.050
ON_FLOOR = 0.002  # every pad's underside within 2 mm of the table
TIP_LIMIT = math.radians(15.0)  # tipped: the cooker leaned more than 15 degrees at any point
BRACKET_MOVE = 0.005  # the bracket moved more than 5 mm or turned more than 3 degrees at any point
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the cooker
SUCCESS_HOLD_S = 4.0  # long enough that the demo video ends with the arms moved aside


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _lean(s) -> float:
    return math.acos(float(np.clip(_rot(s.quat_xyzw)[2, 2], -1.0, 1.0)))


def _yaw(s) -> float:
    r = _rot(s.quat_xyzw)
    return math.atan2(r[1, 0], r[0, 0])


def _foot_world(cooker, foot) -> np.ndarray:
    sy = 1 if foot == "left" else -1
    return np.asarray(cooker.pos) + _rot(cooker.quat_xyzw) @ np.array([FOOT_X, sy * FOOT_Y, 0.0])


def _in_bracket_frame(bracket, p) -> np.ndarray:
    return _rot(bracket.quat_xyzw).T @ (np.asarray(p) - np.asarray(bracket.pos))


def _standing_square(cooker) -> bool:
    """Upright and square to the wall within 3 degrees, all four pads on the table, still."""
    r = _rot(cooker.quat_xyzw)
    pads = [np.asarray(cooker.pos) + r @ np.array([sx * FOOT_X, sy * FOOT_Y, 0.0]) for sx in (1, -1) for sy in (1, -1)]
    return bool(_lean(cooker) <= SQUARE + EPS and abs(_yaw(cooker)) <= SQUARE + EPS
                and max(p[2] for p in pads) <= TOP + ON_FLOOR + EPS and cooker.resting())


def _back_gap(cooker) -> float:
    back = np.asarray(cooker.pos) + _rot(cooker.quat_xyzw) @ np.array([DEPTH / 2, 0.0, 0.0])
    return float(STOP_FACE - back[0])  # to the bracket plate's face on the wall (4 mm proud of the wall)


# Whole-trial record: was the cooker ever leaned past 15 degrees, or the bracket moved over 5 mm or turned over 3
# degrees? Cleared only by a scene reset: the cooker back within 0.5 mm of its start and at rest, after a jump of over
# 20 mm in one control step (a robot cannot move it back that way).
_record = {"fault": False, "prev": None}
# Released: an on_step callback measures, every step, how close both arms' fingers and grippers are to the cooker.
# Offline (near_misses.py) there is no robot, so "clear" stays True.
_grip = {"clear": True}


def _update(objects) -> None:
    cooker, bracket = objects["cooker"], objects["bracket"]
    lot = lots.current()
    pos = np.asarray(cooker.pos, float)
    prev = _record["prev"]
    if prev is not None and np.linalg.norm(pos - np.asarray(lot.pos("cooker"))) < 0.0005 and cooker.resting():
        if np.linalg.norm(pos - prev) > 0.020:
            _record["fault"] = False
    _record["prev"] = pos
    moved = np.linalg.norm(np.asarray(bracket.pos)[:2] - np.asarray(lot.pos("bracket"))[:2])
    if (_lean(cooker) > TIP_LIMIT + EPS or moved > BRACKET_MOVE + EPS or abs(_yaw(bracket)) > SQUARE + EPS
            or _lean(bracket) > SQUARE + EPS):
        _record["fault"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    c = sim.states()["cooker"]
    boxes = COOKER.world_boxes({"base": (np.asarray(c.pos), _rot(c.quat_xyzw))})
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
    _grip["clear"] = all(analysis.hulls_to_box(hulls, ctr, r, s)[0] > CLEAR for _, ctr, r, s in boxes)


def _installed(objects) -> bool:
    cooker, bracket = objects["cooker"], objects["bracket"]
    f = _in_bracket_frame(bracket, _foot_world(cooker, lots.current().foot))
    return bool(_standing_square(cooker) and _back_gap(cooker) <= HOME_GAP + EPS
                and abs(f[0]) <= BR_DEPTH / 2 + EPS and abs(f[1]) <= FOOT_IN_Y + EPS and _grip["clear"])


def _lined_up(objects) -> bool:
    cooker, bracket = objects["cooker"], objects["bracket"]
    f = _in_bracket_frame(bracket, _foot_world(cooker, lots.current().foot))
    moved = cooker.pos[0] - lots.current().pos("cooker")[0]
    return bool(_standing_square(cooker) and moved >= MOVED_BACK - EPS
                and abs(f[1]) <= LINED_Y + EPS and _grip["clear"])


def success(objects):
    """Back against the wall, square, with the rear foot over the bracket inside its tunnel; released and still."""
    _update(objects)
    return bool(not _record["fault"] and _installed(objects))


def distance(objects):
    return max(0.0, _back_gap(objects["cooker"]))


def score(objects):
    """1.0 on success; 0.25 if released, square, moved back at least 5 cm and lined up with the bracket but not yet
    in it; 0 otherwise."""
    _update(objects)
    if _record["fault"]:
        return 0.0
    if _installed(objects):
        return 1.0
    return 0.25 if _lined_up(objects) else 0.0


# --- demo: fixed waypoints from the lot's layout (what the top camera shows) --------------------------------------
TIP = 0.013  # finger tips below the TCP
PUSH_Z = TOP + 0.055 + TIP  # fingertips 5.5 cm above the table
TRAVEL = TOP + 0.10  # below the cooker's top: the arms go round it, never over
X_OFF = 0.0125  # measured: the closed fingers' face is this far ahead of the TCP when pushing along x
Y_OFF = 0.039  # measured: this far to the side when pushing along y (fingers close along y)
CLEAR_GAP = 0.03  # approach this far off a face
PRESS = 0.010  # final push commanded this far past the wall contact
PUSH_Y = 0.095  # the two arms push the front face at y +-9.5 cm (closer, their elbows meet)


def _w(x, y, z):
    return (float(x), float(y), float(z))


def _line_y(lot) -> float:
    """Cooker centre y with its bracketed foot in line with the tunnel (the bracket shows in the top camera)."""
    by = lot.pos("bracket")[1]
    return by - (FOOT_Y if lot.foot == "left" else -FOOT_Y)


def demo():
    """One arm pushes the cooker's side face until its foot on the bracket's side is in line with the bracket's
    tunnel; then both arms push the front face straight back until the back meets the wall, and withdraw."""
    lot = lots.current()
    sx, sy, _ = lot.pos("cooker")
    ty = _line_y(lot)
    sign = -1.0 if ty < sy else 1.0  # direction of the sideways push
    side_arm = "left" if sign < 0 else "right"
    front = FRONT0 - X_OFF - 0.045  # TCP x in front of the cooker, clear of it (it may drift back a little)
    face0, face1 = sy - sign * WIDTH / 2, ty - sign * WIDTH / 2
    tcp0 = face0 - sign * (Y_OFF + CLEAR_GAP)
    tcp1 = face1 - sign * Y_OFF
    side = [
        Waypoint(1.5, _w(front, tcp0, TRAVEL), FINGER_CLOSED, "out to the side"),
        Waypoint(1.5, _w(sx, tcp0, TRAVEL), FINGER_CLOSED, "along the side"),
        Waypoint(1.5, _w(sx, tcp0, PUSH_Z), FINGER_CLOSED, "down beside the cooker"),
        Waypoint(abs(tcp1 - tcp0) / 0.02, _w(sx, tcp1, PUSH_Z), FINGER_CLOSED, "push sideways into line"),
        Waypoint(1.0, _w(sx, tcp1, PUSH_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.0, _w(sx, tcp1 - sign * CLEAR_GAP, PUSH_Z), FINGER_CLOSED, "back off"),
        Waypoint(1.0, _w(sx, tcp1 - sign * CLEAR_GAP, TRAVEL), FINGER_CLOSED, "up"),
        Waypoint(1.5, _w(front, tcp1 - sign * CLEAR_GAP, TRAVEL), FINGER_CLOSED, "back to the front"),
    ]
    side_t = sum(w.duration for w in side)
    home = WALL_FACE - DEPTH - X_OFF + PRESS
    plans = {}
    for arm, s in (("left", 1.0), ("right", -1.0)):
        y0, y1 = sy + s * (PUSH_Y + 0.02), ty + s * PUSH_Y
        first = side if arm == side_arm else [
            Waypoint(1.5, _w(front, y0, TRAVEL), FINGER_CLOSED, "in front"),
            Waypoint(side_t - 1.5, _w(front, y0, TRAVEL), FINGER_CLOSED, "wait"),
        ]
        clear = [Waypoint(2.0, _w(0.32, s * 0.30, TRAVEL), FINGER_CLOSED, "straight down, well outside the cooker"),
                 Waypoint(1.5, _w(0.14, s * 0.24, TRAVEL), FINGER_CLOSED, "round to its front")]
        plans[arm] = clear + first + [
            Waypoint(1.5, _w(front, y1 + s * 0.02, TRAVEL), FINGER_CLOSED, "to the front"),
            Waypoint(1.5, _w(front, y1 + s * 0.02, PUSH_Z), FINGER_CLOSED, "down to the front"),
            Waypoint(1.0, _w(front + 0.03, y1, PUSH_Z), FINGER_CLOSED, "in toward the face"),
            Waypoint((home - front) / 0.03, _w(home, y1, PUSH_Z), FINGER_CLOSED, "push straight back"),
            Waypoint(1.0, _w(home, y1, PUSH_Z), FINGER_CLOSED, "hold"),
            Waypoint(1.5, _w(home - 0.05, y1, PUSH_Z), FINGER_CLOSED, "withdraw"),
            Waypoint(1.0, _w(home - 0.05, y1, TRAVEL), FINGER_CLOSED, "up"),
            Waypoint(2.0, _w(0.15, s * 0.24, TRAVEL), FINGER_CLOSED, "aside"),
            Waypoint(2.0, _w(0.15, s * 0.24, TRAVEL), FINGER_CLOSED, "wait"),
        ]
    return plans
