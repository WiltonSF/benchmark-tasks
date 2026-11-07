"""On the station table, in front of the right arm: a plate nest on a liquid-handler deck, a grey pocket with low
walls on three sides. At the far end of the pocket stands a black spring tab, taller than the walls, in a grey
housing; it can be pushed away from the pocket. A white deep-well plate lies on the table nearer the robot, beside
the nest.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import lots  # noqa: E402
import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

GREY, DARK, BLACK, WHITE = (0.62, 0.64, 0.66), (0.40, 0.41, 0.43), (0.10, 0.10, 0.11), (0.93, 0.93, 0.91)

# Deep-well plate (loose): SBS footprint, deep-well height.
PLATE = (0.128, 0.086, 0.044)
PLATE_MASS = 0.08
PLATE_XY = (0.20, -0.21)

# Nest (fixed) in world coordinates. x away from the robot, y to its left.
NEST_Y = -0.12
FLOOR_T = 0.004  # deck plate: the nest floor is 4 mm above the table
FLOOR_Z = TOP + FLOOR_T
WALL_H = 0.010  # walls stand 10 mm above the floor
DATUM_X = 0.265  # inner face of the near (datum) wall
PLAY_Y = 0.0015  # side play each side
POCKET_W = PLATE[1] + 2 * PLAY_Y
SHORT = 0.004  # at rest the tab leaves the pocket this much shorter than the plate
TAB_FACE_X = DATUM_X + PLATE[0] - SHORT  # the tab's pocket face at rest
TAB = (0.008, 0.030, WALL_H + 0.025)  # x, y, height above the floor: 25 mm above the rim
TAB_TRAVEL = 0.015
TAB_SPRING, TAB_PRELOAD = 60.0, 0.025  # N/m; the spring's rest point is set 25 mm past the tab's stop
FAR_X = TAB_FACE_X + TAB[0] + TAB_TRAVEL + 0.004  # far end of the nest: behind the tab's travel

X0 = (DATUM_X - 0.010 + FAR_X) / 2  # part origin: middle of the nest, on the table
L = FAR_X - (DATUM_X - 0.010)


def _p(x, y, z):
    """Part frame from world x, y and a height above the table."""
    return (x - X0, y - NEST_Y, z)


NEST_SHAPES = [
    parts.box((L, POCKET_W + 0.012, FLOOR_T), at=_p(X0, NEST_Y, FLOOR_T / 2), color=GREY),  # deck plate
    parts.box((0.010, POCKET_W + 0.012, WALL_H), at=_p(DATUM_X - 0.005, NEST_Y, FLOOR_T + WALL_H / 2), color=DARK),
    *[parts.box((L - 0.010, 0.006, WALL_H), at=_p(X0 + 0.005, NEST_Y + s * (POCKET_W / 2 + 0.003),
                                                  FLOOR_T + WALL_H / 2), color=DARK) for s in (1, -1)],
    # tab housing behind the tab (the tab is jointed to this part, so it passes through freely)
    parts.box((0.012, 0.040, TAB[2]), at=_p(FAR_X - 0.006, NEST_Y, FLOOR_T + TAB[2] / 2), color=DARK),
]
TAB_JOINT = parts.Slider(
    "tab",
    shapes=[parts.box(TAB, at=_p(TAB_FACE_X + TAB[0] / 2, NEST_Y, FLOOR_T + TAB[2] / 2), color=BLACK)],
    axis=(1.0, 0.0, 0.0),  # pushed away from the pocket
    travel=TAB_TRAVEL,
    mass=0.02,
    spring=TAB_SPRING,
    damping=0.5,
)
NEST = parts.Part("nest", pos=(X0, NEST_Y, TOP), shapes=NEST_SHAPES, joints=[TAB_JOINT])
PLATE_BOX = Box("deep_well_plate", pos=(*PLATE_XY, TOP + PLATE[2] / 2), size=PLATE, mass=PLATE_MASS, color=WHITE)
OBJECTS = [NEST, PLATE_BOX]

# Lot A (the screenshot): the plate to the robot's right of the nest, long side front to back. Lot B: on the other
# side of the nest, nearer the midline. Lot C: back on the right, turned a quarter turn (long side left to right).
LOT_B_XY = (0.18, -0.02)
LOT_C_XY = (0.165, -0.225)
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A"),
    lots.Lot("B", place={"deep_well_plate": (*LOT_B_XY, TOP + PLATE[2] / 2)}),
    lots.Lot("C", place={"deep_well_plate": (*LOT_C_XY, TOP + PLATE[2] / 2)}, yaw={"deep_well_plate": math.pi / 2}),
)
SUCCESS_HOLD_S = 2.0  # the trial ends early once success() has held this long


@parts.on_step
def _preload(sim):
    """The tab's spring is preloaded: its rest point sits past the stop, so the tab presses on with about 1.5 N."""
    sim.set_target("nest", "tab", -TAB_PRELOAD)


ON = 0.001
HELD_BACK = 0.002
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the plate
EPS = 1e-6


def _rot(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tilt_deg(state) -> float:
    return math.degrees(math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0))))


def near_end_x(state) -> float:
    """x of the plate's near end face, from its centre and true length (not the widened bounding box)."""
    r = _rot(state.quat_xyzw)
    return float(state.pos[0] - abs(r[0, 0]) * PLATE[0] / 2)


def end_bottoms(state) -> list[float]:
    """Heights of the four corners of the plate's underside (both ends), from its centre, orientation and true size."""
    r = _rot(state.quat_xyzw)
    h = np.asarray(PLATE) / 2
    return [float(state.pos[2] + r[2] @ (sx * h[0], sy * h[1], -h[2])) for sx in (1, -1) for sy in (1, -1)]


# Released: in the simulator an on_step callback measures, every step, how close both arms' fingers and grippers are
# to the plate (link meshes against the plate's true box). Offline (near_misses.py) there is no robot, so "clear"
# stays True.
_grip = {"clear": True}


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    p = sim.states()["deep_well_plate"]
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
    gap = analysis.hulls_to_box(hulls, np.asarray(p.pos), _rot(p.quat_xyzw), np.asarray(PLATE))[0]
    _grip["clear"] = gap > CLEAR


def _grade(objects) -> float:
    p, n = objects["deep_well_plate"], objects["nest"]
    flat_on_floor = all(abs(z - FLOOR_Z) <= ON + EPS for z in end_bottoms(p))
    in_pocket = abs(p.pos[1] - NEST_Y) <= PLAY_Y + 0.001 + EPS
    at_datum = DATUM_X - 0.0005 - EPS <= near_end_x(p) <= DATUM_X + ON + EPS
    tab_on_plate = n.joints["tab"] >= HELD_BACK - EPS
    ok = flat_on_floor and in_pocket and at_datum and tab_on_plate and p.resting() and _grip["clear"]
    return 1.0 if ok else 0.0


def success(objects):
    """Plate flat on the nest floor, against the datum wall, held there by the tab, still, the gripper clear."""
    return _grade(objects) == 1.0


def score(objects):
    """Pass/fail: 1.0 as success, else 0 (plate on the tab, short of the datum, off the floor, elsewhere)."""
    return _grade(objects)


def distance(objects):
    p = objects["deep_well_plate"]
    return float(np.hypot(p.pos[0] - (DATUM_X + PLATE[0] / 2), p.pos[1] - NEST_Y))


# -- scripted demo: the right arm ---------------------------------------------------------------------------------
TIP = 0.013
GRIP_DOWN = 0.012  # finger tips 12 mm below the plate's top
HOLD_Z = PLATE[2] - GRIP_DOWN + TIP  # TCP height above the plate's underside
UP = 0.15
PUSH_OVERSHOOT = 0.010  # the arm gives way against the tab's spring (about 2 N, roughly 1 cm at the fingertips)
# Released 2 mm above the floor, the tab's friction held the plate's far end 1.3 mm up (0.6 degrees, traced), so the
# demo commands the plate 2 mm into the floor and opens there: the carried plate rests on the floor as it lets go.
RELEASE_DZ = -0.002
# Traced: in lots B and C the open fingers (about 4 mm clear of the plate a side) were still 3 to 5 mm off their aim
# while coming down fast and landed on the plate's edge, so the arm stops 30 mm above grip height and settles first.
HOVER = 0.030


# Measured in traces: the carried plate sits to the robot's right of the commanded TCP (it shifts in the grip and the
# arm sags): 7 mm with a fast descent onto the plate, 2.4 to 4.9 mm (mean 3.8) in the three lots once the arm settles
# above the plate first, so the demo aims 3.8 mm to the left over the nest.
AIM_Y = NEST_Y + 0.0038


def demo():
    # The plate's start and heading come from the trial's lot (what the cameras show). A plate turned a quarter turn is
    # gripped across its width with the grip turned to match, then turned back to front-to-back on the way to the nest.
    lot = lots.current()
    px, py, _ = lot.pos("deep_well_plate")
    yaw0 = lot.yaw_of("deep_well_plate")
    centre_seated = DATUM_X + PLATE[0] / 2
    start_x = centre_seated + SHORT - 0.005 - 0.003  # plate's far face 5 mm short of the tab, near end clear of the datum
    push_x = centre_seated + SHORT + 0.008 + PUSH_OVERSHOOT  # tab pushed back about 8 mm past the plate's length
    above_rim = FLOOR_Z + WALL_H + 0.003  # plate underside 3 mm above the rim
    on_floor = FLOOR_Z + RELEASE_DZ  # set the plate down on the floor before opening (see RELEASE_DZ)
    right = [
        Waypoint(1.5, (px, py, TOP + UP), FINGER_OPEN, "over the plate", yaw0),
        Waypoint(1.0, (px, py, TOP + HOLD_Z + HOVER), FINGER_OPEN, "just above the plate", yaw0),
        Waypoint(0.8, (px, py, TOP + HOLD_Z + HOVER), FINGER_OPEN, "let the arm settle", yaw0),
        Waypoint(1.2, (px, py, TOP + HOLD_Z), FINGER_OPEN, "down to the plate", yaw0),
        Waypoint(0.8, (px, py, TOP + HOLD_Z), FINGER_CLOSED, "grip across its width", yaw0),
        Waypoint(1.2, (px, py, TOP + UP), FINGER_CLOSED, "lift", yaw0),
        Waypoint(2.0, (start_x, AIM_Y, TOP + UP), FINGER_CLOSED, "over the nest, offset toward the tab"),
        Waypoint(1.5, (start_x, AIM_Y, above_rim + HOLD_Z), FINGER_CLOSED, "lower to just above the rim"),
        Waypoint(1.5, (push_x, AIM_Y, above_rim + HOLD_Z), FINGER_CLOSED, "push the tab back with the plate's end"),
        Waypoint(1.0, (push_x, AIM_Y, on_floor + HOLD_Z), FINGER_CLOSED, "lower onto the pocket floor"),
        Waypoint(0.8, (push_x, AIM_Y, on_floor + HOLD_Z), FINGER_OPEN, "let go"),
        Waypoint(1.2, (push_x - 0.01, AIM_Y, TOP + UP), FINGER_OPEN, "lift clear"),
    ]
    return {"right": right}
