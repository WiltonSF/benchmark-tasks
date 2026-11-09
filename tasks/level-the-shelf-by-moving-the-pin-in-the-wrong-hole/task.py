"""On the station table, between the arms: an adjustable steel rack. Two dark grey uprights stand to the left and right,
each with a front and a back column of square holes at three levels. Four orange T-handle pull pins pass through
the holes from the outside, each with a collar against the upright and a T-handle outside it, and stick out on the
inside. A light blue shelf board with a front lip and a round front rail rests on the pins' inner ends. Which hole
each pin is in can change between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

ORANGE, DARK, BLUE, RAIL = (0.95, 0.45, 0.05), (0.22, 0.23, 0.25), (0.35, 0.55, 0.80), (0.30, 0.20, 0.12)

# Uprights: 40 mm thick (y), 260 mm long (x), 130 mm tall; inner faces 115 mm either side of the midline.
CX = 0.30  # the shelf's centre, in front of the arm bases
INNER, THICK, LENGTH, HEIGHT = 0.115, 0.040, 0.260, 0.130
OUTER = INNER + THICK
COLS = (0.21, 0.39)  # front and back hole columns (world x)
LEVELS = (0.060, 0.085, 0.110)  # hole centres above the table: levels 1, 2, 3
HOLE = 0.014  # square holes


def _upright_shapes():
    """Boxes making a solid upright with two columns of 14 mm square through-holes (part frame: centre of its
    footprint, on the table)."""
    h = HOLE / 2
    xs = [c - CX for c in COLS]
    out = []
    edges = [-LENGTH / 2, xs[0] - h, xs[0] + h, xs[1] - h, xs[1] + h, LENGTH / 2]
    for a, b in ((edges[0], edges[1]), (edges[2], edges[3]), (edges[4], edges[5])):  # full-height strips
        out.append(parts.box((b - a, THICK, HEIGHT), at=((a + b) / 2, 0.0, HEIGHT / 2), color=DARK))
    zs = [0.0] + [v for z in LEVELS for v in (z - h, z + h)] + [HEIGHT]
    for x in xs:  # between the holes of a column
        for a, b in zip(zs[0::2], zs[1::2]):
            out.append(parts.box((HOLE, THICK, b - a), at=(x, 0.0, (a + b) / 2), color=DARK))
    return out


UPRIGHT_L = parts.Part("upright_left", pos=(CX, INNER + THICK / 2, TOP), shapes=_upright_shapes())
UPRIGHT_R = parts.Part("upright_right", pos=(CX, -(INNER + THICK / 2), TOP), shapes=_upright_shapes())

# Pin. Part frame: origin on the shank's axis at the collar's inner face (the upright's outer face when seated);
# +y points out of the upright. Shank 10 mm across, 126 mm long: 80 mm inward of the collar (40 in the upright,
# 40 sticking out on the inside) and 46 mm outward. Collar 20 mm across, 4 mm thick. T-handle 14 x 5 x 34 mm.
SHANK_R, IN_LEN, OUT_LEN = 0.005, 0.080, 0.046
TAB_Y, TAB = 0.041, (0.014, 0.005, 0.034)
TAB_Z = 0.007  # the handle runs from 10 mm below the axis to 24 mm above it
PIN_MASS = 0.04
SEAT_DZ = -0.002  # a seated pin's axis rests 2 mm below its hole's centre
PIN_SHAPES = [
    parts.cylinder(SHANK_R, IN_LEN + OUT_LEN, at=(0.0, (OUT_LEN - IN_LEN) / 2, 0.0), color=ORANGE, axis="y"),
    parts.cylinder(0.010, 0.004, at=(0.0, 0.002, 0.0), color=ORANGE, axis="y"),
    parts.box(TAB, at=(0.0, TAB_Y, TAB_Z), color=ORANGE),
]
PINS = {  # name: (column x, side +1 left / -1 right)
    "pin_fl": (COLS[0], 1), "pin_fr": (COLS[0], -1), "pin_bl": (COLS[1], 1), "pin_br": (COLS[1], -1),
}


def pin_pos(name, level):
    x, side = PINS[name]
    return (x, side * (OUTER + 0.0005), TOP + LEVELS[level] + SEAT_DZ)


def _pin(name, level):
    return parts.Part(name, pos=pin_pos(name, level), mass=PIN_MASS, yaw=0.0 if PINS[name][1] > 0 else math.pi,
                      shapes=PIN_SHAPES)


# Shelf. Part frame: origin at the centre of the board's underside. Board 240 (front to back) x 200 x 10 mm; a front
# lip 15 x 200 x 15 mm on top of its front edge; a round rail 12 mm across and 170 mm long on two 10 mm posts on the
# lip, its axis 41 mm above the underside.
BOARD = (0.240, 0.200, 0.010)
LIP_X = -BOARD[0] / 2 + 0.0075
RAIL_Z, RAIL_R = 0.041, 0.006
SHELF_MASS = 0.4
SUPPORT = LEVELS[1] + SEAT_DZ + SHANK_R  # the shelf's underside when it rests on level-2 pins
SHELF = parts.Part(
    "shelf",
    pos=(CX, 0.0, TOP + SUPPORT + 0.0005),
    mass=SHELF_MASS,
    shapes=[
        parts.box(BOARD, at=(0.0, 0.0, BOARD[2] / 2), color=BLUE),
        parts.box((0.015, BOARD[1], 0.015), at=(LIP_X, 0.0, 0.0175), color=BLUE),
        parts.box((0.010, 0.010, 0.010), at=(LIP_X, 0.075, 0.030), color=RAIL),
        parts.box((0.010, 0.010, 0.010), at=(LIP_X, -0.075, 0.030), color=RAIL),
        parts.cylinder(RAIL_R, 0.170, at=(LIP_X, 0.0, RAIL_Z), color=RAIL, axis="y"),
    ],
)

LOT_A_LEVELS = {"pin_fl": 0, "pin_fr": 1, "pin_bl": 1, "pin_br": 1}
PIN_PARTS = {n: _pin(n, LOT_A_LEVELS[n]) for n in PINS}
OBJECTS = [UPRIGHT_L, UPRIGHT_R, SHELF, *PIN_PARTS.values()]

# Lot A (the screenshot): the front-left pin one level low. Lot B: the front-right pin one level high (the shelf is
# set down from above it).
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", wrong="pin_fl", start_level=0, place={"shelf": SHELF.pos}),
    lots.Lot("B", wrong="pin_fr", start_level=2,
             place={"pin_fl": pin_pos("pin_fl", 1), "pin_fr": pin_pos("pin_fr", 2),
                    "shelf": (CX, 0.0, TOP + LEVELS[2] + SEAT_DZ + SHANK_R + 0.0005)}),
)

EPS = 1e-6
SEAT_X, SEAT_Y, SEAT_Z = 0.004, 0.004, 0.006  # seated: collar within 4 mm of the face; axis within 4 / 6 mm
SEAT_ANGLE = math.radians(10.0)
FLAT = math.radians(2.0)
SHELF_DZ = 0.004  # the underside within 4 mm of the level-2 support height
SHELF_XY = 0.015
MOVE_X, MOVE_AXIAL, MOVE_Z = 0.008, 0.005, 0.008  # a correct pin "moved": 8 mm sideways, 5 along its hole, 8 up or down
ON_TABLE = 0.020
CLEAR = 0.005


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def seated(name, st, level) -> bool:
    """The pin's shank through the hole of its own column at `level`, its collar against the upright's outer face."""
    x, side = PINS[name]
    p = np.asarray(st.pos, float)
    out = float(_rot(st.quat_xyzw)[:, 1] @ np.array([0.0, side, 0.0]))
    return bool(
        out >= math.cos(SEAT_ANGLE) - EPS
        and abs(p[0] - x) <= SEAT_X + EPS
        and abs(p[1] - side * OUTER) <= SEAT_Y + EPS
        and abs(p[2] - (TOP + LEVELS[level])) <= SEAT_Z + EPS
    )


def _level(shelf) -> bool:
    p = np.asarray(shelf.pos, float)
    tilt = math.acos(float(np.clip(_rot(shelf.quat_xyzw)[2, 2], -1.0, 1.0)))
    return bool(
        tilt <= FLAT + EPS
        and abs(p[2] - (TOP + SUPPORT)) <= SHELF_DZ + EPS
        and abs(p[0] - CX) <= SHELF_XY + EPS
        and abs(p[1]) <= SHELF_XY + EPS
    )


# Whole-trial record: has any of the three correctly placed pins moved more than 5 mm along its hole or 8 mm sideways,
# or 8 mm up or down, from where it settled at the start? Cleared at every scene reset (an on_reset hook in the
# simulator; offline, all four pins back within 0.5 mm of their starts after a jump of over 20 mm in one step).
_record = {"fault": False, "base": None, "prev": None}
# Released: no finger, gripper body or wrist link within 5 mm. Set every step in the simulator; offline there is no
# robot, so it stays True.
_grip = {"clear": True}


@parts.on_reset
def _new_trial(sim) -> None:
    _record.update(fault=False, base=None, prev=None)
    _grip["clear"] = True


def _update(objects) -> None:
    lot = lots.current()
    now = {n: np.asarray(objects[n].pos, float) for n in PINS}
    prev = _record["prev"]
    if prev is not None:
        home = all(np.linalg.norm(now[n] - np.asarray(lot.pos(n))) < 0.0005 and objects[n].resting() for n in PINS)
        if home and max(np.linalg.norm(now[n] - prev[n]) for n in PINS) > 0.020:
            _record.update(fault=False, base=None)
    _record["prev"] = now
    if _record["base"] is None:
        _record["base"] = dict(now)
    for n in PINS:
        if n == lot.wrong:
            continue
        d = now[n] - _record["base"][n]
        if abs(d[0]) > MOVE_X + EPS or abs(d[1]) > MOVE_AXIAL + EPS or abs(d[2]) > MOVE_Z + EPS:
            _record["fault"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    boxes = []
    for name, part in [("shelf", SHELF), *PIN_PARTS.items()]:
        s = st[name]
        boxes += part.world_boxes({"base": (np.asarray(s.pos), _rot(s.quat_xyzw))})
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right", "gripper", "link5"):  # fingers, gripper body, wrist
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for _, c, r, s in boxes)


def _pins_ok(objects) -> bool:
    return all(seated(n, objects[n], 1) and objects[n].resting() for n in PINS)


def _done(objects) -> bool:
    shelf = objects["shelf"]
    return bool(not _record["fault"] and _pins_ok(objects) and _level(shelf) and shelf.resting() and _grip["clear"])


def success(objects):
    """All four pins in their level-2 holes and the shelf flat on them, let go and still; no correctly placed pin
    was moved during the trial."""
    _update(objects)
    return _done(objects)


def distance(objects):
    lot = lots.current()
    x, side = PINS[lot.wrong]
    goal = np.array([x, side * OUTER, TOP + LEVELS[1]])
    return float(np.linalg.norm(np.asarray(objects[lot.wrong].pos) - goal))


def score(objects):
    """1.0 on success. 0.5: all four pins in their level-2 holes, everything let go and still, the shelf not on the
    table, no correct pin moved, but the shelf not flat on them. Otherwise 0."""
    _update(objects)
    if _done(objects):
        return 1.0
    shelf = objects["shelf"]
    if (not _record["fault"] and _pins_ok(objects) and _grip["clear"] and shelf.resting()
            and shelf.low[2] > TOP + ON_TABLE + EPS):
        return 0.5
    return 0.0


# --- demo ----------------------------------------------------------------------------------------------------------
TIP = 0.013  # pad tips below the TCP
NARROW = FINGER_OPEN * 0.020 / 0.094  # pads about 20 mm apart
MID = FINGER_OPEN * 0.040 / 0.094  # pads about 40 mm apart
RAIL_X = CX + LIP_X
LIFT = 0.032  # the rail raised this far above its level-2 height while the pin is moved
PULL = 0.090  # the pin drawn out this far, so its inner end clears the upright
# The rail's height at the start in each lot (from the settled start in a trace; see README).
RAIL_START = {"A": TOP + SUPPORT + RAIL_Z - 0.011, "B": TOP + SUPPORT + RAIL_Z + 0.009}
DX = {"A": -0.0025, "B": 0.0045}  # tracking compensation while the pin is carried, from traces
INSERT_DZ = 0.0025  # the carried pin rides about 0.6 mm low; aim its axis 2 mm above the seated height


def demo():
    """One arm holds the shelf up by its front rail; the other draws the wrong pin out by its T-handle, moves it to
    the level-2 hole of the same column and pushes it home; then the shelf is set down on the four pins. Which pin
    and which arm follow the trial's layout, as the cameras show it."""
    lot = lots.current()
    worker = "left" if PINS[lot.wrong][1] > 0 else "right"
    lifter = "right" if worker == "left" else "left"
    x, side = PINS[lot.wrong]
    ry = -0.03 * side  # the lifting arm takes the rail on the far side of the midline
    flat = TOP + SUPPORT + RAIL_Z
    r0 = RAIL_START[lot.name]
    grip_dz = TIP - 0.004  # pad tips 4 mm below the rail's axis
    yaw = math.pi / 2 if lifter == "right" else -math.pi / 2  # each wrist turned the way it reaches
    lift = [
        Waypoint(1.5, (RAIL_X, ry, r0 + 0.06), MID, "over the rail", yaw),
        Waypoint(2.0, (RAIL_X, ry, r0 + grip_dz), MID, "down to the rail", yaw),
        Waypoint(0.8, (RAIL_X, ry, r0 + grip_dz), FINGER_CLOSED, "grip the rail", yaw),
        Waypoint(2.0, (RAIL_X, ry, flat + LIFT + grip_dz), FINGER_CLOSED, "hold the shelf up", yaw),
        Waypoint(12.2, (RAIL_X, ry, flat + LIFT + grip_dz), FINGER_CLOSED, "hold", yaw),
        Waypoint(2.0, (RAIL_X, ry, flat + grip_dz - 0.003), FINGER_CLOSED, "set the shelf down", yaw),
        Waypoint(0.8, (RAIL_X, ry, flat + grip_dz - 0.003), MID, "let go", yaw),
        Waypoint(1.5, (RAIL_X, ry, flat + 0.08), MID, "up", yaw),
        Waypoint(1.0, (RAIL_X - 0.08, ry, flat + 0.08), MID, "back", yaw),
    ]
    ty = side * (OUTER + TAB_Y)  # the handle's centre when seated
    z0 = TOP + LEVELS[lot.start_level] + SEAT_DZ
    z1 = TOP + LEVELS[1] + SEAT_DZ + INSERT_DZ
    cx = x + DX[lot.name]
    pz = 0.006 + TIP  # pad tips 6 mm above the axis, on the handle above the shank
    work = [
        Waypoint(1.5, (x, ty, z0 + 0.08), NARROW, "over the handle"),
        Waypoint(5.0, (x, ty, z0 + 0.08), NARROW, "wait for the shelf to be held up"),
        Waypoint(2.0, (x, ty, z0 + pz), NARROW, "down to the handle"),
        Waypoint(0.8, (x, ty, z0 + pz), FINGER_CLOSED, "grip the handle"),
        Waypoint(2.5, (cx, ty + side * PULL, z0 + pz), FINGER_CLOSED, "draw the pin out"),
        Waypoint(1.5, (cx, ty + side * PULL, z1 + pz), FINGER_CLOSED, "to the level-2 hole"),
        Waypoint(3.0, (cx, ty - side * 0.002, z1 + pz), FINGER_CLOSED, "push it home"),
        Waypoint(0.6, (cx, ty - side * 0.002, z1 + pz), NARROW, "let go"),
        Waypoint(1.2, (x, ty, z1 + 0.08), NARROW, "up"),
        Waypoint(1.0, (x, ty + side * 0.04, z1 + 0.08), NARROW, "aside"),
    ]
    return {worker: work, lifter: lift}
