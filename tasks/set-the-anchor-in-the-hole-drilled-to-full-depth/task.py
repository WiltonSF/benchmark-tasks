"""On the station table, in front of the arms: a grey concrete slab with three drilled holes in a row from left to
right, all the same size and all dark inside. To its left, nearer the robot, a yellow holder in which a steel anchor
bolt stands upright: a round shaft, a wide washer and a square nut on top.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import lots  # noqa: E402
import parts  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

CONCRETE, DARK, YELLOW = (0.62, 0.62, 0.60), (0.10, 0.10, 0.11), (0.90, 0.72, 0.10)
STEEL, NUT = (0.72, 0.74, 0.76), (0.55, 0.57, 0.60)

# Slab: fixed, 80 mm front to back, 240 mm left to right, 60 mm tall. Three 18 mm square openings, each lined with
# 1 mm dark liners, so every hole is 16 mm square. Every hole is drilled 50 mm deep (a 10 mm floor). In two of them a
# hidden dark steel plug, 25 mm tall, stands on the floor: that is the hole that hit rebar, 25 mm deep.
SLAB_X, SLAB_Y = 0.32, 0.145
SLAB = (0.080, 0.240, 0.060)
SLAB_H = SLAB[2]
OPEN = 0.018
LINER = 0.001
HOLE = OPEN - 2 * LINER  # 16 mm
FLOOR = 0.010
HOLE_DY = (0.075, 0.0, -0.075)  # hole 1 (left), 2 (middle), 3 (right), from the slab's centre
HOLES = [(SLAB_X, SLAB_Y + dy) for dy in HOLE_DY]


def _slab_shapes():
    sx, sy, sz = SLAB
    o = OPEN / 2
    out = [
        parts.box(((sx / 2 - o), sy, sz), at=(-(sx / 2 + o) / 2, 0.0, sz / 2), color=CONCRETE),  # near wall
        parts.box(((sx / 2 - o), sy, sz), at=((sx / 2 + o) / 2, 0.0, sz / 2), color=CONCRETE),  # far wall
    ]
    edges = [-sy / 2] + [e for dy in sorted(HOLE_DY) for e in (dy - o, dy + o)] + [sy / 2]
    for a, b in zip(edges[0::2], edges[1::2]):
        out.append(parts.box((OPEN, b - a, sz), at=(0.0, (a + b) / 2, sz / 2), color=CONCRETE))
    depth = sz - FLOOR
    for dy in HOLE_DY:
        out.append(parts.box((OPEN, OPEN, FLOOR), at=(0.0, dy, FLOOR / 2), color=DARK))
        zc = FLOOR + depth / 2
        for s in (-1, 1):
            out.append(parts.box((LINER, OPEN, depth), at=(s * (o - LINER / 2), dy, zc), color=DARK))
            out.append(parts.box((HOLE, LINER, depth), at=(0.0, dy + s * (o - LINER / 2), zc), color=DARK))
    return out


SLAB_PART = parts.Part("slab", pos=(SLAB_X, SLAB_Y, TOP), shapes=_slab_shapes())

# Plugs: loose only so the lots can move them; they sit in the holes and cannot be reached.
PLUG = (0.015, 0.015, 0.025)
PLUG_MASS = 0.15


def _plug_at(i):
    x, y = HOLES[i]
    return (x, y, TOP + FLOOR + PLUG[2] / 2)


PLUG_P = Box("plug_p", pos=_plug_at(0), size=PLUG, color=DARK, mass=PLUG_MASS)
PLUG_Q = Box("plug_q", pos=_plug_at(2), size=PLUG, color=DARK, mass=PLUG_MASS)
PROUD = PLUG[2] - (SLAB[2] - FLOOR - 0.045)  # 20 mm: the washer above the slab when the shaft stands on a plug
# Anchor bolt. Part frame: origin at the centre of the washer's underside, z up along the shaft.
SHAFT_R, SHAFT_L = 0.006, 0.040
TIP_R, TIP_L = 0.0045, 0.005  # a narrower tip, standing in for the anchor's chamfered nose
LENGTH = SHAFT_L + TIP_L  # 45 mm below the washer
WASHER_R, WASHER_T = 0.013, 0.004
NUT_W, NUT_H = 0.012, 0.018
ANCHOR_MASS = 0.06

# Holder: fixed yellow block 44 x 44 x 42 mm with a 16 mm square hole through it to the table.
HOLDER_XY = (0.20, 0.28)
HOLDER = (0.044, 0.044, 0.042)


def _holder_shapes():
    sx, sy, sz = HOLDER
    o = HOLE / 2
    w = (sx / 2 - o)
    return [
        parts.box((w, sy, sz), at=(-(sx / 2 + o) / 2, 0.0, sz / 2), color=YELLOW),
        parts.box((w, sy, sz), at=((sx / 2 + o) / 2, 0.0, sz / 2), color=YELLOW),
        parts.box((HOLE, w, sz), at=(0.0, -(sy / 2 + o) / 2, sz / 2), color=YELLOW),
        parts.box((HOLE, w, sz), at=(0.0, (sy / 2 + o) / 2, sz / 2), color=YELLOW),
    ]


HOLDER_PART = parts.Part("holder", pos=(*HOLDER_XY, TOP), shapes=_holder_shapes())

ANCHOR_START_Z = TOP + LENGTH  # shaft tip on the table inside the holder; the washer 3 mm above the holder
ANCHOR = parts.Part(
    "anchor",
    pos=(*HOLDER_XY, ANCHOR_START_Z),
    mass=ANCHOR_MASS,
    shapes=[
        parts.cylinder(SHAFT_R, SHAFT_L, at=(0.0, 0.0, -SHAFT_L / 2), color=STEEL, axis="z"),
        parts.cylinder(TIP_R, TIP_L, at=(0.0, 0.0, -SHAFT_L - TIP_L / 2), color=STEEL, axis="z"),
        parts.cylinder(WASHER_R, WASHER_T, at=(0.0, 0.0, WASHER_T / 2), color=STEEL, axis="z"),
        parts.box((NUT_W, NUT_W, NUT_H), at=(0.0, 0.0, WASHER_T + NUT_H / 2), color=NUT),
    ],
)

OBJECTS = [SLAB_PART, HOLDER_PART, ANCHOR, PLUG_P, PLUG_Q]

# Lot A (the screenshot): the full-depth hole is the middle one. Lot B: the right one. Lot C: the left one.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", deep=1),
    lots.Lot("B", deep=2, place={"plug_p": _plug_at(0), "plug_q": _plug_at(1)}),
    lots.Lot("C", deep=0, place={"plug_p": _plug_at(1), "plug_q": _plug_at(2)}),
)

EPS = 1e-6
CENTRE = 0.003  # the anchor's axis within 3 mm of the hole's centre, front to back and left to right
FLUSH = 0.001  # the washer's underside within 1 mm of the slab top
MAX_TILT = math.radians(3.0)
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the anchor
SUCCESS_HOLD_S = 1.0


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tilt_of(state) -> float:
    return math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0)))


# Released: an on_step callback measures, every step, how close both arms' fingers and grippers come to the anchor.
# Offline (near_misses.py) there is no robot, so "clear" stays True.
_grip = {"clear": True}


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    a = sim.states()["anchor"]
    boxes = ANCHOR.world_boxes({"base": (np.asarray(a.pos), _rot(a.quat_xyzw))})
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


def seated_in(objects) -> int | None:
    """The hole (0, 1, 2) the anchor stands seated in, flush and upright, or None."""
    a = objects["anchor"]
    if tilt_of(a) > MAX_TILT + EPS:
        return None
    if abs(float(a.pos[2]) - (TOP + SLAB_H)) > FLUSH + EPS:
        return None
    for i, (hx, hy) in enumerate(HOLES):
        if abs(float(a.pos[0]) - hx) <= CENTRE + EPS and abs(float(a.pos[1]) - hy) <= CENTRE + EPS:
            return i
    return None


def success(objects):
    """The anchor stands flush and upright in the full-depth hole, let go, and everything is still."""
    hole = seated_in(objects)
    still = all(objects[n].resting() for n in ("anchor", "plug_p", "plug_q"))
    return bool(hole is not None and hole == lots.current().deep and still and _grip["clear"])


def distance(objects):
    hx, hy = HOLES[lots.current().deep]
    a = objects["anchor"].pos
    return float(np.linalg.norm([a[0] - hx, a[1] - hy, a[2] - (TOP + SLAB_H)]))


def score(objects):
    """Pass/fail: 1.0 on success, otherwise 0."""
    return 1.0 if success(objects) else 0.0


# --- demo: left arm only ----------------------------------------------------------------------------------------
GRIP_DZ = 0.020  # TCP above the washer's underside: the pad tips 7 mm up the nut
NUT_PINCH = -0.003  # pads 6 mm apart: about 3 mm past each face of the 12 mm nut
WIDE = -0.015  # pads 30 mm apart
EASE = -0.009  # pads 18 mm apart: let go of the nut gently, 3 mm clear of each face
CARRY = 0.125  # washer underside while carried: the shaft tip 80 mm up, 20 mm over the slab
DROP = SLAB_H + 0.020  # let go with the washer 20 mm above flush: the shaft tip 25 mm into the hole (the first
# traces reached 5 mm short of the commanded depth), 5 mm over a plug
# Where the held anchor settles against the commanded TCP differs by hole (traced): about 3 mm short toward the robot,
# and, farther right, up to 3 mm to the robot's left. The demo aims off by these amounts.
AIM = ((0.003, 0.0), (0.003, -0.0015), (0.004, -0.003))
TRAVEL = 0.16


def _w(x, y, z):
    return (x, y, TOP + z)


def demo():
    """Left arm: lift the anchor out of its holder, then try it in each hole in turn, left to right: lower its tip
    into the hole and let go. Where it stops 20 mm proud it is gripped again and lifted out; where it drops flush it
    is left. The number of tries follows the lot: it stands for seeing the anchor stop proud in the cameras."""
    hx0, hy0 = HOLDER_XY
    g = GRIP_DZ
    plan = [
        Waypoint(1.5, _w(hx0, hy0, TRAVEL), WIDE, "over the anchor"),
        Waypoint(1.5, _w(hx0, hy0, 0.045 + g), WIDE, "down to the nut"),
        Waypoint(0.8, _w(hx0, hy0, 0.045 + g), NUT_PINCH, "pinch the nut"),
        Waypoint(1.5, _w(hx0, hy0, CARRY + g), NUT_PINCH, "lift out of the holder"),
    ]
    deep = lots.current().deep
    for i in range(deep + 1):
        x, y = HOLES[i]
        x, y = x + AIM[i][0], y + AIM[i][1]
        plan += [
            Waypoint(2.0, _w(x, y, CARRY + g), NUT_PINCH, f"over hole {i + 1}"),
            Waypoint(1.5, _w(x, y, DROP + g), NUT_PINCH, f"tip into hole {i + 1}"),
            Waypoint(1.0, _w(x, y, DROP + g), EASE, "let go"),
            Waypoint(1.2, _w(x, y, DROP + g + 0.035), WIDE, "rise 35 mm clear and look"),
        ]
        if i < deep:
            proud = SLAB_H + PROUD
            plan += [
                Waypoint(1.0, _w(x, y, proud + g), WIDE, "stopped proud: down to the nut"),
                Waypoint(0.8, _w(x, y, proud + g), NUT_PINCH, "pinch the nut"),
                Waypoint(1.5, _w(x, y, CARRY + g), NUT_PINCH, "lift out"),
            ]
    x, y = HOLES[deep]
    x, y = x + AIM[deep][0], y + AIM[deep][1]
    plan += [
        Waypoint(1.0, _w(x, y, TRAVEL), WIDE, "dropped flush: up"),
        Waypoint(1.0, _w(x - 0.10, y, TRAVEL), WIDE, "back toward the robot"),
        Waypoint(2.5, _w(x - 0.10, y, TRAVEL), WIDE, "wait"),
    ]
    return {"left": plan}
