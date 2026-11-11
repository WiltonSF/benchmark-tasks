"""On the station table, between the arms: a steel sorting nest. A cross-shaped channel on top is roofed over, too low
for a hand to enter, and a small aluminium part stands at the centre where its arms meet. Three square drop-throughs
open in the channel floor, one to the robot's left, one to its right and one beyond the centre; each opens above a
bore in the nest and is open above through the roof. A yellow tote stands in one of the bores, its rim
just below the channel floor; the other bores are bare down to a black floor. Three slide knock-outs, each with a red knob above the roof, sit at the left, right and
near ends of the channel. Which bore holds the tote changes between trials.
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
from yam_arena.task_file import TABLE_TOP, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, DARK, BLACK = (0.55, 0.57, 0.60), (0.20, 0.21, 0.23), (0.04, 0.04, 0.05)
RED, BLADE, YELLOW, ALU = (0.80, 0.10, 0.08), (0.40, 0.42, 0.45), (0.95, 0.78, 0.10), (0.82, 0.83, 0.85)

# Nest, in its own frame: origin on the table under the channel centre, x away from the robot, y to its left, z up.
X0, Y0 = 0.27, 0.0
FLOOR = 0.050  # channel floor, above the table
ROOF_UNDER, ROOF_TOP = 0.086, 0.092  # 36 mm headroom
HALF_W = 0.020  # channel half-width
Y_END, X_NEAR, X_FAR = 0.096, -0.036, 0.096  # inner faces of the channel's end walls
XB = (-0.048, 0.108)  # nest footprint
YB = (-0.108, 0.108)
# Drop-throughs (and the bores under them): (x0, x1, y0, y1) in the nest frame. Each is 50 mm along its arm, 40 across.
BORES = {
    "left": (-0.020, 0.020, 0.030, 0.080),
    "right": (-0.020, 0.020, -0.080, -0.030),
    "far": (0.030, 0.080, -0.020, 0.020),
}
BORE_FLOOR = 0.004  # black plate on the table at the bottom of each bore


def _in_cross(x, y) -> bool:
    return (abs(x) < HALF_W and abs(y) < Y_END) or (abs(y) < HALF_W and X_NEAR < x < X_FAR)


def _in_bore(x, y):
    for name, (x0, x1, y0, y1) in BORES.items():
        if x0 < x < x1 and y0 < y < y1:
            return name
    return None


def _nest_shapes():
    """The fixed nest as boxes on a grid: solid outside the channel, floor and roof inside it, open over the bores. The
    drop-throughs are 42 mm deep from the roof top to the channel floor, so from the top camera (about 61 degrees
    down, from behind the robot) only a strip of a bore's far side shows; straight down from above a drop-through,
    the tote or the bare black bore shows plainly."""
    xs = [XB[0], X_NEAR, -HALF_W, HALF_W, 0.030, 0.080, X_FAR, XB[1]]
    ys = [YB[0], -Y_END, -0.080, -0.030, -HALF_W, HALF_W, 0.030, 0.080, Y_END, YB[1]]
    out = []
    for i in range(len(xs) - 1):
        x0, x1 = xs[i], xs[i + 1]
        runs: list[list] = []  # merge neighbouring cells of one kind along y
        for j in range(len(ys) - 1):
            y0, y1 = ys[j], ys[j + 1]
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            kind = "bore" if _in_bore(cx, cy) else ("channel" if _in_cross(cx, cy) else "solid")
            if runs and runs[-1][0] == kind:
                runs[-1][2] = y1
            else:
                runs.append([kind, y0, y1])
        for kind, y0, y1 in runs:
            sx, sy, cx, cy = x1 - x0, y1 - y0, (x0 + x1) / 2, (y0 + y1) / 2
            if kind == "solid":
                out.append(parts.box((sx, sy, ROOF_TOP), at=(cx, cy, ROOF_TOP / 2), color=STEEL))
            elif kind == "channel":
                if not (HALF_W < cx < 0.030 and abs(cy) < HALF_W):  # that floor is part of the far run below
                    out.append(parts.box((sx, sy, FLOOR), at=(cx, cy, FLOOR / 2), color=STEEL))
                out.append(parts.box((sx, sy, ROOF_TOP - ROOF_UNDER), at=(cx, cy, (ROOF_UNDER + ROOF_TOP) / 2),
                                     color=STEEL))
            else:
                out.append(parts.box((sx, sy, BORE_FLOOR), at=(cx, cy, BORE_FLOOR / 2), color=BLACK))

    # One unbroken floor from the centre to the far drop-through, overlapping the centre: a seam across the part's
    # path would catch it.
    out.append(parts.box((0.030 + HALF_W, 2 * HALF_W, FLOOR), at=((0.030 - HALF_W) / 2, 0.0, FLOOR / 2), color=STEEL))
    # Dark slots in the roof where the knob stems run.
    out.append(parts.box((0.006, 2 * Y_END, 0.001), at=(0.0, 0.0, ROOF_TOP + 0.0005), color=DARK))
    out.append(parts.box((0.082, 0.006, 0.001), at=(0.005, 0.0, ROOF_TOP + 0.0005), color=DARK))
    return out


# Knock-outs: a blade in the channel (bottom 2 mm above the floor), a stem through the roof slot and a red knob.
BLADE_T, BLADE_W, BLADE_H = 0.008, 0.036, 0.030
BLADE_Z = FLOOR + 0.002 + BLADE_H / 2
KNOB = 0.020
KNOB_Z = ROOF_TOP + 0.006 + 0.024 / 2  # knob 98 to 122 mm above the table
STEM_Z0 = FLOOR + 0.002 + BLADE_H
END_REST = 0.088  # the left and right blades' centres, beyond the side drop-throughs
NEAR_REST = -0.028
TRAVEL_SIDE, TRAVEL_NEAR = 0.130, 0.070


def _knockout(name, at_xy, along_x, axis, travel):
    x, y = at_xy
    blade = (BLADE_T, BLADE_W, BLADE_H) if along_x else (BLADE_W, BLADE_T, BLADE_H)
    stem_h = KNOB_Z - 0.012 - STEM_Z0
    return parts.Slider(
        name, axis=axis, travel=travel, mass=0.05, damping=2.0,
        shapes=[
            parts.box(blade, at=(x, y, BLADE_Z), color=BLADE),
            parts.box((0.006, 0.006, stem_h), at=(x, y, STEM_Z0 + stem_h / 2), color=BLADE),
            parts.box((KNOB, KNOB, 0.024), at=(x, y, KNOB_Z), color=RED),
        ],
    )


NEST = parts.Part(
    "sorting_nest",
    pos=(X0, Y0, TOP),
    shapes=_nest_shapes(),
    joints=[
        _knockout("left_knockout", (0.0, END_REST), False, (0.0, -1.0, 0.0), TRAVEL_SIDE),
        _knockout("right_knockout", (0.0, -END_REST), False, (0.0, 1.0, 0.0), TRAVEL_SIDE),
        _knockout("near_knockout", (NEAR_REST, 0.0), True, (1.0, 0.0, 0.0), TRAVEL_NEAR),
    ],
)

# The part: an upright aluminium slug at the channel centre.
PART_R, PART_H = 0.010, 0.028
PART = Cylinder("part", pos=(X0, Y0, TOP + FLOOR + PART_H / 2), radius=PART_R, height=PART_H, mass=0.06, color=ALU)

# Tote: 38 mm square, 43 mm tall, 2 mm walls on a 4 mm floor; its rim 3 mm below the channel floor. It stands in a
# bore against the bore wall nearest the channel centre. Origin: the centre of its underside.
TOTE_OUT, TOTE_WALL, TOTE_FLOOR = 0.038, 0.002, 0.004
TOTE_H = FLOOR - 0.003 - BORE_FLOOR
TOTE_INNER = TOTE_OUT / 2 - TOTE_WALL  # 17 mm
_w, _h = TOTE_OUT, TOTE_H
TOTE = parts.Part(
    "tote",
    pos=(0.0, 0.0, 0.0),  # replaced below
    mass=0.4,
    shapes=[
        parts.box((_w, _w, TOTE_FLOOR), at=(0.0, 0.0, TOTE_FLOOR / 2), color=YELLOW),
        parts.box((_w, TOTE_WALL, _h - TOTE_FLOOR), at=(0.0, _w / 2 - TOTE_WALL / 2, (_h + TOTE_FLOOR) / 2), color=YELLOW),
        parts.box((_w, TOTE_WALL, _h - TOTE_FLOOR), at=(0.0, -_w / 2 + TOTE_WALL / 2, (_h + TOTE_FLOOR) / 2),
                  color=YELLOW),
        parts.box((TOTE_WALL, _w - 2 * TOTE_WALL, _h - TOTE_FLOOR), at=(_w / 2 - TOTE_WALL / 2, 0.0, (_h + TOTE_FLOOR) / 2),
                  color=YELLOW),
        parts.box((TOTE_WALL, _w - 2 * TOTE_WALL, _h - TOTE_FLOOR),
                  at=(-_w / 2 + TOTE_WALL / 2, 0.0, (_h + TOTE_FLOOR) / 2), color=YELLOW),
    ],
)
_GAP = 0.001  # tote to the bore wall nearest the centre
TOTE_AT = {
    "right": (X0, Y0 - 0.030 - _GAP - TOTE_OUT / 2, TOP + BORE_FLOOR),
    "left": (X0, Y0 + 0.030 + _GAP + TOTE_OUT / 2, TOP + BORE_FLOOR),
    "far": (X0 + 0.030 + _GAP + TOTE_OUT / 2, Y0, TOP + BORE_FLOOR),
}
TOTE = parts.Part("tote", pos=TOTE_AT["right"], mass=TOTE.mass, shapes=TOTE.shapes)

OBJECTS = [NEST, PART, TOTE]

# Lot A (the screenshot): the tote in the right bore. Lot B: the left bore. Lot C: the far bore.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", bore="right", place={"tote": TOTE_AT["right"]}),
    lots.Lot("B", bore="left", place={"tote": TOTE_AT["left"]}),
    lots.Lot("C", bore="far", place={"tote": TOTE_AT["far"]}),
)

EPS = 1e-6
TOTE_MOVE, TOTE_TILT = 0.005, math.radians(10.0)
DROPPED = 0.005  # the part's lowest point more than 5 mm below the channel floor: it has gone down a bore
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the part
SUCCESS_HOLD_S = 3.0  # long enough for the demo to lift off and go back before the trial ends


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(state) -> float:
    return math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0)))


def _tote_disturbed(tote) -> bool:
    start = np.asarray(lots.current().pos("tote"), float)
    return bool(np.linalg.norm(np.asarray(tote.pos) - start) > TOTE_MOVE + EPS or _tilt(tote) > TOTE_TILT + EPS)


def _in_tote(part, tote) -> bool:
    """The part's centre inside the tote's inner walls, above its floor and below its rim; its top below the channel floor. (The
    centre, not the bounding box's bottom: a tilted cylinder's box reaches below the cylinder itself.)"""
    local = _rot(tote.quat_xyzw).T @ (np.asarray(part.pos) - np.asarray(tote.pos))
    return bool(
        abs(local[0]) <= TOTE_INNER + EPS and abs(local[1]) <= TOTE_INNER + EPS
        and part.high[2] <= TOP + FLOOR + EPS
        and part.pos[2] >= tote.pos[2] + TOTE_FLOOR - EPS
        and part.pos[2] <= tote.pos[2] + TOTE_H + EPS
    )


def _over_tote(part, tote) -> bool:
    local = _rot(tote.quat_xyzw).T @ (np.asarray(part.pos) - np.asarray(tote.pos))
    return bool(abs(local[0]) <= TOTE_OUT / 2 + EPS and abs(local[1]) <= TOTE_OUT / 2 + EPS)


def _down_bare_bore(part, tote) -> bool:
    """The part has gone more than 5 mm below the channel floor anywhere but over the tote."""
    return bool(part.low[2] < TOP + FLOOR - DROPPED - EPS and not _over_tote(part, tote))


# Whole-trial record: did the part ever go down a bore without the tote, or the tote move over 5 mm or tilt over 10
# degrees? Cleared only by a scene reset: the part back within 0.5 mm of its start and at rest, after a jump of over
# 20 mm in one control step (a robot cannot move it back that way, and it cannot come back out of a bore).
_record = {"fault": False, "prev": None}
# Released: in the simulator an on_step callback measures, every step, how close both arms' fingers and grippers come
# to the part. Offline (near_misses.py) there is no robot, so "clear" stays True.
_grip = {"clear": True}


def _update(objects) -> None:
    part, tote = objects["part"], objects["tote"]
    pos = np.asarray(part.pos, float)
    prev = _record["prev"]
    if prev is not None and np.linalg.norm(pos - np.asarray(PART.pos)) < 0.0005 and part.resting():
        if np.linalg.norm(pos - prev) > 0.020:
            _record["fault"] = False
    _record["prev"] = pos
    if _down_bare_bore(part, tote) or _tote_disturbed(tote):
        _record["fault"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    part = sim.states()["part"]
    size = np.array([2 * PART_R, 2 * PART_R, PART_H])
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
    _grip["clear"] = analysis.hulls_to_box(hulls, np.asarray(part.pos), _rot(part.quat_xyzw), size)[0] > CLEAR


def _done(objects) -> bool:
    part, tote = objects["part"], objects["tote"]
    return bool(_in_tote(part, tote) and part.resting() and tote.resting() and _grip["clear"])


def success(objects):
    """The part rests in the tote, let go; it never went down a bare bore and the tote was not moved."""
    _update(objects)
    return bool(not _record["fault"] and _done(objects))


def distance(objects):
    return float(np.linalg.norm(objects["part"].pos[:2] - objects["tote"].pos[:2]))


def score(objects):
    """Pass/fail: 1.0 on success, otherwise 0."""
    _update(objects)
    return 1.0 if not _record["fault"] and _done(objects) else 0.0


# --- demo: one arm pushes one knob -------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
PUSH_Z = TOP + KNOB_Z - 0.008 + TIP  # tips 4 mm above the knob's underside, 10 mm above the roof
TRAVEL = TOP + 0.15
STANDOFF = KNOB / 2 + 0.020  # TCP this far from the knob's centre before the push
OVERSHOOT = 0.008  # commanded past the knob's end stop: the arm gives way, the stop holds the knob


LOOK_Z = TOP + 0.125
PARK = {"left": (0.22, 0.22), "right": (0.22, -0.22)}


def _above(name):
    x0, x1, y0, y1 = BORES[name]
    return (X0 + (x0 + x1) / 2, Y0 + (y0 + y1) / 2)


def _look():
    """Look down each drop-through with a wrist camera, one arm at a time: the left arm over the left and far
    drop-throughs, then the right arm over the right one. Each arm then parks to its own side."""
    w = lambda d, xy, label: Waypoint(d, (*xy, LOOK_Z), FINGER_CLOSED, label)  # noqa: E731
    left = [w(1.5, _above("left"), "over the left drop-through"), w(0.8, _above("left"), "look"),
            w(1.2, _above("far"), "over the far drop-through"), w(0.8, _above("far"), "look"),
            w(1.5, PARK["left"], "park")]
    t_left = sum(x.duration for x in left)
    right = [w(t_left, PARK["right"], "wait parked"), w(1.5, _above("right"), "over the right drop-through"),
             w(0.8, _above("right"), "look"), w(1.5, PARK["right"], "park")]
    t = sum(x.duration for x in right)
    left.append(w(t - t_left, PARK["left"], "wait parked"))
    return {"left": left, "right": right}


def _push(arm, start, end):
    (sx, sy), (ex, ey) = start, end
    return [
        Waypoint(1.5, (sx, sy, TRAVEL), FINGER_CLOSED, "over the knob's outer side"),
        Waypoint(1.5, (sx, sy, PUSH_Z), FINGER_CLOSED, "down beside the knob"),
        Waypoint(4.0, (ex, ey, PUSH_Z), FINGER_CLOSED, "push the knob along its slot"),
        Waypoint(0.5, (ex, ey, PUSH_Z), FINGER_CLOSED, "hold at the stop"),
        Waypoint(0.8, (ex, ey, PUSH_Z + 0.015), FINGER_CLOSED, "up off the knob"),
        Waypoint(1.5, (sx, sy, TRAVEL), FINGER_CLOSED, "back over the start"),
        Waypoint(1.5, (*PARK[arm], LOOK_Z), FINGER_CLOSED, "park"),
        Waypoint(4.0, (*PARK[arm], LOOK_Z), FINGER_CLOSED, "wait while the part settles in the tote"),
    ]


def demo():
    """Look down the three drop-throughs, then push the knob on the side opposite the drop-through with the tote
    under it, so the part goes down that drop-through: the left arm for the left or near knob, the right arm for the
    right knob. Which drop-through showed the tote is the trial's lot."""
    plan = _look()
    bore = lots.current().bore
    if bore == "right":  # left knob pushed toward the robot's right
        y0 = Y0 + END_REST + STANDOFF
        y1 = Y0 + END_REST - TRAVEL_SIDE + KNOB / 2 - OVERSHOOT
        plan["left"] += _push("left", (X0, y0), (X0, y1))
    elif bore == "left":
        y0 = Y0 - END_REST - STANDOFF
        y1 = Y0 - END_REST + TRAVEL_SIDE - KNOB / 2 + OVERSHOOT
        plan["right"] += _push("right", (X0, y0), (X0, y1))
    else:
        x0 = X0 + NEAR_REST - STANDOFF
        x1 = X0 + NEAR_REST + TRAVEL_NEAR - KNOB / 2 + OVERSHOOT
        plan["left"] += _push("left", (x0, Y0), (x1, Y0))
    return plan
