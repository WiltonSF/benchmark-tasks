"""On the station table, in front of the left arm: a small canvas tool bag with a black hook handle on top. Behind
it and to the robot's right stand two bars on posts, running left to right at the same height: a round red sprinkler
pipe with a brass sprinkler head and a red glass bulb under its middle, and a plain square grey hanging rail. Which bar stands
where changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import lots  # noqa: E402
import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

GREEN, BLACK, RED = (0.20, 0.35, 0.22), (0.08, 0.08, 0.09), (0.80, 0.10, 0.08)
BRASS, GLASS_RED, GREY, DARK = (0.78, 0.62, 0.25), (0.95, 0.25, 0.25), (0.62, 0.64, 0.67), (0.25, 0.26, 0.28)

# Tool bag. Part frame: origin at the centre of the body's underside, x away from the robot, z up. The handle is an
# inverted J: an upright at the bag's near end, a crossbar along x and a lip hanging from its far end. The opening
# under the crossbar (between the upright and the lip) is 40 mm long and 60 mm tall; below the lip it is open
# down to the body, so a bar can pass in under the lip.
BODY = (0.080, 0.050, 0.025)
UPRIGHT_X, LIP_X = -0.025, 0.025  # centres; each 10 mm long, so the opening runs from x -20 to +20 mm
HOOK_W = 0.012
BAR_UNDER = 0.085  # the crossbar's underside, above the bag's underside
CROSSBAR_T = 0.010
LIP_BOTTOM = 0.065
OPENING = 0.020  # half-length of the opening
BAG_MASS = 0.25
BAG_XY = (0.22, 0.17)

BAG = parts.Part(
    "tool_bag",
    pos=(*BAG_XY, TOP),
    mass=BAG_MASS,
    shapes=[
        parts.box(BODY, at=(0.0, 0.0, BODY[2] / 2), color=GREEN),
        parts.box((0.010, HOOK_W, BAR_UNDER + CROSSBAR_T - BODY[2]),
                  at=(UPRIGHT_X, 0.0, (BAR_UNDER + CROSSBAR_T + BODY[2]) / 2), color=BLACK),  # upright
        parts.box((0.060, HOOK_W, CROSSBAR_T), at=(0.0, 0.0, BAR_UNDER + CROSSBAR_T / 2), color=BLACK),  # crossbar
        parts.box((0.010, HOOK_W, BAR_UNDER - LIP_BOTTOM), at=(LIP_X, 0.0, (BAR_UNDER + LIP_BOTTOM) / 2),
                  color=BLACK),  # lip
    ],
)

# Two stands: a 6 mm base plate, two posts and a bar 16 mm across along y, its top 105 mm up.
# They are heavy (3 kg) loose parts only so that the lots can swap them; nothing in the task moves them.
BAR_R, BAR_TOP = 0.008, 0.105
BAR_ZC = BAR_TOP - BAR_R
POST_Y, BAR_LEN, PLATE = 0.080, 0.176, (0.050, 0.180, 0.006)
STAND_MASS = 3.0


def _stand(bar, extra=()):
    return [
        parts.box(PLATE, at=(0.0, 0.0, PLATE[2] / 2), color=DARK),
        parts.box((0.016, 0.016, BAR_ZC - PLATE[2]), at=(0.0, POST_Y, (BAR_ZC + PLATE[2]) / 2), color=DARK),
        parts.box((0.016, 0.016, BAR_ZC - PLATE[2]), at=(0.0, -POST_Y, (BAR_ZC + PLATE[2]) / 2), color=DARK),
        bar,
        *extra,
    ]


# The pipe is round; the rail is a square steel bar 16 mm on a side (a flat top, so a hung bag settles instead of
# rocking on it).
PIPE_BAR = parts.cylinder(BAR_R, BAR_LEN, at=(0.0, 0.0, BAR_ZC), color=RED, axis="y")
RAIL_BAR = parts.box((2 * BAR_R, BAR_LEN, 2 * BAR_R), at=(0.0, 0.0, BAR_ZC), color=GREY)


TEE = parts.cylinder(0.012, 0.030, at=(0.0, 0.0, BAR_ZC), color=RED, axis="y")  # pipe tee at the bar's middle
HEAD_SHAPES = (
    parts.cylinder(0.010, 0.012, at=(0.0, 0.0, 0.079), color=BRASS, axis="z"),  # head body, 73 to 85 mm
    parts.cylinder(0.0035, 0.010, at=(0.0, 0.0, 0.068), color=GLASS_RED, axis="z"),  # glass bulb, 63 to 73 mm
    parts.cylinder(0.016, 0.004, at=(0.0, 0.0, 0.061), color=BRASS, axis="z"),  # deflector, 59 to 63 mm
)
NEAR_SPOT = (0.33, 0.21, TOP)  # behind the bag: the bar nearest it
FAR_SPOT = (0.30, 0.00, TOP)  # to the robot's right
PIPE = parts.Part("sprinkler_pipe", pos=NEAR_SPOT, mass=STAND_MASS, shapes=_stand(PIPE_BAR, (TEE, *HEAD_SHAPES)))
RAIL = parts.Part("hanging_rail", pos=FAR_SPOT, mass=STAND_MASS, shapes=_stand(RAIL_BAR))

OBJECTS = [BAG, PIPE, RAIL]

# Lot A (the screenshot): the pipe nearest the bag, the rail to the right. Lot B swaps the two stands.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A"),
    lots.Lot("B", place={"sprinkler_pipe": FAR_SPOT, "hanging_rail": NEAR_SPOT}),
)

EPS = 1e-6
ON_BAR = 0.003  # the bar's top within 3 mm below the crossbar's underside
MAX_TILT = math.radians(20.0)
HANG_CLEAR = 0.010  # hung: the bag's lowest point more than 10 mm above the table
OFF_TABLE = 0.005  # on the pipe: the bag's lowest point more than 5 mm above the table
HEAD_TOUCH = 0.002  # the bag or a gripper within 2 mm of the sprinkler head
STAND_MOVE, STAND_TILT = 0.010, math.radians(5.0)  # the pipe stand may move at most 10 mm and tilt at most 5 degrees
POST_REACH = POST_Y - 0.008 - HOOK_W / 2  # 66 mm: the hook's centre may be this far from the stand's centre
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the bag
SUCCESS_HOLD_S = 2.0


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(state) -> float:
    return math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0)))


def _hooked_on(bag, stand) -> bool:
    """The stand's bar runs through the hook's opening with its top within 3 mm below the crossbar's underside, at a
    point between the stand's posts."""
    rs, rb = _rot(stand.quat_xyzw), _rot(bag.quat_xyzw)
    t = float(np.dot(bag.pos - stand.pos, rs[:, 1]))
    if abs(t) > POST_REACH + EPS:
        return False
    axis = stand.pos + rs @ np.array([0.0, 0.0, BAR_ZC]) + t * rs[:, 1]
    local = rb.T @ (axis - bag.pos)
    top = local[2] + BAR_R
    return bool(abs(local[0]) <= OPENING + EPS and BAR_UNDER - ON_BAR - EPS <= top <= BAR_UNDER + EPS)


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


def _bag_boxes(bag):
    return [(c, r, s) for _, c, r, s in BAG.world_boxes({"base": (np.asarray(bag.pos), _rot(bag.quat_xyzw))})]


def _shape_boxes(stand, shapes):
    r = _rot(stand.quat_xyzw)
    return [(stand.pos + r @ np.asarray(s.at), r, 2 * s.half) for s in shapes]


def _touches(bag, stand, shapes, margin) -> bool:
    """Any of the bag's own shapes (not its bounding box) within `margin` of any of `shapes` (cylinders as boxes)."""
    return any(_overlap(a, b, margin) for a in _bag_boxes(bag) for b in _shape_boxes(stand, shapes))


def _stand_disturbed(pipe) -> bool:
    start = np.asarray(lots.current().pos("sprinkler_pipe"), float)
    return bool(np.linalg.norm(np.asarray(pipe.pos) - start) > STAND_MOVE + EPS or _tilt(pipe) > STAND_TILT + EPS)


# Whole-trial record: did the bag ever hang on, rest its hook on, or touch the sprinkler pipe while off the table; did
# the bag or a gripper come within 2 mm of the sprinkler head; did the pipe stand move over 10 mm or tilt over 5 degrees?
# Cleared only by a scene reset: the bag back within 0.5 mm of its start and at rest, after a jump of over 20 mm in
# one control step (a robot cannot move it back that way).
_record = {"fault": False, "prev": None}
# Released: in the simulator an on_step callback measures, every step, how close both arms' fingers and grippers are
# to the bag. Offline (near_misses.py) there is no robot, so "clear" stays True.
_grip = {"clear": True, "near_head": False}


def _update(objects) -> None:
    bag = objects["tool_bag"]
    pos = np.asarray(bag.pos, float)
    prev = _record["prev"]
    start = np.asarray(lots.current().pos("tool_bag"), float)
    if prev is not None and np.linalg.norm(pos - start) < 0.0005 and bag.resting():
        if np.linalg.norm(pos - prev) > 0.020:
            _record["fault"] = False
    _record["prev"] = pos
    pipe = objects["sprinkler_pipe"]
    off_table = bag.low[2] > TOP + OFF_TABLE + EPS
    if ((off_table and (_hooked_on(bag, pipe) or _touches(bag, pipe, (PIPE_BAR, TEE), 0.0)))
            or _touches(bag, pipe, HEAD_SHAPES, HEAD_TOUCH) or _stand_disturbed(pipe) or _grip["near_head"]):
        _record["fault"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    bag = st["tool_bag"]
    boxes = BAG.world_boxes({"base": (np.asarray(bag.pos), _rot(bag.quat_xyzw))})
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
    head = _shape_boxes(st["sprinkler_pipe"], HEAD_SHAPES)
    _grip["near_head"] = any(analysis.hulls_to_box(hulls, c, r, s)[0] < HEAD_TOUCH for c, r, s in head)


def _hung(objects) -> bool:
    bag = objects["tool_bag"]
    return bool(
        _hooked_on(bag, objects["hanging_rail"])
        and bag.low[2] > TOP + HANG_CLEAR + EPS
        and _tilt(bag) <= MAX_TILT + EPS
        and bag.resting()
        and _grip["clear"]
    )


def success(objects):
    """The bag hangs by its hook from the grey rail, let go and still; it never hung on the pipe or touched the
    sprinkler head."""
    _update(objects)
    return bool(not _record["fault"] and _hung(objects))


def distance(objects):
    return float(np.linalg.norm(objects["tool_bag"].pos[:2] - objects["hanging_rail"].pos[:2]))


def score(objects):
    """Pass/fail: 1.0 on success, otherwise 0."""
    _update(objects)
    return 1.0 if not _record["fault"] and _hung(objects) else 0.0


# --- demo: left arm only ----------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
GRIP_DZ = BAR_UNDER + 0.002 + TIP  # TCP above the bag's underside: tips 2 mm above the crossbar's underside
CARRY_ORIGIN = 0.052  # bag underside while carried: its body 12 mm under the bar, the lip 12 mm over it
HANG_ORIGIN = BAR_TOP - BAR_UNDER + 0.003  # commanded 3 mm above contact; the loaded arm sags about 4 mm
WIDE = -0.020  # pads 40 mm apart around the 12 mm crossbar
TRAVEL = 0.15


def _w(x, y, z):
    return (x, y, TOP + z)


def demo():
    """Left arm: grip the hook's crossbar, lift, carry the bag in front of the grey rail, slide it on so the rail
    passes under the lip into the hook, lower it onto the rail and let go. The rail's place is the one the cameras
    show for the trial's lot."""
    bx, by = BAG_XY
    rx, ry, _ = lots.current().pos("hanging_rail")
    carry, hang = CARRY_ORIGIN + GRIP_DZ, HANG_ORIGIN + GRIP_DZ
    front = rx - 0.075  # the bag's far end 35 mm short of the rail
    return {"left": [
        Waypoint(1.5, _w(bx, by, TRAVEL), WIDE, "over the hook"),
        Waypoint(1.5, _w(bx, by, GRIP_DZ), WIDE, "down to the crossbar"),
        Waypoint(0.8, _w(bx, by, GRIP_DZ), FINGER_CLOSED, "grip"),
        Waypoint(1.5, _w(bx, by, carry), FINGER_CLOSED, "lift"),
        Waypoint(2.5, _w(front, ry, carry), FINGER_CLOSED, "carry in front of the rail"),
        Waypoint(2.0, _w(rx, ry, carry), FINGER_CLOSED, "slide on: the rail passes under the lip"),
        Waypoint(1.5, _w(rx, ry, hang), FINGER_CLOSED, "lower onto the rail"),
        Waypoint(0.6, _w(rx, ry, hang), WIDE, "release"),
        Waypoint(1.2, _w(rx, ry, TRAVEL), WIDE, "up"),
        Waypoint(1.0, _w(rx - 0.08, ry, TRAVEL), WIDE, "back toward the robot"),
        Waypoint(4.0, _w(rx - 0.08, ry, TRAVEL), WIDE, "wait while the bag settles"),
    ]}
