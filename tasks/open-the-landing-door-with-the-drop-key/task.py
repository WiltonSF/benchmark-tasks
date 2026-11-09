"""On the station table, ahead of the arms: a pale landing wall panel standing across the table, with a narrow
upright key slot near its left end. Behind the wall is a sliding door; its right end sticks out past the wall's right
end and carries a black pull handle. On the table in front of the left arm lies a drop key: a thin steel rod with a
grip block at its near end and a short red upturned hook at its far end. An orange latch bar lies out of sight
behind the wall.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WALL_C, DOOR_C, BLACK, STEEL, LATCH_C = (0.80, 0.78, 0.72), (0.55, 0.60, 0.66), (0.08, 0.08, 0.09), \
    (0.70, 0.72, 0.75), (0.85, 0.55, 0.10)
HOOK_RED = (0.80, 0.08, 0.06)


def _box(origin, lo, hi, color):
    """A kit box from world-frame corners (x, y, z above the table) for a part whose origin is `origin`."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    at = (lo + hi) / 2 - np.asarray(origin, float)
    return parts.box(tuple(float(v) for v in hi - lo), at=tuple(float(v) for v in at), color=color)


# Landing wall (fixed): 10 mm thick at x 0.36-0.37, y -0.12 to 0.18, 150 mm tall, with a key slot 20 mm wide
# (y 0.11-0.13) from 45 to 115 mm up.
WALL_X0, WALL_X1, WALL_Y0, WALL_Y1, WALL_H = 0.36, 0.37, -0.12, 0.18, 0.15
SLOT_Y0, SLOT_Y1, SLOT_Z0, SLOT_Z1 = 0.11, 0.13, 0.045, 0.115
O_DOOR = (0.37, 0.0, 0.0)
WALL_SHAPES = [
    _box(O_DOOR, (WALL_X0, WALL_Y0, 0.0), (WALL_X1, SLOT_Y0, WALL_H), WALL_C),
    _box(O_DOOR, (WALL_X0, SLOT_Y1, 0.0), (WALL_X1, WALL_Y1, WALL_H), WALL_C),
    _box(O_DOOR, (WALL_X0, SLOT_Y0, 0.0), (WALL_X1, SLOT_Y1, SLOT_Z0), WALL_C),
    _box(O_DOOR, (WALL_X0, SLOT_Y0, SLOT_Z1), (WALL_X1, SLOT_Y1, WALL_H), WALL_C),
]
# The door (a slider toward the robot's right, -y, up to 100 mm): a panel behind the wall, a lug on its back near its
# left end, and a pull handle on its front beyond the wall's right end.
DOOR_Y0, DOOR_Y1 = -0.21, 0.09
LUG = ((0.385, 0.062, 0.060), (0.400, 0.080, 0.075))
HANDLE = ((0.350, -0.200, 0.030), (0.375, -0.180, 0.120))  # far enough out that the gripper stays clear of the wall
DOOR_SHAPES = [
    _box(O_DOOR, (0.375, DOOR_Y0, 0.005), (0.385, DOOR_Y1, 0.130), DOOR_C),
    _box(O_DOOR, *LUG, DOOR_C),
    _box(O_DOOR, *HANDLE, BLACK),
]
DOOR = parts.Part(
    "landing_door", pos=(O_DOOR[0], O_DOOR[1], TOP), shapes=WALL_SHAPES,
    joints=[parts.Slider("door", shapes=DOOR_SHAPES, axis=(0.0, -1.0, 0.0), travel=0.10, mass=0.4, damping=2.0)],
)

# Gravity latch (a separate fixed part, so its bar collides with the door's lug): a short post behind the wall and an
# orange bar hinged on it. The bar lies along the wall at 80-90 mm, with a catch hanging from its free end in front of
# the lug (on the lug's right). A knee at the bar's post end drops to the pivot at 55 mm, below the catch's contact
# with the lug (60-75 mm), so pulling the door turns the bar down onto its stop instead of camming it up. Raising
# the bar's free end 15 degrees or more lets the lug pass under the catch.
O_LATCH = (0.394, 0.16, 0.055)
PIVOT_Y = 0.16
POST = _box(O_LATCH, (0.388, 0.165, 0.0), (0.400, 0.180, 0.060), DOOR_C)
BAR = _box(O_LATCH, (0.388, 0.050, 0.080), (0.400, 0.165, 0.090), LATCH_C)
KNEE = _box(O_LATCH, (0.388, 0.155, 0.050), (0.400, 0.165, 0.080), LATCH_C)
CATCH = _box(O_LATCH, (0.388, 0.050, 0.060), (0.400, 0.060, 0.080), LATCH_C)
LATCH = parts.Part(
    "latch", pos=(O_LATCH[0], O_LATCH[1], TOP + O_LATCH[2]), shapes=[POST],
    joints=[parts.Hinge("bar", shapes=[BAR, KNEE, CATCH], pivot=(0.0, 0.0, 0.0), axis=(-1.0, 0.0, 0.0),
                        lower=0.0, upper=0.9, mass=0.05, damping=0.005)],
)

# Drop key (loose, 60 g). Part frame: origin under the grip block's centre; the rod runs along +x.
KEY_XY = (0.12, 0.22)
BLOCK = parts.box((0.020, 0.030, 0.030), at=(0.0, 0.0, 0.015), color=BLACK)
ROD = parts.box((0.200, 0.010, 0.010), at=(0.110, 0.0, 0.005), color=STEEL)
HOOK = parts.box((0.010, 0.010, 0.020), at=(0.205, 0.0, 0.010), color=HOOK_RED)
KEY = parts.Part("drop_key", pos=(*KEY_XY, TOP), mass=0.06, shapes=[BLOCK, ROD, HOOK])

OBJECTS = [DOOR, LATCH, KEY]

EPS = 1e-6
OPEN = 0.060  # door open: slid at least 60 mm
AJAR = 0.010  # partly open: slid at least 10 mm (a pull without the key moves it about 2 mm)
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the door
STILL = 0.0005  # door joint moved under 0.5 mm in a control step
BEHIND = ((WALL_X1, WALL_Y0, 0.0), (0.50, WALL_Y1, 0.30))  # the space behind the wall, x/y/z above the table


# Whole-trial record: whether a gripper ever entered the space behind the wall.
# Cleared by the simulator's reset hook, or (offline) by a scene reset: the key back within 0.5 mm of its start and
# at rest, door closed and latch down, after the key jumped over 20 mm in one control step.
_record = {"fault": False, "prev_key": None, "prev_door": None, "door_moving": False}
# In the simulator an on_step callback measures, every step, where both arms' fingers and grippers are and whether
# the door moved over 0.5 mm since the last step. Offline (near_misses.py) there is no robot and no step history, so
# "clear" stays True, "behind" False and the door counts as still.
_grip = {"clear": True, "behind": False}


@parts.on_reset
def _new_trial(sim) -> None:
    _record.update(fault=False, prev_key=None, prev_door=None, door_moving=False)


def _update(objects) -> None:
    key, door, latch = objects["drop_key"], objects["landing_door"], objects["latch"]
    kp = np.asarray(key.pos, float)
    q_door, q_bar = float(door.joints["door"]), float(latch.joints["bar"])
    prev = _record["prev_key"]
    home = (np.linalg.norm(kp - np.asarray(KEY.pos)) < 0.0005 and key.resting() and q_door < 0.0005
            and q_bar < math.radians(1.0))
    if prev is not None and home and np.linalg.norm(kp - prev) > 0.020:
        _record.update(fault=False)
    _record["prev_key"] = kp
    if _grip["behind"]:
        _record["fault"] = True


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    door = sim.states()["landing_door"]
    q, pd = float(door.joints["door"]), _record["prev_door"]
    _record["door_moving"] = pd is not None and abs(q - pd) > STILL
    _record["prev_door"] = q
    p, r = door.links["door"]
    boxes = [(c, rr, s) for _, c, rr, s in DOOR.world_boxes({"door": (np.asarray(p), _rot(r))})]
    lo, hi = np.asarray(BEHIND[0]), np.asarray(BEHIND[1])
    lo, hi = lo + [0, 0, TOP], hi + [0, 0, TOP]
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
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, rr, s)[0] > CLEAR for c, rr, s in boxes)
    zone = ((lo + hi) / 2, np.eye(3), hi - lo)
    _grip["behind"] = analysis.hulls_to_box(hulls, *zone)[0] < 0.0


def _slid(objects, at_least) -> bool:
    """The door slid at least `at_least`, let go and still."""
    return bool(objects["landing_door"].joints["door"] >= at_least - EPS and not _record["door_moving"]
                and _grip["clear"])


def key_out(objects) -> bool:
    """The key wholly in front of the wall's front face."""
    return bool(objects["drop_key"].high[0] <= WALL_X0 + EPS)


def _open(objects) -> bool:
    return _slid(objects, OPEN) and key_out(objects)


def success(objects):
    """The door stands open by 60 mm or more, let go and still, the key back out in front of the wall; no gripper
    ever reached behind the wall."""
    _update(objects)
    return bool(not _record["fault"] and _open(objects))


def distance(objects):
    return float(max(0.0, OPEN - objects["landing_door"].joints["door"]))


def score(objects):
    """1.0 on success; 0.5 for a door left slid at least 10 mm, let go and still; otherwise 0."""
    _update(objects)
    if _record["fault"]:
        return 0.0
    if _open(objects):
        return 1.0
    return 0.5 if _slid(objects, AJAR) else 0.0


# --- demo ---------------------------------------------------------------------------------------------------------
TIP = 0.013
KEY_GRIP = 0.008 + TIP  # TCP above the key's underside: tips 8 mm up the grip block
KEY_PINCH = -0.007  # pads about 14 mm apart on the 20 mm block (fingers close along x)
ROD_IN = 0.055  # rod underside while inserted: 10 mm above the slot's bottom
RAISE = 0.028  # the loaded arm sags: 20 mm commanded lifted the key 14 mm on the GPU
HANDLE_PINCH = -0.007  # 20 mm handle
OPEN_F = -0.030
TRAVEL = 0.15
YAW_KEY = -math.pi / 2  # +90 degrees is out of reach for the left arm here


def _w(x, y, z):
    return (float(x), float(y), float(TOP + z))


def demo():
    """Left arm: pick up the key by its block, carry it level to the slot, push the hook through under the latch
    bar, raise it and hold. Right arm: then pull the door open by its handle and let go. Left arm: lower the key, draw
    it out and put it down."""
    kx, ky = KEY_XY
    sy = (SLOT_Y0 + SLOT_Y1) / 2
    hook_x = 0.394  # the hook under the bar, behind the door's plane
    in_x = hook_x - 0.205  # key origin when inserted
    out_x = WALL_X0 - 0.215 - 0.015  # tip 15 mm short of the wall
    zin, zup = ROD_IN + KEY_GRIP, ROD_IN + RAISE + KEY_GRIP
    k = dict(yaw=YAW_KEY)
    left = [
        Waypoint(1.5, _w(kx, ky, TRAVEL), OPEN_F, "over the key", **k),
        Waypoint(1.5, _w(kx, ky, KEY_GRIP), OPEN_F, "down to the block", **k),
        Waypoint(0.8, _w(kx, ky, KEY_GRIP), KEY_PINCH, "grip", **k),
        Waypoint(1.2, _w(kx, ky, zin), KEY_PINCH, "lift level", **k),
        Waypoint(1.5, _w(out_x, sy, zin), KEY_PINCH, "in line with the slot", **k),
        Waypoint(2.0, _w(in_x, sy, zin), KEY_PINCH, "through the slot, hook under the bar", **k),
        Waypoint(1.2, _w(in_x, sy, zup), KEY_PINCH, "raise: lift the latch", **k),
        Waypoint(5.0, _w(in_x, sy, zup), KEY_PINCH, "hold the latch up", **k),
        Waypoint(1.0, _w(in_x, sy, zin), KEY_PINCH, "lower", **k),
        Waypoint(2.0, _w(out_x, sy, zin), KEY_PINCH, "draw out", **k),
        Waypoint(1.5, _w(kx, ky, KEY_GRIP + 0.007), KEY_PINCH, "back to the table", **k),
        Waypoint(0.6, _w(kx, ky, KEY_GRIP + 0.007), OPEN_F, "release", **k),
        Waypoint(1.2, _w(kx, ky, TRAVEL), OPEN_F, "up", **k),
    ]
    hx, hy = (HANDLE[0][0] + HANDLE[1][0]) / 2, (HANDLE[0][1] + HANDLE[1][1]) / 2
    hz = HANDLE[1][2] - 0.025 + TIP  # tips 25 mm below the handle's top
    pull = 0.095  # the door sprang back about 20 mm when let go after an 80 mm pull
    right = [
        Waypoint(9.0, _w(hx, hy, TRAVEL), OPEN_F, "wait over the handle"),
        Waypoint(1.5, _w(hx, hy, hz), OPEN_F, "down to the handle"),
        Waypoint(0.8, _w(hx, hy, hz), HANDLE_PINCH, "grip"),
        Waypoint(2.5, _w(hx, hy - pull, hz), HANDLE_PINCH, "pull the door open"),
        Waypoint(0.6, _w(hx, hy - pull, hz), OPEN_F, "release"),
        Waypoint(1.2, _w(hx, hy - pull, TRAVEL), OPEN_F, "up"),
        Waypoint(6.0, _w(hx, hy - pull, TRAVEL), OPEN_F, "wait"),
    ]
    return {"left": left, "right": right}
