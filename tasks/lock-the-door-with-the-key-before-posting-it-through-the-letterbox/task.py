"""On the station table, in front of the left arm: a door section standing across the table, facing the robot. Its
front is a cream panel 20 mm thick with a 200 mm wide, 60 mm tall letter slot whose floor is 140 mm above the table;
a brass flap hangs over the slot on the outside, hinged along its top edge, and swings inward when something pushes
it. Behind the panel is a closed hallway box. Below the slot, to the robot's right, is the door's lock: a dark
cover plate with a horizontal keyway, behind which a brass bolt slides sideways toward a brass keeper on the door
frame. The lock has no knob; only a key in the keyway can move the bolt. A key lies on a grey block in front of the
door, its steel blade pointing at the door and a red fob at the robot's end.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m). Heights below are above the table top.
# The door part's frame: origin on the table at the middle of the slot, on the door's outer face; x into the house.
YC = 0.12  # the slot's centre line
X_OUT = 0.36  # the door's outer face (toward the robot)
DOOR_T = 0.020
X_IN = X_OUT + DOOR_T  # the door's inner face
SLOT_W, SLOT_Z0, SLOT_Z1 = 0.200, 0.140, 0.200  # slot width; floor and ceiling heights
DOOR_W, DOOR_H = 0.360, 0.260
HALL_D = 0.220  # box depth behind the door, inside
HALL_FLOOR = 0.005  # a smooth floor board inside the box
WALL = 0.010

CREAM, BRASS, GREY, DARK = (0.93, 0.89, 0.78), (0.78, 0.62, 0.25), (0.55, 0.56, 0.58), (0.18, 0.18, 0.20)
RED, STEEL, FRAME = (0.80, 0.12, 0.10), (0.72, 0.74, 0.77), (0.40, 0.30, 0.22)

FLAP_T, FLAP_W, FLAP_H = 0.003, 0.210, 0.066
PIVOT_Z = SLOT_Z1 + 0.004
FLAP_SHUT_GAP = 0.005  # the flap's lower edge at most 5 mm inside the door face counts as shut
FLAP_SHUT = math.asin(FLAP_SHUT_GAP / FLAP_H)  # rad, about 0.076

# Lock (door frame coordinates). The cover plate stands 6 mm proud of the door face; its keyway is 14 mm tall,
# runs from y -158 to -82 mm and continues as a pocket through the panel into a lock case behind it. At the back of the
# pocket, 34 to 40 mm behind the cover's face (deeper than a fingertip reaches), the bolt carries two dogs 20 mm apart; a key blade
# between them moves the bolt. Thrown, the bolt runs 40 mm toward the robot's right into the keeper on the door frame.
LOCK_Z = 0.080
COVER_T = 0.006
KEYWAY_Y0, KEYWAY_Y1, KEYWAY_H = -0.158, -0.082, 0.014
COVER_Y0, COVER_Y1, COVER_Z0, COVER_Z1 = -0.175, -0.065, 0.055, 0.105
DOG_Y = -0.100  # middle of the gap between the dogs, unlocked
DOG_GAP, DOG_T = 0.020, 0.004
DOG_X = 0.031  # dogs 28 to 34 mm behind the door face (34 to 40 mm behind the cover's face), in the lock case
CASE_X1 = 0.040  # the lock case behind the panel reaches 20 mm into the hallway
BOLT_TRAVEL, LOCKED = 0.040, 0.035
TONGUE_Z = 0.098  # the tongue runs above the keyway (92 to 104 mm up), hidden by the cover plate until thrown


def _b(size, centre, color):
    return parts.box(size, at=centre, color=color)


_side_w = (DOOR_W - SLOT_W) / 2
_kz0, _kz1 = LOCK_Z - KEYWAY_H / 2, LOCK_Z + KEYWAY_H / 2
_door = [
    # below the slot, with the lock's pocket through it
    _b((DOOR_T, DOOR_W, _kz0), (DOOR_T / 2, 0.0, _kz0 / 2), CREAM),
    _b((DOOR_T, DOOR_W, SLOT_Z0 - _kz1), (DOOR_T / 2, 0.0, (_kz1 + SLOT_Z0) / 2), CREAM),
    _b((DOOR_T, DOOR_W / 2 - KEYWAY_Y1, KEYWAY_H), (DOOR_T / 2, (KEYWAY_Y1 + DOOR_W / 2) / 2, LOCK_Z), CREAM),
    _b((DOOR_T, KEYWAY_Y0 + DOOR_W / 2, KEYWAY_H), (DOOR_T / 2, (KEYWAY_Y0 - DOOR_W / 2) / 2, LOCK_Z), CREAM),
    _b((DOOR_T, DOOR_W, DOOR_H - SLOT_Z1), (DOOR_T / 2, 0.0, (SLOT_Z1 + DOOR_H) / 2), CREAM),  # above it
    *(_b((DOOR_T, _side_w, SLOT_Z1 - SLOT_Z0), (DOOR_T / 2, s * (SLOT_W + _side_w) / 2, (SLOT_Z0 + SLOT_Z1) / 2), CREAM)
      for s in (1, -1)),  # beside it
    # door frame post on the robot's right, and the keeper on it
    _b((DOOR_T, 0.040, DOOR_H), (DOOR_T / 2, -DOOR_W / 2 - 0.020, DOOR_H / 2), FRAME),
    # brass keeper: two jaws 15 mm wide and 12 mm proud with a 14 mm gap between them, at the frame's far edge; when
    # locked the bolt's red tongue shows for 25 mm between the cover plate's end and the keeper, and in its jaws
    _b((0.012, 0.015, 0.009), (-0.006, -0.2075, TONGUE_Z + 0.0115), BRASS),
    _b((0.012, 0.015, 0.009), (-0.006, -0.2075, TONGUE_Z - 0.0115), BRASS),
    # cover plate around the keyway
    _b((COVER_T, COVER_Y1 - COVER_Y0, COVER_Z1 - _kz1), (-COVER_T / 2, (COVER_Y0 + COVER_Y1) / 2, (_kz1 + COVER_Z1) / 2),
       DARK),
    _b((COVER_T, COVER_Y1 - COVER_Y0, _kz0 - COVER_Z0), (-COVER_T / 2, (COVER_Y0 + COVER_Y1) / 2, (COVER_Z0 + _kz0) / 2),
       DARK),
    _b((COVER_T, KEYWAY_Y0 - COVER_Y0, KEYWAY_H), (-COVER_T / 2, (COVER_Y0 + KEYWAY_Y0) / 2, LOCK_Z), DARK),
    _b((COVER_T, COVER_Y1 - KEYWAY_Y1, KEYWAY_H), (-COVER_T / 2, (KEYWAY_Y1 + COVER_Y1) / 2, LOCK_Z), DARK),
    # lock case on the panel's inner face, around the pocket's continuation
    _b((CASE_X1 - DOOR_T, COVER_Y1 - COVER_Y0, COVER_Z1 - _kz1),
       ((DOOR_T + CASE_X1) / 2, (COVER_Y0 + COVER_Y1) / 2, (_kz1 + COVER_Z1) / 2), DARK),
    _b((CASE_X1 - DOOR_T, COVER_Y1 - COVER_Y0, _kz0 - COVER_Z0),
       ((DOOR_T + CASE_X1) / 2, (COVER_Y0 + COVER_Y1) / 2, (COVER_Z0 + _kz0) / 2), DARK),
    _b((CASE_X1 - DOOR_T, KEYWAY_Y0 - COVER_Y0, KEYWAY_H), ((DOOR_T + CASE_X1) / 2, (COVER_Y0 + KEYWAY_Y0) / 2, LOCK_Z), DARK),
    _b((CASE_X1 - DOOR_T, COVER_Y1 - KEYWAY_Y1, KEYWAY_H), ((DOOR_T + CASE_X1) / 2, (KEYWAY_Y1 + COVER_Y1) / 2, LOCK_Z), DARK),
    _b((0.004, KEYWAY_Y1 - KEYWAY_Y0, KEYWAY_H), (CASE_X1 - 0.002, (KEYWAY_Y0 + KEYWAY_Y1) / 2, LOCK_Z), DARK),
]
_hall = [
    *(_b((HALL_D, WALL, DOOR_H), (DOOR_T + HALL_D / 2, s * (DOOR_W - WALL) / 2, DOOR_H / 2), GREY) for s in (1, -1)),
    _b((WALL, DOOR_W, DOOR_H), (DOOR_T + HALL_D + WALL / 2, 0.0, DOOR_H / 2), GREY),  # back
    _b((HALL_D, DOOR_W - 2 * WALL, HALL_FLOOR), (DOOR_T + HALL_D / 2, 0.0, HALL_FLOOR / 2), GREY),  # floor
    _b((HALL_D + WALL, DOOR_W, WALL), (DOOR_T + (HALL_D + WALL) / 2, 0.0, DOOR_H + WALL / 2), GREY),  # roof
]
FLAP = parts.Hinge(
    "flap",
    shapes=[_b((FLAP_T, FLAP_W, FLAP_H), (-FLAP_T / 2, 0.0, PIVOT_Z - FLAP_H / 2), BRASS)],
    pivot=(-FLAP_T / 2, 0.0, PIVOT_Z),
    axis=(0.0, -1.0, 0.0),  # positive swings the lower edge inward (+x)
    lower=0.0,
    upper=1.45,
    mass=0.05,
    spring=0.005,
    damping=0.005,
)
BOLT = parts.Slider(
    "bolt",
    shapes=[
        _b((0.012, 0.100, 0.014), (0.010, -0.120, 0.058), BRASS),  # the bolt, inside the door below the pocket
        _b((0.006, DOG_T, 0.032), (DOG_X, DOG_Y - (DOG_GAP + DOG_T) / 2, LOCK_Z - 0.008), BRASS),  # dogs
        _b((0.006, DOG_T, 0.032), (DOG_X, DOG_Y + (DOG_GAP + DOG_T) / 2, LOCK_Z - 0.008), BRASS),
    ],
    axis=(0.0, -1.0, 0.0),  # thrown toward the robot's right
    travel=BOLT_TRAVEL,
    mass=0.05,
    damping=1.0,
)
# The bolt's visible end: a red tongue 40 mm long and 12 mm tall, hidden under the cover plate while unlocked; thrown, it crosses the
# gap past the cover's end into the keeper's jaws. In the simulator it is its own slide, held on the bolt's position
# by a stiff spring every control step (kit links do not chain); scoring reads the bolt itself.
TONGUE = parts.Slider(
    "tongue",
    shapes=[_b((0.004, 0.040, 0.012), (-0.003, -0.155, TONGUE_Z), RED)],
    axis=(0.0, -1.0, 0.0),
    travel=BOLT_TRAVEL,
    mass=0.01,
    spring=2000.0,
    damping=10.0,
)
DOOR = parts.Part("door", pos=(X_OUT, YC, TOP), shapes=[*_door, *_hall], joints=[FLAP, BOLT, TONGUE],
                  friction=(0.3, 0.25))

# The key: a red fob 100 x 25 x 12 mm and a steel blade 50 mm long, 8 x 8 mm, on one centre line (150 mm overall).
# Part frame: origin at the middle of the fob's rear end, x along the key toward the blade.
FOB_L, KEY_W, KEY_H, BLADE_L, BLADE_W = 0.100, 0.025, 0.012, 0.050, 0.008
KEY_L = FOB_L + BLADE_L
RISER = Box("key_block", pos=(0.24, 0.20, TOP + 0.015), size=(0.08, 0.04, 0.03), color=GREY)
KEY_START = (0.17, 0.20, TOP + 0.030 + KEY_H / 2 + 0.0005)
KEY = parts.Part(
    "key",
    pos=KEY_START,
    mass=0.04,
    shapes=[
        parts.box((FOB_L, KEY_W, KEY_H), at=(FOB_L / 2, 0.0, 0.0), color=RED),
        parts.box((BLADE_L, BLADE_W, BLADE_W), at=(FOB_L + BLADE_L / 2, 0.0, 0.0), color=STEEL),
    ],
    friction=(0.5, 0.4),
)

OBJECTS = [DOOR, KEY, RISER]

EPS = 1e-6
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the key or the bolt
STILL = 0.0005  # the bolt moving under 0.5 mm per control step
ON = 0.002
POST_TOL = 0.002  # posted: the key's box at most 2 mm in front of the inner face (a key leaning on it)


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# Filled every control step in the simulator by the hook below. Offline there is no robot: clear and still.
_sim = {"clear": True, "still": True, "prev": None}


@parts.on_reset
def _reset(sim) -> None:
    _sim.update(clear=True, still=True, prev=None)


@parts.on_step
def _measure(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    door, key = st["door"], st["key"]
    b = door.joints["bolt"]
    sim.set_target("door", "tongue", b)
    prev = _sim["prev"]
    _sim["still"] = bool(prev is None or abs(b - prev) < STILL)
    _sim["prev"] = b
    boxes = [(c, r, s) for _, c, r, s in KEY.world_boxes({"base": (np.asarray(key.pos), _rot(key.quat_xyzw))})]
    boxes += [(c, r, s) for _, c, r, s in DOOR.world_boxes({j: (np.asarray(door.links[j][0]), _rot(door.links[j][1]))
                                                           for j in ("bolt", "tongue")})]
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
    _sim["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in boxes)


def _posted(k) -> bool:
    """The whole key inside the hallway box: behind the door's inner face (within 2 mm), in front of the back wall, between the
    side walls and under the roof."""
    half = DOOR_W / 2 - WALL
    return bool(
        k.low[0] >= X_IN - POST_TOL - EPS and k.high[0] <= X_IN + HALL_D + EPS
        and k.low[1] >= YC - half - EPS and k.high[1] <= YC + half + EPS
        and k.high[2] <= TOP + DOOR_H + EPS
    )


def _in_lock(k) -> bool:
    """Any part of the key's box within the cover plate's outline, closer than 2 mm in front of the plate."""
    return bool(
        k.high[0] >= X_OUT - COVER_T - 0.002 - EPS and k.low[0] <= X_OUT + DOOR_T
        and k.high[1] >= YC + COVER_Y0 - EPS and k.low[1] <= YC + COVER_Y1 + EPS
        and k.high[2] >= TOP + COVER_Z0 - EPS and k.low[2] <= TOP + COVER_Z1 + EPS
    )


def _on_floor(k) -> bool:
    return bool(k.low[2] < TOP - 0.010)


def _grade(objects) -> float:
    door, k = objects["door"], objects["key"]
    if not (_sim["clear"] and _sim["still"] and k.resting()) or _on_floor(k):
        return 0.0
    locked = door.joints["bolt"] >= LOCKED - EPS
    flap_shut = door.joints["flap"] <= FLAP_SHUT + EPS
    posted = _posted(k)
    if locked and posted and flap_shut:
        return 1.0
    if locked and not posted and not _in_lock(k):
        return 0.5
    return 0.0


def success(objects):
    """The door locked and the key posted into the hallway box with the flap shut, let go, all still."""
    return _grade(objects) == 1.0


def score(objects):
    """1.0 as success; 0.5 the door locked with the key out of the lock and not posted; else 0."""
    return _grade(objects)


# --- demo: left arm --------------------------------------------------------------------------------------------
TIP = 0.013  # pad tips below the TCP
PAD_BACK = 0.013  # the pads reach 13 mm behind the TCP: gripping flush with the fob's rear end
HAND_FRONT = 0.034  # the hand's widest part, above the slot, reaches 34 mm ahead of the TCP
SAG = 0.004  # the loaded arm settles about 4 mm low
HOLD = -0.009  # pads 18 mm apart: 3.5 mm past the fob's 25 mm faces
TRAVEL_Z = 0.19


def _w(x, y, z):
    return (float(x), float(y), float(TOP + z))


def demo():
    """Grip the fob's rear end, put the blade into the keyway between the dogs, slide it 42 mm to throw the bolt, draw
    it out, carry it to the letter slot and push it through the flap; let go, then push its rear end on with the closed
    fingertips until it drops into the hallway box."""
    kx, ky, kz = KEY_START
    gx = kx + PAD_BACK
    grip_z = 0.030 + 0.003 + TIP  # tips 3 mm above the block
    ahead = KEY_L - PAD_BACK  # blade tip ahead of the TCP
    lock_y = YC + DOG_Y
    lock_z = LOCK_Z - KEY_H / 2 + 0.003 + TIP  # measured: the key rides level here, no sag allowance
    line_up = X_OUT - COVER_T - 0.015 - ahead
    insert = X_OUT + 0.034 - ahead  # blade tip 34 mm behind the door face, between the dogs
    slot_z = (SLOT_Z0 + SLOT_Z1 - KEY_H) / 2 + 0.003 + TIP + SAG
    post_line = X_OUT - FLAP_T - 0.010 - ahead
    home = X_OUT - HAND_FRONT - 0.002
    push_z = SLOT_Z0 + 0.006 + TIP  # closed tips 6 mm above the slot floor
    push_home = home  # the hand's widest part 2 mm short of the door above the slot
    return {"left": [
        Waypoint(1.5, _w(gx, ky, TRAVEL_Z), FINGER_OPEN, "over the fob"),
        Waypoint(1.5, _w(gx, ky, grip_z), -0.020, "down"),
        Waypoint(0.8, _w(gx, ky, grip_z), HOLD, "grip"),
        Waypoint(1.2, _w(gx, ky, lock_z + 0.03), HOLD, "lift"),
        Waypoint(2.0, _w(line_up, lock_y, lock_z), HOLD, "line up with the keyway"),
        Waypoint(1.5, _w(insert, lock_y, lock_z), HOLD, "blade into the lock"),
        Waypoint(2.5, _w(insert, lock_y - 0.050, lock_z), HOLD, "throw the bolt"),
        Waypoint(1.5, _w(line_up, lock_y - 0.050, lock_z), HOLD, "draw the key out"),
        Waypoint(2.0, _w(post_line, YC, slot_z), HOLD, "up to the letter slot"),
        Waypoint(3.0, _w(home, YC, slot_z), HOLD, "push it through the flap"),
        Waypoint(0.6, _w(home, YC, slot_z), -0.020, "let go"),
        Waypoint(1.0, _w(home - 0.05, YC, slot_z), 0.0, "back off, close the fingers"),
        Waypoint(1.0, _w(home - 0.05, YC, push_z), 0.0, "down to the slot floor"),
        Waypoint(2.0, _w(push_home, YC, push_z), 0.0, "push the key's end on"),
        Waypoint(1.0, _w(push_home - 0.06, YC, push_z), 0.0, "back off"),
        Waypoint(1.0, _w(push_home - 0.06, YC, TRAVEL_Z), -0.020, "up"),
        Waypoint(3.0, _w(0.20, 0.25, TRAVEL_Z), -0.020, "park and wait"),
    ]}
