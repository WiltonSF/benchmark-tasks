"""On the station table, in front of the right arm: a small red fire extinguisher standing in a fixed stand. On its
steel valve head, a black carry handle and above it a black squeeze lever, both pointing toward the robot; a steel
safety pin with a ring on its end passes across the head through two lugs, under the lever. A short black hose
leaves the extinguisher's far side and ends in a black horn on a swivel, pointing straight ahead. Further out stand
two bins: a red waste bin with orange and yellow flames rising out of it, and a blue recycling bin with a sheet of
paper on top, one ahead to the left and one ahead to the right of the horn.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import lots  # noqa: E402
import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

RED, STEEL, BLACK, GREY = (0.80, 0.10, 0.10), (0.62, 0.64, 0.66), (0.10, 0.10, 0.11), (0.50, 0.51, 0.53)
BIN_RED, BLUE, WHITE = (0.65, 0.12, 0.10), (0.15, 0.32, 0.80), (0.93, 0.93, 0.92)
ORANGE, YELLOW = (0.98, 0.45, 0.05), (1.00, 0.80, 0.10)

# --- the extinguisher: fixed; frame on the table at the body's axis -----------------------------------------------
EX, EY = 0.23, -0.06
BODY_R, BODY_H = 0.040, 0.090
PIN_X, PIN_Z = -0.050, 0.117  # pin axis (along y), resting on the bottom of the lug holes
PIN_R = 0.005
HOLE = (0.112, 0.124)  # lug hole z range (12 mm tall)
LUG_Z = (0.100, 0.135)  # lugs stand on the lug bracket
LUG_Y = (0.015, 0.025)  # |y| of each lug's faces
LUG_X = 0.020  # lug size front to back
HOLE_X = 0.0055  # half the hole's width front to back: 11 mm, so the 10 mm pin cannot be pushed out from under the tab
_wall = LUG_X / 2 - HOLE_X


def _lug(sign):
    y0, y1 = sorted((sign * LUG_Y[0], sign * LUG_Y[1]))
    yc, yw = (y0 + y1) / 2, y1 - y0
    return [
        parts.box((LUG_X, yw, HOLE[0] - LUG_Z[0]), at=(PIN_X, yc, (LUG_Z[0] + HOLE[0]) / 2), color=STEEL),
        parts.box((LUG_X, yw, LUG_Z[1] - HOLE[1]), at=(PIN_X, yc, (HOLE[1] + LUG_Z[1]) / 2), color=STEEL),
        parts.box((_wall, yw, HOLE[1] - HOLE[0]), at=(PIN_X - HOLE_X - _wall / 2, yc, sum(HOLE) / 2), color=STEEL),
        parts.box((_wall, yw, HOLE[1] - HOLE[0]), at=(PIN_X + HOLE_X + _wall / 2, yc, sum(HOLE) / 2), color=STEEL),
    ]


BODY = [
    parts.box((0.10, 0.10, 0.008), at=(0.0, 0.0, 0.004), color=GREY),  # floor stand
    parts.cylinder(BODY_R, BODY_H - 0.008, at=(0.0, 0.0, 0.008 + (BODY_H - 0.008) / 2), color=RED),
    parts.box((0.040, 0.040, 0.030), at=(0.0, 0.0, 0.105), color=STEEL),  # valve head, z 0.09-0.12
    parts.box((0.080, 0.020, 0.008), at=(-0.060, 0.0, 0.096), color=BLACK),  # carry handle, z 0.092-0.100
    parts.box((LUG_X, 0.052, 0.008), at=(PIN_X, 0.0, 0.096), color=BLACK),  # lug bracket on the handle
    *_lug(1), *_lug(-1),
]

LEVER_PIVOT = (0.020, 0.0, 0.140)
TAB_LOW = PIN_Z + PIN_R + 0.0015  # 1.5 mm above the pin
TAB_H = 0.135 - TAB_LOW
LEVER_UP = 0.33  # rad: the lever's end meets the carry handle
LEVER = parts.Hinge(
    "lever", pivot=LEVER_PIVOT, axis=(0.0, -1.0, 0.0), lower=0.0, upper=LEVER_UP, mass=0.015, spring=0.3,
    damping=0.005,
    shapes=[
        parts.box((0.125, 0.020, 0.010), at=(-0.0375, 0.0, 0.140), color=BLACK),  # x 0.025 .. -0.100
        parts.box((0.020, 0.012, TAB_H), at=(PIN_X, 0.0, TAB_LOW + TAB_H / 2), color=BLACK),  # tab over the pin
    ],
)
HORN_PIVOT = (BODY_R, 0.0, 0.040)
HORN_MOUTH = 0.180  # local x of the horn's mouth at swivel 0
HORN = parts.Hinge(
    "horn", pivot=HORN_PIVOT, axis=(0.0, 0.0, 1.0), lower=-1.0, upper=1.0, mass=0.05, damping=0.05,
    shapes=[
        parts.cylinder(0.007, 0.090, at=(0.085, 0.0, 0.040), color=BLACK, axis="x"),  # hose
        parts.cylinder(0.013, 0.050, at=(0.155, 0.0, 0.040), color=BLACK, axis="x"),  # horn
    ],
)
EXT = parts.Part("extinguisher", pos=(EX, EY, TOP), shapes=BODY, joints=[LEVER, HORN])

# --- the pin: loose steel rod across the head (along y) with a ring on its -y end --------------------------------
PIN = parts.Part(
    "pin", pos=(EX + PIN_X, EY, TOP + PIN_Z), mass=0.06,
    shapes=[
        parts.cylinder(PIN_R, 0.060, at=(0.0, 0.0, 0.0), color=STEEL, axis="y"),  # y -0.030 .. 0.030
        parts.box((0.005, 0.030, 0.005), at=(0.0, -0.045, 0.022), color=STEEL),  # ring top bar
        parts.box((0.005, 0.030, 0.005), at=(0.0, -0.045, -0.010), color=STEEL),  # ring bottom bar
        parts.box((0.005, 0.005, 0.037), at=(0.0, -0.0325, 0.006), color=STEEL),
        parts.box((0.005, 0.005, 0.037), at=(0.0, -0.0575, 0.006), color=STEEL),
    ],
)

# --- bins: loose, heavy; 25 cm from the horn's swivel at 40 degrees either side of straight ahead ------------------
SWIVEL_W = (EX + HORN_PIVOT[0], EY)
AIM_R, AIM_DEG = 0.25, 40.0
BIN = (0.09, 0.09, 0.10)


def _bin_at(sign):
    a = math.radians(sign * AIM_DEG)
    return (SWIVEL_W[0] + AIM_R * math.cos(a), SWIVEL_W[1] + AIM_R * math.sin(a), TOP)


LEFT, RIGHT = _bin_at(1), _bin_at(-1)
AMBER = (1.00, 0.62, 0.08)
# Flames: five tapered tongues rising above the rim, each three stacked pieces narrowing upward (orange, amber,
# yellow). (x, y, total height) in the bin's frame.
TONGUES = [(0.0, 0.0, 0.085), (-0.022, 0.018, 0.065), (0.020, 0.020, 0.055), (0.018, -0.020, 0.070),
           (-0.020, -0.018, 0.050)]
FLAMES = []
for tx, ty, th in TONGUES:
    z = BIN[2] + 0.010
    for frac, width, col in ((0.45, 0.018, ORANGE), (0.35, 0.012, AMBER), (0.20, 0.006, YELLOW)):
        FLAMES.append(parts.box((width, width, frac * th), at=(tx, ty, z + frac * th / 2), color=col))
        z += frac * th
WASTE = parts.Part(
    "waste_bin", pos=LEFT, mass=1.5,
    shapes=[
        parts.box(BIN, at=(0.0, 0.0, BIN[2] / 2), color=BIN_RED),
        parts.box((0.070, 0.070, 0.010), at=(0.0, 0.0, BIN[2] + 0.005), color=ORANGE),  # burning layer at the rim
        *FLAMES,
    ],
)
RECYCLING = parts.Part(
    "recycling_bin", pos=RIGHT, mass=1.5,
    shapes=[
        parts.box(BIN, at=(0.0, 0.0, BIN[2] / 2), color=BLUE),
        parts.box((0.06, 0.045, 0.004), at=(0.0, 0.0, BIN[2] + 0.002), color=WHITE),
    ],
)
OBJECTS = [EXT, PIN, WASTE, RECYCLING]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", fire_sign=1),
    lots.Lot("B", fire_sign=-1, place={"waste_bin": RIGHT, "recycling_bin": LEFT}),
)

SUCCESS_HOLD_S = 1.5
EPS = 1e-6
PRESSED = 0.28  # rad: lever squeezed (its end within about 6 mm of the handle)
PARTIAL = 0.12  # rad: lever pressed far enough to start a discharge (the pin, left in, lets it reach about 0.09)
UP = 0.05  # rad: lever let go
AIM_TOL = 0.040  # horn's line within 40 mm of the burning bin's centre
DISCHARGE_S = 2.0
PIN_OUT = 0.060  # pin moved 60 mm or more from its place: clear of both lugs
BIN_MOVE = 0.020
CLEAR = 0.005


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def aim_miss(objects) -> float:
    """Distance (m, seen from above) from the burning bin's centre to the horn's line, ahead of its mouth."""
    swivel = float(objects["extinguisher"].joints["horn"])
    d = np.array([math.cos(swivel), math.sin(swivel)])
    mouth = np.array(SWIVEL_W) + (HORN_MOUTH - HORN_PIVOT[0]) * d
    v = np.asarray(objects["waste_bin"].pos[:2], float) - mouth
    along = float(v @ d)
    if along <= 0:
        return float(np.linalg.norm(v))
    return float(abs(v[0] * d[1] - v[1] * d[0]))


def pin_out(objects) -> bool:
    start = np.array(PIN.pos)
    return bool(np.linalg.norm(np.asarray(objects["pin"].pos) - start) >= PIN_OUT - EPS)


def lever(objects) -> float:
    return float(objects["extinguisher"].joints["lever"])


def _bins_moved(objects) -> bool:
    lot = lots.current()
    return any(np.linalg.norm(np.asarray(objects[n].pos[:2]) - np.asarray(lot.pos(n)[:2])) > BIN_MOVE + EPS
               for n in ("waste_bin", "recycling_bin"))


# Whole-trial record, updated once per control step: how long the lever has been squeezed with the pin out and the
# horn on the fire (the discharge), and whether it was ever pressed 0.12 rad or more while the horn pointed anywhere
# else, or a bin was moved. Reset by the simulator's scene reset (on_reset); offline, by a state with every object at its start.
_rec = {"live": False, "t": 0.0, "since": None, "discharged": False, "fault": False}
STEP = 0.05


def _clear_record():
    _rec.update(t=0.0, since=None, discharged=False, fault=False)


def _update(objects, t) -> None:
    _rec["t"] = t
    squeezed = lever(objects) >= PRESSED - EPS
    aimed = aim_miss(objects) <= AIM_TOL + EPS
    if lever(objects) >= PARTIAL - EPS and not aimed:
        _rec["fault"] = True
    if _bins_moved(objects):
        _rec["fault"] = True
    if squeezed and aimed and pin_out(objects):
        if _rec["since"] is None:
            _rec["since"] = t
        if t - _rec["since"] >= DISCHARGE_S - EPS:
            _rec["discharged"] = True
    else:
        _rec["since"] = None


@parts.on_reset
def _reset(sim):
    _rec["live"] = True
    _clear_record()
    _grip["clear"] = True


@parts.on_step
def _step(sim):
    _rec["live"] = True
    _update(sim.states(), float(sim.t))
    _measure_grip(sim)


def _at_start(objects) -> bool:
    lot = lots.current()
    ext = objects["extinguisher"]
    return bool(
        abs(lever(objects)) < 1e-9 and abs(float(ext.joints["horn"])) < 1e-9
        and np.allclose(objects["pin"].pos, PIN.pos, atol=1e-9)
        and all(np.allclose(objects[n].pos, lot.pos(n), atol=1e-9) for n in ("waste_bin", "recycling_bin"))
    )


def _offline(objects, step=True) -> None:
    """Offline (near_misses.py), each score() call is one control step of 50 ms, and a state with every object at
    its start is a new trial."""
    if _rec["live"]:
        return
    if _at_start(objects):
        _clear_record()
    elif step:
        _update(objects, _rec["t"] + STEP)


# Released: every step an on_step callback measures how close both arms' fingers and gripper bodies are to the lever,
# the horn and the pin. Offline there is no robot, so "clear" stays True.
_grip = {"clear": True}


def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    ext, pin = st["extinguisher"], st["pin"]
    links = {k: (np.asarray(p), _rot(q)) for k, (p, q) in ext.links.items()}
    boxes = EXT.world_boxes(links) + PIN.world_boxes({"base": (np.asarray(pin.pos), _rot(pin.quat_xyzw))})
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
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] >= CLEAR - EPS for _, c, r, s in boxes)


def _done(objects) -> bool:
    return bool(
        _rec["discharged"] and not _rec["fault"]
        and lever(objects) <= UP + EPS
        and _grip["clear"]
        and all(objects[n].resting() for n in ("pin", "waste_bin", "recycling_bin", "extinguisher"))
    )


def success(objects):
    """Discharged onto the fire (pin out, lever squeezed, horn on the burning bin, 2 s together), never squeezed
    while pointing elsewhere; then let go, grippers clear and everything still."""
    _offline(objects, step=False)
    return _done(objects)


def distance(objects):
    return aim_miss(objects)


def score(objects):
    """Pass/fail: 1.0 on success, otherwise 0."""
    _offline(objects)
    return 1.0 if _done(objects) else 0.0


# --- scripted demo: right arm only ----------------------------------------------------------------------------------
# Pull the pin out by its ring (toward the robot's right) and drop it on the table; take the horn and swing it on its
# swivel to the burning bin, whichever side it is on in this trial (seen in the layout); let go; press the lever down
# with closed fingers to the handle and hold 3 s; let go and lift away. Fixed waypoints per lot.
TIP = 0.013
OPEN = -0.035
RING_GRIP = -0.001
HORN_GRIP = -0.010  # pads 20 mm apart on the 26 mm horn
RING_X, RING_Y, RING_Z = EX + PIN_X, EY - 0.045, TOP + PIN_Z + 0.022
HORN_R = 0.115  # grip on the horn, from the swivel axis
HORN_Z = TOP + HORN_PIVOT[2] + TIP
LEVER_END = (EX - 0.090, EY)
PRESS_Z = TOP + 0.140 - 0.045 + 0.005 + TIP  # closed tips 45 mm below the lever's top at rest


def demo():
    sign = lots.current().fire_sign
    target = math.radians(sign * AIM_DEG)
    w = [
        Waypoint(1.5, (RING_X, RING_Y, RING_Z + TIP + 0.04), OPEN, "over the pin's ring", math.pi / 2),
        Waypoint(1.2, (RING_X, RING_Y, RING_Z + TIP - 0.002), OPEN, "down to the ring", math.pi / 2),
        Waypoint(0.8, (RING_X, RING_Y, RING_Z + TIP - 0.002), RING_GRIP, "grip the ring", math.pi / 2),
        Waypoint(2.0, (RING_X, RING_Y - 0.090, RING_Z + TIP + 0.005), RING_GRIP, "pull the pin out", math.pi / 2),
        Waypoint(0.8, (RING_X, RING_Y - 0.090, RING_Z + TIP + 0.005), OPEN, "drop it", math.pi / 2),
        Waypoint(1.0, (RING_X, RING_Y - 0.090, RING_Z + 0.08), OPEN, "up", math.pi / 2),
    ]
    hx, hy = SWIVEL_W[0] + HORN_R, SWIVEL_W[1]
    w += [
        Waypoint(1.5, (hx, hy, HORN_Z + 0.05), OPEN, "over the horn"),
        Waypoint(1.2, (hx, hy, HORN_Z), OPEN, "down to the horn"),
        Waypoint(0.8, (hx, hy, HORN_Z), HORN_GRIP, "grip the horn"),
    ]
    n = 8
    for i in range(1, n + 1):
        a = target * i / n
        w.append(Waypoint(0.4, (SWIVEL_W[0] + HORN_R * math.cos(a), SWIVEL_W[1] + HORN_R * math.sin(a), HORN_Z),
                          HORN_GRIP, "swing the horn to the fire", a))
    ax, ay = SWIVEL_W[0] + HORN_R * math.cos(target), SWIVEL_W[1] + HORN_R * math.sin(target)
    w += [
        Waypoint(0.8, (ax, ay, HORN_Z), OPEN, "let go of the horn", target),
        Waypoint(1.0, (ax, ay, HORN_Z + 0.03), OPEN, "up", target),
        Waypoint(1.5, (*LEVER_END, TOP + 0.20), FINGER_CLOSED, "over the lever"),
        Waypoint(1.0, (*LEVER_END, TOP + 0.140 + 0.010 + TIP), FINGER_CLOSED, "onto the lever"),
        Waypoint(1.5, (*LEVER_END, PRESS_Z), FINGER_CLOSED, "squeeze"),
        Waypoint(3.0, (*LEVER_END, PRESS_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.0, (*LEVER_END, TOP + 0.20), FINGER_CLOSED, "let go"),
        Waypoint(1.5, (0.15, -0.25, TOP + 0.15), FINGER_CLOSED, "park"),
    ]
    return {"right": w}
