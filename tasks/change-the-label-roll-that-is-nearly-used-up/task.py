"""On the station table, in a row the same distance in front of the arms: three dark grey label printers, each with
an upright black spindle on its deck and a label roll on the spindle (white labels on a brown cardboard core). The
rolls have different amounts of labels left on them. In front of the printers stand a full spare roll and an open
grey bin. Which printer holds which roll, and where the spare and the bin stand, change between trials.
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
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

CASE, SLOT, SPINDLE = (0.20, 0.21, 0.23), (0.04, 0.04, 0.05), (0.06, 0.06, 0.07)
LABELS, CORE, RED, BIN_GREY = (0.93, 0.93, 0.90), (0.62, 0.44, 0.26), (0.85, 0.08, 0.06), (0.55, 0.57, 0.60)

# Printers: fixed. Part frame: origin at the centre of the body's underside, x away from the robot.
BODY = (0.100, 0.100, 0.040)
DECK = BODY[2]  # the deck's height above the table
SPINDLE_R, SPINDLE_H = 0.015, 0.022  # 30 mm across, 22 mm tall
STUB_R, STUB_H = 0.011, 0.006  # a 22 mm lead-in on top: the spindle's top is 28 mm above the deck
PRINTER_X = 0.34
COLUMNS = {"left": 0.13, "centre": 0.0, "right": -0.13}


def _printer(name, y):
    return parts.Part(
        name,
        pos=(PRINTER_X, y, TOP),
        shapes=[
            parts.box(BODY, at=(0.0, 0.0, DECK / 2), color=CASE),
            parts.box((0.004, 0.064, 0.006), at=(-BODY[0] / 2 - 0.001, 0.0, 0.028), color=SLOT),  # label exit
            parts.cylinder(SPINDLE_R, SPINDLE_H, at=(0.0, 0.0, DECK + SPINDLE_H / 2), color=SPINDLE, axis="z"),
            parts.cylinder(STUB_R, STUB_H, at=(0.0, 0.0, DECK + SPINDLE_H + STUB_H / 2), color=SPINDLE, axis="z"),
        ],
    )


PRINTERS = {col: _printer(f"{col}_printer", y) for col, y in COLUMNS.items()}

# Rolls: loose, 40 mm tall, with a real 36 mm hole through the core. Part frame: origin at the centre of the roll's
# underside, axis up. The kit has only solid cylinders, so each ring (core, labels) is a ring of overlapping upright
# cylinders ("lobes"): its inner surface is 18 to 19.6 mm from the axis, its outer edge scalloped by about 2 mm.
ROLL_H = 0.040
HOLE_R = 0.018
FULL_R, LOW_R = 0.043, 0.025  # 86 mm across full; 50 mm nearly used up
FULL_MASS, LOW_MASS = 0.17, 0.04


def _ring(rm, rc, n, z0, z1, color):
    return [
        parts.cylinder(rc, z1 - z0, at=(rm * math.cos(2 * math.pi * k / n), rm * math.sin(2 * math.pi * k / n),
                                         (z0 + z1) / 2), color=color, axis="z")
        for k in range(n)
    ]


def _full_shapes():
    return [*_ring(0.0205, 0.0025, 28, 0.0, ROLL_H + 0.0005, CORE),  # core, 18 to 23 mm
            *_ring(0.033, 0.010, 16, 0.0, ROLL_H, LABELS)]  # labels, 23 to 43 mm


def _low_shapes():
    # Core 18 to 24 mm; a thin band of labels 20 to 25 mm, its top 12 mm red: the end-of-roll stripe.
    return [*_ring(0.021, 0.003, 24, 0.0, ROLL_H, CORE),
            *_ring(0.0225, 0.0025, 30, 0.0, ROLL_H - 0.012, LABELS),
            *_ring(0.0225, 0.0025, 30, ROLL_H - 0.012, ROLL_H + 0.0005, RED)]


def _roll(name, full, pos):
    return parts.Part(name, pos=pos, mass=FULL_MASS if full else LOW_MASS,
                      shapes=_full_shapes() if full else _low_shapes())


def cup(col):
    """Where a roll seated on that printer's spindle has its origin (the deck, on the spindle's axis)."""
    return (PRINTER_X, COLUMNS[col], TOP + DECK)


FRONT_X = 0.17


def front(col):
    return (FRONT_X, COLUMNS[col], TOP)


LOW = _roll("low_roll", False, cup("left"))
FULL_A = _roll("full_roll_a", True, cup("centre"))
FULL_B = _roll("full_roll_b", True, cup("right"))
SPARE = _roll("spare_roll", True, front("left"))
ROLL_R = {"low_roll": LOW_R, "full_roll_a": FULL_R, "full_roll_b": FULL_R, "spare_roll": FULL_R}

# Bin: loose but heavy (1.5 kg) only so the lots can move it. 100 x 100 mm inside, 4 mm walls 40 mm tall, 4 mm floor.
BIN_IN, BIN_WALL, BIN_FLOOR, BIN_H = 0.100, 0.004, 0.004, 0.040
_wo = BIN_IN / 2 + BIN_WALL / 2
_wh = BIN_H - BIN_FLOOR
BIN = parts.Part(
    "grey_bin",
    pos=front("centre"),
    mass=1.5,
    shapes=[
        parts.box((BIN_IN + 2 * BIN_WALL, BIN_IN + 2 * BIN_WALL, BIN_FLOOR), at=(0.0, 0.0, BIN_FLOOR / 2),
                  color=BIN_GREY),
        parts.box((BIN_WALL, BIN_IN + 2 * BIN_WALL, _wh), at=(_wo, 0.0, BIN_FLOOR + _wh / 2), color=BIN_GREY),
        parts.box((BIN_WALL, BIN_IN + 2 * BIN_WALL, _wh), at=(-_wo, 0.0, BIN_FLOOR + _wh / 2), color=BIN_GREY),
        parts.box((BIN_IN, BIN_WALL, _wh), at=(0.0, _wo, BIN_FLOOR + _wh / 2), color=BIN_GREY),
        parts.box((BIN_IN, BIN_WALL, _wh), at=(0.0, -_wo, BIN_FLOOR + _wh / 2), color=BIN_GREY),
    ],
)

OBJECTS = [*PRINTERS.values(), BIN, LOW, FULL_A, FULL_B, SPARE]

# Lots. Each lot puts the low roll on a different printer, and the spare and the bin in front of different printers,
# so that "a fixed printer", "the printer behind the spare", "the printer behind the bin" and "the printer with
# nothing in front of it" each pick the low roll in exactly one lot of three.
#   A (screenshot): low roll left;   spare in front of left,  bin in front of centre.
#   B:              low roll centre; spare in front of left,  bin in front of right.
#   C:              low roll right;  spare in front of centre, bin in front of right.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", low="left"),
    lots.Lot("B", low="centre", place={"low_roll": cup("centre"), "full_roll_a": cup("left"),
                                        "grey_bin": front("right")}),
    lots.Lot("C", low="right", place={"low_roll": cup("right"), "full_roll_b": cup("left"),
                                       "spare_roll": front("centre"), "grey_bin": front("right")}),
)

EPS = 1e-6
SEAT_Z = 0.001  # seated: the roll's lowest point within 1 mm of the deck
SEAT_XY = 0.004  # and its axis within 4 mm of the spindle's (3 mm of play)
MAX_TILT = math.radians(5.0)
BIN_Z = 0.020  # in the bin: lowest point within 20 mm of the bin floor
TABLE_Z = 0.002  # on the table: lowest point within 2 mm of the table top
PULLED = 0.010  # a full roll in a printer moving more than 10 mm at any point is a fault
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the spare or the old roll
FULLS = ("full_roll_a", "full_roll_b")
NAMES = ("low_roll", *FULLS, "spare_roll", "grey_bin")


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _axis(state) -> np.ndarray:
    return _rot(state.quat_xyzw)[:, 2]


def _tilt(state) -> float:
    return math.acos(float(np.clip(_axis(state)[2], -1.0, 1.0)))


def _lowest(state, radius) -> float:
    """The roll's lowest point, from its true round shape (not its bounding box)."""
    a = _axis(state)
    rim = radius * math.sqrt(max(0.0, 1.0 - a[2] ** 2))
    ends = (state.pos[2], state.pos[2] + ROLL_H * a[2])
    return float(min(ends) - rim)


def _centre(state) -> np.ndarray:
    return np.asarray(state.pos, float) + _axis(state) * ROLL_H / 2


def _seated(objects, name, col) -> bool:
    s, c = objects[name], cup(col)
    return bool(
        abs(_lowest(s, ROLL_R[name]) - c[2]) <= SEAT_Z + EPS
        and _tilt(s) <= MAX_TILT + EPS
        and math.hypot(s.pos[0] - c[0], s.pos[1] - c[1]) <= SEAT_XY + EPS
    )


def _in_bin(objects, name) -> bool:
    s, b = objects[name], objects["grey_bin"]
    rb = _rot(b.quat_xyzw)
    local = rb.T @ (_centre(s) - np.asarray(b.pos, float))
    return bool(
        rb[2, 2] > math.cos(math.radians(10.0))
        and abs(local[0]) <= BIN_IN / 2 + EPS
        and abs(local[1]) <= BIN_IN / 2 + EPS
        and _lowest(s, ROLL_R[name]) <= b.pos[2] + BIN_FLOOR + BIN_Z + EPS
    )


def _on_table(objects, name) -> bool:
    return bool(abs(_lowest(objects[name], ROLL_R[name]) - TOP) <= TABLE_Z + EPS)


def _full_cols(lot):
    """The printer each full roll stands on in this lot."""
    out = {}
    for n in FULLS:
        y = lot.pos(n)[1]
        out[n] = min(COLUMNS, key=lambda c: abs(COLUMNS[c] - y))
    return out


# Whole-trial record: did either full roll in a printer ever move more than 10 mm from its start? Cleared only by a
# scene reset: every roll and the bin back within 0.5 mm of their starts and at rest, after one of them jumped more
# than 20 mm in one control step (a robot cannot move them that way).
_record = {"fault": False, "prev": None}
# Released: in the simulator an on_step callback measures, every step, how close both arms' fingers and grippers
# are to the spare and the old roll. Offline (near_misses.py) there is no robot, so "clear" stays True.
_grip = {"clear": True}


def _update(objects) -> None:
    lot = lots.current()
    pos = {n: np.asarray(objects[n].pos, float) for n in NAMES}
    prev = _record["prev"]
    if prev is not None:
        home = all(np.linalg.norm(pos[n] - np.asarray(lot.pos(n))) < 0.0005 and objects[n].resting() for n in NAMES)
        jumped = any(np.linalg.norm(pos[n] - prev[n]) > 0.020 for n in NAMES)
        if home and jumped:
            _record["fault"] = False
    _record["prev"] = pos
    if any(np.linalg.norm(pos[n] - np.asarray(lot.pos(n))) > PULLED + EPS for n in FULLS):
        _record["fault"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    boxes = []
    for name, part in (("spare_roll", SPARE), ("low_roll", LOW)):
        s = st[name]
        boxes += part.world_boxes({"base": (np.asarray(s.pos), _rot(s.quat_xyzw))})
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


def _tier(objects) -> float:
    if _record["fault"]:
        return 0.0
    lot = lots.current()
    if not (
        _seated(objects, "spare_roll", lot.low)
        and all(_seated(objects, n, col) for n, col in _full_cols(lot).items())
        and _grip["clear"]
        and all(objects[n].resting() for n in NAMES)
    ):
        return 0.0
    if _in_bin(objects, "low_roll"):
        return 1.0
    if _on_table(objects, "low_roll"):
        return 0.5
    return 0.0


def success(objects):
    """The spare is on the spindle of the printer that held the nearly used-up roll, the old roll is in the bin, the
    other two printers' rolls are still on their spindles and never moved more than 10 mm, nothing is held, and all
    is still."""
    _update(objects)
    return _tier(objects) == 1.0


def distance(objects):
    return float(np.linalg.norm(np.asarray(objects["spare_roll"].pos) - np.asarray(cup(lots.current().low))))


def score(objects):
    """1.0 on success; 0.5 if everything holds except that the old roll was set down on the table, not in the bin;
    otherwise 0."""
    _update(objects)
    return _tier(objects)


# --- demo -----------------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
GRIP_DOWN = 0.010  # fingers reach 10 mm down the roll's side
GRIP_DZ = ROLL_H - GRIP_DOWN + TIP  # TCP above a roll's underside when gripping it
TRAVEL = TOP + 0.13
LOW_OPEN, LOW_SHUT = -0.035, -(LOW_R - 0.003)  # pads 70 mm apart, then closed 3 mm past the 50 mm roll's faces
FULL_OPEN, FULL_SHUT = -0.047, -(FULL_R - 0.003)  # fully open (94 mm), then 3 mm past the 86 mm roll's faces
CLEAR_SPINDLE = DECK + SPINDLE_H + STUB_H + 0.017  # a carried roll's underside 17 mm over the spindle top
# Offset of a carried roll from the commanded TCP (m, per arm), subtracted when fitting; zero was enough in traces.
FIT_OFFSET = {"left": (0.0, 0.0), "right": (0.0, 0.0)}
ARM_SIDE = {"left": 1.0, "right": -1.0}


def _w(x, y, z):
    return (float(x), float(y), float(z))


def _bin_old(arm, src, dst):
    """Lift the old roll off its spindle at src (x, y) and drop it into the bin at dst (x, y)."""
    (cx, cy), (bx, by) = src, dst
    cz = TOP + DECK
    return [
        Waypoint(1.5, _w(cx, cy, TRAVEL), LOW_OPEN, "over the old roll"),
        Waypoint(1.5, _w(cx, cy, cz + GRIP_DZ), LOW_OPEN, "down around it"),
        Waypoint(0.8, _w(cx, cy, cz + GRIP_DZ), LOW_SHUT, "grip"),
        Waypoint(1.5, _w(cx, cy, TOP + CLEAR_SPINDLE + GRIP_DZ), LOW_SHUT, "lift off the spindle"),
        Waypoint(1.5, _w(bx, by, TOP + BIN_H + 0.015 + GRIP_DZ), LOW_SHUT, "carry over the bin"),
        Waypoint(0.5, _w(bx, by, TOP + BIN_H + 0.015 + GRIP_DZ), LOW_OPEN, "drop it in"),
        Waypoint(1.0, _w(bx, by, TRAVEL), FULL_OPEN, "up"),
    ]


def _fit_spare(arm, src, dst):
    """Pick up the spare at src (x, y) and lower it over the spindle at dst (x, y)."""
    (sx, sy), (cx, cy) = src, dst
    ox, oy = FIT_OFFSET[arm]
    fx, fy = cx - ox, cy - oy
    cz = TOP + DECK
    carry = TOP + CLEAR_SPINDLE + GRIP_DZ
    return [
        Waypoint(1.5, _w(sx, sy, TRAVEL), FULL_OPEN, "over the spare"),
        Waypoint(1.5, _w(sx, sy, TOP + GRIP_DZ), FULL_OPEN, "down around it"),
        Waypoint(0.8, _w(sx, sy, TOP + GRIP_DZ), FULL_SHUT, "grip"),
        Waypoint(1.2, _w(sx, sy, carry), FULL_SHUT, "lift"),
        Waypoint(2.0, _w(fx, fy, carry), FULL_SHUT, "carry over the spindle"),
        Waypoint(1.0, _w(fx, fy, carry), FULL_SHUT, "settle over it"),
        Waypoint(2.0, _w(fx, fy, cz + 0.006 + GRIP_DZ), FULL_SHUT, "lower over the spindle"),
        Waypoint(0.6, _w(fx, fy, cz + 0.006 + GRIP_DZ), FULL_OPEN, "release"),
        Waypoint(1.0, _w(fx, fy, TRAVEL), FULL_OPEN, "up"),
    ]


def _park(arm, y, t):
    return [Waypoint(t, _w(0.18, y, TRAVEL), FULL_OPEN, "back toward the robot, wait")]


def demo():
    """Lot-aware by what the cameras show: lift the thin red-striped roll off its spindle and drop it in the bin,
    then lower the spare over that spindle and let go. An arm only works its own side and the centre: the centre
    printer's old roll is taken by the arm on the bin's side, and the spare by the arm on the spare's side. The demo
    reads nothing from the simulator."""
    lot = lots.current()
    low = cup(lot.low)[:2]
    spare = lot.pos("spare_roll")[:2]
    bin_xy = lot.pos("grey_bin")[:2]
    side = lambda y: "left" if y > 1e-9 else ("right" if y < -1e-9 else None)  # noqa: E731
    bin_arm = side(bin_xy[1]) or side(low[1])
    spare_arm = side(spare[1]) or side(low[1])
    a = _bin_old(bin_arm, low, bin_xy)
    if bin_arm == spare_arm:
        a += _fit_spare(bin_arm, spare, low) + _park(bin_arm, ARM_SIDE[bin_arm] * 0.20, 3.0)
        return {bin_arm: a}
    a += _park(bin_arm, ARM_SIDE[bin_arm] * 0.25, 1.0)
    wait = sum(w.duration for w in a)
    b = [Waypoint(wait, _w(0.18, ARM_SIDE[spare_arm] * 0.25, TRAVEL), FULL_OPEN, "wait for the other arm")]
    b += _fit_spare(spare_arm, spare, low) + _park(spare_arm, ARM_SIDE[spare_arm] * 0.20, 3.0)
    return {bin_arm: a, spare_arm: b}
