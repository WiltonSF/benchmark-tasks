"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state or (centre, size or None, velocity or None[, quat_xyzw])}, expected score,
expected success). Cases run in order on one copy of task.py, so the full tote's whole-trial records (a grey tote
gone down past its lugs, its carton moved over 5 mm) carry from one case to the next; a "new trial" case (every
object back at its start after a jump) clears them. The full tote and the empty tote each earn 0.5.

There is no robot offline, so both grey totes are taken as let go (more than 5 mm from every finger) except in the
cases that say they are held; in the simulator this is measured every step and gives no credit until measured.
"""

import math

import numpy as np

import lots
import parts
from task import BLUE_XY, CARTON_Z, GREY, LUG_TOP, NEST_RIM, H, TOP

grip = parts.__dict__.setdefault("_stack_nest_totes_grip", {})


def _q(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return (*(a * math.sin(angle / 2)), math.cos(angle / 2))


def _mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


AS_IS, TURNED = (0.0, 0.0, 0.0, 1.0), _q((0, 0, 1), math.pi)
NEST_BOTTOM = NEST_RIM - H  # 38.3 mm
PRESSED_BOTTOM = 0.054  # nested onto the carton's top


# Which blue tote is full in each lot, as the cases name them ("full"/"empty"); the case's lot is set while it is read.
_LOT = {"now": "A"}
_FULL = {"A": "left", "B": "right"}


def _side(key):
    if key in ("left", "right"):
        return key
    full = _FULL[_LOT["now"]]
    return full if key == "full" else ("right" if full == "left" else "left")


def grey(name, key, bottom, quat=AS_IS, dx=0.0, vel=None):
    """Grey tote `name` with its underside `bottom` m above the table, centred over blue tote `key` ("full",
    "empty", "left" or "right")."""
    bx, by = BLUE_XY[_side(key)]
    return GREY[name].state(pos=(bx + dx, by, TOP + bottom), quat_xyzw=quat, lin_vel=vel)


def stack(name, quat=TURNED, **kw):
    return grey(name, "full", LUG_TOP, quat, **kw)


def nest(name, quat=AS_IS, **kw):
    return grey(name, "empty", NEST_BOTTOM, quat, **kw)


def carton_at(key, d=0.0):
    bx, by = BLUE_XY[_side(key)]
    return ((bx + d, by, CARTON_Z), None, None)


def carton_moved(d):
    return carton_at("full", d)


def held(name, label, objs, want):
    """A case in which grey tote `name` is still within 5 mm of a finger (None: the grip was never measured)."""
    return (label, objs, want, False, name)


NEW = ("new trial: everything back at its start", {}, 0.0, False)

CASES_RAW = [
    ("left turned and stacked on the full tote, right nested in the empty one",
     {"grey_left": stack("left"), "grey_right": nest("right")}, 1.0, True),
    ("left stacked, right left on the table", {"grey_left": stack("left")}, 0.5, False),
    ("right nested, left left on the table", {"grey_right": nest("right")}, 0.5, False),
    ("never turn: right nested, left nested onto the carton (pressed)",
     {"grey_left": grey("left", "full", PRESSED_BOTTOM), "grey_right": nest("right")}, 0.5, False),
    ("then the left lifted out, turned and stacked: it pressed earlier, so only the empty tote counts",
     {"grey_left": stack("left"), "grey_right": nest("right")}, 0.5, False),
    NEW,
    ("always turn: left stacked, right turned and so stacked on the empty tote's lugs, not nested",
     {"grey_left": stack("left"), "grey_right": grey("right", "empty", LUG_TOP, TURNED)}, 0.5, False),
    NEW,
    ("swapped: the right grey tote turned and stacked on the full tote, the left nested in the empty one",
     {"grey_right": stack("right"), "grey_left": nest("left")}, 1.0, True),
    ("left lowered nest-wise over the full tote to an underside 63.0 mm up, 2 mm below the lug tops (limit)",
     {"grey_left": grey("left", "full", LUG_TOP - 0.002)}, 0.0, False),
    ("then turned and stacked, right nested: never went past the lugs",
     {"grey_left": stack("left"), "grey_right": nest("right")}, 1.0, True),
    ("left lowered nest-wise over the full tote to 62.9 mm: past the lugs, counts",
     {"grey_left": grey("left", "full", LUG_TOP - 0.0021)}, 0.0, False),
    ("then stacked, right nested: the full tote stays at 0, the empty one earns 0.5",
     {"grey_left": stack("left"), "grey_right": nest("right")}, 0.5, False),
    NEW,
    ("left 1.5 mm above the lugs (exact limit)",
     {"grey_left": grey("left", "full", LUG_TOP + 0.0015, TURNED), "grey_right": nest("right")}, 1.0, True),
    ("left 2 mm above the lugs: held up", {"grey_left": grey("left", "full", LUG_TOP + 0.002, TURNED),
                                           "grey_right": nest("right")}, 0.5, False),
    ("left on the lugs but tilted 2 degrees about x",
     {"grey_left": stack("left", _mul(_q((1, 0, 0), math.radians(2)), TURNED)), "grey_right": nest("right")},
     0.5, False),
    ("left on the lugs, still moving",
     {"grey_left": stack("left", vel=(0.0, 0.0, -0.05)), "grey_right": nest("right")}, 0.5, False),
    ("left 5 mm off centre (limit)", {"grey_left": stack("left", dx=0.005), "grey_right": nest("right")}, 1.0, True),
    ("left 6 mm off centre", {"grey_left": stack("left", dx=0.006), "grey_right": nest("right")}, 0.5, False),
    ("right wedged 8 mm high in the taper (limit)",
     {"grey_left": stack("left"), "grey_right": grey("right", "empty", NEST_BOTTOM + 0.008)}, 1.0, True),
    ("right 9 mm above where it nests: not down in the tote",
     {"grey_left": stack("left"), "grey_right": grey("right", "empty", NEST_BOTTOM + 0.009)}, 0.5, False),
    ("right 4 mm below where it nests (limit)",
     {"grey_left": stack("left"), "grey_right": grey("right", "empty", NEST_BOTTOM - 0.004)}, 1.0, True),
    ("right 5 mm below where it nests: not a nested tote",
     {"grey_left": stack("left"), "grey_right": grey("right", "empty", NEST_BOTTOM - 0.005)}, 0.5, False),
    ("right wedged 4 mm high and cocked 1.3 degrees (as in a GPU trace)",
     {"grey_left": stack("left"), "grey_right": grey("right", "empty", NEST_BOTTOM + 0.004,
                                                     _q((0, 1, 0), math.radians(1.3)))}, 1.0, True),
    ("right nested but tilted 4 degrees about y",
     {"grey_left": stack("left"), "grey_right": nest("right", _q((0, 1, 0), math.radians(4)))}, 0.5, False),
    ("right upside down over the empty tote at nest height",
     {"grey_left": stack("left"), "grey_right": grey("right", "empty", NEST_BOTTOM + H, _q((1, 0, 0), math.pi))},
     0.5, False),
    ("both stacked, carton moved 5 mm (limit)",
     {"grey_left": stack("left"), "grey_right": nest("right"), "carton": carton_moved(0.005)}, 1.0, True),
    ("both done, carton moved 6 mm", {"grey_left": stack("left"), "grey_right": nest("right"),
                                      "carton": carton_moved(0.006)}, 0.5, False),
    ("then the carton put back where it started: it was moved earlier, so only the empty tote counts",
     {"grey_left": stack("left"), "grey_right": nest("right")}, 0.5, False),
    NEW,
    ("both grey totes in the empty tote, one nested in the other: only the empty tote counts",
     {"grey_right": nest("right"), "grey_left": grey("left", "empty", NEST_BOTTOM + (NEST_RIM - H))}, 0.5, False),
    ("a grey tote turned and set on the full tote's rim corners (lifted 10 mm): not on the lugs",
     {"grey_left": grey("left", "full", LUG_TOP + 0.010, TURNED), "grey_right": nest("right")}, 0.5, False),
    ("carton lifted out, left nested to the full tote's floor",
     {"carton": ((0.20, 0.30, TOP + 0.025), None, None), "grey_left": grey("left", "full", NEST_BOTTOM)}, 0.0, False),
    ("then the carton put back and the left stacked, right nested: the full tote stays at 0",
     {"grey_left": stack("left"), "grey_right": nest("right")}, 0.5, False),
    NEW,
    held("left", "left stacked but a finger still within 5 mm of it",
         {"grey_left": stack("left"), "grey_right": nest("right")}, 0.5),
    held("right", "right nested but a finger still within 5 mm of it",
         {"grey_left": stack("left"), "grey_right": nest("right")}, 0.5),
    held(None, "grip state unknown (never measured): no credit",
         {"grey_left": stack("left"), "grey_right": nest("right")}, 0.0),
    ("both let go again", {"grey_left": stack("left"), "grey_right": nest("right")}, 1.0, True),
]


# Lot B: the carton is in the right blue tote, so the right grey tote must be turned and stacked and the left one
# nested. Every case gives the carton's place, since the checker starts each case from lot A's layout.
_LOT["now"] = "B"
C_B = {"carton": carton_at("full")}
NEW_B = ("lot B, new trial: everything back at lot B's start", dict(C_B), 0.0, False)
CASES_B = [
    NEW_B,
    ("lot B: right turned and stacked on the full tote, left nested in the empty one",
     {**C_B, "grey_right": stack("right"), "grey_left": nest("left")}, 1.0, True),
    NEW_B,
    ("lot B, fixed habit from lot A (turn the left tote only): the left stacks on the empty tote's lugs and the right "
     "nests onto the carton", {**C_B, "grey_left": grey("left", "empty", LUG_TOP, TURNED),
                               "grey_right": grey("right", "full", PRESSED_BOTTOM)}, 0.0, False),
    NEW_B,
    ("lot B, never turn: left nested, right nested onto the carton",
     {**C_B, "grey_left": nest("left"), "grey_right": grey("right", "full", PRESSED_BOTTOM)}, 0.5, False),
    NEW_B,
    ("lot B, always turn: right stacked, left stacked on the empty tote's lugs",
     {**C_B, "grey_right": stack("right"), "grey_left": grey("left", "empty", LUG_TOP, TURNED)}, 0.5, False),
    NEW_B,
    ("lot B: both done but the carton moved 6 mm", {"carton": carton_moved(0.006), "grey_right": stack("right"),
                                                    "grey_left": nest("left")}, 0.5, False),
]
_LOT["now"] = "A"
CASES_RAW[0:0] = [
    ("lot A, fixed habit from lot B (turn the right tote only): the right stacks on the empty tote's lugs and the "
     "left nests onto the carton", {"grey_right": grey("right", "empty", LUG_TOP, TURNED),
                                    "grey_left": grey("left", "full", PRESSED_BOTTOM)}, 0.0, False),
    NEW,
]


class _GripCase(dict):
    """The case's object changes; setting the grip state happens when the checker reads them."""

    def __init__(self, changes, held_name):
        super().__init__(changes)
        self._held = held_name

    def items(self):
        lots.use("A")
        if self._held is None:
            grip["left"] = grip["right"] = None
        else:
            grip["left"] = grip["right"] = True
            grip[self._held] = False
        return super().items()


def _plain(changes, lot="A"):
    class _Released(dict):
        def items(self):
            lots.use(lot)
            grip["left"] = grip["right"] = True
            return super().items()
    return _Released(changes)


CASES = [(c[0], _GripCase(c[1], c[4]), c[2], c[3]) if len(c) == 5 else (c[0], _plain(c[1]), c[2], c[3])
         for c in CASES_RAW]
CASES += [(c[0], _plain(c[1], "B"), c[2], c[3]) for c in CASES_B]
