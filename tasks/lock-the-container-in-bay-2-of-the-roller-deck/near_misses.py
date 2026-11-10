"""Near-successes and unintended strategies for the offline checker, in lot A (container starting in bay 1). Each
case: (label, {object: state}, expected score, expected success). States come from the parts' own geometry, placed
in the sloping deck's frame. Cases run in order and share the whole-trial record, so a lift fault stays until a scene
reset (an empty case after the container was moved).

At import it also checks, offline, that no blind routine of pedal presses can end with both locks up in more than
one of the three lots (each lot starts a different set of locks down), and scores a blind two-sweep routine.
"""

import math

import lots
import numpy as np
from task import (BAY_Y, BAY_LEN, BOX, CENTRE_TOL, CONTAINER, DECK, LANE_C, LOCKED, LOCK_Y, R_DECK, ROLL_TOP,
                  TOP, score, success, to_world)


def _quat(m):
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    q = np.array([x, y, z, w])
    return tuple(q / np.linalg.norm(q))


def _r(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def box(y, x=LANE_C, dz=0.0, rot=None, vel=None):
    """The container at deck-frame (x, y), its underside dz above the roller tops, turned by rot in the deck frame."""
    m = R_DECK @ (np.eye(3) if rot is None else rot)
    return CONTAINER.state(pos=to_world(x, y, ROLL_TOP + dz), quat_xyzw=_quat(m), lin_vel=vel)


def deck(l12=0.0, l23=0.0):
    return DECK.state(joints={"lock_12": l12, "lock_23": l23})


DOWN = 0.023  # where a dropped lock rests
ON_12 = BAY_Y[1] + BAY_LEN / 2 - BOX[1] / 2  # in bay 2, resting down-slope on the 1|2 lock
ON_23 = BAY_Y[1] - BAY_LEN / 2 + BOX[1] / 2
STOWED = {"container": box(ON_12), "roller_deck": deck()}
SCENE_RESET = {}

CASES = [
    ("stowed: in bay 2 resting on the 1|2 lock, both locks up", STOWED, 1.0, True),
    ("stowed against the 2|3 lock", {"container": box(ON_23), "roller_deck": deck()}, 1.0, True),
    ("in bay 2, the 1|2 lock down", {"container": box(ON_12), "roller_deck": deck(l12=DOWN)}, 0.0, False),
    ("in bay 2, the 2|3 lock down", {"container": box(ON_12), "roller_deck": deck(l23=DOWN)}, 0.0, False),
    ("in bay 2, both locks down", {"container": box(BAY_Y[1]), "roller_deck": deck(DOWN, DOWN)}, 0.0, False),
    ("1|2 lock held 4 mm down (pedal pressed)", {"container": box(ON_12), "roller_deck": deck(l12=0.004)}, 0.0, False),
    ("1|2 lock 3 mm down: up", {"container": box(ON_12), "roller_deck": deck(l12=LOCKED)}, 1.0, True),
    ("1|2 lock half up", {"container": box(ON_12), "roller_deck": deck(l12=0.012)}, 0.0, False),
    ("still rolling in bay 2", {"container": box(ON_12, vel=(0.0, 0.05, 0.0)), "roller_deck": deck()}, 0.0, False),
    ("centre 15 mm off bay 2's centre", {"container": box(BAY_Y[1] + CENTRE_TOL), "roller_deck": deck()}, 1.0, True),
    ("centre 16 mm off", {"container": box(BAY_Y[1] + CENTRE_TOL + 0.001), "roller_deck": deck()}, 0.0, False),
    ("turned 4 degrees in the lane", {"container": box(BAY_Y[1], rot=_r("z", 4)), "roller_deck": deck()}, 1.0, True),
    ("left in bay 1 (start)", {}, 0.0, False),
    ("rolled back into bay 1 past the dropped 1|2 lock", {"roller_deck": deck(l12=DOWN)}, 0.0, False),
    ("pushed on into bay 3, locked there (wrong bay)", {"container": box(BAY_Y[2] + 0.006),
                                                       "roller_deck": deck()}, 0.0, False),
    ("straddling the dropped 1|2 lock", {"container": box(LOCK_Y[0]), "roller_deck": deck(l12=DOWN)}, 0.0, False),
    ("tilted 6 degrees on a lock edge", {"container": box(BAY_Y[1], dz=0.002, rot=_r("x", 6)),
                                         "roller_deck": deck()}, 0.0, False),
    ("pushed off the deck onto the table", {"container": CONTAINER.state(pos=(LANE_C, 0.0, TOP)),
                                            "roller_deck": deck()}, 0.0, False),
    ("resting 3 mm above the rollers", {"container": box(BAY_Y[1], dz=0.003), "roller_deck": deck()}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("carried 9 mm over the rollers: no fault", {"container": box(LOCK_Y[0], dz=0.009), "roller_deck": deck()},
     0.0, False),
    ("then stowed: 1.0", STOWED, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("lifted 11 mm over the 1|2 lock", {"container": box(LOCK_Y[0], dz=0.011), "roller_deck": deck()}, 0.0, False),
    ("then set down stowed: still 0", STOWED, 0.0, False),
    ("tipped onto its side over a lock: still 0", {"container": box(BAY_Y[1] - 0.04, dz=0.05, rot=_r("x", 90)),
                                                   "roller_deck": deck()}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("after the reset, stowed: 1.0", STOWED, 1.0, True),
]


def _blind_routines():
    """A blind routine presses each pedal a fixed number of times, whatever it sees. Each full press toggles a lock,
    so it ends with each lock flipped (odd count) or not (even). Score the best possible end (container stowed in
    bay 2) with the locks such a routine leaves, in every lot."""
    import task

    out = {}
    for p12 in (0, 1):
        for p23 in (0, 1):
            per_lot = []
            for lot in lots.all_lots():
                lots.use(lot.name)
                down = {k: (k in lot.down) ^ bool(n) for k, n in (("lock_12", p12), ("lock_23", p23))}
                st = {"roller_deck": deck(DOWN if down["lock_12"] else 0.0, DOWN if down["lock_23"] else 0.0),
                      "container": box(ON_12)}
                task._record.update(fault=False, prev=None)
                per_lot.append(score(st))
            out[(p12, p23)] = per_lot
    lots.use(None)
    lots.use("A")
    task._record.update(fault=False, prev=None)
    return out


_BLIND = _blind_routines()
assert all(sum(v) <= 1.0 for v in _BLIND.values()), _BLIND
# The two-sweep routine (drop and raise the 1|2 lock, then drop and raise the 2|3 lock: two presses each) leaves the
# locks as each lot started them, so at best it stows with both up in lot A only: a mean of at most 0.33.
TWO_SWEEP = _BLIND[(0, 0)]
assert TWO_SWEEP == [1.0, 0.0, 0.0], TWO_SWEEP
lots.use(None)
