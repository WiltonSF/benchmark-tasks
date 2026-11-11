"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A
(first part inner, second outer). Each case: (label, {object: spec}, expected score, expected success). Cases run in
order and share the whole-trial record, so a strike stays until a scene reset (an empty case after the head moved).
"""

import math

import numpy as np

from task import DRILL, HEAD, PART_A, PART_B, Q_PARK, R_IN, R_OUT, R_START, TOP, ARM_TOP, at, WP_SIZE

HEAD_Z = TOP + ARM_TOP


def drill(q):
    return DRILL.state(joints={"swing": math.radians(q)})


def head(q, r, vel=None):
    return HEAD.state(pos=(*at(q, r), HEAD_Z), lin_vel=vel)


def parked(q=Q_PARK, r=R_IN, vel=None):
    return {"radial_drill": drill(q), "drill_head": head(q, r, vel)}


def moved(part, dx=0.0, dy=0.0, quat=None):
    p = np.asarray(part.pos) + (dx, dy, 0.0)
    a = part.yaw / 2
    q = quat if quat is not None else (0.0, 0.0, math.sin(a), math.cos(a))
    return (tuple(p), WP_SIZE, None, q)


def tilt(part, deg):
    """The part tipped `deg` about its own long axis (approximately: about world x, then its yaw)."""
    a, t = part.yaw / 2, math.radians(deg) / 2
    qz = np.array([0.0, 0.0, math.sin(a), math.cos(a)])
    qx = np.array([math.sin(t), 0.0, 0.0, math.cos(t)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    q = (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
         w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)
    return moved(part, quat=q)


RESET = {}
CASES = [
    ("parked, head run in", parked(), 1.0, True),
    ("parked at the 2 deg tolerance", parked(Q_PARK - 2.0), 1.0, True),
    ("arm 2.1 deg short of the stop", parked(Q_PARK - 2.1), 0.0, False),
    ("head 5 mm out from its inner end", parked(r=R_IN + 0.005), 1.0, True),
    ("head 6 mm out: parked, not run in", parked(r=R_IN + 0.006), 0.5, False),
    ("parked with the head run out", parked(r=R_OUT), 0.5, False),
    ("parked, head still moving", parked(vel=(0.05, 0.0, 0.0)), 0.0, False),
    ("arm half way, head in", parked(55.0), 0.0, False),
    ("head run in at the start, arm not swung", parked(0.0), 0.0, False),
    ("start", {}, 0.0, False),
    ("part A nudged 3 mm, then parked: 1.0", {"part_a": moved(PART_A, 0.003), **parked()}, 1.0, True),
    ("scene reset", RESET, 0.0, False),
    # Strikes from here on; each is followed by a scene reset.
    ("part A pushed 4 mm by the spindle", {"part_a": moved(PART_A, 0.004), "drill_head": head(20.0, R_START),
                                           "radial_drill": drill(20.0)}, 0.0, False),
    ("then parked, part A back in place: still 0", parked(), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("after the reset, parked: 1.0", parked(), 1.0, True),
    ("scene reset", RESET, 0.0, False),
    ("part B pushed 5 mm sideways", {"part_b": moved(PART_B, 0.0, 0.005)}, 0.0, False),
    ("then parked with the head run out: still 0", parked(r=R_OUT), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("part B tipped 4 deg", {"part_b": tilt(PART_B, 4.0), **parked(60.0)}, 0.0, False),
    ("then parked: still 0", parked(), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("part A lifted out of the way, swung past", {"part_a": ((PART_A.pos[0], PART_A.pos[1], PART_A.pos[2] + 0.08),
                                                             WP_SIZE, None), **parked(40.0)}, 0.0, False),
    ("then part A put back exactly, parked: still 0", parked(), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("part A tipped 2.9 deg (no strike), parked: 1.0", {"part_a": tilt(PART_A, 2.9), **parked()}, 1.0, True),
]
