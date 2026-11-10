"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A
(bin under the counter left of the midline, can on the right). Each case: (label, {object: spec}, expected score,
expected success); a spec is a ready state (PEDAL_BIN.state(...)) or a (centre, size, velocity[, quat]) tuple for
the can. Cases run in order and share the whole-trial record, so a fault stays until a scene reset (an empty case
after the can was moved).
"""

import math

from task import BIN_Y, CAN_H, CAN_R, CX0, HINGE_X, PEDAL_BIN, TOP, UNDER_X, WALL

CAN_SIZE = (2 * CAN_R, 2 * CAN_R, CAN_H)
OUT_X = CX0 - HINGE_X - 0.010  # bin pulled out: hinge 10 mm in front of the counter


def quat_y(deg):
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def bin_at(x, lid=0.0, pedal=0.0, quat=None, z=0.0):
    return {"pedal_bin": PEDAL_BIN.state(pos=(x, BIN_Y, TOP + z), joints={"lid": lid, "pedal": pedal},
                                         quat_xyzw=quat)}


def can_in(x, dx=0.0, dy=0.0, lying=False):
    if lying:
        return {"can": ((x + dx, BIN_Y + dy, TOP + WALL + CAN_R), CAN_SIZE, None, (0.7071068, 0.0, 0.0, 0.7071068))}
    return {"can": ((x + dx, BIN_Y + dy, TOP + WALL + CAN_H / 2), CAN_SIZE, None)}


SCENE_RESET = {}

CASES = [
    ("pulled out, can standing in the bin, lid shut", {**bin_at(OUT_X), **can_in(OUT_X)}, 1.0, True),
    ("pulled out, can lying in the bin, lid shut", {**bin_at(OUT_X), **can_in(OUT_X, lying=True)}, 1.0, True),
    ("pulled out, can in the bin's corner", {**bin_at(OUT_X), **can_in(OUT_X, dx=-0.020, dy=0.020)}, 1.0, True),
    ("pulled out, can in the bin, lid 4 degrees open", {**bin_at(OUT_X, lid=math.radians(4)), **can_in(OUT_X)},
     1.0, True),
    ("pulled out, can in the bin, lid 5 degrees open (the limit)",
     {**bin_at(OUT_X, lid=math.radians(5)), **can_in(OUT_X)}, 1.0, True),
    ("pulled out, can in the bin, lid 6 degrees open", {**bin_at(OUT_X, lid=math.radians(6)), **can_in(OUT_X)},
     0.0, False),
    ("pulled out, lid held open on the pedal, can in", {**bin_at(OUT_X, lid=1.5, pedal=0.25), **can_in(OUT_X)},
     0.0, False),
    ("pulled out (hinge 10 mm clear), can still on the table", bin_at(OUT_X), 0.0, False),
            ("pulled out only 40 mm, can on the table", bin_at(UNDER_X - 0.040), 0.0, False),
    ("pulled out, can on the floor",
     {**bin_at(OUT_X), "can": ((0.20, -0.15, TOP - 0.20), CAN_SIZE, None)}, 0.0, False),
    ("pulled out, can lying on the shut lid",
     {**bin_at(OUT_X), "can": ((OUT_X, BIN_Y, TOP + 0.086 + CAN_R), CAN_SIZE, None,
                               (0.7071068, 0.0, 0.0, 0.7071068))}, 0.0, False),
    ("pulled out, lid open, can still on the table", bin_at(OUT_X, lid=1.5, pedal=0.25), 0.0, False),
    ("bin under the counter, pedal pressed, lid stopped by the counter", bin_at(UNDER_X, lid=0.25, pedal=0.25),
     0.0, False),
    ("bin under the counter untouched, can laid beside it", {"can": ((CX0 - 0.03, 0.0, TOP + CAN_H / 2), CAN_SIZE,
                                                                     None)}, 0.0, False),
    ("pulled out, can in, but the can still moving",
     {**bin_at(OUT_X), "can": ((OUT_X, BIN_Y, TOP + WALL + CAN_H / 2), CAN_SIZE, (0.0, 0.0, -0.1))}, 0.0, False),
    ("can in the bin while still under the counter, lid shut (can only get there with the bin pulled)",
     {**bin_at(UNDER_X), **can_in(UNDER_X)}, 1.0, True),
    ("pulled out, bin tilted 8 degrees, can in", {**bin_at(OUT_X, quat=quat_y(8), z=0.004), **can_in(OUT_X)},
     1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("bin tipped 35 degrees", bin_at(OUT_X, quat=quat_y(35), z=0.03), 0.0, False),
    ("then set upright with the can in: still 0", {**bin_at(OUT_X), **can_in(OUT_X)}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("after the reset, can in the pulled-out bin: 1.0", {**bin_at(OUT_X), **can_in(OUT_X)}, 1.0, True),
]
