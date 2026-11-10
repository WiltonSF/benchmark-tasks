"""Near-successes and unintended strategies, scored offline by the factory checker (lot A: both catches down).

Each case: (label, {object: ready state or (centre, size or None, velocity or None[, quat_xyzw])}, expected score,
expected success).
"""

import math

from task import (BAR, BAR_END, BAR_Y, BAR_Z, CATCH_UP, DOWN_MAX, HANG_Z, LEATHERS, LOOP_IN_Y, LOOP_IN_Z, LOOP_X,
                  POST_X, SADDLE, TOP, X0)

ON_X = X0 + BAR_END + 0.045
L, R = LEATHERS["left"], LEATHERS["right"]


def saddle(left=0.0, right=0.0):
    return SADDLE.state(joints={"catch_left": left, "catch_right": right})


def hang(part, side, x=ON_X, dy=0.0, dz=0.0, quat=(0.0, 0.0, 0.0, 1.0), vel=None):
    y = (1 if side == "left" else -1) * BAR_Y
    return part.state(pos=(x, y + dy, TOP + HANG_Z + dz), quat_xyzw=quat, lin_vel=vel)


def tilt(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


BOTH = {"leather_left": hang(L, "left"), "leather_right": hang(R, "right")}
SLACK_Y = (LOOP_IN_Y - BAR[1]) / 2
CASES = [
    ("start: both on their pegs", {}, 0.0, False),
    ("both hung, catches down", {**BOTH, "saddle": saddle()}, 1.0, True),
    ("both hung, each on the other's bar", {"leather_left": hang(L, "right"), "leather_right": hang(R, "left")}, 1.0,
     True),
    ("left hung, right still on its peg", {"leather_left": hang(L, "left")}, 0.5, False),
    ("right hung, left on the floor", {"leather_right": hang(R, "right"),
                                       "leather_left": L.state(pos=(0.2, 0.3, 0.02), quat_xyzw=tilt(90))}, 0.5, False),
    ("both hung, left catch flipped up", {**BOTH, "saddle": saddle(left=CATCH_UP)}, 0.0, False),
    ("both hung, both catches up", {**BOTH, "saddle": saddle(CATCH_UP, CATCH_UP)}, 0.0, False),
    ("both hung, a catch 30 degrees up (limit)", {**BOTH, "saddle": saddle(left=DOWN_MAX)}, 1.0, True),
    ("both hung, a catch swung 20 degrees toward the saddle (still down)", {**BOTH, "saddle": saddle(right=-0.35)},
     1.0, True),
    ("both hung, a catch 31 degrees up", {**BOTH, "saddle": saddle(left=math.radians(31))}, 0.0, False),
    ("one hung, other catch left up, other leather on its peg", {"leather_right": hang(R, "right"),
                                                                 "saddle": saddle(left=CATCH_UP)}, 0.0, False),
    ("both catches pushed down, leathers left on the pegs", {"saddle": saddle()}, 0.0, False),
    ("left loop at the open end (limit)", {**BOTH, "leather_left": hang(L, "left", x=X0 + BAR_END + LOOP_X / 2)},
     1.0, True),
    ("left loop 1 mm off the open end", {**BOTH, "leather_left": hang(L, "left", x=X0 + BAR_END + LOOP_X / 2 - 0.001)},
     0.5, False),
    ("left loop against the post (limit)", {**BOTH, "leather_left": hang(L, "left", x=X0 + POST_X - LOOP_X / 2)},
     1.0, True),
    ("left loop sideways at the slack's limit", {**BOTH, "leather_left": hang(L, "left", dy=SLACK_Y)}, 1.0, True),
    ("left loop 1 mm past the slack (would cut the bar)", {**BOTH, "leather_left": hang(L, "left", dy=SLACK_Y + 0.001)},
     0.5, False),
    ("left loop set on top of the bar, not round it", {**BOTH, "leather_left": hang(L, "left", dz=LOOP_IN_Z / 2 + 0.006
                                                                                       + 0.020)}, 0.5, False),
    ("left leather held 16 mm up off the bar", {**BOTH, "leather_left": hang(L, "left", dz=0.0161)}, 0.5, False),
    ("left leather swung 15 degrees", {**BOTH, "leather_left": hang(L, "left", quat=tilt(15))}, 1.0, True),
    ("left leather swung 25 degrees", {**BOTH, "leather_left": hang(L, "left", quat=tilt(25))}, 0.5, False),
    ("left leather still swinging", {**BOTH, "leather_left": hang(L, "left", vel=(0.0, 0.05, 0.0))}, 0.0, False),
    ("left leather standing on the table under the bar", {**BOTH, "leather_left": hang(L, "left", dz=-(HANG_Z - 0.100))},
     0.5, False),
]
