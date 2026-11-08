"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A
(the full-depth hole is the middle one, hole 2). Each case: (label, {object: state}, expected score, expected
success); a state is ready-made from the part's own geometry (ANCHOR.state(...)) or a plain (pos, size, vel, quat).
"""

import math

from task import ANCHOR, HOLES, PLUG, PROUD, SLAB_H, TOP, _plug_at

Z_FLUSH = TOP + SLAB_H


def tilt_x(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def anchor(i=1, dx=0.0, dy=0.0, dz=0.0, quat=None, vel=None):
    x, y = HOLES[i]
    return {"anchor": ANCHOR.state(pos=(x + dx, y + dy, Z_FLUSH + dz), quat_xyzw=quat, lin_vel=vel)}


CASES = [
    ("flush in the full-depth hole, centred", anchor(1), 1.0, True),
    ("flush, axis 3 mm off centre front to back (limit)", anchor(1, dx=0.003), 1.0, True),
    ("axis 3.1 mm off centre", anchor(1, dy=0.0031), 0.0, False),
    ("washer 1 mm above the slab (limit)", anchor(1, dz=0.001), 1.0, True),
    ("washer 1.1 mm above the slab", anchor(1, dz=0.0011), 0.0, False),
    ("tilted 3 degrees (limit)", anchor(1, quat=tilt_x(3.0)), 1.0, True),
    ("tilted 3.2 degrees", anchor(1, quat=tilt_x(3.2)), 0.0, False),
    ("flush but still moving", anchor(1, vel=(0.0, 0.0, -0.02)), 0.0, False),
    ("in the full-depth hole, 10 mm proud (dropped in, not yet down)", anchor(1, dz=0.010), 0.0, False),
    ("habit: the nearest hole (hole 1), on its plug, 20 mm proud", anchor(0, dz=PROUD), 0.0, False),
    ("habit: the last hole (hole 3), on its plug", anchor(2, dz=PROUD), 0.0, False),
    ("pressed into hole 1 as if flush (cannot happen: the plug is rigid)", anchor(0), 0.0, False),
    ("pressed into hole 3 as if flush", anchor(2), 0.0, False),
    ("lying on the slab top across the middle hole",
     {"anchor": ANCHOR.state(pos=(HOLES[1][0], HOLES[1][1], Z_FLUSH + 0.013), quat_xyzw=tilt_x(90))}, 0.0, False),
    ("left standing in its holder", {}, 0.0, False),
    ("on the table beside the slab", {"anchor": ANCHOR.state(pos=(0.24, 0.05, TOP + 0.045))}, 0.0, False),
    ("flush in the middle hole but a plug still settling",
     {**anchor(1), "plug_p": (_plug_at(0), PLUG, (0.0, 0.0, 0.05))}, 0.0, False),
]
