"""Near-successes and unintended strategies, scored offline by the factory checker (lot A).

Each case: (label, {object: ready state or (centre, size or None, velocity or None[, quat_xyzw])}, expected score,
expected success).
"""

import math

import task
from task import (BOLT_MARGIN, CLICK, FLOOR, FOB, HEIGHT, IN_X0, IN_X1, IN_Y, KEY, OFF, SAFE, SHUT, TOP, X0, Y0)

FOB_Z = TOP + FLOOR + FOB[2] / 2
IN = (X0 + IN_X0 + 0.029, Y0, FOB_Z)  # where the demo leaves the key
LEN = 0.022 + 0.024  # fob plus blade along x
TURN90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
ON_SIDE = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))


def safe(lid=0.0, **w):
    return SAFE.state(joints={"lid": lid, "left": 0.0, "middle": 0.0, "right": 0.0, **w})


def key(x=IN[0], y=IN[1], z=IN[2], quat=(0.0, 0.0, 0.0, 1.0), vel=None):
    return KEY.state(pos=(x, y, z), quat_xyzw=quat, lin_vel=vel)


LOCKED = safe(middle=CLICK)
CASES = [
    ("start: lid open, key on the table", {"key_safe": safe(1.90)}, 0.0, False),
    ("key inside, lid open, on the code", {"key_safe": safe(1.90), "key": key()}, 0.0, False),
    ("key inside, lid shut, on the code (shut, not locked)", {"key_safe": safe(), "key": key()}, 0.0, False),
    ("key inside, lid shut, middle wheel one number off (locked)", {"key_safe": LOCKED, "key": key()}, 1.0, True),
    ("key inside, lid shut, left wheel one number back", {"key_safe": safe(left=-CLICK), "key": key()}, 1.0, True),
    ("key inside, lid shut, all three wheels off", {"key_safe": safe(left=CLICK, middle=2 * CLICK, right=-CLICK),
                                                    "key": key()}, 1.0, True),
    ("wheel turned a full turn back to the code", {"key_safe": safe(middle=2 * math.pi), "key": key()}, 0.0, False),
    ("wheel exactly half a number off (limit)", {"key_safe": safe(middle=OFF), "key": key()}, 1.0, True),
    ("wheel just under half a number off", {"key_safe": safe(middle=OFF - 0.001), "key": key()}, 0.0, False),
    ("lid 2 degrees open (limit), locked wheel", {"key_safe": safe(SHUT, middle=CLICK), "key": key()}, 1.0, True),
    ("lid 2.1 degrees open, locked wheel", {"key_safe": safe(math.radians(2.1), middle=CLICK), "key": key()}, 0.0,
     False),
    ("wheels turned first: lid held 9 degrees open by the bolt", {"key_safe": safe(0.15, middle=CLICK), "key": key()},
     0.0, False),
    ("wheel turned with the lid just over 2 degrees open: held by the bolt",
     {"key_safe": safe(SHUT + BOLT_MARGIN, middle=CLICK), "key": key()}, 0.0, False),
    ("wheels turned first, lid open, key inside", {"key_safe": safe(1.90, middle=CLICK), "key": key()}, 0.0, False),
    ("locked shut with the key still on the table", {"key_safe": LOCKED}, 0.0, False),
    ("locked shut, key on top of the lid", {"key_safe": LOCKED, "key": key(z=TOP + HEIGHT + 0.008 + FOB[2] / 2)},
     0.0, False),
    ("locked, key on the strip in front of the lid", {"key_safe": LOCKED, "key": key(x=X0 - 0.045,
                                                                                    z=TOP + HEIGHT + FOB[2] / 2)},
     0.0, False),
    ("locked, key against the back wall (inside, limit)",
     {"key_safe": LOCKED, "key": key(x=X0 + IN_X1 - LEN + FOB[0] / 2)}, 1.0, True),
    ("locked, key 1 mm past the back wall (outside)",
     {"key_safe": LOCKED, "key": key(x=X0 + IN_X1 - LEN + FOB[0] / 2 + 0.001)}, 0.0, False),
    ("locked, key turned across the box against the left wall",
     {"key_safe": LOCKED, "key": key(y=Y0 + IN_Y - LEN + FOB[0] / 2, quat=TURN90)}, 1.0, True),
    ("locked, key lying on its side inside", {"key_safe": LOCKED, "key": key(z=TOP + FLOOR + FOB[1] / 2,
                                                                         quat=ON_SIDE)}, 1.0, True),
    ("locked, key still moving", {"key_safe": LOCKED, "key": key(vel=(0.0, 0.05, 0.0))}, 0.0, False),
    ("locked, key knocked off the table", {"key_safe": LOCKED, "key": key(x=0.20, y=0.20, z=FOB[2] / 2)}, 0.0, False),
]


def _bolt_order_check() -> None:
    """The bolt, in order: a lid open 2.5 degrees when a wheel is turned is held at least 2 degrees + 0.02 rad open
    however hard it is then pressed, and frees only when every wheel is back on the code. Shut first, then turned:
    locked. Runs the task's interlock rule on recorded limits, without a simulator."""
    limits, real = [], task._write_limits
    task._write_limits = lambda sim, lo, hi: limits.append((lo, hi))
    task._sim.update(limits=None, lock_at=None, bolt_at=None)
    j = {"lid": math.radians(2.5), "left": 0.0, "middle": CLICK, "right": 0.0}
    task._interlock(None, j)
    for lid in (0.04, 0.03, 0.0):  # pressed down: the limit must not follow
        task._interlock(None, {**j, "lid": lid})
    assert all(lo >= SHUT + BOLT_MARGIN - 1e-9 for lo, _ in limits), limits
    task._interlock(None, {**j, "middle": 0.0})
    assert limits[-1] == (0.0, task.LID_MAX), limits
    task._interlock(None, {**j, "lid": 0.0, "middle": 0.0})
    task._interlock(None, {**j, "lid": 0.0})
    assert limits[-1] == (0.0, 0.001), limits
    task._write_limits = real
    task._sim.update(limits=None, lock_at=None, bolt_at=None)


_bolt_order_check()
