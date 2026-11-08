"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state or (centre, size or None, velocity or None[, quat_xyzw])}, expected score,
expected success).
"""

import math

from task import (BOLT_TRAVEL, DOOR, DOOR_T, FLAP_SHUT, HALL_FLOOR, KEY, KEY_H, KEY_L, KEY_START, LOCK_Z, SLOT_Z0, TOP,
                  X_IN, X_OUT, YC, DOG_Y)


def door(bolt=0.0, flap=0.0):
    return DOOR.state(joints={"bolt": bolt, "flap": flap})


def key(x, y, z, vel=None, yaw=0.0):
    return KEY.state(pos=(x, y, z), quat_xyzw=(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)), lin_vel=vel)


ON_BLOCK = key(*KEY_START)
POSTED = key(X_IN + 0.030, YC, TOP + HALL_FLOOR + KEY_H / 2)
IN_LOCK = key(X_OUT + 0.034 - KEY_L, YC + DOG_Y - 0.040, TOP + LOCK_Z)  # blade between the dogs, bolt thrown
ON_TABLE = key(0.20, 0.10, TOP + KEY_H / 2)
ON_SLOT_FLOOR = key(X_OUT - 0.049, YC, TOP + SLOT_Z0 + KEY_H / 2)  # rear 49 mm out, centre over the slot floor
ON_FLOOR = key(0.20, 0.10, KEY_H / 2)
THROWN = door(BOLT_TRAVEL)

CASES = [
    ("start", {}, 0.0, False),
    ("bolt pushed 10 mm (key never used): impossible, scored as unlocked", {"door": door(0.010)}, 0.0, False),
    ("door locked, key put back on its block", {"door": THROWN, "key": ON_BLOCK}, 0.5, False),
    ("door locked at 35 mm (limit), key on its block", {"door": door(0.035), "key": ON_BLOCK}, 0.5, False),
    ("bolt at 34 mm (not locked), key on its block", {"door": door(0.034), "key": ON_BLOCK}, 0.0, False),
    ("door locked, key left in the lock", {"door": THROWN, "key": IN_LOCK}, 0.0, False),
    ("door locked, key dropped on the table", {"door": THROWN, "key": ON_TABLE}, 0.5, False),
    ("door locked, key lying in the slot, not through", {"door": THROWN, "key": ON_SLOT_FLOOR}, 0.5, False),
    ("door locked, key on the floor", {"door": THROWN, "key": ON_FLOOR}, 0.0, False),
    ("door locked, key posted, flap shut", {"door": THROWN, "key": POSTED}, 1.0, True),
    ("door locked at 35 mm, key posted", {"door": door(0.035), "key": POSTED}, 1.0, True),
    ("flap 5 mm open (limit), key posted, locked", {"door": door(BOLT_TRAVEL, FLAP_SHUT), "key": POSTED}, 1.0, True),
    ("flap held open, key posted, locked", {"door": door(BOLT_TRAVEL, FLAP_SHUT + 0.02), "key": POSTED}, 0.0, False),
    ("key posted, door unlocked (locked out)", {"key": POSTED}, 0.0, False),
    ("key posted, bolt at 34 mm", {"door": door(0.034), "key": POSTED}, 0.0, False),
    ("key posted but still sliding", {"door": THROWN, "key": key(X_IN + 0.030, YC, TOP + HALL_FLOOR + KEY_H / 2,
                                                                    vel=(0.05, 0.0, 0.0))}, 0.0, False),
    ("key on its block, nothing else done", {"key": ON_BLOCK}, 0.0, False),
    ("key leaning 2 mm in front of the inner face (limit)", {"door": THROWN, "key": key(X_IN - 0.002, YC, TOP + HALL_FLOOR + KEY_H / 2)}, 1.0, True),
    ("key 3 mm in front of the inner face (still in the slot)", {"door": THROWN, "key": key(X_IN - 0.003, YC, TOP + SLOT_Z0 + KEY_H / 2)}, 0.5, False),
    ("key just behind the inner face (limit)", {"door": THROWN, "key": key(X_OUT + DOOR_T, YC, TOP + HALL_FLOOR + KEY_H / 2)}, 1.0, True),
]
