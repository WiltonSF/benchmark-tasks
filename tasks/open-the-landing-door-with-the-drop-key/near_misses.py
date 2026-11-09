"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py). Each case:
(label, {object: spec}, expected score, expected success); a spec is a ready state from the part's own geometry.
Cases run in order and share the whole-trial record, so a case can build on the ones before it; a scene reset is an
empty case after the key was moved away. A gripper reaching behind the wall, a held door and a moving door are
measured from the robot's links and the door's motion in the simulator and cannot be posed here.
"""

import math

from task import DOOR, KEY, LATCH, TOP


def door(q):
    return {"landing_door": DOOR.state(joints={"door": q})}


def latch(deg):
    return {"latch": LATCH.state(joints={"bar": math.radians(deg)})}


KEY_IN = {"drop_key": KEY.state(pos=(0.20, 0.12, TOP + 0.055))}  # rod through the slot, hook behind the wall


def key_at(x):
    """The key lying on the table at its start's y, its grip block's centre at x (the hook's far face at x + 0.21)."""
    return {"drop_key": KEY.state(pos=(x, 0.22, TOP))}


SCENE_RESET = {}

CASES = [
    ("door closed, latch down", {}, 0.0, False),
    ("latch lifted 30 degrees, door closed", {**latch(30), **KEY_IN}, 0.0, False),
    ("door pulled 2 mm against the latch", {**door(0.002), **KEY_IN}, 0.0, False),
    ("door 9 mm open", {**door(0.009), **KEY_IN}, 0.0, False),
    ("door 10 mm open", {**door(0.010), **KEY_IN}, 0.5, False),
    ("door 59 mm open, key back on the table", {**door(0.059), **key_at(0.12)}, 0.5, False),
    ("door 60 mm open, key still through the slot", {**door(0.060), **KEY_IN}, 0.5, False),
    ("door 60 mm open, key back on the table", {**door(0.060), **key_at(0.12)}, 1.0, True),
    ("door fully open, key tip exactly at the wall's face", {**door(0.100), **key_at(0.15)}, 1.0, True),
    ("door fully open, key tip 1 mm into the slot", {**door(0.100), **key_at(0.151)}, 0.5, False),
    ("door open 100 mm with the latch still held up by the key", {**door(0.100), **latch(30), **KEY_IN}, 0.5, False),
    ("door closed again at the end", {**door(0.0), **key_at(0.12)}, 0.0, False),
    ("key moved away", key_at(0.10), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("after the reset, door 60 mm, key on the table: 1.0", {**door(0.060), **key_at(0.12)}, 1.0, True),
]

if __name__ == "__main__":
    import task

    start = {o.name: o.state() for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_ok in CASES:
        st = dict(start, **changes)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "" if (got == want and ok == want_ok) else "   <-- MISMATCH"
        bad += bool(flag)
        print(f"{got:4.2f}  {label}{flag}")
    raise SystemExit(1 if bad else 0)
