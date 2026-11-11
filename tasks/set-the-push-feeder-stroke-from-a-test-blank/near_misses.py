"""Near-successes and unintended strategies, scored offline by the factory checker (lot A).

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw]) or a ready state}, expected score,
expected success). Arm checks (the light curtain, letting go) need the simulator; _run() also tests the records.
"""

import math

from task import BLANK, FLOOR_H, MAIN_Y, NEST, ROOF_T, TEST_Y, TOP, X0, LANE_L, INNER_W, WALL_H

Z = TOP + FLOOR_H + BLANK[2] / 2  # lying on the lane floor
NEAR, FAR = X0 + NEST[0], X0 + NEST[1]
HALF = BLANK[0] / 2
MOVING = (0.05, 0.0, 0.0)


def yawed(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


CASES = [
    ("centred in the nest", {"blank": (((NEAR + FAR) / 2, MAIN_Y, Z), None, None)}, 1.0, True),
    ("near end exactly on the nest's near edge", {"blank": ((NEAR + HALF, MAIN_Y, Z), None, None)}, 1.0, True),
    ("far end exactly on the nest's far edge", {"blank": ((FAR - HALF, MAIN_Y, Z), None, None)}, 1.0, True),
    ("1 mm short: near end over the grey floor", {"blank": ((NEAR + HALF - 0.001, MAIN_Y, Z), None, None)}, 0.0, False),
    ("1 mm past: far end over the grey floor", {"blank": ((FAR - HALF + 0.001, MAIN_Y, Z), None, None)}, 0.0, False),
    ("in the nest but still sliding", {"blank": (((NEAR + FAR) / 2, MAIN_Y, Z), None, MOVING)}, 0.0, False),
    ("in the nest, turned 8 degrees in the channel", {"blank": (((NEAR + FAR) / 2, MAIN_Y, Z), None, None, yawed(8))},
     1.0, True),
    ("turned 8 degrees with its centre at the edge limit: corners over the edge",
     {"blank": ((FAR - HALF, MAIN_Y, Z), None, None, yawed(8))}, 0.0, False),
    ("stopped short under the guard", {"blank": ((X0 + 0.20, MAIN_Y, Z), None, None)}, 0.0, False),
    ("stopped between the guard and the nest", {"blank": ((NEAR - 0.01, MAIN_Y, Z), None, None)}, 0.0, False),
    ("past the nest, still on the lane end", {"blank": ((X0 + LANE_L - 0.01, MAIN_Y, Z), None, None)}, 0.0, False),
    ("in the scrap bin", {"blank": ((X0 + LANE_L + 0.04, MAIN_Y, TOP + 0.003 + BLANK[2] / 2), None, None)}, 0.0, False),
    ("over the nest but 3 mm up (riding on something)", {"blank": (((NEAR + FAR) / 2, MAIN_Y, Z + 0.003), None, None)},
     0.0, False),
    ("standing on end in the nest", {"blank": (((NEAR + FAR) / 2, MAIN_Y, TOP + FLOOR_H + BLANK[0] / 2), None, None,
                                               (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)))}, 0.0, False),
    ("the test blank in the test slide's nest, press blank unmoved",
     {"test_blank": (((NEAR + FAR) / 2, TEST_Y, Z), None, None)}, 0.0, False),
    ("test blank moved into the press lane's nest, press blank unmoved",
     {"test_blank": (((NEAR + FAR) / 2, MAIN_Y, Z), None, None)}, 0.0, False),
    ("press blank lifted out and set on the test slide's nest",
     {"blank": (((NEAR + FAR) / 2, TEST_Y, Z), None, None)}, 0.0, False),
    ("press blank lying on the slotted roof over the nest",
     {"blank": (((NEAR + FAR) / 2, MAIN_Y, TOP + FLOOR_H + WALL_H + ROOF_T + BLANK[2] / 2), None, None)},
     0.0, False),
]


def _run() -> int:
    """Score every case with task.py's score() and success(), then check the arm records."""
    import numpy as np

    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, quat=None):
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        r = task._rot(q)
        half = np.abs(r) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: o.size for o in task.OBJECTS}
    start = {o.name: o.state() for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, (pos, size, vel, *quat) in changes.items():
            st[name] = state(pos, sizes[name] if size is None else size, vel, quat[0] if quat else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    good = dict(start)
    good["blank"] = state(((NEAR + FAR) / 2, MAIN_Y, Z), sizes["blank"])
    for label, rec in (("in the nest, but an arm crossed the light curtain", {"crossed": True}),
                       ("in the nest, but a gripper within 5 mm of the blank", {"clear": False})):
        saved = dict(task._record)
        task._record.update(rec)
        got = float(task.score(good))
        task._record.update(saved)
        flag = "ok  " if got == 0.0 else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
