"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw]) or a ready state}, expected score,
expected success).

Fixed habits checked (each scores 0): put the bar in the obvious gap against the nearest stop (the flip stop);
slide it endwise along the fence; lay it over whatever is in the way; keep clear of the fixture's moving parts and load
it beside them, past the flip stop's reach; leave the bar where it is.
"""

import math

from task import BAR, END_X, FENCE_Y, FIXTURE, FLIP_X, TOP

L, W, H = BAR
UP = FIXTURE.state(joints={"flip": math.radians(135)})
HALF_UP = FIXTURE.state(joints={"flip": math.radians(100)})
EDGE = FIXTURE.state(joints={"flip": math.radians(125)})
JUST_UNDER = FIXTURE.state(joints={"flip": math.radians(124.9)})


def bar(end_gap=0.0, fence_gap=0.0, z=0.0):
    """The bar lying flat along the fence: end face end_gap short of the end stop, side fence_gap off the fence."""
    return (END_X - end_gap - L / 2, FENCE_Y - fence_gap - W / 2, TOP + H / 2 + z)


def yawq(deg):
    return (0.0, 0.0, math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2))


def tiltq(deg, axis="x"):
    s, c = math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2)
    return (s, 0.0, 0.0, c) if axis == "x" else (0.0, s, 0.0, c)


MOVING = (0.0, 0.0, -0.05)

CASES = [
    ("habit: do nothing", {}, 0.0, False),
    ("habit: slide endwise along the fence into the flip stop",
     {"long_bar": ((FLIP_X[0] - L / 2, FENCE_Y - W / 2 - 0.001, TOP + H / 2), None, None)}, 0.0, False),
    ("loaded, stop up", {"fixture": UP, "long_bar": (bar(), None, None)}, 1.0, True),
    ("loaded, end gap exactly 1 mm, fence gap exactly 1.5 mm",
     {"fixture": UP, "long_bar": (bar(0.001, 0.0015), None, None)}, 1.0, True),
    ("loaded, turned end for end (yaw 180)", {"fixture": UP, "long_bar": (bar(), None, None, yawq(180))}, 1.0, True),
    ("loaded, flipped upside down", {"fixture": UP, "long_bar": (bar(), None, None, tiltq(180))}, 1.0, True),
    ("end gap 1.1 mm", {"fixture": UP, "long_bar": (bar(0.0011), None, None)}, 0.5, False),
    ("end gap 5 mm", {"fixture": UP, "long_bar": (bar(0.005), None, None)}, 0.5, False),
    ("end gap 5.1 mm", {"fixture": UP, "long_bar": (bar(0.0051), None, None)}, 0.0, False),
    ("fence gap 1.6 mm", {"fixture": UP, "long_bar": (bar(0.0, 0.0016), None, None)}, 0.0, False),
    ("skewed: far end on the fence, near end 3 mm off",
     {"fixture": UP, "long_bar": (bar(0.0, 0.0015), None, None, yawq(-math.degrees(0.003 / L)))}, 0.0, False),
    ("skewed 0.4 deg: both ends within 1.5 mm",
     {"fixture": UP, "long_bar": ((END_X - L / 2 - 0.0002, FENCE_Y - W / 2 - 0.0007, TOP + H / 2), None, None,
                                  yawq(-0.4))}, 1.0, True),
    ("loaded but still moving", {"fixture": UP, "long_bar": (bar(), None, MOVING)}, 0.0, False),
    ("loaded but 1.5 mm above the table (held)", {"fixture": UP, "long_bar": (bar(z=0.0015), None, None)}, 0.0,
     False),
    ("bar tilted 3 deg about its long axis", {"fixture": UP, "long_bar": (bar(z=0.0005), None, None, tiltq(3))},
     0.0, False),
    ("bar on its 25 mm edge against the fence and end stop",
     {"fixture": UP, "long_bar": ((END_X - L / 2, FENCE_Y - H / 2, TOP + W / 2), None, None, tiltq(90))}, 0.0,
     False),
    ("stop up, bar still at its start", {"fixture": UP}, 0.0, False),
    ("stop exactly at 125 deg (held by gravity), bar loaded", {"fixture": EDGE, "long_bar": (bar(), None, None)}, 1.0, True),
    ("stop at 124.9 deg, bar loaded", {"fixture": JUST_UNDER, "long_bar": (bar(), None, None)}, 0.0, False),
    ("bar loaded, stop lowered back onto the bar (40 deg)",
     {"fixture": FIXTURE.state(joints={"flip": math.radians(40)}), "long_bar": (bar(), None, None)}, 0.0, False),
    ("end gap 5 mm, stop lowered onto the bar",
     {"fixture": FIXTURE.state(joints={"flip": math.radians(40)}), "long_bar": (bar(0.005), None, None)}, 0.0, False),
    ("stop up only (no partial credit)", {"fixture": UP}, 0.0, False),
    ("stop at 124.9 deg", {"fixture": JUST_UNDER}, 0.0, False),
    ("stop held at 100 deg, bar at its start", {"fixture": HALF_UP}, 0.0, False),
    ("stop up, bar fallen to the floor", {"fixture": UP, "long_bar": ((0.3, -0.15, 0.10), None, None)}, 0.0, False),
    ("habit: nearest stop: bar against the flip stop (short-part position)",
     {"long_bar": ((FLIP_X[0] - L / 2, FENCE_Y - W / 2, TOP + H / 2), None, None)}, 0.0, False),
    ("habit: lay it over the obstruction: bar on the flip stop, far end on the table",
     {"long_bar": ((END_X - L / 2, FENCE_Y - W / 2, TOP + H / 2 + 0.012), None, None, tiltq(8, "y"))}, 0.0, False),
    ("habit: avoid the mechanism: bar 46 mm off the fence, past the stop, at the end stop",
     {"long_bar": (bar(0.0, 0.046), None, None)}, 0.0, False),
    ("bar behind the fence against its back face", {"fixture": UP,
     "long_bar": ((END_X - L / 2, FENCE_Y + 0.020 + W / 2, TOP + H / 2), None, None)}, 0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import numpy as np

    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, quat=None):
        """As the simulator reports it: a rotated object's world-aligned bounding box is wider than the object."""
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        x, y, z, w = q
        r = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        half = np.abs(r) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: o.size for o in task.OBJECTS}
    start = {o.name: (o.state() if hasattr(o, 'state') else state(o.pos, o.size)) for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, spec in changes.items():
            if isinstance(spec, ObjectState):
                st[name] = spec
                continue
            pos, size, vel, *quat = spec
            st[name] = state(pos, sizes[name] if size is None else size, vel, quat[0] if quat else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
