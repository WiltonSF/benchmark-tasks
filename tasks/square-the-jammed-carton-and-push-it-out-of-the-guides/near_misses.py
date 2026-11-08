"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). Turned or tipped cases give a quaternion, so the bounding box widens as the simulator reports it.
"""

import math

from task import CARTON, CY, SKEW, START_X, TOP, X0, XE, XS

Z = TOP + CARTON[2] / 2
HX = CARTON[0] / 2


def yawq(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def at(x, y=CY, z=Z, deg=0.0, vel=None):
    return {"carton": ((x, y, z), None, vel, yawq(deg))}


def lowx_at(low_x, deg=0.0):
    """Centre x giving this bounding-box near end for a carton turned deg."""
    a = math.radians(deg)
    half = HX * abs(math.cos(a)) + CARTON[1] / 2 * abs(math.sin(a))
    return low_x + half


START_DEG = math.degrees(SKEW)
SIDE = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))  # tipped 90 deg onto an end

CASES = [
    ("pushed out square, 10 mm past the far end", at(lowx_at(XE + 0.010)), 1.0, True),
    ("out exactly at the far end", at(lowx_at(XE)), 1.0, True),
    ("out, turned 2 deg", at(lowx_at(XE + 0.010, 2.0), deg=2.0), 1.0, True),
    ("out, turned 30 deg after leaving the guides", at(lowx_at(XE + 0.02, 30.0), deg=30.0), 1.0, True),
    ("1 mm short of the far end, square in the narrow part", at(lowx_at(XE - 0.001)), 0.5, False),
    ("square in the narrow part, 40 mm past the shoulder", at(XS + 0.040 - HX), 0.5, False),
    ("square in the narrow part, exactly 30 mm past the shoulder", at(XS + 0.030 - HX), 0.5, False),
    ("square, 29 mm past the shoulder", at(XS + 0.029 - HX), 0.0, False),
    ("still jammed at the shoulder (start pose)", at(START_X, deg=START_DEG), 0.0, False),
    ("pushed harder: jammed, turned 10 deg", at(START_X + 0.001, deg=10.0), 0.0, False),
    ("backed off and squared, still in the wide part", at(XS - 0.03 - HX), 0.0, False),
    ("out but still moving", at(lowx_at(XE + 0.010), vel=(0.05, 0.0, 0.0)), 0.0, False),
    ("out but held 5 mm up", at(lowx_at(XE + 0.010), z=Z + 0.005), 0.0, False),
    ("out but tipped onto its end", {"carton": ((XE + 0.04, CY, TOP + CARTON[0] / 2), None, None, SIDE)}, 0.0, False),
    ("out but tilted 8 deg (riding on something)",
     {"carton": ((lowx_at(XE + 0.02), CY, Z + 0.006), None, None,
                 (math.sin(math.radians(4)), 0.0, 0.0, math.cos(math.radians(4))))}, 0.0, False),
    ("out past the far end, centre exactly 6 cm to the left of the lane line", at(lowx_at(XE + 0.01), y=CY + 0.06),
     1.0, True),
    ("out past the far end but 7 cm to the side", at(lowx_at(XE + 0.01), y=CY + 0.07), 0.0, False),
    ("backed off until its near end is exactly at the guides' near end", at(X0 + HX), 0.0, False),
    ("then pushed out through the lane (path still clean)", at(lowx_at(XE + 0.010)), 1.0, True),
    # Around the guides, judged over the trial (these cases build on each other, as a trial would):
    ("around: backed out of the near end", at(0.10), 0.0, False),
    ("around: pushed back in line, square, inside the wide part", at(XS - 0.05 - HX), 0.0, False),
    ("around: then out past the far end in line", at(lowx_at(XE + 0.010)), 0.0, False),
    ("scene reset to the start", at(START_X, deg=START_DEG), 0.0, False),
    ("after the reset, pushed out through the lane", at(lowx_at(XE + 0.010)), 1.0, True),
    ("scene reset to the start again", at(START_X, deg=START_DEG), 0.0, False),
    ("around: pushed sideways beside the left guide, past its outer face", at(0.30, y=CY + 0.13), 0.0, False),
    ("around: then pushed forward beside it and back in line past the far end", at(lowx_at(XE + 0.010)), 0.0, False),
    ("scene reset to the start, last", at(START_X, deg=START_DEG), 0.0, False),
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
    start = {o.name: state(o.pos, o.size) for o in task.OBJECTS}
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
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
