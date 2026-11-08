"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state or (centre, size or None, velocity or None[, quat_xyzw])}, expected score,
expected success). The drawer's cases are its parts.py state with its body (bottom centre) placed and turned.
"""

import math

from task import CY, DEPTH, DRAWER, FOOT_Y, RZ, STOP_PULL, TOP, WIDTH, XF

BACK = DEPTH / 2  # the drawer's back face behind its bottom centre


def q_about(axis, deg):
    a = math.radians(deg) / 2
    v = [0.0, 0.0, 0.0]
    v["xyz".index(axis)] = math.sin(a)
    return (*v, math.cos(a))


def drawer(back_s, z=0.0, deg=0.0, axis="y", vel=None, y=CY):
    """The drawer with its back face back_s in front of the chest face (negative: inside), bottom z above the table."""
    return {"drawer": DRAWER.state(pos=(XF + back_s + BACK, y, TOP + z), quat_xyzw=q_about(axis, deg), lin_vel=vel)}


CASES = [
    ("out, on the table, back 10 mm clear of the chest", drawer(0.010), 1.0, True),
    ("out, back exactly at the chest's face", drawer(0.0), 1.0, True),
    ("back 1 mm inside the face", drawer(-0.001), 0.0, False),
    ("set down beside the chest, its side exactly at the chest's left side", drawer(-0.10, y=FOOT_Y[1] + WIDTH / 2),
     1.0, True),
    ("set down beside the chest, 1 mm over the chest's footprint", drawer(-0.10, y=FOOT_Y[1] + WIDTH / 2 - 0.001),
     0.0, False),
    ("shut (start)", drawer(-DEPTH, RZ), 0.0, False),
    ("pulled straight to the stop, on the runners", drawer(-DEPTH + STOP_PULL, RZ), 0.0, False),
    ("pulled to the stop and lifted level", drawer(-DEPTH + STOP_PULL, RZ + 0.010), 0.0, False),
    ("out, on the table, turned 30 deg about the vertical", drawer(0.08, deg=30.0, axis="z"), 1.0, True),
    ("out but held 5 mm above the table", drawer(0.010, z=0.005), 0.0, False),
    ("out but still moving", drawer(0.010, vel=(0.05, 0.0, 0.0)), 0.0, False),
    ("out but tipped 8 deg (propped on its handle)", drawer(0.010, z=0.0, deg=-8.0), 0.0, False),
    ("out, lying on its side", drawer(0.03, z=DEPTH / 2, deg=90.0, axis="x"), 0.0, False),
    ("out, upside down", drawer(0.03, z=0.07, deg=180.0, axis="x"), 0.0, False),
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
