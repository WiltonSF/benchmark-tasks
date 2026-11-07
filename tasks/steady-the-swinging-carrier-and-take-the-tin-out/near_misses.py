"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw]) or a ready state}, expected score,
expected success). Offline there are no simulator steps, so the carrier's swing is its angle in the case.
Cases run in order against one copy of the task, and the whole-trial records are never cleared offline, so every
case that trips a record comes last.
"""

import math

import numpy as np

from task import (BX, BY, CARRIER, FLOOR_TOP, PAD_H, PAD_XY, PIVOT_X, PIVOT_Z, RIM_Z, TIN_H, TIN_R, TOP, _rot_y)

D = math.radians


def swing(deg):
    return CARRIER.state(joints={"swing": D(deg)})


def tilt_y(deg):
    a = D(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def on_pad(dx=0.0, dz=0.0):
    return (PAD_XY[0] + dx, PAD_XY[1], TOP + PAD_H + TIN_H / 2 + dz)


def leaning_on_pad(deg):
    """Tilted about y, its lowest edge on the pad."""
    a = D(deg)
    dz = math.cos(a) * TIN_H / 2 + math.sin(a) * TIN_R - TIN_H / 2
    return (on_pad(dz=dz), None, None, tilt_y(deg))


def in_basket(deg, lift=0.0):
    """The tin standing on the basket floor (raised by ``lift`` along the basket's vertical) with the carrier at
    ``deg``; its quaternion tilts with the basket."""
    pivot = np.array([BX + PIVOT_X, BY, TOP + PIVOT_Z])
    p = np.array([BX, BY, TOP + FLOOR_TOP + TIN_H / 2 + lift]) - pivot
    return (tuple(pivot + _rot_y(D(deg)) @ p), None, None, tilt_y(deg))


MOVING = (0.0, 0.0, -0.05)
lift_out = RIM_Z - FLOOR_TOP + 0.010  # base 10 mm above the rim

CASES = [
    ("tin on the pad, carrier hanging still", {"tin": (on_pad(), None, None), "carrier": swing(0.0)}, 1.0, True),
    ("tin on the pad, carrier at 1.9 deg", {"tin": (on_pad(), None, None), "carrier": swing(1.9)}, 1.0, True),
    ("tin on the pad, carrier at 2.0 deg: still swinging", {"tin": (on_pad(), None, None), "carrier": swing(2.0)},
     0.5, False),
    ("tin on the pad, carrier swinging 8 deg", {"tin": (on_pad(), None, None), "carrier": swing(-8.0)}, 0.5, False),
    ("tin 25 mm off the pad centre", {"tin": (on_pad(dx=0.025), None, None)}, 1.0, True),
    ("tin 26 mm off the pad centre", {"tin": (on_pad(dx=0.026), None, None)}, 0.0, False),
    ("tin leaning 10 deg on the pad", {"tin": leaning_on_pad(10.0)}, 1.0, True),
    ("tin leaning 11 deg on the pad", {"tin": leaning_on_pad(11.0)}, 0.0, False),
    ("tin held 2 mm above the pad", {"tin": (on_pad(dz=0.002), None, None)}, 1.0, True),
    ("tin held 3 mm above the pad", {"tin": (on_pad(dz=0.003), None, None)}, 0.0, False),
    ("tin on the pad, still moving", {"tin": (on_pad(), None, MOVING)}, 0.0, False),
    ("tin still in the basket, carrier steadied", {"carrier": swing(0.0)}, 0.0, False),
    ("tin in the basket, carrier at the end of its swing", {"carrier": swing(11.0), "tin": in_basket(11.0)},
     0.0, False),
    ("tin lifted clear of the rim, carrier steady", {"carrier": swing(1.0), "tin": in_basket(1.0, lift_out)},
     0.0, False),
    ("tin leaning 19 deg against the basket wall", {"carrier": swing(5.0), "tin": in_basket(5.0)[:3] + (tilt_y(24.0),)},
     0.0, False),
    ("then on the pad, carrier still: nothing tripped", {"tin": (on_pad(), None, None)}, 1.0, True),
    # From here on each case trips a whole-trial record, so every later case scores 0 as well.
    ("tin plucked out over the rim while the carrier swings 5 deg",
     {"carrier": swing(5.0), "tin": in_basket(5.0, lift_out)}, 0.0, False),
    ("... then set on the pad with the carrier still", {"tin": (on_pad(), None, None)}, 0.0, False),
    ("tin tipped 25 deg in the basket", {"carrier": swing(0.0), "tin": in_basket(0.0)[:3] + (tilt_y(25.0),)},
     0.0, False),
    ("tin dropped on the table beside the pad",
     {"tin": ((PAD_XY[0] + 0.08, PAD_XY[1], TOP + TIN_H / 2), None, None)}, 0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(), as the factory checker does."""
    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, quat=None):
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        half = np.abs(task._quat_matrix(q)) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    sizes = {"tin": (2 * task.TIN_R, 2 * task.TIN_R, task.TIN_H), "pad": (2 * task.PAD_R, 2 * task.PAD_R, task.PAD_H)}
    start = {"carrier": CARRIER.state(), "tin": state(task.TIN.pos, sizes["tin"]), "pad": state(task.PAD.pos, sizes["pad"])}
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
