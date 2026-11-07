"""Near-successes, fixed habits and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state}, expected score, expected success). Every object is a parts.py Part, so each
case gives its state from Part.state(pos=..., quat_xyzw=...): pos is the part's origin (the bolt's shank end, the
gasket's underside centre). Offline there is no robot, so the release check passes.
"""

import math

from task import BOLTS, CX, CY, HOLES, LOW_H, RING, RING_START_X, SEATED_Z, TOP


def tilt_q(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def ring(dx=0.0, dy=0.0, dz=0.0, tilt=0.0, vel=None):
    return RING.state(pos=(CX + dx, CY + dy, TOP + LOW_H + dz), quat_xyzw=tilt_q(tilt), lin_vel=vel)


RING_OUT_FRONT = RING.state()  # never moved: still on the block in front


def bolt(i, hole, dx=0.0, dz=0.0, tilt=0.0):
    hx, hy = HOLES[hole]
    return BOLTS[i].state(pos=(CX + hx + dx, CY + hy, TOP + SEATED_Z + dz), quat_xyzw=tilt_q(tilt))


def four(**kw):
    return {"bolt_1": bolt(0, "far_left"), "bolt_2": bolt(1, "near_left"), "bolt_3": bolt(2, "far_right"),
            "bolt_4": bolt(3, "near_right"), **kw}


def far_only(**kw):
    return {"bolt_1": bolt(0, "far_left"), "bolt_3": bolt(2, "far_right"), **kw}


CASES = [
    ("gasket centred, four bolts in", four(gasket=ring()), 1.0, True),
    ("gasket 4.0 mm off the axis", four(gasket=ring(dx=-0.004)), 1.0, True),
    ("gasket 4.1 mm off the axis", four(gasket=ring(dx=-0.0029, dy=0.0029)), 0.0, False),
    ("gasket 1.5 mm up (riding on something)", four(gasket=ring(dz=0.0015)), 1.0, True),
    ("gasket 2 mm up", four(gasket=ring(dz=0.002)), 0.0, False),
    ("gasket tilted 2.9 deg", four(gasket=ring(tilt=2.9)), 1.0, True),
    ("gasket tilted 3.1 deg", four(gasket=ring(tilt=3.1)), 0.0, False),
    ("gasket still moving", four(gasket=ring(vel=(0.02, 0.0, 0.0))), 0.0, False),
    ("halfway: gasket centred against the two far bolts, near holes empty", far_only(gasket=ring(dx=0.001)), 0.5, False),
    ("gasket centred, far bolts and one near bolt in", {**far_only(gasket=ring()), "bolt_2": bolt(1, "near_left")},
     0.5, False),
    # Fixed habits.
    ("habit: all four bolts first, gasket left on its block", four(gasket=RING_OUT_FRONT), 0.0, False),
    ("habit: nearest holes first, gasket stopped against them short of the axis",
     {"bolt_2": bolt(1, "near_left"), "bolt_4": bolt(3, "near_right"), "gasket": ring(dx=-0.040)}, 0.0, False),
    ("gasket pushed in and centred, no bolts", {"gasket": ring()}, 0.0, False),
    ("gasket centred with only the near bolts in", {"bolt_2": bolt(1, "near_left"), "bolt_4": bolt(3, "near_right"),
                                                     "gasket": ring()}, 0.0, False),
    ("gasket pushed too far, 10 mm past the axis, four bolts in", four(gasket=ring(dx=0.010)), 0.0, False),
    # Bolt limits.
    ("a bolt leaning 3 deg in its hole (head 2.5 mm off centre)", four(gasket=ring(), bolt_2=bolt(1, "near_left", tilt=3.0)), 1.0, True),
    ("a near bolt tilted 9 deg: not in, so the far-bolt tier", four(gasket=ring(), bolt_2=bolt(1, "near_left", tilt=9.0)), 0.5, False),
    ("a bolt head 2 mm above the flange", four(gasket=ring(), bolt_4=bolt(3, "near_right", dz=0.002)), 1.0, True),
    ("a near bolt head 2.5 mm up (resting on the gasket): the far-bolt tier",
     four(gasket=ring(), bolt_4=bolt(3, "near_right", dz=0.0025)), 0.5, False),
    ("a bolt 3 mm off its hole centre", four(gasket=ring(), bolt_1=bolt(0, "far_left", dx=0.003)), 1.0, True),
    ("a bolt 3.5 mm off its hole centre", four(gasket=ring(), bolt_1=bolt(0, "far_left", dx=0.0035)), 0.0, False),
    ("gasket left where it starts, nothing done", {"gasket": RING.state(pos=(CX + RING_START_X, CY, TOP + LOW_H))},
     0.0, False),
]


def _run() -> int:
    import task

    start = {o.name: o.state() for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = {**start, **changes}
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
