"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A.
Each case: (label, {object: ready state}, expected score, expected success). States come from the parts' own
geometry (VALVE.state(joints=...), PIPE.state(pos=..., quat_xyzw=...)).

Offline there is no robot, so the release test (no gripper within 5 mm of handle or tube) always passes here; in the
simulator an on_step callback measures it every step and a held handle or tube scores 0.
"""

import math

from task import DECK_TOP, MOUTH_STOP, P_LEN, PIPE, PIPE_A, VALVE, VX, VY

PA = PIPE_A


def yaw(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def handle(deg):
    return VALVE.state(joints={"handle": math.radians(deg)})


def tube_on_handle(deg, vel=None):
    """The tube slid over the handle up to the hub, the handle turned `deg` from shut."""
    a = math.radians(deg)
    r = MOUTH_STOP + P_LEN / 2
    return PIPE.state(pos=(VX - r * math.cos(a), VY - r * math.sin(a), DECK_TOP), quat_xyzw=yaw(a), lin_vel=vel)


def case(deg, tube=None):
    return {"valve": handle(deg), "tube": tube_on_handle(deg) if tube is None else tube}


MOVING = (0.0, -0.05, 0.0)

CASES = [
    ("open: at the stop, tube still on the handle", case(90.0), 1.0, True),
    ("open to exactly 85 degrees (5 degrees short)", case(85.0), 1.0, True),
    ("84.9 degrees: half open tier", case(84.9), 0.5, False),
    ("turned exactly 45 degrees", case(45.0), 0.5, False),
    ("turned 44.9 degrees", case(44.9), 0.0, False),
    ("turned 20 degrees", case(20.0), 0.0, False),
    ("bare handle pushed hard: 3 degrees of give, tube on the deck",
     {"valve": handle(3.0), "tube": PIPE.state(pos=PA)}, 0.0, False),
    ("tube slid over the shut handle, not turned", case(0.0), 0.0, False),
    ("open, but the tube still moving", case(90.0, tube_on_handle(90.0, MOVING)), 0.0, False),
    ("half open, tube still moving", case(60.0, tube_on_handle(60.0, MOVING)), 0.0, False),
    ("open, tube taken off and laid back on the deck", {"valve": handle(90.0), "tube": PIPE.state(pos=PA)}, 1.0, True),
    ("nothing done", {}, 0.0, False),
    ("tube moved across the deck, valve shut",
     {"tube": PIPE.state(pos=(0.20, -0.30, DECK_TOP), quat_xyzw=yaw(math.pi / 2))}, 0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(), from the start states; print and count mismatches."""
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
