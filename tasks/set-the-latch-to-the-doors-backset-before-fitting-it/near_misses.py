"""Near-successes, fixed habits and unintended strategies, scored offline by the factory checker.

CASES are scored in lot A (the 70 mm door on the left), as factory/check.py does. Fixed habits checked there (each
scores 0): fit both latches as found (both at 60 mm: the left door needs 70); extend both (the right door needs 60);
extend the right one; seat a wrong-length latch drawn 10 mm proud so its hub lines up; drop a spindle onto a hub that
is off the bore. "Extend the left one" scores 1.0 in lot A; lot B (the doors swapped) is what stops it.

Habits across lots (habit_check, run on import, so check.py fails if it breaks): every fixed end state a policy could
produce without reading the doors (each latch at 60 or 70 mm, each spindle aimed 60 or 70 mm in, on either side) is
built as it would come to rest in lot A and in lot B and scored there; none may average more than 0.5. That covers
"extend the left one" (1.0 then 0) and "extend the right one" (0 then 1.0). The release check (no gripper within 5 mm)
runs only in the simulator and counts as met here.
"""

import itertools
import math

import lots
from task import (DOOR_T, DOOR_UP, DOORS, HUB, HUB_DZ, LATCHES, LONG, OBJECTS, SEAT_Z, SHORT, SPINDLE, SPINDLES, TOP,
                  X_EDGE)

PART = {o.name: o for o in OBJECTS}
YL, BL = DOORS["door_70"]  # lot A
YR, BR = DOORS["door_60"]


def latch(name, y, setting, proud=0.0, yaw_deg=0.0, vel=None):
    q = (0.0, 0.0, math.sin(math.radians(yaw_deg) / 2), math.cos(math.radians(yaw_deg) / 2))
    return PART[name].state(joints={"hub": setting - SHORT}, pos=(X_EDGE - proud, y, SEAT_Z), quat_xyzw=q, lin_vel=vel)


def spindle(name, x, y, bottom=TOP, tilt_deg=0.0):
    s, c = math.sin(math.radians(tilt_deg) / 2), math.cos(math.radians(tilt_deg) / 2)
    return PART[name].state(pos=(x, y, bottom + SPINDLE[2] / 2), quat_xyzw=(s, 0.0, 0.0, c))


def full(**over):
    st = {"latch_left": latch("latch_left", YL, LONG), "latch_right": latch("latch_right", YR, SHORT),
          "spindle_left": spindle("spindle_left", X_EDGE + BL, YL),
          "spindle_right": spindle("spindle_right", X_EDGE + BR, YR)}
    st.update(over)
    return st


ON_HUB = SEAT_Z + HUB_DZ + HUB / 2  # a spindle that cannot pass rests on the hub's top
CASES = [
    ("both doors done", full(), 1.0, True),
    ("latches swapped between doors (each set right)",
     full(latch_left=latch("latch_left", YR, SHORT), latch_right=latch("latch_right", YL, LONG)), 1.0, True),
    ("spindles swapped", full(spindle_left=spindle("spindle_left", X_EDGE + BR, YR),
                              spindle_right=spindle("spindle_right", X_EDGE + BL, YL)), 1.0, True),
    ("faceplate exactly 3 mm proud (hub 3 mm off)",
     full(latch_left=latch("latch_left", YL, LONG, proud=0.003)), 1.0, True),
    ("faceplate 3.1 mm proud", full(latch_left=latch("latch_left", YL, LONG, proud=0.0031)), 0.0, False),
    ("latch turned 3.5 deg in its bore", full(latch_left=latch("latch_left", YL, LONG, yaw_deg=3.5)), 0.0, False),
    ("both latches right, no spindles", {"latch_left": latch("latch_left", YL, LONG),
                                         "latch_right": latch("latch_right", YR, SHORT)}, 0.5, False),
    ("both latches right, one spindle", full(spindle_right=PART["spindle_right"].state()), 0.5, False),
    ("spindle tilted 6 deg", full(spindle_left=spindle("spindle_left", X_EDGE + BL, YL, tilt_deg=6)), 0.5, False),
    ("habit: fit as found (both at 60 mm), spindle resting on the off-bore hub",
     full(latch_left=latch("latch_left", YL, SHORT),
          spindle_left=spindle("spindle_left", X_EDGE + BL, YL, bottom=ON_HUB)), 0.0, False),
    ("habit: fit as found, no spindles", {"latch_left": latch("latch_left", YL, SHORT),
                                          "latch_right": latch("latch_right", YR, SHORT)}, 0.0, False),
    ("habit: extend the left one (lot A: right; lot B: both wrong, see habit_check)", full(), 1.0, True),
    ("habit: extend the right one, spindles on the hubs",
     full(latch_left=latch("latch_left", YL, SHORT), latch_right=latch("latch_right", YR, LONG),
          spindle_left=spindle("spindle_left", X_EDGE + BL, YL, bottom=ON_HUB),
          spindle_right=spindle("spindle_right", X_EDGE + BR, YR, bottom=ON_HUB)), 0.0, False),
    ("habit: extend both", {"latch_left": latch("latch_left", YL, LONG),
                            "latch_right": latch("latch_right", YR, LONG)}, 0.0, False),
    ("habit: wrong length drawn 10 mm proud to line the hub up",
     full(latch_left=latch("latch_left", YL, SHORT, proud=-(LONG - SHORT))), 0.0, False),
    ("habit: wrong length 10 mm proud the other way (60 door, 70 latch)",
     full(latch_right=latch("latch_right", YR, LONG, proud=LONG - SHORT)), 0.0, False),
    ("habit: do nothing", {}, 0.0, False),
    ("done but a latch still moving", full(latch_right=latch("latch_right", YR, SHORT, vel=(0.0, 0.0, -0.05))), 0.0,
     False),
    ("spindle beside the door on the table", full(spindle_left=spindle("spindle_left", X_EDGE - 0.05, YL)), 0.5, False),
]


DOOR_TOP = TOP + DOOR_UP + DOOR_T  # a spindle aimed off the face bore rests on the door's top


def _end_state(settings, aims, backset_at):
    """Where a fixed habit leaves things: latch on side y at settings[y], seated; the spindle on that side aimed
    aims[y] in from the edge. It passes only if the aim, the setting and that door's backset all agree; aimed at
    the face bore with the latch set wrong it rests on the hub; aimed anywhere else it stands on the door's top."""
    st = {}
    for (lname, y), sname in zip(LATCHES.items(), ("spindle_left", "spindle_right")):
        s, a, b = settings[y], aims[y], backset_at[y]
        st[lname] = latch(lname, y, s)
        bottom = TOP if abs(a - b) < 1e-9 and abs(s - b) < 1e-9 else ON_HUB if abs(a - b) < 1e-9 else DOOR_TOP
        st[sname] = spindle(sname, X_EDGE + a, y, bottom=bottom)
    return st


def habit_check():
    """Score every lot-blind end state in each lot. Returns [(label, [score per lot], mean)], worst first."""
    import task

    ys = list(LATCHES.values())
    out = []
    try:
        for set_l, set_r, aim_l, aim_r in itertools.product((SHORT, LONG), repeat=4):
            settings, aims = dict(zip(ys, (set_l, set_r))), dict(zip(ys, (aim_l, aim_r)))
            scores = []
            for lot in lots.all_lots():
                lots.use(lot.name)
                backset_at = {y: b for y, b in task.door_layout().values()}
                st = {o.name: o.state() for o in OBJECTS}
                st.update(_end_state(settings, aims, backset_at))
                scores.append(1.0 if task.success(st) else float(task.score(st)))
            label = (f"latches left {set_l * 1000:.0f} / right {set_r * 1000:.0f} mm, "
                     f"spindles aimed {aim_l * 1000:.0f} / {aim_r * 1000:.0f} mm")
            if (set_l, set_r) == (LONG, SHORT) and (aim_l, aim_r) == (LONG, SHORT):
                label = "extend the left one: " + label
            if (set_l, set_r) == (SHORT, LONG) and (aim_l, aim_r) == (SHORT, LONG):
                label = "extend the right one: " + label
            out.append((label, scores, sum(scores) / len(scores)))
    finally:
        lots.use(None)
    return sorted(out, key=lambda r: -r[2])


HABITS = habit_check()
if HABITS[0][2] > 0.5:
    raise AssertionError(f"a lot-blind habit averages {HABITS[0][2]:.2f} over the lots: {HABITS[0][0]}")


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
    print("habits across lots (score in lot A, lot B: mean; each must be 0.5 or less):")
    for label, scores, mean in HABITS:
        flag = "ok  " if mean <= 0.5 else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {', '.join(f'{v:.1f}' for v in scores)}: {mean:.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
