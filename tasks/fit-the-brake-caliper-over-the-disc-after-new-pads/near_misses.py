"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state or (centre, size or None, velocity or None[, quat_xyzw])}, expected score,
expected success). factory/check.py cannot place grippers, so every state in CASES counts as released; GRIP_CASES,
scored only by this file's own run (``python near_misses.py``), put a gripper beside the fitted caliper.

Fixed habits for this layout and what defeats each:
- "pick the part up and set it on its target" (the obvious move): with new pads the gap is 8 mm narrower than the
  disc, so the caliper perches on the rim: 0.
- "turn it to look tidy / either way round": the caliper must keep its piston on the robot's side: turned 180 deg, 0.
- "push the piston back and set the caliper down anywhere" (e.g. pinching the near leg with one finger in the pad
  gap): no credit unless it is fitted over the disc.
"""

import math

import numpy as np

from yam_arena.demo import grip_rotation

from task import BOSS, BOSS_AT, CAL_XY, CAL_Z0, CALIPER, DISC_R, DISC_T, DISC_TOP, DISC_XY, PAD, PAD_Z, TOP, TRAVEL


def q_yaw(deg):
    return (0.0, 0.0, math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2))


def q_about(axis, deg):
    a = math.radians(deg) / 2
    return (*(v * math.sin(a) for v in axis), math.cos(a))


def cal(x=DISC_XY[0], y=DISC_XY[1], z=DISC_TOP, piston=TRAVEL, quat=None, vel=None):
    return {"caliper": CALIPER.state(joints={"piston": piston}, pos=(x, y, z), quat_xyzw=quat, lin_vel=vel)}


# Perched: new pads (20 mm gap) on a 28 mm disc rest on the rim with their bottoms on the disc's top.
PERCH_Z = DISC_TOP - PAD_Z + PAD[2] / 2
BENCH = (*CAL_XY, CAL_Z0)

CASES = [
    ("fitted on the bracket, piston fully back", cal(), 1.0, True),
    ("fitted, piston back 8 mm (just enough to pass)", cal(piston=0.008), 1.0, True),
    ("fitted, ears 2 mm above the abutments (limit)", cal(z=DISC_TOP + 0.002), 1.0, True),
    ("ears 3 mm above the abutments (held up)", cal(z=DISC_TOP + 0.003), 0.0, False),
    ("fitted 4 mm off centre along the disc axis (limit)", cal(x=DISC_XY[0] + 0.004), 1.0, True),
    ("fitted but tipped 6 deg", cal(quat=q_about((1, 0, 0), 6)), 0.0, False),
    ("fitted, still moving", cal(vel=(0.0, 0.0, -0.05)), 0.0, False),
    ("habit: set straight on the disc with new pads: perched on the rim", cal(z=PERCH_Z, piston=0.0), 0.0, False),
    ("perched on the rim, piston pushed back 4 mm", cal(z=PERCH_Z, piston=0.004), 0.0, False),
    ("habit: fitted turned 180 deg (piston on the far side)", cal(quat=q_yaw(180)), 0.0, False),
    ("exploit: piston pinched fully back, caliper set back down on the bench", {"caliper": CALIPER.state(
        joints={"piston": TRAVEL}, pos=BENCH)}, 0.0, False),
    ("piston pushed back fully, caliper set down on the bench 10 cm from the disc", {"caliper": CALIPER.state(
        joints={"piston": TRAVEL}, pos=(CAL_XY[0], CAL_XY[1] + 0.10, CAL_Z0))}, 0.0, False),
    ("piston pushed back fully, caliper set level on top of the bracket's abutments, 2 cm off the disc", cal(
        x=DISC_XY[0] + 0.020), 0.0, False),
    ("untouched on the bench", {"caliper": CALIPER.state(pos=BENCH)}, 0.0, False),
    ("piston back, caliper held 5 mm above the bench", {"caliper": CALIPER.state(
        joints={"piston": TRAVEL}, pos=(*CAL_XY, CAL_Z0 + 0.005))}, 0.0, False),
    ("piston back, caliper lying on its side on the bench", {"caliper": CALIPER.state(
        joints={"piston": TRAVEL}, pos=(*CAL_XY, TOP + 0.056), quat_xyzw=q_about((1, 0, 0), 90))}, 0.0, False),
    ("knocked off the table", {"caliper": CALIPER.state(pos=(0.3, -0.5, TOP - 0.3))}, 0.0, False),
]
_ = (DISC_R, DISC_T)


def boss_pinch(gap):
    """The fitted caliper with a closed right gripper over its boss (fingers closing along y), lowered until its
    lowest finger point is ``gap`` above the boss's top."""
    import task

    st = cal()["caliper"]
    boss_top = np.array([DISC_XY[0] + BOSS_AT[0], DISC_XY[1] + BOSS_AT[1], DISC_TOP + BOSS_AT[2] + BOSS[2] / 2])
    rot, half = grip_rotation(0.0), 0.0

    def grip(dz):
        return {"right_robot": (boss_top + [0.0, 0.0, dz], rot, half)}

    lo, hi = 0.0, 0.06
    for _ in range(60):
        m = (lo + hi) / 2
        task._GRIP.clear()
        task._GRIP.update(grip(m))
        lo, hi = (lo, m) if task.hands_gap(st) > gap else (m, hi)
    task._GRIP.clear()
    return {"caliper": st, "_grip": grip(hi)}


GRIP_CASES = [
    ("fitted, a gripper's finger 4 mm from the caliper (not released)", boss_pinch(0.004), 0.0, False),
    ("fitted, every finger 6 mm clear (released)", boss_pinch(0.006), 1.0, True),
]


def _run() -> int:
    import task

    bad = 0
    for label, changes, want, want_success in GRIP_CASES + CASES:
        changes = dict(changes)
        task._GRIP.clear()
        task._GRIP.update(changes.pop("_grip", {}))
        st = {o.name: o.state() for o in (task.BRAKE, task.CALIPER)}
        st.update(changes)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    task._GRIP.clear()
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
