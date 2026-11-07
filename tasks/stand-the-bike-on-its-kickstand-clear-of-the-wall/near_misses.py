"""Near-successes, habits and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state}, expected score, expected success). Cases run in order on one copy of the
task module; the lying-down cases come last because that record is kept for the rest of the trial.
"""

import math

import numpy as np

from task import (BIKE, BIKE_XY, DEPLOYED, LEAN0, SADDLE, SADDLE_AT, TOP, WHEEL_W, _foot_low, _lean_pose,
                  _quat_matrix, _rot_y)
from yam_arena.demo import FINGER_OPEN, grip_rotation


def pose(lean_deg, stand=DEPLOYED, dx=0.0, dz=0.0, vel=None):
    """The bike leaning ``lean_deg`` toward the robot (negative: toward the wall) on its tyres' edge on that side,
    shifted ``dx`` along x and ``dz`` up."""
    a = -math.radians(lean_deg)  # rotation about +y: positive tips the top toward the wall
    side = -1.0 if lean_deg > 0 else 1.0
    edge = np.array([BIKE_XY[0] + side * WHEEL_W / 2, BIKE_XY[1], TOP])
    origin = edge + _rot_y(a) @ (np.array([*BIKE_XY, TOP]) - edge) + [dx, 0.0, dz]
    q = (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))
    return BIKE.state(joints={"stand": stand}, pos=origin, quat_xyzw=q, lin_vel=vel)


def on_stand_lean():
    """The lean toward the robot at which the deployed stand's foot reaches the table."""
    lo, hi = 0.0, 30.0
    for _ in range(60):
        m = (lo + hi) / 2
        lo, hi = (m, hi) if _foot_low(pose(m)) > TOP else (lo, m)
    return hi


REST = on_stand_lean()
p0, q0 = _lean_pose(LEAN0)
START_STAND_DOWN = BIKE.state(joints={"stand": DEPLOYED}, pos=p0, quat_xyzw=q0)

# A right-arm gripper with open fingers closing along x, astride the saddle of the bike on its stand, its tips 14 mm
# below the saddle's top, slid toward the wall until its robot-side finger is ``gap`` from the bike.
def astride(gap):
    import task

    bike = pose(REST)
    top = np.asarray(bike.pos) + _quat_matrix(bike.quat_xyzw) @ np.array(
        [SADDLE_AT[0], SADDLE_AT[1], SADDLE_AT[2] + SADDLE[2] / 2])
    rot, half = grip_rotation(math.pi / 2), -FINGER_OPEN

    def grip(dx):
        return {"right_robot": (top + [dx, 0.0, 0.013 - 0.014], rot, half)}

    lo, hi = 0.0, 0.08  # the near finger meets the saddle's robot side as the hand slides toward the wall
    for _ in range(60):
        m = (lo + hi) / 2
        task._GRIP.clear()
        task._GRIP.update(grip(m))
        lo, hi = (m, hi) if task.hands_gap(bike) > gap else (lo, m)
    task._GRIP.clear()
    return {"bike": bike, "_grip": grip(lo)}


# Cases with a gripper beside the bike. factory/check.py cannot place grippers, so only this file's own run scores
# them (``python near_misses.py``).
GRIP_CASES = [
    ("on its stand, a finger 4 mm from the saddle (not released)", astride(0.004), 0.0, False),
    ("on its stand, every finger 6 mm clear (released)", astride(0.006), 1.0, True),
]

CASES = [
    ("on its stand, leaning toward the robot, clear of the wall", {"bike": pose(REST)}, 1.0, True),
    ("on its stand but pushed 5 cm toward the wall (2 cm clear)", {"bike": pose(REST, dx=0.050)}, 0.0, False),
    ("on its stand, still moving", {"bike": pose(REST, vel=(0.0, 0.0, -0.05))}, 0.0, False),
    ("held 1 cm up in the stand's pose", {"bike": pose(REST, dz=0.010)}, 0.0, False),
    ("habit, stand kicked down but the bike left leaning on the wall", {"bike": START_STAND_DOWN}, 0.0, False),
    ("habit, pulled upright with the stand folded", {"bike": pose(0.0, stand=0.0)}, 0.0, False),
    ("habit, pulled upright with the stand down (foot clear of the table)", {"bike": pose(0.0)}, 0.0, False),
    ("habit, stand half down (40 degrees), bike upright", {"bike": pose(0.0, stand=math.radians(40))}, 0.0, False),
    ("leaning 30 degrees toward the robot, stand folded (not yet down)", {"bike": pose(30.0, stand=0.0)}, 0.0,
     False),
    ("then stood on its stand: 30 degrees is not lying down", {"bike": pose(REST)}, 1.0, True),
    ("habit, pulled over with the stand folded: lying on its side", {"bike": pose(80.0, stand=0.0)}, 0.0, False),
    ("lay down, then stood on its stand (the fall still counts)", {"bike": pose(REST)}, 0.0, False),
]


def _run() -> int:
    import task

    start = {o.name: o.state() if hasattr(o, "state") else None for o in task.OBJECTS}
    from yam_arena.task_file import ObjectState

    w = task.WALL
    start["wall"] = ObjectState(np.asarray(w.pos), np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3),
                                np.asarray(w.pos) - np.asarray(w.size) / 2, np.asarray(w.pos) + np.asarray(w.size) / 2)
    bad = 0
    for label, changes, want, want_success in GRIP_CASES + CASES:
        changes = dict(changes)
        task._GRIP.clear()
        task._GRIP.update(changes.pop("_grip", {}))
        st = dict(start, **changes)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    task._GRIP.clear()
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
