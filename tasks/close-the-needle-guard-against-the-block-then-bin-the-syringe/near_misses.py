"""Near-successes, unintended strategies and fixed habits, scored offline by the factory checker (factory/check.py).

Each case: (label, {object: spec}, expected score, expected success). The syringe's spec is a ready state from its
own kinematics (SYRINGE.state(...)). The scorer keeps a whole-trial record, so cases run in order: a case that
closes the guard (well or badly) is followed by a scene reset before the next attempt. Offline there are no grippers,
so the free-hand rule (no gripper near the guard while it closes) is covered in the README and the trace, not here.
"""

import math

import numpy as np

import task  # noqa: E402
from task import (AXIS_Z, BARREL_R, FLANGE_HALF, BIN_C, BIN_FLOOR_TOP, BLOCK_X, BLOCK_Y, CATCH, GUARD_PIVOT, GUARD_T, SYR0,
                  SYRINGE, TOP)

OPEN, SHUT = 0.0, math.pi / 2
MOVING = (0.0, 0.0, -0.05)


def syr(guard=OPEN, x=SYR0[0], y=SYR0[1], z=AXIS_Z, vel=None):
    return {"syringe": SYRINGE.state(joints={"guard": guard}, pos=(x, y, z), lin_vel=vel)}


LIFTED = AXIS_Z + 0.015


def against_block(deg):
    """Partway through the demo's closing: the guard at deg, the block's corner on its outer half, 35 mm out."""
    x, y = task._syringe_at(min(deg, 90), 0.0)
    return syr(math.radians(deg), x=x, y=y, z=LIFTED)


def hinge_by_block(deg):
    """The guard at deg with its hinge 6 mm from the block's corner, so only its inner end is near the block."""
    x = BLOCK_X[1] - GUARD_PIVOT[0] + 0.004
    y = BLOCK_Y[0] - 0.006
    return syr(math.radians(deg), x=x, y=y, z=LIFTED)


def pads_on_guard(state, side=-1.0):
    """A free gripper opened wide, one finger pad pressed on the guard's outer half from the side it closes toward."""
    pts = task.guard_points(state, start=0.5)
    mid = pts[len(pts) // 2]
    t = state.joints["guard"]
    push = np.array([math.cos(t), -math.sin(t), 0.0])  # the guard's face toward the needle side
    pad = [mid + push * (task.GUARD_T / 2 + 0.002) + np.array([0.0, 0.0, dz]) for dz in (0.0, 0.02, 0.04)]
    return {"grippers": task.GripperProbe.make({"left_robot": (mid + np.array([0.0, 0.0, 0.0]), 0.047, pad)})}


IN_BIN_Z = BIN_FLOOR_TOP + FLANGE_HALF
RESET = ("scene reset", syr(), 0.0, False)


def good_close():
    """The closing sweep against the block, step by step, ending caught (score 0 while held in the air)."""
    return [(f"closing against the block, {d} deg", against_block(d), 0.0, False) for d in (30, 45, 60, 75, 86)]


CASES = [
    *good_close(),
    ("guard closed against the block, syringe back on the ridges", syr(SHUT), 0.5, False),
    ("then in the container, still", syr(SHUT, x=BIN_C[0], y=BIN_C[1], z=IN_BIN_Z), 1.0, True),
    ("in the container but still falling", syr(SHUT, x=BIN_C[0], y=BIN_C[1], z=IN_BIN_Z, vel=MOVING), 0.0, False),
    ("held 3 mm above the container floor", syr(SHUT, x=BIN_C[0], y=BIN_C[1], z=IN_BIN_Z + 0.003), 0.0, False),
    ("held 3 mm above the ridges", syr(SHUT, z=AXIS_Z + 0.003), 0.0, False),
    ("lying across the container's rim", syr(SHUT, x=BIN_C[0], y=BIN_C[1], z=TOP + 0.090 + FLANGE_HALF), 0.0, False),
    ("dropped on the table beside the tray", syr(SHUT, x=0.30, y=-0.30, z=TOP + FLANGE_HALF), 0.0, False),
    ("then put in the container: still 0 (it was on the table)",
     syr(SHUT, x=BIN_C[0], y=BIN_C[1], z=IN_BIN_Z), 0.0, False),
    RESET,
    ("habit: binned straight away, guard open", syr(OPEN, x=BIN_C[0], y=BIN_C[1], z=IN_BIN_Z), 0.0, False),
    ("habit: left on the tray with the guard open", syr(OPEN), 0.0, False),
    ("guard pushed shut in the air above the tray, 45 deg", syr(math.radians(45), z=AXIS_Z + 0.05), 0.0, False),
    ("pushed in the air, then caught", syr(SHUT, z=AXIS_Z + 0.05), 0.0, False),
    ("then binned: 0 (closed in the air)", syr(SHUT, x=BIN_C[0], y=BIN_C[1], z=IN_BIN_Z), 0.0, False),
    RESET,
    ("guard swung shut by the container wall while dropped in, 45 deg",
     syr(math.radians(45), x=BIN_C[0], y=BIN_C[1] + 0.02, z=TOP + 0.06), 0.0, False),
    ("caught inside the container", syr(SHUT, x=BIN_C[0], y=BIN_C[1], z=IN_BIN_Z), 0.0, False),
    RESET,
    ("guard 40 deg against the block, let swing back open, then closed in the air",
     against_block(40), 0.0, False),
    ("swung back open", syr(math.radians(10), z=AXIS_Z + 0.05), 0.0, False),
    ("closed in the air, 60 deg", syr(math.radians(60), z=AXIS_Z + 0.05), 0.0, False),
    ("caught in the air", syr(SHUT, z=AXIS_Z + 0.05), 0.0, False),
    ("back on the ridges: 0", syr(SHUT), 0.0, False),
    RESET,
    ("exploit: hinge beside the block, guard pushed shut by a wide-open free gripper, 45 deg",
     {**hinge_by_block(45), **pads_on_guard(hinge_by_block(45)["syringe"])}, 0.0, False),
    ("... 88 deg, clicked", {**hinge_by_block(88), **pads_on_guard(hinge_by_block(88)["syringe"])}, 0.0, False),
    ("then binned: 0", syr(SHUT, x=BIN_C[0], y=BIN_C[1], z=IN_BIN_Z), 0.0, False),
    RESET,
    ("hinge beside the block, outer half in the air, no gripper seen, 45 deg", hinge_by_block(45), 0.0, False),
    ("... clicked", hinge_by_block(88), 0.0, False),
    ("then binned: 0 (only the inner end was near the block)", syr(SHUT, x=BIN_C[0], y=BIN_C[1], z=IN_BIN_Z),
     0.0, False),
    RESET,
    ("free gripper pushing the guard while its outer half also touches the block, 45 deg",
     {**against_block(45), **pads_on_guard(against_block(45)["syringe"])}, 0.0, False),
    ("... clicked", {**against_block(88), **pads_on_guard(against_block(88)["syringe"])}, 0.0, False),
    ("then binned: 0 (a hand was at the guard)", syr(SHUT, x=BIN_C[0], y=BIN_C[1], z=IN_BIN_Z), 0.0, False),
    RESET,
    ("guard 4.5 deg short of the catch, on the ridges", syr(CATCH - math.radians(0.5)), 0.0, False),
]


def _run() -> int:
    import task

    base = {o.name: o.state() for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(base) | changes
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
