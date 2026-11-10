"""Near-successes and unintended strategies, scored offline with task.py's own score() and success().

Each case: (label, {object: ready state}, expected score, expected success). Cases run in order against one copy of
the task module, so the whole-trial fault record carries from one case to the next: cases that trip it come last.
Offline there is no robot, so "let go" always holds.
"""

from __future__ import annotations

import math

import numpy as np

import task

MOVING = (0.0, 0.0, -0.05)


def _yaw(a):
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


def box_at(gap=0.002, dy=0.0, yaw=0.0, lift=0.0, vel=None):
    """The transfer box with its open end `gap` short of the cage face, centred `dy` off the doorway, turned `yaw`."""
    mx, my = task.FACE_X - gap, task.CAGE_XY[1] + dy
    cx, cy = mx - task.BOX_L / 2 * math.cos(yaw), my - task.BOX_L / 2 * math.sin(yaw)
    return {"transfer_box": task.BOX.state(pos=(cx, cy, task.TOP + lift), quat_xyzw=_yaw(yaw), lin_vel=vel)}


def door(q):
    return {"cage": task.CAGE.state(joints={"door": q})}


UP, DOWN = task.TRAVEL, 0.0


CASES = [
    ("nothing done", {}, 0.0, False),
    ("docked 2 mm off, door down", {**box_at(), **door(DOWN)}, 0.5, False),
    ("docked, door up on its catch", {**box_at(), **door(UP)}, 1.0, True),
    ("docked 6 mm off, door up", {**box_at(gap=0.006), **door(UP)}, 1.0, True),
    ("docked, door 46 mm up on its catch", {**box_at(), **door(task.CATCH_AT)}, 1.0, True),
    ("docked, door 45 mm up (catch not taken)", {**box_at(), **door(0.045)}, 0.5, False),
    ("docked 11 mm to the left, door up", {**box_at(dy=0.011), **door(UP)}, 1.0, True),
    ("docked 5 degrees crooked, door up", {**box_at(yaw=math.radians(4.9)), **door(UP)}, 1.0, True),
    ("box 7 mm short of the face, door down", {**box_at(gap=0.007), **door(DOWN)}, 0.0, False),
    ("box 12 mm to the right, door down", {**box_at(dy=-0.012), **door(DOWN)}, 0.0, False),
    ("box 6 degrees crooked, door down", {**box_at(yaw=math.radians(6.0)), **door(DOWN)}, 0.0, False),
    ("box open end away from the cage (closed back to the door)",
     {**box_at(yaw=math.pi), **door(DOWN)}, 0.0, False),
    ("habit: carried straight in lot C (closed back at the face)", {**box_at(yaw=math.pi), **door(DOWN)}, 0.0, False),
    ("habit: no turn in lot A (side of the box at the face)",
     {**box_at(yaw=math.pi / 2), **door(DOWN)}, 0.0, False),
    ("box held 10 mm up at the face, door down", {**box_at(lift=0.010), **door(DOWN)}, 0.0, False),
    ("docked but still sliding", {**box_at(vel=MOVING), **door(DOWN)}, 0.0, False),
    # From here the fault record is set and stays set (as in a real trial, until a scene reset).
    ("door raised 25 mm before the box is docked", {**door(0.025)}, 0.0, False),
    ("then the box docked and the door up on its catch", {**box_at(), **door(UP)}, 0.0, False),
]


def _run() -> int:
    bad = 0
    for label, changes, want, want_success in CASES:
        st = {o.name: o.state() for o in task.OBJECTS}
        st.update(changes)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
