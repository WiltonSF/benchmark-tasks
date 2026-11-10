"""Near-successes and unintended strategies, scored offline with task.py's own score() and success().

Each case: (label, {object: ready state}, expected score, expected success). Every case that lays out a lot B or C
stack gives all six books, so it does not depend on the lot. Books in the case are built from a
spine sequence with task.stack_poses(); the lid from the case part's kinematics.
"""

from __future__ import annotations

import math

import numpy as np

import task

MOVING = (0.0, 0.0, -0.05)


def _quat(roll, yaw, pitch=0.0):
    """xyzw of Rx(roll) @ Ry(pitch) @ Rz(yaw)."""
    def q(axis, a):
        v = np.zeros(4)
        v[axis] = math.sin(a / 2)
        v[3] = math.cos(a / 2)
        return v

    def mul(a, b):
        x1, y1, z1, w1 = a
        x2, y2, z2, w2 = b
        return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                         w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
    return mul(mul(q(0, roll), q(1, pitch)), q(2, yaw))


def stacked(seq, books=None, dx=0.0, dy=0.0, vel=None, dys=None):
    """States for books stacked in the case, spines `seq` from the bottom (books 1..6 unless named)."""
    books = books or [1, 2, 3, 4, 5, 6][: len(seq)]
    out = {}
    for n, c, (y, z, roll) in zip(books, seq, task.stack_poses(seq, dys)):
        yaw = task.SPINE_LEFT if c == "L" else task.SPINE_RIGHT
        out[f"book_{n}"] = task.BOOKS[n].state(
            pos=(task.PLACE_X + dx, task.PLACE_Y + dy + y, task.FLOOR + z), quat_xyzw=_quat(roll, yaw), lin_vel=vel)
    return out


def lid(angle):
    return {"book_case": task.CASE.state(joints={"lid": angle})}


SHUT, OPEN = lid(task.LID_SHUT), lid(0.0)
ALT = "LRLRLR"


def on_table(n, x, y):
    """Book n lying flat on the table outside the case."""
    return {f"book_{n}": task.BOOKS[n].state(pos=(x, y, task.TOP + task.REST_Z), quat_xyzw=_quat(0.0, 0.0))}


def _on_edge():
    """Four books stood on edge across the case (spine up, length front to back), two laid flat across their tops,
    spines left then right: no counting needed, and every top-of-stack and fit check alone would pass."""
    out = {}
    for i, k in enumerate([1, 2, 3, 4]):
        y = task.CASE_Y - 0.035 + i * 0.023
        out[f"book_{k}"] = task.BOOKS[k].state(pos=(task.PLACE_X, y, task.FLOOR + task.BOOK_W / 2 + 0.0005),
                                               quat_xyzw=_quat(math.pi / 2, 0.0))
    for k, c, (y, z, roll) in zip([5, 6], "LR", task.stack_poses("LR")):
        yaw = task.SPINE_LEFT if c == "L" else task.SPINE_RIGHT
        out[f"book_{k}"] = task.BOOKS[k].state(pos=(task.PLACE_X, task.PLACE_Y + y, task.FLOOR + 0.0645 + z),
                                               quat_xyzw=_quat(roll, yaw))
    return out


CASES = [
    ("alternating stack, lid shut", {**stacked(ALT), **SHUT}, 1.0, True),
    ("balanced LLRRLR, lid shut", {**stacked("LLRRLR"), **SHUT}, 1.0, True),
    ("balanced LLLRRR (tallest 3/3), lid shut", {**stacked("LLLRRR"), **SHUT}, 1.0, True),
    ("lot C order L L R R L R, lid shut", {**stacked("LLRRLR", [1, 6, 2, 3, 4, 5]), **SHUT}, 1.0, True),
    ("alternating, lid 1.4 degrees off its stop", {**stacked(ALT), **lid(task.LID_SHUT + math.radians(1.4))},
     1.0, True),
    ("alternating, lid 1.6 degrees off its stop", {**stacked(ALT), **lid(task.LID_SHUT + math.radians(1.6))},
     0.5, False),
    ("alternating stack, lid still open", {**stacked(ALT), **OPEN}, 0.5, False),
    ("alternating stack, lid half shut", {**stacked(ALT), **lid(-0.8)}, 0.5, False),
    ("alternating stack shifted 10 mm left, lid shut", {**stacked(ALT, dy=0.010), **SHUT}, 1.0, True),
    ("alternating stack shifted 20 mm left: overhangs the floor", {**stacked(ALT, dy=0.020), **SHUT}, 0.0, False),
    ("stack shifted 15 mm toward the far rail: through it", {**stacked(ALT, dx=0.015), **SHUT}, 0.0, False),
    ("4 spines left, 2 right, lid shut (top leans 15 degrees)", {**stacked("LRLRLL"), **SHUT}, 0.0, False),
    ("4/2, lid open", {**stacked("LLRLRL"), **OPEN}, 0.0, False),
    ("habit: every book turned the same way (6/0)", {**stacked("LLLLLL"), **SHUT}, 0.0, False),
    ("habit: lot A's turns replayed in lot B (R + RLRLR)", {**stacked("RRLRLR"), **SHUT}, 0.0, False),
    ("habit: alternate from the top in lot C (L L + R L R L)",
     {**stacked("LLRLRL", [1, 6, 2, 3, 4, 5]), **SHUT}, 0.0, False),
    ("five books packed level, one left on the table, lid shut",
     {**stacked("LRLRL"), **on_table(6, 0.32, 0.37), **SHUT}, 0.0, False),
    ("alternating stack still settling", {**stacked(ALT, vel=MOVING), **SHUT}, 0.0, False),
    ("top book tipped 5 degrees along its length",
     {**stacked(ALT), "book_6": task.BOOKS[6].state(pos=(task.PLACE_X, task.PLACE_Y, task.FLOOR + task.stack_poses(ALT)[5][1]),
                                                    quat_xyzw=_quat(0.0, task.SPINE_RIGHT, math.radians(5.0))),
      **SHUT}, 0.0, False),
    ("four books stood on edge, two laid flat across them, lid shut", {**_on_edge(), **SHUT}, 0.0, False),
    ("tallest balanced stack's books lean at most 23 degrees: still flat", {**stacked("LLLRRR"), **OPEN}, 0.5, False),
    ("nothing done (lot A start)", {}, 0.0, False),
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
