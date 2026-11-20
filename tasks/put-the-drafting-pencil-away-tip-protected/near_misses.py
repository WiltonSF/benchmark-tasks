"""Scoring counterexamples, run offline: every tier, both sides of each limit, and the shortcuts."""
import copy
import math

import numpy as np

import lots
import parts
import task

P, T = task.PENCIL, task.TRAY
FLOOR = task.TOP + task.TRAY_FLOOR


def quat(yaw=task.PENCIL_YAW_A, roll=0.0, pitch_deg=0.0):
    r = parts._rot((0, 0, 1), yaw) @ parts._rot((0, 1, 0), math.radians(pitch_deg)) @ parts._rot((1, 0, 0), roll)
    return parts._matrix_to_quat(r)


def tray_pos(lot="A"):
    return lots.Lot.pos(next(x for x in task.LOTS if x.name == lot), T.name)


IN_TRAY_A = (tray_pos()[0], tray_pos()[1], FLOOR + 0.005)  # axis 5 mm above the floor: lying on its grip


def states(pos=IN_TRAY_A, carrier=task.TRAVEL, clip=0.0, yaw=task.PENCIL_YAW_A, roll=0.0, pitch=0.0, v=0.0, w=0.0,
           tray=None, tray_tilt=0.0):
    pencil = P.state(joints={"carrier": carrier, "clip": clip}, pos=pos, quat_xyzw=quat(yaw, roll, pitch),
                     lin_vel=(v, 0.0, 0.0))
    pencil = parts.PartState(**{**pencil.__dict__, "ang_vel": np.array([0.0, 0.0, w])})
    tp = tray or tray_pos()
    tr = T.state(pos=tp, quat_xyzw=parts._matrix_to_quat(parts._rot((0, 0, 1), task.PENCIL_YAW_A)
                                                          @ parts._rot((1, 0, 0), math.radians(tray_tilt))))
    return {P.name: pencil, T.name: tr}


def grade(objs, held=1.0, known=True, invalid=False, dropped=False, stale=False):
    task.REC.update(known=known, invalid=invalid, dropped=dropped, held=held)
    task.REC["stamp"] = () if stale else task._stamp(objs[P.name], objs[T.name])
    return task.score(objs)


# check.py runs these against the task's own history, which is unobserved before any simulator step: all score 0.
CASES = [
    ("housed and in the tray, but no observed history", states(), 0.0, False),
    ("pencil at its start, sleeve out", {P.name: P.state()}, 0.0, False),
]

ON_REST = (*task.PENCIL_XY_A, task.TOP + task.AXIS_Z)
# The tray's long axis is along world y in lot A; its inner wall is 20 mm from the centre line in world x.
X_EDGE = IN_TRAY_A[0] + task.TRAY_IN[1] / 2  # world x of the far inner wall


def run():
    old = copy.deepcopy(task.REC)
    lots.use("A")
    r = []
    try:
        r.append(("tip housed, lying in the tray, let go, still 1 s: 1.0", grade(states()) == 1.0))
        r.append(("same, only 0.95 s let go and still: 0", grade(states(), held=0.95) == 0.0))
        r.append(("tip housed, left on the rest: 0.5", grade(states(pos=ON_REST)) == 0.5))
        r.append(("in the tray with the sleeve still out (rear button pressed instead): 0.5",
                  grade(states(carrier=0.0003)) == 0.5))
        r.append(("sleeve pushed 3.6 mm in (0.4 mm proud), in the tray: 0.5",
                  grade(states(carrier=task.TRAVEL - 0.0004)) == 0.5))
        r.append(("sleeve exactly 0.3 mm proud, in the tray: 1.0", grade(states(carrier=task.HOUSED_MIN)) == 1.0))
        r.append(("no-op: sleeve out, on the rest: 0", grade(states(pos=ON_REST, carrier=0.0003)) == 0.0))
        r.append(("clip held open but the sleeve still out, on the rest: 0",
                  grade(states(pos=ON_REST, carrier=0.0003, clip=0.2)) == 0.0))
        r.append(("housed, rolled 90 degrees onto its clip side in the tray: 1.0",
                  grade(states(roll=math.pi / 2)) == 1.0))
        r.append(("housed, turned 10 degrees in the tray, inside the walls: 1.0",
                  grade(states(yaw=task.PENCIL_YAW_A + math.radians(10))) == 1.0))
        r.append(("housed, turned 30 degrees, an end over the wall: 0.5",
                  grade(states(yaw=task.PENCIL_YAW_A + math.radians(30))) == 0.5))
        r.append(("housed, one end 40 mm past the tray's end wall: 0.5",
                  grade(states(pos=(IN_TRAY_A[0], IN_TRAY_A[1] + 0.06, IN_TRAY_A[2]))) == 0.5))
        side = X_EDGE - 0.005 - 1e-4  # grip surface 0.1 mm inside the inner wall
        r.append(("housed, against the inner side wall: 1.0", grade(states(pos=(side, IN_TRAY_A[1], IN_TRAY_A[2])))
                  == 1.0))
        r.append(("housed, 1 mm over the side wall: 0.5",
                  grade(states(pos=(X_EDGE - 0.004, IN_TRAY_A[1], IN_TRAY_A[2]))) == 0.5))
        r.append(("axis exactly 9 mm above the floor: 1.0",
                  grade(states(pos=(*IN_TRAY_A[:2], FLOOR + task.AXIS_MAX))) == 1.0))
        r.append(("axis 9.5 mm above the floor (held up): 0.5",
                  grade(states(pos=(*IN_TRAY_A[:2], FLOOR + task.AXIS_MAX + 0.0005))) == 0.5))
        r.append(("propped 11 degrees from level: 0.5",
                  grade(states(pos=(*IN_TRAY_A[:2], FLOOR + 0.006), pitch=11.0)) == 0.5))
        r.append(("tray knocked up 8 degrees: 0.5", grade(states(tray_tilt=8.0)) == 0.5))
        r.append(("pencil moving 6 mm/s: not still, no hold: 0", grade(states(v=0.006), held=0.0) == 0.0))
        r.append(("pencil fell off the table at any time: 0", grade(states(), dropped=True) == 0.0))
        r.append(("hand state unknown: 0", grade(states(), known=False) == 0.0))
        r.append(("a bad step earlier: 0", grade(states(), invalid=True) == 0.0))
        r.append(("stale state: 0", grade(states(), stale=True) == 0.0))
        r.append(("NaN carrier reading: 0", grade(states(carrier=math.nan)) == 0.0))
        s = states()
        r.append(("still at 5 mm/s and 0.05 rad/s", task.still([0.005, 0.0], [0.05, 0.0], 0.0)))
        r.append(("not still at 5.1 mm/s", not task.still([0.0051, 0.0], [0.0, 0.0], 0.0)))
        r.append(("not still while the tray turns 0.06 rad/s", not task.still([0.0, 0.0], [0.0, 0.06], 0.0)))
        r.append(("not still while the clip moves 0.003 rad a step", not task.still([0.0, 0.0], [0.0, 0.0], 0.003)))
        p0 = (np.asarray(s[P.name].pos), task._rot(s[P.name]))
        rolled = states(roll=0.006)[P.name]  # 0.006 rad turn in one 0.1 s step = 0.06 rad/s
        r.append(("pose-change turn rate: 0.006 rad in 0.1 s is 0.06 rad/s",
                  abs(task.motion(p0, rolled, 0.1)[1] - 0.06) < 1e-6))
        r.append(("no previous pose: not still", task.motion(None, rolled, 0.1) == (math.inf, math.inf)))
        # the let-go rule, on sampled points (box distance, not just vertices)
        bx = task.boxes(P, s[P.name]) + task.boxes(T, s[T.name])
        c, rr, h = task.boxes(P, s[P.name])[4]  # barrel
        top = c + rr @ np.array([0.0, 0.0, h[2]])
        r.append(("hand point 0.5 mm above the barrel's top face centre: touching",
                  task.hand_gap([np.array([top + (0, 0, 0.0005)])], bx) <= task.CLEAR))
        r.append(("hand point 5.1 mm above the barrel (and clear of the clip): clear",
                  task.hand_gap([np.array([c + rr @ np.array([-0.035, 0, 0]) + (0, 0, h[2] + 0.0051)])], bx)
                  > task.CLEAR))
        r.append(("finger pad 4 mm off the grip's side: touching",
                  task.hand_gap([np.array([task.boxes(P, s[P.name])[3][0] + (0.009, 0, 0)])], bx) <= task.CLEAR))
        r.append(("NaN hand points fail closed", task.hand_gap([np.full((3, 3), np.nan)], bx) == -math.inf))
        r.append(("no hand points fail closed", task.hand_gap([], bx) == -math.inf))
        # lot B: the tray moves to the robot's left and turns; a pencil laid where lot A's tray was earns no tray credit
        lots.use("B")
        tb = (*task.TRAY_B, task.TOP)
        trb = T.state(pos=tb, quat_xyzw=parts._matrix_to_quat(parts._rot((0, 0, 1), task.TRAY_YAW_B)))
        st = states(pos=(tb[0], tb[1], FLOOR + 0.005), yaw=task.TRAY_YAW_B)
        st[T.name] = trb
        r.append(("lot B: housed, lying in lot B's tray: 1.0", grade(st) == 1.0))
        st2 = states()
        st2[T.name] = trb
        r.append(("lot B: housed, laid where lot A's tray stood: 0.5", grade(st2) == 0.5))
    finally:
        lots.use(None)
        task.REC.clear()
        task.REC.update(old)
    for name, ok in r:
        print("ok  " if ok else "FAIL", name)
    return sum(not ok for _, ok in r)


if __name__ == "__main__":
    raise SystemExit(run())
