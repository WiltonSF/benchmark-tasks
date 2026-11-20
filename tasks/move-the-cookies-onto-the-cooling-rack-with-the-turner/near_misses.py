"""Near-successes and unintended strategies, scored offline by the offline checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-trial record what the step callback records for a trial in which no
gripper touched a cookie, no cookie was dropped, put on the counter or turned over, the hands ended 50 mm from
everything and the scene has been still for 1 s. The cases then vary where the cookies and the turner end up. _run()
afterwards drives the step-side function observe() directly, step by step, to check the whole-trial rule (a gripper
on a cookie), the per-cookie spoil rules (dropped, on the counter, turned over), the 10 mm let-go distance, the 1 s
hold and the unknown-arm rule, in both lots, and raises if anything scores wrongly.
"""

import math

import numpy as np

import lots
import task
from task import CK_H, FLOOR, NAMES, RACK_C, RK_X, RK_Y, SPOTS, TOP, WIRE_TOP

lots.use("A")
CLEAR = 0.05
PARTS = dict(zip(NAMES, task.COOKIE_PARTS))


def q_tilt(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def cookie(n, x, y, z, quat=(0.0, 0.0, 0.0, 1.0), vz=0.0):
    return PARTS[n].state(pos=(x, y, z), quat_xyzw=quat, lin_vel=(0.0, 0.0, vz))


def turner(pos=None, yaw=None):
    p = task.TURNER.pos if pos is None else pos
    y = task.TURNER_YAW if yaw is None else yaw
    return task.TURNER.state(pos=p, quat_xyzw=task._qz(y))


def scene(cookies, t=None):
    out = {"pan": task.PAN.state(), "rack": task.RACK.state(), "rest": task.REST.state(),
           "turner": turner() if t is None else t}
    out.update(cookies)
    return out


def on_spots():
    return {n: cookie(n, *SPOTS[i], WIRE_TOP) for i, n in enumerate(NAMES)}


def start(lot="A"):
    return {n: cookie(n, *task.COOKIES_AT[lot][i], FLOOR) for i, n in enumerate(NAMES)}


def preset():
    task.reset_record()
    task._rec.update(steps=1200, gap=CLEAR, t=60.0, at_rest=True)


def with_(base, **kw):
    out = dict(base)
    out.update(kw)
    return out


preset()
S = on_spots()
P = start()
CASES = [
    ("three cookies flat on the rack, turner back on its rest, hands clear, still 1 s", scene(S), 1.0, True),
    ("three cookies on the rack anywhere wholly over the wires",
     scene({n: cookie(n, RACK_C[0] + (i - 1) * 0.09, RACK_C[1] + 0.06, WIRE_TOP) for i, n in enumerate(NAMES)}), 1.0,
     True),
    ("a cookie's outline exactly at the rack's edge",
     scene(with_(S, cookie_1=cookie("cookie_1", RACK_C[0] - RK_X / 2 + task.CK_R, SPOTS[0][1], WIRE_TOP))), 1.0, True),
    ("a cookie hanging 1 mm over the rack's edge",
     scene(with_(S, cookie_1=cookie("cookie_1", RACK_C[0] - RK_X / 2 + task.CK_R - 0.001, SPOTS[0][1], WIRE_TOP))), 0.5,
     False),
    ("a cookie hanging over the rack's left edge by 20 mm",
     scene(with_(S, cookie_1=cookie("cookie_1", SPOTS[0][0], RACK_C[1] + RK_Y / 2 - task.CK_R + 0.02, WIRE_TOP))), 0.5,
     False),
    ("three on the rack but the turner left lying on the pan",
     scene(S, turner((0.30, -0.025, FLOOR + task.DROP), 0.0)), 0.75, False),
    ("three on the rack, turner on its rest but turned 20 degrees", scene(S, turner(yaw=task.TURNER_YAW + math.radians(20))),
     0.75, False),
    ("turner on its rest 25 mm from its marked place", scene(S, turner((task.TURNER_AT[0] + 0.025, task.TURNER_AT[1],
                                                                          task.TURNER_Z))), 1.0, True),
    ("two on the rack, one still on the pan", scene(with_(S, cookie_3=P["cookie_3"])), 0.5, False),
    ("one on the rack, two still on the pan", scene(with_(P, cookie_1=S["cookie_1"])), 0.25, False),
    ("nothing moved", scene(P), 0.0, False),
    ("a cookie upside down on the rack", scene(with_(S, cookie_2=cookie("cookie_2", *SPOTS[1], WIRE_TOP + CK_H,
                                                                          q_tilt(180.0)))), 0.5, False),
    ("a cookie on the rack leaning 10 degrees (on another's rim)",
     scene(with_(S, cookie_2=cookie("cookie_2", *SPOTS[1], WIRE_TOP + 0.003, q_tilt(10.0)))), 0.5, False),
    ("a cookie on the rack leaning 8 degrees", scene(with_(S, cookie_2=cookie("cookie_2", *SPOTS[1], WIRE_TOP + 0.003,
                                                                              q_tilt(8.0)))), 1.0, True),
    ("a cookie stacked on another on the rack", scene(with_(S, cookie_2=cookie("cookie_2", *SPOTS[0], WIRE_TOP + CK_H))),
     0.5, False),
    ("a cookie still on the turner's blade 10 mm over the wires",
     scene(with_(S, cookie_3=cookie("cookie_3", *SPOTS[2], WIRE_TOP + 0.010))), 0.5, False),
    ("three cookies on the counter beside the rack",
     scene({n: cookie(n, 0.30 + 0.09 * i, -0.45, TOP) for i, n in enumerate(NAMES)}), 0.0, False),
]

# -- step-by-step trials ------------------------------------------------------------------------------------------------
HZ = 20.0


class Trial:
    def __init__(self, lot="A"):
        lots.use(lot)
        task.reset_record()
        self.lot = lot
        self.t = 0.0
        self.c = start(lot)
        self.tr = turner()
        self.step(n=2)

    def step(self, gaps=(0.30, 0.30), n=1):
        for _ in range(n):
            self.t += 1.0 / HZ
            task.observe(scene(self.c, self.tr), gaps, self.t)

    def carry(self, i, via=None, vz=0.0, gaps=(0.05, 0.05)):
        """Cookie i lifted on the blade over the pan's rim, then onto its spot on the rack."""
        n = NAMES[i]
        x0, y0 = task.COOKIES_AT[self.lot][i]
        sx, sy = SPOTS[i]
        self.tr = turner((x0 - task.BLADE_C, y0, FLOOR + task.DROP), 0.0)
        for z in np.linspace(FLOOR, TOP + 0.06, 8):
            self.c[n] = cookie(n, x0, y0, float(z))
            self.step(gaps)
        if via is not None:
            self.c[n] = via
            self.step(gaps)
        for k in range(1, 9):
            self.c[n] = cookie(n, x0 + (sx - x0) * k / 8, y0 + (sy - y0) * k / 8, TOP + 0.06)
            self.step(gaps)
        self.c[n] = cookie(n, sx, sy, WIRE_TOP + 0.004, vz=vz)
        self.step(gaps)
        self.c[n] = cookie(n, sx, sy, WIRE_TOP)
        self.step(gaps, n=3)

    def finish(self, gaps=(CLEAR, CLEAR), still_s=1.2):
        self.tr = turner()
        self.step((0.03, 0.005), n=4)
        self.step(gaps, n=int(round(still_s * HZ)))

    def score(self):
        return task.score(scene(self.c, self.tr))


def expect(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"near_misses step test '{label}': score {got}, expected {want}; record {task._rec}")


def _run():
    for lot in ("A", "B"):
        tr = Trial(lot)
        expect(f"lot {lot}: nothing done", tr.score(), 0.0)
        for i in range(3):
            tr.carry(i)
        tr.finish()
        expect(f"lot {lot}: three cookies carried on the turner to the rack, turner back, let go, still", tr.score(),
               1.0)

    tr = Trial()
    tr.carry(0)
    tr.carry(1, gaps=(0.0015, 0.0015))
    tr.carry(2)
    tr.finish()
    expect("a gripper within 1.5 mm of a cookie at some step (picked by hand)", tr.score(), 0.0)
    tr = Trial()
    tr.carry(0)
    tr.carry(1, vz=-0.65)
    tr.carry(2)
    tr.finish()
    expect("one cookie falling at 0.65 m/s onto the rack (dropped)", tr.score(), 0.5)
    tr = Trial()
    tr.carry(0)
    tr.carry(1, vz=-0.55)
    tr.carry(2)
    tr.finish()
    expect("one cookie set down at 0.55 m/s", tr.score(), 1.0)
    tr = Trial()
    tr.carry(0, via=cookie("cookie_1", 0.25, -0.45, TOP))
    tr.carry(1)
    tr.carry(2)
    tr.finish()
    expect("one cookie on the counter on the way, then put on the rack", tr.score(), 0.5)
    tr = Trial()
    tr.carry(0)
    tr.carry(1)
    tr.carry(2, via=cookie("cookie_3", 0.40, -0.10, TOP + 0.06, q_tilt(90.0)))
    tr.finish()
    expect("one cookie turned on its edge on the way", tr.score(), 0.5)
    tr = Trial()
    for i in range(3):
        tr.carry(i)
    tr.tr = turner()
    tr.step((0.05, 0.005), n=30)
    expect("turner on its rest but a finger still 5 mm from it", tr.score(), 0.0)
    tr = Trial()
    for i in range(3):
        tr.carry(i)
    tr.finish(still_s=0.9)
    expect("everything still only 0.9 s", tr.score(), 0.0)
    tr = Trial()
    for i in range(2):
        tr.carry(i)
    task.observe(scene(tr.c, tr.tr), None, tr.t + 0.05)
    tr.t += 0.05
    tr.carry(2)
    tr.finish()
    expect("arm state unreadable at one step", tr.score(), 0.0)
    tr = Trial()
    tr.carry(0)
    tr.finish()
    expect("one cookie moved, turner back", tr.score(), 0.25)
    lots.use("A")
    preset()


_run()
