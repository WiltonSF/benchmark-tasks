"""Near misses and unintended strategies for removing the crimp seal with the plier decapper.

The crimp, the whole-trial rules and the still and hands-clear timers are a history the simulator callback builds
each control step by calling ``task.update`` with the states and the contact forces it reads. The behaviour checks
below drive that same function step by step from a fresh history and assert what each strategy earns; importing
this file fails if any of them is wrong. CASES (run by factory/check.py) then checks end states on top of a torn
crimp, everything still for 1 s and the hands 30 mm clear for 1 s.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

import parts  # noqa: E402
import task  # noqa: E402

TOP = task.TOP
B0 = np.array((*task.BOTTLE_XY, TOP))
W0 = np.array((*task.BEAKER_XY, TOP))
EPS = 1e-6


def qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def tilt_q(deg, axis=(1.0, 0.0, 0.0)):
    return tuple(parts._matrix_to_quat(parts._rot(axis, math.radians(deg))))


def start():
    return {o.name: o.state() for o in task.OBJECTS if o.mass is not None}


def seal_at(pos, yaw=0.0, quat=None):
    return task.SEAL.state(pos=pos, quat_xyzw=quat if quat is not None else qz(yaw))


def on_finish(yaw=0.0):
    return seal_at(B0 + (0, 0, task.SEAL_Z), yaw)


def in_beaker_pos(dx=0.0, dy=0.0, dz=0.0):
    return W0 + np.array((dx, dy, 0.003 + dz))


def steps(seq, jaw=(20.0, 20.0), finger=0.0, glass=0.0, clear=True, t0=None):
    """Feed ``seq`` (a list of state dicts) one 0.05 s control step each; return the last update's result."""
    t = REC_T[0] if t0 is None else t0
    out = None
    for o in seq:
        t += 0.05
        st = start()
        st.update(o)
        out = task.update(st, t, jaw[0], jaw[1], finger, glass, clear)
    REC_T[0] = t
    return out


REC_T = [0.0]


def fresh():
    task.reset_record()
    REC_T[0] = 0.0


def twist(deg, n=None, jaw=(20.0, 20.0), **kw):
    """The seal turned on its finish from 0 to ``deg`` in 2-degree steps, with the given jaw forces."""
    n = n or max(1, int(round(abs(deg) / 2)))
    return steps([{task.SEAL.name: on_finish(math.radians(deg * k / n))} for k in range(1, n + 1)], jaw=jaw, **kw)


def end(o, secs=1.2, **kw):
    steps([o] * int(round(secs / 0.05)), jaw=(0.0, 0.0), **kw)
    st = start()
    st.update(o)
    return task.score(st)


def check(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"near-miss behaviour '{label}': got {got}, want {want}")
    BEHAVIOUR.append((label, got))


BEHAVIOUR = []
REMOVED = {task.SEAL.name: seal_at(B0 + (0, 0, task.SEAL_Z + 0.040))}
DONE = {task.SEAL.name: seal_at(in_beaker_pos())}

# doing nothing, or only squeezing, never tears the crimp
fresh(); check("do nothing", end({}), 0.0)
fresh(); steps([{}] * 60, jaw=(25.0, 25.0)); check("squeeze 3 s without turning", float(task.REC["released"]), 0.0)
# turning with one jaw, or with too light a squeeze, does not count
fresh(); twist(180, jaw=(25.0, 0.0)); check("turn with one jaw pressing", float(task.REC["released"]), 0.0)
fresh(); twist(180, jaw=(task.GRIP_N - 0.01, task.GRIP_N - 0.01))
check(f"turn at {task.GRIP_N - 0.01:.2f} N per jaw", float(task.REC["released"]), 0.0)
fresh(); twist(90, jaw=(task.GRIP_N, task.GRIP_N))
check(f"turn 90 deg at exactly {task.GRIP_N:.0f} N per jaw", float(task.REC["released"]), 1.0)
fresh(); twist(180, n=400)  # 0.45 degrees a step: jitter-sized steps never add up to a tear
check("creep 180 deg in 0.45-degree steps", float(task.REC["released"]), 0.0)
fresh(); twist(90, n=180); check("turn 90 deg in exactly 0.5-degree steps", float(task.REC["released"]), 1.0)
fresh(); twist(89); check("turn 89 deg", float(task.REC["released"]), 0.0)
fresh(); twist(45)  # there and most of the way back: 45 + 44 = 89 degrees of turning
steps([{task.SEAL.name: on_finish(math.radians(45 - 2 * k))} for k in range(1, 23)])
check("turn 45 deg and back 44 deg (89 in all)", float(task.REC["released"]), 0.0)
steps([{task.SEAL.name: on_finish(math.radians(-1.5))}]); check("... and 2.5 deg more", float(task.REC["released"]), 1.0)
# a torn crimp on its own: the seal still sits on the finish
fresh(); twist(100); check("crimp torn, seal left on the neck", end({task.SEAL.name: on_finish(math.radians(100))}), 0.25)
# the full job
fresh(); twist(100); check("seal in the beaker, plier back on the counter, hands clear", end(DONE), 1.0)
fresh(); twist(100); end(DONE, clear=False); check("... hands clear for only 0.5 s", end(DONE, secs=0.5), 0.75)
MOVING = {task.SEAL.name: task.SEAL.state(pos=in_beaker_pos(), lin_vel=(0.0, 0.0, -0.2))}
fresh(); twist(100); steps([MOVING] + [DONE] * 18, jaw=(0.0, 0.0))
check("... everything still for only 0.9 s", task.score({**start(), **DONE}), 0.0)
# whole-trial rules stay broken even when the scene is put right afterwards
fresh(); twist(100, finger=0.51); check("a finger presses the seal (0.51 N)", end(DONE), 0.0)
fresh(); twist(100, finger=0.5); check("a finger brushes the seal (0.5 N)", end(DONE), 1.0)
fresh(); twist(100, glass=20.1); check("the jaws press the glass with 20.1 N", end(DONE), 0.0)
fresh(); twist(100, glass=20.0); check("the jaws touch the glass with 20 N", end(DONE), 1.0)
fresh(); twist(100)
steps([{task.BOTTLE.name: task.BOTTLE.state(quat_xyzw=tilt_q(46))}])
check("bottle knocked over then stood back up", end(DONE), 0.0)
fresh(); twist(100)
steps([{task.BEAKER.name: task.BEAKER.state(quat_xyzw=tilt_q(46))}])
check("beaker knocked over then stood back up", end(DONE), 0.0)
# pulling the crimped seal off without turning it leaves it on the neck in the simulator (the callback holds it
# there); were it ever recorded off the bottle, no crimp was torn, so it earns nothing
fresh(); check("crimped seal lifted off without a turn", end(REMOVED), 0.0)

# end states, on top of a torn crimp, everything still 1.2 s and the hands 30 mm clear 1.2 s
fresh(); twist(100); end({})
assert task.REC["still_s"] >= 1.0 and task.REC["clear_s"] >= 1.0 and task.REC["released"] and not task.REC["zero"]

P = task.PLIER.name
LIFTED_PLIER = task.PLIER.state(pos=(task.PLIER_XY[0], task.PLIER_XY[1], TOP + 0.12), joints={"jaw": 0.4})
r_in = task.BEAKER_R - task.BEAKER_WALL - task.SKIRT_OUT
CASES = [
    ("torn crimp, seal on the neck", {task.SEAL.name: on_finish(1.0)}, 0.25, False),
    ("seal lifted 9 mm (gap under 10 mm)", {task.SEAL.name: seal_at(B0 + (0, 0, task.SEAL_Z + 0.0090))}, 0.25, False),
    ("seal lifted 40 mm", {task.SEAL.name: seal_at(B0 + (0, 0, task.SEAL_Z + 0.040))}, 0.5, False),
    ("seal on the counter beside the bottle", {task.SEAL.name: seal_at(B0 + (0.0, -0.060, 0.0))}, 0.5, False),
    ("seal leaning on the beaker's outside", {task.SEAL.name: seal_at(W0 + (0.0, -0.046, 0.0))}, 0.5, False),
    ("seal on the beaker floor, plier on the counter", DONE, 1.0, True),
    ("seal at the beaker wall (exactly fits)", {task.SEAL.name: seal_at(in_beaker_pos(dx=r_in))}, 1.0, True),
    ("seal 0.5 mm into the wall", {task.SEAL.name: seal_at(in_beaker_pos(dx=r_in + 0.0005))}, 0.5, False),
    ("seal propped above the rim", {task.SEAL.name: seal_at(in_beaker_pos(dz=task.BEAKER_H - 0.003))}, 0.5, False),
    ("seal in the beaker, plier still held up", {**DONE, P: LIFTED_PLIER}, 0.75, False),
    ("seal in the beaker, plier standing on end",
     {**DONE, P: task.PLIER.state(pos=(*task.PLIER_XY, TOP + 0.03), quat_xyzw=tilt_q(80))}, 0.75, False),
    ("seal in the beaker, bottle tipped 11 deg", {**DONE, task.BOTTLE.name: task.BOTTLE.state(quat_xyzw=tilt_q(11))},
     0.0, False),
    ("seal in the beaker, bottle lifted 4 mm",
     {**DONE, task.BOTTLE.name: task.BOTTLE.state(pos=B0 + (0, 0, 0.004))}, 0.0, False),
    ("seal in the beaker, beaker tipped 12 deg",
     {**DONE, task.BEAKER.name: task.BEAKER.state(quat_xyzw=tilt_q(12))}, 0.5, False),
    ("seal in the beaker, still falling", MOVING, 0.0, False),
]
