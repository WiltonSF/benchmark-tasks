"""Near misses and unintended strategies for switching the microscope to its oil objective.

The drops, the whole-trial rules and the still and hands-clear timers are a history the simulator callback builds
each control step by calling ``task.update`` with the states, the pad forces on the dropper's bulb and the contact
force on the objectives. The behaviour checks below drive that same function step by step from a fresh history and
assert what each strategy earns; importing this file fails if any of them is wrong. CASES (run by the offline checker)
then check end states on top of a correct drop, the 100x clicked in and everything still with the hands clear.
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
W0 = np.array((task.X0, task.Y0, TOP))
B0 = np.array((*task.BOTTLE_XY, TOP))
EPS = 1e-6
REC_T = [0.0]
BEHAVIOUR = []


def qy(deg):
    return tuple(parts._matrix_to_quat(parts._rot((0, 1, 0), math.radians(deg))))


def scope(q):
    return task.MICROSCOPE.state(joints={"nosepiece": q})


SEATED = task.DROPPER.state(pos=B0 + (0, 0, task.NECK_TOP))


def dropper_at(tip_local, lean=45.0):
    """The dropper leaned ``lean`` degrees toward the microscope with its tip at ``tip_local`` (microscope frame)."""
    r = parts._rot((0, 1, 0), math.radians(-lean))
    return task.DROPPER.state(pos=W0 + np.asarray(tip_local) - r @ np.array((0, 0, task.TIP_Z)),
                              quat_xyzw=tuple(parts._matrix_to_quat(r)))


SPOT = np.array((0.0, 0.0, task.SLIDE_TOP + 0.006))
LIFTED = task.DROPPER.state(pos=B0 + (0, 0, task.NECK_TOP + 0.09))


def start():
    return {task.MICROSCOPE.name: scope(0.0), task.BOTTLE.name: task.BOTTLE.state(), task.DROPPER.name: SEATED}


def fresh():
    task.reset_record()
    REC_T[0] = 0.0
    task.REC["loaded"] = task.LOAD


def steps(seq, pads=(0.0, 0.0), touch=0.0, clear=False):
    """Feed ``seq`` (a list of {name: state} changes) one 0.05 s control step each."""
    for o in seq:
        REC_T[0] += 0.05
        st = start()
        st.update(o)
        task.update(st, REC_T[0], pads, touch, clear)


def turn(q0, q1, extra=None, step_deg=3.0, **kw):
    n = max(1, int(round(abs(math.degrees(q1 - q0)) / step_deg)))
    steps([{task.MICROSCOPE.name: scope(q0 + (q1 - q0) * k / n), **(extra or {})} for k in range(1, n + 1)], **kw)


HALF, Q100 = math.radians(-45), task.Q100


def drop_at(tip, q=HALF, pads=(12.0, 12.0), dropper=None):
    d = dropper or dropper_at(tip)
    steps([{task.MICROSCOPE.name: scope(q), task.DROPPER.name: LIFTED}] * 2)
    steps([{task.MICROSCOPE.name: scope(q), task.DROPPER.name: d}] * 2, pads=(3.0, 3.0))
    steps([{task.MICROSCOPE.name: scope(q), task.DROPPER.name: d}], pads=pads)
    steps([{task.MICROSCOPE.name: scope(q), task.DROPPER.name: d}] * 2, pads=(3.0, 3.0))


def finish(q=Q100, dropper=SEATED, clear=True, secs=1.2):
    o = {task.MICROSCOPE.name: scope(q), task.DROPPER.name: dropper}
    steps([o] * int(round(secs / 0.05)), clear=clear)
    st = start()
    st.update(o)
    return task.score(st)


def check(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"near-miss behaviour '{label}': got {got}, want {want}")
    BEHAVIOUR.append((label, got))


def good(q_end=Q100, **kw):
    fresh(); turn(0.0, HALF); drop_at(SPOT); turn(HALF, q_end)
    return finish(q=q_end, **kw)


check("do nothing", (fresh(), finish(q=0.0))[1], 0.0)
check("the full job: half-turn, one drop on the lit spot, 100x clicked in, dropper home, hands clear", good(), 1.0)
check("... hands clear for only 0.5 s", (good(clear=False), finish(secs=0.5))[1], 0.75)
check("... dropper left on the bench", good(dropper=task.DROPPER.state(pos=B0 + (0.06, 0, 0.0075),
                                                                      quat_xyzw=qy(90))), 0.5)
check("... 100x 1.9 deg short of its stop", good(q_end=Q100 + math.radians(1.9)), 1.0)
check("... 100x 2.5 deg short of its stop", good(q_end=Q100 + math.radians(2.5)), 0.25)
fresh(); turn(0.0, HALF); drop_at(SPOT)
check("drop on the lit spot, nosepiece left half-way", finish(q=HALF), 0.25)
# the oil goes on with the 40x still in the light path: it lands on (or under) the 40x front lens
fresh(); drop_at(SPOT, q=0.0); turn(0.0, Q100)
check("drop put on with the 40x still over the lit spot", finish(), 0.0)
fresh(); drop_at(SPOT + (0.003, 0.0, 0.0), q=0.0); turn(0.0, Q100)
check("drop 3 mm beside the 40x's front lens", finish(), 0.0)
# drop placed at the half-turn, then the nosepiece turned back the short way: the 40x crosses the oil
fresh(); turn(0.0, HALF); drop_at(SPOT); turn(HALF, 0.0); turn(0.0, HALF); turn(HALF, Q100)
check("40x turned back through the oil, then the 100x put in", finish(), 0.0)
fresh(); turn(0.0, HALF); drop_at(SPOT); turn(HALF, math.radians(-30)); turn(math.radians(-30), Q100)
check("turned back to -30 deg (40x front lens 1.4 mm up, 20 mm away) and on again", finish(), 1.0)
# overshooting the 100x to the 4x (which passes 25 mm high) and back is fine; the 40x never comes round again
fresh(); turn(0.0, HALF); drop_at(SPOT); turn(HALF, math.radians(-200)); turn(math.radians(-200), Q100)
check("turned past the 100x to the 4x and back", finish(), 1.0)
fresh(); turn(0.0, HALF); drop_at(SPOT); turn(HALF, math.radians(30)); turn(math.radians(30), Q100)
check("turned the other way first (40x back over the oil)", finish(), 0.0)
# placement and amount
fresh(); turn(0.0, HALF); drop_at(SPOT + (0.0031, 0.0, 0.0)); turn(HALF, Q100)
check("drop centre 3.1 mm off the lit spot", finish(), 0.0)
fresh(); turn(0.0, HALF); drop_at(SPOT + (0.0029, 0.0, 0.0)); turn(HALF, Q100)
check("drop centre 2.9 mm off the lit spot", finish(), 1.0)
fresh(); turn(0.0, HALF); drop_at(SPOT); drop_at(SPOT); turn(HALF, Q100)
check("two drops", finish(), 1.0)
fresh(); turn(0.0, HALF); drop_at(SPOT); drop_at(SPOT); drop_at(SPOT); turn(HALF, Q100)
check("three drops", finish(), 0.0)
fresh(); turn(0.0, HALF); drop_at(SPOT, pads=(36.0, 36.0)); turn(HALF, Q100)
check("bulb crushed (36 N): a stream", finish(), 0.0)
fresh(); turn(0.0, HALF); drop_at(SPOT + (0.0, -0.040, 0.0)); turn(HALF, Q100)
check("drop off the slide's end, on the stage", finish(), 0.0)
fresh(); turn(0.0, HALF); drop_at(SPOT + (0.0, 0.0, 0.030)); turn(HALF, Q100)
check("squeezed 36 mm above the slide (it splashes)", finish(), 0.0)
fresh(); turn(0.0, HALF); drop_at(SPOT, pads=(9.9, 12.0)); turn(HALF, Q100)
check("squeeze 9.9 N on one pad: no drop", finish(), 0.0)
fresh(); turn(0.0, HALF); drop_at(SPOT, dropper=dropper_at(SPOT, lean=-140.0)); turn(HALF, Q100)
check("squeezed with the tip pointing up: no drop", finish(), 0.0)
fresh(); turn(0.0, HALF, touch=0.11); drop_at(SPOT); turn(HALF, Q100)
check("nosepiece turned by pushing an objective (0.11 N)", finish(), 0.0)
fresh(); turn(0.0, HALF, touch=0.1); drop_at(SPOT); turn(HALF, Q100)
check("an objective's contact sensor reads 0.1 N (no touch)", finish(), 1.0)
# straight to the 100x with no oil, then oil squeezed beside its nose (the tip over the nose catches it)
c100, _ = task.nose("100x", Q100)
fresh(); turn(0.0, Q100); drop_at(np.array((c100[0] + 0.006, c100[1], task.SLIDE_TOP + 0.004)), q=Q100)
check("turned straight to the 100x, then oil squeezed 6 mm beside its nose", finish(), 0.0)
fresh(); turn(0.0, HALF); drop_at(SPOT + (0.0, 0.0, -0.007)); turn(HALF, Q100)
check("tip below the slide's top (under the stage) when squeezed", finish(), 0.0)
# only a grip centre on the rubber bulb squeezes: one on the glass stem or shoulder does not
check("grip centre on the bulb counts", float(task.on_bulb(SEATED, SEATED.pos + (0, 0, task.BULB_Z))), 1.0)
check("grip centre on the glass shoulder does not", float(task.on_bulb(SEATED, SEATED.pos + (0, 0, -0.006))), 0.0)
check("grip centre on the glass stem does not", float(task.on_bulb(SEATED, SEATED.pos + (0, 0, -0.040))), 0.0)
fresh(); turn(0.0, HALF); drop_at(SPOT); turn(HALF, Q100)
steps([{task.BOTTLE.name: task.BOTTLE.state(quat_xyzw=qy(50))}])
check("bottle knocked over, then stood back up", finish(), 0.0)

# end states (check.py): on top of a good drop and the 100x clicked in, still and clear for 1.2 s
fresh(); turn(0.0, HALF); drop_at(SPOT); turn(HALF, Q100); finish()
assert task.REC["still_s"] >= 1.0 and task.REC["clear_s"] >= 1.0 and not task.REC["zero"]
M, D = task.MICROSCOPE.name, task.DROPPER.name
CASES = [
    ("100x in the oil, dropper home", {M: scope(Q100), D: SEATED}, 1.0, True),
    ("nosepiece left half-way", {M: scope(HALF), D: SEATED}, 0.25, False),
    ("40x back in the light path (end state only)", {M: scope(0.0), D: SEATED}, 0.25, False),
    ("dropper held up over the bottle", {M: scope(Q100), D: LIFTED}, 0.5, False),
    ("dropper standing 4 mm high in the neck", {M: scope(Q100), D: task.DROPPER.state(pos=B0 + (0, 0, task.NECK_TOP + 0.004))},
     0.5, False),
    ("dropper home but bottle lifted 4 mm",
     {M: scope(Q100), task.BOTTLE.name: task.BOTTLE.state(pos=B0 + (0, 0, 0.004)),
      D: task.DROPPER.state(pos=B0 + (0, 0, 0.004 + task.NECK_TOP))}, 0.5, False),
    ("dropper still moving", {M: scope(Q100), D: task.DROPPER.state(pos=B0 + (0, 0, task.NECK_TOP),
                                                                   lin_vel=(0.0, 0.0, -0.05))}, 0.0, False),
]
