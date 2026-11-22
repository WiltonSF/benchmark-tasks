"""Near misses and unintended strategies for fitting the extensometer.

The drag record, the damage rules and the clip fit are a history the simulator builds after every physics substep
(240 Hz) by calling ``task.physics_update`` with the extensometer's pose and the contact forces it reads; the tier
history is built every control step (20 Hz) by ``task.control_update``. The behaviour checks below drive those same
functions from a fresh history and assert what each strategy earns; importing this file fails if any is wrong.
CASES (run by the offline checker) then check end states on top of a correct fit with the hands clear for 1.2 s.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

import parts  # noqa: E402
import task  # noqa: E402

EPS = 1e-9
Q0 = np.array((0.0, 0.0, 0.0, 1.0))
FIT = np.array((task.XF - task.L_TIP, task.Y, task.AXIS))  # sensor origin with the knife edges on the face
SUB = 12  # physics substeps per control step
_T = [0.0]
_POSE = [None]


def q_about(axis, deg):
    return parts._matrix_to_quat(parts._rot(axis, math.radians(deg)))


def fresh(start=None):
    task.reset_record()
    _T[0] = 0.0
    _POSE[0] = (np.asarray(task.SENSOR_A if start is None else start, float), Q0)


def contact(pos, q, force):
    """The specimen force the simulator would report: ``force`` once any knife edge touches, else 0."""
    near = task.specimen_distance(task.blade_points(pos, q)) <= task.TOUCH_GAP + 1e-7
    return force if bool(np.any(near)) else 0.0


def move(to, q=Q0, secs=0.5, force=0.5, machine=0.0, hand=10.0, net=0.6, clear=False, touching=True):
    """Move the held extensometer to ``to`` in ``secs`` (straight line), feeding substeps and control steps."""
    p0, q0 = _POSE[0]
    to = np.asarray(to, float)
    q = np.asarray(q, float)
    n = max(1, int(round(secs * 20)))
    for i in range(1, n + 1):
        for k in range(1, SUB + 1):
            if task.REC["fit"] is not None:
                p, qq = task.REC["fit"]["pos"], task.REC["fit"]["quat"]  # the clips hold it there
            else:
                a = ((i - 1) * SUB + k) / (n * SUB)
                p, qq = p0 + (to - p0) * a, q
            task.physics_update(p, qq, contact(p, qq, force), machine, hand, net)
        if task.REC["fit"] is not None:
            p, qq = task.REC["fit"]["pos"], task.REC["fit"]["quat"]
        _T[0] += 0.05
        st = {task.SENSOR.name: task.SENSOR.state(pos=p, quat_xyzw=qq)}
        task.control_update(st, _T[0], clear, touching)
    _POSE[0] = (p, qq)


def let_go(secs=1.2):
    """Fingers open and the hand leaves: no hand force, hands clear."""
    move(_POSE[0][0], _POSE[0][1], secs, hand=0.0, net=0.0, clear=True, touching=False)


def now():
    p, q = _POSE[0]
    return task.score({task.SENSOR.name: task.SENSOR.state(pos=p, quat_xyzw=q)})


def approach(dy=0.0, dz=0.0, q=Q0, push=0.0015, force=0.5):
    """The maker's route: carry above the edge, lower the clips over it with the knife edges 1.5 mm off the face,
    then ease the knife edges onto the face."""
    off = np.array((0.0, dy, dz))
    move(FIT + off + (-0.0015, 0, 0.050), q, 2.0)
    move(FIT + off + (-0.0015, 0, 0.0), q, 2.0)
    move(FIT + off + (push, 0, 0.0), q, 1.0, force=force)


def check(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"near miss '{label}': scored {got}, expected {want} ({task.REC['why']})")


# 1. The maker's hook route, released and clear for 1.2 s: full score.
fresh(); approach(); let_go(); check("clean fit, released, clear 1.2 s", now(), 1.0)
assert task.REC["fit"]["quality"]["ok"] and float(np.max(task.REC["drag"])) < 1e-6
# 2. The same in layout B's start.
fresh(task.SENSOR_B); approach(); let_go(); check("clean fit from layout B", now(), 1.0)
# 3. Released for only 0.5 s: the 1 s hold is not met, the hold tier is 0.75.
fresh(); approach(); let_go(0.5); check("released for 0.5 s only", now(), 0.75)
# 4. Still holding the housing at the end: 0.75.
fresh(); approach(); move(_POSE[0][0], secs=1.5); check("fitted but still held", now(), 0.75)
# 5. Knife edges put on the face, then slid 0.6 mm along it before the clips take it: drag mark, 0.
fresh(); move(FIT + (-0.0015, 0.0006, 0.05), secs=1.0); move(FIT + (-0.0015, 0.0006, 0.0), secs=1.0)
move(FIT + (0.0, 0.0006, 0.0), secs=0.5, force=0.05)  # touching at 0.05 N, below the clip-fit force
move(FIT + (0.0, 0.0, 0.0), secs=0.5, force=0.05)  # slid 0.6 mm along the axis while touching
move(FIT + (0.0015, 0.0, 0.0), secs=0.5); let_go(); check("slid 0.6 mm then seated", now(), 0.0)
# 6. A 0.4 mm slide is under the 0.5 mm limit: full score (the limit is where the rule says).
fresh(); move(FIT + (-0.0015, 0.0004, 0.05), secs=1.0); move(FIT + (-0.0015, 0.0004, 0.0), secs=1.0)
move(FIT + (0.0, 0.0004, 0.0), secs=0.5, force=0.05); move(FIT, secs=0.5, force=0.05)
move(FIT + (0.0015, 0.0, 0.0), secs=0.5); let_go(); check("slid 0.4 mm then seated", now(), 1.0)
# 7. Scrubbing +-0.3 mm three times leaves a 0.6 mm track: 0. Scrubbing +-0.2 mm leaves a 0.4 mm track, which is
# under the 0.5 mm limit (the rule reads the track's length, as a loupe would): 1.0.
for amp, want in ((0.0003, 0.0), (0.0002, 1.0)):
    fresh(); move(FIT + (-0.0015, 0, 0.05), secs=1.0); move(FIT + (-0.0015, 0, 0.0), secs=1.0)
    move(FIT, secs=0.5, force=0.05)
    for _ in range(3):
        move(FIT + (0, amp, 0), secs=0.2, force=0.05)
        move(FIT + (0, -amp, 0), secs=0.2, force=0.05)
    move(FIT + (0.0015, 0, 0), secs=0.5); let_go(); check(f"scrubbed +-{amp * 1000:.1f} mm", now(), want)
# 8. Lowered straight down with the knife edges rubbing the face (the naive way): 0.
fresh(); move(FIT + (0.0, 0, 0.05), secs=1.0, force=0.05); move(FIT, secs=1.0, force=0.05)
move(FIT + (0.0015, 0, 0), secs=0.5); let_go(); check("lowered while touching the face", now(), 0.0)
# 9. Fitted 3 mm off the gauge marks along the axis: fitted, outside the gauge tolerance, 0.5.
fresh(); approach(dy=0.003); let_go(); check("3 mm off the marks", now(), 0.5)
# 10. Fitted with the knife edges leaning 4 degrees in the face plane: 0.5.
fresh(); approach(q=q_about((1, 0, 0), 4.0)); let_go(); check("knife edges leaning 4 deg", now(), 0.5)
# 11. 1.0 mm above the centre line is inside the tolerance; 1.8 mm leaves the 13 mm narrow face: no fit.
fresh(); approach(dz=0.001); let_go(); check("1 mm above centre", now(), 1.0)
fresh(); approach(dz=0.0018); assert task.REC["fit"] is None
# 12. Only one knife edge touches (sensor turned 2 degrees about the vertical): the clips do not take it; held
# there with the clips over the edge it shows 0.25; let go, it falls.
fresh(); approach(q=q_about((0, 0, 1), 2.0), push=-0.0004); assert task.REC["fit"] is None
move(_POSE[0][0], q_about((0, 0, 1), 2.0), secs=1.2); check("one knife edge, clips over the edge, held", now(), 0.25)
move(np.array((FIT[0], FIT[1], task.TOP + 0.002)), q_about((0, 0, 1), 2.0), secs=0.3, hand=0.0, net=0.0,
     clear=True, touching=False)
check("one knife edge, let go, falls", now(), 0.0)
# 13. Pressed onto the face with 9 N: dent, 0.
fresh(); approach(force=9.0); let_go(); check("pressed with 9 N", now(), 0.0)
# 14. Fitted, then the hand pushes or pulls it with 2.5 N before letting go: knife edges slip, 0. At 1.5 N: 1.0.
fresh(); approach(); move(_POSE[0][0], secs=0.3, net=2.5); let_go(); check("pushed 2.5 N after the fit", now(), 0.0)
fresh(); approach(); move(_POSE[0][0], secs=0.3, net=1.5); let_go(); check("pushed 1.5 N after the fit", now(), 1.0)
# 15. Struck the machine with 9 N on the way: 0.
fresh(); move(FIT + (-0.0015, 0, 0.05), secs=1.0, machine=9.0); approach(); let_go(); check("struck machine", now(), 0.0)
# 16. Crushed the housing with a 45 N grip: 0.
fresh(); move(FIT + (-0.0015, 0, 0.05), secs=1.0, hand=45.0); approach(); let_go(); check("crushed housing", now(), 0.0)
# 17. Clips not over the edge: upside down at the fitted spot (clips under the lower edge), or 1.5 mm too low (the
# clip wire would rest across the edge's top, not above it), or hooks still in front of the back face: no fit.
flip = q_about((1, 0, 0), 180.0)
assert task.hooked(FIT, Q0) and task.seated(FIT, Q0)
assert not task.hooked(FIT, flip) and not task.hooked(FIT + (-0.010, 0, 0), Q0)
assert not task.hooked(FIT + (0, 0, -0.0035), Q0)
fresh(); approach(q=flip); let_go(); check("fitted upside down", now(), 0.0)
# 18. Never touched: the extensometer still in its case scores 0.
fresh(); move(task.SENSOR_A, secs=1.5, hand=0.0, net=0.0, clear=True, touching=False); check("left in case", now(), 0.0)
# 19. Dropped on the bench: 0.
fresh(); move(np.array((0.25, 0.05, task.TOP + 0.0195)), secs=0.5, hand=0.0, net=0.0, clear=True, touching=False)
check("dropped on the bench", now(), 0.0)

# End states for check.py: each runs on top of a clean fit with the hands clear for 1.2 s.
fresh(); approach(); let_go()
S = task.SENSOR.name
CASES = [
    ("fitted where the clips took it, hands clear 1.2 s", {S: task.SENSOR.state(pos=task.REC["fit"]["pos"])}, 1.0,
     True),
    ("reported 2 mm from where the clips took it", {S: task.SENSOR.state(pos=task.REC["fit"]["pos"] + (0, .002, 0))},
     0.0, False),
    ("reported back in its case", {S: task.SENSOR.state(pos=task.SENSOR_A)}, 0.0, False),
    ("reported turned 5 degrees", {S: task.SENSOR.state(pos=task.REC["fit"]["pos"], quat_xyzw=q_about((0, 0, 1), 5))},
     0.0, False),
]
