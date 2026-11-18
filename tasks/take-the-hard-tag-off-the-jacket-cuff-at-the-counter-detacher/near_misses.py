"""Near misses and unintended strategies for taking the hard tag off the cuff.

The score rests on a whole-trial record (pin travel counted only while the dome sits in the detacher's well, the
let-go, the 1 s still hold, unknown states), so each case drives the scorer's own recorder (`task.record`, which the
simulator calls every control step) through a sequence of tag and cuff poses and hand points, then reads the score.

Poses are built from the task's own kinematics: the tag either seated flat in the well or lying off it, and the cuff
placed so that its pin is drawn a given distance along the tag's pin axis."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task  # noqa: E402

DT = 0.05
B = task.B
FAR = np.array([[0.05, 0.45, B + 0.30], [0.05, -0.45, B + 0.30]])
FAILS: list[str] = []
I = (0.0, 0.0, 0.0, 1.0)
SEAT = (task.D[0], task.D[1], B)  # the tag flat on the counter, dome in the well
OFF_WELL = (task.D[0] + 0.0, task.D[1] - 0.120, B)  # the tag flat on the counter beside the cuff (not in the well)
SOMEWHERE_UP = (task.D[0], task.D[1], B + 0.040)  # the tag lifted 40 mm out of the well, level


def tag_at(pos, q=I):
    return task.TAG.state(pos=pos, quat_xyzw=q)


def cuff_drawn(tag_st, drawn, aside=(0.0, 0.0, 0.0)):
    """The cuff with its pin drawn ``drawn`` metres out along the tag's axis (cuff level), then moved by ``aside``."""
    r = task._rot(tag_st.quat_xyzw)
    hinge = np.asarray(tag_st.pos, float) + r @ np.array([0.0, 0.0, task.HZ + drawn]) + np.asarray(aside, float)
    return task.CUFF.state(pos=hinge, quat_xyzw=tag_st.quat_xyzw)


def world(tag_pos, drawn, aside=(0.0, 0.0, 0.0), q=I):
    t = tag_at(tag_pos, q)
    return {"counter": task.COUNTER.state(), "tag": t, "cuff": cuff_drawn(t, drawn, aside)}


def run(steps, state, hands=FAR, reset=True):
    if reset:
        task.reset_record()
    for k in range(steps):
        s = state(k) if callable(state) else state
        h = hands(k) if callable(hands) else hands
        task.record(s, h, DT)


def pull(tag_pos, a, b, n=10, hands=FAR, reset=False, aside=(0.0, 0.0, 0.0)):
    """Draw the pin from a to b metres in n steps, the tag at ``tag_pos``."""
    run(n, lambda k: world(tag_pos, a + (b - a) * (k + 1) / n, aside), hands=hands, reset=reset)


AWAY = (0.0, -0.07, -0.025)  # the cuff carried 70 mm aside and set down: pin out and clear
FULL = 0.025


def demo_like(still=24, hands=FAR):
    run(5, world(SEAT, 0.0))  # seated, nothing drawn
    pull(SEAT, 0.0, FULL)  # drawn 25 mm while seated
    run(still, world(SEAT, FULL, AWAY), hands=hands, reset=False)


def near(tag_pos, dz):
    """A hand point dz metres above the tag's top face at its free end (plus the far points)."""
    return np.vstack([FAR, [tag_pos[0], tag_pos[1] + 0.040, tag_pos[2] + task.TAG_H + dz]])


def case(label, setup, want):
    setup()
    got = task.score(None)
    good = abs(got - want) <= 1e-9
    if not good:
        FAILS.append(f"{label}: got {got}, want {want}")
    print(f"{'ok  ' if good else 'FAIL'} {label}: {got}")


HIST_CASES = [
    ("start layout, nothing done", lambda: run(30, {"counter": task.COUNTER.state(), "tag": task.TAG.state(),
                                                    "cuff": task.CUFF.state()}), 0.0),
    ("tag seated, pin drawn 25 mm in the well, cuff set aside, hands clear, still 1.2 s", lambda: demo_like(), 1.0),
    ("the same, still only 0.5 s", lambda: demo_like(still=10), 0.75),
    ("the same, a hand 3 mm over the tag", lambda: demo_like(hands=near(SEAT, 0.003)), 0.75),
    ("the same, a hand 6 mm over the tag", lambda: demo_like(hands=near(SEAT, 0.006)), 1.0),
    ("pin out but the cuff still held (hand 2 mm off the cuff)", lambda: demo_like(
        hands=np.vstack([FAR, [SEAT[0], SEAT[1] - 0.07 - 0.052, B - 0.025 + 0.0235 + 0.002]])), 0.5),
    ("pin out, cuff let go with its tip 5 mm from the pin hole", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, FULL), run(24, world(SEAT, task.DEPTH0 + 0.005), reset=False)),
     0.5),
    ("pin out, cuff let go with its tip 10 mm above the pin hole (limit)", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, FULL),
        run(24, world(SEAT, task.DEPTH0 + task.CLEAR_TAG), reset=False)), 1.0),
    ("drawn 4.9 mm in the well, then stopped", lambda: (run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.0049)), 0.0),
    ("drawn 5.0 mm in the well, then stopped", lambda: (run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.005)), 0.25),
    ("drawn 15.4 mm in the well: not out", lambda: (run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.0154)), 0.25),
    ("pull without the detacher: tag off the well, pin drawn all the way", lambda: (
        run(5, world(OFF_WELL, 0.0)), pull(OFF_WELL, 0.0, FULL),
        run(24, world(OFF_WELL, FULL, AWAY), reset=False)), 0.0),
    ("late pull: seated, the tag lifted out of the well, then the pin drawn", lambda: (
        run(5, world(SEAT, 0.0)), run(5, world(SOMEWHERE_UP, 0.0), reset=False), pull(SOMEWHERE_UP, 0.0, FULL),
        run(24, world(SOMEWHERE_UP, FULL, AWAY), reset=False)), 0.0),
    ("late pull, then the tag put back in the well with the pin already out", lambda: (
        run(5, world(SEAT, 0.0)), run(5, world(OFF_WELL, 0.0), reset=False), pull(OFF_WELL, 0.0, FULL),
        run(5, world(SEAT, FULL), reset=False), run(24, world(SEAT, FULL, AWAY), reset=False)), 0.0),
    ("drawn 8 mm in the well, the tag taken out, the rest drawn outside", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.008), run(3, world(SOMEWHERE_UP, 0.008), reset=False),
        pull(SOMEWHERE_UP, 0.008, FULL), run(24, world(SOMEWHERE_UP, FULL, AWAY), reset=False)), 0.25),
    ("drawn 8 mm in the well, out, reseated and finished in the well", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.008), run(3, world(SOMEWHERE_UP, 0.008), reset=False),
        run(3, world(SEAT, 0.008), reset=False), pull(SEAT, 0.008, FULL),
        run(24, world(SEAT, FULL, AWAY), reset=False)), 1.0),
    ("drawn 10 mm in the well, then pushed 8 mm back in", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.010), pull(SEAT, 0.010, 0.002)), 0.0),
    ("drawn 10 mm in the well, out of the well, pushed back 6 mm", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.010), run(2, world(SOMEWHERE_UP, 0.010), reset=False),
        pull(SOMEWHERE_UP, 0.010, 0.004)), 0.0),
    ("dome 4.5 mm off the well's axis (limit): counts as seated", lambda: (
        run(5, world((SEAT[0] + 0.0045, SEAT[1], B), 0.0)), pull((SEAT[0] + 0.0045, SEAT[1], B), 0.0, FULL),
        run(24, world((SEAT[0] + 0.0045, SEAT[1], B), FULL, AWAY), reset=False)), 1.0),
    ("dome 4.6 mm off the well's axis: not seated", lambda: (
        run(5, world((SEAT[0] + 0.0046, SEAT[1], B), 0.0)), pull((SEAT[0] + 0.0046, SEAT[1], B), 0.0, FULL),
        run(24, world((SEAT[0] + 0.0046, SEAT[1], B), FULL, AWAY), reset=False)), 0.0),
    ("tag leaning 9 degrees over the well, pin drawn: not seated", lambda: run(
        10, lambda k: world(SEAT, FULL * (k + 1) / 10, q=task._qx(math.radians(9)))), 0.0),
    ("full run, but the tag knocked off the counter once", lambda: (
        demo_like(), run(1, world((task.D[0], task.D[1], B - 0.05), FULL, AWAY), reset=False),
        run(24, world(SEAT, FULL, AWAY), reset=False)), 0.0),
    ("hand state unknown for one step", lambda: (
        demo_like(), run(1, world(SEAT, FULL, AWAY), hands=None, reset=False)), 0.0),
    ("object state unknown for one step", lambda: (demo_like(), run(1, None, reset=False)), 0.0),
]

for _label, _setup, _want in HIST_CASES:
    case(_label, _setup, _want)
print(f"{len(HIST_CASES) - len(FAILS)}/{len(HIST_CASES)} as expected")
if FAILS:
    raise SystemExit(f"{len(FAILS)} near-miss case(s) failed: {FAILS}")

demo_like()  # leave a valid full-score history for check.py's end-state case
CASES = [
    ("end state of a valid history: pin out in the well, cuff aside, hands clear, still", {}, 1.0, True),
]
