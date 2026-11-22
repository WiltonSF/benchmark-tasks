"""Near misses and unintended strategies for the charging-port lint task.

The lint rule is a history of brush poses, so the behaviour cases drive the scorer's own recorder (`task.record`,
which the simulator callback calls every control step with the phone's and the toothbrush's poses). Each behaviour
case starts from a fresh history, moves the toothbrush along a path (its bristle field's centre given in the port's
coordinates: u through the phone's thickness, v across its width, d depth below the mouth, all in mm), and scores
with the toothbrush set down at its start, both hands clear and everything still. CASES (run by the offline checker)
then checks end states on top of a full, correct history."""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

import task  # noqa: E402

PHONE = (np.asarray(task.PHONE_POS, float), task.R_PHONE)
RB = task.R_BRUSH
FIELD = np.array([0.0, task.HEAD_Y, task.TIP_Z])  # bristle field's centre (tips), brush frame
LOW = np.array([0.0, task.HEAD_Y - task.TUFT[1] / 2, task.TIP_Z])  # its lowest edge
START = (np.asarray(task.BRUSH.pos, float), RB)


def at_port(u, v, d):
    """Brush pose with its bristle field's centre at port coordinates (mm)."""
    w = task.from_port(np.array([u, v, d]) / 1000.0, PHONE)
    return (w - RB @ FIELD, RB)


def run(poses):
    for p in poses:
        task.record(PHONE, p)


def line(a, b, step=0.5):
    """Port-coordinate points (mm) from a to b, about ``step`` mm apart."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
    return [at_port(*(a + (b - a) * k / n)) for k in range(n + 1)]


def insert(u=0.0, v=0.0, d=4.6, sweep=1.0, reps=1, step=0.5, end=None):
    """Down the port's axis to depth d at ``sweep`` mm before the centre, one stroke to ``sweep`` mm past it and back
    to the centre (``reps`` times), then straight out 20 mm: the scripted demo's motion. With ``end``: one stroke to
    ``end`` and straight out from there."""
    path = line((u, v, -20.0), (u, v - sweep, d), step)
    if end is not None:
        run(path + line((u, v - sweep, d), (u, end, d), step) + line((u, end, d), (u, end, -20.0), step))
        return
    for _ in range(reps):
        path += line((u, v - sweep, d), (u, v + sweep, d), step) + line((u, v + sweep, d), (u, v, d), step)
    path += line((u, v, d), (u, v, -20.0), step)
    run(path)


def towel(dx=0.0, n=21):
    """The bristles' lowest edge 0.3 mm over the towel, wiped 1 mm a step (20 mm) across it."""
    for k in range(n):
        c = np.array([task.TOWEL_C[0] + 0.010 - 0.001 * k + dx, task.TOWEL_C[1] + 0.020, task.TOWEL_TOP + 0.0003])
        task.record(PHONE, (c - RB @ LOW, RB))


def touch_towel(steps=20):
    """The loaded bristles set on the towel and held still (no sliding)."""
    c = np.array([task.TOWEL_C[0], task.TOWEL_C[1], task.TOWEL_TOP + 0.0003])
    for _ in range(steps):
        task.record(PHONE, (c - RB @ LOW, RB))


def set_down():
    for _ in range(3):
        task.record(PHONE, START)


def released(hold=None):
    task._S.update(known=True, clear=True, supported=True, hold=task.HOLD_STEPS if hold is None else hold)


def start_states():
    return {o.name: o.state() for o in task.OBJECTS}


def full():
    insert()
    towel()
    set_down()


RESULTS = []


def case(label, setup, want, objects=None):
    task.reset_state()
    setup()
    if "known" not in getattr(setup, "keep", ()):
        released()
    for k, v in getattr(setup, "after", {}).items():
        task._S[k] = v
    st = start_states()
    st.update(objects or {})
    got = task.score(st)
    RESULTS.append((label, got, want))
    mark = "ok  " if abs(got - want) < 1e-9 else "FAIL"
    print(f"{mark} {label}: {got} (want {want}; left {task.remaining(PHONE, START)}, towel {task.on_towel()}, "
          f"modes {list(task._S['mode'])})")


def with_flags(fn, **flags):
    fn.after = flags
    return fn


case("bristles 4.6 mm in, one 2 mm stroke along the opening, out, wiped on the towel, brush laid down", full, 1.0)
case("nothing done", lambda: None, 0.0)
case("bristles pushed 4.6 mm in and pulled straight out (no sideways stroke), towel, laid down",
     lambda: (insert(sweep=0.0), towel(), set_down()), 0.0)
case("swept only 3.0 mm deep (above the lint's 4.0 mm reach)", lambda: (insert(d=3.0), towel(), set_down()), 0.0)
case("swept with the field centre 3.2 mm deep: its deepest tips (3.8 mm) short of the lint", lambda: (insert(d=3.2), towel(),
                                                                                       set_down()), 0.0)
case("swept 4.4 mm deep (reaches the lint)", lambda: (insert(d=4.4), towel(), set_down()), 1.0)
case("swept with the field centre 5.8 mm deep (deepest tips under the 6.5 mm packing line)", lambda: (insert(d=5.8), towel(), set_down()), 1.0)
case("jabbed 7.0 mm deep before sweeping: lint packed against the back wall",
     lambda: (insert(d=7.0), towel(), set_down()), 0.0)
case("brushed across the outside of the mouth (tips 0.5 mm above it)", lambda: (insert(d=-0.5), towel(), set_down()),
     0.0)
case("one short stroke, 0.3 mm each way (1.2 mm of travel, under 2 mm)", lambda: (insert(sweep=0.3), towel(), set_down()),
     0.0)
case("teleport-sized jumps (4 mm a step) while in the port", lambda: (run(
    line((0, 0, -20), (0, 0, 4.6)) + [at_port(0, 4.0 * (-1) ** k, 4.6) for k in range(30)]
    + line((0, 0, 4.6), (0, 0, -20))), towel(), set_down()), 0.0)
case("lint hooked, then the stroke carried on to 3.5 mm past centre and out from there: the two tufts dragged into "
     "the slot's end wall are scraped off and stay", lambda: (insert(end=3.5), towel(), set_down()), 0.5)
case("all lint out, the loaded bristles set on the towel and lifted without sliding (no wipe)",
     lambda: (insert(), touch_towel(), set_down()), 0.75)
case("lint wiped only 8 mm across the towel (under 10 mm)", lambda: (insert(), towel(n=9), set_down()), 0.75)
case("bristle field 15 mm off along the width: only the two tufts on that side come out, wiped on the towel",
     lambda: (insert(v=15.0), towel(), set_down()), 0.5)
case("field 5.5 mm toward the back glass and 15 mm along (centre 3.3 mm deep; its tilted tips reach 5.4 mm in the opening): one tuft out", lambda: (insert(u=5.5, v=15.0, d=3.3), towel(),
                                                                                 set_down()), 0.25)
case("all lint out on the bristles, toothbrush laid down without wiping it on the towel",
     lambda: (insert(), set_down()), 0.75)
case("all lint out, then the loaded bristles pushed back into the port: the lint goes back in",
     lambda: (insert(), run(line((0, 0, -20), (0, 0, 4.6)) + line((0, 0, 4.6), (0, 0, -20))), towel(), set_down()),
     0.0)
case("all lint out and wiped off, then the clean bristles go back in and out", lambda: (
    insert(), towel(), run(line((0, 0, -20), (0, 0, 4.6)) + line((0, 0, 4.6), (0, 0, -20))), set_down()), 1.0)
case("all lint out and wiped off, then the bristles pushed 7 mm in (the depth rule holds all trial)", lambda: (
    insert(), towel(), run(line((0, 0, -20), (0, 0, 7.0)) + line((0, 0, 7.0), (0, 0, -20))), set_down()), 0.0)
case("all lint out and wiped off, then the bristles pushed 12 mm in: the brush's rigid head enters the port end",
     lambda: (insert(), towel(), run(line((0, 0, -20), (0, 0, 12.0)) + line((0, 0, 12.0), (0, 0, -20))), set_down()),
     0.0)
case("lint wiped on the counter just off the towel's edge (not on it)", lambda: (insert(), towel(dx=0.08), set_down()),
     0.75)
case("full clean but a hand still within 30 mm (holding the brush)", with_flags(full, clear=False), 0.0)
case("full clean, still for only 19 steps (under 1 s)", with_flags(full, hold=task.HOLD_STEPS - 1), 0.0)
case("full clean, still for exactly 20 steps (1 s)", with_flags(full, hold=task.HOLD_STEPS), 1.0)
case("full clean but a hand or the brush's head went into the port end", with_flags(full, hard_in_port=True), 0.0)
case("full clean but the phone was knocked in its stand", with_flags(full, knocked=True), 0.0)
case("full clean but the brush or phone fell off the counter", with_flags(full, dropped=True), 0.0)
case("full clean but a pose or hand read failed (fail closed)", with_flags(full, broken=True), 0.0)
case("full clean but the scene was never read (fail closed)", with_flags(full, known=False), 0.0)

BAD = [r for r in RESULTS if abs(r[1] - r[2]) > 1e-9]
if BAD:
    raise SystemExit(f"near_misses: {len(BAD)} behaviour case(s) failed: {BAD}")

# End-state cases on top of a full, correct history (run in order by the offline checker).
task.reset_state()
full()
released()
B = task.BRUSH
P = task.PHONE
BP = np.asarray(B.pos, float)


def _turn_y(deg):
    m = task.R_PHONE @ task.parts._rot((0.0, 1.0, 0.0), np.radians(deg))
    return tuple(task.parts._matrix_to_quat(m))


CASES = [
    ("cleaned, wiped, toothbrush back where it lay, all still", {}, 1.0, True),
    ("toothbrush still sliding (2 cm/s)", {"toothbrush": B.state(lin_vel=(0.0, 0.02, 0.0))}, 0.0, False),
    ("toothbrush lying on the paper towel", {"toothbrush": B.state(pos=(0.40, -0.21, task.TOP + task.REST_H + 0.0015))},
     1.0, True),
    ("toothbrush 1 cm above the counter (not set down)", {"toothbrush": B.state(pos=tuple(BP + [0, 0, 0.010]))}, 0.0,
     False),
    ("toothbrush left lying across the phone's top", {"toothbrush": B.state(pos=(0.33, -0.12, task.TOP + 0.16))}, 0.0,
     False),
    ("toothbrush left on the stand's base against the phone",
     {"toothbrush": B.state(pos=(0.29, -0.125, task.TOP + 0.0045 + task.REST_H))}, 0.0, False),
    ("phone moved 8 mm in its stand", {"phone": P.state(pos=tuple(np.asarray(P.pos) + [0.0, 0.008, 0.0]))}, 0.0,
     False),
    ("phone tipped 6 degrees further back", {"phone": P.state(quat_xyzw=_turn_y(6.0))}, 0.0, False),
    ("phone settled 0.3 mm onto its ledge (normal)", {"phone": P.state(pos=tuple(
        np.asarray(P.pos) - task.R_PHONE @ [0.0, 0.0, 0.0003]))}, 1.0, True),
    ("cleaned, wiped, toothbrush back where it lay, still (after the cases above)", {}, 1.0, True),
]
