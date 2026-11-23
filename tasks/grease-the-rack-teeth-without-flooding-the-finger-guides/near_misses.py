"""Near misses and unintended strategies for the rack-greasing task.

Greasing is a history of brush contacts, so the behaviour cases drive the scorer's own recorder (`task.record_brush`,
which the simulator callback calls every control step with the force the simulator reports between the brush and
each rack, and the poses of the brush and the racks). Each behaviour case starts from a fresh history and checks
score() with both hands clear, the racks lying where they started in the tray, the brush back across the tray and
everything still. CASES (run by the offline checker) then checks end states on top of a fully greased history."""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

import task  # noqa: E402

RNG = np.random.default_rng(11)
RACK0 = {n: p.state() for n, p in zip(task.RACK_NAMES, task.RACKS)}
TUFT_Z = task.TUFT_AT[2]  # the bristle block's middle above the brush's origin
HALF_W = task.FLANGE_W / 2
BOTTOM = {n: float(RACK0[n].pos[2]) - task.RACK_H / 2 for n in task.RACK_NAMES}


def start_states():
    return {o.name: o.state() for o in task.OBJECTS}


def released(hold=None):
    """Both hands 30 mm clear and the scene still for the 1 s hold (what the simulator callback keeps)."""
    task._S.update(known=True, clear=True, hold=task.HOLD_STEPS if hold is None else hold)


def brush_at(tuft_centre, yaw=0.0):
    """The brush's state with its bristle block's middle at ``tuft_centre`` (world), bristles up."""
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return task.BRUSH.state(pos=tuple(np.asarray(tuft_centre, float) - [0.0, 0.0, TUFT_Z]), quat_xyzw=q)


def feed(name, centres, force=2.0, forces=None, yaw=0.0):
    """Slide the bristle block's middle through ``centres`` (world), pressing rack ``name`` with ``force`` N."""
    for c in centres:
        task.record_brush(forces if forces is not None else {name: force}, brush_at(c, yaw), RACK0)
    task.no_brush()


def to_world(name, local):
    """A point in rack ``name``'s frame (x along it, y across it, z up from its middle) in the world."""
    p, r = np.asarray(RACK0[name].pos, float), task.parts._quat_to_matrix(RACK0[name].quat_xyzw)
    return p + r @ np.asarray(local, float)


def wiggle(name, local_centres):
    return [tuple(to_world(name, c)) for c in local_centres]


def side_centres(name, s, out=0.0, low=0.009, n=3):
    """The bristles' packed core beside side s (+1/-1 across the rack) with its long face ``out`` m off the flange
    (negative: pressed in), its base ``low`` m below the rack's back face, slid +/- 3 mm along the rack ``n`` times
    (as the demo does). The block's 20 mm is across the rack, its 56 mm along it."""
    y = s * (HALF_W + task.TUFT_CORE[0] / 2 + out)
    z = -task.RACK_H / 2 - low + task.TUFT[2] / 2
    pts = []
    for _ in range(n):
        xs = list(np.arange(0.0, 0.003 + 1e-9, 0.0005)) + list(np.arange(0.003, -0.003 - 1e-9, -0.0005)) + \
            list(np.arange(-0.003, 1e-9, 0.0005))
        pts += [(x, y, z) for x in xs]
    return wiggle(name, pts)


def back_centres(name, gap=0.0, half=0.006, n=2):
    """The bristles' packed core under the rack, its top ``gap`` m below the back face (negative: pressed in), slid
    +/- ``half`` m along the rack ``n`` times."""
    z = -task.RACK_H / 2 - gap - task.TUFT_CORE[2] + task.TUFT[2] / 2
    xs = []
    for _ in range(n):
        xs += list(np.arange(0.0, half + 1e-9, 0.001)) + list(np.arange(half, -half - 1e-9, -0.001)) + \
            list(np.arange(-half, 1e-9, 0.001))
    return wiggle(name, [(x, 0.0, z) for x in xs])


def top_centres(name, half=0.020, n=1):
    """The bristle block brushed along the rack's top (the tapped face), pressed down on it."""
    z = task.RACK_H / 2 - 0.0015 + task.TUFT[2] / 2
    xs = []
    for _ in range(n):
        xs += list(np.arange(-half, half + 1e-9, 0.001)) + list(np.arange(half, -half - 1e-9, -0.001))
    return wiggle(name, [(x, 0.0, z) for x in xs])


def grease(name, force=2.0, sides=(1, -1), back=True, **kw):
    for s in sides:
        feed(name, side_centres(name, s, **kw), force=force)
    if back:
        feed(name, back_centres(name), force=force)


def full(force=2.0):
    for n in task.RACK_NAMES:
        grease(n, force=force)


RESULTS = []


def case(label, setup, want, objects=None):
    task.reset_state()
    released()
    setup()
    st = start_states()
    st.update(objects or {})
    got = task.score(st)
    RESULTS.append((label, got, want))
    mark = "ok  " if abs(got - want) < 1e-9 else "FAIL"
    print(f"{mark} {label}: {got} (want {want}; zones {task.n_zones()}, racks full {task.racks_full()}, "
          f"tapped {task.tapped_greased()})")


FLOOR_BRUSH = brush_at((0.300, 0.0, task.FLOOR + task.HEAD[2] + TUFT_Z))  # brush lying bristles up on the floor

case("both racks greased all round, brush back across the tray, hands clear, still", full, 1.0)
case("nothing done", lambda: None, 0.0)
case("both racks greased, brush left lying on the tray floor", full, 0.75, {"brush": FLOOR_BRUSH})
case("one rack greased all round (the other untouched)", lambda: grease("rack_1"), 0.5)
case("one rack all round, the other's back only", lambda: (grease("rack_1"), grease("rack_2", sides=())), 0.5)
case("naive, racks left lying in the tray: the brush's head on the floor puts its tips over the tapped face",
     lambda: [grease(n, back=False, low=-0.012) for n in task.RACK_NAMES], 0.0)
case("sides of both racks held up, no back pass: the back face's middle third stays bare (4 zones)",
     lambda: [grease(n, back=False) for n in task.RACK_NAMES], 0.25)
case("the backs of both racks only (2 zones)", lambda: [grease(n, sides=()) for n in task.RACK_NAMES], 0.25)
case("one side of one rack only (1 zone)", lambda: grease("rack_1", sides=(1,), back=False), 0.0)
case("everything, then one pass along a rack's top: grease on the tapped face",
     lambda: (full(), feed("rack_1", top_centres("rack_1"))), 0.0)
case("a short dab on the tapped face before the rest (3 mm each way)",
     lambda: (feed("rack_2", top_centres("rack_2", half=0.0015, n=1)), full()), 0.0)
case("everything, then the block slid along the top 6 mm above the tapped face (out of reach): still dry",
     lambda: (full(), feed("rack_1", [(x, y, z + 0.0075) for x, y, z in top_centres("rack_1")])), 1.0)
case("too light: 0.19 N everywhere", lambda: full(force=0.19), 0.0)
case("exactly 0.2 N counts", lambda: full(force=0.2), 1.0)
case("a hard press (30 N) still greases", lambda: full(force=30.0), 1.0)
case("brush moved all round both racks, no contact reported (hover)", lambda: full(force=0.0), 0.0)
case("side passes 6 mm off the flanges and back passes 6 mm under (out of reach), force reported",
     lambda: [(feed(n, side_centres(n, 1, out=0.006)), feed(n, side_centres(n, -1, out=0.006)),
               feed(n, back_centres(n, gap=0.006))) for n in task.RACK_NAMES], 0.0)
case("force read on rack_2 while the brush slides along rack_1 (wrong rack)",
     lambda: (feed("rack_1", side_centres("rack_1", 1), forces={"rack_2": 2.0}),
              feed("rack_1", back_centres("rack_1"), forces={"rack_2": 2.0}),
              feed("rack_1", side_centres("rack_1", -1), forces={"rack_2": 2.0})), 0.0)
case("dabs: block pressed at 20 points along each face without sliding", lambda: [
    (task.record_brush({n: 2.0}, brush_at(c), RACK0), task.no_brush())
    for n in task.RACK_NAMES for f in (lambda m: side_centres(m, 1), lambda m: side_centres(m, -1), back_centres)
    for c in f(n)[::8]], 0.0)
case("block pressed and held still on each face, jittering 0.05 mm a step for 2,000 steps", lambda: [
    feed(n, [np.asarray(f(n)[0]) + RNG.uniform(-0.00005, 0.00005, 3) for _ in range(2000)])
    for n in task.RACK_NAMES for f in (lambda m: side_centres(m, 1), lambda m: side_centres(m, -1), back_centres)],
     0.0)
def creep(step):
    """Each face worked by a slow straight creep of ``step`` m a control step, 30 steps per pass."""
    for n in task.RACK_NAMES:
        for s_ in (1, -1):
            c0 = np.asarray(side_centres(n, s_)[0])
            feed(n, [tuple(c0 + to_world(n, (step * k, 0.0, 0.0)) - to_world(n, (0.0, 0.0, 0.0))) for k in range(30)])
        c0 = np.asarray(back_centres(n)[0])
        feed(n, [tuple(c0 + to_world(n, (step * k, 0.0, 0.0)) - to_world(n, (0.0, 0.0, 0.0))) for k in range(30)])


case("a creep at 2.9 mm/s (0.145 mm a step, 0.58 mm in 0.2 s) counts nothing", lambda: creep(0.000145), 0.0)
case("a stroke at 3.1 mm/s (0.155 mm a step) counts", lambda: creep(0.000155), 1.0)
case("teleport-sized steps (13 mm a step) are not brushing", lambda: [
    feed(n, [tuple(np.asarray(side_centres(n, s)[0]) + [0.0, 0.0065 * (-1) ** k, 0.0]) for k in range(40)])
    for n in task.RACK_NAMES for s in (1, -1)], 0.0)
case("side passes too high (bristle base 14 mm above the back face): the tips reach over onto the tapped face",
     lambda: [grease(n, low=-0.014) for n in task.RACK_NAMES], 0.0)
case("side passes with the bristle tips 0.4 mm above the rack's top edge: they touch the tapped face",
     lambda: [grease(n, low=0.0055) for n in task.RACK_NAMES], 0.0)
case("side passes with the bristle tips 0.6 mm below the rack's top edge: every face but the tapped one",
     lambda: [grease(n, low=0.0065) for n in task.RACK_NAMES], 1.0)
case("side passes far too low (bristle base 20 mm below the back face): upper stem sides missed, backs done",
     lambda: [grease(n, low=0.020) for n in task.RACK_NAMES], 0.25)
case("all six zones but a hand still within 30 mm (holding a rack)", lambda: (full(), task._S.update(clear=False)),
     0.0)
case("all six zones, let go, still for only 19 steps (under 1 s)", lambda: (full(), released(task.HOLD_STEPS - 1)),
     0.0)
case("all six zones, let go, still for exactly 20 steps (1 s)", lambda: (full(), released(task.HOLD_STEPS)), 1.0)
case("all six zones but a rack landed on the mat on the way", lambda: (full(), task._S.update(landed=True)), 0.0)
case("all six zones but the bristles touched the opened gripper or its parts",
     lambda: (full(), task._S.update(gripper_hit=True)), 0.0)
case("all six zones but a contact or hand read failed (fail closed)", lambda: (full(), task._S.update(broken=True)),
     0.0)
case("all six zones but the hands were never read (fail closed)", lambda: (full(), task._S.update(known=False)), 0.0)


def _missing(k):
    """Everything, then ``k`` strips of rack_1's +y side zone left dry (of its 48)."""
    full()
    z = np.flatnonzero(task.CELL_ZONE == 1)
    task._S["travel"]["rack_1"][z[:k]] = 0.0


case("a side zone with 44 of its 48 strips greased (92%) counts", lambda: _missing(4), 1.0)
case("a side zone with 43 of its 48 strips greased (90% is 43.2: not enough)", lambda: _missing(5), 0.5)

def pad_points(name, local_centre, half=(0.008, 0.0015, 0.006), step=0.001):
    """A finger pad's surface as points: a box of half sizes ``half`` (rack frame) centred at ``local_centre``."""
    g = [np.arange(-h, h + 1e-9, step) for h in half]
    pts = np.array([[a, b, c] for a in g[0] for b in g[1] for c in g[2]]) + np.asarray(local_centre, float)
    return np.array([to_world(name, q) for q in pts])


def end_grip(name):
    """Both pads on a rack's two end faces, as the demo holds it."""
    x = task.RACK_L / 2 + 0.0015
    return {"left_robot": np.concatenate([pad_points(name, (s * x, 0.0, 0.0), half=(0.0015, 0.008, 0.008))
                                          for s in (1, -1)])}


def side_grip(name, xs=(0.0,)):
    """16 mm pads pinching a rack across its flange sides (on the greased faces), centred at each x along it."""
    y = task.FLANGE_W / 2 + 0.0015
    return {"left_robot": np.concatenate([pad_points(name, (x, s * y, task.Z_FL0 + task.H_FLANGE / 2))
                                          for s in (1, -1) for x in xs])}


def on_tapped(name):
    """A pad laid on a rack's tapped face."""
    return {"left_robot": pad_points(name, (0.0, 0.0, task.Z_TAP + 0.0015), half=(0.008, 0.002, 0.0015))}


case("everything, then held by its two end faces the whole way (the demo's grip): nothing wiped",
     lambda: (full(), [task.record_hands(end_grip(n), RACK0) for n in task.RACK_NAMES]), 1.0)
case("everything, then rack_1 pinched once across its greased flanges: 4 of 48 strips a side wiped (under 10%)",
     lambda: (full(), task.record_hands(side_grip("rack_1"), RACK0)), 1.0)
case("everything, then rack_1 pinched at both ends across its greased flanges: 8 of 48 a side wiped, zones lost",
     lambda: (full(), task.record_hands(side_grip("rack_1", xs=(-0.02, 0.02)), RACK0)), 0.5)
case("rack_1 pinched across its flange sides before greasing, then everything: nothing to wipe",
     lambda: (task.record_hands(side_grip("rack_1"), RACK0), full()), 1.0)
case("everything, then a greasy finger (it pinched greased flanges) touches rack_2's tapped face",
     lambda: (full(), task.record_hands(side_grip("rack_1"), RACK0), task.record_hands(on_tapped("rack_2"), RACK0)),
     0.0)
case("everything, then a clean finger rests on a tapped face: no grease there",
     lambda: (full(), task.record_hands(on_tapped("rack_2"), RACK0)), 1.0)

BAD = [r for r in RESULTS if abs(r[1] - r[2]) > 1e-9]
if BAD:
    raise SystemExit(f"near_misses: {len(BAD)} behaviour case(s) failed: {BAD}")

# geometry checks of the whole-trial rules (the simulator callback applies them every step)
_mat_rack = task.RACKS[0].state(pos=(0.300, 0.150, task.MAT_TOP + task.RACK_H / 2))
assert task.rack_landed_outside(_mat_rack), "a rack on the mat beside the tray must count as landed"
assert not task.rack_landed_outside(RACK0["rack_1"]), "a rack in the tray has not landed"
assert not task.rack_landed_outside(task.RACKS[0].state(pos=(0.300, 0.05, task.TOP + 0.08 + task.RACK_H / 2))), \
    "a held rack over the tray has not landed"
assert not task.bristles_near_gripper(task.BRUSH.state()), "the brush at rest is clear of the gripper"
_b = task.HANDE_CYLS[1]
assert task.bristles_near_gripper(brush_at((_b[1][0] - _b[2] - task.TUFT[0] / 2 - 0.0005, _b[1][1], _b[1][2]))), \
    "bristles 0.5 mm from the gripper body count"
assert not task.bristles_near_gripper(brush_at((_b[1][0] - _b[2] - task.TUFT[0] / 2 - 0.003, _b[1][1], _b[1][2]))), \
    "bristles 3 mm from the gripper body do not"
_c = task.SYRINGE[0]
assert task.bristles_near_gripper(brush_at((_c[0], _c[1], _c[2] + task.SYRINGE[1] + task.TUFT[2] / 2 + 0.0005))), \
    "bristles resting on the syringe count"

# End-state cases on top of a fully greased, released history (run in order by the offline checker).
task.reset_state()
released()
full()
R1 = task.RACKS[0]
B0 = task.BRUSH.state()


def _q(axis, deg):
    a = np.radians(deg) / 2
    v = np.asarray(axis, float) * math.sin(a)
    return (float(v[0]), float(v[1]), float(v[2]), float(math.cos(a)))


P1 = np.asarray(RACK0["rack_1"].pos)
CASES = [
    ("greased, racks where they started in the tray, brush back across it, all still", {}, 1.0, True),
    ("rack_1 still sliding (2 cm/s)", {"rack_1": R1.state(lin_vel=(0.02, 0.0, 0.0))}, 0.0, False),
    ("rack_1 held 1 cm above the tray floor, still", {"rack_1": R1.state(pos=tuple(P1 + [0, 0, 0.010]))}, 0.0, False),
    ("rack_1 resting 2.5 mm high (within 3 mm of the floor)", {"rack_1": R1.state(pos=tuple(P1 + [0, 0, 0.0025]))},
     1.0, True),
    ("rack_1 on the mat beside the tray", {"rack_1": R1.state(pos=(0.300, 0.150, task.MAT_TOP + task.RACK_H / 2))},
     0.0, False),
    ("rack_1 lying on its side in the tray", {"rack_1": R1.state(
        pos=tuple(P1 + [0, 0, task.FLANGE_W / 2 - task.RACK_H / 2]), quat_xyzw=_q((1, 0, 0), 90))}, 1.0, True),
    ("rack_1 laid back down tapped face down in the tray", {"rack_1": R1.state(quat_xyzw=_q((1, 0, 0), 180))}, 1.0,
     True),
    ("rack_1 leaning in the tray at 30 degrees, a corner on the floor, all inside the walls", {"rack_1": R1.state(
        pos=(0.300, 0.070, task.FLOOR + 0.012), quat_xyzw=_q((1, 0, 0), 30))}, 1.0, True),
    ("rack_1 lying across the top of the tray's +y wall", {"rack_1": R1.state(
        pos=(0.300, 0.0825, task.WALL_TOP + task.RACK_H / 2))}, 0.0, False),
    ("brush lying on the tray floor", {"brush": FLOOR_BRUSH}, 0.75, False),
    ("brush across the walls but bristles down", {"brush": task.BRUSH.state(
        pos=(task.BRUSH_X, task.BRUSH_HEAD_Y, task.WALL_TOP), quat_xyzw=_q((0, 1, 0), 180))}, 0.75, False),
    ("brush turned along the tray, resting on its floor", {"brush": task.BRUSH.state(
        pos=(0.300, 0.0, task.FLOOR + task.HEAD[2]), quat_xyzw=_q((0, 0, 1), 90))}, 0.75, False),
    ("brush across the walls 3 mm lower than resting (within 3 mm)", {"brush": task.BRUSH.state(
        pos=tuple(np.asarray(B0.pos) - [0, 0, 0.003]))}, 1.0, True),
    ("brush still sliding", {"brush": task.BRUSH.state(lin_vel=(0.0, 0.02, 0.0))}, 0.0, False),
    ("everything back where it started, still (after the cases above)", {}, 1.0, True),
]
