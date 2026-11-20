"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so each case first
replays the step-side function observe() for 1.2 s of control steps with the given end state and the hands 50 mm away
(unless the case says otherwise), and then scores it. _run() afterwards drives observe() directly to check the 1 s
hold, the drop rule over the whole trial, unknown states, face sampling and the other lots, and raises if anything
scores wrongly.
"""

import math

import numpy as np

import lots
import parts
import task
from task import SCREEN, SEAT, TRAVEL

lots.use("A")
OUT = 0.0  # latch fully out (bolt 9.5 mm proud)
SHUT = TRAVEL  # tab squeezed to the lug (bolt 2 mm inside the edge)


def screen(dx=0.0, dy=0.0, dz=0.0, ql=OUT, qr=OUT, rot=None, vz=0.0):
    """The screen at its seat moved by dx (toward the window), dy, dz, with the latches at ql, qr."""
    r = np.eye(3) if rot is None else rot
    return SCREEN.state(pos=tuple(SEAT + np.array([dx, dy, dz])), quat_xyzw=tuple(parts._matrix_to_quat(r)),
                        joints={"latch_l": ql, "latch_r": qr}, lin_vel=(0.0, 0.0, vz))


def replay(objects, gap=0.05, seconds=1.2):
    """Fill the trial record as the simulator would after ``seconds`` at 20 Hz with these states."""
    task.reset_record()
    for k in range(int(round(seconds * 20)) + 1):
        task.observe(objects, gap, k / 20.0)
    return task.score(objects)


def task_start():
    return {o.name: o.state() for o in task.OBJECTS}


def case(label, st, want, gap=0.05):
    objs = {"screen": st}
    replay({**task_start(), **objs}, gap=gap)
    return (label, objs, want, want >= 1.0)


class _Lazy(list):
    """The checker scores each case right after building it, so each case is replayed just before it is checked."""

    def __init__(self, makers):
        super().__init__(range(len(makers)))
        self._makers = makers

    def __iter__(self):
        for m in self._makers:
            yield m()

    def __getitem__(self, i):
        return self._makers[i]()


def tipped(deg, axis=(0.0, 1.0, 0.0)):
    return parts._rot(axis, math.radians(deg))


MAKERS = [
    lambda: case("seated on all four sides, both latches out in their holes, hands clear, 1 s still", screen(), 1.0),
    lambda: case("seated, both bolts 3.0 mm into their holes (latches 3.5 mm in: exact limit)",
                 screen(ql=0.0035, qr=0.0035), 1.0),
    lambda: case("seated, left bolt only 2.5 mm into its hole", screen(ql=0.0040), 0.5),
    lambda: case("seated, left latch held squeezed shut", screen(ql=SHUT), 0.5),
    lambda: case("seated, both latches squeezed shut (nothing holds it)", screen(ql=SHUT, qr=SHUT), 0.0),
    lambda: case("2.0 mm off the stop (exact limit)", screen(dx=-0.0020), 1.0),
    lambda: case("2.5 mm off the stop, standing in the opening", screen(dx=-0.0025), 0.0),
    lambda: case("bottom 1.5 mm above the sill (exact limit)", screen(dz=0.0015), 1.0),
    lambda: case("bottom 2.0 mm above the sill (hung on the bolts)", screen(dz=0.0020), 0.0),
    lambda: case("leaning 2 degrees in from the stop at the top (top 12 mm off the stop)",
                 screen(rot=tipped(-2.0)), 0.0),
    lambda: case("shifted 2.9 mm toward the left jamb, both latched", screen(dy=0.0029), 1.0),
    lambda: case("racked 1 degree in the window's plane", screen(rot=tipped(1.0, (1.0, 0.0, 0.0))), 0.0),
    lambda: case("still leaning on the stool where it started", SCREEN.state(), 0.0),
    lambda: case("seated and latched, a fingertip 4 mm from a tab (still held)", screen(), 0.0, gap=0.004),
    lambda: case("seated and latched, nearest hand point 5.5 mm away", screen(), 1.0, gap=0.0055),
    lambda: case("upside down in the opening (latches at the top)", screen(dz=task.H, rot=tipped(180.0, (1, 0, 0))),
                 0.0),
    lambda: case("turned round, mesh face to the window (housings behind)",
                 screen(dx=-0.0, rot=tipped(180.0, (0, 0, 1))), 0.0),
]
CASES = _Lazy(MAKERS)


def _run():
    start = task_start()
    good = {**start, "screen": screen()}
    assert replay(good, seconds=0.9) == 0.0, "0.9 s of stillness must not score"
    assert replay(good, seconds=1.0) == 1.0, "1.0 s of stillness must score"
    # a drop at any step ends credit for the trial, even if the screen is put back
    task.reset_record()
    for k in range(40):
        st = screen(dz=0.10 - (0.04 if k == 4 else 0.0)) if k in (3, 4) else screen()
        task.observe({**good, "screen": st}, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "a 4 cm fall in one step with the hands away must end credit"
    task.reset_record()
    held = {3: 0.10, 4: 0.06, 5: 0.03, 6: 0.0}
    for k in range(40):
        st = screen(dz=held[k]) if k in held else screen()
        task.observe({**good, "screen": st}, 0.01 if k in held else 0.05, k / 20.0)
    assert task.score(good) == 1.0, "lowering it fast while holding it is not a drop"
    # movement during the hold restarts it
    task.reset_record()
    for k in range(25):
        task.observe({**good, "screen": screen(dx=-0.0015 if k == 15 else 0.0)}, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "a 1.5 mm movement 0.45 s before the end must restart the hold"
    # unknown hands or screen at any step: no credit
    task.reset_record()
    for k in range(30):
        task.observe(good, None if k == 3 else 0.05, k / 20.0)
    assert task.score(good) == 0.0, "an unknown hand state must give no credit"
    task.reset_record()
    bad = {**good, "screen": SCREEN.state(pos=(float("nan"), 0.0, 0.0))}
    for k in range(30):
        task.observe(bad if k == 2 else good, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "an unknown screen state must give no credit"
    # the hand gap: a point 0.5 mm off a tab's outer face is caught; a flat hull face is sampled
    st = screen()
    tab = SEAT + np.array([task.TAB_X, task.Y_TAB + task.TAB[1] / 2 + 0.0005, task.B_Z])
    g = task.hand_gap(good, np.array([tab]))
    assert 0.0004 < g < 0.0006, g
    from yam_arena.analysis import Hull
    cube = np.array([[x, y, z] for x in (0, 0.03) for y in (0, 0.03) for z in (0, 0.03)], float)
    planes = np.array([[1, 0, 0, 0.03], [-1, 0, 0, 0], [0, 1, 0, 0.03], [0, -1, 0, 0], [0, 0, 1, 0.03], [0, 0, -1, 0]],
                      float)
    face = task._hull_face_points(Hull(vertices=cube, planes=planes))
    assert len(face) > 80, "hull faces must be sampled"
    assert np.min(np.linalg.norm(face - np.array([0.015, 0.015, 0.03]), axis=1)) < 0.003, "face middles sampled"
    # other lots: same scoring of the seated screen
    for name in ("B", "C"):
        lots.use(name)
        assert task.tier(good) == 1.0
    lots.use("A")
    task.reset_record()
    del st


_run()
