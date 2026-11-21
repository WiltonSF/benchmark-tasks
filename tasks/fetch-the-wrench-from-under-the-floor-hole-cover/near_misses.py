"""Near-successes and unintended strategies, scored offline by the checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-trial record what the step callback records once the grippers have been
more than 5 mm from the cover and the wrench, and the end state still, for 1 s. _run() then drives the step-side
function observe() directly to check the let-go distance (with face sampling), the 1 s hold, stillness, the
missing-measurement rule and lot B, and raises if anything scores wrongly. The headline check: a wrench held above
the tote's floor, inside the tote, never earns full credit, whether or not a gripper is still on it.
"""

import math

import numpy as np

import lots
import task
from task import COVER, DECK_TOP, TOP, TOTE, TOTE_FLOOR, TOTE_H, WRENCH, HEAD_T, GAP
from yam_arena.analysis import Hull

lots.use("A")
OX, OY = task.OPEN_XY
TX, TY, _ = task.TOTE_A
WZ = HEAD_T / 2  # the wrench's origin above whatever its heads rest on


def quat(axis, deg):
    a = math.radians(deg) / 2
    v = np.asarray(axis, float) * math.sin(a)
    return (float(v[0]), float(v[1]), float(v[2]), math.cos(a))


def mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def cover(dx=0.0, dy=0.0, dz=0.0, q=None):
    """The cover; by default seated over the opening (plate on the deck, cleats in the opening)."""
    return COVER.state(pos=(OX + dx, OY + dy, DECK_TOP + dz), quat_xyzw=q)


def wrench(x, y, z, q=None):
    return WRENCH.state(pos=(x, y, z), quat_xyzw=q)


def in_tote(dx=0.0, dy=0.0, dz=0.0, q=None):
    """The wrench flat on the tote's floor, along x, its heads down (dz: how far above the floor)."""
    return wrench(TX + dx, TY + dy, TOP + TOTE_FLOOR + WZ + dz, q)


def tote(dz=0.0, q=None, xy=(TX, TY)):
    return TOTE.state(pos=(xy[0], xy[1], TOP + dz), quat_xyzw=q)


def done(**kw):
    c, t = kw.pop("c", None) or cover(), kw.pop("t", None) or tote()
    return {"cover": c, "wrench": in_tote(**kw), "tote": t}


def preset():
    task.reset_record()
    task._rec.update(steps=50, gap_cover=0.05, gap_wrench=0.05, since=9.0, held=True, t=10.0, tier=1.0)


preset()
IX = task.TOTE_IN[0] / 2 - task.W_LEN / 2  # how far the wrench can sit off the tote's middle along x
IY = task.TOTE_IN[1] / 2 - task.JAW_OUT  # and along y (its widest part is the open end, 32 mm)
UP = quat((0, 0, 1), 90.0)
CASES = [
    ("cover seated, wrench flat on the tote floor, both let go, still 1 s", done(), 1.0, True),
    ("as above, cover turned a quarter turn (cleats still in the opening)", done(c=cover(q=UP)), 1.0, True),
    ("cover seated 10 mm off centre (all the play the cleats allow)", done(c=cover(dx=0.010, dy=0.010)), 1.0, True),
    ("cover resting 3 mm proud (exactly the limit)", done(c=cover(dz=0.003)), 1.0, True),
    ("cover resting 3.5 mm proud", done(c=cover(dz=0.0035)), 0.0, False),
    ("wrench flipped over in the tote", done(q=quat((1, 0, 0), 180.0)), 1.0, True),
    ("wrench against the tote's +x wall exactly", done(dx=IX), 1.0, True),
    ("wrench 1 mm through the tote's +x wall line", done(dx=IX + 0.001), 0.5, False),
    ("wrench against the tote's -y wall exactly", done(dy=-IY), 1.0, True),
    ("wrench 2 mm above the tote floor (the limit)", done(dz=0.002), 1.0, True),
    ("wrench held 3 mm above the tote floor", done(dz=0.003), 0.5, False),
    ("wrench held 30 mm above the tote floor, inside the walls", done(dz=0.030), 0.5, False),
    ("wrench held level with the rim inside the tote", done(dz=TOTE_H - TOTE_FLOOR), 0.5, False),
    ("wrench leaning on the +x wall, ring end on the floor", done(dx=-0.030, dz=0.035, q=quat((0, 1, 0), -25.0)),
     0.5, False),
    ("wrench lying across the -y rim", {"cover": cover(), "tote": tote(),
                                       "wrench": wrench(TX, TY - task.TOTE_IN[1] / 2, TOP + TOTE_H + WZ, UP)},
     0.5, False),
    ("wrench on the table beside the tote", {"cover": cover(), "tote": tote(),
                                            "wrench": wrench(TX, TY + 0.15, TOP + WZ)}, 0.5, False),
    ("wrench on the deck, hole covered", {"cover": cover(), "tote": tote(),
                                         "wrench": wrench(OX - 0.13, 0.17, DECK_TOP + WZ)}, 0.5, False),
    ("wrench left lying on top of the seated cover", {"cover": cover(), "tote": tote(),
                                                     "wrench": wrench(OX + 0.08, OY + 0.07,
                                                                      DECK_TOP + task.PLY_T + WZ)}, 0.5, False),
    ("wrench in the tote, tote tipped 10 degrees", done(t=tote(q=quat((1, 0, 0), 10.0))), 0.5, False),
    ("wrench in the tote, tote lifted 5 mm off the table", done(t=tote(dz=0.005), dz=0.005), 0.5, False),
    ("wrench in the tote, cover left off on the table", done(c=COVER.state(pos=(OX, 0.40, TOP + task.CLEAT_H))),
     0.0, False),
    ("wrench in the tote, cover set back on its cleats, not dropped in (askew, 45 degrees)",
     done(c=cover(dz=task.CLEAT_H, q=quat((0, 0, 1), 45.0))), 0.0, False),
    ("wrench in the tote, cover askew 20 degrees, standing on its cleats", done(c=cover(dz=task.CLEAT_H,
                                                                                         q=quat((0, 0, 1), 20.0))),
     0.0, False),
    ("wrench in the tote, cover slid 40 mm, one cleat on the deck (tilted)",
     done(c=cover(dy=0.040, dz=0.009, q=quat((1, 0, 0), 4.0))), 0.0, False),
    ("wrench in the tote, cover over the hole but lifted 1 cm", done(c=cover(dz=0.010)), 0.0, False),
    ("wrench in the tote, cover pushed half off the opening",
     done(c=COVER.state(pos=(OX, OY + 0.14, DECK_TOP + task.CLEAT_H))), 0.0, False),
    ("start: wrench in the opening, cover seated", {"cover": cover(), "tote": tote(),
                                                   "wrench": WRENCH.state()}, 0.0, False),
    ("wrench in the opening, cover off", {"cover": COVER.state(pos=(OX, 0.40, TOP + task.CLEAT_H)), "tote": tote(),
                                          "wrench": WRENCH.state()}, 0.0, False),
    ("wrench propped half out of the opening, cover seated over it",
     {"cover": cover(), "tote": tote(), "wrench": wrench(OX + 0.05, OY, TOP + 0.04, quat((0, 1, 0), -25.0))},
     0.0, False),
    ("wrench dropped off the table onto the floor, hole covered", {"cover": cover(), "tote": tote(),
                                                                 "wrench": wrench(0.60, 0.70, TOP - 0.70)}, 0.0, False),
    ("wrench out on the table, cover left off", {"cover": COVER.state(pos=(OX, 0.40, TOP + task.CLEAT_H)),
                                                 "tote": tote(), "wrench": wrench(TX, TY + 0.15, TOP + WZ)},
     0.0, False),
]


def box_hull(center, half):
    half = np.asarray(half, float)
    v = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half + np.asarray(center)
    planes = []
    for i in range(3):
        for sign in (-1.0, 1.0):
            n = np.zeros(3)
            n[i] = sign
            planes.append([*n, sign * center[i] + half[i]])
    return Hull(vertices=v, planes=np.array(planes))


def _check(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"{label}: score {got}, expected {want}")


def _hold(states, gaps, steps=25, dt=0.05):
    task.reset_record()
    for k in range(steps):
        s = states(k) if callable(states) else states
        g = gaps(k) if callable(gaps) else gaps
        task.observe(s, g, k * dt)
    return s


def _run() -> None:
    dt = 0.05
    end = done()
    far = (0.05, 0.05)

    # The hold: 1 s of the full end state, let go and still, measured step by step.
    task.reset_record()
    for k in range(20):
        task.observe(end, far, k * dt)
    _check("0.95 s into the hold", task.score(end), 0.0)
    task.observe(end, far, 20 * dt)
    _check("1.0 s into the hold", task.score(end), 1.0)
    task.observe(end, (0.05, 0.004), 21 * dt)  # a finger comes back to within 4 mm of the wrench
    _check("a finger 4 mm from the wrench", task.score(end), 0.0)
    for k in range(22, 50):
        task.observe(end, (0.0049, 0.05), k * dt)
    _check("a finger 4.9 mm from the cover: not let go", task.score(end), 0.0)
    for k in range(50, 80):
        task.observe(end, (0.0051, 0.0051), k * dt)
    _check("every gripper point just over 5 mm away", task.score(end), 1.0)

    # The reviewer's case: the wrench held inside the tote above its floor never earns full credit, held for any time.
    for dz in (0.003, 0.010, 0.030, 0.080):
        s = _hold(done(dz=dz), (0.05, 0.0), steps=60)
        _check(f"wrench held {dz * 1000:.0f} mm over the tote floor, gripper on it", task.score(s), 0.0)
        s = _hold(done(dz=dz), far, steps=60)  # even if the grippers were somehow reported clear
        _check(f"wrench {dz * 1000:.0f} mm over the tote floor, grippers reported clear", task.score(s), 0.5)
    s = _hold(done(), (0.05, 0.0), steps=60)
    _check("wrench gripped while touching the tote floor", task.score(s), 0.0)
    s = _hold(done(), (0.05, 0.0049), steps=60)
    _check("a fingertip 4.9 mm from the wrench on the tote floor", task.score(s), 0.0)

    # Stillness: the wrench still rocking 3 mm, or the cover still settling, earns nothing.
    s = _hold(lambda k: done(dx=0.003 * (k % 2)), far, steps=40)
    _check("wrench rocking 3 mm in the tote", task.score(s), 0.0)
    s = _hold(lambda k: done(c=cover(dz=0.0025 * (k % 2))), far, steps=40)
    _check("cover rocking 2.5 mm on the deck", task.score(s), 0.0)
    s = _hold(lambda k: done(dx=0.0015 * (k % 2)), far, steps=40)
    _check("wrench shifting 1.5 mm (under the 2 mm stillness limit)", task.score(s), 1.0)
    s = _hold(lambda k: done(dx=0.0015 * (k % 2), dy=0.0015 * (k % 2)), far, steps=40)
    _check("wrench shifting 1.5 mm in x and in y at once (2.1 mm)", task.score(s), 0.0)
    s = _hold(lambda k: done(t=tote(q=quat((0, 0, 1), 1.5 * (k % 2)))), far, steps=40)
    _check("tote turning 1.5 degrees back and forth", task.score(s), 0.0)

    # The tier changing restarts the second: 0.5 held, then the wrench lands in the tote.
    half = {"cover": cover(), "tote": tote(), "wrench": wrench(TX, TY + 0.15, TOP + WZ)}
    task.reset_record()
    for k in range(25):
        task.observe(half, far, k * dt)
    _check("hole covered, wrench on the table, 1.25 s", task.score(half), 0.5)
    task.observe(end, far, 25 * dt)
    _check("wrench just arrived in the tote", task.score(end), 0.0)
    for k in range(26, 46):
        task.observe(end, far, k * dt)
    _check("...and still for 1 s", task.score(end), 1.0)

    # Missing measurements fail closed for the whole trial.
    task.reset_record()
    task.observe(end, None, 0.0)
    for k in range(1, 40):
        task.observe(end, far, k * dt)
    _check("grippers unreadable at one step", task.score(end), 0.0)
    task.reset_record()
    _check("no step recorded at all", task.score(end), 0.0)

    # Let-go is measured on sampled hull faces, not only corners: a 120 x 80 mm face 0.5 mm over the handle's top
    # with every corner far away.
    c = end["cover"]
    top = DECK_TOP + task.HANDLE_TOP
    h = box_hull((OX, OY, top + 0.0005 + 0.01), (0.060, 0.040, 0.010))
    corners_only = task.gap_to(COVER, c, h.vertices)
    sampled = task.gap_to(COVER, c, task._face_points(h))
    if not (corners_only > task.CLEAR and sampled < task.CLEAR):
        raise AssertionError(f"face sampling: corners {corners_only:.4f} m, sampled {sampled:.4f} m")

    # Lot B: the tote on the robot's left and the cover turned a quarter turn; the end state scores 1.0 there, and a
    # wrench laid where lot A's tote stood does not count.
    lots.use("B")
    tb = tote(xy=(TX, -TY))
    endb = {"cover": cover(q=UP), "tote": tb, "wrench": wrench(TX, -TY, TOP + TOTE_FLOOR + WZ)}
    s = _hold(endb, far)
    _check("lot B: wrench in the tote on the left, cover seated", task.score(s), 1.0)
    habit = {"cover": cover(q=UP), "tote": tb, "wrench": wrench(TX, TY, TOP + WZ)}
    s = _hold(habit, far)
    _check("lot B: wrench laid where lot A's tote was", task.score(s), 0.5)
    lots.use("A")
    preset()


_run()
assert GAP > 0
