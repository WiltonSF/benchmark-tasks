"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-trial record what the step callback records for a trial in which no hand
came within 1 mm of the lid, the hands ended 50 mm from the laptop, and the end state has held still for 1 s. The
cases then vary the end state: where the laptop rests, how level, how far the lid has turned. _run() afterwards
drives the step-side function observe() directly, step by step, to check the whole-trial rules (a hand on the lid,
the lid turned past 25 degrees, the laptop tilted past 25 degrees, down on the bare desk, off the desk), the 1 s
hold, the 30 mm let-go distance and the unknown-arm rule in both lots, and raises if anything scores wrongly.
"""

import math

import numpy as np

import lots
import task
from task import BOOKS_C, BOOKS_TOP, STAND_TOP, TARGET, TOP

lots.use("A")
CLEAR = (0.05, 0.05)
LID_TOUCH = (0.0005, 0.0005)


def qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def q_tilt(deg, yaw=0.0, axis="y"):
    """Yaw first, then a lean of ``deg`` about the world x or y axis."""
    a = math.radians(deg) / 2
    qa = np.array([math.sin(a), 0.0, 0.0, math.cos(a)]) if axis == "x" else np.array([0.0, math.sin(a), 0.0,
                                                                                         math.cos(a)])
    x1, y1, z1, w1 = qa
    x2, y2, z2, w2 = qz(yaw)
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def lot_of(name):
    return lots.all_lots()[lots.index(name)]


def laptop(pos, yaw=0.0, lid=0.0, quat=None):
    return task.LAPTOP.state(pos=pos, quat_xyzw=quat if quat is not None else qz(yaw), joints={"lid": lid})


def start(lot="A"):
    lt = lot_of(lot)
    return {"laptop": laptop(lt.pos("laptop"), lt.yaw_of("laptop")), "books": task.BOOKS.state(),
            "stand": task.STAND.state()}


def placed(lot="A", dx=0.0, dy=0.0, dz=0.0, lid=0.0, quat=None):
    s = start(lot)
    s["laptop"] = laptop((TARGET[0] + dx, TARGET[1] + dy, STAND_TOP + dz), 0.0, lid, quat)
    return s


def held_over(lot="A", z=0.04):
    s = start(lot)
    s["laptop"] = laptop((TARGET[0], TARGET[1], STAND_TOP + z), 0.0)
    return s


def lifted(lot="A", z=0.03):
    s = start(lot)
    lt = lot_of(lot)
    x, y, _ = lt.pos("laptop")
    s["laptop"] = laptop((x, y, BOOKS_TOP + z), lt.yaw_of("laptop"))
    return s


def preset():
    task.reset_record()
    task._rec.update(steps=400, gaps=CLEAR, since=18.0, held=True, t=20.0, lifted=True)


preset()
CASES = [
    ("laptop on the stand, front 55 mm over its front edge, hands clear, still 1 s", placed(), 1.0, True),
    ("35 mm further back on the stand (front overhang 20 mm)", placed(dx=0.035), 1.0, True),
    ("36 mm further back (front overhang 19 mm)", placed(dx=0.036), 0.5, False),
    ("centred front to back on the stand (front overhang 29 mm)", placed(dx=task.STAND_C[0] - TARGET[0]), 1.0, True),
    ("35 mm further forward (front overhang 90 mm)", placed(dx=-0.035), 1.0, True),
    ("36 mm further forward (front overhang 91 mm)", placed(dx=-0.036), 0.5, False),
    ("on the stand turned 15 degrees", placed(quat=qz(math.radians(15.0))), 1.0, True),
    ("on the stand turned 16 degrees", placed(quat=qz(math.radians(16.0))), 0.5, False),
    ("on the stand the wrong way round (screen toward the robot)", placed(quat=qz(math.pi)), 0.5, False),
    ("25 mm to the left of the stand's middle", placed(dy=0.025), 1.0, True),
    ("26 mm to the left", placed(dy=0.026), 0.5, False),
    ("held 4 mm above the stand", placed(dz=0.004), 0.5, False),
    ("resting on the stand tilted 5 degrees", placed(quat=q_tilt(5.0)), 1.0, True),
    ("on the stand but tilted 7 degrees (propped on something)", placed(quat=q_tilt(7.0)), 0.5, False),
    ("on the stand, lid 20 degrees further open", placed(lid=math.radians(20.0)), 1.0, True),
    ("on the stand, lid pushed 30 degrees toward closed", placed(lid=math.radians(-30.0)), 0.0, False),
    ("on the stand, lid shut (laptop asleep, upload stopped)", placed(lid=-task.OPEN), 0.0, False),
    ("held 40 mm over the stand", held_over(), 0.5, False),
    ("lifted 30 mm off the books", lifted(), 0.25, False),
    ("lifted only 5 mm off the books", lifted(z=0.005), 0.0, False),
    ("still on the books", start(), 0.0, False),
]


def _expect(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"{label}: scored {got}, expected {want}")
    print(f"ok    {label}: {got}")


def _run(lot, steps, label, want):
    """Drive observe() over ``steps``: (objects, gaps) per 0.05 s step, then score the last scene."""
    lots.use(lot)
    task.reset_record()
    t, last = 0.0, None
    for objects, g in steps:
        t += 0.05
        last = objects
        task.observe(objects, g, t)
    _expect(f"lot {lot}: {label}", float(task.score(last)), want)


def seq(*parts):
    out = []
    for n, objects, g in parts:
        out += [(objects, g)] * n
    return out


def _checks():
    for lot in ("A", "B"):
        s0, up, ov, pl = start(lot), lifted(lot), held_over(lot), placed(lot)
        dn = []  # lowered from 40 mm above the stand onto it, 5 mm a step (0.1 m/s)
        for k in range(1, 8):
            d = dict(ov)
            d["laptop"] = laptop((TARGET[0], TARGET[1], STAND_TOP + 0.04 - 0.005 * k), 0.0)
            dn.append((1, d, CLEAR))
        _run(lot, seq((40, s0, CLEAR)), "nothing done", 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR)), "lifted by the base off the books", 0.25)
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), (20, ov, CLEAR)), "carried over the stand", 0.5)
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), (20, ov, CLEAR), *dn, (19, pl, CLEAR)),
             "set on the stand, still 0.95 s", 0.75)
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), (20, ov, CLEAR), *dn, (22, pl, CLEAR)),
             "set on the stand, still 1.1 s", 1.0)
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), (20, ov, CLEAR), *dn, (30, pl, (0.05, 0.025))),
             "set on the stand, a finger still 25 mm from the base", 0.75)
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), (20, ov, CLEAR), *dn, (30, pl, (0.0011, 0.0011))),
             "the closest hand point 1.1 mm from the lid all through the end (not let go)", 0.75)
        # the naive errand: the laptop held by its display and carried to the stand
        _run(lot, seq((10, s0, CLEAR), (5, s0, LID_TOUCH), (20, up, LID_TOUCH), (20, ov, LID_TOUCH),
                      (30, pl, CLEAR)), "naive: held by the lid's edges, carried and set on the stand", 0.0)
        _run(lot, seq((10, s0, CLEAR), (1, s0, (0.0009, 0.0009)), (20, up, CLEAR), (20, ov, CLEAR), *dn, (30, pl, CLEAR)),
             "a fingertip brushed the screen once (0.9 mm) before the base grip, then placed", 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), (20, ov, CLEAR), *dn, (1, pl, (0.0, 0.0)), (30, pl, CLEAR)),
             "a gripper knocked the lid once while letting go", 0.0)
        shut = dict(ov)
        shut["laptop"] = laptop(ov["laptop"].pos, 0.0, lid=math.radians(-26.0))
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), (2, shut, CLEAR), (20, ov, CLEAR), *dn, (30, pl, CLEAR)),
             "lid swung 26 degrees toward closed while carried, then back, placed", 0.0)
        tipped = dict(up)
        tipped["laptop"] = laptop(up["laptop"].pos, 0.0, quat=q_tilt(26.0, lot_of(lot).yaw_of("laptop")))
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), (2, tipped, CLEAR), (20, ov, CLEAR), *dn, (30, pl, CLEAR)),
             "laptop tilted 26 degrees in the grip, then placed", 0.0)
        tilted_ok = dict(up)
        tilted_ok["laptop"] = laptop(up["laptop"].pos, 0.0, quat=q_tilt(20.0, lot_of(lot).yaw_of("laptop")))
        _run(lot, seq((10, s0, CLEAR), (20, tilted_ok, CLEAR), (20, ov, CLEAR), *dn, (30, pl, CLEAR)),
             "laptop tilted 20 degrees in the grip, then placed", 1.0)
        down = dict(up)
        down["laptop"] = laptop((0.30, 0.0, TOP), 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), (3, down, CLEAR), (20, ov, CLEAR), *dn, (30, pl, CLEAR)),
             "laptop set down on the bare desk for 0.15 s, then placed", 0.0)
        drop = []
        for k in range(4):  # let go 60 mm above the stand: it falls 20 mm a step
            d = dict(ov)
            d["laptop"] = laptop((TARGET[0], TARGET[1], STAND_TOP + 0.06 - 0.02 * k), 0.0)
            drop.append((1, d, CLEAR))
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), (20, held_over(lot, z=0.06), CLEAR), *drop, (30, pl, CLEAR)),
             "let go 60 mm above the stand (falls at 0.4 m/s), lands right", 0.0)
        slow = []
        for k in range(13):  # lowered 5 mm a step (0.1 m/s), the fastest the rule allows by a margin
            d = dict(ov)
            d["laptop"] = laptop((TARGET[0], TARGET[1], STAND_TOP + 0.06 - 0.005 * k), 0.0)
            slow.append((1, d, CLEAR))
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), *slow, (30, pl, CLEAR)),
             "lowered onto the stand at 0.1 m/s", 1.0)
        slid = dict(s0)  # slid 60 mm forward on the books, never lifted (its front now 115 mm past their edge)
        lt = lot_of(lot)
        x, y, _ = lt.pos("laptop")
        slid["laptop"] = laptop((x - 0.060, y, BOOKS_TOP), lt.yaw_of("laptop"))
        _run(lot, seq((10, s0, CLEAR), (30, slid, CLEAR)), "slid 60 mm forward on the books, never lifted", 0.0)
        across = dict(s0)  # resting across the books' right edge and the stand's left edge, middle over the gap
        gap_y = (BOOKS_C[1] - task.BOOK_W / 2 + task.STAND_C[1] + task.STAND_W / 2) / 2
        across["laptop"] = laptop((TARGET[0], gap_y, BOOKS_TOP + 0.0075), quat=q_tilt(-4.2, 0.0, axis="x"))
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), (30, across, CLEAR)),
             "set down resting across the books and the stand, let go", 0.0)
        gone = dict(up)
        gone["laptop"] = laptop((0.30, 0.0, TOP - 0.05), 0.0)
        _run(lot, seq((10, s0, CLEAR), (3, gone, CLEAR)), "laptop dropped off the desk", 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), (20, ov, CLEAR), (3, pl, None), (30, pl, CLEAR)),
             "arms unreadable for some steps", 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, up, CLEAR), (20, ov, CLEAR), *dn, (30, pl, CLEAR), (25, lifted(lot), CLEAR)),
             "placed, then taken back to above the books", 0.25)
    lots.use("A")
    preset()


if __name__ == "__main__":
    _checks()
    st = start("A")
    for label, changes, want, want_success in CASES:
        s = dict(st)
        s.update(changes)
        got = 1.0 if task.success(s) else float(task.score(s))
        _expect(label, got, want)
else:
    _checks()
