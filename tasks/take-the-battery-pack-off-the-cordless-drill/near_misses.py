"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-trial record what the step callback records for a trial in which both
buttons were pressed in together, the grippers are 50 mm away and the end state has held for 1 s. _run() then drives
the step-side function observe() directly to check the button record, the 1 s hold, the let-go distance (with face
sampling), the whole-trial fall rule, the missing-measurement rule and lot B, and raises if any scores wrongly.
"""

import math

import numpy as np

import lots
import task
from task import DRILL, PACK, RAIL_Z, TOP, TRAY, TRAY_FLOOR
from yam_arena.analysis import Hull

lots.use("A")
SX, SY = task.SEAT
TX, TY = task.TRAY_XY


def quat(axis, deg):
    a = math.radians(deg) / 2
    v = np.asarray(axis, float) * math.sin(a)
    return (float(v[0]), float(v[1]), float(v[2]), math.cos(a))


def drill(dx=0.0, dy=0.0, dz=-RAIL_Z, q=None):
    """The drill; by default standing on the bench where it started (its foot on the bench, 42.5 mm lower)."""
    return DRILL.state(pos=(SX + dx, SY + dy, TOP + dz), quat_xyzw=q)


def pack(x, y, z, q=None, btn=(0.0, 0.0)):
    return PACK.state(pos=(x, y, z), quat_xyzw=q, joints={"btn_l": btn[0], "btn_r": btn[1]})


def in_tray(dx=0.0, dy=0.0, dz=0.0, q=None):
    return pack(TX + dx, TY + dy, TOP + TRAY_FLOOR + dz, q)


def done(**kw):
    return {"drill": kw.pop("d", drill()), "pack": in_tray(**kw)}


def preset(pressed=True):
    task.reset_record()
    task._rec.update(steps=50, pressed=pressed, gap_pack=0.05, gap_drill=0.05, since=9.0, held=True, t=10.0)


preset()
IX = task.TRAY_IN[0] / 2 - task.PACK_W / 2  # how far the pack can sit off the tray's middle along x
IY = task.TRAY_IN[1] / 2 - task.PACK_L / 2
CASES = [
    ("pack flat on the tray floor, drill standing, both let go, 1 s", done(), 1.0, True),
    ("pack against the tray's +x wall exactly", done(dx=IX), 1.0, True),
    ("pack 1 mm over the tray's +x wall", done(dx=IX + 0.001), 0.5, False),
    ("pack against the tray's -y wall exactly", done(dy=-IY), 1.0, True),
    ("pack 1 mm over the -y wall", done(dy=-IY - 0.001), 0.5, False),
    ("pack 1.5 mm above the floor (lowest corners)", done(dz=0.0015), 1.0, True),
    ("pack held 2 mm above the floor", done(dz=0.002), 0.5, False),
    ("pack held 3 cm above the tray", done(dz=0.030), 0.5, False),
    ("pack leaning 8 degrees on a wall", done(dz=0.006, q=quat((1, 0, 0), 8.0)), 0.5, False),
    ("pack on its side in the tray", done(dz=0.020, q=quat((0, 1, 0), 90.0)), 0.5, False),
    ("pack on the bench beside the tray", {"drill": drill(), "pack": pack(TX, 0.10, TOP)}, 0.5, False),
    ("pack in the tray, drill left lying 30 degrees over", done(d=drill(dz=0.0, q=quat((1, 0, 0), 30.0))), 0.5, False),
    ("pack in the tray, drill held 5 mm above the bench", done(d=drill(dz=-RAIL_Z + 0.005)), 0.5, False),
    ("pack in the tray, drill tilted 9 degrees on the bench", done(d=drill(dz=-RAIL_Z + 0.003,
                                                                           q=quat((0, 1, 0), 9.0))), 0.5, False),
    ("pack in the tray, tray tipped 10 degrees", {"drill": drill(), "pack": in_tray(),
                                                  "tray": TRAY.state(quat_xyzw=quat((1, 0, 0), 10.0))}, 0.5, False),
    ("pack and drill both carried into the tray together", {"drill": DRILL.state(pos=(TX, TY, TOP + TRAY_FLOOR)),
                                                            "pack": in_tray()}, 0.0, False),
    ("seated (do nothing; buttons pressed earlier)", {"drill": drill(dz=0.0), "pack": pack(SX, SY, TOP)}, 0.0, False),
    ("slid 9 mm along the rails", {"drill": drill(dz=0.0), "pack": pack(SX, SY + 0.009, TOP)}, 0.0, False),
    ("slid exactly 10 mm along the rails", {"drill": drill(dz=0.0), "pack": pack(SX, SY + 0.010, TOP)}, 0.25,
     False),
    ("slid 50 mm, still on the rails", {"drill": drill(dz=0.0), "pack": pack(SX, SY + 0.050, TOP)}, 0.25, False),
    ("pack 79 mm from its seat", {"drill": drill(dz=0.0), "pack": pack(SX, SY + 0.079, TOP)}, 0.25, False),
    ("pack exactly 80 mm from its seat (off the rails)", {"drill": drill(dz=0.0), "pack": pack(SX, SY + 0.080, TOP)},
     0.5, False),
    ("pack slid out along the bench, drill set down", {"drill": drill(), "pack": pack(SX, SY + 0.15, TOP)}, 0.5,
     False),
]


def _states(**kw):
    out = {"drill": drill(), "pack": in_tray(), "tray": TRAY.state()}
    out.update(kw)
    return out


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


def _run() -> None:
    dt = 0.05
    end = _states()
    far = (0.05, 0.05)

    # The hold: 1 s of the full end state, let go and still, measured step by step.
    task.reset_record()
    task._rec["pressed"] = True
    for k in range(20):
        task.observe(end, far, k * dt)
    _check("0.95 s into the hold", task.score(end), 0.5)
    task.observe(end, far, 20 * dt)
    _check("1.0 s into the hold", task.score(end), 1.0)
    task.observe(end, (0.004, 0.05), 21 * dt)  # a finger comes back to within 4 mm of the pack
    _check("a finger 4 mm from the pack restarts the hold", task.score(end), 0.5)
    for k in range(22, 50):
        task.observe(end, (0.05, 0.0049), k * dt)
    _check("a finger 4.9 mm from the drill: not let go", task.score(end), 0.5)
    for k in range(50, 80):
        task.observe(end, (0.0051, 0.0051), k * dt)
    _check("every finger just over 5 mm away", task.score(end), 1.0)

    # Stillness: the pack still settling (moving 3 mm within the second) does not count.
    task.reset_record()
    task._rec["pressed"] = True
    for k in range(30):
        s = _states(pack=in_tray(dx=0.003 * (k % 2)))
        task.observe(s, far, k * dt)
    _check("pack rocking 3 mm in the tray", task.score(s), 0.5)

    # The buttons: both pressed in 1.5 mm at the same step, or no credit at all.
    task.reset_record()
    slid = {"drill": drill(dz=0.0), "pack": pack(SX, SY + 0.05, TOP), "tray": TRAY.state()}
    for k, b in enumerate([(0.004, 0.0), (0.0, 0.004), (0.0014, 0.004)]):
        st = dict(slid, pack=pack(SX, SY, TOP, btn=b))
        task.observe(st, far, k * dt)
    _check("buttons pressed one at a time, then the pack slid", task.score(slid), 0.0)
    task.observe(dict(slid, pack=pack(SX, SY, TOP, btn=(0.0015, 0.0015))), far, 4 * dt)
    _check("both 1.5 mm in at one step, then slid", task.score(slid), 0.25)
    for k in range(5, 40):
        task.observe(end, far, k * dt)
    _check("...and then placed and held", task.score(end), 1.0)

    # The whole-trial fall rule: the drill tipped past 45 degrees once, then set back up.
    task.reset_record()
    task._rec["pressed"] = True
    task.observe(_states(drill=drill(dz=0.0, q=quat((1, 0, 0), 46.0))), far, 0.0)
    for k in range(1, 40):
        task.observe(end, far, k * dt)
    _check("drill knocked over once, then stood up again", task.score(end), 0.0)
    task.reset_record()
    task._rec["pressed"] = True
    task.observe(_states(drill=drill(dz=0.0, q=quat((1, 0, 0), 44.0))), far, 0.0)
    for k in range(1, 40):
        task.observe(end, far, k * dt)
    _check("drill tipped 44 degrees and back", task.score(end), 1.0)
    task.reset_record()
    task._rec["pressed"] = True
    task.observe(_states(pack=pack(SX, SY + 0.2, TOP - 0.06)), far, 0.0)
    for k in range(1, 40):
        task.observe(end, far, k * dt)
    _check("pack dropped off the bench, then put in the tray", task.score(end), 0.0)

    # Missing measurements fail closed for the whole trial.
    task.reset_record()
    task._rec["pressed"] = True
    task.observe(end, None, 0.0)
    for k in range(1, 40):
        task.observe(end, far, k * dt)
    _check("grippers unreadable at one step", task.score(end), 0.0)
    task.reset_record()
    _check("no step recorded at all", task.score(end), 0.0)

    # Let-go is measured on sampled hull faces, not only corners: a 120 x 80 mm face 0.5 mm over the pack's top with
    # every corner far away.
    p = end["pack"]
    top = float(p.high[2])
    h = box_hull((p.pos[0], p.pos[1], top + 0.0005 + 0.01), (0.060, 0.040, 0.010))
    corners_only = task.gap_to(PACK, p, h.vertices)
    sampled = task.gap_to(PACK, p, task._face_points(h))
    if not (corners_only > task.CLEAR and sampled < task.CLEAR):
        raise AssertionError(f"face sampling: corners {corners_only:.4f} m, sampled {sampled:.4f} m")
    # The buttons count as the pack: a fingertip 3 mm off a button cap is not let go.
    btn = [b for name, b in [(n, (c, r, s)) for n, c, r, s in task._part_boxes(PACK, p)] if name == "btn_l"][0]
    c, r, s = btn
    tip = np.asarray(c) + r @ np.array([s[0] / 2 + 0.003, 0.0, 0.0])
    if not task.gap_to(PACK, p, tip[None, :]) < task.CLEAR:
        raise AssertionError("a point 3 mm off a button is not seen")

    # Lot B: the same end state mirrored scores 1.0; lot A's tray position scores no full credit there.
    lots.use("B")
    task.reset_record()
    task._rec["pressed"] = True
    yaw = quat((0, 0, 1), 180.0)
    endb = {"drill": DRILL.state(pos=(SX, -SY, TOP - RAIL_Z), quat_xyzw=yaw),
            "pack": PACK.state(pos=(TX, -TY, TOP + TRAY_FLOOR), quat_xyzw=yaw),
            "tray": TRAY.state(pos=(TX, -TY, TOP))}
    for k in range(25):
        task.observe(endb, far, k * dt)
    _check("lot B: pack in the tray on the right", task.score(endb), 1.0)
    lots.use("A")
    preset()


_run()
