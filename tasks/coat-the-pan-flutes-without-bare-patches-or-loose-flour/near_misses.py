"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-trial record what the step callback records for a finished trial: every
cell of the pan's inside greased (0.05 mm) and floured (2 mg/cm^2 stuck, all it holds), no loose flour, the hands 50 mm away and
the end held still for 1 s. The cases then vary the end state (where the pan stands, its lean, the tools).

_checks() then drives the step-side function observe() and the layer rules directly and raises if anything scores
wrongly: nothing done; shortening only; flour only on a bare pan (it all falls out when turned over); a greased and
floured pan never turned over (loose flour left in the flutes); the loose-flour limits at 0.5 g and 2 g; bare spots
of 7 and 10.4 mm; 35, 24, 23 and 12 good zones; a lump; flour on the counter; the pan let fall; hands 29 and 31 mm
away; a 0.95 s and a 1.05 s hold; unreadable arms; the brush pressed on the pan's outside (lays nothing inside);
loose flour staying on the upright pan and sliding out when it is turned over; and lot B.
"""

import math

import numpy as np

import lots
import task
from task import AREA, BRUSH, DREDGER, PAN, SECTION, SHEET_FLOOR, TOP, X, ZONE, Field

lots.use("A")
FLAT = (0.0, 0.0, 0.0, 1.0)
GAP = 0.05


def quat_tilt(deg, axis=(1.0, 0.0, 0.0)):
    a = math.radians(deg) / 2
    ax = np.asarray(axis, float) / np.linalg.norm(axis)
    return (*(ax * math.sin(a)), math.cos(a))


def start(lot="A"):
    lt = lots.all_lots()[lots.index(lot)]
    out = {}
    for spec in (task.PAN, task.RAMEKIN, task.REST, task.BRUSH, task.DREDGER):
        if spec.name in lt.yaw:
            q = (0.0, 0.0, math.sin(lt.yaw[spec.name] / 2), math.cos(lt.yaw[spec.name] / 2))
        else:
            q = spec.quat if spec.quat is not None else (0.0, 0.0, 0.0, 1.0)
        out[spec.name] = spec.state(pos=lt.pos(spec.name), quat_xyzw=q)
    out[task.SHEET.name] = task.SHEET.state()
    return out


def settled(lot="A", **kw):
    """The lot's start with the pan resting on the sheet pan, and named objects moved: name=(pos, quat)."""
    s = start(lot)
    p = s[PAN.name]
    s[PAN.name] = PAN.state(pos=(p.pos[0], p.pos[1], SHEET_FLOOR - task.PAN_LOW), quat_xyzw=p.quat_xyzw)
    for n, (pos, quat) in kw.items():
        spec = {PAN.name: PAN, BRUSH.name: BRUSH, DREDGER.name: DREDGER}[n]
        s[n] = spec.state(pos=pos, quat_xyzw=quat)
    return s


def moving(s, v=(0.0, 0.0, 0.01)):
    p = s[PAN.name]
    out = dict(s)
    out[PAN.name] = PAN.state(pos=p.pos, quat_xyzw=p.quat_xyzw, lin_vel=v)
    return out


def done_field():
    f = Field()
    f.g[:] = 0.05
    f.f[:] = task.F_CAP
    f.dredger = 15000.0
    f.caught = 10000.0
    return f


def preset(fld=None):
    task.reset_record()
    task._rec.update(steps=400, since=18.0, held=True, t=20.0, gap=GAP, field=fld or done_field())


P0 = settled()[PAN.name].pos
preset()
CASES = [
    ("pan coated, no loose flour, upright on the sheet pan, tools back, hands clear, still 1 s", settled(), 1.0, True),
    ("pan leaning exactly 10 degrees (propped on a handle)", settled(bundt_pan=(P0, quat_tilt(10.0))), 1.0, True),
    ("pan leaning 11 degrees", settled(bundt_pan=(P0, quat_tilt(11.0))), 0.0, False),
    ("pan left upside down on the sheet pan",
     settled(bundt_pan=((P0[0], P0[1], SHEET_FLOOR + 0.083), quat_tilt(180.0, (0, 1, 0)))), 0.0, False),
    ("pan's feet exactly 4 mm above the sheet pan.s floor", settled(bundt_pan=((P0[0], P0[1], P0[2] + 0.004), FLAT)), 1.0, True),
    ("pan held 5 mm above the sheet pan.s floor", settled(bundt_pan=((P0[0], P0[1], P0[2] + 0.005), FLAT)), 0.0, False),
    ("pan set on the counter beside the sheet pan", settled(bundt_pan=((0.33, 0.30, TOP - task.PAN_LOW), FLAT)), 0.0,
     False),
    ("brush left lying in the pan", settled(pastry_brush=((P0[0] + 0.06, P0[1], P0[2] + 0.03), quat_tilt(90.0, (0, 1, 0)))),
     0.0, False),
    ("dredger left standing in the pan's ring", settled(flour_dredger=((P0[0] + 0.06, P0[1], P0[2] + 0.01), FLAT)), 0.0,
     False),
    ("pan still rocking at 10 mm/s", moving(settled()), 0.0, False),
]


def _expect(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"{label}: scored {got}, expected {want}")
    print(f"ok    {label}: {got}")


def gap_of(g):
    return lambda: g


def _run(lot, scenes, label, want, fld=None, prep=None):
    """Drive observe() over the scenes (one per 0.05 s step) with a ready field, then score the last scene."""
    lots.use(lot)
    task.reset_record()
    if fld is not None:
        task._rec["field"] = fld
    if prep:
        prep()
    t, last = 0.0, None
    for objects, g in scenes:
        t += 0.05
        last = objects
        task.observe(objects, gap_of(g), t)
    _expect(f"lot {lot}: {label}", float(task.score(last)), want)


def hold(lot="A", n=30, g=GAP, s=None):
    return [(s or settled(lot), g)] * n


def bare_patch(fld, n):
    """Strip the flour from ``n`` cells in a row down one flute's outer wall; returns the patch's length (m)."""
    cells = [20 * task.N_S + 8 + k for k in range(n)]
    fld.f[cells] = 0.0
    return task.extent(np.array(cells))


def _checks():
    for lot in ("A", "B"):
        _run(lot, hold(lot), "nothing done", 0.0)
        _run(lot, hold(lot, 25), "coated pan, 1.25 s still", 1.0, done_field())
        _run(lot, hold(lot, 19), "coated pan, 0.95 s still", 0.0, done_field())
    f = done_field()
    f.f[:] = 0.0
    _run("A", hold(), "shortening everywhere, no flour", 0.0, f)
    f = Field()
    f.f[:] = 0.0
    f.loose[:] = 0.0
    _run("A", hold(), "bare pan", 0.0, f)
    # loose flour left in the flutes
    for mg, want in ((500.0, 1.0), (510.0, 0.75), (2000.0, 0.75), (2010.0, 0.25), (6000.0, 0.25)):
        f = done_field()
        f.loose[ZONE == 1] = mg / np.count_nonzero(ZONE == 1)
        _run("A", hold(), f"coated, {mg / 1000:.2f} g of loose flour left on one flute's floor", want, f)
    # bare spots
    f = done_field()
    n = bare_patch(f, 2)
    _run("A", hold(), f"one bare spot {n * 1000:.1f} mm long (under the 10 mm limit)", 1.0, f)
    f = done_field()
    n = bare_patch(f, 3)
    _run("A", hold(), f"one bare spot {n * 1000:.1f} mm long: 35 of 36 zones", 0.5, f)
    for good, want in ((24, 0.5), (23, 0.25), (12, 0.25), (11, 0.0)):
        f = done_field()
        f.f[ZONE >= good] = 0.0
        _run("A", hold(), f"{good} zones coated, the rest shortening only", want, f)
    # lumps, spills, drops, hands, arms
    f = done_field()
    f.g[100] = 2.01
    _run("A", hold(), "one lump of shortening 2.01 mm thick", 0.25, f)
    f = done_field()
    f.g[100] = 1.97
    _run("A", hold(), "1.97 mm of shortening plus 0.04 mm of flour: 2.01 mm, a lump", 0.25, f)
    f = done_field()
    f.g[100] = 1.95
    _run("A", hold(), "thickest deposit 1.99 mm in all", 1.0, f)
    f = done_field()
    f.spilled = 2010.0
    _run("A", hold(), "2.01 g of flour on the counter", 0.25, f)
    f = done_field()
    f.spilled = 1990.0
    _run("A", hold(), "1.99 g of flour on the counter", 1.0, f)
    drop = moving(settled(), (0.0, 0.0, -0.7))
    _run("A", [(drop, GAP)] + hold(), "pan let fall onto the sheet pan (0.7 m/s) earlier", 0.0, done_field())
    _run("A", hold(g=0.029), "a fingertip 29 mm from the pan at the end", 0.0, done_field())
    _run("A", hold(g=0.031), "every hand point 31 mm away", 1.0, done_field())
    _run("A", [(settled(), None)] * 3 + hold(), "arms unreadable for some steps at the end (fails closed)", 0.0, done_field())
    lifted = settled(bundt_pan=((P0[0], P0[1], P0[2] + 0.05), FLAT))
    _run("A", hold(n=10) + hold(n=3, s=lifted) + hold(n=10), "coated, picked up again, 0.5 s still", 0.0,
         done_field())
    _rules()
    lots.use("A")
    preset()


def _rules():
    """The declared layer rules, checked directly."""
    f = Field()
    f.brush_load = 3.0
    # the brush pressed on the pan's outside: its roots are outside the rim, so nothing is laid inside
    out = np.array([[0.105, 0.0, 0.04]])
    if f.brush(out, out + (0.0, 0.0, 0.055), 0.05):
        raise AssertionError("brushing the pan's outside laid shortening inside")
    # pressed on the outside of the wall just under the rim, its roots above the rim but outside it: nothing inside
    side = np.array([[task.BODY_R + 0.002, 0.0, task.RIM_Z - 0.010]])
    if f.brush(side, side + (0.004, 0.0, 0.05), 0.05):
        raise AssertionError("brushing the outside just under the rim laid shortening inside")
    # pressed into the floor from above: lays shortening
    c = int(np.argmin(np.abs(task.CELL_S - (task.S_WALL + task.S_ARC / 2)) + np.abs(task.CELL_TH - 0.3)))
    p = np.array([X[c] + task.NOUT[c] * 0.003, X[c] - task.NOUT[c] * 0.005])  # one pressed in, one still above it
    if not f.brush(p, p + (0.0, 0.0, 0.055), 0.05):
        raise AssertionError("bristles pressed 3 mm into the floor laid nothing")
    g = Field()
    g.brush_load = 3.0
    if g.brush(p[:1], p[:1] + (0.0, 0.0, 0.055), 0.05):  # no part of the bristles in the pan's opening: its outside
        raise AssertionError("a bristle block wholly inside the metal laid shortening")
    print("ok    brush rules: nothing laid from outside, laid when pressed inside")
    # flour dusted on a bare pan: none sticks, all loose; upright it stays; turned over it all leaves
    f = Field()
    upright = np.eye(3)
    for _ in range(20):
        f.dust(np.array([0.33 + 0.06, 0.0, task.PAN_A[2] + 0.15]), np.array(task.PAN_A), upright, task.sheet_inside,
               0.05)
    landed = f.loose.sum()
    for _ in range(40):
        f.settle(np.array(task.PAN_A), upright, task.sheet_inside)
    if f.f.sum() > 0 or f.loose.sum() < 0.99 * landed - 1e-9:
        raise AssertionError("flour on a bare upright pan stuck or left it")
    over = task.parts._rot((0, 1, 0), math.pi)
    for _ in range(80):
        f.settle(np.array(task.PAN_A) + (0, 0, 0.1), over, task.sheet_inside)
    if f.loose.sum() > 1e-6:
        raise AssertionError(f"{f.loose.sum()} mg of loose flour stayed in the turned-over pan")
    print(f"ok    flour rules: {landed:.0f} mg on a bare pan, none stuck, all out when turned over")
    # on a greased pan it sticks up to 2 mg/cm^2 per cell
    f = done_field()
    f.f[:] = 0.0
    f.loose[:] = AREA * 0.05
    f.stick()
    if not np.allclose(f.f, task.F_CAP) or not np.allclose(f.loose, AREA * (0.05 - task.F_CAP)):
        raise AssertionError("sticking does not stop at 2 mg/cm^2")
    print("ok    flour sticks to shortening up to 2 mg/cm^2, the rest stays loose")
    if not (SECTION.min() == 0 and SECTION.max() == 2 and len(np.unique(ZONE)) == 36):
        raise AssertionError("zones")


if __name__ == "__main__":
    _checks()
    for label, changes, want, want_success in CASES:
        s = dict(start("A"))
        s.update(changes)
        got = 1.0 if task.success(s) else float(task.score(s))
        _expect(label, got, want)
else:
    _checks()
