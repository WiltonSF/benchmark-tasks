"""Near-successes, fixed habits and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state}, expected score, expected success). Every object is a parts.py Part, so each
case gives its state from Part.state(pos=..., quat_xyzw=...): pos is the part's origin (the bolt's shank end, the
gasket's underside centre, the upper flange's underside centre). Offline there is no robot, so the release check
passes; _run() below also checks the release rule with a gripper placed near a part.
"""

import math
import os

import task
from task import (BOLTS, CX, CY, GAP, HOLES, LOW_TOP, RING, RING_START_X, SEATED_Z, SHANK_L, SPACERS, TOP, UP_TOP,
                  UP_Z0, UPPER)

os.environ[task.OFFLINE_FLAG] = "1"  # the task credits these offline cases as let go


def tilt_q(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def ring(dx=0.0, dy=0.0, dz=0.0, tilt=0.0):
    return RING.state(pos=(CX + dx, CY + dy, TOP + LOW_TOP + dz), quat_xyzw=tilt_q(tilt))


RING_OUT_FRONT = RING.state()  # never moved: still on the block in front


def bolt(i, hole, dx=0.0, dz=0.0, tilt=0.0):
    hx, hy = HOLES[hole]
    return BOLTS[i].state(pos=(CX + hx + dx, CY + hy, TOP + SEATED_Z + dz), quat_xyzw=tilt_q(tilt))


def four(**kw):
    return {"bolt_1": bolt(0, "far_left"), "bolt_2": bolt(1, "near_left"), "bolt_3": bolt(2, "far_right"),
            "bolt_4": bolt(3, "near_right"), **kw}


def far_only(**kw):
    return {"bolt_1": bolt(0, "far_left"), "bolt_3": bolt(2, "far_right"), **kw}


def near_only(**kw):
    return {"bolt_2": bolt(1, "near_left"), "bolt_4": bolt(3, "near_right"), **kw}


def upper(dx=0.0, dy=0.0, dz=0.0, tilt=0.0, turn=0.0):
    if turn:
        a = math.radians(turn) / 2
        q = (0.0, 0.0, math.sin(a), math.cos(a))
    else:
        q = tilt_q(tilt)
    return UPPER.state(pos=(CX + dx, CY + dy, TOP + UP_Z0 + dz), quat_xyzw=q)


def bolt_from_head(i, hole, dx=0.0, tilt=0.0):
    """A bolt whose head's underside sits on the upper flange's top, dx from the hole's centre, leaning tilt degrees
    about that point (about the x axis)."""
    hx, hy = HOLES[hole]
    t = math.radians(tilt)
    axis = (0.0, -math.sin(t), math.cos(t))
    head = (CX + hx + dx, CY + hy, TOP + UP_TOP)
    return BOLTS[i].state(pos=tuple(h - a * SHANK_L for h, a in zip(head, axis)), quat_xyzw=tilt_q(tilt))


def four_shifted(dx, **kw):
    """The upper flange slid dx along x on its spacers, every bolt in its hole moved with it."""
    return {"upper_flange": upper(dx=dx), "bolt_1": bolt(0, "far_left", dx=dx), "bolt_2": bolt(1, "near_left", dx=dx),
            "bolt_3": bolt(2, "far_right", dx=dx), "bolt_4": bolt(3, "near_right", dx=dx), **kw}


# Spacers out and the joint closed onto the gasket: the upper flange 4 mm above the lower one, the bolts with it.
CLOSE = -(GAP - 0.004)
CLOSED = {
    "upper_flange": upper(dz=CLOSE),
    "spacer_1": SPACERS[0].state(pos=(0.20, 0.10, TOP)), "spacer_2": SPACERS[1].state(pos=(0.20, -0.10, TOP)),
    **{f"bolt_{i + 1}": bolt(i, h, dz=CLOSE) for i, h in enumerate(("far_left", "near_left", "far_right", "near_right"))},
}

CASES = [
    ("gasket centred, four bolts in", four(gasket=ring()), 1.0, True),
    ("gasket 4.0 mm off the axis", four(gasket=ring(dx=-0.004)), 1.0, True),
    ("gasket 4.1 mm off the axis: no gasket, so no bolt credit", four(gasket=ring(dx=-0.0029, dy=0.0029)), 0.0, False),
    ("gasket 1.5 mm up (riding on something)", four(gasket=ring(dz=0.0015)), 1.0, True),
    ("gasket 2 mm up", four(gasket=ring(dz=0.002)), 0.0, False),
    ("gasket tilted 2.9 deg", four(gasket=ring(tilt=2.9)), 1.0, True),
    ("gasket tilted 3.1 deg", four(gasket=ring(tilt=3.1)), 0.0, False),
    # Partial credit: 0.2 for the centred gasket, 0.2 more per bolt, in any order.
    ("gasket centred, no bolts", {"gasket": ring()}, 0.2, False),
    ("gasket centred against the two far bolts, near holes empty", far_only(gasket=ring(dx=0.001)), 0.6, False),
    ("gasket centred, both far bolts and one near bolt", {**far_only(gasket=ring()), "bolt_2": bolt(1, "near_left")},
     0.8, False),
    ("gasket centred, both near bolts and one far bolt (the reviewer's case)",
     {**near_only(gasket=ring()), "bolt_1": bolt(0, "far_left")}, 0.8, False),
    ("gasket centred with only the near bolts in", near_only(gasket=ring()), 0.6, False),
    ("gasket centred, one far bolt only", {"gasket": ring(), "bolt_3": bolt(2, "far_right")}, 0.4, False),
    # Fixed habits and wrong outcomes.
    ("habit: all four bolts first, gasket left on its block", four(gasket=RING_OUT_FRONT), 0.0, False),
    ("habit: nearest holes first, gasket stopped against them short of the axis",
     near_only(gasket=ring(dx=-0.040)), 0.0, False),
    ("three bolts in, gasket left on its block", {**far_only(), "bolt_2": bolt(1, "near_left")}, 0.0, False),
    ("gasket pushed too far, 10 mm past the axis, four bolts in", four(gasket=ring(dx=0.010)), 0.0, False),
    # The loose upper flange: bolts are judged in its holes, wherever it is.
    ("upper flange slid 2 mm on its spacers, bolts in its holes", four_shifted(0.002, gasket=ring()), 1.0, True),
    ("upper flange slid 3 mm on its spacers, bolts in its holes", four_shifted(0.003, gasket=ring()), 1.0, True),
    # The reviewer's cases: the upper flange removed or shifted, and bolts through the upper flange only.
    ("upper flange lifted away and set on the table, gasket centred on the lower flange",
     {"gasket": ring(), "upper_flange": UPPER.state(pos=(0.20, 0.0, TOP))}, 0.0, False),
    ("upper flange lifted away, gasket centred, four bolts through the lower flange only",
     {**four(gasket=ring()), "upper_flange": UPPER.state(pos=(0.20, 0.0, TOP))}, 0.0, False),
    ("upper flange shifted 3.5 mm with its bolts: not over the gasket", four_shifted(0.0035, gasket=ring()), 0.0,
     False),
    ("upper flange shifted 10 mm, bolts left in the lower holes", {**four(gasket=ring()), "upper_flange": upper(dx=0.010)},
     0.0, False),
    ("upper flange turned 2.5 deg about the pipe", {**four(gasket=ring()), "upper_flange": upper(turn=2.5)}, 0.0, False),
    ("upper flange tipped 3.5 deg", {"gasket": ring(), "upper_flange": upper(tilt=3.5)}, 0.0, False),
    ("upper flange held 2 mm above its spacers (18 mm gap)", {"gasket": ring(), "upper_flange": upper(dz=0.002)}, 0.0,
     False),
    ("bolt through the upper flange only, head seated, shank 4.5 mm off the lower hole (flange 3 mm over)",
     {**four_shifted(0.003, gasket=ring()), "bolt_2": bolt(1, "near_left", dx=0.0045)}, 0.8, False),
    ("bolt through the upper flange only, head seated, shank leaning 8 deg past the lower hole",
     four(gasket=ring(), bolt_2=bolt_from_head(1, "near_left", tilt=8.0)), 0.8, False),
    ("bolt in the upper hole, its end resting on the lower flange beside the hole",
     four(gasket=ring(), bolt_2=BOLTS[1].state(pos=(CX + HOLES["near_left"][0] + 0.0045, CY + HOLES["near_left"][1],
                                                     TOP + LOW_TOP))), 0.8, False),
    ("spacers out, joint closed onto the centred gasket, four bolts in", {**CLOSED, "gasket": ring()}, 1.0, True),
    # Bolt limits.
    ("a bolt leaning 3 deg in its holes (shank 2.1 mm off the upper hole's centre)",
     four(gasket=ring(), bolt_2=bolt(1, "near_left", tilt=3.0)), 1.0, True),
    ("a near bolt tilted 9 deg: not in", four(gasket=ring(), bolt_2=bolt(1, "near_left", tilt=9.0)), 0.8, False),
    ("a bolt head 2 mm above the flange", four(gasket=ring(), bolt_4=bolt(3, "near_right", dz=0.002)), 1.0, True),
    ("a near bolt head 2.5 mm up: not in",
     four(gasket=ring(), bolt_4=bolt(3, "near_right", dz=0.0025)), 0.8, False),
    ("a bolt 2.5 mm off its hole centres", four(gasket=ring(), bolt_1=bolt(0, "far_left", dx=0.0025)), 1.0, True),
    ("a bolt 2.6 mm off its hole centres: not in", four(gasket=ring(), bolt_1=bolt(0, "far_left", dx=0.0026)), 0.8,
     False),
    ("gasket left where it starts, nothing done", {"gasket": RING.state(pos=(CX + RING_START_X, CY, TOP + LOW_TOP))},
     0.0, False),
]


def _window_cases(start) -> int:
    """The stillness rule: each scored part's last 10 poses (0.5 s) must stay within 1 mm and 0.5 degrees of its
    current pose, and a part with fewer than 10 poses recorded is not still. Each case fills the windows by hand: every
    part at rest for 10 steps, then one part's window changed."""
    final = {**start, **four(gasket=ring())}
    parts_ = task._STILL_PARTS

    def run(label, moving=None, poses=None, short=None, want=1.0):
        task._WIN.clear()
        for name in parts_:
            k = task.STILL_STEPS - 1 if name == short else task.STILL_STEPS
            for i in range(k):
                st = final[name]
                if name == moving:  # earlier poses of the moving part, ending at its final pose
                    st = poses(i, st)
                task.track_pose(name, st)
        got = 1.0 if task.success(final) else float(task.score(final))
        task._WIN.clear()
        flag = "ok  " if got == want else "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
        return flag == "FAIL"

    def slid(dx):  # pose i of 10 is (9 - i) / 9 of dx short of the final pose, along x
        def f(i, st):
            p = list(st.pos)
            p[0] -= dx * (task.STILL_STEPS - 1 - i) / (task.STILL_STEPS - 1)
            return type(st)(**{**st.__dict__, "pos": tuple(p)})
        return f

    def turned(deg):  # turned deg about z at the window's start, back to the final pose at its end
        def f(i, st):
            a = math.radians(deg) * (task.STILL_STEPS - 1 - i) / (task.STILL_STEPS - 1) / 2
            return type(st)(**{**st.__dict__, "quat_xyzw": (0.0, 0.0, math.sin(a), math.cos(a))})
        return f

    bad = 0
    bad += run("every part at rest for 0.5 s: in", want=1.0)
    bad += run("gasket slid 2 mm within the last 0.5 s: not still, so 0", "gasket", slid(0.002), want=0.0)
    bad += run("upper flange still sliding: 2 mm within the last 0.5 s, so 0", "upper_flange", slid(0.002), want=0.0)
    bad += run("upper flange crept 0.9 mm within the last 0.5 s: still", "upper_flange", slid(0.0009), want=1.0)
    bad += run("a bolt slid 2 mm within the last 0.5 s: not counted", "bolt_4", slid(0.002), want=0.8)
    bad += run("a bolt turned 0.6 deg within the last 0.5 s: not counted", "bolt_2", turned(0.6), want=0.8)
    bad += run("a bolt turned 0.4 deg within the last 0.5 s: counted", "bolt_2", turned(0.4), want=1.0)
    bad += run("upper flange window not full yet (9 of 10 steps): 0", short="upper_flange", want=0.0)
    bad += run("a bolt's window not full yet: not counted", short="bolt_1", want=0.8)
    return bad


def _history_cases(start) -> int:
    """The whole-trial rule: the upper flange moved off during the trial and put back exactly still scores 0."""
    bad = 0
    full = {**start, **four(gasket=ring())}
    checks = [
        ("upper flange lifted 30 mm off during the trial, gasket and bolts in, flange put back", upper(dz=0.030), 0.0),
        ("upper flange slid 12 mm aside during the trial, then put back", upper(dx=-0.012), 0.0),
        ("upper flange tipped 6 deg during the trial, then settled back", upper(tilt=6.0), 0.0),
        ("upper flange lifted 4 mm during the trial (under 5 mm), then settled back", upper(dz=0.004), 1.0),
        ("spacers out during the trial, joint closed onto the gasket (lowered, not lifted)", upper(dz=CLOSE), 1.0),
    ]
    for label, during, want in checks:
        task._FLANGE["moved"] = False
        task.track_flange(start["upper_flange"])
        task.track_flange(during)
        got = 1.0 if task.success(full) else float(task.score(full))
        flag = "ok  " if got == want else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    task._FLANGE["moved"] = False
    return bad


def _closure_check() -> int:
    """The joint can close onto the gasket: with the gasket centred and the spacers out, every part of the gasket under
    the flange faces (ring and tab alike) is 4 mm thick, so the lowered upper flange's face rests on the gasket's top,
    and nothing else stands between the faces."""
    half = task.HALF
    tops = []
    for s in RING.shapes:  # the ring is centred on the axis, unturned, its underside on the lower flange
        x, y, z = s.at
        hx, hy, hz = s.half
        if abs(x) - hx < half and abs(y) - hy < half:  # any part of the shape inside the flange faces
            tops.append(LOW_TOP + z + hz)
    tallest = max(tops)
    face = UP_Z0 + CLOSE  # the upper flange's underside with the spacers out, the joint closed
    bad = 0
    for label, ok in [
        (f"tallest gasket part between the faces {1000 * (tallest - LOW_TOP):.1f} mm (the ring is 4.0 mm)",
         abs((tallest - LOW_TOP) - task.RING_T) < 1e-9),
        (f"closed flange face {1000 * (face - tallest):+.1f} mm from the gasket's top (within 0.5 mm)",
         abs(face - tallest) <= 0.0005),
        ("tab reaches past the flange's near edge: end 64.5 mm outside it",
         abs((task.RING_OUT + task.TAB_LEN) - half - 0.0645) < 1e-6),
    ]:
        flag = "ok  " if ok else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag}       closure: {label}")
    # Every other loose part inside the flange faces is a bolt, which passes through the holes; the spacers come out.
    others = [o.name for o in task.OBJECTS if o.mass is not None and o.name not in ("gasket", "upper_flange")
              and not o.name.startswith(("bolt_", "spacer_"))]
    flag = "ok  " if not others else "FAIL"
    bad += flag == "FAIL"
    print(f"{flag}       closure: nothing else between the faces {others or ''}")
    return bad


def _release_cases(start) -> int:
    """The release rule, which check.py cannot reach offline: a gripper within 5 mm, or no measurement yet."""
    bad = 0
    full = {**start, **four(gasket=ring())}
    names = ["upper_flange", "gasket", *task.BOLT_NAMES]
    checks = [
        ("gripper 4.9 mm from a near bolt: that bolt not counted", {n: 0.05 for n in names} | {"bolt_4": 0.0049}, 0.8),
        ("gripper 5.0 mm from a near bolt: counted", {n: 0.05 for n in names} | {"bolt_4": 0.005}, 1.0),
        ("gripper still on the gasket: nothing counted", {n: 0.05 for n in names} | {"gasket": 0.0}, 0.0),
        ("upper flange held in place by a gripper: nothing counted", {n: 0.05 for n in names} | {"upper_flange": 0.0},
         0.0),
        ("gripper 4.9 mm from the upper flange: nothing counted", {n: 0.05 for n in names} | {"upper_flange": 0.0049},
         0.0),
    ]
    for label, gaps, want in checks:
        task._GAP.clear()
        task._GAP.update(gaps)
        got = 1.0 if task.success(full) else float(task.score(full))
        flag = "ok  " if got == want else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    task._GAP.clear()
    os.environ[task.OFFLINE_FLAG] = "0"  # in the simulator, before the first step has measured the grippers
    got = float(task.score(full))
    os.environ[task.OFFLINE_FLAG] = "1"
    flag = "ok  " if got == 0.0 else "FAIL"
    bad += flag == "FAIL"
    print(f"{flag} {got:4.2f}  no gripper measurement yet: fails closed")
    return bad


def _run() -> int:
    start = {o.name: o.state() for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = {**start, **changes}
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad + _closure_check() + _window_cases(start) + _history_cases(start) + _release_cases(start)


if __name__ == "__main__":
    raise SystemExit(_run())
