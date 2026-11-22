"""Near misses and unintended strategies for painting the TacTip pin tips.

Painting is a history of presses, so the behaviour cases drive the scorer's own recorder (`task.record_press`,
which the simulator callback calls every control step with the force the simulator reports between the former and
the bar, and both poses). Each case starts from a fresh history and checks score() with both hands clear and
everything still unless it says otherwise. CASES (run by factory/check.py) then checks end states."""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

import lots  # noqa: E402
import task  # noqa: E402

I3 = np.eye(3)


def start():
    return {o.name: o.state() for o in task.OBJECTS}


def released(hold=None):
    """Both hands 30 mm clear and the scene still for the 1 s hold (what the simulator callback keeps)."""
    task._S.update(known=True, clear=True, hold=task.HOLD_STEPS if hold is None else hold)


def lift():
    """The dome taken off the face for long enough that the next touch is a new contact."""
    for _ in range(task.OFF_STEPS):
        task.no_press()


def fresh():
    task.reset_state()
    task._S["shown"] = set()


def bar_state(beta=0.0, face=(0.315, 0.0, task.TOP + 0.1)):
    """The bar held with its painted end's centre at ``face``, tipped up ``beta`` degrees toward +y (lot A)."""
    b = math.radians(beta)
    n = np.array([0.0, math.cos(b), math.sin(b)])
    x_axis = n
    z_axis = np.cross(np.array([1.0, 0.0, 0.0]), n)
    r = np.c_[x_axis, np.cross(z_axis, x_axis), z_axis]
    q = task.parts._matrix_to_quat(r)
    return task.BAR.state(pos=np.asarray(face) - n * task.BAR_L / 2, quat_xyzw=q), n, np.asarray(face)


def press(d, force=3.0, beta=0.0, spin=0.0, lateral=(0.0, 0.0), sink=0.0, bar=None):
    """One control step: the dome turned so its direction ``d`` (in F) faces the bar's end, touching it (``sink``
    m past first touch), pressed with ``force`` N."""
    rs, n, face = bar_state(beta) if bar is None else bar
    r1 = task._rot_between(np.asarray(d, float) / np.linalg.norm(d), -n)
    rf = task._axis_angle(n, spin) @ r1
    u, v = task._orth(n)
    o = face + lateral[0] * u + lateral[1] * v + (task.R_TIP - sink) * n
    fs = task.FORMER.state(pos=o, quat_xyzw=task.parts._matrix_to_quat(rf))
    task.record_press(force, fs, rs)
    return fs, rs


def apex_dir(theta_deg, az_deg=0.0):
    t, a = math.radians(theta_deg), math.radians(az_deg)
    return np.array([math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), -math.cos(t)])


def demo_presses(alpha=None, force=3.0):
    """Every dab of the demo's own plan, as single loaded steps."""
    dabs = task.dab_list()
    for e in dabs:
        bar = bar_state(e["beta"], face=e["touch"] - task.R_TIP * e["n"])
        rs, n, face = bar
        fs = task.FORMER.state(pos=e["touch"], quat_xyzw=task.parts._matrix_to_quat(e["rf"]))
        task.record_press(force, fs, rs)
        lift()


def end(objects=None):
    st = start()
    st.update(objects or {})
    return task.score(st)


def run_behaviour_cases():
    out = []

    def all_painted():
        task._S["painted"][:] = True
        task._S["wet"] = True
        released()

    def n_painted(k):
        def f():
            task._S["painted"][:k] = True
            task._S["wet"] = True
            released()
        return f

    fresh(); all_painted(); out.append(("every pin painted, former hung, bar down, hands clear", end(), 1.0))
    for k, want in ((task.N_PINS - 1, 0.75), (479, 0.75), (478, 0.5), (266, 0.5), (265, 0.25), (133, 0.25),
                    (132, 0.0)):
        fresh(); n_painted(k)(); out.append((f"{k} pins painted", end(), want))
    fresh(); all_painted(); task._S["hold"] = task.HOLD_STEPS - 1
    out.append(("every pin painted, still only 0.95 s", end(), 0.0))
    fresh(); all_painted(); task._S["clear"] = False
    out.append(("every pin painted, a hand within 30 mm", end(), 0.0))
    fresh(); all_painted(); task._S["known"] = False
    out.append(("simulator state unknown (fail closed)", end(), 0.0))
    # the demo's own plan, as recorded presses, paints every pin (at the planned 1.7 N and at a firm 5 N)
    for f in (2.0, 5.0):
        fresh(); demo_presses(force=f); released()
        out.append((f"the demo's {len(task.dab_list())} dabs at {f} N paint all {task.N_PINS} pins", end(), 1.0))
    fresh(); demo_presses(force=1.0); released()
    out.append(("the demo's dabs at only 1.0 N leave gaps (fewer than 90 % painted)", end(),
                0.75 if task.n_painted() >= 479 else (0.5 if task.n_painted() >= 266 else 0.25)))
    # breaking rules
    fresh(); demo_presses(force=3.0); press(apex_dir(0), force=19.5); lift(); released()
    out.append(("one press at 19.5 N folds the pins (skin between them in the paint)", end(), 0.0))
    fresh(); demo_presses(force=3.0); press(apex_dir(0), force=18.9); lift(); released()
    out.append(("a firm press at 18.9 N is allowed", end(), 1.0))
    fresh(); demo_presses(force=3.0)
    for k in range(6):  # the dome slid 0.5 mm a step for 6 steps, pressed: 2.5 mm of slide
        press(apex_dir(30), force=3.0, lateral=(0.0005 * k, 0.0))
    lift(); released()
    out.append(("dome slid 2.5 mm over the wet paint in one press (smear)", end(), 0.0))
    fresh(); demo_presses(force=3.0)
    for k in range(4):  # 1.5 mm of slide: under the limit
        press(apex_dir(30), force=3.0, lateral=(0.0005 * k, 0.0))
    lift(); released()
    out.append(("dome crept 1.5 mm in one press (under the 2 mm limit)", end(), 1.0))
    fresh(); demo_presses(force=3.0)
    rs, n, face = bar_state(0.0)
    u, v = task._orth(n)
    r0 = task._rot_between(apex_dir(30), -n)
    for k in range(5):  # rolling: the centre moves 3 degrees' worth of arc a step and the dome turns to match
        step = task.R_TIP * math.radians(3.0) * k
        rf = task._axis_angle(np.cross(n, u), step / task.R_TIP) @ r0
        o = face - 0.004 * u + step * u + task.R_TIP * n
        task.record_press(3.0, task.FORMER.state(pos=o, quat_xyzw=task.parts._matrix_to_quat(rf)), rs)
    lift(); released()
    out.append(("dome rolled 12 degrees without sliding", end(), 1.0))
    fresh(); demo_presses(force=3.0)
    for k in range(14):  # a light wipe: the force flickers under 0.3 N while the dome slides 0.4 mm a step
        press(apex_dir(30), force=(0.5 if k % 2 else 0.2), lateral=(0.0004 * k, 0.0))
    lift(); released()
    out.append(("a light wipe whose force flickers under 0.3 N still smears", end(), 0.0))
    fresh(); demo_presses(force=3.0)
    for k in range(8):  # scrubbing back and forth 0.6 mm: no net slide, 4.2 mm of path
        press(apex_dir(30), force=3.0, lateral=(0.0006 * (k % 2), 0.0))
    lift(); released()
    out.append(("scrubbing back and forth 0.6 mm (4.2 mm of path) smears", end(), 0.0))
    fresh(); demo_presses(force=3.0)
    for k in range(6):  # twisting the dome 4 degrees a step about the face's normal, pressed
        press(apex_dir(20), force=3.0, spin=math.radians(4.0 * k))
    lift(); released()
    out.append(("twisting the dome 20 degrees on the paint smears", end(), 0.0))
    fresh(); demo_presses(force=3.0)
    for k in range(10):  # a light wipe, 3.6 mm in all, its force flickering 0.5 / 0.2 N
        press(apex_dir(30), force=(0.5 if k % 2 else 0.2), lateral=(0.0004 * k, 0.0))
    lift(); released()
    out.append(("a 3.6 mm light wipe with the force flickering 0.5 / 0.2 N smears", end(), 0.0))
    fresh(); demo_presses(force=3.0)
    for k in range(6):  # wobbling the dome +-4 degrees about the face's normal: no net turn, 20 degrees of turning
        press(apex_dir(20), force=3.0, spin=math.radians(4.0 * (k % 2)))
    lift(); released()
    out.append(("wobbling the dome +-4 degrees on the paint (20 degrees of turning) smears", end(), 0.0))
    fresh(); demo_presses(force=3.0)
    for k in range(3):  # a 2 degree twist is allowed
        press(apex_dir(20), force=3.0, spin=math.radians(1.0 * k))
    lift(); released()
    out.append(("a 2 degree twist in one press is allowed", end(), 1.0))
    fresh(); demo_presses(force=3.0); press(apex_dir(97), force=0.0); lift(); released()
    out.append(("the skirt laid on the wet paint with no reported force", end(), 0.0))
    fresh(); demo_presses(force=3.0); press(apex_dir(93), force=2.0); lift(); released()
    out.append(("press centred 93 degrees from the apex: the skirt in the paint", end(), 0.0))
    fresh(); demo_presses(force=3.0); press(apex_dir(91), force=2.0); lift(); released()
    out.append(("press centred 91 degrees from the apex: pins only", end(), 1.0))
    # shortcuts
    fresh(); press(apex_dir(0), force=18.0); lift(); released()
    out.append((f"one hard straight-down press ({task.n_painted()} pins)", end(), 0.0))
    fresh()
    for th in range(0, 61, 6):  # the bar stood on its dry end, face up: the wrist leans the dome at most about 60 deg
        for az in range(0, 360, 10):
            press(apex_dir(th, az), force=4.0, beta=90.0); lift()
    released()
    out.append((f"bar stood face up only, dome leaned up to 60 degrees ({task.n_painted()} pins)", end(), 0.5))
    fresh()
    for th in (0, 20, 40, 60, 80):  # wiping round the face instead of dabbing
        press(apex_dir(th), force=3.0)
    for k in range(8):
        press(apex_dir(80), force=3.0, lateral=(0.001 * k, 0.0))
    lift(); released()
    out.append(("wiping the dome across the face (smear)", end(), 0.0))
    fresh(); demo_presses(force=3.0)
    rs, n, face = bar_state(0.0)
    task.record_press(3.0, task.FORMER.state(pos=face + (task.R_TIP + 0.02) * n), rs)  # far off: no contact
    released()
    out.append(("a reported force with the dome 20 mm off the face paints nothing new", end(), 1.0))
    fresh()
    rs, n, face = bar_state(0.0)
    o = face + n * task.R_TIP + np.array([0.0, 0.0, 0.03])  # pressed on the bar's dry side, not its end
    task.record_press(3.0, task.FORMER.state(pos=o), rs); lift(); released()
    out.append(("pressing on the bar's dry side paints nothing", end(), 0.0))
    # contamination after painting
    fresh(); press(apex_dir(0), force=3.0); lift()
    fs = task.FORMER.state(pos=(0.30, 0.20, task.TOP + task.R_TIP - 0.0001),
                           quat_xyzw=task.parts._matrix_to_quat(task._rot_between(apex_dir(90), np.array([0, 0, -1.0]))))
    out.append(("after a first dab, an unpainted pin on the table is not ruled", float(task.tips_touch_table(fs, task._S["painted"])), 0.0))
    out.append(("... and the same pose with a painted pin down is", float(task.tips_touch_table(task.FORMER.state(
        pos=(0.30, 0.20, task.TOP + task.R_TIP - 0.0001)), task._S["painted"])), 1.0))
    fresh(); all_painted(); task._S["ruled"].add("painted pins touched the table"); released()
    out.append(("painted pins touched the table", end(), 0.0))
    fresh(); all_painted()
    fs = task.FORMER.state()
    pts = task.tips_world(fs)[:1] + np.array([0.0, 0.0, -0.0002])
    out.append(("a hand point on a painted pin is detected", float(task.hands_on_tips(pts, fs, task._S["painted"])), 1.0))
    rs, n, face = bar_state(0.0)
    out.append(("a hand point on the wet circle is detected", float(task.hands_in_paint((face + 0.0002 * n)[None], rs)),
                1.0))
    out.append(("a hand point 1 mm off the wet circle is not", float(task.hands_in_paint((face + 0.001 * n)[None], rs)),
                0.0))
    return out


def _former_at(dz=0.0, tilt_deg=0.0, dx=0.0):
    p = np.asarray(task.FORMER.pos) + np.array([dx, 0.0, dz])
    q = (math.sin(math.radians(tilt_deg) / 2), 0.0, 0.0, math.cos(math.radians(tilt_deg) / 2))
    return task.FORMER.state(pos=p, quat_xyzw=q)


lots.use("A")
_painted = np.ones(task.N_PINS, bool)


def _setup_all():
    task.reset_state()
    task._S.update(painted=_painted.copy(), wet=True, known=True, clear=True, hold=task.HOLD_STEPS, shown=set())


_setup_all()
_bk = np.asarray(task.BEAKER.pos)
_rp = np.asarray(task.BAR.pos)
_rq = tuple(task.BAR.state().quat_xyzw)
CASES = [
    ("every pin painted, former hung on the rim, bar back down", {}, 1.0, True),
    ("former held 3 mm above the rim (still in the hand)", {"former": _former_at(dz=0.003)}, 0.0, False),
    ("former hung but tilted 10 degrees", {"former": _former_at(tilt_deg=10.0)}, 0.0, False),
    ("former 14 mm off the beaker's axis, pins touching the glass", {"former": _former_at(dx=0.014)}, 0.0, False),
    ("former lying on the table", {"former": task.FORMER.state(pos=(0.30, 0.20, task.TOP + 0.020),
                                                               quat_xyzw=(0.7071, 0.0, 0.0, 0.7071))}, 0.0, False),
    ("bar stood on its painted end", {"bar": task.BAR.state(pos=(_rp[0], _rp[1], task.TOP + task.BAR_L / 2),
                                                            quat_xyzw=(0.0, 0.7071, 0.0, 0.7071))}, 0.0, False),
    ("bar stood on its dry end, painted end up", {"bar": task.BAR.state(
        pos=(_rp[0], _rp[1], task.TOP + task.BAR_L / 2), quat_xyzw=(0.0, -0.7071, 0.0, 0.7071))}, 1.0, True),
    ("bar left held 20 mm up", {"bar": task.BAR.state(pos=(_rp[0], _rp[1], _rp[2] + 0.02), quat_xyzw=_rq)}, 0.0,
     False),
    ("bar still moving", {"bar": task.BAR.state(pos=_rp, quat_xyzw=_rq, lin_vel=(0.05, 0.0, 0.0))}, 0.0, False),
    ("beaker pushed 20 mm (former still on it)", {
        "beaker": task.BEAKER.state(pos=_bk + np.array([0.02, 0.0, 0.0])),
        "former": task.FORMER.state(pos=np.asarray(task.FORMER.pos) + np.array([0.02, 0.0, 0.0]))}, 0.0, False),
    ("beaker pushed 14 mm (former still on it)", {
        "beaker": task.BEAKER.state(pos=_bk + np.array([0.014, 0.0, 0.0])),
        "former": task.FORMER.state(pos=np.asarray(task.FORMER.pos) + np.array([0.014, 0.0, 0.0]))}, 1.0, True),
]


if __name__ == "__main__":
    bad = 0
    for label, got, want in run_behaviour_cases():
        flag = "ok " if abs(got - want) < 1e-9 else "BAD"
        bad += flag == "BAD"
        print(f"{flag} {got:4.2f} (want {want:4.2f})  {label}")
    lots.use("B")
    _setup_all()
    b = {o.name: o.state() for o in task.OBJECTS}
    b["beaker"] = task.BEAKER.state(pos=lots.current().pos("beaker"))
    b["former"] = task.FORMER.state(pos=lots.current().pos("former"))
    b["bar"] = task.BAR.state(pos=lots.current().pos("bar"), quat_xyzw=(0.0, 0.0, -0.7071, 0.7071))
    got = task.score(b)
    print(f"{'ok ' if got == 1.0 else 'BAD'} {got:4.2f} (want 1.00)  lot B: every pin painted, all put back")
    bad += got != 1.0
    lots.use("A")
    _setup_all()
    print("behaviour cases:", "PASS" if not bad else f"{bad} BAD")
    sys.exit(1 if bad else 0)
else:
    _setup_all()
