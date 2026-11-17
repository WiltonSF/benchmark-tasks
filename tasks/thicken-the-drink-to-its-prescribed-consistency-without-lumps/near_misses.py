"""Near misses and unintended strategies for thickening the drink, in lot A (30 mL of thickener) unless stated.

The score rests on a history (the powder's tracked volumes, lumps, whether the hands were clear, the still hold),
so behaviour cases drive the scorer's own recorder (`task.record`, which the simulator calls every control step)
through a sequence of states. Most move the whisk and the scoop as if held rigidly by the demo's own targets
(their poses only), so the powder model itself decides what lands where, what is wetted and what lumps. CASES (run
by factory/check.py) then check end states on top of a full, valid history."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import lots  # noqa: E402
import task  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

lots.use("A")
DT = task.DT
P = task.parts


def q_of(R):
    """xyzw from a rotation matrix (Shepperd's method: exact for half turns too)."""
    m = np.asarray(R, float)
    tr = np.trace(m)
    k = int(np.argmax([tr, m[0, 0], m[1, 1], m[2, 2]]))
    if k == 0:
        w = math.sqrt(1 + tr) / 2
        x, y, z = (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w)
    elif k == 1:
        x = math.sqrt(1 + 2 * m[0, 0] - tr) / 2
        w, y, z = (m[2, 1] - m[1, 2]) / (4 * x), (m[0, 1] + m[1, 0]) / (4 * x), (m[0, 2] + m[2, 0]) / (4 * x)
    elif k == 2:
        y = math.sqrt(1 + 2 * m[1, 1] - tr) / 2
        w, x, z = (m[0, 2] - m[2, 0]) / (4 * y), (m[0, 1] + m[1, 0]) / (4 * y), (m[1, 2] + m[2, 1]) / (4 * y)
    else:
        z = math.sqrt(1 + 2 * m[2, 2] - tr) / 2
        w, x, y = (m[1, 0] - m[0, 1]) / (4 * z), (m[0, 2] + m[2, 0]) / (4 * z), (m[1, 2] + m[2, 1]) / (4 * z)
    return (float(x), float(y), float(z), float(w))


def start_pose(name):
    lot = lots.current()
    return np.array(lot.pos(name)), P._rot((0, 0, 1), lot.yaw_of(name))


SPEC = {"tumbler": task.TUMBLER, "whisk": task.WHISK, "scoop": task.SCOOP, "holder": task.HOLDER,
        "coaster": task.COASTER, "placemat": task.PLACEMAT}


def world(**poses):
    """States of every object: start poses unless given as name=(pos, R[, lin_vel])."""
    out = {}
    for name, spec in SPEC.items():
        if name in poses:
            p, R = poses[name][0], poses[name][1]
            v = poses[name][2] if len(poses[name]) > 2 else None
        else:
            p, R = start_pose(name) if spec.mass is not None else (np.array(spec.pos), np.eye(3))
            v = None
        out[name] = spec.state(pos=tuple(float(x) for x in p), quat_xyzw=q_of(R),
                               lin_vel=None if v is None else tuple(float(x) for x in v))
    return out


def T(p, R):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = R, p
    return m


def held_track(plan_wps, arm, obj, until=None, amp=1.0, centre=None):
    """The object's world pose at each control step while the arm's demo plan holds it rigidly (from the step the
    fingers have closed to the step they open), else None. `amp` scales the arm's motion about `centre` (to model
    the arm tracking a small stirring circle with less than its full amplitude)."""
    plan = ArmPlan(list(plan_wps), station.BASES[arm])
    n = int(round((until or plan.duration) / DT))
    p0, R0 = start_pose(obj)
    grip_T = None
    out = []
    for k in range(n):
        pos, rot, finger, label = plan.target((k + 1) * DT)
        if centre is not None and label == "stir":
            pos = centre + amp * (pos - centre)
        closed = finger > -0.010
        if closed and grip_T is None:
            grip_T = np.linalg.inv(T(pos, rot)) @ T(p0, R0)
        if not closed and grip_T is not None:
            out.append(None)
            continue
        out.append(None if grip_T is None else T(pos, rot) @ grip_T)
    return out


def play(no_stir=False, stir_amp=1.0, pour_speed=1.0, lot="A", stir_late=0.0, released=True,
         end=None, tumbler_shift=None):
    """Run the demo's object motion through the recorder. Options model unintended strategies."""
    lots.use(lot)
    task.reset_state()
    plans = task.demo()
    stir_arm, pour_arm = task._arms(lots.current())
    lt = lots.current()
    d = np.array([*lt.d, 0.0])
    sc_pts = np.array([w.pos for w in plans[stir_arm] if w.label == "stir"])
    sw = held_track(plans[stir_arm], stir_arm, "whisk", amp=stir_amp, centre=sc_pts.mean(axis=0))
    pw = plans[pour_arm]
    if pour_speed != 1.0:
        pw = [type(w)(w.duration / pour_speed if w.label.startswith("scoop") else w.duration, w.pos, w.finger,
                      w.label, yaw=float(w.yaw), tilt=getattr(w, "tilt", 0.0), heading=getattr(w, "heading", 0.0))
              for w in pw]
    sc = held_track(pw, pour_arm, "scoop")
    n = max(len(sw), len(sc)) + 40
    tum0, tumR = start_pose("tumbler")
    if tumbler_shift is not None:
        tum0 = tum0 + np.asarray(tumbler_shift)
    whisk_last, scoop_last = start_pose("whisk"), start_pose("scoop")
    w_rel_out = False
    for k in range(n):
        wm = sw[k] if k < len(sw) else None
        if no_stir and wm is not None:
            wm = None  # the whisk is never brought to the drink
        if stir_late and wm is not None and k * DT < stir_late:
            wm = None
        sm = sc[k] if k < len(sc) else None
        holding = wm is not None or sm is not None
        if wm is not None:
            whisk_last = (wm[:3, 3], wm[:3, :3])
        elif k > 0 and sw[min(k, len(sw) - 1)] is None and any(x is not None for x in sw[:k]) and not w_rel_out:
            # released into the holder: it drops to the floor of the cup
            hp, _ = start_pose("whisk")
            whisk_last = (hp, np.eye(3))
            w_rel_out = True
        if sm is not None:
            scoop_last = (sm[:3, 3], sm[:3, :3])
        elif any(x is not None for x in sc[:k]):
            p, R = scoop_last
            scoop_last = (np.array([p[0], p[1], task.MAT_Z]), R)
        st = world(tumbler=(tum0, tumR), whisk=whisk_last, scoop=scoop_last)
        task.record(st, (not holding) and released)
    return st


def scripted(seconds, whisk_tip, gamma, lot="A"):
    """Drive the whisk and the scoop directly: the whisk stands upright with its tip at whisk_tip(t) (tumbler frame,
    m, or None: lifted out of the drink), and the scoop is held over the tumbler tipped gamma(t) degrees. Every
    step is recorded with the hands not clear (still holding)."""
    lots.use(lot)
    task.reset_state()
    lt = lots.current()
    tum0, tumR = start_pose("tumbler")
    syaw = lt.yaw_of("scoop")
    st = None
    for k in range(int(round(seconds / DT))):
        t = (k + 1) * DT
        tip = whisk_tip(t)
        if tip is None:
            wp = (tum0 + [0.0, 0.0, 0.12], np.eye(3))
        else:
            wp = (tum0 + np.asarray(tip), np.eye(3))
        o, R = task.pour_pose(lt.d, lt.e, syaw, gamma(t))
        st = world(tumbler=(tum0, tumR), whisk=wp, scoop=(o, R))
        task.record(st, False)
    return st


TIP_Z = task.T_FLOOR + 0.006


def circle(r, hz, t0=0.0, cx=0.0):
    return lambda t: (cx + r * math.cos(2 * math.pi * hz * (t - t0)), r * math.sin(2 * math.pi * hz * (t - t0)), TIP_Z)


def ramp(t_start, t_end, g0, g1):
    return lambda t: g0 + (g1 - g0) * min(1.0, max(0.0, (t - t_start) / max(t_end - t_start, 1e-9)))


def finish(st=None, s=1.5, clear=True):
    st = st or world()
    for _ in range(round(s / DT)):
        task.record(st, clear)
    return st


RESULTS = []


def case(label, run, want):
    st = run()
    got = task.score(st if st is not None else world())
    if want is None:  # report only
        want = got
    RESULTS.append((label, got, want))
    L = task._L
    print(f"{'ok  ' if abs(got - want) < 1e-9 else 'FAIL'} {label}: {got} (want {want}) [lot {lots.current().name} "
          f"wetted {task.hydrated_share():.3f} dry {L['dry'].sum():.2f} lumps {L['lumps'].sum():.3f} spill "
          f"{L['spill']:.2f} scoop {L['scoop']:.2f} uniform {task.uniform()}]")


def made(share=1.0, even=True):
    """A history in which the drink was made: `share` of the dose wetted, even or all in one region."""
    task.reset_state()
    task._L["known"] = True
    dose = task._L["dose"]
    if even:
        task._L["hyd"][:] = share * dose / task.N_REG
    else:
        task._L["hyd"][3] = share * dose
    task._L["scoop"] = dose * (1 - share)


if True:
    case("demo, lot A", lambda: play(), 1.0)
    case("demo, lot B (45 mL, mirrored)", lambda: play(lot="B"), 1.0)
    case("demo, lot C (45 mL, tumbler moved)", lambda: play(lot="C"), 1.0)
    case("demo with the arm reaching 60% of its stirring amplitude", lambda: play(stir_amp=0.6), 1.0)
    case("demo with the arm reaching half its stirring amplitude", lambda: play(stir_amp=0.5), 1.0)
    case("demo with the arm reaching 40% of its stirring amplitude", lambda: play(stir_amp=0.4), 1.0)
    case("demo with the arm reaching 30% of its stirring amplitude", lambda: play(stir_amp=0.3), 0.0)
    case("scoop jumped to 42 degrees in one step under a 1 cm, 2 Hz whisk (a dump)",
         lambda: scripted(40.0, circle(0.010, 2.0), lambda t: 0.0 if t < 5.0 else 42.0), 0.0)
    case("scoop tipped from 18 to 42 degrees in 1 s under a 1 cm, 2 Hz whisk",
         lambda: scripted(40.0, circle(0.010, 2.0), ramp(5.0, 6.0, 18.0, 42.0)), 0.0)
    case("whisked hard, held still through a dump, restarted 3 s later",
         lambda: scripted(40.0, lambda t: circle(0.010, 2.0)(t) if (t < 5.0 or t > 8.0) else (0.010, 0.0, TIP_Z),
                          lambda t: 0.0 if t < 5.0 else 42.0), 0.0)
    case("scoop tipped 18 to 42 degrees over 58 s under a 1.2 cm, 1 Hz whisk (scripted control, still held)",
         lambda: scripted(85.0, circle(0.012, 1.0, cx=0.006), ramp(5.0, 63.0, 18.0, 42.0)), 0.75)
    case("scoop tipped over 44 s while the whisk only trembles +-1 mm every step",
         lambda: scripted(75.0, lambda t: (0.006 + 0.001 * (-1) ** round(t / DT), 0.0, TIP_Z), ramp(5.0, 49.0, 18.0, 42.0)),
         0.0)
    case("scoop tipped over 44 s, whisk swung in a 1.2 cm circle at 4 Hz (credited at 0.2 m/s, not its 0.3 m/s)",
         lambda: scripted(75.0, circle(0.012, 4.0, cx=0.006), ramp(5.0, 49.0, 18.0, 42.0)), None)
    case("poured slowly, never stirred", lambda: play(no_stir=True), 0.0)
    case("poured slowly, stirring only begins after the pour", lambda: play(stir_late=45.0), 0.0)
    case("poured twice as fast while stirring", lambda: play(pour_speed=2.0), 1.0)
    case("poured three times as fast while stirring", lambda: play(pour_speed=3.0), 0.0)
    case("poured four times as fast while stirring", lambda: play(pour_speed=4.0), 0.0)
    case("lot A's demo replayed with lot C's tumbler position", lambda: play(tumbler_shift=(-0.03, 0.07, 0.0)), 0.0)
    case("demo, but the hands are never seen clear", lambda: play(released=False), 0.75)

    def no_hands():
        play()
        task._L["known"] = False
        return world()

    case("demo, hand state unknown", no_hands, 0.0)
    case("nothing done", lambda: (task.reset_state(), finish())[1], 0.0)
    case("drink made, even, all set down", lambda: (made(), finish())[1], 1.0)
    case("exactly 95% wetted, even", lambda: (made(0.95), finish())[1], 1.0)
    case("94.9% wetted, even", lambda: (made(0.949), finish())[1], 0.25)
    case("exactly half wetted", lambda: (made(0.50), finish())[1], 0.25)
    case("49.9% wetted", lambda: (made(0.499), finish())[1], 0.0)
    case("all wetted, but in one region (not stirred even)", lambda: (made(1.0, even=False), finish())[1], 0.5)

    def uneven(f):
        made()
        task._L["hyd"][2] *= 1 + f
        task._L["hyd"][5] *= 1 - f
        return finish()

    case("regions within 10%", lambda: uneven(0.0999), 1.0)
    case("one region 10.5% over", lambda: uneven(0.105), 0.5)

    def dry_left(ml):
        made()
        task._L["dry"][1] = ml
        return finish()

    case("0.29 mL dry powder still floating (under 1%)", lambda: dry_left(0.29), 1.0)
    case("0.31 mL dry powder still floating", lambda: dry_left(0.31), 0.5)

    def lumps(ml):
        made()
        task._L["lumps"][4] = ml
        return finish()

    case("0.049 mL of lumps", lambda: lumps(0.049), 1.0)
    case("0.05 mL of lumps (one lump)", lambda: lumps(0.05), 0.0)

    def spilt(ml):
        made()
        task._L["spill"] = ml
        return finish()

    case("1.49 mL thickener on the mat (under 5%)", lambda: spilt(1.49), 1.0)
    case("1.5 mL thickener on the mat (5%)", lambda: spilt(1.5), 0.0)

    def tipped(deg):
        made()
        th = math.radians(deg)
        R = P._rot((1, 0, 0), th)
        st = world(tumbler=(start_pose("tumbler")[0], R))
        task.record(st, True)
        return finish()

    case("drink made, tumbler tipped 24 degrees once, set back", lambda: tipped(24.0), 1.0)
    case("drink made, tumbler tipped 26 degrees once (juice spilt), set back", lambda: tipped(26.0), 0.0)
    case("drink made, only 0.9 s still", lambda: (made(), finish(s=0.9))[1], 0.75)

    BAD = [r for r in RESULTS if abs(r[1] - r[2]) > 1e-9]
    if BAD:
        raise SystemExit(f"near_misses: {len(BAD)} behaviour case(s) failed: {BAD}")

# Hands: a gripper face lying 0.5 mm from the tumbler's wall with no vertex near it must not count as clear.
_tw = world()["tumbler"]
_x = float(_tw.pos[0]) - task.T_ROUT - 0.0005
_face = np.array([[_x, float(_tw.pos[1]) + dy, float(_tw.pos[2]) + 0.03 + dz] for dy in np.linspace(-0.006, 0.006, 5)
                  for dz in np.linspace(-0.006, 0.006, 5)])
_verts_far = _face[[0, 4, 20, 24]] + np.array([-0.0, 0.0, 0.0])
assert not task.points_clear(_face, world()), "a face 0.5 mm from the tumbler counted as clear"
assert task.points_clear(_face - np.array([0.01, 0.0, 0.0]), world()), "a face 10 mm away counted as touching"
_hulls = {}
from yam_arena import analysis  # noqa: E402

# A flat gripper face: the face sampler must put points in its middle, not just at its corners.
_h = analysis.link_hulls()["gripper"][0]
_pts = task._face_points(_h)
assert len(_pts) > 4 * len(_h.vertices), "face sampling added too few points"
print("ok   hands: a face 0.5 mm away is not clear; 10 mm away is clear; faces sampled "
      f"({len(_pts)} points on a {len(_h.vertices)}-vertex hull)")

# End states on top of a full, valid history (the drink made and even, 1.5 s set down and clear).
made()
finish()
TP, TR = start_pose("tumbler")
HP = np.array(lots.current().pos("whisk"))
SP, SR = start_pose("scoop")


def S(name, p, R, vel=None):
    return SPEC[name].state(pos=tuple(float(x) for x in p), quat_xyzw=q_of(R),
                            lin_vel=None if vel is None else tuple(float(x) for x in vel))


RX = lambda deg: P._rot((1, 0, 0), math.radians(deg))  # noqa: E731
CASES = [
    ("everything set down", {}, 1.0, True),
    ("tumbler 14 mm off its spot", {"tumbler": S("tumbler", TP + [0.014, 0, 0], TR)}, 1.0, True),
    ("tumbler 16 mm off its spot", {"tumbler": S("tumbler", TP + [0.016, 0, 0], TR)}, 0.75, False),
    ("tumbler leaning 6 degrees", {"tumbler": S("tumbler", TP, RX(6))}, 0.75, False),
    ("tumbler held 4 mm up", {"tumbler": S("tumbler", TP + [0, 0, 0.004], TR)}, 0.75, False),
    ("whisk left standing in the tumbler", {"whisk": S("whisk", TP + [0, 0, task.T_FLOOR], np.eye(3))}, 0.75, False),
    ("whisk lying on the mat", {"whisk": S("whisk", HP + [0.0, 0.08, 0.0], RX(90))}, 0.75, False),
    ("whisk in its cup, leaning 9 degrees", {"whisk": S("whisk", HP + [0.004, 0, 0], RX(9))}, 1.0, True),
    ("whisk in its cup, still moving", {"whisk": S("whisk", HP, np.eye(3), (0.05, 0, 0))}, 0.75, False),
    ("scoop on the mat 10 cm away", {"scoop": S("scoop", SP + [0.0, 0.10, 0.0], SR)}, 1.0, True),
    ("scoop held 4 mm up", {"scoop": S("scoop", SP + [0.0, 0.0, 0.004], SR)}, 0.75, False),
    ("scoop leaning 9 degrees", {"scoop": S("scoop", SP, RX(9) @ SR)}, 1.0, True),
    ("scoop on its side", {"scoop": S("scoop", SP + [0.0, 0.0, 0.03], RX(90) @ SR)}, 0.75, False),
    ("scoop still moving", {"scoop": S("scoop", SP, SR, (0.05, 0.0, 0.0))}, 0.75, False),
]
