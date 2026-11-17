"""Near misses and unintended strategies for emptying the bedpan into the graduate, in lot A (250 mL).

The score rests on a history (the urine's tracked volumes, whether the hands were clear, the still hold), so behaviour
cases drive the scorer's own recorder (`task.record`, which the simulator calls every control step) through a
sequence of states. Some move the pan through whole pours (its poses only, as if held rigidly), so the liquid model
itself decides what lands in the graduate. CASES (run by factory/check.py) then check end states on top of a full,
valid history."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import lots  # noqa: E402
import task  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

lots.use("A")
PAN, GRAD = task.BEDPAN, task.GRADUATE
V0 = lots.current().volume
STEPS_PER_S = 20


def qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def pan_at(dx=0.0, dy=0.0, dz=0.0, yaw=0.0, tilt=0.0, vel=None):
    x, y, z = lots.current().pos("bedpan")
    q = qz(yaw) if not tilt else qx(tilt)
    return PAN.state(pos=(x + dx, y + dy, z + dz), quat_xyzw=q, lin_vel=vel)


def grad_at(dx=0.0, dz=0.0, tilt=0.0):
    return GRAD.state(pos=(task.G_XY[0] + dx, task.G_XY[1], task.TOP + dz), quat_xyzw=qx(tilt) if tilt else None)


def world(pan=None, grad=None):
    return {"bedpan": pan if pan is not None else pan_at(), "graduate": grad if grad is not None else grad_at(),
            "underpad": task.UNDERPAD.state()}


def steps(seconds, make, clear=True):
    st = None
    for _ in range(round(seconds * STEPS_PER_S)):
        st = make()
        task.record(st, clear)
    return st


def poured(grad_ml, spill_ml=0.0):
    """The history of a pour that ended with grad_ml in the graduate and spill_ml spilled (the rest in the pan)."""
    task._L.update(grad=grad_ml, spill=spill_ml, pan=max(0.0, V0 - grad_ml - spill_ml), known=True)


def _maa(R):
    a = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))
    if a < 1e-9:
        return np.array([1.0, 0.0, 0.0]), 0.0
    return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(a)), a


def play(frames, grad=None, speed=1.0):
    """Move the pan through (duration s, origin, rotation) keyframes as if held rigidly, recording every step."""
    o_prev, R_prev = frames[0][1], frames[0][2]
    prev = None
    for du, o, R in frames[1:]:
        n = max(1, int(round(du / speed / 0.05)))
        ax, an = _maa(R_prev.T @ R)
        for k in range(1, n + 1):
            f = k / n
            s = f * f * (3 - 2 * f)
            Rk = R_prev @ axis_angle_matrix(ax, an * s)
            ok = o_prev + (o - o_prev) * s
            vel = np.zeros(3) if prev is None else (ok - prev) / 0.05
            prev = ok
            p = PAN.state(pos=tuple(ok), quat_xyzw=tuple(task.parts._matrix_to_quat(Rk)), lin_vel=tuple(vel))
            task.record(world(p, grad), False)
        o_prev, R_prev = o, R


def demo_frames():
    return task._plan_poses()[0]


def pour_over(c_loc, n_loc, step_s=2.0):
    """Tip the pan slowly over the given rim point (part frame) and its outward direction, held over the graduate."""
    psi = math.pi - math.atan2(n_loc[1], n_loc[0])
    o0 = np.array(lots.current().pos("bedpan"))
    gx, gy = task.G_XY

    def at(cx, cz, th):
        R = task._pan_rot(psi, th)
        return np.array([gx + cx, gy, task.TOP + cz]) - R @ c_loc, R

    fr = [(0.0, o0, task._pan_rot(0.0, 0.0)), (3.0, *at(0.11, 0.19, 0.0))]
    for th, cx, cz in ((10, 0.11, 0.182), (20, 0.10, 0.177), (24, 0.07, 0.173), (26, 0.018, 0.172), (36, 0.018, 0.165),
                       (48, 0.018, 0.158), (60, 0.018, 0.154), (68, 0.018, 0.152), (80, 0.018, 0.150)):
        fr.append((step_s, *at(cx, cz, th)))
    fr.append((5.0, *at(0.018, 0.150, 80)))
    return fr


def finish(clear=True, s=1.5):
    return steps(s, world, clear)


RESULTS = []


def case(label, run, want):
    task.reset_state()
    task._L["known"] = True
    st = run()
    got = task.score(st if st is not None else world())
    RESULTS.append((label, got, want))
    print(f"{'ok  ' if abs(got - want) < 1e-9 else 'FAIL'} {label}: {got} (want {want}) "
          f"[in {task._L['grad']:.1f} spilled {task._L['spill']:.1f} mL]")


SIDE = (np.array([-task.IN_HX, 0.0, task.PH]), np.array([-1.0, 0.0]))  # the front (long) wall's middle
END = (np.array([0.0, -task.IN_HY, task.PH]), np.array([0.0, -1.0]))  # the open end wall's middle
CORNER = (task._corner(-1.0), np.array([-1.0, -1.0]) / math.sqrt(2))

case("demo's pour, pan back on its pad, let go, still", lambda: (play(demo_frames()), finish())[1], 1.0)
case("demo's pour, still holding the pan", lambda: (play(demo_frames()), finish(clear=False))[1], 0.5)
case("demo's pour twice as fast", lambda: (play(demo_frames(), speed=2.0), finish())[1], 1.0)
case("corner pour, tipped in 3 s (slops over)", lambda: (play(pour_over(*CORNER, step_s=0.33)), finish())[1], 0.0)
case("tipped over the long front side, slowly", lambda: (play(pour_over(*SIDE)), finish())[1], 0.0)
case("tipped over the open end, slowly", lambda: (play(pour_over(*END)), finish())[1], 0.0)
case("corner pour, slowly, over 80 degrees", lambda: (play(pour_over(*CORNER)), finish())[1], 1.0)
case("pan turned upside down over the graduate", lambda: (play([
    (0.0, np.array(lots.current().pos("bedpan")), np.eye(3)),
    (3.0, np.array([task.G_XY[0], task.G_XY[1], task.TOP + 0.25]), axis_angle_matrix(np.array([1.0, 0, 0]), math.pi))]),
    finish())[1], 0.0)
case("good pour, then the graduate knocked over", lambda: (play(demo_frames()),
                                                           steps(1.0, lambda: world(grad=grad_at(tilt=80.0)), True))[1],
     0.0)
case("nothing done", lambda: finish(), 0.0)
case("pan carried around, never tipped", lambda: (play(demo_frames()[:9]), finish())[1], 0.0)
case("97% in, 1.2% spilled, all set down", lambda: (poured(242.5, 3.0), finish())[1], 1.0)
case("exactly 95% in", lambda: (poured(0.95 * V0), finish())[1], 1.0)
case("94.9% in", lambda: (poured(0.949 * V0), finish())[1], 0.25)
case("spill just under 2%", lambda: (poured(0.96 * V0, 0.0199 * V0), finish())[1], 1.0)
case("spill exactly 2%", lambda: (poured(0.96 * V0, 0.02 * V0), finish())[1], 0.0)
case("half poured (50%), none spilled", lambda: (poured(0.50 * V0), finish())[1], 0.25)
case("49.9% poured", lambda: (poured(0.499 * V0), finish())[1], 0.0)
case("full pour but only 0.9 s still", lambda: (poured(0.97 * V0), finish(s=0.9))[1], 0.5)
case("full pour, pan left on the graduate side of the table",
     lambda: (poured(0.97 * V0), steps(1.5, lambda: world(pan=pan_at(dx=-0.26, dz=-0.002)), True))[1], 0.5)



def face_on_graduate_rim():
    """A flat hand face 0.5 mm above the graduate's rim, 200 mm across, so its corners are over 5 mm from every shape:
    a check on corners alone would call the hand clear. The scorer samples the face itself, so it is not clear."""
    tri = np.array([[0.115, 0.0, 0.0], [-0.0575, 0.0996, 0.0], [-0.0575, -0.0996, 0.0]])
    task._HAND_PTS["_flat_face"] = task.sample_triangles([tri])
    st = world()
    pose = np.eye(4)
    pose[:3, 3] = (task.G_XY[0], task.G_XY[1], task.TOP + task.GH + 0.0005)
    from yam_arena import analysis
    corners = tri + pose[:3, 3]
    far = min(analysis.box_signed_distance(corners, np.asarray(c), np.asarray(r), np.asarray(s)).min()
              for spec, name in ((PAN, "bedpan"), (GRAD, "graduate"))
              for _, c, r, s in spec.world_boxes({"base": (np.asarray(st[name].pos), task._R(st[name].quat_xyzw))}))
    assert far > task.CLEAR + 0.001, f"face corners only {far * 1000:.1f} mm away"
    return task.hands_clear_of([("_flat_face", pose)], st)


case("full pour, a flat hand face 0.5 mm above the graduate's rim (corners far off)",
     lambda: (poured(0.97 * V0), steps(1.5, world, face_on_graduate_rim()))[1], 0.5)

BAD = [r for r in RESULTS if abs(r[1] - r[2]) > 1e-9]
if BAD:
    raise SystemExit(f"near_misses: {len(BAD)} behaviour case(s) failed: {BAD}")

# End states on top of a full, valid history (97% in the graduate, nothing spilled, 1.5 s set down and clear).
case("history for CASES", lambda: (poured(0.97 * V0), finish())[1], 1.0)
CASES = [
    ("pan on its pad, graduate standing", {"bedpan": pan_at()}, 1.0, True),
    ("pan 39 mm off its spot", {"bedpan": pan_at(dx=0.039)}, 1.0, True),
    ("pan 42 mm off its spot", {"bedpan": pan_at(dx=0.042)}, 0.5, False),
    ("pan turned round (handle the other way)", {"bedpan": pan_at(yaw=math.pi)}, 1.0, True),
    ("pan turned 20 degrees", {"bedpan": pan_at(yaw=math.radians(20))}, 0.5, False),
    ("pan held 4 mm up", {"bedpan": pan_at(dz=0.004)}, 0.5, False),
    ("pan leaning 6 degrees", {"bedpan": pan_at(tilt=6.0, dz=0.012)}, 0.5, False),
    ("pan still moving", {"bedpan": pan_at(vel=(0.05, 0.0, 0.0))}, 0.5, False),
    ("graduate leaning 4.9 degrees", {"graduate": grad_at(tilt=4.9)}, 1.0, True),
    ("graduate leaning 5.5 degrees", {"graduate": grad_at(tilt=5.5)}, 0.5, False),
    ("graduate held 4 mm up", {"graduate": grad_at(dz=0.004)}, 0.5, False),
    ("graduate moved 10 cm, standing", {"graduate": grad_at(dx=-0.10)}, 1.0, True),
]

