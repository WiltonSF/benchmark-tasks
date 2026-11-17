"""Near misses and unintended strategies for the ice-point calibration, in lot A (dial starts 6 °F high).

The score rests on a history (the stem's temperature, how long the sensing area has been in the water, whether the
hands were clear, the still hold), so behaviour cases drive the scorer's own recorder (`task.record`, which the
simulator callback calls every control step) through a sequence of states. Each behaviour case starts from a fresh
history. CASES (run by factory/check.py) then check end states on top of a full, valid history."""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import lots  # noqa: E402
import task  # noqa: E402

lots.use("A")
K, ICE = task.K, task.ICE_F
TH, CUP, WR = task.THERMO, task.CUP, task.WRENCH
RING_Z = task.TOP + task.RING_TOP
STEPS_PER_S = 20


def qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def thermo(reading=None, head=0.0, dz=0.0, dx=0.0, spin=0.0, tilt=0.0, vel=None):
    """The thermometer in its clip (lifted dz, moved dx, turned spin rad about its axis), its head turned `head` rad
    on the stem, or turned so the dial shows `reading` at the recorder's current stem temperature."""
    h = head if reading is None else K * (reading - task._S["temp"] - lots.current().error)
    q = qz(spin) if not tilt else qx(tilt)
    return TH.state(joints={"head": h}, pos=(task.XS + dx, task.YS, RING_Z + dz), quat_xyzw=q, lin_vel=vel)


def wrench_on_riser():
    return WR.state()


def wrench_on_nut():
    return WR.state(pos=(task.XS + 0.0005, task.YS, RING_Z + task.NUT_H / 2), quat_xyzw=qz(0.0))


def world(th=None, wr=None, cup=None):
    return {"thermometer": th if th is not None else thermo(), "wrench": wr if wr is not None else wrench_on_riser(),
            "cup": cup if cup is not None else CUP.state()}


def steps(seconds, make, clear=True):
    """Record `seconds` of control steps; make() builds each step's states (so the pointer follows the stem's
    temperature as it changes). Returns the last states."""
    st = None
    for _ in range(round(seconds * STEPS_PER_S)):
        st = make()
        task.record(st, clear)
    return st


RESULTS = []


def case(label, run, want):
    task.reset_state()
    st = run()
    got = task.score(st) if st is not None else task.score(world())
    RESULTS.append((label, got, want))
    print(f"{'ok  ' if abs(got - want) < 1e-9 else 'FAIL'} {label}: {got} (want {want})")


def settle(s=32.0):
    """The wrench on the nut, the head held still, for s seconds of ice water (the hands are on the parts)."""
    return steps(s, lambda: world(thermo(head=0.0), wrench_on_nut()), clear=False)


def finish(reading, hold=1.5, **kw):
    """The head set so the dial shows `reading`, everything let go and still for `hold` seconds."""
    return steps(hold, lambda: world(thermo(reading=reading, **kw)))


case("full: settled 32 s, set to 32.0 °F, let go 1.5 s", lambda: (settle(), finish(32.0))[1], 1.0)
case("set to 34.0 °F exactly (edge of the 2 °F band)", lambda: (settle(), finish(34.0))[1], 1.0)
case("set to 29.9 °F", lambda: (settle(), finish(29.9))[1], 0.5)
case("set to 34.1 °F", lambda: (settle(), finish(34.1))[1], 0.5)
case("set to 36.0 °F exactly (edge of the 4 °F band)", lambda: (settle(), finish(36.0))[1], 0.5)
case("set to 36.1 °F", lambda: (settle(), finish(36.1))[1], 0.0)
case("nothing done: dial left 6 °F high, all still 40 s", lambda: steps(40, lambda: world(thermo(head=0.0))), 0.0)
case("hands state never read (fail closed)", lambda: None, 0.0)
case("turned the wrong way by the error: reads 44 °F",
     lambda: (settle(), finish(None, head=+6.0 * K))[1], 0.0)
case("whole thermometer spun 0.5 rad by the head (nut not held): reading unchanged",
     lambda: (settle(), finish(None, head=0.0, spin=0.5))[1], 0.0)
def _set_early():
    steps(3, lambda: world(thermo(head=0.0)))
    h = thermo(reading=ICE).joints["head"]  # the dial made to read 32 °F while the stem is still warm
    return steps(37, lambda: world(thermo(head=h)))


def _set_warm(back_s):
    settle(5)
    steps(20, lambda: world(thermo(head=0.0, dz=0.060)), clear=False)  # lifted 60 mm: the sensing area is out
    h = thermo(reading=ICE, dz=0.060).joints["head"]  # set to read 32 °F out of the water
    return steps(back_s, lambda: world(thermo(head=h)))


case("one hand, no wrench: pinched, pushed down and twisted 12 degrees; the friction fit (0.08 N m) holds, so the "
     "whole thermometer turned in the ring and the dial still reads 6 °F high",
     lambda: (settle(), finish(None, head=0.0, spin=math.radians(12.3)))[1], 0.0)
case("set to read 32 at 3 s while the stem was still settling, let go, checked at 40 s", _set_early, 0.0)
case("lifted 60 mm out, set to 32 while warm, put back and let go 40 s", lambda: _set_warm(40), 0.0)
case("lifted 60 mm out, set to 32 while warm, put back and let go 1.5 s", lambda: _set_warm(1.5), 0.0)
case("set to 32 then lifted 45 mm and let go there (sensing area partly out)",
     lambda: (settle(), steps(5, lambda: world(thermo(head=-6.0 * K, dz=0.045))))[1], 0.0)
case("set to 32, put back in the clip only 25 s before the end",
     lambda: (settle(), steps(5, lambda: world(thermo(head=-6.0 * K, dz=0.045)), clear=False),
              steps(25, lambda: world(thermo(head=-6.0 * K))))[2], 0.0)
case("set to 32, a hand within 5 mm (not let go)", lambda: (settle(), steps(2, lambda: world(thermo(reading=32.0)),
                                                                          clear=False))[1], 0.0)
case("set to 32, let go only 0.95 s", lambda: (settle(), finish(32.0, hold=0.95))[1], 0.0)
case("set to 32, wrench left on the nut", lambda: (settle(), steps(1.5, lambda: world(thermo(reading=32.0),
                                                                                        wrench_on_nut())))[1], 0.0)
case("set to 32, cup tipped 12 degrees once (spilled), then stood back",
     lambda: (settle(), steps(0.1, lambda: world(thermo(reading=32.0), cup=CUP.state(quat_xyzw=qx(12.0)))),
              finish(32.0))[2], 0.0)
case("set to 32, thermometer swinging", lambda: (settle(), finish(32.0, vel=(0.05, 0.0, 0.0)))[1], 0.0)


def _plate_over_handle():
    """A flat 40 x 40 mm, 4 mm thick hand face lying 0.5 mm above the wrench's handle, centred on it, so every corner
    is about 28 mm from the handle and only the face is near."""
    import numpy as np
    from yam_arena import analysis

    w = wrench_on_riser()
    top = float(w.high[2])
    hx, hy = task.W_JAW0[0] - 0.1035, task.W_JAW0[1]  # the handle's centre
    lo = np.array([hx - 0.02, hy - 0.02, top + 0.0005])
    hi = np.array([hx + 0.02, hy + 0.02, top + 0.0045])
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6),
             (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    hull = analysis._hull_from_triangles(np.array([[corners[i] for i in f] for f in faces]))
    st = world()
    vertex_clear = task.clear_of_parts(hull.vertices, st)
    face_clear = task.clear_of_parts(task.surface_points(hull), st)
    assert vertex_clear and not face_clear, (vertex_clear, face_clear)
    return face_clear


def _face_on_wrench():
    settle()
    clear = _plate_over_handle()
    return steps(1.5, lambda: world(thermo(reading=32.0)), clear=clear)


case("set to 32, a flat hand face 0.5 mm above the wrench (no vertex within 5 mm): not let go", _face_on_wrench, 0.0)

# The one scripted catch (the wrench holds the nut rigidly only while its jaw is seated): which poses count as seated.
def _seat(label, wr_pos_off=(0.0, 0.0, 0.0), wr_yaw=0.0, th_yaw=0.0, want=True):
    th = thermo(spin=th_yaw)
    p = (task.XS + wr_pos_off[0], task.YS + wr_pos_off[1], RING_Z + task.NUT_H / 2 + wr_pos_off[2])
    wr = WR.state(pos=p, quat_xyzw=qz(wr_yaw))
    got = task.wrench_seated(th, wr)
    RESULTS.append((label, float(got), float(want)))
    print(f"{'ok  ' if got == want else 'FAIL'} catch: {label}: seated={got} (want {want})")


_seat("jaw fully on the nut, square", want=True)
_seat("jaw 1.5 mm short of fully on (edge of the tolerance)", wr_pos_off=(-0.0015, 0.0, 0.0), want=True)
_seat("jaw half on the nut (5.5 mm short): no hold, the nut can turn in it", wr_pos_off=(-0.0055, 0.0, 0.0),
      want=False)
_seat("jaw 3 mm above the nut's mid-height", wr_pos_off=(0.0, 0.0, 0.003), want=False)
_seat("jaw on the nut but 8 degrees off its flats", wr_yaw=math.radians(8.0), want=False)
_seat("wrench seated, handle turned 20 degrees with the nut following (still seated: the nut turns with the "
      "wrench)", wr_yaw=math.radians(20.0), th_yaw=math.radians(20.0), wr_pos_off=(0.0, 0.0, 0.0), want=True)
_seat("seated on the next pair of flats (60 degrees round)", wr_yaw=math.radians(60.0), want=True)

# Doing nothing in every lot: each dial starts more than 4 °F off (A +6, B -9, C -4.5), so it scores 0.
for _lot in ("A", "B", "C"):
    lots.use(_lot)
    case(f"lot {_lot}: nothing done, dial left at its error, all still 40 s",
         lambda: steps(40, lambda: world(thermo(head=0.0))), 0.0)
lots.use("A")

BAD = [r for r in RESULTS if abs(r[1] - r[2]) > 1e-9]
if BAD:
    raise SystemExit(f"near_misses: {len(BAD)} behaviour case(s) failed: {BAD}")


# End states on top of a full, valid history (32 s settled with the wrench on, then set to 32 °F and let go 1.5 s).
# factory/check.py scores each against that history; objects not listed stay at their start.
case("history for CASES", lambda: (settle(), finish(32.0))[1], 1.0)
_RIGHT = thermo(reading=32.0)
CASES = [
    ("set to 32, let go, wrench on the riser", {"thermometer": _RIGHT}, 1.0, True),
    ("set to 33.9 °F", {"thermometer": thermo(reading=33.9)}, 1.0, True),
    ("set to 29.5 °F", {"thermometer": thermo(reading=29.5)}, 0.5, False),
    ("set to 27.9 °F", {"thermometer": thermo(reading=27.9)}, 0.0, False),
    ("nut 1.9 mm above the ring (still resting on it)", {"thermometer": thermo(reading=32.0, dz=0.0019)}, 1.0, True),
    ("nut 2.5 mm above the ring (held up)", {"thermometer": thermo(reading=32.0, dz=0.0025)}, 0.0, False),
    ("stem 3 mm off the ring's centre (out of the clip)", {"thermometer": thermo(reading=32.0, dx=0.003)}, 0.0,
     False),
    ("thermometer leaning 9 degrees", {"thermometer": thermo(reading=32.0, tilt=9.0)}, 0.0, False),
    ("wrench left on the nut", {"thermometer": _RIGHT, "wrench": wrench_on_nut()}, 0.0, False),
    ("wrench put down 8 mm from the head", {"thermometer": _RIGHT, "wrench": WR.state(
        pos=(task.XS, task.YS + 0.023 + 0.008 + 0.0102, RING_Z + 0.011), quat_xyzw=qz(0.0))}, 0.0, False),
    ("wrench set down on the table by the cup",
     {"thermometer": _RIGHT, "wrench": WR.state(pos=(0.20, 0.05, task.TOP + 0.004))}, 1.0, True),
    ("thermometer lifted out and laid on the table", {"thermometer": TH.state(
        pos=(0.20, -0.20, task.TOP + task.HEAD_R), quat_xyzw=qx(90.0))}, 0.0, False),
]
