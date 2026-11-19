"""Near-successes and unintended strategies, scored offline (run this file, or a checker that reads CASES). Each case:
(label, {object: state}, expected score, expected success). Cases run in order and share the whole-trial record,
which only a scene reset clears (in the simulator, the harness's reset; here, a "scene reset" case).

Offline there is no robot: the arms count as measured clear and touching nothing unless a case marks them otherwise
(in the simulator they are measured every step from face-sampled link hulls, and are unknown, no credit, until then).
Every case is in lot A unless it says otherwise.
"""

import dataclasses
import math

import numpy as np

import lots
import parts
import task
from task import CONSOLE, DOCK_PART, JOYCON, CON_START, HX, HZ, JHY, TY, LIE, STILL_STEPS

lots.use("A")
MM = 1e-3


def _offline_clear(objects):
    j = objects.get("joycon")
    pts = getattr(j, "_offline_points", None)
    if pts is not None:
        boxes = (task.envelope(CONSOLE, objects["console"]) + task.envelope(DOCK_PART, objects["dock"])
                 + task.envelope(JOYCON, j))
        return task._min_distance(pts, boxes) > task.CLEAR
    return getattr(j, "_offline_clear", True)


def _offline_touch(objects):
    j = objects.get("joycon")
    if getattr(j, "_offline_reset", False):
        task._clear_record()
    lots.use(getattr(j, "_offline_lot", "A"))
    pts = getattr(j, "_offline_points", None)
    if pts is not None:
        return {"screen": task._min_distance(pts, [task.glass_box(objects["console"])]) <= task.TOUCH}
    return getattr(j, "_offline_touch", {})


g = parts.__dict__.setdefault("_joycon_task_grip", {})
g["offline_clear"], g["offline_touch"] = _offline_clear, _offline_touch


def _mark(st, mark):
    for k, v in mark.items():
        object.__setattr__(st, f"_offline_{k}", v)
    return st


def _rot(axis, deg):
    return parts._rot(axis, math.radians(deg))


def scene(dx=0.0, dy=0.0, dz=0.0, latch=0.0, turn=None, con_dz=0.0, con_tilt=0.0, vz=0.0, lot="A", joy=None, **mark):
    """The Joy-Con offset (dx, dy, dz) from its seated place on the console (console frame; +y toward the console,
    +z up), turned by `turn` (a rotation in its seated frame), its catch pushed in `latch`; the console raised con_dz
    in its slot and tilted con_tilt degrees about the slot's length. `joy` = (pos, J rotation) places the Joy-Con
    anywhere instead. Keyword marks set offline stand-ins (clear=, touch=, points=, reset=)."""
    rc = _rot((0, 1, 0), con_tilt)
    low = np.array([CON_START[0], CON_START[1], CON_START[2] - HZ + con_dz])
    pc = low + rc @ np.array([0.0, 0.0, HZ])
    con = CONSOLE.state(pos=tuple(pc), quat_xyzw=tuple(parts._matrix_to_quat(rc)))
    if joy is None:
        rj = rc @ (np.eye(3) if turn is None else turn)
        pj = pc + rc @ (np.array([0.0, -TY - JHY, 0.0]) + [dx, dy, dz])
    else:
        pj, rj = np.asarray(joy[0], float), joy[1]
    j = JOYCON.state(joints={"latch": latch}, pos=tuple(pj), quat_xyzw=tuple(parts._matrix_to_quat(rj @ LIE.T)),
                     lin_vel=(0.0, 0.0, vz))
    return {"console": con, "dock": DOCK_PART.state(), "joycon": _mark(j, {"lot": lot, **mark})}


def reset():
    st = {"console": CONSOLE.state(), "dock": DOCK_PART.state(), "joycon": _mark(JOYCON.state(), {"reset": True})}
    return ("scene reset", st, 0.0, False)


def held(label, state, want, keep=False):
    """A state reached and held still 1 s, then one more step: credit needs the hold. Unless `keep`, a scene reset
    comes first (offline, jumping between cases would otherwise count as a drop or a lift)."""
    moving = dict(state)
    j = state["joycon"]
    moving["joycon"] = _mark(dataclasses.replace(j, pos=tuple(np.asarray(j.pos) + [0, 0, 0.001])),
                             {k[len("_offline_"):]: v for k, v in vars(j).items() if k.startswith("_offline_")})
    out = [] if keep else [reset()]
    out.append((f"{label}: still moving", moving, 0.0, False))
    out += [(f"{label} (still, step {k + 1})", state, 0.0, False) for k in range(STILL_STEPS)]
    out.append((f"{label} (still 1 s)", state, want, want == 1.0))
    out.append((f"{label} (still 1 s + 1 step)", state, want, want == 1.0))
    return out


def plate(gap, where="joy", st=None):
    """A flat arm face 120 x 120 mm, `gap` from a part: lying level over the seated Joy-Con's top end, or standing
    against the screen. Every corner is about 60 mm from its centre; only the face itself is near."""
    h = np.array([0.060, 0.060, 0.002]) if where == "joy" else np.array([0.030, 0.030, 0.002])
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
    planes = np.array([[1, 0, 0, h[0]], [-1, 0, 0, h[0]], [0, 1, 0, h[1]], [0, -1, 0, h[1]], [0, 0, 1, h[2]],
                       [0, 0, -1, h[2]]], float)
    pts = task._face_points(corners, planes)
    st = SEATED if st is None else st
    if where == "joy":
        top = np.asarray(st["joycon"].pos) + [0.0, 0.0, task.JHZ + 0.0002]  # the slider's top is flush; boxes
        lo, hi = JOYCON._local_box()
        centre = np.array([np.asarray(st["joycon"].pos)[0], np.asarray(st["joycon"].pos)[1], top[2]])
        # the Joy-Con's envelope top, plus the gap
        r = parts._quat_to_matrix(st["joycon"].quat_xyzw)
        corners_w = [np.asarray(st["joycon"].pos) + r @ np.array([a, b, c]) for a in (lo[0], hi[0])
                     for b in (lo[1], hi[1]) for c in (lo[2], hi[2])]
        centre[2] = max(c[2] for c in corners_w) + gap + h[2]
    else:  # vertical, facing the screen: turn the plate's normal from z to -x
        pts = pts @ parts._rot((0, 1, 0), math.pi / 2).T
        corners = corners @ parts._rot((0, 1, 0), math.pi / 2).T
        centre = np.asarray(st["console"].pos) + [-HX - 0.1e-3 - gap - h[2], 0.0, 0.0]
    return [pts + centre], [corners + centre]


SEATED = scene()
UPSIDE_DOWN = _rot((0, 1, 0), 180.0)  # top end down, stick to the back
BACKWARD = _rot((0, 0, 1), 180.0)  # rail facing away from the console
LYING_A = lots.all_lots()[0].pos("joycon")

CASES = [
    ("start: Joy-Con lying on the table", {}, 0.0, False),
    *held("seated, catch in its notch, released", SEATED, 1.0),
    *held("seated but still held (a pad within 5 mm)", scene(clear=False), 0.0),
    *held("seated, arms not yet measured (fail closed)", scene(clear=None), 0.0),
    *held("seated, a flat arm face 0.5 mm over the Joy-Con's top (no corner near)",
          scene(points=plate(0.0005)[0]), 0.0),
    *held("seated, the same face 6 mm over it: clear", scene(points=plate(0.006)[0]), 1.0),
    *held("1.0 mm above the stop, catch out", scene(dz=0.0010), 1.0),
    *held("1.01 mm above the stop: on the rail, not seated", scene(dz=0.00101), 0.5),
    *held("seated, catch pushed in 0.25 mm", scene(latch=0.00025), 1.0),
    *held("seated, catch pushed in 0.26 mm (not in its notch)", scene(latch=0.00026), 0.5),
    *held("stopped on the catch, 7 mm short, catch pushed in", scene(dz=0.007, latch=0.0006), 0.5),
    *held("on the rail with 30 mm of slider in the channel", scene(dz=0.069, latch=0.0), 0.5),
    *held("on the rail with 29.9 mm in the channel", scene(dz=0.0691), 0.0),
    *held("seated 0.6 mm off the rail line (x)", scene(dx=0.0006), 1.0),
    *held("0.61 mm off the rail line (x)", scene(dx=0.00061), 0.0),
    *held("seated 0.6 mm out from the console", scene(dy=-0.0006), 1.0),
    *held("pressed on sideways: level with seated, resting 2.9 mm out on the channel's lips", scene(dy=-0.0029), 0.0),
    *held("pressed on sideways, held there by friction 20 mm up", scene(dy=-0.0029, dz=0.020), 0.0),
    *held("seated, turned 2 degrees", scene(turn=_rot((1, 0, 0), 2.0)), 1.0),
    *held("seated, turned 2.1 degrees: on the rail only", scene(turn=_rot((1, 0, 0), 2.1)), 0.5),
    *held("upside down on the rail, 3 mm high (its slider's top end on the stop)",
          scene(dz=0.003, turn=UPSIDE_DOWN), 0.0),
    *held("backwards (rail away from the console) beside it", scene(dy=-0.006, turn=BACKWARD), 0.0),
    *held("resting on the console's top end", scene(dz=task.CH + 0.001, dy=0.010), 0.0),
    *held("lying on the table where it started", {"joycon": _mark(JOYCON.state(), {})}, 0.0),
    *held("seated, but the console raised 3.1 mm in the dock", scene(con_dz=0.0031), 0.0),
    *held("seated, the console raised 3 mm in the dock", scene(con_dz=0.0030), 1.0),
    *held("seated, the console tipped 3.5 degrees in its slot", scene(con_tilt=3.5), 0.0),
    *held("lot B: seated", scene(lot="B"), 1.0),
    reset(),
    ("the console lifted 5.1 mm out of the dock", scene(dz=0.050, con_dz=0.0051), 0.0, False),
    *held("then seated and released (a lift counts for the whole trial)", SEATED, 0.0, keep=True),
    reset(),
    ("the console lifted 5.0 mm", scene(dz=0.050, con_dz=0.0050), 0.0, False),
    *held("then seated and released", SEATED, 1.0, keep=True),
    reset(),
    ("the console tipped 11 degrees in the dock", scene(dz=0.050, con_tilt=11.0), 0.0, False),
    *held("then seated and released", SEATED, 0.0, keep=True),
    reset(),
    ("a pad face 0.5 mm from the screen glass", scene(dz=0.050, points=plate(0.0005, "screen")[0]), 0.0, False),
    *held("then seated and released (a screen touch counts for the whole trial)", SEATED, 0.0, keep=True),
    reset(),
    ("a pad face 1.5 mm from the screen glass (not touching)", scene(dz=0.050, points=plate(0.0015, "screen")[0]),
     0.0, False),
    *held("then seated and released", SEATED, 1.0, keep=True),
    reset(),
    ("dropped: the Joy-Con falling at 0.61 m/s", scene(dz=0.050, vz=-0.61), 0.0, False),
    *held("then seated and released", SEATED, 0.0, keep=True),
    reset(),
    ("slid down at 0.6 m/s", scene(dz=0.050, vz=-0.60), 0.0, False),
    *held("then seated and released", SEATED, 1.0, keep=True),
    reset(),
    ("arm state unknown for one step", scene(dz=0.050, touch=None), 0.0, False),
    *held("then seated and released", SEATED, 0.0, keep=True),
    *held("after the reset: seated and released", SEATED, 1.0),
]

_JB = task.envelope(JOYCON, SEATED["joycon"])
assert task._min_distance(plate(0.0005)[1], _JB) > task.CLEAR and task._min_distance(plate(0.0005)[0], _JB) < 0.001


def run() -> int:
    """Score every case in order, as the simulator does: one fresh state dict per control step, which success() and
    then score() both receive. Print and count the cases that differ from what they declare."""
    start = {"console": CONSOLE.state(), "dock": DOCK_PART.state(), "joycon": JOYCON.state()}
    bad = 0
    for label, changes, want, want_ok in CASES:
        st = dict(start, **changes)
        ok = bool(task.success(st))
        got = float(task.score(st))
        good = ok == want_ok and abs(got - want) < 1e-9
        bad += not good
        if not good or "(still 1 s)" in label or "still" not in label:
            print(("ok  " if good else "FAIL"), f"{got:.2f}", label)
    print(f"{len(CASES) - bad} of {len(CASES)} cases as declared")
    return bad


if __name__ == "__main__":
    raise SystemExit(run())
