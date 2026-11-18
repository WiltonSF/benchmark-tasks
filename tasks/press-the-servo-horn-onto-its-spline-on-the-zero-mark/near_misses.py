"""Near-successes and unintended strategies, scored offline by the factory checker. Each case: (label, {object: spec},
expected score, expected success); a spec is a ready state (Part.state(...)). Cases run in order and share the
whole-trial record, cleared only by a scene reset.

Offline there is no robot and no physics: the arms count as measured clear unless a case marks them otherwise (in the
simulator they are measured every step, and are None, no credit, until then), and the press-fit spring's rest point
follows the crests (a released horn) unless a case sets the push.
"""

import dataclasses
import math

import numpy as np

import parts
import task
from task import B, HORN, INDEX, MARK_DEG, SERVO, STANDOFF, STILL_STEPS, SX, SY, TOP


def _offline_clear(objects):
    h = objects.get("horn")
    pts = getattr(h, "_offline_points", None)
    if pts is not None:
        return task._arms_clear(pts, task.horn_boxes(h))
    return getattr(h, "_offline_clear", True) if h is not None else True


parts.__dict__.setdefault("_horn_task_grip", {})["offline_clear"] = _offline_clear


def _offline_rest(objects):
    s = objects.get("servo")
    q = float(s.joints["crests"]) if s is not None else 0.0
    return q - getattr(s, "_offline_push", 0.0) / task.K


parts.__dict__["_horn_task_grip"]["offline_rest"] = _offline_rest


def _quat(yaw_deg=0.0, roll_deg=0.0):
    r = parts._rot((0, 0, 1), math.radians(yaw_deg)) @ parts._rot((1, 0, 0), math.radians(roll_deg))
    return tuple(float(v) for v in parts._matrix_to_quat(r))


def horn(a=0.0, depth=STANDOFF, dx=0.0, dy=0.0, tilt=0.0, push=0.0, lift=0.0, **mark):
    """The horn on the shaft, its arm `a` degrees from the zero mark, pressed `depth` onto the teeth (STANDOFF:
    home; 0: standing on the crests), `lift` above them, its axis (dx, dy) off the shaft's. Keyword marks set
    offline stand-ins (clear=, points=)."""
    z = TOP + B + STANDOFF - depth + lift
    st = HORN.state(pos=(SX + dx, SY + dy, z), quat_xyzw=_quat(MARK_DEG + a, tilt))
    for k, v in mark.items():
        object.__setattr__(st, f"_offline_{k}", v)
    sv = SERVO.state(joints={"crests": max(depth, 0.0) if lift <= 0.001 else 0.0})
    object.__setattr__(sv, "_offline_push", push)
    return {"horn": st, "servo": sv}


def held(label, state, want):
    """A state reached and held still 1 s, then one more step: full credit needs the hold."""
    moving = dict(state)
    h = state["horn"]
    moving["horn"] = dataclasses.replace(h, pos=tuple(np.asarray(h.pos) + [0, 0, 0.001]))
    for k, v in vars(h).items():
        if k.startswith("_offline_"):
            object.__setattr__(moving["horn"], k, v)
    out = [(f"{label}: still moving", moving, 0.0, False)]
    out += [(f"{label} (still, step {k + 1})", state, 0.0, False) for k in range(STILL_STEPS)]
    out.append((f"{label} (still 1 s)", state, want, False))
    out.append((f"{label} (still 1 s + 1 step)", state, want, want == 1.0))
    return out


def plate(gap):
    """A flat arm face 120 x 120 mm lying level `gap` above the seated horn's top: every corner is about 60 mm
    away, only the face itself is near."""
    h = np.array([0.060, 0.060, 0.001])
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
    planes = np.array([[1, 0, 0, h[0]], [-1, 0, 0, h[0]], [0, 1, 0, h[1]], [0, -1, 0, h[1]], [0, 0, 1, h[2]],
                       [0, 0, -1, h[2]]], float)
    centre = np.array([SX, SY, TOP + B + task.HUB_H + gap + h[2]])
    return [task._face_points(corners, planes) + centre], corners + centre


SEATED = horn()
RESET = {}
E = 1e-5
CRESTS = 0.0

CASES = [
    ("start: horn on its foam pad", {}, 0.0, False),
    *held("standing on the crests, arm on the mark, not pressed", horn(depth=CRESTS), 0.0),
    *held("standing on the crests as picked, 68 degrees off (4 off a tooth)", horn(a=-68.0, depth=CRESTS), 0.0),
    *held("pressed 0.9 mm onto the mark's tooth (under 1 mm), released", horn(depth=0.0009), 0.0),
    *held("pressed 1 mm onto the mark's tooth, released: started", horn(depth=0.001), 0.5),
    ("pulled off the teeth", horn(depth=0.0, lift=0.005), 0.0, False),
    *held("started but still held (a pad within 5 mm)", horn(depth=0.002, clear=False), 0.0),
    *held("seated, arms not yet measured (fail closed)", horn(clear=None), 0.0),
    *held("seated, a flat arm face 0.5 mm over the hub (no corner near)", horn(points=plate(0.0005)[0]), 0.0),
    *held("seated, the same face 6 mm over the hub: clear", horn(points=plate(0.006)[0]), 1.0),
    ("pulled off", horn(depth=0.0, lift=0.005), 0.0, False),
    *held("seated 0.4 mm high (over 0.3 mm)", horn(depth=STANDOFF - 0.0004), 0.5),
    *held("seated 0.3 mm high (limit)", horn(depth=STANDOFF - 0.0003), 1.0),
    *held("seated on the mark, released", SEATED, 1.0),
    *held("seated tilted 2.1 degrees (over 2)", horn(tilt=2.1), 0.5),
    *held("seated tilted 2.0 degrees", horn(tilt=2.0), 1.0),
    ("pulled off", horn(depth=0.0, lift=0.005), 0.0, False),
    *held(f"seated one tooth clockwise ({INDEX} degrees off the mark)", horn(a=INDEX), 0.0),
    ("pulled off", horn(depth=0.0, lift=0.005), 0.0, False),
    *held("seated one tooth the other way", horn(a=-INDEX), 0.0),
    ("pulled off", horn(depth=0.0, lift=0.005), 0.0, False),
    *held("seated five teeth round (72 degrees: the wiggle-on from as picked)", horn(a=-72.0), 0.0),
    ("pulled off the wrong tooth", horn(depth=0.0, lift=0.005), 0.0, False),
    *held("then refitted on the mark (no damage): judged by where it ends", SEATED, 1.0),
    ("pulled off", horn(depth=0.0, lift=0.005), 0.0, False),
    *held("seated 2.5 degrees off the mark (catch limit)", horn(a=2.5), 1.0),
    ("pulled off", horn(depth=0.0, lift=0.005), 0.0, False),
    *held("pressed on 2.6 degrees off the mark", horn(a=2.6), 0.0),
    ("scene reset", RESET, 0.0, False),
    ("seated one tooth off, then twisted 14.4 degrees onto the mark while on the teeth: stripped",
     horn(a=INDEX), 0.0, False),
    *held("now on the mark (after twisting)", SEATED, 0.0),
    ("pulled off", horn(depth=0.0, lift=0.005), 0.0, False),
    *held("refitted on the mark (the teeth are stripped for the whole trial)", SEATED, 0.0),
    ("scene reset", RESET, 0.0, False),
    *held("after the reset: seated and released", SEATED, 1.0),
    ("seated, then turned 4 degrees on the teeth (limit, not stripped)", horn(a=4.0), 0.0, False),
    *held("back on the mark", SEATED, 1.0),
    ("seated, then shoved 2.4 mm sideways: forced off the shaft", horn(dx=0.0024), 0.0, False),
    *held("pushed back", SEATED, 0.0),
    ("scene reset", RESET, 0.0, False),
    *held("seated 2.0 mm off the axis (limit)", horn(dy=0.002), 1.0),
    ("pulled off", horn(depth=0.0, lift=0.005), 0.0, False),
    *held("seated 2.01 mm off the axis", horn(dy=0.002 + E), 0.0),
    ("scene reset", RESET, 0.0, False),
    *held("left standing on the shaft's tip, 3 mm above the crests", horn(depth=0.0, lift=0.003), 0.0),
    *held("dropped on the servo's top", {"horn": HORN.state(pos=(SX - 0.02, SY, TOP + task.PLATE_T + task.CASE_H)),
                                         "servo": SERVO.state()}, 0.0),
    *held("left lying on the table", {"horn": HORN.state(pos=(0.25, 0.05, TOP))}, 0.0),
]

_b = task.horn_boxes(SEATED["horn"])
assert task._arms_clear([plate(0.0005)[1]], _b) and not task._arms_clear(plate(0.0005)[0], _b)
assert task._arms_clear(plate(0.006)[0], _b) and not task._arms_clear(plate(0.004)[0], _b)
