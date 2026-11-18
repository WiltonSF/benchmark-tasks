"""Near-successes and unintended strategies, scored offline by the factory checker. Each case: (label, {object: spec},
expected score, expected success); a spec is a ready state (Part.state(...)). Cases run in order and share the
whole-trial record, cleared only by a scene reset.

Offline there is no robot and no physics: the arms count as measured clear unless a case marks them otherwise (in the
simulator they are measured every step, and are None, no credit, until then), and the press-fit spring's rest point
follows the entry layer (a released board) unless a case sets the push.
"""

import dataclasses
import math

import numpy as np

import parts
import task
from task import BOARD, CONTROLLER, CX, CY, HT, PIN_L, PITCH, STILL_STEPS


def _offline_clear(objects):
    b = objects.get("addon_board")
    pts = getattr(b, "_offline_points", None)
    if pts is not None:
        return task._arms_clear(pts, task.board_boxes(b))
    return getattr(b, "_offline_clear", True) if b is not None else True


parts.__dict__.setdefault("_hdr_task_grip", {})["offline_clear"] = _offline_clear


def _quat(yaw_deg=0.0, roll_deg=0.0, pitch_deg=0.0):
    r = (parts._rot((0, 0, 1), math.radians(yaw_deg)) @ parts._rot((1, 0, 0), math.radians(roll_deg))
         @ parts._rot((0, 1, 0), math.radians(pitch_deg)))
    return tuple(float(v) for v in parts._matrix_to_quat(r))


def board(dx=0.0, dy=0.0, depth=PIN_L, yaw=0.0, roll=0.0, pitch=0.0, push=0.0, **mark):
    """The add-on board over the header, its pins `depth` into the sockets (PIN_L: fully home; 0: tips on the
    entries), its centre (dx, dy) off the header's. `push` is how hard it is pressed (N, beyond what the entry layer
    has given way to). Keyword marks set offline stand-ins (clear=, points=)."""
    st = BOARD.state(pos=(CX + dx, CY + dy, HT + PIN_L - depth), quat_xyzw=_quat(yaw, roll, pitch))
    for k, v in mark.items():
        object.__setattr__(st, f"_offline_{k}", v)
    ctl = CONTROLLER.state(joints={"entry": max(depth, 0.0)})
    object.__setattr__(ctl, "_offline_push", push)
    return {"addon_board": st, "controller": ctl}


def _offline_rest(objects):
    """Offline: the spring's rest point is where the entry layer is, less the stated push."""
    ctl = objects.get("controller")
    q = float(ctl.joints["entry"]) if ctl is not None else 0.0
    return q - getattr(ctl, "_offline_push", 0.0) / task.K


parts.__dict__["_hdr_task_grip"]["offline_rest"] = _offline_rest


def held(label, state, want):
    """A state reached and held still 1 s, then one more step: full credit needs the hold."""
    moving = dict(state)
    if "addon_board" in state:
        b = state["addon_board"]
        moving["addon_board"] = dataclasses.replace(b, pos=tuple(np.asarray(b.pos) + [0, 0, 0.001]))
        for k, v in vars(b).items():
            if k.startswith("_offline_"):
                object.__setattr__(moving["addon_board"], k, v)
    out = [(f"{label}: still moving", moving, 0.0, False)]
    out += [(f"{label} (still, step {k + 1})", state, 0.0, False) for k in range(STILL_STEPS)]
    out.append((f"{label} (still 1 s)", state, want, False))
    out.append((f"{label} (still 1 s + 1 step)", state, want, want == 1.0))
    return out


def plate(gap):
    """A flat arm face 120 x 120 mm lying level just above the seated board's heat sink, `gap` above it: every corner
    is about 60 mm away, only the face itself is near."""
    h = np.array([0.060, 0.060, 0.001])
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
    planes = np.array([[1, 0, 0, h[0]], [-1, 0, 0, h[0]], [0, 1, 0, h[1]], [0, -1, 0, h[1]], [0, 0, 1, h[2]],
                       [0, 0, -1, h[2]]], float)
    centre = np.array([CX, CY, HT + task.BOARD_T + task.SINK_H + gap + h[2]])
    return [task._face_points(corners, planes) + centre], corners + centre


SEATED = board()
RESET = {}
C = task.CAPTURE
E = 1e-5

CASES = [
    ("start: board on its foam holder", {}, 0.0, False),
    *held("lifted and let go over the header, pins 3 mm above the sockets (nothing holds it there)",
          board(depth=-0.003), 0.0),
    *held("pin tips resting on the socket entries, lined up, released (not pressed)", board(depth=0.0), 0.0),
    *held("pressed 1 mm in (under 2 mm), released", board(depth=0.001), 0.0),
    *held("pressed 2 mm into the right sockets, released: started", board(depth=0.002), 0.5),
    *held("started but still held (a pad within 5 mm)", board(depth=0.003, clear=False), 0.0),
    *held("seated, arms not yet measured (fail closed)", board(clear=None), 0.0),
    *held("seated, a flat arm face 0.5 mm over the heat sink (no corner near)", board(points=plate(0.0005)[0]),
          0.0),
    *held("seated, the same face 6 mm over the heat sink: clear", board(points=plate(0.006)[0]), 1.0),
    *held("seated 0.4 mm high (over 0.3 mm)", board(depth=PIN_L - 0.0004), 0.5),
    *held("seated 0.3 mm high (limit)", board(depth=PIN_L - 0.0003), 1.0),
    *held("seated, every pin in its own socket, released", SEATED, 1.0),
    *held("offset by one row (2.54 mm across): fully down, one row outside the header", board(dx=PITCH), 0.0),
    *held("offset by one row the other way", board(dx=-PITCH), 0.0),
    *held("offset by one pin along the rows: fully down, one pair past the end", board(dy=PITCH), 0.0),
    *held("offset by one pin the other way", board(dy=-PITCH), 0.0),
    *held("turned 180 degrees (pin 1 on pin 40): fully down", board(yaw=180.0), 0.0),
    *held(f"seated {C * 1000:.1f} mm off across (limit of the lead-in)", board(dx=C), 1.0),
    *held(f"seated {C * 1000:.1f} mm off along (limit)", board(dy=-C), 1.0),
    ("scene reset", RESET, 0.0, False),
    *held(f"seated {C * 1000 + 0.01:.2f} mm off across (on the plastic, not in the sockets)", board(dx=C + E),
          0.0),
    ("scene reset", RESET, 0.0, False),
    *held("seated turned 1.6 degrees (end pins 0.71 mm off)", board(yaw=1.6), 0.0),
    ("scene reset", RESET, 0.0, False),
    *held("seated turned 1.5 degrees (end pins 0.66 mm off)", board(yaw=1.5), 1.0),
    ("scene reset", RESET, 0.0, False),
    ("pins on the plastic 1.27 mm off, pressed with 11 N (not bent)", board(dx=PITCH / 2, depth=0.0, push=11.0),
     0.0, False),
    *held("then lined up and seated", SEATED, 1.0),
    ("pins on the plastic 1.27 mm off, pressed with 12 N: bent", board(dx=PITCH / 2, depth=0.0, push=12.0), 0.0,
     False),
    *held("then lined up and seated (a bent pin counts for the whole trial)", SEATED, 0.0),
    ("scene reset", RESET, 0.0, False),
    *held("after the reset: seated and released", SEATED, 1.0),
    ("pressed in tilted 3.5 degrees across the rows, 1 mm in: bent", board(depth=0.001, pitch=3.5), 0.0, False),
    *held("then pressed level and seated", SEATED, 0.0),
    ("scene reset", RESET, 0.0, False),
    ("pressed in tilted 3.3 degrees along the rows, 1 mm in (not bent)", board(depth=0.001, roll=3.3), 0.0, False),
    *held("then seated level", SEATED, 1.0),
    ("seated, then dragged 1 mm sideways: bent", board(dx=0.001), 0.0, False),
    *held("pushed back over its sockets", SEATED, 0.0),
    ("scene reset", RESET, 0.0, False),
    *held("seated, then pulled off and dropped on the controller beside the header",
          {"addon_board": BOARD.state(pos=(CX + 0.04, CY, task.PCB_TOP + 0.006)),
           "controller": CONTROLLER.state(joints={"entry": 0.0})}, 0.0),
    *held("left lying on the table", {"addon_board": BOARD.state(pos=(0.25, 0.05, task.TOP + 0.006))}, 0.0),
]

_b = task.board_boxes(SEATED["addon_board"])
assert task._arms_clear([plate(0.0005)[1]], _b) and not task._arms_clear(plate(0.0005)[0], _b)
assert task._arms_clear(plate(0.006)[0], _b) and not task._arms_clear(plate(0.004)[0], _b)
