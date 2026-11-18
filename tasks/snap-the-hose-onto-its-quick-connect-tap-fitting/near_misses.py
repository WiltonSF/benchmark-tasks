"""Near-successes and unintended strategies, scored offline by the factory checker. Each case: (label, {object: spec},
expected score, expected success); a spec is a ready state from the part's own geometry (PLUG.state(...)). Cases run
in order and share the whole-trial record, cleared only by a scene reset.

Offline there is no robot and no physics, so the arms are set here as measured clear (in the simulator they are
measured every step, and are None, no full credit, until then), and the pull test's response is given as the state the
hose end takes while it is pulled: a locked connector comes out by its slack, an unlocked one keeps coming.
"""

import math

import numpy as np

import parts
import task
from task import (_AXIS_OUT, _C_POS, _C_ROT, ABORT, CLEAR, COLLAR_OUT, COUPLER, GIVE_MAX, GIVE_MIN, HOME, NOSE_L, PAST_PINS, PLUG, PULL_N,
                  PULL_STEPS, SETTLE_STEPS, STILL_STEPS, TOP, YAW_C)

# Offline stand-ins, read from marks on a case's hose_end state: the arms' measured clearance (default clear) and
# whether the pull's force was really set (default yes).
def _offline_clear(objects):
    """A case's marked clearance; or, for a case carrying arm hull points, the real check on them."""
    pts = getattr(objects["hose_end"], "_offline_points", None)
    if pts is not None:
        return task._arms_clear(pts, task._boxes({"coupler": COUPLER.state(), **objects}))
    return getattr(objects["hose_end"], "_offline_clear", True)


parts.__dict__.setdefault("_hose_task_grip", {})["offline_clear"] = _offline_clear
parts.__dict__.setdefault("_hose_task_pull", {"phase": "idle"})["offline_applied"] = (
    lambda objects: 0.0 if getattr(objects["hose_end"], "_offline_no_force", False) else PULL_N)

Q_HOME = tuple(parts._matrix_to_quat(_C_ROT))


def plug(f=0.0, side=0.0, up=0.0, tilt=0.0, vel=None, pos=None, quat=None):
    """The hose end with its flange face f into the mouth (negative: in front of it), offset sideways/up from the
    bore's axis, its axis tilted `tilt` degrees (in the horizontal plane)."""
    if pos is None:
        pos = _C_POS + _C_ROT @ np.array([f, side, up])
    if quat is None:
        r = _C_ROT @ parts._rot((0, 0, 1), math.radians(tilt))
        quat = parts._matrix_to_quat(r)
    return {"hose_end": PLUG.state(pos=tuple(pos), quat_xyzw=tuple(quat), lin_vel=vel)}


def no_force(state):
    object.__setattr__(state["hose_end"], "_offline_no_force", True)
    return state


GIVE = 0.0004  # a locked connector comes out by its groove's slack under the pull
HOME_S = plug()
RESET = {}


def grip(flag):
    """A home state with the arms' measured clearance set to flag (None: not measured; False: a gripper within 5 mm)."""
    st = plug()
    object.__setattr__(st["hose_end"], "_offline_clear", flag)
    return st


def plate(gap):
    """A flat arm face 120 x 120 mm lying level over the collar, `gap` above its top: every corner of it is about
    60 mm from the collar, only the face itself is near. Returned as the face sample points the scorer uses."""
    h = np.array([0.060, 0.060, 0.001])
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
    planes = np.array([[1, 0, 0, h[0]], [-1, 0, 0, h[0]], [0, 1, 0, h[1]], [0, -1, 0, h[1]], [0, 0, 1, h[2]],
                       [0, 0, -1, h[2]]], float)
    centre = _C_POS + np.array([0.0, 0.0, COLLAR_OUT + gap + h[2]])
    return [task._face_points(corners, planes) + centre], corners + centre


def plate_state(gap):
    st = plug()
    object.__setattr__(st["hose_end"], "_offline_points", plate(gap)[0])
    return st


def tested(label, state, score, pulled=None, success=None, steps=PULL_STEPS):
    """A state reached and held still for 1 s, then put through the pull test (the hose end in `pulled` while it is
    pulled; default: a locked connector's slack), then SETTLE_STEPS steps released, after which the score is given.
    Each case starts from the hose end 5 mm out (moving), which also restarts the test."""
    pulled = plug(f=-GIVE) if pulled is None else pulled
    pre = None
    out = [(f"{label}: still moving", {**state, **plug(vel=(0.0, 0.05, 0.0))}, pre, False)]
    out += [(f"{label} (step {k + 1} of the 1 s hold)", state, pre, False) for k in range(STILL_STEPS - 1)]
    out.append((f"{label} (held 1 s: the pull test starts)", state, pre, False))
    out += [(f"{label} (pulled, step {k + 1})", pulled, pre, False) for k in range(steps)]
    out += [(f"{label} (pull off, step {k + 1})", state, pre, False) for k in range(SETTLE_STEPS - 1)]
    out.append((f"{label} (pull test done)", state, score, False))
    out.append((f"{label} (pull test done + 1 step)", state, score, score == 1.0 if success is None else success))
    return out


def _fill(cases):
    """Cases with expected score None take the running partial-credit tier (0.5 once home has been reached)."""
    return [(lab, st, 0.5 if sc is None else sc, su) for lab, st, sc, su in cases]


def out_by(d):
    return plug(f=-d)


CASES = _fill([
    ("start: hose end lying on the table", {}, 0.0, False),
    ("connector lifted, 40 mm in front of the mouth", plug(f=-0.040), 0.0, False),
    ("nose in the chamfer and against the pins (6 mm in)", plug(f=-NOSE_L + 0.006), 0.0, False),
    (f"nose {PAST_PINS * 1000 - 0.1:.1f} mm in (just short of past the pins)", plug(f=-NOSE_L + PAST_PINS - 0.0001),
     0.0, False),
    ("nose past the pins but 6.5 degrees askew", plug(f=-NOSE_L + PAST_PINS + 0.001, tilt=6.5), 0.0, False),
    (f"nose {PAST_PINS * 1000:.1f} mm in, on the axis: past the pins", plug(f=-NOSE_L + PAST_PINS), 0.25, False),
    ("then pulled back out and dropped on the table", {"hose_end": PLUG.state(pos=(0.25, 0.10, TOP + 0.0125))},
     0.25, False),
    (f"flange {HOME * 1000 + 0.1:.1f} mm short of the mouth (not home)", plug(f=-HOME - 0.0001), 0.25, False),
    ("flange 1.7 mm off the axis (not home)", plug(f=0.0, side=0.0017), 0.25, False),
    ("home but 5.5 degrees askew (not home)", plug(f=0.0, tilt=5.5), 0.25, False),
    (f"flange {HOME * 1000:.1f} mm short of the mouth: home", plug(f=-HOME), 0.5, False),
    ("home, still held: no pull test, no full credit", HOME_S, 0.5, False),
    *tested("[home] arms not yet measured (fail closed)", grip(None), 0.5, success=False),
    *tested("[home] a gripper within 5 mm (still holding)", grip(False), 0.5, success=False),
    *tested("[home] a flat arm face resting 0.5 mm over the collar, no corner near (face-sampled check)",
            plate_state(0.0005), 0.5, success=False),
    *tested("[home] the same face 6 mm over the collar: clear, locked", plate_state(0.006), 1.0),
    *tested("[home] locked: comes out 0.4 mm under the 10 N pull", HOME_S, 1.0),
    *tested("[home] holds the pull but the collar is stuck 6 mm back on a pin (no test)",
            {**HOME_S, "coupler": COUPLER.state(joints={"collar": 0.006})}, 0.5, success=False),
    *tested("[home] collar 1.0 mm off its stop (limit): locked",
            {**HOME_S, "coupler": COUPLER.state(joints={"collar": 0.001})}, 1.0),
    *tested("[home] collar 1.1 mm off its stop (no test)",
            {**HOME_S, "coupler": COUPLER.state(joints={"collar": 0.0011})}, 0.5, success=False),
    ("then the hose end moved: the test starts again", plug(vel=(0.0, 0.02, 0.0)), 0.5, False),
    *tested("[home] locked, pull never applied (force not set)", HOME_S, 0.5,
            pulled=no_force(plug(f=-GIVE)), success=False),
    *tested(f"[home] pull applied but it moved under {GIVE_MIN * 1000:.2f} mm", HOME_S, 0.5,
            pulled=plug(f=-GIVE_MIN + 0.00001), success=False),
    *tested(f"[home] comes out {GIVE_MAX * 1000:.1f} mm under the pull (limit)", HOME_S, 1.0,
            pulled=out_by(GIVE_MAX - 1e-7)),
    *tested(f"[home] comes out {GIVE_MAX * 1000 + 0.1:.1f} mm, then back home", HOME_S, 0.5,
            pulled=out_by(GIVE_MAX + 0.0001), success=False),
    # an unlocked connector (collar let go too early, stuck back on the pins) keeps coming out: the pull stops at 3 mm
    ("[home] unlocked: still", HOME_S, 0.5, False),
    *[(f"unlocked, held still (step {k + 1})", HOME_S, 0.5, False) for k in range(STILL_STEPS - 1)],
    ("unlocked: the pull starts", HOME_S, 0.5, False),
    ("unlocked: 1 mm out", out_by(0.001), 0.5, False),
    ("unlocked: 3.5 mm out, the test fails and stops", out_by(ABORT + 0.0005), 0.5, False),
    ("unlocked: slid 40 mm out and lying in front of the mouth", plug(f=-0.040), 0.5, False),
    ("unlocked: pushed back home by hand, held", HOME_S, 0.5, False),
    ("dropped off the table", {"hose_end": PLUG.state(pos=(0.3, 0.3, TOP - 0.03))}, 0.0, False),
    ("scene reset (hose end back at its start in one step)", RESET, 0.0, False),
    ("after the reset: start, no record", {}, 0.0, False),
    *tested("after the reset, home and locked", HOME_S, 1.0),
])

# the face check matters: by its corners alone the 0.5 mm plate reads clear
_boxes0 = task._boxes({"coupler": COUPLER.state(), **plug()})
assert task._arms_clear([plate(0.0005)[1]], _boxes0) and not task._arms_clear(plate(0.0005)[0], _boxes0)
assert task._arms_clear(plate(CLEAR + 0.0005)[0], _boxes0) and not task._arms_clear(plate(CLEAR - 0.0005)[0], _boxes0)
assert abs(float(np.linalg.norm(_AXIS_OUT)) - 1.0) < 1e-9 and YAW_C < 0 and ABORT > GIVE_MAX
