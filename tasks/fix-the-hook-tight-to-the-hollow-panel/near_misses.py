"""Near-successes and unintended strategies, scored offline by the factory checker. Each case: (label, {object: spec},
expected score, expected success); a spec is a ready state from the part's own geometry (HOOK.state(...)). Cases run
in order and share the whole-trial record (the toggle running off the bolt), cleared only by a scene reset.

Offline there is no robot, so the arms' clearance is set here as measured clear; in the simulator it is measured
every step and is None (no credit) until then.
"""

import math

import parts
from task import DRIVER, FACE, GAP0, HOLE, HOOK, LEAD, RUN_OFF, STILL_STEPS, TOP

parts.__dict__.setdefault("_hook_task_grip", {})["clear"] = True

X0, Y0, Z0 = HOOK.pos


def quat_z(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def quat_x(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def hook(thread=GAP0, dz=0.0, yaw=0.0, quat=None, vel=None, pos=None):
    q = quat if quat is not None else quat_z(yaw)
    p = pos if pos is not None else (X0, Y0, Z0 + dz)
    return {"hook": HOOK.state(pos=p, quat_xyzw=q, lin_vel=vel,
                               joints={"thread": thread, "nut": thread / LEAD * 2 * math.pi})}


TIGHT = hook()
RESET = {}


def held(label, state, score, success=None):
    """A state reached and then held for 1 s: score() counts control steps with the hook's pose unchanged and at rest,
    and no credit comes before 20 of them (1 s at 20 Hz) after the step it arrived on. success() sees the count left
    by the step before, so it holds one step later. Each hold starts from the hook lifted 2 mm (a different pose)."""
    out = [(f"{label}: hook still settling, 2 mm up", {**state, **hook(dz=0.002)}, 0.0, False)]
    out += [(f"{label} (step {k + 1} of the 1 s hold)", state, 0.0, False) for k in range(STILL_STEPS)]
    out.append((f"{label} (held 1 s)", state, score, False))
    out.append((f"{label} (held 1 s + 1 step)", state, score, score == 1.0 if success is None else success))
    return out


CASES = [
    ("start: hook loose, wings 3 mm behind the back face", {}, 0.0, False),
    *held("tight and square", TIGHT, 1.0),
    ("then the hook moving: the hold starts again", hook(vel=(0.0, 0.0, 0.02)), 0.0, False),
    ("tight and at rest again, first step", TIGHT, 0.0, False),
    *held("tight, 2.9 degrees off the line", hook(yaw=2.9), 1.0),
    *held("tight, 3.1 degrees off the line", hook(yaw=-3.1), 0.5),
    *held("tight, 10 degrees off the line", hook(yaw=10.0), 0.5),
    *held("tight, 10.5 degrees off the line", hook(yaw=10.5), 0.0),
    *held("tight but turned round (plate on the hole's other side)", hook(yaw=180.0), 0.0),
    *held("wings 0.5 mm off the back face (limit)", hook(thread=GAP0 - 0.0005 + 1e-7), 1.0),
    *held("wings 0.6 mm off the back face", hook(thread=GAP0 - 0.0006), 0.0),
    *held("half the thread taken up", hook(thread=GAP0 / 2), 0.0),
    *held("turned without the pull: toggle spun, no travel", hook(thread=0.0), 0.0),
    *held("plate held 0.4 mm off the face, wings on the back", hook(thread=GAP0 - 0.0004, dz=0.0004), 1.0),
    *held("plate held 0.6 mm off the face, wings on the back", hook(thread=GAP0 - 0.0006, dz=0.0006), 0.0),
    *held("plate tilted 1 degree (one end lifted)", hook(quat=quat_x(1.0)), 0.0),
    *held("tight, screwdriver left standing in the head",
          {**TIGHT, "screwdriver": DRIVER.state(pos=(HOLE[0], HOLE[1], TOP + FACE + 0.0046))}, 0.0),
    *held("tight, screwdriver standing on the panel 4 mm from the plate",
          {**TIGHT, "screwdriver": DRIVER.state(pos=(HOLE[0] + 0.02 + 0.004 + 0.0175, HOLE[1] + 0.05, TOP + FACE))},
          0.0),
    *held("tight, screwdriver back in its holder", TIGHT, 1.0),
    # whole trial: the toggle run off the bolt's end (unscrewed) is lost in the cavity
    ("toggle unscrewed to the bolt's end", hook(thread=RUN_OFF), 0.0, False),
    *held("then tight: still 0", TIGHT, 0.0),
    ("hook lifted 30 mm", hook(thread=0.0, dz=0.030), 0.0, False),
    ("scene reset (hook back at its start in one step)", RESET, 0.0, False),
    *held("after the reset, tight and square", TIGHT, 1.0),
]
