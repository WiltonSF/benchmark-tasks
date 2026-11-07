"""Near-successes, unintended strategies and fixed habits, scored offline by factory/check.py. Cases run in order
against one copy of the task module and each is one step of a trial, so the whole-trial record (the most the cone
has been wound, and any run-back of more than 30 degrees) carries from case to case. A case with the cone at 0 and
the pin up is the start of a new trial. Offline the gripper-clearance check is skipped (counted as clear) and the
cone counts as still."""

import math

from task import PIN_LATCH, PIN_TRAVEL, WINDER

DOWN = PIN_TRAVEL  # pin pressed and latched


def w(deg, pin=0.0):
    return {"winder": WINDER.state(joints={"cone": math.radians(deg), "pin": pin})}


START = w(0.0)
CASES = [
    ("start", START, 0.0, False),
    ("wound 268 degrees and resting on the pressed pin (three quarter turns)", w(268.0, DOWN), 1.0, True),
    ("new trial", START, 0.0, False),
    ("wound 178, resting on the pin (two quarter turns)", w(178.0, DOWN), 0.5, False),
    ("new trial", START, 0.0, False),
    ("habit: one quarter turn and pin it (88 degrees)", w(88.0, DOWN), 0.25, False),
    ("new trial", START, 0.0, False),
    ("habit: pin pressed first, before any winding (locked at 0)", w(0.0, DOWN), 0.0, False),
    ("new trial", START, 0.0, False),
    ("wound 268 but the pin left up (held there)", w(268.0), 0.0, False),
    ("pin pushed only part way (not latched)", w(268.0, PIN_LATCH - 0.001), 0.0, False),
    ("pin at the latch point", w(268.0, PIN_LATCH), 1.0, True),
    ("new trial", START, 0.0, False),
    ("habit: one arm winds as far as it can follow (150 degrees)", w(150.0), 0.0, False),
    ("... lets go to take a new grip: the cone runs back almost to its stop", w(0.5), 0.0, False),
    ("... then winds three quarter turns and pins it: still 0 (it ran back)", w(268.0, DOWN), 0.0, False),
    ("new trial", START, 0.0, False),
    ("habit: one arm winds 150, lets go, and the cone catches on the pin at 88", w(150.0), 0.0, False),
    ("... resting on the pin at 88 (ran back 62 degrees)", w(88.0, DOWN), 0.0, False),
    ("new trial", START, 0.0, False),
    ("wound to 298, eased back exactly 30 to rest on the pin at 268", w(298.0), 0.0, False),
    ("... resting on the pin at 268", w(268.0, DOWN), 1.0, True),
    ("new trial", START, 0.0, False),
    ("wound to 298.1, back 30.1 degrees: a run-back", w(298.1), 0.0, False),
    ("... resting on the pin at 268: 0", w(268.0, DOWN), 0.0, False),
    ("new trial", START, 0.0, False),
    ("a handover gives back 29 degrees", w(200.0), 0.0, False),
    ("... (171 degrees)", w(171.0), 0.0, False),
    ("... then wound on and pinned at 268", w(282.0), 0.0, False),
    ("... resting on the pin", w(268.0, DOWN), 1.0, True),
    ("new trial", START, 0.0, False),
    ("wound 265 exactly, pinned (tolerance edge)", w(265.0, DOWN), 1.0, True),
    ("new trial", START, 0.0, False),
    ("wound 264.9, pinned", w(264.9, DOWN), 0.0, False),
    ("new trial", START, 0.0, False),
    ("wound 271 exactly, pin down", w(271.0, DOWN), 1.0, True),
    ("new trial", START, 0.0, False),
    ("wound 271.1, pin down", w(271.1, DOWN), 0.0, False),
    ("new trial", START, 0.0, False),
    ("between tiers: 135 degrees, pin down", w(135.0, DOWN), 0.0, False),
    ("new trial", START, 0.0, False),
    ("two quarter turns but 4 degrees off the lock (182), pin down", w(182.0, DOWN), 0.0, False),
    ("new trial", START, 0.0, False),
    ("habit: dragged round to 90 by the disc, pin down but not resting on it (91.1)", w(91.1, DOWN), 0.0, False),
    ("new trial", START, 0.0, False),
    ("wound 91 exactly, pin down (one quarter, tolerance edge)", w(91.0, DOWN), 0.25, False),
    ("new trial", START, 0.0, False),
    ("over-wound to the 300 degree limit, no pin", w(300.0), 0.0, False),
]


def _run() -> int:
    import task

    bad = 0
    for label, st, want, want_success in CASES:
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
