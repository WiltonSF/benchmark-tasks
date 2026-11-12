"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A.
Each case: (label, {object: state}, expected score, expected success). Cases share the whole-trial record, so once
the setup is disturbed every later case scores 0 until a scene reset (offline: the exact start state, an empty case).
Offline there is no robot, so the grippers count as clear and the bezel as still; the simulator measures both.
"""

import math

from task import BLOCK_PART, BLOCK_POS, DIV, INDICATOR, LEAD, POINTER_START, Q_CONTACT

ZERO = -POINTER_START  # the bezel joint that puts the zero line under the pointer


def bezel(deg_off=0.0, plunger=Q_CONTACT, pointer=POINTER_START):
    return {"dial_indicator": INDICATOR.state(joints={"bezel": ZERO + math.radians(deg_off), "plunger": plunger,
                                                      "pointer": pointer})}


def block(dx=0.0, dy=0.0, yaw_deg=0.0):
    a = math.radians(yaw_deg) / 2
    return {"steel_block": BLOCK_PART.state(pos=(BLOCK_POS[0] + dx, BLOCK_POS[1] + dy, BLOCK_POS[2]),
                                            quat_xyzw=(0.0, 0.0, math.sin(a), math.cos(a)))}


D = math.degrees(DIV)  # 3.6 degrees
RESET = {}

CASES = [
    ("zero exactly under the pointer", bezel(), 1.0, True),
    ("zero 2 divisions past, clockwise", bezel(-2 * D), 1.0, True),
    ("zero 2 divisions short, counter-clockwise", bezel(2 * D), 1.0, True),
    ("zero 2.1 divisions off: loosely zeroed", bezel(2.1 * D), 0.5, False),
    ("zero 5 divisions off", bezel(-5 * D), 0.5, False),
    ("zero 5.1 divisions off", bezel(5.1 * D), 0.0, False),
    ("zero turned a full turn plus on target", bezel(360.0), 1.0, True),
    ("zero 180 degrees off", bezel(180.0), 0.0, False),
    ("zero set to 12 o'clock (opposite the stem), habit", {"dial_indicator": INDICATOR.state(
        joints={"bezel": math.pi})}, 0.0, False),
    ("bezel left as it started", RESET, 0.0, False),
    ("plunger held 1 division in, zero on the new reading", bezel(-1 * D, plunger=Q_CONTACT - LEAD / 100,
                                                               pointer=POINTER_START + DIV), 1.0, True),
    ("block nudged 0.4 mm sideways, zeroed", {**bezel(), **block(dy=0.0004)}, 1.0, True),
    ("scene reset", RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("plunger pushed 0.03 mm in (pointer reads 3 divisions more)", bezel(plunger=Q_CONTACT - 0.00003,
                                                                       pointer=POINTER_START + 3 * DIV), 0.0, False),
    ("then let back and zeroed: still 0", bezel(), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("naive: plunger pushed until the pointer reads zero",
     bezel(plunger=Q_CONTACT - 0.00063, pointer=POINTER_START + 63 * DIV), 0.0, False),
    ("then let back and zeroed: still 0", bezel(), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("block pulled 2 mm off the plunger, then zero set on the new reading",
     {**bezel(-2 * 100 * D, plunger=Q_CONTACT + 0.002, pointer=POINTER_START - 2 * 2 * math.pi), **block(dx=0.002)},
     0.0, False),
    ("then put back and zeroed: still 0", bezel(), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("block slid 0.6 mm sideways", {**bezel(), **block(dy=0.0006)}, 0.0, False),
    ("then slid back and zeroed: still 0", bezel(), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("block turned 1.5 degrees", {**bezel(), **block(yaw_deg=1.5)}, 0.0, False),
    ("then turned back: still 0", bezel(), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("after the reset, zeroed: 1.0", bezel(), 1.0, True),
]
