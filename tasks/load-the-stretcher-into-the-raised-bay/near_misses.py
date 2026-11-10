"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A (sill
340 mm in front of the arm bases). Each case: (label, {object: state}, expected score, expected success). States come
from the parts' own geometry. Cases run in order and share the whole-trial record, so a fault stays until a scene
reset (an empty case after the stretcher was moved).
"""

import math

from task import BAY, BAY_H, D, FOLD_UP, L, SILL as SILLS, SKID_H, START, STRETCHER, TOP

SILL = SILLS["A"]
FOLD = {"front_legs": FOLD_UP, "rear_legs": FOLD_UP}
FRONT = {"front_legs": FOLD_UP}
Y = START[1]
Z_IN = TOP + BAY_H - (D - SKID_H)  # origin height with both ends' skids on the bay floor


def st(x, z=TOP + 0.0005, legs=None, quat=None, vel=None):
    return STRETCHER.state(pos=(x, Y, z), joints=legs or {}, quat_xyzw=quat, lin_vel=vel)


def pitch(deg):  # positive: head end down
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def rear_past(p):  # deck centre x with the foot end p past the sill
    return SILL + p + L / 2


def front_past(p):
    return SILL + p - L / 2


def head_in(x):
    """Head end on the bay floor (front legs folded), foot end on its rear legs on the table: pitched slightly."""
    dz = (TOP + 0.0005 + D) - (TOP + BAY_H + SKID_H)  # rear deck underside above front deck underside
    ang = math.degrees(math.atan2(dz, L - 0.015))
    return st(x, z=TOP + 0.0005 + (BAY_H + SKID_H - D) / 2 + 0.0, legs=FRONT, quat=pitch(ang))


IN = {"stretcher": st(rear_past(0.030), z=Z_IN, legs=FOLD)}
RESET = {}

CASES = [
    ("start: standing on its legs, short of the sill", {}, 0.0, False),
    ("loaded: both pairs folded, foot end 30 mm past the sill", IN, 1.0, True),
    ("loaded, foot end exactly 20 mm past", {"stretcher": st(rear_past(0.020), z=Z_IN, legs=FOLD)}, 1.0, True),
    ("foot end 19 mm past", {"stretcher": st(rear_past(0.019), z=Z_IN, legs=FOLD)}, 0.5, False),
    ("loaded but rear pair at 74 degrees", {"stretcher": st(rear_past(0.03), z=Z_IN,
                                                            legs={"front_legs": FOLD_UP,
                                                                  "rear_legs": math.radians(74)})}, 0.5, False),
    ("loaded, rear pair at exactly 75 degrees", {"stretcher": st(rear_past(0.03), z=Z_IN,
                                                                 legs={"front_legs": FOLD_UP,
                                                                       "rear_legs": math.radians(75)})}, 1.0, True),
    ("loaded but still sliding", {"stretcher": st(rear_past(0.03), z=Z_IN, legs=FOLD, vel=(0.02, 0, 0))}, 0.0, False),
    ("both folded, pitched 6 degrees (an end off the floor)", {"stretcher": st(rear_past(0.03), z=Z_IN, legs=FOLD, quat=pitch(6))},
     0.0, False),
    ("both folded, held 4 mm above the floor", {"stretcher": st(rear_past(0.03), z=Z_IN + 0.004, legs=FOLD)}, 0.0,
     False),
    ("both folded, sitting on the floor 3 mm down (exact)", {"stretcher": st(rear_past(0.03), z=Z_IN + 0.003,
                                                                             legs=FOLD)}, 1.0, True),
    ("pushed in with legs standing (front legs on the bay floor)",
     {"stretcher": st(rear_past(0.03), z=TOP + BAY_H + 0.0005)}, 0.0, False),
    ("head end in, front pair folded, rear on its legs", {"stretcher": head_in(front_past(0.045))}, 0.5, False),
    ("head end only 17 mm past the sill", {"stretcher": head_in(front_past(0.017))}, 0.0, False),
    ("head end in but its pair still standing (on the floor)",
     {"stretcher": st(front_past(0.045), z=TOP + 0.0005)}, 0.0, False),
    ("both pairs folded on the table, short of the sill (released early)",
     {"stretcher": st(START[0], z=TOP + SKID_H - D, legs=FOLD)}, 0.0, False),
    ("head end in, foot end dropped to the table (rear released unsupported)",
     {"stretcher": st(front_past(0.06), z=TOP + 0.04, legs=FOLD, quat=pitch(-19))}, 0.0, False),
    ("then lifted back and loaded: still 0 (the drop is judged over the whole trial)", IN, 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("head end dropped to the table early (front pair folded, pitched 15.5 degrees)",
     {"stretcher": st(START[0], z=TOP + 0.03, legs=FRONT, quat=pitch(15.5))}, 0.0, False),
    ("then the head end lifted back onto the bay and loaded: still 0", IN, 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("pitched 11.9 degrees on the way (no fault)", {"stretcher": st(START[0], z=TOP + 0.02, quat=pitch(11.9))}, 0.0,
     False),
    ("then loaded: 1.0", IN, 1.0, True),
    ("scene reset", RESET, 0.0, False),
    ("pitched 12.1 degrees on the way (fault)", {"stretcher": st(START[0], z=TOP + 0.02, quat=pitch(12.1))}, 0.0,
     False),
    ("then loaded: still 0", IN, 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("pitched exactly 12 degrees (no fault)", {"stretcher": st(START[0], z=TOP + 0.02, quat=pitch(12.0))}, 0.0,
     False),
    ("then loaded: 1.0", IN, 1.0, True),
    ("scene reset", RESET, 0.0, False),
    ("bay shoved 11 mm", {"bay": BAY.state(pos=(SILL + 0.011, Y, TOP)), **IN}, 0.0, False),
    ("then loaded: still 0", IN, 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("bay shoved 9 mm (allowed)", {"bay": BAY.state(pos=(SILL + 0.009, Y, TOP)), **IN}, 1.0, True),
    ("scene reset", RESET, 0.0, False),
    ("stretcher tipped 31 degrees", {"stretcher": st(START[0], z=TOP + 0.03, quat=pitch(31))}, 0.0, False),
    ("then loaded: still 0", IN, 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("loaded after a clean trial", IN, 1.0, True),
]
