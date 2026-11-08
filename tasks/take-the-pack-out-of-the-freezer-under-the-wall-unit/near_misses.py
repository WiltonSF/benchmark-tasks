"""Near-successes and unintended strategies, scored offline by the factory checker (lot A: blue pack at the back
left, white pack at the back right).

Each case: (label, {object: (pos, size or None, lin_vel or None[, quat_xyzw])}, expected score, expected success).
The release check (no gripper within 5 mm of a pack) runs only in the simulator; offline it reads as released.
"""

import math

from task import FLOOR, FX0, FY1, IX0, IX1, IY1, MAT_C, MAT_S, MAT_T, PACK, PACK_X, RIM, SLOT_L, SLOT_R, TOP

ON_MAT = TOP + MAT_T + PACK[2] / 2
ON_TABLE = TOP + PACK[2] / 2
MX, MY = MAT_C
MOVING = (0.0, 0.0, -0.05)


def yq(deg):
    return (0.0, 0.0, math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2))


def xq(deg):
    return (math.sin(math.radians(deg) / 2), 0.0, 0.0, math.cos(math.radians(deg) / 2))


LYING = (MX, MY, TOP + MAT_T + PACK[1] / 2)  # on its long side: 45 mm tall

CASES = [
    # full score
    ("blue pack standing on the mat, white pack in the freezer", {"blue_pack": ((MX, MY, ON_MAT), None, None)}, 1.0,
     True),
    ("blue pack lying on its side on the mat", {"blue_pack": (LYING, None, None, xq(90))}, 1.0, True),
    ("blue pack on the mat, turned 30 degrees", {"blue_pack": ((MX, MY, ON_MAT), None, None, yq(30))}, 1.0, True),
    ("blue pack centre exactly on the mat's edge", {"blue_pack": ((MX + MAT_S / 2, MY, ON_MAT), None, None)}, 1.0,
     True),
    ("blue pack exactly 1 mm above the mat", {"blue_pack": ((MX, MY, ON_MAT + 0.001), None, None)}, 1.0, True),
    ("white pack nudged forward inside the freezer", {"blue_pack": ((MX, MY, ON_MAT), None, None),
                                                      "white_pack": ((PACK_X - 0.08, SLOT_R, FLOOR + PACK[2] / 2),
                                                                     None, None)}, 1.0, True),
    ("white pack knocked over inside the freezer", {"blue_pack": ((MX, MY, ON_MAT), None, None),
                                                    "white_pack": ((PACK_X, SLOT_R, FLOOR + PACK[1] / 2), None, None,
                                                                   xq(90))}, 1.0, True),
    # partial: out of the freezer and released on the table, not on the mat
    ("blue pack on the table in front of the freezer", {"blue_pack": ((FX0 - 0.05, 0.10, ON_TABLE), None, None)}, 0.5,
     False),
    ("blue pack centre just past the mat's edge (on the table)",
     {"blue_pack": ((MX + MAT_S / 2 + 0.001, MY, ON_TABLE), None, None)}, 0.5, False),
    ("blue pack 1.5 mm above the mat (held, not set down)", {"blue_pack": ((MX, MY, ON_MAT + 0.0015), None, None)}, 0.0,
     False),
    ("blue pack on the mat but still moving", {"blue_pack": ((MX, MY, ON_MAT), None, MOVING)}, 0.0, False),
    ("blue pack on the table, still moving", {"blue_pack": ((FX0 - 0.05, 0.10, ON_TABLE), None, MOVING)}, 0.0, False),
    # zero
    ("nothing done", {}, 0.0, False),
    ("blue pack dragged to the front of the freezer, still inside",
     {"blue_pack": ((IX0 + 0.04, SLOT_L, FLOOR + PACK[2] / 2), None, None)}, 0.0, False),
    ("blue pack left on the freezer's rim (front wall)", {"blue_pack": ((FX0 + 0.0075, SLOT_L, RIM + PACK[2] / 2),
                                                                       None, None)}, 0.0, False),
    ("blue pack on the table against the freezer's side, overlapping its footprint",
     {"blue_pack": ((0.30, FY1 + PACK[1] / 2 - 0.002, ON_TABLE), None, None)}, 0.0, False),
    ("blue pack fallen off the table", {"blue_pack": ((0.30, 0.45, TOP - 0.30), None, None)}, 0.0, False),
    ("white pack taken to the mat instead (habit: the right-hand pack)",
     {"white_pack": ((MX, MY, ON_MAT), None, None)}, 0.0, False),
    ("both packs taken out, blue on the mat", {"blue_pack": ((MX, MY, ON_MAT), None, None),
                                               "white_pack": ((MX - 0.08, MY + 0.08, ON_TABLE), None, None)}, 0.0,
     False),
    ("both packs on the mat, stacked", {"blue_pack": ((MX, MY, ON_MAT), None, None),
                                        "white_pack": ((MX, MY, ON_MAT + PACK[2]), None, None)}, 0.0, False),
    ("white pack moved out onto the table, blue pack left in the freezer",
     {"white_pack": ((FX0 - 0.05, 0.0, ON_TABLE), None, None)}, 0.0, False),
    ("blue pack set on top of the white pack in the freezer",
     {"blue_pack": ((PACK_X, SLOT_R, FLOOR + 1.5 * PACK[2]), None, None)}, 0.0, False),
    ("white pack lifted onto the freezer's rim (centre above the rim)",
     {"blue_pack": ((MX, MY, ON_MAT), None, None),
      "white_pack": ((IX1 - 0.02, IY1 - 0.03, RIM + PACK[2] / 2 + 0.001), None, None)}, 0.0, False),
]
