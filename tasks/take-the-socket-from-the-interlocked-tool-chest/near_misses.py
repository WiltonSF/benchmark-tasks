"""Near-successes and unintended strategies, scored offline by the factory checker (lot A: the middle drawer propped).

Each case: (label, {object: ready state or (centre, size or None, velocity or None[, quat_xyzw])}, expected score,
expected success).
"""

import math

from task import (CHEST, FLOOR_T, HANDLE_R, HEIGHT, MAT, MAT_SIZE, OPEN, SLOT_BOTTOM, SOCKET_SIZE, SOCKET_XY, TOOL,
                  TOP, X0, tool_start)

S = 0.5 ** 0.5
LYING = (0.0, S, 0.0, S)  # turned 90 degrees about y: the screwdriver lying along x
START = {"middle": OPEN}


def chest(**q):
    return CHEST.state(joints={**START, **q})


def tool_lying(x=0.22, y=0.24, z=TOP + HANDLE_R):
    return TOOL.state(pos=(x, y, z), quat_xyzw=LYING)


def socket_in_bottom(q, dz=0.0):
    return ((SOCKET_XY[0] - q, SOCKET_XY[1], TOP + SLOT_BOTTOM["bottom"] + FLOOR_T + SOCKET_SIZE / 2 + dz), None, None)


ON_MAT = ((MAT.pos[0], MAT.pos[1], TOP + MAT_SIZE[2] + SOCKET_SIZE / 2), None, None)
ON_TABLE = ((0.20, -0.08, TOP + SOCKET_SIZE / 2), None, None)
ON_FLOOR = TOOL.state(pos=(0.22, 0.24, 0.0 + HANDLE_R), quat_xyzw=LYING)
TILT20 = (0.0, math.sin(math.radians(10)), 0.0, math.cos(math.radians(10)))
SHUT_DONE = {"tool_chest": chest(middle=0.0), "screwdriver": tool_lying()}
OPEN_DONE = {"tool_chest": chest(middle=0.0, bottom=0.130), "screwdriver": tool_lying()}

CASES = [
    ("start: middle drawer propped, tool in its holder", {"tool_chest": chest()}, 0.0, False),
    ("bottom drawer pulled 3 mm against its bolt", {"tool_chest": chest(bottom=0.003)}, 0.0, False),
    ("middle drawer pushed in 30 mm until the tool jams", {"tool_chest": chest(middle=OPEN - 0.030)}, 0.0, False),
    ("top drawer pulled 3 mm against its bolt", {"tool_chest": chest(top=0.003)}, 0.0, False),
    ("tool lifted out and lying on the table, middle drawer still open (no credit)", {"tool_chest": chest(), "screwdriver": tool_lying()}, 0.0, False),
    ("tool on the chest's top panel, middle drawer open",
     {"tool_chest": chest(), "screwdriver": tool_lying(X0, 0.05, TOP + HEIGHT + HANDLE_R)}, 0.0, False),
    ("tool knocked off the table onto the floor", {"tool_chest": chest(), "screwdriver": ON_FLOOR}, 0.0, False),
    ("tool out, middle drawer shut", dict(SHUT_DONE), 0.5, False),
    ("tool out, middle drawer 5 mm open (limit)", {**SHUT_DONE, "tool_chest": chest(middle=0.005)}, 0.5, False),
    ("tool out, middle drawer 6 mm open", {**SHUT_DONE, "tool_chest": chest(middle=0.006)}, 0.0, False),
    ("tool laid inside the middle drawer and the drawer shut",
     {"tool_chest": chest(middle=0.0), "screwdriver": tool_lying(X0 - 0.03, 0.0, TOP + SLOT_BOTTOM["middle"] + FLOOR_T
                                                                   + HANDLE_R)}, 0.5, False),
    ("middle drawer shut with the tool still where it stood (cannot happen)",
     {"tool_chest": chest(middle=0.0), "screwdriver": TOOL.state(pos=tool_start("middle"))}, 0.0, False),
    ("middle shut, tool on the floor", {**SHUT_DONE, "screwdriver": ON_FLOOR}, 0.0, False),
    ("bottom drawer open 130 mm, socket in it", {**OPEN_DONE, "red_socket": socket_in_bottom(0.130)}, 0.75, False),
    ("bottom drawer open 100 mm (limit), socket in it",
     {**OPEN_DONE, "tool_chest": chest(middle=0.0, bottom=0.100), "red_socket": socket_in_bottom(0.100)}, 0.75, False),
    ("bottom drawer open 99 mm, socket in it",
     {**OPEN_DONE, "tool_chest": chest(middle=0.0, bottom=0.099), "red_socket": socket_in_bottom(0.099)}, 0.5, False),
    ("socket dropped on the table off the mat", {**OPEN_DONE, "red_socket": ON_TABLE}, 0.75, False),
    ("socket on the mat", {**OPEN_DONE, "red_socket": ON_MAT}, 1.0, True),
    ("socket on the mat, bottom drawer pushed shut again",
     {**OPEN_DONE, "tool_chest": chest(middle=0.0), "red_socket": ON_MAT}, 1.0, True),
    ("socket on the mat at its edge (inside the outline)",
     {**OPEN_DONE, "red_socket": ((MAT.pos[0] + MAT_SIZE[0] / 2 - SOCKET_SIZE / 2, MAT.pos[1],
                                   TOP + MAT_SIZE[2] + SOCKET_SIZE / 2), None, None)}, 1.0, True),
    ("socket over the mat's edge (1 mm outside the outline)",
     {**OPEN_DONE, "red_socket": ((MAT.pos[0] + MAT_SIZE[0] / 2 - SOCKET_SIZE / 2 + 0.001, MAT.pos[1],
                                   TOP + MAT_SIZE[2] + SOCKET_SIZE / 2), None, None)}, 0.75, False),
    ("socket on the chest's top panel", {**OPEN_DONE, "red_socket": ((X0, 0.0, TOP + HEIGHT + SOCKET_SIZE / 2), None, None)},
     0.75, False),
    ("socket fallen off the table", {**OPEN_DONE, "red_socket": ((0.20, -0.08, SOCKET_SIZE / 2), None, None)}, 0.0, False),
    ("socket held 5 mm above the mat", {**OPEN_DONE, "red_socket": ((MAT.pos[0], MAT.pos[1],
                                                                       TOP + MAT_SIZE[2] + SOCKET_SIZE / 2 + 0.005),
                                                                      None, None)}, 0.5, False),
    ("socket on the mat, still moving", {**OPEN_DONE, "red_socket": (ON_MAT[0], None, (0.0, 0.05, 0.0))}, 0.0, False),
    ("socket on the mat, tool on the floor", {**OPEN_DONE, "red_socket": ON_MAT, "screwdriver": ON_FLOOR}, 0.0, False),
    ("socket on the mat tilted 20 degrees (resting on the mat's edge)",
     {**OPEN_DONE, "red_socket": (ON_MAT[0], None, None, TILT20)}, 0.5, False),
    ("socket lifted 10 mm off the bottom drawer floor (held)",
     {**OPEN_DONE, "red_socket": socket_in_bottom(0.130, 0.010)}, 0.5, False),
]
