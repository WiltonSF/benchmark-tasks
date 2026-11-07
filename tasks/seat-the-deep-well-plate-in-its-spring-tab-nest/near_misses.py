"""Near-successes, wrong outcomes and fixed habits, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw]) or a Part state}, expected score,
expected success). The nest's joint "tab" is how far the tab is held back from its stop (m).
"""

import math

from task import DATUM_X, FLOOR_Z, NEST, NEST_Y, PLATE, SHORT, TAB, TOP, WALL_H

SEATED = (DATUM_X + PLATE[0] / 2, NEST_Y, FLOOR_Z + PLATE[2] / 2)
HELD = NEST.state(joints={"tab": SHORT})  # plate against the datum: tab held back 4 mm
FREE = NEST.state(joints={"tab": 0.0})


def tilted_about_y(deg):
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def pitched(rise):
    """Near end's underside on the floor at the datum, far end's underside ``rise`` m higher (a proud plate)."""
    L, H = PLATE[0], PLATE[2]
    phi = -math.asin(rise / L)
    c = (DATUM_X + math.cos(phi) * L / 2, NEST_Y, FLOOR_Z - (L / 2 * math.sin(phi) - H / 2 * math.cos(phi)))
    return (c, None, None, (0.0, math.sin(phi / 2), 0.0, math.cos(phi / 2)))


def at(dx=0.0, dz=0.0, dy=0.0, vel=None, quat=None):
    c = (SEATED[0] + dx, SEATED[1] + dy, SEATED[2] + dz)
    return (c, None, vel) if quat is None else (c, None, vel, quat)


# Dropped straight down centred: the far end lands on the tab's top (35 mm above the floor), the near end on the floor.
_lean = math.degrees(math.atan2(TAB[2] - 0.0, PLATE[0]))

CASES = [
    ("seated: on the floor, at the datum, tab held back 4 mm", {"deep_well_plate": at(), "nest": HELD}, 1.0, True),
    ("0.8 mm from the datum (tab held back 3.2 mm)", {"deep_well_plate": at(dx=0.0008),
                                                       "nest": NEST.state(joints={"tab": SHORT - 0.0008})}, 1.0, True),
    ("2 mm short of the datum (friction stopped it)", {"deep_well_plate": at(dx=0.002),
                                                       "nest": NEST.state(joints={"tab": SHORT - 0.002})}, 0.0, False),
    ("at the datum but the tab not touching (cannot happen; scorer still says 0)",
     {"deep_well_plate": at(), "nest": FREE}, 0.0, False),
    ("held 3 mm above the floor", {"deep_well_plate": at(dz=0.003), "nest": HELD}, 0.0, False),
    ("still moving", {"deep_well_plate": at(vel=(-0.05, 0.0, 0.0)), "nest": HELD}, 0.0, False),
    ("tilted 3 degrees", {"deep_well_plate": at(dz=0.003, quat=tilted_about_y(3.0)), "nest": HELD}, 0.0, False),
    ("proud: near end on the floor, far end's underside 4.4 mm up (1.97 degrees, the reviewer's case)",
     {"deep_well_plate": pitched(0.0044), "nest": HELD}, 0.0, False),
    ("far end's underside 0.9 mm up (0.4 degrees)", {"deep_well_plate": pitched(0.0009), "nest": HELD}, 1.0, True),
    ("far end's underside exactly 1.0 mm up", {"deep_well_plate": pitched(0.0010), "nest": HELD}, 1.0, True),
    ("far end's underside 1.1 mm up", {"deep_well_plate": pitched(0.0011), "nest": HELD}, 0.0, False),
    ("far end 1.3 mm up after a release 2 mm above the floor (0.6 degrees, traced before the fix)",
     {"deep_well_plate": pitched(0.0013), "nest": HELD}, 0.0, False),
    ("habit: dropped straight down, far end on the tab", {
        "deep_well_plate": ((SEATED[0] + 0.002, NEST_Y, FLOOR_Z + TAB[2] / 2 + 0.004), None, None,
                            tilted_about_y(-_lean)), "nest": FREE}, 0.0, False),
    ("habit: set down on the deck beside the nest", {"deep_well_plate": ((SEATED[0], NEST_Y - 0.10,
                                                                          TOP + PLATE[2] / 2), None, None)}, 0.0, False),
    ("habit: pushed in past the tab but left with the tab still back 9 mm, 5 mm off the datum",
     {"deep_well_plate": at(dx=0.005), "nest": NEST.state(joints={"tab": 0.009})}, 0.0, False),
    ("released 6 mm up: wedged 3.6 mm above the floor between tab and datum (traced)",
     {"deep_well_plate": at(dz=0.0036, dx=-0.0002), "nest": NEST.state(joints={"tab": 0.0041})}, 0.0, False),
    ("lying on the walls' tops", {"deep_well_plate": at(dz=WALL_H + 0.0), "nest": FREE}, 0.0, False),
]
