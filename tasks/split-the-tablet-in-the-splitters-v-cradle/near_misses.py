"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A
(tablet starting in front of the V, to the robot's left). Each case: (label, {object: state}, expected score,
expected success). States come from the parts' own geometry. Cases run in order and share the whole-trial record,
so a fault stays until a scene reset (an empty case after the tablet was moved).
"""

import math

import numpy as np

from task import PLATE_T, POSES, SEAT_X, SPLITTER, TAB_STARTS, TABLET, TOP, WALL_L, to_world

PT = TOP + PLATE_T
HX = POSES["A"][0][0]
LEFT_START = to_world("A", TAB_STARTS["A"])
ON_LINE = to_world("A", TAB_STARTS["C"])  # lot C's start, used here in lot A: on the blade line, under the bar

S2 = math.sqrt(2.0)
OPEN = SPLITTER.state()  # standing, joint 0 (the sim leans it back onto its stop)
LEANING = SPLITTER.state(joints={"lid": -0.15})
DOWN = SPLITTER.state(joints={"lid": math.pi / 2})  # flat: the bar's underside 10 mm above the plate
HALF_DOWN = SPLITTER.state(joints={"lid": 1.0})
EMPTY_SHUT = SPLITTER.state(joints={"lid": math.pi / 2 + 0.12})


def tab(x, y, z=PT, quat=None, vel=None):
    return TABLET.state(pos=(x, y, z), quat_xyzw=quat, lin_vel=vel)


def tilt_x(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


SEATED = tab(HX + SEAT_X, 0.0)
def off_left_wall(gap):
    """Touching the right wall, `gap` from the left wall: slid along the right wall away from the apex."""
    d = np.array([-1.0, -1.0]) / S2  # along the right wall, away from the apex
    c = np.array([HX + SEAT_X, 0.0]) + d * gap
    return tab(float(c[0]), float(c[1]))


SCENE_RESET = {}

CASES = [
    ("start: tablet in front of the V, lid open", {}, 0.0, False),
    ("tablet seated, lid standing open", {"tablet": SEATED, "splitter": OPEN}, 0.5, False),
    ("tablet seated, lid leaning on its stop", {"tablet": SEATED, "splitter": LEANING}, 0.5, False),
    ("tablet seated, lid half down", {"tablet": SEATED, "splitter": HALF_DOWN}, 0.5, False),
    ("tablet seated, blade resting on it", {"tablet": SEATED, "splitter": DOWN}, 1.0, True),
    ("tablet seated, blade 0.9 mm above it", {"tablet": SEATED,
                                              "splitter": SPLITTER.state(joints={"lid": math.pi / 2 - 0.0009 / 0.040})},
     1.0, True),
    ("tablet seated, blade 2 mm above it (lid propped)", {"tablet": SEATED,
                                                          "splitter": SPLITTER.state(joints={"lid": math.pi / 2 - 0.08})},
     0.5, False),
    ("tablet seated but still moving", {"tablet": tab(HX + SEAT_X, 0.0, vel=(0.03, 0.0, 0.0)), "splitter": OPEN}, 0.0,
     False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("1 mm from the left wall exactly (still seated)", {"tablet": off_left_wall(0.001 - 1e-7), "splitter": OPEN},
     0.5, False),
    ("1.2 mm from the left wall: not seated", {"tablet": off_left_wall(0.0012), "splitter": OPEN}, 0.0, False),
    ("on the blade line, 8 mm short of both walls", {"tablet": tab(HX + SEAT_X - 0.008 * S2, 0.0), "splitter": OPEN},
     0.0, False),
    ("tablet tipped 15 degrees against the walls", {"tablet": tab(HX + SEAT_X, 0.0, z=PT + 0.002, quat=tilt_x(15)),
                                                    "splitter": OPEN}, 0.0, False),
    ("tablet riding 2 mm up on a wall", {"tablet": tab(HX + SEAT_X, 0.0, z=PT + 0.002), "splitter": OPEN}, 0.0, False),
    ("tablet still at its start, lid shut empty", {"tablet": tab(*LEFT_START), "splitter": EMPTY_SHUT}, 0.0,
     False),
    ("then pushed into the V under the shut lid (cannot pass; no fault)", {"tablet": SEATED, "splitter": OPEN}, 0.5,
     False),
    ("scene reset", SCENE_RESET, 0.0, False),
    # Faults: the blade brought down on a tablet that is not seated.
    ("blade down on a tablet against the right wall only, 4 mm off the line",
     {"tablet": off_left_wall(0.004 * S2), "splitter": DOWN}, 0.0, False),
    ("then seated with the blade on it: still 0", {"tablet": SEATED, "splitter": DOWN}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("after the reset, seated and blade down: 1.0", {"tablet": SEATED, "splitter": DOWN}, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("blade down on a tablet on the line but 8 mm short of the walls",
     {"tablet": tab(HX + SEAT_X - 0.008 * S2, 0.0), "splitter": DOWN}, 0.0, False),
    ("then pushed home under the blade: still 0", {"tablet": SEATED, "splitter": DOWN}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("blade down beside the tablet (tablet 20 mm off the line, not under the bar): no fault",
     {"tablet": tab(HX + SEAT_X - 0.03, 0.020), "splitter": EMPTY_SHUT}, 0.0, False),
    ("then lid lifted, tablet seated, blade down: 1.0", {"tablet": SEATED, "splitter": DOWN}, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("seated, blade on it, tablet 1 mm up (the on-plate limit)", {"tablet": tab(HX + SEAT_X, 0.0, z=PT + 0.001),
                                                                   "splitter": DOWN}, 1.0, True),
    ("seated, blade on it, tablet bounced 1.5 mm up: not counted, no fault",
     {"tablet": tab(HX + SEAT_X, 0.0, z=PT + 0.0015), "splitter": DOWN}, 0.0, False),
    ("then settled back: 1.0", {"tablet": SEATED, "splitter": DOWN}, 1.0, True),
    ("seated, blade on it, tablet 1.4 mm from the left wall (a bounce): no fault",
     {"tablet": off_left_wall(0.0014), "splitter": DOWN}, 0.0, False),
    ("then settled back: 1.0", {"tablet": SEATED, "splitter": DOWN}, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("blade closed at once on a tablet lying on the blade line, short of the V",
     {"tablet": tab(*ON_LINE), "splitter": DOWN}, 0.0, False),
    ("then pushed home under the blade: still 0", {"tablet": SEATED, "splitter": DOWN}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("a wall shoved 3 mm, tablet seated against the walls' old place",
     {"wall_left": WALL_L.state(pos=(WALL_L.pos[0] + 0.003, WALL_L.pos[1], WALL_L.pos[2]))}, 0.0, False),
    ("then seated, blade down: still 0", {"tablet": SEATED, "splitter": DOWN}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("splitter shoved 4 mm", {"splitter": SPLITTER.state(pos=(HX - 0.004, 0.0, TOP))}, 0.0, False),
    ("then seated, blade down: still 0", {"tablet": SEATED, "splitter": DOWN}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("seated, blade on it, then tablet pulled out from under: fault", {"tablet": tab(HX + SEAT_X - 0.006, 0.0),
                                                                       "splitter": DOWN}, 0.0, False),
    ("then reseated: still 0", {"tablet": SEATED, "splitter": DOWN}, 0.0, False),
]
