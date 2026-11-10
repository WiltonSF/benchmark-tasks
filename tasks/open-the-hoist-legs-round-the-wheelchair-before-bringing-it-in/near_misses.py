"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A
(hoist and chair on the midline). Each case: (label, {object: state}, expected score, expected success). States come
from the parts' own geometry. Cases run in order and share the whole-trial record, so a fault stays until a scene
reset (an empty case after the hoist was moved).
"""

import math

from task import CHAIR, CHAIR_X, HOIST, HOIST_X0, HOIST_X1, LEGS_OPEN, LEG_OPEN, TOP

OPEN = {"left_leg": LEG_OPEN, "right_leg": LEG_OPEN}


def hoist(x, y=0.0, legs=None, z=TOP, quat=None, vel=None):
    return HOIST.state(pos=(x, y, z), joints=legs or {}, quat_xyzw=quat, lin_vel=vel)


def yaw_q(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def tilt_y(deg):
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


IN_PLACE = {"hoist": hoist(HOIST_X1, legs=OPEN)}
SCENE_RESET = {}

CASES = [
    ("start: legs closed, 150 mm short", {}, 0.0, False),
    ("legs open, not moved in", {"hoist": hoist(HOIST_X0, legs=OPEN)}, 0.5, False),
    ("legs open exactly at 0.45 rad, not moved in",
     {"hoist": hoist(HOIST_X0, legs={"left_leg": LEGS_OPEN, "right_leg": LEGS_OPEN})}, 0.5, False),
    ("one leg at 0.44 rad", {"hoist": hoist(HOIST_X0, legs={"left_leg": 0.44, "right_leg": LEG_OPEN})}, 0.0, False),
    ("one leg open only", {"hoist": hoist(HOIST_X0, legs={"left_leg": LEG_OPEN})}, 0.0, False),
    ("legs open, dragged 10 mm back from its start", {"hoist": hoist(HOIST_X0 - 0.010, legs=OPEN)}, 0.0, False),
    ("legs open, 2 mm back from its start (within the allowance)", {"hoist": hoist(HOIST_X0 - 0.002, legs=OPEN)}, 0.5,
     False),
    ("legs flung open by spinning the hoist, left turned 90 degrees", {"hoist": hoist(HOIST_X0, legs=OPEN,
                                                                                    quat=yaw_q(90))}, 0.0, False),
    ("legs open, hoist turned 16 degrees", {"hoist": hoist(HOIST_X0, legs=OPEN, quat=yaw_q(16))}, 0.0, False),
    ("legs open, hoist turned 14 degrees", {"hoist": hoist(HOIST_X0, legs=OPEN, quat=yaw_q(14))}, 0.5, False),
    ("legs open, 50 mm short of position", {"hoist": hoist(HOIST_X1 - 0.050, legs=OPEN)}, 0.5, False),
    ("legs open, spreader 31 mm short of the seat centre", {"hoist": hoist(HOIST_X1 - 0.031, legs=OPEN)}, 0.5,
     False),
    ("legs open, spreader 29 mm short", {"hoist": hoist(HOIST_X1 - 0.029, legs=OPEN)}, 1.0, True),
    ("legs open, spreader over the seat centre", IN_PLACE, 1.0, True),
    ("in position but turned 3 degrees", {"hoist": hoist(HOIST_X1, legs=OPEN, quat=yaw_q(3))}, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("in position but turned 10 degrees: a leg meets the chair", {"hoist": hoist(HOIST_X1, legs=OPEN, quat=yaw_q(10))}, 0.0, False),
    ("in position, still rolling", {"hoist": hoist(HOIST_X1, legs=OPEN, vel=(0.02, 0.0, 0.0))}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("legs closed, pushed into the chair's front legs", {"hoist": hoist(CHAIR_X - 0.050 - 0.200 + 0.0005)}, 0.0,
     False),
    ("then legs opened and moved into place: still 0", IN_PLACE, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("legs closed, stopped 2 mm short of the chair: no fault", {"hoist": hoist(CHAIR_X - 0.050 - 0.200 - 0.002)},
     0.0, False),
    ("then opened and moved in: 1.0", IN_PLACE, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("one leg open, pushed in: the closed leg hits the chair",
     {"hoist": hoist(HOIST_X1, legs={"left_leg": LEG_OPEN})}, 0.0, False),
    ("then the other opened: still 0", IN_PLACE, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("legs open but hoist pushed on until its mast meets the seat", {"hoist": hoist(CHAIR_X - 0.050 - 0.030,
                                                                                   legs=OPEN)}, 0.0, False),
    ("then pulled back into place: still 0", IN_PLACE, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("chair shoved 6 mm", {"wheelchair": CHAIR.state(pos=(CHAIR_X + 0.006, 0.0, TOP)), **IN_PLACE}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("hoist lifted 11 mm (over the chair)", {"hoist": hoist(HOIST_X0 + 0.05, z=TOP + 0.011)}, 0.0, False),
    ("then set down in place: still 0", IN_PLACE, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("hoist tipped 16 degrees", {"hoist": hoist(HOIST_X0, legs=OPEN, z=TOP + 0.0, quat=tilt_y(16))}, 0.0, False),
    ("then in place: still 0", IN_PLACE, 0.0, False),
]
