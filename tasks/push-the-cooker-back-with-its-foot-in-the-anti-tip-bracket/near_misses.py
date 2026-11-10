"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A
(bracket under the rear-left foot of a cooker centred at y +4 cm; cooker starts at y -1 cm). Each case: (label,
{object: spec}, expected score, expected success); a spec is a ready state from the part's own geometry. Cases run in
order and share the whole-trial record, so a fault stays until a scene reset (an empty case after the cooker moved).
"""

import math

from task import (A_BR, A_START, BRACKET, BR_DEPTH, COOKER, DEPTH, FOOT_X, FOOT_Y, HOME_X, STOP_FACE, TOP)

BY = A_BR[1]  # tunnel centre line in lot A
TY = BY - FOOT_Y  # cooker centre y in line: +4 cm


def yaw_q(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def lean_q(deg):
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def cooker(x=HOME_X, y=TY, z=TOP, quat=None, vel=None):
    return {"cooker": COOKER.state(pos=(x, y, z), quat_xyzw=quat, lin_vel=vel)}


HOME = cooker()
RESET = {}
SHORT = HOME_X - 0.030  # pad against the side wall's front: stopped about 3 cm short

CASES = [
    ("installed: back at the wall, foot centred in the tunnel", HOME, 1.0, True),
    ("installed, back 10 mm off the bracket plate (foot still under the hood)", cooker(x=HOME_X - 0.010), 1.0, True),
    ("back 11 mm off the bracket plate", cooker(x=HOME_X - 0.011), 0.25, False),
    ("installed, foot 6 mm off the tunnel's centre line", cooker(y=TY + 0.006), 1.0, True),
    ("home, foot 7 mm off the centre line", cooker(y=TY - 0.007), 0.0, False),
    ("home against the wall, foot beside the bracket (unsecured, looks done)", cooker(y=TY + 0.040), 0.0, False),
    ("home against the wall, the wrong rear foot in line", cooker(y=BY + FOOT_Y), 0.0, False),
    ("installed but turned 3 degrees", cooker(quat=yaw_q(3.0)), 1.0, True),
    ("home, turned 4 degrees", cooker(quat=yaw_q(4.0)), 0.0, False),
    ("installed but still sliding", cooker(vel=(0.03, 0.0, 0.0)), 0.0, False),
    ("pushed straight from the start: pad against the side wall, 3 cm short", cooker(x=SHORT, y=A_START[1]), 0.0,
     False),
    ("lined up and 5 cm back, not yet in: 0.25", cooker(x=A_START[0] + 0.050), 0.25, False),
    ("lined up, 4.9 cm back", cooker(x=A_START[0] + 0.049), 0.0, False),
    ("lined up within 5 mm, 6 cm back", cooker(x=A_START[0] + 0.060, y=TY + 0.005), 0.25, False),
    ("6 mm out of line, 6 cm back", cooker(x=A_START[0] + 0.060, y=TY + 0.006), 0.0, False),
    ("lined up, 6 cm back, turned 4 degrees", cooker(x=A_START[0] + 0.060, quat=yaw_q(4.0)), 0.0, False),
    ("moved back 6 cm at the start's y (out of line)", cooker(x=A_START[0] + 0.060, y=A_START[1]), 0.0, False),
    ("only moved sideways into line", cooker(x=A_START[0], y=TY), 0.0, False),
    ("habit: centred on the wall's middle and pushed home (foot 4 cm off line)", cooker(y=0.0), 0.0, False),
    ("habit: pushed straight home from the start (foot 5 cm off, beside the bracket)", cooker(y=A_START[1]), 0.0, False),
    ("cooker lifted 5 mm over the bracket at home", cooker(z=TOP + 0.005), 0.0, False),
    ("cooker leaning 5 degrees on the wall", cooker(z=TOP + 0.001, quat=lean_q(-5.0)), 0.0, False),
    ("cooker at its start", {}, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("cooker tipped 20 degrees", cooker(x=A_START[0], y=A_START[1], z=TOP + 0.01, quat=lean_q(20.0)), 0.0, False),
    ("then set back and installed: still 0", HOME, 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("after the reset, installed: 1.0", HOME, 1.0, True),
    ("scene reset", RESET, 0.0, False),
    ("bracket dragged 6 mm toward the robot",
     {"bracket": BRACKET.state(pos=(A_BR[0] - 0.006, BY, TOP)), **cooker(x=HOME_X - 0.006)}, 0.0, False),
    ("bracket put back, cooker installed: still 0", HOME, 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("bracket nudged 4 mm, cooker installed over it",
     {"bracket": BRACKET.state(pos=(A_BR[0] - 0.004, BY, TOP)), **cooker(x=HOME_X - 0.004)}, 1.0, True),
    ("scene reset", RESET, 0.0, False),
    ("bracket dragged sideways with the foot into line",
     {"bracket": BRACKET.state(pos=(A_BR[0], A_START[1] + FOOT_Y, TOP)), **cooker(y=A_START[1])}, 0.0, False),
]

# Not cases (checked by hand): the back's x is WALL_FACE - DEPTH/2 at HOME_X; the bracket's depth BR_DEPTH and the
# foot's FOOT_X put the rear pad's centre over the bracket's centre at home.
assert abs(HOME_X + FOOT_X - A_BR[0]) < 0.0011 and abs(HOME_X + DEPTH / 2 - STOP_FACE) < 1e-9 and BR_DEPTH > 0
