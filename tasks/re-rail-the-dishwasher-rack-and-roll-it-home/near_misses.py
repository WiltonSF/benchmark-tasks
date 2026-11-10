"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py). Scoring is the
same in both lots; the cases use lot A's layout (the rack starts against the right wall with its left wheels
dropped). Each case: (label, {object: state}, expected score, expected success); a state comes from the rack's own
geometry (RACK.state(...)). Cases run in order and share the whole-trial record, so a heater fault stays until a
scene reset (an empty case after the rack was moved).
"""

import math

from task import DROP, HALF_L, HEATER_DX, HEATER_X, HOME_X, LOT_A_XY, ON_RAILS_Z, RACK, TOP, WHEEL_Y

HOME = (HOME_X, 0.0, ON_RAILS_Z)


def roll(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def pitch(deg):
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def yaw(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def rack(x, y=0.0, z=ON_RAILS_Z, quat=None, vel=None):
    return {"rack": RACK.state(pos=(x, y, z), quat_xyzw=quat, lin_vel=vel)}


def dropped(x, side=1.0, y0=None):
    """The rack turned about one side's wheel centres until the other side's wheels sit on the tub floor."""
    y0 = -side * 0.0195 if y0 is None else y0
    a = -side * DROP
    pivot_y = y0 - side * WHEEL_Y
    return rack(x, pivot_y + side * WHEEL_Y * math.cos(a), ON_RAILS_Z - WHEEL_Y * math.sin(DROP),
                quat=roll(math.degrees(a)))


SCENE_RESET = {}
HEATER_FACE = HEATER_X - HEATER_DX / 2

CASES = [
    ("home: all four wheels on the rails, back against the wall", rack(*HOME), 1.0, True),
    ("home, back 15 mm from the wall (the limit)", rack(HOME_X - 0.015), 1.0, True),
    ("back 16 mm from the wall: on the rails, not home", rack(HOME_X - 0.016), 0.5, False),
    ("on the rails at the start spot, not pushed in", rack(LOT_A_XY[0]), 0.5, False),
    ("on the rails near the open front", rack(0.24), 0.5, False),
    ("home, shifted 10 mm right: left wheels' centres on the rails' inner edge", rack(HOME_X, -0.010), 1.0, True),
    ("home, shifted 10.5 mm right: left wheels' centres just off the rail", rack(HOME_X, -0.0105), 0.0, False),
    ("home, shifted 10.5 mm left: right wheels' centres just off the rail", rack(HOME_X, 0.0105), 0.0, False),
    ("home, 3 mm above the rails (dropping onto them)", rack(HOME_X, z=ON_RAILS_Z + 0.003), 1.0, True),
    ("held 4 mm above the rails at home", rack(HOME_X, z=ON_RAILS_Z + 0.004), 0.0, False),
    ("home but rolling", rack(*HOME[:2], vel=(0.02, 0.0, 0.0)), 0.0, False),
    ("home, turned 3 degrees in yaw (wheels still over the rails)", rack(HOME_X - 0.005, quat=yaw(3)), 1.0, True),
    ("home, pitched 2.5 degrees (front wheels lifted)", rack(HOME_X, z=ON_RAILS_Z + 0.0033, quat=pitch(2.5)), 0.0,
     False),
    ("start: left side down, against the right wall", dropped(LOT_A_XY[0], 1.0), 0.0, False),
    ("left side down, pushed in to 3 mm short of the heater", dropped(HEATER_FACE - HALF_L - 0.003), 0.0, False),
    ("then lifted on and pushed home: 1.0 (it never touched the heater)", rack(*HOME), 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("rack lifted out and set on the door", rack(0.08, 0.0, TOP + 0.006 + 0.015), 0.0, False),
    ("rack set on the table beside the dishwasher", rack(0.25, 0.30, TOP + 0.015), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("left side down, pushed into the heater", dropped(HEATER_FACE - HALF_L - 0.001), 0.0, False),
    ("then lifted on and pushed home: still 0", rack(*HOME), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("after the reset, home: 1.0", rack(*HOME), 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("right side down (lot B's fault), pushed into the heater", dropped(HEATER_FACE - HALF_L - 0.001, -1.0), 0.0,
     False),
    ("then home: still 0", rack(*HOME), 0.0, False),
]
