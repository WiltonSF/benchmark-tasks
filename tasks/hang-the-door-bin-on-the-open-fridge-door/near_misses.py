"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), with the
door square to the arms (lot A's layout). Pass/fail. Each case: (label, {object: state}, expected score, expected success); states come from
the parts' own geometry (Part.state(...)).
"""

import math

import numpy as np

from task import BIN_D, CH_Z1, DOOR, DOOR_BIN, DOOR_H, DOOR_XY, FOOT, LIP_X0, STOP_Z, TOP

FLUSH_X = LIP_X0 - BIN_D / 2  # the bin origin's x' with its back against the channel fronts
MID_X = FLUSH_X - 0.003  # the middle of the tabs' 6 mm of play


def q_yaw(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def q_roll(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def q_pitch(deg):
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def hung(dx=0.0, dy=0.0, dz=0.0, door_yaw=0.0, quat=None, vel=None):
    """The bin at the hung pose in the door frame, offset (dx into the door, dy along it, dz up)."""
    a = math.radians(door_yaw)
    r = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    xy = np.asarray(DOOR_XY) + r @ np.array([MID_X + dx, dy])
    out = {"door_bin": DOOR_BIN.state(pos=(xy[0], xy[1], TOP + STOP_Z + dz),
                                      quat_xyzw=quat if quat is not None else q_yaw(door_yaw), lin_vel=vel)}
    if door_yaw:
        out["fridge_door"] = DOOR.state(pos=(*DOOR_XY, TOP), quat_xyzw=q_yaw(door_yaw))
    return out


CASES = [
    ("hung: both tabs on the stops, back against the channel fronts", hung(), 1.0, True),
    ("hung, tabs 3 mm above the stops (settling)", hung(dz=0.003), 1.0, True),
    ("tabs 4 mm above the stops, caught in the channels", hung(dz=0.004), 0.0, False),
    ("tabs half way down the channels, let go and still (jammed)", hung(dz=0.020), 0.0, False),
    ("heads 9 mm into the channels, let go and still", hung(dz=CH_Z1 - STOP_Z - 0.009), 0.0, False),
    ("heads 10 mm into the channels, let go and still (jammed)", hung(dz=CH_Z1 - STOP_Z - 0.010), 0.0, False),
    ("hung, back against the channel fronts", hung(dx=0.003), 1.0, True),
    ("hung, heads pulled forward against the lips", hung(dx=-0.003), 1.0, True),
    ("bin resting on the channel tops", hung(dz=CH_Z1 - STOP_Z), 0.0, False),
    ("hung, 3.5 mm along the door (the pocket's play)", hung(dy=0.0035), 1.0, True),
    ("4 mm along the door: a head outside its pocket", hung(dy=0.004), 0.0, False),
    ("hung but still swinging", hung(vel=(0.0, 0.0, 0.03)), 0.0, False),
    ("one tab on its stop, the bin leaning 4 degrees", hung(dz=0.0056, quat=q_roll(4.0)), 0.0, False),
    ("hung, leaning forward 6 degrees on its tabs", hung(dz=-0.0046, quat=q_pitch(-6.0)), 1.0, True),
    ("leaning forward 11 degrees", hung(dx=0.003, dz=-0.0081, quat=q_pitch(-11.0)), 0.0, False),
    ("hung, turned 1.5 degrees to the door (the play allows about 2)", hung(quat=q_yaw(1.5)), 1.0, True),
    ("bin held against the door in front of the channels, at hang height", hung(dx=-0.030), 0.0, False),
    ("bin upside down with its tabs in the channels", hung(dz=0.050, quat=(1.0, 0.0, 0.0, 0.0)), 0.0, False),
    ("bin left on the table at its start", {}, 0.0, False),
    ("bin set on the table against the door's foot", {"door_bin": DOOR_BIN.state(pos=(0.31, 0.0, TOP))}, 0.0,
     False),
    ("bin set on top of the door", {"door_bin": DOOR_BIN.state(pos=(DOOR_XY[0] + 0.015, 0.0, TOP + FOOT[2] + DOOR_H))},
     0.0, False),
    ("door turned 30 degrees, bin hung square to it", hung(door_yaw=30.0), 1.0, True),
    ("door turned 30 degrees, bin hung square to it, tabs 4 mm up", hung(door_yaw=30.0, dz=0.004), 0.0, False),
    ("door turned -40 degrees, bin hung square to it", hung(door_yaw=-40.0), 1.0, True),
    ("door turned 30 degrees, bin at yaw 0 in front of it", {"fridge_door": DOOR.state(pos=(*DOOR_XY, TOP),
                                                                                        quat_xyzw=q_yaw(30.0)),
                                                              **{k: v for k, v in hung().items()}}, 0.0, False),
]
