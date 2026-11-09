"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A.

Each case: (label, {object: spec}, expected score, expected success). Offline a pusher counts as released when it has
moved more than 2 mm off its stop (in the simulator, only its button releases it). The fixture-moved case is last:
its record lasts for the rest of the trial.
"""

import math

from task import (BLANK, BLANK_LOCAL, BLANK_Z, END_FACE_X, FIX_XY, FIXTURE, GAP, SHORT_END, SHORT_SIDE,
                  SIDE_FACE_Y, TOP)

CX, CY = FIX_XY[0] + END_FACE_X - BLANK[0] / 2, FIX_XY[1] + SIDE_FACE_Y - BLANK[1] / 2  # clamped centre, lot A
SX, SY = FIX_XY[0] + BLANK_LOCAL[0], FIX_XY[1] + BLANK_LOCAL[1]  # start centre, lot A


def fixture(end=None, side=None, dx=0.0):
    """Pusher positions: None leaves it latched at 0; a number is how far its face is from the clamped blank's face."""
    q = {}
    if end is not None:
        q["end_pusher"] = GAP + SHORT_END - end
    if side is not None:
        q["side_pusher"] = GAP + SHORT_SIDE - side
    pos = (FIXTURE.pos[0] + dx, FIXTURE.pos[1], FIXTURE.pos[2])
    return {"fixture": FIXTURE.state(joints=q, pos=pos)}


def blank(dx=0.0, dy=0.0, dz=0.0, quat=None, vel=None):
    spec = ((CX + dx, CY + dy, BLANK_Z + dz), None, vel)
    return {"blank": spec if quat is None else (*spec, quat)}


def yaw(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


CASES = [
    ("clamped: both locators, both pushers on it", {**blank(), **fixture(0.0, 0.0)}, 1.0, True),
    ("1 mm off the end locator, pusher following", {**blank(dx=-0.001), **fixture(0.001, 0.0)}, 1.0, True),
    ("2 mm off the end locator", {**blank(dx=-0.002), **fixture(0.002, 0.0)}, 0.0, False),
    ("1 mm off the side locator", {**blank(dy=-0.001), **fixture(0.0, 0.001)}, 1.0, True),
    ("2 mm off the side locator", {**blank(dy=-0.002), **fixture(0.0, 0.002)}, 0.0, False),
    ("wrong order: pinned 15 mm short of the end locator, both released",
     {**blank(dx=-SHORT_END), **fixture(SHORT_END, 0.0)}, 0.0, False),
    ("both buttons at once: the side pusher wins, blank pinned short",
     {**blank(dx=-SHORT_END), **fixture(SHORT_END, 0.0)}, 0.0, False),
    ("against both locators, side pusher still latched", {**blank(), **fixture(0.0, None)}, 0.0, False),
    ("against both locators, end pusher still latched", {**blank(), **fixture(None, 0.0)}, 0.0, False),
    ("against both locators, a pusher 3 mm off the blank", {**blank(), **fixture(0.003, 0.0)}, 0.0, False),
    ("turned 2 degrees (one corner off the end locator)", {**blank(dx=-0.0006, quat=yaw(2)), **fixture(0.0, 0.0)},
     0.0, False),
    ("lifted 3 mm off the plate", {**blank(dz=0.003), **fixture(0.0, 0.0)}, 0.0, False),
    ("still moving", {**blank(vel=(0.02, 0.0, 0.0)), **fixture(0.0, 0.0)}, 0.0, False),
    ("only the end pusher released, blank on the end locator", {**blank(dy=-SHORT_SIDE), **fixture(0.0, None)},
     0.0, False),
    ("blank lifted off the plate and set on the table", {"blank": ((0.20, 0.30, TOP + BLANK[2] / 2), None, None)},
     0.0, False),
    ("blank untouched, both pushers released and on it (wrong order end state)",
     {"blank": ((SX, CY, BLANK_Z), None, None), **fixture(SHORT_END, 0.0)}, 0.0, False),
    ("fixture moved 4 mm, blank clamped in it", {**blank(dx=0.004), **fixture(0.0, 0.0, dx=0.004)}, 1.0, True),
    # Last: the whole-trial fixture record stays tripped.
    ("fixture shoved 6 mm, blank clamped in it", {**blank(dx=0.006), **fixture(0.0, 0.0, dx=0.006)}, 0.0, False),
    ("fixture put back after being shoved, blank clamped", {**blank(), **fixture(0.0, 0.0)}, 0.0, False),
]
