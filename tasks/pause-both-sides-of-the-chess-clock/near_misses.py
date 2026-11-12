"""Near-successes and unintended strategies, scored offline by the factory checker, in lot A (left cap raised, the
left clock running). Each case: (label, {object: state}, expected score, expected success). Cases share the
whole-trial record; once the right clock has run, or the clock was shoved or tipped, every later case scores 0 until
a scene reset (offline: the exact start state, an empty case). Offline the hands count as clear and the rocker as
still; the simulator measures both.
"""

import math

from task import CAP_Y, CLOCK, CLOCK_XY, STOP, TOP

LEFT_UP = STOP


def rocker_mm(mm, pos=None, quat=None, vel=None):
    """The rocker with the left cap `mm` above level (negative: below)."""
    return {"chess_clock": CLOCK.state(joints={"rocker": math.asin(mm / 1000 / CAP_Y)}, pos=pos, quat_xyzw=quat,
                                       lin_vel=vel)}


def tilt_y(deg):
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


RESET = {}
CASES = [
    ("left cap up at its stop, as it started", RESET, 0.0, False),
    ("level", rocker_mm(0.0), 1.0, True),
    ("left cap 1.5 mm above level (limit)", rocker_mm(1.5), 1.0, True),
    ("left cap 1.6 mm above level: left clock still runs", rocker_mm(1.6), 0.0, False),
    ("left cap 3 mm up: half pressed", rocker_mm(3.0), 0.0, False),
    ("left cap 1.5 mm below level (limit)", rocker_mm(-1.5), 1.0, True),
    ("level but the clock still sliding", rocker_mm(0.0, vel=(0.05, 0.0, 0.0)), 0.0, False),
    ("level, clock nudged 9 mm", rocker_mm(0.0, pos=(CLOCK_XY[0] - 0.009, CLOCK_XY[1], TOP)), 1.0, True),
    ("scene reset", RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("naive: left cap pressed fully, right clock started", rocker_mm(-6.0), 0.0, False),
    ("then brought back level: still 0", rocker_mm(0.0), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("pressed 1.6 mm past level for a moment", rocker_mm(-1.6), 0.0, False),
    ("then back to level: still 0", rocker_mm(0.0), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("clock shoved 11 mm", {"chess_clock": CLOCK.state(pos=(CLOCK_XY[0] + 0.011, CLOCK_XY[1], TOP))}, 0.0, False),
    ("then paused: still 0", rocker_mm(0.0), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("clock tipped 6 degrees", {"chess_clock": CLOCK.state(quat_xyzw=tilt_y(6), pos=(*CLOCK_XY, TOP + 0.003))},
     0.0, False),
    ("then set down and paused: still 0", rocker_mm(0.0), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("after the reset, paused: 1.0", rocker_mm(0.0), 1.0, True),
]
