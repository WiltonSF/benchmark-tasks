"""Near-successes and wrong outcomes, scored offline by the factory checker (lot A: fire on the left).

Each case: (label, {object: state}, expected score, expected success). Cases run in order against one copy of the
task module; offline each score() call counts as one 50 ms control step, and a case with every object at its start
clears the trial record (a new trial). So a "hold" is a run of identical cases. The extinguisher's joints: "lever"
(rad squeezed, 0 up, 0.33 at the handle) and "horn" (swivel, rad counter-clockwise seen from above, 0 straight
ahead).
"""

import math

import numpy as np

from task import AIM_DEG, EXT, PIN

FIRE = math.radians(AIM_DEG)  # lot A: the burning bin is on the left
PIN_GONE = PIN.state(pos=np.array(PIN.pos) + [0.0, -0.12, -0.11])  # pulled out and dropped on the table


def s(lever=0.0, horn=0.0, pin=True):
    out = {"extinguisher": EXT.state(joints={"lever": lever, "horn": horn})}
    if not pin:
        out["pin"] = PIN_GONE
    return out


START = ("untouched (new trial)", {}, 0.0, False)


def hold(label, state, steps, want_last=0.0, ok_last=False):
    """steps identical cases; only the last is labelled with the expected result after the hold."""
    return [(f"{label} ({k + 1}/{steps})", state, 0.0 if k < steps - 1 else want_last,
             False if k < steps - 1 else ok_last) for k in range(steps)]


CASES = (
    [START]
    # Full method: pin out, aim, squeeze 2 s (41 calls: success is checked before score's step, so 41 > 40), let go.
    + hold("pin out, aimed, squeezed", s(0.30, FIRE, pin=False), 41)
    + [("then let go, horn still on the fire", s(0.0, FIRE, pin=False), 1.0, True)]
    # Squeezed only 1.5 s.
    + [START] + hold("squeezed 1.5 s", s(0.30, FIRE, pin=False), 30)
    + [("then let go after 1.5 s", s(0.0, FIRE, pin=False), 0.0, False)]
    # Not squeezed far enough (0.27 rad).
    + [START] + hold("lever only to 0.27 rad", s(0.27, FIRE, pin=False), 45)
    + [("then let go after a short squeeze", s(0.0, FIRE, pin=False), 0.0, False)]
    # Horn 9 degrees off the bin centre (about 20 mm at the bin: inside the 40 mm) and 15 degrees (outside).
    + [START] + hold("aimed 9 degrees off", s(0.30, FIRE - math.radians(9), pin=False), 41)
    + [("then let go, 9 degrees off", s(0.0, FIRE - math.radians(9), pin=False), 1.0, True)]
    + [START] + hold("aimed 25 degrees off", s(0.30, FIRE - math.radians(25), pin=False), 41)
    + [("then let go, 25 degrees off", s(0.0, FIRE - math.radians(25), pin=False), 0.0, False)]
    # Still squeezed at the end.
    + [START] + hold("discharge, lever still held at the end", s(0.30, FIRE, pin=False), 45)
    # Habits.
    + [START] + hold("habit: squeeze where it points (straight ahead), pin out", s(0.30, 0.0, pin=False), 41)
    + [("then let go", s(0.0, 0.0, pin=False), 0.0, False)]
    + [START] + hold("habit: discharge at the blue bin (swung right), then onto the fire",
                     s(0.30, -FIRE, pin=False), 5)
    + hold("  ... then swung to the fire and squeezed 2 s", s(0.30, FIRE, pin=False), 41)
    + [("then let go after discharging at both", s(0.0, FIRE, pin=False), 0.0, False)]
    + [START] + hold("habit: aimed at the fire, pin still in (lever cannot move)", s(0.0, FIRE, pin=True), 45)
    # Partial squeezes while pointing elsewhere: 0.12 rad or more at the blue bin is a fault for the whole trial.
    + [START] + hold("lever 0.11 rad at the blue bin", s(0.11, -FIRE, pin=False), 5)
    + hold("  ... then swung to the fire and squeezed 2 s", s(0.30, FIRE, pin=False), 41)
    + [("then let go after a 0.11 rad press at the blue bin", s(0.0, FIRE, pin=False), 1.0, True)]
    + [START] + hold("lever 0.12 rad at the blue bin", s(0.12, -FIRE, pin=False), 5)
    + hold("  ... then swung to the fire and squeezed 2 s", s(0.30, FIRE, pin=False), 41)
    + [("then let go after a 0.12 rad press at the blue bin", s(0.0, FIRE, pin=False), 0.0, False)]
    + [START] + hold("lever 0.12 rad straight ahead (at nothing)", s(0.12, 0.0, pin=False), 3)
    + hold("  ... then swung to the fire and squeezed 2 s", s(0.30, FIRE, pin=False), 41)
    + [("then let go after a 0.12 rad press at nothing", s(0.0, FIRE, pin=False), 0.0, False)]
    + [START] + hold("pin in, lever pushed to 0.09 rad (as on the GPU) at nothing", s(0.09, 0.0, pin=True), 5)
    + hold("  ... then pin out, aimed, squeezed 2 s", s(0.30, FIRE, pin=False), 41)
    + [("then let go after the pin-in push", s(0.0, FIRE, pin=False), 1.0, True)]
    + [START] + hold("pin pulled and horn aimed, never squeezed", s(0.0, FIRE, pin=False), 45)
    + [START, ("pin pulled only", s(pin=False), 0.0, False)]
)

