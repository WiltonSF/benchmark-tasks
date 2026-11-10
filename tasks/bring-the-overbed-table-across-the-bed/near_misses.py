"""Near-successes and unintended strategies, scored offline by the factory checker, in lot A (the turned table). Each case: (label,
{object: spec}, expected score, expected success); a spec is a ready state from the table's own geometry. Cases run
in order and share the whole-trial record, so a fault stays until a scene reset (an empty case after the table was
moved).
"""

import math

from task import BED_X0, FOOT_X, GRIP_X, HOME, START_TURNED, TABLE, TOP, UNDER_IN

HX, HY = HOME


def yawq(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def tiltq(deg, yaw=0.0):
    # tilted about the y axis (forward), then yawed
    a, b = math.radians(deg) / 2, math.radians(yaw) / 2
    qy = (0.0, math.sin(a), 0.0, math.cos(a))
    qz = (0.0, 0.0, math.sin(b), math.cos(b))
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qy
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def table(x=HX, y=HY, z=TOP, yaw=0.0, quat=None, vel=None):
    q = quat if quat is not None else yawq(yaw)  # the declared start is turned; give every case its own yaw
    return {"overbed_table": TABLE.state(pos=(x, y, z), quat_xyzw=q, lin_vel=vel)}


FOOT_MID = sum(FOOT_X) / 2
X_40 = BED_X0 + UNDER_IN - FOOT_MID  # column x with the feet's centres exactly 40 mm inside the bed
RESET = {}

CASES = [
    ("in position", table(), 1.0, True),
    ("in position, still sliding", table(vel=(0.02, 0.0, 0.0)), 0.0, False),
    ("column against the bed", table(x=BED_X0 - 0.0125), 1.0, True),
    ("feet exactly 40 mm under", table(x=X_40), 1.0, True),
    ("feet 39 mm under", table(x=X_40 - 0.001), 0.0, False),
    ("top 60 mm from the hands along the bed", table(y=HY - 0.060), 1.0, True),
    ("top 61 mm from the hands: across, out of reach", table(y=HY - 0.061), 0.0, False),
    ("top 30 mm to the left of the hands", table(y=HY + 0.030), 1.0, True),
    ("across toward the head end, 70 mm from the hands", table(y=HY - 0.070), 0.0, False),
    ("left foot 39 mm inside the bed's left end", table(y=0.165 - 0.039 - 0.065), 0.0, False),
    ("turned 5 degrees, in position", table(yaw=5.0), 1.0, True),
    ("table at its start", {}, 0.0, False),
    ("lot A turned about the grip block, then pushed straight in: a foot past the bed's right end",
     table(y=START_TURNED[1] + GRIP_X), 0.0, False),
    ("lot A turned about the column, then pushed straight in: top 75 mm from the hands", table(y=START_TURNED[1]),
     0.0, False),
    ("pushed in halfway, feet not under", table(x=0.27), 0.0, False),
    ("turned 90 degrees and pushed to the bed: top short of it", table(x=BED_X0 - 0.0125, yaw=90.0), 0.0, False),
    ("turned 90 the other way, pushed to the bed", table(x=BED_X0 - 0.0125, yaw=-90.0), 0.0, False),
    ("column first (turned 180), pushed to the bed", table(x=BED_X0 - 0.0125, yaw=180.0), 0.0, False),
    ("tilted 6 degrees on something, otherwise in position", table(z=TOP + 0.0005, quat=tiltq(6.0)), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("lifted 21 mm while in place", table(z=TOP + 0.021), 0.0, False),
    ("then set down in position: still 0", table(), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("after the reset, in position: 1.0", table(), 1.0, True),
    ("scene reset", RESET, 0.0, False),
    ("lifted 19 mm: no fault, not scored while up", table(z=TOP + 0.019), 0.0, False),
    ("then set down in position: 1.0", table(), 1.0, True),
    ("scene reset", RESET, 0.0, False),
    ("tipped 11 degrees", table(x=0.25, z=TOP + 0.01, quat=tiltq(11.0)), 0.0, False),
    ("then in position: still 0", table(), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("tipped 9 degrees: no fault", table(x=0.25, z=TOP + 0.01, quat=tiltq(9.0)), 0.0, False),
    ("then in position: 1.0", table(), 1.0, True),
    ("scene reset", RESET, 0.0, False),
    ("table set on the bed over the patient (lifted, top on the hands)",
     table(x=0.36, z=TOP + 0.090), 0.0, False),
    ("then in position: still 0", table(), 0.0, False),
]
