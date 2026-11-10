"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A
(lower lever pointing away from the robot, upper lever to the robot's left). Each case: (label, {object: spec},
expected score, expected success); a spec is a ready state from the part's own geometry (UPPER.state(...)). Cases
run in order and share the whole-trial record, so a fault stays until a scene reset (an empty case after the
upper joint was moved).
"""

import math

from task import DROP, LOWER, LOWER_XY, RIM, TENON_L, TOP, UPPER

LX, LY = LOWER_XY


def yaw(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def tilt_x(deg, yaw_deg=0.0):
    """Tilted about world x after a yaw (approximately, for small tilts: compose tilt then yaw)."""
    a, b = math.radians(deg) / 2, math.radians(yaw_deg) / 2
    qx = (math.sin(a), 0.0, 0.0, math.cos(a))
    qz = (0.0, 0.0, math.sin(b), math.cos(b))
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def upper(deg=0.0, dz=0.0, dx=0.0, dy=0.0, quat=None, vel=None):
    """The upper joint over the socket, its lever at `deg` from the lower lever, shoulder `dz` above the rim."""
    return {"upper_joint": UPPER.state(pos=(LX + dx, LY + dy, TOP + RIM + dz),
                                       quat_xyzw=quat if quat is not None else yaw(deg), lin_vel=vel)}


HOME = upper()
SCENE_RESET = {}

CASES = [
    ("home, levers lined up", HOME, 1.0, True),
    ("home, levers 10 degrees apart", upper(10), 1.0, True),
    ("home, levers 12 degrees apart (the limit)", upper(12), 1.0, True),
    ("home, levers 12.5 degrees apart", upper(12.5), 0.0, False),
    ("home, levers -12 degrees apart", upper(-12), 1.0, True),
    ("home, levers 90 degrees apart (no turn from the holder: pads miss)", upper(90), 0.0, False),
    ("home, levers 180 degrees apart", upper(180), 0.0, False),
    ("home, levers 120 degrees apart", upper(120), 0.0, False),
    ("standing on a key post, 12 mm proud, 30 degrees off", upper(30, dz=DROP), 0.0, False),
    ("lined up but 3 mm proud", upper(0, dz=0.003), 0.0, False),
    ("lined up, 2 mm proud (the limit)", upper(0, dz=0.002), 1.0, True),
    ("lined up, 2 mm low (the limit)", upper(0, dz=-0.002), 1.0, True),
    ("lined up, axis 3 mm off centre", upper(0, dx=-0.003), 1.0, True),
    ("lined up but tilted 6 degrees", upper(quat=tilt_x(6)), 0.0, False),
    ("lined up, tilted 4 degrees", upper(quat=tilt_x(4)), 1.0, True),
    ("lined up and home but still moving", upper(vel=(0.0, 0.0, -0.05)), 0.0, False),
    ("upper joint still in its holder", {}, 0.0, False),
    ("upper joint lying on its side on the table",
     {"upper_joint": UPPER.state(pos=(0.22, 0.05, TOP + 0.025), quat_xyzw=(0.7071068, 0.0, 0.0, 0.7071068))},
     0.0, False),
    ("upper joint upside down on the rim, lever lined up (not home)",
     upper(quat=(1.0, 0.0, 0.0, 0.0), dz=0.055), 0.0, False),
    ("upper joint stood beside the lower joint on the table",
     {"upper_joint": UPPER.state(pos=(LX - 0.08, LY, TOP + TENON_L), quat_xyzw=yaw(0))}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("lower joint shoved 11 mm", {"lower_joint": LOWER.state(pos=(LX - 0.011, LY, TOP))}, 0.0, False),
    ("then put back and joined lined up: still 0", HOME, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("after the reset, joined lined up: 1.0", HOME, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("lower joint tipped 6 degrees", {"lower_joint": LOWER.state(pos=(LX, LY, TOP + 0.003), quat_xyzw=tilt_x(6))},
     0.0, False),
    ("then joined lined up: still 0", HOME, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("lower joint nudged 9 mm, joint home lined up on it: 1.0",
     {"lower_joint": LOWER.state(pos=(LX - 0.009, LY, TOP)),
      "upper_joint": UPPER.state(pos=(LX - 0.009, LY, TOP + RIM), quat_xyzw=yaw(0))}, 1.0, True),
]
