"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A
(lever pointing away from the robot). Each case: (label, {object: state}, expected score, expected success). Cases
run in order and share the whole-trial record of the light; the record is cleared only by the simulator's reset,
so cases after a light fault are declared failing.
"""

import math

from task import ARM, ELBOW_XY, LEVER, SEAT_Z, TOP


def yaw_q(deg, tilt_deg=0.0):
    """Lever turned `deg` about the vertical (0 points away from the robot), then tipped `tilt_deg` about its x."""
    y, t = math.radians(deg) / 2, math.radians(tilt_deg) / 2
    qz = (0.0, 0.0, math.sin(y), math.cos(y))
    qx = (math.sin(t), 0.0, 0.0, math.cos(t))
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def lever(deg, lift=0.0, tilt=0.0, vel=None):
    return {"clamp_lever": LEVER.state(pos=(*ELBOW_XY, TOP + SEAT_Z + lift), quat_xyzw=yaw_q(deg, tilt), lin_vel=vel)}


def light(turn_deg, screw_deg=0.0):
    return {"light_arm": ARM.state(joints={"tilt": math.radians(turn_deg), "screw": math.radians(screw_deg)})}


CASES = [
    ("lever at its start, pointing away", lever(0), 0.0, False),
    ("stowed: seated pointing back along the arm", lever(90), 1.0, True),
    ("stowed 15 deg short (limit)", lever(75), 1.0, True),
    ("stowed 15 deg past (limit)", lever(105), 1.0, True),
    ("16 deg short", lever(74), 0.0, False),
    ("16 deg past", lever(106), 0.0, False),
    ("pointing along the arm toward the light", lever(-90), 0.0, False),
    ("pointing toward the robot", lever(180), 0.0, False),
    ("lot B start: pointing along the arm toward the light", lever(-90), 0.0, False),
    ("lifted 1.5 mm (limit)", lever(90, lift=0.0015), 1.0, True),
    ("lifted 2 mm: resting on the drive, not seated", lever(90, lift=0.002), 0.0, False),
    ("held up off the drive at the stow angle", lever(90, lift=0.007), 0.0, False),
    ("seated but tipped 6 deg", lever(90, tilt=6.0), 0.0, False),
    ("stowed but still moving", lever(90, vel=(0.02, 0.0, 0.0)), 0.0, False),
    ("stowed, light turned 2 deg (limit)", {**lever(90), **light(2.0)}, 1.0, True),
    ("stowed, screw turned 9 deg, light unmoved", {**lever(90), **light(0.0, 9.0)}, 1.0, True),
    # From here the light has drooped once in this trial: nothing scores until the scene reset.
    ("turned the lever on the drive: light drooped", {**lever(90), **light(40.0, 80.0)}, 0.0, False),
    ("light pushed back up afterwards", {**lever(90), **light(0.0, 80.0)}, 0.0, False),
    ("screw tightened again, light back", lever(90), 0.0, False),
]
