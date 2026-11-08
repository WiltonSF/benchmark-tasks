"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A
(driver on the right-hand stand). Each case: (label, {object: state}, expected score, expected success); states come
from the parts' own geometry. Offline there is no robot, so the grippers count as clear.
"""

import math

from task import DRIFT, DRIVER, FACE, HOUSING, HX, HY, PROUD, REACH, SX, SY, TOP

MM = 0.001


def bush(depth_mm):
    """The bushing with its top `depth_mm` below the face (negative: proud)."""
    return {"housing": HOUSING.state(joints={"seat": PROUD + depth_mm * MM})}


def tilt_x(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


DRIVER_IN_BORE = {"driver": DRIVER.state(pos=(HX, HY, FACE - REACH))}  # collar on the face
DRIFT_ON_BORE = {"drift": DRIFT.state(pos=(HX, HY, FACE))}  # its mushroomed end on the face over the bore
DRIVER_BACK = {"driver": DRIVER.state(pos=(SX, -SY, DRIVER.pos[2]))}
DRIVER_ON_FACE = {"driver": DRIVER.state(pos=(HX + 0.02, HY, FACE + 0.014), quat_xyzw=tilt_x(90))}

CASES = [
    ("start: bushing 1 mm below the face", {}, 0.0, False),
    ("flush with the face", bush(0.0), 0.0, False),
    ("1 mm below the face", bush(1.0), 0.0, False),
    ("4.2 mm below", bush(4.2), 0.0, False),
    ("4.25 mm below (limit)", bush(4.25), 1.0, True),
    ("5.0 mm below, driver back in its stand", {**bush(5.0), **DRIVER_BACK}, 1.0, True),
    ("5.75 mm below (limit)", bush(5.75), 1.0, True),
    ("5.8 mm below: overdriven", bush(5.8), 0.0, False),
    ("14 mm below: overdriven to the bottom", bush(14.0), 0.0, False),
    ("5 mm, driver left standing in the bore", {**bush(5.0), **DRIVER_IN_BORE}, 0.0, False),
    ("drift pushed home: its end on the face, sleeve untouched", {**bush(1.0), **DRIFT_ON_BORE}, 0.0, False),
    ("5 mm, drift stood on the face over the bore", {**bush(5.0), **DRIFT_ON_BORE}, 1.0, True),
    ("5 mm, driver laid on the face beside the bore", {**bush(5.0), **DRIVER_ON_FACE}, 1.0, True),
    ("5 mm, driver still falling", {**bush(5.0), "driver": DRIVER.state(pos=(SX, -SY, DRIVER.pos[2] + 0.02),
                                                                          lin_vel=(0.0, 0.0, -0.2))}, 0.0, False),
    ("5 mm, driver dropped off the table", {**bush(5.0), "driver": DRIVER.state(pos=(0.2, -0.6, TOP - 0.3))},
     0.0, False),
    ("driver dropped into the bore onto the bushing", {"driver": DRIVER.state(pos=(HX, HY, FACE + PROUD))},
     0.0, False),
]
