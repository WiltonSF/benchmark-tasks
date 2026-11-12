"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A
(the pump starts released on its mat). Each case: (label, {object: state}, expected score, expected success).
Offline, a plunger pushed fully in counts as caught; in the simulator the catch is run by on_step.
"""

import math

from task import BARREL_HALF, BARREL_R, BASE_T, CRADLE_XY, LATCH, MAT_T, PUMP, PUMP_START, STROKE, TOP

CX, CY = CRADLE_XY
IN_X = CX - BARREL_HALF  # barrel middle with its front face on the stop
IN_Z = TOP + BASE_T + BARREL_R


def yaw_q(deg, roll=0.0):
    a, b = math.radians(deg) / 2, math.radians(roll) / 2
    # yaw about z, then roll about the barrel's x
    qz = (0.0, 0.0, math.sin(a), math.cos(a))
    qx = (math.sin(b), 0.0, 0.0, math.cos(b))
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def pump(x=IN_X, y=CY, z=IN_Z, plunger=STROKE, button=0.0, yaw=0.0, vel=None):
    return {"pump": PUMP.state(pos=(x, y, z), quat_xyzw=yaw_q(yaw), lin_vel=vel,
                               joints={"plunger": plunger, "button": button})}


CASES = [
    ("start: released on the mat", {}, 0.0, False),
    ("cocked, on the mat (not in the cradle)", pump(*PUMP_START, TOP + MAT_T + BARREL_R), 0.0, False),
    ("in the cradle, released (plunger out)", pump(plunger=0.0), 0.0, False),
    ("in the cradle, plunger half in", pump(plunger=0.035), 0.0, False),
    ("in the cradle, cocked", pump(), 1.0, True),
    ("in the cradle, plunger at the catch point", pump(plunger=LATCH), 1.0, True),
    ("in the cradle, plunger 1 mm short of the catch", pump(plunger=LATCH - 0.001), 0.0, False),
    ("cocked, front face 10 mm short of the stop (limit)", pump(x=IN_X - 0.010), 1.0, True),
    ("cocked, front face 11 mm short of the stop", pump(x=IN_X - 0.011), 0.0, False),
    ("cocked, 4 mm off the centre line (limit)", pump(y=CY + 0.004), 1.0, True),
    ("cocked, 5 mm off the centre line", pump(y=CY + 0.005), 0.0, False),
    ("cocked, held 3 mm above the channel floor", pump(z=IN_Z + 0.003), 0.0, False),
    ("cocked, resting 2 mm up (limit)", pump(z=IN_Z + 0.002), 1.0, True),
    ("cocked, in the cradle turned 8 degrees (limit)", pump(yaw=8.0), 1.0, True),
    ("cocked, turned 9 degrees", pump(yaw=9.0), 0.0, False),
    ("cocked, nozzle toward the robot", pump(x=IN_X, yaw=180.0), 0.0, False),
    ("cocked, in the cradle, still moving", pump(vel=(0.02, 0.0, 0.0)), 0.0, False),
    ("cocked, on the cradle's rail tops", pump(z=IN_Z + 0.008), 0.0, False),
]
