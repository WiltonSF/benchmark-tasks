"""Near-successes and unintended strategies, scored offline by the factory checker, in lot A (the front-left pin
one level low). Each case: (label, {object: ready state}, expected score, expected success). Every pin is judged by
where it ends: moving one of the three correct pins during the trial costs nothing if it ends seated at level 2."""

import math

from task import PIN_PARTS, SHELF, SUPPORT, TOP, pin_pos

FL = PIN_PARTS["pin_fl"]


def pin(name, level, dx=0.0, dy=0.0, dz=0.0, quat=None):
    x, y, z = pin_pos(name, level)
    return {name: PIN_PARTS[name].state(pos=(x + dx, y + dy, z + dz), quat_xyzw=quat)}


def shelf(dz=0.0, tilt_deg=0.0, dx=0.0):
    a = math.radians(tilt_deg) / 2
    return {"shelf": SHELF.state(pos=(SHELF.pos[0] + dx, 0.0, TOP + SUPPORT + dz),
                                 quat_xyzw=(math.sin(a), 0.0, 0.0, math.cos(a)))}


FIXED = {**pin("pin_fl", 1), **shelf()}
RESET = pin("pin_fl", 0)

CASES = [
    ("fixed: front-left pin at level 2, shelf flat on four pins", FIXED, 1.0, True),
    ("shelf 4 mm high (resting on a drooped pin)", {**pin("pin_fl", 1), **shelf(dz=0.004)}, 1.0, True),
    ("shelf 5 mm high (held above the pins)", {**pin("pin_fl", 1), **shelf(dz=0.005)}, 0.5, False),
    ("pins fixed, shelf still swinging on them", {**pin("pin_fl", 1),
                                                   "shelf": SHELF.state(pos=(SHELF.pos[0], 0.0, TOP + SUPPORT + 0.02),
                                                                        lin_vel=(0.0, 0.0, 0.05))}, 0.0, False),
    ("shelf tilted 2 degrees", {**pin("pin_fl", 1), **shelf(tilt_deg=2.0)}, 1.0, True),
    ("shelf tilted 3 degrees", {**pin("pin_fl", 1), **shelf(tilt_deg=3.0)}, 0.5, False),
    ("shelf slid 16 mm forward", {**pin("pin_fl", 1), **shelf(dx=-0.016)}, 0.5, False),
    ("pin pushed home but collar 5 mm proud", {**pin("pin_fl", 1, dy=0.005), **shelf()}, 0.0, False),
    ("pin collar 4 mm proud", {**pin("pin_fl", 1, dy=0.0035), **shelf()}, 1.0, True),
    ("pin at level 3 instead", {**pin("pin_fl", 2), **shelf()}, 0.0, False),
    ("pin drawn out, held in the air", {**pin("pin_fl", 1, dy=0.09), **shelf(dz=0.03, tilt_deg=8)}, 0.0, False),
    ("pin fixed, shelf dropped on the table", {**pin("pin_fl", 1),
                                               "shelf": SHELF.state(pos=(0.30, 0.0, TOP))}, 0.0, False),
    ("start layout again", {**RESET, **shelf(tilt_deg=8)}, 0.0, False),
    # the three correct pins are judged by their final seating, with no whole-trial movement gate
    ("back-right pin pulled 6 mm along its hole and left there", {**FIXED, **pin("pin_br", 1, dy=-0.006)}, 0.0, False),
    ("then pushed back home, all pins and shelf correct: 1.0", FIXED, 1.0, True),
    ("back-right pin nudged 3.5 mm out along its hole and left (collar 4.0 mm off the face)",
     {**FIXED, **pin("pin_br", 1, dy=-0.0035)}, 1.0, True),
    ("back-left pin moved down a level and left there", {**FIXED, **pin("pin_bl", 0)}, 0.0, False),
    ("then moved back to level 2, shelf flat: 1.0", FIXED, 1.0, True),
    ("back-right pin lifted 5 mm in its hole, then fixed", {**FIXED, **pin("pin_br", 1, dz=0.005)}, 1.0, True),
    ("back-right pin axis 7 mm above its hole's centre", {**FIXED, **pin("pin_br", 1, dz=0.009)}, 0.0, False),
    ("front-right pin moved 5 mm sideways in its hole", {**FIXED, **pin("pin_fr", 1, dx=0.005)}, 0.0, False),
]
