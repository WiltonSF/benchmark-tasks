"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state}, expected score, expected success). States come from each part's own
kinematics (Part.state), as the simulator reports them; the frame's tube is on its 17-degree stop unless stated.
"""

import math

import numpy as np

from task import BAND_TOP, FRAME, LEAN, MOUTH, PIVOT, POST, STAND_XY, TOP, U

ON_STOP = FRAME.state(joints={"tube": LEAN})
MOVING = (0.0, 0.0, -0.05)


def q(axis, deg):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    s = math.sin(math.radians(deg) / 2)
    return (*(a * s), math.cos(math.radians(deg) / 2))


ALIGNED = q((-1, 0, 0), math.degrees(LEAN))


def post_in(depth, quat=ALIGNED, side=0.0, vel=None):
    """The post with its bottom end ``depth`` below the mouth along the tube axis, shifted ``side`` along x."""
    bottom = MOUTH - U * depth + np.array([side, 0.0, 0.0])
    return {"frame": ON_STOP, "seatpost": POST.state(pos=bottom, quat_xyzw=quat, lin_vel=vel)}


CASES = [
    ("on the bottom stop, in line", post_in(0.090), 1.0, True),
    ("band's top edge exactly at the mouth", post_in(BAND_TOP), 1.0, True),
    ("1 mm short: band's top edge showing", post_in(BAND_TOP - 0.001), 0.0, False),
    ("band centre at the mouth, its top half showing", post_in(BAND_TOP - 0.003), 0.0, False),
    ("jammed upright 13 mm into the mouth", post_in(0.013, quat=(0, 0, 0, 1)), 0.0, False),
    ("upright, as deep as the bore allows (40 mm)", post_in(0.040, quat=(0, 0, 0, 1)), 0.0, False),
    ("in at 80 mm, 3 degrees off the tube's axis", post_in(0.080, quat=q((-1, 0, 0), 20.0)), 1.0, True),
    ("in at 80 mm, 4 degrees off the tube's axis", post_in(0.080, quat=q((-1, 0, 0), 21.0)), 0.0, False),
    ("in at full depth but still sliding", post_in(0.090, vel=MOVING), 0.0, False),
    ("post 4 mm off the axis sideways (against the wall)", post_in(0.085, side=0.002), 1.0, True),
    ("post beside the tube at full depth (not in the bore)", post_in(0.085, side=0.030), 0.0, False),
    ("still upright in its stand", {}, 0.0, False),
    ("lying on the table", {"seatpost": POST.state(pos=(0.20, 0.15, TOP + 0.014), quat_xyzw=q((0, 1, 0), 90))},
     0.0, False),
    ("tube pushed upright off its stop, post dropped straight in to the bottom",
     {"frame": FRAME.state(joints={"tube": 0.0}),
      "seatpost": POST.state(pos=PIVOT + np.array([0, 0, 0.005]), quat_xyzw=(0, 0, 0, 1))}, 0.0, False),
    ("tube 1 degree off its stop, post seated in it",
     {"frame": FRAME.state(joints={"tube": LEAN - math.radians(1.0)}),
      "seatpost": POST.state(pos=PIVOT + np.array([0, math.sin(LEAN - math.radians(1.0)),
                                                   math.cos(LEAN - math.radians(1.0))]) * 0.005,
                             quat_xyzw=q((-1, 0, 0), math.degrees(LEAN) - 1.0))}, 1.0, True),
    ("post in the stand upside down", {"seatpost": POST.state(pos=(*STAND_XY, TOP + 0.152), quat_xyzw=(1, 0, 0, 0))},
     0.0, False),
]
