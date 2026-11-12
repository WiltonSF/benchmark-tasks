"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py). Each case:
(label, {object: state}, expected score, expected success). Offline there is no robot: every case stands for a state
held for 1 s with both grippers clear. In the simulator the release check fails closed (no credit until both
grippers have been measured more than 5 mm away and both hinges are still).
"""

import math

import numpy as np

import parts  # noqa: E402

parts.OFFLINE_CASES = True  # the task credits these offline cases as held 1 s and let go (never in the simulator)
import task as T  # noqa: E402
from task import CAMERA, TOP  # noqa: E402

D = math.radians


def cam(flip, swivel, pos=None, tilt=0.0, vel=None):
    q = None
    p = None if pos is None else np.asarray(pos, float)
    if tilt:
        q = T.parts._matrix_to_quat(T.parts._rot((1.0, 0.0, 0.0), tilt))
        st = CAMERA.state(joints={"flip": flip, "swivel": swivel}, quat_xyzw=q)
        p = np.asarray(CAMERA.pos, float) + (0.0, 0.0, TOP - float(st.low[2]))
    return {"camera": CAMERA.state(joints={"flip": flip, "swivel": swivel}, pos=p, quat_xyzw=q, lin_vel=vel)}


LIM = T.SWIVEL_MAX - T.GLASS_IN
CASES = [
    ("declared pose: folded in the shooting position, glass out", {}, 0.0, False),
    ("start (both lots): swung out 175 degrees, flat, glass up", cam(D(175), D(90)), 0.0, False),
    ("turned the wrong way on to the -90 stop: flat, glass down", cam(D(175), D(-90)), 0.0, False),
    ("folded flush, glass in", cam(0.0, T.SWIVEL_MAX), 1.0, True),
    ("folded 3 degrees off shut, glass in", cam(T.FOLDED, T.SWIVEL_MAX), 1.0, True),
    ("folded 3.1 degrees off shut, glass in: turned only", cam(T.FOLDED + D(0.1), T.SWIVEL_MAX), 0.5, False),
    ("folded flush, swivel at 172 degrees", cam(0.0, LIM), 1.0, True),
    ("folded flush, swivel at 171.9 degrees", cam(0.0, LIM - D(0.1)), 0.0, False),
    ("folded flush, glass out (folded straight back)", cam(0.0, 0.0), 0.0, False),
    ("folded flush, glass out, swivel 8 degrees off", cam(0.0, D(8.0)), 0.0, False),
    ("swung out at its stop, turned glass in", cam(D(175), T.SWIVEL_MAX), 0.5, False),
    ("swung out 90 degrees, turned glass in", cam(D(90), T.SWIVEL_MAX), 0.5, False),
    ("swung out, turned upright the wrong way (glass facing forward)", cam(D(175), 0.0), 0.0, False),
    ("swung out, turned 150 degrees (not yet square)", cam(D(175), D(150)), 0.0, False),
    ("swung out 90 degrees, flat", cam(D(90), D(90)), 0.0, False),
    ("folded flush, glass in, camera still sliding", cam(0.0, T.SWIVEL_MAX, vel=(0.03, 0.0, 0.0)), 0.0, False),
    ("folded flush, glass in, camera lifted 5 mm", cam(0.0, T.SWIVEL_MAX, pos=np.asarray(CAMERA.pos) + (0, 0, 0.005)),
     0.0, False),
    ("folded flush, glass in, camera tipped 10 degrees", cam(0.0, T.SWIVEL_MAX, tilt=D(10)), 0.0, False),
    ("folded flush, glass in, camera knocked onto its side", cam(0.0, T.SWIVEL_MAX, tilt=D(90)), 0.0, False),
    ("turned glass in, camera on its side", cam(D(175), T.SWIVEL_MAX, tilt=D(90)), 0.0, False),
]
