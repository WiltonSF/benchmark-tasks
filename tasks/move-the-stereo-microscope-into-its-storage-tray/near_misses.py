"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A.
Each case: (label, {object: spec}, expected score, expected success); a spec is a ready state from a part's own
geometry. Offline there is no robot: every case stands for a state held for 1 s with both grippers clear. In the
simulator the release check fails closed. Cases run in order and share the whole-trial record, so a fault stays until
a scene reset (an empty case after the microscope was moved).
"""

import math
import os

import numpy as np

os.environ["STEREO_MICROSCOPE_TASK_OFFLINE_CHECK"] = "1"

import task as T  # noqa: E402
from task import EYE_SEAT_Z, EYES, EYEPIECES, FLOOR_TOP, SCOPE, TRAY_IN, TRAY_XY, YAW_A  # noqa: E402

SPEC = {n: e for n, e in zip(EYES, EYEPIECES)}


def q_of(r):
    return tuple(float(v) for v in T.parts._matrix_to_quat(r))


def scene(xy, z, yaw=YAW_A, tilt_deg=0.0, rise=None, side=None, vel=None, eye_vel=None, eye_tilt=False):
    """The microscope with its base underside centre at (xy, z), turned `yaw`, tipped `tilt_deg` about its own y
    axis, and both eyepieces where they sit in their tubes plus `rise` (name -> m up the tube) and `side` (name -> m
    sideways)."""
    r = T._rz(yaw) @ T.parts._rot((0.0, 1.0, 0.0), math.radians(tilt_deg))
    p = np.array([xy[0], xy[1], z])
    out = {"microscope": SCOPE.state(pos=tuple(p), quat_xyzw=q_of(r), lin_vel=vel)}
    for n in EYES:
        ex, ey = T.EYE_LOCAL[n]
        local = np.array([ex + (side or {}).get(n, 0.0), ey, EYE_SEAT_Z + (rise or {}).get(n, 0.0)])
        out[n] = SPEC[n].state(pos=tuple(p + r @ local), quat_xyzw=q_of(r), lin_vel=eye_vel)
    return out


def in_tray(dx=0.0, dy=0.0, dz=0.0, **kw):
    return scene((TRAY_XY[0] + dx, TRAY_XY[1] + dy), FLOOR_TOP + dz, **kw)


def carried(z=0.03, **kw):
    return scene((0.21, 0.12), T.TOP + z, vel=(0.0, -0.1, 0.0), eye_vel=(0.0, -0.1, 0.0), **kw)


def tipped_in_tray(deg):
    """Tipped about its own y axis with one base edge on the floor."""
    lift = T.BASE[0] / 2 * math.sin(math.radians(deg))
    return in_tray(dz=lift, tilt_deg=deg)


# The base is 135 mm along world y and 100 mm along world x in lot A; the tray inside is 150 x 175 mm.
EDGE_Y = (TRAY_IN[1] - T.BASE[0]) / 2  # 20 mm of room each side along y
EDGE_X = (TRAY_IN[0] - T.BASE[1]) / 2  # 25 mm along x
SCENE_RESET = {}
E1 = EYES[0]
SIDE_XY = (0.21, 0.175)  # on the table between the start spot and the tray
BESIDE = scene(SIDE_XY, T.TOP)

CASES = [
    ("in the tray, centred, eyepieces seated", in_tray(), 1.0, True),
    ("1 mm above the floor", in_tray(dz=0.001), 1.0, True),
    ("held 3 mm above the floor", in_tray(dz=0.003), 0.0, False),
    ("still moving in the tray", in_tray(vel=(0.0, 0.0, -0.05)), 0.0, False),
    ("base touching the rim's inside along y", in_tray(dy=EDGE_Y), 1.0, True),
    ("base 1 mm over the rim along y", in_tray(dy=EDGE_Y + 0.001), 0.0, False),
    ("base touching the rim's inside along x", in_tray(dx=-EDGE_X), 1.0, True),
    ("base 1 mm over the rim along x", in_tray(dx=-EDGE_X - 0.001), 0.0, False),
    ("turned 180 degrees in the tray", in_tray(yaw=YAW_A + math.pi), 1.0, True),
    ("turned 90 degrees: base 135 mm across a 150 mm inside", in_tray(yaw=0.0), 1.0, True),
    ("tilted exactly 3 degrees", tipped_in_tray(3.0), 1.0, True),
    ("tilted 4 degrees", tipped_in_tray(4.0), 0.0, False),
    ("an eyepiece 1 mm up at the end", in_tray(rise={E1: 0.001}), 1.0, True),
    ("an eyepiece 2 mm up at the end", in_tray(rise={E1: 0.002}), 0.0, False),
    ("an eyepiece still rattling", in_tray(eye_vel=(0.0, 0.03, 0.0)), 0.0, False),
    ("carried with an eyepiece 5 mm up exactly", carried(rise={E1: 0.005}), 0.0, False),
    ("then set in the tray: 1.0", in_tray(), 1.0, True),
    ("carried tilted exactly 10 degrees", carried(tilt_deg=10.0), 0.0, False),
    ("then set in the tray: still 1.0", in_tray(), 1.0, True),
    ("left on the table at its start", {}, 0.0, False),
    ("pushed against the tray's outside", scene((TRAY_XY[0], TRAY_XY[1] + 0.16), T.TOP), 0.0, False),
    ("base resting across the rim", scene((TRAY_XY[0], TRAY_XY[1] + 0.07), T.TOP + 0.016), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    # The 0.5 tier: lifted clear with the eyepieces seated, then set down upright elsewhere and let go.
    ("pushed along the table beside the tray, never lifted", BESIDE, 0.0, False),
    ("lifted 16 mm, eyepieces in, still moving", carried(z=0.016), 0.0, False),
    ("then set down upright on the table beside the tray: 0.5", BESIDE, 0.5, False),
    ("then held still 3 mm above the table", scene(SIDE_XY, T.TOP + 0.003), 0.0, False),
    ("then on the table tilted 4 degrees", scene(SIDE_XY, T.TOP + T.BASE[0] / 2 * math.sin(math.radians(4.0)),
                                                 tilt_deg=4.0), 0.0, False),
    ("then on the table with an eyepiece 2 mm up", scene(SIDE_XY, T.TOP, rise={E1: 0.002}), 0.0, False),
    ("then wholly in the tray: 1.0", in_tray(), 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("lifted exactly 15 mm", carried(z=0.015), 0.0, False),
    ("then set down on the table: 0 (not over 15 mm)", BESIDE, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("pushed up onto the 12 mm rim, flat", scene((TRAY_XY[0], TRAY_XY[1] + 0.13), T.TOP + 0.012), 0.0, False),
    ("then pushed back down onto the table: 0 (not a lift)", BESIDE, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("lifted 30 mm with an eyepiece 2 mm up", carried(rise={E1: 0.002}), 0.0, False),
    ("then set down on the table, eyepiece pushed back: 0", BESIDE, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("lifted 30 mm, eyepieces in", carried(), 0.0, False),
    ("then an eyepiece knocked 6 mm up", carried(rise={E1: 0.006}), 0.0, False),
    ("then set down on the table, eyepiece pushed back: still 0", BESIDE, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("lifted by an eyepiece: it slides 20 mm out, the microscope stays",
     scene((0.21, 0.23), T.TOP, rise={E1: 0.020}), 0.0, False),
    ("then pushed back in and the microscope carried to the tray: still 0", in_tray(), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("after the reset, in the tray: 1.0", in_tray(), 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("an eyepiece 5.5 mm up while carried", carried(rise={E1: 0.0055}), 0.0, False),
    ("then in the tray: still 0", in_tray(), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("an eyepiece knocked sideways out of its tube", carried(side={E1: 0.004}), 0.0, False),
    ("then in the tray: still 0", in_tray(), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("tipped 11 degrees while carried", carried(tilt_deg=11.0), 0.0, False),
    ("then in the tray: still 0", in_tray(), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("dropped: falling at 0.7 m/s", carried(z=0.02).__or__({"microscope": SCOPE.state(
        pos=(0.21, 0.12, T.TOP + 0.02), quat_xyzw=q_of(T._rz(YAW_A)), lin_vel=(0.0, 0.0, -0.7))}), 0.0, False),
    ("then in the tray: still 0", in_tray(), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("both eyepieces lifted out and set in the tray beside it",
     {**in_tray(), **{n: SPEC[n].state(pos=(TRAY_XY[0] - 0.06, TRAY_XY[1] + s * 0.07, FLOOR_TOP))
                      for n, s in zip(EYES, (-1, 1))}}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
]
