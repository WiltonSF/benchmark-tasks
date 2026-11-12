"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py). Each case:
(label, {object: spec}, expected score, expected success). Offline there is no robot: every case stands for a state
held for 1 s with both grippers clear. In the simulator the release check fails closed (no credit until both
grippers have been measured more than 5 mm away).
"""

import math
import os

import numpy as np

os.environ["GLUE_GUN_TASK_OFFLINE_CHECK"] = "1"  # the task module credits these offline cases as held 1 s and let go

import task as T
from task import GUN, HEEL_X, MAT_SIZE, MAT_XY, STAND_OPEN, pose

PARK = math.radians(13.3)  # resting on the handle's front edge and the open stand's foot (as in the demo trace)


def parked(stand=STAND_OPEN, lean=PARK, dx=0.0, dy=0.0, dz=0.0, vel=None):
    s = pose(lean, HEEL_X, 0.0, stand=stand, dz=dz)
    return GUN.state(joints={"stand": stand}, pos=np.asarray(s.pos) + (dx, dy, 0.0), quat_xyzw=s.quat_xyzw,
                     lin_vel=vel)


def on_mat(state, dx=0.0, dy=0.0):
    """The same orientation, shifted and dropped so its lowest point rests on the mat."""
    p = np.asarray(state.pos) + (dx, dy, T.MAT_TOP - float(state.low[2]))
    return GUN.state(joints=state.joints, pos=p, quat_xyzw=state.quat_xyzw)


def turned(rx=0.0, ry=0.0, stand=0.0):
    """Turned about the gun's own x (roll) or y (pitch) axis, then set down on the mat."""
    r = T.parts._rot((0, 0, 1), T.GUN_YAW) @ T.parts._rot((0, 1, 0), ry) @ T.parts._rot((1, 0, 0), rx)
    s = GUN.state(joints={"stand": stand}, pos=GUN.pos, quat_xyzw=T.parts._matrix_to_quat(r))
    return on_mat(s)


def feet_x_max(state):
    pts = T._corners(T._boxes(state))
    low = pts[:, 2].min()
    return float(pts[pts[:, 2] <= low + 0.002][:, 0].max())


_P = parked()
_EDGE = MAT_XY[0] + MAT_SIZE[0] / 2 - feet_x_max(_P)  # shift that puts the outermost foot on the mat's far edge

CASES = [
    ("declared pose (upright, stand folded) before it settles", {}, 0.0, False),
    ("start: tipped onto its nozzle, stand folded", {"glue_gun": T.START}, 0.0, False),
    ("parked on its open stand", {"glue_gun": parked()}, 1.0, True),
    ("parked, still rocking", {"glue_gun": parked(vel=(0.0, 0.03, 0.0))}, 0.0, False),
    ("parked, held 5 mm above the mat", {"glue_gun": parked(dz=0.005)}, 0.0, False),
    ("parked with the stand pushed back to 66 degrees, leaning 17.7", {"glue_gun": on_mat(
        pose(math.radians(17.7), HEEL_X, 0.0, stand=math.radians(66.0)))}, 1.0, True),
    ("parked, outermost foot on the mat's edge", {"glue_gun": parked(dx=_EDGE)}, 1.0, True),
    ("parked, outermost foot 1 mm past the mat's edge", {"glue_gun": parked(dx=_EDGE + 0.001)}, 0.0, False),
    ("parked on the bare table beside the mat", {"glue_gun": on_mat(parked(dx=0.25))}, 0.0, False),
    ("stand pressed open, gun still tipped on its nozzle",
     {"glue_gun": pose(T.tipped_lean(), HEEL_X, 0.0, stand=math.radians(48.0))}, 0.0, False),
    ("stood upright on its handle alone, stand folded (cannot stand: tips forward)",
     {"glue_gun": GUN.state()}, 0.0, False),
    ("upright on its handle, stand open but its foot clear of the mat", {"glue_gun": GUN.state(
        joints={"stand": STAND_OPEN})}, 0.0, False),
    ("lying on its side, stand folded", {"glue_gun": turned(rx=math.pi / 2)}, 0.0, False),
    ("lying on its other side, stand open", {"glue_gun": turned(rx=-math.pi / 2, stand=STAND_OPEN)}, 0.0, False),
    ("stood on its tail end, nozzle up", {"glue_gun": turned(ry=-math.pi / 2)}, 0.0, False),
    ("upside down on its housing", {"glue_gun": turned(rx=math.pi)}, 0.0, False),
]
