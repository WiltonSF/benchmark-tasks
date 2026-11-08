"""On the station table, between the arms: a small wooden chest of drawers standing with its front facing away
from the robot. Its upper drawer is shut; the lower drawer has been taken out, so the space under the upper drawer's
runners is open. The upper drawer has a bar handle on its front. Nothing else is on the table.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Chest frame: s is distance out of the chest along the drawer's pull direction (0 at the chest's front face,
# negative inside), v runs across the opening, z is up from the table. The chest faces away from the arms:
# s points along world +x and v along world +y.
XF = 0.25  # world x of the chest's front face
CY = -0.12  # world y of the opening's centre line


def W(s: float, v: float, z: float) -> tuple[float, float, float]:
    """Chest frame to world."""
    return (XF + s, CY + v, TOP + z)


def wsize(ds: float, dv: float, dz: float) -> tuple[float, float, float]:
    return (ds, dv, dz)


WOOD, DARK_WOOD, RUNNER_C = (0.62, 0.45, 0.28), (0.40, 0.27, 0.15), (0.50, 0.36, 0.22)
DRAWER_C, HANDLE_C, TAB_C = (0.80, 0.66, 0.46), (0.15, 0.15, 0.16), (0.85, 0.20, 0.15)

DEPTH, WIDTH, HEIGHT = 0.140, 0.160, 0.070  # the drawer: deep (s), wide (v), tall (z)
PLY = 0.006  # drawer wall and bottom thickness
RZ = 0.040  # runner tops: the drawer's bottom when it is in the chest
INNER = 0.085  # half the chest's inside width
LEDGE = 0.015  # runner ledge width
SIDE = 0.012  # chest side, roof and back thickness
CHEST_D = 0.160  # inside depth, face to back panel
RAIL_D = 0.008  # front rail depth (s)
RAIL_Z = RZ + HEIGHT + 0.012  # rail underside: 12 mm over the drawer's top
TAB_H = 0.016  # stop tab: 16 mm above the drawer's back top edge
TAB_W = 0.080
ROOF_Z = RZ + HEIGHT + TAB_H + 0.015  # roof underside
HANDLE = (0.012, 0.060, 0.020)  # bar handle: thick (s), long (v), tall (z)
STEM = 0.045  # handle stands 45 mm off the drawer front
HANDLE_S = STEM + HANDLE[0] / 2  # bar centre ahead of the drawer front
HANDLE_Z = 0.045  # bar centre above the drawer's bottom
DRAWER_MASS = 0.25


def _cbox(name, s0, s1, v0, v1, z0, z1, color):
    return Box(name, pos=W((s0 + s1) / 2, (v0 + v1) / 2, (z0 + z1) / 2), size=wsize(s1 - s0, v1 - v0, z1 - z0),
               color=color)


CHEST = [
    _cbox("chest_side_a", -CHEST_D - SIDE, 0.0, -INNER - SIDE, -INNER, 0.0, ROOF_Z + SIDE, WOOD),
    _cbox("chest_side_b", -CHEST_D - SIDE, 0.0, INNER, INNER + SIDE, 0.0, ROOF_Z + SIDE, WOOD),
    _cbox("chest_back", -CHEST_D - SIDE, -CHEST_D, -INNER, INNER, 0.0, ROOF_Z + SIDE, WOOD),
    _cbox("chest_top", -CHEST_D, 0.0, -INNER, INNER, ROOF_Z, ROOF_Z + SIDE, WOOD),
    _cbox("front_rail", -RAIL_D, 0.0, -INNER, INNER, RAIL_Z, ROOF_Z, DARK_WOOD),
    _cbox("runner_a", -CHEST_D, 0.0, -INNER, -INNER + LEDGE, RZ - 0.010, RZ, RUNNER_C),
    _cbox("runner_b", -CHEST_D, 0.0, INNER - LEDGE, INNER, RZ - 0.010, RZ, RUNNER_C),
]


def _dbox(s0, s1, v0, v1, z0, z1, color):
    """A drawer shape in the drawer's part frame: origin at its bottom centre, same axes as the world."""
    return parts.box((s1 - s0, v1 - v0, z1 - z0), at=((s0 + s1) / 2, (v0 + v1) / 2, (z0 + z1) / 2), color=color)


_HD, _HW = DEPTH / 2, WIDTH / 2
DRAWER_SHAPES = [
    _dbox(-_HD, _HD, -_HW, _HW, 0.0, PLY, DRAWER_C),  # bottom
    _dbox(-_HD, _HD, -_HW, -_HW + PLY, PLY, HEIGHT, DRAWER_C),  # sides
    _dbox(-_HD, _HD, _HW - PLY, _HW, PLY, HEIGHT, DRAWER_C),
    _dbox(_HD - PLY, _HD, -_HW + PLY, _HW - PLY, PLY, HEIGHT, DRAWER_C),  # front
    _dbox(-_HD, -_HD + PLY, -_HW + PLY, _HW - PLY, PLY, HEIGHT, DRAWER_C),  # back
    _dbox(-_HD, -_HD + PLY, -TAB_W / 2, TAB_W / 2, HEIGHT, HEIGHT + TAB_H, TAB_C),  # stop tab
    _dbox(_HD + STEM, _HD + STEM + HANDLE[0], -HANDLE[1] / 2, HANDLE[1] / 2,
          HANDLE_Z - HANDLE[2] / 2, HANDLE_Z + HANDLE[2] / 2, HANDLE_C),  # bar
    *[_dbox(_HD, _HD + STEM, sv * 0.022 - 0.004, sv * 0.022 + 0.004, HANDLE_Z - 0.006, HANDLE_Z + 0.006, HANDLE_C)
      for sv in (-1, 1)],  # stems
]
START_S = -DEPTH / 2  # drawer centre (s) when shut: its front flush with the chest's face
DRAWER = parts.Part("drawer", pos=W(START_S, 0.0, RZ), mass=DRAWER_MASS, shapes=DRAWER_SHAPES)
OBJECTS = [*CHEST, DRAWER]

STOP_PULL = DEPTH - RAIL_D - PLY  # how far it pulls out before the tab meets the rail (126 mm)

EPS = 1e-6
UPRIGHT_DEG = 5.0
ON_TABLE = 0.001
CLEAR = 0.005


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# Released: an on_step callback measures, every step, the distance from both arms' finger and gripper hulls to the
# drawer's shapes. Offline (near_misses.py) there is no robot, so "clear" stays True.
_grip = {"clear": True}


@parts.on_reset
def _grip_reset(sim):
    _grip["clear"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    d = sim.states()["drawer"]
    boxes = DRAWER.world_boxes({"base": (np.asarray(d.pos, float), _rot(d.quat_xyzw))})
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right", "gripper"):
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] >= CLEAR - EPS for _, c, r, s in boxes)


# The chest's footprint on the table (world x, y): from its back panel to its front, across both sides.
FOOT_X = (XF - CHEST_D - SIDE, XF)
FOOT_Y = (CY - INNER - SIDE, CY + INNER + SIDE)


def _out(st) -> bool:
    """Clear of the chest: the drawer's box does not overlap the chest's footprint seen from above."""
    lo, hi = st.low, st.high
    overlap = (float(hi[0]) > FOOT_X[0] + EPS and float(lo[0]) < FOOT_X[1] - EPS
               and float(hi[1]) > FOOT_Y[0] + EPS and float(lo[1]) < FOOT_Y[1] - EPS)
    return not overlap


def success(objects):
    """The drawer stands upright on the table clear of the chest's footprint, let go and still."""
    d = objects["drawer"]
    return bool(
        _out(d)
        and tilt.tilt_of(d.quat_xyzw) <= UPRIGHT_DEG + EPS
        and abs(float(d.low[2]) - TOP) <= ON_TABLE + EPS
        and d.resting()
        and _grip["clear"]
    )


def distance(objects):
    return float(max(0.0, XF - float(objects["drawer"].low[0])))


def score(objects):
    """Pass/fail: 1.0 as success(), else 0."""
    return 1.0 if success(objects) else 0.0


# -- scripted demo: fixed waypoints, no state read ---------------------------------------------------------------
# The right arm pinches the bar handle across its 12 mm thickness (the pads close along s, across the lean axis, so
# the lean cannot turn the bar in the pads), pulls the drawer to just short of its stop, leans it front-up while
# pulling on so the tab passes under the rail, draws it clear, levels it and sets it on the table.
TIP = 0.013
GRIP_DZ = -HANDLE[2] / 2 + 0.002 + TIP  # TCP above the bar centre: pad tips 2 mm above the bar's underside
FINGER_BAR = -(HANDLE[0] / 2 - 0.003)  # close 3 mm past the bar's faces
FINGER_NEAR = -(HANDLE[0] / 2 + 0.007)  # open 7 mm clear of each face
LEAN = 20.0  # degrees, front up while the tab passes under the rail
LEAN2 = 30.0  # then further, once the tab is under
YAW = math.pi / 2  # fingers close along world x, across the bar's thickness
HEADING = 0.0  # the fingertips swing toward the pull direction (world +x, away from the arms)
OVER_STOP = 0.006  # the straight pull is commanded this far past the stop, so the soft arm brings the tab to the rail
DROP = 0.007  # release height above the table


def _handle_tcp(pivot_s: float, pivot_z: float, deg: float) -> tuple[float, float]:
    """TCP (s, z) with the drawer's back-bottom edge at (pivot_s, pivot_z) and the drawer leaned front-up deg."""
    a = math.radians(deg)
    ds, dz = DEPTH + HANDLE_S, HANDLE_Z + GRIP_DZ
    return pivot_s + ds * math.cos(a) - dz * math.sin(a), pivot_z + ds * math.sin(a) + dz * math.cos(a)


def _wp(t, s, z, finger, label, deg=0.0):
    pos = W(s, 0.0, z)
    if deg == 0.0:
        return Waypoint(t, pos, finger, label, yaw=YAW)
    return tilt.Waypoint(t, pos, finger, label, yaw=YAW, tilt=deg, heading=HEADING)


def demo():
    s0, z0 = _handle_tcp(-DEPTH, RZ, 0.0)
    pull = STOP_PULL - 0.002
    w = [
        _wp(1.5, s0 + 0.01, z0 + 0.04, FINGER_NEAR, "over the handle"),
        _wp(1.2, s0, z0, FINGER_NEAR, "down round the bar"),
        _wp(0.8, s0, z0, FINGER_BAR, "pinch the bar"),
        _wp(3.0, s0 + STOP_PULL + OVER_STOP, z0, FINGER_BAR, "pull out to the stop"),
        _wp(0.5, s0 + pull, z0, FINGER_BAR, "ease off the stop"),
    ]
    back = -DEPTH + pull  # the drawer's back-bottom edge, on the runners
    for deg in (10.0, LEAN):
        s, z = _handle_tcp(back, RZ, deg)
        w.append(_wp(1.2, s, z, FINGER_BAR, f"lean front up {deg:.0f} deg", deg))
    for ds, deg in ((0.012, LEAN), (0.025, LEAN2), (0.05, LEAN2), (0.09, LEAN2)):  # draw it out, the tab under the rail
        s, z = _handle_tcp(back + ds, RZ, deg)
        w.append(_wp(1.2, s, z, FINGER_BAR, "draw it out under the rail", deg))
    s_end = _handle_tcp(0.015, 0.0, 0.0)[0]  # level, with the back 15 mm clear of the face
    w += [
        _wp(2.0, s_end, z0 - 0.015, FINGER_BAR, "level it", 0.0),
        _wp(1.5, s_end, GRIP_DZ + HANDLE_Z + DROP, FINGER_BAR, "lower it to the table"),
        _wp(0.6, s_end, GRIP_DZ + HANDLE_Z + DROP, FINGER_NEAR, "let go"),
        _wp(1.0, s_end - 0.01, GRIP_DZ + HANDLE_Z + 0.035, FINGER_NEAR, "up and away"),
    ]
    return {"right": w}
