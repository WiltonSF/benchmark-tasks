"""On the station table, between the arms and a little way out: a C-hook carrier hanging from a high pivot on a
grey stand behind it, like a carrier on an overhead conveyor. The carrier is an open orange basket with a grab tab
sticking out of its front, hung from a thin rod that rises behind the basket and runs forward to the pivot. An open
blue tin stands loose in the middle of the basket floor, taller than the basket walls. On the table to the robot's
left is a flat green pad. When the trial starts the carrier is swinging toward and away from the robot, about 12
degrees each way.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

ORANGE, ROD_C, STAND_C = (0.92, 0.45, 0.10), (0.25, 0.25, 0.27), (0.55, 0.56, 0.58)
TIN_C, PAD_C = (0.20, 0.40, 0.85), (0.15, 0.65, 0.25)

# World frame: x away from the arm bases, y to the robot's left, z up (m). Carrier coordinates are relative to the
# basket's centre on the table (BX, BY, TOP) with the carrier hanging still.
BX, BY = 0.30, 0.0
W = 0.005  # basket wall and floor thickness
INNER = 0.100  # inside width and depth of the basket
OUTER = INNER + 2 * W
FLOOR_Z = 0.025  # underside of the basket floor above the table
FLOOR_TOP = FLOOR_Z + W
WALL_H = 0.035  # walls rise 35 mm above the floor
RIM_Z = FLOOR_TOP + WALL_H
TAB = (0.070, 0.020, 0.010)  # grab tab: 70 mm out from the front wall, 20 mm wide, 10 mm thick
TAB_X = -OUTER / 2 - TAB[0] / 2
TAB_Z = RIM_Z - TAB[2] / 2
ROD_R = 0.004
PIVOT_Z = 0.550  # hinge axis above the table
SPINE_X = OUTER / 2 + 0.060  # the rising rod, 60 mm behind the basket's back wall
BUMPER = (0.020, OUTER, 0.026)  # front bar under the front wall: balances the rod so the basket hangs level
BUMPER_X = -OUTER / 2 + BUMPER[0] / 2

LINK_SHAPES = [
    parts.box((OUTER, OUTER, W), at=(0.0, 0.0, FLOOR_Z + W / 2), color=ORANGE),
    *[parts.box((W, OUTER, WALL_H), at=(s * (INNER / 2 + W / 2), 0.0, FLOOR_TOP + WALL_H / 2), color=ORANGE)
      for s in (-1, 1)],
    *[parts.box((INNER, W, WALL_H), at=(0.0, s * (INNER / 2 + W / 2), FLOOR_TOP + WALL_H / 2), color=ORANGE)
      for s in (-1, 1)],
    parts.box(TAB, at=(TAB_X, 0.0, TAB_Z), color=ORANGE),
    parts.box(BUMPER, at=(BUMPER_X, 0.0, FLOOR_Z + BUMPER[2] / 2 - 0.002), color=ORANGE),
    # Two short rods from the back wall to the spine, at floor and rim height.
    *[parts.cylinder(ROD_R, SPINE_X - OUTER / 2, at=((OUTER / 2 + SPINE_X) / 2, 0.0, z), color=ROD_C, axis="x")
      for z in (FLOOR_TOP + 0.006, RIM_Z - 0.006)],
    parts.cylinder(ROD_R, PIVOT_Z - FLOOR_TOP, at=(SPINE_X, 0.0, (PIVOT_Z + FLOOR_TOP) / 2), color=ROD_C),
]


def _com(shapes):
    """Volume-weighted centre of a link (the simulator spreads a link's mass uniformly over its colliders)."""
    v = np.array([np.prod(2 * s.half) * (math.pi / 4 if s.kind == "cylinder" else 1.0) for s in shapes])
    c = np.array([s.at for s in shapes])
    return (v[:, None] * c).sum(axis=0) / v.sum()


# The top rod runs from the spine forward to the pivot, which sits right above the loaded carrier's centre of mass.
PIVOT_X = 0.0
LINK_SHAPES.append(parts.cylinder(ROD_R, SPINE_X - PIVOT_X + ROD_R, at=((SPINE_X + PIVOT_X) / 2, 0.0, PIVOT_Z),
                                  color=ROD_C, axis="x"))
LINK_SHAPES.append(parts.cylinder(0.008, 0.024, at=(PIVOT_X, 0.0, PIVOT_Z), color=ROD_C, axis="y"))  # pivot boss
LINK_COM = _com(LINK_SHAPES)
LINK_MASS = 0.30
SWING_DAMPING = 0.0035  # N m s/rad

STAND_X = 0.27  # grey post behind the carrier
STAND_SHAPES = [
    parts.box((0.10, 0.10, 0.010), at=(STAND_X, 0.0, 0.005), color=STAND_C),
    parts.box((0.030, 0.030, PIVOT_Z + 0.02), at=(STAND_X, 0.0, (PIVOT_Z + 0.02) / 2), color=STAND_C),
    # Arm from the post forward to the pivot, beside the carrier's top rod, and the axle across to the pivot.
    parts.box((STAND_X - PIVOT_X + 0.015, 0.016, 0.016), at=((STAND_X + PIVOT_X) / 2, 0.026, PIVOT_Z + 0.004),
              color=STAND_C),
    parts.cylinder(0.004, 0.026, at=(PIVOT_X, 0.013, PIVOT_Z), color=STAND_C, axis="y"),
]

CARRIER = parts.Part(
    "carrier", pos=(BX, BY, TOP), shapes=STAND_SHAPES,
    joints=[parts.Hinge("swing", shapes=LINK_SHAPES, pivot=(PIVOT_X, 0.0, PIVOT_Z), axis=(0.0, 1.0, 0.0),
                        lower=-0.8, upper=0.8, mass=LINK_MASS, damping=SWING_DAMPING)],
)

TIN_R, TIN_H, TIN_MASS = 0.015, 0.090, 0.06  # tall and narrow: it tips over at about 0.33 g sideways
TIN = Cylinder("tin", pos=(BX, BY, TOP + FLOOR_TOP + TIN_H / 2), radius=TIN_R, height=TIN_H, mass=TIN_MASS,
               color=TIN_C)
PAD_XY, PAD_R, PAD_H = (0.20, 0.20), 0.040, 0.004
PAD = Cylinder("pad", pos=(*PAD_XY, TOP + PAD_H / 2), radius=PAD_R, height=PAD_H, color=PAD_C)

OBJECTS = [CARRIER, TIN, PAD]

START_SPEED = 0.90  # rad/s through hanging at the start (about 0.2 rad each way)
SUCCESS_HOLD_S = 1.0
STEADY_DEG = 2.0  # "steady": swinging less than 2 deg either side of its mean over the last 1.5 s
WINDOW_S = 1.5
TIP_DEG = 20.0  # the tin counts as tipped past 20 deg from the basket's (or, out of it, the world's) vertical
UPRIGHT_DEG = 10.0
PAD_TOL_Z = 0.002
PAD_CENTRE_TOL = 0.025  # tin centre within 25 mm of the pad centre: its whole foot on the pad
RELEASE_GAP = 0.005  # let go: no part of either gripper within 5 mm of the tin
TABLE_TOUCH = 0.002

# -- whole-trial records, cleared only by a scene reset (on_reset), which the robot cannot cause -------------------
_SWING: list[tuple[float, float]] = []  # (time, swing angle) every control step, while running
_HIST = {"tipped": False, "dropped": False, "out_while_swinging": False}
_GAP = {"tin": math.inf}  # closest any gripper link came to the tin at the last step (m); offline: none present


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _quat_matrix(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _angle(a, b) -> float:
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))


@parts.on_reset
def _start_swing(sim):
    """Clear the records, then set the carrier swinging: it passes through hanging, moving away from the robot
    fast enough to swing about 0.2 rad each way, and the tin moves with it."""
    import torch

    _SWING.clear()
    _GAP["tin"] = math.inf
    for k in _HIST:
        _HIST[k] = False
    base = sim.env.unwrapped
    carrier, tin = base.scene["carrier"], base.scene["tin"]
    ids = carrier.find_joints(["swing"], preserve_order=True)[0]
    dev = base.device
    w = -START_SPEED  # rad/s about +y; negative swings the basket toward +x
    carrier.write_joint_velocity_to_sim_index(velocity=torch.tensor([[w]], device=dev), joint_ids=ids)
    pose = tin.data.root_link_pose_w
    pose = pose.torch if hasattr(pose, "torch") else pose
    origin = base.scene.env_origins[0].cpu().numpy()
    r = pose[0, :3].cpu().numpy().astype(float) - origin - np.array([BX + PIVOT_X, BY, TOP + PIVOT_Z])
    vel = torch.tensor([[w * r[2], 0.0, -w * r[0], 0.0, w, 0.0]], dtype=torch.float32, device=dev)
    tin.write_root_velocity_to_sim_index(root_velocity=vel)


_HAND_LINKS = ("gripper", "tip_left", "tip_right")


def _gripper_gap(sim, tin) -> float:
    """Closest distance (m) from either arm's gripper and finger links, at their real simulated poses (fingers as
    they are), to the tin's box."""
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = base.scene.env_origins[0].cpu().numpy()
    r = _quat_matrix(tin.quat_xyzw)
    size = np.array([2 * TIN_R, 2 * TIN_R, TIN_H])
    centre = np.asarray(tin.pos, float)
    best = math.inf
    for arm in ("left_robot", "right_robot"):
        rob = base.scene[arm]
        poses = rob.data.body_link_pose_w
        poses = (poses.torch if hasattr(poses, "torch") else poses)[0].cpu().numpy().astype(float)
        links = {}
        for i, n in enumerate(rob.body_names):
            if n in _HAND_LINKS:
                t = np.eye(4)
                t[:3, :3] = _quat_matrix(poses[i, 3:])
                t[:3, 3] = poses[i, :3] - origin
                links[n] = t
        d, _ = analysis.hulls_to_box(analysis.world_hulls(links), centre, r, size)
        best = min(best, float(d))
    return best


@parts.on_step
def _record_swing(sim):
    states = sim.states()
    _GAP["tin"] = _gripper_gap(sim, states["tin"])
    q = float(states["carrier"].joints["swing"])
    _SWING.append((sim.t, q))
    while _SWING and _SWING[0][0] < sim.t - WINDOW_S:
        _SWING.pop(0)


def _swing_deg(objects) -> float:
    """Half the swing's peak-to-peak range over the last 1.5 s (deg). Offline (no simulator steps) the current
    angle from hanging stands in for it."""
    if len(_SWING) >= 2:
        qs = [q for _, q in _SWING]
        return math.degrees((max(qs) - min(qs)) / 2)
    return abs(math.degrees(objects["carrier"].joints["swing"]))


def _basket_frame(objects):
    """(origin, rotation) of the carrier link: origin at the pivot, axes as the hanging basket's."""
    p, q = objects["carrier"].links["swing"]
    return np.asarray(p, float), _quat_matrix(q)


def _tin_in_basket(objects) -> bool:
    """The tin's centre is inside the basket's inside wall faces and its base below the rim (in the basket's own
    frame)."""
    o, r = _basket_frame(objects)
    c = r.T @ (np.asarray(objects["tin"].pos, float) - o)  # bounding-box centre is the cylinder's centre
    c += np.array([PIVOT_X, 0.0, PIVOT_Z])  # back to carrier coordinates
    base_z = c[2] - TIN_H / 2  # bottom of the tin along the basket's vertical (exact while upright in it)
    return bool(abs(c[0]) < INNER / 2 and abs(c[1]) < INNER / 2 and base_z < RIM_Z)


def _tin_up(objects):
    return _quat_matrix(objects["tin"].quat_xyzw)[:, 2]


def _record(objects) -> None:
    tin = objects["tin"]
    inside = _tin_in_basket(objects)
    ref = _basket_frame(objects)[1][:, 2] if inside else np.array([0.0, 0.0, 1.0])
    if _angle(_tin_up(objects), ref) > TIP_DEG + 1e-6:
        _HIST["tipped"] = True
    if tin.low[2] <= TOP + TABLE_TOUCH + 1e-6:
        _HIST["dropped"] = True
    lifted = tin.low[2] > TOP + PAD_H + PAD_TOL_Z + 1e-6  # off the table and the pad: in the hand or falling
    if not inside and lifted and _swing_deg(objects) >= STEADY_DEG - 1e-6:
        _HIST["out_while_swinging"] = True


def _on_pad(objects) -> bool:
    tin = objects["tin"]
    d = math.hypot(tin.pos[0] - PAD_XY[0], tin.pos[1] - PAD_XY[1])
    return bool(
        _angle(_tin_up(objects), np.array([0.0, 0.0, 1.0])) <= UPRIGHT_DEG + 1e-6
        and abs(tin.low[2] - (TOP + PAD_H)) <= PAD_TOL_Z + 1e-6
        and d <= PAD_CENTRE_TOL + 1e-6
        and tin.resting()
        and _GAP["tin"] >= RELEASE_GAP - 1e-6
    )


def _failed() -> bool:
    return _HIST["tipped"] or _HIST["dropped"] or _HIST["out_while_swinging"]


def success(objects):
    """The tin stands upright and still on the pad, the carrier is steady, and nothing went wrong on the way."""
    _record(objects)
    return bool(not _failed() and _on_pad(objects) and _swing_deg(objects) < STEADY_DEG - 1e-6)


def score(objects):
    """1.0 as success(); 0.5 if the tin is on the pad that way but the carrier is swinging again; else 0."""
    _record(objects)
    if _failed() or not _on_pad(objects):
        return 0.0
    return 1.0 if _swing_deg(objects) < STEADY_DEG - 1e-6 else 0.5


# -- scripted demo (left arm): fixed waypoints timed from the start, no state read ---------------------------------
TIP_PAST_TCP = 0.013
TRAVEL_Z = TOP + 0.16
FINGER_READY = -0.018  # fingers 18 mm each side of centre: 8 mm clear of the tab (the fingers close slowly)
FINGER_TAB = -(TAB[1] / 2 - 0.003)  # close only 3 mm past the faces, so both fingers meet the part
FINGER_TIN = -(TIN_R - 0.003)
PERIOD = 1.429  # s, measured on the GPU (idle trace: 63 half swings in 45 s)
FIRST_TURN = 1.071  # s, first turning point toward the robot
CATCH_N = 3  # catch at the third turning point toward the robot
CATCH_AMP = 0.2015  # rad, the swing there (it decays from 0.204 to 0.179 rad over 45 s when left alone)


def _tab_at(q):
    """World position of the grab tab's grip point (40 mm out from the front wall) with the carrier at angle q."""
    pivot = np.array([PIVOT_X, 0.0, PIVOT_Z])
    p = np.array([-OUTER / 2 - 0.040, 0.0, TAB_Z]) - pivot
    w = pivot + _rot_y(q) @ p
    return np.array([BX + w[0], BY + w[1], TOP + w[2]])


def demo():
    t_catch = FIRST_TURN + (CATCH_N - 1) * PERIOD  # time of the turning point
    grip_dz = TIP_PAST_TCP - 0.004  # finger tips 4 mm above the tab's underside
    g = _tab_at(CATCH_AMP)
    above = (g[0], g[1], g[2] + grip_dz + 0.070)
    at = (g[0], g[1], g[2] + grip_dz)
    w = []
    t0 = 2.5
    # Wait high, drop to the tab's turning point while the basket is at the far end of its swing, let the tab slide
    # in between the open fingers as it comes back, and close as it comes to a stop.
    w.append(Waypoint(t0, above, FINGER_OPEN, "wait above the tab's turning point"))
    w.append(Waypoint(t_catch - 0.80 - t0, above, FINGER_OPEN, "hold"))
    w.append(Waypoint(0.40, at, FINGER_READY, "drop to the turning point, fingers part closed"))
    w.append(Waypoint(0.25, at, FINGER_READY, "let the tab slide in"))
    w.append(Waypoint(0.30, at, FINGER_TAB, "pinch at the turning point"))
    for k, q in enumerate((0.15, 0.10, 0.05, 0.0)):
        p = _tab_at(q)
        w.append(Waypoint(0.8, (p[0], p[1], p[2] + grip_dz), FINGER_TAB, f"ease it down {k}"))
    hang = _tab_at(0.0)
    w.append(Waypoint(1.5, (hang[0], hang[1], hang[2] + grip_dz), FINGER_TAB, "hold still"))
    w.append(Waypoint(0.6, (hang[0], hang[1], hang[2] + grip_dz), FINGER_OPEN, "let go"))
    w.append(Waypoint(0.8, (hang[0], hang[1], hang[2] + grip_dz + 0.04), FINGER_OPEN, "back off up"))
    # Take the tin: grip its top 30 mm from above, lift it clear of the rim, carry it to the pad, set it down 6 mm up.
    tin_top = TOP + FLOOR_TOP + TIN_H
    gz = tin_top - 0.030 + TIP_PAST_TCP
    lift = gz + (RIM_Z + TOP + 0.010) - (TOP + FLOOR_TOP)
    place = TOP + PAD_H + TIN_H - 0.030 + TIP_PAST_TCP + 0.006
    w += [
        Waypoint(1.2, (BX, BY, gz + 0.03), FINGER_OPEN, "over the tin"),
        Waypoint(1.0, (BX, BY, gz), FINGER_OPEN, "down to the tin"),
        Waypoint(0.6, (BX, BY, gz), FINGER_TIN, "grip"),
        Waypoint(1.5, (BX, BY, lift), FINGER_TIN, "lift straight out"),
        Waypoint(1.8, (*PAD_XY, lift), FINGER_TIN, "carry"),
        Waypoint(1.2, (*PAD_XY, place), FINGER_TIN, "lower"),
        Waypoint(0.6, (*PAD_XY, place), FINGER_OPEN, "release"),
        Waypoint(1.0, (*PAD_XY, TRAVEL_Z), FINGER_OPEN, "up"),
    ]
    return {"left": w}
