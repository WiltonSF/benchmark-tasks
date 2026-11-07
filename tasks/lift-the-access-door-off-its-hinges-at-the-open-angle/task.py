"""On the station table, ahead of the right arm: a small grey sheet-metal unit cabinet with a cream service door hung
on two lift-off hinge pins (dark) at its right-hand edge. The door is unlatched and stands part open toward the robot.
Its upper hinge sleeve has a small tab, and a grey keeper plate sits just above the upper hinge. A short dark post on
the table is the door stop. To the right of the unit, a yellow door rack: two low walls with a narrow slot between
them.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

STEEL, PIN_STEEL, DOOR_CREAM, RACK_YELLOW = (0.55, 0.57, 0.60), (0.12, 0.12, 0.14), (0.90, 0.84, 0.66), (0.92, 0.75, 0.10)

# -- the hinge -------------------------------------------------------------------------------------------------------
HX, HY = 0.36, -0.22  # the hinge axis (both pins), world
B = TOP + 0.012  # the door's bottom edge as hung: it rests on the lower collar
PIN_R, PIN_H = 0.005, 0.025
COLLAR_R = 0.011
UPPER_LEAF_Z = (B + 0.055, B + 0.062)  # the upper hinge leaf, from the cabinet to the pin
LOWER_PIN_Z = (B, B + PIN_H)
UPPER_PIN_Z = (UPPER_LEAF_Z[1], UPPER_LEAF_Z[1] + PIN_H)
LEAF_X = (-0.006, 0.034)  # the leaf, relative to the axis (toward the cabinet)

COLLAR = Cylinder("lower_collar", pos=(HX, HY, (TOP + B) / 2), radius=COLLAR_R, height=B - TOP, color=STEEL)
LOWER_PIN = Cylinder("lower_pin", pos=(HX, HY, sum(LOWER_PIN_Z) / 2), radius=PIN_R, height=PIN_H, color=PIN_STEEL)
UPPER_PIN = Cylinder("upper_pin", pos=(HX, HY, sum(UPPER_PIN_Z) / 2), radius=PIN_R, height=PIN_H, color=PIN_STEEL)
UPPER_LEAF = Box("upper_leaf", pos=(HX + sum(LEAF_X) / 2, HY, sum(UPPER_LEAF_Z) / 2),
                 size=(LEAF_X[1] - LEAF_X[0], 0.012, UPPER_LEAF_Z[1] - UPPER_LEAF_Z[0]), color=STEEL)

# -- the cabinet, the keeper over the upper hinge, and the door stop ------------------------------------------------
CAB_X = (HX + LEAF_X[1], HX + 0.140)
CAB_Y = (HY - 0.012, HY + 0.185)
CAB_TOP = B + 0.116
CABINET = Box("cabinet", pos=(sum(CAB_X) / 2, sum(CAB_Y) / 2, (TOP + CAB_TOP) / 2),
              size=(CAB_X[1] - CAB_X[0], CAB_Y[1] - CAB_Y[0], CAB_TOP - TOP), color=STEEL)

DOOR_H = 0.100
# The upper sleeve carries a tab on its outer side (away from the panel). Above it, a fixed keeper plate on the
# cabinet side of the upper hinge leaves the tab 6 mm to rise, except through a notch that the tab only reaches with
# the door between about 80 and 95 degrees open. Directions here are world angles about the hinge axis, 0 = toward
# the cabinet (+x), 90 = toward the robot's left; the tab points (door angle - 90) degrees.
TAB_R = (0.009, 0.024)  # out from the axis
TAB_W, TAB_H = 0.004, 0.006
TAB_GAP = 0.006  # from the tab's top to the keeper's underside, as hung
TAB_TOP = B + 0.084  # flush with the upper sleeve's top
KEEPER_T = 0.005
KEEPER_Z = (TAB_TOP + TAB_GAP, TAB_TOP + TAB_GAP + KEEPER_T)
KEEPER_R = (0.0135, 0.028)
KEEPER_SEGS = ((-100.0, -76.0, -52.0, -29.0), (21.0, 35.0, 50.0))  # degrees: the two plates either side of the notch


def _keeper_boxes():
    out = []
    r_mid, depth = sum(KEEPER_R) / 2, KEEPER_R[1] - KEEPER_R[0]
    k = 0
    for segs in KEEPER_SEGS:
        for a0, a1 in zip(segs[:-1], segs[1:]):
            am, half = math.radians((a0 + a1) / 2), math.radians((a1 - a0) / 2)
            length = 2 * KEEPER_R[1] * math.tan(half)
            c = (HX + r_mid * math.cos(am), HY + r_mid * math.sin(am), sum(KEEPER_Z) / 2)
            out.append(Box(f"keeper_{k}", pos=c, size=(depth, length, KEEPER_T), yaw=am, color=STEEL))
            k += 1
    # the bracket arms that carry the two plates from the cabinet face
    x0 = HX + 0.016
    for name, (y0, y1) in (("keeper_arm_right", (HY - 0.030, HY - 0.008)), ("keeper_arm_left", (HY + 0.010, HY + 0.030))):
        out.append(Box(name, pos=((x0 + CAB_X[0]) / 2, (y0 + y1) / 2, sum(KEEPER_Z) / 2),
                       size=(CAB_X[0] - x0, y1 - y0, KEEPER_T), color=STEEL))
    return out


KEEPER = _keeper_boxes()

# A door stop: a post the panel meets at about 125 degrees open.
STOP_DEG, STOP_R, STOP_POST_R, STOP_H = 125.0, 0.080, 0.006, 0.025
STOP = Cylinder("door_stop", pos=(HX + STOP_R * math.cos(math.radians(STOP_DEG + 90)) - 0.011 * math.cos(math.radians(STOP_DEG)),
                                  HY + STOP_R * math.sin(math.radians(STOP_DEG + 90)) - 0.011 * math.sin(math.radians(STOP_DEG)),
                                  TOP + STOP_H / 2), radius=STOP_POST_R, height=STOP_H, color=(0.15, 0.15, 0.17))

# -- the door (loose) ------------------------------------------------------------------------------------------------
# Door frame: origin on the hinge axis at the door's mid-height; local +y runs from the hinge to the free edge, local
# +x is the inner face's normal (toward the cabinet when closed). Yaw = how far it stands open.
DOOR_T, DOOR_W0, DOOR_W1 = 0.010, 0.009, 0.160  # panel thickness; panel from 9 mm to 160 mm out from the axis
HOLE_HALF, RING_T, SLEEVE_H = 0.006, 0.003, 0.018  # square knuckle sleeves: 12 mm hole, 3 mm walls, 18 mm tall
LOWER_SLEEVE_Z = -DOOR_H / 2 + SLEEVE_H / 2  # sleeve centres, door frame
UPPER_SLEEVE_Z = -DOOR_H / 2 + 0.066 + SLEEVE_H / 2  # 66 to 84 mm above the door's bottom edge
START_DEG = 30.0
DOOR_MASS = 0.12


def _ring(zc):
    o = HOLE_HALF + RING_T
    return [
        parts.box((RING_T, 2 * o, SLEEVE_H), at=(+(HOLE_HALF + RING_T / 2), 0.0, zc), color=STEEL),
        parts.box((RING_T, 2 * o, SLEEVE_H), at=(-(HOLE_HALF + RING_T / 2), 0.0, zc), color=STEEL),
        parts.box((2 * HOLE_HALF, RING_T, SLEEVE_H), at=(0.0, +(HOLE_HALF + RING_T / 2), zc), color=STEEL),
        parts.box((2 * HOLE_HALF, RING_T, SLEEVE_H), at=(0.0, -(HOLE_HALF + RING_T / 2), zc), color=STEEL),
    ]


PANEL = parts.box((DOOR_T, DOOR_W1 - DOOR_W0, DOOR_H), at=(0.0, (DOOR_W0 + DOOR_W1) / 2, 0.0), color=DOOR_CREAM)
TAB = parts.box((TAB_W, TAB_R[1] - TAB_R[0], TAB_H), at=(0.0, -sum(TAB_R) / 2, UPPER_SLEEVE_Z + SLEEVE_H / 2 - TAB_H / 2),
                color=STEEL)
DOOR_SHAPES = [PANEL, *_ring(LOWER_SLEEVE_Z), *_ring(UPPER_SLEEVE_Z), TAB]
DOOR = parts.Part("door", pos=(HX, HY, B + DOOR_H / 2 + 0.0005), shapes=DOOR_SHAPES, mass=DOOR_MASS,
                  yaw=math.radians(START_DEG))

# -- the rack --------------------------------------------------------------------------------------------------------
RACK_Y = -0.33
RACK_X = (0.215, 0.335)
SLOT_W, RACK_WALL_T, RACK_WALL_H = 0.018, 0.010, 0.050
RACK_FLOOR = TOP
RACK = [Box(f"rack_wall_{s}", pos=(sum(RACK_X) / 2, RACK_Y + sign * (SLOT_W + RACK_WALL_T) / 2, TOP + RACK_WALL_H / 2),
            size=(RACK_X[1] - RACK_X[0], RACK_WALL_T, RACK_WALL_H), color=RACK_YELLOW)
        for s, sign in (("left", 1.0), ("right", -1.0))]

OBJECTS = [CABINET, COLLAR, LOWER_PIN, UPPER_LEAF, UPPER_PIN, *KEEPER, STOP, *RACK, DOOR]


# -- geometry helpers ------------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _local_to_world(st, local) -> np.ndarray:
    return np.asarray(st.pos, float) + np.asarray(local, float) @ _rot(st.quat_xyzw).T


_SIGNS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
PANEL_CORNERS = np.asarray(PANEL.at) + _SIGNS * np.asarray(PANEL.size) / 2


def upright_deg(st) -> float:
    """Angle between the door's long edges' normal in its plane (its local up) and the vertical, either way up."""
    return float(np.degrees(np.arccos(np.clip(abs(_rot(st.quat_xyzw)[2, 2]), 0.0, 1.0))))


def open_deg(st) -> float:
    r = _rot(st.quat_xyzw)
    return float(np.degrees(math.atan2(-r[0, 1], r[1, 1])))


def _sleeve_on_pin(st, zc, pin_z) -> bool:
    """A sleeve is on its pin while the pin's axis passes inside its hole: the pin top above the sleeve's bottom, and
    the sleeve's centre within 5 mm + the hole's half width (+ the pin's radius) of the axis, i.e. not clear by 5 mm."""
    c = _local_to_world(st, (0.0, 0.0, zc))
    up = _rot(st.quat_xyzw)[:, 2]
    bottom = c[2] - abs(up[2]) * SLEEVE_H / 2
    horiz = math.hypot(c[0] - HX, c[1] - HY)
    near = horiz < HOLE_HALF + RING_T + PIN_R + 0.005
    return bool(near and bottom < pin_z[1] + 0.005 and c[2] + SLEEVE_H / 2 > pin_z[0] - 0.005)


def off_pins(st) -> bool:
    return not (_sleeve_on_pin(st, LOWER_SLEEVE_Z, LOWER_PIN_Z) or _sleeve_on_pin(st, UPPER_SLEEVE_Z, UPPER_PIN_Z))


def in_rack(st) -> bool:
    """Standing in the rack: the panel's lowest corner within 3 mm of the slot floor, the door upright within 10
    degrees, and the middle of its bottom edge between the rack's walls and its ends."""
    corners = _local_to_world(st, PANEL_CORNERS)
    low = corners[np.argsort(corners[:, 2])[:4]]  # the bottom edge's four corners
    mid = low.mean(axis=0)
    return bool(
        corners[:, 2].min() - RACK_FLOOR <= 0.003 + EPS
        and upright_deg(st) <= 10.0 + EPS
        and abs(mid[1] - RACK_Y) <= SLOT_W / 2 + EPS
        and RACK_X[0] - EPS <= mid[0] <= RACK_X[1] + EPS
    )


# -- released: from the robot's finger and gripper collision hulls, as in the simulator ----------------------------
CLEAR = 0.005  # no finger or gripper surface within 5 mm of the door
_GRIP = {"clear": True}
_LINKS = ("gripper", "tip_left", "tip_right")


def _surface_local(step=0.005) -> np.ndarray:
    pts = []
    for s in DOOR_SHAPES:
        h, c = np.asarray(s.size) / 2, np.asarray(s.at)
        grids = [np.linspace(-v, v, max(2, int(np.ceil(2 * v / step)) + 1)) for v in h]
        for axis in range(3):
            a, b = [i for i in range(3) if i != axis]
            ga, gb = np.meshgrid(grids[a], grids[b], indexing="ij")
            for sign in (-1.0, 1.0):
                p = np.zeros((ga.size, 3))
                p[:, a], p[:, b], p[:, axis] = ga.ravel(), gb.ravel(), sign * h[axis]
                pts.append(p + c)
    return np.concatenate(pts)


_SURFACE = _surface_local()


def _robot_hulls(sim):
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        link_poses = {}
        for i, n in enumerate(asset.body_names):
            if n in _LINKS:
                t = np.eye(4)
                t[:3, :3], t[:3, 3] = _rot(poses[i, 3:]), poses[i, :3] - origin
                link_poses[n] = t
        out += analysis.world_hulls(link_poses)
    return out


def _clearance(hulls, pts) -> float:
    lo, hi = pts.min(0) - 0.02, pts.max(0) + 0.02
    best = np.inf
    for _, h in hulls:
        if np.any(h.vertices.max(0) < lo) or np.any(h.vertices.min(0) > hi):
            continue
        d = (pts @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1)  # a lower bound on each point's distance
        best = min(best, float(d.min()))
    return best


@parts.on_step
def _check_grip(sim):
    d = sim.states()["door"]
    _GRIP["clear"] = _clearance(_robot_hulls(sim), _local_to_world(d, _SURFACE)) >= CLEAR - EPS


def released() -> bool:
    return bool(_GRIP["clear"])


# -- scoring ---------------------------------------------------------------------------------------------------------
def success(objects):
    """The door off both pins, standing in the rack, no gripper within 5 mm of it, and still."""
    d = objects["door"]
    return bool(off_pins(d) and in_rack(d) and d.resting() and released())


def score(objects):
    """Pass/fail: 1.0 as success, otherwise 0."""
    return 1.0 if success(objects) else 0.0


def distance(objects):
    d = objects["door"]
    mid = _local_to_world(d, (0.0, (DOOR_W0 + DOOR_W1) / 2, -DOOR_H / 2))
    return float(np.linalg.norm(mid - np.array((sum(RACK_X) / 2, RACK_Y, RACK_FLOOR))))


# -- scripted demo ---------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
GRIP_R = 0.081  # along the door from the axis: over its centre of mass
GRIP_DEPTH = 0.015  # finger tips this far below the door's top edge
HALF_AROUND = -0.016  # fingers 6 mm clear of each face while closing round the panel
HALF_GRIP = -0.002  # 3 mm past each face
SWING_TO = 88.0  # degrees open, commanded
OFF_PINS = 0.032  # lift: the sleeves leave the pins at 25 mm; at 37 mm the lower sleeve would meet the upper leaf
SIDESTEP = 0.035  # then straight back toward the robot, out from under the upper leaf and the keeper
LIFT = 0.060  # door bottom this far above its hung height while carrying
DROP = 0.006  # released with its bottom edge 6 mm above the slot floor
CARRY_DY = -0.0023  # the held door lands 2.3 mm to the robot's left of the grip target (GPU trace)


def _grip_xy(deg):
    a = math.radians(deg)
    return HX - GRIP_R * math.sin(a), HY + GRIP_R * math.cos(a)


def demo():
    """Right arm: grips the door's top edge over its centre of mass, swings it open about its pins to the angle where
    its sleeve's tab lines up with the keeper's notch, lifts it off the pins, carries it over the rack, lowers it into the slot
    and lets go."""
    tcp_hung = B + DOOR_H - GRIP_DEPTH + TIP
    hover = TOP + 0.17
    yaw0 = math.radians(START_DEG - 90.0)
    gx, gy = _grip_xy(START_DEG)
    plan = [
        Waypoint(1.5, (gx, gy, hover), HALF_AROUND, "over the door's top edge", yaw0),
        Waypoint(1.5, (gx, gy, tcp_hung), HALF_AROUND, "fingers either side of the panel", yaw0),
        Waypoint(0.8, (gx, gy, tcp_hung), HALF_GRIP, "grip", yaw0),
    ]
    deg = START_DEG
    while deg < SWING_TO - 1e-9:
        deg = min(SWING_TO, deg + 6.0)
        plan.append(Waypoint(0.45, (*_grip_xy(deg), tcp_hung), HALF_GRIP, f"swing to {deg:.0f}",
                             math.radians(deg - 90.0)))
    sx, sy = _grip_xy(SWING_TO)
    yaw_s = math.radians(SWING_TO - 90.0)
    up = tcp_hung + LIFT
    # Squared to the rack (yaw 0): the door's panel runs along x, its hinge end toward +x, the grip over the slot.
    rx, ry = HX - GRIP_R, RACK_Y + CARRY_DY
    above = RACK_FLOOR + RACK_WALL_H + 0.015 + DOOR_H - GRIP_DEPTH + TIP  # bottom edge 15 mm over the walls
    down = RACK_FLOOR + DROP + DOOR_H - GRIP_DEPTH + TIP
    plan += [
        Waypoint(0.6, (sx, sy, tcp_hung), HALF_GRIP, "settle open", yaw_s),
        Waypoint(2.0, (sx, sy, tcp_hung + OFF_PINS), HALF_GRIP, "lift off the pins", yaw_s),
        Waypoint(1.2, (sx - SIDESTEP, sy, tcp_hung + OFF_PINS), HALF_GRIP, "back from the hinge", yaw_s),
        Waypoint(1.0, (sx - SIDESTEP, sy, up), HALF_GRIP, "up to carry height", yaw_s),
        Waypoint(1.2, (sx - SIDESTEP, sy, up), HALF_GRIP, "square it", 0.0),
        Waypoint(1.5, (sx - SIDESTEP, ry, up), HALF_GRIP, "across, clear of the hinge", 0.0),
        Waypoint(1.2, (rx, ry, up), HALF_GRIP, "over the rack", 0.0),
        Waypoint(1.0, (rx, ry, above), HALF_GRIP, "just over the walls", 0.0),
        Waypoint(1.0, (rx, ry, above), HALF_GRIP, "steady", 0.0),
        Waypoint(1.5, (rx, ry, down), HALF_GRIP, "into the slot", 0.0),
        Waypoint(0.6, (rx, ry, down), FINGER_OPEN, "let go", 0.0),
        Waypoint(1.2, (rx, ry, hover), FINGER_OPEN, "up and away", 0.0),
    ]
    return {"right": plan}
