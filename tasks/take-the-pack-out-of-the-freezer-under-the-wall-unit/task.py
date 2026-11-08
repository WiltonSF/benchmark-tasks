"""On the station table, in front of the left arm: a small white chest freezer standing in front of a wall, under a
grey wall cupboard. The freezer's pale blue lid is hinged along its back edge and stands part open; an orange bumper
on the cupboard's lower front edge stops it opening further. Inside, near the back under the lid, two food packs of the same size stand side by side: one blue, one
white. A grey mat lies on the table to the left of the freezer.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import lots  # noqa: E402
import parts  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

WHITE, LID_BLUE, GREY, DARK, BLUE, PACK_WHITE, MAT, BUMPER, WALL_C = (
    (0.93, 0.93, 0.92), (0.62, 0.76, 0.86), (0.50, 0.50, 0.52), (0.20, 0.20, 0.22), (0.15, 0.30, 0.85),
    (0.95, 0.95, 0.95), (0.45, 0.45, 0.45), (0.95, 0.55, 0.10), (0.80, 0.78, 0.74))

# -- the freezer (world frame: x away from the arm bases, y to the robot's left, z up; m) -------------------------
FX0, FX1 = 0.230, 0.470  # front and back outer faces
FY0, FY1 = -0.060, 0.220  # right and left outer faces
WALL = 0.015
BASE_T = 0.020
RIM = TOP + 0.120  # top of the walls
FLOOR = TOP + BASE_T  # inside floor: 100 mm below the rim
IX0, IX1, IY0, IY1 = FX0 + WALL, FX1 - WALL, FY0 + WALL, FY1 - WALL  # inside faces
FC = ((FX0 + FX1) / 2, (FY0 + FY1) / 2)


def _wall(at, size):
    return parts.box(size, at=(at[0] - FC[0], at[1] - FC[1], at[2] - TOP), color=WHITE)


_H = RIM - TOP
_BODY = [
    _wall((FC[0], FC[1], TOP + BASE_T / 2), (FX1 - FX0, FY1 - FY0, BASE_T)),
    _wall((FX0 + WALL / 2, FC[1], TOP + _H / 2), (WALL, FY1 - FY0, _H)),
    _wall((FX1 - WALL / 2, FC[1], TOP + _H / 2), (WALL, FY1 - FY0, _H)),
    _wall((FC[0], FY0 + WALL / 2, TOP + _H / 2), (FX1 - FX0 - 2 * WALL, WALL, _H)),
    _wall((FC[0], FY1 - WALL / 2, TOP + _H / 2), (FX1 - FX0 - 2 * WALL, WALL, _H)),
]

# The lid: a 12 mm pale blue panel the size of the freezer top with a handle along its front edge, hinged along its
# top back edge. A counterbalance hinge holds it up: a weighted arm behind the hinge (under the cupboard, out of the top
# camera's view) outweighs the lid, so the lid swings up until its top face meets the bumper on the wall cupboard's
# lower front edge (OPEN_DEG). The shapes are written closed (joint 0); the lid rises
# to the cupboard during the embodiment's settle at reset, before the first camera frame.
LID_T = 0.012
OPEN_DEG = 47.0
PIVOT = np.array([FX1, 0.0, RIM + LID_T])  # world x, (unused y), z
LID_L = FX1 - FX0
CW = (0.120, 0.070)  # counterbalance arm: depth behind the hinge, height (up from the lid's underside)
CW_GAP = 0.005


def _lid_shapes():
    """Lid shapes closed (joint 0), in the part frame."""
    px, pz = PIVOT[0] - FC[0], PIVOT[2] - TOP
    panel = parts.box((LID_L, FY1 - FY0, LID_T), at=(px - LID_L / 2, 0.0, pz - LID_T / 2), color=LID_BLUE)
    handle = parts.box((0.015, 0.12, 0.020), at=(px - LID_L + 0.0075, 0.0, pz + 0.010), color=GREY)
    weight = parts.box((CW[0], FY1 - FY0, CW[1]), at=(px + CW_GAP + CW[0] / 2, 0.0, pz - LID_T + CW[1] / 2), color=GREY)
    return [panel, handle, weight]


LID = parts.Hinge("lid", shapes=_lid_shapes(), pivot=(PIVOT[0] - FC[0], 0.0, PIVOT[2] - TOP), axis=(0.0, 1.0, 0.0),
                  lower=0.0, upper=math.radians(60.0), mass=1.0, spring=0.0, damping=0.05)
FREEZER = parts.Part("freezer", pos=(FC[0], FC[1], TOP), shapes=_BODY, joints=[LID])


def lid_point(u, w, deg=OPEN_DEG):
    """World (x, z) of the closed-lid point (u, w) relative to the hinge (u < 0 forward, w < 0 down), lid open deg."""
    a = math.radians(deg)
    return PIVOT[0] + u * math.cos(a) + w * math.sin(a), PIVOT[2] - u * math.sin(a) + w * math.cos(a)


# -- the wall and the wall cupboard -----------------------------------------------------------------------------------
# The freezer stands 14.5 cm out from the wall, room for the counterbalance arm to swing down behind it. The cupboard
# hangs on the wall; an orange rubber bumper along its lower front edge lies on the lid's top face at OPEN_DEG, 45 %
# of the way from the hinge to the lid's front edge.
WALL_X = FX1 + 0.145
BACK_WALL = Box("wall", pos=(WALL_X + 0.010, FC[1], TOP + 0.200), size=(0.020, 0.50, 0.400), color=WALL_C)
BUMP = 0.012  # the bumper's square section
_ex, _ez = lid_point(-0.45 * LID_L, 0.0)
CUP_X0, CUP_X1, CUP_Z0, CUP_H = _ex, WALL_X, _ez + BUMP, 0.14  # the cupboard sits on the bumper strip
CUPBOARD = Box("wall_cupboard", pos=((CUP_X0 + CUP_X1) / 2, FC[1], CUP_Z0 + CUP_H / 2),
               size=(CUP_X1 - CUP_X0, FY1 - FY0 + 0.02, CUP_H), color=GREY)
BUMPER_BOX = Box("cupboard_bumper", pos=(_ex + BUMP / 2, FC[1], _ez + BUMP / 2),
                 size=(BUMP, FY1 - FY0 + 0.02, BUMP), color=BUMPER)

# -- the packs: 60 mm front to back, 45 mm across, 50 mm tall, 0.1 kg; standing at the back -------------------------
PACK = (0.060, 0.045, 0.050)
PACK_X = IX1 - 0.030 - PACK[0] / 2
SLOT_L, SLOT_R = FC[1] + 0.050, FC[1] - 0.050  # left and right places at the back
_pz = FLOOR + PACK[2] / 2
BLUE_PACK = Box("blue_pack", pos=(PACK_X, SLOT_L, _pz), size=PACK, mass=0.10, color=BLUE)
WHITE_PACK = Box("white_pack", pos=(PACK_X, SLOT_R, _pz), size=PACK, mass=0.10, color=PACK_WHITE)

MAT_C, MAT_S, MAT_T = (0.30, 0.36), 0.12, 0.004
MAT_BOX = Box("mat", pos=(MAT_C[0], MAT_C[1], TOP + MAT_T / 2), size=(MAT_S, MAT_S, MAT_T), color=MAT)

OBJECTS = [FREEZER, BACK_WALL, CUPBOARD, BUMPER_BOX, BLUE_PACK, WHITE_PACK, MAT_BOX]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A"),
    lots.Lot("B", place={"blue_pack": (PACK_X, SLOT_R, _pz), "white_pack": (PACK_X, SLOT_L, _pz)}),
)


# -- released: from the robot's finger and gripper collision hulls, as in the simulator ----------------------------
CLEAR = 0.005  # no finger or gripper surface within 5 mm of either pack
_LINKS = ("gripper", "tip_left", "tip_right")
_GRIP = {"clear": True}


def _surface_local(size, step=0.005) -> np.ndarray:
    h = np.asarray(size) / 2
    grids = [np.linspace(-v, v, max(2, int(np.ceil(2 * v / step)) + 1)) for v in h]
    pts = []
    for axis in range(3):
        a, b = [k for k in range(3) if k != axis]
        ga, gb = np.meshgrid(grids[a], grids[b], indexing="ij")
        for sign in (-1.0, 1.0):
            p = np.zeros((ga.size, 3))
            p[:, a], p[:, b], p[:, axis] = ga.ravel(), gb.ravel(), sign * h[axis]
            pts.append(p)
    return np.concatenate(pts)


_PACK_SURFACE = _surface_local(PACK)


def _robot_hulls(sim):
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        link_poses = {}
        for k, n in enumerate(asset.body_names):
            if n in _LINKS:
                t = np.eye(4)
                t[:3, :3], t[:3, 3] = parts._quat_to_matrix(poses[k, 3:]), poses[k, :3] - origin
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
    states = sim.states()
    hulls = _robot_hulls(sim)
    clear = True
    for name in ("blue_pack", "white_pack"):
        st = states[name]
        pts = _PACK_SURFACE @ parts._quat_to_matrix(st.quat_xyzw).T + np.asarray(st.pos)
        clear = clear and _clearance(hulls, pts) >= CLEAR - EPS
    _GRIP["clear"] = clear


def released() -> bool:
    return bool(_GRIP["clear"])


# -- scoring ---------------------------------------------------------------------------------------------------------
ON_TOL = 0.001  # a pack's lowest point within 1 mm of the surface it rests on


def in_freezer(st) -> bool:
    """Centre inside the freezer's inside walls and below its rim."""
    p = np.asarray(st.pos, float)
    return bool(IX0 - EPS <= p[0] <= IX1 + EPS and IY0 - EPS <= p[1] <= IY1 + EPS and p[2] <= RIM + EPS)


def clear_of_freezer(st) -> bool:
    """Wholly outside the freezer's outer footprint."""
    lo, hi = np.asarray(st.low, float), np.asarray(st.high, float)
    return bool(hi[0] <= FX0 + EPS or lo[0] >= FX1 - EPS or hi[1] <= FY0 + EPS or lo[1] >= FY1 - EPS)


def on_mat(st) -> bool:
    """Lowest point within 1 mm of the mat's top and centre over the mat."""
    p = np.asarray(st.pos, float)
    return bool(abs(float(st.low[2]) - (TOP + MAT_T)) <= ON_TOL + EPS
                and abs(p[0] - MAT_C[0]) <= MAT_S / 2 + EPS and abs(p[1] - MAT_C[1]) <= MAT_S / 2 + EPS)


def on_table(st) -> bool:
    """Lowest point within 1 mm of the table top or of the mat's top."""
    return bool(abs(float(st.low[2]) - TOP) <= ON_TOL + EPS or abs(float(st.low[2]) - (TOP + MAT_T)) <= ON_TOL + EPS)


def _settled(objects) -> bool:
    return bool(objects["blue_pack"].resting() and objects["white_pack"].resting() and released())


def success(objects):
    """Blue pack on the mat, white pack still in the freezer, both still, no gripper within 5 mm of either."""
    b, w = objects["blue_pack"], objects["white_pack"]
    return bool(on_mat(b) and clear_of_freezer(b) and in_freezer(w) and _settled(objects))


def score(objects):
    """1.0 as success; 0.5 with the blue pack out of the freezer, on the table but not on the mat (white pack still in
    the freezer, both still, released); otherwise 0."""
    if success(objects):
        return 1.0
    b, w = objects["blue_pack"], objects["white_pack"]
    if on_table(b) and clear_of_freezer(b) and in_freezer(w) and _settled(objects):
        return 0.5
    return 0.0


def distance(objects):
    p = np.asarray(objects["blue_pack"].pos, float)
    return float(np.linalg.norm(p - np.array((MAT_C[0], MAT_C[1], TOP + MAT_T + PACK[2] / 2))))


# -- scripted demo: fixed waypoints per lot; the leaning reach uses tilt.py (fingertips lean toward +x) -------------
LEAN = 53.0  # degrees: fingertips forward, so the wrist trails low under the lid
HOLD = -(PACK[1] / 2 - 0.003)  # close 3 mm past the pack's faces
OPEN = -0.036
TRAVEL_Z = TOP + 0.16
PARK = (0.20, 0.36, TOP + 0.16)
GRIP_Z = FLOOR + 0.034


def _along(c, s, lean=None):
    """The point ``s`` m back along the approach axis from ``c`` (toward the wrist)."""
    a = math.radians(LEAN if lean is None else lean)
    return (c[0] - s * math.sin(a), c[1], c[2] + s * math.cos(a))


def demo():
    lot = lots.current()
    px, py, _ = lot.pos("blue_pack")
    grip = (px, py, GRIP_Z)
    up = (px, py, GRIP_Z + 0.008)
    lean = dict(tilt=LEAN, heading=0.0)
    W = tilt.Waypoint
    mx, my = MAT_C
    place = (mx, my, TOP + MAT_T + PACK[2] / 2 + 0.006)
    w = [
        Waypoint(1.5, (0.16, py, TOP + 0.18), OPEN, "in front of the freezer"),
        W(1.5, _along(grip, 0.30), OPEN, "lean, in line with the pack", **lean),
        W(2.0, _along(grip, 0.06), OPEN, "in under the lid", **lean),
        W(1.0, grip, OPEN, "round the pack", **lean),
        W(0.8, grip, HOLD, "grip", **lean),
        W(0.8, up, HOLD, "lift off the floor", **lean),
        W(2.0, _along(up, 0.30), HOLD, "back out under the lid", **lean),
        W(2.0, _along(place, 0.12), HOLD, "over the mat, still leaning", **lean),
        W(1.2, place, HOLD, "down", **lean),
        W(0.8, place, FINGER_OPEN, "let go", **lean),
        W(1.2, _along(place, 0.12), FINGER_OPEN, "back off along the lean", **lean),
        Waypoint(1.5, PARK, FINGER_OPEN, "park"),
    ]
    return {"left": w}
