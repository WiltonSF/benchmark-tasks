"""On the station table: a grey steel frame bar runs across the table, well out in front of the arms. It has two
shallow rectangular pockets in its top, one on each side of the midline, and in one far corner of each pocket stands
a small square peg. Nearer the robot, three grey hinge brackets lie flat on the table in a row across it. Each
bracket is a flat base plate with a square notch cut from one corner and an upright leg along the long edge next to
the notch.
"""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import lots  # noqa: E402
import parts  # noqa: E402

from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
# ---- frame bar with two pockets --------------------------------------------------------------------------------
FRAME_X, FRAME = 0.330, (0.060, 0.320, 0.020)  # centre x; depth (x), length (y), height
POCKET = (0.049, 0.074, 0.008)  # across (x), along (y), depth
POCKET_Y = {"left": 0.100, "right": -0.100}
PEG = (0.014, 0.014, 0.006)
FLOOR = TOP + FRAME[2] - POCKET[2]  # pocket floor height
STEEL, PEG_C, ALU = (0.42, 0.44, 0.47), (0.15, 0.15, 0.17), (0.70, 0.72, 0.75)


def _peg_xy(side):
    """Peg centre: in the far corner of the pocket, on the pocket's own side (far-left / far-right)."""
    s = 1.0 if side == "left" else -1.0
    return (FRAME_X + POCKET[0] / 2 - PEG[0] / 2, POCKET_Y[side] + s * (POCKET[1] / 2 - PEG[1] / 2))


def _frame():
    fx, fy, fz = FRAME
    lower = fz - POCKET[2]
    shapes = [parts.box((fx, fy, lower), at=(0.0, 0.0, lower / 2), color=STEEL)]
    zc = lower + POCKET[2] / 2
    edges = sorted([-fy / 2, fy / 2] + [y + d * POCKET[1] / 2 for y in POCKET_Y.values() for d in (-1, 1)])
    for a, b in zip(edges[0::2], edges[1::2]):  # full-depth top layer between and outside the pockets
        shapes.append(parts.box((fx, b - a, POCKET[2]), at=(0.0, (a + b) / 2, zc), color=STEEL))
    wall = (fx - POCKET[0]) / 2
    for y in POCKET_Y.values():  # the walls in front of and behind each pocket
        for s in (-1, 1):
            shapes.append(parts.box((wall, POCKET[1], POCKET[2]), at=(s * (POCKET[0] / 2 + wall / 2), y, zc),
                                    color=STEEL))
    for side in POCKET_Y:
        px, py = _peg_xy(side)
        shapes.append(parts.box(PEG, at=(px - FRAME_X, py, lower + PEG[2] / 2), color=PEG_C))
    return parts.Part("frame", pos=(FRAME_X, 0.0, TOP), shapes=shapes)


# ---- brackets ----------------------------------------------------------------------------------------------------
# Bracket frame: origin at the centre of the full base rectangle, mid-thickness; x across (leg side +x), y along.
BASE = (0.041, 0.066, 0.008)
NOTCH = 0.020
LEG_T, LEG_H = 0.008, 0.030
LEG_LEN = BASE[1] - NOTCH
BRACKET_MASS = 0.08
BRACKET_Z = TOP + BASE[2] / 2


def _bracket(name, hand, pos, yaw=0.0):
    """Left hand: seen from the base's -x edge with the leg away, the notch is at the left (+y) end."""
    s = 1.0 if hand == "left" else -1.0
    bx, by, bz = BASE
    shapes = [
        parts.box((bx, by - NOTCH, bz), at=(0.0, -s * NOTCH / 2, 0.0), color=ALU),
        parts.box((bx - NOTCH, NOTCH, bz), at=(-NOTCH / 2, s * (by - NOTCH) / 2, 0.0), color=ALU),
        parts.box((LEG_T, LEG_LEN, LEG_H), at=(bx / 2 - LEG_T / 2, -s * NOTCH / 2, bz / 2 + LEG_H / 2), color=ALU),
    ]
    return parts.Part(name, pos=pos, yaw=yaw, mass=BRACKET_MASS, shapes=shapes)


TRAY_X = 0.210
TRAY_Y = (0.110, 0.0, -0.110)  # left, middle, right place
LEFT_HAND = "bracket_l"
RIGHT_HANDS = ("bracket_r1", "bracket_r2")
HANDS = {LEFT_HAND: "left", "bracket_r1": "right", "bracket_r2": "right"}


def _at(i):
    return (TRAY_X, TRAY_Y[i], BRACKET_Z)


BRACKETS = {LEFT_HAND: _bracket(LEFT_HAND, "left", _at(1)),
            "bracket_r1": _bracket("bracket_r1", "right", _at(0)),
            "bracket_r2": _bracket("bracket_r2", "right", _at(2))}
FRAME_PART = _frame()
OBJECTS = [FRAME_PART, *BRACKETS.values()]
Q = np.pi / 2
# Lot A (as declared, the screenshot): left-hand bracket at the middle place, legs away from the robot. Lot B: at the
# left place. Lot C: at the right place, and every bracket turned a quarter turn (the left-hand one with its leg to
# the robot's left, the right-hand ones with their legs to the right).
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", right_pick="bracket_r2"),
    lots.Lot("B", right_pick="bracket_r2", place={LEFT_HAND: _at(0), "bracket_r1": _at(1)}),
    lots.Lot("C", right_pick="bracket_r1", place={LEFT_HAND: _at(2), "bracket_r1": _at(1), "bracket_r2": _at(0)},
             yaw={LEFT_HAND: Q, "bracket_r1": -Q, "bracket_r2": -Q}),
)


# ---- scoring -----------------------------------------------------------------------------------------------------
EPS = 1e-6
SEAT_TOL = 0.001  # every corner of the base's underside within 1 mm of the pocket floor
FLAT_DEG = 2.0
CLEAR = 0.005  # released: no finger or gripper surface within 5 mm of any bracket
SUCCESS_HOLD_S = 2.0


def _rot(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


_CORNERS = np.array([[sx * BASE[0] / 2, sy * BASE[1] / 2, -BASE[2] / 2] for sx in (-1, 1) for sy in (-1, 1)])


def _seated_in(st, side, name) -> bool:
    """Flat on this pocket's floor (all four underside corners within 1 mm of it, upright within 2 degrees), inside
    the pocket, and clear of the pocket's peg: no part of the peg under the bracket's base (it stands in the notch)."""
    r, p = _rot(st.quat_xyzw), np.asarray(st.pos, float)
    if r[2, 2] < np.cos(np.radians(FLAT_DEG)) - EPS:
        return False
    w = p + _CORNERS @ r.T
    if np.any(np.abs(w[:, 2] - FLOOR) > SEAT_TOL + EPS):
        return False
    if np.any(np.abs(w[:, 0] - FRAME_X) > POCKET[0] / 2 + EPS) or \
            np.any(np.abs(w[:, 1] - POCKET_Y[side]) > POCKET[1] / 2 + EPS):
        return False
    px, py = _peg_xy(side)
    g = np.linspace(-0.5, 0.5, 8)
    peg = np.array([(px + a * PEG[0], py + b * PEG[1], FLOOR) for a in g for b in g])
    local = (peg - p) @ r  # peg points in the bracket's frame
    for s in BRACKETS[name].shapes[:2]:  # the base plate's two boxes (the notch is the corner they leave out)
        inside = np.all(np.abs(local[:, :2] - np.asarray(s.at[:2])) < np.asarray(s.half[:2]) - 0.0005, axis=1)
        if np.any(inside):
            return False
    return True


# Released: measured in the simulator after every control step, from both arms' gripper-body and fingertip collision hulls
# against points on every bracket's surface. Offline (near_misses.py) it is taken as true.
_GRIP = {"clear": True}
_LINKS = ("gripper", "tip_left", "tip_right")


def _box_surface(size, step=0.005) -> np.ndarray:
    h = np.asarray(size, float) / 2
    grids = [np.linspace(-v, v, max(2, int(np.ceil(2 * v / step)) + 1)) for v in h]
    pts = []
    for axis in range(3):
        a, b = [i for i in range(3) if i != axis]
        ga, gb = np.meshgrid(grids[a], grids[b], indexing="ij")
        for sign in (-1.0, 1.0):
            q = np.zeros((ga.size, 3))
            q[:, a], q[:, b], q[:, axis] = ga.ravel(), gb.ravel(), sign * h[axis]
            pts.append(q)
    return np.concatenate(pts)


_SURF = {n: np.concatenate([np.asarray(s.at) + _box_surface(2 * s.half) for s in b.shapes])
         for n, b in BRACKETS.items()}


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
    st = sim.states()
    pts = np.concatenate([np.asarray(st[n].pos) + _SURF[n] @ _rot(st[n].quat_xyzw).T for n in BRACKETS])
    _GRIP["clear"] = _clearance(_robot_hulls(sim), pts) >= CLEAR - EPS


def released(objects) -> bool:
    return bool(_GRIP["clear"])


def _pockets(objects):
    """{pocket side: True if some bracket is seated in it}."""
    out = {}
    for side in POCKET_Y:
        out[side] = False
        for n in BRACKETS:
            if _seated_in(objects[n], side, n):
                out[side] = True
    return out


def _still(objects) -> bool:
    return all(objects[n].resting() for n in BRACKETS)


def success(objects):
    """A bracket seated flat in each pocket, let go, everything still."""
    p = _pockets(objects)
    return bool(p["left"] and p["right"] and released(objects) and _still(objects))


def distance(objects):
    goal = np.array((FRAME_X, POCKET_Y["left"], FLOOR + BASE[2] / 2))
    return float(np.linalg.norm(np.asarray(objects[LEFT_HAND].pos, float) - goal))


def score(objects):
    """1.0 as success; 0.5 the left pocket's bracket seated (the only one that fits it) with none seated in the right
    pocket, let go and still; 0 otherwise."""
    if success(objects):
        return 1.0
    p = _pockets(objects)
    return 0.5 if p["left"] and not p["right"] and released(objects) and _still(objects) else 0.0


# ---- scripted demo (lot-aware by role: where this lot put each bracket and how it is turned; no state reads) -----
TIP = 0.0129  # finger tips below the TCP
GRIP_TIPS = BASE[2] + LEG_H - 0.015  # finger tips 15 mm below the leg's top, above the base
LEG_GRIP = -0.001  # fingers closed to 2 mm on the 8 mm leg
DROP = 0.005  # set down with the base 5 mm above the pocket floor, then let go
TRAVEL_Z = TOP + 0.13
YAW_X = {"left": -Q, "right": Q}  # fingers closing along x at the pocket (a wrist limit picks the sign)
TURN_SAG = {"left": (0.0044, -0.0038), "right": (0.0, 0.0)}  # aim offset after a quarter turn, from traces
PARK = {"left": (0.18, 0.28, TOP + 0.14), "right": (0.18, -0.28, TOP + 0.14)}


def _leg_point(hand):
    """The middle of the leg's length, in the bracket frame (x, y)."""
    s = 1.0 if hand == "left" else -1.0
    return np.array((BASE[0] / 2 - LEG_T / 2, -s * NOTCH / 2))


def _fit(arm, name, lot):
    hand = HANDS[name]
    yaw0 = lot.yaw_of(name)
    c0 = np.asarray(lot.pos(name)[:2])
    rz = lambda a: np.array(((np.cos(a), -np.sin(a)), (np.sin(a), np.cos(a))))  # noqa: E731
    g0 = c0 + rz(yaw0) @ _leg_point(hand)
    g1 = np.array((FRAME_X, POCKET_Y[arm])) + _leg_point(hand)  # seated with yaw 0
    if abs(yaw0) > 1e-6:  # turned in the grip on the way: the carried bracket lags (trace: 4.4 mm near, 3.8 mm left)
        g1 = g1 + np.asarray(TURN_SAG[arm])
    grip_yaw0 = YAW_X[arm] + yaw0  # the same grasp on the leg, turned with the bracket
    pick_z = TOP + GRIP_TIPS + TIP
    place_z = FLOOR + DROP + GRIP_TIPS + TIP
    return [
        Waypoint(2.0, (*g0, TRAVEL_Z), FINGER_OPEN, "over the leg", grip_yaw0),
        Waypoint(1.5, (*g0, pick_z), FINGER_OPEN, "down either side of it", grip_yaw0),
        Waypoint(0.8, (*g0, pick_z), LEG_GRIP, "grip the leg", grip_yaw0),
        Waypoint(1.5, (*g0, TRAVEL_Z), LEG_GRIP, "lift", grip_yaw0),
        Waypoint(2.5, (*g1, TRAVEL_Z), LEG_GRIP, "carry over the pocket, turning it square", YAW_X[arm]),
        Waypoint(2.0, (*g1, place_z), LEG_GRIP, "lower into the pocket", YAW_X[arm]),
        Waypoint(0.8, (*g1, place_z), LEG_GRIP, "settle", YAW_X[arm]),
        Waypoint(0.6, (*g1, place_z), FINGER_OPEN, "let go", YAW_X[arm]),
        Waypoint(1.2, (*g1, TRAVEL_Z), FINGER_OPEN, "up", YAW_X[arm]),
        Waypoint(1.5, PARK[arm], FINGER_OPEN, "park", 0.0),
    ]


def demo():
    """The left arm fits the left-hand bracket to the left pocket while the right arm waits clear; then the right arm
    fits a right-hand bracket to the right pocket. One at a time, because in lots B and C the arms' paths cross."""
    lot = lots.current()
    left = _fit("left", LEFT_HAND, lot)
    t_left = sum(w.duration for w in left)
    right = [Waypoint(1.5, PARK["right"], FINGER_OPEN, "park", 0.0),
             Waypoint(t_left - 1.5, PARK["right"], FINGER_OPEN, "wait, clear of the left arm", 0.0)]
    right += _fit("right", lot.right_pick, lot)
    end = sum(w.duration for w in right) + 3.0
    left.append(Waypoint(end - t_left, PARK["left"], FINGER_OPEN, "wait, clear of the right arm", 0.0))
    right.append(Waypoint(3.0, PARK["right"], FINGER_OPEN, "wait, hands clear", 0.0))
    return {"left": left, "right": right}
