"""On the station table, in front of the arms and a little to the robot's left: a grey book transit case with a
floor, a low rail across each end, a post at each corner and open long sides. Its flat lid is hinged along the
case's right-hand top edge and stands open, leaning out to the right of the case. Six hardback books of one size, each
with a black spine along one long edge, belong in the case. Some are already in it; the rest lie flat on the table to
the robot's left with their spines toward the robot. How many books start in the case, and which way their spines
face, changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots: its reset hooks run before the lots move objects)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# --- books ------------------------------------------------------------------------------------------------------
# Part frame: origin at the centre of the book's body (the boards and page block), x along its 100 mm length, y across
# its 64 mm width, the spine at +y, z up. The body is a 12 mm slab; under it run two strips the full length of the book,
# 5 mm wide, at its two long edges: 10 mm deep under the spine and 2 mm under the fore-edge. So a book is 22 mm thick at
# the spine and 14 mm at the fore-edge, and lying alone on a flat surface it leans 7.7 degrees, spine side up.
BOOK_L, BOOK_W, BODY_T = 0.100, 0.064, 0.012
STRIP_W, SPINE_DROP, FORE_DROP = 0.005, 0.010, 0.002
SPINE_BAND = 0.014  # the black spine band seen from above
BOOK_MASS = 0.12
LEAN = math.atan2(SPINE_DROP - FORE_DROP, BOOK_W - STRIP_W)  # 7.7 degrees

BLACK, CREAM = (0.06, 0.06, 0.07), (0.93, 0.90, 0.80)
COVERS = {
    1: (0.70, 0.12, 0.12), 2: (0.12, 0.30, 0.70), 3: (0.15, 0.50, 0.25),
    4: (0.80, 0.60, 0.15), 5: (0.50, 0.25, 0.60), 6: (0.10, 0.55, 0.60),
}


def _book_shapes(n):
    cover = COVERS[n]
    return [
        parts.box((BOOK_L, BOOK_W - SPINE_BAND, BODY_T), at=(0.0, -SPINE_BAND / 2, 0.0), color=cover),
        parts.box((BOOK_L, SPINE_BAND, BODY_T), at=(0.0, BOOK_W / 2 - SPINE_BAND / 2, 0.0), color=BLACK),
        parts.box((BOOK_L, STRIP_W, SPINE_DROP), at=(0.0, BOOK_W / 2 - STRIP_W / 2, -BODY_T / 2 - SPINE_DROP / 2),
                  color=BLACK),
        parts.box((BOOK_L, STRIP_W, FORE_DROP), at=(0.0, -BOOK_W / 2 + STRIP_W / 2, -BODY_T / 2 - FORE_DROP / 2),
                  color=CREAM),
    ]


# --- case -------------------------------------------------------------------------------------------------------
CASE_X, CASE_Y = 0.34, 0.015
FLOOR_T, WALL_T = 0.006, 0.006
INNER_X = 0.124  # between the end walls
CASE_W = 0.110  # floor and lid width across the open sides (y)
INNER_H = 0.114  # floor top to the lid's underside when shut
FLOOR = TOP + FLOOR_T
ROOF = FLOOR + INNER_H
HINGE_Y = CASE_Y - CASE_W / 2  # the lid's hinge runs along the case's right-hand top edge
LID_T, LID_W = 0.005, CASE_W
RAIL_H, POST = 0.030, 0.008  # end rails' height; corner posts' width
LIP = (0.012, 0.012)  # a lifting lip along the lid's free edge, on its outer face
CASE_LEN = INNER_X + 2 * WALL_T
GREY, DARK = (0.55, 0.57, 0.60), (0.30, 0.31, 0.33)
LID_SHUT = -math.pi / 2  # joint angle with the lid shut; 0 is upright
LID_OPEN = 0.5  # the open stop: leaning 29 degrees out from upright, away from the case

CASE = parts.Part(
    "book_case",
    pos=(CASE_X, CASE_Y, TOP),
    shapes=[
        parts.box((CASE_LEN, CASE_W, FLOOR_T), at=(0.0, 0.0, FLOOR_T / 2), color=GREY),
        # Low rails across the two ends, and a post at each corner carrying the lid.
        *[parts.box((WALL_T, CASE_W, RAIL_H), at=(sx * (INNER_X + WALL_T) / 2, 0.0, FLOOR_T + RAIL_H / 2), color=GREY)
          for sx in (-1, 1)],
        *[parts.box((WALL_T, POST, INNER_H), at=(sx * (INNER_X + WALL_T) / 2, sy * (CASE_W - POST) / 2,
                                                  FLOOR_T + INNER_H / 2), color=GREY)
          for sx in (-1, 1) for sy in (-1, 1)],
    ],
    joints=[parts.Hinge(
        "lid",
        # Written upright above the hinge, its inner face in the plane of the case's right-hand edge; the lip puts its
        # centre of mass outside the hinge, so as the scene settles it falls out to its open stop at LID_OPEN. Turning
        # it by -90 degrees about +x lays it flat over the case, its underside at ROOF.
        shapes=[
            parts.box((CASE_LEN, LID_T, LID_W), at=(0.0, -CASE_W / 2 - LID_T / 2, FLOOR_T + INNER_H + LID_W / 2),
                      color=DARK),
            parts.box((CASE_LEN, LIP[0], LIP[1]),
                      at=(0.0, -CASE_W / 2 - LID_T - LIP[0] / 2, FLOOR_T + INNER_H + LID_W - LIP[1] / 2), color=DARK),
        ],
        pivot=(0.0, -CASE_W / 2, FLOOR_T + INNER_H), axis=(1.0, 0.0, 0.0), lower=LID_SHUT, upper=LID_OPEN,
        mass=0.10, damping=0.04)],
)

# Where the books go: centred along the case, 10 mm left of its middle (clear of the standing lid).
PLACE_X, PLACE_Y = CASE_X, CASE_Y + 0.010
REST_Z = BODY_T / 2 + SPINE_DROP + 0.001  # body centre above a surface with the spine strip just clear of it

# Table spots, lot A. Books lie with their length along y and the spine toward the robot (yaw +90 degrees).
BENCH = {2: (0.36, 0.21), 3: (0.36, 0.32), 4: (0.36, 0.43), 5: (0.46, 0.26), 6: (0.46, 0.37)}
BENCH_YAW = math.pi / 2
SPINE_LEFT, SPINE_RIGHT = 0.0, math.pi  # yaw of a book in the case

BOOKS = {1: parts.Part("book_1", pos=(PLACE_X, PLACE_Y, FLOOR + REST_Z), yaw=SPINE_LEFT, mass=BOOK_MASS,
                       shapes=_book_shapes(1))}
for _n, (_x, _y) in BENCH.items():
    BOOKS[_n] = parts.Part(f"book_{_n}", pos=(_x, _y, TOP + REST_Z), yaw=BENCH_YAW, mass=BOOK_MASS,
                           shapes=_book_shapes(_n))
NAMES = [f"book_{n}" for n in BOOKS]

OBJECTS = [CASE, *BOOKS.values()]

# Lot A (the screenshot): book 1 in the case, spine to the robot's left. Lot B: book 1 spine to the right. Lot C:
# book 1 spine left and book 6 on it, also spine left. Lot A gives book 1 a zero move so every lot settles alike.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"book_1": BOOKS[1].pos}, start=("L",)),
    lots.Lot("B", yaw={"book_1": SPINE_RIGHT}, start=("R",)),
    lots.Lot("C", place={"book_6": (PLACE_X, PLACE_Y, FLOOR + 0.0245 + 0.002 + REST_Z)},
             yaw={"book_6": SPINE_LEFT}, start=("L", "L")),
)

# --- scoring ----------------------------------------------------------------------------------------------------
EPS = 1e-6
LEVEL = math.radians(4.0)  # the top book's covers within 4 degrees of level
FLAT = math.radians(30.0)  # every book lying flat: its covers within 30 degrees of level (none on edge or end)
SHUT_TOL = math.radians(1.5)  # the lid within 1.5 degrees of its closed stop (its free edge about 3 mm)
INSIDE_TOL = 0.001
CLEAR = 0.005  # released: no part of either gripper within 5 mm of a book or the lid
SUCCESS_HOLD_S = 1.0
X_IN = (CASE_X - INNER_X / 2, CASE_X + INNER_X / 2)
Y_IN = (CASE_Y - CASE_W / 2, CASE_Y + CASE_W / 2)


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _corners(name, st) -> np.ndarray:
    """Every corner of the book's four boxes, world frame (its true outline, not a bounding box)."""
    boxes = BOOKS[int(name[-1])].world_boxes({"base": (np.asarray(st.pos, float), _rot(st.quat_xyzw))})
    signs = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], float)
    return np.concatenate([c + (signs * (s / 2)) @ r.T for _, c, r, s in boxes])


def _inside(pts, below_roof: bool) -> bool:
    """Inside the case: within its end walls and floor outline seen from above, not below the floor top, and (if asked)
    nowhere above the lid's underside."""
    ok = (
        pts[:, 0].min() >= X_IN[0] - INSIDE_TOL - EPS and pts[:, 0].max() <= X_IN[1] + INSIDE_TOL + EPS
        and pts[:, 1].min() >= Y_IN[0] - INSIDE_TOL - EPS and pts[:, 1].max() <= Y_IN[1] + INSIDE_TOL + EPS
        and pts[:, 2].min() >= FLOOR - INSIDE_TOL - EPS
    )
    return bool(ok and (not below_roof or pts[:, 2].max() <= ROOF + EPS))


def _tilt(st) -> float:
    return math.acos(float(np.clip(_rot(st.quat_xyzw)[2, 2], -1.0, 1.0)))


def _top_book(objects) -> str:
    return max(NAMES, key=lambda n: float(objects[n].pos[2]))


def _lid_angle(objects) -> float:
    return float(objects["book_case"].joints["lid"])


# Released: in the simulator an on_step callback measures, every step, how close both arms' fingers and grippers are
# to every shape of every book and of the lid. Offline (near_misses.py) there is no robot, so "clear" stays True.
_grip = {"clear": True}


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    boxes = []
    for n in NAMES:
        s = st[n]
        boxes += BOOKS[int(n[-1])].world_boxes({"base": (np.asarray(s.pos), _rot(s.quat_xyzw))})
    case = st["book_case"]
    lid_pos, lid_q = case.links["lid"]
    boxes += CASE.world_boxes({"lid": (np.asarray(lid_pos), _rot(lid_q))})
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right", "gripper", "link5", "link4"):
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for _, c, r, s in boxes)


def _packed_level(objects) -> bool:
    """All six books flat, inside the case and below the lid's underside, the top book level, everything let go and
    still."""
    return bool(
        all(_inside(_corners(n, objects[n]), below_roof=True) for n in NAMES)
        and all(_tilt(objects[n]) <= FLAT + EPS for n in NAMES)
        and _tilt(objects[_top_book(objects)]) <= LEVEL + EPS
        and all(objects[n].resting() for n in NAMES)
        and _grip["clear"]
    )


def _shut(objects) -> bool:
    return _lid_angle(objects) <= LID_SHUT + SHUT_TOL + EPS


def success(objects):
    """All six books packed in the case with a level top, the lid shut, let go and still."""
    return bool(_packed_level(objects) and _shut(objects))


def distance(objects):
    return float(sum(np.linalg.norm(objects[n].pos[:2] - np.array((PLACE_X, PLACE_Y))) for n in NAMES))


def score(objects):
    """1.0 on success; 0.5 if the six books are packed with a level top but the lid is not yet shut; otherwise 0."""
    if not _packed_level(objects):
        return 0.0
    return 1.0 if _shut(objects) else 0.5


# --- demo -------------------------------------------------------------------------------------------------------
# Every motion is fixed by the lot's layout, which the cameras show: how many books are in the case and which way
# their spines face. The left arm packs; the right arm then tips the lid over.
TIP = 0.013  # finger tips below the TCP
GRIP_Z = 0.016  # TCP above the table to grip a book lying there (tips 3 mm up)
HELD_LOW = 0.021  # a held book's lowest point below the TCP (its body levels in the grip)
DROP = 0.006  # released with its lowest point this far above the stack
TRAVEL = TOP + 0.155
BENCH_HOVER = TOP + 0.10  # over the table spots (the far row is out of reach higher up)
SIDE = 0.075  # books come in over the case's open left side, this far left of their place
SLIDE = 0.015  # and this far above their release height
OPEN_BOOK = -0.039  # pads 78 mm apart around the 64 mm book
PINCH = -(BOOK_W / 2 - 0.005)  # closed 5 mm past the book's faces, so neither finger shoves it off centre


def stack_poses(seq, dy=None):
    """(y offset, z, roll) of each book's body centre, m above the case floor, for books stacked flat at the case
    centre with spines `seq` ('L'/'R') from the bottom; roll > 0 lifts the robot-left (+y) side. Each book lies on the
    flat top of the one below (the floor first) and its own strips tip it LEAN further toward its fore-edge side.
    `dy` shifts each book sideways along the surface it lands on (m)."""
    sp = BOOK_W / 2 - STRIP_W / 2
    a, p0, out = 0.0, np.zeros(2), []
    for i, c in enumerate(seq):
        s = 1.0 if c == "L" else -1.0
        ang = a + s * LEAN
        r = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
        q = p0 + np.array([math.cos(a), math.sin(a)]) * ((dy[i] if dy else 0.0) / math.cos(a))
        origin = q - r @ np.array([0.0, -(BODY_T / 2 + (SPINE_DROP + FORE_DROP) / 2)])
        out.append((float(origin[0]), float(origin[1]), ang))
        a = ang
        c_top = origin + r @ np.array([0.0, BODY_T / 2])
        p0 = c_top - np.array([math.cos(a), math.sin(a)]) * (c_top[0] / math.cos(a))
    return out


def stack_top(seq) -> float:
    """Highest point (m above the case floor) of the stack stack_poses() gives."""
    sp = BOOK_W / 2 - STRIP_W / 2
    top = 0.0
    for c, (y0, z0, ang) in zip(seq, stack_poses(seq)):
        s = 1.0 if c == "L" else -1.0
        r = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
        pts = [(y, z) for y in (-BOOK_W / 2, BOOK_W / 2) for z in (-BODY_T / 2, BODY_T / 2)]
        pts += [(s * sp + e, -BODY_T / 2 - SPINE_DROP) for e in (-STRIP_W / 2, STRIP_W / 2)]
        pts += [(-s * sp + e, -BODY_T / 2 - FORE_DROP) for e in (-STRIP_W / 2, STRIP_W / 2)]
        for p in pts:
            top = max(top, float(z0 + (r @ np.array(p))[1]))
    return top


def plan(lot):
    """(book, spine side) in packing order for a lot: keep the spines shared three and three."""
    start = list(lot.start)
    bench = [n for n in (2, 3, 4, 5, 6) if not (lot.name == "C" and n == 6)]
    need = {"L": 3 - start.count("L"), "R": 3 - start.count("R")}
    seq, order = list(start), []
    for n in bench:
        tilt = seq.count("L") - seq.count("R")  # the stack top leans toward the side with fewer spines
        if not need["L"] or not need["R"]:
            side = "L" if need["L"] else "R"
        elif tilt:
            side = "R" if tilt > 0 else "L"
        else:
            side = "R" if seq[-1] == "L" else "L"
        need[side] -= 1
        seq.append(side)
        order.append((n, side))
    return order


def demo():
    lot = lots.current()
    seq = list(lot.start)
    left = []
    for n, side in plan(lot):
        bx, by = BENCH[n]
        yaw = math.pi / 2 if side == "L" else -math.pi / 2  # turn the spine from the robot to that side
        z_rel = stack_top(seq) + DROP + HELD_LOW
        seq.append(side)
        zp = FLOOR + z_rel
        left += [
            Waypoint(1.2, (bx, by, BENCH_HOVER), OPEN_BOOK, f"over book {n}", yaw),
            Waypoint(1.5, (bx, by, TOP + GRIP_Z), OPEN_BOOK, "down", yaw),
            Waypoint(0.6, (bx, by, TOP + GRIP_Z), OPEN_BOOK, "settle", yaw),
            Waypoint(0.6, (bx, by, TOP + GRIP_Z), PINCH, "grip", yaw),
            Waypoint(0.9, (bx, by, BENCH_HOVER), PINCH, "lift", yaw),
            Waypoint(2.0, (PLACE_X, PLACE_Y + SIDE, TRAVEL), PINCH, f"carry, spine {side}", 0.0),
            Waypoint(1.0, (PLACE_X, PLACE_Y + SIDE, zp + SLIDE), PINCH, "down beside the case", 0.0),
            Waypoint(1.2, (PLACE_X, PLACE_Y, zp + SLIDE), PINCH, "in over the stack", 0.0),
            Waypoint(0.8, (PLACE_X, PLACE_Y, zp), PINCH, "lower", 0.0),
            Waypoint(0.5, (PLACE_X, PLACE_Y, zp), OPEN_BOOK, "release", 0.0),
            Waypoint(0.7, (PLACE_X, PLACE_Y, zp + 0.022), OPEN_BOOK, "up", 0.0),
            Waypoint(0.8, (PLACE_X, PLACE_Y + SIDE, zp + 0.022), OPEN_BOOK, "out", 0.0),
        ]
    left += [Waypoint(1.5, (0.22, 0.25, TRAVEL), OPEN_BOOK, "clear of the case")]
    busy = sum(w.duration for w in left)
    hy = HINGE_Y - LID_T  # the lid's outer face, upright
    zpush = ROOF + 0.018
    right = [
        Waypoint(busy - 3.0, (0.24, -0.12, TRAVEL), FINGER_CLOSED, "wait"),
        Waypoint(2.0, (CASE_X, hy - 0.075, zpush), FINGER_CLOSED, "beside the open lid"),
        Waypoint(2.0, (CASE_X, hy + 0.010, zpush), FINGER_CLOSED, "tip the lid over"),
        Waypoint(1.0, (CASE_X, hy - 0.050, zpush), FINGER_CLOSED, "back"),
        Waypoint(1.0, (0.24, -0.12, TRAVEL), FINGER_CLOSED, "away"),
        Waypoint(4.0, (0.24, -0.12, TRAVEL), FINGER_CLOSED, "wait while the lid settles"),
    ]
    return {"left": left, "right": right}
