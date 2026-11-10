"""On the station table, in front of the arms and a little to the robot's right: a grey cage with a barred roof and a small
square doorway at floor level in the face toward the robot. A vertical sliding door closes the doorway; a handle on
top of the door stands above the cage wall. A grey cat crouches at the back of the cage. To the robot's left, on the
table, lies a blue transfer box with a carry handle on top, open at one end; a yellow band on its roof and a yellow rim
mark the open end. Which way the open end faces at the start changes between trials.
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

GREY, DARK, STEEL = (0.62, 0.64, 0.67), (0.25, 0.26, 0.28), (0.45, 0.47, 0.50)
BLUE, BLACK, YELLOW, RED, CAT = (0.18, 0.38, 0.75), (0.06, 0.06, 0.07), (0.95, 0.80, 0.10), (0.80, 0.12, 0.10), (0.48, 0.46, 0.44)

# --- cage ---------------------------------------------------------------------------------------------------------
# Part frame: origin on the table under the cage's centre, x away from the robot, z up. 150 x 150 mm outside, walls
# 5 mm, 80 mm tall, a 5 mm floor. The doorway is 50 mm wide and 50 mm tall, from the floor's top up, centred in the
# near wall.
CAGE_XY = (0.40, -0.06)
SIDE, WALL, HEIGHT, FLOOR_T = 0.150, 0.005, 0.080, 0.005
DOOR_W, DOOR_H = 0.050, 0.050
ROOF_BAR = 0.004
FACE_X = CAGE_XY[0] - SIDE / 2  # the near face of the cage (world x)
SILL = FLOOR_T  # doorway bottom above the table
DOOR_PLATE = (0.003, 0.064, 0.055)  # the door: 3 mm plate, 64 mm wide, 55 mm tall, just inside the near wall
DOOR_X = -SIDE / 2 + WALL + DOOR_PLATE[0] / 2 + 0.0005  # part frame
STEM = (0.003, 0.010, 0.032)  # from the door's top (60 mm) up to the handle bar
BAR = (0.012, 0.050, 0.008)  # handle bar, 92 to 100 mm up
BAR_Z = 0.096
TRAVEL = 0.052  # door travel: raised fully, its bottom is 57 mm up, above the doorway's 55 mm top
CATCH_AT = 0.046  # the catch takes the door once its bottom is 46 mm up (51 mm above the table)
OPEN_FAULT = 0.020  # the doorway counts as open once the door's bottom is more than 20 mm up
DOOR_SPRING = 10.0  # N/m: with gravity, the door drops shut when let go before the catch takes it

_h = SIDE / 2
_cage_shapes = [
    parts.box((SIDE, SIDE, FLOOR_T), at=(0.0, 0.0, FLOOR_T / 2), color=GREY),
    parts.box((WALL, SIDE, HEIGHT), at=(_h - WALL / 2, 0.0, HEIGHT / 2), color=GREY),  # far wall
    parts.box((SIDE - 2 * WALL, WALL, HEIGHT), at=(0.0, _h - WALL / 2, HEIGHT / 2), color=GREY),  # left wall
    parts.box((SIDE - 2 * WALL, WALL, HEIGHT), at=(0.0, -_h + WALL / 2, HEIGHT / 2), color=GREY),  # right wall
    # Near wall around the doorway: two side pieces and a lintel.
    parts.box((WALL, _h - DOOR_W / 2, HEIGHT), at=(-_h + WALL / 2, (_h + DOOR_W / 2) / 2, HEIGHT / 2), color=GREY),
    parts.box((WALL, _h - DOOR_W / 2, HEIGHT), at=(-_h + WALL / 2, -(_h + DOOR_W / 2) / 2, HEIGHT / 2), color=GREY),
    parts.box((WALL, DOOR_W, HEIGHT - SILL - DOOR_H), at=(-_h + WALL / 2, 0.0, (HEIGHT + SILL + DOOR_H) / 2),
              color=GREY),
    # A barred roof: four 4 mm bars front to back across the top, clear of the door's stem.
    *[parts.box((SIDE, ROOF_BAR, ROOF_BAR), at=(0.0, y, HEIGHT + ROOF_BAR / 2), color=DARK)
      for y in (-0.054, -0.018, 0.018, 0.054)],
    # The catch: a red pin on the wall's top beside the door's stem (scored by the door's height, not its look).
    parts.box((0.010, 0.008, 0.008), at=(-_h + WALL / 2, 0.012, HEIGHT + 0.004), color=RED),
]
CAGE = parts.Part(
    "cage",
    pos=(*CAGE_XY, TOP),
    shapes=_cage_shapes,
    joints=[parts.Slider(
        "door",
        shapes=[
            parts.box(DOOR_PLATE, at=(DOOR_X, 0.0, SILL + DOOR_PLATE[2] / 2), color=STEEL),
            parts.box(STEM, at=(DOOR_X, 0.0, SILL + DOOR_PLATE[2] + STEM[2] / 2), color=STEEL),
            parts.box(BAR, at=(DOOR_X, 0.0, BAR_Z), color=DARK),
        ],
        axis=(0.0, 0.0, 1.0), travel=TRAVEL, mass=0.04, spring=DOOR_SPRING, damping=0.5)],
)

CAT_SIZE = (0.050, 0.070, 0.040)
CAT_BOX = parts.Part("cat", pos=(CAGE_XY[0] + 0.035, CAGE_XY[1], TOP + FLOOR_T), mass=0.3,
                     shapes=[parts.box(CAT_SIZE, at=(0.0, 0.0, CAT_SIZE[2] / 2), color=CAT),
                             parts.box((0.012, 0.030, 0.012), at=(-0.031, 0.0, 0.034), color=CAT)])  # head

# --- transfer box -------------------------------------------------------------------------------------------------
# Part frame: origin on the table under the box's centre, x along its length with the open end at +x, z up.
BOX_L, BOX_W, BOX_H, BOX_T = 0.100, 0.080, 0.066, 0.004
HANDLE = (0.060, 0.010, 0.012)  # on the roof, along the box
HANDLE_TOP = BOX_H + HANDLE[2]  # 78 mm
BAND = 0.012  # yellow band across the roof at the open end
RIM = 0.006  # yellow rim round the open end
MOUTH = (BOX_W - 2 * BOX_T, BOX_H - 2 * BOX_T)  # 72 mm wide, 58 mm tall inside
BOX_MASS = 0.25
_box_shapes = [
    parts.box((BOX_L, BOX_W, BOX_T), at=(0.0, 0.0, BOX_T / 2), color=BLUE),  # floor
    parts.box((BOX_L - BAND, BOX_W, BOX_T), at=(-BAND / 2, 0.0, BOX_H - BOX_T / 2), color=BLUE),  # roof
    parts.box((BAND, BOX_W, BOX_T), at=(BOX_L / 2 - BAND / 2, 0.0, BOX_H - BOX_T / 2), color=YELLOW),  # roof band
    parts.box((BOX_L, BOX_T, BOX_H - 2 * BOX_T), at=(0.0, BOX_W / 2 - BOX_T / 2, BOX_H / 2), color=BLUE),
    parts.box((BOX_L, BOX_T, BOX_H - 2 * BOX_T), at=(0.0, -BOX_W / 2 + BOX_T / 2, BOX_H / 2), color=BLUE),
    parts.box((BOX_T, BOX_W - 2 * BOX_T, BOX_H - 2 * BOX_T), at=(-BOX_L / 2 + BOX_T / 2, 0.0, BOX_H / 2),
              color=BLUE),  # closed back
    parts.box(HANDLE, at=(-0.005, 0.0, BOX_H + HANDLE[2] / 2), color=BLACK),
    # A yellow rim round the open end: the ends of the two side walls and the floor.
    *[parts.box((RIM, BOX_T, BOX_H - 2 * BOX_T), at=(BOX_L / 2 - RIM / 2, sy * (BOX_W / 2 - BOX_T / 2), BOX_H / 2),
                color=YELLOW) for sy in (-1, 1)],
    parts.box((RIM, BOX_W, BOX_T), at=(BOX_L / 2 - RIM / 2, 0.0, BOX_T / 2), color=YELLOW),
]
BOX_SPOT = (0.30, 0.20)
MOUTH_LEFT, MOUTH_RIGHT, MOUTH_NEAR = math.pi / 2, -math.pi / 2, math.pi  # box yaw: where its open end points
BOX = parts.Part("transfer_box", pos=(*BOX_SPOT, TOP), yaw=MOUTH_LEFT, mass=BOX_MASS, shapes=_box_shapes)

OBJECTS = [CAGE, CAT_BOX, BOX]

# Lot A (the screenshot): the box's open end points left. Lot B: right. Lot C: away from the robot, toward the far
# side of the table (already the way it must face the cage). The box starts on the same spot in every lot; lot A gives
# it a zero move so every lot settles alike.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"transfer_box": BOX.pos}),
    lots.Lot("B", yaw={"transfer_box": MOUTH_RIGHT}),
    lots.Lot("C", yaw={"transfer_box": MOUTH_NEAR}),
)

# --- scoring ------------------------------------------------------------------------------------------------------
EPS = 1e-6
DOCK_GAP = 0.006  # the open end within 6 mm of the cage face
DOCK_YAW = math.radians(5.0)
DOCK_SIDE = (MOUTH[0] - DOOR_W) / 2  # 11 mm: the doorway stays inside the mouth
ON_TABLE = 0.002
MAX_TILT = math.radians(5.0)
CAT_MOVE = 0.020
CLEAR = 0.005
RELEASE_LINKS = ("tip_left", "tip_right", "gripper", "link1", "link2", "link3", "link4", "link5")  # every moving link
SUCCESS_HOLD_S = 1.0


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _door(objects) -> float:
    return float(objects["cage"].joints["door"])


def docked(objects) -> bool:
    """The box stands on the table, square to the cage face within 5 degrees, its open end within 6 mm of the face
    (and not through it), and the doorway wholly inside the open end seen from the front."""
    b = objects["transfer_box"]
    r = _rot(b.quat_xyzw)
    if math.acos(float(np.clip(r[2, 2], -1.0, 1.0))) > MAX_TILT + EPS or b.low[2] > TOP + ON_TABLE + EPS:
        return False
    if abs(math.atan2(r[1, 0], r[0, 0])) > DOCK_YAW + EPS:  # the open end (+x of the box) points at the cage
        return False
    mouth = np.asarray(b.pos) + r @ np.array([BOX_L / 2, 0.0, 0.0])
    gap = FACE_X - float(mouth[0])
    return bool(-0.001 - EPS <= gap <= DOCK_GAP + EPS and abs(float(mouth[1]) - CAGE_XY[1]) <= DOCK_SIDE + EPS)


# Whole-trial record, cleared at each scene reset: was the doorway open (door more than 20 mm up) at any moment while
# the box was not docked, and did the cat move more than 20 mm?
_record = {"fault": False}
_catch = {"caught": False}
_grip = {"clear": True}


def _update(objects) -> None:
    if _door(objects) > OPEN_FAULT + EPS and not docked(objects):
        _record["fault"] = True
    cat = objects["cat"]
    start = np.asarray(CAT_BOX.pos, float)
    if np.linalg.norm(np.asarray(cat.pos, float) - start) > CAT_MOVE + EPS:
        _record["fault"] = True


@parts.on_reset
def _reset(sim) -> None:
    _record["fault"] = False
    _catch["caught"] = False
    sim.set_target("cage", "door", 0.0)


@parts.on_step
def _step(sim) -> None:
    from yam_arena import analysis

    st = sim.states()
    # The catch: once the door's bottom passes 46 mm, a spring pin holds it up (modelled by moving the door spring's
    # rest point above its top stop, so the spring holds it there against gravity).
    if not _catch["caught"] and float(st["cage"].joints["door"]) >= CATCH_AT - EPS:
        _catch["caught"] = True
    sim.set_target("cage", "door", 0.15 if _catch["caught"] else 0.0)
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    b = st["transfer_box"]
    boxes = BOX.world_boxes({"base": (np.asarray(b.pos), _rot(b.quat_xyzw))})
    door_pos, door_q = st["cage"].links["door"]
    boxes += CAGE.world_boxes({"door": (np.asarray(door_pos), _rot(door_q))})
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in RELEASE_LINKS:
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for _, c, r, s in boxes)


def _settled(objects) -> bool:
    return bool(docked(objects) and objects["transfer_box"].resting() and _grip["clear"])


def _door_up(objects) -> bool:
    """Up on its catch: let go with its bottom 46 mm or more above the sill (the catch takes it there)."""
    return bool(_door(objects) >= CATCH_AT - EPS)


def success(objects):
    """Docked, the door up on its catch, everything let go and still, and no fault during the trial."""
    _update(objects)
    return bool(not _record["fault"] and _settled(objects) and _door_up(objects))


def distance(objects):
    b = objects["transfer_box"]
    return float(np.linalg.norm(np.asarray(b.pos[:2]) - np.array((FACE_X - BOX_L / 2, CAGE_XY[1]))))


def score(objects):
    """1.0 on success; 0.5 docked and let go with the door still down; 0 after any fault or otherwise."""
    _update(objects)
    if _record["fault"] or not _settled(objects):
        return 0.0
    return 1.0 if _door_up(objects) else 0.5


# --- demo ---------------------------------------------------------------------------------------------------------
# Fixed by the lot's layout (the cameras show which way the box's open end points). The left arm takes the box by its
# roof handle, turns it so the open end faces the cage, sets it down against the cage face and lets go; then the
# right arm lifts the door by its handle until the catch takes it, and lets go.
HANDLE_GRIP_Z = TOP + HANDLE_TOP + 0.003  # finger tips 10 mm below the handle's top, 2 mm above the roof
OPEN_HANDLE = -0.020  # pads 40 mm apart around the 10 mm handle
PINCH_HANDLE = -0.003  # closed 2 mm past the handle's faces
LIFT_Z = TOP + 0.135
SET_GAP = -0.004  # aimed 4 mm into the face: the loaded arm falls about 6 mm short in x, and the face stops the box
DROP = 0.004
DOCK_X = FACE_X - SET_GAP - BOX_L / 2 - 0.005  # the TCP over the handle's centre (5 mm behind the box's centre)
DOOR_GRIP_Z = TOP + BAR_Z + BAR[2] / 2 + 0.003  # tips 10 mm below the bar's top
OPEN_BAR, PINCH_BAR = -0.016, -0.004
DOOR_WORLD_X = CAGE_XY[0] + DOOR_X
BACK_X = FACE_X - BOX_L  # the box's closed back, docked
PUSH_Z = TOP + 0.070  # closed finger tips 9 mm below the top of the box's back wall
PUSH_PAST = 0.002  # TCP target: the closed fingers' front face (about 8 mm ahead of the TCP) 10 mm past the docked back


def demo():
    lot = lots.current()
    yaw0 = lot.yaw_of("transfer_box")
    bx, by, _ = lot.pos("transfer_box")
    hx, hy = bx - 0.005 * math.cos(yaw0), by - 0.005 * math.sin(yaw0)  # the handle's centre
    grip_yaw = yaw0  # fingers across the handle; turning the wrist back to 0 points the open end at the cage
    dy = CAGE_XY[1]
    left = []
    dock_x = DOCK_X
    if abs(abs(yaw0) - math.pi) < 0.1:
        # Open end toward the robot: a half turn is past the wrist's range, so turn it a quarter turn on the spot first
        # (open end left, as in lot A), set it down and take it again.
        left += [
            Waypoint(1.5, (hx, hy, LIFT_Z), OPEN_HANDLE, "over the handle", 0.0),
            Waypoint(1.2, (hx, hy, HANDLE_GRIP_Z), OPEN_HANDLE, "down", 0.0),
            Waypoint(0.5, (hx, hy, HANDLE_GRIP_Z), OPEN_HANDLE, "settle", 0.0),
            Waypoint(0.6, (hx, hy, HANDLE_GRIP_Z), PINCH_HANDLE, "grip", 0.0),
            Waypoint(0.8, (hx, hy, HANDLE_GRIP_Z + 0.02), PINCH_HANDLE, "lift a little", 0.0),
            Waypoint(1.5, (bx, by, HANDLE_GRIP_Z + 0.02), PINCH_HANDLE, "quarter turn", -math.pi / 2),
            Waypoint(0.8, (bx, by, HANDLE_GRIP_Z + DROP), PINCH_HANDLE, "set down", -math.pi / 2),
            Waypoint(0.5, (bx, by, HANDLE_GRIP_Z + DROP), OPEN_HANDLE, "let go", -math.pi / 2),
            Waypoint(0.8, (bx, by, LIFT_Z), OPEN_HANDLE, "up", -math.pi / 2),
        ]
        yaw0 = MOUTH_LEFT
        dock_x = DOCK_X + 0.003  # regripped, the box sits 3 mm further back in the fingers (measured)
        hx, hy, grip_yaw = bx - 0.005, by + 0.001, yaw0  # where the handle lands after the turn (measured)
    left += [
        Waypoint(1.5, (hx, hy, LIFT_Z), OPEN_HANDLE, "over the handle", grip_yaw),
        Waypoint(1.2, (hx, hy, HANDLE_GRIP_Z), OPEN_HANDLE, "down", grip_yaw),
        Waypoint(0.5, (hx, hy, HANDLE_GRIP_Z), OPEN_HANDLE, "settle", grip_yaw),
        Waypoint(0.6, (hx, hy, HANDLE_GRIP_Z), PINCH_HANDLE, "grip", grip_yaw),
        Waypoint(1.0, (hx, hy, LIFT_Z), PINCH_HANDLE, "lift", grip_yaw),
        Waypoint(2.5, (dock_x - 0.03, dy, LIFT_Z), PINCH_HANDLE, "carry, open end to the cage", 0.0),
        Waypoint(1.2, (dock_x - 0.03, dy, HANDLE_GRIP_Z + DROP), PINCH_HANDLE, "lower", 0.0),
        Waypoint(1.2, (dock_x, dy, HANDLE_GRIP_Z + DROP), PINCH_HANDLE, "up to the face", 0.0),
        Waypoint(0.5, (dock_x, dy, HANDLE_GRIP_Z + DROP), OPEN_HANDLE, "release", 0.0),
        Waypoint(1.0, (dock_x - 0.02, dy, LIFT_Z - 0.02), OPEN_HANDLE, "up", 0.0),
        # Nudge the box's closed back so its open end sits flush on the face (the loaded arm stops a few mm short).
        Waypoint(1.0, (BACK_X - 0.03, dy, LIFT_Z - 0.02), FINGER_CLOSED, "behind the box", 0.0),
        Waypoint(0.8, (BACK_X - 0.03, dy, PUSH_Z), FINGER_CLOSED, "down behind it", 0.0),
        Waypoint(1.2, (BACK_X + PUSH_PAST, dy, PUSH_Z), FINGER_CLOSED, "push it flush", 0.0),
        Waypoint(0.6, (BACK_X - 0.02, dy, PUSH_Z), FINGER_CLOSED, "back off", 0.0),
        Waypoint(1.5, (0.22, 0.22, LIFT_Z), OPEN_HANDLE, "away"),
    ]
    busy = sum(w.duration for w in left)
    right = [
        Waypoint(busy - 1.0, (0.24, -0.20, LIFT_Z), PINCH_BAR, "wait"),
        Waypoint(1.5, (DOOR_WORLD_X, dy, DOOR_GRIP_Z + 0.03), OPEN_BAR, "over the door handle", math.pi / 2),
        Waypoint(1.0, (DOOR_WORLD_X, dy, DOOR_GRIP_Z), OPEN_BAR, "down", math.pi / 2),
        Waypoint(0.6, (DOOR_WORLD_X, dy, DOOR_GRIP_Z), PINCH_BAR, "grip", math.pi / 2),
        Waypoint(2.0, (DOOR_WORLD_X, dy, DOOR_GRIP_Z + TRAVEL + 0.004), PINCH_BAR, "raise the door", math.pi / 2),
        Waypoint(0.5, (DOOR_WORLD_X, dy, DOOR_GRIP_Z + TRAVEL + 0.004), OPEN_BAR, "let go: the catch holds it",
                 math.pi / 2),
        Waypoint(1.0, (DOOR_WORLD_X - 0.04, dy, DOOR_GRIP_Z + TRAVEL + 0.02), OPEN_BAR, "back", math.pi / 2),
        Waypoint(1.0, (0.24, -0.20, LIFT_Z), OPEN_BAR, "away"),
        Waypoint(3.0, (0.24, -0.20, LIFT_Z), OPEN_BAR, "wait"),
    ]
    return {"left": left, "right": right}
