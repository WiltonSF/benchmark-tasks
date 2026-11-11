"""Two spring push feeders side by side on the station table, each a grey slide lane 36 mm wide on a 30 mm riser with
a spring pusher at its near end: a black knob on a white rod, pulled back toward the robot, drives a white pusher face
that strikes the blank lying in front of it. Beside each knob, a white scale has yellow, orange, red and purple
stroke marks. Far along each lane is a green nest, then the lane ends over a dark red scrap bin.

The lane on the robot's right is the press feed: a dark grey guard roof covers it from just past the knob to two
yellow light-curtain posts, and a roof with a narrow viewing slot covers the rest of it, so its blue blank cannot
be reached. A white plate beside it reads 1.5. The lane on the robot's left is an open test slide with a blue test
blank from the same batch.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402  (before lots.py, so the friction reset runs before the lots settle)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
SUCCESS_HOLD_S = 1.0

# --- geometry (world m; lane parts have their origin at the near end of the floor, on the table) -----------------
X0 = 0.23  # near end of both lane floors
MAIN_Y, TEST_Y = -0.12, 0.12
FLOOR_H = 0.030  # lane floor top above the table
LANE_L = 0.42  # floor length: the lane ends at X0 + 0.42
INNER_W, WALL_T, WALL_H = 0.036, 0.006, 0.024  # channel width, side walls (height above the floor)
NEST = (0.28, 0.40)  # green nest along the floor, from the near end (m)
ROOF = (0.020, 0.250)  # guard roof over the press lane, from the near end
ROOF_T = 0.004
SLOT = 0.010  # viewing slot along the centre of the press lane's slotted roof, from the guard to the lane end
NEST_CAP_W = 0.016  # green caps on the walls marking the nest
KNOB_X = -0.020  # knob post centre at rest, from the near end (behind the floor)
FACE_FRONT = 0.070  # pusher face's front at rest
BLANK = (0.040, 0.030, 0.015)  # blank: length along the lane, width, height
BLANK_M = 0.03
BLANK_START = FACE_FRONT + 0.002 + BLANK[0] / 2  # blank centre at the start, from the near end

K, PUSHER_M, DAMP, OVER, BACK = 100.0, 0.05, 0.3, 0.015, 0.080  # spring N/m, kg, N s/m, overtravel, stroke limit
MARKS = (0.0263, 0.0323, 0.0395, 0.0484)  # stroke marks 1 to 4: knob pulled back this far (each 1.225 times the last)
TICK = 0.003  # tick width along the knob's path

CURTAIN_X = X0 + ROOF[1]  # light-curtain plane: the guard roof's far end
CURTAIN_Y = (MAIN_Y - 0.07, MAIN_Y + 0.07)  # the posts' inner faces bound the curtain across the press lane

LANE_MU = 0.10  # both lanes' surface: static = dynamic
FACTOR = 1.5  # the press lane's sliding friction is this many times the test slide's, for every batch
# Sliding friction against the test slide per lot (the press lane's is FACTOR times it). PhysX combines the blank's
# and the lane's coefficients as their mean, so each blank gets 2 * friction - LANE_MU.
LOT_FRICTION = {"A": 0.18, "B": 0.08, "C": 0.12}


def blank_mu(lot: str, press: bool) -> float:
    f = LOT_FRICTION[lot] * (FACTOR if press else 1.0)
    return 2 * f - LANE_MU

GREY, MID, DARK, WHITE, BLACK = (0.70, 0.70, 0.72), (0.45, 0.45, 0.48), (0.22, 0.22, 0.24), (0.92, 0.92, 0.92), (0.08, 0.08, 0.08)
BLUE, GREEN, SCRAP, YELLOW = (0.15, 0.30, 0.85), (0.15, 0.65, 0.25), (0.45, 0.10, 0.10), (0.95, 0.80, 0.10)
MARK_COLORS = ((0.95, 0.85, 0.10), (0.95, 0.50, 0.05), (0.85, 0.10, 0.10), (0.55, 0.15, 0.75))  # yellow, orange, red, purple


class Pusher(parts.Slider):
    """A spring slider pulled back along +axis, which can also overtravel OVER past its rest point."""

    @property
    def lower(self) -> float:
        return -OVER


def _lane(name, y, guarded):
    z = FLOOR_H
    nest_len = NEST[1] - NEST[0]
    shapes = [parts.box((LANE_L, INNER_W + 2 * WALL_T, FLOOR_H), at=(LANE_L / 2, 0, z / 2), color=GREY)]
    half = (INNER_W + 2 * WALL_T - SLOT) / 2  # each half of the slotted roof
    for side in (1, -1):
        shapes.append(parts.box((LANE_L, WALL_T, WALL_H), at=(LANE_L / 2, side * (INNER_W + WALL_T) / 2, z + WALL_H / 2),
                                color=MID))
        if guarded:  # slotted roof from the guard to the lane end, the nest marked green on top of it
            shapes.append(parts.box((LANE_L - ROOF[1], half, ROOF_T),
                                    at=((ROOF[1] + LANE_L) / 2, side * (SLOT + half) / 2, z + WALL_H + ROOF_T / 2), color=DARK))
            shapes.append(parts.box((nest_len, half, 0.002),
                                    at=((NEST[0] + NEST[1]) / 2, side * (SLOT + half) / 2, z + WALL_H + ROOF_T + 0.001),
                                    color=GREEN))
        else:  # the nest's marks: green caps on both walls, overhanging outward
            shapes.append(parts.box((nest_len, NEST_CAP_W, 0.002),
                                    at=((NEST[0] + NEST[1]) / 2, side * (INNER_W / 2 + NEST_CAP_W / 2), z + WALL_H + 0.001),
                                    color=GREEN))
    # Scale beside the knob's path, on the side toward the midline, with the four stroke marks.
    sy = -np.sign(y) * 0.034
    shapes.append(parts.box((0.100, 0.020, 0.002), at=(KNOB_X - 0.040, sy, 0.001), color=WHITE))
    for s, c in zip(MARKS, MARK_COLORS):
        shapes.append(parts.box((TICK, 0.020, 0.002), at=(KNOB_X - s, sy, 0.003), color=c))
    if guarded:
        roof_len = ROOF[1] - ROOF[0]
        shapes.append(parts.box((roof_len, INNER_W + 2 * WALL_T, ROOF_T), at=((ROOF[0] + ROOF[1]) / 2, 0, z + WALL_H + ROOF_T / 2),
                                color=DARK))
        for side in (1, -1):  # light-curtain posts at the roof's far end
            shapes.append(parts.box((0.012, 0.012, 0.150), at=(ROOF[1] + 0.006, side * 0.076, 0.075), color=YELLOW))
    return parts.Part(name, pos=(X0, y, TOP), friction=(LANE_MU, LANE_MU), shapes=shapes, joints=[
        Pusher("pusher", axis=(-1, 0, 0), travel=BACK, mass=PUSHER_M, spring=K, damping=DAMP, shapes=[
            parts.box((0.008, 0.028, 0.012), at=(FACE_FRONT - 0.004, 0, z + 0.009), color=WHITE),
            parts.box((FACE_FRONT - 0.008 - KNOB_X, 0.008, 0.008), at=((FACE_FRONT - 0.008 + KNOB_X) / 2, 0, z + 0.009),
                      color=WHITE),
            parts.cylinder(0.007, 0.042, at=(KNOB_X, 0, z + 0.024), color=BLACK, axis="z")])])


def _bin(name, y):
    x0, x1, h, t = X0 + LANE_L + 0.004, X0 + LANE_L + 0.090, 0.030, 0.004
    xc, w = (x0 + x1) / 2, 0.080
    return parts.Part(name, pos=(xc, y, TOP), shapes=[
        parts.box((x1 - x0, w, 0.003), at=(0, 0, 0.0015), color=SCRAP),
        parts.box((t, w, h), at=((x1 - x0 - t) / 2, 0, h / 2), color=SCRAP),
        parts.box((x1 - x0, t, h), at=(0, (w - t) / 2, h / 2), color=SCRAP),
        parts.box((x1 - x0, t, h), at=(0, -(w - t) / 2, h / 2), color=SCRAP)])


def _blank(name, y, press):
    mu = blank_mu("A", press)
    return parts.Part(name, pos=(X0 + BLANK_START, y, TOP + FLOOR_H + BLANK[2] / 2 + 0.0005), mass=BLANK_M,
                      friction=(mu, mu), shapes=[parts.box(BLANK, at=(0, 0, 0), color=BLUE)])


# A white plate beside the press lane reading "1.5" in black seven-segment figures (read from the robot's side:
# the figures' tops point away from the robot). Figures 28 mm tall, 14 mm wide, strokes 3 mm.
PLATE_C = (0.33, MAIN_Y - 0.062)
_DIGIT_H, _DIGIT_W, _STROKE = 0.028, 0.014, 0.003
_SEGS = {"1": "bc", "5": "afgcd"}


def _figure(ch, cx, cy):
    h, w, t = _DIGIT_H, _DIGIT_W, _STROKE
    out = []
    for seg in _SEGS[ch]:  # u: to the figure's right (-y), v: up (+x)
        u, v, horiz = {"a": (0, h / 2, 1), "b": (w / 2, h / 4, 0), "c": (w / 2, -h / 4, 0), "d": (0, -h / 2, 1),
                       "e": (-w / 2, -h / 4, 0), "f": (-w / 2, h / 4, 0), "g": (0, 0, 1)}[seg]
        size = (t, w + t, 0.002) if horiz else (h / 2 + t, t, 0.002)
        out.append(parts.box(size, at=(cx + v, cy - u, 0.003), color=BLACK))
    return out


FACTOR_PLATE = parts.Part("factor_plate", pos=(PLATE_C[0], PLATE_C[1], TOP), shapes=[
    parts.box((0.044, 0.064, 0.002), at=(0, 0, 0.001), color=WHITE),
    *_figure("1", 0, 0.020), parts.box((_STROKE + 0.001, _STROKE + 0.001, 0.002), at=(-_DIGIT_H / 2, 0.003, 0.003), color=BLACK),
    *_figure("5", 0, -0.016)])


PRESS_LANE = _lane("press_lane", MAIN_Y, True)
TEST_LANE = _lane("test_slide", TEST_Y, False)
PRESS_BIN, TEST_BIN = _bin("scrap_bin", MAIN_Y), _bin("test_bin", TEST_Y)
BLANK_P, BLANK_T = _blank("blank", MAIN_Y, True), _blank("test_blank", TEST_Y, False)
OBJECTS = [PRESS_LANE, TEST_LANE, PRESS_BIN, TEST_BIN, FACTOR_PLATE, BLANK_P, BLANK_T]

# Lots: the blanks' batch changes their sliding friction (not their look). The right press mark (index into MARKS):
# A purple (4), B orange (2), C red (3); on the test slide the landing mark is one lower. Lot A gets a zero move so
# every lot settles the same way after the friction is set.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", mark=3, place={"blank": BLANK_P.pos}),
    lots.Lot("B", mark=1, place={"blank": BLANK_P.pos}),
    lots.Lot("C", mark=2, place={"blank": BLANK_P.pos}),
)


@parts.on_reset
def _batch_friction(sim) -> None:
    """Give both blanks this lot's surface friction, and start a new trial's records."""
    import torch
    import warp as wp

    lot = lots.current().name
    base = sim.env.unwrapped
    for name in ("blank", "test_blank"):
        mu = blank_mu(lot, name == "blank")
        asset = base.scene[name]
        view = getattr(asset, "root_view", None) or asset.root_physx_view
        m = wp.to_torch(view.get_material_properties()).clone()
        m[:, :, 0] = mu
        m[:, :, 1] = mu
        ids = torch.arange(m.shape[0], dtype=torch.int32)
        view.set_material_properties(wp.from_torch(m.contiguous(), dtype=wp.float32), wp.from_torch(ids, dtype=wp.int32))
    _record["crossed"] = False
    _record["clear"] = True


# --- scoring ---------------------------------------------------------------------------------------------------
# Whole trial: did any part of either arm ever cross the light curtain (beyond CURTAIN_X, between the posts' inner
# faces)? Set by an on_step callback in the simulator; cleared only by the simulator's reset of a new trial.
# Released: no part of either gripper within 5 mm of the press blank. Offline (near_misses.py) there is no robot.
CLEAR = 0.005
MAX_TILT = 10.0  # degrees: the blank lies flat
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
_record = {"crossed": False, "clear": True}


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


@parts.on_step
def _watch_arms(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    blank = sim.states()["blank"]
    hulls, grip = [], []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n not in ARM_LINKS:
                continue
            m = np.eye(4)
            m[:3, :3] = _rot(poses[i, 3:])
            m[:3, 3] = poses[i, :3] - origin
            hs = analysis.world_hulls({n: m})
            hulls += hs
            if n in ("tip_left", "tip_right", "gripper"):
                grip += hs
    for _, h in hulls:
        v = np.asarray(h.vertices)
        if np.any((v[:, 0] > CURTAIN_X + EPS) & (v[:, 1] > CURTAIN_Y[0]) & (v[:, 1] < CURTAIN_Y[1])):
            _record["crossed"] = True
            break
    c, r = np.asarray(blank.pos), _rot(blank.quat_xyzw)
    _record["clear"] = analysis.hulls_to_box(grip, c, r, np.asarray(BLANK))[0] > CLEAR


def _x_extent(state) -> tuple[float, float]:
    """The blank's true footprint along the lane (its box turned by its yaw)."""
    r = _rot(state.quat_xyzw)
    half = np.abs(r) @ (np.asarray(BLANK) / 2)
    return float(state.pos[0] - half[0]), float(state.pos[0] + half[0])


def _in_nest(blank) -> bool:
    lo, hi = _x_extent(blank)
    on_floor = abs(blank.low[2] - (TOP + FLOOR_H)) <= 0.0015 + EPS
    in_lane = abs(blank.pos[1] - MAIN_Y) <= INNER_W / 2 + EPS
    flat = _rot(blank.quat_xyzw)[2, 2] >= math.cos(math.radians(MAX_TILT)) - EPS
    return bool(lo >= X0 + NEST[0] - EPS and hi <= X0 + NEST[1] + EPS and on_floor and in_lane and flat)


def success(objects):
    """The press blank lies wholly in the green nest, at rest, let go; no arm ever crossed the light curtain."""
    blank = objects["blank"]
    return bool(not _record["crossed"] and _record["clear"] and _in_nest(blank) and blank.resting())


def distance(objects):
    return float(abs(objects["blank"].pos[0] - (X0 + (NEST[0] + NEST[1]) / 2)))


def score(objects):
    """Pass/fail: 1.0 on success, otherwise 0."""
    return 1.0 if success(objects) else 0.0


# --- demo ------------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
GRIP_Z = FLOOR_H + 0.036 + TIP  # TCP height: tips 6 mm below the knob's top
WIDE = -0.020  # pads 40 mm apart
PINCH = -0.004  # pads 8 mm apart: about 3 mm into each side of the 14 mm knob
# The soft arm gives way under the spring's pull, so the demo commands more than the mark. Measured on the GPU
# (commanded pull -> knob stroke at release): 38.2 -> 26.1 mm, 53.4 -> 36.7 mm, 74.8 -> 52.6 mm.
PULL = (0.0385, 0.0471, 0.0572, 0.0691)  # commanded pull for marks 1 to 4
UP = 0.12


def _pull(y, mark, label):
    """Grip the knob, pull it back to the mark (commanding extra for the arm's give), let go, lift clear."""
    kx = X0 + KNOB_X
    back = kx - PULL[mark]
    return [
        Waypoint(1.5, (kx, y, TOP + UP), WIDE, f"over the {label} knob"),
        Waypoint(1.2, (kx, y, TOP + GRIP_Z), WIDE, "down round the knob"),
        Waypoint(0.6, (kx, y, TOP + GRIP_Z), PINCH, "grip"),
        Waypoint(2.0, (back, y, TOP + GRIP_Z), PINCH, "pull back to the mark"),
        Waypoint(0.6, (back, y, TOP + GRIP_Z), PINCH, "hold"),
        Waypoint(0.3, (back, y, TOP + GRIP_Z), WIDE, "let go"),
        Waypoint(1.0, (back, y, TOP + UP), WIDE, "up"),
    ]


def demo():
    """Left arm: fire the test blank at the orange mark (2) and watch where it stops: past the nest, in it, or short.
    The test-slide mark that lands is one lower if it went past, the same if it landed, one higher if it stopped short.
    Then right arm: fire the press blank one mark above that, since the press lane drags 1.5 times as much and each
    mark stores 1.5 times the energy of the one before. The choice is precomputed per lot, from what the test shot
    shows in that lot."""
    lot = lots.current()
    left = _pull(TEST_Y, 1, "test slide")
    wait = sum(w.duration for w in left) + 2.5
    right = [Waypoint(wait, (0.15, -0.30, TOP + 0.20), FINGER_OPEN, "wait for the test shot")]
    right += _pull(MAIN_Y, lot.mark, "press feed")
    right.append(Waypoint(3.0, (X0 + KNOB_X - 0.06, MAIN_Y, TOP + UP), WIDE, "stand clear"))
    return {"left": left, "right": right}
