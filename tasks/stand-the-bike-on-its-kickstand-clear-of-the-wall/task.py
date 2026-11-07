"""On the station table, in front of the arms: a model bicycle about 30 cm long, standing on its two wheels with
its length running left to right, front wheel on the robot's left. Its side stand is folded up, pointing back
along the frame, on the side facing the robot. The bike leans back against a grey wall behind it, its handlebar
resting on the wall.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m). Bike frame: origin on the table under
# the middle of the wheels' contact line, x across the bike (toward the wall), y along it (front +y), z up.
BIKE_XY = (0.270, -0.030)
WHEEL_R, WHEEL_W, WHEEL_Y = 0.060, 0.012, 0.110
TUBE = 0.012
SADDLE = (0.025, 0.060, 0.012)
SADDLE_AT = (0.0, -0.050, 0.150)
BAR = (0.120, 0.012, 0.012)  # handlebar, across the bike
BAR_AT = (0.0, 0.100, 0.148)
BIKE_MASS = 0.70

# Side stand: a cranked leg on the robot's side of the frame, pivoting on an axis along x at the bottom bracket.
STAND_PIVOT = (-0.010, -0.030, 0.050)
CRANK = 0.050  # the leg's crank out toward the robot
LEG = 0.040  # the leg, from the crank's end
LEG_T = 0.008
DEPLOYED = math.pi / 2  # rad: the leg points straight down (against its stop)
SNAP = math.radians(45.0)  # detent: past this it snaps down, short of it back up

RED, BLACK, GREY, ORANGE = (0.80, 0.12, 0.10), (0.06, 0.06, 0.06), (0.62, 0.63, 0.65), (1.00, 0.55, 0.00)

_px, _py, _pz = STAND_PIVOT
_stand = parts.Hinge(
    "stand",
    shapes=[
        parts.box((CRANK, LEG_T, LEG_T), at=(_px - CRANK / 2, _py, _pz), color=ORANGE),
        parts.box((LEG_T, LEG, LEG_T), at=(_px - CRANK + LEG_T / 2, _py - LEG / 2, _pz), color=ORANGE),
    ],
    pivot=STAND_PIVOT,
    axis=(1.0, 0.0, 0.0),  # positive swings the leg from pointing back (-y) to pointing down
    lower=0.0,
    upper=DEPLOYED,
    mass=0.02,
    spring=0.05,
    damping=0.002,
)
BIKE = parts.Part(
    "bike",
    pos=(*BIKE_XY, TOP),
    shapes=[
        parts.cylinder(WHEEL_R, WHEEL_W, at=(0.0, -WHEEL_Y, WHEEL_R), color=BLACK, axis="x"),
        parts.cylinder(WHEEL_R, WHEEL_W, at=(0.0, WHEEL_Y, WHEEL_R), color=BLACK, axis="x"),
        parts.box((TUBE, 0.200, TUBE), at=(0.0, 0.020, 0.130), color=RED),  # top tube
        parts.box((TUBE, 0.150, TUBE), at=(0.0, -0.035, 0.056), color=RED),  # chain stay to the bottom bracket
        parts.box((TUBE, TUBE, 0.090), at=(0.0, -0.030, 0.100), color=RED),  # seat tube
        parts.box((TUBE, TUBE, 0.085), at=(0.0, 0.100, 0.1015), color=RED),  # head tube and fork
        parts.box((TUBE, 0.130, TUBE), at=(0.0, 0.035, 0.090), color=RED),  # down tube
        parts.box(SADDLE, at=SADDLE_AT, color=BLACK),
        parts.box(BAR, at=BAR_AT, color=BLACK),
    ],
    joints=[_stand],
    mass=BIKE_MASS,
)

WALL_FACE = BIKE_XY[0] + 0.090  # the wall's face, toward the robot
WALL = Box("wall", pos=(WALL_FACE + 0.010, 0.0, TOP + 0.125), size=(0.020, 0.500, 0.250), color=GREY)
OBJECTS = [BIKE, WALL]


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _wall_lean() -> float:
    """Lean (rad, top toward the wall) at which the handlebar's end touches the wall, the bike pivoting on its
    tyres' wall-side edge."""
    edge = np.array([WHEEL_W / 2, 0.0, 0.0])
    corner = np.array([BAR_AT[0] + BAR[0] / 2, 0.0, BAR_AT[2] + BAR[2] / 2])
    lo, hi = 0.0, 0.5
    for _ in range(60):
        a = (lo + hi) / 2
        x = BIKE_XY[0] + edge[0] + (_rot_y(a) @ (corner - edge))[0]
        lo, hi = (a, hi) if x < WALL_FACE - 0.0005 else (lo, a)
    return lo


LEAN0 = _wall_lean()


def _lean_pose(lean: float):
    """Body origin and quaternion (x, y, z, w) with the bike leaning ``lean`` toward the wall on its tyres' edge."""
    edge = np.array([BIKE_XY[0] + WHEEL_W / 2, BIKE_XY[1], TOP])
    origin = edge + _rot_y(lean) @ (np.array([*BIKE_XY, TOP]) - edge) + [0.0, 0.0, 0.0005]
    return origin, np.array([0.0, math.sin(lean / 2), 0.0, math.cos(lean / 2)])


# -- scoring ---------------------------------------------------------------------------------------------------

EPS = 1e-6
DOWN_DEG = 35.0  # the bike lay down: leaned more than 35 degrees at any time
LEAN_MIN, LEAN_MAX = 4.0, 14.0  # standing on its stand: leaning 4 to 14 degrees toward the stand side
ON_TOL = 0.002  # tyres and foot within 2 mm of the table
WALL_CLEAR = 0.030
_H = {}


def _clear():
    _H.clear()
    _H.update(down=False)


_clear()


def _quat_matrix(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _lean_toward_robot(bike) -> float:
    """Degrees the bike leans toward the robot (the stand side); negative leans toward the wall."""
    up = _quat_matrix(bike.quat_xyzw)[:, 2]
    return math.degrees(math.atan2(-up[0], up[2]))


def _tyre_low(bike) -> float:
    """Lowest point of the two tyres (world z)."""
    r = _quat_matrix(bike.quat_xyzw)
    p = np.asarray(bike.pos)
    a = np.linspace(0, 2 * math.pi, 73)
    lows = []
    for wy in (-WHEEL_Y, WHEEL_Y):
        pts = [np.array([sx * WHEEL_W / 2, wy + WHEEL_R * math.cos(t), WHEEL_R + WHEEL_R * math.sin(t)])
               for sx in (-1, 1) for t in a]
        lows.append(min((p + r @ q)[2] for q in pts))
    return max(lows)  # both tyres must be down: the higher of the two lowest points


def _foot_low(bike) -> float:
    """Lowest corner of the stand's leg (world z)."""
    boxes = BIKE.world_boxes({"stand": (np.asarray(bike.links["stand"][0]), _quat_matrix(bike.links["stand"][1]))})
    return min(float((c - np.abs(r) @ (s / 2))[2]) for _, c, r, s in boxes)


# Grippers. The scorer is given only the objects, so an on_step callback reads both arms' joints each control step
# and keeps each gripper's pose (from the arm's own kinematics) and half opening. Each finger is modelled as a box
# 10 mm thick (along the closing direction, outside the pad face), 20 mm wide and 45 mm long back from its tip. The
# bike is released when every point of every finger is more than 5 mm from every shape of the bike. Offline there are
# no grippers, so every state counts as released.
BASES = {"left_robot": (0.0, 0.305, 0.75), "right_robot": (0.0, -0.305, 0.75)}  # station arm bases (world)
FINGER_T, FINGER_W, FINGER_L = 0.010, 0.020, 0.045
HANDS_OFF = 0.005
_GRIP: dict = {}


@parts.on_step
def _read_grippers(sim) -> None:
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    base = sim.env.unwrapped
    for name, bpos in BASES.items():
        robot = base.scene[name]
        arm_ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
        fin_ids = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        q = robot.data.joint_pos.torch[0].detach().cpu().numpy().astype(float)
        pos, rot = ik.tcp_pose(q[arm_ids], bpos)
        _GRIP[name] = (pos, rot, float(np.max(np.abs(q[fin_ids]))))


def _finger_points(pos, rot, half) -> np.ndarray:
    """Points 2-3 mm apart through both fingers' boxes (world). Gripper frame: x the closing direction, z the
    approach axis (the pad tips are 13 mm past the TCP)."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    gx = np.linspace(half, half + FINGER_T, 5)
    gy = np.linspace(-FINGER_W / 2, FINGER_W / 2, 9)
    gz = np.linspace(PAD_TIP_PAST_TCP - FINGER_L, PAD_TIP_PAST_TCP, 19)
    local = np.array(np.meshgrid(gx, gy, gz, indexing="ij")).reshape(3, -1).T
    both = np.concatenate([local, local * [-1.0, 1.0, 1.0]])
    return np.asarray(pos) + both @ np.asarray(rot).T


def _bike_boxes(bike):
    poses = {"base": (np.asarray(bike.pos, float), _quat_matrix(bike.quat_xyzw))}
    poses["stand"] = (np.asarray(bike.links["stand"][0], float), _quat_matrix(bike.links["stand"][1]))
    return BIKE.world_boxes(poses)


def hands_gap(bike) -> float:
    """Smallest distance (m) from any finger to any shape of the bike; infinite with no grippers."""
    if not _GRIP:
        return math.inf
    pts = np.concatenate([_finger_points(*g) for g in _GRIP.values()])
    gap = math.inf
    for _, c, r, size in _bike_boxes(bike):
        local = (pts - c) @ r
        d = np.linalg.norm(np.maximum(np.abs(local) - size / 2, 0.0), axis=1)
        gap = min(gap, float(d.min()))
    return gap


def released(bike) -> bool:
    return hands_gap(bike) > HANDS_OFF + EPS


def _update(objects) -> None:
    if abs(tilt.tilt_of(objects["bike"].quat_xyzw)) > DOWN_DEG:
        _H["down"] = True


def success(objects):
    """On its stand, leaning toward the stand side, clear of the wall, released, still, and never lay down."""
    _update(objects)
    bike = objects["bike"]
    lean = _lean_toward_robot(bike)
    return bool(
        not _H["down"]
        and LEAN_MIN - EPS <= lean <= LEAN_MAX + EPS
        and abs(_tyre_low(bike) - TOP) <= ON_TOL + EPS
        and abs(_foot_low(bike) - TOP) <= ON_TOL + EPS
        and bike.high[0] <= WALL_FACE - WALL_CLEAR + EPS
        and bike.resting()
        and released(bike)
    )


def score(objects):
    """Pass/fail: 1.0 as success(), else 0."""
    return 1.0 if success(objects) else 0.0


@parts.on_reset
def _new_episode(sim):
    """A new episode: clear the record, fold the stand and lean the bike back against the wall."""
    import torch

    _clear()
    _detent["target"] = None
    sim.set_target("bike", "stand", -0.3)
    base = sim.env.unwrapped
    bike = base.scene["bike"]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    p, q = _lean_pose(LEAN0)
    pose = torch.tensor([[*(p + origin), *q]], dtype=torch.float32, device=base.device)
    bike.write_root_pose_to_sim_index(root_pose=pose, env_ids=[0])
    bike.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=base.device), env_ids=[0])


_detent = {"target": None}


@parts.on_step
def _stand_detent(sim):
    """Side-stand spring: past 45 degrees it snaps down against its stop, short of that back up."""
    q = sim.states()["bike"].joints["stand"]
    target = DEPLOYED + 0.3 if q >= SNAP else -0.3
    if target != _detent["target"]:
        _detent["target"] = target
        sim.set_target("bike", "stand", target)


# -- demo ------------------------------------------------------------------------------------------------------

TIP = 0.013
HIGH = TOP + 0.16


def _world(point, lean=None):
    """A bike-frame point in the world with the bike at its starting lean."""
    p, q = _lean_pose(LEAN0 if lean is None else lean)
    return p + _quat_matrix(q) @ np.asarray(point, float)


def demo():
    """Right arm only. Fingers closed, it presses the folded stand's leg down from above, past its detent, and draws
    back. Then, fingers open across the saddle (one finger each side), it draws the saddle toward the robot, so the
    bike tips back through upright onto its stand, then backs off the saddle and rises clear at once. Straight-down grips only."""
    W_ = tilt.Waypoint
    # The leg's end, 30 mm from the pivot along the folded leg.
    leg_end = _world((_px - CRANK + LEG_T / 2, _py - 0.030, _pz + LEG_T / 2))
    press_top = leg_end + [0.0, 0.0, TIP + 0.004]
    press_low = np.array([leg_end[0], leg_end[1] + 0.012, TOP + 0.022 + TIP])  # drive it well past 45 degrees
    right = [
        W_(1.5, (press_top[0], press_top[1], HIGH), FINGER_CLOSED, "over the stand"),
        W_(1.2, tuple(press_top), FINGER_CLOSED, "onto the leg"),
        W_(2.0, tuple(press_low), FINGER_CLOSED, "press it down past the catch"),
        W_(0.5, tuple(press_low), FINGER_CLOSED, "hold"),
        W_(1.2, (press_low[0] - 0.04, press_low[1], press_low[2] + 0.01), FINGER_CLOSED, "draw back off the leg"),
        W_(1.0, (0.18, -0.28, HIGH), FINGER_CLOSED, "park"),
    ]
    saddle = _world((SADDLE_AT[0], SADDLE_AT[1], SADDLE_AT[2] + SADDLE[2] / 2))
    grip_z = saddle[2] + TIP - 0.014  # tips 14 mm below the saddle top
    ya = math.pi / 2  # fingers close along x: one each side of the saddle
    # The press can rock the bike off the wall to upright (trace), so come down between the saddle's two possible
    # places: the open fingers (94 mm apart) straddle it either way.
    sx, sy = (saddle[0] + BIKE_XY[0]) / 2, saddle[1]
    rest = _world(SADDLE_AT, lean=-math.radians(10.0))
    right = right[:5] + [
        W_(0.8, (press_low[0] - 0.04, press_low[1], grip_z + 0.035), FINGER_OPEN, "up, clear of the bike"),
        W_(1.0, (sx - 0.04, sy, grip_z + 0.035), FINGER_OPEN, "turn the fingers", yaw=ya),
        W_(1.0, (sx, sy, grip_z + 0.035), FINGER_OPEN, "over the saddle", yaw=ya),
        W_(1.0, (sx, sy, grip_z + 0.02), FINGER_OPEN, "down to the saddle", yaw=ya),
        W_(1.0, (sx, sy, grip_z), FINGER_OPEN, "astride the saddle", yaw=ya),
        W_(2.5, (rest[0] - 0.025, sy, grip_z - 0.004), FINGER_OPEN, "draw it over onto the stand", yaw=ya),
        # Let go at once: the far finger backs off the saddle and the hand rises clear before the bike settles, so
        # the 1 s still hold runs with no finger near the bike.
        W_(0.4, (rest[0] - 0.012, sy, grip_z - 0.004), FINGER_OPEN, "back off the saddle", yaw=ya),
        W_(0.8, (rest[0] - 0.012, sy, HIGH), FINGER_OPEN, "up and clear", yaw=ya),
        W_(1.0, (0.18, -0.28, HIGH), FINGER_OPEN, "park"),
    ]
    return {"right": right}


if __name__ == "__main__":
    print("lean0 deg", math.degrees(LEAN0))
    print("worst IK error (mm, deg):", tilt.check_plan(demo()))
