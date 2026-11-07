"""On the station table, in front of the right arm: a brake disc standing on edge, fixed in place, its faces toward
and away from the robot, on a short hub. Beside the disc's top edge, one on each side, stand the two fixed abutment
blocks of the caliper bracket. Nearer the robot, the brake caliper lies on the bench the right way up, with a short
fixed yellow stop block on the bench between it and the robot. The caliper is red, with two legs (one each side of
where the disc goes), a grip boss on a bar across its top, two ears at its ends that rest on the bracket's
abutments, and an open window above its two tan brake pads. The pad nearer the robot sits on the caliper's steel
piston, which slides in the leg behind it. The pads are new.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m). The disc's axis runs along x.
DISC_R, DISC_T = 0.040, 0.028
DISC_XY = (0.33, -0.08)
DISC_CZ = TOP + 0.045  # centre height
DISC_TOP = DISC_CZ + DISC_R  # 0.085 above the table
HUB_R = 0.012
SILVER, STEEL = (0.78, 0.79, 0.80), (0.35, 0.36, 0.38)
RED, TAN, BRIGHT, YELLOW = (0.80, 0.12, 0.10), (0.82, 0.68, 0.48), (0.92, 0.93, 0.95), (0.95, 0.80, 0.10)
ABUT_Y = (0.044, 0.058)  # each abutment's span from the disc's centre plane, either side in y
ABUT_X = 0.052  # length along x: fits between the caliper legs (56 mm apart inside) with 2 mm play each side
ABUT_TOP = DISC_TOP - 0.005

_abut_h = ABUT_TOP - TOP
BRAKE = parts.Part(
    "brake_disc", pos=(*DISC_XY, TOP),
    shapes=[
        parts.cylinder(DISC_R, DISC_T, at=(0.0, 0.0, DISC_CZ - TOP), color=SILVER, axis="x"),
        parts.cylinder(HUB_R, 0.040, at=(DISC_T / 2 + 0.020, 0.0, DISC_CZ - TOP), color=STEEL, axis="x"),
        parts.box((0.040, 0.030, DISC_CZ - TOP - HUB_R), at=(DISC_T / 2 + 0.030, 0.0, (DISC_CZ - TOP - HUB_R) / 2),
                  color=STEEL),  # hub support
        *[parts.box((ABUT_X, ABUT_Y[1] - ABUT_Y[0], _abut_h), at=(0.0, s * sum(ABUT_Y) / 2, _abut_h / 2), color=STEEL)
          for s in (-1, 1)],
    ],
)

# Caliper, in its own frame: origin on the centre plane between the legs at the height of the disc's top when fitted
# (x along the disc's axis, the inboard side toward the robot at -x). Pads: outboard fixed, inner face at +18 mm;
# inboard on the piston, inner face at -2 mm (new pads: a 20 mm gap) and at -18 mm pushed fully back (36 mm).
PAD = (0.010, 0.050, 0.025)  # x, y, z
PAD_Z = -0.0075  # pad centre: from 20 mm below to 5 mm above the origin
TRAVEL = 0.016
GAP0 = 0.020
LEG_X = (0.028, 0.036)
LEG_Z = (-0.025, 0.015)
EAR_Y = (0.046, 0.056)
EAR_Z = (-0.005, 0.005)
BAR_Y, BAR_Z = (-0.040, -0.030), (0.015, 0.023)
BOSS = (0.020, 0.020, 0.020)
BOSS_AT = (0.0, -0.035, BAR_Z[1] + BOSS[2] / 2)
CAL_MASS = 0.40
PISTON_R, PISTON_L = 0.009, 0.006  # short enough to stay inside the near leg when pushed fully back (trace)
PISTON_K = 1000.0  # N/m: with the ratchet's 0.5 mm trail, about 1 N moves the piston. GPU traces: the arm's push
# could not move it at 5 N or 10 N, so it stays at 1 N (see the README on pushing it back with the disc)
CAL_XY = (0.25, -0.22)
CAL_Z0 = TOP - LEG_Z[0]  # on the bench: the legs' feet on the table

_leg_h = LEG_Z[1] - LEG_Z[0]
_leg_c = (LEG_Z[0] + LEG_Z[1]) / 2
CALIPER = parts.Part(
    "caliper", pos=(*CAL_XY, CAL_Z0), mass=CAL_MASS,
    shapes=[
        *[parts.box((LEG_X[1] - LEG_X[0], 2 * EAR_Y[1], _leg_h), at=(s * sum(LEG_X) / 2, 0.0, _leg_c), color=RED)
          for s in (-1, 1)],
        *[parts.box((2 * LEG_X[0], EAR_Y[1] - EAR_Y[0], EAR_Z[1] - EAR_Z[0]),
                    at=(0.0, s * sum(EAR_Y) / 2, sum(EAR_Z) / 2), color=RED) for s in (-1, 1)],
        parts.box((2 * LEG_X[1], BAR_Y[1] - BAR_Y[0], BAR_Z[1] - BAR_Z[0]),
                  at=(0.0, sum(BAR_Y) / 2, sum(BAR_Z) / 2), color=RED),
        parts.box(BOSS, at=BOSS_AT, color=RED),
        parts.box(PAD, at=(GAP0 - 0.002 + PAD[0] / 2, 0.0, PAD_Z), color=TAN),  # outboard pad, fixed
    ],
    joints=[parts.Slider(
        "piston", axis=(-1.0, 0.0, 0.0), travel=TRAVEL, mass=0.03, spring=PISTON_K, damping=1.0,
        shapes=[parts.box(PAD, at=(-0.002 - PAD[0] / 2, 0.0, PAD_Z), color=TAN),
                # the piston: a bright steel cylinder behind the near pad. Pushed fully back it slides into the near
                # leg (its jointed body, which PhysX does not collide it with) and stays inside it; a longer one
                # poked out of the leg's back, hit the stop block and stopped the push at 10.5 mm (trace)
                parts.cylinder(PISTON_R, PISTON_L, at=(-0.012 - PISTON_L / 2, 0.0, PAD_Z), color=BRIGHT, axis="x")])],
)

STOP_H = 0.010
STOP = Box("stop_block", pos=(CAL_XY[0] - LEG_X[1] - 0.008, CAL_XY[1], TOP + STOP_H / 2), size=(0.010, 0.10, STOP_H),
           color=YELLOW)

OBJECTS = [BRAKE, CALIPER, STOP]

# -- piston ratchet: pushed back, the piston stays there; moving it takes about 1 N -------------------------------
HOLD_BACK = 0.0005  # the piston's rest point trails the furthest point reached by 0.5 mm
EPS = 1e-6
_SET = {"piston": 0.0}


@parts.on_step
def _ratchet(sim) -> None:
    q = float(sim.states()["caliper"].joints["piston"])
    if q - HOLD_BACK > _SET["piston"] + EPS:
        _SET["piston"] = q - HOLD_BACK
    sim.set_target("caliper", "piston", _SET["piston"])


@parts.on_reset
def _new_episode(sim) -> None:
    _SET["piston"] = 0.0
    sim.set_target("caliper", "piston", 0.0)


# -- grippers: the scorer is given only the objects, so an on_step callback reads both arms' joints each control
# step and keeps each gripper's pose (from the arm's own kinematics) and half opening. Each finger is modelled as a box
# 10 mm thick (along the closing direction, outside the pad face), 20 mm wide and 45 mm long back from its tip. The
# caliper is released when every point of every finger is more than 5 mm from every shape of the caliper (body,
# pads and piston). Offline, with no grippers, it is released.
BASES = {"left_robot": (0.0, 0.305, 0.75), "right_robot": (0.0, -0.305, 0.75)}
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


def hands_gap(st) -> float:
    """Smallest distance (m) from any finger to any shape of the caliper; infinite with no grippers."""
    if not _GRIP:
        return math.inf
    poses = {"base": (np.asarray(st.pos, float), _rot(st.quat_xyzw)),
             "piston": (np.asarray(st.links["piston"][0], float), _rot(st.links["piston"][1]))}
    pts = np.concatenate([_finger_points(*g) for g in _GRIP.values()])
    gap = math.inf
    for _, c, r, size in CALIPER.world_boxes(poses):
        local = (pts - c) @ r
        gap = min(gap, float(np.linalg.norm(np.maximum(np.abs(local) - size / 2, 0.0), axis=1).min()))
    return gap


def released(st) -> bool:
    return hands_gap(st) > HANDS_OFF + EPS


# -- scoring --------------------------------------------------------------------------------------------------------
SEAT_TOL = (0.001, 0.002)  # the ears from 1 mm below to 2 mm above the abutments' tops
X_TOL, Y_TOL = 0.004 + EPS, 0.010 + EPS
LEVEL_DEG, UPRIGHT_DEG, YAW_DEG = 5.0, 10.0, 10.0


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tilt_deg(st) -> float:
    return float(np.degrees(np.arccos(np.clip(_rot(st.quat_xyzw)[2, 2], -1.0, 1.0))))


def yaw_deg(st) -> float:
    r = _rot(st.quat_xyzw)
    return float(abs(np.degrees(math.atan2(r[1, 0], r[0, 0]))))


def fitted(st) -> bool:
    """Hung on the bracket over the disc: ears on the abutments, centred over the disc, level, the right way round."""
    dz = st.pos[2] - DISC_TOP  # the ears' undersides sit 5 mm below the origin, on abutments 5 mm below the disc top
    return bool(
        -SEAT_TOL[0] - EPS <= dz <= SEAT_TOL[1] + EPS
        and abs(st.pos[0] - DISC_XY[0]) <= X_TOL
        and abs(st.pos[1] - DISC_XY[1]) <= Y_TOL
        and tilt_deg(st) <= LEVEL_DEG + EPS
        and yaw_deg(st) <= YAW_DEG + EPS
    )


def success(objects):
    c = objects["caliper"]
    return bool(fitted(c) and c.resting() and released(c))


def score(objects):
    """Pass/fail: 1.0 when fitted, released and still (and so once that has held for 1 s); 0 otherwise."""
    return 1.0 if success(objects) else 0.0


def distance(objects):
    c = objects["caliper"].pos
    return float(np.linalg.norm(c - np.array((*DISC_XY, DISC_TOP))))


# -- scripted demo ------------------------------------------------------------------------------------------------
# Right arm only. (1) Fingers closing along x (yaw 90 degrees), open: the far finger goes down through the window into
# the pad gap beside the piston's pad, the near one outside the caliper, and the hand moves toward the robot, pushing
# the pad back its full 16 mm (plus overshoot, the arm is soft) while the stop block holds the caliper. It lifts out.
# (2) Fingers along y (yaw 0): it pinches the grip boss, lifts the caliper with its pads 1 cm above the disc's top,
# carries it over the disc, lowers it until the ears are a few mm above the abutments and lets go.
TIP = 0.013
FINGER_T = 0.010  # an open finger's thickness outside the pad gap, along the closing direction
HALF_OPEN = -FINGER_OPEN  # 0.047
TRAVEL_Z = TOP + 0.16
PUSH_OVER = 0.022
RELEASE_UP = 0.004
SAG_X, SAG_Y, SAG_Z = 0.003, -0.004, 0.004  # right arm carrying the caliper: it lands 2.7 mm short in x and 4 mm to the robot's right (trace)
PUSH_SAG_X = 0.006  # aims the far finger at the middle of the pad gap; the arm settles about 2 mm short in x here (trace)


def demo():
    cx, cy = CAL_XY
    # (1) far finger's inner face 2 mm past the pad's face (the gap runs from -2 to +18 mm in the caliper frame)
    push_x0 = cx + 0.004 - HALF_OPEN + PUSH_SAG_X
    push_x1 = push_x0 - TRAVEL - PUSH_OVER
    tips_z = CAL_Z0 + PAD_Z  # the pad's mid-height
    push_tcp_z = tips_z + TIP
    yaw_x = math.pi / 2
    py = cy + 0.006  # finger 6 mm toward +y of the pads' centre, clear of the boss on the -y side
    # (2) boss pinch
    bx, by = cx + BOSS_AT[0], cy + BOSS_AT[1]
    boss_tcp = lambda origin_z: origin_z + BOSS_AT[2] - BOSS[2] / 4 + TIP  # noqa: E731  tips 5 mm above the boss's bottom
    lift_origin = DISC_TOP - PAD_Z + PAD[2] / 2 + 0.010  # pads' bottoms 1 cm above the disc top
    dx, dy = DISC_XY[0] + BOSS_AT[0] + SAG_X, DISC_XY[1] + BOSS_AT[1] - SAG_Y
    seat_origin = DISC_TOP + RELEASE_UP
    w = [
        Waypoint(1.5, (push_x0, py, TRAVEL_Z), FINGER_OPEN, "over the pad gap", yaw_x),
        Waypoint(1.5, (push_x0, py, push_tcp_z), FINGER_OPEN, "far finger down into the gap", yaw_x),
        Waypoint(2.5, (push_x1, py, push_tcp_z), FINGER_OPEN, "push the piston back", yaw_x),
        Waypoint(0.5, (push_x1 + 0.004, py, push_tcp_z), FINGER_OPEN, "ease off", yaw_x),
        Waypoint(1.2, (push_x1 + 0.004, py, TRAVEL_Z), FINGER_OPEN, "out of the gap", yaw_x),
        Waypoint(1.5, (bx, by, TRAVEL_Z), FINGER_OPEN, "over the boss"),
        Waypoint(1.2, (bx, by, boss_tcp(CAL_Z0)), FINGER_OPEN, "down to the boss"),
        Waypoint(0.8, (bx, by, boss_tcp(CAL_Z0)), FINGER_CLOSED, "pinch"),
        Waypoint(1.5, (bx, by, boss_tcp(lift_origin) + SAG_Z), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (dx, dy, boss_tcp(lift_origin) + SAG_Z), FINGER_CLOSED, "over the disc"),
        Waypoint(0.6, (dx, dy, boss_tcp(lift_origin) + SAG_Z), FINGER_CLOSED, "settle"),
        Waypoint(1.5, (dx, dy, boss_tcp(seat_origin) + SAG_Z), FINGER_CLOSED, "lower over the disc"),
        Waypoint(0.6, (dx, dy, boss_tcp(seat_origin) + SAG_Z), FINGER_CLOSED, "steady"),
        Waypoint(0.6, (dx, dy, boss_tcp(seat_origin) + SAG_Z), FINGER_OPEN, "let go"),
        Waypoint(1.2, (dx, dy, TRAVEL_Z), FINGER_OPEN, "up"),
    ]
    return {"right": w}
