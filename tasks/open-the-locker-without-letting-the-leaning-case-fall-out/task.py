"""On the station table, in front of the arms: a row of three small grey steel valuables lockers, of the kind
found at gyms, pools, schools and workplaces. Each compartment has its own pale yellow door with vent slots, a number
plate and a key lock. The robot's compartment is the right-hand one. Its door is hinged on its right edge (seen from
the robot) and swings toward the robot; a magnetic catch holds it shut, and past the catch it swings freely. Its lock
is open, and a black bar pull stands off the door near the hinge. The other two doors are locked. Inside the
right-hand compartment: a slim blue gift box holding a glass bottle of perfume. A green rubber mat lies on the table
in front of the lockers.
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

# World frame: x away from the arm bases, y to the robot's left, z up (m).
XD = 0.36  # the compartment's front opening (inner face of the shut door)
HY = -0.10  # its right inside wall, where the door is hinged
W, D = 0.24, 0.35  # inside width (y) and depth (x) of each compartment
P = 0.010  # floor plate: the compartment floor is 10 mm above the table
H = 0.165  # inside height of each compartment, floor to shelf
COLS, ROWS = 3, 1  # the bank: a row of three compartments; the robot's is the right-hand one (column 0)
WALL = 0.010
DOOR_T = 0.008
HANDLE_R, HANDLE_FROM_HINGE, HANDLE_OFF, HANDLE_Z = 0.006, 0.04, 0.040, P + 0.09  # bar handle on the door
CATCH = 0.05  # rad: the magnetic catch holds the door shut until it is pulled this far open
CATCH_PRELOAD = 0.15  # rad past shut: the catch's hold (0.3 N m/rad x 0.15 rad = 0.045 N m)

# The gift box (a glass bottle of perfume inside): 25 mm thick (x), 110 mm wide (y), 160 mm tall, 150 g. Its
# base's front edge is 35 mm behind the door, and its near (right) side 40 mm from the hinge.
CASE = (0.025, 0.110, 0.160)
CASE_MASS = 0.15
CASE_GAP = 0.035
CASE_FROM_HINGE = 0.040
CASE_Y = HY + CASE_FROM_HINGE + CASE[1] / 2
CASE_EDGE = np.array([XD + CASE_GAP, CASE_Y, TOP + P])  # front edge of its base, on the locker floor
# Leaning on the shut door: the top front edge 0.5 mm behind it, tipped forward about 12.5 degrees.
LEAN0 = math.asin((CASE_GAP - 0.0005) / CASE[2])

MAT = (0.140, 0.140, 0.002)
MAT_XY = (0.180, 0.080)
MAT_TOP = TOP + MAT[2]

GREY, DOOR_YELLOW, BLACK = (0.62, 0.63, 0.65), (0.90, 0.82, 0.55), (0.06, 0.06, 0.06)
CASE_BLUE, GREEN = (0.10, 0.25, 0.85), (0.15, 0.60, 0.25)
SLOT, PLATE, CHROME = (0.25, 0.25, 0.27), (0.95, 0.95, 0.92), (0.80, 0.80, 0.82)
PITCH_Y, PITCH_Z = W + WALL, H + WALL  # compartment spacing across and up the bank


def _door_face(y0: float, z0: float):
    """Vent slots, number plate and key lock on the outer face of a door whose compartment starts at (y0, z0)
    (part frame: the right inside wall's y, the compartment floor's z). They stand 0.6 to 5 mm proud of the face."""
    yc, x_face = y0 + W / 2, -DOOR_T
    out = [parts.box((0.0006, 0.080, 0.004), at=(x_face - 0.0003, yc + 0.02, z0 + H - 0.022 - 0.010 * k), color=SLOT)
           for k in range(4)]
    out.append(parts.box((0.0006, 0.032, 0.016), at=(x_face - 0.0003, yc + 0.02, z0 + H - 0.075), color=PLATE))
    lock_y = y0 + W - 0.002 - 0.030  # 30 mm in from the door's free edge
    out.append(parts.cylinder(0.009, 0.005, at=(x_face - 0.0025, lock_y, z0 + H / 2), color=CHROME, axis="x"))
    return out


_door = parts.Hinge(
    "door",
    shapes=[
        parts.box((DOOR_T, W - 0.004, H - 0.004), at=(-DOOR_T / 2, W / 2, P + H / 2), color=DOOR_YELLOW),
        parts.cylinder(HANDLE_R, 0.060, at=(-DOOR_T - HANDLE_OFF, HANDLE_FROM_HINGE, HANDLE_Z), color=BLACK,
                       axis="z"),
        parts.box((HANDLE_OFF, 0.008, 0.008), at=(-DOOR_T - HANDLE_OFF / 2, HANDLE_FROM_HINGE, HANDLE_Z + 0.026),
                  color=BLACK),
        parts.box((HANDLE_OFF, 0.008, 0.008), at=(-DOOR_T - HANDLE_OFF / 2, HANDLE_FROM_HINGE, HANDLE_Z - 0.026),
                  color=BLACK),
        *_door_face(0.0, P),
    ],
    pivot=(-DOOR_T / 2, 0.0, P + H / 2),
    axis=(0.0, 0.0, 1.0),  # positive opens: the free (left) edge swings toward the robot
    lower=0.0,
    upper=1.65,
    mass=0.15,
    spring=0.30,
    damping=0.30,
)
_sx = (D + WALL + DOOR_T) / 2 - DOOR_T  # centre x of the full-depth walls (they run from the door's face to the back)
_BW = COLS * PITCH_Y + WALL  # the bank's outer width (y), from the right outer wall's outer face
_BH = P + ROWS * PITCH_Z  # its height above the table
_bank = [
    parts.box((D + WALL, _BW, P), at=((D + WALL) / 2, _BW / 2 - WALL, P / 2), color=GREY),  # floor plate
    parts.box((WALL, _BW, _BH), at=(D + WALL / 2, _BW / 2 - WALL, _BH / 2), color=GREY),  # back
]
for c in range(COLS + 1):  # upright walls between and beside the columns
    _bank.append(parts.box((D + WALL + DOOR_T, WALL, _BH), at=(_sx, c * PITCH_Y - WALL / 2, _BH / 2), color=GREY))
for r in range(ROWS):  # the shelf over each row (the top one is the bank's roof)
    _bank.append(parts.box((D + WALL + DOOR_T, _BW, WALL), at=(_sx, _BW / 2 - WALL, P + H + r * PITCH_Z + WALL / 2),
                           color=GREY))
for c in range(COLS):  # the other doors are locked shut: fixed in place
    for r in range(ROWS):
        if (c, r) == (0, 0):
            continue
        y0, z0 = c * PITCH_Y, P + r * PITCH_Z
        _bank.append(parts.box((DOOR_T, W - 0.004, H - 0.004), at=(-DOOR_T / 2, y0 + W / 2, z0 + H / 2),
                               color=DOOR_YELLOW))
        _bank += _door_face(y0, z0)
LOCKER = parts.Part(
    "locker",
    pos=(XD, HY, TOP),
    shapes=_bank,
    joints=[_door],
)
# Listed upright here; at every reset it is tipped forward onto the door (see _new_episode).
CASE_BOX = Box("blue_box", pos=(XD + CASE_GAP + CASE[0] / 2, CASE_Y, TOP + P + CASE[2] / 2), size=CASE,
               mass=CASE_MASS, color=CASE_BLUE)
MAT_BOX = Box("green_mat", pos=(*MAT_XY, TOP + MAT[2] / 2), size=MAT, color=GREEN)

OBJECTS = [LOCKER, CASE_BOX, MAT_BOX]


def _lean_pose(lean: float):
    """Centre and quaternion (x, y, z, w) of the box tipped ``lean`` rad forward about its base's front edge."""
    c, s = math.cos(lean), math.sin(lean)
    centre = CASE_EDGE + np.array([c * CASE[0] / 2 - s * CASE[2] / 2, 0.0, s * CASE[0] / 2 + c * CASE[2] / 2 + 0.0005])
    return centre, np.array([0.0, -math.sin(lean / 2), 0.0, math.cos(lean / 2)])


# -- scoring ---------------------------------------------------------------------------------------------------

EPS = 1e-6
TIPPED_DEG = 45.0  # the box tipped: it leaned more than 45 degrees at any time (it is printed this way up)
UPRIGHT_DEG = 5.0  # standing: its long axis within 5 degrees of vertical
ON_TOL = 0.0015  # standing on a surface: its lowest corner within 1.5 mm of it (the demo lets go 5 mm up)
DROP_SPEED = 0.45  # m/s: dropped if the box ever moves downward faster than this (a free fall of about 1 cm)
_H = {}  # trial history, cleared at every reset


def _clear():
    _H.clear()
    _H.update(tipped=False, dropped=False)


_clear()


def _update(objects) -> None:
    case = objects["blue_box"]
    if tilt.tilt_of(case.quat_xyzw) > TIPPED_DEG:
        _H["tipped"] = True
    if float(case.lin_vel[2]) < -DROP_SPEED - EPS:
        _H["dropped"] = True


# -- release check: no gripper link within 5 mm of the box ------------------------------------------------------
# In the simulator an on_step hook measures, every step, the box's distance to both arms' grippers and finger tips.
# Before the first measurement after a reset the check fails closed (no credit). Offline (near_misses.py sets
# OFFLINE_FLAG; no simulator, no robot) each near-miss case stands for a state with the grippers away.
RELEASE_GAP = 0.005  # m: let go, every gripper link at least 5 mm from the box
_GAP: dict[str, float] = {}  # "blue_box" -> closest gripper distance at the last step (m)
OFFLINE_FLAG = "LOCKER_CASE_TASK_OFFLINE_CHECK"
_HAND_LINKS = ("gripper", "tip_left", "tip_right")


@parts.on_step
def _record_gap(sim):
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    hands = []
    for arm in ("left_robot", "right_robot"):
        rob = base.scene[arm]
        poses = parts._np(rob.data.body_link_pose_w)[0]  # per link: x, y, z, qx, qy, qz, qw
        links = {}
        for i, n in enumerate(rob.body_names):
            if n in _HAND_LINKS:
                t = np.eye(4)
                t[:3, :3] = parts._quat_to_matrix(poses[i, 3:])
                t[:3, 3] = poses[i, :3] - origin
                links[n] = t
        hands += analysis.world_hulls(links)
    case = sim.states()["blue_box"]
    rot = parts._quat_to_matrix(case.quat_xyzw)
    _GAP["blue_box"] = float(analysis.hulls_to_box(hands, np.asarray(case.pos, float), rot, np.asarray(CASE))[0])


def _offline() -> bool:
    import os

    return os.environ.get(OFFLINE_FLAG) == "1"


def _released() -> bool:
    """Let go: every gripper link at least 5 mm from the box. No measurement yet: no credit (fails closed)."""
    if "blue_box" not in _GAP:
        return _offline()
    return _GAP["blue_box"] >= RELEASE_GAP - EPS


def _spoiled() -> bool:
    """Tipped past 45 degrees or dropped at any time in the trial."""
    return bool(_H["tipped"] or _H["dropped"])


def _standing(case, surface_z: float) -> bool:
    return bool(
        tilt.tilt_of(case.quat_xyzw) <= UPRIGHT_DEG + EPS
        and abs(case.low[2] - surface_z) <= ON_TOL + EPS
        and case.resting()
        and _released()
    )


def _on_mat(case) -> bool:
    x0, y0 = MAT_XY[0] - MAT[0] / 2, MAT_XY[1] - MAT[1] / 2
    x1, y1 = MAT_XY[0] + MAT[0] / 2, MAT_XY[1] + MAT[1] / 2
    return bool(case.low[0] >= x0 - EPS and case.high[0] <= x1 + EPS and case.low[1] >= y0 - EPS
                and case.high[1] <= y1 + EPS)


def success(objects):
    """The box stands upright and still, entirely on the green mat, and was never tipped past 45 degrees or dropped."""
    _update(objects)
    case = objects["blue_box"]
    return bool(not _spoiled() and _standing(case, MAT_TOP) and _on_mat(case))


def score(objects):
    """1.0 as success(). 0.5: never tipped past 45 degrees or dropped, and it stands upright and still on the table or
    the mat outside the locker, but not entirely on the mat. 0 if it ever leaned more than 45 degrees or moved
    downward faster than 0.45 m/s, and otherwise."""
    if success(objects):
        return 1.0
    if _spoiled():
        return 0.0
    case = objects["blue_box"]
    out = case.high[0] <= XD - DOOR_T + EPS
    if out and (_standing(case, TOP) or _standing(case, MAT_TOP)):
        return 0.5
    return 0.0


def distance(objects):
    c = objects["blue_box"].pos
    return float(np.hypot(c[0] - MAT_XY[0], c[1] - MAT_XY[1]))


@parts.on_reset
def _new_episode(sim):
    """A new episode: clear the tipping and drop record and the gripper measurement, set the catch, and tip the box
    forward onto the shut door."""
    import torch

    _clear()
    _GAP.clear()
    sim.set_target("locker", "door", -CATCH_PRELOAD)
    base = sim.env.unwrapped
    case = base.scene["blue_box"]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    centre, q = _lean_pose(LEAN0)
    pose = torch.tensor([[*(centre + origin), *q]], dtype=torch.float32, device=base.device)
    case.write_root_pose_to_sim_index(root_pose=pose, env_ids=[0])
    case.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=base.device), env_ids=[0])


@parts.on_step
def _door_spring(sim):
    """The catch holds the door within its first 3 degrees; past it the door swings freely (damped)."""
    phi = float(sim.states()["locker"].joints["door"])
    sim.set_target("locker", "door", -CATCH_PRELOAD if phi < CATCH else phi)


# -- demo ------------------------------------------------------------------------------------------------------

TIP = 0.013
HIGH = TOP + 0.16
CLEAR_AT = 1.50  # rad: swung this far, the door is clear of the opening
PINCH_YAW = -math.pi / 2
CATCH_AT = 0.95  # rad: how far the right arm opens the door before the left arm takes the box
CATCH_LEAN = math.asin((CASE_GAP + CASE_FROM_HINGE * math.tan(CATCH_AT)) / CASE[2])  # the box leans with it
PINCH_TILT = 30.0  # degrees: the left hand's fingertips lean away from the robot, fingers closing across the box
PINCH_Y = 0.025  # world y of the pinch, near the box's free (left) end where the door is furthest away
PINCH_DOWN = 0.010  # TCP this far down the box from its top
FINGER_AROUND = -0.020  # finger joint for pads 40 mm apart, around the 25 mm box


def _handle(phi: float, z: float):
    """TCP on the handle bar with the door open ``phi``; the grip yaw follows the door."""
    px, py = XD - DOOR_T / 2, HY
    lx, ly = -DOOR_T / 2 - HANDLE_OFF, HANDLE_FROM_HINGE
    c, s = math.cos(phi), math.sin(phi)
    return (px + c * lx - s * ly, py + s * lx + c * ly, z), phi


def _pinch_point(lean: float):
    """The pinch TCP on the box leaning ``lean`` (rad) about its base's front edge."""
    up = np.array([-math.sin(lean), 0.0, math.cos(lean)])
    centre, _ = _lean_pose(lean)
    p = centre + up * (CASE[2] / 2 - PINCH_DOWN)
    return np.array([p[0], PINCH_Y, p[2]])


def demo():
    """Right arm: grips the handle, pulls the door past the catch and opens it slowly to 54 degrees, holds it there
    while the left arm takes the box, then swings it on to 86 degrees, clear of the opening, and lets go.
    Left arm (tilt.py): fingertips leaning 30 degrees away from the robot and closing front to back, it reaches down
    into the gap between the door and the locker and pinches the box's top; once the door is clear it lifts the
    case out, stands it upright, sets it on the mat, lets go and rises straight up clear of it."""
    W_ = tilt.Waypoint
    hz = TOP + HANDLE_Z
    right = [W_(1.5, _handle(0.0, HIGH)[0], FINGER_OPEN, "over the handle")]
    right += [W_(1.5, _handle(0.0, hz)[0], FINGER_OPEN, "down round the handle")]
    right += [W_(0.6, _handle(0.0, hz)[0], FINGER_CLOSED, "grip the handle")]
    n = 8
    for k in range(1, n + 1):
        p, yaw = _handle(CATCH_AT * k / n, hz)
        right.append(W_(0.5, p, FINGER_CLOSED, f"open {k}", yaw=yaw))
    p_hold, yaw_hold = _handle(CATCH_AT, hz)
    right.append(W_(2.3 + 0.5, p_hold, FINGER_CLOSED, "hold the door", yaw=yaw_hold))
    m = 6
    for k in range(1, m + 1):
        p, yaw = _handle(CATCH_AT + (CLEAR_AT - CATCH_AT) * k / m, hz)
        right.append(W_(0.5, p, FINGER_CLOSED, f"swing it clear {k}", yaw=yaw))
    p_end, yaw_end = _handle(CLEAR_AT, hz)
    right.append(W_(0.5, p_end, FINGER_OPEN, "let go", yaw=yaw_end))
    right.append(W_(0.6, (p_end[0], p_end[1], HIGH), FINGER_OPEN, "up off the handle", yaw=yaw_end))
    right.append(W_(1.0, (0.18, -0.28, HIGH), FINGER_OPEN, "park"))
    t_clear = 3.6 + n * 0.5 + 2.8 + m * 0.5 + 0.5  # the door is clear and let go

    lean = CATCH_LEAN
    tl = PINCH_TILT
    pinch = _pinch_point(lean)
    axis = np.array([math.sin(math.radians(tl)), 0.0, -math.cos(math.radians(tl))])  # approach direction
    above = pinch - axis * 0.07
    ya = PINCH_YAW  # fingers close along x (front to back across the box)
    lean_kw = dict(yaw=ya, tilt=tl, heading=0.0)
    wait = 3.6 + n * 0.5 - 1.5  # above the gap as the door reaches CATCH_AT
    left = [
        W_(wait, (0.20, 0.25, HIGH), FINGER_OPEN, "wait for the door"),
        W_(1.5, tuple(above), FINGER_AROUND, "above the gap", **lean_kw),
        W_(1.5, tuple(pinch), FINGER_AROUND, "down onto the box top", **lean_kw),
        W_(0.8, tuple(pinch), FINGER_CLOSED, "pinch", **lean_kw),
    ]
    left.append(W_(t_clear - sum(w.duration for w in left), tuple(pinch), FINGER_CLOSED,
                   "hold it while the door is swung clear", **lean_kw))
    left += [
        W_(1.0, tuple(pinch + [0.0, 0.0, 0.012]), FINGER_CLOSED, "lift off the floor", **lean_kw),
        W_(2.0, tuple(pinch + [-0.11, 0.0, 0.05]), FINGER_CLOSED, "draw it out", **lean_kw),
    ]
    # Upright: with the grip turned back by the box's lean, the box hangs straight down from the pinch.
    # The box hangs 16 mm behind and 21 mm to the right of the pinch (trace): aim the pinch off by that much.
    tx, ty = MAT_XY[0] + 0.016, MAT_XY[1] + 0.021
    hang = CASE[2] - PINCH_DOWN  # TCP above the box's bottom when upright
    up_kw = dict(yaw=ya, tilt=PINCH_TILT - math.degrees(lean), heading=0.0)
    left += [
        W_(1.5, (tx, ty, MAT_TOP + hang + 0.04), FINGER_CLOSED, "stand it up over the mat", **up_kw),
        W_(1.2, (tx, ty, MAT_TOP + hang + 0.005), FINGER_CLOSED, "lower", **up_kw),
        W_(0.6, (tx, ty, MAT_TOP + hang + 0.005), FINGER_OPEN, "let go", **up_kw),
        # Finger tips start 23 mm below the box's top; 50 mm up leaves them well clear of the 5 mm release check.
        W_(1.0, (tx, ty, MAT_TOP + hang + 0.055), FINGER_OPEN, "straight up, clear of the box", **up_kw),
        W_(1.0, (tx - 0.05, ty, MAT_TOP + hang + 0.065), FINGER_OPEN, "away", yaw=ya),
    ]
    return {"left": left, "right": right}


if __name__ == "__main__":
    print("worst IK error (mm, deg):", tilt.check_plan(demo()))
