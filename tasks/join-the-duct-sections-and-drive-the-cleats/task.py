"""On the station table: a fixed grey sheet-metal duct section with its open end toward the robot, a flange standing
out from each side of that end, and an S-slip lip over its top edge. Nearer the robot, a
loose duct section of the same size sits on the table, not lined up with the fixed one, with matching side flanges
at its far end and a grip bar on its top. On each side of the joint stands a loose orange drive cleat, a channel standing
upright with its slot facing the duct; the slot widens over its bottom few millimetres.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
import lots  # noqa: E402  (after parts, so its reset hooks run first)
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

GALV, GALV_B, DARK, CLEAT_C = (0.62, 0.64, 0.66), (0.72, 0.74, 0.76), (0.15, 0.15, 0.17), (0.95, 0.55, 0.05)
FLANGE_C = (0.22, 0.23, 0.25)  # flanges darkened so the joint reads in the top camera

# Duct: axis along x. Sections are 120 mm wide (y), 40 mm tall, 120 mm long, 1.6 mm sheet.
DW, DH, DL, SHEET = 0.120, 0.040, 0.120, 0.0016
FL_T, FL_OUT = 0.002, 0.015  # side flanges: 2 mm thick, standing 15 mm out from each side, full height
LIP_L, LIP_T, LIP_GAP = 0.010, 0.004, 0.0015  # S-slip lip: 10 mm over the joint, 1.5 mm clear of the top sheet
XJ = 0.340  # the joint: the fixed section's open end
Y0 = 0.000  # the duct's centre line


def _duct_shapes(x0: float, color, flange_at: float):
    """A duct section from x0 to x0 + DL (part frame at its centre bottom), flanges at its end ``flange_at``
    (+1: far end, -1: near end)."""
    c = []
    c.append(parts.box((DL, DW, SHEET), at=(0.0, 0.0, SHEET / 2), color=color))
    c.append(parts.box((DL, DW, SHEET), at=(0.0, 0.0, DH - SHEET / 2), color=color))
    for s in (-1, 1):
        c.append(parts.box((DL, SHEET, DH - 2 * SHEET), at=(0.0, s * (DW / 2 - SHEET / 2), DH / 2), color=color))
        fx = flange_at * (DL / 2 - FL_T / 2)
        c.append(parts.box((FL_T, FL_OUT, DH), at=(fx, s * (DW / 2 + FL_OUT / 2), DH / 2), color=FLANGE_C))
    return c


A_SHAPES = _duct_shapes(0.0, GALV, -1.0) + [
    # S-slip lip at the open end, over the top sheet, reaching 10 mm toward the robot.
    parts.box((LIP_L, DW, LIP_T), at=(-DL / 2 - LIP_L / 2, 0.0, DH + LIP_GAP + LIP_T / 2), color=DARK),
]
DUCT_A = parts.Part("fixed_duct", pos=(XJ + DL / 2, Y0, TOP), shapes=A_SHAPES)

BAR = (0.020, 0.040, 0.015)  # grip bar on the middle of the loose section's top
B_SHAPES = _duct_shapes(0.0, GALV_B, 1.0) + [
    parts.box(BAR, at=(0.0, 0.0, DH + BAR[2] / 2), color=DARK),
]
B_START = (XJ - DL / 2 - 0.060, Y0 + 0.025)  # 60 mm short, 25 mm to the robot's left
B_YAW = math.radians(8.0)
DUCT_B = parts.Part("loose_duct", pos=(*B_START, TOP), yaw=B_YAW, mass=0.150, shapes=B_SHAPES)

# Drive cleats: upright channels 55 mm tall, 24 mm (x) by 20 mm (y), with an 8 mm slot 12 mm deep running their full
# height, open toward the duct. Over the bottom 4 mm the slot widens to 13 mm (each cheek set 2.5 mm further out), a
# stepped lead-in. Part frame: origin at the bottom middle of the slot's mouth.
CL_H, CL_X, CL_Y, SLOT, SLOT_D = 0.055, 0.024, 0.020, 0.008, 0.012
LEAD_H, LEAD = 0.004, 0.0025  # the lead-in: bottom 4 mm, cheeks 2.5 mm further out each side


def _cleat_shapes(side: float):
    """side +1: the cleat for the left flanges (slot opens toward -y); -1 for the right."""
    cheek = (CL_X - SLOT) / 2
    back = CL_Y - SLOT_D
    up = CL_H - LEAD_H
    out = []
    for s in (-1, 1):
        out.append(parts.box((cheek, SLOT_D, up), at=(s * (SLOT / 2 + cheek / 2), side * SLOT_D / 2, LEAD_H + up / 2),
                             color=CLEAT_C))
        out.append(parts.box((cheek, SLOT_D, LEAD_H), at=(s * (SLOT / 2 + LEAD + cheek / 2), side * SLOT_D / 2,
                                                         LEAD_H / 2), color=CLEAT_C))
    out.append(parts.box((CL_X, back, up), at=(0.0, side * (SLOT_D + back / 2), LEAD_H + up / 2), color=CLEAT_C))
    out.append(parts.box((CL_X + 2 * LEAD, back, LEAD_H), at=(0.0, side * (SLOT_D + back / 2), LEAD_H / 2),
                         color=CLEAT_C))
    return out


FLANGE_EDGE = DW / 2 + FL_OUT  # the flanges' outer edges, from the centre line
CLEAT_DEPTH = 0.0075  # a driven cleat's slot takes 7.5 mm of the flanges (4.5 mm short of its back)
CLEAT_Y = {"left": Y0 + FLANGE_EDGE - CLEAT_DEPTH, "right": Y0 - FLANGE_EDGE + CLEAT_DEPTH}  # slot mouth, driven
CLEAT_START = {"left": (0.270, 0.150), "right": (0.270, -0.150)}
CLEATS = {
    side: parts.Part(f"{side}_cleat", pos=(*CLEAT_START[side], TOP), mass=0.030,
                     shapes=_cleat_shapes(1.0 if side == "left" else -1.0))
    for side in ("left", "right")
}

OBJECTS = [DUCT_A, DUCT_B, *CLEATS.values()]

# Lot A is the layout above; lot B mirrors the loose section's start (25 mm to the robot's right, turned 8 deg the
# other way), so a fixed shift or turn joins it in one lot only. Lot A gets a zero move so it settles like lot B.
B_START_B = (B_START[0], Y0 - (B_START[1] - Y0))
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"loose_duct": (*B_START, TOP)}),
    lots.Lot("B", place={"loose_duct": (*B_START_B, TOP)}, yaw={"loose_duct": -B_YAW}),
)

EPS = 1e-6
HOME_X, HOME_Y, HOME_YAW = 0.002, 0.002, math.radians(1.5)
GAP_MAX = 0.001  # flanges touching: gap under 1 mm
CLEAT_LOW = 0.003  # a driven cleat's bottom within 3 mm of the table
CLEAT_TILT = math.radians(5.0)
CLEAT_YAW = math.radians(10.0)
ON = 0.0015
CLEAR = 0.005  # released: no part of either gripper within 5 mm of a scored part
SUCCESS_HOLD_S = 1.5


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _yaw(r) -> float:
    return math.atan2(r[1, 0], r[0, 0])


def _tilt(r) -> float:
    return math.acos(float(np.clip(r[2, 2], -1.0, 1.0)))


def b_lead(b):
    """The loose section's leading end: (x of its flanges' far face, y of its centre line, yaw, tilt)."""
    r = _rot(b.quat_xyzw)
    lead = np.asarray(b.pos) + r @ np.array([DL / 2, 0.0, 0.0])
    return float(lead[0]), float(lead[1]), _yaw(r), _tilt(r)


FLANGE_MID = DW / 2 + FL_OUT / 2  # the middle of each side flange, from the centre line


def flange_face(b, side: str) -> float:
    """x of the loose section's far flange face on ``side``, at the middle of that flange."""
    s = 1.0 if side == "left" else -1.0
    r = _rot(b.quat_xyzw)
    return float((np.asarray(b.pos) + r @ np.array([DL / 2, s * FLANGE_MID, DH / 2]))[0])


def b_home(objects) -> bool:
    """At each side's flange pair, the loose flange within 1 mm of the fixed one (gap under 1 mm); centred within
    2 mm, square within 1.5 deg, tilted under 1.5 deg, flat on the table under the lip."""
    b = objects["loose_duct"]
    _, y, yaw, tilt = b_lead(b)
    faces_ok = all(XJ - GAP_MAX - EPS <= flange_face(b, side) <= XJ + EPS for side in ("left", "right"))
    return bool(faces_ok and abs(y - Y0) <= HOME_Y + EPS and abs(yaw) <= HOME_YAW + EPS
                and tilt <= HOME_YAW + EPS and abs(b.low[2] - TOP) <= ON + EPS)


def cleat_on(objects, cleat: str, side: str) -> bool:
    """Cleat ``cleat`` ("left_cleat" or "right_cleat") on the flange pair on ``side``: upright (5 deg), its slot
    across the joint and opening toward the duct (10 deg), its bottom within 3 mm of the table, its 8 mm slot (above
    the lead-in) straddling both flanges of that side and taking at least 5 mm of them. The two cleats are the same
    part mirrored, so either fits either side (turned round)."""
    c = objects[cleat]
    r = _rot(c.quat_xyzw)
    if _tilt(r) > CLEAT_TILT + EPS or c.low[2] > TOP + CLEAT_LOW + EPS:
        return False
    s = 1.0 if side == "left" else -1.0
    own = 1.0 if cleat == "left_cleat" else -1.0
    opening = r @ np.array([0.0, -own, 0.0])  # the slot's opening direction, world
    if float(opening @ np.array([0.0, -s, 0.0])) < math.cos(CLEAT_YAW) - EPS:  # toward the duct, within 10 deg
        return False
    p = np.asarray(c.pos)
    lo, hi = flange_face(objects["loose_duct"], side) - FL_T, XJ + FL_T  # the two flanges' outer faces along x
    if not (p[0] - SLOT / 2 - EPS <= lo and hi <= p[0] + SLOT / 2 + EPS):
        return False
    take = s * (Y0 + s * FLANGE_EDGE - p[1])  # how far the flanges reach into the slot
    return bool(take >= 0.005 - EPS)


def cleats_on(objects) -> int:
    """How many flange pairs have a cleat on them, either cleat on either side, no cleat counted twice."""
    def on(c, side):
        return int(cleat_on(objects, c, side))
    return max(on("left_cleat", "left") + on("right_cleat", "right"),
               on("right_cleat", "left") + on("left_cleat", "right"))


# Released: in the simulator an on_step callback measures, every control step, how close any part of either gripper
# (its body and both finger tips, as convex hulls) comes to the section's and both cleats' own boxes. Offline
# (near_misses.py) there is no robot, so "clear" stays True.
SCORED = ("loose_duct", "left_cleat", "right_cleat")
_grip = {"clear": True}
_PARTS = {p.name: p for p in OBJECTS}


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right", "gripper"):
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    clear = True
    for name in SCORED:
        o = st[name]
        for _, c, r, size in _PARTS[name].world_boxes({"base": (np.asarray(o.pos), _rot(o.quat_xyzw))}):
            if analysis.hulls_to_box(hulls, c, r, size)[0] <= CLEAR:
                clear = False
                break
        if not clear:
            break
    _grip["clear"] = clear


def _still(objects) -> bool:
    return bool(_grip["clear"] and all(objects[n].resting() for n in SCORED))


def success(objects):
    return bool(b_home(objects) and cleats_on(objects) == 2 and _still(objects))


def distance(objects):
    x, y, _, _ = b_lead(objects["loose_duct"])
    return float(np.hypot(XJ - x, Y0 - y))


def score(objects):
    """1.0 on success; 0.6 the loose section home and one flange pair with a cleat on; 0.25 the section home and no
    cleat on. Every tier needs the section and both cleats released and still. 0 otherwise."""
    if not (b_home(objects) and _still(objects)):
        return 0.0
    return (0.25, 0.6, 1.0)[cleats_on(objects)]


# --- demo -----------------------------------------------------------------------------------------------------------
TIP = 0.013
CLEAT_GRIP_DOWN = 0.008  # finger tips 8 mm below a cleat's top
CLEAT_PINCH = -(CL_X / 2 - 0.003)  # close 3 mm past the cleat's cheeks, not fully (a full close holds it off centre)
CLEAT_EASE = -(CL_X / 2 + 0.002)  # let go by opening only 2 mm clear of the cheeks (opening fully kicked it round)
# Where each arm leaves a held cleat relative to where it is sent (x, y), measured in GPU traces; the demo aims off by
# the opposite amount. Fixed numbers, not read from the scene.
CLEAT_AIM = {"left": (-0.0016, 0.0015), "right": (-0.0017, 0.0008)}
PARK = {"left": (0.15, 0.26, TOP + 0.12), "right": (0.15, -0.26, TOP + 0.12)}


def demo():
    """Left arm: grip the loose section's bar, lift it 10 mm, turn it square and line it up, set it down 5 mm short,
    slide it home under the lip, let go and move aside. Right arm: carry the right cleat over the joint's right
    flanges, lower it until its lead-in is 3 mm over them, pause, lower 6 mm more, slide it down the flanges to 6 mm
    above the table and let go, then press it down to the table with the closed fingers and park. Left arm: the same with the left cleat."""
    lot = lots.current()
    sx, sy, _ = lot.pos("loose_duct")
    syaw = lot.yaw_of("loose_duct")
    tips = DH + BAR[2] - 0.004  # finger tips 11 mm below the bar's top
    gz, lift = TOP + tips + TIP, 0.010
    hx = XJ - DL / 2  # the loose section's centre when home
    near_x, home_x = hx - 0.005, hx + 0.040  # set down 5 mm short; push 40 mm past contact (the arm gives)
    # Where the arm leaves the section across the duct depends on the side it comes from (GPU traces: 5 mm to the
    # robot's left coming from the left, 2.5 mm coming from the right); aim off by that. Lot A's start is on the left.
    ly = Y0 - (0.0051 if sy > Y0 else 0.0025)
    yb = -math.pi / 2  # fingers close along x, across the bar's 20 mm
    left = [
        Waypoint(1.5, (sx, sy, gz + 0.05), FINGER_OPEN, "over the bar", yb + syaw),
        Waypoint(1.2, (sx, sy, gz), FINGER_OPEN, "down", yb + syaw),
        Waypoint(0.8, (sx, sy, gz), FINGER_CLOSED, "grip the bar", yb + syaw),
        Waypoint(1.0, (sx, sy, gz + lift), FINGER_CLOSED, "lift 10 mm", yb + syaw),
        Waypoint(2.5, (near_x, ly, gz + lift), FINGER_CLOSED, "square and in line, 5 mm short", yb),
        Waypoint(0.8, (near_x, ly, gz + 0.001), FINGER_CLOSED, "set down", yb),
        Waypoint(1.5, (home_x, ly, gz + 0.001), FINGER_CLOSED, "slide home under the lip", yb),
        Waypoint(1.2, (home_x, ly, gz + 0.001), FINGER_CLOSED, "hold home", yb),
        Waypoint(0.6, (home_x, ly, gz + 0.001), FINGER_OPEN, "let go of the section", yb),
        Waypoint(1.0, (hx, ly, gz + 0.05), FINGER_OPEN, "up", yb),
        Waypoint(1.5, PARK["left"], FINGER_OPEN, "aside"),
    ]
    t_home = sum(w.duration for w in left)

    def drive(side, arm_wait):
        sgn = 1.0 if side == "left" else -1.0
        mid = sgn * CL_Y / 2  # grip the cleat across its middle, not at its slot's mouth (its origin)
        cx0, cy0 = CLEAT_START[side][0], CLEAT_START[side][1] + mid
        ax, ay = CLEAT_AIM[side]
        tx, ty = XJ + ax, CLEAT_Y[side] + mid + ay
        hold = CL_H - CLEAT_GRIP_DOWN + TIP  # TCP height above the cleat's bottom while it is held
        g0 = TOP + hold
        flange_top = TOP + DH
        g_hover = flange_top + 0.010 + hold  # cleat bottom 10 mm over the flanges' tops
        g_lead = flange_top + 0.003 + hold  # 3 mm over them
        g_drop = flange_top - 0.003 + hold  # 6 mm lower: the narrow slot 2 mm down over them
        g_low = TOP + 0.006 + hold  # cleat bottom 6 mm over the table: let go there and it drops
        press = TOP + CL_H + TIP - 0.003  # closed tips 3 mm below the top of a cleat standing on the table
        yc = math.pi / 2 if side == "right" else -math.pi / 2  # fingers close along x, on the cleat's cheeks
        return [
            Waypoint(arm_wait, (cx0, cy0, g0 + 0.05), FINGER_OPEN, f"wait over the {side} cleat", yc),
            Waypoint(1.2, (cx0, cy0, g0), FINGER_OPEN, "down", yc),
            Waypoint(0.8, (cx0, cy0, g0), CLEAT_PINCH, f"grip the {side} cleat", yc),
            Waypoint(1.5, (cx0, cy0, g_hover + 0.01), CLEAT_PINCH, "lift", yc),
            Waypoint(2.0, (tx, ty, g_hover), CLEAT_PINCH, "over the flanges", yc),
            Waypoint(1.0, (tx, ty, g_lead), CLEAT_PINCH, "lead-in 3 mm over them", yc),
            Waypoint(1.0, (tx, ty, g_lead), CLEAT_PINCH, "pause", yc),
            Waypoint(1.0, (tx, ty, g_drop), CLEAT_PINCH, "6 mm lower, onto the flanges", yc),
            Waypoint(0.5, (tx, ty, g_drop), CLEAT_PINCH, "pause", yc),
            Waypoint(2.5, (tx, ty, g_low), CLEAT_PINCH, "slide it down the flanges, still held", yc),
            Waypoint(0.8, (tx, ty, g_low), CLEAT_EASE, "ease the fingers off", yc),
            Waypoint(1.0, (tx, ty, g_low + 0.05), CLEAT_EASE, "up", yc),
            Waypoint(0.6, (tx, ty, g_drop + 0.03), FINGER_CLOSED, "close the fingers", yc),
            Waypoint(1.5, (tx, ty, press), FINGER_CLOSED, "press it down to the table", yc),
            Waypoint(0.8, (tx, ty, press), FINGER_CLOSED, "hold", yc),
            Waypoint(1.2, (tx, ty, press + 0.05), FINGER_CLOSED, "up", yc),
        ]

    right = [Waypoint(1.5, PARK["right"], FINGER_OPEN, "aside"),
             Waypoint(t_home - 1.5, PARK["right"], FINGER_OPEN, "wait aside")]
    right += drive("right", 1.5)
    right.append(Waypoint(1.5, PARK["right"], FINGER_OPEN, "park"))
    t_right = sum(w.duration for w in right)
    left.append(Waypoint(t_right - t_home, PARK["left"], FINGER_OPEN, "wait aside"))
    left += drive("left", 1.5)
    left.append(Waypoint(1.5, PARK["left"], FINGER_OPEN, "park"))
    return {"left": left, "right": right}
