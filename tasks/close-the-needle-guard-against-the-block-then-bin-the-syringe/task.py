"""On the station table, in front of the right arm: a fixed white tray with a low rim, a grey block standing on it and
two pairs of low ridges. A used syringe lies across the ridges, its needle pointing away from the robot, with the
orange safety guard at the needle's base swung open, square to the needle. To the robot's left of the tray stands a
fixed open-top yellow sharps container.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, ObjectState  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WHITE, CLEAR, GREY, STEEL, ORANGE, YELLOW, RED = ((0.93, 0.93, 0.92), (0.80, 0.85, 0.88), (0.45, 0.46, 0.48),
                                                   (0.70, 0.71, 0.73), (0.95, 0.45, 0.08), (0.95, 0.80, 0.10),
                                                   (0.80, 0.15, 0.12))

TRAY_BLUE, BLOCK_GREY = (0.16, 0.22, 0.32), (0.72, 0.73, 0.75)

# Tray (fixed). Floor 4 mm thick; rim 4 mm wide, 15 mm above the floor.
TRAY_C, TRAY_IN, TRAY_FLOOR, RIM_W, RIM_H = (0.340, -0.165), (0.240, 0.160), 0.004, 0.004, 0.015
FLOOR_TOP = TOP + TRAY_FLOOR
RIDGE_H, RIDGE_GAP, RIDGE_W, RIDGE_L = 0.011, 0.008, 0.004, 0.010  # ridge pairs: 8 mm apart inside
SYR0 = (0.310, -0.200)  # the barrel's middle at the start
RIDGE_X = (-0.030, 0.030)  # ridge pairs, along the barrel from its middle
BARREL_R, BARREL_L = 0.009, 0.090
AXIS_Z = FLOOR_TOP + RIDGE_H + math.sqrt(BARREL_R ** 2 - (RIDGE_GAP / 2) ** 2)  # the barrel's axis, resting
FLANGE_HALF = AXIS_Z - FLOOR_TOP - 0.0002  # the flange's lower wing 0.2 mm above the tray floor

BLOCK_X, BLOCK_Y, BLOCK_H = (0.380, 0.400), (-0.150, -0.130), 0.050  # activation block on the tray floor


def _tray_shapes():
    cx, cy = TRAY_C
    ox, oy = TRAY_IN[0] + 2 * RIM_W, TRAY_IN[1] + 2 * RIM_W
    rim_z = TRAY_FLOOR + RIM_H / 2
    out = [
        parts.box((ox, oy, TRAY_FLOOR), at=(0.0, 0.0, TRAY_FLOOR / 2), color=TRAY_BLUE),
        parts.box((RIM_W, oy, RIM_H), at=(-ox / 2 + RIM_W / 2, 0.0, rim_z), color=TRAY_BLUE),
        parts.box((RIM_W, oy, RIM_H), at=(ox / 2 - RIM_W / 2, 0.0, rim_z), color=TRAY_BLUE),
        parts.box((TRAY_IN[0], RIM_W, RIM_H), at=(0.0, oy / 2 - RIM_W / 2, rim_z), color=TRAY_BLUE),
        parts.box((TRAY_IN[0], RIM_W, RIM_H), at=(0.0, -oy / 2 + RIM_W / 2, rim_z), color=TRAY_BLUE),
        parts.box((BLOCK_X[1] - BLOCK_X[0], BLOCK_Y[1] - BLOCK_Y[0], BLOCK_H),
                  at=(sum(BLOCK_X) / 2 - cx, sum(BLOCK_Y) / 2 - cy, TRAY_FLOOR + BLOCK_H / 2), color=BLOCK_GREY),
    ]
    for rx in RIDGE_X:
        for s in (-1, 1):
            out.append(parts.box((RIDGE_L, RIDGE_W, RIDGE_H),
                                 at=(SYR0[0] + rx - cx, SYR0[1] + s * (RIDGE_GAP + RIDGE_W) / 2 - cy,
                                     TRAY_FLOOR + RIDGE_H / 2), color=GREY))
    return out


TRAY = parts.Part("tray", pos=(*TRAY_C, TOP), shapes=_tray_shapes())

# Sharps container (fixed, open top).
BIN_C, BIN_IN, BIN_WALL, BIN_FLOOR, BIN_H = (0.300, 0.000), (0.220, 0.090), 0.004, 0.004, 0.090
BIN_FLOOR_TOP = TOP + BIN_FLOOR
BIN_RIM = TOP + BIN_H


def _bin_shapes():
    ox, oy = BIN_IN[0] + 2 * BIN_WALL, BIN_IN[1] + 2 * BIN_WALL
    wz = (BIN_H + BIN_FLOOR) / 2
    wh = BIN_H - BIN_FLOOR
    return [
        parts.box((ox, oy, BIN_FLOOR), at=(0.0, 0.0, BIN_FLOOR / 2), color=YELLOW),
        parts.box((BIN_WALL, oy, wh), at=(-ox / 2 + BIN_WALL / 2, 0.0, wz), color=YELLOW),
        parts.box((BIN_WALL, oy, wh), at=(ox / 2 - BIN_WALL / 2, 0.0, wz), color=YELLOW),
        parts.box((BIN_IN[0], BIN_WALL, wh), at=(0.0, oy / 2 - BIN_WALL / 2, wz), color=YELLOW),
        parts.box((BIN_IN[0], BIN_WALL, wh), at=(0.0, -oy / 2 + BIN_WALL / 2, wz), color=RED),
    ]


BIN = parts.Part("sharps_bin", pos=(*BIN_C, TOP), shapes=_bin_shapes())

# Syringe (loose). Part frame: origin on the barrel's axis at its middle; x toward the needle.
HUB_X = (BARREL_L / 2, BARREL_L / 2 + 0.010)
NEEDLE_R, NEEDLE_L = 0.003, 0.040
NEEDLE_X = (HUB_X[1], HUB_X[1] + NEEDLE_L)  # the tip at 95 mm
GUARD_PIVOT = (HUB_X[0] + 0.007, 0.0, 0.0)  # vertical hinge in the hub
GUARD_L, GUARD_T = 0.046, 0.010  # the closed guard reaches 3 mm past the tip
GUARD_SPRING = 0.02  # N m/rad: holds the guard open (or, once caught, shut)
CATCH = math.pi / 2 - math.radians(4.0)  # the guard clicks shut within 4 deg of closed
SHUT_TOL = math.radians(10.0)  # scored closed: caught, and within 10 deg of lying along the needle

GUARD = parts.Hinge(
    "guard",
    shapes=[parts.box((GUARD_T, GUARD_L, GUARD_T), at=(GUARD_PIVOT[0], GUARD_L / 2, 0.0), color=ORANGE)],
    pivot=GUARD_PIVOT,
    axis=(0.0, 0.0, -1.0),  # positive swings it from square to the needle (+y) to along it (+x)
    lower=0.0,
    upper=math.pi / 2,
    mass=0.010,
    spring=GUARD_SPRING,
    damping=0.002,
)
SYRINGE = parts.Part(
    "syringe",
    pos=(*SYR0, AXIS_Z),
    mass=0.030,
    shapes=[
        parts.cylinder(BARREL_R, BARREL_L, at=(0.0, 0.0, 0.0), color=CLEAR, axis="x"),
        # Finger flange, its two wings up and down: the lower wing rests on the tray floor, so the syringe can't roll.
        parts.box((0.004, 0.012, 2 * FLANGE_HALF), at=(-BARREL_L / 2 - 0.002, 0.0, 0.0), color=CLEAR),
        parts.cylinder(0.003, 0.028, at=(-BARREL_L / 2 - 0.018, 0.0, 0.0), color=WHITE, axis="x"),  # plunger rod
        parts.cylinder(0.008, 0.004, at=(-BARREL_L / 2 - 0.034, 0.0, 0.0), color=WHITE, axis="x"),  # thumb press
        parts.cylinder(0.005, HUB_X[1] - HUB_X[0], at=(sum(HUB_X) / 2, 0.0, 0.0), color=ORANGE, axis="x"),
        parts.cylinder(NEEDLE_R, NEEDLE_L, at=(sum(NEEDLE_X) / 2, 0.0, 0.0), color=STEEL, axis="x"),
    ],
    joints=[GUARD],
)

OBJECTS = [TRAY, BIN, SYRINGE]

EPS = 1e-6
SWING_FROM = math.radians(30.0)  # closing is judged from 30 deg on (60 deg short of closed)
TOUCH = 0.003  # the guard touching a fixed surface: within 3 mm
HAND_CLEAR = 0.020  # no finger pad point within 20 mm of the guard's centre line while it closes
ON = 0.0015
FELL = 0.002  # down on the table: lowest point within 2 mm of the table top


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _fixed_boxes():
    """(centre, half size) of every fixed surface, axis-aligned in the world."""
    out = []
    for part in (TRAY, BIN):
        for _, c, r, s in part.world_boxes({"base": (np.asarray(part.pos, float), part.rotation)}):
            out.append((np.asarray(c), np.abs(r) @ (np.asarray(s) / 2)))
    return out


FIXED = _fixed_boxes()


def guard_points(syr, n=12, start=0.0):
    """World points along the guard's centre line, from ``start`` (a fraction of its length out from the hinge) to
    its end."""
    lp, lq = syr.links["guard"]
    r = _rot(lq)
    return [np.asarray(lp) + r @ np.array([0.0, GUARD_L * (start + (1 - start) * k / (n - 1)), 0.0]) for k in range(n)]


def _box_dist(p, c, h) -> float:
    return float(np.linalg.norm(np.maximum(np.abs(p - c) - h, 0.0)))


def _touching(syr) -> bool:
    """Some part of the guard's outer half within 3 mm of a fixed surface (its faces are GUARD_T / 2 off its centre
    line)."""
    return any(_box_dist(p, c, h) <= GUARD_T / 2 + TOUCH + EPS for p in guard_points(syr, start=0.5)
               for c, h in FIXED)


def true_low(syr) -> float:
    """The syringe's true lowest point (its bounding box counts a turned cylinder as a box, which reaches lower)."""
    r = _rot(syr.quat_xyzw)
    p = np.asarray(syr.pos, float)
    low = math.inf
    for s in SYRINGE.shapes:
        c = p + r @ np.asarray(s.at)
        h = np.asarray(s.half)
        if s.kind == "cylinder":
            axis = r[:, 0] if s.axis == "x" else (r[:, 1] if s.axis == "y" else r[:, 2])
            rad, length = s.size[0], s.size[1]
            az = abs(axis[2])
            low = min(low, c[2] - length / 2 * az - rad * math.sqrt(max(0.0, 1 - az * az)))
        else:
            low = min(low, c[2] - float(np.abs(r[2]) @ h))
    lp, lq = syr.links["guard"]
    rg = _rot(lq)
    for s in GUARD.shapes:
        c = np.asarray(lp) + rg @ (np.asarray(s.at) - np.asarray(GUARD_PIVOT))
        low = min(low, c[2] - float(np.abs(rg[2]) @ np.asarray(s.half)))
    return float(low)


def _over(syr, centre, inner) -> bool:
    c = (np.asarray(syr.low) + np.asarray(syr.high)) / 2
    return bool(abs(c[0] - centre[0]) < inner[0] / 2 and abs(c[1] - centre[1]) < inner[1] / 2)


# Grippers: an on_step callback reads both arms' joints each control step and keeps, per gripper, its tip point (between
# the finger tips, from the arm's own kinematics), its half opening, and sample points over both finger pads (each pad
# 40 mm wide and the 40 mm of finger above its tip, at the finger's current opening). Offline there are no grippers;
# near_misses.py can pass stand-in grippers as a pseudo-object named "grippers" (a GripperProbe).
BASES = {"left_robot": (0.0, 0.305, 0.75), "right_robot": (0.0, -0.305, 0.75)}
FINGER_T, HANDS_OFF = 0.010, 0.005
PAD_W, PAD_LEN = 0.040, 0.040
_GRIP: dict = {}


class GripperProbe(ObjectState):
    """Offline stand-in grippers for near misses: ``grippers`` is {name: (tip, half_opening, pad_points)}."""

    @classmethod
    def make(cls, grippers: dict) -> "GripperProbe":
        z = np.zeros(3)
        probe = cls(z, np.array([0.0, 0.0, 0.0, 1.0]), z, z, z, z)
        object.__setattr__(probe, "grippers", grippers)
        return probe


def pad_points(tip, rot, half):
    """Sample points over both finger pads of a gripper with TCP rotation rot, tip point tip and half opening half."""
    from yam_arena.demo import grip_rotation

    r0 = grip_rotation(0.0)
    close = rot @ (r0.T @ np.array([0.0, 1.0, 0.0]))  # the fingers' closing direction
    width = rot @ (r0.T @ np.array([1.0, 0.0, 0.0]))  # across a pad
    up = -rot[:, 2]  # from the tip back up the finger
    pts = []
    for sgn in (-1.0, 1.0):
        for w in (-0.5, 0.0, 0.5):
            for t in (0.0, 0.5, 1.0):
                pts.append(np.asarray(tip) + sgn * half * close + w * PAD_W * width + t * PAD_LEN * up)
    return pts


@parts.on_step
def _read_grippers(sim) -> None:
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, PAD_TIP_PAST_TCP

    base = sim.env.unwrapped
    for name, bpos in BASES.items():
        robot = base.scene[name]
        arm_ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
        fin_ids = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        q = robot.data.joint_pos.torch[0].detach().cpu().numpy().astype(float)
        pos, rot = ik.tcp_pose(q[arm_ids], bpos)
        tip = pos + rot @ np.array([0.0, 0.0, PAD_TIP_PAST_TCP])
        half = float(np.max(np.abs(q[fin_ids])))
        _GRIP[name] = (tip, half, pad_points(tip, rot, half))


_PROBE: dict = {}


def _grippers():
    return _PROBE if _PROBE else _GRIP


def _hand_at_guard(syr) -> bool:
    """Some finger pad point within HAND_CLEAR of the guard's centre line, anywhere along it."""
    pts = guard_points(syr)
    return any(np.linalg.norm(np.asarray(q) - p) < HAND_CLEAR for _, _, pads in _grippers().values() for q in pads
               for p in pts)


def _released(syr) -> bool:
    lo, hi = np.asarray(syr.low, float), np.asarray(syr.high, float)
    for tip, half, _ in _grippers().values():
        if tip[2] > hi[2] + HANDS_OFF:
            continue
        side = np.maximum(np.maximum(lo[:2] - tip[:2], tip[:2] - hi[:2]), 0.0)
        if np.linalg.norm(side) <= half + FINGER_T + HANDS_OFF:
            return False
    return True


# The guard catch: the spring holds the guard open until it is within 4 deg of closed; then the catch holds it shut
# for the rest of the trial (a scene reset reopens it).
_catch = {"caught": False}


@parts.on_reset
def _reset_catch(sim):
    _catch["caught"] = False
    sim.set_target("syringe", "guard", 0.0)


@parts.on_step
def _guard_catch(sim):
    q = float(sim.states()["syringe"].joints["guard"])
    if not _catch["caught"] and q >= CATCH:
        _catch["caught"] = True
    sim.set_target("syringe", "guard", math.pi / 2 if _catch["caught"] else 0.0)


# Whole-trial record. "closed_ok": the guard reached the catch, and at every step on the way from 30 deg open it was
# touching a fixed surface (block, tray rim or floor, container) with no gripper's tip point within 20 mm of its outer
# half, and with the syringe over the tray. "closed_bad": it closed any other way. "dropped": the syringe was ever down
# on the table outside the tray and the container. Cleared only by a scene reset: the syringe back within 5 mm of
# its start, at rest and its guard open, after a jump of over 20 mm in one control step.
_START = np.array([*SYR0, AXIS_Z])
RESET_TOL = 0.005  # the syringe settles about 3 mm from its written start on the ridges
_record = {"closed_ok": False, "closed_bad": False, "fault": False, "dropped": False, "prev": None}


def _update(syr, objects=None) -> None:
    _PROBE.clear()
    probe = (objects or {}).get("grippers")
    if hasattr(probe, "grippers"):  # duck-typed: near_misses may import its own copy of this module
        _PROBE.update(probe.grippers)
    pos = np.asarray(syr.pos, float)
    prev = _record["prev"]
    if (prev is not None and np.linalg.norm(pos - _START) < RESET_TOL and syr.resting() and syr.joints["guard"] < 0.05
            and np.linalg.norm(pos - prev) > 0.020):
        _record.update(closed_ok=False, closed_bad=False, fault=False, dropped=False)
    _record["prev"] = pos
    q = syr.joints["guard"]
    done = _record["closed_ok"] or _record["closed_bad"]
    if not done and q >= SWING_FROM - EPS:
        if not (_touching(syr) and not _hand_at_guard(syr) and _over(syr, TRAY_C, TRAY_IN)):
            _record["fault"] = True
        if q >= CATCH - EPS:
            _record["closed_bad" if _record["fault"] else "closed_ok"] = True
    elif not done and q < SWING_FROM:
        _record["fault"] = False  # swung back open before closing: a fresh attempt
    if true_low(syr) <= TOP + FELL and not (_over(syr, TRAY_C, TRAY_IN) or _over(syr, BIN_C, BIN_IN)):
        _record["dropped"] = True


def _guarded(syr) -> bool:
    return bool(_record["closed_ok"] and not _record["closed_bad"] and syr.joints["guard"] >= math.pi / 2 - SHUT_TOL - EPS)


def _in_bin(syr) -> bool:
    lo, hi = np.asarray(syr.low), np.asarray(syr.high)
    x0, x1 = BIN_C[0] - BIN_IN[0] / 2, BIN_C[0] + BIN_IN[0] / 2
    y0, y1 = BIN_C[1] - BIN_IN[1] / 2, BIN_C[1] + BIN_IN[1] / 2
    return bool(lo[0] >= x0 - EPS and hi[0] <= x1 + EPS and lo[1] >= y0 - EPS and hi[1] <= y1 + EPS
                and hi[2] <= BIN_RIM + EPS and abs(true_low(syr) - BIN_FLOOR_TOP) <= ON + EPS)


def _on_tray(syr) -> bool:
    """Lying on the tray: over it, its lowest point (the flange wing, on the ridges) within 1.5 mm of the floor."""
    return bool(_over(syr, TRAY_C, TRAY_IN) and abs(true_low(syr) - FLOOR_TOP) <= ON + EPS)


def success(objects):
    syr = objects["syringe"]
    _update(syr, objects)
    return bool(not _record["dropped"] and _guarded(syr) and _in_bin(syr) and syr.resting() and _released(syr))


def distance(objects):
    syr = objects["syringe"]
    return float(np.hypot(syr.pos[0] - BIN_C[0], syr.pos[1] - BIN_C[1]))


def score(objects):
    """1.0 on success; 0.5 the guard closed against a surface and the syringe released and still on the tray; 0
    otherwise (and 0 for the trial once the guard has closed another way or the syringe has been on the table)."""
    syr = objects["syringe"]
    _update(syr, objects)
    if _record["dropped"] or _record["closed_bad"] or not _guarded(syr) or not syr.resting() or not _released(syr):
        return 0.0
    if _in_bin(syr):
        return 1.0
    return 0.5 if _on_tray(syr) else 0.0


# --- demo: right arm only ------------------------------------------------------------------------------------------
TIP = 0.013
GRIP_DZ = 0.008  # TCP above the barrel's axis: finger tips 5 mm below it
LIFT = 0.015  # held this far above its resting height while working the guard
LET_GO = -0.020  # fingers opened to 40 mm in the container


CONTACT_R = 0.035  # the block's corner works on the guard 35 mm out from its hinge (the guard's outer half)


def _syringe_at(deg, push):
    """Where the barrel's middle must be for the block's near corner to touch the guard, turned deg from open, 35 mm
    out along it; push > 0 presses the corner that far into the guard."""
    t = math.radians(deg)
    d = np.array([math.sin(t), math.cos(t)])  # along the guard (0 deg: +y, 90 deg: +x)
    n = np.array([-math.cos(t), math.sin(t)])  # the guard face the block works on
    corner = np.array([BLOCK_X[1], BLOCK_Y[0]])
    rel = np.array(GUARD_PIVOT[:2]) + CONTACT_R * d + (GUARD_T / 2 - push) * n
    return corner - rel


def demo():
    """Right arm: grip the barrel at its middle, lift it a little, bring the open guard against the block's near
    corner, then move the syringe so the corner rolls the guard round, working on its outer half, until it lies along
    the needle and clicks; then carry the syringe over the container, lower it and let go."""
    z = AXIS_Z + GRIP_DZ
    zl = z + LIFT
    sx, sy = SYR0
    carry = BIN_RIM + BARREL_R + 0.025 + GRIP_DZ
    put = BIN_FLOOR_TOP + FLANGE_HALF + 0.012 + GRIP_DZ
    a = _syringe_at(0, -0.004)
    right = [
        Waypoint(1.5, (sx, sy, z + 0.05), -0.03, "over the barrel"),
        Waypoint(1.2, (sx, sy, z), -0.03, "down"),
        Waypoint(0.8, (sx, sy, z), FINGER_CLOSED, "grip"),
        Waypoint(0.8, (sx, sy, zl), FINGER_CLOSED, "lift a little"),
        Waypoint(1.8, (a[0], a[1] - 0.02, zl), FINGER_CLOSED, "along, the guard's end clear of the block"),
        Waypoint(1.2, (a[0], a[1], zl), FINGER_CLOSED, "guard just past the block's corner"),
    ]
    for deg in range(10, 100, 10):
        p = _syringe_at(min(deg, 92), 0.002)
        right.append(Waypoint(0.5, (p[0], p[1], zl), FINGER_CLOSED, f"roll the guard {deg}"))
    p = _syringe_at(92, 0.002)
    right += [
        Waypoint(0.6, (p[0], p[1], zl), FINGER_CLOSED, "hold"),
        Waypoint(1.0, (p[0] - 0.01, p[1] - 0.015, zl), FINGER_CLOSED, "away from the block"),
        Waypoint(1.5, (p[0] - 0.01, p[1] - 0.015, carry), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (BIN_C[0], BIN_C[1], carry), FINGER_CLOSED, "over the container"),
        Waypoint(1.5, (BIN_C[0], BIN_C[1], put), FINGER_CLOSED, "lower in"),
        Waypoint(0.6, (BIN_C[0], BIN_C[1], put), LET_GO, "let go"),
        Waypoint(1.5, (BIN_C[0], BIN_C[1], carry + 0.03), LET_GO, "up"),
    ]
    left = [Waypoint(2.0, (0.15, 0.30, TOP + 0.15), FINGER_CLOSED, "parked")]
    return {"left": left, "right": right}
