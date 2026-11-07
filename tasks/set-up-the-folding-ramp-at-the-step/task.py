"""On the station table: a fixed grey step, 35 mm high and 200 mm wide, with its front face toward the robot. In
front of it lies a loose aluminium folding ramp, folded: two panels, each 100 mm long and 70 mm wide with a dark deck
and a 25 mm high silver rail along each side, joined by a hinge across the rail tops at the end nearest the robot.
Folded, one panel lies face down on the other, rails to rails.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Ramp. Part frame: origin on the table under the hinge, x away from the arms, y to the robot's left. The base panel
# lies on x 0..L; the hinge runs along y at the rail tops over x = 0. The second panel starts folded on top of it.
L, W = 0.100, 0.070  # panel length and width
DECK_T, RAIL_W, RAIL_H = 0.006, 0.006, 0.025
PIVOT = (0.0, 0.0, RAIL_H)
X_H, Y_R = 0.22, 0.02  # the hinge's world x, y at the start
DECK, RAIL, BARREL = (0.22, 0.23, 0.24), (0.78, 0.79, 0.81), (0.45, 0.46, 0.48)
RAIL_Y = W / 2 - RAIL_W / 2


def _panel(z0: float, flip: bool):
    """A panel's shapes on x 0..L. flip: lying face down, with its deck on top (z0 is the panel's lowest point)."""
    deck_z = z0 + (RAIL_H - DECK_T / 2 if flip else DECK_T / 2)
    return [
        parts.box((L, W, DECK_T), at=(L / 2, 0.0, deck_z), color=DECK),
        parts.box((L, RAIL_W, RAIL_H), at=(L / 2, RAIL_Y, z0 + RAIL_H / 2), color=RAIL),
        parts.box((L, RAIL_W, RAIL_H), at=(L / 2, -RAIL_Y, z0 + RAIL_H / 2), color=RAIL),
    ]


CATCH_K = 4.0  # N m/rad: the hinge catch's stiffness once it has caught
CATCH_AT = math.pi - math.radians(2.0)  # the catch engages within 2 deg of flat

HINGE = parts.Hinge(
    "hinge",
    shapes=_panel(RAIL_H, flip=True),
    pivot=PIVOT,
    axis=(0.0, -1.0, 0.0),  # positive lifts the folded panel's free end and swings it over toward the robot
    lower=0.0,
    upper=math.pi,  # flat: the two panels in line
    mass=0.10,
    spring=CATCH_K,
    damping=0.04,
)
RAMP = parts.Part(
    "ramp",
    pos=(X_H, Y_R, TOP),
    mass=0.12,
    shapes=[*_panel(0.0, flip=False),
            parts.cylinder(0.004, W - 2 * RAIL_W - 0.004, at=PIVOT, color=BARREL, axis="y")],
    joints=[HINGE],
)

STEP_X0, STEP_SIZE, STEP_Y = 0.37, (0.100, 0.200, 0.035), Y_R  # step front face, size, centre y
STEP_TOP = TOP + STEP_SIZE[2]
STEP = Box("step", pos=(STEP_X0 + STEP_SIZE[0] / 2, STEP_Y, TOP + STEP_SIZE[2] / 2), size=STEP_SIZE,
           color=(0.55, 0.53, 0.50))

OBJECTS = [RAMP, STEP]

# The hinge catch. Until it catches, the hinge's drive target follows the hinge, so it turns freely; once the ramp
# is opened to within 2 deg of flat, the target is set to flat and the hinge stays straight (it no longer folds or
# sags at the middle). It stays caught until the scene is reset.
_catch = {"caught": False, "prev": 0.0}


@parts.on_reset
def _reset_catch(sim):
    _catch.update(caught=False, prev=0.0)
    sim.set_target("ramp", "hinge", 0.0)


@parts.on_step
def _hinge_catch(sim):
    q = float(sim.states()["ramp"].joints["hinge"])
    if not _catch["caught"] and q >= CATCH_AT:
        _catch["caught"] = True
    if _catch["caught"]:
        sim.set_target("ramp", "hinge", math.pi)
    else:
        sim.set_target("ramp", "hinge", q + 0.5 * (q - _catch["prev"]))  # free: the target leads the hinge a little
    _catch["prev"] = q

EPS = 1e-6
FLAT = math.radians(8.0)  # unfolded: the hinge within 8 deg of flat
UP = math.radians(30.0)  # rails up: the base panel's deck within 30 deg of facing up
ON_FLOOR = 0.0015  # the low end's bottom edge at most 1.5 mm above the table
ON_EDGE = 0.002  # the underside within 2 mm of the step's top at its front edge
ONTO = 0.015  # the high end at least 15 mm past the step's front edge
SQUARE = math.radians(10.0)  # the ramp's run within 10 deg of square to the step's front face
FELL = 0.005  # off the table: lowest point more than 5 mm below the table top


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def ends(ramp):
    """World points at the middle of each free end's bottom edge: (base panel end, second panel end). For the second
    panel, its unfolded bottom face is the face that is up while folded."""
    a = np.asarray(ramp.pos) + _rot(ramp.quat_xyzw) @ np.array([L, 0.0, 0.0])
    lp, lq = ramp.links["hinge"]
    b = np.asarray(lp) + _rot(lq) @ (np.array([L, 0.0, 2 * RAIL_H]) - np.asarray(PIVOT))
    return a, b


def _unfolded(ramp) -> bool:
    """The hinge within 8 deg of flat, and the rails up (the base panel's deck facing within 30 deg of up)."""
    up = _rot(ramp.quat_xyzw)[2, 2]
    return bool(ramp.joints["hinge"] >= math.pi - FLAT - EPS and up >= math.cos(UP) - EPS)


def _on_table(ramp) -> bool:
    return bool(ramp.low[2] >= TOP - FELL - EPS)


def _end_down(p) -> bool:
    """An end's bottom edge resting on the table, or on the step's top over the step."""
    on_floor = abs(p[2] - TOP) <= ON_FLOOR + EPS
    over_step = STEP_X0 - EPS <= p[0] <= STEP_X0 + STEP_SIZE[0] + EPS and abs(p[1] - STEP_Y) <= STEP_SIZE[1] / 2 + EPS
    return bool(on_floor or (over_step and abs(p[2] - STEP_TOP) <= ON_FLOOR + EPS))


def _across_edge(ramp) -> bool:
    """One end on the floor in front of the step and the underside on the step's front edge (within 2 mm)."""
    a, b = ends(ramp)
    low, high = (a, b) if a[0] < b[0] else (b, a)
    if not (abs(low[2] - TOP) <= ON_FLOOR + EPS and low[0] < STEP_X0 < high[0]):
        return False
    run = high - low
    z_edge = low[2] + run[2] * (STEP_X0 - low[0]) / run[0]
    return bool(abs(z_edge - STEP_TOP) <= ON_EDGE + EPS)


def _supported(ramp) -> bool:
    """Lying on something with no help: both ends' bottom edges down on the table or the step; one end on the floor
    and the underside on the step's edge; or lying level on the step's top with its middle over the step."""
    a, b = ends(ramp)
    mid = (a + b) / 2
    on_step = (abs(ramp.low[2] - STEP_TOP) <= ON_FLOOR + EPS and abs(a[2] - b[2]) <= 0.003
               and STEP_X0 <= mid[0] <= STEP_X0 + STEP_SIZE[0] and abs(mid[1] - STEP_Y) <= STEP_SIZE[1] / 2)
    return bool((_end_down(a) and _end_down(b)) or _across_edge(ramp) or on_step)


# Grippers. The scorer is given only the objects, so an on_step callback reads both arms' joints each control step
# and keeps each gripper's tip point (between the finger tips, from the arm's own kinematics) and half opening. The
# ramp is released when every tip point is more than 5 mm above the ramp's top, or farther from its bounding box
# sideways than its half opening plus 10 mm (a finger's thickness) plus 5 mm. Offline there are no grippers.
BASES = {"left_robot": (0.0, 0.305, 0.75), "right_robot": (0.0, -0.305, 0.75)}
FINGER_T, HANDS_OFF = 0.010, 0.005
_GRIP: dict = {}


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
        _GRIP[name] = (tip, float(np.max(np.abs(q[fin_ids]))))


def _released(ramp) -> bool:
    lo, hi = np.asarray(ramp.low, float), np.asarray(ramp.high, float)
    for tip, half in _GRIP.values():
        if tip[2] > hi[2] + HANDS_OFF:
            continue
        side = np.maximum(np.maximum(lo[:2] - tip[:2], tip[:2] - hi[:2]), 0.0)
        if np.linalg.norm(side) <= half + FINGER_T + HANDS_OFF:
            return False
    return True


def _set_at_step(ramp) -> bool:
    """Unfolded, one end on the floor in front of the step, the underside on the step's front edge, the other end at
    least 15 mm onto the step, square to it within 10 deg and within its width."""
    if not _unfolded(ramp):
        return False
    a, b = ends(ramp)
    low, high = (a, b) if a[0] < b[0] else (b, a)
    if not (TOP - 0.002 <= low[2] <= TOP + ON_FLOOR + EPS and low[0] < STEP_X0):
        return False
    if high[0] < STEP_X0 + ONTO - EPS:
        return False
    run = high - low
    if abs(math.atan2(run[1], run[0])) > SQUARE + EPS:
        return False
    z_edge = low[2] + run[2] * (STEP_X0 - low[0]) / run[0]  # the underside's height over the step's front edge
    if abs(z_edge - STEP_TOP) > ON_EDGE + EPS:
        return False
    half = STEP_SIZE[1] / 2
    across = np.array([-run[1], run[0]]) / np.hypot(run[0], run[1]) * W / 2
    return all(abs(p[1] + s * across[1] - STEP_Y) <= half + EPS for p in (low, high) for s in (-1, 1))


def success(objects):
    """The ramp unfolded and set from the floor onto the step, released and at rest."""
    ramp = objects["ramp"]
    return bool(_set_at_step(ramp) and ramp.resting() and _released(ramp))


def distance(objects):
    a, b = ends(objects["ramp"])
    return float(max(0.0, STEP_X0 + ONTO - max(a[0], b[0])))


def score(objects):
    """1.0 on success; 0.5 unfolded flat, rails up, released, at rest and lying on the table or the step (both ends
    down, or across the step's edge), but not set at the step; 0 otherwise."""
    ramp = objects["ramp"]
    if not ramp.resting() or not _released(ramp) or not _on_table(ramp):
        return 0.0
    if _set_at_step(ramp):
        return 1.0
    return 0.5 if _unfolded(ramp) and _supported(ramp) else 0.0


# --- demo ---------------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
HALF_PADDLE = 0.010  # closed fingers, turned to close along x: 10 mm either side of the TCP in x


def _w(rel_x, rel_z, y=Y_R):
    """World point from x, z relative to the hinge pivot at the start."""
    return (X_H + rel_x, y, TOP + PIVOT[2] + rel_z)


def _swing(u, v, deg):
    """Where a point (u along the second panel from the pivot, v off the pivot plane toward its back) is with the
    panel turned up by deg."""
    t = math.radians(deg)
    return u * math.cos(t) - v * math.sin(t), u * math.sin(t) + v * math.cos(t)


GRIP_U, GRIP_V = L - 0.020, 0.020  # left TCP on the folded panel: 20 mm in from its free end, tips 7 mm above the base
LIFT_TO = 60.0
PUSH_V = RAIL_H - DECK_T  # the deck's inner face, between the rails
OVER_TO = 112.0  # pushed past upright
CARRY_X = STEP_X0 + 0.035 - L  # hinge x when set down: the far end 35 mm past the step's front edge
SET_Z = STEP_TOP + 0.003  # the ramp's underside 3 mm above the step when let go
GRIP_Z = 0.021  # TCP above the ramp's underside when gripping its rails at the hinge


def demo():
    """Left arm: grip the folded panel's free end and swing it up to 60 deg with a tilted grip, hold it while the
    right arm's closed fingers come against its free end, let go and back off. Right arm: push the panel over past
    upright, then lift away so it settles flat. Left arm: grip the unfolded ramp across its rails at the hinge, carry
    it over the step and let go just above it, so the low end drops to the floor."""
    lift_steps = [0, 10, 20, 30, 40, 50, 60]
    left = [
        Waypoint(1.5, _w(*_swing(GRIP_U, GRIP_V + 0.06, 0)), FINGER_OPEN, "over the free end"),
        Waypoint(1.2, _w(*_swing(GRIP_U, GRIP_V, 0)), FINGER_OPEN, "down to the rails"),
        Waypoint(0.8, _w(*_swing(GRIP_U, GRIP_V, 0)), FINGER_CLOSED, "grip"),
    ]
    for d in lift_steps[1:]:
        left.append(tilt.Waypoint(0.5, _w(*_swing(GRIP_U, GRIP_V, d)), FINGER_CLOSED, f"swing up {d}", tilt=d,
                                  heading=0.0))
    lean = dict(tilt=LIFT_TO, heading=0.0)
    t_up = 1.5 + 1.2 + 0.8 + 0.5 * (len(lift_steps) - 1)
    left += [
        tilt.Waypoint(3.5, _w(*_swing(GRIP_U, GRIP_V, LIFT_TO)), FINGER_CLOSED, "hold", **lean),
        tilt.Waypoint(0.8, _w(*_swing(GRIP_U, GRIP_V, LIFT_TO)), FINGER_OPEN, "let go", **lean),
        tilt.Waypoint(1.2, _w(*_swing(GRIP_U, GRIP_V + 0.05, LIFT_TO)), FINGER_OPEN, "back off", **lean),
        Waypoint(1.5, (0.18, 0.22, TOP + 0.13), FINGER_OPEN, "clear to the left"),
    ]
    t_free = t_up + 3.5 + 0.8 + 1.2 + 1.5

    # Right arm: a paddle of closed fingers (closing along x), its -x face against the panel's free end.
    def paddle(deg, push=0.003):
        cx, cz = _swing(L, PUSH_V, deg)
        return _w(cx + HALF_PADDLE - push, cz + TIP - 0.004, Y_R)

    ry = math.pi / 2
    start_x, start_z = paddle(LIFT_TO, push=-0.006)[0] - X_H, paddle(LIFT_TO)[2] - TOP - PIVOT[2]
    right = [
        Waypoint(2.0, (0.15, -0.25, TOP + 0.10), FINGER_CLOSED, "out of the way", ry),
        Waypoint(t_up - 2.0, (0.15, -0.25, TOP + 0.10), FINGER_CLOSED, "wait while the left swings it up", ry),
        Waypoint(1.5, _w(start_x + 0.035, start_z + 0.01), FINGER_CLOSED, "beside it", ry),
        Waypoint(1.2, _w(start_x, start_z), FINGER_CLOSED, "in toward the free end", ry),
        Waypoint(0.8, paddle(LIFT_TO), FINGER_CLOSED, "against the free end", ry),
        Waypoint(t_free - t_up - 3.5, paddle(LIFT_TO), FINGER_CLOSED, "hold while the left lets go", ry),
    ]
    for d in (70, 80, 90, 100, OVER_TO):
        right.append(Waypoint(0.7, paddle(d), FINGER_CLOSED, f"push over {d}", ry))
    px, _, pz = paddle(OVER_TO)
    right += [
        Waypoint(1.0, (px + 0.03, Y_R, pz + 0.03), FINGER_CLOSED, "lift away", ry),
        Waypoint(1.5, (0.15, -0.25, TOP + 0.10), FINGER_OPEN, "park", ry),
    ]
    t_flat = t_free + 0.7 * 5 + 1.0 + 2.5  # time for the panel to settle flat

    travel = SET_Z + GRIP_Z + 0.015
    t_left = sum(w.duration for w in left)
    left += [
        Waypoint(t_flat - t_left - 1.5, (0.18, 0.22, TOP + 0.13), FINGER_OPEN, "wait clear"),
        Waypoint(1.5, (X_H, Y_R, TOP + 0.08), FINGER_OPEN, "over the hinge"),
        Waypoint(1.2, (X_H, Y_R, TOP + GRIP_Z), FINGER_OPEN, "down to the rails"),
        Waypoint(0.8, (X_H, Y_R, TOP + GRIP_Z), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (X_H, Y_R, travel), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (CARRY_X, Y_R, travel), FINGER_CLOSED, "carry over the step"),
        Waypoint(1.0, (CARRY_X, Y_R, SET_Z + GRIP_Z), FINGER_CLOSED, "lower"),
        Waypoint(0.6, (CARRY_X, Y_R, SET_Z + GRIP_Z), FINGER_OPEN, "let go"),
        Waypoint(1.2, (CARRY_X, Y_R, TOP + 0.09), FINGER_OPEN, "up"),
    ]
    return {"left": left, "right": right}
