"""On the station table, in front of the left arm: a black granite surface plate. On it a 0.01 mm dial indicator lies
face up, held by its stem in the clamp of a fixed holder bar on a magnetic base. Its plunger points away from the
robot and its contact point presses against the near face of a loose steel block, which rests on the plate against
a fixed steel parallel. The indicator has a knurled chrome bezel carrying the white graduated dial, with a red zero
line, black lines at 25, 50 and 75 and black dots every 10 divisions, and a black pointer over the dial.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so the reset hooks run before the lots settle)
import lots  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

GRANITE, STEEL, CHROME = (0.12, 0.12, 0.13), (0.55, 0.57, 0.60), (0.80, 0.81, 0.83)
DARK, BLACK, WHITE, RED = (0.25, 0.26, 0.28), (0.05, 0.05, 0.05), (0.96, 0.96, 0.94), (0.85, 0.10, 0.10)
LOW_FRICTION = (0.25, 0.20)  # steel on granite

# -- surface plate, parallel and holder (one fixed part) ------------------------------------------------------------
PLATE = (0.24, 0.24, 0.040)
PLATE_XY = (0.30, 0.14)
P = TOP + PLATE[2]  # the plate's working surface

# Indicator: body axis vertical (face up) through C. Heights are above the plate's surface.
C = (0.25, 0.16)
CASE_R, CASE_Z0, CASE_Z1 = 0.036, 0.005, 0.025
BEZEL_R, BEZEL_Z0, BEZEL_Z1 = 0.0375, 0.025, 0.031  # knurled ring, 75 mm across
DIAL_R = 0.033
STEM_R, STEM_X0, STEM_X1 = 0.004, 0.034, 0.058  # 8 mm stem along +x from the case
AXIS_Z = 0.015  # plunger axis height
LEAD = 0.001  # plunger travel per pointer revolution (m): 100 divisions of 0.01 mm
DIV = 2 * math.pi / 100  # one division of the dial (rad)
TRAVEL = 0.010  # plunger range
TIP_X0 = 0.074  # contact point's end at plunger position 0 (fully pushed in), from C
COMPRESSION = 0.00237  # the block holds the plunger 2.37 mm in from its free end
Q_CONTACT = TRAVEL - COMPRESSION  # plunger position (m, outward from fully in) at the block
POINTER_REVS = 2  # whole pointer turns at that reading (no revolution counter is modelled)

def pointer_target(q: float) -> float:
    """The pointer's joint angle (rad, clockwise from above) for plunger position q (m): one turn per 1 mm."""
    return 2 * math.pi * ((TRAVEL - q) / LEAD - POINTER_REVS)


POINTER_START = pointer_target(Q_CONTACT)  # 0.37 turn: the pointer points 133.2 degrees clockwise from +x

# Holder: clamp on the stem, a bar along y to a magnetic base on the robot's right.
CLAMP_X = 0.052
BASE_XY = (C[0] + CLAMP_X, C[1] - 0.110)
CLAMP = (0.010, 0.016, 0.016)  # around the stem
ROD_LEN = (C[1] - CLAMP[1] / 2) - (BASE_XY[1] + 0.030)  # from the base's left face to the clamp

# The steel block (loose) and the parallel behind it (fixed).
BLOCK = (0.050, 0.040, 0.030)
BLOCK_X0 = C[0] + TIP_X0 + Q_CONTACT  # its near face
BLOCK_POS = (BLOCK_X0 + BLOCK[0] / 2, C[1], P)
PARALLEL = (0.015, 0.080, 0.040)
PARALLEL_X = BLOCK_X0 + BLOCK[0] + PARALLEL[0] / 2

BENCH = parts.Part(
    "bench",
    pos=(*PLATE_XY, TOP),
    friction=LOW_FRICTION,
    shapes=[
        parts.box(PLATE, at=(0.0, 0.0, PLATE[2] / 2), color=GRANITE),
        parts.box(PARALLEL, at=(PARALLEL_X - PLATE_XY[0], C[1] - PLATE_XY[1], PLATE[2] + PARALLEL[2] / 2),
                  color=STEEL),
        # magnetic base and its holder bar along y to the stem clamp
        parts.box((0.050, 0.060, 0.045), at=(BASE_XY[0] - PLATE_XY[0], BASE_XY[1] - PLATE_XY[1], PLATE[2] + 0.0225),
                  color=DARK),
        parts.cylinder(0.005, ROD_LEN, at=(BASE_XY[0] - PLATE_XY[0], BASE_XY[1] + 0.030 + ROD_LEN / 2 - PLATE_XY[1],
                                            PLATE[2] + AXIS_Z), color=STEEL, axis="y"),
        parts.box(CLAMP, at=(C[0] + CLAMP_X - PLATE_XY[0], C[1] - PLATE_XY[1], PLATE[2] + AXIS_Z), color=DARK),
    ],
)

BLOCK_PART = parts.Part(
    "steel_block", pos=BLOCK_POS, mass=0.47, friction=LOW_FRICTION,
    shapes=[parts.box(BLOCK, at=(0.0, 0.0, BLOCK[2] / 2), color=STEEL)],
)

# Bezel: the knurled ring and the dial it carries. Its zero line is along the link's +x at joint 0.
DOT_R = 0.0015
MARK_Z = BEZEL_Z1 + 0.00075
_dots = [parts.cylinder(DOT_R, 0.0005, at=(0.029 * math.cos(-k * 36 * math.pi / 180),
                                            0.029 * math.sin(-k * 36 * math.pi / 180), MARK_Z), color=BLACK)
         for k in range(1, 10)]  # 10, 20, ... 90 divisions, clockwise from 0
BEZEL_SHAPES = [
    parts.cylinder(BEZEL_R, BEZEL_Z1 - BEZEL_Z0, at=(0.0, 0.0, (BEZEL_Z0 + BEZEL_Z1) / 2), color=CHROME),
    parts.cylinder(DIAL_R, 0.0005, at=(0.0, 0.0, BEZEL_Z1 + 0.00025), color=WHITE),
    parts.box((0.013, 0.004, 0.0005), at=(0.0265, 0.0, MARK_Z), color=RED),  # 0
    parts.box((0.0025, 0.008, 0.0005), at=(0.0, -0.028, MARK_Z), color=BLACK),  # 25 (clockwise from 0)
    parts.box((0.008, 0.0025, 0.0005), at=(-0.028, 0.0, MARK_Z), color=BLACK),  # 50
    parts.box((0.0025, 0.008, 0.0005), at=(0.0, 0.028, MARK_Z), color=BLACK),  # 75
    *_dots,
]
K_BEZEL = 0.2  # N m/rad: the drive that holds the bezel where it is left (its friction)
BEZEL = parts.Hinge("bezel", shapes=BEZEL_SHAPES, pivot=(0.0, 0.0, BEZEL_Z0), axis=(0.0, 0.0, 1.0),
                    lower=-6 * math.pi, upper=6 * math.pi, mass=0.04, spring=K_BEZEL, damping=0.005)

# Pointer: turns clockwise (seen from above) as the plunger goes in. Written along +x.
PTR_Z = MARK_Z + 0.00025 + 0.0015 + 0.0005
POINTER = parts.Hinge(
    "pointer",
    shapes=[parts.box((0.031, 0.003, 0.001), at=(0.0085, 0.0, PTR_Z), color=BLACK),
            parts.cylinder(0.003, 0.002, at=(0.0, 0.0, PTR_Z + 0.0005), color=BLACK)],
    pivot=(0.0, 0.0, PTR_Z), axis=(0.0, 0.0, -1.0), lower=-4 * math.pi, upper=4 * math.pi,
    mass=0.01, spring=0.2, damping=0.002,
)
K_PLUNGER = 400.0  # N/m: about 0.95 N gauge force at the block
PLUNGER = parts.Slider(
    "plunger",
    shapes=[parts.cylinder(0.002, 0.020, at=(0.060, 0.0, AXIS_Z), color=CHROME, axis="x"),
            parts.cylinder(0.003, 0.004, at=(TIP_X0 - 0.002, 0.0, AXIS_Z), color=STEEL, axis="x")],
    axis=(1.0, 0.0, 0.0), travel=TRAVEL, mass=0.02, spring=K_PLUNGER, damping=6.0,
)
class _Indicator(parts.Part):
    """The indicator's offline state defaults to its settled start: plunger on the block, pointer at its reading and
    the bezel where the current lot leaves it."""

    def state(self, joints=None, pos=None, quat_xyzw=None, lin_vel=None):
        start = {"plunger": Q_CONTACT, "pointer": POINTER_START, "bezel": math.radians(lots.current().zero_deg)}
        return super().state(joints=start | dict(joints or {}), pos=pos, quat_xyzw=quat_xyzw, lin_vel=lin_vel)


INDICATOR = _Indicator(
    "dial_indicator",
    pos=(*C, P),
    shapes=[
        parts.cylinder(CASE_R, CASE_Z1 - CASE_Z0, at=(0.0, 0.0, (CASE_Z0 + CASE_Z1) / 2), color=CHROME),
        parts.cylinder(STEM_R, STEM_X1 - STEM_X0, at=((STEM_X0 + STEM_X1) / 2, 0.0, AXIS_Z), color=CHROME, axis="x"),
    ],
    joints=[BEZEL, POINTER, PLUNGER],
)
OBJECTS = [BENCH, INDICATOR, BLOCK_PART]



# Lots: the bezel was left turned by the last job. Its zero line starts at a world angle (degrees, counter-clockwise
# from +x seen from above); the pointer is at -133.2 (226.8) in every lot. The block is "moved" by zero so every lot
# settles again after the reset hooks.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", zero_deg=331.8, place={"steel_block": BLOCK_POS}),
    lots.Lot("B", zero_deg=151.8, place={"steel_block": BLOCK_POS}),
    lots.Lot("C", zero_deg=266.8, place={"steel_block": BLOCK_POS}),
)

EPS = 1e-6
FULL_TOL = 2 * DIV + EPS  # 2 divisions (7.2 degrees)
PART_TOL = 5 * DIV + EPS  # 5 divisions (18 degrees)
READING_TOL = 2 * LEAD / 100  # the plunger may move at most 2 divisions (0.02 mm) from its start
BLOCK_MOVE, BLOCK_TURN = 0.0005, math.radians(1.0)
CLEAR = 0.005  # released: nothing of either gripper within 5 mm of the indicator or the block
STILL = math.radians(0.5)  # the bezel turned less than this in the last control step
SUCCESS_HOLD_S = 2.0


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def zero_error(ind) -> float:
    """Angle (rad) from the pointer to the bezel's zero line, seen from above, in -pi..pi."""
    # The zero line's world angle is the bezel joint (counter-clockwise; the line lies along +x at 0); the pointer's
    # is minus its joint (clockwise).
    return _wrap(ind.joints["bezel"] + ind.joints["pointer"])


def _quat_turn(q) -> float:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return 2 * math.acos(min(1.0, abs(w)))


# Whole-trial record, cleared only by a scene reset (the simulator's reset hook; offline, the exact start state).
_record = {"fault": False, "ref_q": Q_CONTACT, "ref_block": None, "need_ref": False}
# Measured in the simulator every step: are both grippers clear, and is the bezel still? None until measured, which
# scores nothing (fail closed). Offline there is no robot: both count as True.
_sim = {"on": False, "clear": None, "still": None, "prev_bezel": None}


def _start_offline(objects) -> bool:
    ind, blk = objects["dial_indicator"], objects["steel_block"]
    s = INDICATOR.state()
    return bool(
        all(abs(ind.joints[k] - s.joints[k]) < 1e-9 for k in s.joints)
        and np.allclose(blk.pos, lots.current().pos("steel_block"), atol=1e-9)
    )


def _update(objects) -> None:
    ind, blk = objects["dial_indicator"], objects["steel_block"]
    if not _sim["on"] and _start_offline(objects):
        _record.update(fault=False, ref_q=Q_CONTACT, ref_block=None)
    if _record["ref_block"] is None:
        _record["ref_block"] = (np.asarray(blk.pos, float).copy(), np.asarray(blk.quat_xyzw, float).copy())
    if abs(ind.joints["plunger"] - _record["ref_q"]) > READING_TOL + 1e-9:
        _record["fault"] = True
    p0, q0 = _record["ref_block"]
    dq = _quat_mul(_quat_conj(q0), np.asarray(blk.quat_xyzw, float))
    if np.linalg.norm(np.asarray(blk.pos) - p0) > BLOCK_MOVE + EPS or _quat_turn(dq) > BLOCK_TURN + EPS:
        _record["fault"] = True


def _quat_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def _released() -> bool:
    if not _sim["on"]:
        return True
    return bool(_sim["clear"] is True and _sim["still"] is True)


def _level(objects) -> float:
    """1.0, 0.5 or 0 for the zero line's error alone, after release, with no fault."""
    if _record["fault"] or not _released() or not objects["steel_block"].resting():
        return 0.0
    e = abs(zero_error(objects["dial_indicator"]))
    return 1.0 if e <= FULL_TOL else 0.5 if e <= PART_TOL else 0.0


def success(objects):
    """The bezel's zero within 2 divisions of the pointer, let go and still, the setup never disturbed."""
    _update(objects)
    return _level(objects) == 1.0


def distance(objects):
    return abs(zero_error(objects["dial_indicator"]))


def score(objects):
    """1.0 within 2 divisions; 0.5 within 5; else 0. Zero for the whole trial once the setup is disturbed."""
    _update(objects)
    return _level(objects)


# -- simulator hooks -------------------------------------------------------------------------------------------------
@parts.on_reset
def _reset(sim) -> None:
    _sim.update(on=True, clear=None, still=None, prev_bezel=None)
    _record.update(fault=False, ref_q=Q_CONTACT, ref_block=None, need_ref=True)
    sim.set_target("dial_indicator", "plunger", TRAVEL)  # the spring pushes the plunger out onto the block
    sim.set_target("dial_indicator", "pointer", POINTER_START)
    sim.set_target("dial_indicator", "bezel", math.radians(lots.current().zero_deg))
    _DEMO["look"] = True


@parts.on_step
def _step(sim) -> None:
    from yam_arena import analysis

    _sim["on"] = True
    st = sim.states()
    ind = st["dial_indicator"]
    if _record["need_ref"]:  # the first step after a reset: the settled reading and block pose
        _record.update(ref_q=float(ind.joints["plunger"]), need_ref=False,
                       ref_block=(np.asarray(st["steel_block"].pos, float).copy(),
                                  np.asarray(st["steel_block"].quat_xyzw, float).copy()))
    q = float(ind.joints["plunger"])
    sim.set_target("dial_indicator", "plunger", TRAVEL)
    sim.set_target("dial_indicator", "pointer", pointer_target(q))
    b = float(ind.joints["bezel"])
    sim.set_target("dial_indicator", "bezel", b)  # friction: the bezel stays where it is left
    prev = _sim["prev_bezel"]
    _sim["still"] = prev is not None and abs(b - prev) < STILL
    _sim["prev_bezel"] = b
    # Released: no part of either arm's fingers or gripper within 5 mm of the indicator (all links) or the block.
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    hulls, bodies = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right", "gripper"):
                m = np.eye(4)
                m[:3, :3] = parts._quat_to_matrix(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
                bodies += 1
    links = {"base": (np.asarray(ind.pos), parts._quat_to_matrix(ind.quat_xyzw))}
    links.update({k: (np.asarray(v[0]), parts._quat_to_matrix(v[1])) for k, v in ind.links.items()})
    boxes = INDICATOR.world_boxes(links)
    blk = st["steel_block"]
    boxes += BLOCK_PART.world_boxes({"base": (np.asarray(blk.pos), parts._quat_to_matrix(blk.quat_xyzw))})
    # Fail closed: both arms' two fingertips and gripper (six bodies) must all be found and measured.
    _sim["clear"] = bool(bodies == 6 and hulls
                         and all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for _, c, r, s in boxes))
    if _DEMO["look"]:  # the scripted demo's look at the dial, on the first step of a trial (see demo())
        _DEMO["look"] = False
        _aim_demo(dial_reading(ind))



# -- scripted demo: the left arm pinches the bezel across its rim and turns it ------------------------------------
TIP = 0.013
PINCH_Z = P + BEZEL_Z0 + 0.001 + TIP  # pad tips 1 mm above the bezel's lower edge
HOVER_Z = P + 0.090
PINCH = -(BEZEL_R - 0.003)  # pads 3 mm inside the rim's faces: 69 mm apart on a 75 mm bezel


def dial_reading(ind) -> int:
    """What a person reads on the dial: the pointer's position on the bezel's own scale, in whole divisions (0-99)
    clockwise from the zero line."""
    cw = (ind.joints["bezel"] + ind.joints["pointer"]) % (2 * math.pi)  # zero line's angle minus the pointer's
    return int(round(cw / DIV)) % 100


# The bezel turn (degrees, counter-clockwise seen from above) that brings the zero line under a pointer reading r:
# back by r divisions, or on by 100 - r, whichever is shorter.
TURN_FOR_READING = {r: (-r if r <= 50 else 100 - r) * 3.6 for r in range(100)}
_DEMO = {"look": False, "turns": []}  # the demo's turn waypoints, as (waypoint, fraction of the turn)


def _aim_demo(reading: int) -> None:
    turn = math.radians(TURN_FOR_READING[reading])
    for w, frac in _DEMO["turns"]:
        w.yaw = -turn / 2 + turn * frac


def _in_simulator() -> bool:
    return any(m == "isaaclab" or m.startswith("isaacsim") for m in sys.modules)


def demo():
    """Left arm: over the bezel with the fingers wide, down, pinch the rim across a diameter, turn the wrist until the
    zero line is under the pointer, open and lift away. The turn comes from reading the dial as a person would: the
    pointer's position on the dial in whole divisions (dial_reading), looked up in TURN_FOR_READING. In the simulator
    the dial is read on the trial's first step, before the arm reaches the bezel; offline, from the start state."""
    cx, cy = C
    steps = 6
    w = [
        Waypoint(2.0, (cx, cy, HOVER_Z), FINGER_OPEN, "over the bezel"),
        Waypoint(1.5, (cx, cy, PINCH_Z), FINGER_OPEN, "down beside the rim"),
        Waypoint(0.8, (cx, cy, PINCH_Z), PINCH, "pinch the bezel"),
    ]
    turns = [(w[0], 0.0), (w[1], 0.0), (w[2], 0.0)]
    for i in range(1, steps + 1):
        w.append(Waypoint(0.6, (cx, cy, PINCH_Z), PINCH, "turn the bezel"))
        turns.append((w[-1], i / steps))
    w += [
        Waypoint(0.6, (cx, cy, PINCH_Z), PINCH, "hold"),
        Waypoint(0.8, (cx, cy, PINCH_Z), FINGER_OPEN, "let go"),
        Waypoint(1.2, (cx, cy, HOVER_Z), FINGER_OPEN, "lift"),
    ]
    turns += [(w[-3], 1.0), (w[-2], 1.0), (w[-1], 1.0)]
    w += [
        Waypoint(1.2, (cx - 0.10, cy + 0.02, HOVER_Z), FINGER_OPEN, "back toward the robot", 0.0),
        Waypoint(3.0, (cx - 0.10, cy + 0.02, HOVER_Z), FINGER_OPEN, "wait", 0.0),
    ]
    _DEMO["turns"] = turns
    _aim_demo(0 if _in_simulator() else dial_reading(INDICATOR.state()))
    return {"left": w}
