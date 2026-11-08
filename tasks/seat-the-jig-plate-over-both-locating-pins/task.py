"""On the station table, straight ahead of the robot: a dark-grey steel fixture base, a bar running side to side, with
two short steel locating pins standing on its top: a large round pin and a small diamond pin. To the robot's right, a
loose aluminium jig plate lies flat on a narrow grey riser. The plate has two square through-holes on its centre
line, a large one and a small one. Where the fixture stands, which way round it is, and how the plate lies on its
riser change between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402
import lots  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, PIN_BLUE, GREY = (0.30, 0.31, 0.33), (0.10, 0.35, 0.90), (0.55, 0.56, 0.58)  # pins painted blue to stand out
ALU = (0.80, 0.62, 0.20)  # anodised gold aluminium, so the plate stands out from the steel

# Fixture: base bar and pins, one rigid part. Part frame: origin at the base's underside centre, x away from the
# robot, y to its left (as drawn, lot A). It is heavy and loose only so that lots can move it; a real fixture is
# clamped, and moving it scores 0.
BASE_X, BASE_Y = 0.32, 0.0
BASE_SIZE = (0.04, 0.20, 0.02)  # x, y, z
BASE_H = BASE_SIZE[2]
PIN_H = 0.012
ROUND_R = 0.010  # round pin: 20 mm across
DIAMOND = (0.010, 0.008)  # diamond pin, relieved: 10 mm across the pin line (x), 8 mm along it (y)
PIN_Y = 0.045  # pins at PIN_MID +- PIN_Y: 90 mm apart
PIN_MID = -0.030  # the pins' midpoint, 30 mm right of the base's centre (part frame)
ROUND_AT = (0.0, PIN_MID + PIN_Y)  # left
DIAMOND_AT = (0.0, PIN_MID - PIN_Y)  # right
FIXTURE_MASS = 5.0
FIXTURE = parts.Part(
    "fixture", pos=(BASE_X, BASE_Y, TOP), mass=FIXTURE_MASS,
    shapes=[
        parts.box(BASE_SIZE, at=(0.0, 0.0, BASE_H / 2), color=STEEL),
        parts.cylinder(ROUND_R, PIN_H, at=(*ROUND_AT, BASE_H + PIN_H / 2), color=PIN_BLUE, axis="z"),
        parts.box((*DIAMOND, PIN_H), at=(*DIAMOND_AT, BASE_H + PIN_H / 2), color=PIN_BLUE),
    ],
)

# Jig plate, in its own frame: long axis along local y, holes on the centre line at local y HOLES: the 26 mm hole
# (for the round pin) at -35 mm and the 16 mm hole (for the diamond pin) at +55 mm, 90 mm apart, their midpoint
# 10 mm off the plate's middle.
PLATE_W, PLATE_L, PLATE_T = 0.07, 0.14, 0.008  # local x, local y, thickness
BIG, SMALL = 0.026, 0.016  # square holes
HOLES = (-0.035, 0.055)  # big, small
_hb, _hs, _hl = BIG / 2, SMALL / 2, PLATE_L / 2
_side_w = PLATE_W / 2 - _hb  # 22 mm strips either side of the holes' line
_cuts = [-_hl, HOLES[0] - _hb, HOLES[0] + _hb, HOLES[1] - _hs, HOLES[1] + _hs, _hl]
PLATE_SHAPES = [
    parts.box((_side_w, PLATE_L, PLATE_T), at=(-(_hb + _side_w / 2), 0.0, 0.0), color=ALU),
    parts.box((_side_w, PLATE_L, PLATE_T), at=(+(_hb + _side_w / 2), 0.0, 0.0), color=ALU),
    *[parts.box((BIG, y1 - y0, PLATE_T), at=(0.0, (y0 + y1) / 2, 0.0), color=ALU)
      for y0, y1 in zip(_cuts[0::2], _cuts[1::2])],  # the centre strip, around and between the holes
    *[parts.box((_hb - _hs, SMALL, PLATE_T), at=(sx * (_hs + _hb) / 2, HOLES[1], 0.0), color=ALU)
      for sx in (-1.0, 1.0)],  # narrowing the centre strip's gap to the small hole
]

RISER_SIZE = (0.04, 0.12, 0.02)  # long side to side, under the plate's long axis
RISER_X, RISER_Y = 0.42, -0.17  # beyond the right gripper, out of its shadow
RISER = Box("riser", pos=(RISER_X, RISER_Y, TOP + RISER_SIZE[2] / 2), size=RISER_SIZE, color=GREY)
PLATE_Z = TOP + RISER_SIZE[2] + PLATE_T / 2 + 0.0005
PLATE_YAW = math.radians(15.0)  # long axis 15 degrees off side to side, small hole at the left end
PLATE = parts.Part("jig_plate", pos=(RISER_X, RISER_Y, PLATE_Z), shapes=PLATE_SHAPES, mass=0.2, yaw=PLATE_YAW)

OBJECTS = [FIXTURE, RISER, PLATE]

# Lots: the fixture's way round and the plate's way round on its riser change independently, so the plate's start
# does not tell which way to turn it. The plate lies 15 degrees off the bar's direction, so seating it needs either a
# 15-degree clockwise or a 165-degree anticlockwise turn (seen from above): A and D need the long anticlockwise turn,
# B and C the short clockwise one. A (the screenshot): as drawn (round pin on the left; the plate's small hole at its
# left-hand end). B: the fixture turned end for end (round pin on the right) and 8 mm further from the robot; the
# plate as in A. C: the fixture as drawn, 7 mm nearer the robot and 12 mm to its left; the plate turned end for end
# (small hole at the right-hand end). D: the fixture turned end for end, 5 mm further out and 10 mm to the right; the
# plate turned end for end. Lot A gets a zero-distance move so it is settled and re-rendered like the others.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"fixture": (BASE_X, BASE_Y, TOP)}),
    lots.Lot("B", place={"fixture": (BASE_X + 0.008, BASE_Y, TOP)}, yaw={"fixture": math.pi}),
    lots.Lot("C", place={"fixture": (BASE_X - 0.007, BASE_Y + 0.012, TOP)}, yaw={"jig_plate": PLATE_YAW + math.pi}),
    lots.Lot("D", place={"fixture": (BASE_X + 0.005, BASE_Y - 0.010, TOP)},
             yaw={"fixture": math.pi, "jig_plate": PLATE_YAW + math.pi}),
)

FLAT_TOL = 0.0015  # every underside corner within 1.5 mm of the fixture's top
PLAY = BIG / 2 - ROUND_R  # 3 mm: how far the round pin's centre can be from its hole's centre, each way
EPS = 1e-6
SUCCESS_HOLD_S = 3.0  # the end state must hold for 3 s
CLEAR = 0.005  # released: no finger or gripper surface within 5 mm of the plate
FIX_MOVE, FIX_TURN = 0.005, math.radians(3.0)


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = np.asarray(quat_xyzw, float) / np.linalg.norm(quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _corners(state) -> np.ndarray:
    """World positions of the plate's eight corners (rows: underside or top, either way up)."""
    r = _rot(state.quat_xyzw)
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    return np.asarray(state.pos, float) + (signs * np.array([PLATE_W, PLATE_L, PLATE_T]) / 2) @ r.T


def _fix_frame(objects):
    f = objects["fixture"]
    return np.asarray(f.pos, float), _rot(f.quat_xyzw)


def _fix_point(objects, local) -> np.ndarray:
    p, r = _fix_frame(objects)
    return p + r @ np.asarray(local, float)


def _base_top(objects) -> float:
    return float(_fix_point(objects, (0.0, 0.0, BASE_H))[2])


def _flat_on_base(objects) -> bool:
    """The four lowest corners all within 1.5 mm of the fixture's top (so the plate is flat and down on it)."""
    z = np.sort(_corners(objects["jig_plate"])[:, 2])[:4]
    return bool(np.all(np.abs(z - _base_top(objects)) <= FLAT_TOL + EPS))


def _in_plate(state, xy) -> np.ndarray:
    """Where a vertical line at world xy meets the plate's mid-plane, in the plate's frame."""
    p0, r = np.asarray(state.pos, float), _rot(state.quat_xyzw)
    n = r[:, 2]
    z = p0[2] - (n[0] * (xy[0] - p0[0]) + n[1] * (xy[1] - p0[1])) / n[2]
    return r.T @ (np.array([xy[0], xy[1], z]) - p0)


def _pins_through(objects) -> bool:
    """The round pin's axis at least its radius inside the big hole's walls (at most 3 mm off its centre each way),
    and every vertical edge of the diamond pin inside the small hole."""
    plate = objects["jig_plate"]
    if abs(_rot(plate.quat_xyzw)[2, 2]) < 0.5:
        return False
    rp = _in_plate(plate, _fix_point(objects, (*ROUND_AT, BASE_H))[:2])
    if not (abs(rp[0]) <= PLAY + EPS and abs(rp[1] - HOLES[0]) <= PLAY + EPS):
        return False
    hx, hy = DIAMOND[0] / 2, DIAMOND[1] / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            c = _in_plate(plate, _fix_point(objects, (DIAMOND_AT[0] + sx * hx, DIAMOND_AT[1] + sy * hy, BASE_H))[:2])
            if not (abs(c[0]) <= _hs + EPS and abs(c[1] - HOLES[1]) <= _hs + EPS):
                return False
    return True


# Trial record: the fixture is clamped in reality, so moving it more than 5 mm or turning it more than 3 degrees
# from where its lot puts it, at any time, scores 0. Cleared at every reset, and offline by a scene reset (the plate
# back within 0.5 mm of its start and still, after a jump of over 20 mm in one step).
_H = {"fault": False, "prev": None}


def _yaw_of(r) -> float:
    return math.atan2(r[1, 0], r[0, 0])


def _update(objects) -> None:
    lot = lots.current()
    plate = objects["jig_plate"]
    pos = np.asarray(plate.pos, float)
    start = np.asarray(lot.pos("jig_plate"), float)
    prev = _H["prev"]
    if prev is not None and np.linalg.norm(pos - prev) > 0.020 and np.linalg.norm(pos - start) < 0.0005 \
            and plate.resting():
        _H["fault"] = False
    _H["prev"] = pos
    p, r = _fix_frame(objects)
    turn = abs((_yaw_of(r) - lot.yaw_of("fixture") + math.pi) % (2 * math.pi) - math.pi)
    tipped = math.acos(min(1.0, abs(float(r[2, 2]))))
    if np.linalg.norm(p - np.asarray(lot.pos("fixture"), float)) > FIX_MOVE + EPS or turn > FIX_TURN + EPS \
            or tipped > FIX_TURN + EPS:
        _H["fault"] = True


@parts.on_reset
def _new_episode(sim):
    _H.update(fault=False, prev=None)


# Released: measured in the simulator after every control step, from the robot's finger and gripper collision hulls
# against points on the plate's outer surface. Offline (near_misses.py, no simulator) it is taken as true.
_GRIP = {"clear": True}
_LINKS = ("gripper", "tip_left", "tip_right")


def _surface_local(step=0.005) -> np.ndarray:
    """Points every 5 mm or less over the plate's six outer faces, in its own frame."""
    h = np.array([PLATE_W, PLATE_L, PLATE_T]) / 2
    grids = [np.linspace(-v, v, max(2, int(np.ceil(2 * v / step)) + 1)) for v in h]
    pts = []
    for axis in range(3):
        a, b = [i for i in range(3) if i != axis]
        ga, gb = np.meshgrid(grids[a], grids[b], indexing="ij")
        for sign in (-1.0, 1.0):
            p = np.zeros((ga.size, 3))
            p[:, a], p[:, b], p[:, axis] = ga.ravel(), gb.ravel(), sign * h[axis]
            pts.append(p)
    return np.concatenate(pts)


_SURFACE = _surface_local()


def _robot_hulls(sim):
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        link_poses = {}
        for i, n in enumerate(asset.body_names):
            if n in _LINKS:
                t = np.eye(4)
                t[:3, :3], t[:3, 3] = _rot(poses[i, 3:]), poses[i, :3] - origin
                link_poses[n] = t
        out += analysis.world_hulls(link_poses)
    return out


def _clearance(hulls, pts) -> float:
    lo, hi = pts.min(0) - 0.02, pts.max(0) + 0.02
    best = np.inf
    for _, h in hulls:
        if np.any(h.vertices.max(0) < lo) or np.any(h.vertices.min(0) > hi):
            continue
        d = (pts @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1)  # a lower bound on each point's distance
        best = min(best, float(d.min()))
    return best


@parts.on_step
def _check_grip(sim):
    p = sim.states()["jig_plate"]
    pts = np.asarray(p.pos, float) + _SURFACE @ _rot(p.quat_xyzw).T
    _GRIP["clear"] = _clearance(_robot_hulls(sim), pts) >= CLEAR - EPS


def released(objects) -> bool:
    return bool(_GRIP["clear"])


def _seated(objects) -> bool:
    p = objects["jig_plate"]
    return bool(_flat_on_base(objects) and _pins_through(objects) and p.resting() and released(objects))


def success(objects):
    """Plate flat on the fixture, the round pin through the big hole and the diamond pin through the small one, no
    gripper within 5 mm of it, still; the fixture never moved."""
    _update(objects)
    return bool(not _H["fault"] and _seated(objects))


def score(objects):
    """Pass/fail: 1.0 as success, else 0."""
    return 1.0 if success(objects) else 0.0


# -- scripted demo ----------------------------------------------------------------------------------------------

TIP = 0.013  # finger tips below the TCP
HOLD = -(PLATE_W / 2 - 0.003)  # close 3 mm past the plate's long edges (a full close grips a wide part off centre)
# Carry lead (m): with a 4 mm outward lead the held plate set down 1.3-3.2 mm too far out and about 2.7 mm right of
# its target in a GPU trace.
COMP = (0.0025, 0.0025)
COMP_YAW = 0.0  # rad, per unit of turn sign


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def demo():
    """Right arm: grip the plate across its width at its middle, lift it, turn it while carrying it over the pins so
    that its big hole is over the big pin, lower it until the pins are in the holes and let go. The fixture's place
    and turn, and the plate's turn on its riser, are the ones the cameras show for the trial's lot."""
    lot = lots.current()
    fx, fy, _ = lot.pos("fixture")
    fyaw = lot.yaw_of("fixture")
    pyaw0 = lot.yaw_of("jig_plate")
    up = TOP + 0.10
    base_top = TOP + BASE_H
    grip_z = TOP + RISER_SIZE[2] + 0.001 + TIP  # fingertips 1 mm above the plate's underside (clear of the riser)
    over_z = base_top + PIN_H + 0.007 + 0.001 + TIP  # commanded 7 mm above the pin tops (the arm sags 4 mm)
    drop_z = base_top + 0.006 + 0.001 + TIP  # underside 6 mm above the fixture: pins half way into the holes
    c, s_ = math.cos(fyaw), math.sin(fyaw)
    mid = np.array([fx, fy]) + np.array([[c, -s_], [s_, c]]) @ np.array([0.0, PIN_MID])  # pins' midpoint
    pyaw = fyaw + math.pi  # the plate's small-hole end toward the diamond pin
    off = np.array([-math.sin(pyaw), math.cos(pyaw)]) * (HOLES[0] + HOLES[1]) / 2
    turn = _wrap(pyaw - pyaw0)
    # Grip yaw: fingers across the plate's width. Either of the two such grips holds it; take the one whose end yaw
    # (after the turn) is nearer a quarter turn left, which the right wrist can reach over the pins (its last joint
    # turns only +-120 degrees, so it can't do a clockwise quarter turn from a grip across the width).
    g0 = min((_wrap(pyaw0 - math.pi / 2 + k * math.pi) for k in (0, 1)), key=lambda g: abs(_wrap(g + turn - math.pi / 2)))
    g1 = g0 + turn + COMP_YAW * math.copysign(1.0, turn)
    cx, cy = (float(v) for v in mid - off + np.array(COMP))
    right = [
        Waypoint(1.5, (RISER_X, RISER_Y, up), FINGER_OPEN, "over the plate", g0),
        Waypoint(1.2, (RISER_X, RISER_Y, grip_z), FINGER_OPEN, "down beside the plate's long edges", g0),
        Waypoint(0.8, (RISER_X, RISER_Y, grip_z), HOLD, "grip across the width", g0),
        Waypoint(1.0, (RISER_X, RISER_Y, up), HOLD, "lift", g0),
        Waypoint(2.0, (cx, cy, up), HOLD, "carry over the pins, turning it big hole to big pin", g1),
        Waypoint(1.5, (cx, cy, over_z), HOLD, "down to just above the pins", g1),
        Waypoint(1.0, (cx, cy, over_z), HOLD, "settle", g1),
        Waypoint(1.0, (cx, cy, drop_z), HOLD, "onto the pins", g1),
        Waypoint(0.6, (cx, cy, drop_z), FINGER_OPEN, "let go", g1),
        Waypoint(1.0, (cx, cy - 0.05, up), FINGER_OPEN, "up and away", g1),
    ]
    return {"right": right}
