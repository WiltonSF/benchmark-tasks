"""On the station table, ahead of the robot: a low wooden counter (a top on two end walls, open at the front and
back) and a white pedal bin with a grey lid hinged along its back edge, a black pull fin at the top of its front and
a black foot pedal at its front foot. A red drinks can stands upright on the table to one side. Where the bin and
the can stand changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so its hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WOOD, WHITE, GREY, BLACK, RED = (0.62, 0.45, 0.28), (0.92, 0.92, 0.90), (0.50, 0.52, 0.55), (0.07, 0.07, 0.08), \
    (0.80, 0.08, 0.08)

# Counter (fixed). World frame: its front edge 30 cm in front of the arm bases, 200 mm deep, 300 mm wide on the
# midline; the top's underside 106 mm up.
CX0, C_DEPTH, C_WIDTH, C_T = 0.30, 0.200, 0.300, 0.012
UNDER = 0.106
COUNTER = parts.Part(
    "counter",
    pos=(CX0 + C_DEPTH / 2, 0.0, TOP),
    shapes=[
        parts.box((C_DEPTH, C_WIDTH, C_T), at=(0.0, 0.0, UNDER + C_T / 2), color=WOOD),
        parts.box((C_DEPTH, C_T, UNDER), at=(0.0, C_WIDTH / 2 - C_T / 2, UNDER / 2), color=WOOD),
        parts.box((C_DEPTH, C_T, UNDER), at=(0.0, -C_WIDTH / 2 + C_T / 2, UNDER / 2), color=WOOD),
    ],
)

# Pedal bin (loose). Part frame: origin at the centre of its underside, x away from the robot, z up.
B, WALL, RIM = 0.100, 0.005, 0.080  # 100 x 100 mm, walls 5 mm, rim 80 mm up
INNER = B / 2 - WALL  # 45 mm: half the inside
LID_T = 0.006
HINGE_X = B / 2  # the lid's hinge runs along the back top edge
FIN = dict(x0=-0.075, x1=-B / 2, y0=-0.006, y1=0.006, z0=0.060, z1=0.075)  # on the front's centre line
PEDAL = dict(x0=-0.130, x1=-B / 2, y0=0.0, y1=0.050, z0=0.020, z1=0.026)
BIN_MASS = 0.30
LID_OPEN = 1.5  # rad: where the pedal linkage drives the lid
PEDAL_ON, PEDAL_OFF = 0.12, 0.06  # rad: pressed past 0.12 the lid opens; let up below 0.06 it shuts
LID = parts.Hinge(
    "lid",
    shapes=[parts.box((B, B, LID_T), at=(0.0, 0.0, RIM + LID_T / 2), color=GREY)],
    pivot=(HINGE_X, 0.0, RIM),
    axis=(0.0, 1.0, 0.0),  # positive lifts the front edge
    lower=0.0,
    upper=1.6,
    mass=0.04,
    spring=0.3,
    damping=0.05,
)
PEDAL_HINGE = parts.Hinge(
    "pedal",
    shapes=[parts.box((PEDAL["x1"] - PEDAL["x0"], PEDAL["y1"] - PEDAL["y0"], PEDAL["z1"] - PEDAL["z0"]),
                      at=((PEDAL["x0"] + PEDAL["x1"]) / 2, (PEDAL["y0"] + PEDAL["y1"]) / 2,
                          (PEDAL["z0"] + PEDAL["z1"]) / 2), color=BLACK)],
    pivot=(-B / 2, (PEDAL["y0"] + PEDAL["y1"]) / 2, (PEDAL["z0"] + PEDAL["z1"]) / 2),
    axis=(0.0, -1.0, 0.0),  # positive pushes the pedal's front end down
    lower=0.0,
    upper=0.3,
    mass=0.02,
    spring=0.25,
    damping=0.02,
)


def _bin_shapes():
    h = RIM - WALL
    return [
        parts.box((B, B, WALL), at=(0.0, 0.0, WALL / 2), color=WHITE),
        parts.box((WALL, B, h), at=(-B / 2 + WALL / 2, 0.0, WALL + h / 2), color=WHITE),
        parts.box((WALL, B, h), at=(B / 2 - WALL / 2, 0.0, WALL + h / 2), color=WHITE),
        parts.box((B - 2 * WALL, WALL, h), at=(0.0, B / 2 - WALL / 2, WALL + h / 2), color=WHITE),
        parts.box((B - 2 * WALL, WALL, h), at=(0.0, -B / 2 + WALL / 2, WALL + h / 2), color=WHITE),
        parts.box((FIN["x1"] - FIN["x0"], FIN["y1"] - FIN["y0"], FIN["z1"] - FIN["z0"]),
                  at=((FIN["x0"] + FIN["x1"]) / 2, (FIN["y0"] + FIN["y1"]) / 2, (FIN["z0"] + FIN["z1"]) / 2),
                  color=BLACK),
    ]


UNDER_X = CX0 + 0.020  # lots A and B: the bin's front face 30 mm in front of the counter, 70 mm under it
BIN_Y = 0.05
PEDAL_BIN = parts.Part("pedal_bin", pos=(UNDER_X, BIN_Y, TOP), mass=BIN_MASS, shapes=_bin_shapes(),
                       joints=[LID, PEDAL_HINGE], friction=(0.5, 0.4))
CAN_R, CAN_H = 0.025, 0.070
CAN_XY = (0.20, -0.15)
CAN = Cylinder("can", pos=(*CAN_XY, TOP + CAN_H / 2), radius=CAN_R, height=CAN_H, mass=0.015, color=RED)
OBJECTS = [COUNTER, PEDAL_BIN, CAN]

# The bin always starts under the counter, in a different place in each lot, and the can moves to the other side.
# A (the screenshot): bin left of the midline, front face 30 mm out; can front right. B: bin right of the midline,
# can front left. C: bin left, pushed 15 mm further in (front face 15 mm out); can far right, beside the counter.
DEEP_X = UNDER_X + 0.015
CAN_B, CAN_C = (0.20, 0.15), (0.28, -0.21)
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"can": CAN.pos, "pedal_bin": PEDAL_BIN.pos}),  # zero move: forces the settle and first frame
    lots.Lot("B", place={"can": (*CAN_B, CAN.pos[2]), "pedal_bin": (UNDER_X, -BIN_Y, TOP)}),
    lots.Lot("C", place={"can": (*CAN_C, CAN.pos[2]), "pedal_bin": (DEEP_X, BIN_Y, TOP)}),
)

EPS = 1e-6
SHUT = math.radians(5.0)
UPRIGHT = math.radians(10.0)
TIPPED = math.radians(30.0)
CLEAR = 0.005  # released: no gripper within 5 mm of the bin (any part) or the can


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(state) -> float:
    return math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0)))


def _local(objects, p) -> np.ndarray:
    b = objects["pedal_bin"]
    return _rot(b.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(b.pos, float))


def hinge_x(objects) -> float:
    """World x of the lid's hinge line (the bin's back top edge)."""
    b = objects["pedal_bin"]
    return float((np.asarray(b.pos) + _rot(b.quat_xyzw) @ np.array([HINGE_X, 0.0, RIM]))[0])


def can_in_bin(objects) -> bool:
    c = _local(objects, objects["can"].pos)
    return bool(abs(c[0]) <= INNER + EPS and abs(c[1]) <= INNER + EPS and WALL - EPS <= c[2] <= RIM + EPS)


_record = {"fault": False, "prev": None}
_grip = {"clear": True}
_link = {"open": False}


def _update(objects) -> None:
    can = objects["can"]
    pos = np.asarray(can.pos, float)
    prev = _record["prev"]
    start = np.asarray(lots.current().pos("can"), float)
    if prev is not None and np.linalg.norm(pos - start) < 0.0005 and can.resting():
        if np.linalg.norm(pos - prev) > 0.020:
            _record["fault"] = False
    _record["prev"] = pos
    if _tilt(objects["pedal_bin"]) > TIPPED + EPS:
        _record["fault"] = True


@parts.on_step
def _pedal_link(sim) -> None:
    """The pedal's linkage: pressed past 0.12 rad, it drives the lid open; let up below 0.06 rad, the lid falls
    shut."""
    q = sim.states()["pedal_bin"].joints.get("pedal", 0.0)
    if q > PEDAL_ON:
        _link["open"] = True
    elif q < PEDAL_OFF:
        _link["open"] = False
    sim.set_target("pedal_bin", "lid", LID_OPEN if _link["open"] else 0.0)


@parts.on_reset
def _reset_link(sim) -> None:
    _link["open"] = False


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    b, c = st["pedal_bin"], st["can"]
    poses = {"base": (np.asarray(b.pos), _rot(b.quat_xyzw))}
    poses.update({k: (np.asarray(v[0]), _rot(v[1])) for k, v in b.links.items()})
    boxes = [(cc, r, s) for _, cc, r, s in PEDAL_BIN.world_boxes(poses)]
    boxes.append((np.asarray(c.pos), _rot(c.quat_xyzw), np.array([2 * CAN_R, 2 * CAN_R, CAN_H])))
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        lp = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right", "gripper"):
                m = np.eye(4)
                m[:3, :3] = _rot(lp[i, 3:])
                m[:3, 3] = lp[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    _grip["clear"] = all(analysis.hulls_to_box(hulls, cc, r, s)[0] > CLEAR for cc, r, s in boxes)


def _settled(objects) -> bool:
    b, c = objects["pedal_bin"], objects["can"]
    return bool(b.resting() and c.resting() and _grip["clear"] and _tilt(b) <= UPRIGHT + EPS)


def _lid_shut(objects) -> bool:
    return bool(objects["pedal_bin"].joints["lid"] <= SHUT + EPS)


def _full(objects) -> bool:
    return bool(_settled(objects) and _lid_shut(objects) and can_in_bin(objects))


def success(objects):
    """The can is in the bin below the rim, the lid shut, the bin upright; let go and still; never tipped."""
    _update(objects)
    return bool(not _record["fault"] and _full(objects))


def distance(objects):
    return float(np.linalg.norm(_local(objects, objects["can"].pos)[:2]))


def score(objects):
    """Pass/fail: 1.0 on success, otherwise 0."""
    _update(objects)
    return 1.0 if not _record["fault"] and _full(objects) else 0.0


# --- demo -------------------------------------------------------------------------------------------------------
TIP = 0.013
TRAVEL = 0.15
END_X = UNDER_X - 0.105  # where the demo pulls the bin to (its hinge 45 mm clear of the counter)
LAG = 0.010  # the pull falls about 10 mm short (traced)
FIN_OPEN, FIN_SHUT = -0.012, -0.003
CAN_OPEN, CAN_SHUT = -0.035, -0.022
CAN_CARRY = RIM + 0.018  # the can's underside 18 mm over the rim, 12 mm over the shut lid


def _w(x, y, z):
    return (x, y, TOP + z)


def demo():
    """The arm on the bin's side pulls it out by its fin, then presses the pedal
    and holds it; the other arm brings the can over the open bin, lets it drop in and backs off; then the pedal is let
    up and the lid falls shut. Where the bin and can stand is what the cameras show for the trial's lot."""
    lot = lots.current()
    bx, by, _ = lot.pos("pedal_bin")
    cx, cy, _ = lot.pos("can")
    near, far = ("left", "right") if by > 0 else ("right", "left")
    fin_x, fin_y = bx + (FIN["x0"] + FIN["x1"]) / 2, by + (FIN["y0"] + FIN["y1"]) / 2
    fin_z = FIN["z0"] + 0.002 + TIP
    pull = bx - END_X + LAG
    ox = END_X
    ped_x, ped_y = ox - 0.110, by + (PEDAL["y0"] + PEDAL["y1"]) / 2  # 50 mm clear of the lid's front edge
    press = PEDAL["z1"] - 0.022 + TIP  # tips commanded 22 mm into the pedal's top (its stop is 15 mm down)
    near_plan = [
        Waypoint(1.5, _w(fin_x, fin_y, TRAVEL), FIN_OPEN, "over the fin"),
        Waypoint(1.5, _w(fin_x, fin_y, fin_z), FIN_OPEN, "down to the fin"),
        Waypoint(0.8, _w(fin_x, fin_y, fin_z), FIN_SHUT, "grip the fin"),
        Waypoint(3.5, _w(fin_x - pull, fin_y, fin_z), FIN_SHUT, "pull the bin out"),
        Waypoint(0.5, _w(fin_x - pull, fin_y, fin_z), FIN_SHUT, "settle"),
        Waypoint(0.6, _w(fin_x - pull, fin_y, fin_z), FIN_OPEN, "let go"),
        Waypoint(1.0, _w(fin_x - pull, fin_y, TRAVEL), FIN_OPEN, "up"),
    ]
    near_plan += [
        Waypoint(1.2, _w(ped_x, ped_y, TRAVEL), FINGER_CLOSED, "over the pedal, fingers shut"),
        Waypoint(1.2, _w(ped_x, ped_y, press + 0.03), FINGER_CLOSED, "down to the pedal"),
        Waypoint(1.2, _w(ped_x, ped_y, press), FINGER_CLOSED, "press the pedal"),
        Waypoint(8.0, _w(ped_x, ped_y, press), FINGER_CLOSED, "hold it while the can goes in"),
        Waypoint(1.0, _w(ped_x, ped_y, press + 0.05), FINGER_CLOSED, "let the pedal up"),
        Waypoint(1.0, _w(ped_x - 0.05, ped_y, TRAVEL), FINGER_CLOSED, "back off"),
        Waypoint(3.0, _w(ped_x - 0.05, ped_y, TRAVEL), FINGER_CLOSED, "wait"),
    ]
    i_press = next(i for i, w in enumerate(near_plan) if w.label == "press the pedal")
    t_open = sum(w.duration for w in near_plan[:i_press + 1]) + 1.0  # the lid is up a second into the hold
    can_z = CAN_H - 0.022 + TIP  # TCP above the can's underside: tips 22 mm down its side
    side = -0.10 if far == "right" else 0.10  # waits clear of the lid's swing and the bin's side
    pick = [
        Waypoint(1.5, _w(cx, cy, can_z), CAN_OPEN, "down round the can"),
        Waypoint(0.8, _w(cx, cy, can_z), CAN_SHUT, "grip"),
        Waypoint(1.5, _w(cx, cy, CAN_CARRY + can_z), CAN_SHUT, "lift"),
        Waypoint(2.0, _w(ox, by + side, CAN_CARRY + can_z), CAN_SHUT, "beside the bin"),
    ]
    # Wait over the can, out of the other arm's way, until the bin is out; arrive beside it as the lid opens.
    wait = max(t_open - 1.5 - sum(w.duration for w in pick), 0.5)
    far_plan = [
        Waypoint(1.5, _w(cx, cy, TRAVEL), CAN_OPEN, "over the can"),
        Waypoint(wait, _w(cx, cy, TRAVEL), CAN_OPEN, "wait while the bin is pulled out"),
        *pick,
        Waypoint(0.5, _w(ox, by + side, CAN_CARRY + can_z), CAN_SHUT, "wait for the lid"),
        Waypoint(1.5, _w(ox, by, CAN_CARRY + can_z), CAN_SHUT, "over the open bin"),
        Waypoint(0.6, _w(ox, by, CAN_CARRY + can_z), CAN_OPEN, "let it drop in"),
        Waypoint(1.0, _w(ox, by, TRAVEL + 0.01), CAN_OPEN, "up"),
        Waypoint(1.5, _w(cx, cy, TRAVEL), CAN_OPEN, "back off"),
    ]
    return {near: near_plan, far: far_plan}
