"""On the station table, straight ahead of the robot: a dark blue steel tool chest with three drawers, one above the
other, each with a black handle fin on its front. One of the upper drawers stands a little open, with a screwdriver
(yellow handle, grey shaft) standing upright in a holder near the front of that drawer. A red socket is in the
bottom drawer. A grey rubber mat lies on the table to the robot's right.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so its reset hooks run before the lots move objects)
import lots  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BODY_BLUE, FRONT_GREY, BLACK = (0.10, 0.17, 0.35), (0.70, 0.72, 0.75), (0.06, 0.06, 0.07)
STEEL, YELLOW, RED, MAT_GREY = (0.55, 0.57, 0.60), (0.95, 0.78, 0.10), (0.85, 0.10, 0.08), (0.30, 0.31, 0.33)

# The chest. Part frame: origin on the table under the middle of the chest, x away from the robot, z up.
X0, Y0 = 0.41, 0.0
DEPTH, WIDTH, HEIGHT = 0.20, 0.30, 0.138
FRONT = -DEPTH / 2  # front face (part frame); world x 0.31
SLOT_BOTTOM = {"bottom": 0.004, "middle": 0.048, "top": 0.092}  # each drawer slot is 40 mm tall
SLOT_H = 0.040
FIN_Y = {"bottom": -0.12, "middle": -0.06, "top": 0.0}
DRAWERS = ("bottom", "middle", "top")

CARCASS = [
    parts.box((DEPTH, WIDTH, 0.004), at=(0.0, 0.0, 0.002), color=BODY_BLUE),  # plinth
    parts.box((DEPTH, 0.010, 0.128), at=(0.0, 0.145, 0.068), color=BODY_BLUE),  # left wall
    parts.box((DEPTH, 0.010, 0.128), at=(0.0, -0.145, 0.068), color=BODY_BLUE),  # right wall
    parts.box((0.008, 0.280, 0.128), at=(DEPTH / 2 - 0.004, 0.0, 0.068), color=BODY_BLUE),  # back
    parts.box((DEPTH, 0.280, 0.004), at=(0.0, 0.0, 0.046), color=BODY_BLUE),  # divider over the bottom slot
    parts.box((DEPTH, 0.280, 0.004), at=(0.0, 0.0, 0.090), color=BODY_BLUE),  # divider over the middle slot
    parts.box((DEPTH, WIDTH, 0.006), at=(0.0, 0.0, 0.135), color=BODY_BLUE),  # top panel, 132-138 mm
]

# A drawer: a tray 180 deep x 258 wide, its front flush with the chest's front when shut, a 60 mm handle fin, a tool
# holder (16 mm square hole, 35 mm deep) near its front (left in the middle drawer, right in the top drawer) and an
# interlock lug on its right side.
D_Y0, D_Y1 = -0.122, 0.136
D_YC, D_W = (D_Y0 + D_Y1) / 2, D_Y1 - D_Y0
FLOOR_T = 0.005  # floor top above the slot bottom
HOLDER_X = FRONT + 0.021  # holder centre, part frame x
HOLDER_Y = {"middle": 0.100, "top": -0.100}  # the two holders stand on opposite sides
HOLE, HOLDER_H = 0.016, 0.035  # 35 mm deep: the handle rests on the rim, the shaft's tip just off the floor
LUG_X, LUG_Y = FRONT + 0.020, -0.130  # 10 mm along x, 16 mm across (y -138 to -122 mm), 10 mm tall
TRAVEL = 0.140


def _drawer(name):
    b = SLOT_BOTTOM[name]
    zc = b + 0.020
    hx, hy = HOLDER_X, HOLDER_Y.get(name, 0.0)
    hz = b + FLOOR_T + HOLDER_H / 2
    return parts.Slider(
        name,
        shapes=[
            parts.box((0.008, D_W, 0.038), at=(FRONT + 0.004, D_YC, zc), color=FRONT_GREY),  # front
            parts.box((0.180, D_W, 0.004), at=(FRONT + 0.098, D_YC, b + 0.003), color=STEEL),  # floor
            parts.box((0.180, 0.004, 0.020), at=(FRONT + 0.098, D_Y0 + 0.002, b + 0.015), color=STEEL),  # sides
            parts.box((0.180, 0.004, 0.020), at=(FRONT + 0.098, D_Y1 - 0.002, b + 0.015), color=STEEL),
            parts.box((0.004, D_W, 0.020), at=(FRONT + 0.186, D_YC, b + 0.015), color=STEEL),  # back
            parts.box((0.060, 0.012, 0.024), at=(FRONT - 0.030, FIN_Y[name], zc), color=BLACK),  # handle fin
            parts.box((0.010, 0.016, 0.010), at=(LUG_X, LUG_Y, zc), color=STEEL),  # interlock lug
            *([] if name == "bottom" else [  # tool holder (the upper two drawers)
                parts.box((0.004, 0.024, HOLDER_H), at=(hx - 0.010, hy, hz), color=BLACK),
                parts.box((0.004, 0.024, HOLDER_H), at=(hx + 0.010, hy, hz), color=BLACK),
                parts.box((0.016, 0.004, HOLDER_H), at=(hx, hy - 0.010, hz), color=BLACK),
                parts.box((0.016, 0.004, HOLDER_H), at=(hx, hy + 0.010, hz), color=BLACK),
            ]),
        ],
        axis=(-1.0, 0.0, 0.0),  # out toward the robot
        travel=TRAVEL,
        mass=0.30,
        damping=1.0,
    )


# Interlock bolts: one per drawer, in the right wall, sliding along +y into the path of that drawer's lug. Shut, a
# drawer's lug stands 3 mm behind its bolt; with the bolt out the drawer opens 3 mm and stops.
BOLT_OUT = 0.014
BOLT_GAP = 0.003


def _bolt(name):
    zc = SLOT_BOTTOM[name] + 0.020
    return parts.Slider(
        f"bolt_{name}",
        shapes=[parts.box((0.008, 0.010, 0.010), at=(LUG_X - 0.005 - BOLT_GAP - 0.004, -0.145, zc), color=STEEL)],
        axis=(0.0, 1.0, 0.0),
        travel=BOLT_OUT,
        mass=0.02,
        spring=20000.0,  # stiff: a pulled drawer cannot shove its bolt aside
        damping=40.0,
    )


CHEST = parts.Part(
    "tool_chest",
    pos=(X0, Y0, TOP),
    shapes=CARCASS,
    joints=[*(_drawer(d) for d in DRAWERS), *(_bolt(d) for d in DRAWERS)],
)

# The screwdriver: a 12 mm grey shaft 35 mm long under a 22 mm yellow handle 30 mm long (65 mm in all, 50 g).
# Part frame: origin at the centre of the shaft's tip.
TOOL_LEN, HANDLE_R, HANDLE_Z0 = 0.065, 0.011, 0.035
TOOL = parts.Part(
    "screwdriver",
    pos=(0.0, 0.0, 0.0),  # set below to lot A's place
    mass=0.05,
    shapes=[
        parts.cylinder(0.006, HANDLE_Z0, at=(0.0, 0.0, HANDLE_Z0 / 2), color=STEEL),
        parts.cylinder(HANDLE_R, TOOL_LEN - HANDLE_Z0, at=(0.0, 0.0, (TOOL_LEN + HANDLE_Z0) / 2), color=YELLOW),
    ],
)
OPEN = 0.070  # the propped drawer stands 70 mm open


def tool_start(propped):
    """World position of the screwdriver's tip standing in the holder of the propped drawer."""
    return (X0 + HOLDER_X - OPEN, Y0 + HOLDER_Y[propped], TOP + SLOT_BOTTOM[propped] + FLOOR_T + 0.0005)


TOOL = parts.Part(TOOL.name, pos=tool_start("middle"), mass=TOOL.mass, shapes=TOOL.shapes)

SOCKET_SIZE = 0.022
SOCKET_XY = (X0 + FRONT + 0.050, 0.070)  # 50 mm in from the bottom drawer's front, shut
SOCKET = Box("red_socket", pos=(*SOCKET_XY, TOP + SLOT_BOTTOM["bottom"] + FLOOR_T + SOCKET_SIZE / 2 + 0.0005),
             size=(SOCKET_SIZE,) * 3, mass=0.04, color=RED)
MAT_SIZE = (0.10, 0.10, 0.004)
MAT = Box("grey_mat", pos=(0.20, -0.22, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=MAT_GREY)

OBJECTS = [CHEST, TOOL, SOCKET, MAT]

# Lot A (the screenshot): the middle drawer propped open by the screwdriver. Lot B: the top drawer. The socket stays
# in the bottom drawer; its zero move makes the lots kit settle the scene again after the reset hook below.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", propped="middle", place={"red_socket": SOCKET.pos}),
    lots.Lot("B", propped="top", place={"red_socket": SOCKET.pos}),
)

EPS = 1e-6
OPEN_AT = 0.005  # a drawer counts as open (and bolts the others) once it is out 5 mm or more
SHUT = 0.005  # shut: within 5 mm of closed
BOLT_ONLY_BELOW = 0.0025  # a bolt goes out only in front of a drawer that is shut to within 2.5 mm
TARGET_OPEN = 0.100  # bottom drawer out 100 mm or more: the socket can be lifted out
ON = 0.001  # resting on a surface: bottom within 1 mm of it
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the socket, the screwdriver or a drawer
STILL = 0.0005  # a drawer moving under 0.5 mm per control step (1 cm/s)
FLAT = math.radians(5.0)
OUT_OF_HOLDER = 0.020  # the screwdriver's tip more than 20 mm from where it stood in the holder


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# Filled every control step in the simulator by the hook below. Offline (near_misses.py) there is no robot, so the
# grippers count as clear and the drawers as still.
_sim = {"clear": True, "still": True, "prev": None, "limits": None}


def _bolt_targets(q) -> dict[str, float]:
    """Each drawer's bolt goes out when another drawer is open 5 mm or more and this one is shut within 2.5 mm, and
    stays out (the drawer held at 3 mm) until no other drawer is open 5 mm or more."""
    out = {}
    was = _sim["limits"]
    for i, d in enumerate(DRAWERS):
        other_open = any(q[o] >= OPEN_AT - EPS for o in DRAWERS if o != d)
        bolted = was is not None and was[i] < TRAVEL
        out[d] = BOLT_OUT if other_open and (q[d] < BOLT_ONLY_BELOW or bolted) else 0.0
    return out


@parts.on_reset
def _set_lot(sim) -> None:
    """Prop this lot's drawer open with the screwdriver standing in its holder; bolt the other two drawers."""
    import torch

    _sim.update(clear=True, still=True, prev=None)
    propped = lots.current().propped
    base = sim.env.unwrapped
    dev = base.device
    chest = base.scene["tool_chest"]
    q = {d: (OPEN if d == propped else 0.0) for d in DRAWERS}
    bolts = _bolt_targets(q)
    names = [*DRAWERS, *(f"bolt_{d}" for d in DRAWERS)]
    values = [q[d] for d in DRAWERS] + [bolts[d] for d in DRAWERS]
    _sim["limits"] = None
    _apply_interlock(sim, q)  # limits first, so the propped drawer's new position is inside its range
    ids = chest.find_joints(names, preserve_order=True)[0]
    chest.write_joint_position_to_sim_index(position=torch.tensor([values], device=dev), joint_ids=ids)
    chest.write_joint_velocity_to_sim_index(velocity=torch.zeros((1, len(ids)), device=dev), joint_ids=ids)
    tool = base.scene["screwdriver"]
    origin = base.scene.env_origins[0]
    pose = torch.tensor([[*tool_start(propped), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32, device=dev)
    pose[:, :3] += origin[:3].to(pose.dtype)
    tool.write_root_pose_to_sim_index(root_pose=pose)
    tool.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=dev))


def _apply_interlock(sim, q) -> None:
    """Move each bolt out or in, and hold a bolted drawer at its bolt: its slide's outer limit becomes 3 mm (the lug's
    clearance) while its bolt is out, and the full 140 mm otherwise. The bolt link shows the state; the limit is what
    stops the drawer (the kit's links do not collide with one another reliably enough to carry a hard pull)."""
    import torch

    bolts = _bolt_targets(q)
    for d, v in bolts.items():
        sim.set_target("tool_chest", f"bolt_{d}", v)
    limits = tuple(BOLT_GAP if bolts[d] > 0 else TRAVEL for d in DRAWERS)
    if limits == _sim["limits"]:
        return
    _sim["limits"] = limits
    base = sim.env.unwrapped
    chest = base.scene["tool_chest"]
    ids = chest.find_joints(list(DRAWERS), preserve_order=True)[0]
    lim = torch.tensor([[[0.0, v] for v in limits]], dtype=torch.float32, device=base.device)
    chest.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


def _hulls(base):
    from yam_arena import analysis

    origin = parts._np(base.scene.env_origins[0])
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
    return hulls


def _scored_boxes(st):
    chest = st["tool_chest"]
    poses = {d: (np.asarray(chest.links[d][0]), _rot(chest.links[d][1])) for d in DRAWERS}
    boxes = [(c, r, s) for _, c, r, s in CHEST.world_boxes(poses)]
    tool = st["screwdriver"]
    boxes += [(c, r, s) for _, c, r, s in TOOL.world_boxes({"base": (np.asarray(tool.pos), _rot(tool.quat_xyzw))})]
    sock = st["red_socket"]
    boxes.append((np.asarray(sock.pos), _rot(sock.quat_xyzw), np.full(3, SOCKET_SIZE)))
    return boxes


@parts.on_step
def _interlock(sim) -> None:
    """Move the bolts as the drawers stand; measure how close the grippers are to the scored parts."""
    from yam_arena import analysis

    st = sim.states()
    q = st["tool_chest"].joints
    _apply_interlock(sim, q)
    now = np.array([q[d] for d in DRAWERS])
    prev = _sim["prev"]
    _sim["still"] = bool(prev is None or np.max(np.abs(now - prev)) < STILL)
    _sim["prev"] = now
    hulls = _hulls(sim.env.unwrapped)
    _sim["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in _scored_boxes(st))


def _flat(state) -> bool:
    return bool(np.max(np.abs(_rot(state.quat_xyzw)[2, :])) >= math.cos(FLAT) - EPS)


def _socket_on_mat(s) -> bool:
    top = MAT.pos[2] + MAT_SIZE[2] / 2
    inside = all(MAT.pos[i] - MAT_SIZE[i] / 2 - EPS <= s.low[i] and s.high[i] <= MAT.pos[i] + MAT_SIZE[i] / 2 + EPS
                 for i in (0, 1))
    return bool(inside and abs(s.low[2] - top) <= ON + EPS and _flat(s))


def _socket_out_at_rest(s) -> bool:
    """Out of the chest and resting flat on something: its bottom within 1 mm of the table, the mat's top or the
    chest's top panel."""
    tops = (TOP, MAT.pos[2] + MAT_SIZE[2] / 2, TOP + HEIGHT)
    in_chest = s.high[0] > X0 + FRONT + EPS and abs(s.pos[1] - Y0) < WIDTH / 2 and s.low[2] < TOP + HEIGHT - 0.001
    return bool(not in_chest and any(abs(s.low[2] - z) <= ON + EPS for z in tops) and _flat(s))


def _socket_in_open_bottom(s, q_bottom) -> bool:
    """Resting on the bottom drawer's floor, inside the drawer's outline as it now stands."""
    floor = TOP + SLOT_BOTTOM["bottom"] + FLOOR_T
    x0 = X0 + FRONT - q_bottom + 0.008
    x1 = X0 + FRONT - q_bottom + 0.184
    inside = x0 - EPS <= s.low[0] and s.high[0] <= x1 + EPS and Y0 + D_Y0 - EPS <= s.low[1] and s.high[1] <= Y0 + D_Y1 + EPS
    return bool(inside and abs(s.low[2] - floor) <= ON + EPS and q_bottom >= TARGET_OPEN - EPS)


def _off_table(state) -> bool:
    """Fallen off the table: its lowest point more than 10 mm below the table top."""
    return bool(state.low[2] < TOP - 0.010)


def _grade(objects) -> float:
    s, t, chest = objects["red_socket"], objects["screwdriver"], objects["tool_chest"]
    if not (_sim["clear"] and _sim["still"] and s.resting() and t.resting()) or _off_table(t) or _off_table(s):
        return 0.0
    q = chest.joints
    if _socket_on_mat(s):
        return 1.0
    if _socket_out_at_rest(s) or _socket_in_open_bottom(s, q["bottom"]):
        return 0.75
    propped = lots.current().propped
    left_holder = np.linalg.norm(np.asarray(t.pos) - np.asarray(tool_start(propped))) > OUT_OF_HOLDER + EPS
    if q[propped] <= SHUT + EPS and left_holder:
        return 0.5
    return 0.0


def success(objects):
    """The socket rests on the mat, let go, with everything still and the screwdriver not on the floor."""
    return _grade(objects) == 1.0


def score(objects):
    """1.0 as success; 0.75 the bottom drawer open with the socket in it, or the socket resting flat out of the
    chest; 0.5 the propped drawer shut with the screwdriver out of its holder; else 0. All released and still."""
    return _grade(objects)


# --- demo -------------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
HIGH = 0.16  # TCP travel height above the table
PULL = 0.130


def _w(x, y, z):
    return (float(x), float(y), float(TOP + z))


def _fin(name, opened):
    """TCP over the middle of a drawer's fin, tips 5 mm below its centre line, with the drawer `opened` m out."""
    return (X0 + FRONT - 0.045 - opened, Y0 + FIN_Y[name], SLOT_BOTTOM[name] + 0.020 - 0.005 + TIP)


def demo():
    """The arm on the screwdriver's side lifts it out of its holder, carries it clear of the chest and stands it on the
    table (left arm: middle drawer, to the left of the chest; right arm: top drawer, to the right of the chest). The
    right arm then pushes the propped drawer shut by its fin, pulls the bottom drawer out and moves the socket to the
    mat. Which drawer is propped, and where its screwdriver stands, is what the cameras show for the trial's lot."""
    propped = lots.current().propped
    tx, ty, tz = tool_start(propped)
    tz -= TOP
    grip_z = tz + HANDLE_Z0 + 0.008 + TIP  # tips 8 mm up the handle
    lift = 0.050  # the tip clears the 35 mm holder
    stand_z = 0.007 + HANDLE_Z0 + 0.008 + TIP
    if ty > 0:  # left arm, out past the drawer's left side, onto the table left of the chest
        side, put, park = 0.20, (0.22, 0.24), (0.20, 0.25)
    else:  # right arm, out past the drawer's right side, onto the table right of the chest
        side, put, park = -0.22, (0.36, -0.22), (0.20, -0.25)
    take = [
        Waypoint(2.0, _w(tx, ty, grip_z + 0.04), FINGER_OPEN, "over the screwdriver"),
        Waypoint(1.5, _w(tx, ty, grip_z), -0.020, "down to the handle"),
        Waypoint(0.8, _w(tx, ty, grip_z), -0.008, "grip the handle"),
        Waypoint(1.5, _w(tx, ty, grip_z + lift), -0.008, "lift it out of the holder"),
        Waypoint(1.5, _w(tx, side, grip_z + lift), -0.008, "sideways, clear of the drawer"),
        Waypoint(2.0, _w(*put, stand_z), -0.008, "stand it on the table"),
        Waypoint(0.6, _w(*put, stand_z), -0.020, "let go"),
        Waypoint(1.2, _w(*put, HIGH), -0.020, "up"),
        Waypoint(1.5, _w(*park, HIGH), -0.020, "park"),
    ]
    if ty > 0:
        left, right_start = take, [Waypoint(9.0, _w(0.20, -0.25, HIGH), FINGER_OPEN, "wait")]
    else:
        left, right_start = [Waypoint(2.0, _w(0.20, 0.25, HIGH), FINGER_OPEN, "park")], take
    px, py, pz = _fin(propped, OPEN)
    sx, sy, sz = _fin(propped, -0.004)  # 4 mm past shut: the drawer stops at its closed limit
    bx, by, bz = _fin("bottom", 0.0)
    ox = bx - PULL - 0.005
    sock_x = SOCKET_XY[0] - PULL
    sock_z = SLOT_BOTTOM["bottom"] + FLOOR_T + SOCKET_SIZE / 2 + TIP
    place_z = MAT_SIZE[2] + 0.007 + SOCKET_SIZE / 2 + TIP
    mx, my = MAT.pos[0], MAT.pos[1]
    right = [
        *right_start,
        Waypoint(2.0, _w(px, py, pz + 0.04), -0.020, "over the propped drawer's fin"),
        Waypoint(1.2, _w(px, py, pz), -0.020, "down to the fin"),
        Waypoint(0.8, _w(px, py, pz), -0.003, "grip the fin"),
        Waypoint(2.0, _w(sx, sy, sz), -0.003, "push the drawer shut"),
        Waypoint(0.6, _w(sx, sy, sz), -0.020, "let go"),
        Waypoint(1.2, _w(sx - 0.02, sy, sz + 0.04), -0.020, "up"),
        Waypoint(2.0, _w(bx, by, bz + 0.04), -0.020, "over the bottom drawer's fin"),
        Waypoint(1.2, _w(bx, by, bz), -0.020, "down to the fin"),
        Waypoint(0.8, _w(bx, by, bz), -0.003, "grip the fin"),
        Waypoint(3.0, _w(ox, by, bz), -0.003, "pull the bottom drawer out"),
        Waypoint(0.6, _w(ox, by, bz), -0.020, "let go"),
        Waypoint(1.2, _w(ox, by, bz + 0.08), -0.020, "up"),
        Waypoint(2.0, _w(sock_x, SOCKET_XY[1], sock_z + 0.08), -0.020, "over the socket"),
        Waypoint(1.2, _w(sock_x, SOCKET_XY[1], sock_z), -0.020, "down to the socket"),
        Waypoint(0.8, _w(sock_x, SOCKET_XY[1], sock_z), -0.008, "grip the socket"),
        Waypoint(1.2, _w(sock_x, SOCKET_XY[1], sock_z + 0.08), -0.008, "lift it out"),
        Waypoint(2.0, _w(mx, my, place_z + 0.06), -0.008, "carry to the mat"),
        Waypoint(1.0, _w(mx, my, place_z), -0.008, "lower"),
        Waypoint(0.6, _w(mx, my, place_z), -0.020, "release"),
        Waypoint(1.2, _w(mx, my, HIGH), -0.020, "up"),
        Waypoint(1.5, _w(0.20, -0.25, HIGH), -0.020, "park"),
        Waypoint(2.0, _w(0.20, -0.25, HIGH), -0.020, "wait"),
    ]
    return {"left": left, "right": right}
