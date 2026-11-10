"""On the station table, straight ahead of the robot: a grey combination key safe, its lid hinged along the back edge
and standing open. Three black number wheels are set into the strip in front of the lid, each showing part of its
rim above the top with a white index tooth on it. A key (red fob, brass blade) lies on the table beside the safe.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so its reset hook runs before the lots move the key)
import lots  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

BODY, LID_GREY, BLACK, WHITE = (0.42, 0.44, 0.47), (0.55, 0.57, 0.60), (0.06, 0.06, 0.07), (0.95, 0.95, 0.95)
RED, BRASS = (0.80, 0.10, 0.08), (0.80, 0.65, 0.25)

# The safe. Part frame: origin on the table under the middle of the safe, x away from the robot, z up.
X0, Y0 = 0.28, 0.0
DEPTH, WIDTH, HEIGHT = 0.126, 0.092, 0.050
FRONT, BACK = -DEPTH / 2, DEPTH / 2
STRIP = 0.048  # the wheel strip at the front, solid
WALL, FLOOR = 0.006, 0.005
IN_X0, IN_X1 = FRONT + STRIP, BACK - WALL  # compartment inside, part frame: x -15 to +57 mm
IN_Y = WIDTH / 2 - WALL  # +-40 mm

BODY_SHAPES = [
    parts.box((STRIP, WIDTH, HEIGHT), at=(FRONT + STRIP / 2, 0.0, HEIGHT / 2), color=BODY),  # wheel strip
    parts.box((DEPTH - STRIP, WIDTH, FLOOR), at=((IN_X0 + BACK) / 2, 0.0, FLOOR / 2), color=BODY),  # floor
    parts.box((DEPTH - STRIP, WALL, HEIGHT), at=((IN_X0 + BACK) / 2, IN_Y + WALL / 2, HEIGHT / 2), color=BODY),
    parts.box((DEPTH - STRIP, WALL, HEIGHT), at=((IN_X0 + BACK) / 2, -IN_Y - WALL / 2, HEIGHT / 2), color=BODY),
    parts.box((WALL, 2 * IN_Y, HEIGHT), at=(BACK - WALL / 2, 0.0, HEIGHT / 2), color=BODY),  # back wall
]

# The lid: 78 x 92 x 8 mm on the top of the compartment, hinged along the back top edge. Opening lifts its front.
LID_T = 0.008
LID_MAX = 1.92  # rad, 110 degrees: leaning back past upright
LID = parts.Hinge(
    "lid",
    pivot=(BACK, 0.0, HEIGHT),
    axis=(0.0, 1.0, 0.0),
    lower=0.0,
    upper=LID_MAX,
    mass=0.10,
    spring=3.0,  # drive for the stiff-hinge hold below
    damping=0.05,
    shapes=[parts.box((DEPTH - STRIP, WIDTH, LID_T), at=((IN_X0 + BACK) / 2, 0.0, HEIGHT + LID_T / 2), color=LID_GREY)],
)

# Wheels: 40 mm across, 10 mm thick, axes along y 12 mm below the top (8 mm of rim stands proud), 25 mm apart. Each
# has a white index tooth 10 x 10 mm standing 12 mm proud of its rim. The code is every tooth at the top.
WHEEL_R, WHEEL_T, AXIS_Z, WHEEL_X = 0.020, 0.010, HEIGHT - 0.012, FRONT + 0.022
WHEEL_Y = {"left": 0.025, "middle": 0.0, "right": -0.025}
TOOTH_R0, TOOTH_R1 = 0.018, 0.032
CLICK = math.radians(36)  # ten numbers a turn
WHEELS = tuple(WHEEL_Y)


def _wheel(name):
    y = WHEEL_Y[name]
    return parts.Hinge(
        name,
        pivot=(WHEEL_X, y, AXIS_Z),
        axis=(0.0, 1.0, 0.0),
        lower=-3.1,
        upper=3.1,
        mass=0.02,
        spring=0.15,  # drive for the click detent below
        damping=0.005,
        shapes=[parts.cylinder(WHEEL_R, WHEEL_T, at=(WHEEL_X, y, AXIS_Z), color=BLACK, axis="y"),
                parts.box((0.010, WHEEL_T, TOOTH_R1 - TOOTH_R0), at=(WHEEL_X, y, AXIS_Z + (TOOTH_R0 + TOOTH_R1) / 2),
                          color=WHITE)],
    )


SAFE = parts.Part("key_safe", pos=(X0, Y0, TOP), shapes=BODY_SHAPES, joints=[LID, *(_wheel(w) for w in WHEELS)])

# The key: a red fob 22 x 20 x 22 mm with a brass blade 24 x 12 x 3 mm along its bottom edge, 30 g. Part frame: origin
# at the fob's centre, the blade toward +x.
FOB = (0.022, 0.020, 0.022)
BLADE = (0.024, 0.012, 0.003)
KEY_A = (0.235, 0.170, TOP + FOB[2] / 2)
KEY = parts.Part(
    "key",
    pos=KEY_A,
    mass=0.03,
    shapes=[parts.box(FOB, at=(0.0, 0.0, 0.0), color=RED),
            parts.box(BLADE, at=(FOB[0] / 2 + BLADE[0] / 2, 0.0, -FOB[2] / 2 + BLADE[2] / 2), color=BRASS)],
)

OBJECTS = [SAFE, KEY]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", lid=1.90, place={"key": KEY_A}),  # a zero move, so lot A also settles and takes a fresh frame
    lots.Lot("B", lid=1.57, place={"key": (0.335, 0.150, TOP + FOB[2] / 2)}),
    lots.Lot("C", lid=1.75, place={"key": (0.160, 0.090, TOP + FOB[2] / 2)}),
)

# Mechanism. A wheel is off the code once its tooth is 18 degrees (half a number) or more from the top; the click
# detent then carries it to the next number. With every wheel on the code the lid opens and shuts freely. With a
# wheel off the code: if the lid is shut (within 2 degrees) it is locked there; if it is open, the bolt stands out
# and the lid stops 9 degrees (0.15 rad) short of shut.
OFF = CLICK / 2
SHUT = math.radians(2.0)
BOLT = 0.15
BOLT_MARGIN = 0.02  # rad: a bolt that meets a nearly shut lid still holds it this far above the shut range
LID_HOLD = 0.02  # rad: the lid's stiff hinge holds it where it is left, within this
STILL = 0.005  # rad per control step


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def off_code(joints) -> bool:
    return any(abs(_wrap(float(joints[w]))) >= OFF - EPS for w in WHEELS)


def _rot(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# Filled every control step in the simulator by the hooks below. Offline (near_misses.py) there is no robot, so the
# grippers count as clear and the joints as still.
_sim = {"clear": True, "still": True, "prev": None, "limits": None, "lock_at": None, "bolt_at": None, "lid_t": 0.0, "wheel_t": {}}


def _write_limits(sim, lo: float, hi: float) -> None:
    import torch

    if _sim["limits"] == (lo, hi):
        return
    _sim["limits"] = (lo, hi)
    base = sim.env.unwrapped
    safe = base.scene["key_safe"]
    ids = safe.find_joints(["lid"], preserve_order=True)[0]
    lim = torch.tensor([[[lo, hi]]], dtype=torch.float32, device=base.device)
    safe.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


def _interlock(sim, q) -> None:
    lid = float(q["lid"])
    if not off_code(q):
        _sim["lock_at"] = _sim["bolt_at"] = None
        _write_limits(sim, 0.0, LID_MAX)
    elif _sim["lock_at"] is not None or (_sim["bolt_at"] is None and lid <= SHUT + EPS):
        if _sim["lock_at"] is None:
            _sim["lock_at"] = max(lid, 0.001)
        _write_limits(sim, 0.0, _sim["lock_at"])
    else:
        # The bolt stands out where it first met the lid, and stays there until every wheel is back on the code: a
        # lid open when a wheel was turned can never be pressed down into the locked range.
        if _sim["bolt_at"] is None:
            _sim["bolt_at"] = max(min(lid, BOLT), SHUT + BOLT_MARGIN)
        _write_limits(sim, _sim["bolt_at"], LID_MAX)


@parts.on_reset
def _set_lot(sim) -> None:
    """Open the lid to this lot's angle; every wheel on the code; the lid free."""
    import torch

    _sim.update(clear=True, still=True, prev=None, limits=None, lock_at=None, bolt_at=None)
    base = sim.env.unwrapped
    safe = base.scene["key_safe"]
    _write_limits(sim, 0.0, LID_MAX)
    names = ["lid", *WHEELS]
    values = [float(lots.current().lid)] + [0.0] * len(WHEELS)
    ids = safe.find_joints(names, preserve_order=True)[0]
    safe.write_joint_position_to_sim_index(position=torch.tensor([values], device=base.device), joint_ids=ids)
    safe.write_joint_velocity_to_sim_index(velocity=torch.zeros((1, len(ids)), device=base.device), joint_ids=ids)
    _sim["lid_t"] = values[0]
    _sim["wheel_t"] = {w: 0.0 for w in WHEELS}
    sim.set_target("key_safe", "lid", values[0])
    for w in WHEELS:
        sim.set_target("key_safe", w, 0.0)


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
    safe, key = st["key_safe"], st["key"]
    poses = {j: (np.asarray(safe.links[j][0]), _rot(safe.links[j][1])) for j in ("lid", *WHEELS)}
    boxes = [(c, r, s) for link, c, r, s in SAFE.world_boxes(poses) if link != "base"]
    boxes += [(c, r, s) for _, c, r, s in KEY.world_boxes({"base": (np.asarray(key.pos), _rot(key.quat_xyzw))})]
    return boxes


@parts.on_step
def _step(sim) -> None:
    """Click detents on the wheels, the stiff lid hinge, the bolt, and how close the grippers are to the scored parts."""
    from yam_arena import analysis

    st = sim.states()
    q = st["key_safe"].joints
    for w in WHEELS:  # each wheel springs to its nearest number
        t = round(float(q[w]) / CLICK) * CLICK
        if t != _sim["wheel_t"].get(w):
            _sim["wheel_t"][w] = t
            sim.set_target("key_safe", w, t)
    lid = float(q["lid"])
    t = _sim["lid_t"]
    if lid <= SHUT + EPS:
        t = -LID_HOLD  # dropped onto its seat
    elif abs(lid - t) > LID_HOLD:
        t = lid - math.copysign(LID_HOLD, lid - t)
    if t != _sim["lid_t"]:
        _sim["lid_t"] = t
        sim.set_target("key_safe", "lid", t)
    _interlock(sim, q)
    now = np.array([float(q[j]) for j in ("lid", *WHEELS)])
    prev = _sim["prev"]
    _sim["still"] = bool(prev is None or np.max(np.abs(now - prev)) < STILL)
    _sim["prev"] = now
    hulls = _hulls(sim.env.unwrapped)
    _sim["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in _scored_boxes(st))


CLEAR = 0.005  # released: no part of either gripper within 5 mm of the key, the lid or a wheel


def key_corners(k) -> np.ndarray:
    """The corners of the key's two boxes (fob and blade), world frame."""
    pts = []
    for _, c, r, size in KEY.world_boxes({"base": (np.asarray(k.pos, float), _rot(k.quat_xyzw))}):
        h = np.asarray(size, float) / 2
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    pts.append(np.asarray(c, float) + np.asarray(r) @ (h * (sx, sy, sz)))
    return np.array(pts)


def key_inside(k) -> bool:
    """Every corner of the key inside the compartment's inner outline and below its rim."""
    p = key_corners(k)
    x0, x1 = X0 + IN_X0, X0 + IN_X1
    return bool(np.all(p[:, 0] >= x0 - EPS) and np.all(p[:, 0] <= x1 + EPS) and np.all(np.abs(p[:, 1] - Y0) <= IN_Y + EPS)
                and np.all(p[:, 2] <= TOP + HEIGHT + EPS))


def _grade(objects) -> float:
    safe, key = objects["key_safe"], objects["key"]
    q = safe.joints
    if not (_sim["clear"] and _sim["still"] and key.resting()):
        return 0.0
    if key_inside(key) and float(q["lid"]) <= SHUT + EPS and off_code(q):
        return 1.0
    return 0.0


def success(objects):
    """The key inside, the lid shut and at least one wheel off the code; let go and still."""
    return _grade(objects) == 1.0


def score(objects):
    """1.0 as success, else 0."""
    return _grade(objects)


# --- demo -------------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
HIGH = 0.12  # TCP travel height above the table
GRIP, WIDE = -0.008, -0.020  # fingers on the 20 mm fob (2 mm squeeze), and open around it


def _w(x, y, z):
    return (float(x), float(y), float(TOP + z))


def demo():
    """Left arm: picks the key up by its fob where the cameras show it and lowers it into the safe, blade toward the
    back, letting go 8 mm above the floor. Right arm: pushes the lid forward from behind, presses it down shut,
    then pushes the middle wheel's tooth away from the robot so the wheel turns off its number."""
    kx, ky, _ = lots.current().pos("key")
    gz = FOB[2] - 0.008 + TIP  # TCP with the tips 8 mm below the fob's top
    px, py = X0 + IN_X0 + 0.029, Y0  # fob centre over the compartment
    drop_z = FLOOR + 0.008 + FOB[2] - 0.008 + TIP
    left = [
        Waypoint(2.0, _w(kx, ky, gz + 0.06), WIDE, "over the key"),
        Waypoint(1.2, _w(kx, ky, gz), WIDE, "down to the fob"),
        Waypoint(0.8, _w(kx, ky, gz), GRIP, "grip the fob"),
        Waypoint(1.2, _w(kx, ky, HIGH), GRIP, "lift"),
        Waypoint(1.2, _w(px, ky, HIGH), GRIP, "in front of the lid"),
        Waypoint(1.5, _w(px, py, HIGH), GRIP, "over the safe"),
        Waypoint(1.5, _w(px, py, drop_z), GRIP, "lower it in"),
        Waypoint(0.6, _w(px, py, drop_z), WIDE, "let go"),
        Waypoint(1.2, _w(px, py, HIGH), WIDE, "up"),
        Waypoint(1.5, _w(0.20, 0.25, HIGH), WIDE, "park"),
    ]
    hinge_x, hinge_z = X0 + BACK, HEIGHT
    ly = Y0 - 0.035
    push_h = hinge_z + 0.030 + TIP
    press_x = X0 + IN_X0 + 0.030
    wx = X0 + WHEEL_X
    wz = AXIS_Z + 0.023 + TIP  # tips 3 mm above the rim, level with the tooth's top part
    right = [
        Waypoint(11.7, _w(0.20, -0.25, HIGH), FINGER_OPEN, "wait for the key"),
        Waypoint(0.8, _w(0.20, -0.25, HIGH), 0.0, "close the fingers"),
        Waypoint(2.0, _w(hinge_x + 0.050, ly, push_h + 0.012), 0.0, "behind the lid"),
        Waypoint(1.2, _w(hinge_x + 0.050, ly, push_h), 0.0, "down behind the lid"),
        Waypoint(2.0, _w(hinge_x - 0.035, ly, push_h), 0.0, "push the lid forward"),
        Waypoint(1.0, _w(hinge_x - 0.035, ly, HIGH), 0.0, "up"),
        Waypoint(1.0, _w(press_x, ly, HIGH), 0.0, "over the lid's middle"),
        Waypoint(2.0, _w(press_x, ly, HEIGHT + LID_T - 0.010 + TIP), 0.0, "press it shut"),
        Waypoint(1.0, _w(press_x, ly, HIGH), 0.0, "up"),
        Waypoint(1.5, _w(wx - 0.025, Y0, wz + 0.04), 0.0, "in front of the middle wheel"),
        Waypoint(1.0, _w(wx - 0.025, Y0, wz), 0.0, "down"),
        Waypoint(1.5, _w(wx + 0.015, Y0, wz), 0.0, "push the tooth away"),
        Waypoint(1.0, _w(wx + 0.015, Y0, HIGH), 0.0, "up"),
        Waypoint(1.5, _w(0.20, -0.25, HIGH), 0.0, "park"),
        Waypoint(2.0, _w(0.20, -0.25, HIGH), 0.0, "wait"),
    ]
    return {"left": left, "right": right}
