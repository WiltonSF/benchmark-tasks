"""On the station table, straight ahead of the robot: a saddle on a stand, a brown seat block with a steel stirrup bar
standing out from each side on a post at its far end. Each bar is open at the end facing the robot, where a small
red catch is hinged. Two stirrup leathers (brown strap with a loop at the top and a steel iron at the bottom) hang
loop-up on two pegs of a grey rack in front of the saddle, one on each side.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so its reset hook runs before the lots settle)
import lots  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

BROWN, DARK_BROWN, STEEL, RED, GREY = (0.45, 0.25, 0.12), (0.30, 0.16, 0.07), (0.62, 0.64, 0.67), (0.85, 0.10, 0.08), \
    (0.40, 0.41, 0.43)
SIDES = ("left", "right")
SIGN = {"left": 1.0, "right": -1.0}

# The saddle (fixed). Part frame: origin on the table at the saddle's middle, x away from the robot, z up.
X0 = 0.32
SEAT = (0.160, 0.110, 0.060)  # seat block, 110 to 170 mm up
SEAT_Z0 = 0.110
BAR_Y = 0.110  # each bar's centre line, either side of the midline
BAR_Z = 0.140  # bar centre height
BAR = (0.080, 0.010, 0.012)  # along x, 10 wide, 12 tall
BAR_END = -0.050  # the open end (part frame x), facing the robot; the bar runs to +0.030
POST_X = 0.030  # the post joins the bar's far end to the seat block


def _saddle_shapes():
    shapes = [
        parts.box((0.040, 0.040, SEAT_Z0), at=(0.0, 0.0, SEAT_Z0 / 2), color=GREY),  # stand
        parts.box(SEAT, at=(0.0, 0.0, SEAT_Z0 + SEAT[2] / 2), color=BROWN),
    ]
    for s in SIDES:
        y = SIGN[s] * BAR_Y
        shapes.append(parts.box(BAR, at=(BAR_END + BAR[0] / 2, y, BAR_Z), color=STEEL))
        inner, outer = SEAT[1] / 2, BAR_Y + BAR[1] / 2
        shapes.append(parts.box((0.010, outer - inner, BAR[2]), at=(POST_X + 0.005, SIGN[s] * (inner + outer) / 2, BAR_Z),
                                color=STEEL))
    return shapes


# Safety catches: a red tab 6 (x) x 12 (y) x 22 mm hinged about y 4 mm in front of the bar's open end and 6 mm above
# the bar's top. 0: hanging down beside the bar's end (open). Turned toward the robot and up to 155 degrees it stands
# up, leaning 25 degrees toward the robot, its tip 26 mm above the bar's top, across the loop's path (closed). Hanging, it swings
# freely up to 20 degrees toward the saddle when a loop brushes it.
CATCH_X = BAR_END - 0.004
CATCH_L = 0.022
CATCH_UP = math.radians(155)
CATCH_Z = BAR_Z + BAR[2] / 2 + 0.006


def _catch(side):
    y = SIGN[side] * BAR_Y
    return parts.Hinge(
        f"catch_{side}",
        pivot=(CATCH_X, y, CATCH_Z),
        axis=(0.0, 1.0, 0.0),
        lower=-0.35,
        upper=2.9,
        mass=0.01,
        spring=0.05,  # drive for the stiff-hinge hold below
        damping=0.002,
        shapes=[parts.box((0.006, 0.012, CATCH_L), at=(CATCH_X, y, CATCH_Z - CATCH_L / 2), color=RED)],
    )


SADDLE = parts.Part("saddle", pos=(X0, 0.0, TOP), shapes=_saddle_shapes(), joints=[_catch(s) for s in SIDES])

# Peg rack (fixed): a grey column on each side with a peg like the bars (open end toward the robot), 220 mm out.
PEG_Y, PEG_END, PEG_X1 = 0.220, 0.170, 0.230  # world


def _rack_shapes():
    shapes = []
    for s in SIDES:
        y = SIGN[s] * PEG_Y
        shapes.append(parts.box((PEG_X1 - PEG_END, BAR[1], BAR[2]), at=((PEG_END + PEG_X1) / 2, y, BAR_Z), color=GREY))
        shapes.append(parts.box((0.012, 0.020, BAR_Z + BAR[2] / 2), at=(PEG_X1 + 0.006, y, (BAR_Z + BAR[2] / 2) / 2),
                                color=GREY))
    return shapes


RACK = parts.Part("peg_rack", pos=(0.0, 0.0, TOP), shapes=_rack_shapes())

# A stirrup leather (rigid, 0.12 kg): a loop of 6 mm brown bar, 30 mm along x, 36 x 40 mm outside, opening 24 (y) x
# 28 mm (z); under it a strap 30 x 6 x 60 mm, and a steel iron 50 x 30 x 20 mm at the bottom. Part frame: origin at
# the centre of the loop's opening.
LOOP_X, LOOP_IN_Y, LOOP_IN_Z, LOOP_BAR = 0.030, 0.024, 0.028, 0.006
STRAP_Z0, STRAP_L = -0.020, 0.060
IRON = (0.050, 0.030, 0.020)


def _leather_shapes():
    oy, oz = LOOP_IN_Y / 2 + LOOP_BAR / 2, LOOP_IN_Z / 2 + LOOP_BAR / 2
    full_y = LOOP_IN_Y + 2 * LOOP_BAR
    return [
        parts.box((LOOP_X, full_y, LOOP_BAR), at=(0.0, 0.0, oz), color=DARK_BROWN),
        parts.box((LOOP_X, full_y, LOOP_BAR), at=(0.0, 0.0, -oz), color=DARK_BROWN),
        parts.box((LOOP_X, LOOP_BAR, LOOP_IN_Z), at=(0.0, oy, 0.0), color=DARK_BROWN),
        parts.box((LOOP_X, LOOP_BAR, LOOP_IN_Z), at=(0.0, -oy, 0.0), color=DARK_BROWN),
        parts.box((LOOP_X, 0.006, STRAP_L), at=(0.0, 0.0, STRAP_Z0 - STRAP_L / 2), color=BROWN),
        parts.box(IRON, at=(0.0, 0.0, STRAP_Z0 - STRAP_L - IRON[2] / 2), color=STEEL),
    ]


HANG_Z = BAR_Z + BAR[2] / 2 - LOOP_IN_Z / 2  # loop centre height when its top rests on a bar or peg (132 mm)
START_X = 0.190  # loop centre on its peg


def _leather(side):
    return parts.Part(f"leather_{side}", pos=(START_X, SIGN[side] * PEG_Y, TOP + HANG_Z), mass=0.12,
                      shapes=_leather_shapes())


LEATHERS = {s: _leather(s) for s in SIDES}
OBJECTS = [SADDLE, RACK, *LEATHERS.values()]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", up=("right",), place={"leather_left": LEATHERS["left"].pos}),  # a zero move: lot A settles too
    lots.Lot("B", up=("left",), place={"leather_left": LEATHERS["left"].pos}),
    lots.Lot("C", up=("left", "right"), place={"leather_left": LEATHERS["left"].pos}),
)

DOWN_MAX = math.radians(30)  # a catch counts as down within 30 degrees of hanging
HOLD = 0.05  # rad: a catch's stiff hinge holds it where it is left, within this
STILL = 0.005  # rad per control step
CLEAR = 0.005  # released: no part of either gripper within 5 mm of a leather or a catch
OFF_TABLE = 0.010  # a hung leather's lowest point at least 10 mm above the table
UPRIGHT = math.radians(20)


def _rot(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def hung_on(leather, side) -> bool:
    """The bar passes through the loop: the loop's opening around the bar's section, the loop wholly between the bar's
    open end and its post, the leather hanging upright with its lowest point clear of the table."""
    c = np.asarray(leather.pos, float)
    r = _rot(leather.quat_xyzw)
    if r[2, 2] < math.cos(UPRIGHT) - EPS:
        return False
    y = SIGN[side] * BAR_Y
    bar_top, bar_bot = TOP + BAR_Z + BAR[2] / 2, TOP + BAR_Z - BAR[2] / 2
    across = abs(c[1] - y) <= (LOOP_IN_Y - BAR[1]) / 2 + EPS
    height = c[2] + LOOP_IN_Z / 2 >= bar_top - EPS and c[2] - LOOP_IN_Z / 2 <= bar_bot + EPS
    along = c[0] - LOOP_X / 2 >= X0 + BAR_END - EPS and c[0] + LOOP_X / 2 <= X0 + POST_X + EPS
    return bool(across and height and along and leather.low[2] >= TOP + OFF_TABLE - EPS)


# Filled every control step in the simulator by the hooks below. Offline (near_misses.py) there is no robot, so the
# grippers count as clear and the catches as still.
_sim = {"clear": True, "still": True, "prev": None, "target": {}}


@parts.on_reset
def _set_lot(sim) -> None:
    """Stand this lot's catches up; the others hang down."""
    import torch

    _sim.update(clear=True, still=True, prev=None, gap=1.0)
    base = sim.env.unwrapped
    saddle = base.scene["saddle"]
    up = lots.current().up
    names = [f"catch_{s}" for s in SIDES]
    values = [CATCH_UP if s in up else 0.0 for s in SIDES]
    ids = saddle.find_joints(names, preserve_order=True)[0]
    saddle.write_joint_position_to_sim_index(position=torch.tensor([values], device=base.device), joint_ids=ids)
    saddle.write_joint_velocity_to_sim_index(velocity=torch.zeros((1, len(ids)), device=base.device), joint_ids=ids)
    for n, v in zip(names, values):
        _sim["target"][n] = v
        sim.set_target("saddle", n, v)


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
    sad = st["saddle"]
    poses = {f"catch_{s}": (np.asarray(sad.links[f"catch_{s}"][0]), _rot(sad.links[f"catch_{s}"][1])) for s in SIDES}
    boxes = [(c, r, s) for link, c, r, s in SADDLE.world_boxes(poses) if link != "base"]
    for side, part in LEATHERS.items():
        o = st[part.name]
        boxes += [(c, r, s) for _, c, r, s in part.world_boxes({"base": (np.asarray(o.pos), _rot(o.quat_xyzw))})]
    return boxes


_GAPS = os.environ.get("TASK_GAPS") == "1"
_BARS = [b for b in SADDLE.world_boxes({"base": (np.asarray(SADDLE.pos), np.eye(3))})
         if b[0] == "base" and abs(abs(b[1][1]) - BAR_Y) < 0.04]  # the two bars and two posts


@parts.on_step
def _step(sim) -> None:
    """The catches' stiff hinges, and how close the grippers are to the leathers and catches."""
    from yam_arena import analysis

    st = sim.states()
    q = st["saddle"].joints
    for s in SIDES:
        n = f"catch_{s}"
        v, t = float(q[n]), _sim["target"].get(n, 0.0)
        if abs(v - t) > HOLD:
            t = v - math.copysign(HOLD, v - t)
            _sim["target"][n] = t
            sim.set_target("saddle", n, t)
    now = np.array([float(q[f"catch_{s}"]) for s in SIDES])
    prev = _sim["prev"]
    _sim["still"] = bool(prev is None or np.max(np.abs(now - prev)) < STILL)
    _sim["prev"] = now
    hulls = _hulls(sim.env.unwrapped)
    _sim["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in _scored_boxes(st))
    if _GAPS:  # debugging only (TASK_GAPS=1): the closest the fingers come to the fixed bars and posts
        d, link = min(analysis.hulls_to_box(hulls, c, r, sz) for _, c, r, sz in _BARS)
        if d < _sim.get("gap", 1.0):
            _sim["gap"] = d
            print(f"[gaps] t={sim.t:.2f}s closest to a bar or post: {d * 1000:.1f} mm ({link})", flush=True)


def catch_down(joints, side) -> bool:
    return abs(float(joints[f"catch_{side}"])) <= DOWN_MAX + EPS


def _grade(objects) -> float:
    q = objects["saddle"].joints
    leathers = [objects[p.name] for p in LEATHERS.values()]
    if not (_sim["clear"] and _sim["still"] and all(o.resting() for o in leathers)):
        return 0.0
    if not all(catch_down(q, s) for s in SIDES):
        return 0.0
    hung = 0
    for side in SIDES:
        if any(hung_on(o, side) for o in leathers):
            hung += 1
    return 0.5 * hung


def success(objects):
    """A leather hangs on each bar and both catches are down; let go and still."""
    return _grade(objects) == 1.0


def score(objects):
    """0.5 for each bar with a leather hanging on it, if both catches are down at the end; else 0."""
    return _grade(objects)


# --- demo -------------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
GRIP, WIDE = -0.015, -0.028  # fingers on the loop's sides (36 mm across), and open around them
DEMO_TOUCHES = ("saddle", "peg_rack")  # check.py sees the fingers as closed; open, they pass either side of the bar
CARRY = 0.009  # carried with the loop's opening 9 mm above where it hangs: its top clears the hanging catch's hinge


def _w(x, y, z):
    return (float(x), float(y), float(TOP + z))


def _arm(side):
    """One arm's moves: flip its catch down if the cameras show it up, then take its leather off the peg and hang it
    on the bar from the open end."""
    sg = SIGN[side]
    py, by = sg * PEG_Y, sg * BAR_Y
    gz = HANG_Z + 0.005  # TCP 5 mm above the loop's centre; the pads on its sides
    high = gz + 0.050
    yw = math.pi / 2 if side == "right" else -math.pi / 2
    moves = []
    if side in lots.current().up:
        cx = X0 + CATCH_X - 0.010
        tz = CATCH_Z - CATCH_L - 0.006 + TIP  # tips 6 mm below the hanging tab's bottom
        sweep = X0 + CATCH_X - 0.004
        moves += [
            Waypoint(2.0, _w(cx, by, CATCH_Z + 0.040 + TIP), 0.0, "over the raised catch", yaw=yw),
            Waypoint(2.0, _w(cx, by, tz), 0.0, "press the catch over and down", yaw=yw),
            Waypoint(1.0, _w(sweep, by, tz), 0.0, "sweep it in to hang", yaw=yw),
            Waypoint(1.0, _w(cx - 0.020, by, tz), 0.0, "back off", yaw=yw),
            Waypoint(1.0, _w(cx - 0.020, by, high), 0.0, "up", yaw=yw),
        ]
    on_bar = X0 + BAR_END + 0.045  # loop centre on the bar: 30 mm in from the end
    before = X0 + BAR_END - 0.030
    moves += [
        Waypoint(2.0, _w(START_X, py, high), WIDE, "over the leather"),
        Waypoint(1.2, _w(START_X, py, gz), WIDE, "down round the loop"),
        Waypoint(0.8, _w(START_X, py, gz), GRIP, "grip the loop"),
        Waypoint(1.0, _w(START_X, py, gz + 0.008), GRIP, "lift it off the peg"),
        Waypoint(1.5, _w(PEG_END - 0.040, py, gz + 0.008), GRIP, "slide it off the peg's end"),
        Waypoint(2.0, _w(before, by, gz + CARRY), GRIP, "in line with the bar"),
        Waypoint(2.0, _w(on_bar, by, gz + CARRY), GRIP, "slide it onto the bar"),
        Waypoint(1.0, _w(on_bar, by, gz - 0.004), GRIP, "lower it onto the bar"),
        Waypoint(0.8, _w(on_bar, by, gz - 0.004), WIDE, "let go"),
        Waypoint(1.2, _w(on_bar, by, high), WIDE, "up"),
        Waypoint(1.5, _w(0.20, sg * 0.25, high), WIDE, "park"),
        Waypoint(2.0, _w(0.20, sg * 0.25, high), WIDE, "wait"),
    ]
    return moves


def demo():
    """Each arm works its own side. Where a catch stands up, a closed fingertip presses it over toward the robot and
    down until it hangs. Then the fingers take the leather's loop by its sides, lift it off its peg, slide it off the
    peg's end, carry it in line with the bar, slide it onto the bar from the open end and lower it to hang."""
    return {"left": _arm("left"), "right": _arm("right")}
