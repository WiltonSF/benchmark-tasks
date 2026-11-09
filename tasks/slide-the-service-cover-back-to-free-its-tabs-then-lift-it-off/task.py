"""On the station table, between the arms and a little to the left of the midline: a dark grey equipment unit, an open
box with a pale aluminium service cover lying in the top of it. The cover has a square grip post at its centre. Along one
edge of the unit an orange strip stands just over the cover's edge. Which edge carries the strip changes between trials.
To the robot's left lies a flat grey mat.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so its reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

DARK, PALE, MAT_GREY, POST = (0.22, 0.23, 0.25), (0.86, 0.85, 0.80), (0.55, 0.56, 0.58), (0.30, 0.32, 0.36)
ORANGE = (0.95, 0.42, 0.05)  # the lip end: back wall, upright and lip

# Chassis. Part frame: origin on the table under the cover's centre, +x toward the lip edge, z up (mm noted).
# Floor 184 x 140 x 6; walls up to the seat at 60; the cover lies on the wall tops. At the +x edge a back wall
# carries an upright (x 86-92, up to 68) and a lip plate (x 71-92, z 68-72) that reaches 4 mm over the cover. Under
# the lip, keeper blocks (x 75-85, z 60-68) leave two slots 24 mm wide for the cover's tabs.
L, W = 0.184, 0.140
SEAT = 0.060
WALL = 0.016
SIDE_IN = 0.055  # side walls from |y| 55 to 70 mm, so the cover bears on them
LIP_FRONT, LIP_BOTTOM, LIP_TOP = 0.071, 0.068, 0.072
UPRIGHT_X = 0.086
TAB_Y, TAB_W, TAB_L = 0.035, 0.020, 0.010  # tab centres at y +-35 mm, 20 mm wide, reaching 10 mm past the edge
SLOT_HALF = TAB_W / 2 + 0.002  # 2 mm clearance each side of a tab
CHASSIS_MASS = 5.0
CHASSIS_XY = (0.32, 0.06)


def _keepers():
    """Keeper blocks under the lip between and beside the tab slots."""
    xs, zc, h = (0.075 + 0.085) / 2, (SEAT + LIP_BOTTOM) / 2, LIP_BOTTOM - SEAT
    lo_in, hi_in = TAB_Y - SLOT_HALF, TAB_Y + SLOT_HALF  # 23, 47 mm
    out = [parts.box((0.010, 2 * lo_in, h), at=(xs, 0.0, zc), color=DARK)]
    for s in (1, -1):
        out.append(parts.box((0.010, W / 2 - hi_in, h), at=(xs, s * (hi_in + W / 2) / 2, zc), color=DARK))
    return out


CHASSIS_SHAPES = [
    parts.box((L, W, 0.006), at=(0.0, 0.0, 0.003), color=DARK),  # floor
    parts.box((WALL, W, SEAT - 0.006), at=(-L / 2 + WALL / 2, 0.0, (SEAT + 0.006) / 2), color=DARK),  # front wall
    parts.box((WALL, W, SEAT - 0.006), at=(L / 2 - WALL / 2, 0.0, (SEAT + 0.006) / 2), color=ORANGE),  # back wall
    *[parts.box((L - 2 * WALL, W / 2 - SIDE_IN, SEAT - 0.006), at=(0.0, s * (SIDE_IN + W / 2) / 2, (SEAT + 0.006) / 2),
                color=DARK) for s in (1, -1)],  # side walls
    parts.box((L / 2 - UPRIGHT_X, W, LIP_BOTTOM - SEAT), at=((UPRIGHT_X + L / 2) / 2, 0.0, (SEAT + LIP_BOTTOM) / 2),
              color=ORANGE),  # upright
    parts.box((L / 2 - LIP_FRONT, W, LIP_TOP - LIP_BOTTOM),
              at=((LIP_FRONT + L / 2) / 2, 0.0, (LIP_BOTTOM + LIP_TOP) / 2), color=ORANGE),  # lip
    *_keepers(),
]
CHASSIS = parts.Part("chassis", pos=(*CHASSIS_XY, TOP), mass=CHASSIS_MASS, shapes=CHASSIS_SHAPES)

# Cover. Part frame: origin at the centre of its underside, +x toward its tabbed edge.
COVER_L, COVER_W, COVER_T = 0.150, 0.122, 0.006
POST_W, POST_H = 0.024, 0.025
COVER_MASS = 0.30  # aluminium: 150 x 122 x 6 mm is 0.30 kg
PLATE = parts.box((COVER_L, COVER_W, COVER_T), at=(0.0, 0.0, COVER_T / 2), color=PALE)
TABS = [parts.box((TAB_L, TAB_W, COVER_T), at=(COVER_L / 2 + TAB_L / 2, s * TAB_Y, COVER_T / 2), color=PALE)
        for s in (1, -1)]
GRIP = parts.box((POST_W, POST_W, POST_H), at=(0.0, 0.0, COVER_T + POST_H / 2), color=POST)
COVER = parts.Part("cover", pos=(*CHASSIS_XY, TOP + SEAT), mass=COVER_MASS, shapes=[PLATE, *TABS, GRIP])

# Mat: fixed, 220 mm front to back, 200 mm wide, 3 mm thick.
MAT_SIZE = (0.220, 0.200, 0.003)
MAT_XY = (0.24, 0.27)
MAT = Box("mat", pos=(*MAT_XY, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=MAT_GREY)
MAT_TOP = TOP + MAT_SIZE[2]

OBJECTS = [CHASSIS, COVER, MAT]

# Lot A (the screenshot): the lip on the far edge. Lot B: unit and cover turned 180 degrees (lip on the near edge).
# Lot C: turned 90 degrees anticlockwise seen from above (lip on the robot's left edge). Lot A gets a zero move so it
# too settles and takes a fresh first frame. A diagonal habit (20 mm toward the robot and 20 mm right, then lift and
# carry to the mat) scored 0 in all three lots on the GPU. A fourth lot with the lip on the right was tried and
# dropped: there the carry toward the mat is itself the free direction, so the same habit scored 1.0.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", yaw={"chassis": 0.0, "cover": 0.0}),
    lots.Lot("B", yaw={"chassis": math.pi, "cover": math.pi}),
    lots.Lot("C", yaw={"chassis": math.pi / 2, "cover": math.pi / 2}),
)

EPS = 1e-6
SEATED_Z = 0.002  # in its seat: the cover's underside within 2 mm of the seat height
SEATED_TILT = math.radians(3.0)
ON_MAT = 0.002  # on the mat: lowest point within 2 mm of the mat's top
MAT_TILT = math.radians(5.0)
CHASSIS_MOVE, CHASSIS_TILT = 0.003, math.radians(3.0)
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the cover


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(state) -> float:
    return math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0)))


def _corners(shape, pos, rot) -> np.ndarray:
    """The 8 corners of a cover shape, world frame."""
    signs = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], float)
    local = np.asarray(shape.at) + signs * shape.half
    return np.asarray(pos) + local @ rot.T


def _in_chassis(cover, chassis):
    """The cover's pose in the chassis frame: (origin, rotation)."""
    rc = _rot(chassis.quat_xyzw)
    return rc.T @ (np.asarray(cover.pos) - np.asarray(chassis.pos)), rc.T @ _rot(cover.quat_xyzw)


def tabs_clear(cover, chassis) -> bool:
    """Both tabs wholly in front of the lip's front edge (measured along the chassis, away from the lip)."""
    p, r = _in_chassis(cover, chassis)
    far = max(float(_corners(t, p, r)[:, 0].max()) for t in TABS)
    return far <= LIP_FRONT + EPS


def _seated_flat(cover, chassis) -> bool:
    p, r = _in_chassis(cover, chassis)
    tilt = math.acos(float(np.clip(r[2, 2], -1.0, 1.0)))
    under = float(_corners(PLATE, p, r)[:, 2].min())
    return bool(abs(under - SEAT) <= SEATED_Z + EPS and tilt <= SEATED_TILT + EPS and abs(p[0]) <= L / 2)


def _on_mat(cover) -> bool:
    r = _rot(cover.quat_xyzw)
    c = _corners(PLATE, cover.pos, r)
    (mx, my), (hx, hy) = MAT_XY, (MAT_SIZE[0] / 2, MAT_SIZE[1] / 2)
    inside = bool(np.all(np.abs(c[:, 0] - mx) <= hx + EPS) and np.all(np.abs(c[:, 1] - my) <= hy + EPS))
    low = float(cover.low[2])
    return bool(inside and MAT_TOP - 0.001 - EPS <= low <= MAT_TOP + ON_MAT + EPS and _tilt(cover) <= MAT_TILT + EPS)


def _chassis_disturbed(chassis) -> bool:
    start = np.asarray(lots.current().pos("chassis"), float)
    return bool(np.linalg.norm(np.asarray(chassis.pos) - start) > CHASSIS_MOVE + EPS or _tilt(chassis) > CHASSIS_TILT + EPS)


# Whole-trial record: did the chassis ever move more than 3 mm or tilt more than 3 degrees? Cleared only by a scene
# reset: the simulator's reset hook, or (offline) the cover and chassis back within 0.5 mm of their starts and at rest
# after one of them jumped over 20 mm in one control step (a robot cannot move them that way).
_record = {"fault": False, "prev": None}
# Released: in the simulator an on_step callback measures, every step, how close both arms' fingers and grippers are
# to the cover. Offline (near_misses.py) there is no robot, so "clear" stays True.
_grip = {"clear": True}


@parts.on_reset
def _new_trial(sim) -> None:
    _record["fault"], _record["prev"] = False, None


def _update(objects) -> None:
    cover, chassis = objects["cover"], objects["chassis"]
    lot = lots.current()
    now = np.r_[np.asarray(cover.pos, float), np.asarray(chassis.pos, float)]
    prev = _record["prev"]
    home = all(np.linalg.norm(np.asarray(objects[n].pos) - np.asarray(lot.pos(n))) < 0.0005 and objects[n].resting()
               for n in ("cover", "chassis"))
    if prev is not None and home and max(np.linalg.norm(now[:3] - prev[:3]), np.linalg.norm(now[3:] - prev[3:])) > 0.020:
        _record["fault"] = False
    _record["prev"] = now
    if _chassis_disturbed(chassis):
        _record["fault"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    cover = sim.states()["cover"]
    boxes = COVER.world_boxes({"base": (np.asarray(cover.pos), _rot(cover.quat_xyzw))})
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
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for _, c, r, s in boxes)


def _done(objects) -> bool:
    cover = objects["cover"]
    return bool(_on_mat(cover) and cover.resting() and _grip["clear"])


def success(objects):
    """The cover lies flat on the mat, let go and still, and the unit was never shoved."""
    _update(objects)
    return bool(not _record["fault"] and _done(objects))


def distance(objects):
    return float(np.linalg.norm(np.asarray(objects["cover"].pos[:2]) - np.asarray(MAT_XY)))


def score(objects):
    """1.0 on success; 0.5 for a cover freed but left in place; otherwise 0."""
    _update(objects)
    if _record["fault"]:
        return 0.0
    if _done(objects):
        return 1.0
    cover, chassis = objects["cover"], objects["chassis"]
    if tabs_clear(cover, chassis) and _seated_flat(cover, chassis) and cover.resting() and _grip["clear"]:
        return 0.5
    return 0.0


# --- demo: left arm only ----------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
GRIP_ABOVE = COVER_T + 0.006 + TIP  # TCP above the cover's underside: tips 6 mm above the cover's top
PINCH = -0.009  # pads about 18 mm apart: 3 mm past each face of the 24 mm post
OPEN = -0.030
SLIDE = 0.020  # 6 mm more than the 14 mm that frees the tabs
LIFT = 0.100  # cover underside while carried
RELEASE = 0.007  # cover underside above the mat at release
TRAVEL = 0.15


def _w(x, y, z):
    return (float(x), float(y), float(TOP + z))


def demo():
    """Left arm: grip the post, slide the cover straight away from the lip until its edge is out from under it, lift
    it, carry it over the mat, lower it and let go. The lip's edge is the one the cameras show for the trial's lot."""
    lot = lots.current()
    cx, cy, _ = lot.pos("cover")
    psi = lot.yaw_of("chassis")
    dx, dy = -math.cos(psi), -math.sin(psi)  # away from the lip
    sx, sy = cx + SLIDE * dx, cy + SLIDE * dy
    mx, my = MAT_XY
    seat = SEAT + GRIP_ABOVE
    # Fingers close across the slide (the slide is then carried by the pads' grip, as along x); a slide along y
    # pushed against one finger only moved the cover 1 mm on the GPU.
    yaw = 0.0 if abs(dx) > abs(dy) else -math.pi / 2
    return {"left": [Waypoint(w.duration, w.pos, w.finger, w.label, yaw) for w in [
        Waypoint(1.5, _w(cx, cy, TRAVEL), OPEN, "over the post"),
        Waypoint(1.5, _w(cx, cy, seat), OPEN, "down to the post"),
        Waypoint(0.8, _w(cx, cy, seat), PINCH, "grip"),
        Waypoint(2.0, _w(sx, sy, seat), PINCH, "slide away from the lip"),
        Waypoint(1.5, _w(sx, sy, LIFT + GRIP_ABOVE), PINCH, "lift"),
        Waypoint(2.5, _w(mx, my, LIFT + GRIP_ABOVE), PINCH, "carry over the mat"),
        Waypoint(1.5, _w(mx, my, MAT_SIZE[2] + RELEASE + GRIP_ABOVE), PINCH, "lower"),
        Waypoint(0.6, _w(mx, my, MAT_SIZE[2] + RELEASE + GRIP_ABOVE), OPEN, "release"),
        Waypoint(1.2, _w(mx, my, TRAVEL), OPEN, "up"),
        Waypoint(3.0, _w(mx, my, TRAVEL), OPEN, "wait while the cover settles"),
    ]]}
