"""On the station table, a little in front of the robot: two white-painted door slabs lying flat side by side on low
blocks, their near edges facing the robot, each with a row of dark marks every 10 mm along both sides of its top. Each door has a square bore running into the middle of its near edge and
a square bore straight down through its face, crossing the edge bore some way in from the edge. In front of each
door a dark bench block runs up to the door edge, and on it lies a tubular latch in line with the edge bore, its
brass faceplate toward the robot: a steel tube with a brass hub at its far end. The hub has a hole straight down
through it and a small ridge across the top of its back wall, and slides along the tube to a second, longer setting, where it clicks.
Further out to each side, a holder carries an upright square steel spindle.
"""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import lots  # noqa: E402
import parts  # noqa: E402

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
X_EDGE = 0.30  # near edge of both doors
DOOR_L, DOOR_W, DOOR_T, DOOR_UP = 0.14, 0.12, 0.040, 0.020  # depth (x), width (y), thickness, raised on blocks
# Lot A (as declared): the 70 mm door on the robot's left. Lot B swaps the two doors. Value: bore centre y, backset.
DOORS = {"door_70": (0.09, 0.070), "door_60": (-0.09, 0.060)}
DOOR_MASS = 3.0  # loose but heavy, on the kit's grippy material (static 1.2, dynamic 1.0), so a lot can move it
TICK = (0.002, 0.012, 0.0006)  # dark scale marks on the door top, every 10 mm from the near edge
BORE = 0.030  # edge bore, square
FACE_BORE = 0.016  # face bore, square: the spindle (10 mm) passes only where the hub's 16 mm hole lines up with it
SKIN = (DOOR_T - BORE) / 2  # 5 mm of door above and below the edge bore
ZC = TOP + DOOR_UP + DOOR_T / 2  # edge bore centre line

# Latch, in its own frame: origin at the centre of the faceplate's back (the face that seats on the door edge),
# x along the tube into the door.
FACE = (0.004, 0.050, 0.020)  # faceplate: longer than the bore is wide, so it is the depth stop
FACE_DZ = (FACE[2] - 0.016) / 2  # its lower edge flush with the tube's underside
TUBE = (0.050, 0.016, 0.016)
HUB = 0.022  # hub block, 22 mm wide and tall (HUB_L: 24 mm long), with a 16 mm square hole straight down through it
HUB_L = 0.024
HUB_DZ = (HUB - TUBE[2]) / 2 + 0.001  # the hub's underside 1 mm above the tube's, so it never drags
TAB = (0.004, 0.012, 0.004)  # a ridge across the top of the hub's back wall, for sliding it
HUB_HOLE = 0.016
SHORT, LONG = 0.060, 0.070  # backset settings: faceplate back to hub centre
STEEL, BRASS, WHITE, DARK, WOOD = (0.62, 0.64, 0.66), (0.80, 0.65, 0.28), (0.93, 0.93, 0.90), (0.20, 0.20, 0.22), \
    (0.55, 0.40, 0.25)
_hw = (HUB - HUB_HOLE) / 2  # side walls
_hx = (HUB_L - HUB_HOLE) / 2  # end walls


def _hub_shapes():
    c = SHORT
    return [
        parts.box((_hx, HUB, HUB), at=(c - HUB_L / 2 + _hx / 2, 0.0, HUB_DZ), color=BRASS),
        parts.box((_hx, HUB, HUB), at=(c + HUB_L / 2 - _hx / 2, 0.0, HUB_DZ), color=BRASS),
        parts.box((HUB_HOLE, _hw, HUB), at=(c, HUB / 2 - _hw / 2, HUB_DZ), color=BRASS),
        parts.box((HUB_HOLE, _hw, HUB), at=(c, -HUB / 2 + _hw / 2, HUB_DZ), color=BRASS),
        parts.box((0.030, 0.008, 0.008), at=(TUBE[0] - 0.015, 0.0, 0.0), color=STEEL),  # slide bar inside the tube
        parts.box(TAB, at=(c - HUB_L / 2 + _hx / 2, 0.0, HUB_DZ + HUB / 2 + TAB[2] / 2), color=DARK),  # thumb ridge
    ]


STAND_X = X_EDGE - 0.11  # faceplate back at the start
RAIL_Z = ZC - BORE / 2  # bench top: level with the edge bore's floor
LZ = RAIL_Z + TUBE[2] / 2  # latch axis on the rail
SEAT_Z = ZC - BORE / 2 + TUBE[2] / 2  # latch axis seated on the bore floor
RAIL = (X_EDGE - STAND_X + 0.030, 0.060, RAIL_Z - TOP)  # a bench block from behind the latch to the door edge


def _latch(name, y):
    return parts.Part(
        name, pos=(STAND_X, y, LZ), mass=0.060,
        shapes=[parts.box(FACE, at=(-FACE[0] / 2, 0.0, FACE_DZ), color=BRASS),
                parts.box(TUBE, at=(TUBE[0] / 2, 0.0, 0.0), color=STEEL)],
        joints=[parts.Slider("hub", shapes=_hub_shapes(), axis=(1.0, 0.0, 0.0), travel=LONG - SHORT, mass=0.015,
                             spring=60.0, damping=2.0)],
    )


def _door(name, y, backset):
    """A door slab with its edge bore (along x, at mid-thickness) and face bore (vertical, at the backset)."""
    z0, z1 = DOOR_UP, DOOR_UP + DOOR_T
    lo, hi = z0 + SKIN, z1 - SKIN  # edge bore floor and roof
    h = BORE / 2
    bore_end = backset + 0.025
    s = []
    f = FACE_BORE / 2
    for za, zb in ((z0, lo), (hi, z1)):  # bottom and top skins, each with the face-bore hole
        zm, zt = (za + zb) / 2, zb - za
        s += [parts.box((backset - f, DOOR_W, zt), at=((backset - f) / 2, 0.0, zm), color=WHITE),
              parts.box((DOOR_L - backset - f, DOOR_W, zt), at=((DOOR_L + backset + f) / 2, 0.0, zm), color=WHITE),
              parts.box((FACE_BORE, DOOR_W / 2 - f, zt), at=(backset, (DOOR_W / 2 + f) / 2, zm), color=WHITE),
              parts.box((FACE_BORE, DOOR_W / 2 - f, zt), at=(backset, -(DOOR_W / 2 + f) / 2, zm), color=WHITE)]
    zm, zt = (lo + hi) / 2, hi - lo
    s += [parts.box((DOOR_L, DOOR_W / 2 - h, zt), at=(DOOR_L / 2, (DOOR_W / 2 + h) / 2, zm), color=WHITE),
          parts.box((DOOR_L, DOOR_W / 2 - h, zt), at=(DOOR_L / 2, -(DOOR_W / 2 + h) / 2, zm), color=WHITE),
          parts.box((DOOR_L - bore_end, BORE, zt), at=((DOOR_L + bore_end) / 2, 0.0, zm), color=WHITE)]
    for k in range(1, int(DOOR_L * 100 + 1e-6)):  # scale marks at 10, 20, ... 130 mm in, along both long sides
        for side in (-1.0, 1.0):
            s.append(parts.box(TICK, at=(0.010 * k, side * (DOOR_W / 2 - 0.004 - TICK[1] / 2), z1 + TICK[2] / 2),
                               color=DARK))
    for bx in (0.005, DOOR_L - 0.025):  # two blocks under the slab, clear of the face bore
        s.append(parts.box((0.020, DOOR_W, DOOR_UP), at=(bx + 0.010, 0.0, DOOR_UP / 2), color=WOOD))
    # Bench block in front of the door, under the latch, up to the door edge.
    s.append(parts.box(RAIL, at=(-RAIL[0] / 2, 0.0, RAIL[2] / 2), color=DARK))
    return parts.Part(name, pos=(X_EDGE, y, TOP), mass=DOOR_MASS, shapes=s)


SPINDLE = (0.010, 0.010, 0.090)
HOLDER_X = 0.24
HOLDER_Y = {"spindle_left": 0.24, "spindle_right": -0.24}
HOLDER_H = 0.030


def _holder(name, y):
    g, t = 0.014, 0.008  # hole, wall
    o = g + 2 * t
    return parts.Part(name, pos=(HOLDER_X, y, TOP), shapes=[
        parts.box((t, o, HOLDER_H), at=((g + t) / 2, 0.0, HOLDER_H / 2), color=DARK),
        parts.box((t, o, HOLDER_H), at=(-(g + t) / 2, 0.0, HOLDER_H / 2), color=DARK),
        parts.box((g, t, HOLDER_H), at=(0.0, (g + t) / 2, HOLDER_H / 2), color=DARK),
        parts.box((g, t, HOLDER_H), at=(0.0, -(g + t) / 2, HOLDER_H / 2), color=DARK)])


LATCHES = {"latch_left": 0.09, "latch_right": -0.09}  # each in line with the door bore on its side
SPINDLES = {n: parts.Part(n, pos=(HOLDER_X, y, TOP + SPINDLE[2] / 2), mass=0.030,
                          shapes=[parts.box(SPINDLE, color=STEEL)]) for n, y in HOLDER_Y.items()}
OBJECTS = [*(_door(n, y, b) for n, (y, b) in DOORS.items()),
           *(_holder(f"holder_{n.split('_')[1]}", y) for n, y in HOLDER_Y.items()),
           *(_latch(n, y) for n, y in LATCHES.items()), *SPINDLES.values()]
_DOOR_POS = {o.name: o.pos for o in OBJECTS if o.name in DOORS}
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A"),
    lots.Lot("B", place={"door_70": _DOOR_POS["door_60"], "door_60": _DOOR_POS["door_70"]}),
)


def door_layout():
    """This trial's doors: name -> (bore centre y, backset), from the current lot's starting positions."""
    lot = lots.current()
    return {n: (float(lot.pos(n)[1]), b) for n, (_, b) in DOORS.items()}

# Two-position backset, as on a real adjustable latch: the hub's spring pulls it to the nearer setting, 60 or 70 mm.
# Past half-way the rest point jumps to the other setting, so the hub clicks into it.
_click = {n: 0.0 for n in LATCHES}


@parts.on_reset
def _click_reset(sim):
    for n in LATCHES:
        _click[n] = 0.0
        sim.set_target(n, "hub", 0.0)


@parts.on_step
def _click_step(sim):
    st = sim.states()
    for n in LATCHES:
        target = LONG - SHORT if st[n].joints["hub"] >= (LONG - SHORT) / 2 else 0.0
        if target != _click[n]:
            _click[n] = target
            sim.set_target(n, "hub", target)


# ---- scoring -------------------------------------------------------------------------------------------------
EPS = 1e-6
SEAT_TOL = 0.003  # faceplate back at most 3 mm proud of the door edge
AXIS_DEG = 3.0
HUB_TOL = 0.003  # hub hole centre within 3 mm of the face bore's axis
SPINDLE_DEG = 5.0
CLEAR = 0.005  # released: no finger or gripper surface within 5 mm of a latch or spindle
SUCCESS_HOLD_S = 3.0


def _rot(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _hub_centre(latch) -> np.ndarray:
    return np.asarray(latch.pos, float) + _rot(latch.quat_xyzw) @ np.array([SHORT + latch.joints["hub"], 0.0, HUB_DZ])


def _fitted(latch, y, backset) -> bool:
    """Seated in this door's edge bore with its hub hole on the face bore."""
    r = _rot(latch.quat_xyzw)
    p = np.asarray(latch.pos, float)
    seated = (X_EDGE - SEAT_TOL - EPS <= p[0] <= X_EDGE + 0.001 and abs(p[1] - y) <= 0.004
              and abs(p[2] - SEAT_Z) <= 0.004 and r[0, 0] >= np.cos(np.radians(AXIS_DEG)) - EPS)
    hub = _hub_centre(latch)
    on_bore = np.hypot(hub[0] - (X_EDGE + backset), hub[1] - y) <= HUB_TOL + EPS
    return bool(seated and on_bore and latch.resting())


def _spindle_through(sp, latch, y, backset) -> bool:
    """Upright, on the face bore's axis, and its lower end below the hub's underside."""
    r = _rot(sp.quat_xyzw)
    up = abs(r[2, 2]) >= np.cos(np.radians(SPINDLE_DEG)) - EPS
    p = np.asarray(sp.pos, float)
    on_axis = np.hypot(p[0] - (X_EDGE + backset), p[1] - y) <= HUB_HOLE / 2 + EPS
    below = sp.low[2] < _hub_centre(latch)[2] - HUB / 2
    return bool(up and on_axis and below and sp.resting())


# Released: measured in the simulator after every control step, from both arms' finger and gripper collision hulls
# against points on every latch's and spindle's surface. Offline (near_misses.py) it is taken as true.
_GRIP = {"clear": True}
_LINKS = ("gripper", "tip_left", "tip_right")


def _box_surface(size, step=0.004) -> np.ndarray:
    h = np.asarray(size, float) / 2
    grids = [np.linspace(-v, v, max(2, int(np.ceil(2 * v / step)) + 1)) for v in h]
    pts = []
    for axis in range(3):
        a, b = [i for i in range(3) if i != axis]
        ga, gb = np.meshgrid(grids[a], grids[b], indexing="ij")
        for sign in (-1.0, 1.0):
            q = np.zeros((ga.size, 3))
            q[:, a], q[:, b], q[:, axis] = ga.ravel(), gb.ravel(), sign * h[axis]
            pts.append(q)
    return np.concatenate(pts)


_LATCH_PTS = np.concatenate([
    _box_surface(FACE) + [-FACE[0] / 2, 0, FACE_DZ],
    _box_surface(TUBE) + [TUBE[0] / 2, 0, 0],
])
_HUB_PTS = _box_surface((HUB_L, HUB, HUB))
_SPINDLE_PTS = _box_surface(SPINDLE)


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
    st = sim.states()
    pts = []
    for n in LATCHES:
        s = st[n]
        r = _rot(s.quat_xyzw)
        pts += [np.asarray(s.pos) + _LATCH_PTS @ r.T, _hub_centre(s) + _HUB_PTS @ r.T]
    for n in SPINDLES:
        s = st[n]
        pts.append(np.asarray(s.pos) + _SPINDLE_PTS @ _rot(s.quat_xyzw).T)
    _GRIP["clear"] = _clearance(_robot_hulls(sim), np.concatenate(pts)) >= CLEAR - EPS


def released(objects) -> bool:
    return bool(_GRIP["clear"])


def _doors(objects):
    """Per door: (latch fitted, spindle through it). Either latch and either spindle may go in either door. Each door
    is judged where the current lot started it."""
    out = []
    for y, b in door_layout().values():
        fitted = [n for n in LATCHES if _fitted(objects[n], y, b)]
        through = any(_spindle_through(objects[s], objects[n], y, b) for n in fitted for s in SPINDLES)
        out.append((bool(fitted), through))
    return out


def success(objects):
    """Both doors: latch seated at the right length with a spindle through its hub; let go; still."""
    return bool(released(objects) and all(f and t for f, t in _doors(objects)))


def distance(objects):
    d = 0.0
    for n, y in LATCHES.items():
        d += float(np.linalg.norm(np.asarray(objects[n].pos) - np.array((X_EDGE, y, ZC))))
    return d


def score(objects):
    """1.0 as success; 0.5 both latches seated at the right lengths without both spindles through; otherwise 0."""
    if success(objects):
        return 1.0
    return 0.5 if released(objects) and all(f for f, _ in _doors(objects)) else 0.0


# ---- scripted demo -------------------------------------------------------------------------------------------
TIP = 0.0129  # finger tips below the TCP
TIP_FRONT = 0.0094  # closed tips' front face ahead of the TCP
TRAVEL_Z = TOP + 0.12
SAG = {"left": (0.0026, -0.0005, 0.004), "right": (-0.0010, -0.0017, 0.004)}  # spindle carry compensation (m), traces
# Lowering the spindle, the right arm sags toward its base by about 3 mm in x and y (traces): aim that much beyond.
LOWER_SAG = {"left": (0.0, 0.0), "right": (0.005, 0.005)}
SEATED_SHORT = 0.0024  # pushed home, the faceplate settles 2.4 mm short of the door edge in the simulator (traces)


def _fit(arm, latch_y, backset, extend):
    """One arm: optionally draw the hub out to the long setting; push the latch by its faceplate along its rail and
    home into the edge bore; take the spindle from its holder and drop it through the hub."""
    w = []
    sx, sy, sz = SAG[arm]
    if extend:
        tab_back = STAND_X + SHORT - HUB_L / 2 + _hx / 2 - TAB[0] / 2
        tz = LZ + HUB_DZ + HUB / 2 + TAB[2] / 2 + TIP  # closed tips level with the tab's middle
        w += [
            Waypoint(1.5, (tab_back - TIP_FRONT - 0.010, latch_y, TRAVEL_Z), FINGER_CLOSED, "behind the hub's ridge"),
            Waypoint(1.2, (tab_back - TIP_FRONT - 0.010, latch_y, tz), FINGER_CLOSED, "down behind it"),
            Waypoint(1.5, (tab_back - TIP_FRONT + 0.012, latch_y, tz), FINGER_CLOSED, "slide the hub out past half-way"),
            Waypoint(0.6, (tab_back - TIP_FRONT - 0.005, latch_y, tz), FINGER_CLOSED, "back off: it clicks to 70 mm"),
            Waypoint(0.8, (tab_back - TIP_FRONT - 0.005, latch_y, TRAVEL_Z), FINGER_CLOSED, "up"),
        ]
    pz_push = LZ + TIP - 0.002  # closed tips 2 mm below the latch's axis, on the faceplate's front
    back = STAND_X - FACE[0] - TIP_FRONT - 0.012
    w += [
        Waypoint(1.5, (back, latch_y, TRAVEL_Z), FINGER_CLOSED, "behind the faceplate"),
        Waypoint(1.2, (back, latch_y, pz_push), FINGER_CLOSED, "down behind it"),
        Waypoint(1.0, (STAND_X - FACE[0] - TIP_FRONT, latch_y, pz_push), FINGER_CLOSED, "touch the faceplate"),
        Waypoint(5.0, (X_EDGE - FACE[0] - TIP_FRONT + 0.025, latch_y, pz_push), FINGER_CLOSED,
                 "push it along the bench and home into the bore"),
        Waypoint(0.6, (X_EDGE - FACE[0] - TIP_FRONT - 0.020, latch_y, pz_push), FINGER_CLOSED, "back off"),
        Waypoint(0.8, (X_EDGE - 0.04, latch_y, TRAVEL_Z), FINGER_OPEN, "up"),
    ]
    return w


def _drop_spindle(arm, name, latch_y, backset):
    """One arm takes a spindle from its holder, carries it over the face bore and lowers it through the hub."""
    sx, sy, sz = SAG[arm]
    lx, ly = LOWER_SAG[arm]
    hx, hy = HOLDER_X, HOLDER_Y[name]
    top = TOP + SPINDLE[2]
    pz = top - 0.030 + TIP  # tips 30 mm below the spindle top
    bx = X_EDGE - SEATED_SHORT / 2 + backset  # between the face bore and the hub hole, which ends up 2.4 mm nearer
    return [
        Waypoint(1.5, (hx, hy, TRAVEL_Z), FINGER_OPEN, "over the spindle"),
        Waypoint(1.2, (hx, hy, pz), FINGER_OPEN, "down to it"),
        Waypoint(0.8, (hx, hy, pz), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (hx, hy, pz + 0.075), FINGER_CLOSED, "lift out of the holder"),
        Waypoint(1.8, (bx + sx, latch_y + sy, pz + 0.075 + sz), FINGER_CLOSED, "over the face bore"),
        Waypoint(1.2, (bx + sx + lx / 2, latch_y + sy + ly / 2, pz + 0.075 + sz), FINGER_CLOSED, "let it stop swinging"),
        Waypoint(3.5, (bx + sx + lx, latch_y + sy + ly, pz + 0.007 + sz), FINGER_CLOSED, "down through the hub"),
        Waypoint(0.6, (bx + sx + lx, latch_y + sy + ly, pz + 0.007 + sz), FINGER_OPEN, "let go"),
        Waypoint(1.0, (bx, latch_y, TRAVEL_Z), FINGER_OPEN, "up"),
    ]


def demo():
    """Each arm fits the latch on its side into the door the current lot puts there, drawing the hub out to 70 mm
    only for the 70 mm door (the left door in lot A, the right one in lot B)."""
    backset = {round(y, 3): b for y, b in door_layout().values()}
    plans = {}
    for arm, latch in (("left", "latch_left"), ("right", "latch_right")):
        y = LATCHES[latch]
        b = backset[round(y, 3)]
        plans[arm] = _fit(arm, y, b, extend=b > SHORT + 0.005) + _drop_spindle(arm, f"spindle_{arm}", y, b)
    left, right = plans["left"], plans["right"]
    park = {"left": (0.18, 0.28, TRAVEL_Z), "right": (0.18, -0.28, TRAVEL_Z)}
    t = max(sum(w.duration for w in left), sum(w.duration for w in right))
    for arm, w in (("left", left), ("right", right)):
        w.append(Waypoint(1.5, park[arm], FINGER_OPEN, "park"))
        w.append(Waypoint(t - sum(x.duration for x in w) + 5.5, park[arm], FINGER_OPEN, "wait, hands clear"))
    return {"left": left, "right": right}
