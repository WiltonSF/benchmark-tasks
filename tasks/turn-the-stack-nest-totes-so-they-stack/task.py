"""On the station table stand two blue plastic stack-nest totes, one in front of each arm, fixed in place. The left
blue tote holds a brown carton of parts; the right blue tote is empty. In front of each blue tote, nearer the robot,
an empty grey tote of the same model stands open side up on the table. Every tote is an open-topped container,
75 mm tall, with tapered walls, a floor and a thick vertical lip round its rim (216 x 166 mm over the lip). Each long side wall has two yellow
lugs on its inner face near the rim and, at the same places, a recess moulded into its lower part (seen from inside
as a dark bump). The grey totes are loose and light; their walls can be pinched at the rim.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import parts  # noqa: E402
import lots  # noqa: E402  (after parts, so its reset hooks run first)
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 3.0  # both totes must qualify for 3 s: time for the arms to park clear

BLUE_C, GREY_C = (0.15, 0.33, 0.75), (0.58, 0.60, 0.62)
LUG_C, RECESS_C, CARTON_C = (0.95, 0.80, 0.15), (0.20, 0.21, 0.24), (0.66, 0.49, 0.30)

# -- the tote (one model for all four) ------------------------------------------------------------------------------
# Tote frame: origin at the centre of the underside, x across the tote (150 mm), y along it (200 mm), z up.
OUT = (0.150, 0.200)  # outside at the rim
H = 0.075  # height
T = 0.004  # wall and floor thickness
TAPER = math.radians(6.0)  # each wall leans out 6 degrees from vertical
TAN = math.tan(TAPER)
BAND = 0.020  # vertical rim lip round the top 20 mm of each wall
LIP = 0.012  # the rim lip's thickness: from the wall's inner face out to 8 mm beyond the wall
TIER = 0.033  # the walls' lower tier ends here (the recesses' height)
XB, YB = OUT[0] / 2 - H * TAN, OUT[1] / 2 - H * TAN  # outside half sizes at the bottom: 67.1 x 92.1 mm
RIB_Y = (0.060, -0.025)  # lug and recess centres along each long wall, from the tote centre (not symmetric)
LUG = (0.014, 0.010)  # lug width along the wall, height
LUG_TOP = 0.065  # lug tops 10 mm below the rim
LUG_FACE = OUT[0] / 2 - T - 0.0105  # lug inner faces 10.5 mm in from the lip (60.5 mm from the centre line)
RECESS = (0.030, 0.012)  # recess width along the wall (outside), depth into the tote
MASS = 0.15  # kg, a grey tote


def _xo(z: float) -> float:
    """Outside half width across the tote at height z (tapered part of the wall)."""
    return XB + z * TAN


def _yo(z: float) -> float:
    return YB + z * TAN


def _lean(axis, sign):
    a = sign * TAPER
    return (*(np.asarray(axis, float) * math.sin(a / 2)), math.cos(a / 2))


def _long_wall(sx: int, z0: float, z1: float, y0: float, y1: float, inset: float, color):
    """A piece of a long (+-x) wall between heights z0 and z1 and along y from y0 to y1, its outer face `inset`
    inside the tote's outer face, leaning with the taper."""
    zc = (z0 + z1) / 2
    x = sx * (_xo(zc) - inset - T / 2 / math.cos(TAPER))
    return parts.box((T, y1 - y0, (z1 - z0) / math.cos(TAPER)), at=(x, (y0 + y1) / 2, zc), color=color,
                     quat=_lean((0, 1, 0), sx))


def tote_shapes(body):
    """Every shape of one tote, in the tote frame."""
    s = []
    cuts = sorted((y - RECESS[0] / 2, y + RECESS[0] / 2) for y in RIB_Y)
    # Floor: a middle strip clear of the recesses, and edge strips between them.
    s.append(parts.box((2 * (XB - RECESS[1]), 2 * YB, T), at=(0.0, 0.0, T / 2), color=body))
    spans, y0 = [], -YB
    for a, b in cuts:
        spans.append((y0, a))
        y0 = b
    spans.append((y0, YB))
    for sx in (1, -1):
        for a, b in spans:
            s.append(parts.box((RECESS[1], b - a, T), at=(sx * (XB - RECESS[1] / 2), (a + b) / 2, T / 2), color=body))
    for sx in (1, -1):
        # Long walls. Lower tier: plain between the recesses; each recess is an inset plate, two side plates and a
        # top plate (a moulded pocket open to the outside and the bottom).
        yl = _yo(0.0)  # the lower tier spans the bottom length, so no corner sticks out below
        for a, b in spans:
            s.append(_long_wall(sx, 0.0, TIER, max(a, -yl), min(b, yl), 0.0, body))
        for a, b in cuts:
            s.append(_long_wall(sx, 0.0, TIER, a, b, RECESS[1], RECESS_C))
            for e in (a + T / 2, b - T / 2):
                s.append(parts.box((RECESS[1] + T, T, TIER), at=(sx * (XB - (RECESS[1] + T) / 2), e, TIER / 2),
                                   color=RECESS_C))
            s.append(parts.box((RECESS[1] + T, b - a, T), at=(sx * (_xo(TIER) - (RECESS[1] + T) / 2), (a + b) / 2,
                                                              TIER - T / 2), color=RECESS_C))
        yu = _yo(TIER)
        s.append(_long_wall(sx, TIER, H - BAND, -yu, yu, 0.0, body))
        s.append(parts.box((LIP, OUT[1] - 2 * T + 2 * LIP, BAND), at=(sx * (OUT[0] / 2 - T + LIP / 2), 0.0, H - BAND / 2),
                           color=body))
        # Lugs on the lip's inner face, standing 10.5 mm out from it.
        x_wall = OUT[0] / 2 - T + 0.001  # 1 mm into the rim lip
        for y in RIB_Y:
            w = x_wall - LUG_FACE
            s.append(parts.box((w, LUG[0], LUG[1]), at=(sx * (LUG_FACE + w / 2), y, LUG_TOP - LUG[1] / 2), color=LUG_C))
    for sy in (1, -1):  # end walls, between the long walls
        for z0, z1 in ((0.0, TIER), (TIER, H - BAND)):
            zc = (z0 + z1) / 2
            y = sy * (_yo(zc) - T / 2 / math.cos(TAPER))
            s.append(parts.box((2 * (_xo(z0) - T), T, (z1 - z0) / math.cos(TAPER)), at=(0.0, y, zc), color=body,
                               quat=_lean((1, 0, 0), -sy)))
        s.append(parts.box((OUT[0] - 2 * T, LIP, BAND), at=(0.0, sy * (OUT[1] / 2 - T + LIP / 2), H - BAND / 2),
                           color=body))
    return s


CONTACT = 0.002  # collider contact offset of the totes: under the 2.6 mm play between nested walls

BLUE_XY = {"left": (0.380, 0.150), "right": (0.380, -0.200)}  # the right lane sits further out: see _plan
BLUE = {k: parts.Part(f"blue_{k}", pos=(*xy, TOP), shapes=tote_shapes(BLUE_C), contact_offset=CONTACT)
        for k, xy in BLUE_XY.items()}

# Carton of parts on the floor of the full blue tote: top 54 mm above the table, 1 mm under the lugs. As declared
# (lot A) it is in the left blue tote; lot B puts it in the right one.
CARTON_SIZE = (0.090, 0.150, 0.050)
CARTON_Z = TOP + T + CARTON_SIZE[2] / 2 + 0.0005
CARTON = Box("carton", pos=(*BLUE_XY["left"], CARTON_Z), size=CARTON_SIZE, mass=0.4, color=CARTON_C)

# Grey (empty) totes, both turned the same way as the blue totes (yaw 0).
GREY_START = {"left": (0.180, 0.150), "right": (0.180, -0.200)}
GREY = {k: parts.Part(f"grey_{k}", pos=(*xy, TOP + 0.0005), mass=MASS, shapes=tote_shapes(GREY_C),
                      contact_offset=CONTACT) for k, xy in GREY_START.items()}

OBJECTS = [*BLUE.values(), CARTON, *GREY.values()]

# Layout lots: which blue tote holds the carton. Trial k uses lot k mod 2 (run with --epochs 2). Lot A's
# zero-distance move makes it settle and take a fresh first frame like lot B.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", full="left", place={"carton": CARTON.pos}),
    lots.Lot("B", full="right", place={"carton": (*BLUE_XY["right"], CARTON_Z)}),
)

# -- scoring --------------------------------------------------------------------------------------------------------
STACK_RIM = LUG_TOP + H  # 0.140: rim of a tote resting on the lugs
STACK_TOL = 0.0015
NEST_RIM = T / math.sin(TAPER) + H  # 0.11327: rim of a tote nested in an empty one (walls touching)
NEST_BAND = (-0.004, 0.008)  # rim corners from 4 mm below to 8 mm above that: a tote can wedge a little high in the taper
LEVEL = math.radians(3.0)  # upright: its axis within 3 degrees of vertical
XY_TOL = 0.005  # centre within 5 mm of the blue tote's centre
CARTON_TOL = 0.005
PRESS_Z = LUG_TOP - 0.002  # underside middle below 63 mm over the full tote: gone down past its lugs
RESET_TOL, RESET_JUMP = 0.0005, 0.020
RIM = (OUT[0] - 2 * T + 2 * LIP, OUT[1] - 2 * T + 2 * LIP)  # 166 x 216 mm over the rim lip
RIM_CORNERS = np.array([[sx * RIM[0] / 2, sy * RIM[1] / 2, H] for sx in (1, -1) for sy in (1, -1)])


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rim_heights(st) -> np.ndarray:
    """Heights above the table of a tote's four outer rim corners, from its pose and true dimensions."""
    return (np.asarray(st.pos, float)[:, None] + _rot(st.quat_xyzw) @ RIM_CORNERS.T)[2] - TOP


def full_side() -> str:
    """The blue tote holding the carton in this trial's lot."""
    return lots.current().full


def empty_side() -> str:
    return "right" if full_side() == "left" else "left"


def _seated(st, side: str, rim: float, band) -> bool:
    d = np.asarray(st.pos[:2], float) - BLUE_XY[side]
    h = rim_heights(st) - rim
    return bool(
        _rot(st.quat_xyzw)[2, 2] >= math.cos(LEVEL) - 1e-9
        and np.all(h >= band[0] - 1e-6) and np.all(h <= band[1] + 1e-6)
        and np.all(np.abs(d) <= XY_TOL + 1e-6)
        and st.resting()
    )


def stacked(st) -> bool:
    """On the full tote's lugs: upright, every rim corner 140 +/- 1.5 mm above the table, centred, still."""
    return _seated(st, full_side(), STACK_RIM, (-STACK_TOL, STACK_TOL))


def nested(st) -> bool:
    """Nested in the empty tote: upright, every rim corner 109.27 to 121.27 mm above the table, centred, still."""
    return _seated(st, empty_side(), NEST_RIM, NEST_BAND)


def _over_full(st) -> bool:
    d = np.asarray(st.pos[:2], float) - BLUE_XY[full_side()]
    return bool(abs(d[0]) < OUT[0] / 2 - T and abs(d[1]) < OUT[1] / 2 - T)


def _carton_ok(objects) -> bool:
    start = np.asarray(lots.current().pos("carton"), float)
    return bool(np.linalg.norm(np.asarray(objects["carton"].pos) - start) <= CARTON_TOL + 1e-6)


# Let go: in the simulator an on_step hook measures, every control step, whether every finger and gripper of both
# arms is more than 5 mm from each grey tote. Until it has measured (and offline, where there is no robot) the value
# is None, which gives no credit.
CLEAR = 0.005
_grip = parts.__dict__.setdefault("_stack_nest_totes_grip", {"left": None, "right": None})

# Trial history for the full tote, checked at every control step: whether a grey tote ever went down past its lugs,
# and whether its carton ever moved more than 5 mm. Cleared only at a new trial: every loose object back within
# 0.5 mm of its start in this trial's lot and at rest, having got there by a jump of more than 20 mm in one step (a
# scene reset, not reachable by the robot).
_FREE = [o.name for o in OBJECTS if o.free]
_HISTORY = {"pressed": False, "moved": False, "prev": None}


def _update(objects) -> None:
    lot = lots.current()
    now = {n: np.asarray(objects[n].pos, float) for n in _FREE}
    home = all(np.linalg.norm(now[n] - np.asarray(lot.pos(n), float)) < RESET_TOL and objects[n].resting()
               for n in _FREE)
    prev = _HISTORY["prev"]
    jumped = prev is None or any(np.linalg.norm(now[n] - prev[n]) > RESET_JUMP for n in _FREE)
    if home and jumped:
        _HISTORY["pressed"] = _HISTORY["moved"] = False
    _HISTORY["prev"] = now
    if not _carton_ok(objects):
        _HISTORY["moved"] = True
    for g in GREY:
        st = objects[f"grey_{g}"]
        if _over_full(st) and st.pos[2] - TOP < PRESS_Z - 1e-6:  # the middle of its underside
            _HISTORY["pressed"] = True


def full_ok(objects) -> bool:
    """The full tote qualifies: a let-go grey tote is stacked on its lugs, and during the trial no grey tote went
    down past its lugs and its carton never moved more than 5 mm."""
    if _HISTORY["pressed"] or _HISTORY["moved"] or not _carton_ok(objects):
        return False
    return any(_grip[g] is True and stacked(objects[f"grey_{g}"]) for g in GREY)


def empty_ok(objects) -> bool:
    """The empty tote qualifies: a let-go grey tote is nested in it."""
    return any(_grip[g] is True and nested(objects[f"grey_{g}"]) for g in GREY)


def score(objects):
    """0.5 for the full tote and 0.5 for the empty tote, each scored on its own."""
    _update(objects)
    return 0.5 * full_ok(objects) + 0.5 * empty_ok(objects)


def success(objects):
    return score(objects) == 1.0


def _grey_boxes(st, name):
    return [(c, r, s) for _, c, r, s in GREY[name].world_boxes({"base": (np.asarray(st.pos), _rot(st.quat_xyzw))})]


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    states = sim.states()
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
    for g in GREY:
        boxes = _grey_boxes(states[f"grey_{g}"], g)
        _grip[g] = bool(hulls) and all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in boxes)


@parts.on_reset
def _reset_grip(sim) -> None:
    _grip["left"] = _grip["right"] = None


# -- scripted demo: open loop from the published start layout -------------------------------------------------------
# The demo looks at where the carton starts (the top camera shows it in the first frame) and takes the blue tote it
# is in as the full one; it never reads the lot's name or role. Each arm pinches the rim lip of one long wall of its
# grey tote, the fingers closing across the wall. The arm whose grey tote goes onto the full tote turns it end for end
# in the air, in four eighth turns about the tote's centre, while carrying it across; the other carries its tote into
# the empty tote as it stands. At the end both arms park out to the sides, clear of the totes.
BAND_X = OUT[0] / 2 - T + LIP / 2  # 0.077: the rim lip's middle from the tote's centre line
GRIP_Y = 0.015  # along the wall, between the lugs
GRIP_BELOW_RIM = 0.010  # TCP 10 mm below the rim: the pads cover the 20 mm rim lip
CARRY_BOTTOM = H + 0.012  # underside 12 mm above the blue rims while carried
STACK_DROP = LUG_TOP + 0.005  # a stacking tote is let go with its underside 5 mm above the lugs
NEST_DROP = NEST_RIM - H + 0.006  # a nesting tote is let go 6 mm above where it nests
OPEN_W = -0.020  # fingers 40 mm apart: room round the 12 mm rim lip
GRIP_W = FINGER_CLOSED  # sent fully closed: the 12 mm lip holds the pads 6 mm short, about 30 N of pinch
# Tracking error while carrying (x, y, z in metres), cancelled by aiming off, and the few degrees the tote turns in
# the fingers as they close, cancelled by turning the wrist back once the tote is held. Fixed numbers, calibrated
# once from GPU traces for each arm and move (turned or not).
SAG = {("left", True): (0.0081, -0.0008, 0.004), ("right", False): (0.0125, 0.0018, 0.004),
       ("right", True): (0.0114, -0.0006, 0.004), ("left", False): (0.0108, -0.0019, 0.004)}
YAW_TRIM = {("left", True): math.radians(-2.2), ("right", False): math.radians(-4.0),
            ("right", True): math.radians(-4.0), ("left", False): math.radians(-3.2)}
PARK = {"left": (0.12, 0.42, TOP + 0.20), "right": (0.12, -0.42, TOP + 0.20)}


def _tcp(c, psi, gx, gy, bottom, sag, aim=1.0):
    """TCP for a tote centred at c and turned psi, gripped at (gx, gy) in the tote frame; `aim` scales the sideways
    aim-off (0 where the tote was picked up, 1 over the blue tote)."""
    r = np.array([[math.cos(psi), -math.sin(psi)], [math.sin(psi), math.cos(psi)]])
    p = np.asarray(c, float) + r @ (gx, gy)
    sx, sy, sz = sag
    return (float(p[0] + aim * sx), float(p[1] + aim * sy), float(TOP + bottom + H - GRIP_BELOW_RIM + sz))


def _plan(side: str, turn: bool):
    """One arm's plan. Turning, it grips the far (+x) wall and turns the tote half a turn (the left arm from yaw -90
    to +90 degrees, the right from +90 to -90, each the way its wrist reaches); not turning, it grips the near wall.
    A half turn plus the few degrees of trim uses nearly all of the wrist's +-120 degrees; the right lane is set 200 mm
    out from the midline so that the right wrist's turn ends inside its range."""
    sgn = 1.0 if side == "left" else -1.0
    gx, yaw0 = (BAND_X, -sgn * math.pi / 2) if turn else (-BAND_X, -sgn * math.pi / 2)
    start, goal = GREY_START[side], BLUE_XY[side]
    sag, drop = SAG[(side, turn)], STACK_DROP if turn else NEST_DROP
    g = (gx, sgn * GRIP_Y)
    at = _tcp(start, 0.0, *g, 0.0, sag, aim=0.0)
    grip = (at[0], at[1], at[2] - sag[2])  # empty-handed on the table nothing sags
    w = [
        Waypoint(1.5, (grip[0], grip[1], grip[2] + 0.05), OPEN_W, "over the rim lip", yaw0),
        Waypoint(1.2, grip, OPEN_W, "descend", yaw0),
        Waypoint(0.8, grip, GRIP_W, "pinch", yaw0),
        Waypoint(1.5, _tcp(start, 0.0, *g, CARRY_BOTTOM, sag, aim=0.0), GRIP_W, "lift", yaw0),
    ]
    psi = sgn * math.pi if turn else 0.0
    if turn:
        for k in range(1, 5):
            s = k / 4
            c = (start[0] + s * (goal[0] - start[0]), start[1] + s * (goal[1] - start[1]))
            w.append(Waypoint(1.0, _tcp(c, s * psi, *g, CARRY_BOTTOM, sag, aim=s), GRIP_W, f"turn {45 * k} deg",
                              yaw0 + s * psi))
    else:
        w.append(Waypoint(2.0, _tcp(goal, 0.0, *g, CARRY_BOTTOM, sag), GRIP_W, "carry", yaw0))
    y1 = yaw0 + psi
    w += [
        Waypoint(0.8, _tcp(goal, psi, *g, CARRY_BOTTOM, sag), GRIP_W, "settle", y1),
        Waypoint(1.5, _tcp(goal, psi, *g, drop, sag), GRIP_W, "lower into the mouth", y1),
        Waypoint(0.8, _tcp(goal, psi, *g, drop, sag), GRIP_W, "steady", y1),
        Waypoint(0.6, _tcp(goal, psi, *g, drop, sag), OPEN_W, "release", y1),
        Waypoint(1.2, _tcp(goal, psi, *g, CARRY_BOTTOM + 0.03, sag), OPEN_W, "up", y1),
    ]
    trim = YAW_TRIM[(side, turn)]
    w = w[:3] + [Waypoint(p.duration, p.pos, p.finger, p.label, p.yaw + trim) for p in w[3:]]
    w += [Waypoint(2.0, PARK[side], OPEN_W, "park clear", 0.0), Waypoint(1.5, PARK[side], OPEN_W, "wait", 0.0)]
    return w


def demo():
    carton = np.asarray(lots.current().pos("carton")[:2], float)  # where the carton starts, as the camera shows it
    full = min(BLUE_XY, key=lambda k: float(np.linalg.norm(carton - BLUE_XY[k])))
    return {side: _plan(side, side == full) for side in ("left", "right")}
