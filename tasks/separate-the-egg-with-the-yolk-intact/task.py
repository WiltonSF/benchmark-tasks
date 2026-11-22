"""On a kitchen counter in front of a tiled backsplash: a raw egg has been cracked into a clear 12 cm glass bowl, and
its round yellow yolk sits whole in the clear white in the middle of the bowl. Next to it is an empty, smaller clear
glass bowl. An orange silicone squeeze egg separator, a hollow fish-shaped bulb with its round mouth at the bottom,
stands mouth-down on a small white saucer. Lot B is the mirror image of lot A (left and right swapped).
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so the reset hooks run before lots move objects)
import lots  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
DT = 0.05  # control step (s), 20 Hz
SUCCESS_HOLD_S = 1.0
HOLD_STEPS = 20  # 1 s of the scored end state, at 20 Hz

GLASS = (0.78, 0.86, 0.90)
WHITE = (0.92, 0.91, 0.78)  # raw egg white (clear, faintly yellow)
YOLK_YELLOW = (0.98, 0.60, 0.04)
ORANGE = (0.96, 0.44, 0.10)
LIP_ORANGE = (0.80, 0.26, 0.06)
CHINA = (0.97, 0.97, 0.95)
TILE = (0.86, 0.87, 0.84)


def _qz(a: float):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _disc(r: float, z: float, t: float, color, n: int = 12):
    """A flat round plate of radius r and thickness t (top at z + t), as n radial boxes (no creeping cylinder)."""
    w = 2 * r * math.tan(math.pi / n) + 0.001
    return [parts.box((r, w, t), at=(r / 2 * math.cos(2 * math.pi * k / n), r / 2 * math.sin(2 * math.pi * k / n),
                                     z + t / 2), color=color, quat=_qz(2 * math.pi * k / n)) for k in range(n)]


def _fan(name, r, z, color, n=24, cx=0.0, cy=0.0, visible=True):
    pts = [(cx, cy, z)] + [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n), z)
                           for k in range(n)]
    tris = [(0, 1 + k, 1 + (k + 1) % n) for k in range(n)]
    return parts.Visual(name, tuple(pts), tuple(tris), color=color, visible=visible)


# -- the two glass bowls -----------------------------------------------------------------------------------------
# Duralex Lys stacking bowls (tempered glass): 12 cm (120 mm across, 49 mm tall, 310 mL) for the egg and its white,
# 10.5 cm (105 mm, 40 mm tall, 205 mL) for the yolk. Retail catalogue sizes; the floor radius, 3 mm wall and 4 mm
# floor are estimates from the shape. Part frame: origin on the counter under the bowl's centre.
class Bowl:
    def __init__(self, r_floor, r_rim, height, mass):
        self.r_floor, self.r_rim, self.height, self.mass = r_floor, r_rim, height, mass
        self.floor_t, self.glass = 0.004, 0.003
        self.flare = math.atan((r_rim - r_floor) / (height - self.floor_t))  # wall angle from upright

    def inner_r(self, z):
        """Inner radius at height z above the counter."""
        return self.r_floor + max(0.0, z - self.floor_t) * math.tan(self.flare)

    def shapes(self, n=16):
        out = _disc(self.r_floor + self.glass, 0.0, self.floor_t, GLASS)
        a = self.flare
        slant = (self.height - self.floor_t) / math.cos(a)
        rm, zm = (self.r_floor + self.r_rim) / 2, (self.floor_t + self.height) / 2
        for k in range(n):
            phi = 2 * math.pi * k / n
            nh = np.array([math.cos(phi), math.sin(phi), 0.0])
            t = np.array([-math.sin(phi), math.cos(phi), 0.0])
            s = math.sin(a) * nh + math.cos(a) * np.array([0, 0, 1.0])
            out_n = math.cos(a) * nh - math.sin(a) * np.array([0, 0, 1.0])
            c = rm * nh + np.array([0, 0, zm]) + out_n * self.glass / 2
            rot = np.column_stack([out_n, t, s])
            w = 2 * rm * math.tan(math.pi / n) + 0.004
            out.append(parts.box((self.glass, w, slant + 0.002), at=tuple(c), color=GLASS,
                                 quat=tuple(parts._matrix_to_quat(rot))))
        return out


WB = Bowl(0.028, 0.057, 0.049, 0.230)  # egg (white) bowl, Lys 12 cm; mass an estimate
YB = Bowl(0.025, 0.0495, 0.040, 0.160)  # yolk bowl, Lys 10.5 cm; mass an estimate

# -- the yolk ------------------------------------------------------------------------------------------------------
# Large egg yolk, about 17 g: a 36 x 36 x 24 mm flattened dome (16.3 mL), rigid. Its state (whole or burst) is a
# declared rule, see README.md.
YOLK_R = np.array([0.018, 0.018, 0.012])
V_YOLK = 4.0 / 3.0 * math.pi * float(np.prod(YOLK_R)) * 1e6  # mL
YOLK_MASS = 0.017

# -- the white -----------------------------------------------------------------------------------------------------
WHITE0 = 31.5  # mL: the white of one large egg (about 33 g)
CLING = 1.0  # mL of thick white that leaves with the yolk whenever the yolk is drawn up (estimate)


def _bowl_volume(b: Bowl, h: float) -> float:
    """mL of the bowl's inside from its floor up to h above the floor (a frustum)."""
    r0, r1 = b.r_floor, b.r_floor + h * math.tan(b.flare)
    return math.pi * h * (r0 * r0 + r0 * r1 + r1 * r1) / 3 * 1e6


def _yolk_below(h: float) -> float:
    """mL of a yolk lying on the floor that is below h above the floor."""
    c = 2 * YOLK_R[2]
    h = min(max(h, 0.0), c)
    a = YOLK_R[0] * YOLK_R[1] / YOLK_R[2] ** 2
    z = h - YOLK_R[2]  # from the yolk's centre
    v = math.pi * a * ((YOLK_R[2] ** 2 * z - z ** 3 / 3) - (YOLK_R[2] ** 2 * -YOLK_R[2] + YOLK_R[2] ** 3 / 3))
    return v * 1e6


def white_level(ml: float, yolk_in: bool) -> float:
    """Height of the white's surface above the egg bowl's floor (m)."""
    lo, hi = 0.0, WB.height - WB.floor_t
    for _ in range(40):
        mid = (lo + hi) / 2
        if _bowl_volume(WB, mid) - (_yolk_below(mid) if yolk_in else 0.0) < ml:
            lo = mid
        else:
            hi = mid
    return lo


# -- the squeeze separator -------------------------------------------------------------------------------------------
# Peleg Design YolkFish (food-grade silicone, about 100 x 40 x 70 mm, about 38 g): an orange fish-shaped bulb, mouth
# down. Part frame: origin at the centre of the mouth's underside, z up through the body, the squeeze across y.
# Estimated geometry: a 6 mm lip ring 32 mm across with a 22 mm opening, a head, a body and a tail fin. Each 4 mm
# side wall (56 x 52 mm) is a sprung slider that the fingers press in; pressing both in by 10 mm (the stop) pushes
# 30 mL out of the bulb.
LIP_IN, LIP_OUT, LIP_H = 0.011, 0.016, 0.006
A_OPEN = math.pi * LIP_IN ** 2
WALL_Z = 0.059  # wall centre above the mouth; the grip height
WALL_HALF_Y = 0.020  # the walls' outer faces at rest
TRAVEL = 0.010
WALL_SPRING = 400.0  # N/m per wall (estimate for a 1.5-2 mm silicone bulb wall)
ML_PER_M = 1500.0  # mL pushed out per metre of wall travel, per wall (15 mL per 10 mm)
SEP_BOX = (np.array([0.0, 0.0, 0.0505]), np.array([0.032, 0.020, 0.0505]))  # centre, half sizes (part frame)


def _lips():
    rm, n = (LIP_IN + LIP_OUT) / 2, 12
    w = 2 * rm * math.tan(math.pi / n) + 0.0015
    return [parts.box((LIP_OUT - LIP_IN, w, LIP_H), at=(rm * math.cos(2 * math.pi * k / n),
                                                         rm * math.sin(2 * math.pi * k / n), LIP_H / 2),
                      color=LIP_ORANGE, quat=_qz(2 * math.pi * k / n)) for k in range(n)]


def _eye(name, side):
    """A 7 mm black eye on the head's flat side (visual only)."""
    y, n = side * 0.0172, 12
    pts = [(0.010, y, 0.022)] + [(0.010 + 0.0035 * math.cos(2 * math.pi * k / n), y, 0.022 + 0.0035 * math.sin(2 * math.pi * k / n))
                                 for k in range(n)]
    return parts.Visual(name, tuple(pts), tuple((0, 1 + k, 1 + (k + 1) % n) for k in range(n)), color=(0.05, 0.05, 0.05))


SEP_SHAPES = _lips() + [
    parts.box((0.040, 0.034, 0.022), at=(0.0, 0.0, 0.020), color=ORANGE),  # head (hollow above the lips)
    parts.box((0.060, 0.022, 0.056), at=(0.0, 0.0, 0.059), color=ORANGE),  # body
    parts.box((0.048, 0.006, 0.014), at=(0.0, 0.0, 0.094), color=ORANGE),  # tail fin
]


def _wall(name, side):
    return parts.Slider(name, shapes=(parts.box((0.056, 0.004, 0.052), at=(0.0, side * (WALL_HALF_Y - 0.002), WALL_Z),
                                                color=ORANGE),),
                        axis=(0.0, -float(side), 0.0), travel=TRAVEL, mass=0.008, spring=WALL_SPRING, damping=2.0,
                        max_effort=50.0)


# -- layout (lot A; lot B is its mirror image) -------------------------------------------------------------------------
WB_XY = (0.355, -0.150)
YB_XY = (0.245, -0.035)
SAUCER_XY = (0.215, -0.255)
SAUCER_R, SAUCER_T = 0.060, 0.006
SEP_START = (SAUCER_XY[0], SAUCER_XY[1], TOP + SAUCER_T + 0.0005)
YOLK_START = (WB_XY[0], WB_XY[1], float(TOP + WB.floor_t + YOLK_R[2] + 0.0005))  # plain floats for Isaac

white_visual = _fan("white", WB.inner_r(WB.floor_t + white_level(WHITE0, True)) - 0.001,
                    WB.floor_t + white_level(WHITE0, True), WHITE, n=32)
carry_visuals = [_fan(f"carry{k}", r, YB.floor_t + 0.0006, WHITE, n=24, visible=False)
                 for k, r in ((1, 0.020), (2, 0.026), (5, 0.032))]

EGG_BOWL = parts.Part("egg_bowl", pos=(WB_XY[0], WB_XY[1], TOP), shapes=WB.shapes(), mass=WB.mass,
                      friction=(0.8, 0.7), visuals=(white_visual,))
YOLK_BOWL = parts.Part("yolk_bowl", pos=(YB_XY[0], YB_XY[1], TOP), shapes=YB.shapes(), mass=YB.mass,
                       friction=(0.8, 0.7), visuals=tuple(carry_visuals))
burst = parts.Visual("burst", tuple([(0.0, 0.0, -0.0105)] + [
    ((0.030 + 0.006 * math.sin(3 * k)) * math.cos(2 * math.pi * k / 20),
     (0.030 + 0.006 * math.sin(3 * k)) * math.sin(2 * math.pi * k / 20), -0.0105) for k in range(20)]),
    tuple((0, 1 + k, 1 + (k + 1) % 20) for k in range(20)), color=YOLK_YELLOW, visible=False)
YOLK = parts.Part("yolk", pos=YOLK_START, shapes=[parts.ellipsoid(tuple(2 * YOLK_R), color=YOLK_YELLOW)],
                  mass=YOLK_MASS, friction=(0.3, 0.2), angular_damping=8.0, contact_offset=0.002, visuals=(burst,),
                  report=("base",))
SEPARATOR = parts.Part("separator", pos=SEP_START, shapes=SEP_SHAPES, mass=0.022, friction=(1.0, 0.9),
                       joints=[_wall("wall_l", 1), _wall("wall_r", -1)], contact_offset=0.002,
                       visuals=(_eye("eye_l", 1), _eye("eye_r", -1)),
                       report=("base", "wall_l", "wall_r"))
SAUCER = parts.Part("saucer", pos=(SAUCER_XY[0], SAUCER_XY[1], TOP), shapes=_disc(SAUCER_R, 0.0, SAUCER_T, CHINA),
                    mass=0.150, friction=(0.8, 0.7))
BACKSPLASH = parts.Part("backsplash", pos=(0.575, 0.0, TOP + 0.15), shapes=[parts.box((0.014, 0.80, 0.30), color=TILE)])

OBJECTS = [EGG_BOWL, YOLK_BOWL, YOLK, SEPARATOR, SAUCER, BACKSPLASH]


def _mirror(p):
    return (float(p[0]), -float(p[1]), float(p[2]))


LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", arm="right", place={"yolk": YOLK_START}),  # a zero move: settles and redraws the first frame
    lots.Lot("B", arm="left", place={n.name: _mirror(n.pos) for n in (EGG_BOWL, YOLK_BOWL, YOLK, SEPARATOR, SAUCER)}),
)

# -- declared rules (README.md, "How the egg is modelled") -------------------------------------------------------------
SEAL_ABOVE = 0.001  # the lips' underside at most 1 mm above the yolk's top (a film of white) ...
SEAL_BELOW = 0.0035  # ... and at most 3.5 mm below it (the yolk's dome is inside the 22 mm opening)
SEAL_OFF = 0.006  # the mouth's centre within 6 mm of the yolk's top, sideways
UPRIGHT = math.cos(math.radians(25))  # mouth pointing down within 25 degrees
EXIT_ML = 8.0  # mL squeezed out after the yolk is drawn up before it leaves the lips
PRESS_N = 0.5  # the separator pressing on the yolk with more than this bursts it (estimate)
SPEED_MAX = 0.8  # m/s: a yolk moving faster (a fall of about 33 mm, or a squirt) bursts when it lands (estimate)
HAND_TOUCH = 0.002  # any robot surface within 2 mm of the yolk is a touch: hard pads or links on a yolk burst it
BOWL_TILT = 15.0  # degrees: a bowl tipped further (or lifted 20 mm) pours raw egg over its rim
BOWL_LIFT = 0.020
CLEAR = 0.030  # hands clear: every gripper point at least 30 mm from the separator, the yolk and both bowls
PARK = (0.40, 0.0, TOP - 0.30)  # where the simulator keeps the yolk body while the yolk is inside the separator

S = parts.__dict__.setdefault("egg_separate_state", {})


def reset_state():
    S.clear()
    S.update(known=False, broken=False, ruptured=False, why="", upset=False, inside=False, white_bowl=WHITE0,
             sep_white=0.0, yolk_white=0.0, spill=0.0, intake=0.0, expel=0.0, prev=None, hold=0, clear=False,
             maxspeed=0.0, maxpress=0.0, views=None, phys=0, release=None, events=[], shown=set(), t=0.0,
             last_out=None, hold_full=0)


if not S:
    reset_state()


def burst(why: str) -> None:
    if not S["ruptured"]:
        S["ruptured"], S["why"] = True, why
        S["events"].append(("burst", round(S["t"], 2), why))


def _R(st):
    return parts._quat_to_matrix(st.quat_xyzw)


def yolk_top(y) -> float:
    r = _R(y)
    return float(y.pos[2] + np.linalg.norm(r[2] * YOLK_R))


def yolk_bottom(y) -> float:
    r = _R(y)
    return float(y.pos[2] - np.linalg.norm(r[2] * YOLK_R))


def upright(sep) -> bool:
    return bool(_R(sep)[2, 2] >= UPRIGHT - EPS)


def on_yolk(sep, y) -> bool:
    """The mouth sits on the yolk: upright, centred within 6 mm, its underside from 3.5 mm below to 1 mm above the
    yolk's top (the dome rises 2.5 mm into the 22 mm opening before the lip ring meets it)."""
    if S["inside"] or not upright(sep):
        return False
    gap = float(sep.pos[2]) - yolk_top(y)
    off = float(np.linalg.norm(np.asarray(sep.pos[:2]) - np.asarray(y.pos[:2])))
    return off <= SEAL_OFF + EPS and -SEAL_BELOW - EPS <= gap <= SEAL_ABOVE + EPS


def white_surface(st) -> float:
    """World height of the white's surface in the egg bowl."""
    b = st["egg_bowl"]
    yolk_in = (not S["inside"]) and float(np.linalg.norm(np.asarray(st["yolk"].pos[:2]) - np.asarray(b.pos[:2]))) < WB.r_floor
    return float(b.pos[2] + WB.floor_t + white_level(max(S["white_bowl"], 0.0), yolk_in))


def submerged(sep, st) -> bool:
    b = st["egg_bowl"]
    d = float(np.linalg.norm(np.asarray(sep.pos[:2]) - np.asarray(b.pos[:2])))
    z = white_surface(st)
    return bool(d < WB.inner_r(z - float(b.pos[2])) - EPS and float(sep.pos[2]) < z - EPS)


def destination(sep, st) -> str:
    """Where material leaving the mouth lands: straight down from the mouth."""
    p = np.asarray(sep.pos, float)
    for key, name, b in (("yolk", "yolk_bowl", YB), ("white", "egg_bowl", WB)):
        c = np.asarray(st[name].pos, float)
        if float(np.linalg.norm(p[:2] - c[:2])) <= b.r_rim - 0.003 + EPS and p[2] >= c[2] + b.floor_t - EPS:
            return key
    return "spill"


def deposit(ml: float, where: str) -> None:
    if ml <= 0:
        return
    key = {"yolk": "yolk_white", "white": "white_bowl", "spill": "spill"}[where]
    S[key] += ml


def floor_below(xy, st) -> float:
    """Top of whatever is straight below xy: a bowl's floor, the saucer, or the counter."""
    xy = np.asarray(xy, float)
    for name, b in (("yolk_bowl", YB), ("egg_bowl", WB)):
        c = np.asarray(st[name].pos, float)
        if float(np.linalg.norm(xy - c[:2])) <= b.inner_r(b.height):
            return float(c[2] + b.floor_t)
    c = np.asarray(st["saucer"].pos, float)
    if float(np.linalg.norm(xy - c[:2])) <= SAUCER_R:
        return float(c[2] + SAUCER_T)
    return TOP


def bulb_step(sep, st) -> dict | None:
    """One control step of the bulb rule. Returns a yolk release {pos, vel} when the yolk leaves the mouth."""
    q = sep.joints
    squeezed = (max(q["wall_l"], 0.0) + max(q["wall_r"], 0.0)) * ML_PER_M
    prev, S["prev"] = S["prev"], squeezed
    if prev is None:
        return None
    dv = squeezed - prev  # > 0: pushed out of the mouth; < 0: drawn in
    y = st["yolk"]
    sealed = on_yolk(sep, y)
    if not sealed:
        S["intake"] = 0.0
    out = None
    if dv < -EPS:
        vol = -dv
        if sealed:
            S["intake"] += vol
            if S["intake"] >= V_YOLK - EPS:
                vol = S["intake"] - V_YOLK
                S.update(inside=True, intake=0.0, expel=0.0)
                take = min(CLING, S["white_bowl"])
                S["white_bowl"] -= take
                S["sep_white"] += take
                S["events"].append(("drawn up", round(S["t"], 2)))
            else:
                vol = 0.0
        if vol > EPS and upright(sep) and submerged(sep, st):
            take = min(vol, max(S["white_bowl"], 0.0))
            S["white_bowl"] -= take
            S["sep_white"] += take
    elif dv > EPS:
        vol = dv
        where = destination(sep, st)
        if S["inside"]:
            S["expel"] += vol
            if S["expel"] >= EXIT_ML - EPS:
                R = _R(sep)
                down = -R[:, 2]
                v_exit = vol * 1e-6 / DT / A_OPEN
                c = np.asarray(sep.pos, float) + down * (YOLK_R[2] + 0.0005)
                floor = floor_below(c[:2], st)
                room = float(sep.pos[2]) - floor
                if room < 2 * YOLK_R[2] - 0.002 - EPS:
                    burst("squeezed out against the surface below the mouth")
                c[2] = max(c[2], floor + YOLK_R[2] + 0.0005)
                vel = np.asarray(sep.lin_vel, float) + down * v_exit
                S.update(inside=False, expel=0.0)
                cling = min(CLING, S["sep_white"])
                S["sep_white"] -= cling
                deposit(cling, where)
                S["last_out"] = where  # where the yolk was last squeezed out: it must reach the small bowl this way
                out = dict(pos=c, vel=vel, v_exit=v_exit)
                S["events"].append(("squeezed out", round(S["t"], 2), where, round(v_exit, 3)))
                vol = 0.0
            else:
                vol = 0.0
        if vol > EPS and S["sep_white"] > EPS:
            gone = min(vol, S["sep_white"])
            S["sep_white"] -= gone
            deposit(gone, where)
    return out


def bowl_upset(st) -> bool:
    for name, start in (("egg_bowl", None), ("yolk_bowl", None)):
        b = st[name]
        if _R(b)[2, 2] < math.cos(math.radians(BOWL_TILT)) - EPS or float(b.pos[2]) > TOP + BOWL_LIFT + EPS:
            return True
    return False


def yolk_in_bowl(st) -> bool:
    y, b = st["yolk"], st["yolk_bowl"]
    d = float(np.linalg.norm(np.asarray(y.pos[:2]) - np.asarray(b.pos[:2])))
    return bool(d <= YB.r_floor + EPS and abs(yolk_bottom(y) - (float(b.pos[2]) + YB.floor_t)) <= 0.004 + EPS
                and y.resting())


def sep_corners(sep) -> np.ndarray:
    c, h = SEP_BOX
    pts = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h + c
    return pts @ _R(sep).T + np.asarray(sep.pos, float)


def _box_surface(step=0.004):
    c, h = SEP_BOX
    axes = [np.linspace(-v, v, max(2, int(np.ceil(2 * v / step)) + 1)) for v in h]
    pts = []
    for k in range(3):
        for sgn in (-1.0, 1.0):
            g = np.meshgrid(*[axes[i] if i != k else np.array([sgn * h[k]]) for i in range(3)], indexing="ij")
            pts.append(np.stack([x.ravel() for x in g], axis=1))
    return np.concatenate(pts) + c


SEP_SURFACE = _box_surface()  # the separator's box (it encloses the walls), part frame


def separator_off_yolk(st) -> bool:
    """Every point of the separator's box at least 5 mm from the yolk's surface (it is not resting on the yolk)."""
    sep = st["separator"]
    pts = SEP_SURFACE @ _R(sep).T + np.asarray(sep.pos, float)
    return yolk_gap(pts, st["yolk"]) >= 0.005 - EPS


def separator_down(st) -> bool:
    """Put down: resting on the counter or the saucer (lowest corner within 5 mm), clear of both bowls' rims."""
    sep = st["separator"]
    pts = sep_corners(sep)
    low = float(pts[:, 2].min())
    sau = np.asarray(st["saucer"].pos, float)
    on_saucer = float(np.linalg.norm(np.asarray(sep.pos[:2]) - sau[:2])) <= SAUCER_R + EPS
    surface = float(sau[2] + SAUCER_T) if on_saucer else TOP
    if abs(low - surface) > 0.005 + EPS or not sep.resting():
        return False
    for name, b in (("yolk_bowl", YB), ("egg_bowl", WB)):
        c = np.asarray(st[name].pos, float)
        if float(np.linalg.norm(pts[:, :2] - c[:2], axis=1).min()) <= b.r_rim + 0.002 + EPS:
            return False
    return True


def end_state(st) -> bool:
    """The state every scored level needs: the whole yolk lying still in the small bowl, put there by squeezing it out
    of the separator over that bowl, out of the separator, and no part of the separator within 5 mm of it."""
    if S["inside"] or S["last_out"] != "yolk" or not yolk_in_bowl(st):
        return False
    return separator_off_yolk(st)


def score(st) -> float:
    if not S["known"] or S["broken"] or S["ruptured"] or S["upset"]:
        return 0.0
    if not end_state(st) or S["hold"] < HOLD_STEPS:
        return 0.0
    spill = S["spill"] + S["sep_white"]  # white still in the separator drips out of it when it is put down
    carry = S["yolk_white"]
    if spill > 5.0 + EPS or carry > 10.0 + EPS:
        return 0.0
    if carry <= 2.0 + EPS and spill <= 1.0 + EPS:
        full = S["clear"] and separator_down(st) and S["hold_full"] >= HOLD_STEPS
        return 1.0 if full else 0.75
    if carry <= 5.0 + EPS and spill <= 2.0 + EPS:
        return 0.5
    return 0.25


def success(st) -> bool:
    return score(st) == 1.0


# -- simulator hooks -----------------------------------------------------------------------------------------------------
def _arr(x):
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy())
    return np.asarray(x)


HAND = ("gripper", "tip_left", "tip_right")
_CACHE = {}


def hand_points(base):
    """Gripper and finger surfaces, sampled every 3 mm over each hull face (cached per link)."""
    from scipy.spatial import ConvexHull
    from yam_arena import analysis

    origin = parts._np(base.scene.env_origins[0])
    out = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, name in enumerate(asset.body_names):
            if name not in HAND:
                continue
            if name not in _CACHE:
                vv = analysis.world_hulls({name: np.eye(4)})
                if not vv:
                    raise RuntimeError("missing hand hull " + name)
                v = np.concatenate([h.vertices for _, h in vv])
                samples = [v]
                for tri in ConvexHull(v).simplices:
                    a, b, c = v[tri]
                    n = max(1, int(np.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / .003)))
                    samples.append(np.array([a + (b - a) * u / n + (c - a) * w / n for u in range(n + 1) for w in range(n + 1 - u)]))
                _CACHE[name] = np.unique(np.round(np.concatenate(samples) / .0025).astype(np.int32), axis=0) * .0025
            out.append(_CACHE[name] @ parts._quat_to_matrix(poses[i, 3:]).T + poses[i, :3] - origin)
    if len(out) != 6:
        raise RuntimeError("unknown hand state")
    return np.concatenate(out)


_ALL = {}


def robot_points(base):
    """Every link of both arms, sampled every 4 mm over each hull face (cached): for the touch rule."""
    from scipy.spatial import ConvexHull
    from yam_arena import analysis

    origin = parts._np(base.scene.env_origins[0])
    out = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, name in enumerate(asset.body_names):
            if name not in _ALL:
                vv = analysis.world_hulls({name: np.eye(4)})
                if not vv:
                    _ALL[name] = None
                    continue
                samples = []
                for _, h in vv:
                    v = h.vertices
                    samples.append(v)
                    for tri in ConvexHull(v).simplices:
                        a, b, c = v[tri]
                        n = max(1, int(np.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / .004)))
                        samples.append(np.array([a + (b - a) * u / n + (c - a) * w / n for u in range(n + 1) for w in range(n + 1 - u)]))
                _ALL[name] = np.unique(np.round(np.concatenate(samples) / .002).astype(np.int32), axis=0) * .002
            if _ALL[name] is not None:
                out.append(_ALL[name] @ parts._quat_to_matrix(poses[i, 3:]).T + poses[i, :3] - origin)
    if not out:
        raise RuntimeError("unknown robot state")
    return np.concatenate(out)


def yolk_gap(points, y) -> float:
    """Lower bound on the distance from points to the yolk's surface (scaled radial distance)."""
    local = (points - np.asarray(y.pos, float)) @ _R(y)
    return float(((np.linalg.norm(local / YOLK_R, axis=1) - 1.0) * YOLK_R.min()).min())


def hands_clear(points, st) -> bool:
    sep = st["separator"]
    c, h = SEP_BOX
    local = (points - np.asarray(sep.pos, float)) @ _R(sep) - c
    if float(np.linalg.norm(np.maximum(np.abs(local) - h, 0.0), axis=1).min()) < CLEAR - EPS:
        return False
    if not S["inside"] and yolk_gap(points, st["yolk"]) < CLEAR - EPS:
        return False
    for name, b in (("yolk_bowl", YB), ("egg_bowl", WB)):
        p = np.asarray(st[name].pos, float)
        d = np.linalg.norm(points[:, :2] - p[:2], axis=1)
        near = (d < b.r_rim + CLEAR) & (points[:, 2] < p[2] + b.height + CLEAR)
        if near.any():
            return False
    return True


def _write_yolk(base, pos, vel=(0.0, 0.0, 0.0)):
    import torch

    obj = base.scene["yolk"]
    pose = torch.tensor([[float(pos[0]), float(pos[1]), float(pos[2]), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                        device=base.device)
    pose[:, :3] += base.scene.env_origins[0][:3].to(pose.dtype)
    obj.write_root_pose_to_sim_index(root_pose=pose)
    v = torch.tensor([[float(vel[0]), float(vel[1]), float(vel[2]), 0.0, 0.0, 0.0]], dtype=torch.float32,
                     device=base.device)
    obj.write_root_velocity_to_sim_index(root_velocity=v)


def _press_view():
    from isaaclab_physx.physics import PhysxManager

    sv = PhysxManager.get_physics_sim_view()
    root = parts._PATHS["separator"]
    return sv.create_rigid_contact_view(parts._PATHS["yolk"], filter_patterns=[f"{root}/base", f"{root}/wall_l",
                                                                               f"{root}/wall_r"],
                                        max_contact_data_count=64)


@parts.on_physics_step
def monitor_physics(sim):
    """Every 240 Hz substep: the separator's push on the yolk and the yolk's speed (while it is in a bowl)."""
    try:
        base = sim.env.unwrapped
        S["phys"] += 1
        if S["inside"]:
            return
        if S["views"] is None:
            S["views"] = _press_view()
        fm = _arr(S["views"].get_contact_force_matrix(dt=base.physics_dt)).reshape(-1, 3)
        if not np.isfinite(fm).all():
            raise RuntimeError("non-finite yolk contact force")
        press = float(np.linalg.norm(fm.sum(0)))
        S["maxpress"] = max(S["maxpress"], press)
        if press > PRESS_N + EPS:
            burst(f"pressed by the separator at {press:.2f} N")
        y = parts._read(base, YOLK)
        speed = float(np.linalg.norm(y.lin_vel))
        if not np.isfinite(speed):
            raise RuntimeError("non-finite yolk speed")
        if S["release"] is None or S["phys"] > S["release"]:
            S["maxspeed"] = max(S["maxspeed"], speed)
            if speed > SPEED_MAX + EPS:
                burst(f"moving at {speed:.2f} m/s")
    except Exception as exc:  # fail closed
        if not S["broken"]:
            import traceback

            print("[egg] PHYSICS MONITOR FAILED " + repr(exc) + "\n" + traceback.format_exc(), flush=True)
        S["broken"] = True


@parts.on_reset
def on_reset(sim):
    shown = set(S.get("shown", set()))
    views = S.get("views")
    reset_state()
    S["views"] = views
    for part, vis in shown:
        sim.set_visible(part, vis, False)


@parts.on_step
def on_step(sim):
    try:
        base = sim.env.unwrapped
        S["t"] = float(sim.t)
        st = sim.states()
        sep, y = st["separator"], st["yolk"]
        rel = bulb_step(sep, st)
        if rel is not None:
            _write_yolk(base, rel["pos"], rel["vel"])
            S["release"] = S["phys"] + 1  # the speed rule starts with the next substep, at the exit speed
            if rel["v_exit"] > SPEED_MAX + EPS:
                burst(f"squirted out at {rel['v_exit']:.2f} m/s")
        elif S["inside"]:
            _write_yolk(base, PARK)
        if S["upset"] is False and bowl_upset(st):
            S["upset"] = True
            S["events"].append(("bowl tipped or lifted", round(S["t"], 2)))
        pts = hand_points(base)
        if not S["inside"] and yolk_gap(robot_points(base), y) <= HAND_TOUCH + EPS:
            burst("touched by the robot")
        S["clear"] = hands_clear(pts, st)
        S["known"] = True
        S["hold"] = S["hold"] + 1 if end_state(st) else 0
        S["hold_full"] = S["hold_full"] + 1 if (S["hold"] > 0 and S["clear"] and separator_down(st)) else 0
        want = set()
        if S["ruptured"]:
            want.add(("yolk", "burst"))
        for k, ml in ((1, 0.5), (2, 2.0), (5, 4.0)):
            if S["yolk_white"] >= ml - EPS:
                want.add(("yolk_bowl", f"carry{k}"))
        for part, vis in want - S["shown"]:
            sim.set_visible(part, vis, True)
        S["shown"] |= want
        if int(round(sim.t / DT)) % 10 == 0:
            print("[egg] " + json.dumps(dict(
                t=round(sim.t, 2), score=score(st), broken=S["broken"], inside=S["inside"], ruptured=S["ruptured"], why=S["why"], upset=S["upset"],
                walls=[round(sep.joints["wall_l"], 4), round(sep.joints["wall_r"], 4)], intake=round(S["intake"], 2),
                white_bowl=round(S["white_bowl"], 2), sep_white=round(S["sep_white"], 2),
                yolk_white=round(S["yolk_white"], 2), spill=round(S["spill"], 2), maxpress=round(S["maxpress"], 3),
                maxspeed=round(S["maxspeed"], 3), clear=S["clear"], hold=S["hold"],
                mouth=[round(float(v), 4) for v in sep.pos], yolk=[round(float(v), 4) for v in y.pos],
                yolk_top=round(yolk_top(y), 4), events=S["events"][-3:]), default=float), flush=True)
    except Exception as exc:  # fail closed
        S["broken"] = True
        import traceback

        print("[egg] FAILED " + repr(exc) + "\n" + traceback.format_exc(), flush=True)


# -- reference demo (open loop from the published layout; no simulator state) ---------------------------------------
GRIP = -0.018  # pads 36 mm apart: 2 mm into each 40 mm-apart wall, enough to hold the bulb
SQUEEZE = -0.0105  # pads 21 mm apart: both walls in 9.5 mm, 28.5 mL out
SEAL_AIM = -0.0010  # the mouth's underside 1 mm below the yolk's top: the dome just inside the opening
SET_DOWN = 0.0015


def demo():
    lot = lots.current()
    arm = lot.arm
    sep0 = np.array(lot.pos("separator"), float)
    yolk = np.array(lot.pos("yolk"), float)
    yb = np.array(lot.pos("yolk_bowl"), float)
    top = yolk[2] + YOLK_R[2]

    def at(xy, mouth_z):  # TCP for the mouth at xy, height mouth_z
        return (float(xy[0]), float(xy[1]), float(mouth_z + WALL_Z))

    up = WB.height + 0.035 + TOP
    plan = [
        Waypoint(2.0, at(sep0, sep0[2] + 0.08), FINGER_OPEN, "above the separator"),
        Waypoint(1.5, at(sep0, sep0[2]), FINGER_OPEN, "fingers either side of the bulb"),
        Waypoint(0.8, at(sep0, sep0[2]), GRIP, "hold the bulb"),
        Waypoint(1.5, at(sep0, sep0[2] + 0.10), GRIP, "lift"),
        Waypoint(1.0, at(sep0, sep0[2] + 0.10), SQUEEZE, "squeeze the air out, away from the egg"),
        Waypoint(2.5, at(yolk, up), SQUEEZE, "over the yolk"),
        Waypoint(1.5, at(yolk, top + 0.012), SQUEEZE, "down toward the yolk"),
        Waypoint(1.2, at(yolk, top + SEAL_AIM), SQUEEZE, "mouth onto the yolk"),
        Waypoint(0.5, at(yolk, top + SEAL_AIM), SQUEEZE, "settle"),
        Waypoint(2.0, at(yolk, top + SEAL_AIM), GRIP, "let the bulb open: the yolk is drawn up"),
        Waypoint(0.5, at(yolk, top + SEAL_AIM), GRIP, "wait"),
        Waypoint(1.5, at(yolk, up), GRIP, "lift out of the bowl"),
        Waypoint(2.5, at(yb, TOP + YB.height + 0.035), GRIP, "over the yolk bowl"),
        Waypoint(1.5, at(yb, TOP + YB.floor_t + 2 * YOLK_R[2] + SET_DOWN), GRIP, "mouth just above the bowl floor"),
        Waypoint(0.5, at(yb, TOP + YB.floor_t + 2 * YOLK_R[2] + SET_DOWN), GRIP, "settle"),
        Waypoint(2.5, at(yb, TOP + YB.floor_t + 2 * YOLK_R[2] + SET_DOWN), SQUEEZE, "squeeze gently: the yolk slides out"),
        Waypoint(0.5, at(yb, TOP + YB.floor_t + 2 * YOLK_R[2] + SET_DOWN), SQUEEZE, "wait"),
        Waypoint(1.5, at(yb, TOP + YB.height + 0.035), SQUEEZE, "lift straight up"),
        Waypoint(2.5, at(sep0, sep0[2] + 0.04), SQUEEZE, "over the saucer"),
        Waypoint(1.5, at(sep0, sep0[2] + 0.001), SQUEEZE, "stand it on the saucer"),
        Waypoint(1.0, at(sep0, sep0[2] + 0.001), FINGER_OPEN, "let go"),
        Waypoint(1.5, at(sep0, sep0[2] + 0.12), FINGER_OPEN, "hands up"),
        Waypoint(2.0, (0.17, 0.26 if arm == "left" else -0.26, TOP + 0.20), FINGER_OPEN, "clear"),
        Waypoint(3.0, (0.17, 0.26 if arm == "left" else -0.26, TOP + 0.20), FINGER_OPEN, "still"),
    ]
    return {arm: plan}
