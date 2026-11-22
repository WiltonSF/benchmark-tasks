"""A student compound microscope stands on a laboratory bench, its stage toward the robot and its head turned so the
eyepieces face away. A stained smear slide
is clipped on the stage and the 40x objective is in the light path over the lit spot of the smear. The revolving
nosepiece under the head carries four objectives with coloured bands: 4x (red), 10x (yellow), 40x (blue) and the
100x oil objective (white). To the robot's right a 30 mL amber glass dropping bottle of immersion oil stands on the
bench with its glass dropper, red rubber bulb up, in its neck.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (import before lots: its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

CREAM, BLACK, STEEL, DARK = (0.86, 0.85, 0.80), (0.08, 0.08, 0.09), (0.70, 0.72, 0.75), (0.22, 0.22, 0.24)
GLASS, SMEAR, LIT, LABEL = (0.78, 0.88, 0.90), (0.72, 0.35, 0.55), (1.00, 0.97, 0.75), (0.95, 0.95, 0.95)
AMBER, RUBBER, OIL = (0.55, 0.30, 0.08), (0.62, 0.10, 0.08), (0.95, 0.78, 0.30)
BANDS = {"4x": (0.85, 0.10, 0.10), "10x": (0.95, 0.85, 0.10), "40x": (0.15, 0.45, 0.95), "100x": (0.97, 0.97, 0.97)}


def _qz(a: float):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _qx_to(d) -> tuple:
    """Quaternion (xyzw) turning +x onto direction ``d`` (a prism's axis is its x)."""
    return tuple(float(v) for v in parts._matrix_to_quat(parts._x_to(d)))


def _rod(a, b, r, color, sides=16, collide=True):
    """A round rod (many-sided prism) from point a to point b (part frame)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    return parts.prism(r, float(np.linalg.norm(d)), sides, tuple((a + b) / 2), color, collide=collide,
                       quat=_qx_to(d))


# --- Swift SW350B student compound microscope (estimates from its listed size; part frame: origin on the bench under
# the optical axis, x away from the robot (toward the arm), y to the robot's left, z up) ---------------------------
X0, Y0 = 0.300, 0.000  # the optical axis on the bench
STAGE_TOP = 0.120
SLIDE_TOP = STAGE_TOP + 0.001  # 1 mm slide on the stage
SLIDE_X, SLIDE_Y = 0.025, 0.075  # the slide lies left-right on the stage
PARFOCAL = 0.045  # objective shoulder to the slide's top surface
BETA = math.radians(15)  # the nosepiece axis leans 15 degrees toward the robot
AXIS = (-math.sin(BETA), 0.0, math.cos(BETA))
RC = 0.028  # objective mounting circle radius
MOUNT0 = np.array((0.0, 0.0, SLIDE_TOP + PARFOCAL))  # shoulder of the objective in the light path
PIVOT = np.array((RC, 0.0, MOUNT0[2] + RC * math.tan(BETA)))  # nosepiece centre on its axis
RING_R, RING_L = 0.036, 0.018  # knurled ring radius and thickness
# objective: (turret angle at the start, working distance, barrel length to the front lens)
OBJ = {"40x": (0.0, 0.0006), "100x": (math.pi / 2, 0.0002), "4x": (math.pi, 0.017), "10x": (-math.pi / 2, 0.007)}
Q100 = -math.pi / 2  # nosepiece angle with the 100x in the light path
DETENT, PLAY, HOLD_K = math.radians(10), 0.04, 0.4  # click stops pull within 10 deg; elsewhere it holds ~0.016 N m
NOSE_R, NOSE_L, LENS_L, BARREL_R = 0.0045, 0.008, 0.0015, 0.010


def turret_rot(psi: float) -> np.ndarray:
    return parts._rot(AXIS, psi)


def objective_frame(name: str, q: float) -> tuple[np.ndarray, np.ndarray]:
    """(shoulder point, unit axis pointing to the front lens) of an objective, part frame, nosepiece at q."""
    r = turret_rot(OBJ[name][0] + q)
    return PIVOT + r @ (MOUNT0 - PIVOT), r @ np.array((0.0, 0.0, -1.0))


def objective_len(name: str) -> float:
    return PARFOCAL - OBJ[name][1]


def _objective_shapes(name: str):
    m, o = objective_frame(name, 0.0)
    L = objective_len(name)
    return (
        _rod(m, m + o * (L - NOSE_L), BARREL_R, BLACK),
        _rod(m + o * (L - NOSE_L), m + o * (L - LENS_L), NOSE_R, STEEL),
        _rod(m + o * (L - LENS_L), m + o * L, NOSE_R - 0.001, (0.35, 0.40, 0.45), collide=False),
        _rod(m + o * 0.006, m + o * 0.011, BARREL_R + 0.0004, BANDS[name], collide=False),
    )


RING_C = PIVOT + np.asarray(AXIS) * (RING_L / 2)
_E1, _E2 = np.array((math.cos(BETA), 0.0, math.sin(BETA))), np.array((0.0, 1.0, 0.0))  # the ring's plane


RING_SHAPES = (parts.prism(RING_R, RING_L, 36, tuple(RING_C), DARK, quat=_qx_to(AXIS)),)  # knurled ring


def _microscope_shapes():
    s = [
        parts.box((0.215, 0.150, 0.030), (0.0375, 0.0, 0.015), CREAM),  # base
        parts.cylinder(0.022, 0.004, (0.0, 0.0, 0.032), LIT, collide=False),  # lamp window
        parts.box((0.045, 0.050, 0.200), (0.1225, 0.0, 0.130), CREAM),  # arm (pillar), behind the nosepiece
        parts.box((0.105, 0.046, 0.065), (0.070, 0.0, 0.2275), CREAM),  # arm neck over the nosepiece
        parts.cylinder(0.020, 0.006, (RC, 0.0, 0.192), STEEL),  # nosepiece mount
        parts.box((0.110, 0.076, 0.050), (0.030, 0.0, 0.285), CREAM),  # binocular head
        parts.box((0.140, 0.140, 0.010), (0.0, 0.0, STAGE_TOP - 0.005), BLACK),  # stage
        parts.box((0.060, 0.050, 0.020), (0.0750, 0.0, 0.100), BLACK),  # stage bracket to the arm
        parts.cylinder(0.018, 0.050, (0.0, 0.0, 0.085), BLACK),  # condenser
        parts.cylinder(0.020, 0.015, (0.1225, 0.0325, 0.075), BLACK, axis="y"),  # coarse focus knobs
        parts.cylinder(0.020, 0.015, (0.1225, -0.0325, 0.075), BLACK, axis="y"),
        parts.cylinder(0.009, 0.040, (0.045, -0.080, 0.085), BLACK),  # stage knobs, under the stage's right
        # the slide, clipped in the mechanical stage's holder (back jaw and left-end spring finger)
        parts.box((SLIDE_X, SLIDE_Y, 0.001), (0.0, 0.0, STAGE_TOP + 0.0005), GLASS),
        parts.box((0.018, 0.034, 0.0002), (0.0, 0.006, SLIDE_TOP + 0.0001), SMEAR, collide=False),
        parts.box((0.022, 0.020, 0.0002), (0.0, -0.026, SLIDE_TOP + 0.0001), LABEL, collide=False),
        parts.cylinder(0.002, 0.0002, (0.0, 0.0, SLIDE_TOP + 0.00025), LIT, collide=False),  # the lit spot
        parts.box((0.006, 0.100, 0.003), (SLIDE_X / 2 + 0.003, 0.0, STAGE_TOP + 0.0015), STEEL),
        parts.box((0.020, 0.005, 0.003), (0.0, SLIDE_Y / 2 + 0.0025, STAGE_TOP + 0.0015), STEEL),
    ]
    for side in (1.0, -1.0):  # the head is turned (it rotates 360 degrees): eyepieces lean 30 degrees to the back
        a = np.array((0.070, side * 0.032, 0.310))
        d = np.array((math.sin(math.radians(30)), 0.0, math.cos(math.radians(30))))
        s.append(_rod(a, a + d * 0.070, 0.012, CREAM))
        s.append(_rod(a + d * 0.070, a + d * 0.100, 0.0145, BLACK))
    return s


# drop visuals: one small amber disc every 2 mm round the lit spot, shown where a drop lands
DROP_R, DROP_H = 0.0025, 0.0012
GRID = 0.002
GRID_N = 4  # -8..+8 mm


def _disc(cx, cy, z0, r, h, n=12):
    pts = [(cx, cy, z0 + h)] + [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n),
                                  z0 + h * 0.4) for k in range(n)]
    tris = [(0, 1 + k, 1 + (k + 1) % n) for k in range(n)]
    return tuple(pts), tuple(tris)


def drop_visual_name(i: int, j: int) -> str:
    return f"drop_{i + GRID_N}_{j + GRID_N}"


DROP_VISUALS = []
for _i in range(-GRID_N, GRID_N + 1):
    for _j in range(-GRID_N, GRID_N + 1):
        _p, _t = _disc(_i * GRID, _j * GRID, SLIDE_TOP, DROP_R, DROP_H)
        DROP_VISUALS.append(parts.Visual(drop_visual_name(_i, _j), _p, _t, OIL))

MICROSCOPE = parts.Part(
    "microscope", pos=(X0, Y0, TOP), mass=None, shapes=_microscope_shapes(), friction=(1.0, 0.9),
    contact_offset=0.001, visuals=tuple(DROP_VISUALS),
    report=tuple(f"obj_{n}" for n in OBJ),
    joints=[parts.Hinge("nosepiece", shapes=RING_SHAPES, pivot=tuple(PIVOT), axis=AXIS, lower=-3.7, upper=3.7,
                        mass=0.08, spring=HOLD_K, damping=0.02, max_effort=2.0)]
    + [parts.Fixed(f"obj_{n}", shapes=_objective_shapes(n), at=tuple(objective_frame(n, 0.0)[0]), mass=0.03,
                   parent="nosepiece") for n in OBJ],
)

# --- Kimble 15035-30 30 mL dropping bottle (12/18 ground joint) with its glass dropper and rubber bulb -----------
BOT_R, BOT_WALL, BOT_H = 0.018, 0.002, 0.062
NECK_IN, NECK_OUT, NECK_Z0, NECK_TOP = 0.0072, 0.0105, 0.068, 0.088
NECK_LEAD = 0.0088  # the socket's mouth flares for its last 3 mm


def _ring(r_in, r_out, z0, z1, color, n=16):
    rm, t = (r_in + r_out) / 2, r_out - r_in
    return [parts.box((t, 2 * math.pi * rm / n * 1.08, z1 - z0), (rm * math.cos(2 * math.pi * k / n),
                       rm * math.sin(2 * math.pi * k / n), (z0 + z1) / 2), color, quat=_qz(2 * math.pi * k / n))
            for k in range(n)]


BOTTLE_SHAPES = ([parts.cylinder(BOT_R, 0.002, (0, 0, 0.001), AMBER)]
                 + _ring(BOT_R - BOT_WALL, BOT_R, 0.002, BOT_H, AMBER)
                 + _ring(NECK_IN, BOT_R, BOT_H, NECK_Z0, AMBER)
                 + _ring(NECK_IN, NECK_OUT, NECK_Z0, NECK_TOP - 0.003, AMBER)
                 + _ring(NECK_LEAD, NECK_OUT, NECK_TOP - 0.003, NECK_TOP, AMBER)
                 + [parts.box((0.002, 0.024, 0.030), (-BOT_R - 0.0002, 0.0, 0.032), LABEL, collide=False)])

# dropper frame: origin at the bottom of its glass shoulder, which rests on the neck's top when seated
STOPPER_R, STEM_R, TIP_Z, BULB_R = 0.0062, 0.0035, -0.072, 0.0075
BULB_Z = 0.019  # bulb centre: where the hand holds it
DROPPER_SHAPES = [
    parts.cylinder(0.0095, 0.003, (0, 0, 0.0015), GLASS),
    parts.cylinder(STOPPER_R, 0.018, (0, 0, -0.009), GLASS),
    parts.cylinder(STEM_R, 0.050, (0, 0, -0.043), GLASS),
    parts.cylinder(0.002, 0.004, (0, 0, TIP_Z + 0.002), GLASS),
    parts.cylinder(BULB_R, 0.032, (0, 0, BULB_Z), RUBBER),
    parts.sphere(BULB_R, (0, 0, 0.035), RUBBER),
]

BOTTLE_XY = (0.205, -0.150)
BOTTLE = parts.Part("oil_bottle", pos=(*BOTTLE_XY, TOP), mass=0.075, shapes=BOTTLE_SHAPES, friction=(0.8, 0.7),
                    contact_offset=0.001, solver=(64, 4))
DROPPER = parts.Part("oil_dropper", pos=(*BOTTLE_XY, TOP + NECK_TOP), mass=0.012, shapes=DROPPER_SHAPES,
                     friction=(1.0, 0.9), contact_offset=0.001, report=("base",), solver=(64, 4))

OBJECTS = [MICROSCOPE, BOTTLE, DROPPER]

# lot B: the bottle (with its dropper) further right and back; the microscope (fixed, jointed) never moves
B_BOTTLE = (0.255, -0.190)
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={BOTTLE.name: BOTTLE.pos}),  # a zero move still runs the settle and a fresh first frame
    lots.Lot("B", place={BOTTLE.name: (*B_BOTTLE, TOP), DROPPER.name: (*B_BOTTLE, TOP + NECK_TOP)}),
)

# --- the scorer's record ------------------------------------------------------------------------------------------
SQUEEZE_N, REARM_N, STREAM_N = 10.0, 5.0, 35.0  # N per pad on the bulb: one drop, re-armed, a stream (estimates)
LOAD = 3  # drops the dropper holds each time it comes out of the bottle
MAX_DROPS = 2
SPOT_R = 0.003  # drop centre within 3 mm of the optical axis
FALL_H = 0.025  # a drop leaves the tip only within 25 mm above what is under it; higher it splashes (off the slide)
TOUCH_N = 0.1  # N of robot-finger or dropper contact on an objective that voids the trial (any real touch)
Q_TOL = math.radians(2)
HOLD_S, HAND_GAP = 1.0, 0.030
REC: dict = parts.__dict__.setdefault("_oil_record", {})


def reset_record() -> None:
    REC.clear()
    REC.update(known=False, zero=False, why="", drops=[], loaded=0, armed=True, q=0.0, target=0.0, t=None,
               still_s=0.0, clear_s=0.0, checked=False, shown=[], peak={"pad": 0.0, "touch": 0.0})


reset_record()


def _R(s) -> np.ndarray:
    return parts._quat_to_matrix(s.quat_xyzw)


def tilt_deg(s) -> float:
    return math.degrees(math.acos(float(np.clip(_R(s)[2, 2], -1.0, 1.0))))


def _local(p) -> np.ndarray:
    """A world point in the microscope's frame."""
    return np.asarray(p, float) - np.array((X0, Y0, TOP))


def dropper_tip(d) -> np.ndarray:
    return _local(d.pos + _R(d) @ np.array((0.0, 0.0, TIP_Z)))


def nose(name: str, q: float) -> tuple[np.ndarray, float]:
    """(front-lens centre (part frame), lowest point of the nose above the slide) for an objective at nosepiece q."""
    m, o = objective_frame(name, q)
    c = m + o * objective_len(name)
    tilt = math.acos(float(np.clip(-o[2], -1.0, 1.0)))
    return c, float(c[2] - SLIDE_TOP - NOSE_R * math.sin(tilt))


def on_slide(p) -> bool:
    return abs(p[0]) <= SLIDE_X / 2 + EPS and abs(p[1]) <= SLIDE_Y / 2 + EPS


def bottle_up(b) -> bool:
    return tilt_deg(b) <= 10.0 + EPS and abs(float(b.pos[2]) - TOP) <= 0.003 + EPS


def seated(b, d) -> bool:
    """Dropper home in the bottle: its shoulder within 3 mm above the neck top, centred within 2 mm, within 10 deg."""
    rel = _R(b).T @ (d.pos - b.pos)
    axis_dev = math.degrees(math.acos(float(np.clip((_R(b)[:, 2] @ _R(d)[:, 2]), -1.0, 1.0))))
    return (bottle_up(b) and math.hypot(rel[0], rel[1]) <= 0.002 + EPS and NECK_TOP - 0.0005 - EPS <= rel[2]
            <= NECK_TOP + 0.003 + EPS and axis_dev <= 10.0 + EPS)


def in_bottle(b, d) -> bool:
    """The dropper's tip is down inside the bottle's neck (it reloads there)."""
    tip = _R(b).T @ (d.pos + _R(d) @ np.array((0.0, 0.0, TIP_Z)) - b.pos)
    return math.hypot(tip[0], tip[1]) <= NECK_IN + EPS and tip[2] <= NECK_TOP - 0.020 + EPS


def wetted(q: float) -> bool:
    """The 100x is at its click stop with a drop under its front lens."""
    if abs(q - Q100) > Q_TOL + EPS:
        return False
    return any(math.hypot(p[0], p[1]) <= SPOT_R + EPS for p in REC["drops"])


def level(o) -> float:
    if not REC["drops"] or len(REC["drops"]) > MAX_DROPS:
        return 0.0
    if math.hypot(*REC["drops"][0][:2]) > SPOT_R + EPS:
        return 0.0
    q = float(o[MICROSCOPE.name].joints["nosepiece"])
    if not wetted(q):
        return 0.25
    if not seated(o[BOTTLE.name], o[DROPPER.name]):
        return 0.5
    if REC.get("clear_s", 0.0) >= HOLD_S - EPS:
        return 1.0
    return 0.75


def score(objects) -> float:
    """The tier the end state shows, once the nosepiece, bottle and dropper have been still for HOLD_S."""
    try:
        if not REC.get("known") or REC.get("zero") or REC.get("still_s", 0.0) < HOLD_S - EPS:
            return 0.0
        if not all(objects[n].resting(0.005, 0.1) for n in (BOTTLE.name, DROPPER.name)):
            return 0.0
        return float(level(objects))
    except (KeyError, TypeError, ValueError, AttributeError, IndexError):
        return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


def _zero(why: str) -> None:
    if not REC["zero"]:
        print(f"[oil] score 0 for this trial: {why}", file=sys.stderr, flush=True)
    REC.update(zero=True, why=REC["why"] or why)


def _fouled(q0: float, q1: float) -> str:
    """A dry objective's front lens through a drop while the nosepiece turned from q0 to q1 (checked every 1 deg)."""
    n = max(2, int(abs(q1 - q0) / math.radians(1.0)) + 2)
    for k in range(n):
        q = q0 + (q1 - q0) * k / (n - 1)
        for name in ("40x", "10x", "4x"):
            c, low = nose(name, q)
            if low >= DROP_H - EPS:
                continue
            for p in REC["drops"]:
                if math.hypot(c[0] - p[0], c[1] - p[1]) < DROP_R + NOSE_R - EPS:
                    return name
    return ""


def _land(tip) -> None:
    """One drop leaves the tip and falls straight down onto whatever is below it, where the tip is."""
    q = REC["q"]
    for name in OBJ:  # an objective's nose right under the tip catches it (oil on a barrel or lens is a fault)
        c, low = nose(name, q)
        if math.hypot(c[0] - tip[0], c[1] - tip[1]) < NOSE_R + DROP_R and tip[2] > c[2] - EPS:
            _zero(f"oil dropped on the {name} objective")
            REC["drops"].append(np.array((tip[0], tip[1], SLIDE_TOP)))
            return
    h = tip[2] - SLIDE_TOP
    if not on_slide(tip) or h <= EPS or h > FALL_H + EPS:
        _zero(f"oil dropped off the slide at {np.round(tip * 1000, 1).tolist()} mm")
        REC["drops"].append(np.array((tip[0], tip[1], SLIDE_TOP)))
        return
    REC["drops"].append(np.array((tip[0], tip[1], SLIDE_TOP)))
    if len(REC["drops"]) > MAX_DROPS:
        _zero(f"{len(REC['drops'])} drops on the slide")


def update(o, t: float, pads: tuple[float, float], touch: float, clear: bool) -> None:
    """One control step of history: drops from the dropper, the whole-trial rules, the still and clear timers.
    ``pads`` are the two finger pads' forces on the dropper's rubber bulb (a hand holding the glass stem or shoulder
    squeezes nothing: the simulator passes 0 for a hand whose grip centre is not on the bulb)."""
    dt = 0.05 if REC["t"] is None else max(0.0, t - REC["t"])
    m, b, d = o[MICROSCOPE.name], o[BOTTLE.name], o[DROPPER.name]
    q = float(m.joints["nosepiece"])
    q_prev = REC["q"] if REC["t"] is not None else q
    pk = REC["peak"]
    pad = min(pads)
    pk["pad"], pk["touch"] = max(pk["pad"], max(pads)), max(pk["touch"], touch)
    if touch > TOUCH_N + EPS:
        _zero(f"a robot finger or the dropper pressed an objective ({touch:.2f} N)")
    if tilt_deg(b) > 45.0 + EPS:
        _zero("the oil bottle fell over")
    if in_bottle(b, d):
        REC["loaded"] = LOAD
    down = float(_R(d)[2, 2]) >= 0.5 - EPS  # the bulb is up and the tip down, within 60 degrees of vertical
    if not in_bottle(b, d) and down and REC["loaded"] > 0:
        if max(pads) > STREAM_N + EPS:
            _zero(f"the bulb was crushed ({max(pads):.1f} N): a stream of oil")
            for _ in range(REC["loaded"]):
                _land(dropper_tip(d))
            REC["loaded"] = 0
        elif REC["armed"] and pad >= SQUEEZE_N - EPS:
            _land(dropper_tip(d))
            REC["loaded"] -= 1
            REC["armed"] = False
    if pad <= REARM_N + EPS:
        REC["armed"] = True
    REC["q"] = q
    foul = _fouled(q_prev, q) if REC["drops"] else ""
    if foul:
        _zero(f"the {foul} objective went through the oil")
    turning = abs(q - q_prev) > math.radians(0.2)
    still = (not turning) and all(x.resting(0.005, 0.1) for x in (b, d))
    REC["still_s"] = REC["still_s"] + dt if still else 0.0
    REC["clear_s"] = REC["clear_s"] + dt if clear else 0.0
    REC.update(t=t, known=True)


# --- simulator side ---------------------------------------------------------------------------------------------
HAND_LINKS = ("gripper", "tip_left", "tip_right")
_SIM: dict = {}
_FACE_PTS: dict = {}


def _arr(x) -> np.ndarray:
    x = x.torch if hasattr(x, "torch") else x
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy())
    return np.asarray(x)


def _views(base):
    from isaaclab.sim.utils import get_current_stage
    from isaaclab_physx.physics import PhysxManager
    from pxr import UsdPhysics

    sv = PhysxManager.get_physics_sim_view()
    env = parts._PATHS[BOTTLE.name].rsplit("/", 1)[0]
    hands, pads = [], []

    def walk(prim):
        for c in prim.GetAllChildren():
            yield c
            yield from walk(c)

    for prim in get_current_stage().GetPrimAtPath(env).GetAllChildren():
        if "Arm" in prim.GetName():
            for p in walk(prim):
                if p.GetName() in HAND_LINKS and p.HasAPI(UsdPhysics.RigidBodyAPI):
                    hands.append(p.GetPath().pathString)
    if len(hands) != 6:
        raise RuntimeError(f"found {len(hands)} hand links, expected 6")
    pads = [h for h in hands if h.rsplit("/", 1)[1] in ("tip_left", "tip_right")]
    root = parts._PATHS[MICROSCOPE.name]
    drop = sv.create_rigid_contact_view(parts._PATHS[DROPPER.name], filter_patterns=pads, max_contact_data_count=128)
    objs = [sv.create_rigid_contact_view(f"{root}/obj_{n}", filter_patterns=hands + [parts._PATHS[DROPPER.name]],
                                         max_contact_data_count=128) for n in OBJ]
    arms = {}
    for i, h in enumerate(pads):
        arm = "left_robot" if "/LeftArm" in h else "right_robot" if "/RightArm" in h else None
        if arm is None:
            raise RuntimeError(f"cannot tell which arm the pad {h} is on")
        arms.setdefault(arm, []).append(i)
    if sorted(len(v) for v in arms.values()) != [2, 2]:
        raise RuntimeError(f"pads per arm {arms}")
    return drop, objs, len(pads), arms


def _hull_surface(h, spacing=0.003):
    from scipy.spatial import ConvexHull

    v = h.vertices
    out = [v]
    for tri in ConvexHull(v).simplices:
        a, b, c = v[tri]
        n = max(1, int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / spacing)))
        out.append(np.array([a + (b - a) * u / n + (c - a) * w / n for u in range(n + 1) for w in range(n + 1 - u)]))
    return np.unique(np.round(np.concatenate(out) / 0.0025).astype(np.int64), axis=0) * 0.0025


def hand_points(base) -> np.ndarray:
    """Points on the faces (every 3 mm) of both arms' gripper and finger collision hulls (env frame)."""
    from yam_arena import analysis

    origin = parts._np(base.scene.env_origins[0])
    pts = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in HAND_LINKS:
                if n not in _FACE_PTS:
                    _FACE_PTS[n] = np.concatenate([_hull_surface(h) for h in analysis.link_hulls().get(n, [])])
                pts.append(_FACE_PTS[n] @ parts._quat_to_matrix(poses[i, 3:]).T + (poses[i, :3] - origin))
    if len(pts) != 6:
        raise RuntimeError("hand links not found")
    return np.concatenate(pts)


def hands_clear(points, o, gap) -> bool:
    """Every hand point at least ``gap`` from every shape box of the nosepiece and objectives, bottle and dropper."""
    from yam_arena import analysis

    lo, hi = points.min(axis=0), points.max(axis=0)
    for spec in (MICROSCOPE, BOTTLE, DROPPER):
        st = o[spec.name]
        if spec is not MICROSCOPE and (np.maximum(np.maximum(st.low - hi, lo - st.high), 0.0) ** 2).sum() >= gap ** 2:
            continue  # the hands' whole box is farther than gap from this object's box
        poses = {"base": (st.pos, _R(st))}
        poses.update({k: (np.asarray(v[0]), parts._quat_to_matrix(v[1])) for k, v in getattr(st, "links", {}).items()})
        for link, c, r, size in spec.world_boxes(poses):
            if spec is MICROSCOPE and link == "base":
                continue  # the microscope's fixed body: only the nosepiece and objectives count
            if float(analysis.box_signed_distance(points, c, r, size).min()) < gap - EPS:
                return False
    return True


def tcp_point(base, arm: str) -> np.ndarray:
    """World grip centre (between the finger pads) of one arm, env frame."""
    from yam_arena.constants import TCP_OFFSET

    origin = parts._np(base.scene.env_origins[0])
    rob = base.scene[arm]
    pose = parts._np(rob.data.body_link_pose_w)[0, rob.body_names.index("gripper")]
    return pose[:3] - origin + parts._quat_to_matrix(pose[3:]) @ np.array(TCP_OFFSET)


def on_bulb(d, p) -> bool:
    """A grip centre within the rubber bulb's length and 12 mm of the dropper's axis."""
    rel = _R(d).T @ (np.asarray(p, float) - d.pos)
    return -0.003 - EPS <= rel[2] <= 0.040 + EPS and math.hypot(rel[0], rel[1]) <= 0.012 + EPS


def _show_drops(sim) -> None:
    for p in REC["drops"][len(REC["shown"]):]:
        i, j = int(round(p[0] / GRID)), int(round(p[1] / GRID))
        name = drop_visual_name(i, j) if max(abs(i), abs(j)) <= GRID_N else ""
        if name:
            sim.set_visible(MICROSCOPE.name, name, True)
        REC["shown"].append(name)


def _start_ok(o) -> str:
    lot = lots.current()
    for spec in (BOTTLE, DROPPER):
        err = float(np.linalg.norm(o[spec.name].pos[:2] - np.asarray(lot.pos(spec.name))[:2]))
        if err > 0.004 + EPS:
            return f"{spec.name} starts {err * 1000:.1f} mm from lot {lot.name}"
    if abs(float(o[MICROSCOPE.name].joints["nosepiece"])) > math.radians(2):
        return "nosepiece does not start at the 40x"
    return ""


@parts.on_reset
def _on_reset(sim) -> None:
    reset_record()
    _SIM.clear()
    try:
        for v in DROP_VISUALS:
            sim.set_visible(MICROSCOPE.name, v.name, False)
    except Exception as exc:  # noqa: BLE001
        print(f"[oil] could not hide drop visuals: {exc}", file=sys.stderr, flush=True)


@parts.on_step
def _on_step(sim) -> None:
    row = {}
    try:
        base = sim.env.unwrapped
        o = sim.states()
        t = float(sim.t)
        if not REC["checked"]:
            REC["checked"] = True
            REC["loaded"] = LOAD
            why = _start_ok(o)
            print(f"[oil] trial start lot {lots.current().name}: "
                  + ", ".join(f"{n} {np.round(o[n].pos, 4).tolist()}" for n in (BOTTLE.name, DROPPER.name))
                  + f", nosepiece {math.degrees(float(o[MICROSCOPE.name].joints['nosepiece'])):.2f} deg",
                  file=sys.stderr, flush=True)
            if why:
                _zero("start: " + why)
        if "views" not in _SIM:
            _SIM["views"] = _views(base)
        drop_view, obj_views, npads, pad_arms = _SIM["views"]
        pdt = base.physics_dt
        fp = np.linalg.norm(_arr(drop_view.get_contact_force_matrix(dt=pdt)).reshape(-1, 3), axis=1)
        if fp.shape != (npads,) or not np.isfinite(fp).all():
            raise RuntimeError(f"pad contact data shape {fp.shape}")
        # each arm's two pads on the dropper; a hand counts as squeezing only with its grip centre on the bulb
        d = o[DROPPER.name]
        pairs = []
        for arm, idx in pad_arms.items():
            f = sorted(float(fp[i]) for i in idx)
            pairs.append(tuple(f) if on_bulb(d, tcp_point(base, arm)) else (0.0, 0.0))
        pads = max(pairs, key=lambda p: p[0]) if pairs else (0.0, 0.0)
        touch = 0.0
        for v in obj_views:
            ft = np.linalg.norm(_arr(v.get_contact_force_matrix(dt=pdt)).reshape(-1, 3), axis=1)
            if not np.isfinite(ft).all():
                raise RuntimeError("objective contact data not finite")
            touch = max(touch, float(ft.max()) if ft.size else 0.0)
        # the hands-clear check is costly: it matters only once the end state already shows the 0.75 level
        clear = level(o) >= 0.75 - EPS and hands_clear(hand_points(base), o, HAND_GAP)
        update(o, t, (float(pads[0]), float(pads[1])), touch, clear)
        _show_drops(sim)
        # the nosepiece: it holds where it is left (friction) and clicks into a stop within 10 degrees of one
        q = REC["q"]
        stop = round(q / (math.pi / 2)) * (math.pi / 2)
        if abs(q - stop) <= DETENT + EPS:
            tgt = stop
        else:
            tgt = min(max(REC["target"], q - PLAY), q + PLAY)
        if abs(tgt - REC["target"]) > 1e-6:
            REC["target"] = tgt
            sim.set_target(MICROSCOPE.name, "nosepiece", tgt)
        d = o[DROPPER.name]
        row = dict(t=t, q_deg=math.degrees(q), target_deg=math.degrees(tgt), pads=list(map(float, pads)),
                   touch=touch, drops=[np.round(p * 1000, 2).tolist() for p in REC["drops"]],
                   loaded=REC["loaded"], level=level(o), still_s=REC["still_s"], clear_s=REC["clear_s"],
                   zero=bool(REC["zero"]), why=REC["why"], dropper=d.pos.tolist(), dropper_tilt=tilt_deg(d),
                   tip_mm=np.round(dropper_tip(d) * 1000, 2).tolist(), bottle=o[BOTTLE.name].pos.tolist(),
                   seated=bool(seated(o[BOTTLE.name], d)))
    except Exception as exc:  # noqa: BLE001  fail closed: an unreadable trial scores 0
        if not REC["zero"]:
            import traceback

            traceback.print_exc()
        _zero(f"scorer error {type(exc).__name__}: {exc}")
        row = {"t": float(getattr(sim, "t", 0.0)), "error": str(exc)}
    if os.environ.get("TRACE_OUT"):
        try:
            import json

            base = sim.env.unwrapped
            from yam_arena.constants import TCP_OFFSET

            origin = parts._np(base.scene.env_origins[0])
            for arm in ("left_robot", "right_robot"):
                rob = base.scene[arm]
                pose = parts._np(rob.data.body_link_pose_w)[0, rob.body_names.index("gripper")]
                row[arm] = (pose[:3] - origin + parts._quat_to_matrix(pose[3:]) @ np.array(TCP_OFFSET)).tolist()
                row[arm + "_fingers"] = parts._np(rob.data.joint_pos)[0, -2:].tolist()
            row["lot"] = lots.current().name
            with pathlib.Path(os.environ["TRACE_OUT"]).with_name("oil.jsonl").open("a") as f:
                f.write(json.dumps(row, default=float) + "\n")
        except Exception as exc:  # noqa: BLE001
            if not _SIM.get("logerr"):
                _SIM["logerr"] = True
                print(f"[oil] trace logging skipped: {exc}", file=sys.stderr, flush=True)


# --- scripted demo: open loop from the visible layout ---------------------------------------------------------------
DEMO_TOUCHES = ("microscope",)
W0 = np.array((X0, Y0, TOP))
BULB_W = 2 * BULB_R
LEAN = 45.0  # degrees the dropper is leaned toward the microscope
PINCH_D, PINCH_DZ, PINCH_LEAN = 0.008, 0.0, 44.0  # grip chord 8 mm in from the rim, at the ring's mid-height
TURN1, TURN2 = (180.0, 135.0), (135.0, 92.0)  # where round the ring the hand holds it, start and end (deg)
TIP_FIX = (0.0, 0.0, 0.0)
DOCK_FIX = (0.0017, 0.0010, 0.0)  # the held dropper sits this far off the hand's centre (trace 1)


def _finger(span: float, press: float) -> float:
    return float(np.clip(-(span / 2 - press), FINGER_OPEN, 0.0))


def demo():
    lot = lots.current()
    bot = np.asarray(lot.pos(BOTTLE.name))
    hold, squeeze = _finger(BULB_W, 0.0008), _finger(BULB_W, 0.0040)

    def wp(t, pos, f, label, yaw=0.0):
        return Waypoint(float(t), tuple(float(v) for v in pos), float(f), label, float(yaw))

    def tw(t, pos, f, label, yaw=0.0, lean=0.0, heading=0.0):
        return tilt.Waypoint(float(t), tuple(float(v) for v in pos), float(f), label, yaw=float(yaw), tilt=lean,
                             heading=heading)

    # left hand: turns the nosepiece by its knurled ring, held between the fingertips at two points on the ring's rim
    # either side of an objective (never touching the objectives), and carried round with it
    def pinch_pose(phi, radius=None, dz=0.0):
        r = RING_R - PINCH_D if radius is None else radius
        pos = W0 + RING_C + r * (math.cos(phi) * _E1 + math.sin(phi) * _E2) + np.array((0.0, 0.0, PINCH_DZ + dz))
        return pos, dict(lean=PINCH_LEAN, heading=phi + math.pi, yaw=phi - math.pi)

    chord = 2 * math.sqrt(RING_R ** 2 - (RING_R - PINCH_D) ** 2)
    pinch = _finger(chord, 0.0025)

    def turn(phi0, phi1, label):
        out = []
        p, o = pinch_pose(phi0, RING_R + 0.045, 0.02)
        out.append(tw(2.0, p, FINGER_OPEN, f"open hand beside the ring ({label})", **o))
        p, o = pinch_pose(phi0)
        out.append(tw(1.5, p, FINGER_OPEN, "fingertips either side of the ring", **o))
        out.append(tw(0.8, p, pinch, "hold the ring by its rim", **o))
        n = max(1, int(round(abs(math.degrees(phi1 - phi0)) / 5)))
        for k in range(1, n + 1):
            p, o = pinch_pose(phi0 + (phi1 - phi0) * k / n)
            out.append(tw(0.35, p, pinch, f"turn the ring ({label})", **o))
        out.append(tw(0.6, p, FINGER_OPEN, "let go of the ring", **o))
        p, o = pinch_pose(phi1, RING_R + 0.045, 0.02)
        out.append(tw(1.2, p, FINGER_OPEN, "hand out from the ring", **o))
        return out

    park = W0 + np.array((-0.060, 0.170, 0.230))
    park_o = dict(lean=40.0, heading=-math.pi / 2, yaw=-math.pi / 2)
    L = turn(math.radians(TURN1[0]), math.radians(TURN1[1]), "half-way")
    L.append(tw(2.0, park, 0.0, "wait clear of the stage", **park_o))
    t_left1 = sum(w.duration for w in L)
    # right hand: dropper out, one drop on the lit spot, dropper back
    bulb = bot + np.array((0.0, 0.0, NECK_TOP + BULB_Z))
    up = np.array((0.0, 0.0, 1.0))
    lift = 0.090
    g = np.array((math.sin(math.radians(LEAN)), 0.0, -math.cos(math.radians(LEAN))))
    tip_off = (BULB_Z - TIP_Z) * g  # from the hand's centre point to the dropper's tip, leaned
    spot = W0 + np.array((0.0, 0.0, SLIDE_TOP + 0.006)) + np.asarray(TIP_FIX)
    at_spot = spot - tip_off
    lean = dict(lean=LEAN, heading=0.0)
    dock = np.asarray(DOCK_FIX)
    front = W0 + np.array((-0.120, -0.040, 0.200))  # carried upright here, clear of the eyepieces
    R = [wp(max(3.0, t_left1 - 9.0), bulb + up * 0.06, FINGER_OPEN, "over the dropper"),
         wp(1.5, bulb, FINGER_OPEN, "down to the bulb"),
         wp(1.0, bulb, hold, "hold the bulb"),
         wp(2.0, bulb + up * lift, hold, "draw the dropper out"),
         wp(2.0, front, hold, "in front of the microscope"),
         tw(2.5, at_spot - g * 0.050 + up * 0.02, hold, "lean the dropper in front of the stage", **lean),
         tw(2.5, at_spot, hold, "tip over the lit spot", **lean),
         tw(0.8, at_spot, squeeze, "squeeze one drop", **lean),
         tw(0.8, at_spot, hold, "ease the bulb", **lean),
         tw(2.0, at_spot - g * 0.050 + up * 0.02, hold, "draw the tip back", **lean),
         wp(2.5, front, hold, "upright again"),
         wp(2.0, bulb + up * lift + dock, hold, "over the bottle"),
         wp(2.5, bulb + up * 0.002 + dock, hold, "dropper down into the neck"),
         wp(1.0, bulb + up * 0.002, FINGER_OPEN, "let go"),
         wp(1.5, bulb + up * 0.07, FINGER_OPEN, "hand up"),
         wp(1.5, bulb + up * 0.07 + np.array((-0.08, -0.04, 0.0)), FINGER_OPEN, "hand clear")]
    t_drop_done = sum(w.duration for w in R[:10])
    # left hand again, once the dropper is out of the way: on to the 100x click stop
    L.append(tw(max(1.0, t_drop_done - t_left1 - 1.5), park, 0.0, "wait for the drop", **park_o))
    L += turn(math.radians(TURN2[0]), math.radians(TURN2[1]), "on to the 100x")
    L.append(tw(2.0, park, 0.0, "hand clear", **park_o))
    tl, tr = sum(w.duration for w in L), sum(w.duration for w in R)
    if tl < tr:
        L.append(tw(tr - tl + 2.0, park, 0.0, "hands clear", **park_o))
    else:
        R.append(wp(tl - tr + 2.0, bulb + up * 0.07 + np.array((-0.08, -0.04, 0.0)), FINGER_OPEN, "hands clear"))
    return {"left": L, "right": R}
