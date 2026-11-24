"""A laboratory preparation counter with a reagent shelf at the back. An empty 100 mL glass serum bottle stands
on the counter with a grey rubber stopper held in its neck by a silver aluminum crimp seal. A steel plier decapper
with grey plastic-coated handles lies open on the counter to the robot's right, and an empty 250 mL glass waste
beaker stands near the back. The seal, the bottle, the plier and the beaker are separate loose objects.
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

GLASS, STOPPER, ALU = (0.62, 0.78, 0.80), (0.33, 0.33, 0.35), (0.80, 0.81, 0.84)
STEEL, GRIP, SHELF_WHITE, BEAKER_GLASS = (0.70, 0.72, 0.75), (0.42, 0.40, 0.42), (0.86, 0.86, 0.84), (0.70, 0.84, 0.86)


def _qz(a: float):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _rz(a: float) -> np.ndarray:
    return parts._rot((0, 0, 1), a)


# --- WHEATON 223747 serum bottle, 100 mL: 52 mm x 95 mm with its 20 mm crimp finish (heights above its base) ------
BOTTLE_R = 0.026
LIP_R, LIP_Z0, LIP_Z1 = 0.0099, 0.0875, 0.0925  # the glass finish bead the seal is crimped over
BOTTLE_SHAPES = [
    parts.cylinder(BOTTLE_R, 0.070, (0, 0, 0.035), GLASS),
    parts.cylinder(0.020, 0.004, (0, 0, 0.072), GLASS),
    parts.cylinder(0.013, 0.004, (0, 0, 0.076), GLASS),
    parts.cylinder(0.0085, 0.0095, (0, 0, 0.08275), GLASS),
    # the bead and the stopper flange sit inside the seal, which collides as a solid: shown, not collided
    parts.cylinder(LIP_R, LIP_Z1 - LIP_Z0, (0, 0, (LIP_Z0 + LIP_Z1) / 2), GLASS, collide=False),
    parts.cylinder(0.0095, 0.002, (0, 0, 0.0935), STOPPER, collide=False),  # W224100-405 stopper flange
]

# --- WHEATON 224178-01 20 mm open-top aluminum seal: skirt over the bead, annular top (origin: skirt bottom) -------
SEAL_Z = LIP_Z0  # crimped: the skirt's lower edge at the bottom of the bead
SKIRT_IN, SKIRT_OUT, SKIRT_H, TOP_T, HOLE_R = 0.0103, 0.0108, 0.0071, 0.0005, 0.0066
SEAL_SIDES = 32  # explicit convex collider; 0.052 mm maximum inward radial approximation


def _seal_shapes():
    """Collided as a 32-sided solid 21.5 mm x 7.6 mm convex prism (the skirt with the bead and stopper flange inside it, as one
    rigid piece), shown as the aluminum skirt and annular top. The stopper stays in the bottle's neck."""
    out = [parts.prism(SKIRT_OUT - 0.00005, SKIRT_H + TOP_T, SEAL_SIDES,
                       (0, 0, (SKIRT_H + TOP_T) / 2), ALU,
                       quat=(0.0, -math.sqrt(.5), 0.0, math.sqrt(.5)))]
    rm = (SKIRT_IN + SKIRT_OUT) / 2
    for k in range(16):
        a = 2 * math.pi * k / 16
        out.append(parts.box((SKIRT_OUT - SKIRT_IN, 2 * math.pi * rm / 16 * 1.04, SKIRT_H),
                             (rm * math.cos(a), rm * math.sin(a), SKIRT_H / 2), ALU, quat=_qz(a), collide=False))
    ra = (HOLE_R + SKIRT_OUT) / 2
    for k in range(12):
        a = 2 * math.pi * (k + 0.5) / 12
        out.append(parts.box((SKIRT_OUT - HOLE_R, 2 * math.pi * ra / 12 * 1.12, TOP_T),
                             (ra * math.cos(a), ra * math.sin(a), SKIRT_H + TOP_T / 2 + 0.00005), ALU,
                             quat=_qz(a), collide=False))
    return out


# --- WHEATON 224373 plier decapper: two crossed steel halves on one rivet, grey plastic-coated handles -------------
# Plier frame: origin at the rivet, lying flat (z up), jaws toward +y, handles toward -y. Written OPEN (joint 0):
# half A is the body, half B (the joint's link) is turned OPEN_ANGLE away. Joint +q closes the jaws.
JAW_Y, JAW_R, JAW_T, JAW_H = 0.024, 0.01085, 0.003, 0.007  # cradle centre, cradle radius, wall, thickness
HANDLE_H, HANDLE_W = 0.010, 0.013
OPEN_ANGLE = 0.55  # rad between the start (open) pose and jaws closed on the seal
CLOSED_STOP = OPEN_ANGLE + 0.17  # empty jaws close until their tips meet
RIVET_TAU = 0.48  # N m estimated static/sliding pin friction; unmeasured, replaces the prior damping cap
RIVET_D = 0.0  # no viscous drive; the native pin friction supplies passive resistance
CONTACT_SOLVER = (255, 64)  # numerical convergence probe, not a material parameter
PIVOT_Z = 0.005  # the rivet's height above the counter when the plier lies flat on its handles


def _half(side: float):
    """One half in the closed-on-seal pose: jaw on the -side*x... side=+1 is half A (jaw at -x, handle at +x)."""
    s = -side  # x sign of this half's jaw
    out = []
    rc = JAW_R + JAW_T / 2
    for th in np.radians([-50, -25, 0, 25, 50]):
        c = (s * rc * math.cos(th), JAW_Y + rc * math.sin(th), 0.0)
        out.append(parts.box((JAW_T, rc * math.radians(25) * 1.12, JAW_H), c, STEEL,
                             quat=_qz(math.atan2(c[1] - JAW_Y, c[0]))))
    out.append(parts.box((0.0045, 0.020, JAW_H), (s * 0.0158, 0.022, 0.0), STEEL))  # jaw back
    a, b = np.array((s * 0.0135, 0.0125)), np.array((-s * 0.0045, -0.011))  # through the rivet to the handle
    m, d = (a + b) / 2, b - a
    out.append(parts.box((float(np.linalg.norm(d)) + 0.004, 0.009, JAW_H), (m[0], m[1], 0.0), STEEL,
                         quat=_qz(math.atan2(d[1], d[0]))))
    h0, h1 = np.array((-s * 0.0045, -0.011)), np.array((-s * 0.0115, -0.032))  # steel shank, splaying out
    m, d = (h0 + h1) / 2, h1 - h0
    out.append(parts.box((0.009, float(np.linalg.norm(d)) + 0.004, JAW_H), (m[0], m[1], 0.0), STEEL,
                         quat=_qz(math.atan2(d[1], d[0]) - math.pi / 2)))
    out.append(parts.box((HANDLE_W, 0.090, HANDLE_H), (-s * 0.0130, -0.075, 0.0), GRIP))  # plastic-coated grip
    out.append(parts.cylinder(0.0075, JAW_H, (0, 0, 0), STEEL))  # rivet boss
    return out


def _turned(shapes, angle):
    """Shapes turned about the rivet (plier z axis) by ``angle``."""
    r = _rz(angle)
    out = []
    for sh in shapes:
        at = r @ np.asarray(sh.at)
        q = parts._matrix_to_quat(r @ sh.R)
        if sh.kind == "box":
            out.append(parts.box(sh.size, tuple(at), sh.color, quat=tuple(q), collide=sh.collide))
        else:
            out.append(parts.cylinder(sh.size[0], sh.size[1], tuple(at), sh.color, sh.axis, sh.collide))
    return out


HALF_A, HALF_B_CLOSED = _half(+1), _half(-1)

# --- PYREX 1000-250 250 mL low-form beaker (maker: about 68 mm OD x 90 mm) -----------------------------------------
BEAKER_R, BEAKER_H, BEAKER_WALL = 0.034, 0.090, 0.0018


def _beaker_shapes():
    out = [parts.cylinder(BEAKER_R, 0.003, (0, 0, 0.0015), BEAKER_GLASS)]
    rm = BEAKER_R - BEAKER_WALL / 2
    for k in range(28):
        a = 2 * math.pi * k / 28
        out.append(parts.box((BEAKER_WALL, 2 * math.pi * rm / 28 * 1.05, BEAKER_H - 0.003),
                             (rm * math.cos(a), rm * math.sin(a), 0.003 + (BEAKER_H - 0.003) / 2), BEAKER_GLASS,
                             quat=_qz(a)))
    return out


# --- layout A (the screenshot) ------------------------------------------------------------------------------------
BOTTLE_XY = (0.290, 0.050)
PLIER_XY = (0.250, -0.120)  # rivet
BEAKER_XY = (0.370, -0.045)

BOTTLE = parts.Part("serum_bottle", pos=(*BOTTLE_XY, TOP), mass=0.11, shapes=BOTTLE_SHAPES, friction=(0.7, 0.6),
                    contact_offset=0.002, report=("base",), solver=CONTACT_SOLVER)
SEAL = parts.Part("aluminum_seal", pos=(*BOTTLE_XY, TOP + SEAL_Z), mass=0.001, shapes=_seal_shapes(),
                  friction=(0.6, 0.5), contact_offset=0.001, report=("base",), solver=CONTACT_SOLVER)
PLIER = parts.Part(
    "plier_decapper", pos=(*PLIER_XY, TOP + PIVOT_Z), mass=0.08, shapes=HALF_A, friction=(0.9, 0.8),
    contact_offset=0.002, report=("base", "jaw"),
    joints=[parts.Hinge("jaw", shapes=tuple(_turned(HALF_B_CLOSED, -OPEN_ANGLE)), pivot=(0, 0, 0), axis=(0, 0, 1),
                        lower=-0.05, upper=CLOSED_STOP, mass=0.08, spring=0.0, damping=RIVET_D, max_effort=RIVET_TAU,
                        friction=RIVET_TAU, dynamic_friction=RIVET_TAU)],
)
BEAKER = parts.Part("waste_beaker", pos=(*BEAKER_XY, TOP), mass=0.11, shapes=_beaker_shapes(), friction=(0.7, 0.6),
                    contact_offset=0.002)
SHELF = parts.Part("reagent_shelf", pos=(0.640, 0.0, TOP), shapes=[
    parts.box((0.020, 0.80, 0.42), (0.020, 0.0, 0.21), SHELF_WHITE),
    parts.box((0.16, 0.80, 0.016), (-0.065, 0.0, 0.26), SHELF_WHITE),
    parts.box((0.16, 0.018, 0.26), (-0.065, 0.391, 0.13), SHELF_WHITE),
    parts.box((0.16, 0.018, 0.26), (-0.065, -0.391, 0.13), SHELF_WHITE),
])

OBJECTS = [BOTTLE, SEAL, PLIER, BEAKER, SHELF]

# --- layout B: the bottle and the beaker moved. The jointed plier is not moved between lots: a per-trial move of an
# articulated part landed it 12-19 mm off its target in earlier runs, so it starts where it is declared in both lots.
B_BOTTLE, B_BEAKER = (0.270, 0.080), (0.365, -0.015)
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={BOTTLE.name: BOTTLE.pos}),  # a zero move still runs the settle and a fresh first frame
    lots.Lot("B", place={BOTTLE.name: (*B_BOTTLE, TOP), SEAL.name: (*B_BOTTLE, TOP + SEAL_Z),
                         BEAKER.name: (*B_BEAKER, TOP)}),
)

# --- the crimp: a native joint between the bottle and the seal, and the scorer's record ----------------------------
# Single replaceable tear-torque ESTIMATE, not a measured decapper value. Mathaes2016 PDA70(1):12-29
# reports residual seal forces7-115 N. Assumed50 N, friction0.6, mean radius8.3 mm =>0.6*50*.0083~0.25 N m.
# The separate phone description of light handle/pivot effort is not a seal-tear measurement.
CRIMP_TAU = 0.25  # N m estimated equal/opposite seal-bottle resistance
CRIMP_K = 2.5  # reference N m/rad=0.25/0.1; assumed0.1 rad compliance (~1/16 quarter-turn)
DRIVE_GAIN_SCALE = 2.0  # native5 N m/rad caps at0.05 rad; credited-slip deadband is0.1 rad
CRIMP_D = 0.001  # N m s/rad numerical damping estimate
# Assumed slip path two skirt heights:2*.0071/.01075=1.32 rad, rounded UP to quarter-turn.
TEAR_TURN = math.pi / 2  # conservative unmeasured path assumption, not a maker-reported tear angle
# Coulomb bound per jaw:0.25/(2*0.5*0.01075)=23.26 N, rounded UP (mu0.5 is the model estimate).
GRIP_N = 25.0  # N per jaw, tighter than the prior10 N engagement gate
# Stress-scale proxy: SCHOTT BOROFLOAT33 reference25 MPa * assumed0.8 mm^2 patch=20 N.
GLASS_N = 20.0  # estimated proxy, not a WHEATON chip measurement; see README source/limits
FINGER_N = 0.5
HOLD_S = 1.0
REMOVED_GAP, HAND_GAP = 0.010, 0.030
REC: dict = parts.__dict__.setdefault("_seal_record", {})


def reset_record() -> None:
    REC.clear()
    REC.update(known=False, zero=False, why="", released=False, turn=0.0, phi=None, plastic=0.0, q_crimp=0.0, rivet=0.0, still_s=0.0,
               clear_s=0.0, t=None, checked=False, t_release=None,
               peak={"jawA": 0.0, "jawB": 0.0, "glass": 0.0, "finger": 0.0}, work_readback=[])


reset_record()


def _R(s) -> np.ndarray:
    return parts._quat_to_matrix(s.quat_xyzw)


def tilt_deg(s) -> float:
    return math.degrees(math.acos(float(np.clip(_R(s)[2, 2], -1.0, 1.0))))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def seal_yaw(bottle, seal) -> float:
    m = _R(bottle).T @ _R(seal)
    return math.atan2(m[1, 0], m[0, 0])


def _cyl_dist(p, r, z0, z1) -> float:
    """Distance from a point (bottle frame) to a solid z-cylinder."""
    dr = max(math.hypot(p[0], p[1]) - r, 0.0)
    dz = max(z0 - p[2], p[2] - z1, 0.0)
    return math.hypot(dr, dz)


SEAL_BOUND = math.hypot(SKIRT_OUT, (SKIRT_H + TOP_T) / 2)  # sphere round the seal's centre


def seal_gap(bottle, seal) -> float:
    """Clearance between the seal (its bounding sphere) and the bottle's glass and stopper."""
    c = _R(bottle).T @ (seal.pos + _R(seal) @ np.array((0, 0, (SKIRT_H + TOP_T) / 2)) - bottle.pos)
    d = min(_cyl_dist(c, sh.size[0], sh.at[2] - sh.size[1] / 2, sh.at[2] + sh.size[1] / 2) for sh in BOTTLE_SHAPES)
    return d - SEAL_BOUND


def _seal_points(seal) -> np.ndarray:
    """Points round the seal's outer rim, top and bottom (world)."""
    a = np.linspace(0, 2 * math.pi, 36, endpoint=False)
    ring = np.stack([SKIRT_OUT * np.cos(a), SKIRT_OUT * np.sin(a), np.zeros_like(a)], axis=1)
    pts = np.concatenate([ring, ring + (0.0, 0.0, SKIRT_H + TOP_T)])
    return pts @ _R(seal).T + seal.pos


def standing(s, tol=0.003) -> bool:
    return tilt_deg(s) <= 10.0 + EPS and abs(float(s.pos[2]) - TOP) <= tol + EPS


def in_beaker(beaker, seal) -> bool:
    if not standing(beaker):
        return False
    loc = (_seal_points(seal) - beaker.pos) @ _R(beaker)
    return bool(np.hypot(loc[:, 0], loc[:, 1]).max() <= BEAKER_R - BEAKER_WALL + EPS
                and loc[:, 2].min() >= 0.003 - 0.001 - EPS and loc[:, 2].max() <= BEAKER_H + EPS)


def plier_down(p) -> bool:
    return tilt_deg(p) <= 15.0 + EPS and -0.002 - EPS <= float(p.low[2]) - TOP <= 0.004 + EPS


def level(o) -> float:
    """The endpoint tier the current states show (the crimp and hands-clear history come from the record)."""
    b, s, p = o[BOTTLE.name], o[SEAL.name], o[PLIER.name]
    if not standing(b) or not REC.get("released"):
        return 0.0
    if seal_gap(b, s) < REMOVED_GAP - EPS:
        return 0.25
    if not in_beaker(o[BEAKER.name], s):
        return 0.5
    if plier_down(p) and REC.get("clear_s", 0.0) >= HOLD_S - EPS:
        return 1.0
    return 0.75


def score(objects) -> float:
    """The tier the end state shows, once the bottle, seal, beaker and plier have been still for HOLD_S."""
    try:
        if not REC.get("known") or REC.get("zero") or REC.get("still_s", 0.0) < HOLD_S - EPS:
            return 0.0
        if not all(objects[n].resting(0.005, 0.1) for n in (BOTTLE.name, SEAL.name, PLIER.name, BEAKER.name)):
            return 0.0
        return float(level(objects))
    except (KeyError, TypeError, ValueError, AttributeError):
        return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


def _zero(why: str) -> None:
    if not REC["zero"]:
        print(f"[decapper] score 0 for this trial: {why}", file=sys.stderr, flush=True)
    REC.update(zero=True, why=REC["why"] or why)


def update(o, t: float, jaw_a: float, jaw_b: float, finger: float, glass: float, clear: bool) -> bool:
    """One control step of history, from the states and the contact forces (N) the simulator reports: the
    whole-trial rules, the crimp, and the still and hands-clear timers. True while the crimp holds (the simulator's
    bottle-seal joint stays on until then).

    Credited slip ignores deflection up to CRIMP_TAU / CRIMP_K (twice the native force-cap deflection). Beyond it, the reference target follows
    the relative angle and adds absolute plastic turn only while both jaws press with GRIP_N or more. The angular
    force drive resists that path with the unchanged CRIMP_TAU cap. Turning bottle and seal together adds nothing.
    This callback never writes either body's pose or velocity."""
    dt = 0.05 if REC["t"] is None else max(0.0, t - REC["t"])
    pk = REC["peak"]
    for k, v in (("jawA", jaw_a), ("jawB", jaw_b), ("finger", finger), ("glass", glass)):
        pk[k] = max(pk[k], float(v))
    b, s, p, w = o[BOTTLE.name], o[SEAL.name], o[PLIER.name], o[BEAKER.name]
    if finger > FINGER_N + EPS:
        _zero(f"a robot finger pressed the seal ({finger:.2f} N)")
    if glass > GLASS_N + EPS:
        _zero(f"the plier pressed the glass with {glass:.1f} N")
    if tilt_deg(b) > 45.0 + EPS or tilt_deg(w) > 45.0 + EPS:
        _zero("glassware fell over")
    if not REC["released"]:
        now = seal_yaw(b, s)
        if REC["phi"] is not None:
            REC["q_crimp"] += _wrap(now - REC["phi"])
        REC["phi"] = now
        delta = REC["q_crimp"] - REC["plastic"]
        yield_angle = CRIMP_TAU / CRIMP_K
        if abs(delta) > yield_angle:
            dp = math.copysign(abs(delta) - yield_angle, delta)
            REC["plastic"] += dp
            if jaw_a >= GRIP_N - EPS and jaw_b >= GRIP_N - EPS:
                REC["turn"] += abs(dp)
        if REC["turn"] >= TEAR_TURN - EPS:
            REC.update(released=True, t_release=t)
            print(f"[decapper] crimp torn at t={t:.2f}s after {math.degrees(REC['turn']):.0f} deg of slip",
                  file=sys.stderr, flush=True)
    still = all(x.resting(0.005, 0.1) for x in (b, s, p, w))
    REC["still_s"] = REC["still_s"] + dt if still else 0.0
    REC["clear_s"] = REC["clear_s"] + dt if clear else 0.0
    REC.update(t=t, known=True)
    return not REC["released"]


# --- simulator side: start check, crimp, rivet friction, contacts, hands --------------------------------------------
HAND_LINKS = ("gripper", "tip_left", "tip_right")
_SIM: dict = {}
_FACE_PTS: dict = {}


def _arr(x) -> np.ndarray:
    """A torch tensor or warp array from the simulator as a NumPy array."""
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
    hands = []

    def walk(prim):
        for c in prim.GetAllChildren():
            yield c
            yield from walk(c)

    for prim in get_current_stage().GetPrimAtPath(env).GetAllChildren():
        if "Arm" in prim.GetName():
            hands += [p.GetPath().pathString for p in walk(prim)
                      if p.GetName() in HAND_LINKS and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(hands) != 6:
        raise RuntimeError(f"found {len(hands)} hand links, expected 6")
    jaws = [f"{parts._PATHS[PLIER.name]}/base", f"{parts._PATHS[PLIER.name]}/jaw"]
    seal = sv.create_rigid_contact_view(parts._PATHS[SEAL.name], filter_patterns=jaws + hands,
                                        max_contact_data_count=128)
    glass = sv.create_rigid_contact_view(parts._PATHS[BOTTLE.name], filter_patterns=jaws, max_contact_data_count=128)
    if os.environ.get("TRACE_OUT"):
        try:
            _SIM["tool_hands"] = sv.create_rigid_contact_view(
                f"{parts._PATHS[PLIER.name]}/*", filter_patterns=hands, max_contact_data_count=128)
            _SIM["hand_paths"] = hands
        except Exception as exc:
            print(f"[decapper] handle diagnostic unavailable: {exc}", file=sys.stderr, flush=True)
    return seal, glass


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
    """Every hand point at least ``gap`` from every shape box of the bottle, seal, plier and beaker."""
    from yam_arena import analysis

    for spec in (BOTTLE, SEAL, PLIER, BEAKER):
        st = o[spec.name]
        if float(np.linalg.norm(np.maximum(np.maximum(st.low - points, points - st.high), 0.0), axis=1).min()) >= gap:
            continue  # every hand point is farther than gap from this object's whole box
        poses = {"base": (st.pos, _R(st))}
        poses.update({k: (np.asarray(v[0]), parts._quat_to_matrix(v[1])) for k, v in getattr(st, "links", {}).items()})
        for _, c, r, size in spec.world_boxes(poses):
            if float(analysis.box_signed_distance(points, c, r, size).min()) < gap - EPS:
                return False
    return True


_CRIMP: dict = {}  # the bottle-seal joint's prim path, and whether it is switched on


def _stage():
    from isaaclab.sim.utils import get_current_stage

    return get_current_stage()


def _author_crimp(seal_path: str) -> None:
    """The crimp as a native PhysX joint between the bottle and the seal, written with the scene (before the
    simulation starts): a revolute joint about the bottle's axis at the bottom of the skirt (every other relative
    motion locked, no collision between the two) whose elastic angular force drive has stiffness CRIMP_K and damping CRIMP_D, capped at
    CRIMP_TAU. The solver applies the resisting torque to the seal and the equal and opposite torque to the bottle."""
    from pxr import Gf, Sdf, UsdPhysics

    stage = _stage()
    path = f"{seal_path.rsplit('/', 1)[0]}/crimp_joint"
    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    j.CreateAxisAttr("Z")
    j.CreateBody0Rel().SetTargets([Sdf.Path(parts._PATHS[BOTTLE.name])])
    j.CreateBody1Rel().SetTargets([Sdf.Path(seal_path)])
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, float(SEAL_Z)))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    j.CreateCollisionEnabledAttr().Set(False)
    j.CreateExcludeFromArticulationAttr().Set(True)
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(DRIVE_GAIN_SCALE * CRIMP_K * math.pi / 180)
    drive.CreateDampingAttr(CRIMP_D * math.pi / 180)  # USD angular drive gains are per degree
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateTargetVelocityAttr(0.0)
    drive.CreateMaxForceAttr(CRIMP_TAU)  # N m, unchanged estimate
    j.CreateJointEnabledAttr().Set(True)
    _CRIMP.update(path=path, on=True)
    print(f"[decapper] crimp joint {path}: {parts._PATHS[BOTTLE.name]} - {seal_path}, resists {CRIMP_TAU} N m",
          file=sys.stderr, flush=True)


_author_parts = parts._author


def _author_with_crimp(part, root_path, translation, orientation):
    root = _author_parts(part, root_path, translation, orientation)
    if part.name == PLIER.name:
        from pxr import PhysxSchema
        art = PhysxSchema.PhysxArticulationAPI.Get(_stage(), root_path + "/base")
        art.GetSolverPositionIterationCountAttr().Set(CONTACT_SOLVER[0])
        art.GetSolverVelocityIterationCountAttr().Set(CONTACT_SOLVER[1])
    if part.name == SEAL.name:  # the bottle is written first (OBJECTS order), so both bodies exist now
        _author_crimp(root_path)
    return root


parts._author = _author_with_crimp


def _set_crimp(on: bool) -> None:
    """Switch the crimp joint on (a new trial: the reset puts the seal back on the finish) or off (the crimp tore:
    the seal is a loose part held only by what touches it)."""
    from pxr import UsdPhysics

    if _CRIMP.get("path") and _CRIMP.get("on") != on:
        UsdPhysics.Joint.Get(_stage(), _CRIMP["path"]).GetJointEnabledAttr().Set(bool(on))
        _CRIMP["on"] = on


def _prepare_crimp_reset():
    """Reset only the constraint before the stock reset places both bodies. No in-trial pose writes."""
    if _CRIMP.get("path"):
        from pxr import UsdPhysics
        drive = UsdPhysics.DriveAPI(_stage().GetPrimAtPath(_CRIMP["path"]), "angular")
        drive.GetTargetPositionAttr().Set(0.0)
        _set_crimp(True)


lots.before_reset = _prepare_crimp_reset


def jaw_torque(base, seal_view, o, components=False):
    """Torque (N m) the two plier jaws put on the seal about the bottle's axis, from the contact normal and friction
    forces PhysX reports at each jaw contact point. The seal's 1 g has next to no inertia, so while the crimp holds
    this is the torque the crimp joint resists with (and passes to the bottle). Logged, not scored."""
    pdt = base.physics_dt
    origin = parts._np(base.scene.env_origins[0])
    b = o[BOTTLE.name]
    rb = _R(b)
    c, ax = b.pos + rb @ np.array((0.0, 0.0, SEAL_Z)) + origin, rb[:, 2]
    fn, pn, nn, _, cn, sn = (_arr(x) for x in seal_view.get_contact_data(dt=pdt))
    ff, pf, cf, sf = (_arr(x) for x in seal_view.get_friction_data(dt=pdt))
    cn, sn, cf, sf = cn.reshape(-1), sn.reshape(-1), cf.reshape(-1), sf.reshape(-1)
    fn, pn, nn = fn.reshape(-1), pn.reshape(-1, 3), nn.reshape(-1, 3)
    ff, pf = ff.reshape(-1, 3), pf.reshape(-1, 3)
    tau_n, tau_f = 0.0, 0.0
    for k in (0, 1):  # the two plier halves
        i, n = int(sn[k]), int(cn[k])
        for f, pt in zip(fn[i:i + n, None] * nn[i:i + n], pn[i:i + n]):
            tau_n += float(np.cross(pt - c, f) @ ax)
        i, n = int(sf[k]), int(cf[k])
        for f, pt in zip(ff[i:i + n], pf[i:i + n]):
            tau_f += float(np.cross(pt - c, f) @ ax)
    return (tau_n, tau_f) if components else tau_n + tau_f


def _trace_physics(sim, seal_view):
    """Read every physics substep during traces; diagnostic only, no forces or state writes."""
    if not os.environ.get("TRACE_OUT") or "physics_sub" in _SIM:
        return
    import json
    import omni.physx

    elapsed = [float(sim.t)]

    def read(dt):
        elapsed[0] += float(dt)
        try:
            base = sim.env.unwrapped
            o = sim.states()
            row = dict(t=elapsed[0], tau=jaw_torque(base, seal_view, o),
                       tau_parts=jaw_torque(base, seal_view, o, components=True),
                       jaw=(_arr(seal_view.get_contact_force_matrix(dt=base.physics_dt))).reshape(-1, 3)[:2].tolist(),
                       released=bool(REC["released"]), lot=lots.current().name)
            with pathlib.Path(os.environ["TRACE_OUT"]).with_name("crimp-physics.jsonl").open("a") as f:
                f.write(json.dumps(row, default=float) + "\n")
        except Exception as exc:
            if not _SIM.get("physics_log_error"):
                _SIM["physics_log_error"] = True
                print(f"[decapper] physics trace unavailable: {exc}", file=sys.stderr, flush=True)

    _SIM["physics_sub"] = omni.physx.get_physx_interface().subscribe_physics_on_step_events(read, False, 100)


def _start_ok(o) -> str:
    lot = lots.current()
    for spec in (BOTTLE, PLIER, BEAKER):
        err = float(np.linalg.norm(o[spec.name].pos[:2] - np.asarray(lot.pos(spec.name))[:2]))
        if err > 0.004 + EPS:
            return f"{spec.name} starts {err * 1000:.1f} mm from lot {lot.name}"
    yaw = math.atan2(_R(o[PLIER.name])[1, 0], _R(o[PLIER.name])[0, 0])
    if abs(_wrap(yaw - lot.yaw.get(PLIER.name, 0.0))) > math.radians(3):
        return f"plier starts turned {math.degrees(yaw):.1f} deg in lot {lot.name}"
    return ""


@parts.on_reset
def _on_reset(sim) -> None:
    _prepare_crimp_reset()
    reset_record()
    _SIM.clear()


@parts.on_step
def _on_step(sim) -> None:
    row = {}
    try:
        base = sim.env.unwrapped
        o = sim.states()
        t = float(sim.t)
        if not REC["checked"]:
            REC["checked"] = True
            if os.environ.get("TRACE_OUT"):
                prim = _stage().GetPrimAtPath(_CRIMP.get("path", "/none"))
                print(f"[decapper] crimp state {_CRIMP}, prim valid {prim.IsValid()}, "
                      f"{[str(a.GetName()) + '=' + str(a.Get()) for a in prim.GetAttributes()] if prim.IsValid() else ''}",
                      file=sys.stderr, flush=True)
            why = _start_ok(o)
            print(f"[decapper] trial start lot {lots.current().name}: "
                  + ", ".join(f"{n} {np.round(o[n].pos, 4).tolist()}" for n in (BOTTLE.name, PLIER.name, BEAKER.name)),
                  file=sys.stderr, flush=True)
            if why:
                _zero("start: " + why)
        if "views" not in _SIM:
            _SIM["views"] = _views(base)
        seal_view, glass_view = _SIM["views"]
        _trace_physics(sim, seal_view)
        pdt = base.physics_dt
        fs = np.linalg.norm(_arr(seal_view.get_contact_force_matrix(dt=pdt)).reshape(-1, 3), axis=1)
        fg = np.linalg.norm(_arr(glass_view.get_contact_force_matrix(dt=pdt)).reshape(-1, 3), axis=1)
        if fs.shape != (8,) or fg.shape != (2,) or not (np.isfinite(fs).all() and np.isfinite(fg).all()):
            raise RuntimeError(f"contact data shapes {fs.shape} {fg.shape}")
        jaw_a, jaw_b, finger, glass = float(fs[0]), float(fs[1]), float(fs[2:].max()), float(fg.max())
        clear = hands_clear(hand_points(base), o, HAND_GAP)
        try:
            tau_parts = jaw_torque(base, seal_view, o, components=True)
            tau = sum(tau_parts)
        except Exception as exc:  # noqa: BLE001  logging only
            tau, tau_parts = None, None
            if not _SIM.get("tauerr"):
                _SIM["tauerr"] = True
                print(f"[decapper] jaw torque not read: {exc}", file=sys.stderr, flush=True)
        before_turn, before_release = REC["turn"], REC["released"]
        if not update(o, t, jaw_a, jaw_b, finger, glass, clear):
            _set_crimp(False)
        # Read-only force evidence in every mode, including each final demo lot.
        if REC["turn"] > before_turn and tau is not None:
            rb = o[BOTTLE.name]
            by = math.degrees(math.atan2(_R(rb)[1, 0], _R(rb)[0, 0]))
            REC["work_readback"].append((abs(tau), jaw_a, jaw_b, by, tilt_deg(rb)))
        if not before_release and REC["released"]:
            a = np.asarray(REC["work_readback"], dtype=float)
            if len(a):
                print(f"[decapper] native work readback lot {lots.current().name}: n={len(a)}, "
                      f"mean_abs_tau={a[:,0].mean():.6f} N m, range={a[:,0].min():.6f}..{a[:,0].max():.6f}, "
                      f"min_jaws={a[:,1].min():.3f}/{a[:,2].min():.3f} N, "
                      f"bottle_yaw={a[:,3].min():.3f}..{a[:,3].max():.3f} deg, max_tilt={a[:,4].max():.3f} deg",
                      file=sys.stderr, flush=True)
        if _CRIMP.get("on"):
            from pxr import UsdPhysics
            drive = UsdPhysics.DriveAPI(_stage().GetPrimAtPath(_CRIMP["path"]), "angular")
            drive.GetTargetPositionAttr().Set(math.degrees(REC["plastic"]))
        # Passive friction pin: no stored spring energy or angle target. Physical contact must open it.
        b, s, p = o[BOTTLE.name], o[SEAL.name], o[PLIER.name]
        q = float(p.joints["jaw"])
        lv = level(o)
        row = dict(t=t, tau=tau, tau_parts=tau_parts, plastic_deg=math.degrees(REC["plastic"]), q_crimp=REC["q_crimp"], jawA=jaw_a, jawB=jaw_b, finger=finger, glass=glass,
                   turn_deg=math.degrees(REC["turn"]), released=bool(REC["released"]), q=q, rivet=REC["rivet"],
                   seal_yaw=math.degrees(seal_yaw(b, s)), bottle_yaw=math.degrees(math.atan2(_R(b)[1, 0], _R(b)[0, 0])),
                   seal_w=float(np.asarray(s.ang_vel)[2]) if getattr(s, "ang_vel", None) is not None else None,
                   bottle_w=float(np.asarray(b.ang_vel)[2]) if getattr(b, "ang_vel", None) is not None else None,
                   level=lv, still_s=REC["still_s"], clear_s=REC["clear_s"], zero=bool(REC["zero"]),
                   bottle=b.pos.tolist(), bottle_tilt=tilt_deg(b), seal=s.pos.tolist(), plier=p.pos.tolist(),
                   plier_yaw=math.degrees(math.atan2(_R(p)[1, 0], _R(p)[0, 0])), plier_tilt=tilt_deg(p),
                   seal_gap=seal_gap(b, s), beaker=o[BEAKER.name].pos.tolist(),
                   cradle_a=(p.pos + _R(p) @ np.array((0.0, JAW_Y, 0.0))).tolist(),
                   cradle_b=(np.asarray(p.links["jaw"][0]) + parts._quat_to_matrix(p.links["jaw"][1])
                             @ (_rz(-OPEN_ANGLE) @ np.array((0.0, JAW_Y, 0.0)))).tolist())
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
            if "views" in _SIM:
                sv = _SIM["views"][0]
                fn, pts, normals, sep, counts, starts = (_arr(x) for x in sv.get_contact_data(dt=base.physics_dt))
                sep, counts, starts = sep.reshape(-1), counts.reshape(-1), starts.reshape(-1)
                row["jaw_sep_min_mm"] = [float(sep[int(starts[k]):int(starts[k])+int(counts[k])].min()) * 1000
                                         if int(counts[k]) else None for k in (0, 1)]
                row["seal_quat"] = list(o[SEAL.name].quat_xyzw)
                row["plier_quat"] = list(o[PLIER.name].quat_xyzw)
                row["jaw_link_pose"] = [list(x) for x in o[PLIER.name].links["jaw"]]
            if "tool_hands" in _SIM:
                forces = _arr(_SIM["tool_hands"].get_contact_force_matrix(dt=base.physics_dt))
                row["tool_hand_N"] = np.linalg.norm(forces.reshape(-1, 6, 3), axis=2).tolist()
                row["tool_hand_paths"] = _SIM["hand_paths"]
                row["rivet_velocity"] = _arr(base.scene[PLIER.name].data.joint_vel).tolist()
            for arm in ("left_robot", "right_robot"):
                rob = base.scene[arm]
                pose = parts._np(rob.data.body_link_pose_w)[0, rob.body_names.index("gripper")]
                row[arm] = (pose[:3] - origin + parts._quat_to_matrix(pose[3:]) @ np.array(TCP_OFFSET)).tolist()
                row[arm + "_quat"] = pose[3:].tolist()
                row[arm + "_fingers"] = parts._np(rob.data.joint_pos)[0, -2:].tolist()
                row[arm + "_hand_poses"] = {n: parts._np(rob.data.body_link_pose_w)[0, rob.body_names.index(n)].tolist() for n in HAND_LINKS}
            row["lot"] = lots.current().name
            with pathlib.Path(os.environ["TRACE_OUT"]).with_name("decapper.jsonl").open("a") as f:
                f.write(json.dumps(row, default=float) + "\n")
        except Exception as exc:  # noqa: BLE001  logging never affects the trial
            if not _SIM.get("logerr"):
                _SIM["logerr"] = True
                print(f"[decapper] trace logging skipped: {exc}", file=sys.stderr, flush=True)


# --- scripted demo: open loop from the visible layout ---------------------------------------------------------------
GRIP_L = 0.045  # m behind the rivet, along the plier's midline, where the hand holds the handles
TCP_ABOVE = 0.011  # the hand's centre point above the plier's mid-plane when gripping the handles
DOCK_FIX = (0.003, 0.002, 0.002)  # m: measured original 45 mm grasp approach, revalidation pending
LEFT_FIX = (0.0, 0.0, 0.0)


def _span(q: float, L: float = GRIP_L, grip_only: bool = True) -> float:
    """Outer width across both handles, L behind the rivet, square to the midline, at joint position q."""
    bis = -(OPEN_ANGLE - q) / 2
    d = np.array((-math.sin(bis), math.cos(bis)))
    perp = np.array((d[1], -d[0]))
    pts = []
    for sh in HALF_A + _turned(HALF_B_CLOSED, -OPEN_ANGLE + q):
        if (grip_only and sh.color != GRIP) or sh.kind != "box":
            continue
        g = np.stack(np.meshgrid(np.linspace(-.5, .5, 14), np.linspace(-.5, .5, 240), [0.0]), -1).reshape(-1, 3)
        pts.append((np.asarray(sh.at) + (g * np.asarray(sh.size)) @ sh.R.T)[:, :2])
    pts = np.concatenate(pts)
    sel = np.abs(pts @ d + L) < 0.003
    s = pts[sel] @ perp
    return float(s.max() - s.min())


SPAN_OPEN, SPAN_SEAL = _span(0.0), _span(OPEN_ANGLE)


def _finger(span: float, press: float) -> float:
    """Finger target that pushes each pad ``press`` metres past the handles (5 N per mm)."""
    return float(np.clip(-(span / 2 - press), FINGER_OPEN, 0.0))


def demo():
    lot = lots.current()
    b, pv, w = (np.asarray(lot.pos(n)) for n in (BOTTLE.name, PLIER.name, BEAKER.name))
    pyaw = lot.yaw.get(PLIER.name, 0.0)
    fix, up = np.asarray(DOCK_FIX), np.array((0.0, 0.0, 1.0))
    mid = pyaw + math.pi / 2 - OPEN_ANGLE / 2
    direction = np.array((math.cos(mid), math.sin(mid), 0.0))
    prelift, support_l = 0.0, 0.040
    support_comp = .009  # m: left loaded-hand shortfall measured in trace 224240
    grip = pv - direction * GRIP_L
    grip[2] = TOP + PIVOT_Z + TCP_ABOVE + prelift
    support = pv - direction * support_l
    support[2] = TOP + PIVOT_Z + TCP_ABOVE
    hold, squeeze = _finger(SPAN_OPEN, .0016), _finger(SPAN_SEAL, .007)
    relax = _finger(SPAN_SEAL, .0008)
    support_f = _finger(_span(.15, L=support_l, grip_only=False), .002)
    support_squeeze = _finger(_span(OPEN_ANGLE, L=support_l, grip_only=False), .002)
    axis = np.array((b[0], b[1], TOP + SEAL_Z + SKIRT_H / 2 + TCP_ABOVE))
    reach = JAW_Y + GRIP_L

    def wp(t, pos, f, label, yaw):
        return Waypoint(float(t), tuple(float(v) for v in pos), float(f), label, float(yaw))

    def sw(t, pos, f, label, yaw=-math.pi / 2):
        return tilt.Waypoint(float(t), tuple(float(v) for v in pos), float(f), label,
                             yaw=float(yaw), tilt=30, heading=float(yaw))

    def bw(t, pos, f, label):
        return tilt.Waypoint(float(t), tuple(float(v) for v in pos), float(f), label,
                             yaw=-math.pi / 2, tilt=40, heading=-math.pi / 2)

    def around(gamma, dz=0.0):
        centre = axis  # the bottle is held before clamping
        offset = np.array((fix[0], -reach + fix[1], 0.0))
        return centre + _rz(-gamma) @ offset + up * (fix[2] + dz)

    support_over = around(0.0, .035) + np.array((0.0, GRIP_L - support_l, 0.0)) + up * support_comp
    support_dock = around(0.0) + np.array((0.0, GRIP_L - support_l, 0.0)) + up * support_comp
    over = np.array((w[0], w[1] - reach, TOP + .130 + TCP_ABOVE)) + fix
    # The right hand picks near the tool COM; the left holds the bottle before clamping.
    R = [wp(2.5, grip + up * .08, FINGER_OPEN, "over the raised handles", mid),
         wp(1.5, grip, FINGER_OPEN, "down to the raised handles", mid),
         wp(1.0, grip, hold, "hold the open handles", mid),
         wp(2.5, grip + up * .125, hold, "lift the open plier", mid),
         wp(3.0, around(0.0, .035), hold, "open jaws over the seal", math.pi / 2),
         wp(1.5, around(0.0), hold, "lower the jaws round the seal", math.pi / 2),
         wp(4.0, around(0.0), squeeze, "squeeze the handles", math.pi / 2),
         wp(.5, around(0.0), squeeze, "settle", math.pi / 2),
         wp(5.0, around(0.0), squeeze, "wait for the bottle grip", math.pi / 2)]
    gam = math.radians(65)
    for sweep in (() if os.environ.get("DECAP_DEMO") == "lift" else (1, -1)):
        for k in range(1, 14):
            g = gam * (k / 13 if sweep > 0 else 1 - k / 13)
            R.append(wp(.3, around(g), squeeze, "twist the seal", math.pi / 2 - g))
        if sweep > 0:
            R.append(wp(1.5, around(gam), squeeze, "hold the outward twist", math.pi / 2 - gam))
    # Fixed recovery geometry from native trace 20261124T000053, relative to the beaker.
    c45 = math.sqrt(.5)
    tool_r = np.array(((1., 0., 0.), (0., c45, -c45), (0., c45, c45)))
    rest_rivet = w + np.array((0.0, -.0401, .0889))
    release_l = .020
    support_release = rest_rivet + tool_r @ np.array((0., -release_l, 0.)) + up * TCP_ABOVE
    release_f = _finger(_span(.57, L=release_l, grip_only=False), .002)
    cargo_shift = np.array((0., .010, .020))
    loaded_release = support_release + cargo_shift + up * support_comp
    grasp_b = w + np.array((-.0185, -.0875, .0498)) + tool_r @ (up * TCP_ABOVE)  # pad offset follows tool normal
    single_approach = _finger(.020, 0.)  # entry clearance for native tracking error; close after enclosing B
    single_f = 0.0  # full close on the isolated handle, stock actuator effort limit unchanged
    pivot = rest_rivet + cargo_shift
    opening = .25
    open_r = tool_r @ _rz(-opening) @ tool_r.T
    spread_b = pivot + open_r @ (grasp_b - pivot)
    repose_yaw = math.radians(-40)
    repose_r = _rz(repose_yaw) @ tool_r
    repose_root = w + np.array((-.030, -.012, .090))
    repose_b = repose_root + repose_r @ np.array((-.013, -.080, TCP_ABOVE)) + np.array((.0003, .0007, .0022))
    repose_a = repose_root + repose_r @ np.array((.013, -.040, TCP_ABOVE))
    repose_open = repose_r @ _rz(-opening) @ repose_r.T
    repose_spread = repose_root + repose_open @ (repose_b - repose_root)
    right_park = np.array((.25, -.28, TOP + .180))

    def rw(t, pos, f, label):
        return tilt.Waypoint(float(t), tuple(pos), f, label, yaw=math.pi / 2, tilt=45, heading=math.pi / 2)

    def ow(t, pos, f, label):
        point = rw(t, pos, f, label)
        point.yaw = tilt._TiltedYaw(math.pi / 2 - opening,
                                   open_r @ tilt.rotation(math.pi / 2, 45, math.pi / 2))
        return point

    def cw(t, pos, f, label):
        yaw = math.pi / 2 + repose_yaw
        return tilt.Waypoint(float(t), tuple(pos), float(f), label, yaw=yaw, tilt=45, heading=yaw)

    def co(t, pos, f, label):
        point = cw(t, pos, f, label)
        point.yaw = tilt._TiltedYaw(math.pi / 2 + repose_yaw - opening,
                                   repose_open @ tilt.rotation(math.pi / 2 + repose_yaw, 45, math.pi / 2 + repose_yaw))
        return point

    a_grip_fix = np.array((.006, .004, -.003))  # native v43 hand-pose correction, not a physical parameter

    def aw(t, pos, f, label, lean=6):
        return tilt.Waypoint(float(t), tuple(np.asarray(pos) + a_grip_fix), float(f), label,
                             yaw=-math.pi / 2 + repose_yaw, tilt=lean, heading=-math.pi / 2)

    R += [wp(.5, around(0.0), squeeze, "hold", math.pi / 2),
          wp(2.5, around(0.0), squeeze, "wait for the other hand to clear", math.pi / 2),
          wp(1.5, around(0.0, .045), squeeze, "lift the seal off", math.pi / 2),
          wp(3., over, squeeze, "jaws over the waste beaker", math.pi / 2),
          wp(1., over, relax, "ease the squeeze", math.pi / 2),
          wp(1.5, over, relax, "let the tool settle on the rim", math.pi / 2),
          wp(1.5, right_park, FINGER_OPEN, "clear for the support grasp", math.pi / 2),
          wp(5., right_park, FINGER_OPEN, "wait for the support and lift", math.pi / 2),
          rw(1.5, grasp_b + up * .06, single_approach, "over the other handle"),
          rw(1.5, grasp_b, single_approach, "straddle one handle"),
          rw(1., grasp_b, single_f, "grip one handle"),
          rw(1., grasp_b, single_f, "settle the handle grip"),
          rw(1., grasp_b, single_f, "wait for bridged support to clear"),
          cw(4., repose_b, single_f, "reposition the held tool over the beaker"),
          cw(6.5, repose_b, single_f, "wait for isolated A capture"),
          co(2., repose_spread, single_f, "spread the isolated handles physically"),
          co(1.5, repose_spread, single_f, "let the freed seal fall"),
          co(1., repose_spread, FINGER_OPEN, "let go of the spread handle"),
          wp(1.5, right_park, FINGER_OPEN, "right hand clear", math.pi / 2),
          wp(13., right_park, FINGER_OPEN, "right hand stays clear", math.pi / 2)]

    body = b + np.asarray(LEFT_FIX) + up * .034
    if os.environ.get("DECAP_DEMO") in ("onehand", "lift"):
        park = body + np.array((-.05, .10, .10))
        return {"left": [bw(2., park, FINGER_OPEN, "clear the bottle"),
                         bw(40., park, FINGER_OPEN, "no bottle holding hand")], "right": R}
    L = [bw(3.5, body + np.array((0., .03, .06)), FINGER_OPEN, "over the bottle"),
         bw(1.5, body, FINGER_OPEN, "down to the bottle"),
         bw(1., body, -.023, "hold the bottle")]
    off = next(i for i, x in enumerate(R) if x.label == "hold") + 1
    rest = sum(x.duration for x in R[:off]) - sum(x.duration for x in L)
    assert rest >= 0.0
    L += [bw(rest, body, -.023, "steady the bottle"),
          bw(1., body, FINGER_OPEN, "let go"),
          bw(1.5, body + np.array((0.0, .06, .06)), FINGER_OPEN, "hand clear")]
    rim_time = sum(x.duration for x in R[:next(i for i, x in enumerate(R) if x.label == "let the tool settle on the rim") + 1])
    L += [bw(rim_time - sum(x.duration for x in L), body + np.array((0., .06, .06)), FINGER_OPEN, "wait for the resting tool"),
          sw(2., support_release + up * .06, FINGER_OPEN, "over the steel near the rivet"),
          sw(1.5, support_release, FINGER_OPEN, "straddle the steel near the rivet"),
          sw(1., support_release, release_f, "hold near the rivet"),
          sw(2., loaded_release, release_f, "lift and centre the tool over the beaker")]
    isolate_time = sum(x.duration for x in R[:next(i for i, x in enumerate(R) if x.label == "settle the handle grip") + 1])
    L += [sw(isolate_time - sum(x.duration for x in L), loaded_release, release_f, "wait for B capture"),
          sw(1., loaded_release + up * .040, FINGER_OPEN, "release the bridging support"),
          sw(4., loaded_release + up * .040, FINGER_OPEN, "clear the tool reposition"),
          aw(2., repose_a + up * .040, single_approach, "over A only", lean=30),
          aw(2., repose_a, single_approach, "straddle A only"),
          aw(1.5, repose_a, 0., "capture A only"),
          aw(1., repose_a, 0., "settle isolated A grip")]
    release_time = sum(x.duration for x in R[:next(i for i, x in enumerate(R) if x.label == "let go of the spread handle") + 1])
    counter_root = np.array((pv[0] + .030, pv[1] + .070, TOP + .0889))
    counter_grip = counter_root + repose_r @ np.array((.013, -.040, TCP_ABOVE))
    L += [aw(release_time - sum(x.duration for x in L), repose_a, 0., "anchor A during opening"),
          aw(1.5, repose_a + np.array((-.020, .010, .020)), 0., "lift the empty decapper"),
          aw(3., counter_grip + up * .040, 0., "back over the counter"),
          aw(2., counter_grip, 0., "set the rear handles on the counter"),
          aw(1., counter_grip, FINGER_OPEN, "release the decapper"),
          aw(1.5, counter_grip + up * .100, FINGER_OPEN, "left hand clear", lean=30),
          aw(5., counter_grip + up * .100, FINGER_OPEN, "hands clear", lean=30)]
    return {"left": L, "right": R}
