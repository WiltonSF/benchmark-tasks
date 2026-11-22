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


def _seal_shapes():
    """Collided as one solid 21.5 mm x 7.6 mm cylinder (the skirt with the bead and stopper flange inside it, as one
    rigid piece), shown as the aluminum skirt and annular top. The stopper stays in the bottle's neck."""
    out = [parts.cylinder(SKIRT_OUT - 0.00005, SKIRT_H + TOP_T, (0, 0, (SKIRT_H + TOP_T) / 2), ALU)]
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
RIVET_K, RIVET_PLAY = 6.0, 0.08  # N m/rad, rad: the rivet holds up to 0.48 N m, then slips
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
                    contact_offset=0.002, report=("base",), solver=(64, 4))
SEAL = parts.Part("aluminum_seal", pos=(*BOTTLE_XY, TOP + SEAL_Z), mass=0.001, shapes=_seal_shapes(),
                  friction=(0.6, 0.5), contact_offset=0.001, report=("base",), solver=(64, 4))
PLIER = parts.Part(
    "plier_decapper", pos=(*PLIER_XY, TOP + PIVOT_Z), mass=0.08, shapes=HALF_A, friction=(0.9, 0.8),
    contact_offset=0.002,
    joints=[parts.Hinge("jaw", shapes=tuple(_turned(HALF_B_CLOSED, -OPEN_ANGLE)), pivot=(0, 0, 0), axis=(0, 0, 1),
                        lower=-0.05, upper=CLOSED_STOP, mass=0.08, spring=RIVET_K, damping=0.04, max_effort=20.0)],
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

# --- the scorer's record (kept on the shared parts module so near_misses.py can drive the same record) --------------
TEAR_TURN = math.pi / 2  # rad the gripped seal must turn on the finish before its crimp counts as torn
GRIP_N = 10.0  # N each jaw must press the seal for a turn to count (an estimate: below it the jaws slip)
TURN_MIN = math.radians(0.5)  # a smaller turn in one control step is solver jitter, not twisting
GLASS_N = 20.0  # N of jaw-on-glass contact that counts as a chipped bottle
FINGER_N = 0.5  # N of robot-finger contact on the seal that voids the trial
HOLD_S = 1.0
REMOVED_GAP, HAND_GAP = 0.010, 0.030
REC: dict = parts.__dict__.setdefault("_seal_record", {})


def reset_record() -> None:
    REC.clear()
    REC.update(known=False, zero=False, why="", released=False, turn=0.0, phi=0.0, rivet=0.0, still_s=0.0,
               clear_s=0.0, t=None, checked=False, t_release=None,
               peak={"jawA": 0.0, "jawB": 0.0, "glass": 0.0, "finger": 0.0})


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
    whole-trial rules, the crimp, and the still and hands-clear timers. True while the seal is still crimped (the
    simulator then holds it on its finish, turned REC['phi'] about the bottle's axis)."""
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
        if jaw_a >= GRIP_N - EPS and jaw_b >= GRIP_N - EPS:  # both jaws pressing: the seal turns with them
            now = seal_yaw(b, s)
            step = abs(_wrap(now - REC["phi"]))
            if step >= TURN_MIN - EPS:
                REC["turn"] += step
            REC["phi"] = now
        if REC["turn"] >= TEAR_TURN - EPS:
            REC.update(released=True, t_release=t)
            print(f"[decapper] crimp torn at t={t:.2f}s after {math.degrees(REC['turn']):.0f} deg",
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


def _write_seal(base, phi: float) -> None:
    """Hold the crimped seal on its finish: its pose follows the bottle, turned ``phi`` about the bottle's axis."""
    import torch

    bottle, seal = base.scene[BOTTLE.name], base.scene[SEAL.name]
    bp = _arr(bottle.data.root_link_pose_w)[0].astype(np.float64)
    rb = parts._quat_to_matrix(bp[3:])
    pos = bp[:3] + rb @ np.array((0.0, 0.0, SEAL_Z))
    quat = parts._matrix_to_quat(rb @ _rz(phi))
    pose = seal.data.root_link_pose_w
    pose = (pose.torch if hasattr(pose, "torch") else pose).clone()
    pose[0, :3] = torch.tensor(pos, dtype=pose.dtype, device=pose.device)
    pose[0, 3:] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
    seal.write_root_pose_to_sim_index(root_pose=pose)
    vel = seal.data.default_root_vel
    vel = vel.torch if hasattr(vel, "torch") else vel
    seal.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(vel))


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
            why = _start_ok(o)
            print(f"[decapper] trial start lot {lots.current().name}: "
                  + ", ".join(f"{n} {np.round(o[n].pos, 4).tolist()}" for n in (BOTTLE.name, PLIER.name, BEAKER.name)),
                  file=sys.stderr, flush=True)
            if why:
                _zero("start: " + why)
        if "views" not in _SIM:
            _SIM["views"] = _views(base)
        seal_view, glass_view = _SIM["views"]
        pdt = base.physics_dt
        fs = np.linalg.norm(_arr(seal_view.get_contact_force_matrix(dt=pdt)).reshape(-1, 3), axis=1)
        fg = np.linalg.norm(_arr(glass_view.get_contact_force_matrix(dt=pdt)).reshape(-1, 3), axis=1)
        if fs.shape != (8,) or fg.shape != (2,) or not (np.isfinite(fs).all() and np.isfinite(fg).all()):
            raise RuntimeError(f"contact data shapes {fs.shape} {fg.shape}")
        jaw_a, jaw_b, finger, glass = float(fs[0]), float(fs[1]), float(fs[2:].max()), float(fg.max())
        clear = hands_clear(hand_points(base), o, HAND_GAP)
        if update(o, t, jaw_a, jaw_b, finger, glass, clear):
            _write_seal(base, REC["phi"])
        # the rivet: the halves stay where they are until the squeeze passes RIVET_K * RIVET_PLAY, then slip
        b, s, p = o[BOTTLE.name], o[SEAL.name], o[PLIER.name]
        q = float(p.joints["jaw"])
        tgt = min(max(REC["rivet"], q - RIVET_PLAY), q + RIVET_PLAY)
        if abs(tgt - REC["rivet"]) > 1e-6:
            REC["rivet"] = tgt
            sim.set_target(PLIER.name, "jaw", tgt)
        lv = level(o)
        row = dict(t=t, jawA=jaw_a, jawB=jaw_b, finger=finger, glass=glass,
                   turn_deg=math.degrees(REC["turn"]), released=bool(REC["released"]), q=q, rivet=REC["rivet"],
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
            for arm in ("left_robot", "right_robot"):
                rob = base.scene[arm]
                pose = parts._np(rob.data.body_link_pose_w)[0, rob.body_names.index("gripper")]
                row[arm] = (pose[:3] - origin + parts._quat_to_matrix(pose[3:]) @ np.array(TCP_OFFSET)).tolist()
                row[arm + "_fingers"] = parts._np(rob.data.joint_pos)[0, -2:].tolist()
            row["lot"] = lots.current().name
            with pathlib.Path(os.environ["TRACE_OUT"]).with_name("decapper.jsonl").open("a") as f:
                f.write(json.dumps(row, default=float) + "\n")
        except Exception as exc:  # noqa: BLE001  logging never affects the trial
            if not _SIM.get("logerr"):
                _SIM["logerr"] = True
                print(f"[decapper] trace logging skipped: {exc}", file=sys.stderr, flush=True)


# --- scripted demo: open loop from the visible layout ---------------------------------------------------------------
GRIP_L = 0.036  # m behind the rivet, along the plier's midline, where the hand holds the handles
TCP_ABOVE = 0.011  # the hand's centre point above the plier's mid-plane when gripping the handles
DOCK_FIX = (0.003, 0.002, 0.002)  # measured correction for the held plier's sag (m), from a trace
LEFT_FIX = (0.0, 0.0, 0.0)


def _span(q: float, L: float = GRIP_L) -> float:
    """Outer width across both handles, L behind the rivet, square to the midline, at joint position q."""
    bis = -(OPEN_ANGLE - q) / 2
    d = np.array((-math.sin(bis), math.cos(bis)))
    perp = np.array((d[1], -d[0]))
    pts = []
    for sh in HALF_A + _turned(HALF_B_CLOSED, -OPEN_ANGLE + q):
        if sh.color != GRIP:
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
    b = np.asarray(lot.pos(BOTTLE.name))
    pv = np.asarray(lot.pos(PLIER.name))
    w = np.asarray(lot.pos(BEAKER.name))
    pyaw = lot.yaw.get(PLIER.name, 0.0)
    fix = np.asarray(DOCK_FIX)
    hold, squeeze = _finger(SPAN_OPEN, 0.0016), _finger(SPAN_SEAL, 0.006)  # about 8 N and 30 N per pad
    relax, carry = _finger(SPAN_SEAL, 0.0008), _finger(SPAN_SEAL, 0.0012)  # about 4 N and 6 N

    def wp(t, pos, f, label, yaw):
        return Waypoint(float(t), tuple(float(v) for v in pos), float(f), label, float(yaw))

    # pick the open plier by its handles, square to its midline
    mid = pyaw + math.pi / 2 - OPEN_ANGLE / 2  # direction the open jaws point
    grip = pv + np.array((-math.cos(mid), -math.sin(mid), 0.0)) * GRIP_L
    grip[2] = TOP + PIVOT_Z + TCP_ABOVE
    up = np.array((0, 0, 1.0))
    # dock: jaws pointing +y onto the seal, then turn about the bottle's axis
    axis = np.array((b[0], b[1], TOP + SEAL_Z + SKIRT_H / 2 + TCP_ABOVE))
    reach = JAW_Y + GRIP_L

    def around(gamma, dz=0.0):
        return axis + _rz(-gamma) @ np.array((0.0, -reach, 0.0)) + fix + up * dz

    over = np.array((w[0], w[1] - reach, TOP + 0.130 + TCP_ABOVE)) + fix
    R = [wp(2.5, grip + up * 0.08, FINGER_OPEN, "over the plier", mid),
         wp(1.5, grip, FINGER_OPEN, "down to the handles", mid),
         wp(1.0, grip, hold, "hold the handles lightly", mid),
         wp(2.5, grip + up * 0.125, hold, "lift the open plier", mid),
         wp(3.0, around(0.0, 0.035), hold, "open jaws over the seal", math.pi / 2),
         wp(1.5, around(0.0), hold, "lower the jaws round the seal", math.pi / 2),
         wp(1.0, around(0.0), squeeze, "squeeze the handles", math.pi / 2),
         wp(0.5, around(0.0), squeeze, "settle", math.pi / 2)]
    gam = math.radians(45)
    for sweep in (1, -1, 1, -1):
        for k in range(1, 6):
            g = gam * (k / 5 if sweep > 0 else 1 - k / 5)
            R.append(wp(0.4, around(g), squeeze, "twist the seal", math.pi / 2 - g))
    R += [wp(0.5, around(0.0), squeeze, "hold", math.pi / 2),
          wp(2.5, around(0.0), squeeze, "wait for the other hand to clear", math.pi / 2),
          wp(1.5, around(0.0, 0.045), squeeze, "lift the seal off", math.pi / 2),
          wp(3.0, over, squeeze, "jaws over the waste beaker", math.pi / 2),
          wp(1.0, over, relax, "ease the squeeze", math.pi / 2),
          wp(1.5, over, relax, "let the seal drop", math.pi / 2),
          wp(0.5, over, carry, "hold the plier", math.pi / 2),
          wp(3.0, grip + up * 0.08, carry, "back over the counter", mid),
          wp(1.5, grip + up * 0.004, carry, "lower the plier", mid),
          wp(1.0, grip + up * 0.004, FINGER_OPEN, "let go", mid),
          wp(1.5, grip + up * 0.10, FINGER_OPEN, "hand up", mid),
          wp(3.0, grip + up * 0.10, FINGER_OPEN, "hands clear", mid)]
    lb = b + np.asarray(LEFT_FIX)

    def lw(t, pos, f, label):
        return tilt.Waypoint(float(t), tuple(float(v) for v in pos), float(f), label, yaw=-math.pi / 2, tilt=40,
                             heading=-math.pi / 2)

    body = np.array((lb[0], lb[1], TOP + 0.034))
    off = next(i for i, x in enumerate(R) if x.label == "hold") + 1  # the crimp is torn: let go before the lift
    rest = sum(x.duration for x in R[:off]) - (3.5 + 1.5 + 1.0)
    L = [lw(3.5, body + np.array((0, 0.03, 0.06)), FINGER_OPEN, "over the bottle"),
         lw(1.5, body, FINGER_OPEN, "down to the bottle"),
         lw(1.0, body, -0.023, "hold the bottle"),
         lw(rest, body, -0.023, "steady the bottle"),
         lw(1.0, body, FINGER_OPEN, "let go"),
         lw(1.5, body + np.array((0.0, 0.06, 0.06)), FINGER_OPEN, "hand clear"),
         lw(10.0, body + np.array((-0.05, 0.10, 0.10)), FINGER_OPEN, "hands clear")]
    return {"left": L, "right": R}
