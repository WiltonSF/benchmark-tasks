"""On a home workbench: two planed pine boards, 220 x 60 x 27 mm, lie flat side by side with their long edges 2 mm
apart, their ends toward the robot. In front of their near ends a red 6 in one-hand bar clamp lies flat on its side
with its jaws open, pointing away from the robot, one jaw either side of the boards. Its pistol grip and trigger lie
beside the boards on the robot's left. The clamp is a mechanism: squeezing the trigger toward the grip drives the
steel bar through the handle a few millimetres, a brake keeps the bar from opening again, and the trigger springs
back for the next squeeze. Where the boards and the clamp lie on the bench changes between trials.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

RED, BLACK, STEEL, DARK = (0.80, 0.10, 0.08), (0.06, 0.06, 0.07), (0.62, 0.63, 0.66), (0.20, 0.20, 0.22)
PINE_A, PINE_B = (0.86, 0.70, 0.46), (0.74, 0.55, 0.32)


def _box(lo, hi, color, friction=None):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(float(v) for v in hi - lo), at=tuple(float(v) for v in (lo + hi) / 2), color=color,
                     friction=friction)


# -- the clamp -------------------------------------------------------------------------------------------------------
# Clamp frame (the part frame): u along the bar toward the far (sliding) jaw, v along the jaws (they point +v), w up
# from the bench (the clamp lies on its side). Origin: the fixed jaw's back face on the bench. Sized on a 6 in mini
# one-hand clamp (152 mm max opening, about 60 mm throat, 12.7 x 4.8 mm bar, rated about 620 N / 140 lbf).
H = 0.028  # the handle's and jaws' thickness lying flat
PAD_FACE = 0.006  # the fixed pad's face (u) at rest
OPENING = 0.140  # pad face to pad face at the start
FAR_FACE = PAD_FACE + OPENING
PAD_V = (0.030, 0.068)  # both pads span v 30..68 mm (38 mm of pad)
PAD_W = (0.002, 0.026)
K_PAD = 25000.0  # N/m: the fixed pad is a load-measuring spring, 25 N per mm
PAD_TRAVEL = 0.010
BAR_V, BAR_W = 0.006, (0.0115, 0.0165)  # the bar: 12 mm deep (v +-6 mm), 5 mm thick
BAR_TRAVEL = 0.130
BAR_DRAG = 1.0  # N: the bar's sliding friction in the handle and brake
GRIP_U, GRIP_V, GRIP_W = (-0.086, -0.062), (0.016, 0.110), (0.002, 0.026)  # the pistol grip, 24 mm across
TRIG_U, TRIG_V, TRIG_W = (-0.042, -0.034), (0.034, 0.078), (0.006, 0.022)  # the trigger at rest, 8 mm thick
PIVOT = (-0.038, 0.014, 0.014)  # the trigger's hinge in the handle (axis w)
TRIG_MAX = 0.30  # rad: squeezed fully, its tip near the grip
TRIG_K, TRIG_REST = 0.25, -0.80  # N m/rad, rad: the return spring holds it open with 0.20 N m (0.275 N m squeezed)
PIN = (PIVOT[0], PIVOT[1] + 0.010)  # the trigger's drive pin, 10 mm from the pivot
PIN_R = 0.0025
PLAY = 0.0003  # each side of the pin in the drive plate's slot
TOOTH = 0.0001  # the brake and the drive plate grip the bar in 0.1 mm steps
FINGER_D = 0.046  # a fingertip on the trigger's middle, 46 mm from the pivot: lever 4.6 to 1

HOUSING = [
    _box((-0.090, -0.016, 0.0), (-0.012, 0.016, H), RED),  # round the bar
    _box((-0.012, -0.016, 0.0), (0.0, 0.072, H), RED),  # the fixed jaw's arm
    _box((GRIP_U[0], GRIP_V[0], GRIP_W[0]), (GRIP_U[1], GRIP_V[1], GRIP_W[1]), BLACK, (1.0, 0.9)),  # rubber grip
]
PAD = parts.Slider(
    "pad", axis=(-1.0, 0.0, 0.0), travel=PAD_TRAVEL, mass=0.02, spring=K_PAD, damping=10.0, max_effort=1000.0,
    shapes=(_box((0.0, PAD_V[0], PAD_W[0]), (PAD_FACE, PAD_V[1], PAD_W[1]), BLACK, (0.8, 0.7)),))
BAR = parts.Slider(
    "bar", axis=(-1.0, 0.0, 0.0), travel=BAR_TRAVEL, mass=0.07, damping=0.5, friction=BAR_DRAG,
    dynamic_friction=BAR_DRAG,
    shapes=(_box((-0.115, -BAR_V, BAR_W[0]), (FAR_FACE + 0.018, BAR_V, BAR_W[1]), STEEL),
            _box((FAR_FACE, PAD_V[0], PAD_W[0]), (FAR_FACE + 0.006, PAD_V[1], PAD_W[1]), BLACK, (0.8, 0.7)),
            _box((FAR_FACE + 0.006, -0.010, 0.0005), (FAR_FACE + 0.018, 0.072, H - 0.0005), RED)))
# The drive plate: rides on the bar (a joint along it), its slot round the trigger's pin. It may slide back along the
# bar freely; forward, it stops TOOTH ahead of where it last was on the bar, and there it pushes the bar.
_SLOT = PIN_R + PLAY
PLATE = parts.Slider(
    "plate", parent="bar", axis=(-1.0, 0.0, 0.0), travel=TOOTH, extend=0.20, mass=0.012, damping=0.05,
    shapes=(_box((PIN[0] - _SLOT - 0.003, 0.0175, 0.008), (PIN[0] - _SLOT, 0.031, 0.020), DARK),
            _box((PIN[0] + _SLOT, 0.0175, 0.008), (PIN[0] + _SLOT + 0.003, 0.031, 0.020), DARK)))
TRIGGER = parts.Hinge(
    "trigger", pivot=PIVOT, axis=(0.0, 0.0, 1.0), lower=0.0, upper=TRIG_MAX, mass=0.015, spring=TRIG_K,
    target=TRIG_REST, damping=0.002,
    shapes=(_box((TRIG_U[0], TRIG_V[0], TRIG_W[0]), (TRIG_U[1], TRIG_V[1], TRIG_W[1]), BLACK, (1.0, 0.9)),
            parts.cylinder(PIN_R, 0.012, at=(PIN[0], PIN[1], 0.014), color=DARK)))

CLAMP_START = (0.178, 0.057)  # lot A: the clamp frame's origin on the bench
CLAMP_YAW = -math.pi / 2  # u along world -y, v along world +x
CLAMP = parts.Part("clamp", pos=(*CLAMP_START, TOP), yaw=CLAMP_YAW, mass=0.17, shapes=HOUSING,
                   joints=[PAD, BAR, PLATE, TRIGGER], friction=(0.45, 0.35), contact_offset=0.001,
                   angular_damping=1.0)

# -- the boards ------------------------------------------------------------------------------------------------------
BOARD_L, BOARD_W, BOARD_T = 0.220, 0.060, 0.027  # 5/4 pine, planed to 27 mm
BOARD_MASS = 0.155  # pine, about 440 kg/m3
GAP0 = 0.002
SLIDE = 0.062  # how far the clamp must slide toward the boards for its pads to bear 12 to 50 mm from their ends
NEAR_V = 0.018  # the boards' near ends, in the clamp frame, once it has slid
_u_left = PAD_FACE + 0.010  # 10 mm clear of the fixed pad
LEFT_C = (CLAMP_START[0] + SLIDE + NEAR_V + BOARD_L / 2, CLAMP_START[1] - (_u_left + BOARD_W / 2))
RIGHT_C = (LEFT_C[0], LEFT_C[1] - BOARD_W - GAP0)


def _board(name, xy, color):
    return parts.Part(name, pos=(*xy, TOP), mass=BOARD_MASS, friction=(0.5, 0.45), contact_offset=0.001,
                      angular_damping=1.0,
                      shapes=[_box((-BOARD_L / 2, -BOARD_W / 2, 0.0), (BOARD_L / 2, BOARD_W / 2, BOARD_T), color)])


LEFT = _board("board_left", LEFT_C, PINE_A)
RIGHT = _board("board_right", RIGHT_C, PINE_B)
OBJECTS = [LEFT, RIGHT, CLAMP]

# Lot B: everything 30 mm farther from the robot and 45 mm to its right.
LOT_B = (0.030, -0.045)


def _shift(xy, d):
    return (xy[0] + d[0], xy[1] + d[1], TOP)


LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"clamp": _shift(CLAMP_START, (0, 0)), "board_left": _shift(LEFT_C, (0, 0)),
                         "board_right": _shift(RIGHT_C, (0, 0))}, offset=(0.0, 0.0)),
    lots.Lot("B", place={"clamp": _shift(CLAMP_START, LOT_B), "board_left": _shift(LEFT_C, LOT_B),
                         "board_right": _shift(RIGHT_C, LOT_B)}, offset=LOT_B),
)


# -- geometry ----------------------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def _tilt_deg(st) -> float:
    return math.degrees(math.acos(float(np.clip(_rot(st.quat_xyzw)[2, 2], -1.0, 1.0))))


_CORNERS = np.array([[sx * BOARD_L / 2, sy * BOARD_W / 2, z] for sx in (-1, 1) for sy in (-1, 1)
                     for z in (0.0, BOARD_T)])


def board_corners(st) -> np.ndarray:
    return _CORNERS @ _rot(st.quat_xyzw).T + np.asarray(st.pos, float)


def in_clamp(objects, pts) -> np.ndarray:
    c = objects["clamp"]
    return (np.asarray(pts, float) - np.asarray(c.pos, float)) @ _rot(c.quat_xyzw)


def force(objects) -> float:
    """The clamping force (N): the fixed pad's compression times 25 N/mm."""
    return K_PAD * max(0.0, float(objects["clamp"].joints["pad"]))


def faces(objects) -> tuple[float, float]:
    """Where the two pad faces are along the bar (u, clamp frame, m)."""
    j = objects["clamp"].joints
    return PAD_FACE - float(j["pad"]), FAR_FACE - float(j["bar"])


def joint_gap(objects) -> tuple[float, float]:
    """The joint between the boards (m) at its near and far ends: the right board's inner-edge corners measured
    square off the left board's inner edge (negative would mean overlap)."""
    lb, rb = objects["board_left"], objects["board_right"]
    rl = _rot(lb.quat_xyzw)
    rr = _rot(rb.quat_xyzw)
    out = []
    for sx in (-1.0, 1.0):
        corner = np.asarray(rb.pos, float) + rr @ np.array([sx * BOARD_L / 2, BOARD_W / 2, BOARD_T / 2])
        local = rl.T @ (corner - np.asarray(lb.pos, float))
        out.append(float(-BOARD_W / 2 - local[1]))
    return out[0], out[1]


def ends_offset(objects) -> float:
    """How far apart (m) the two boards' near ends are, along the left board."""
    lb, rb = objects["board_left"], objects["board_right"]
    rl = _rot(lb.quat_xyzw)
    near_r = np.asarray(rb.pos, float) + _rot(rb.quat_xyzw) @ np.array([-BOARD_L / 2, 0.0, 0.0])
    return float(abs(rl.T @ (near_r - np.asarray(lb.pos, float)) @ np.array([1.0, 0.0, 0.0]) + BOARD_L / 2))


def flush(objects) -> bool:
    """Both boards flat on the bench (under 2 degrees) with their tops level within 1 mm."""
    lb, rb = objects["board_left"], objects["board_right"]
    return bool(_tilt_deg(lb) <= 2.0 + EPS and _tilt_deg(rb) <= 2.0 + EPS
                and abs(float(lb.high[2]) - float(rb.high[2])) <= 0.001 + EPS)


ON_EDGE = 0.001  # a pad face within 1 mm of its board's outer edge
BEAR = 0.015  # each pad bears on at least 15 mm of its board's edge, measured from the board's end
CLOSED = 0.0005
ENDS = 0.003
F_MIN = 60.0  # N


PAD_MID = sum(PAD_V) / 2


def edge_u(objects, name, side) -> float:
    """Where a board's outer edge (``side`` +1: its local +y edge) crosses the pads' middle line (v = 49 mm), as u in
    the clamp frame (m)."""
    st = objects[name]
    r, p = _rot(st.quat_xyzw), np.asarray(st.pos, float)
    ends = [p + r @ np.array([sx * BOARD_L / 2, side * BOARD_W / 2, BOARD_T / 2]) for sx in (-1.0, 1.0)]
    a, b = in_clamp(objects, ends)
    if abs(b[1] - a[1]) < 1e-9:
        return math.inf
    k = (PAD_MID - a[1]) / (b[1] - a[1])
    return float(a[0] + k * (b[0] - a[0]))


def near_ends(objects) -> list[float]:
    """Each board's near end (its corner nearest the bar), as v in the clamp frame (m)."""
    return [float(in_clamp(objects, board_corners(objects[n]))[:, 1].min()) for n in ("board_left", "board_right")]


def round_panel(objects) -> bool:
    """Both boards between the jaws at the pads (u), and their near ends where both pads bear on 15 mm or more of
    each edge without the boards over the bar (v), the clamp lying flat (within 5 degrees)."""
    f_fixed, f_far = faces(objects)
    between = bool(edge_u(objects, "board_left", 1.0) >= f_fixed - ON_EDGE - EPS
                   and edge_u(objects, "board_right", -1.0) <= f_far + ON_EDGE + EPS)
    bear = all(BAR_V - EPS <= e <= PAD_V[1] - BEAR + EPS for e in near_ends(objects))
    flat = bool(abs(_tilt_deg(objects["clamp"])) <= 5.0 + EPS)
    return between and bear and flat


def on_edges(objects) -> bool:
    f_fixed, f_far = faces(objects)
    return bool(abs(edge_u(objects, "board_left", 1.0) - f_fixed) <= ON_EDGE + EPS
                and abs(edge_u(objects, "board_right", -1.0) - f_far) <= ON_EDGE + EPS)


def joint_closed(objects) -> bool:
    g = joint_gap(objects)
    return bool(max(g) <= CLOSED + EPS and min(g) >= -0.002)


def clamped(objects) -> bool:
    """The clamp round the panel with both pads on its outer edges and the joint closed: the 0.5 state."""
    return round_panel(objects) and on_edges(objects) and joint_closed(objects)


def pressed(objects) -> bool:
    """Everything the full score needs at one moment, apart from let go and still."""
    return bool(clamped(objects) and force(objects) >= F_MIN - EPS and ends_offset(objects) <= ENDS + EPS
                and flush(objects))


# -- the trial record --------------------------------------------------------------------------------------------------
CLEAR = 0.005  # let go: every sampled point of both grippers more than 5 mm from the clamp and from both boards
STILL_M, HOLD_S = 0.002, 1.0
_rec = parts.__dict__.setdefault("bar_clamp_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", fell=False, gaps=None, snap=None, since=None, level=0.0, t=0.0,
                peak=0.0)


if not _rec:
    reset_record()


def _snap(objects) -> np.ndarray:
    c = objects["clamp"]
    return np.concatenate([board_corners(objects["board_left"]).ravel(), board_corners(objects["board_right"]).ravel(),
                           np.asarray(c.pos, float), _rot(c.quat_xyzw)[:, 0] * 0.1])


def _let_go() -> bool:
    g = _rec["gaps"]
    return g is not None and min(g) > CLEAR + EPS


def level_now(objects) -> float:
    """The tier this moment would earn once let go and still for 1 s (before the whole-trial rules)."""
    if not _let_go():
        return 0.0
    if pressed(objects):
        return 1.0
    if clamped(objects):
        return 0.5
    if round_panel(objects):
        return 0.25
    return 0.0


def observe(objects, gaps, t) -> None:
    """One control step. ``gaps``: the closest gripper point to (the clamp, the left board, the right board) in m, or
    None when the grippers could not be read, which fails the trial."""
    _rec["steps"] += 1
    _rec["t"] = float(t)
    try:
        for name in ("clamp", "board_left", "board_right"):
            st = objects[name]
            if not np.all(np.isfinite(st.pos)) or not np.all(np.isfinite(st.quat_xyzw)):
                raise ValueError(f"{name} state unknown")
        for j in ("pad", "bar", "plate", "trigger"):
            if not math.isfinite(objects["clamp"].joints[j]):
                raise ValueError(f"clamp joint {j} unknown")
        if gaps is None:
            raise ValueError("gripper state unknown")
    except (KeyError, ValueError, TypeError, AttributeError) as e:
        _rec.update(known=False, reason=str(e), since=None, level=0.0)
        return
    for name in ("clamp", "board_left", "board_right"):
        st = objects[name]
        if float(st.pos[2]) < TOP - 0.05 or _tilt_deg(st) > 30.0:
            _rec["fell"] = True
    _rec["peak"] = max(_rec["peak"], force(objects))
    _rec["gaps"] = tuple(float(g) for g in gaps)
    lvl = level_now(objects)
    snap = _snap(objects)
    if lvl <= 0.0:
        _rec.update(since=None, level=0.0, snap=None, now=0.0)
        return
    moved = math.inf if _rec["snap"] is None else float(np.abs(snap - _rec["snap"]).max())
    if _rec["since"] is None or moved > STILL_M + EPS or lvl != _rec.get("now"):
        _rec.update(since=float(t), snap=snap, level=0.0, now=lvl)
    elif float(t) - _rec["since"] >= HOLD_S - EPS:
        _rec["level"] = lvl


# -- scoring ---------------------------------------------------------------------------------------------------------------
def _ok() -> bool:
    return bool(_rec["known"] and not _rec["fell"] and _rec["steps"] > 0)


def success(objects):
    """The boards clamped edge to edge (pads on both outer edges, the joint closed within 0.5 mm, 60 N or more on the
    pad), their near ends within 3 mm, tops level within 1 mm, both grippers clear, and all of it still for 1 s."""
    return bool(_ok() and _rec["level"] >= 1.0 and pressed(objects) and _let_go())


def score(objects):
    """1.0 on success. 0.5: let go and still for 1 s with both pads on the boards' outer edges and the joint closed,
    but not the rest. 0.25: let go and still for 1 s with the clamp round both boards near their ends. 0 otherwise,
    and 0 for the whole trial if a board or the clamp ever left the bench or tipped past 30 degrees, or the simulator
    could not report the grippers or the parts at any step."""
    if not _ok():
        return 0.0
    if success(objects):
        return 1.0
    lvl = min(_rec["level"], level_now(objects))
    return 0.5 if lvl >= 0.5 else (0.25 if lvl >= 0.25 else 0.0)


def distance(objects):
    return float(max(0.0, F_MIN - force(objects)))


# -- grippers: hull faces sampled every 3 mm ---------------------------------------------------------------------------
FACE_STEP = 0.003
_HAND_LINKS = ("gripper", "tip_left", "tip_right")


def _face_points(hull, step=FACE_STEP) -> np.ndarray:
    """Points on every face of a convex hull about ``step`` apart, corners included."""
    pts = [hull.vertices]
    for n, d in zip(hull.planes[:, :3], hull.planes[:, 3]):
        on = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for i in range(len(ring)):
            a, b = ring[i], ring[(i + 1) % len(ring)]
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step)))
            for s_ in range(m + 1):
                for t_ in range(m + 1 - s_):
                    pts.append((c + (a - c) * (s_ / m) + (b - c) * (t_ / m))[None, :])
    pts = np.concatenate(pts)
    _, keep = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(keep)]])


_FACES: dict = {}


def link_points(name) -> np.ndarray:
    if name not in _FACES:
        from yam_arena import analysis

        hs = analysis.link_hulls().get(name, [])
        if not hs:
            raise ValueError(f"no collision hull for {name}")
        _FACES[name] = np.concatenate([_face_points(h) for h in hs])
    return _FACES[name]


def _part_boxes(part, st):
    poses = {"base": (np.asarray(st.pos, float), _rot(st.quat_xyzw))}
    for name, (p, q) in getattr(st, "links", {}).items():
        poses[name] = (np.asarray(p, float), _rot(q))
    return part.world_boxes(poses)


def gap_to(part, st, pts) -> float:
    """Closest distance (m, negative inside) from any point to any shape of the part, links included."""
    from yam_arena import analysis

    near = np.linalg.norm(pts - np.asarray(st.pos, float), axis=1) < 0.40
    if not near.any():
        return 0.10
    pts = pts[near]
    return float(min(analysis.box_signed_distance(pts, c, r, s).min() for _, c, r, s in _part_boxes(part, st)))


def gaps(objects, pts):
    return (gap_to(CLAMP, objects["clamp"], pts), gap_to(LEFT, objects["board_left"], pts),
            gap_to(RIGHT, objects["board_right"], pts))


def _hand_points(sim) -> np.ndarray | None:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                r = _rot(poses[i, 3:])
                out.append(link_points(n) @ r.T + (poses[i, :3] - origin))
                found += 1
    if found != 2 * len(_HAND_LINKS):
        return None
    pts = np.concatenate(out)
    return pts if np.isfinite(pts).all() else None


def _fingers(sim) -> dict:
    """Finger joints and gripper-link position of both arms, for trace files only (never scored)."""
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = {}
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        q = parts._np(asset.data.joint_pos)[0]
        names = list(asset.joint_names)
        poses = parts._np(asset.data.body_link_pose_w)[0]
        g = poses[list(asset.body_names).index("gripper"), :3] - origin
        out[arm] = [round(float(q[names.index(n)]), 4) for n in ("joint7", "joint8")] + [round(float(v), 4) for v in g]
    return out


# -- the clamp's brake and drive plate ---------------------------------------------------------------------------------
# Both are one-way grips on the bar, as in a real one-hand clamp, made as joint limits that follow the bar:
# - the brake: the bar may slide shut (its joint rising) but never back open past where it has been (less TOOTH);
# - the drive plate rides on the bar with the trigger's pin in its slot. It may slide back along the bar (the trigger
#   springing open), and its forward stop follows it back to TOOTH ahead of where it is. A squeeze pushes the plate
#   against that stop, so the pin drives the bar shut and the bar's load comes back through the pin onto the trigger.
_lim = parts.__dict__.setdefault("bar_clamp_limits", {})


def _write_limit(sim, joint, lo, hi) -> None:
    import torch

    base = sim.env.unwrapped
    art = base.scene["clamp"]
    ids = art.find_joints([joint], preserve_order=True)[0]
    lim = torch.tensor([[[float(lo), float(hi)]]], dtype=torch.float32, device=base.device)
    art.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)
    _lim[joint] = (float(lo), float(hi))


def _limits_home(sim) -> None:
    import torch

    _write_limit(sim, "bar", BAR.lower, BAR.upper)
    _write_limit(sim, "plate", PLATE.lower, PLATE.upper)
    base = sim.env.unwrapped
    art = base.scene["clamp"]
    ids = art.find_joints(["pad", "bar", "plate", "trigger"], preserve_order=True)[0]
    zero = torch.zeros((1, len(ids)), dtype=torch.float32, device=base.device)
    art.write_joint_state_to_sim_index(position=zero, velocity=zero, joint_ids=ids)


@parts.on_step
def _one_way(sim) -> None:
    if _lim.get("last_t") == sim.t:
        return
    _lim["last_t"] = sim.t
    j = sim.states()["clamp"].joints
    lo, hi = _lim.get("bar", (BAR.lower, BAR.upper))
    brake = min(BAR.upper, float(j["bar"]) - TOOTH)
    if brake > lo + 1e-5:
        _write_limit(sim, "bar", brake, hi)
    plo, phi = _lim.get("plate", (PLATE.lower, PLATE.upper))
    stop = float(j["plate"]) + TOOTH
    if stop < phi - 1e-5:
        _write_limit(sim, "plate", plo, stop)


# -- simulator hooks -------------------------------------------------------------------------------------------------
def _write_pose(base, name, x, y, z, a) -> None:
    import torch

    obj = base.scene[name]
    pose = torch.tensor([[x, y, z, 0.0, 0.0, math.sin(a / 2), math.cos(a / 2)]], dtype=torch.float32,
                        device=base.device)
    pose[:, :3] += base.scene.env_origins[0][:3].to(pose.dtype)
    obj.write_root_pose_to_sim_index(root_pose=pose)
    obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=base.device))


def _apply_exact(emb) -> bool:
    """A new trial's layout: every object written exactly at its lot's pose (the clamp open, its joints at 0),
    settled, written again and rendered, so the trial's first frame shows the true start."""
    base = emb._base
    lot = lots.current()

    class _S:  # what _limits_home needs
        env = emb._env

    for _ in range(2):
        for name in ("board_left", "board_right", "clamp"):
            _write_pose(base, name, *lot.pos(name), lot.yaw_of(name))
        _limits_home(_S)
        for _ in range(max(1, round(0.25 / base.step_dt))):
            emb._env.step(emb._ready)
    for _ in range(lots.RENDERS):
        base.sim.render()
    return True


lots._apply = _apply_exact


@parts.on_reset
def _new_trial(sim) -> None:
    _limits_home(sim)
    _lim["last_t"] = None
    reset_record()
    _rec["last_t"] = None


@parts.on_step
def _step(sim) -> None:
    if _rec.get("last_t") == sim.t:  # a second copy of this module: step once
        return
    _rec["last_t"] = sim.t
    try:
        objects = sim.states()
    except Exception:  # noqa: BLE001  unknown state: no credit
        objects = {}
    try:
        pts = _hand_points(sim)
        g = gaps(objects, pts) if pts is not None else None
    except Exception:  # noqa: BLE001
        g = None
    observe(objects, g, float(sim.t))
    if os.environ.get("TRACE_OUT") and _rec["steps"] % 2 == 0 and objects:
        try:
            j = objects["clamp"].joints
            row = dict(t=round(float(sim.t), 3), score=score(objects), force=round(force(objects), 1),
                       bar=round(j["bar"], 5), plate=round(j["plate"], 5), trig=round(j["trigger"], 4),
                       pad=round(j["pad"], 5), lim=_lim.get("bar"), plim=_lim.get("plate"),
                       gap=[round(v, 5) for v in joint_gap(objects)], ends=round(ends_offset(objects), 4),
                       round=round_panel(objects), edges=on_edges(objects), closed=joint_closed(objects),
                       level=_rec["level"], known=_rec["known"], reason=_rec["reason"], fell=_rec["fell"],
                       gaps=_rec["gaps"], clamp=np.round(objects["clamp"].pos, 4).tolist(),
                       left=np.round(objects["board_left"].pos, 4).tolist(),
                       right=np.round(objects["board_right"].pos, 4).tolist(), fingers=_fingers(sim))
            with pathlib.Path(os.environ["TRACE_OUT"]).with_name("clamp.jsonl").open("a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with pathlib.Path(os.environ["TRACE_OUT"]).with_name("clamp.jsonl").open("a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- demo: open-loop from the start layout the cameras show -------------------------------------------------------------
# The left hand takes the clamp by its grip and trigger together (one fingertip on the grip's back like a palm, the
# other on the trigger), slides it toward the boards until its jaws are round their near ends, then squeezes and
# relaxes the trigger, stroke by stroke, until the jaws have pressed the boards together, and lets go. The right arm
# waits. Positions come from the published start layout of the lot only.
GRIP_TCP_Z = 0.016  # fingertips 3 mm above the bench
HAND_V = 0.060  # along the trigger, 46 mm from its pivot
SQUEEZE_SEP = 0.026  # commanded finger separation for a squeeze (about 6 mm past the squeezed faces: about 31 N)
RELAX_SEP = 0.060  # 4 mm clear of the grip and of the sprung-open trigger
APPROACH_SEP = 0.066
STROKES = 14
SQ_U = (GRIP_U[0] + (TRIG_U[1] - FINGER_D * math.sin(TRIG_MAX))) / 2  # the hand's middle with the trigger squeezed
RX_U = (GRIP_U[0] + TRIG_U[1]) / 2  # ... with it open


def _world(lot, u, v, z, slid):
    ox, oy = lot.offset
    x0, y0 = CLAMP_START[0] + ox + (SLIDE if slid else 0.0), CLAMP_START[1] + oy
    return (x0 + v, y0 - u, TOP + z)


def _sep(s):
    return -s / 2


def demo():
    lot = lots.current()
    w = lambda d, u, slid, sep, lab, z=GRIP_TCP_Z: Waypoint(d, _world(lot, u, HAND_V, z, slid), _sep(sep), lab, 0.0)  # noqa: E731
    left = [
        w(2.5, RX_U, False, APPROACH_SEP, "above the grip and trigger", z=0.09),
        w(1.5, RX_U, False, APPROACH_SEP, "fingers down either side"),
        w(0.8, SQ_U, False, SQUEEZE_SEP, "take the clamp (squeeze)"),
        w(2.5, SQ_U, True, SQUEEZE_SEP, "slide it until the jaws are round the boards"),
        w(0.4, SQ_U, True, SQUEEZE_SEP, "settle"),
    ]
    for k in range(STROKES):
        left += [w(0.5, RX_U, True, RELAX_SEP, f"relax {k + 1}"),
                 w(0.8, SQ_U, True, SQUEEZE_SEP, f"squeeze {k + 1}"),
                 w(0.2, SQ_U, True, SQUEEZE_SEP, f"hold {k + 1}")]
    left += [
        w(0.6, RX_U, True, APPROACH_SEP, "let go"),
        w(1.2, RX_U, True, APPROACH_SEP, "straight up", z=0.11),
        Waypoint(1.5, (0.20, 0.25, TOP + 0.20), FINGER_OPEN, "park", 0.0),
        Waypoint(8.0, (0.20, 0.25, TOP + 0.20), FINGER_OPEN, "wait", 0.0),
    ]
    right = [Waypoint(2.0, (0.20, -0.25, TOP + 0.20), FINGER_OPEN, "park", 0.0),
             Waypoint(sum(x.duration for x in left) - 2.0, (0.20, -0.25, TOP + 0.20), FINGER_OPEN, "wait", 0.0)]
    return {"left": left, "right": right}
