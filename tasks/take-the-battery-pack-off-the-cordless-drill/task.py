"""On a home workshop bench: a cordless 18 V compact drill/driver stands upright on its battery pack, the pack flat on
the bench. The drill's housing is red with a black grip, and its chuck points along the bench to one side. The pack
(black, with a red top) rides in two rails under the drill's foot and has a small red release button on each side
face, near the chuck end. On the same side of the bench as the chuck lies an empty blue plastic tray. Which way the
chuck points, and so which side the tray is on, changes between trials.
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
import tilt  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

RED, BLACK, DARK, STEEL, BLUE = ((0.75, 0.08, 0.07), (0.07, 0.07, 0.08), (0.16, 0.16, 0.17), (0.62, 0.63, 0.65),
                                 (0.12, 0.32, 0.80))


def _box(lo, hi, color, friction=None):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(float(v) for v in hi - lo), at=tuple(float(v) for v in (lo + hi) / 2), color=color,
                     friction=friction)


# -- the pack ------------------------------------------------------------------------------------------------------
# Frame shared by the pack and the drill at its seat: origin on the bench under the pack's middle, +y toward the
# chuck (the way the pack slides off), x across, z up. Sized on an 18 V compact pack (118 x 79 x 55 mm, 430 g).
PACK_W, PACK_L, BODY_H = 0.079, 0.118, 0.042
NECK_HW, HEAD_HW = 0.024, 0.030  # the rail tongue on top: a 48 mm neck under a 60 mm head (the rail flanges)
NECK_Z, HEAD_Z, PACK_H = (0.042, 0.047), (0.047, 0.055), 0.055
TONGUE_Y = (-0.011, 0.059)  # 70 mm of tongue, open at the chuck end
BTN_PROUD = BTN_TRAVEL = 0.004  # each button stands 4 mm proud and goes 4 mm in (flush)
BTN_Y, BTN_Z = (0.032, 0.050), (0.028, 0.040)  # 18 x 12 mm button faces, near the chuck end
TOOTH_X, TOOTH_Y, TOOTH_Z = (0.0175, 0.0235), (0.038, 0.046), (0.055, 0.0595)  # each button's catch, on top
BTN_K, BTN_PRELOAD, BTN_MASS = 1500.0, 0.004, 0.012  # N/m; 6 N at rest, 12 N pressed flush
PRESSED = 0.0015  # a catch clears the drill's latch pocket once its button is 1.5 mm in (an estimate)
CATCH_FRICTION = (0.2, 0.15)  # moulded acetal-like latch catch, smooth against the pocket wall
BODY_MASS = 0.406  # with the two buttons: 0.43 kg

PACK_SHAPES = [
    _box((-PACK_W / 2, -PACK_L / 2, 0.0), (PACK_W / 2, PACK_L / 2, 0.030), BLACK),
    _box((-PACK_W / 2, -PACK_L / 2, 0.030), (PACK_W / 2, PACK_L / 2, BODY_H), RED),
    _box((-NECK_HW, TONGUE_Y[0], NECK_Z[0]), (NECK_HW, TONGUE_Y[1], NECK_Z[1]), DARK),
    _box((-HEAD_HW, TONGUE_Y[0], HEAD_Z[0]), (HEAD_HW, TONGUE_Y[1], HEAD_Z[1]), DARK),
]


def _button(side):
    s = 1.0 if side > 0 else -1.0
    x0, x1 = PACK_W / 2, PACK_W / 2 + BTN_PROUD
    cap = _box((min(s * x0, s * x1), BTN_Y[0], BTN_Z[0]), (max(s * x0, s * x1), BTN_Y[1], BTN_Z[1]), RED,
               friction=(0.9, 0.8))
    tooth = _box((min(s * TOOTH_X[0], s * TOOTH_X[1]), TOOTH_Y[0], TOOTH_Z[0]),
                 (max(s * TOOTH_X[0], s * TOOTH_X[1]), TOOTH_Y[1], TOOTH_Z[1]), DARK, CATCH_FRICTION)
    return parts.Slider("btn_l" if s > 0 else "btn_r", shapes=(cap, tooth), axis=(-s, 0.0, 0.0), travel=BTN_TRAVEL,
                        mass=BTN_MASS, spring=BTN_K, damping=2.0, target=-BTN_PRELOAD)


# -- the drill -----------------------------------------------------------------------------------------------------
# Sized on an 18 V compact drill/driver (165 mm long, 1.13 kg without its pack). Its foot carries the two rails
# (an L-section each side that hooks under the pack's head) and, in the sole above the head, the latch: a pocket
# round each button's catch, closed toward the chuck, with a channel 2 mm farther in that runs out of the front.
SOLE_LO, SOLE_HI = (0.055, 0.060), (0.060, 0.077)
FOOT_HW = 0.036
SKIRT_X, LIP_X, RAIL_Z, LIP_TOP = (0.031, FOOT_HW), (0.025, 0.031), 0.0425, 0.0465
RAIL_Y = (-0.0115, PACK_L / 2)
CHANNEL_X, POCKET_X, POCKET_Y = (0.013, 0.022), (0.013, 0.024), (0.0375, 0.0465)
HANDLE_Y = 0.0145  # the grip's middle, over the drill's centre of mass
HANDLE_HX, HANDLE_HY, HANDLE_Z = 0.017, 0.025, (0.077, 0.183)
BARREL_R, BARREL_Y, CHUCK_R, CHUCK_Y = 0.028, (-0.045, 0.075), 0.021, (0.075, 0.120)
BARREL_Z = HANDLE_Z[1] + BARREL_R
DRILL_MASS = 1.13
GRIP_FRICTION = (1.2, 1.1)


def _drill_shapes():
    out = [_box((-CHANNEL_X[0], -PACK_L / 2, SOLE_LO[0]), (CHANNEL_X[0], PACK_L / 2, SOLE_LO[1]), RED)]
    for s in (1.0, -1.0):
        def bx(x0, x1, y0, y1, z0, z1, c=RED, fr=None):
            return _box((min(s * x0, s * x1), y0, z0), (max(s * x0, s * x1), y1, z1), c, fr)
        out += [
            bx(POCKET_X[0], POCKET_X[1], -PACK_L / 2, POCKET_Y[0], *SOLE_LO),  # behind the pocket and channel
            bx(CHANNEL_X[1], POCKET_X[1], POCKET_Y[1], PACK_L / 2, *SOLE_LO),  # the catch wall in front of the pocket
            bx(POCKET_X[1], FOOT_HW, -PACK_L / 2, PACK_L / 2, *SOLE_LO),
            bx(*SKIRT_X, *RAIL_Y, RAIL_Z, HEAD_Z[1], BLACK, (0.35, 0.30)),  # rail side wall
            bx(*LIP_X, *RAIL_Y, RAIL_Z, LIP_TOP, BLACK, (0.35, 0.30)),  # rail lip under the pack's head
        ]
    out += [
        _box((-FOOT_HW, -PACK_L / 2, SOLE_HI[0]), (FOOT_HW, PACK_L / 2, SOLE_HI[1]), RED),
        _box((-FOOT_HW, -PACK_L / 2, RAIL_Z), (FOOT_HW, RAIL_Y[0], HEAD_Z[1]), RED),  # rear of the foot, a stop
        _box((-HANDLE_HX, HANDLE_Y - HANDLE_HY, HANDLE_Z[0]), (HANDLE_HX, HANDLE_Y + HANDLE_HY, HANDLE_Z[1]), BLACK,
             GRIP_FRICTION),  # rubber overmoulded grip
        _box((-0.008, HANDLE_Y + HANDLE_HY, 0.150), (0.008, HANDLE_Y + HANDLE_HY + 0.012, 0.175), DARK),  # trigger
        parts.cylinder(BARREL_R, BARREL_Y[1] - BARREL_Y[0], at=(0.0, sum(BARREL_Y) / 2, BARREL_Z), color=RED,
                       axis="y"),
        parts.cylinder(CHUCK_R, CHUCK_Y[1] - CHUCK_Y[0], at=(0.0, sum(CHUCK_Y) / 2, BARREL_Z), color=STEEL, axis="y"),
    ]
    return out


# -- the tray ------------------------------------------------------------------------------------------------------
TRAY_IN = (0.200, 0.150)  # inner x, y
TRAY_WALL, TRAY_H, TRAY_FLOOR = 0.005, 0.045, 0.005
TRAY_MASS = 0.30


def _tray_shapes():
    ix, iy, w = TRAY_IN[0] / 2, TRAY_IN[1] / 2, TRAY_WALL
    return [
        _box((-ix - w, -iy - w, 0.0), (ix + w, iy + w, TRAY_FLOOR), BLUE),
        _box((-ix - w, -iy - w, TRAY_FLOOR), (-ix, iy + w, TRAY_H), BLUE),
        _box((ix, -iy - w, TRAY_FLOOR), (ix + w, iy + w, TRAY_H), BLUE),
        _box((-ix, -iy - w, TRAY_FLOOR), (ix, -iy, TRAY_H), BLUE),
        _box((-ix, iy, TRAY_FLOOR), (ix, iy + w, TRAY_H), BLUE),
    ]


SEAT = (0.280, -0.060)  # lot A: the pack's middle; the chuck points to the robot's left (+y)
TRAY_XY = (0.260, 0.250)
PACK = parts.Part("pack", pos=(*SEAT, TOP), mass=BODY_MASS, shapes=PACK_SHAPES, joints=[_button(1), _button(-1)],
                  friction=(0.7, 0.6), contact_offset=0.002)
DRILL = parts.Part("drill", pos=(*SEAT, TOP), mass=DRILL_MASS, shapes=_drill_shapes(), friction=(0.6, 0.5),
                   contact_offset=0.002)
TRAY = parts.Part("tray", pos=(*TRAY_XY, TOP), mass=TRAY_MASS, shapes=_tray_shapes(), friction=(0.6, 0.5),
                  contact_offset=0.002)
OBJECTS = [DRILL, PACK, TRAY]

# Lot B is lot A turned half round about the bench's centre line: the drill and pack at (0.28, +0.06) with the chuck
# pointing to the robot's right, the tray at (0.26, -0.25).
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"drill": (*SEAT, TOP), "pack": (*SEAT, TOP), "tray": (*TRAY_XY, TOP)}),
    lots.Lot("B", place={"drill": (SEAT[0], -SEAT[1], TOP), "pack": (SEAT[0], -SEAT[1], TOP),
                         "tray": (TRAY_XY[0], -TRAY_XY[1], TOP)},
             yaw={"drill": math.pi, "pack": math.pi}),
)


# -- geometry helpers ----------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def _tilt_deg(st) -> float:
    return math.degrees(math.acos(float(np.clip(_rot(st.quat_xyzw)[2, 2], -1.0, 1.0))))


def pack_in_drill(objects) -> np.ndarray:
    """The pack's origin in the drill's frame (m): (0, 0, 0) when it is seated."""
    d, p = objects["drill"], objects["pack"]
    return _rot(d.quat_xyzw).T @ (np.asarray(p.pos, float) - np.asarray(d.pos, float))


def off_seat(objects) -> float:
    return float(np.linalg.norm(pack_in_drill(objects)))


def _body_corners(p) -> np.ndarray:
    r, o = _rot(p.quat_xyzw), np.asarray(p.pos, float)
    c = np.array([[sx * PACK_W / 2, sy * PACK_L / 2, z] for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, BODY_H)])
    return c @ r.T + o


def in_tray(objects) -> bool:
    """The pack flat on the tray floor (its four lowest body corners within 1.5 mm of the floor) and wholly inside the
    walls (all eight body corners inside the inner walls; the tongue on top lies within the body's outline)."""
    p, t = objects["pack"], objects["tray"]
    rt, ot = _rot(t.quat_xyzw), np.asarray(t.pos, float)
    local = (_body_corners(p) - ot) @ rt
    inside = bool(np.all(np.abs(local[:, 0]) <= TRAY_IN[0] / 2 + EPS) and np.all(np.abs(local[:, 1]) <= TRAY_IN[1] / 2
                                                                               + EPS))
    low = np.sort(local[:, 2])[:4]
    on_floor = bool(np.all(low >= TRAY_FLOOR - 0.0015 - EPS) and np.all(low <= TRAY_FLOOR + 0.0015 + EPS))
    return inside and on_floor


def drill_standing(objects) -> bool:
    """The drill upright on its foot (within 10 degrees) with the foot's underside within 3 mm of the bench."""
    d = objects["drill"]
    return bool(_tilt_deg(d) <= 10.0 + EPS and float(d.low[2]) <= TOP + 0.003 + EPS and float(d.low[2]) >= TOP - 0.003)


def tray_flat(objects) -> bool:
    return bool(_tilt_deg(objects["tray"]) <= 5.0 + EPS)


def buttons_in(objects) -> tuple[float, float]:
    j = objects["pack"].joints
    return float(j["btn_l"]), float(j["btn_r"])


# -- the trial record ----------------------------------------------------------------------------------------------
# Kept on the shared parts module so every copy of this file in one process (the checker and near_misses.py) sees one
# record. Cleared only by the simulator's scene reset at the start of a trial.
SLID, OFF = 0.010, 0.080  # partial tiers: slid 1 cm along the rails; off the drill (8 cm from the seat)
CLEAR = 0.005  # let go: every sampled point of both grippers more than 5 mm from the pack and from the drill
STILL_MM, HOLD_S = 0.002, 1.0
FELL_DEG = 45.0
_rec = parts.__dict__.setdefault("drill_pack_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", pressed=False, fell=False, gap_pack=None, gap_drill=None, snap=None,
                since=None, held=False, t=0.0)


if not _rec:
    reset_record()


def _moved(snap) -> float:
    """How far (m) the pack's corners or the drill have moved since the hold began."""
    return float(np.abs(snap - _rec["snap"]).max()) if _rec.get("snap") is not None else math.inf


def _let_go() -> bool:
    gp, gd = _rec["gap_pack"], _rec["gap_drill"]
    return gp is not None and gd is not None and gp > CLEAR + EPS and gd > CLEAR + EPS


def placed(objects) -> bool:
    """Everything the full score needs at one moment, apart from the 1 s timing."""
    return bool(_rec["pressed"] and off_seat(objects) >= OFF - EPS and in_tray(objects) and drill_standing(objects)
                and tray_flat(objects) and _let_go())


def observe(objects, gaps, t) -> None:
    """One control step. ``gaps``: (closest gripper point to the pack, to the drill) in m, or None when the grippers
    could not be read, which fails the trial (no credit)."""
    _rec["steps"] += 1
    _rec["t"] = float(t)
    try:
        for name in ("drill", "pack", "tray"):
            st = objects[name]
            if not np.all(np.isfinite(st.pos)) or not np.all(np.isfinite(st.quat_xyzw)):
                raise ValueError(f"{name} state unknown")
        b = buttons_in(objects)
        if gaps is None:
            raise ValueError("gripper state unknown")
    except (KeyError, ValueError, TypeError, AttributeError) as e:
        _rec.update(known=False, reason=str(e), since=None, held=False)
        return
    if min(b) >= PRESSED - EPS:
        _rec["pressed"] = True
    d, p = objects["drill"], objects["pack"]
    if _tilt_deg(d) > FELL_DEG or min(float(d.pos[2]), float(p.pos[2])) < TOP - 0.05:
        _rec["fell"] = True
    _rec["gap_pack"], _rec["gap_drill"] = float(gaps[0]), float(gaps[1])
    snap = np.concatenate([np.asarray(d.pos, float), _body_corners(p).ravel()])
    if not placed(objects):
        _rec.update(since=None, held=False, snap=None)
    elif _rec["since"] is None or _moved(snap) > STILL_MM + EPS:  # (re)start the second from here
        _rec.update(since=float(t), held=False, snap=snap)
    else:
        _rec["held"] = float(t) - _rec["since"] >= HOLD_S - EPS


# -- scoring ---------------------------------------------------------------------------------------------------------
def success(objects):
    """The pack is off the drill, flat on the tray floor inside its walls, both arms clear of the pack and the drill,
    the drill standing on the bench, and all of it still, held for 1 s."""
    return bool(_rec["known"] and not _rec["fell"] and _rec["held"] and placed(objects))


def score(objects):
    """1.0 on success. 0.5: both buttons were pressed in together at some step and the pack is now off the drill (its
    middle 8 cm or more from its seat) but not placed, let go and still in the tray for 1 s. 0.25: both buttons were
    pressed in together and the pack has slid 1 cm or more along the rails, still on the drill. 0 otherwise, and 0
    for the whole trial if the drill ever tipped past 45 degrees, the drill or pack fell off the bench, or the
    simulator could not report the grippers or the parts at any step."""
    if not _rec["known"] or _rec["fell"] or _rec["steps"] == 0:
        return 0.0
    if success(objects):
        return 1.0
    if not _rec["pressed"]:
        return 0.0
    s = off_seat(objects)
    if s >= OFF - EPS:
        return 0.5
    if s >= SLID - EPS:
        return 0.25
    return 0.0


def distance(objects):
    return float(max(0.0, OFF - off_seat(objects)))


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
    """A hand link's hull corners and face sample points (link frame), computed once."""
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
    """Closest distance (m, negative inside) from any point to any shape of the part (cylinders as their boxes)."""
    from yam_arena import analysis

    near = np.linalg.norm(pts - np.asarray(st.pos, float), axis=1) < 0.35
    if not near.any():
        return 0.10
    pts = pts[near]
    return float(min(analysis.box_signed_distance(pts, c, r, s).min() for _, c, r, s in _part_boxes(part, st)))


def gaps(objects, pts):
    return gap_to(PACK, objects["pack"], pts), gap_to(DRILL, objects["drill"], pts)


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


def _arm_debug(sim) -> dict:
    """Finger joints and gripper-link positions of both arms, for trace files only (never scored)."""
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = {}
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        q = parts._np(asset.data.joint_pos)[0]
        names = list(asset.joint_names)
        poses = parts._np(asset.data.body_link_pose_w)[0]
        g = poses[list(asset.body_names).index("gripper"), :3] - origin
        out[arm] = dict(f=[round(float(q[names.index(n)]), 4) for n in ("joint7", "joint8")],
                        g=[round(float(v), 4) for v in g])
    return out


def _tip_gaps(sim, objects) -> dict:
    """Each finger's closest distance (mm) to each pack button and to the pack's body, for logs only."""
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    boxes = _part_boxes(PACK, objects["pack"])
    out = {}
    from yam_arena import analysis

    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right"):
                pts = link_points(n) @ _rot(poses[i, 3:]).T + (poses[i, :3] - origin)
                d = {}
                for link, c, r, sz in boxes:
                    v = float(analysis.box_signed_distance(pts, c, r, sz).min())
                    d[link] = min(d.get(link, 1.0), v)
                out[f"{arm[0]}_{n}"] = {k: round(v * 1000, 1) for k, v in d.items()}
    return out


# -- simulator hooks -------------------------------------------------------------------------------------------------
def _write_pose(base, name, x, y, z, a) -> None:
    import torch

    obj = base.scene[name]
    pose = torch.tensor([[x, y, z, 0.0, 0.0, math.sin(a / 2), math.cos(a / 2)]], dtype=torch.float32,
                        device=base.device)
    pose[:, :3] += base.scene.env_origins[0][:3].to(pose.dtype)
    obj.write_root_pose_to_sim_index(root_pose=pose)
    obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=base.device))


def _seat_all(base) -> None:
    lot = lots.current()
    for name in ("drill", "pack", "tray"):
        _write_pose(base, name, *lot.pos(name), lot.yaw_of(name))


def _apply_exact(emb) -> bool:
    """A new trial's layout: every object written exactly at its lot's pose (the pack seated on the drill's rails),
    settled, written again and rendered, so the trial's first frame shows the true start."""
    base = emb._base
    for _ in range(2):
        _seat_all(base)
        for _ in range(max(1, round(0.25 / base.step_dt))):
            emb._env.step(emb._ready)
    for _ in range(lots.RENDERS):
        base.sim.render()
    return True


lots._apply = _apply_exact


@parts.on_reset
def _new_trial(sim) -> None:
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
    pts = None
    try:
        pts = _hand_points(sim)
        g = gaps(objects, pts) if pts is not None else None
    except Exception:  # noqa: BLE001
        g = None
    observe(objects, g, float(sim.t))
    log = os.environ.get("PACK_LOG") or (str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("pack.jsonl"))
                                         if os.environ.get("TRACE_OUT") else None)
    if log and _rec["steps"] % 5 == 0 and objects:
        try:
            row = dict(t=round(float(sim.t), 3), score=score(objects), rel=np.round(pack_in_drill(objects), 4).tolist(),
                       btn=[round(v, 4) for v in buttons_in(objects)], pressed=_rec["pressed"], fell=_rec["fell"],
                       known=_rec["known"], reason=_rec["reason"], gap_pack=_rec["gap_pack"],
                       gap_drill=_rec["gap_drill"], in_tray=in_tray(objects), standing=drill_standing(objects),
                       drill_tilt=round(_tilt_deg(objects["drill"]), 2), drill_low=round(float(objects["drill"].low[2])
                                                                                          - TOP, 4),
                       pack_tilt=round(_tilt_deg(objects["pack"]), 2), held=_rec["held"], arms=_arm_debug(sim))
            row["tips"] = _tip_gaps(sim, objects)
            with open(log, "a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with open(log, "a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- demo: open-loop from the start layout the cameras show -------------------------------------------------------------
# The arm on the chuck's side squeezes the two buttons in one pinch and draws the pack out along the rails while the
# other arm holds the drill by its grip; then the drill is set down on the bench and the pack is carried to the tray.
OPEN = FINGER_OPEN
PINCH = -(PACK_W / 2 - 0.011)  # 11 mm past the pack's faces: the fingers squeeze hard (their 50 N limit)
CARRY = -(PACK_W / 2 - 0.004)  # carrying: 4 mm past the faces, so the pack stays centred
HANDLE_GRIP = -(HANDLE_HX - 0.010)  # 10 mm past the grip's faces: a firm hold (the fingers' 50 N limit)
LET_GO = -0.046
PUSH_LEAN, HOLD_LEAN = 45.0, 55.0  # degrees off straight down
# The leaning right arm settles about 3.5 mm too far out in x at the buttons (measured in lot B traces); the left
# arm settles on target. Aim the right arm that much nearer, so both fingers meet both buttons.
PINCH_X_TRIM = {"left": 0.0, "right": -0.0035}
GRIP_Z = 0.097  # the drill's grip is taken 97 mm above the bench
PULL_1, PULL_2 = 0.076, 0.165  # m along the rails: the tongue clear of the rails, then the pack clear of the foot
LIFT = 0.050  # the holding arm's command once the rails let go (the loaded arm sags; the drill stays about level)
SET_Z = GRIP_Z - RAIL_Z - 0.003  # the grip's height with the drill on the bench, 3 mm lower: the bench takes the
# drill's weight before the fingers open, so the unloaded arm does not spring up with the drill still in its fingers
CARRY_Z, PACK_GRIP_Z = 0.130, 0.022  # (the arm settles about 2 cm above a low target; aim low)
PACK_GRIP_DY = -0.025  # the carry grip: 25 mm short of where the pack's middle is drawn to (the pull falls about
# 8 mm short), so the fingers stay clear of the buttons at the pack's chuck end
X_TRIM = 0.005  # the arm settles about 5 mm short in x over the pack; aim that much farther out
# Where the pack's middle ends up beyond the carry grip, toward the old chuck side, as measured in traces: the left
# arm draws the pack out to about 8 mm short of the command, the right arm (leaning the other way) about 31 mm short.
GRIP_OFF = {"left": 0.017, "right": -0.006}
SET_BACK = 0.050  # the drill is stood back down 5 cm toward the holding arm, clear of the pack


PARK = {"left": (0.20, 0.25, TOP + 0.20), "right": (0.20, -0.25, TOP + 0.20)}


def _rest_pose(arm):
    """The TCP position and rotation of the arm's rest joint pose (READY), as a waypoint position and yaw."""
    from yam_arena import ik, station
    from yam_arena.demo import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return tuple(float(v) for v in pos), tilt._TiltedYaw(0.0, rot)


def _frame(lot):
    sx, sy, _ = lot.pos("drill")
    a = lot.yaw_of("drill")
    return float(sx), float(sy), math.cos(a), math.sin(a)


def _w(lot, p):
    """A drill-frame point (x across, y toward the chuck, z up from the bench) to world coordinates."""
    sx, sy, c, s = _frame(lot)
    return (sx + c * p[0] - s * p[1], sy + s * p[0] + c * p[1], TOP + p[2])


def _dir(lot, ang):
    """World heading of a drill-frame heading."""
    return ang + lot.yaw_of("drill")


def _back(pos, lean, heading, d):
    """``d`` m back from ``pos`` along a gripper leaned ``lean`` degrees toward ``heading``."""
    t = math.radians(lean)
    return (pos[0] - d * math.sin(t) * math.cos(heading), pos[1] - d * math.sin(t) * math.sin(heading),
            pos[2] + d * math.cos(t))


def demo():
    lot = lots.current()
    _, _, c, s = _frame(lot)
    chuck_y = c  # world y of the drill's +y axis (it lies along world y)
    puller = "left" if chuck_y > 0 else "right"
    holder = "right" if puller == "left" else "left"
    yaws = {"right": math.pi / 2, "left": -math.pi / 2}  # both grips close across the drill (along its x)
    tray = lot.pos("tray")

    # holder: lean in toward the chuck, under the motor housing, and pinch the grip across.
    h_head = _dir(lot, math.pi / 2)
    grip = _w(lot, (0.0, HANDLE_Y, GRIP_Z))
    hw = lambda d, pos, f, lab: tilt.Waypoint(d, pos, f, lab, yaw=yaws[holder], tilt=HOLD_LEAN, heading=h_head)  # noqa: E731
    hold = [
        hw(2.5, _back(grip, HOLD_LEAN, h_head, 0.07), OPEN, "behind the grip"),
        hw(1.2, grip, OPEN, "fingers round the grip"),
        hw(0.8, grip, HANDLE_GRIP, "hold the drill"),
    ]
    # puller: lean in from the chuck's side, low, and pinch the two buttons.
    p_head = _dir(lot, -math.pi / 2)
    btn = (0.0, sum(BTN_Y) / 2, sum(BTN_Z) / 2)
    yaw = yaws[puller]
    tx = PINCH_X_TRIM[puller]
    pw = lambda d, pos, f, lab: tilt.Waypoint(d, (pos[0] + tx, pos[1], pos[2]), f, lab, yaw=yaw, tilt=PUSH_LEAN, heading=p_head)  # noqa: E731
    at_btn = _w(lot, btn)
    pull = [
        pw(2.5, _back(at_btn, PUSH_LEAN, p_head, 0.07), OPEN, "out from the buttons"),
        pw(1.2, at_btn, OPEN, "fingers either side of the buttons"),
        pw(0.9, at_btn, OPEN, "wait for the drill to be held"),
        pw(0.8, at_btn, PINCH, "squeeze both buttons"),
        pw(0.6, at_btn, PINCH, "keep them squeezed"),
        pw(1.5, _w(lot, (0.0, btn[1] + PULL_1, btn[2])), PINCH, "slide the pack along the rails"),
    ]
    t1 = sum(w.duration for w in pull)
    hold.append(hw(max(0.3, t1 - sum(w.duration for w in hold)), grip, HANDLE_GRIP, "keep the drill still"))
    lifted = (grip[0], grip[1], grip[2] + LIFT)
    hold.append(hw(1.0, lifted, HANDLE_GRIP, "take the drill's weight"))
    pull.append(pw(1.0, _w(lot, (0.0, btn[1] + PULL_1, btn[2])), PINCH, "wait"))
    pull.append(pw(1.5, _w(lot, (0.0, btn[1] + PULL_2, btn[2])), PINCH, "draw the pack clear of the foot"))
    t2 = sum(w.duration for w in pull)
    hold.append(hw(max(0.3, t2 - sum(w.duration for w in hold)), lifted, HANDLE_GRIP, "hold it up"))
    gb = _w(lot, (0.0, HANDLE_Y - SET_BACK, GRIP_Z))
    down = (gb[0], gb[1], TOP + SET_Z)
    hold += [
        hw(1.5, down, HANDLE_GRIP, "stand the drill on the bench"),
        hw(0.6, down, HANDLE_GRIP, "rest it there"),
        hw(0.8, down, OPEN, "let go of the drill"),
        hw(1.0, _back(down, HOLD_LEAN + 20.0, h_head, 0.05), OPEN, "back out, low"),
        hw(0.8, _back(down, HOLD_LEAN + 5.0, h_head, 0.11), OPEN, "back and up"),
        Waypoint(1.5, PARK[holder], OPEN, "park", 0.0),
    ]
    end = _w(lot, (0.0, btn[1] + PULL_2, btn[2]))
    pc = _w(lot, (0.0, PULL_2 + PACK_GRIP_DY, PACK_GRIP_Z))  # the carry grip on the drawn-out pack
    pc = (pc[0] + X_TRIM, pc[1], pc[2])
    tray_at = (tray[0], tray[1] - chuck_y * GRIP_OFF[puller])
    set_z = TOP + TRAY_FLOOR + 0.006 + PACK_GRIP_Z
    pull += [
        pw(0.6, end, LET_GO, "let go of the buttons"),
        pw(0.8, _back(end, PUSH_LEAN, p_head, 0.06), LET_GO, "back off"),
    ]
    t_clear = sum(w.duration for w in hold[:-1])  # the holding arm has backed out of the drill
    t_now = sum(w.duration for w in pull)
    back = _back(end, PUSH_LEAN, p_head, 0.06)
    pull += [
        pw(max(0.3, t_clear - t_now), back, LET_GO, "wait for the drill to be set down"),
        Waypoint(1.5, (pc[0], pc[1], TOP + CARRY_Z), OPEN, "over the pack", yaw),
        Waypoint(1.5, pc, OPEN, "fingers either side of the pack", yaw),
        Waypoint(0.5, pc, OPEN, "settle", yaw),
        Waypoint(0.8, pc, CARRY, "take the pack", yaw),
        Waypoint(0.8, (pc[0], pc[1], TOP + CARRY_Z), CARRY, "lift it", yaw),
        Waypoint(1.6, (*tray_at, TOP + CARRY_Z), CARRY, "over the tray", yaw),
        Waypoint(1.2, (*tray_at, set_z), CARRY, "lower it to the floor", yaw),
        Waypoint(0.6, (*tray_at, set_z), LET_GO, "let go", yaw),
        Waypoint(0.8, (*tray_at, TOP + 0.13), OPEN, "up", yaw),
        Waypoint(1.5, PARK[puller], OPEN, "park", 0.0),
        Waypoint(2.0, PARK[puller], OPEN, "wait", 0.0),
    ]
    # Every trial starts by taking both arms to their rest pose (where a fresh trial starts them; after an earlier
    # trial they start from its park pose), so each arm's approach to the drill is the same whatever ran before.
    for arm, plan in ((puller, pull), (holder, hold)):
        rest = _rest_pose(arm)
        plan[:0] = [Waypoint(2.0, rest[0], OPEN, "to the rest pose", rest[1]),
                    Waypoint(0.5, rest[0], OPEN, "settle at the rest pose", rest[1])]
    return {puller: pull, holder: hold}
