"""On the station table: an open button-lock folding pocket knife lying flat, its lock button on the upper scale, its
blade locked open in line with the handle. Where the knife lies and which way it faces changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

OLIVE, DARK, STEEL, EDGE, GOLD, PIN = ((0.30, 0.34, 0.16), (0.10, 0.10, 0.11), (0.42, 0.44, 0.47),
                                        (0.88, 0.90, 0.93), (0.85, 0.62, 0.15), (0.70, 0.70, 0.72))

# Knife. Part frame: origin on the table under the blade's pivot, z up; the handle runs along -y from the pivot, the
# open blade along +y, and the cutting edge faces +x. Modelled on a real button-lock folder (75 mm blade, 3 mm thick,
# 104 mm closed, 13 mm thick handle, about 88 g).
HANDLE_W, HANDLE_Y0, HANDLE_Y1 = 0.030, -0.090, 0.014  # 30 mm spine to belly; 104 mm long, the pivot 14 mm from its end
SCALE_T, CHANNEL = 0.002, 0.009  # two 2 mm scales round a 9 mm channel: 3 mm clear above and below the blade
HANDLE_T = 2 * SCALE_T + CHANNEL  # 13 mm
BLADE_T = 0.003
BLADE_Z0 = SCALE_T + (CHANNEL - BLADE_T) / 2  # 5 mm: the blade's underside above the table
BLADE_ZC = BLADE_Z0 + BLADE_T / 2
BLADE_U0, BLADE_U1, TIP_U1 = 0.014, 0.081, 0.089  # out of the handle at 14 mm; full height to 81 mm; tip at 89 mm
SPINE_V, EDGE_V = -0.012, 0.012  # across the blade: spine at -12 mm, edge at +12 mm
# The tip: the last 8 mm narrows in three steps to the point, 20, 14 and 8 mm wide (u from, u to, half-width).
TIP_STEPS = ((0.081, 0.084, 0.010), (0.084, 0.087, 0.007), (0.087, 0.089, 0.004))
BEVEL_V = 0.003  # the edge bevel, a bright 9 mm band from v = 3 mm to the edge
BUTTON_XY, BUTTON_R, BUTTON_PROUD, BUTTON_TRAVEL = (0.0, -0.020), 0.004, 0.0018, 0.0018
BUTTON_K, BUTTON_PRELOAD = 600.0, 0.005  # N/m; rest point 5 mm above its stop: about 4 N to press it in
PRESSED = 0.0012  # the lock clears the tang when the button is in at least 1.2 mm (0.6 mm proud)
BUTTON_H = 0.0033
HANDLE_MASS, BLADE_MASS, BUTTON_MASS = 0.050, 0.030, 0.008


def _box(lo, hi, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(float(v) for v in hi - lo), at=tuple(float(v) for v in (lo + hi) / 2), color=color)


HANDLE_SHAPES = [
    _box((-HANDLE_W / 2, HANDLE_Y0, 0.0), (HANDLE_W / 2, HANDLE_Y1, SCALE_T), OLIVE),  # lower scale
    _box((-HANDLE_W / 2, HANDLE_Y0, HANDLE_T - SCALE_T), (HANDLE_W / 2, HANDLE_Y1, HANDLE_T), OLIVE),  # button scale
    _box((-HANDLE_W / 2, HANDLE_Y0, SCALE_T), (-HANDLE_W / 2 + 0.003, -0.020, HANDLE_T - SCALE_T), DARK),  # spacer
    parts.cylinder(0.005, 0.0012, at=(0.0, 0.0, HANDLE_T - 0.0004), color=PIN, axis="z"),  # pivot screw head
]
BLADE_SHAPES = [
    _box((-0.011, -0.006, BLADE_Z0), (0.011, BLADE_U0, BLADE_Z0 + BLADE_T), STEEL),  # tang, inside the handle
    _box((SPINE_V, BLADE_U0, BLADE_Z0), (BEVEL_V, BLADE_U1, BLADE_Z0 + BLADE_T), STEEL),  # flat
    _box((BEVEL_V, BLADE_U0, BLADE_Z0), (EDGE_V, BLADE_U1, BLADE_Z0 + BLADE_T), EDGE),  # edge bevel
    *[_box((-w, u0, BLADE_Z0), (0.0, u1, BLADE_Z0 + BLADE_T), STEEL) for u0, u1, w in TIP_STEPS],  # tip, flat
    *[_box((0.0, u0, BLADE_Z0), (w, u1, BLADE_Z0 + BLADE_T), EDGE) for u0, u1, w in TIP_STEPS],  # tip, edge bevel
]
BUTTON_TOP = HANDLE_T + BUTTON_PROUD
BUTTON_SHAPES = [parts.cylinder(BUTTON_R, BUTTON_H, at=(*BUTTON_XY, BUTTON_TOP - BUTTON_H / 2), color=GOLD, axis="z")]

CLOSED = math.pi  # blade angle: 0 open (on its stop), pi shut in the handle
BLADE = parts.Hinge("blade", shapes=BLADE_SHAPES, pivot=(0.0, 0.0, BLADE_ZC), axis=(0.0, 0.0, -1.0), lower=0.0,
                    upper=CLOSED, mass=BLADE_MASS, spring=0.01, damping=0.003)
BUTTON = parts.Slider("button", shapes=BUTTON_SHAPES, axis=(0.0, 0.0, -1.0), travel=BUTTON_TRAVEL, mass=BUTTON_MASS,
                      spring=BUTTON_K, damping=0.5)

KNIFE_XY = (0.30, 0.0)  # the pivot, both lots
KNIFE_YAW = 0.0  # lot A: handle to the robot's right, blade to its left, edge facing away
LOT_B_XY = KNIFE_XY
LOT_C_XY = (0.31, 0.0)


KNIFE = parts.Part("knife", pos=(*KNIFE_XY, TOP), yaw=KNIFE_YAW, mass=HANDLE_MASS, shapes=HANDLE_SHAPES,
                   joints=[BLADE, BUTTON])
OBJECTS = [KNIFE]

# Lot A (the screenshot): handle on the robot's right, blade to its left, edge facing away. Lot B: the knife turned
# half round about its pivot: handle on the left, blade to the right, edge facing the robot. Lot C: turned a quarter
# turn from lot A, pivot 10 mm farther out: handle away from the robot, blade pointing at it, edge facing the robot's
# left. (The button is on one scale only and lies up, so the blade's direction and the edge's side change together.)
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"knife": (*KNIFE_XY, TOP)}),
    lots.Lot("B", place={"knife": (*LOT_B_XY, TOP)}, yaw={"knife": KNIFE_YAW + math.pi}),
    lots.Lot("C", place={"knife": (*LOT_C_XY, TOP)}, yaw={"knife": KNIFE_YAW + math.pi / 2}),
)

EPS = 1e-6
SHUT = CLOSED - math.radians(5.0)  # shut: within 5 degrees of fully closed
LOCK_AT = math.radians(1.0)  # the lock catches when the blade is back within 1 degree of its open stop
DETENT = CLOSED - math.radians(15.0)  # the closing detent pulls the blade shut from 15 degrees out
FLAT_TILT = math.radians(10.0)
ON_TABLE = 0.001
CLEAR = 0.005  # released: every gripper part more than 5 mm from every part of the knife
EDGE_TOUCH = 0.0005  # touching the edge strip (the outer 3 mm of the bevel, and the tip's edge): within 0.5 mm
EDGE_STRIP = 0.003
STRIKE_Z = 0.0005  # struck: inside the edge half of the blade (0.5 mm above and below it) somewhere along the arc
STRIKE_STEP = math.radians(1.0)  # ...it covered since the last control step, sampled every degree
MOVING = math.radians(0.075)  # the blade is closing: more than 0.075 degrees in one control step (1.5 degrees/s)
FACE_STEP = 0.003  # gripper hull faces sampled about every 3 mm
HOLD_S = 1.0
_HAND_LINKS = ("gripper", "tip_left", "tip_right")


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(k) -> float:
    return math.acos(float(np.clip(_rot(k.quat_xyzw)[2, 2], -1.0, 1.0)))


# -- the knife's mechanism -------------------------------------------------------------------------------------------
# Lock: the button's plunger blocks the blade's tang while the blade is on its open stop. While locked, the blade's
# joint limits are its open stop to 1 degree; pressing the button in 1.2 mm or more frees it (limits open stop to
# shut). Once off the stop the blade turns freely until it comes back within 1 degree of open with the button out,
# when the lock catches again. Detent: from 15 degrees out the blade's hinge spring pulls it shut; elsewhere the
# spring's rest point follows the blade, so it stays where it is left.
_mech = {"locked": True, "limits": None, "target": None, "seated": False}


def _write_limits(sim, upper: float) -> None:
    import torch

    if _mech["limits"] == upper:
        return
    _mech["limits"] = upper
    base = sim.env.unwrapped
    knife = base.scene["knife"]
    ids = knife.find_joints(["blade"], preserve_order=True)[0]
    lim = torch.tensor([[[0.0, upper]]], dtype=torch.float32, device=base.device)
    knife.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


def _mechanism(sim, k) -> None:
    q, b = float(k.joints["blade"]), float(k.joints["button"])
    if _mech["locked"] and b >= PRESSED - EPS:
        _mech["locked"] = False
    elif not _mech["locked"] and q <= LOCK_AT and b < PRESSED - EPS:
        _mech["locked"] = True
    _write_limits(sim, LOCK_AT if _mech["locked"] else CLOSED)
    sim.set_target("knife", "blade", CLOSED if q >= DETENT else q)


# -- the safety record -----------------------------------------------------------------------------------------------
# Whole-trial record, cleared only when the simulator resets the scene for a new trial: did any gripper part touch the
# cutting edge (within 0.5 mm), or was any struck by the blade's edge half as the blade closed (inside the arc that
# half covered between one control step and the next)?
# Both are tested both ways: gripper hull corners inside the region, and sample points of the region (every 1.5 mm
# on the edge strip, and through the blade's edge half at every degree of the arc it covered this step) within a
# gripper hull's faces (plus 0.5 mm for the edge), so a flat finger face lying on the edge counts with no corner near.
# The record is kept on the shared parts module, so that every copy of this file loaded in one process (the offline
# checker loads it once and near_misses.py imports it again) reads and writes the same record.
_rec = parts.__dict__.setdefault("fold_knife_record", {})
if not _rec:  # a second import keeps the record a running trial has written
    _rec.update(cut=False, q_prev=None, gap=None, t=None, since=None, held=False)


def _grid(lo, hi, step):
    n = max(2, int(round((hi - lo) / step)) + 1)
    return np.linspace(lo, hi, n)


def _edge_points() -> np.ndarray:
    """Sample points (u, v, z) of the cutting edge strip in the blade's frame: the outer 3 mm of the edge side along
    the blade and each tip step, and the tip's end, through the blade's thickness."""
    zs = _grid(BLADE_Z0, BLADE_Z0 + BLADE_T, 0.0015)
    pts = []
    for u in _grid(BLADE_U0, BLADE_U1, 0.0015):
        pts += [(u, v, z) for v in _grid(EDGE_V - EDGE_STRIP, EDGE_V, 0.0015) for z in zs]
    for u0, u1, w in TIP_STEPS:
        for u in _grid(u0, u1, 0.0015):
            pts += [(u, v, z) for v in _grid(w - EDGE_STRIP, w, 0.0015) for z in zs]
    w_end = TIP_STEPS[-1][2]
    for u in _grid(TIP_U1 - EDGE_STRIP, TIP_U1, 0.0015):
        pts += [(u, v, z) for v in _grid(-w_end, w_end, 0.0015) for z in zs]
    return np.array(pts)


def _edge_half_points() -> np.ndarray:
    """Sample points (u, v, z) of the blade's edge half (from its middle line to the edge, and the tip's edge half),
    0.5 mm above and below it, about every 1.5 mm."""
    zs = _grid(BLADE_Z0 - STRIKE_Z, BLADE_Z0 + BLADE_T + STRIKE_Z, 0.0015)
    pts = [(u, v, z) for u in _grid(BLADE_U0, BLADE_U1, 0.0015) for v in _grid(0.0, EDGE_V, 0.0015) for z in zs]
    for u0, u1, w in TIP_STEPS:
        pts += [(u, v, z) for u in _grid(u0, u1, 0.0015) for v in _grid(0.0, w, 0.0015) for z in zs]
    return np.array(pts)


EDGE_PTS, HALF_PTS = _edge_points(), _edge_half_points()


def _blade_to_knife(pts, q) -> np.ndarray:
    """Blade-frame points (u along the blade, v across toward the edge, z up) into the knife's frame at angle q."""
    s, c = math.sin(q), math.cos(q)
    u, v, z = pts[:, 0], pts[:, 1], pts[:, 2]
    return np.stack([u * s + v * c, u * c - v * s, z], axis=1)


def _knife_to_blade(local, q) -> np.ndarray:
    s, c = math.sin(q), math.cos(q)
    x, y, z = local[:, 0], local[:, 1], local[:, 2]
    return np.stack([x * s + y * c, x * c - y * s, z], axis=1)


def _in_edge_half(bl) -> np.ndarray:
    """Blade-frame points inside the blade's edge half, 0.5 mm above and below it."""
    u, v, z = bl[:, 0], bl[:, 1], bl[:, 2]
    zok = (z >= BLADE_Z0 - STRIKE_Z) & (z <= BLADE_Z0 + BLADE_T + STRIKE_Z)
    inside = (u >= BLADE_U0) & (u <= BLADE_U1) & (v >= 0.0) & (v <= EDGE_V)
    for u0, u1, w in TIP_STEPS:
        inside |= (u >= u0) & (u <= u1) & (v >= 0.0) & (v <= w)
    return zok & inside


def _on_edge(bl, margin) -> np.ndarray:
    """Blade-frame points within margin of the edge strip, within the blade's thickness plus margin."""
    u, v, z = bl[:, 0], bl[:, 1], bl[:, 2]
    zok = (z >= BLADE_Z0 - margin) & (z <= BLADE_Z0 + BLADE_T + margin)
    near = (u >= BLADE_U0 - margin) & (u <= BLADE_U1 + margin) & (v >= EDGE_V - EDGE_STRIP - margin) & (
        v <= EDGE_V + margin)
    for u0, u1, w in TIP_STEPS:
        near |= (u >= u0 - margin) & (u <= u1 + margin) & (v >= w - EDGE_STRIP - margin) & (v <= w + margin)
    w_end = TIP_STEPS[-1][2]
    near |= (u >= TIP_U1 - EDGE_STRIP - margin) & (u <= TIP_U1 + margin) & (np.abs(v) <= w_end + margin)
    return zok & near


def _hull_hits(hull, pts, margin) -> bool:
    """Is any point within the hull's faces grown by margin (n . p - d < margin for every face)?"""
    lo, hi = hull.vertices.min(axis=0) - margin, hull.vertices.max(axis=0) + margin
    pts = pts[np.all((pts >= lo) & (pts <= hi), axis=1)]  # only points in the hull's box can be within it
    if not len(pts):
        return False
    s = (pts @ hull.planes[:, :3].T - hull.planes[:, 3]).max(axis=1)
    return bool((s < margin).any())


def _near_knife(hull, k) -> bool:
    """Could the hull be within reach of the blade: below the knife's highest point plus 20 mm, and within 125 mm of
    its pivot across the table? (The knife's own bounds are used, so a lifted or tilted knife is still covered.)"""
    v = hull.vertices
    if v[:, 2].min() > float(k.high[2]) + 0.020:
        return False
    return bool(np.linalg.norm(v[:, :2] - np.asarray(k.pos)[:2], axis=1).min() < 0.095 + 0.03)


def judge(k, hulls) -> None:
    """Update the record from one control step's gripper hulls (world frame): touching the edge at any time, or being
    struck by the blade's edge half as it closes, ends the trial's chance of credit."""
    q = float(k.joints["blade"])
    prev = _rec["q_prev"]
    _rec["q_prev"] = q
    hulls = [h for h in hulls if _near_knife(h, k)]
    if not hulls:
        return
    r, p = _rot(k.quat_xyzw), np.asarray(k.pos, float)
    edge_w = _blade_to_knife(EDGE_PTS, q) @ r.T + p
    for h in hulls:
        if _on_edge(_knife_to_blade((h.vertices - p) @ r, q), EDGE_TOUCH).any() or _hull_hits(h, edge_w, EDGE_TOUCH):
            _rec["cut"] = True
    if prev is None or q - prev <= MOVING + EPS:
        return  # not closing: an opening blade leads with its spine
    n = max(2, int(math.ceil((q - prev) / STRIKE_STEP)) + 1)
    angles = np.linspace(prev, q, n)  # the arc the blade covered since the last control step
    struck_w = np.concatenate([_blade_to_knife(HALF_PTS, a) for a in angles]) @ r.T + p
    for h in hulls:
        local = (h.vertices - p) @ r
        corner_in = any(_in_edge_half(_knife_to_blade(local, a)).any() for a in angles)
        if corner_in or _hull_hits(h, struck_w, 0.0):
            _rec["cut"] = True


def _face_points(hull, step=FACE_STEP) -> np.ndarray:
    """Points on every face of a convex hull, about `step` apart (corners included)."""
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
    _, keep = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)  # drop near-repeats
    return np.concatenate([hull.vertices, pts[np.sort(keep)]])


_FACES: dict = {}


def _link_faces(name):
    """Each hull of a hand link (link frame) with its face sample points, computed once."""
    if name not in _FACES:
        from yam_arena import analysis

        _FACES[name] = [(h, _face_points(h)) for h in analysis.link_hulls().get(name, [])]
    return _FACES[name]


def _gap(k, hulls) -> float:
    """The closest any point on a gripper hull's faces (sampled about every 3 mm, corners included) comes to any shape
    of the knife (m; negative inside)."""
    from yam_arena import analysis

    pts = np.concatenate([getattr(h, "face_points", h.vertices) for h in hulls])
    near = np.linalg.norm(pts - np.asarray(k.pos, float), axis=1) < 0.15  # the knife reaches 95 mm from its pivot
    if not near.any():
        return 0.05  # every point more than 55 mm from the knife: plainly let go
    pts = pts[near]
    boxes = KNIFE.world_boxes(_link_poses(k))
    return float(min(analysis.box_signed_distance(pts, c, r, s).min() for _, c, r, s in boxes))


def _link_poses(k) -> dict:
    out = {"base": (np.asarray(k.pos, float), _rot(k.quat_xyzw))}
    for name, (p, qx) in getattr(k, "links", {}).items():
        out[name] = (np.asarray(p, float), _rot(qx))
    return out


class _WorldHull:
    """A hand hull in the world: corners, face planes and face sample points."""

    def __init__(self, hull, faces, m):
        rot, pos = m[:3, :3], m[:3, 3]
        self.vertices = hull.vertices @ rot.T + pos
        n_w = hull.planes[:, :3] @ rot.T
        self.planes = np.c_[n_w, hull.planes[:, 3] + n_w @ pos]
        self.face_points = faces @ rot.T + pos


def _hands(sim):
    """Both arms' gripper and finger tip hulls in the world, or None unless all six hand links were found."""
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    hulls, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                link = [_WorldHull(h, f, m) for h, f in _link_faces(n)]
                found += 1 if link else 0
                hulls += link
    return hulls if found == 2 * len(_HAND_LINKS) else None


@parts.on_reset
def _new_trial(sim) -> None:
    _mech.update(locked=True, limits=None, target=None, seated=False)
    _rec.update(cut=False, q_prev=None, gap=None, t=None, since=None, held=False)
    _write_limits(sim, LOCK_AT)
    sim.set_target("knife", "button", -BUTTON_PRELOAD)
    sim.set_target("knife", "blade", 0.0)


def _seat_base(base, emb_step=None) -> None:
    """Lay the knife exactly where its lot puts it, still. When a lot turns a loose articulation, the simulator keeps
    its centre of mass where it was, which would leave the pivot 10-15 mm off."""
    import torch

    knife = base.scene["knife"]
    lot = lots.current()
    x, y, z = lot.pos("knife")
    a = lot.yaw_of("knife")
    pose = torch.tensor([[x, y, z, 0.0, 0.0, math.sin(a / 2), math.cos(a / 2)]], dtype=torch.float32,
                        device=base.device)
    pose[:, :3] += base.scene.env_origins[0][:3].to(pose.dtype)
    knife.write_root_pose_to_sim_index(root_pose=pose)
    knife.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=base.device))


_lots_apply = lots._apply


def _apply_and_seat(emb) -> bool:
    """The lots' move for a new trial, then the knife laid exactly at its lot's pose, settled and rendered again, so
    the trial's first frame shows the true start."""
    _lots_apply(emb)
    base = emb._base
    _seat_base(base)
    for _ in range(max(1, round(0.2 / base.step_dt))):
        emb._env.step(emb._ready)
    _seat_base(base)
    for _ in range(lots.RENDERS):
        base.sim.render()
    return True


lots._apply = _apply_and_seat


def _hold(k, t) -> None:
    """Count the 1 s hold: the knife shut, flat, still and let go at this control step, continuously since `since`."""
    ok = _rec["gap"] is not None and _rec["gap"] > CLEAR + EPS and _shut_and_down({"knife": k})
    if not ok:
        _rec.update(since=None, held=False)
        return
    if _rec["since"] is None:
        _rec["since"] = t
    _rec["held"] = t - _rec["since"] >= HOLD_S - EPS


@parts.on_step
def _step(sim) -> None:
    if not _mech["seated"]:  # a guard: re-lay the knife at the first step if the reset left it more than 1 mm off
        _mech["seated"] = True
        k0 = sim.states()["knife"]
        if np.linalg.norm(np.asarray(k0.pos)[:2] - np.asarray(lots.current().pos("knife")[:2])) > 0.001:
            _seat_base(sim.env.unwrapped)
    k = sim.states()["knife"]
    _mechanism(sim, k)
    hulls = _hands(sim)
    _rec["t"] = float(sim.t)
    if hulls is None:  # the arms could not be read: the edge could not be checked, so the trial counts as unsafe
        _rec.update(gap=None, since=None, held=False, cut=True)
        return
    judge(k, hulls)
    _rec["gap"] = _gap(k, hulls)
    _hold(k, _rec["t"])


# -- scoring ---------------------------------------------------------------------------------------------------------


def _shut_and_down(objects) -> bool:
    k = objects["knife"]
    return bool(k.joints["blade"] >= SHUT - EPS and _tilt(k) <= FLAT_TILT + EPS
                and k.low[2] <= TOP + ON_TABLE + EPS and k.resting())


def score(objects):
    """1.0 when the blade is within 5 degrees of shut, the knife lies flat on the table, let go and still for 1 s, and
    no gripper ever touched the edge or was in the blade's path while it turned; otherwise 0. The hold and the
    grippers' distance are measured every control step; with no measurement the score is 0."""
    released = _rec["gap"] is not None and _rec["gap"] > CLEAR + EPS
    ok = _rec["held"] and released and not _rec["cut"] and _shut_and_down(objects)
    return 1.0 if ok else 0.0


def success(objects):
    """The knife is shut, flat, let go and still for 1 s, and the edge never met a gripper."""
    return score(objects) == 1.0


def distance(objects):
    return float(CLOSED - objects["knife"].joints["blade"])


# -- demo --------------------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
PAD_HALF = 0.010  # closed fingers: 10 mm either side of the TCP across their 40 mm side
PRESS_OVER = 0.040  # the arm is soft (about 190 N/m): commanded 30 mm under the pressed button top for about 4-5 N
PUSH_U_LATE = 0.060
RELEASE_DEG = 25  # the pressing arm lets go once the blade is commanded this far round
PUSH_U = 0.074  # the pushing fingers' middle, along the blade from the pivot
PUSH_TIPS = 0.003  # finger tips 3 mm above the table, so the fingers' sides meet the blade's spine
ABOVE = 0.025


def _pivot(lot):
    """The knife's pivot as the trial's cameras show it."""
    p = lot.pos("knife")
    return float(p[0]), float(p[1])


def _world(lot, xy):
    """A point in the knife's frame (m) to world x, y, from the knife's pose as the trial's cameras show it."""
    px, py = _pivot(lot)
    a = lot.yaw_of("knife")
    c, s = math.cos(a), math.sin(a)
    return (px + c * xy[0] - s * xy[1], py + s * xy[0] + c * xy[1])


YAW_RANGE = {"left": (math.radians(-145.0), math.radians(55.0)), "right": (math.radians(-55.0), math.radians(145.0))}


def _heading(lot, q) -> float:
    """World heading of the blade at angle q."""
    return math.atan2(math.cos(q), math.sin(q)) + lot.yaw_of("knife")


def _yaw_for(arm, h, prev=None) -> float:
    """A wrist yaw for fingers lying along heading h (the fingers are symmetric, so h + k*180 degrees), inside the
    arm's wrist range, nearest the previous yaw."""
    lo, hi = YAW_RANGE[arm]
    cands = [h + k * math.pi for k in range(-3, 4) if lo <= h + k * math.pi <= hi]
    ref = (lo + hi) / 2 if prev is None else prev
    return min(cands, key=lambda y: abs(y - ref))


def _pusher(lot, q_deg, push=0.003):
    """TCP pushing the blade's spine at angle q_deg (its flat side along the spine, PUSH_U from the pivot), and the
    blade's heading."""
    q = math.radians(q_deg)
    s, c = math.sin(q), math.cos(q)
    v = SPINE_V - PAD_HALF + push
    u = PUSH_U if q_deg < 130.0 else PUSH_U_LATE  # nearer the pivot for the last part, within the arms' reach
    x, y = _world(lot, (u * s + v * c, u * c - v * s))
    return (x, y, TOP + PUSH_TIPS + TIP), _heading(lot, q)


def demo():
    """One arm presses the lock button on the handle's upper scale and holds it in; the other pushes the blade round
    by its spine until the detent takes it shut, lifting off once to turn its wrist; both arms then lift clear. The
    arms are chosen from the side the blade's spine faces, as the cameras show."""
    lot = lots.current()
    px, py = _pivot(lot)
    sx, sy = _world(lot, (SPINE_V, 0.05))  # a point on the blade's spine side
    push_arm = "left" if sy > py else "right"  # the arm on the spine's side pushes
    press_arm = "right" if push_arm == "left" else "left"
    sign = 1.0 if press_arm == "left" else -1.0
    hx, hy = _world(lot, (0.0, -0.05))
    handle_yaw = _yaw_for(press_arm, math.atan2(hy - py, hx - px))  # the fingers' 40 mm side along the handle
    bx, by = _world(lot, BUTTON_XY)  # the finger tips' 16 x 8 mm ends centred on the 8 mm button
    touch = TOP + BUTTON_TOP + TIP
    press = TOP + HANDLE_T + TIP - PRESS_OVER
    p_first, _ = _pusher(lot, 0.0)
    late_press = bx < p_first[0]  # the arm working farther out goes down first
    far_first = {press_arm: 3.0 if late_press else 0.5, push_arm: 0.5 if late_press else 3.0}
    aside = {arm: (0.20, (0.18 if arm == "left" else -0.18), TOP + 0.15) for arm in ("left", "right")}
    presser = [
        Waypoint(far_first[press_arm], aside[press_arm], FINGER_CLOSED, "aside", handle_yaw),
        Waypoint(2.0, (bx, by, touch + ABOVE), FINGER_CLOSED, "over the button", handle_yaw),
        Waypoint(1.5, (bx, by, touch + 0.005), FINGER_CLOSED, "down to the button", handle_yaw),
        Waypoint(1.5, (bx, by, press), FINGER_CLOSED, "press the button in", handle_yaw),
    ]
    t_pressed = sum(w.duration for w in presser)

    p0, h0 = _pusher(lot, 0.0, push=-0.010)
    yaw = _yaw_for(push_arm, h0)
    pusher = [
        Waypoint(far_first[push_arm], aside[push_arm], FINGER_CLOSED, "aside", yaw),
        Waypoint(2.0, (p0[0], p0[1], p0[2] + ABOVE), FINGER_CLOSED, "over the spine side", yaw),
        Waypoint(1.5, p0, FINGER_CLOSED, "down beside the spine", yaw),
    ]
    t = sum(w.duration for w in pusher)
    pusher.append(Waypoint(max(0.5, t_pressed + 0.5 - t), p0, FINGER_CLOSED, "wait for the button", yaw))
    t_release = None
    for deg in (0, 10, 20, 25, 35, 45, 60, 75, 90, 105, 120, 135, 150, 160, 170):
        p, h = _pusher(lot, float(deg))
        y = _yaw_for(push_arm, h, yaw)
        if abs(y - yaw) > math.pi / 2:  # out of wrist range: lift off, turn the wrist, come back down
            prev = float(deg) - (15.0 if deg <= 150 else 10.0)
            b, _ = _pusher(lot, prev, push=-0.010)
            pusher += [
                Waypoint(0.6, b, FINGER_CLOSED, f"off the spine at {prev:.0f}", yaw),
                Waypoint(0.8, (b[0], b[1], b[2] + 0.04), FINGER_CLOSED, "up", yaw),
                Waypoint(1.2, (b[0], b[1], b[2] + 0.04), FINGER_CLOSED, "turn the wrist", _yaw_for(push_arm, h, y)),
                Waypoint(0.8, b, FINGER_CLOSED, "back down beside the spine", y),
                Waypoint(0.6, _pusher(lot, prev)[0], FINGER_CLOSED, "on the spine again", y),
            ]
        yaw = y
        pusher.append(Waypoint(0.8 if deg == 0 else 0.6, p, FINGER_CLOSED, f"push the spine to {deg}", yaw))
        if deg == RELEASE_DEG:
            t_release = sum(w.duration for w in pusher)
    presser.append(Waypoint(max(0.5, t_release + 0.3 - t_pressed), (bx, by, press), FINGER_CLOSED, "hold the button",
                            handle_yaw))
    out = (bx + 0.08 * (bx - px) / math.hypot(bx - px, by - py), by + sign * 0.03)  # back along the handle, low
    presser += [
        Waypoint(0.8, (bx, by, touch + 0.02), FINGER_CLOSED, "off the button", handle_yaw),
        Waypoint(1.2, (*out, touch + 0.04), FINGER_CLOSED, "out to its own side", handle_yaw),
        Waypoint(1.5, (0.18, sign * 0.28, TOP + 0.12), FINGER_CLOSED, "park", 0.0),
    ]
    t_clear = sum(w.duration for w in presser)
    i = next(i for i, w in enumerate(pusher) if w.label == f"push the spine to {RELEASE_DEG}") + 1
    t_at = sum(w.duration for w in pusher[:i])
    pusher.insert(i, Waypoint(max(0.3, t_clear - t_at), pusher[i - 1].pos, FINGER_CLOSED,
                              "wait while the other arm clears", pusher[i - 1].yaw))
    pe, _ = _pusher(lot, 170.0, push=-0.015)
    pusher += [
        Waypoint(0.8, pe, FINGER_CLOSED, "back off the spine", yaw),
        Waypoint(1.0, (pe[0], pe[1], pe[2] + 0.03), FINGER_CLOSED, "lift clear", yaw),
        Waypoint(1.0, (0.25, -sign * 0.12, TOP + 0.10), FINGER_CLOSED, "back toward its own side", 0.0),
        Waypoint(1.5, (0.15, -sign * 0.25, TOP + 0.15), FINGER_CLOSED, "park", 0.0),
        Waypoint(3.0, (0.15, -sign * 0.25, TOP + 0.15), FINGER_CLOSED, "wait", 0.0),
    ]
    # While the pressing arm is down at the knife, the pushing arm leans its finger tips 30 degrees away from its own
    # base, which keeps its forearm clear of the other arm's (the tips stay where a straight-down grip puts them).
    # Once the button is let go it backs off the spine, stands straight and comes back behind the blade.
    i = next(i for i, w in enumerate(pusher) if w.label == "wait while the other arm clears")
    back = RELEASE_DEG - 12.0  # open-loop pushes fall short: come back in behind where the blade may be
    off, _ = _pusher(lot, float(RELEASE_DEG), push=-0.020)
    b, _ = _pusher(lot, back, push=-0.015)
    on, _ = _pusher(lot, back)
    y0 = pusher[i].yaw
    pusher = [_lean(w, push_arm) for w in pusher[:i + 1]] + [
        _lean(Waypoint(0.6, off, FINGER_CLOSED, "off the spine to stand up", y0), push_arm),
        _lean(Waypoint(0.6, (off[0], off[1], off[2] + 0.02), FINGER_CLOSED, "up a little", y0), push_arm),
        Waypoint(1.0, (b[0], b[1], b[2] + 0.02), FINGER_CLOSED, "straight down again, behind the spine", y0),
        Waypoint(0.6, b, FINGER_CLOSED, "down beside the spine", y0),
        Waypoint(0.8, on, FINGER_CLOSED, "back on the spine", y0),
    ] + pusher[i + 1:]
    return {press_arm: presser, push_arm: pusher}


LEAN = 30.0


def _lean(w, arm):
    """The arm's waypoint with its finger tips leaned LEAN degrees away from its base, the TCP moved so the tips'
    ends stay where the straight-down waypoint puts them."""
    x, y, z = w.pos
    head = tilt.away(arm, (x, y))
    t = math.radians(LEAN)
    d = TIP * math.sin(t)
    pos = (x - d * math.cos(head), y - d * math.sin(head), z - TIP * (1 - math.cos(t)))
    return tilt.Waypoint(w.duration, pos, w.finger, w.label, yaw=float(w.yaw), tilt=LEAN, heading=head)
