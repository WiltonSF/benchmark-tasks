"""A section of plywood floor deck on battens stands on the table in front of the arms. A 200 mm square opening
through it has a plywood hole cover over it: a yellow plate with a wooden handle on top and two cleats underneath
that hang down inside the opening. A combination wrench lies on the subfloor at the bottom of the opening, under the
cover. A dark grey tool tote stands on the table beside the deck. Which side of the deck the tote is on, and which
way the cover's handle runs, change between trials.
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
GAP = 0.0005  # loose parts start this far above what they rest on

PLY, BATTEN, YELLOW, WOOD, STEEL, TOTE_GREY = ((0.74, 0.58, 0.38), (0.48, 0.34, 0.20), (0.96, 0.78, 0.08),
                                               (0.80, 0.62, 0.40), (0.74, 0.75, 0.77), (0.17, 0.18, 0.20))


def _box(lo, hi, color, friction=None):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(float(v) for v in hi - lo), at=tuple(float(v) for v in (lo + hi) / 2), color=color,
                     friction=friction)


# -- the deck and its opening (fixed) -------------------------------------------------------------------------------
# 18 mm plywood on 62 mm battens: 80 mm from the subfloor (the table) to the deck's top. The battens run round the
# opening, so the opening is an 80 mm deep square pocket down to the subfloor.
PLY_T, BATTEN_H = 0.018, 0.062
DECK_H = PLY_T + BATTEN_H
DECK_TOP = TOP + DECK_H
OPEN_XY = (0.260, 0.000)
OPEN_HALF = 0.100  # 200 x 200 mm
DECK_LO, DECK_HI = (-0.170, -0.220), (0.170, 0.220)  # the deck's outline about the opening's centre: 340 x 440 mm


def _deck_shapes():
    h = OPEN_HALF
    (x0, y0), (x1, y1) = DECK_LO, DECK_HI
    rects = [((x0, y0), (-h, y1)), ((h, y0), (x1, y1)), ((-h, y0), (h, -h)), ((-h, h), (h, y1))]
    out = []
    for (a0, b0), (a1, b1) in rects:
        out.append(_box((a0, b0, 0.0), (a1, b1, BATTEN_H), BATTEN))
        out.append(_box((a0, b0, BATTEN_H), (a1, b1, DECK_H), PLY))
    return out


DECK = parts.Part("deck", pos=(*OPEN_XY, TOP), mass=None, shapes=_deck_shapes(), friction=(0.5, 0.45))

# -- the hole cover (loose) -------------------------------------------------------------------------------------------
# Frame: origin at the middle of the plate's underside, x along the handle, z up. 18 mm plywood plate 270 mm square;
# a 150 x 40 x 40 mm softwood handle on top along x; two plywood cleats 180 x 20 x 18 mm under it along x, outer
# faces 180 mm apart, so 10 mm of play each side and each end in the 200 mm opening, their bottom edges bevelled
# 6 mm at 45 degrees as a lead-in.
PLATE = 0.270
HANDLE = (0.150, 0.040, 0.040)
CLEAT_L, CLEAT_W, CLEAT_H, BEVEL = 0.180, 0.020, 0.018, 0.006
CLEAT_Y = 0.080  # cleat centre lines (outer faces at +-90 mm)
COVER_MASS = 0.94  # kg: plate 0.73 (pine plywood, about 560 kg/m3), handle 0.11, cleats 0.08, screws 0.02
OVERLAP_MIN = 0.015  # the plate must lap every edge of the opening by at least this much


def _cleat(cy):
    x, y, h, c = CLEAT_L / 2, CLEAT_W / 2, CLEAT_H, BEVEL
    ring = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    top = [(sx * x, cy + sy * y, 0.0) for sx, sy in ring]
    mid = [(sx * x, cy + sy * y, -h + c) for sx, sy in ring]
    foot = [(sx * (x - c), cy + sy * (y - c), -h) for sx, sy in ring]
    pts = top + mid + foot
    faces = [(0, 3, 2, 1), (8, 9, 10, 11)]
    for i in range(4):
        j = (i + 1) % 4
        faces.append((i, j, 4 + j, 4 + i))
        faces.append((4 + i, 4 + j, 8 + j, 8 + i))
    return parts.hull(pts, faces, color=WOOD)


COVER_SHAPES = [
    _box((-PLATE / 2, -PLATE / 2, 0.0), (PLATE / 2, PLATE / 2, PLY_T), YELLOW),
    _box((-HANDLE[0] / 2, -HANDLE[1] / 2, PLY_T), (HANDLE[0] / 2, HANDLE[1] / 2, PLY_T + HANDLE[2]), WOOD),
    _cleat(+CLEAT_Y),
    _cleat(-CLEAT_Y),
]
COVER_SEAT = (*OPEN_XY, DECK_TOP + GAP)
COVER = parts.Part("cover", pos=COVER_SEAT, mass=COVER_MASS, shapes=COVER_SHAPES, friction=(0.5, 0.45),
                   contact_offset=0.002)

# -- the wrench (loose) -----------------------------------------------------------------------------------------------
# A 14 mm flat-pattern combination wrench, 180 mm long, 0.13 kg. Frame: origin at the middle of the shank, x toward
# the open end, z up from its flat. Ring end: a 26 mm square-edged ring round a 14 mm bore; open end: two jaws 14 mm
# apart; heads 9 mm thick, the shank 16 x 7 mm.
W_LEN = 0.180
HEAD_T, SHANK_T, SHANK_W = 0.009, 0.007, 0.016
RING_C, RING_O, BORE = -0.077, 0.026, 0.014
OPEN_C0, OPEN_C1, JAW_IN, JAW_OUT = 0.055, 0.090, 0.007, 0.016
WRENCH_MASS = 0.13


def _wrench_shapes():
    z0, z1 = -HEAD_T / 2, HEAD_T / 2
    r, b = RING_O / 2, BORE / 2
    rc = RING_C
    return [
        _box((rc + r, -SHANK_W / 2, -SHANK_T / 2), (OPEN_C0, SHANK_W / 2, SHANK_T / 2), STEEL),  # shank
        _box((rc - r, -r, z0), (rc + r, -b, z1), STEEL),  # ring, four sides
        _box((rc - r, b, z0), (rc + r, r, z1), STEEL),
        _box((rc - r, -b, z0), (rc - b, b, z1), STEEL),
        _box((rc + b, -b, z0), (rc + r, b, z1), STEEL),
        _box((OPEN_C0, -JAW_OUT, z0), (OPEN_C0 + 0.012, JAW_OUT, z1), STEEL),  # open end: the throat
        _box((OPEN_C0 + 0.012, JAW_IN, z0), (OPEN_C1, JAW_OUT, z1), STEEL),  # and its two jaws
        _box((OPEN_C0 + 0.012, -JAW_OUT, z0), (OPEN_C1, -JAW_IN, z1), STEEL),
    ]


WRENCH_START = (*OPEN_XY, TOP + HEAD_T / 2 + GAP)  # flat on the subfloor under the opening's middle, along x
WRENCH = parts.Part("wrench", pos=WRENCH_START, mass=WRENCH_MASS, shapes=_wrench_shapes(), friction=(0.45, 0.4),
                    contact_offset=0.002)
RING_END = (1, 2, 3, 4)  # shape indices of each head, for the "lying on the floor" check
OPEN_END = (5, 6, 7)

# -- the tool tote (loose, heavy enough to stay put) -------------------------------------------------------------------
TOTE_IN = (0.280, 0.180)  # inside, x by y
TOTE_WALL, TOTE_FLOOR, TOTE_H = 0.006, 0.006, 0.100
TOTE_MASS = 0.70


def _tote_shapes():
    ix, iy, w = TOTE_IN[0] / 2, TOTE_IN[1] / 2, TOTE_WALL
    return [
        _box((-ix - w, -iy - w, 0.0), (ix + w, iy + w, TOTE_FLOOR), TOTE_GREY),
        _box((-ix - w, -iy - w, TOTE_FLOOR), (-ix, iy + w, TOTE_H), TOTE_GREY),
        _box((ix, -iy - w, TOTE_FLOOR), (ix + w, iy + w, TOTE_H), TOTE_GREY),
        _box((-ix, -iy - w, TOTE_FLOOR), (ix, -iy, TOTE_H), TOTE_GREY),
        _box((-ix, iy, TOTE_FLOOR), (ix, iy + w, TOTE_H), TOTE_GREY),
    ]


TOTE_A = (0.270, -0.360, TOP + GAP)  # lot A: to the robot's right of the deck
TOTE = parts.Part("tote", pos=TOTE_A, mass=TOTE_MASS, shapes=_tote_shapes(), friction=(0.8, 0.7),
                  contact_offset=0.002)

OBJECTS = [DECK, COVER, WRENCH, TOTE]

# Lot B: the tote on the robot's left of the deck, and the cover turned a quarter turn (handle along y).
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"cover": COVER_SEAT, "wrench": WRENCH_START, "tote": TOTE_A},
             yaw={"cover": 0.0, "wrench": 0.0, "tote": 0.0}),
    lots.Lot("B", place={"cover": COVER_SEAT, "wrench": WRENCH_START, "tote": (TOTE_A[0], -TOTE_A[1], TOTE_A[2])},
             yaw={"cover": math.pi / 2, "wrench": 0.0, "tote": 0.0}),
)


# -- geometry ----------------------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def _tilt_deg(st) -> float:
    return math.degrees(math.acos(float(np.clip(_rot(st.quat_xyzw)[2, 2], -1.0, 1.0))))


def _corners(shape) -> np.ndarray:
    """A shape's corner points in its part's frame (a hull's own vertices)."""
    if shape.kind == "hull":
        return np.asarray(shape.points, float)
    h = shape.half
    return np.asarray(shape.at) + np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h


def _world(st, local) -> np.ndarray:
    return np.asarray(local, float) @ _rot(st.quat_xyzw).T + np.asarray(st.pos, float)


def _local(st, world) -> np.ndarray:
    return (np.asarray(world, float) - np.asarray(st.pos, float)) @ _rot(st.quat_xyzw)


_PLATE_UNDER = np.array([[sx * PLATE / 2, sy * PLATE / 2, 0.0] for sx in (-1, 1) for sy in (-1, 1)])
_FEET = np.concatenate([np.asarray(s.points, float)[8:] for s in COVER_SHAPES[2:]])  # the cleats' 8 foot corners
_OPEN_CORNERS = np.array([[OPEN_XY[0] + sx * OPEN_HALF, OPEN_XY[1] + sy * OPEN_HALF, DECK_TOP]
                          for sx in (-1, 1) for sy in (-1, 1)])


def cover_report(objects) -> dict:
    """Is the hole covered: the plate flat on the deck (all four corners of its underside within 3 mm above the deck's
    top), both cleats down in the opening (every foot corner inside the opening's outline and at least 10 mm below the
    deck's top) and the plate lapping all four edges of the opening by 15 mm or more?"""
    c = objects["cover"]
    under = _world(c, _PLATE_UNDER)[:, 2] - DECK_TOP
    flat = bool(np.all(under <= 0.003 + EPS) and np.all(under >= -0.002 - EPS))
    feet = _world(c, _FEET)
    inside = np.abs(feet[:, :2] - np.asarray(OPEN_XY)) <= OPEN_HALF + EPS
    cleats_in = bool(np.all(inside) and np.all(feet[:, 2] <= DECK_TOP - 0.010 + EPS))
    oc = _local(c, _OPEN_CORNERS)
    lap = float(PLATE / 2 - np.abs(oc[:, :2]).max())
    return dict(flat=flat, cleats_in=cleats_in, lap=lap, lift=float(under.max()),
                seated=bool(flat and cleats_in and lap >= OVERLAP_MIN - EPS))


def _wrench_points(st, idx=None) -> np.ndarray:
    shapes = WRENCH.shapes if idx is None else [WRENCH.shapes[i] for i in idx]
    return _world(st, np.concatenate([_corners(s) for s in shapes]))


def tote_ok(objects) -> bool:
    """The tote standing on the table: within 5 degrees of upright, its underside within 3 mm of the table."""
    t = objects["tote"]
    return bool(_tilt_deg(t) <= 5.0 + EPS and float(t.low[2]) <= TOP + 0.003 + EPS)


def wrench_report(objects) -> dict:
    """Where the wrench is. ``in_tote``: every corner inside the tote's inner walls, and both heads (ring end and open
    end) down on the tote's floor (each head's lowest corner within 2 mm of the floor's top), in the tote's frame.
    ``out``: no corner of the wrench is inside the opening's outline below the deck's top (less 2 mm), and none is more
    than 5 mm below the table's top (it has not been dropped off the table)."""
    w, t = objects["wrench"], objects["tote"]
    pts = _local(t, _wrench_points(w))
    inside = bool(np.all(np.abs(pts[:, 0]) <= TOTE_IN[0] / 2 + EPS) and np.all(np.abs(pts[:, 1]) <= TOTE_IN[1] / 2 + EPS))
    heads = [float(_local(t, _wrench_points(w, idx))[:, 2].min()) - TOTE_FLOOR for idx in (RING_END, OPEN_END)]
    on_floor = all(-0.002 - EPS <= h <= 0.002 + EPS for h in heads)
    wp = _wrench_points(w)
    in_hole = (np.abs(wp[:, 0] - OPEN_XY[0]) < OPEN_HALF) & (np.abs(wp[:, 1] - OPEN_XY[1]) < OPEN_HALF) & (
        wp[:, 2] < DECK_TOP - 0.002)
    on_bench = bool(wp[:, 2].min() >= TOP - 0.005 - EPS)  # not dropped off the table
    return dict(inside=inside, heads=heads, on_floor=on_floor, in_tote=bool(inside and on_floor and tote_ok(objects)),
                out=bool(on_bench and not in_hole.any()))


def tier(objects) -> float:
    """The end state's credit before the let-go and stillness rules: 1.0 hole covered and wrench in the tote; 0.5 hole
    covered and the wrench out of the opening but not in the tote; 0 otherwise."""
    if not cover_report(objects)["seated"]:
        return 0.0
    w = wrench_report(objects)
    if w["in_tote"]:
        return 1.0
    return 0.5 if w["out"] else 0.0


# -- the trial record -----------------------------------------------------------------------------------------------
# Kept on the shared parts module so every copy of this file in one process sees one record. Cleared only by the
# simulator's scene reset at the start of a trial.
CLEAR = 0.005  # let go: every sampled point of both grippers more than 5 mm from the cover and from the wrench
STILL_M, HOLD_S = 0.002, 1.0
_rec = parts.__dict__.setdefault("hole_cover_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", gap_cover=None, gap_wrench=None, tier=0.0, since=None, snap=None,
                held=False, t=0.0)


if not _rec:
    reset_record()


_TOTE_CORNERS = _corners(TOTE.shapes[0])


def _snap(objects) -> np.ndarray:
    """Points tracked for stillness: the plate's four under-corners, every wrench corner, the tote floor's corners."""
    return np.concatenate([_world(objects["cover"], _PLATE_UNDER), _wrench_points(objects["wrench"]),
                           _world(objects["tote"], _TOTE_CORNERS)])


def let_go() -> bool:
    gc, gw = _rec["gap_cover"], _rec["gap_wrench"]
    return gc is not None and gw is not None and gc > CLEAR + EPS and gw > CLEAR + EPS


def observe(objects, gaps, t) -> None:
    """One control step. ``gaps``: (closest gripper point to the cover, to the wrench) in m, or None when the grippers
    could not be read, which fails the trial."""
    _rec["steps"] += 1
    _rec["t"] = float(t)
    try:
        for name in ("cover", "wrench", "tote"):
            st = objects[name]
            if not np.all(np.isfinite(st.pos)) or not np.all(np.isfinite(st.quat_xyzw)):
                raise ValueError(f"{name} state unknown")
        if gaps is None or not all(np.isfinite(g) for g in gaps):
            raise ValueError("gripper state unknown")
        now = tier(objects)
        snap = _snap(objects)
    except (KeyError, ValueError, TypeError, AttributeError) as e:
        _rec.update(known=False, reason=str(e), since=None, held=False)
        return
    _rec["gap_cover"], _rec["gap_wrench"] = float(gaps[0]), float(gaps[1])
    moved = math.inf if _rec["snap"] is None else float(np.linalg.norm(snap - _rec["snap"], axis=1).max())
    if now <= 0.0 or not let_go():
        _rec.update(tier=now, since=None, held=False, snap=None)
    elif _rec["since"] is None or now != _rec["tier"] or moved > STILL_M + EPS:  # (re)start the second here
        _rec.update(tier=now, since=float(t), held=False, snap=snap)
    else:
        _rec["held"] = float(t) - _rec["since"] >= HOLD_S - EPS


# -- scoring ---------------------------------------------------------------------------------------------------------
def score(objects):
    """1.0: the cover back over the opening (plate flat on the deck, both cleats down in the opening, the plate lapping
    every edge of the opening by 15 mm or more) and the wrench lying on the tote's floor inside its walls, the tote
    standing on the table. 0.5: the cover back over the opening as above and the wrench out of the opening but not
    in the tote. Either needs both grippers more than 5 mm from the cover and the wrench, with the cover, the wrench
    and the tote still, for 1 s. 0 otherwise, and 0 for the whole trial if the grippers or parts could not be read."""
    if not _rec["known"] or _rec["steps"] == 0 or not _rec["held"] or not let_go():
        return 0.0
    return tier(objects)  # the step callback has just judged this same state (its hold restarts if the tier changes)


def success(objects):
    return bool(score(objects) >= 1.0)


def distance(objects):
    c = np.asarray(objects["cover"].pos, float) - np.asarray(COVER_SEAT, float)
    w = np.asarray(objects["wrench"].pos, float) - np.asarray(objects["tote"].pos, float)
    return float(np.linalg.norm(c) + max(0.0, np.linalg.norm(w[:2]) - 0.05))


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


def gap_to(part, st, pts) -> float:
    """Closest distance (m, negative inside) from any point to any shape of the part (a cleat as its bounding box)."""
    from yam_arena import analysis

    near = np.linalg.norm(pts - np.asarray(st.pos, float), axis=1) < 0.30
    if not near.any():
        return 0.10
    pts = pts[near]
    boxes = part.world_boxes({"base": (np.asarray(st.pos, float), _rot(st.quat_xyzw))})
    return float(min(analysis.box_signed_distance(pts, c, r, s).min() for _, c, r, s in boxes))


def gaps(objects, pts):
    return gap_to(COVER, objects["cover"], pts), gap_to(WRENCH, objects["wrench"], pts)


def _hand_points(sim) -> np.ndarray | None:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                out.append(link_points(n) @ _rot(poses[i, 3:]).T + (poses[i, :3] - origin))
                found += 1
    if found != 2 * len(_HAND_LINKS):
        return None
    pts = np.concatenate(out)
    return pts if np.isfinite(pts).all() else None


def _tcps(sim) -> dict:
    """Each gripper link's position and finger joints, for trace files only (never scored)."""
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = {}
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        q = parts._np(asset.data.joint_pos)[0]
        names = list(asset.joint_names)
        poses = parts._np(asset.data.body_link_pose_w)[0]
        g = poses[list(asset.body_names).index("gripper"), :3] - origin
        out[arm[0]] = dict(f=[round(float(q[names.index(n)]), 4) for n in ("joint7", "joint8")],
                           g=[round(float(v), 4) for v in g])
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


def _apply_exact(emb) -> bool:
    """A new trial's layout: every loose part written exactly at its lot's pose, settled, written again and rendered,
    so the trial's first frame shows the true start."""
    base = emb._base
    lot = lots.current()
    for _ in range(2):
        for name in ("cover", "wrench", "tote"):
            _write_pose(base, name, *lot.pos(name), lot.yaw_of(name))
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
    try:
        pts = _hand_points(sim)
        g = gaps(objects, pts) if pts is not None else None
    except Exception:  # noqa: BLE001
        g = None
    observe(objects, g, float(sim.t))
    log = os.environ.get("COVER_LOG") or (str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("cover.jsonl"))
                                          if os.environ.get("TRACE_OUT") else None)
    if log and _rec["steps"] % 5 == 0 and objects:
        try:
            cr, wr = cover_report(objects), wrench_report(objects)
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, score=score(objects), tier=tier(objects),
                       held=_rec["held"], known=_rec["known"], reason=_rec["reason"],
                       gap_cover=_rec["gap_cover"], gap_wrench=_rec["gap_wrench"],
                       cover=[round(float(v), 4) for v in objects["cover"].pos], cover_tilt=round(_tilt_deg(
                           objects["cover"]), 2), lift=round(cr["lift"], 4), lap=round(cr["lap"], 4),
                       cleats_in=cr["cleats_in"], seated=cr["seated"],
                       wrench=[round(float(v), 4) for v in objects["wrench"].pos],
                       wrench_tilt=round(_tilt_deg(objects["wrench"]), 2), heads=[round(h, 4) for h in wr["heads"]],
                       in_tote=wr["in_tote"], out=wr["out"], tote=[round(float(v), 4) for v in objects["tote"].pos],
                       arms=_tcps(sim))
            with open(log, "a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with open(log, "a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- demo: open-loop from the start layout the cameras show -------------------------------------------------------------
# The arm on the side away from the tote lifts the cover by its handle and sets it down on the table beside the deck;
# the arm on the tote's side reaches down into the opening, takes the wrench and lays it in the tote; then the first
# arm carries the cover back and lowers it until its cleats drop into the opening.
OPEN = FINGER_OPEN
HANDLE_TOP = PLY_T + HANDLE[2]  # above the cover's origin (the plate's underside)
GRIP_BELOW_TOP = 0.012  # TCP 12 mm under the handle's top (pad tips 25 mm under it, 15 mm over the plate)
HANDLE_GRIP = -(HANDLE[1] / 2 - 0.005)  # 5 mm past the handle's faces
WRENCH_OPEN, WRENCH_GRIP = -0.018, -(SHANK_W / 2 - 0.004)
FEET_BELOW_TCP = HANDLE_TOP - GRIP_BELOW_TOP + CLEAT_H  # the cleats' feet hang this far below the TCP
CARRY_Z = TOP + 0.178  # cleat feet 34 mm over the deck while carrying (about as high as the arms reach over the opening)
SEAT_TCP_Z = DECK_TOP + HANDLE_TOP - GRIP_BELOW_TOP
PARK_TCP_Z = TOP + CLEAT_H + HANDLE_TOP - GRIP_BELOW_TOP  # the cover standing on its cleats on the table
PRESS = 0.004  # commanded this far below the resting grip height, so it bottoms out before the fingers open
COVER_ASIDE_Y = 0.400  # where the cover is set down on the table beside the deck (|y|, on the side away from the tote)
WRENCH_TCP_Z = TOP + 0.013 + 0.0015  # pad tips 1.5 mm over the subfloor
WRENCH_TRAVEL_Z = TOP + 0.160
WRENCH_SET_Z = TOP + TOTE_FLOOR + 0.020 + 0.0045 + 0.010  # the wrench's underside 2 cm over the tote's floor
PARK = {"left": (0.150, 0.300, TOP + 0.200), "right": (0.150, -0.300, TOP + 0.200)}


def _dur(plan) -> float:
    return sum(w.duration for w in plan)


def _wait(plan, t, label):
    last = plan[-1]
    return [Waypoint(t - _dur(plan), last.pos, last.finger, label, last.yaw)] if t > _dur(plan) + 1e-9 else []


def demo():
    lot = lots.current()
    tote = lot.pos("tote")
    wrench_arm = "right" if tote[1] < 0 else "left"
    cover_arm = "left" if wrench_arm == "right" else "right"
    side = 1.0 if cover_arm == "left" else -1.0
    cyaw = lot.yaw_of("cover")
    # Fingers close across the handle: yaw 0 closes them along y (a handle along x); a handle along y takes a quarter
    # turn of the wrist (+90 degrees, the reachable way for either arm near the middle).
    hyaw = 0.0 if abs(math.cos(cyaw)) > 0.7 else math.pi / 2
    ox, oy, _ = lot.pos("cover")
    ax, ay = ox, side * COVER_ASIDE_Y
    c = cover_arm
    cover = [
        Waypoint(2.0, PARK[c], OPEN, "park", hyaw),
        Waypoint(2.5, (ox, oy, CARRY_Z), OPEN, "over the handle", hyaw),
        Waypoint(1.5, (ox, oy, SEAT_TCP_Z), OPEN, "fingers either side of the handle", hyaw),
        Waypoint(0.8, (ox, oy, SEAT_TCP_Z), HANDLE_GRIP, "grip the handle", hyaw),
        Waypoint(2.5, (ox, oy, CARRY_Z), HANDLE_GRIP, "lift the cover out", hyaw),
        Waypoint(2.5, (ax, ay, CARRY_Z), HANDLE_GRIP, "carry it aside", hyaw),
        Waypoint(2.0, (ax, ay, PARK_TCP_Z - PRESS), HANDLE_GRIP, "set it down on the table", hyaw),
        Waypoint(0.6, (ax, ay, PARK_TCP_Z - PRESS), OPEN, "let go", hyaw),
        Waypoint(1.0, (ax, ay, CARRY_Z), OPEN, "up", hyaw),
    ]
    w = wrench_arm
    wx, wy, _ = WRENCH_START
    tx, ty, _ = tote
    wrench = [Waypoint(2.0, PARK[w], WRENCH_OPEN, "park", 0.0)]
    wrench += _wait(wrench, _dur(cover), "wait for the cover to be off")
    wrench += [
        Waypoint(2.0, (wx, wy, WRENCH_TRAVEL_Z), WRENCH_OPEN, "over the opening", 0.0),
        Waypoint(2.0, (wx, wy, WRENCH_TCP_Z), WRENCH_OPEN, "down into the opening", 0.0),
        Waypoint(0.8, (wx, wy, WRENCH_TCP_Z), WRENCH_GRIP, "grip the wrench", 0.0),
        Waypoint(2.0, (wx, wy, WRENCH_TRAVEL_Z), WRENCH_GRIP, "lift it out", 0.0),
        Waypoint(2.0, (tx, ty, WRENCH_TRAVEL_Z), WRENCH_GRIP, "over the tote", 0.0),
        Waypoint(1.2, (tx, ty, WRENCH_SET_Z), WRENCH_GRIP, "lower it into the tote", 0.0),
        Waypoint(0.6, (tx, ty, WRENCH_SET_Z), WRENCH_OPEN, "let go", 0.0),
        Waypoint(1.0, (tx, ty, WRENCH_TRAVEL_Z), WRENCH_OPEN, "up", 0.0),
        Waypoint(1.5, PARK[w], WRENCH_OPEN, "park", 0.0),
    ]
    t_out = _dur(wrench[:-4])  # the wrench is over the tote, clear of the opening
    cover += _wait(cover, t_out, "wait for the wrench to be out")
    cover += [
        Waypoint(1.5, (ax, ay, PARK_TCP_Z), OPEN, "fingers either side of the handle", hyaw),
        Waypoint(0.8, (ax, ay, PARK_TCP_Z), HANDLE_GRIP, "grip the handle", hyaw),
        Waypoint(2.0, (ax, ay, CARRY_Z), HANDLE_GRIP, "lift the cover", hyaw),
        Waypoint(2.5, (ox, oy, CARRY_Z), HANDLE_GRIP, "over the opening", hyaw),
        Waypoint(3.0, (ox, oy, SEAT_TCP_Z - PRESS), HANDLE_GRIP, "lower it until the cleats drop in", hyaw),
        Waypoint(1.0, (ox, oy, SEAT_TCP_Z - PRESS), HANDLE_GRIP, "let it settle", hyaw),
        Waypoint(0.6, (ox, oy, SEAT_TCP_Z - PRESS), OPEN, "let go", hyaw),
        Waypoint(1.2, (ox, oy, CARRY_Z), OPEN, "up", hyaw),
        Waypoint(1.5, PARK[c], OPEN, "park", hyaw),
    ]
    wrench += _wait(wrench, _dur(cover), "wait")
    cover.append(Waypoint(3.0, PARK[c], OPEN, "wait", hyaw))
    wrench.append(Waypoint(3.0, PARK[w], WRENCH_OPEN, "wait", 0.0))
    return {cover_arm: cover, wrench_arm: wrench}
