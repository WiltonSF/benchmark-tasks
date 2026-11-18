"""On the station table: a laser printer's 250-sheet paper cassette (Tray 2), pulled out of the printer and lying on
the desk with its front panel and handle toward the robot's right. It holds 100 sheets of A4 copy paper, dropped in
loosely: fanned along their length, a little skewed, and short of the feed edge. The two width guides (linked, so
they move together) are still set for Letter, and the rear length guide for Legal. Each guide has a blue
squeeze-to-release latch.
"""

from __future__ import annotations

import math
import os
import json
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

TRAY_COL = (0.20, 0.21, 0.23)  # dark grey cassette body
GUIDE_COL = (0.62, 0.64, 0.66)  # light grey guides
LATCH_COL = (0.10, 0.42, 0.85)  # blue squeeze latches
PAPER = ((0.97, 0.97, 0.95), (0.90, 0.91, 0.89), (0.97, 0.97, 0.95), (0.90, 0.91, 0.89))

# -- the paper ---------------------------------------------------------------------------------------------------------
A4 = (0.210, 0.297)  # m: x (across the tray) by y (along it)
N_BUNDLES = 4
SHEETS = 25  # per bundle
T = 0.0025  # 25 sheets of 80 g/m2 copy paper, about 0.1 mm each
BUNDLE_MASS = 0.125  # 25 x 5.0 g (A4 at 80 g/m2)

# -- the cassette (fixed on the table) -------------------------------------------------------------------------------
XC = 0.27  # centre line of the paper path (x)
FLOOR_T = 0.003
FLOOR = TOP + FLOOR_T  # where the paper rests
YF = -0.160  # the feed edge: inside face of the front panel; the stack's leading edge must sit against it
WIDE = 0.218  # the width guides' widest setting (Letter/Legal 215.9 mm plus 1 mm a side)
LEGAL = 0.356  # the length guide starts set for Legal (355.6 mm)
GUIDE_H = 0.036  # guide faces stand 36 mm above the table (33 mm above the floor)
PAD_T, PLATE_T = 0.003, 0.004
PAD_TRAVEL = 0.005
PAD_K = 1500.0  # N/m: each guide face is a sprung pad
LEVER_UP = 0.17  # rad: a latch lever squeezed shut
UNLOCK = 0.10  # rad: the latch lets the guide slide once its lever is squeezed this far
LEVER_K = 0.4  # N m/rad: about 4 N at the fingertips to squeeze a latch
LOCK_K, LOCK_C = 5.0e4, 60.0  # a latched guide's hold on its rack (N/m, N s/m)
FREE_C = 30.0  # N s/m: a released guide's sliding drag
W_SLIDE = (-0.070, 0.030)  # y span of the width guides
WN0 = XC - WIDE / 2  # near width guide's pad face (x) at the start
WF0 = XC + WIDE / 2
LF0 = YF + LEGAL  # length guide's pad face (y) at the start
L_SPAN = (XC - 0.070, XC + 0.070)
TAB = 0.030  # latch width
TAB_YC = -0.020  # near width guide's latch centre (y)


def _b(lo, hi, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(hi - lo), tuple((lo + hi) / 2), color)


Z_PAD = (FLOOR + 0.0005, TOP + GUIDE_H)
Z_POST = (TOP + GUIDE_H, TOP + 0.062)
Z_FOOT = (TOP + 0.032, TOP + GUIDE_H)
Z_LEVER = (TOP + GUIDE_H + 0.0005, TOP + 0.062)

# near width guide (moves +x), its latch on the outside
_NEAR_PAD = _b((WN0 - PAD_T, W_SLIDE[0], Z_PAD[0]), (WN0, W_SLIDE[1], Z_PAD[1]), GUIDE_COL)
_NEAR_PLATE = _b((WN0 - PAD_T - PLATE_T, W_SLIDE[0], Z_PAD[0]), (WN0 - PAD_T, W_SLIDE[1], Z_PAD[1]), GUIDE_COL)
_NEAR_POST = _b((WN0 - 0.011, TAB_YC - TAB / 2, Z_POST[0]), (WN0 - 0.003, TAB_YC + TAB / 2, Z_POST[1]), LATCH_COL)
_NEAR_FOOT = _b((WN0 - 0.023, TAB_YC - TAB / 2, Z_FOOT[0]), (WN0 - PAD_T - PLATE_T, TAB_YC + TAB / 2, Z_FOOT[1]),
                GUIDE_COL)
_NEAR_LEVER = _b((WN0 - 0.021, TAB_YC - TAB / 2, Z_LEVER[0]), (WN0 - 0.017, TAB_YC + TAB / 2, Z_LEVER[1]), LATCH_COL)
# far width guide (moves -x), driven through the rack by the near one
_FAR_PAD = _b((WF0, W_SLIDE[0], Z_PAD[0]), (WF0 + PAD_T, W_SLIDE[1], Z_PAD[1]), GUIDE_COL)
_FAR_PLATE = _b((WF0 + PAD_T, W_SLIDE[0], Z_PAD[0]), (WF0 + PAD_T + PLATE_T, W_SLIDE[1], Z_PAD[1]), GUIDE_COL)
_FAR_TOP = _b((WF0 + PAD_T, W_SLIDE[0] + 0.03, Z_POST[0]), (WF0 + PAD_T + PLATE_T, W_SLIDE[1] - 0.03, TOP + 0.050),
              GUIDE_COL)
# length guide (moves -y), its latch on the back
_LEN_PAD = _b((L_SPAN[0], LF0, Z_PAD[0]), (L_SPAN[1], LF0 + PAD_T, Z_PAD[1]), GUIDE_COL)
_LEN_PLATE = _b((L_SPAN[0], LF0 + PAD_T, Z_PAD[0]), (L_SPAN[1], LF0 + PAD_T + PLATE_T, Z_PAD[1]), GUIDE_COL)
_LEN_POST = _b((XC - TAB / 2, LF0 + 0.003, Z_POST[0]), (XC + TAB / 2, LF0 + 0.011, Z_POST[1]), LATCH_COL)
_LEN_FOOT = _b((XC - TAB / 2, LF0 + PAD_T + PLATE_T, Z_FOOT[0]), (XC + TAB / 2, LF0 + 0.023, Z_FOOT[1]), GUIDE_COL)
_LEN_LEVER = _b((XC - TAB / 2, LF0 + 0.017, Z_LEVER[0]), (XC + TAB / 2, LF0 + 0.021, Z_LEVER[1]), LATCH_COL)

# cassette body: floor, front panel with handle, side walls, back wall (inside 304 x 405 mm; walls low at the sides)
X0, X1 = XC - 0.152, XC + 0.152
Y0, Y1 = YF - 0.012, LF0 + 0.049
BODY = (
    _b((X0, Y0, TOP), (X1, Y1, FLOOR), TRAY_COL),
    _b((X0, Y0, TOP), (X1, YF, TOP + 0.065), TRAY_COL),  # front panel, 65 mm tall
    _b((XC - 0.050, Y0 - 0.010, TOP + 0.040), (XC + 0.050, Y0, TOP + 0.050), (0.12, 0.12, 0.13)),  # handle lip
    _b((X0, YF, TOP), (X0 + 0.006, Y1, TOP + 0.030), TRAY_COL),
    _b((X1 - 0.006, YF, TOP), (X1, Y1, TOP + 0.030), TRAY_COL),
    _b((X0, Y1 - 0.009, TOP), (X1, Y1, TOP + 0.030), TRAY_COL),
    # the length guide's rack slot and the width guides' rack slot, drawn on the floor (no collider effect)
    _b((XC - 0.004, LF0 - 0.130, FLOOR), (XC + 0.004, LF0 + 0.020, FLOOR + 0.0002), (0.10, 0.10, 0.11)),
    _b((WN0 - 0.010, TAB_YC - 0.004, FLOOR), (WF0 + 0.010, TAB_YC + 0.004, FLOOR + 0.0002), (0.10, 0.10, 0.11)),
)


def _origin(*shapes):
    return np.mean([s.at for s in shapes], axis=0)


WN = parts.Slider("width_near", shapes=(_NEAR_PLATE, _NEAR_POST, _NEAR_FOOT), axis=(1.0, 0.0, 0.0), travel=0.070,
                  mass=0.04, spring=LOCK_K, damping=LOCK_C, max_effort=400.0)
WF = parts.Slider("width_far", shapes=(_FAR_PLATE, _FAR_TOP), axis=(-1.0, 0.0, 0.0), travel=0.070,
                  mass=0.04, spring=LOCK_K, damping=LOCK_C, max_effort=400.0)
LG = parts.Slider("length", shapes=(_LEN_PLATE, _LEN_POST, _LEN_FOOT), axis=(0.0, -1.0, 0.0), travel=0.150,
                  mass=0.05, spring=LOCK_K, damping=LOCK_C, max_effort=400.0)
WN_PAD = parts.Slider("pad_near", shapes=(_NEAR_PAD,), axis=(-1.0, 0.0, 0.0), travel=PAD_TRAVEL, mass=0.01,
                      spring=PAD_K, damping=1.0, max_effort=50.0, parent="width_near")
WF_PAD = parts.Slider("pad_far", shapes=(_FAR_PAD,), axis=(1.0, 0.0, 0.0), travel=PAD_TRAVEL, mass=0.01,
                      spring=PAD_K, damping=1.0, max_effort=50.0, parent="width_far")
LG_PAD = parts.Slider("pad_length", shapes=(_LEN_PAD,), axis=(0.0, 1.0, 0.0), travel=PAD_TRAVEL, mass=0.01,
                      spring=PAD_K, damping=1.0, max_effort=50.0, parent="length")
WN_LEVER = parts.Hinge("latch_width", shapes=(_NEAR_LEVER,), pivot=(WN0 - 0.019, TAB_YC, TOP + GUIDE_H),
                       axis=(0.0, 1.0, 0.0), lower=0.0, upper=LEVER_UP, mass=0.012, spring=LEVER_K, damping=0.003,
                       max_effort=5.0, parent="width_near")
LG_LEVER = parts.Hinge("latch_length", shapes=(_LEN_LEVER,), pivot=(XC, LF0 + 0.019, TOP + GUIDE_H),
                       axis=(1.0, 0.0, 0.0), lower=0.0, upper=LEVER_UP, mass=0.012, spring=LEVER_K, damping=0.003,
                       max_effort=5.0, parent="length")
CASSETTE = parts.Part("cassette", pos=(0.0, 0.0, 0.0), shapes=BODY,  # shapes given in world coordinates
                     
                      joints=(WN, WF, LG, WN_PAD, WF_PAD, LG_PAD, WN_LEVER, LG_LEVER), friction=(0.35, 0.30))


def _bundle(i):
    return parts.Part(f"paper_{i + 1}", pos=(XC, YF + A4[1] / 2, FLOOR + T / 2 + i * (T + 0.0002)), mass=BUNDLE_MASS,
                      shapes=(parts.box((A4[0], A4[1], T), (0.0, 0.0, 0.0), PAPER[i]),), friction=(0.45, 0.35))


BUNDLES = [_bundle(i) for i in range(N_BUNDLES)]
NAMES = [b.name for b in BUNDLES]
OBJECTS = [CASSETTE, *BUNDLES]


def _lot_place(d, dx):
    return {b.name: (XC + dx[i], YF + d[i] + A4[1] / 2, b.pos[2]) for i, b in enumerate(BUNDLES)}


# Lot A (the screenshot): bottom sheets nearest the feed edge, each bundle 17 mm further back toward the top (3 to
# 54 mm short of the edge). Lot B: staggered the other way (top sheets nearest, bottom 54 mm short), skewed the other way.
FAN_A, FAN_B = (0.003, 0.020, 0.037, 0.054), (0.054, 0.037, 0.020, 0.003)
DX_A = (0.0008, -0.0006, 0.0005, -0.0008)
YAW_A = tuple(math.radians(a) for a in (0.4, -0.5, 0.7, -0.3))
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place=_lot_place(FAN_A, DX_A), yaw={n: a for n, a in zip(NAMES, YAW_A)}),
    lots.Lot("B", place=_lot_place(FAN_B, tuple(-v for v in DX_A)), yaw={n: -a for n, a in zip(NAMES, YAW_A)}),
)
# the declared poses are lot A's, so check.py and the screenshot agree
for _b_ in BUNDLES:
    object.__setattr__(_b_, "pos", LOTS[0].place[_b_.name])
    object.__setattr__(_b_, "yaw", LOTS[0].yaw[_b_.name])

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
CLEAR = 0.005  # let go: every gripper face more than 5 mm from the paper, the guides and their latches
HOLD_S = 1.0
SNUG = 0.001  # a guide counts as against the stack within 1 mm
SQUEEZE = 0.0020  # a pad pressed in more than 2 mm (3 N) bows the stack: the stand-in rule
SQUARE_DEG = 1.0
FLAT_DEG = 2.0
FACE_STEP = 0.003
_HAND_LINKS = ("gripper", "tip_left", "tip_right")
LOCKABLE = (("width_near", "latch_width"), ("length", "latch_length"))


# -- geometry ----------------------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _link_poses(p) -> dict:
    out = {"base": (np.asarray(p.pos, float), _rot(p.quat_xyzw))}
    for name, (pos, qx) in getattr(p, "links", {}).items():
        out[name] = (np.asarray(pos, float), _rot(qx))
    return out


def _bundle_box(s):
    return np.asarray(s.pos, float), _rot(s.quat_xyzw), np.array((A4[0], A4[1], T))


def _corners(s):
    c, r, size = _bundle_box(s)
    k = np.array([[a, b, z] for a in (-0.5, 0.5) for b in (-0.5, 0.5) for z in (-0.5, 0.5)])
    return c + (k * size) @ r.T


def bundle_yaw(s) -> float:
    r = _rot(s.quat_xyzw)
    return math.atan2(r[1, 0], r[0, 0])


def bundle_tilt(s) -> float:
    r = _rot(s.quat_xyzw)
    return math.acos(max(-1.0, min(1.0, r[2, 2])))


def pad_faces(p) -> dict:
    """The stack-side face of each guide's pad, world: near x, far x, length y."""
    poses = _link_poses(p)
    out = {}
    for link, shape, axis, sign in (("pad_near", _NEAR_PAD, 0, +1), ("pad_far", _FAR_PAD, 0, -1),
                                    ("pad_length", _LEN_PAD, 1, -1)):
        pos, r = poses[link]
        j = next(j for j in CASSETTE.joints if j.name == link)
        centre = pos + r @ (np.asarray(shape.at) - j.origin)
        out[link] = float(centre[axis] + sign * shape.half[axis])
    return out


def stack_report(objects) -> dict:
    p = objects["cassette"]
    faces = pad_faces(p)
    corners = np.concatenate([_corners(objects[n]) for n in NAMES])
    report = dict(
        near_gap=float(corners[:, 0].min() - faces["pad_near"]),
        far_gap=float(faces["pad_far"] - corners[:, 0].max()),
        rear_gap=float(faces["pad_length"] - corners[:, 1].max()),
        front=[float(_corners(objects[n])[:, 1].min() - YF) for n in NAMES],
        yaw=[math.degrees(bundle_yaw(objects[n])) for n in NAMES],
        tilt=[math.degrees(bundle_tilt(objects[n])) for n in NAMES],
        top=float(corners[:, 2].max() - FLOOR),
        pressed={k: float(p.joints[k]) for k in ("pad_near", "pad_far", "pad_length")},
        latches={k: float(p.joints[k]) for k in ("latch_width", "latch_length")},
    )
    report["flat"] = bool(max(report["tilt"]) <= FLAT_DEG + EPS and report["top"] <= N_BUNDLES * T + 0.0015)
    report["squeezed"] = bool(max(report["pressed"].values()) > SQUEEZE + EPS)
    report["square"] = bool(max(abs(a) for a in report["yaw"]) <= SQUARE_DEG + EPS
                            and max(report["front"]) <= SNUG + EPS)
    report["length_snug"] = bool(report["rear_gap"] <= SNUG + EPS and report["square"])
    report["width_snug"] = bool(report["near_gap"] <= SNUG + EPS and report["far_gap"] <= SNUG + EPS)
    report["latched"] = bool(max(report["latches"].values()) < UNLOCK - EPS)
    return report


# -- the per-trial record -------------------------------------------------------------------------------------------
_rec = parts.__dict__.setdefault("paper_cassette_record", {})


def fresh() -> None:
    _rec.update(blind=False, gap=None, since=None, held=False, prev=None, t=None, free={g: False for g, _ in LOCKABLE},
                n=0)


if not _rec:
    fresh()


# -- hands and the release check (gripper hull faces sampled every 3 mm, corners included) ----------------------------
def _face_points(hull, step=FACE_STEP) -> np.ndarray:
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


def _link_faces(name):
    if name not in _FACES:
        from yam_arena import analysis

        _FACES[name] = [_face_points(h) for h in analysis.link_hulls().get(name, [])]
    return _FACES[name]


def scored_boxes(objects):
    """(centre, rotation, size) of every scored thing: the paper bundles and every moving part of the guides."""
    p = objects["cassette"]
    poses = {k: v for k, v in _link_poses(p).items() if k != "base"}
    out = [(c, r, s) for _, c, r, s in CASSETTE.world_boxes(poses)]
    out += [_bundle_box(objects[n]) for n in NAMES]
    return out


def hand_gap(points, objects) -> float:
    """Closest any sampled gripper face point comes to a scored box (m; negative inside)."""
    from yam_arena import analysis

    return float(min(analysis.box_signed_distance(points, c, r, s).min() for c, r, s in scored_boxes(objects)))


def _hand_points(sim):
    """World points on both arms' gripper and finger-tip hull faces, or None unless all six links were read."""
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    pts, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                faces = _link_faces(n)
                if faces:
                    found += 1
                rot, pos = _rot(poses[i, 3:]), poses[i, :3] - origin
                pts += [f @ rot.T + pos for f in faces]
    if found != 2 * len(_HAND_LINKS) or not pts:
        return None
    pts = np.concatenate(pts)
    return pts if np.isfinite(pts).all() else None


# -- the latches: a guide slides only while its latch lever is squeezed ----------------------------------------------
def _write_lock(sim, guide: str, locked: bool, at: float | None = None) -> None:
    import torch

    base = sim.env.unwrapped
    asset = base.scene["cassette"]
    ids = asset.find_joints([guide], preserve_order=True)[0]
    k = LOCK_K if locked else 0.0
    c = LOCK_C if locked else FREE_C
    if locked and at is not None:
        sim.set_target("cassette", guide, at)
    asset.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[k]], dtype=torch.float32, device=base.device),
                                             joint_ids=ids)
    asset.write_joint_damping_to_sim_index(damping=torch.tensor([[c]], dtype=torch.float32, device=base.device),
                                           joint_ids=ids)


@parts.on_reset
def _new_trial(sim) -> None:
    fresh()
    for g, _ in LOCKABLE:
        _write_lock(sim, g, True, 0.0)
    for j in CASSETTE.joints:
        sim.set_target("cassette", j.name, 0.0)


def _still_vector(objects):
    p = objects["cassette"]
    v = [objects[n].pos for n in NAMES] + [np.array([p.joints[k] for k in ("width_near", "width_far", "length",
                                                                           "pad_near", "pad_far", "pad_length")])]
    return np.concatenate(v)


@parts.on_step
def _step(sim) -> None:
    objects = sim.states()
    p = objects["cassette"]
    q = p.joints
    for g, lever in LOCKABLE:
        free = q[lever] >= UNLOCK - EPS
        if free != _rec["free"][g]:
            _write_lock(sim, g, not free, q[g])
            _rec["free"][g] = free
    sim.set_target("cassette", "width_far", q["width_near"])  # the rack: the far guide mirrors the near one
    t = float(sim.t)
    vec = _still_vector(objects)
    speed = None
    if _rec["prev"] is not None and t > _rec["prev"][0] + EPS:
        d = np.abs(vec - _rec["prev"][1])
        speed = float(d.max() / (t - _rec["prev"][0]))
    _rec["prev"] = (t, vec)
    _rec["t"] = t
    try:
        pts = _hand_points(sim)
    except Exception as e:  # noqa: BLE001  the arms could not be read: no release can be confirmed
        print(f"[hands] unreadable: {e!r}", file=sys.stderr, flush=True)
        pts = None
    if pts is None:
        _rec.update(blind=True, gap=None, since=None, held=False)
    else:
        _rec["gap"] = hand_gap(pts, objects)
        still = speed is not None and speed < 0.005 and all(objects[n].resting(0.005, 0.1) for n in NAMES)
        if _rec["gap"] > CLEAR + EPS and still and not any(_rec["free"].values()):
            if _rec["since"] is None:
                _rec["since"] = t
            _rec["held"] = t - _rec["since"] >= HOLD_S - EPS
        else:
            _rec.update(since=None, held=False)
    _rec["n"] += 1
    if os.environ.get("TRACE_OUT") and _rec["n"] % 5 == 0:
        row = dict(t=t, gap=_rec["gap"], held=_rec["held"], blind=_rec["blind"], free=dict(_rec["free"]),
                   joints={k: float(v) for k, v in q.items()}, score=score(objects))
        row.update({k: v for k, v in stack_report(objects).items()})
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("cassette.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


# -- scoring -----------------------------------------------------------------------------------------------------------
def score(objects):
    """0 unless both hands are clear (every gripper face more than 5 mm from the paper and the guides), both latches
    are let go and everything has been still for 1 s; 0 if any guide pad is pressed in more than 2 mm (3 N) (the stack is
    squeezed and bows), or any bundle is tilted more than 2 degrees or riding up. Otherwise 0.5 when the length guide
    is within 1 mm of the stack and every bundle is square (yaw within 1 degree) and against the feed edge (within
    1 mm), plus 0.5 when each width guide is within 1 mm of the stack's side."""
    try:
        if _rec["blind"] or not _rec["held"] or _rec["gap"] is None or _rec["gap"] <= CLEAR + EPS:
            return 0.0
        r = stack_report(objects)
    except (KeyError, ValueError, TypeError):
        return 0.0
    if r["squeezed"] or not r["flat"] or not r["latched"]:
        return 0.0
    return 0.5 * r["length_snug"] + 0.5 * r["width_snug"]


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    try:
        r = stack_report(objects)
    except (KeyError, ValueError, TypeError):
        return 1.0
    return float(max(r["rear_gap"], 0.0) + max(r["near_gap"], 0.0) + max(r["far_gap"], 0.0))


# -- scripted demo -----------------------------------------------------------------------------------------------------
# Open loop from the published layout: the guides' visible start settings and the paper's printed size (A4). The
# length guide goes first and pushes the fanned bundles up against the feed edge; then the width guides are closed.
# Each latch is pinched shut (fingers either side of the blue latch), the guide slid with it, and the latch let go.
# The arm is soft (about 150-190 N/m), so a guide is pressed only as hard as the hand is commanded past the edge.
# Each guide is first pushed well past the stack's edge (squaring the stack, which takes a few newtons to slide the
# lowest sheets), then eased back to just past the edge, and let go there.
PUSH_L, EASE_L = 0.045, 0.003
PUSH_W, EASE_W = 0.008, 0.002


def _grip(gap_m: float) -> float:
    return FINGER_OPEN * gap_m / 0.094


PINCH_Z = TOP + 0.060  # tips 13 mm lower, across the top of the latch


def demo():
    open_g, shut_g = _grip(0.032), _grip(0.006)
    # length guide: latch spans y LF0+0.003 .. LF0+0.021 at the start; it must travel LEGAL - A4 length
    ly0 = LF0 + 0.012
    ly1 = ly0 - (LEGAL - A4[1]) - PUSH_L
    ly2 = ly0 - (LEGAL - A4[1]) - EASE_L
    left = [
        Waypoint(1.5, (XC, ly0 + 0.01, TOP + 0.12), FINGER_OPEN, "over the length latch", 0.0),
        Waypoint(1.2, (XC, ly0, PINCH_Z + 0.03), open_g, "open over it", 0.0),
        Waypoint(1.0, (XC, ly0, PINCH_Z), open_g, "down round the latch", 0.0),
        Waypoint(0.8, (XC, ly0, PINCH_Z), shut_g, "squeeze the latch", 0.0),
        Waypoint(4.0, (XC, ly1, PINCH_Z), shut_g, "push the guide and paper up to the feed edge", 0.0),
        Waypoint(1.0, (XC, ly1, PINCH_Z), shut_g, "hold the stack square", 0.0),
        Waypoint(1.0, (XC, ly2, PINCH_Z), shut_g, "ease the guide back to just touching", 0.0),
        Waypoint(0.6, (XC, ly2, PINCH_Z), shut_g, "settle", 0.0),
        Waypoint(0.8, (XC, ly2, PINCH_Z), open_g, "let the latch go", 0.0),
        Waypoint(1.0, (XC, ly2, PINCH_Z + 0.05), open_g, "up", 0.0),
        Waypoint(1.5, (0.20, 0.24, TOP + 0.16), FINGER_OPEN, "park", 0.0),
    ]
    t_left = sum(w.duration for w in left)
    # near width guide: latch spans x WN0-0.021 .. WN0-0.003; it must travel (WIDE - A4 width) / 2
    wx0 = WN0 - 0.012
    wx1 = wx0 + (WIDE - A4[0]) / 2 + PUSH_W
    wx2 = wx0 + (WIDE - A4[0]) / 2 + EASE_W
    yaw = math.pi / 2  # fingers close along x
    right = [
        Waypoint(1.5, (0.20, -0.22, TOP + 0.14), FINGER_OPEN, "aside", yaw),
        Waypoint(max(0.5, t_left - 4.0), (0.20, -0.22, TOP + 0.14), FINGER_OPEN, "wait", yaw),
        Waypoint(1.5, (wx0, TAB_YC, PINCH_Z + 0.03), open_g, "over the width latch", yaw),
        Waypoint(1.0, (wx0, TAB_YC, PINCH_Z), open_g, "down round the latch", yaw),
        Waypoint(0.8, (wx0, TAB_YC, PINCH_Z), shut_g, "squeeze the latch", yaw),
        Waypoint(2.0, (wx1, TAB_YC, PINCH_Z), shut_g, "close the guides on the paper", yaw),
        Waypoint(0.6, (wx1, TAB_YC, PINCH_Z), shut_g, "hold the stack square", yaw),
        Waypoint(0.8, (wx2, TAB_YC, PINCH_Z), shut_g, "ease back to just touching", yaw),
        Waypoint(0.5, (wx2, TAB_YC, PINCH_Z), shut_g, "settle", yaw),
        Waypoint(0.8, (wx2, TAB_YC, PINCH_Z), open_g, "let the latch go", yaw),
        Waypoint(1.0, (wx2, TAB_YC, PINCH_Z + 0.05), open_g, "up", yaw),
        Waypoint(1.5, (0.20, -0.24, TOP + 0.16), FINGER_OPEN, "park", 0.0),
        Waypoint(2.0, (0.20, -0.24, TOP + 0.16), FINGER_OPEN, "wait", 0.0),
    ]
    return {"left": left, "right": right}
