"""On the station table: a square section of painted drywall lies on its back on two wooden battens, held at its
corners by four clips. A white stick-on utility hook is stuck to the painted face near one edge with a white foam
adhesive strip; the strip's grip tab hangs over that edge. Which edge the hook is on changes between trials. A low
grey tray stands on the table nearer the robot.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

# -- colours (sRGB) ----------------------------------------------------------------------------------------------
PAINT = (0.90, 0.88, 0.80)  # eggshell wall paint
GYPSUM = (0.82, 0.80, 0.76)
WOOD = (0.62, 0.47, 0.30)
CLIP = (0.25, 0.25, 0.27)
HOOK_WHITE = (0.97, 0.97, 0.96)
FOAM = (0.99, 0.99, 0.99)
TAB_COL = (0.85, 0.20, 0.15)  # the printed pull tab
TRAY_COL = (0.45, 0.47, 0.50)
TORN = (0.56, 0.44, 0.30)  # bare brown drywall paper where paint came away

# -- the panel (its own frame: origin at the middle of the painted face, z out of the face) -----------------------
PANEL = 0.250  # m square
PANEL_T = 0.0127  # 1/2 in drywall
PANEL_MASS = 0.50  # kg (1/2 in gypsum board, about 8 kg/m^2)
BATTEN_H = 0.050
PANEL_XY = (0.27, 0.0)
FACE_Z = TOP + BATTEN_H + PANEL_T  # the painted face
EDGE = -PANEL / 2  # the edge the tab hangs over, in the panel frame (y)

# -- hook and strip (panel frame, lot A orientation: "down the wall" is -y) ---------------------------------------
HX = 0.0  # hook centre line across the panel
STRIP_W = 0.019  # medium strip, 3/4 in wide
STRIP_T = 0.0012  # foam thickness
CELL = 0.010  # bond cells along the strip
N_CELLS = 5  # 50 mm bonded under the plate
BOND_Y0 = EDGE + 0.005  # the bond's tab end
TAB_PIVOT_Y = EDGE - 0.014  # where the unbonded strip turns down over the edge (15 mm of strip past the bond)
TAB_LEN, TAB_T = 0.015, 0.002
PLATE = (0.024, 0.062, 0.004)  # hook back plate: 24 x 62 mm, 4 mm thick
PLATE_Y0 = EDGE + 0.003
PLATE_Z0 = STRIP_T
STEM = (0.014, 0.014, 0.021)  # the hook's stem, rising from the plate's upper end
STEM_Y = PLATE_Y0 + 0.044
ARM_Y1 = PLATE_Y0 + 0.062  # the hook's arm runs on "up" the wall to here
HOOK_MASS = 0.015
SEG_MASS = 0.008
TAB_MASS = 0.006

# Bond and strip mechanics (see README): the bond holds the plate on stiff springs in all three directions; the
# strip's segments stretch on springs once their bond lets go.
SHEAR_K, LIFT_K = 4000.0, 3000.0  # N/m
SHEAR_BREAK, LIFT_BREAK = 0.004, 0.002  # m: 16 N sideways or 6 N off the face tears the bond out with its paint
SEG_K = 120.0  # N/m per segment: the strip pulls about 7.3 N while it lets go
E_REL = 0.061  # m: a 10 mm cell lets go once its segment is stretched 61 mm (5 cells: 305 mm, 12 in, of pull)
E_MAX = 0.075
LOCKED = 0.0002  # a still-bonded segment can stretch 0.2 mm
PEEL_TEAR = math.radians(30.0)  # a cell that lets go with the tab lifted more than 30 degrees off the face tears
PEEL_RIP = math.radians(45.0)  # lifted more than 45 degrees under half the release pull, the front peels with paint
RIP_E = E_REL * 0.5
TAB_FLAT_K = 0.05  # N m/rad on the tab's hinge once the hook is free


def _cell_y(i: int) -> float:
    return BOND_Y0 + (i + 0.5) * CELL


def _box(lo, hi, color, collide=True):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(hi - lo), at=tuple((lo + hi) / 2), color=color, collide=collide)


_GHOST = (0.0, 0.0, 0.0)
HIDDEN = (HX, STEM_Y, PLATE_Z0 + PLATE[2] + STEM[2] / 2)  # inside the stem


def _ghost():
    return (parts.box((0.002, 0.002, 0.002), at=HIDDEN, color=HOOK_WHITE, collide=False),)


HOOK_SHAPES = (
    _box((HX - PLATE[0] / 2, PLATE_Y0, PLATE_Z0), (HX + PLATE[0] / 2, PLATE_Y0 + PLATE[1], PLATE_Z0 + PLATE[2]),
         HOOK_WHITE),
    _box((HX - STEM[0] / 2, STEM_Y - STEM[1] / 2, PLATE_Z0 + PLATE[2]),
         (HX + STEM[0] / 2, STEM_Y + STEM[1] / 2, PLATE_Z0 + PLATE[2] + STEM[2]), HOOK_WHITE),
    _box((HX - STEM[0] / 2, STEM_Y + STEM[1] / 2, PLATE_Z0 + PLATE[2] + STEM[2] - 0.005),
         (HX + STEM[0] / 2, ARM_Y1, PLATE_Z0 + PLATE[2] + STEM[2]), HOOK_WHITE),
    _box((HX - STEM[0] / 2, ARM_Y1 - 0.005, PLATE_Z0 + PLATE[2] + 0.010),
         (HX + STEM[0] / 2, ARM_Y1, PLATE_Z0 + PLATE[2] + STEM[2] - 0.005), HOOK_WHITE),
)


def _segment(i: int):
    """Segment i (0 = the tab end): its 10 mm of strip, and a thin visual tail that fills the gap it opens as it
    stretches (hidden under the plate and the next segments until then)."""
    y0 = BOND_Y0 + i * CELL
    shapes = [_box((HX - STRIP_W / 2, y0, 0.0001), (HX + STRIP_W / 2, y0 + CELL, STRIP_T - 0.0001), FOAM, False)]
    if i == 0:  # the unbonded strip from the bond to the edge, where the tab turns down
        shapes.append(_box((HX - STRIP_W / 2, TAB_PIVOT_Y, 0.0001), (HX + STRIP_W / 2, y0, STRIP_T - 0.0001), FOAM,
                           False))
    if i < N_CELLS - 1:
        shapes.append(_box((HX - STRIP_W / 2 + 0.001, y0 + CELL, 0.0002),
                           (HX + STRIP_W / 2 - 0.001, y0 + CELL + E_MAX, STRIP_T - 0.0003), FOAM, False))
    parent = "bz" if i == N_CELLS - 1 else f"s{i + 2}"
    return parts.Slider(f"s{i + 1}", shapes=tuple(shapes), axis=(0.0, -1.0, 0.0), travel=E_MAX, mass=SEG_MASS,
                        spring=SEG_K, damping=1.0, parent=parent)


# The tab hangs straight down over the edge at hinge angle 0; it swings out and up about the panel's x axis:
# at 90 degrees it lies in the face's plane pointing away from the bond, beyond 90 it is lifted off the face.
TAB = parts.Hinge(
    "tab",
    shapes=(_box((HX - STRIP_W / 2, TAB_PIVOT_Y - TAB_T / 2, STRIP_T / 2 - TAB_LEN),
                 (HX + STRIP_W / 2, TAB_PIVOT_Y + TAB_T / 2, STRIP_T / 2), TAB_COL),),
    pivot=(HX, TAB_PIVOT_Y, STRIP_T / 2), axis=(-1.0, 0.0, 0.0), lower=-0.35, upper=math.pi, mass=TAB_MASS,
    spring=0.0, damping=0.0005, parent="s1",
)

# Bond slides: the hook's plate can move in x, y and z of the panel, held by the bond's springs until it lets go.
BX = parts.Slider("bx", shapes=_ghost(), axis=(1.0, 0.0, 0.0), travel=0.30, back=0.30, mass=0.02, spring=SHEAR_K,
                  damping=20.0, max_effort=400.0)
BY = parts.Slider("by", shapes=_ghost(), axis=(0.0, 1.0, 0.0), travel=0.30, back=0.30, mass=0.02, spring=SHEAR_K,
                  damping=20.0, max_effort=400.0, parent="bx")
BZ = parts.Slider("bz", shapes=HOOK_SHAPES, axis=(0.0, 0.0, 1.0), travel=0.20, back=0.08, mass=HOOK_MASS,
                  spring=LIFT_K, damping=15.0, max_effort=400.0, parent="by")
SEGMENTS = tuple(_segment(i) for i in reversed(range(N_CELLS)))  # parents first: s5 (on the plate) ... s1

PANEL_PART = parts.Part(
    "panel", pos=(*PANEL_XY, FACE_Z), yaw=0.0, mass=PANEL_MASS,
    shapes=(
        _box((-PANEL / 2, -PANEL / 2, -PANEL_T), (PANEL / 2, PANEL / 2, -0.0004), GYPSUM),
        _box((-PANEL / 2, -PANEL / 2, -0.0004), (PANEL / 2, PANEL / 2, 0.0), PAINT),
    ),
    joints=(BX, BY, BZ, *SEGMENTS, TAB),
)

# Battens and corner clips (fixed): the panel sits on the battens, its corners under the clips' lips.
_CLIP_GAP, _POST, _LIP = 0.0005, 0.008, 0.015


def _frame_shapes():
    px, py = PANEL_XY
    out = [_box((px + sx * 0.085 - 0.015, py - PANEL / 2, TOP), (px + sx * 0.085 + 0.015, py + PANEL / 2,
                TOP + BATTEN_H), WOOD) for sx in (-1, 1)]
    h = PANEL / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = px + sx * h, py + sy * h
            # a post beyond each edge at the corner, and a lip over the corner
            out.append(_box((min(cx, cx - sx * 0.030), min(cy + sy * _CLIP_GAP, cy + sy * (_CLIP_GAP + _POST)), TOP),
                            (max(cx, cx - sx * 0.030), max(cy + sy * _CLIP_GAP, cy + sy * (_CLIP_GAP + _POST)),
                             FACE_Z + 0.004), CLIP))
            out.append(_box((min(cx + sx * _CLIP_GAP, cx + sx * (_CLIP_GAP + _POST)), min(cy, cy - sy * 0.030), TOP),
                            (max(cx + sx * _CLIP_GAP, cx + sx * (_CLIP_GAP + _POST)), max(cy, cy - sy * 0.030),
                             FACE_Z + 0.004), CLIP))
            xs = sorted((cx - sx * _LIP, cx + sx * (_CLIP_GAP + _POST)))
            ys = sorted((cy - sy * _LIP, cy + sy * (_CLIP_GAP + _POST)))
            out.append(_box((xs[0], ys[0], FACE_Z + _CLIP_GAP), (xs[1], ys[1], FACE_Z + 0.004), CLIP))
    return tuple(out)


FRAME = parts.Part("frame", pos=(*PANEL_XY, TOP), shapes=tuple(
    parts.box(s.size, at=tuple(np.asarray(s.at) - np.array([*PANEL_XY, TOP])), color=s.color) for s in _frame_shapes()))

TRAY_XY = (0.078, 0.0)
TRAY_IN = (0.100, 0.150)  # inside, across x and along y (the hook lies along y in both lots)
TRAY_WALL, TRAY_H, TRAY_FLOOR = 0.004, 0.025, 0.003


def _tray_shapes():
    (ax, ay), w = (TRAY_IN[0] / 2, TRAY_IN[1] / 2), TRAY_WALL
    return (
        parts.box((TRAY_IN[0] + 2 * w, TRAY_IN[1] + 2 * w, TRAY_FLOOR), at=(0.0, 0.0, TRAY_FLOOR / 2),
                  color=TRAY_COL),
        parts.box((w, TRAY_IN[1] + 2 * w, TRAY_H), at=(ax + w / 2, 0.0, TRAY_H / 2), color=TRAY_COL),
        parts.box((w, TRAY_IN[1] + 2 * w, TRAY_H), at=(-ax - w / 2, 0.0, TRAY_H / 2), color=TRAY_COL),
        parts.box((TRAY_IN[0], w, TRAY_H), at=(0.0, ay + w / 2, TRAY_H / 2), color=TRAY_COL),
        parts.box((TRAY_IN[0], w, TRAY_H), at=(0.0, -ay - w / 2, TRAY_H / 2), color=TRAY_COL),
    )


TRAY = parts.Part("tray", pos=(*TRAY_XY, TOP), shapes=_tray_shapes())

OBJECTS = [PANEL_PART, FRAME, TRAY]

# Lot A: the hook near the edge on the robot's right, its tab hanging over that edge. Lot B: the panel turned half
# round in its clips, so the hook is near the left edge (and 40 mm farther from the robot) with the tab over it.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"panel": PANEL_PART.pos}),
    lots.Lot("B", place={"panel": PANEL_PART.pos}, yaw={"panel": math.pi}),
)

SUCCESS_HOLD_S = 1.0
CLEAR = 0.005  # let go: every gripper part more than 5 mm from the hook, its strip and tab
HOLD_S = 1.0
IN_TRAY_Z = 0.004  # plate's lowest point within 4 mm of the tray floor
FACE_STEP = 0.003
_HAND_LINKS = ("gripper", "tip_left", "tip_right")


# -- geometry helpers ----------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _link_poses(p) -> dict:
    out = {"base": (np.asarray(p.pos, float), _rot(p.quat_xyzw))}
    for name, (pos, qx) in getattr(p, "links", {}).items():
        out[name] = (np.asarray(pos, float), _rot(qx))
    return out


def _hook_boxes(p, with_strip=True):
    """World boxes (link, centre, rotation, size) of the hook, and of its strip and tab if asked."""
    names = {"bz", "tab"} | ({f"s{i + 1}" for i in range(N_CELLS)} if with_strip else set())
    poses = {k: v for k, v in _link_poses(p).items() if k in names}
    return PANEL_PART.world_boxes(poses)


def tab_lift(p) -> float:
    """How far the tab is lifted off the painted face (rad): 0 lying along the face away from the bond, negative
    hanging down past the edge, positive pulled up off the face."""
    return float(p.joints["tab"]) - math.pi / 2


# -- the bond record (whole trial; cleared only by the simulator's reset for a new trial) -------------------------
_rec = parts.__dict__.setdefault("hook_strip_record", {})


def fresh() -> None:
    _rec.update(released=[False] * N_CELLS, torn=[False] * N_CELLS, freed=False, blind=False, gap=None, t=None,
                since=None, held=False, cause=None,
                hook_prev=None, hook_v=None)


if not _rec:
    fresh()


def judge(p) -> list[int]:
    """Update the bond from one control step's panel state; returns the cells that let go at this step."""
    if _rec["freed"]:
        return []
    q = p.joints
    went = []
    if q["bz"] > LIFT_BREAK + EPS or math.hypot(q["bx"], q["by"]) > SHEAR_BREAK + EPS:
        for i in range(N_CELLS):  # the plate was pulled off: the paint under every bonded cell comes with it
            if not _rec["released"][i]:
                _rec["released"][i] = _rec["torn"][i] = True
                went.append(i)
        _rec["cause"] = _rec["cause"] or "plate pulled off the face"
    else:
        lift = tab_lift(p)
        while not all(_rec["released"]):
            i = _rec["released"].index(False)
            e = float(q[f"s{i + 1}"])
            if e >= E_REL - EPS:
                torn = lift > PEEL_TEAR + EPS
            elif lift > PEEL_RIP + EPS and e >= RIP_E - EPS:
                torn = True
            else:
                break
            _rec["released"][i] = True
            _rec["torn"][i] = torn
            if torn:
                _rec["cause"] = _rec["cause"] or "strip peeled up off the face"
            went.append(i)
            if not torn:
                break  # the next cell has not been stretched yet
    if all(_rec["released"]):
        _rec["freed"] = True
    return went


# -- hands and the release check -----------------------------------------------------------------------------------
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
    _, keep = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(keep)]])


_FACES: dict = {}


def _link_faces(name):
    if name not in _FACES:
        from yam_arena import analysis

        _FACES[name] = [(h, _face_points(h)) for h in analysis.link_hulls().get(name, [])]
    return _FACES[name]


class _WorldHull:
    def __init__(self, hull, faces, m):
        rot, pos = m[:3, :3], m[:3, 3]
        self.vertices = hull.vertices @ rot.T + pos
        n_w = hull.planes[:, :3] @ rot.T
        self.planes = np.c_[n_w, hull.planes[:, 3] + n_w @ pos]
        self.face_points = faces @ rot.T + pos


def gap(p, hulls) -> float:
    """The closest any point on a gripper hull's faces (sampled about every 3 mm, corners included) comes to the
    hook, its strip or its tab (m; negative inside)."""
    from yam_arena import analysis

    pts = np.concatenate([getattr(h, "face_points", h.vertices) for h in hulls])
    boxes = _hook_boxes(p)
    centre = np.mean([c for _, c, _, _ in boxes], axis=0)
    near = np.linalg.norm(pts - centre, axis=1) < 0.60
    if not near.any():
        return 0.05
    pts = pts[near]
    return float(min(analysis.box_signed_distance(pts, c, r, s).min() for _, c, r, s in boxes))


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


# -- end state -------------------------------------------------------------------------------------------------------
def hook_in_tray(p) -> bool:
    """The hook's plate lies inside the tray's walls, its lowest point within 4 mm of the floor, and it is still."""
    boxes = _hook_boxes(p, with_strip=False)
    hook = [b for b in boxes if b[0] == "bz"]
    corners = np.concatenate([c + (np.abs(r) @ (s / 2)) * np.array([[-1.0], [1.0]]) for _, c, r, s in hook])
    lo, hi = corners.min(axis=0), corners.max(axis=0)
    tx, ty = TRAY_XY
    ax, ay = TRAY_IN[0] / 2, TRAY_IN[1] / 2
    inside = lo[0] >= tx - ax - EPS and hi[0] <= tx + ax + EPS and lo[1] >= ty - ay - EPS and hi[1] <= ty + ay + EPS
    down = lo[2] <= TOP + TRAY_FLOOR + IN_TRAY_Z + EPS
    return bool(inside and down)


def hook_pos(p) -> np.ndarray:
    return np.asarray(_link_poses(p)["bz"][0], float)


def _hold(p, t) -> None:
    ok = (_rec["gap"] is not None and _rec["gap"] > CLEAR + EPS and _rec["freed"] and not any(_rec["torn"])
          and _rec["hook_v"] is not None and _rec["hook_v"] < 0.01 and hook_in_tray(p))
    if not ok:
        _rec.update(since=None, held=False)
        return
    if _rec["since"] is None:
        _rec["since"] = t
    _rec["held"] = t - _rec["since"] >= HOLD_S - EPS


# -- simulator hooks -----------------------------------------------------------------------------------------------
def _ids(asset, names):
    return asset.find_joints(list(names), preserve_order=True)[0]


def _write_bond(sim, on: bool) -> None:
    import torch

    base = sim.env.unwrapped
    panel = base.scene["panel"]
    ids = _ids(panel, ("bx", "by", "bz", "tab"))
    # Once free, the spent strip's tab lies back along the strip (a light spring on its hinge) instead of standing
    # on its end under the hook.
    k = [SHEAR_K, SHEAR_K, LIFT_K, 0.0] if on else [0.0, 0.0, 0.0, TAB_FLAT_K]
    c = [20.0, 20.0, 15.0, 0.0005] if on else [0.3, 0.3, 0.3, 0.003]
    panel.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([k], dtype=torch.float32, device=base.device),
                                             joint_ids=ids)
    panel.write_joint_damping_to_sim_index(damping=torch.tensor([c], dtype=torch.float32, device=base.device),
                                           joint_ids=ids)


def _write_strip_limits(sim) -> None:
    """Bonded segments are held at their length; the front segment and those already let go may stretch."""
    import torch

    base = sim.env.unwrapped
    panel = base.scene["panel"]
    names = [f"s{i + 1}" for i in range(N_CELLS)]
    rel = _rec["released"]
    front = rel.index(False) if not all(rel) else N_CELLS
    upper = [E_MAX if (rel[i] or i == front) else LOCKED for i in range(N_CELLS)]
    lim = torch.tensor([[[0.0, u] for u in upper]], dtype=torch.float32, device=base.device)
    panel.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=_ids(panel, names), warn_limit_violation=False)


_draw = {"markers": None, "off": False, "sig": None}


def _draw_torn(sim, p) -> None:
    """Brown tiles where paint came away (cosmetic; scoring never reads them)."""
    if _draw["off"]:
        return
    cells = [i for i in range(N_CELLS) if _rec["torn"][i]]
    sig = tuple(cells)
    if sig == _draw["sig"]:
        return
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        if _draw["markers"] is None:
            lin = tuple((c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4) for c in TORN)
            proto = sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=lin, roughness=0.9))
            _draw["markers"] = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/Visuals/torn_paint",
                                                                            markers={"torn": proto}))
        _draw["sig"] = sig
        m = _draw["markers"]
        if not cells:
            m.set_visibility(False)
            return
        r, o = _rot(p.quat_xyzw), np.asarray(p.pos, float)
        origin = parts._np(sim.env.unwrapped.scene.env_origins[0])
        pts = np.array([o + r @ np.array([HX, _cell_y(i), 0.0003]) + origin for i in cells], np.float32)
        yaw = math.atan2(r[1, 0], r[0, 0])
        quat = np.array([[0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]] * len(cells), np.float32)  # xyzw
        m.set_visibility(True)
        m.visualize(translations=pts, orientations=quat[:, [3, 0, 1, 2]],
                    scales=np.array([[STRIP_W + 0.002, CELL, 0.0006]] * len(cells), np.float32))
    except ImportError:
        _draw["off"] = True
    except Exception as e:  # noqa: BLE001
        _draw["off"] = True
        print(f"torn-paint tiles off ({e!r}); scoring is unaffected", file=sys.stderr, flush=True)


def _seat_panel(base) -> None:
    """Lay the panel exactly at its lot's pose, still (a turned articulation otherwise keeps its centre of mass)."""
    import torch

    panel = base.scene["panel"]
    lot = lots.current()
    x, y, z = lot.pos("panel")
    a = lot.yaw_of("panel")
    pose = torch.tensor([[x, y, z, 0.0, 0.0, math.sin(a / 2), math.cos(a / 2)]], dtype=torch.float32,
                        device=base.device)
    pose[:, :3] += base.scene.env_origins[0][:3].to(pose.dtype)
    panel.write_root_pose_to_sim_index(root_pose=pose)
    panel.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=base.device))


_lots_apply = lots._apply


def _apply_and_seat(emb) -> bool:
    _lots_apply(emb)
    base = emb._base
    _seat_panel(base)
    for _ in range(max(1, round(0.2 / base.step_dt))):
        emb._env.step(emb._ready)
    _seat_panel(base)
    for _ in range(lots.RENDERS):
        base.sim.render()
    return True


lots._apply = _apply_and_seat


@parts.on_reset
def _new_trial(sim) -> None:
    fresh()
    _write_bond(sim, True)
    _write_strip_limits(sim)
    for j in ("bx", "by", "bz", "tab", *(f"s{i + 1}" for i in range(N_CELLS))):
        sim.set_target("panel", j, 0.0)
    _draw_torn(sim, sim.states()["panel"])


DEBUG = False  # set True while tuning: prints the finger tips and the tab every 0.5 s


def _debug(sim, p) -> None:
    _rec["n"] = _rec.get("n", 0) + 1
    if _rec["n"] % 10:
        return
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        tips = [poses[i, :3] - origin for i, n in enumerate(asset.body_names) if n in ("tip_left", "tip_right")]
        out.append(f"{arm[0]} " + " ".join(f"{v * 1000:.0f}" for v in np.mean(tips, axis=0))
                   + f" w{np.linalg.norm(tips[0] - tips[1]) * 1000:.0f}")
    tab = _link_poses(p)["tab"][0]
    print(f"[dbg] t={float(sim.t):.1f} {' | '.join(out)} | tab " + " ".join(f"{v * 1000:.0f}" for v in tab)
          + f" s1 {p.joints['s1'] * 1000:.1f}", file=sys.stderr, flush=True)


@parts.on_step
def _step(sim) -> None:
    p = sim.states()["panel"]
    was_free = _rec["freed"]
    went = judge(p)
    if went:
        print(f"[hook] t={float(sim.t):.2f} cells {went} let go, torn={_rec['torn']}, tab lift "
              f"{math.degrees(tab_lift(p)):.0f} deg, bond {p.joints['bx']*1000:.1f}/{p.joints['by']*1000:.1f}/"
              f"{p.joints['bz']*1000:.1f} mm", file=sys.stderr, flush=True)
        _write_strip_limits(sim)
        _draw_torn(sim, p)
    if _rec["freed"] and not was_free:
        _write_bond(sim, False)
        sim.set_target("panel", "tab", math.pi / 2)
    hulls = _hands(sim)
    if DEBUG:
        _debug(sim, p)
    t, h = float(sim.t), hook_pos(p)
    if _rec["hook_prev"] is not None and t > _rec["hook_prev"][0] + EPS:
        _rec["hook_v"] = float(np.linalg.norm(h - _rec["hook_prev"][1]) / (t - _rec["hook_prev"][0]))
    _rec["hook_prev"] = (t, h)
    _rec["t"] = t
    if hulls is None:  # the arms could not be read: no release can be confirmed, so the trial earns nothing
        _rec.update(gap=None, since=None, held=False, blind=True)
        return
    _rec["gap"] = gap(p, hulls)
    _hold(p, _rec["t"])


# -- scoring ---------------------------------------------------------------------------------------------------------
def score(objects):
    """0 if paint came away anywhere at any time in the trial, or if the grippers could not be measured. Otherwise
    0.5 once the strip has let go of the whole bond, and 1.0 when the hook is also lying in the tray, let go
    (every gripper part more than 5 mm from it, its strip and tab) and still for 1 s."""
    if any(_rec["torn"]) or _rec["blind"] or not _rec["freed"]:
        return 0.0
    p = objects["panel"]
    released = _rec["gap"] is not None and _rec["gap"] > CLEAR + EPS
    if _rec["held"] and released and hook_in_tray(p):
        return 1.0
    return 0.5


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    return float(N_CELLS - sum(_rec["released"]))


# -- scripted demo -----------------------------------------------------------------------------------------------
# Open loop from the lot's published layout (which edge the hook is on, as the top camera shows it). One arm holds
# the hook by its stem; the other pinches the hanging tab by its edges and draws it slowly straight out along the
# face, past the edge, until the strip has let go; then it lets the tab go, and the first arm lifts the hook off
# and lays it in the tray.
TIP = 0.013
ARM_SOFT = 0.045  # the arm is soft (about 190 N/m): its target leads the tab by about 40 mm at the strip's 7 N


def _grip(gap_m: float) -> float:
    return FINGER_OPEN * gap_m / 0.094


def _world(lot, local):
    x, y = lot.pos("panel")[:2]
    a = lot.yaw_of("panel")
    c, s = math.cos(a), math.sin(a)
    return (x + c * local[0] - s * local[1], y + s * local[0] + c * local[1])


def _yaw(arm, h):
    """A wrist yaw (fingers symmetric: h or h + 180 degrees) inside the arm's comfortable range."""
    lo, hi = {"left": (-math.radians(145), math.radians(55)), "right": (-math.radians(55), math.radians(145))}[arm]
    cands = [h + k * math.pi for k in range(-3, 4) if lo <= h + k * math.pi <= hi]
    return min(cands, key=lambda y: abs(y - (lo + hi) / 2))


def demo():
    lot = lots.current()
    a = lot.yaw_of("panel")
    tab_side = _world(lot, (HX, EDGE))
    pull_arm = "right" if tab_side[1] < lot.pos("panel")[1] else "left"
    hold_arm = "left" if pull_arm == "right" else "right"
    across = a + math.pi / 2  # fingers closing across the strip (panel x)
    hy = _yaw(hold_arm, across)
    # the pulling arm reaches far out to its own side: its wrist turned so the fingers' closing axis stays across
    # the strip with the hand's back toward the arm's own base, which keeps the far end of the pull in reach
    want = -math.pi / 2 if pull_arm == "right" else math.pi / 2
    py = min((across + k * math.pi for k in range(-3, 4)), key=lambda y: abs(y - want))
    side = 0.20 if hold_arm == "left" else -0.20
    side_p = -side

    # The pulling hand pinches the hanging tab by its edges and draws it out along the face a little first, so the
    # two hands are not both crowded over the hook; then the other hand holds the hook while the pull goes on.
    pinch_y = TAB_PIVOT_Y - 0.008  # the pads (26 mm across) cover the tab, their inner ends 9 mm short of the edge
    pinch_z = FACE_Z - 0.002  # the pads (16 mm deep) cover the hanging tab
    level = pinch_z + 0.004
    open_p, close_p = _grip(0.034), _grip(STRIP_W - 0.012)  # squeezed 6 mm a side: the tab is pinched by its edges
    p0 = _world(lot, (HX, pinch_y))
    pull = 5 * E_REL + 0.025 + ARM_SOFT
    first = 0.150  # commanded; the soft arm lags the tab about 40 mm behind its target
    puller = [
        Waypoint(1.5, (0.24, side_p * 1.5, TOP + 0.12), FINGER_OPEN, "aside", py),
        Waypoint(2.0, (*p0, pinch_z + 0.04), open_p, "over the tab", py),
        Waypoint(1.2, (*p0, pinch_z), open_p, "down round the tab", py),
        Waypoint(1.0, (*p0, pinch_z), close_p, "pinch the tab", py),
        Waypoint(1.0, (*p0, level), close_p, "level with the face", py),
        Waypoint(2.0, (*_world(lot, (HX, pinch_y - first)), level), close_p, "start drawing the tab out", py),
    ]
    t_started = sum(w.duration for w in puller)

    stem = _world(lot, (HX, STEM_Y))
    grasp_z = FACE_Z + PLATE_Z0 + PLATE[2] + 0.004 + TIP  # finger tips 4 mm above the plate
    open_h, close_h = _grip(0.034), _grip(STEM[0] - 0.006)
    holder = [
        Waypoint(1.5, (0.22, side, TOP + 0.14), FINGER_OPEN, "aside", hy),
        Waypoint(max(0.5, t_started - 1.5), (0.22, side, TOP + 0.14), FINGER_OPEN, "wait", hy),
        Waypoint(2.0, (*stem, grasp_z + 0.025), open_h, "over the hook's stem", hy),
        Waypoint(1.2, (*stem, grasp_z), open_h, "down round the stem", hy),
        Waypoint(1.0, (*stem, grasp_z), close_h, "hold the hook", hy),
    ]
    t_held = sum(w.duration for w in holder)
    puller.append(Waypoint(max(0.5, t_held - t_started), puller[-1].pos, close_p, "wait for the hook to be held", py))
    n = 7
    for k in range(1, n + 1):
        d = first + (pull - first) * k / n
        pk = _world(lot, (HX, pinch_y - d))
        puller.append(Waypoint(1.6, (*pk, level), close_p, f"draw the tab out along the face {k}/{n}", py))
    t_pulled = sum(w.duration for w in puller)
    pe = _world(lot, (HX, pinch_y - pull))
    puller += [
        Waypoint(0.8, (*pe, level), open_p, "let the spent strip go", py),
        Waypoint(1.0, (*pe, pinch_z + 0.06), open_p, "up", py),
        Waypoint(1.5, (0.20, side_p * 1.4, TOP + 0.16), FINGER_OPEN, "park", 0.0),
    ]
    t_let = t_pulled + 0.8 + 0.5

    holder.append(Waypoint(max(0.5, t_let - t_held), (*stem, grasp_z), close_h, "hold until the strip lets go", hy))
    tray_z = TOP + TRAY_FLOOR + 0.006 + (PLATE[2] + 0.004 + TIP)
    # the hook with its spent strip and tab lying flat runs from the plate's upper end to the tab's end
    mid_y = (PLATE_Y0 + PLATE[1] + TAB_PIVOT_Y - TAB_LEN) / 2
    off = np.array(stem) - np.array(_world(lot, (HX, mid_y)))
    drop = (TRAY_XY[0] + off[0], TRAY_XY[1] + off[1])  # that length centred in the tray
    holder += [
        Waypoint(1.2, (*stem, grasp_z + 0.015), close_h, "lift the hook off", hy),
        Waypoint(2.5, (*drop, grasp_z + 0.015), close_h, "over the tray", hy),
        Waypoint(1.5, (*drop, tray_z), close_h, "down into the tray", hy),
        Waypoint(0.8, (*drop, tray_z), open_h, "let go", hy),
        Waypoint(1.0, (*drop, tray_z + 0.06), open_h, "up", hy),
        Waypoint(1.5, (0.18, side * 1.3, TOP + 0.16), FINGER_OPEN, "park", 0.0),
        Waypoint(2.5, (0.18, side * 1.3, TOP + 0.16), FINGER_OPEN, "wait", 0.0),
    ]
    return {hold_arm: holder, pull_arm: puller}
