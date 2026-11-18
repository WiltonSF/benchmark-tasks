"""On the station table, in front of the arms: a low wooden nightstand (25 cm tall) with one drawer, its front facing
the robot. The drawer, full of folded clothes, stands open about 10 cm and has stuck there: the sleeve of the blue
T-shirt on top of the pile is bunched up between the clothes and the rail under the nightstand's top, and part of it
has gone in behind that rail. The drawer has a black bar handle. Nothing else is on the table.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# --- the nightstand (world frame: x away from the arms, y to the robot's left, z up; heights above the table) -----
XF = 0.38  # the carcass's front face (the arms pull the drawer toward -x)
HALF_W = 0.21  # carcass outside half-width (42 cm wide)
DEPTH = 0.32  # carcass outside depth
HEIGHT = 0.25
PANEL = 0.018  # carcass and drawer-front board thickness
OPEN_LO, OPEN_HI = 0.03, 0.19  # the bottom drawer's opening in the carcass (bottom, top)
RAIL = (0.19, 0.232, 0.02)  # the rail under the top's front edge: underside, top, depth (x)
TOP_UNDER = HEIGHT - PANEL  # underside of the top board, behind the rail

D0 = 0.10  # the bottom drawer starts open 10 cm, where it jammed
TRAVEL = 0.25  # its slides stop at 25 cm open
FULL = 0.22  # "fully open": at least 22 cm

FRONT_Z = (0.035, 0.185)  # drawer front (14.5 cm tall)
FRONT_HW = 0.185
BOX_D = 0.272  # drawer box depth behind its front
BOX_HW, BOX_WALL = 0.18, 0.010
BOX_Z = (0.040, 0.160)  # drawer box bottom and side tops
PLY = 0.006
PILE_Z = 0.115  # top of the other folded clothes
SHIRT_Z = 0.130  # top of the folded blue T-shirt
HANDLE = (0.012, 0.12, 0.02)  # bar: thick (x), long (y), tall (z)
STEM = 0.045  # the bar stands 45 mm off the drawer front
HANDLE_Z = 0.125  # bar centre height
STEM_Y = 0.045

WOOD, WOOD_DARK, WOOD_EDGE = (0.70, 0.55, 0.38), (0.46, 0.33, 0.21), (0.60, 0.46, 0.31)
HANDLE_C = (0.10, 0.10, 0.11)
SHIRT_C, SLEEVE_C = (0.92, 0.72, 0.10), (0.95, 0.76, 0.12)  # a mustard-yellow T-shirt
PILE_C = ((0.30, 0.30, 0.33), (0.62, 0.62, 0.64), (0.88, 0.88, 0.86))  # charcoal, grey, white


def _b(x0, x1, y0, y1, z0, z1, color, collide=True):
    """A box from world extents (z above the table), in the nightstand's part frame (origin at the table under the
    carcass's front-face centre)."""
    return parts.box((x1 - x0, y1 - y0, z1 - z0), at=((x0 + x1) / 2 - XF, (y0 + y1) / 2, (z0 + z1) / 2), color=color,
                     collide=collide)


X0, X1 = XF, XF + DEPTH
CARCASS = [
    _b(X0, X1, -HALF_W, -HALF_W + PANEL, 0.0, HEIGHT, WOOD),  # sides
    _b(X0, X1, HALF_W - PANEL, HALF_W, 0.0, HEIGHT, WOOD),
    _b(X1 - PANEL, X1, -HALF_W + PANEL, HALF_W - PANEL, 0.0, HEIGHT, WOOD),  # back
    _b(X0, X1, -HALF_W, HALF_W, HEIGHT - PANEL, HEIGHT, WOOD),  # top
    _b(X0, X1 - PANEL, -HALF_W + PANEL, HALF_W - PANEL, 0.0, OPEN_LO, WOOD_DARK),  # plinth and bottom board
    _b(X0, X0 + RAIL[2], -HALF_W + PANEL, HALF_W - PANEL, RAIL[0], TOP_UNDER, WOOD_EDGE),  # rail under the top's front
    _b(X0, X0 + PANEL, -HALF_W + PANEL, -FRONT_HW, OPEN_LO, HEIGHT - PANEL, WOOD_EDGE),  # face-frame stiles
    _b(X0, X0 + PANEL, FRONT_HW, HALF_W - PANEL, OPEN_LO, HEIGHT - PANEL, WOOD_EDGE),
]

# The bottom drawer, written where it starts (open D0): its front's outer face at x = XF - D0.
FX = XF - D0


def _drawer_shapes():
    fi = FX + PANEL  # front panel's inner face
    bx = FX + PANEL + BOX_D  # box back
    hx = FX - STEM - HANDLE[0] / 2
    pile = [_b(fi + 0.012, bx - 0.012, -0.165, 0.165, BOX_Z[0] + PLY + 0.023 * k, BOX_Z[0] + PLY + 0.023 * (k + 1),
               PILE_C[k]) for k in range(3)]
    return [
        _b(FX, fi, -FRONT_HW, FRONT_HW, FRONT_Z[0], FRONT_Z[1], WOOD),  # front
        _b(fi, bx, -BOX_HW, BOX_HW, BOX_Z[0], BOX_Z[0] + PLY, WOOD_EDGE),  # bottom
        _b(fi, bx, -BOX_HW, -BOX_HW + BOX_WALL, BOX_Z[0], BOX_Z[1], WOOD_EDGE),  # sides
        _b(fi, bx, BOX_HW - BOX_WALL, BOX_HW, BOX_Z[0], BOX_Z[1], WOOD_EDGE),
        _b(bx - BOX_WALL, bx, -BOX_HW + BOX_WALL, BOX_HW - BOX_WALL, BOX_Z[0], BOX_Z[1], WOOD_EDGE),  # back
        *pile,  # three layers of folded clothes, 23 mm each
        _b(fi + 0.027, bx - 0.030, -0.02, 0.165, PILE_Z, SHIRT_Z, SHIRT_C),  # the folded blue T-shirt
        _b(hx - HANDLE[0] / 2, hx + HANDLE[0] / 2, -HANDLE[1] / 2, HANDLE[1] / 2, HANDLE_Z - HANDLE[2] / 2,
           HANDLE_Z + HANDLE[2] / 2, HANDLE_C),  # bar handle
        *[_b(FX - STEM, FX, sy * STEM_Y - 0.004, sy * STEM_Y + 0.004, HANDLE_Z - 0.006, HANDLE_Z + 0.006, HANDLE_C)
          for sy in (-1, 1)],
    ]


# The sleeve: five 9 cm wide, 4 mm thick segments hinged edge to edge across the sleeve (axis y), from the shirt's
# front edge. Centre-line points (x, z) as it starts: up out of the clothes, forward over them, under the rail and
# up behind it.
SLEEVE_Y = (0.055, 0.145)
SLEEVE_T = 0.004
SLEEVE_PTS = ((FX + PANEL + 0.027, SHIRT_Z + SLEEVE_T / 2), (0.332, 0.170), (0.365, 0.178), (0.410, 0.180),
              (0.435, 0.203), (0.470, 0.211))
SEG_MASS = 0.012  # kg each
SEG_SPRING, SEG_DAMP = 1.5, 0.006  # N m/rad, N m s/rad: the fabric's stiffness about each fold line
YIELD = 0.12  # rad: bent further than this from its rest shape, a fold line takes the new shape (stays put)
SEGMENTS = [f"sleeve{k + 1}" for k in range(len(SLEEVE_PTS) - 1)]


def _quat_y(a):
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def _sleeve_joints():
    out = []
    for k in range(len(SLEEVE_PTS) - 1):
        (xa, za), (xb, zb) = SLEEVE_PTS[k], SLEEVE_PTS[k + 1]
        n = math.hypot(xb - xa, zb - za)
        pitch = -math.atan2(zb - za, xb - xa)  # about +y: positive turns +x toward -z
        seg = parts.box((n, SLEEVE_Y[1] - SLEEVE_Y[0], SLEEVE_T),
                        at=((xa + xb) / 2 - XF, sum(SLEEVE_Y) / 2, (za + zb) / 2), color=SLEEVE_C, quat=_quat_y(pitch))
        lower, upper = (-0.72, 1.35) if k == 0 else (-2.6, 2.6)  # the first can lie flat back or lean 40 deg forward
        out.append(parts.Hinge(SEGMENTS[k], shapes=(seg,), pivot=(xa - XF, sum(SLEEVE_Y) / 2, za), axis=(0.0, 1.0, 0.0),
                               lower=lower, upper=upper, mass=SEG_MASS, spring=SEG_SPRING, damping=SEG_DAMP,
                               max_effort=5.0, parent="drawer" if k == 0 else SEGMENTS[k - 1]))
    return out


DRAWER_MASS = 1.6  # kg: the drawer with its clothes
NIGHTSTAND = parts.Part(
    "nightstand", pos=(XF, 0.0, TOP), shapes=CARCASS,
    joints=[parts.Slider("drawer", shapes=tuple(_drawer_shapes()), axis=(-1.0, 0.0, 0.0), travel=TRAVEL - D0,
                         extend=D0, mass=DRAWER_MASS, spring=0.0, damping=20.0, max_effort=400.0,
                         friction=4.0, dynamic_friction=3.0),
            *_sleeve_joints()],
)
OBJECTS = [NIGHTSTAND]

# --- the snag rule (a stand-in for the fabric's response; the same geometry drives the simulator and the score) ----
# Fabric that stands within 5 mm of the rail's underside, inside the carcass, catches on the rail as the drawer comes
# out. Once that fabric is within 10 mm of the rail's back face, a stiff spring (4,000 N/m) holds the drawer back;
# the force it carries is the strain on the sleeve. Over 20 N the sleeve counts as torn.
SNAG_Z = RAIL[0] - 0.005
RAIL_BACK = XF + RAIL[2]
ENGAGE = 0.010
K_SNAG = 4000.0
TEAR_N = 20.0
EPS = 1e-6
CLEAR = 0.005  # let go: no hand point within 5 mm of the drawer or sleeve
STILL_S, STILL_MM = 1.0, 0.002
INSIDE_ABOVE = 0.03  # sleeve points may stand up to 3 cm above the drawer's side tops


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def _grid(c, r, s, n=(5, 3, 2)) -> np.ndarray:
    u = [np.linspace(-0.5, 0.5, k) for k in n]
    g = np.stack(np.meshgrid(*u, indexing="ij"), -1).reshape(-1, 3) * np.asarray(s)
    return c + g @ np.asarray(r).T


def _link_poses(st) -> dict:
    poses = {"base": (np.asarray(st.pos, float), _rot(st.quat_xyzw))}
    for name, (p, q) in st.links.items():
        poses[name] = (np.asarray(p, float), _rot(q))
    return poses


def sleeve_points(st) -> np.ndarray:
    """Points over each sleeve segment's box (world), from the part state."""
    poses = _link_poses(st)
    if any(n not in poses for n in ["drawer", *SEGMENTS]):
        raise KeyError("sleeve state unknown")
    return np.concatenate([_grid(c, r, s) for _, c, r, s in NIGHTSTAND.world_boxes({n: poses[n] for n in SEGMENTS})])


def opening(st) -> float:
    return D0 + float(st.joints["drawer"])


def snag_gap(pts) -> float | None:
    """How far behind the rail's back face the foremost caught fabric is (m), or None if nothing is caught."""
    z = pts[:, 2] - TOP
    caught = (z >= SNAG_Z - EPS) & (pts[:, 0] >= XF - EPS)
    if not caught.any():
        return None
    return float(pts[caught, 0].min() - RAIL_BACK)


def snag_force(gap) -> float:
    return 0.0 if gap is None else K_SNAG * max(0.0, ENGAGE - gap)


def inside_drawer(pts, o: float) -> bool:
    """Every sleeve point within the drawer box (behind its front, ahead of its back, between its sides) and no
    more than 3 cm above its side tops."""
    fi = XF - o + PANEL
    bx = fi + BOX_D - BOX_WALL
    hw = BOX_HW - BOX_WALL
    z = pts[:, 2] - TOP
    return bool(np.all((pts[:, 0] >= fi - EPS) & (pts[:, 0] <= bx + EPS) & (np.abs(pts[:, 1]) <= hw + EPS)
                       & (z >= BOX_Z[0] - EPS) & (z <= BOX_Z[1] + INSIDE_ABOVE + EPS)))


def drawer_boxes(st):
    poses = _link_poses(st)
    return NIGHTSTAND.world_boxes({n: poses[n] for n in ["drawer", *SEGMENTS]})


def hand_gap(hand_pts, st) -> float:
    """Smallest distance from any sampled hand point to the drawer (front, handle, box, clothes) or the sleeve."""
    from yam_arena.analysis import box_signed_distance

    return float(min(box_signed_distance(hand_pts, c, r, s).min() for _, c, r, s in drawer_boxes(st)))


# --- the trial record (in a module of its own so a second copy of this file shares it) ---------------------------
_REC: dict = sys.modules.setdefault("_sleeve_drawer_record", type(sys)("_sleeve_drawer_record")).__dict__.setdefault(
    "rec", {})


def reset_record() -> None:
    _REC.clear()
    _REC.update(t=0.0, steps=0, known=True, reason="", torn=False, peak_n=0.0, open=D0, freed=False, inside=False,
                clear=False, still=False, hist=[], last_t=None)


def record(st, hand_pts, dt: float) -> None:
    """One control step. ``st`` is the nightstand's state and ``hand_pts`` every sampled point of both grippers and
    all four fingertips (world); None for either when the simulator could not give it, which fails the trial."""
    if not _REC:
        reset_record()
    try:
        if st is None:
            raise ValueError("nightstand state unknown")
        pts = sleeve_points(st)
        o = opening(st)
        if hand_pts is None or len(hand_pts) == 0 or not np.isfinite(hand_pts).all():
            raise ValueError("hand state unknown")
        if not np.isfinite(pts).all() or not math.isfinite(o):
            raise ValueError("nightstand state not finite")
        gap = hand_gap(hand_pts, st)
    except (KeyError, ValueError, AttributeError, TypeError) as e:
        _REC["known"] = False
        _REC["reason"] = str(e)
        _REC["t"] += dt
        _REC["steps"] += 1
        return
    g = snag_gap(pts)
    f = snag_force(g)
    _REC["peak_n"] = max(_REC["peak_n"], f)
    if f > TEAR_N + EPS:
        _REC["torn"] = True
    _REC["open"] = o
    _REC["freed"] = g is None
    _REC["inside"] = inside_drawer(pts, o)
    _REC["clear"] = bool(gap > CLEAR + EPS)
    hist = _REC["hist"]
    hist.append((_REC["t"], o, pts))
    while hist and hist[0][0] < _REC["t"] - STILL_S - 1e-9:
        hist.pop(0)
    _REC["still"] = bool(hist[0][0] <= _REC["t"] - STILL_S + 1e-9 + dt
                         and max(max(abs(h_o - o), float(np.abs(h_p - pts).max())) for _, h_o, h_p in hist)
                         <= STILL_MM + EPS)
    _REC["t"] += dt
    _REC["steps"] += 1


def _settled() -> bool:
    """Known all trial, never torn, freed, the whole sleeve in the drawer, both hands clear and everything still."""
    return bool(_REC and _REC["known"] and _REC["steps"] > 0 and not _REC["torn"] and _REC["freed"]
                and _REC["inside"] and _REC["clear"] and _REC["still"])


def success(objects):
    return bool(_settled() and _REC["open"] >= FULL - EPS)


def score(objects):
    """1.0: the bottom drawer open at least 22 cm, the sleeve free of the rail and wholly inside the drawer, both
    hands at least 5 mm clear of drawer and sleeve, all still for 1 s. 0.5: the same with the drawer open less than
    22 cm. 0 otherwise, and 0 for the whole trial once the sleeve's strain passed 20 N (torn) or the hand or
    nightstand state was unknown at any step."""
    if success(objects):
        return 1.0
    return 0.5 if _settled() else 0.0


reset_record()

# --- simulator hooks ---------------------------------------------------------------------------------------------
HAND_LINKS = ("tip_left", "tip_right", "gripper")
SAMPLE = 0.003
_HAND_PTS: dict = {}


def hand_points(link: str) -> np.ndarray:
    """Points on a hand link's collision surfaces (link frame): every vertex plus each triangle face sampled about
    every 3 mm, so a flat face resting on the drawer is seen even when no vertex is near it."""
    if link not in _HAND_PTS:
        import xml.etree.ElementTree as ET

        from yam_arena import ASSET_DIR, YAM_URDF, analysis
        from yam_arena.urdf import rpy_matrix

        pts = []
        for lk in ET.parse(YAM_URDF).getroot().iter("link"):
            if lk.get("name") != link:
                continue
            for col in lk.findall("collision"):
                origin = col.find("origin")
                rot, off = np.eye(3), np.zeros(3)
                if origin is not None:
                    rot = rpy_matrix(np.array([float(v) for v in origin.get("rpy", "0 0 0").split()]))
                    off = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
                tris = analysis.read_stl(ASSET_DIR / col.find("geometry/mesh").get("filename")) @ rot.T + off
                pts.append(_sample_triangles(tris))
        if not pts:
            raise ValueError(f"no collision mesh for {link}")
        _HAND_PTS[link] = np.concatenate(pts)
    return _HAND_PTS[link]


def _sample_triangles(tris) -> np.ndarray:
    pts = []
    for t in np.asarray(tris, float):
        edge = max(np.linalg.norm(t[1] - t[0]), np.linalg.norm(t[2] - t[1]), np.linalg.norm(t[0] - t[2]))
        n = max(1, int(math.ceil(edge / SAMPLE)))
        for i in range(n + 1):
            for j in range(n + 1 - i):
                pts.append(t[0] + (t[1] - t[0]) * i / n + (t[2] - t[0]) * j / n)
    return np.unique(np.round(np.array(pts) / 0.0015), axis=0) * 0.0015


def world_hand_points(link_poses) -> np.ndarray:
    """All hand points in the world from (link name, 4x4 world pose) pairs; all six hand links must be present."""
    if len(link_poses) != 2 * len(HAND_LINKS):
        raise ValueError("hand state unknown")
    return np.concatenate([hand_points(n) @ m[:3, :3].T + m[:3, 3] for n, m in link_poses])


def _sim_hand_points(sim) -> np.ndarray:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in HAND_LINKS:
                m = np.eye(4)
                m[:3, :3] = parts._quat_to_matrix(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                out.append((n, m))
    return world_hand_points(out)


_TARGETS: dict = {}


def _apply_fabric(sim, st) -> None:
    """The fabric stand-ins, every control step: each fold line keeps a bend once pushed past YIELD; and while
    fabric is caught behind the rail, the snag spring holds the drawer back."""
    for name in SEGMENTS:
        q, t = float(st.joints[name]), _TARGETS.get(name, 0.0)
        if abs(q - t) > YIELD:
            t = q - math.copysign(YIELD, q - t)
            _TARGETS[name] = t
            sim.set_target("nightstand", name, t)
    g = snag_gap(sleeve_points(st))
    q = float(st.joints["drawer"])
    if g is not None and g < ENGAGE:
        sim.set_stiffness("nightstand", "drawer", K_SNAG)
        sim.set_target("nightstand", "drawer", q + (g - ENGAGE))
    else:
        sim.set_stiffness("nightstand", "drawer", 0.0)
        sim.set_target("nightstand", "drawer", q)


@parts.on_reset
def _on_reset(sim) -> None:
    reset_record()
    _TARGETS.clear()
    sim.set_stiffness("nightstand", "drawer", 0.0)
    _REC["last_t"] = sim.t


@parts.on_step
def _on_step(sim) -> None:
    if _REC.get("last_t") == sim.t:  # a second copy of this module sharing the record: step it once
        return
    dt = sim.t - (_REC["last_t"] if _REC.get("last_t") is not None else 0.0)
    _REC["last_t"] = sim.t
    try:
        st = sim.states()["nightstand"]
    except Exception:  # noqa: BLE001  unknown state: no credit
        st = None
    try:
        pts = _sim_hand_points(sim)
    except Exception:  # noqa: BLE001  unknown hand state: no credit
        pts = None
    if st is not None:
        _apply_fabric(sim, st)
    record(st, pts, max(dt, 0.0))
    if os.environ.get("TRACE_OUT") and _REC["steps"] % 5 == 0:
        import json

        row = dict(t=round(_REC["t"], 3), known=_REC["known"], reason=_REC["reason"], open=round(_REC["open"], 4),
                   peak_n=round(_REC["peak_n"], 2), torn=_REC["torn"], freed=_REC["freed"], inside=_REC["inside"],
                   clear=_REC["clear"], still=_REC["still"], score=score(None))
        try:
            sp = sleeve_points(st)
            g = snag_gap(sp)
            row.update(gap=None if g is None else round(g, 4), force=round(snag_force(g), 2),
                       sleeve_top=round(float(sp[:, 2].max() - TOP), 4),
                       joints={k: round(v, 4) for k, v in st.joints.items()})
        except Exception:  # noqa: BLE001
            pass
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("drawer.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


# --- demo: fixed waypoints from the published layout, no state read ----------------------------------------------
# The left hand, fingers shut, comes down into the 8 cm gap in front of the rail and presses the bunched sleeve flat
# onto the clothes, which swings the part behind the rail down with it; it lifts away. The right hand then takes the
# bar handle across its thickness, pulls the drawer out to 24 cm, lets go and backs off.
SHUT = 0.0
PRESS = dict(x=0.339, y=sum(SLEEVE_Y) / 2)  # TCP: the 68 mm finger pair centred in the 82 mm gap
TIP = 0.013  # the pad tips reach this far below the TCP
BAR_X = FX - STEM - HANDLE[0] / 2
GRIP_Z = HANDLE_Z - HANDLE[2] / 2 + 0.002 + TIP  # pad tips 2 mm above the bar's underside
BAR_PINCH = -(HANDLE[0] / 2 - 0.003)  # close 3 mm past the bar's faces
BAR_NEAR = -(HANDLE[0] / 2 + 0.007)
PULL_TO = 0.24  # opening the pull is aimed at
LIFT = 0.18  # a straight-down grip reaches only about 0.20 m above the table here
ABOVE = 0.185  # fingertips 13 mm under the drawer front's top: leave the gap sideways, never toward the front


def _w(t, x, y, z, finger, label, yaw=0.0):
    return Waypoint(t, (x, y, TOP + z), finger, label, yaw=yaw)


def naive_demo():
    """For a trace only (TASK_DEMO=naive): pull the handle straight out, as a robot that ignores the sleeve would."""
    y, yaw = 0.0, math.pi / 2
    return {"right": [
        _w(2.0, BAR_X, y, GRIP_Z + 0.05, BAR_NEAR, "over the handle", yaw),
        _w(1.2, BAR_X, y, GRIP_Z, BAR_NEAR, "down round the bar", yaw),
        _w(0.8, BAR_X, y, GRIP_Z, BAR_PINCH, "pinch the bar", yaw),
        _w(4.0, BAR_X - (PULL_TO - D0), y, GRIP_Z, BAR_PINCH, "pull straight out", yaw),
        _w(3.0, BAR_X - (PULL_TO - D0), y, GRIP_Z, BAR_PINCH, "keep pulling", yaw),
    ]}


def demo():
    if os.environ.get("TASK_DEMO") == "naive":
        return naive_demo()
    px, py = PRESS["x"], PRESS["y"]
    left = [
        _w(2.5, px, py, ABOVE, SHUT, "over the gap"),
        _w(2.0, px, py, SHIRT_Z + TIP - 0.004, SHUT, "press the sleeve flat"),
        _w(1.0, px, py, SHIRT_Z + TIP - 0.004, SHUT, "hold it down"),
        _w(1.5, px, py, ABOVE, SHUT, "lift"),
        _w(1.5, px, 0.30, ABOVE, SHUT, "out of the gap sideways"),
        _w(1.5, 0.22, 0.30, LIFT, SHUT, "park left"),
    ]
    yaw = math.pi / 2
    pull = BAR_X - (PULL_TO - D0)
    right = [
        _w(11.0, 0.20, -0.20, LIFT, BAR_NEAR, "wait clear", yaw),
        _w(2.0, BAR_X, 0.0, GRIP_Z + 0.05, BAR_NEAR, "over the handle", yaw),
        _w(1.2, BAR_X, 0.0, GRIP_Z, BAR_NEAR, "down round the bar", yaw),
        _w(0.8, BAR_X, 0.0, GRIP_Z, BAR_PINCH, "pinch the bar", yaw),
        _w(4.0, pull, 0.0, GRIP_Z, BAR_PINCH, "pull the drawer out", yaw),
        _w(0.8, pull, 0.0, GRIP_Z, BAR_NEAR, "let go", yaw),
        _w(1.0, pull, 0.0, GRIP_Z + 0.045, BAR_NEAR, "up off the bar", yaw),
        _w(1.5, pull, -0.26, GRIP_Z + 0.045, BAR_NEAR, "out sideways", yaw),
        _w(1.5, 0.15, -0.26, LIFT, BAR_NEAR, "park right", yaw),
    ]
    return {"left": left, "right": right}
