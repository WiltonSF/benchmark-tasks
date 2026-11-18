"""A shop checkout counter top on the station table. Set flush into the counter, to the robot's left of centre, is a
round magnetic tag detacher: a dark grey disc with a round well in its middle. To the right of it, a navy jacket
cuff lies on the counter with a grey anti-theft hard tag pinned through its hem: the rectangular tag lies under the
cuff's edge, sticking out toward the detacher, and the round silver head of the tag's pin shows on top of the hem.
The tag's lock dome is on its underside, under the pin.
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
EPS = 1e-6
MM = 0.001

# --- the counter top and its flush-mounted detacher (world: x away from the arms, y to the robot's left) --------
B_T = 0.025  # counter top thickness
B = TOP + B_T  # counter surface
D = (0.27, 0.035)  # detacher (well) centre
PUCK_R = 0.037  # detacher top: 74 mm across
WELL_R = 0.014  # the well: 28 mm across ...
WELL_D = 0.007  # ... and 7 mm deep
BOARD = (0.16, 0.40, -0.26, 0.14)  # x0, x1, y0, y1 of the counter top
HOLE = 0.040  # half-size of the square cut-out the detacher sits in

COUNTER_C, PUCK_C, RING_C, WELL_C = (0.78, 0.74, 0.66), (0.20, 0.20, 0.22), (0.72, 0.73, 0.75), (0.08, 0.08, 0.09)


def _b(x0, x1, y0, y1, z0, z1, color, collide=True, friction=None):
    return parts.box((x1 - x0, y1 - y0, z1 - z0), at=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), color=color,
                     collide=collide, friction=friction)


def _counter_shapes():
    """Counter-frame shapes (origin at the well centre on the counter surface)."""
    x0, x1, y0, y1 = BOARD[0] - D[0], BOARD[1] - D[0], BOARD[2] - D[1], BOARD[3] - D[1]
    h = HOLE
    out = [
        _b(x0, x1, y0, -h, -B_T, 0.0, COUNTER_C),  # the counter round the cut-out
        _b(x0, x1, h, y1, -B_T, 0.0, COUNTER_C),
        _b(x0, -h, -h, h, -B_T, 0.0, COUNTER_C),
        _b(h, x1, -h, h, -B_T, 0.0, COUNTER_C),
        _b(-h, h, -h, h, -B_T, -WELL_D, WELL_C),  # the detacher's body under the well floor
    ]
    # the detacher's top, round its 28 mm well: 16 boxes whose inner faces make a 16-sided bore
    n, r_in, r_out = 16, WELL_R, 0.058
    w = 2 * r_out * math.tan(math.pi / n)
    for i in range(n):
        a = 2 * math.pi * i / n
        c = (r_in + r_out) / 2
        q = (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))
        out.append(parts.box((r_out - r_in, w, WELL_D), at=(c * math.cos(a), c * math.sin(a), -WELL_D / 2),
                             color=PUCK_C, quat=q, collide=True))
    # looks only: the round dark top, its bright rim and the well's floor
    out += [
        parts.cylinder(PUCK_R, 0.0006, at=(0.0, 0.0, 0.0003), color=PUCK_C, collide=False),
        parts.cylinder(PUCK_R + 0.002, 0.0004, at=(0.0, 0.0, 0.0002), color=RING_C, collide=False),
        parts.cylinder(WELL_R + 0.002, 0.0008, at=(0.0, 0.0, 0.0004), color=RING_C, collide=False),
        parts.cylinder(WELL_R, 0.0005, at=(0.0, 0.0, -WELL_D + 0.00025), color=WELL_C, collide=False),
    ]
    # the cut-out corners outside the round top look like counter
    return out


COUNTER = parts.Part("counter", pos=(D[0], D[1], B), shapes=_counter_shapes(), friction=(0.5, 0.45))

# --- the hard tag (tag frame: origin on the pin's axis at the body's underside, y along the tag toward its free end,
# z up through the pin) -------------------------------------------------------------------------------------------
TAG_W, TAG_Y0, TAG_Y1, TAG_H = 0.042, -0.014, 0.060, 0.020  # 42 x 74 x 20 mm body
DOME_R, DOME_H = 0.010, 0.006  # lock dome under the pin: 20 mm across, 6 mm proud
CH = 0.0025  # half-width of the square pin channel (5 mm)
FLOOR = 0.0035  # solid floor under the channel
TAG_MASS = 0.030
TAG_C, DOME_C, STEEL = (0.55, 0.57, 0.60), (0.42, 0.44, 0.47), (0.70, 0.71, 0.73)
PLATE_Z = (0.0071, 0.0083)  # the clutch plate, 1.2 mm steel
SLOT = 0.00085  # half-width of the plate's narrow slot (the groove's core is 0.6 mm in radius)
WIDE = 0.0032  # half-width of the plate's wide hole (the shaft is 1.5 mm in radius, with 1 mm of play in its channel)
TRAVEL = 0.006  # the magnet draws the plate 6 mm: the shaft then stands in the wide hole
K_PLATE, REST, PULLED = 150.0, -0.003, 0.012  # N/m; spring rest point when shut and when the magnet pulls


def _tag_shapes():
    w = TAG_W / 2
    return [
        _b(-w, w, TAG_Y0, TAG_Y1, 0.0, FLOOR, TAG_C),  # floor
        _b(-w, w, TAG_Y0, -CH, FLOOR, TAG_H, TAG_C),  # walls round the 5 mm pin channel
        _b(-w, w, CH, TAG_Y1, FLOOR, TAG_H, TAG_C),
        _b(-w, -CH, -CH, CH, FLOOR, TAG_H, TAG_C),
        _b(CH, w, -CH, CH, FLOOR, TAG_H, TAG_C),
        parts.cylinder(DOME_R, DOME_H, at=(0.0, 0.0, -DOME_H / 2), color=DOME_C),  # lock dome
        # looks only: a shallow ring moulded round the pin hole, and the label recess at the free end
        parts.cylinder(0.006, 0.0005, at=(0.0, 0.0, TAG_H + 0.00025), color=DOME_C, collide=False),
        _b(-0.015, 0.015, 0.024, 0.052, TAG_H, TAG_H + 0.0004, (0.48, 0.50, 0.53), collide=False),
    ]


def _plate_shapes():
    z0, z1 = PLATE_Z
    s, wd = SLOT, WIDE
    return (
        _b(s, 0.007, -0.002, 0.004, z0, z1, STEEL),  # narrow slot: its two sides and its end
        _b(-0.007, -s, -0.002, 0.004, z0, z1, STEEL),
        _b(-s, s, s, 0.004, z0, z1, STEEL),
        _b(wd, 0.007, -0.010, -0.002, z0, z1, STEEL),  # wide hole: its two sides and its far end
        _b(-0.007, -wd, -0.010, -0.002, z0, z1, STEEL),
        _b(-0.007, 0.007, -0.012, -0.010, z0, z1, STEEL),
    )


# Start: the tag lies dome-down on the counter, resting on the dome's rim and its free end's bottom edge, so it leans
# 6.84 degrees (tan = 6/50) with the free end down.
TILT = math.atan2(DOME_H, TAG_Y1 - DOME_R)
P0 = (0.27, -0.085)  # the pin's axis at the start


def _qx(a):
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


START_Q = _qx(-TILT)  # turn about x: the free end (+y) down
START_Z = B + TAG_Y1 * math.sin(TILT) + 0.0005  # tag-frame origin: the free end 0.5 mm over the counter
CLUTCH = parts.Slider("clutch", shapes=_plate_shapes(), axis=(0.0, 1.0, 0.0), travel=TRAVEL, mass=0.006,
                      spring=K_PLATE, damping=0.3, target=REST, max_effort=20.0)
TAG = parts.Part("tag", pos=(P0[0], P0[1], START_Z), quat=START_Q, shapes=_tag_shapes(), joints=[CLUTCH],
                 mass=TAG_MASS, friction=(0.45, 0.40), contact_offset=0.0001)

# --- the jacket cuff and the tag pin (cuff frame: the hem hinge, on the pin's axis 1.5 mm above the tag's top) ----
HZ = TAG_H + 0.0015  # hinge height in the tag frame
NAVY, NAVY_D, PIN_C = (0.12, 0.16, 0.36), (0.09, 0.12, 0.27), (0.80, 0.81, 0.83)
CUFF_W = 0.080  # the flattened cuff, 80 mm across
BODY_Y = (-0.082, -0.022)  # the quilted cuff (two layers, 23 mm) behind the tag
HEM_Y = (-0.016, 0.008)  # the hem round the pin, single layer, 3 mm
PIN_R, CORE_R = 0.0015, 0.0006  # 3 mm grooved pin with a 1.2 mm core
PIN_TIP = 0.004 - HZ  # pin tip, 4 mm above the tag's floor: 17.5 mm below the hinge
GROOVE = (0.0065 - HZ, 0.0089 - HZ)  # the groove, 2.4 mm long, round the clutch plate
HEAD_R, HEAD_T = 0.0075, 0.0025  # 15 mm dome head
BODY_MASS, HEM_MASS = 0.080, 0.008


def _cuff_body_shapes():
    w = CUFF_W / 2
    return [
        _b(-w, w, BODY_Y[0], BODY_Y[1], -0.0215, 0.0015, NAVY),  # the cuff's two quilted layers
        _b(-w, w, BODY_Y[1], HEM_Y[0], -0.0015, 0.0015, NAVY),  # the hem's join to the cuff
        # looks only: quilting seams across the cuff
        *[_b(-w, w, y - 0.0008, y + 0.0008, 0.0015, 0.0019, NAVY_D, collide=False) for y in (-0.037, -0.052, -0.067)],
    ]


def _hem_shapes():
    w = CUFF_W / 2
    return (
        _b(-w, w, HEM_Y[0], HEM_Y[1], -0.0015, 0.0015, NAVY),  # hem
        parts.cylinder(HEAD_R, HEAD_T, at=(0.0, 0.0, 0.0015 + HEAD_T / 2), color=PIN_C),  # pin head
        parts.cylinder(0.0055, 0.0015, at=(0.0, 0.0, 0.0015 + HEAD_T + 0.00075), color=PIN_C),  # its dome
        parts.cylinder(PIN_R, 0.0015 - GROOVE[1], at=(0.0, 0.0, (0.0015 + GROOVE[1]) / 2), color=PIN_C),  # shaft
        parts.cylinder(CORE_R, GROOVE[1] - GROOVE[0], at=(0.0, 0.0, sum(GROOVE) / 2), color=PIN_C),  # groove
        parts.cylinder(PIN_R, GROOVE[0] - PIN_TIP, at=(0.0, 0.0, (GROOVE[0] + PIN_TIP) / 2), color=PIN_C),  # tip
    )


HEM = parts.Hinge("hem", shapes=_hem_shapes(), pivot=(0.0, 0.0, 0.0), axis=(1.0, 0.0, 0.0), lower=-0.7, upper=0.7,
                  mass=HEM_MASS, damping=0.002, max_effort=5.0)


def _tag_point(p):
    """World point of a tag-frame point at the start pose."""
    r = parts._quat_to_matrix(np.asarray(START_Q, float))
    return np.array([P0[0], P0[1], START_Z]) + r @ np.asarray(p, float)


CUFF0 = _tag_point((0.0, 0.0, HZ))
CUFF = parts.Part("cuff", pos=tuple(float(v) for v in CUFF0), quat=START_Q, shapes=_cuff_body_shapes(), joints=[HEM],
                  mass=BODY_MASS, friction=(0.6, 0.5), contact_offset=0.0001)
OBJECTS = [COUNTER, TAG, CUFF]

# --- scoring ------------------------------------------------------------------------------------------------------
DEPTH0 = TAG_H - (PIN_TIP + HZ)  # 16 mm of pin inside the tag at the start
SEAT_XY, SEAT_Z, SEAT_TILT = 0.0045, 0.0025, 8.0  # dome in the well: axis, height over the floor, lean
LATERAL = 0.003  # the pin tip counts as in the tag's channel within 3 mm of its axis
OUT_GAP = 0.0005  # "out": the tip at least 0.5 mm above the tag's top face
FULL_TOL = 0.0005
STEP1 = 0.005
CLEAR_TAG = 0.010  # set aside: the tip 10 mm or more from the tag's pin hole
CLEAR = 0.005  # let go: no hand point within 5 mm
STILL_S, STILL_MM = 1.0, 0.002
OFF = B - 0.03  # anything lower than this has left the counter


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def tilt_deg(q) -> float:
    return float(np.degrees(np.arccos(np.clip(_rot(q)[2, 2], -1.0, 1.0))))


def dome_bottom(tag_st) -> np.ndarray:
    return np.asarray(tag_st.pos, float) + _rot(tag_st.quat_xyzw) @ np.array([0.0, 0.0, -DOME_H])


def seated(tag_st) -> bool:
    """The lock dome down in the detacher's well, where the magnet reaches it."""
    p = dome_bottom(tag_st)
    return bool(math.hypot(p[0] - D[0], p[1] - D[1]) <= SEAT_XY + EPS
                and p[2] - (B - WELL_D) <= SEAT_Z + EPS and tilt_deg(tag_st.quat_xyzw) <= SEAT_TILT + EPS)


def pin_tip(cuff_st) -> np.ndarray:
    p, q = cuff_st.links["hem"]
    return np.asarray(p, float) + _rot(q) @ np.array([0.0, 0.0, PIN_TIP])


def tip_in_tag(tag_st, cuff_st) -> np.ndarray:
    return _rot(tag_st.quat_xyzw).T @ (pin_tip(cuff_st) - np.asarray(tag_st.pos, float))


def pin_depth(tag_st, cuff_st) -> float:
    """How far the pin's tip is inside the tag, below its top face (0 once it is out of the channel)."""
    t = tip_in_tag(tag_st, cuff_st)
    if math.hypot(t[0], t[1]) > LATERAL + EPS:
        return 0.0
    return float(max(0.0, TAG_H - t[2]))


def pin_out(tag_st, cuff_st) -> bool:
    t = tip_in_tag(tag_st, cuff_st)
    return bool(math.hypot(t[0], t[1]) > LATERAL + EPS or t[2] >= TAG_H + OUT_GAP - EPS)


def pin_clear(tag_st, cuff_st) -> bool:
    """The pin set aside: its tip 10 mm or more from the tag's pin hole (centre of the top face)."""
    hole = np.asarray(tag_st.pos, float) + _rot(tag_st.quat_xyzw) @ np.array([0.0, 0.0, TAG_H])
    return bool(np.linalg.norm(pin_tip(cuff_st) - hole) >= CLEAR_TAG - EPS)


def part_boxes(part, st):
    poses = {"base": (np.asarray(st.pos, float), _rot(st.quat_xyzw))}
    for name, (p, q) in st.links.items():
        poses[name] = (np.asarray(p, float), _rot(q))
    return part.world_boxes(poses)


def corners(part, st) -> np.ndarray:
    k = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    return np.concatenate([c + (k * (s / 2)) @ r.T for _, c, r, s in part_boxes(part, st)])


def gap_to(hand_pts, boxes) -> float:
    from yam_arena.analysis import box_signed_distance

    return float(min(box_signed_distance(hand_pts, c, r, s).min() for _, c, r, s in boxes))


# --- the trial record (in a module of its own so a second copy of this file shares it) ---------------------------
_REC: dict = sys.modules.setdefault("_hard_tag_record", type(sys)("_hard_tag_record")).__dict__.setdefault("rec", {})


def reset_record() -> None:
    _REC.clear()
    _REC.update(t=0.0, steps=0, known=True, reason="", failed="", drawn=0.0, credited=0.0, seated=False,
                ever_seated=False, out=False, clear=False, tag_free=False, cuff_free=False, still=False, hist=[],
                last_t=None)


def record(states, hand_pts, dt: float) -> None:
    """One control step. ``states`` maps object names to states; ``hand_pts`` holds sampled points of both grippers
    and all four fingertips (world). None for either fails the trial (no credit when the state is unknown)."""
    if not _REC:
        reset_record()
    try:
        if states is None or any(n not in states for n in ("tag", "cuff")):
            raise ValueError("object state unknown")
        if hand_pts is None or len(hand_pts) == 0 or not np.isfinite(hand_pts).all():
            raise ValueError("hand state unknown")
        tag, cuff = states["tag"], states["cuff"]
        if "hem" not in cuff.links:
            raise ValueError("pin state unknown")
        depth = pin_depth(tag, cuff)
        tc, cc = corners(TAG, tag), corners(CUFF, cuff)
        if not (math.isfinite(depth) and np.isfinite(tc).all() and np.isfinite(cc).all()):
            raise ValueError("state not finite")
        g_tag, g_cuff = gap_to(hand_pts, part_boxes(TAG, tag)), gap_to(hand_pts, part_boxes(CUFF, cuff))
    except (KeyError, ValueError, AttributeError, TypeError) as e:
        _REC["known"] = False
        _REC["reason"] = str(e)
        _REC["t"] += dt
        _REC["steps"] += 1
        return
    if tc[:, 2].min() < OFF or cc[:, 2].min() < OFF:
        _REC["failed"] = _REC["failed"] or "tag or cuff off the counter"
    drawn = DEPTH0 - depth
    step = drawn - _REC["drawn"] if _REC["steps"] > 0 else 0.0
    s = seated(tag)
    # Pin travel counts only on steps the dome sits in the well; pushing the pin back in always counts against it.
    _REC["credited"] = float(min(DEPTH0, max(0.0, _REC["credited"] + (step if s else min(step, 0.0)))))
    _REC["drawn"] = drawn
    _REC["seated"] = s
    _REC["ever_seated"] = _REC["ever_seated"] or s
    _REC["out"] = pin_out(tag, cuff)
    _REC["clear"] = pin_clear(tag, cuff)
    _REC["tag_free"] = bool(g_tag > CLEAR + EPS)
    _REC["cuff_free"] = bool(g_cuff > CLEAR + EPS)
    snap = np.concatenate([tc.ravel(), cc.ravel()])
    hist = _REC["hist"]
    hist.append((_REC["t"], snap))
    while hist and hist[0][0] < _REC["t"] - STILL_S - 1e-9:
        hist.pop(0)
    _REC["still"] = bool(hist[0][0] <= _REC["t"] - STILL_S + 1e-9 + dt
                         and max(float(np.abs(h - snap).max()) for _, h in hist) <= STILL_MM + EPS)
    _REC["t"] += dt
    _REC["steps"] += 1


def _ok() -> bool:
    return bool(_REC and _REC["known"] and _REC["steps"] > 0 and not _REC["failed"])


def tier() -> float:
    if not _ok() or _REC["credited"] < STEP1 - EPS:
        return 0.0
    if not (_REC["out"] and _REC["credited"] >= DEPTH0 - FULL_TOL - EPS):
        return 0.25
    if not (_REC["clear"] and _REC["cuff_free"]):
        return 0.5
    if not (_REC["tag_free"] and _REC["still"]):
        return 0.75
    return 1.0


def success(objects):
    return tier() >= 1.0


def score(objects):
    """0.25: the pin drawn 5 mm or more while the tag's dome sat in the detacher's well. 0.5: the pin all the way out
    of the tag, all of it drawn while the dome sat in the well. 0.75: also the cuff let go with the pin's tip 10 mm or
    more from the tag's pin hole. 1.0: also no hand near the tag, and tag and cuff still for 1 s. 0 for the whole
    trial if the tag or cuff leaves the counter or the hand or object state is unknown at any step."""
    return tier()


reset_record()

# --- simulator hooks ---------------------------------------------------------------------------------------------
HAND_LINKS = ("tip_left", "tip_right", "gripper")
SAMPLE = 0.003
_HAND_PTS: dict = {}


def hand_points(link: str) -> np.ndarray:
    """Points on a hand link's collision surfaces (link frame): every vertex plus each triangle face sampled about
    every 3 mm, so a flat face resting on a part is seen even when no vertex is near it."""
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


_MAG: dict = {"on": None}


def _magnet(sim, on: bool) -> None:
    """The detacher's magnet: while the dome sits in the well it draws the steel clutch plate back (its spring's rest
    point moves past the plate's travel); once the dome leaves, the plate's spring pushes it home again."""
    if _MAG["on"] is on:
        return
    sim.set_target("tag", "clutch", PULLED if on else REST)
    _MAG["on"] = on


@parts.on_reset
def _on_reset(sim) -> None:
    reset_record()
    _MAG["on"] = None
    _magnet(sim, False)
    _REC["last_t"] = sim.t


@parts.on_step
def _on_step(sim) -> None:
    if _REC.get("last_t") == sim.t:  # a second copy of this module sharing the record: step it once
        return
    dt = sim.t - (_REC["last_t"] if _REC.get("last_t") is not None else 0.0)
    _REC["last_t"] = sim.t
    try:
        states = sim.states()
    except Exception:  # noqa: BLE001  unknown state: no credit
        states = None
    try:
        pts = _sim_hand_points(sim)
    except Exception:  # noqa: BLE001  unknown hand state: no credit
        pts = None
    record(states, pts, max(dt, 0.0))
    try:
        _magnet(sim, bool(states is not None and seated(states["tag"])))
    except Exception:  # noqa: BLE001
        _magnet(sim, False)
    if os.environ.get("TRACE_OUT") and _REC["steps"] % 5 == 0:
        import json

        row = dict(t=round(_REC["t"], 3), known=_REC["known"], reason=_REC["reason"], failed=_REC["failed"],
                   seated=_REC["seated"], magnet=_MAG["on"], drawn_mm=round(_REC["drawn"] * 1000, 2),
                   credited_mm=round(_REC["credited"] * 1000, 2), out=_REC["out"], clear=_REC["clear"],
                   tag_free=_REC["tag_free"], cuff_free=_REC["cuff_free"], still=_REC["still"], score=tier())
        try:
            tg, cf = states["tag"], states["cuff"]
            row["tag"] = dict(pos=[round(float(v), 4) for v in tg.pos], tilt=round(tilt_deg(tg.quat_xyzw), 2),
                              clutch_mm=round(float(tg.joints["clutch"]) * 1000, 2),
                              dome=[round(float(v), 4) for v in dome_bottom(tg)])
            row["cuff"] = dict(pos=[round(float(v), 4) for v in cf.pos], tilt=round(tilt_deg(cf.quat_xyzw), 2),
                               hem_deg=round(math.degrees(float(cf.joints["hem"])), 2),
                               tip=[round(float(v), 4) for v in pin_tip(cf)],
                               tip_in_tag=[round(float(v) * 1000, 2) for v in tip_in_tag(tg, cf)])
        except Exception:  # noqa: BLE001
            pass
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("hardtag.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


# --- demo: fixed waypoints from the published layout, no state read ----------------------------------------------
# Left hand: take the tag by its free end, lift it with the cuff trailing, carry it over the detacher and set it down
# with its dome over the well, let go and back off. Right hand: take the cuff across its width behind the tag, lift
# it straight up so the pin comes up out of the tag, carry it aside, set it down on the counter and let go.
TIP = 0.013  # pad tips below the TCP
GRIP_Z = B + 0.0035 + TIP  # pad tips 3.5 mm above the counter
TAG_NEAR, TAG_PINCH = -(TAG_W / 2 + 0.012), -(TAG_W / 2 - 0.003)
CUFF_NEAR, CUFF_PINCH = -0.0469, -(CUFF_W / 2 - 0.003)
LIFT_TAG = 0.030
PULL = 0.035  # the cuff is lifted 35 mm: 16 mm of pin plus room
ASIDE = (-0.0, -0.070)  # where the cuff is carried from the detacher (dx, dy)
GRIP_ALONG = 0.040  # the tag is taken 40 mm from the pin, on its free end
PLACE_DY = -0.003  # the loaded left hand sets the tag down about 4 mm to the left of where it is sent (trace)
VARIANT = os.environ.get("DEMO_VARIANT", "")


def _w(t, x, y, z, finger, label, yaw):
    return Waypoint(t, (float(x), float(y), float(z)), finger, label, yaw=yaw)


def demo():
    yl, yr = -math.pi / 2, math.pi / 2  # both hands close their fingers along x
    g0 = _tag_point((0.0, GRIP_ALONG, TAG_H / 2))
    gx, gy0 = g0[0], g0[1]
    gy1 = D[1] + (gy0 - P0[1]) + PLACE_DY  # the same grip with the pin over the well
    left = [
        _w(2.0, gx, gy0, GRIP_Z + 0.05, TAG_NEAR, "over the tag's free end", yl),
        _w(1.5, gx, gy0, GRIP_Z, TAG_NEAR, "down round the tag", yl),
        _w(0.8, gx, gy0, GRIP_Z, TAG_PINCH, "take the tag", yl),
        _w(1.2, gx, gy0, GRIP_Z + LIFT_TAG, TAG_PINCH, "lift it, cuff trailing", yl),
        _w(2.5, gx, gy1, GRIP_Z + LIFT_TAG, TAG_PINCH, "carry it over the detacher", yl),
        _w(1.5, gx, gy1, GRIP_Z, TAG_PINCH, "set it down, dome over the well", yl),
        _w(0.8, gx, gy1, GRIP_Z, TAG_NEAR, "let go", yl),
        _w(1.0, gx, gy1, GRIP_Z + 0.06, TAG_NEAR, "up off the tag", yl),
    ]
    if VARIANT == "late_pull":  # take the tag back out of the well before the cuff is pulled
        left += [
            _w(1.2, gx, gy1, GRIP_Z, TAG_NEAR, "down round the tag again", yl),
            _w(0.8, gx, gy1, GRIP_Z, TAG_PINCH, "take it again", yl),
            _w(1.5, gx, gy1, GRIP_Z + 0.040, TAG_PINCH, "lift the dome out of the well and hold it", yl),
            _w(14.0, gx, gy1, GRIP_Z + 0.040, TAG_PINCH, "hold the tag up", yl),
        ]
    else:
        left.append(_w(1.5, 0.16, 0.26, B + 0.15, TAG_NEAR, "park left", yl))
    t_left = sum(w.duration for w in left)
    # where the cuff lies once the tag is in the well: flat on the counter behind the tag
    cx, cy = D[0], D[1] + (BODY_Y[0] + BODY_Y[1]) / 2
    cz = GRIP_Z
    if VARIANT == "late_pull":
        cz = GRIP_Z + 0.012  # the cuff's front end is held up with the tag
        t_wait = 12.8
    else:
        t_wait = t_left - 1.0
    ax, ay = cx + ASIDE[0], cy + ASIDE[1]
    right = [
        _w(2.0, 0.15, -0.25, B + 0.15, CUFF_NEAR, "wait to the right", yr),
        _w(max(0.5, t_wait - 2.0), 0.15, -0.25, B + 0.15, CUFF_NEAR, "wait for the tag to be set down", yr),
        _w(2.0, cx, cy, cz + 0.05, CUFF_NEAR, "over the cuff", yr),
        _w(1.5, cx, cy, cz, CUFF_NEAR, "down round the cuff", yr),
        _w(0.8, cx, cy, cz, CUFF_PINCH, "take the cuff across its width", yr),
        _w(2.5, cx, cy, cz + PULL, CUFF_PINCH, "lift it straight up: the pin comes out", yr),
        _w(2.0, ax, ay, cz + PULL, CUFF_PINCH, "carry it aside", yr),
        _w(1.5, ax, ay, GRIP_Z + 0.004, CUFF_PINCH, "set it down on the counter", yr),
        _w(0.8, ax, ay, GRIP_Z + 0.004, CUFF_NEAR, "let go", yr),
        _w(1.0, ax, ay, GRIP_Z + 0.06, CUFF_NEAR, "up off the cuff", yr),
        _w(1.5, 0.15, -0.25, B + 0.15, CUFF_NEAR, "park right", yr),
    ]
    return {"left": left, "right": right}
