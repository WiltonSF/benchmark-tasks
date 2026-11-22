"""On a kitchen counter a Samsung Galaxy S24 phone stands upside down in an aluminium desk phone stand, screen
toward the robot, so its USB-C charging port is on top. Four small balls of grey lint sit inside the port, wedged
beside the flat tongue in its middle, about 4.5 mm down. A clean, dry, soft toothbrush lies bristles-down on the
counter to the robot's right, and a folded white paper towel lies flat on the counter beyond it.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402  (before lots, so parts' reset hooks run before the lots' moves)
import lots  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
EPS = 1e-6
HZ = 20

GLASS, FRAME, BACK = (0.05, 0.06, 0.08), (0.42, 0.43, 0.45), (0.56, 0.57, 0.60)
PORT_DARK, TONGUE, ALU = (0.03, 0.03, 0.03), (0.15, 0.15, 0.16), (0.70, 0.71, 0.73)
HANDLE, NECK, BRISTLE, BLUE = (0.12, 0.50, 0.72), (0.14, 0.55, 0.78), (0.93, 0.94, 0.95), (0.20, 0.45, 0.85)
TOWEL, LINT = (0.97, 0.97, 0.95), (0.42, 0.45, 0.55)

# -- the phone: Samsung Galaxy S24 (SM-S921B), 147.0 x 70.6 x 7.6 mm, 167 g --------------------------------------
# Part frame: x through the thickness (screen at -x), y across the width, z along the length; +z is the end with
# the charging port, which is on top (the phone stands upside down).
P_LEN, P_WID, P_THK, P_MASS = 0.147, 0.0706, 0.0076, 0.167
LEAN = math.radians(12.0)  # the stand leans the phone back 12 degrees from vertical (its top toward +x)
R_PHONE = parts._rot((0.0, 1.0, 0.0), LEAN)
EDGE = np.array([0.300, -0.040, TOP + 0.007])  # centre of the phone's lower edge (its real top edge), on the ledge
GAP = 0.0003  # start clearance to the stand
PHONE_POS = EDGE + R_PHONE @ np.array([0.0, 0.0, P_LEN / 2 + GAP])
# USB-C receptacle (USB Type-C specification): opening 8.34 x 2.56 mm, tongue 6.69 x 0.70 mm. Depth 7.5 mm and the
# tongue's top 2.0 mm below the mouth are estimates for this phone.
PORT_V, PORT_U = 0.00834 / 2, 0.00256 / 2  # half opening across the width (v) and the thickness (u)
TONGUE_V, TONGUE_U = 0.00669 / 2, 0.00070 / 2
PORT_DEPTH, TONGUE_TOP = 0.0075, 0.0020
END = 0.014  # the 14 mm port end of the phone is drawn but has no collider (the bristles are a stated rule there)


def _phone_shapes():
    T, W, L = P_THK, P_WID, P_LEN
    z_end = L / 2 - END  # where the body's collider stops
    s = [parts.box((T, W, L - END), at=(0.0, 0.0, -END / 2), color=FRAME)]  # body collider (frame colour)
    # looks: screen glass, back glass, camera rings near the real top (now at -z), side keys
    s += [parts.box((0.0004, W - 0.0016, L - 0.0016), at=(-T / 2 - 0.0001, 0.0, 0.0), color=GLASS, collide=False),
          parts.box((0.0004, W - 0.0016, L - 0.0016), at=(T / 2 + 0.0001, 0.0, 0.0), color=BACK, collide=False)]
    for dz in (0.016, 0.029, 0.042):
        s.append(parts.cylinder(0.0055, 0.0016, at=(T / 2 + 0.0008, W / 2 - 0.013, -L / 2 + dz), color=GLASS,
                                axis="x", collide=False))
    s += [parts.box((0.002, 0.0006, 0.020), at=(0.0, -W / 2 - 0.0002, 0.025), color=FRAME, collide=False),
          parts.box((0.002, 0.0006, 0.010), at=(0.0, -W / 2 - 0.0002, -0.002), color=FRAME, collide=False)]
    # the port end (drawn only): the frame either side of the opening, in front of and behind it, the cavity floor and
    # the tongue
    side = W / 2 - PORT_V
    zc = z_end + END / 2
    s += [parts.box((T, side, END), at=(0.0, sgn * (PORT_V + side / 2), zc), color=FRAME, collide=False)
          for sgn in (-1, 1)]
    wall = T / 2 - PORT_U
    s += [parts.box((wall, 2 * PORT_V, END), at=(sgn * (PORT_U + wall / 2), 0.0, zc), color=FRAME, collide=False)
          for sgn in (-1, 1)]
    floor = END - PORT_DEPTH
    s.append(parts.box((2 * PORT_U, 2 * PORT_V, floor), at=(0.0, 0.0, z_end + floor / 2), color=PORT_DARK,
                       collide=False))
    tl = PORT_DEPTH - TONGUE_TOP
    s.append(parts.box((2 * TONGUE_U, 2 * TONGUE_V, tl), at=(0.0, 0.0, L / 2 - PORT_DEPTH + tl / 2), color=TONGUE,
                       collide=False))
    return s


PHONE = parts.Part("phone", pos=tuple(float(v) for v in PHONE_POS), quat=tuple(parts._matrix_to_quat(R_PHONE)),
                   mass=P_MASS, shapes=_phone_shapes(), friction=(0.6, 0.5))

# -- the stand: an aluminium desk phone stand (about 76 x 76 mm base), fixed on the counter ------------------------
STAND_POS = np.array([0.318, -0.040, TOP + 0.0015])
Q_LEAN = tuple(parts._matrix_to_quat(R_PHONE))


def _stand_shapes():
    def at_phone(local):  # a point given in the phone's frame (relative to the phone centre) -> stand frame
        return tuple(float(v) for v in PHONE_POS - STAND_POS + R_PHONE @ np.asarray(local, float))

    h = P_LEN / 2 + GAP
    s = [parts.box((0.076, 0.076, 0.003), at=(0.0, 0.0, 0.0), color=ALU)]
    # ledge under the phone's lower edge, front lip before the screen, back rest behind it: all leaning with it
    s.append(parts.box((0.016, 0.050, 0.004), at=at_phone((0.0012, 0.0, -h - 0.002)), color=ALU, quat=Q_LEAN))
    s.append(parts.box((0.002, 0.050, 0.008), at=at_phone((-P_THK / 2 - 0.0006 - 0.001, 0.0, -h + 0.004)),
                       color=ALU, quat=Q_LEAN))
    s.append(parts.box((0.004, 0.060, 0.085), at=at_phone((P_THK / 2 + GAP + 0.002, 0.0, -h - 0.004 + 0.0425)),
                       color=ALU, quat=Q_LEAN))
    # strut from the back of the base to the back of the rest
    a = np.array([0.036, 0.0, 0.0015])
    b = np.asarray(at_phone((P_THK / 2 + GAP + 0.004, 0.0, -0.030)))
    d = b - a
    ang = math.atan2(d[0], d[2])
    s.append(parts.box((0.004, 0.030, float(np.linalg.norm(d))), at=tuple((a + b) / 2), color=ALU,
                       quat=tuple(parts._matrix_to_quat(parts._rot((0.0, 1.0, 0.0), ang)))))
    return s


STAND = parts.Part("phone_stand", pos=tuple(float(v) for v in STAND_POS), mass=None, shapes=_stand_shapes())

# -- the toothbrush: a soft manual toothbrush (Oral-B Indicator Contour Clean 35 Soft class), 190 mm, 16 g -------
# Part frame: origin on the handle's centre line where the robot holds it, y along the handle toward the head, z up
# (bristles toward -z). Sizes are estimates (see README).
HANDLE_W, HANDLE_T = 0.013, 0.009
TUFT = (0.010, 0.025, 0.011)  # bristle field: across, along, bristle height
HEAD_Y = 0.085  # centre of the head from the hold point
TIP_Z = -0.017  # bristle tips below the handle's centre line


def _brush_shapes():
    return [parts.box((HANDLE_W, 0.130, HANDLE_T), at=(0.0, -0.026, 0.0), color=HANDLE),
            parts.box((0.007, 0.032, 0.005), at=(0.0, 0.055, -0.002), color=NECK),
            parts.box((0.011, 0.028, 0.005), at=(0.0, HEAD_Y, -0.0035), color=NECK),
            parts.box(TUFT, at=(0.0, HEAD_Y, TIP_Z + TUFT[2] / 2), color=BRISTLE),
            parts.box((0.0102, 0.008, 0.004), at=(0.0, HEAD_Y - 0.006, -0.008), color=BLUE, collide=False)]


# Lying bristles-down it rests on the handle's far end and the near edge of the bristle field: tilted about x.
_END = np.array([0.0, -0.091, -HANDLE_T / 2])
_NEAR = np.array([0.0, HEAD_Y - TUFT[1] / 2, TIP_Z])
REST_TILT = math.atan2(_END[2] - _NEAR[2], _NEAR[1] - _END[1])
R_BRUSH = parts._rot((1.0, 0.0, 0.0), REST_TILT)
REST_H = -float((R_BRUSH @ _END)[2])  # hold point height above the counter at rest
BRUSH_A = np.array([0.220, -0.220, TOP + REST_H])
BRUSH = parts.Part("toothbrush", pos=tuple(float(v) for v in BRUSH_A), quat=tuple(parts._matrix_to_quat(R_BRUSH)),
                   mass=0.016, shapes=_brush_shapes(), friction=(0.8, 0.6))

# -- the paper towel: one Bounty sheet folded in quarters, about 140 x 130 mm, lying flat (fixed) -----------------
TOWEL_SIZE = (0.140, 0.130, 0.0015)
TOWEL_C = np.array([0.420, -0.200, TOP + TOWEL_SIZE[2] / 2])
TOWEL_TOP = TOP + TOWEL_SIZE[2]
TOWEL_PART = parts.Part("paper_towel", pos=tuple(float(v) for v in TOWEL_C), mass=None,
                        shapes=[parts.box(TOWEL_SIZE, color=TOWEL)])

OBJECTS = [PHONE, STAND, BRUSH, TOWEL_PART]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A"),  # the screenshot's layout
    lots.Lot("B", place={"toothbrush": (0.255, -0.265, TOP + REST_H)}),
)

# -- the lint (a stated rule; README "Lint rule") ------------------------------------------------------------------
# Port coordinates: u through the thickness (+u toward the back glass), v across the width, d depth below the mouth.
LINT_UVD = np.array([(su * 0.0009, sv * 0.0022, 0.0045) for su in (-1, 1) for sv in (-1, 1)])
N_LINT = len(LINT_UVD)
REACH = 0.0005  # bristle tips within 0.5 mm above a tuft's depth (or deeper) reach it
V_WINDOW = 0.0015  # ... and within 1.5 mm of it across the port's width, on its side of the tongue
CATCH_TRAVEL = 0.002  # sideways travel (in the port's cross-section) while reaching it, to hook it out
STEP_MAX = 0.003  # a larger jump in one control step counts no travel
PACK_DEPTH = 0.0065  # bristle tips deeper than this in the opening push lint against the back wall (any time)
OUT_D = -0.0005  # a carried tuft above this depth (0.5 mm above the mouth) is out of the port
BACK_IN_D = 0.0005  # a carried tuft that is out and comes back below this depth is wiped back into the port
TOWEL_GAP = 0.0025  # a carried tuft this close above the towel's top, over it, is wiped off ...
TOWEL_WIPE = 0.010  # ... once it has slid this far over the towel
HAND_CLEAR = 0.030
HOLD_STEPS = int(SUCCESS_HOLD_S * HZ)
PHONE_MOVE, PHONE_TURN = 0.006, math.radians(4.0)

# bristle tips: a 0.5 mm grid over the bristle field's ends, in the brush frame
_gx = np.arange(-TUFT[0] / 2, TUFT[0] / 2 + 1e-9, 0.0005)
_gy = np.arange(HEAD_Y - TUFT[1] / 2, HEAD_Y + TUFT[1] / 2 + 1e-9, 0.0005)
TIPS = np.array([(x, y, TIP_Z) for x in _gx for y in _gy])
HARD = [s for s in BRUSH.shapes if s.collide and s.at[2] - s.half[2] > TIP_Z + TUFT[2] - 1e-9]  # handle, neck, head

_S = parts.__dict__.setdefault("_usbc_lint_state", {})


def _box_surface(c, r, size, step=0.004):
    h = np.asarray(size) / 2
    g = [np.linspace(-h[i], h[i], max(2, int(math.ceil(2 * h[i] / step)) + 1)) for i in range(3)]
    pts = np.array([[a, b, d] for a in g[0] for b in g[1] for d in g[2]
                    if abs(abs(a) - h[0]) < 1e-9 or abs(abs(b) - h[1]) < 1e-9 or abs(abs(d) - h[2]) < 1e-9])
    return c + pts @ r.T


_HARD_PTS = np.concatenate([_box_surface(np.asarray(s.at), s.R, 2 * s.half, 0.002) for s in HARD])


def port_end_breached(points, phone_pose) -> bool:
    """Any of ``points`` in the phone's port end or within 1 mm of its top (touching it)."""
    p, r = phone_pose
    loc = (np.asarray(points, float) - p) @ r
    return bool(np.any((np.abs(loc[:, 0]) <= P_THK / 2 + 0.0005) & (np.abs(loc[:, 1]) <= P_WID / 2 + 0.0005)
                       & (loc[:, 2] >= P_LEN / 2 - END) & (loc[:, 2] <= P_LEN / 2 + 0.001)))


def reset_state():
    _S.clear()
    _S.update(mode=np.zeros(N_LINT, int), travel=np.zeros(N_LINT), out=np.zeros(N_LINT, bool), wiped=np.zeros(N_LINT),
              last_w=np.full((N_LINT, 3), np.nan),
              ride=np.zeros((N_LINT, 3)), world=np.zeros((N_LINT, 3)), reached_prev=np.zeros(N_LINT, bool),
              prev_tips=None, known=False, broken=False, packed=False, hard_in_port=False, knocked=False,
              dropped=False, clear=False, supported=False, hold=0, deepest=0.0, log_t=-1.0, views=None)


if not _S:
    reset_state()


def _pose(st):
    return np.asarray(st.pos, float), parts._quat_to_matrix(st.quat_xyzw)


def port_frame(phone_pose):
    """Mouth centre and the unit axes (u, v, a: a points out of the port) in the world."""
    p, r = phone_pose
    return p + r @ np.array([0.0, 0.0, P_LEN / 2]), r[:, 0], r[:, 1], r[:, 2]


def to_port(points, phone_pose):
    m, u, v, a = port_frame(phone_pose)
    rel = np.asarray(points, float) - m
    return np.stack([rel @ u, rel @ v, -(rel @ a)], axis=-1)


def from_port(uvd, phone_pose):
    m, u, v, a = port_frame(phone_pose)
    uvd = np.asarray(uvd, float)
    return m + uvd[..., :1] * u + uvd[..., 1:2] * v - uvd[..., 2:3] * a


def in_cavity(uvd):
    uvd = np.asarray(uvd, float)
    return (np.abs(uvd[..., 0]) < PORT_U) & (np.abs(uvd[..., 1]) < PORT_V) & (uvd[..., 2] > 0.0) & (
        uvd[..., 2] < PORT_DEPTH)


def lint_world(phone_pose, brush_pose):
    """Where each tuft is now (world): in the port, on the bristles or on the towel."""
    out = np.zeros((N_LINT, 3))
    bp, br = brush_pose
    for i in range(N_LINT):
        if _S["mode"][i] == 0:
            out[i] = from_port(LINT_UVD[i], phone_pose)
        elif _S["mode"][i] == 1:
            out[i] = bp + br @ _S["ride"][i]
        else:
            out[i] = _S["world"][i]
    return out


def record(phone_pose, brush_pose):
    """One control step of the lint rule. A tuft still in the port is hooked onto the bristles once bristle tips
    inside the opening, on its side of the tongue and within 1.5 mm of it across the port, are at least as deep as
    its top (4.0 mm) on this step and the last, and the brush has moved at least 2 mm sideways in the port's
    cross-section in such steps. Bristle tips more than 6.5 mm into the opening at any time push lint against the
    back wall: score 0 for the trial. A hooked tuft rides on its bristle; dragged into one of the slot's end walls
    (beyond the opening's 8.34 mm length) below the mouth it is scraped off and stays in the port; carried back into the port after it was out, it is left
    there again; it stays on the towel once it has slid 10 mm over it within 2.5 mm of its top."""
    bp, br = brush_pose
    if port_end_breached(bp + _HARD_PTS @ br.T, phone_pose):  # the brush's rigid head, neck or handle
        _S["hard_in_port"] = True
    tips_w = bp + TIPS @ br.T
    uvd = to_port(tips_w, phone_pose)
    inside = (np.abs(uvd[:, 0]) < PORT_U) & (np.abs(uvd[:, 1]) < PORT_V) & (uvd[:, 2] > 0.0)
    _S["deepest"] = float(uvd[inside, 2].max()) if inside.any() else 0.0
    side = np.sign(uvd[:, 0])
    gap = inside & (np.abs(uvd[:, 0]) > TONGUE_U)  # tips in the two gaps beside the tongue
    if np.any(inside & (uvd[:, 2] > PACK_DEPTH + EPS)):
        _S["packed"] = True
    prev = _S["prev_tips"]
    _S["prev_tips"] = uvd
    step = 0.0
    if prev is not None:
        step = float(np.linalg.norm((uvd - prev)[:, :2], axis=1).mean())
    for i in range(N_LINT):
        if _S["mode"][i] != 0:
            continue
        u0, v0, d0 = LINT_UVD[i]
        reached = bool(np.any(gap & (side == np.sign(u0)) & (np.abs(uvd[:, 1] - v0) <= V_WINDOW + EPS)
                              & (uvd[:, 2] >= d0 - REACH - EPS)))
        if reached and _S["reached_prev"][i] and step <= STEP_MAX + EPS:
            _S["travel"][i] += step
        _S["reached_prev"][i] = reached
        if reached and _S["travel"][i] >= CATCH_TRAVEL - EPS:
            pick = np.flatnonzero(gap & (side == np.sign(u0)))
            k = pick[np.argmin(np.abs(uvd[pick, 0] - u0) + np.abs(uvd[pick, 1] - v0) + np.abs(uvd[pick, 2] - d0))]
            _S["mode"][i], _S["ride"][i], _S["out"][i] = 1, TIPS[k], False
    for i in range(N_LINT):  # carried tufts: out of the port, back into it, or onto the towel
        if _S["mode"][i] != 1:
            continue
        w = bp + br @ _S["ride"][i]
        d = to_port(w, phone_pose)
        last = _S["last_w"][i].copy()
        if d[2] < OUT_D:
            _S["out"][i] = True
        elif (not _S["out"][i] and d[2] > 0.0 and abs(d[1]) >= PORT_V) or (_S["out"][i] and in_cavity(d)
                                                                            and d[2] > BACK_IN_D):
            # dragged into one of the slot's end walls below the mouth (scraped off), or carried back in after it
            # was out: left in the port again
            _S["mode"][i], _S["travel"][i], _S["out"][i], _S["reached_prev"][i] = 0, 0.0, False, False
            _S["wiped"][i], _S["last_w"][i] = 0.0, np.nan
            continue
        near = _S["out"][i] and on_towel_xy(w) and TOWEL_TOP - 0.001 <= w[2] <= TOWEL_TOP + TOWEL_GAP + EPS
        if near:  # sliding over the towel, counted between consecutive steps that both touch it
            slide = float(np.linalg.norm((w - last)[:2])) if np.all(np.isfinite(last)) else 0.0
            _S["wiped"][i] += slide if slide <= STEP_MAX + EPS else 0.0
            _S["last_w"][i] = w
        else:
            _S["wiped"][i], _S["last_w"][i] = 0.0, np.nan
        if near and _S["wiped"][i] >= TOWEL_WIPE - EPS:
            _S["mode"][i], _S["world"][i] = 2, np.array([w[0], w[1], TOWEL_TOP + 0.0006])


def on_towel_xy(p) -> bool:
    return bool(abs(p[0] - TOWEL_C[0]) <= TOWEL_SIZE[0] / 2 + EPS and abs(p[1] - TOWEL_C[1]) <= TOWEL_SIZE[1] / 2 + EPS)


def remaining(phone_pose, brush_pose) -> int:
    """Tufts in the port now: never hooked, or hooked but still inside the opening."""
    w = lint_world(phone_pose, brush_pose)
    return int(sum(1 for i in range(N_LINT) if _S["mode"][i] == 0 or (_S["mode"][i] == 1 and in_cavity(
        to_port(w[i], phone_pose)))))


def on_towel() -> int:
    return int(np.sum(_S["mode"] == 2))


# -- end-state geometry --------------------------------------------------------------------------------------------
_BRUSH_PTS = np.concatenate([_box_surface(np.asarray(s.at), s.R, 2 * s.half) for s in BRUSH.shapes if s.collide])
def brush_points(brush_state):
    p, r = _pose(brush_state)
    return p + _BRUSH_PTS @ r.T


def phone_boxes(phone_state):
    p, r = _pose(phone_state)
    return [(c, rr, s) for _, c, rr, s in PHONE.world_boxes({"base": (p, r)})]


def stand_boxes():
    return [(c, rr, s) for _, c, rr, s in STAND.world_boxes({"base": (STAND_POS, np.eye(3))})]


def brush_set_down(objects) -> bool:
    """Lying on the counter or the towel (its lowest point within 4.5 mm of the counter top) and at least 5 mm from
    the phone and its stand."""
    from yam_arena import analysis

    pts = brush_points(objects["toothbrush"])
    if float(pts[:, 2].min()) > TOP + 0.0045 + EPS:
        return False
    boxes = phone_boxes(objects["phone"]) + stand_boxes()
    return all(float(analysis.box_signed_distance(pts, c, r, s).min()) >= 0.005 - EPS for c, r, s in boxes)


def phone_in_place(phone_state) -> bool:
    p, r = _pose(phone_state)
    turn = math.acos(float(np.clip((np.trace(R_PHONE.T @ r) - 1) / 2, -1.0, 1.0)))
    return bool(np.linalg.norm(p - PHONE_POS) <= PHONE_MOVE + EPS and turn <= PHONE_TURN + EPS)


# -- scoring ---------------------------------------------------------------------------------------------------------
def _ruled_out() -> bool:
    return bool(not _S["known"] or _S["broken"] or _S["packed"] or _S["hard_in_port"] or _S["knocked"]
                or _S["dropped"])


def score(objects):
    """Lint out of the port, judged once the toothbrush is set down, both hands are 30 mm clear of the phone and
    the toothbrush, and everything has been still for 1 s."""
    try:
        if _ruled_out() or not _S["clear"] or _S["hold"] < HOLD_STEPS:
            return 0.0
        ph, br = objects["phone"], objects["toothbrush"]
        if not (ph.resting() and br.resting() and phone_in_place(ph) and brush_set_down(objects)):
            return 0.0
        left = remaining(_pose(ph), _pose(br))
        if left == 0 and on_towel() == N_LINT:
            return 1.0
        if left == 0:
            return 0.75
        if left <= 2:
            return 0.5
        if left < N_LINT:
            return 0.25
        return 0.0
    except (KeyError, ValueError, TypeError, IndexError):
        return 0.0


def success(objects):
    return score(objects) == 1.0


# -- simulator side ---------------------------------------------------------------------------------------------------
HAND_LINKS = ("gripper", "tip_left", "tip_right")
_FACE_PTS: dict[str, np.ndarray] = {}
_LINT_OPS: list = []


def _hull_surface(h, spacing=0.003):
    from scipy.spatial import ConvexHull

    v = h.vertices
    out = [v]
    for tri in ConvexHull(v).simplices:
        a, b, c = v[tri]
        n = max(1, int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / spacing)))
        out.append(np.array([a + (b - a) * x / n + (c - a) * y / n for x in range(n + 1) for y in range(n + 1 - x)]))
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
            if n not in HAND_LINKS:
                continue
            if n not in _FACE_PTS:
                _FACE_PTS[n] = np.concatenate([_hull_surface(h) for h in analysis.link_hulls().get(n, [])])
            pts.append(_FACE_PTS[n] @ parts._quat_to_matrix(poses[i, 3:]).T + (poses[i, :3] - origin))
    if len(pts) != 6:
        raise RuntimeError("hand links not found")
    return np.concatenate(pts)


def hands_clear(points, objects) -> bool:
    from yam_arena import analysis

    p, r = _pose(objects["toothbrush"])
    boxes = phone_boxes(objects["phone"]) + [(c, rr, s) for _, c, rr, s in BRUSH.world_boxes({"base": (p, r)})]
    return all(float(analysis.box_signed_distance(points, c, rr, s).min()) >= HAND_CLEAR - EPS for c, rr, s in boxes)


def _draw(sim, pts):
    """Show the four tufts where the rule has them (small grey spheres, no collider)."""
    from isaaclab.sim.utils import get_current_stage
    from pxr import Gf, UsdGeom

    if not _LINT_OPS:
        stage = get_current_stage()
        env = parts._PATHS["phone"].rsplit("/", 1)[0]
        for i in range(N_LINT):
            g = UsdGeom.Sphere.Define(stage, f"{env}/lint_{i}")
            g.CreateRadiusAttr(0.0011)
            g.CreateDisplayColorAttr().Set([Gf.Vec3f(*LINT)])
            _LINT_OPS.append(g.AddTranslateOp())
    for op, q in zip(_LINT_OPS, pts):
        op.Set(Gf.Vec3d(*(float(v) for v in q)))


@parts.on_reset
def _on_reset(sim) -> None:
    views = _S.get("views")
    reset_state()
    _S["views"] = views


@parts.on_step
def _on_step(sim) -> None:
    try:
        base = sim.env.unwrapped
        st = sim.states()
        ph, br = st["phone"], st["toothbrush"]
        pp, bp = _pose(ph), _pose(br)
        record(pp, bp)
        hands = hand_points(base)
        if port_end_breached(hands, pp):  # a robot finger or gripper in the port end
            _S["hard_in_port"] = True
        if not phone_in_place(ph):
            _S["knocked"] = True
        if float(br.low[2]) < TOP - 0.02 or float(ph.low[2]) < TOP - 0.02:
            _S["dropped"] = True
        _S["clear"] = hands_clear(hands, st)
        _S["supported"] = brush_set_down(st)
        _S["known"] = True
        still = ph.resting() and br.resting()
        _S["hold"] = _S["hold"] + 1 if (_S["clear"] and _S["supported"] and still) else 0
        try:
            _draw(sim, lint_world(pp, bp))
        except Exception as exc:  # noqa: BLE001  (a look only: no effect on the score)
            if _S["log_t"] < 0:
                print(f"[lint] could not draw the lint: {exc!r}", file=sys.stderr, flush=True)
        if sim.t - _S["log_t"] >= 1.0 - 1e-6:
            _S["log_t"] = sim.t
            print("[lint] " + json.dumps(dict(
                t=round(float(sim.t), 2), mode=[int(m) for m in _S["mode"]],
                wiped=[round(float(x) * 1000, 1) for x in _S["wiped"]],
                travel=[round(float(x) * 1000, 2) for x in _S["travel"]], deepest=round(_S["deepest"] * 1000, 2),
                left=remaining(pp, bp), towel=on_towel(), packed=_S["packed"], hard=_S["hard_in_port"],
                knocked=_S["knocked"], dropped=_S["dropped"], clear=_S["clear"], supported=_S["supported"],
                hold=_S["hold"], score=score(st), phone=[round(float(x), 4) for x in ph.pos],
                brush=[round(float(x), 4) for x in br.pos], brush_q=[round(float(x), 4) for x in br.quat_xyzw])),
                flush=True)
    except Exception as exc:  # unreadable poses or hands: no credit for this trial
        _S["broken"] = True
        print(f"[lint] step check failed: {exc!r}", file=sys.stderr, flush=True)


# -- the scripted demo: the right arm, open loop from the visible layout ---------------------------------------------
GRIP = -0.003  # finger position: 6 mm between the pads on the 13 mm handle (a firm hold)
TCP_ABOVE = 0.0046  # TCP above the handle's centre line when holding it (pad tips 3 mm above the counter)
DEPTH = 0.0046  # bristle-field centre this far down the port's axis
SWEEP = 0.0010  # one stroke along the opening from 1 mm one side of centre to 1 mm the other, then back
CARRY_Z = TOP + 0.185
YAW = math.pi / 2  # fingers close along world x, across the handle


def _wp(dur, pos, finger, label):
    return Waypoint(float(dur), tuple(float(v) for v in pos), float(finger), label, YAW)


def demo():
    lot = lots.current()
    hold = np.asarray(lot.pos("toothbrush"), float)  # where the handle's hold point lies, as the camera sees it
    grasp = hold + [0.0, 0.0, TCP_ABOVE]
    tip_c = R_BRUSH @ np.array([0.0, HEAD_Y, TIP_Z]) - [0.0, 0.0, TCP_ABOVE]  # bristle-field centre from the TCP
    low = R_BRUSH @ np.array([0.0, HEAD_Y - TUFT[1] / 2, TIP_Z]) - [0.0, 0.0, TCP_ABOVE]  # its lowest edge
    m, u, v, a = port_frame((PHONE_POS - R_PHONE @ [0.0, 0.0, GAP], R_PHONE))
    into = m - a * DEPTH - tip_c - v * SWEEP
    above = into + a * 0.022
    over = np.array([above[0], above[1], CARRY_Z])
    wipe = np.array([TOWEL_C[0] + 0.010, TOWEL_C[1] + 0.020, TOWEL_TOP + 0.0003]) - low
    w = [
        _wp(2.0, grasp + [0, 0, 0.08], FINGER_OPEN, "over the toothbrush handle"),
        _wp(1.5, grasp, FINGER_OPEN, "down around the handle"),
        _wp(0.8, grasp, GRIP, "close on the handle"),
        _wp(1.2, grasp + [0, 0, 0.06], GRIP, "lift"),
        _wp(1.0, [grasp[0], grasp[1], CARRY_Z], GRIP, "up, clear of the phone"),
        _wp(2.0, over, GRIP, "bristles over the port"),
        _wp(1.0, above, GRIP, "line up on the port's axis"),
        _wp(1.5, into, GRIP, "bristles into the port, about 5 mm"),
        _wp(1.2, into + v * 2 * SWEEP, GRIP, "one stroke along the opening"),
        _wp(0.8, into + v * SWEEP, GRIP, "ease back to its centre"),
        _wp(0.4, into + v * SWEEP, GRIP, "pause"),
    ]
    out = into + v * SWEEP + a * 0.022  # straight back out along the axis from the centre of the stroke
    w += [
        _wp(1.5, out, GRIP, "draw the bristles straight out along the axis"),
        _wp(1.0, [out[0], out[1], CARRY_Z], GRIP, "up"),
        _wp(2.0, [wipe[0], wipe[1], CARRY_Z - 0.06], GRIP, "over the paper towel"),
        _wp(1.5, wipe, GRIP, "bristles onto the towel"),
        _wp(1.5, wipe - [0.020, 0.0, 0.0], GRIP, "wipe the bristles across it"),
        _wp(1.0, wipe - [0.020, 0.0, -0.05], GRIP, "lift off the towel"),
        _wp(2.0, grasp + [0, 0, 0.05], GRIP, "back over its place"),
        _wp(1.5, grasp + [0, 0, 0.0015], GRIP, "lay it down"),
        _wp(0.8, grasp + [0, 0, 0.0015], FINGER_OPEN, "let go"),
        _wp(1.0, grasp + [0, 0, 0.08], FINGER_OPEN, "up"),
        _wp(1.5, (0.15, -0.26, TOP + 0.20), FINGER_OPEN, "clear"),
        _wp(3.0, (0.15, -0.26, TOP + 0.20), FINGER_OPEN, "wait"),
    ]
    total = sum(x.duration for x in w)
    return {"right": w, "left": [_wp(total, (0.15, 0.26, TOP + 0.20), FINGER_OPEN, "stay clear")]}
