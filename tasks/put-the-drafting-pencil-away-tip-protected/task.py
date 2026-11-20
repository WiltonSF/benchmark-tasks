"""A drafting mechanical pencil lies across a small two-post pen rest, its pocket clip on top, beside an empty pen tray.

The pencil is 150 mm long: a silver nose cone with a thin lead-guide sleeve sticking out of it, a 10 mm knurled
metal grip, a dark 9 mm barrel, a silver metal pocket clip along the barrel's back half, and a silver push button
at the back end. The pen rest has a V notch in each post. The pen tray is a light grey plastic channel with low walls.
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402  (before lots.py)
import lots  # noqa: E402
import observation  # noqa: E402
import tilt  # noqa: E402  (scripted demo only)
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.1  # the 1 s release hold is timed inside the step hook
TOP = TABLE_TOP[SCENE]
MM = 0.001

SILVER, KNURL, BARREL, INNER = (0.78, 0.79, 0.80), (0.55, 0.56, 0.58), (0.13, 0.14, 0.16), (0.40, 0.40, 0.42)
REST_C, TRAY_C = (0.86, 0.86, 0.84), (0.70, 0.73, 0.76)


def _cyl(r, x0, x1, color):
    """A cylinder along the pencil's long axis (part x), radius r mm, from x0 to x1 mm."""
    return parts.cylinder(r * MM, (x1 - x0) * MM, ((x0 + x1) / 2 * MM, 0.0, 0.0), color, axis="x")


def _box(lo, hi, color, rot=None):
    lo, hi = np.asarray(lo, float) * MM, np.asarray(hi, float) * MM
    return parts.box(tuple(hi - lo), tuple((lo + hi) / 2), color, rot=rot)


def _ry(a):
    return parts._matrix_to_quat(parts._rot((0, 1, 0), a))


def _rx(a):
    return parts._matrix_to_quat(parts._rot((1, 0, 0), a))


# -- the pencil (part frame: origin on the long axis, +x from the tip towards the push button, z up, clip on top) ----
AXIS_Z = 0.025  # m: the axis above the table while the pencil lies in the rest's notches
NOSE_X, SLEEVE = -71.0, 4.0  # mm: the nose cone's front face; the guide sleeve's length out of it when extended
BARREL_SHAPES = [
    _cyl(1.5, NOSE_X, -67, SILVER), _cyl(2.5, -67, -62, SILVER), _cyl(3.5, -62, -57, SILVER),  # stepped nose cone
    _cyl(5.0, -57, -17, KNURL),  # knurled metal grip, 10 mm across
    _cyl(4.5, -17, 66, BARREL),  # barrel, 9 mm across
    _cyl(3.0, 66, 75, SILVER),  # rear push button (lead advance)
]
# Tip carrier: the guide sleeve and the inner tube it is fixed to. Its spring pulls it back into the nose; a shoulder
# on the tube rests against a tooth on the clip, which holds the sleeve out.
CARRIER_SHAPES = (
    _cyl(0.6, NOSE_X - SLEEVE, NOSE_X, SILVER),  # 1.2 mm lead-guide sleeve, written extended
    _cyl(1.0, NOSE_X, 30, INNER),  # inner tube (inside the barrel)
    _box((30, -1.5, -1.5), (32, 1.5, 3.5), INNER),  # shoulder
)
TRAVEL = SLEEVE * MM  # m: the carrier's travel from extended to housed (its spring end stop)
CARRIER_K, CARRIER_FREE = 20.0, TRAVEL + 0.015  # N/m and the spring's free position: about 0.3-0.4 N preload
# Pocket clip: hinged about a cross pin; pressing its raised back end opens its front jaw and lifts the tooth.
PIVOT = (46.0 * MM, 0.0, 5.6 * MM)
LEVER_L, LEVER_TILT = 16.0, 0.22  # mm, rad: the back end rises away from the barrel
_lc = (46 + LEVER_L / 2 * math.cos(LEVER_TILT), 5.6 + 0.6 + LEVER_L / 2 * math.sin(LEVER_TILT))
CLIP_SHAPES = (
    _box((10, -1.75, 5.0), (46, 1.75, 6.2), SILVER),  # clip blade along the barrel
    _box((10, -1.75, 4.6), (13, 1.75, 5.0), SILVER),  # bead at the jaw end
    _box((43, -2.25, 4.6), (49, 2.25, 6.2), SILVER),  # hinge knuckle
    _box((32.3, -1.0, 2.0), (33.8, 1.0, 5.0), INNER),  # tooth, through a slot in the barrel
    _box((_lc[0] - LEVER_L / 2, -1.75, _lc[1] - 0.6), (_lc[0] + LEVER_L / 2, 1.75, _lc[1] + 0.6), SILVER,
         rot=_ry(-LEVER_TILT)),  # raised press end
)
CLIP_OPEN = 0.24  # rad: the press end meets the barrel
CLIP_K, CLIP_FREE = 0.03, -0.5  # N m/rad and the spring's free angle (beyond the closed stop): preloaded shut
MASS_BARREL, MASS_CARRIER, MASS_CLIP = 0.011, 0.005, 0.004  # kg: estimated split of about 20 g
PENCIL_YAW_A = -math.pi / 2  # lot A: tip to the robot's left
PENCIL_XY_A = (0.34, 0.0)
PENCIL = parts.Part(
    "pencil", pos=(*PENCIL_XY_A, TOP + AXIS_Z), yaw=PENCIL_YAW_A, mass=MASS_BARREL, friction=(0.9, 0.8),
    shapes=BARREL_SHAPES,
    joints=[parts.Slider("carrier", CARRIER_SHAPES, axis=(1, 0, 0), travel=TRAVEL, mass=MASS_CARRIER,
                         spring=CARRIER_K, damping=0.5, max_effort=5.0),
            parts.Hinge("clip", CLIP_SHAPES, pivot=PIVOT, axis=(0, 1, 0), lower=0.0, upper=CLIP_OPEN,
                        mass=MASS_CLIP, spring=CLIP_K, damping=0.001, max_effort=1.0)])

# -- the pen rest (loose; its frame is the pencil's, at table level) -------------------------------------------------
POSTS_X = (-10.0, 40.0)  # mm along the pencil


def _rest_shapes():
    out = [_box((-18, -18, 0), (48, 18, 5), REST_C)]
    s = math.sin(math.pi / 4)
    for xc in POSTS_X:
        out.append(_box((xc - 4, -12, 5), (xc + 4, 12, 14.5), REST_C))
        for side in (-1, 1):  # 45-degree V faces, each tangent to the 9 mm barrel
            c = np.array([xc, side * s * 6.5, AXIS_Z / MM - s * 6.5])
            out.append(_box(c - (4, 7, 2), c + (4, 7, 2), REST_C, rot=_rx(side * math.pi / 4)))
    return out


REST = parts.Part("pen_rest", pos=(*PENCIL_XY_A, TOP), yaw=PENCIL_YAW_A, mass=0.15, friction=(0.9, 0.8),
                  shapes=_rest_shapes())

# -- the pen tray (loose; long axis along its x) ---------------------------------------------------------------------
TRAY_IN = (0.190, 0.040)  # inner length and width
TRAY_WALL, TRAY_H, TRAY_FLOOR = 0.003, 0.015, 0.003
_ox, _oy = TRAY_IN[0] / 2 + TRAY_WALL, TRAY_IN[1] / 2 + TRAY_WALL
TRAY_SHAPES = [
    parts.box((2 * _ox, 2 * _oy, TRAY_FLOOR), (0, 0, TRAY_FLOOR / 2), TRAY_C),
    parts.box((TRAY_WALL, 2 * _oy, TRAY_H), (_ox - TRAY_WALL / 2, 0, TRAY_H / 2), TRAY_C),
    parts.box((TRAY_WALL, 2 * _oy, TRAY_H), (-_ox + TRAY_WALL / 2, 0, TRAY_H / 2), TRAY_C),
    parts.box((2 * _ox - 2 * TRAY_WALL, TRAY_WALL, TRAY_H), (0, _oy - TRAY_WALL / 2, TRAY_H / 2), TRAY_C),
    parts.box((2 * _ox - 2 * TRAY_WALL, TRAY_WALL, TRAY_H), (0, -_oy + TRAY_WALL / 2, TRAY_H / 2), TRAY_C),
]
TRAY_OFFSET = 0.090  # m: tray centre from the pencil's axis, towards the robot, in both lots


def _tray_xy(xy, yaw):
    """The tray centre: TRAY_OFFSET from the pencil towards the robot (along the pencil frame's y axis)."""
    side = np.array([-math.sin(yaw), math.cos(yaw)])
    side = side if side[0] < 0 else -side
    return tuple(float(v) for v in np.asarray(xy) + TRAY_OFFSET * side)


TRAY = parts.Part("pen_tray", pos=(*_tray_xy(PENCIL_XY_A, PENCIL_YAW_A), TOP), yaw=PENCIL_YAW_A, mass=0.12,
                  friction=(0.8, 0.7), shapes=TRAY_SHAPES)

OBJECTS = [PENCIL, REST, TRAY]
TRAY_B, TRAY_YAW_B = (0.29, 0.20), 0.0  # lot B: tray to the robot's left, its long axis pointing away from the robot
LOTS = lots.declare(OBJECTS,
                    lots.Lot("A", place={TRAY.name: TRAY.pos}),  # zero move: forces the settle
                    lots.Lot("B", place={TRAY.name: (*TRAY_B, TOP)}, yaw={TRAY.name: TRAY_YAW_B}))

# -- scoring --------------------------------------------------------------------------------------------------------
HOUSED_MIN = TRAVEL - 0.0003  # carrier travel: the sleeve no more than 0.3 mm proud of the nose
AXIS_MAX = 0.009  # m: pencil axis at most 9 mm above the tray floor (lying on it, not propped on a wall)
LEVEL_DEG = 10.0  # pencil long axis within 10 degrees of level
TRAY_FLAT_DEG = 5.0
CLEAR = 0.005
STILL_V, STILL_W, STILL_Q = 0.005, 0.05, 0.002  # m/s, rad/s, joint motion per control step (rad or m)
HOLD_S = 1.0
TABLE_BOX = ((-0.05, -0.45), (0.60, 0.45))  # world x/y region the pencil must never leave
FLOOR_Z = TOP - 0.02  # the pencil's axis below this means it fell off the table


def _finite(*xs) -> bool:
    return all(np.isfinite(np.asarray(x, float)).all() for x in xs)


def _rot(s) -> np.ndarray:
    q = np.asarray(s.quat_xyzw, float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-6:
        raise ValueError("bad quaternion")
    return parts._quat_to_matrix(q / np.linalg.norm(q))


def tilt_deg(s) -> float:
    return math.degrees(math.acos(float(np.clip(_rot(s)[2, 2], -1.0, 1.0))))


def axis_tilt_deg(s) -> float:
    """How far the pencil's long axis is from level (degrees)."""
    return math.degrees(math.asin(float(np.clip(abs(_rot(s)[2, 0]), 0.0, 1.0))))


def housed(pencil) -> bool:
    """The guide sleeve is drawn back into the nose (no more than 0.3 mm proud)."""
    q = pencil.joints.get("carrier", math.nan)
    return bool(math.isfinite(q) and q >= HOUSED_MIN - 1e-9)


def boxes(spec: parts.Part, s) -> list:
    """(centre, rotation, half size) of every shape of a part in world, from its reported link poses."""
    poses = {"base": (np.asarray(s.pos, float), _rot(s))}
    for name, (p, q) in getattr(s, "links", {}).items():
        poses[name] = (np.asarray(p, float), parts._quat_to_matrix(np.asarray(q, float)))
    if {n for n, _, _ in spec.links} - set(poses):
        raise ValueError("missing link poses")
    return [(c, r, s_ / 2) for _, c, r, s_ in spec.world_boxes(poses)]


def _corners(box_list) -> np.ndarray:
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    return np.concatenate([c + (signs * h) @ r.T for c, r, h in box_list])


def in_tray(pencil, tray) -> bool:
    """Every corner of the pencil's shapes inside the tray's inner walls, lying on the floor, level; tray flat."""
    try:
        rt, tp = _rot(tray), np.asarray(tray.pos, float)
        local = (_corners(boxes(PENCIL, pencil)) - tp) @ rt
        axis = (np.asarray(pencil.pos, float) - tp) @ rt
        return bool(np.all(np.abs(local[:, 0]) <= TRAY_IN[0] / 2 + 1e-6)
                    and np.all(np.abs(local[:, 1]) <= TRAY_IN[1] / 2 + 1e-6)
                    and TRAY_FLOOR - 1e-6 <= axis[2] <= TRAY_FLOOR + AXIS_MAX + 1e-6
                    and axis_tilt_deg(pencil) <= LEVEL_DEG + 1e-6 and tilt_deg(tray) <= TRAY_FLAT_DEG + 1e-6)
    except (ValueError, KeyError, TypeError):
        return False


def hand_gap(clouds, box_list) -> float:
    """Smallest distance from any sampled gripper or fingertip surface point to any of the boxes (m)."""
    if not clouds or not box_list or any(
            c.ndim != 2 or c.shape[1] != 3 or not len(c) or not np.isfinite(c).all() for c in clouds):
        return -math.inf
    pts = np.concatenate(clouds)
    best = math.inf
    for c, r, h in box_list:
        local = np.abs((pts - c) @ r) - h
        out = np.linalg.norm(np.maximum(local, 0.0), axis=1)
        inside = np.minimum(local.max(axis=1), 0.0)
        best = min(best, float((out + inside).min()))
    return best


def still(speeds, turn_rates, dq) -> bool:
    """Every part moves under 5 mm/s and turns under 0.05 rad/s, and no joint moves more than 0.002 a step."""
    return bool(all(v <= STILL_V for v in speeds) and all(w <= STILL_W for w in turn_rates) and dq <= STILL_Q)


def motion(prev, s, dt):
    """(speed m/s, turn rate rad/s) of a part from its pose change over one control step."""
    if prev is None or not dt > 0:
        return math.inf, math.inf
    p0, r0 = prev
    r = _rot(s)
    cos = float(np.clip((np.trace(r0.T @ r) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.linalg.norm(np.asarray(s.pos, float) - p0)) / dt, math.acos(cos) / dt


def _stamp(*states):
    return tuple(float(v) for s in states for v in np.r_[s.pos, s.quat_xyzw, list(s.joints.values())])


REC = {}


def _fresh():
    return dict(known=False, invalid=False, dropped=False, held=0.0, time=0.0, prev=None, q=None, gap=math.inf,
                stamp=(), poses=None)


REC.update(_fresh())


@parts.on_reset
def _reset(sim):
    REC.clear()
    REC.update(_fresh())
    observation.reset()
    sim.set_target(PENCIL.name, "carrier", CARRIER_FREE)  # the tip spring pulls the carrier back into the nose
    sim.set_target(PENCIL.name, "clip", CLIP_FREE)  # the clip spring holds the clip shut, tooth down


@parts.on_step
def _step(sim):
    st = sim.states()
    pencil, tray = st[PENCIL.name], st[TRAY.name]
    dt = float(sim.t) - REC["time"]
    try:
        clouds, force = observation.hand_state(sim)
        gap = hand_gap(clouds, boxes(PENCIL, pencil) + boxes(TRAY, tray))
        REC["known"] = math.isfinite(gap)
    except (ValueError, AttributeError, KeyError, IndexError, TypeError):
        gap, force, REC["known"] = -math.inf, math.nan, False
    q = np.array([pencil.joints.get("carrier", math.nan), pencil.joints.get("clip", math.nan)])
    if (not REC["known"] or not 0 < dt <= 0.100001
            or not _finite(pencil.pos, pencil.lin_vel, pencil.ang_vel, tray.pos, q)):
        REC["invalid"] = True
    x, y, z = (float(v) for v in pencil.pos)
    if not (TABLE_BOX[0][0] <= x <= TABLE_BOX[1][0] and TABLE_BOX[0][1] <= y <= TABLE_BOX[1][1]) or z < FLOOR_Z:
        REC["dropped"] = True
    dq = float(np.max(np.abs(q - REC["q"]))) if REC["q"] is not None and np.isfinite(q).all() else math.inf
    REC["q"] = q
    REC["gap"] = gap
    try:
        moves = [motion(None if REC["poses"] is None else REC["poses"][i], st_, dt) for i, st_ in enumerate((pencil, tray))]
        REC["poses"] = [(np.asarray(st_.pos, float), _rot(st_)) for st_ in (pencil, tray)]
    except ValueError:
        moves, REC["poses"], REC["invalid"] = [(math.inf, math.inf)] * 2, None, True
    ok = (not (REC["invalid"] or REC["dropped"]) and gap > CLEAR + 1e-9
          and still([m[0] for m in moves], [m[1] for m in moves], dq)
          and (housed(pencil) or in_tray(pencil, tray)))
    REC["held"] = REC["held"] + dt if ok and REC["prev"] else 0.0
    REC["prev"] = ok
    REC["time"] = float(sim.t)
    REC["stamp"] = _stamp(pencil, tray)
    if os.environ.get("TRACE_OUT"):
        try:
            tz = round(float(((np.asarray(pencil.pos) - np.asarray(tray.pos)) @ _rot(tray))[2]) * 1000, 2)
        except ValueError:
            tz = None
        row = dict(t=round(REC["time"], 2), lot=lots.current().name, pencil=np.round(pencil.pos, 4).tolist(),
                   axis_tilt=round(axis_tilt_deg(pencil), 2), roll_tilt=round(tilt_deg(pencil), 1),
                   carrier_mm=round(float(q[0]) * 1000, 2), clip=round(float(q[1]), 3),
                   rest=np.round(st[REST.name].pos, 4).tolist(), tray=np.round(tray.pos, 4).tolist(), z_in_tray_mm=tz,
                   v_mm_s=round(float(np.linalg.norm(pencil.lin_vel)) * 1000, 1),
                   w=round(float(np.linalg.norm(pencil.ang_vel)), 4), moved_mm_s=round(moves[0][0] * 1000, 2),
                   turned=round(moves[0][1], 4), tray_moved_mm_s=round(moves[1][0] * 1000, 2), dq=round(dq, 5), gap_mm=round(gap * 1000, 1),
                   finger_N=round(float(force), 2), housed=housed(pencil), in_tray=in_tray(pencil, tray),
                   held=round(REC["held"], 2), dropped=REC["dropped"], invalid=REC["invalid"], score=score(st))
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("pencil.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


def score(objects) -> float:
    """0.5 for the tip housed, 0.5 for the pencil lying in the tray; only once let go and still for 1 s."""
    try:
        pencil, tray = objects[PENCIL.name], objects[TRAY.name]
        if not REC["known"] or REC["invalid"] or REC["dropped"] or REC["stamp"] != _stamp(pencil, tray):
            return 0.0
        if REC["held"] < HOLD_S - 1e-9:
            return 0.0
        return 0.5 * housed(pencil) + 0.5 * in_tray(pencil, tray)
    except (KeyError, AttributeError, TypeError, ValueError):
        return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


# -- demo: open loop from the published layout (the pencil's, rest's and tray's visible start poses) -----------------
GRIP_X = -40.0  # mm: pinch the knurled grip
PRESS_X = 57.0  # mm: on the clip's raised press end
TIP = 0.013  # fingertips below the TCP
PINCH = -0.003  # finger target: about 6 mm between the pads on the 10 mm grip
RELEASE = -0.012  # about 24 mm between the pads: open inside the 40 mm tray
HOLD_TILT, PRESS_TILT = 30.0, 0.0  # degrees each hand's fingertips lean away from its own arm (keeps the arms apart)


def _wrap(a, arm):
    """A gripper yaw for this arm: the fingers are symmetric, so a grip and its half turn are the same; turn the left
    wrist the negative way and the right wrist the positive way near a quarter turn (both stay in reach)."""
    a = (a + math.pi / 2) % math.pi - math.pi / 2
    if arm == "left" and a > math.pi / 4:
        return a - math.pi
    if arm == "right" and a < -math.pi / 4:
        return a + math.pi
    return a


def demo():
    lot = lots.current()
    c = np.asarray(lot.pos(PENCIL.name), float)  # the pencil's axis centre, as seen
    yaw = lot.yaw_of(PENCIL.name)
    rot = parts._rot((0, 0, 1), yaw)
    t = np.asarray(lot.pos(TRAY.name), float)
    tyaw = lot.yaw_of(TRAY.name)  # the pencil goes in along the tray's length
    trot = parts._rot((0, 0, 1), tyaw)

    def at(x_mm, z, origin=c, r=rot):  # a point on the pencil's axis line (x mm along it), z m above the axis
        p = origin + r @ np.array([x_mm * MM, 0.0, 0.0])
        return (float(p[0]), float(p[1]), float(origin[2] + z))

    grip = at(GRIP_X, 0.0)
    holder = "left" if grip[1] > c[1] else "right"  # the hand on the tip's side holds; the other works the clip
    presser = "right" if holder == "left" else "left"
    hy, py = _wrap(yaw, holder), _wrap(yaw + math.pi / 2, presser)  # pads across the pencil; pressing fingers along it

    def tw(arm, deg, d, p, f, label, gyaw):
        """A leaned waypoint that puts the fingertips' centre (TIP below the TCP along the fingers) at p."""
        head = tilt.away(arm, p[:2])
        tcp = np.asarray(p, float) - tilt.rotation(gyaw, deg, head) @ np.array([0.0, 0.0, TIP])
        return tilt.Waypoint(d, tuple(float(v) for v in tcp), f, label, gyaw, tilt=deg, heading=head)

    # Holder: pinch the knurled grip in place on the rest and keep it there while the clip is pressed, then carry the
    # pencil over the tray, lay it in level and let go.
    pz = -0.006  # fingertips 6 mm below the axis
    tray_axis = np.array([t[0], t[1], t[2] + TRAY_FLOOR + 0.005 + 0.006])  # axis 6 mm above lying on the floor
    gt = at(GRIP_X, 0.0, tray_axis, trot)
    ty = _wrap(tyaw, holder)
    lift = 0.05
    hold = [
        tw(holder, HOLD_TILT, 2.5, at(GRIP_X, 0.08), FINGER_OPEN, "above the grip", hy),
        tw(holder, HOLD_TILT, 2.0, at(GRIP_X, pz), FINGER_OPEN, "pads either side of the grip", hy),
        tw(holder, HOLD_TILT, 1.0, at(GRIP_X, pz), PINCH, "pinch the grip", hy),
        tw(holder, HOLD_TILT, 7.0, at(GRIP_X, pz), PINCH, "hold it in the rest", hy),
        tw(holder, HOLD_TILT, 2.0, at(GRIP_X, pz + lift), PINCH, "lift out of the notches", hy),
        tw(holder, HOLD_TILT, 2.5, (gt[0], gt[1], c[2] + pz + lift), PINCH, "over the tray", ty),
        tw(holder, HOLD_TILT, 2.0, (gt[0], gt[1], gt[2] + pz), PINCH, "lower into the tray", ty),
        tw(holder, HOLD_TILT, 1.0, (gt[0], gt[1], gt[2] + pz), RELEASE, "let go", ty),
        tw(holder, HOLD_TILT, 2.0, (gt[0], gt[1], gt[2] + pz + 0.10), RELEASE, "lift clear", ty),
        tw(holder, HOLD_TILT, 4.0, (gt[0], gt[1], gt[2] + pz + 0.10), RELEASE, "stay clear", ty),
    ]
    # Presser: with closed fingers, press the clip's raised back end down to the barrel, then lift away.
    shut = 0.0
    press = [
        tw(presser, PRESS_TILT, 2.5, at(PRESS_X, 0.08), shut, "above the clip's press end", py),
        tw(presser, PRESS_TILT, 3.0, at(PRESS_X, 0.08), shut, "wait for the pinch", py),
        tw(presser, PRESS_TILT, 1.5, at(PRESS_X, 0.012), shut, "onto the press end", py),
        tw(presser, PRESS_TILT, 1.5, at(PRESS_X, 0.0035), shut, "press the clip open", py),
        tw(presser, PRESS_TILT, 1.0, at(PRESS_X, 0.0035), shut, "hold", py),
        tw(presser, PRESS_TILT, 1.5, at(PRESS_X, 0.08), shut, "lift off", py),
    ]
    park = {"left": (0.18, 0.28, TOP + 0.15), "right": (0.18, -0.28, TOP + 0.15)}
    press += [Waypoint(2.0, park[presser], shut, "park"), Waypoint(12.0, park[presser], shut, "wait")]
    return {holder: hold, presser: press}
