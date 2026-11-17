"""On the station table, in front of the arms and a little to the right of centre, stands a clear 20 oz plastic
tumbler (13.1 cm tall, 8.6 cm across) packed with ice cubes and water. A stainless pot clip hooks over its far rim;
its small ring, 6 mm above the rim, holds a bimetal dial stem thermometer upright in the middle of the cup. The
thermometer hangs from its hex calibration nut (10.6 mm across the flats) resting on the ring, with its 12.7 cm
stainless stem down in the ice water. Above the nut sits the round dial head (4.6 cm across): a white face marked
0-220 °F clockwise, a black pointer, and a red tick at 32 °F. The head turns on the stem when twisted. The thermometer
has just been put into the ice water. To the robot's left, on a wooden riser block, lies an open-end
calibration wrench with a blue dipped handle, its handle toward the robot.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
import lots  # noqa: E402  (after parts, so its reset hooks run first)

from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

# --- the dial and the ice bath (assumed values, stated in the README) -----------------------------------------
K_DEG = 300.0 / 220.0  # dial degrees per °F: 0-220 °F over 300 degrees, clockwise seen from above
K = math.radians(K_DEG)  # rad per °F
ICE_F = 32.0  # the bath; the room's temperature is set per lot
TAU_S = 6.0  # first-order time constant of the stem in ice water (s): settled to 0.3 °F after 30 s
SENSE = 0.051  # sensing area: the bottom 51 mm (2 in) of the stem
WET_S = 30.0  # the sensing area must have been fully in the water for the last 30 s
FULL_TOL, HALF_TOL = 2.0, 4.0  # °F: full credit at the FDA Food Code's +-2 °F accuracy for a Fahrenheit-scaled
# food thermometer (4-203.11(B)); half credit within twice that
HOLD_S, DT = 1.0, 0.05
HOLD_STEPS = round(HOLD_S / DT)
CLEAR = 0.005  # no gripper or fingertip within 5 mm of the thermometer or the wrench
WRENCH_GAP = 0.010  # the wrench put down at least 10 mm from the thermometer (box to box)
HANG_Z = 0.002  # the nut's underside within 2 mm above the clip's ring
HANG_XY = 0.0025  # the stem within 2.5 mm of the ring's centre at the ring
HANG_TILT = 8.0  # degrees
CUP_TILT = 10.0  # degrees: tipped further at any time, ice water spills (whole trial)
# The one scripted catch: an open-end wrench on a hex nut is modelled as a rigid hold while seated.
SEAT_XY = 0.0015  # the jaw's centre within 1.5 mm of the stem's axis
SEAT_Z = 0.0025  # the jaw's mid-plane within 2.5 mm of the nut's mid-height
SEAT_YAW = 6.0  # degrees: the jaw square to the nut's flats (modulo 60 degrees)
HEAD_HOLD = 0.08  # N m: the head's friction fit holds against up to this torque on the stem, and slips above it
HEAD_STIFF = 8.0  # N m/rad: how firmly the fit holds below that (at most 0.01 rad of give at 0.08 N m)

# --- the cup -------------------------------------------------------------------------------------------------
XS, YS = 0.260, -0.080  # the stem's axis (the cup's centre)
CUP_H, CUP_R_IN, CUP_WALL, CUP_FLOOR = 0.131, 0.0400, 0.0030, 0.003
WATER_H = 0.100  # water line above the table
RING_TOP = CUP_H + 0.006  # the clip's ring, above the table
RING_IN, RING_OUT, RING_T = 0.0035, 0.0050, 0.0025
CLEAR_PLASTIC = (0.80, 0.84, 0.86)
STEEL = (0.72, 0.73, 0.75)
ICE = (0.90, 0.95, 0.98)
WATER = (0.62, 0.78, 0.90)
_RIM = TOP + CUP_H


def _ring_boxes(r_in, r_out, z_c, height, n, color, collide=True):
    rm, w = (r_in + r_out) / 2, r_out - r_in
    length = 2 * r_out * math.tan(math.pi / n) + 0.0002
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        q = (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))
        out.append(parts.box((w, length, height), at=(rm * math.cos(a), rm * math.sin(a), z_c), color=color, quat=q,
                             collide=collide))
    return out


_CUP_SHAPES = [
    parts.prism(CUP_R_IN + CUP_WALL, CUP_FLOOR, 16, at=(0.0, 0.0, CUP_FLOOR / 2), color=CLEAR_PLASTIC,
                quat=(0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5))),
    *_ring_boxes(CUP_R_IN, CUP_R_IN + CUP_WALL, CUP_H / 2, CUP_H, 16, CLEAR_PLASTIC),
    # the water, to the water line (seen, not simulated as a liquid)
    parts.cylinder(CUP_R_IN - 0.0005, WATER_H - CUP_FLOOR, at=(0.0, 0.0, (WATER_H + CUP_FLOOR) / 2), color=WATER,
                   collide=False),
    # the pot clip on the far rim: outer leg, bridge over the rim, inner riser, arm and the ring the stem passes
    parts.box((0.0015, 0.008, 0.020), at=(CUP_R_IN + CUP_WALL + 0.0008, 0.0, CUP_H - 0.006), color=STEEL),
    parts.box((CUP_WALL + 0.004, 0.008, 0.0015), at=(CUP_R_IN + CUP_WALL / 2, 0.0, CUP_H + 0.00075), color=STEEL),
    parts.box((0.0015, 0.004, 0.006 + 0.010), at=(CUP_R_IN - 0.0008, 0.0, CUP_H - 0.002), color=STEEL),
    parts.box((CUP_R_IN - RING_OUT, 0.004, RING_T), at=((CUP_R_IN + RING_OUT) / 2 - 0.0008, 0.0,
                                                         RING_TOP - RING_T / 2), color=STEEL),
    *_ring_boxes(RING_IN, RING_OUT, RING_TOP - RING_T / 2, RING_T, 8, STEEL),
]
# ice cubes (25 mm) packed round the stem, kept 12 mm or more from it, floating at the water line
for _a, _r, _z in ((0.35, 0.026, 0.006), (1.45, 0.024, 0.002), (2.55, 0.025, 0.008), (3.65, 0.026, 0.003),
                   (4.75, 0.024, 0.007), (5.85, 0.025, 0.001)):
    _CUP_SHAPES.append(parts.box((0.022, 0.022, 0.022), at=(_r * math.cos(_a), _r * math.sin(_a), WATER_H + _z - 0.008),
                                 color=ICE, quat=(0.0, 0.0, math.sin(_a / 2), math.cos(_a / 2))))
for _a, _r, _z in ((0.9, 0.024, 0.055), (2.9, 0.025, 0.040), (4.9, 0.024, 0.060)):  # lower cubes, seen only
    _CUP_SHAPES.append(parts.box((0.020, 0.020, 0.020), at=(_r * math.cos(_a), _r * math.sin(_a), _z), color=ICE,
                                 collide=False))

CUP = parts.Part("cup", pos=(XS, YS, TOP), shapes=_CUP_SHAPES, mass=0.45, friction=(0.8, 0.7),
                 contact_offset=0.001)

# --- the thermometer: part frame origin on the stem's axis at the nut's underside --------------------------------
STEM_R, STEM_L = 0.002, 0.127
POINTER_LEN = 0.023 - 0.004  # 4 mm stem, 127 mm below the nut
NUT_AF, NUT_H = 0.0106, 0.007
NUT_R = NUT_AF / math.sqrt(3)  # to the corners
NECK = 0.004  # bare stem between the nut and the head
HEAD_R, HEAD_H = 0.023, 0.013
HEAD_Z0 = NUT_H + NECK  # the head's underside
FACE_Z = HEAD_Z0 + HEAD_H
_HEX_Q = (-0.5, -0.5, -0.5, 0.5)  # prism axis x -> z; corners toward +-x, flats facing +-y

_THERMO_BASE = [
    parts.cylinder(STEM_R, STEM_L, at=(0.0, 0.0, -STEM_L / 2), color=STEEL),
    parts.prism(NUT_R, NUT_H, 6, at=(0.0, 0.0, NUT_H / 2), color=(0.78, 0.70, 0.45), quat=_HEX_Q),
    parts.cylinder(STEM_R, NECK + 0.001, at=(0.0, 0.0, NUT_H + NECK / 2), color=STEEL),
]


def _tick(temp_f, length, width, color, z):
    """A visual tick on the dial face for temp_f (°F), in the head's frame (32 °F toward the head's +x)."""
    a = -K * (temp_f - ICE_F)
    r = HEAD_R - 0.0025 - length / 2
    q = (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))
    return parts.box((length, width, 0.0006), at=(r * math.cos(a), r * math.sin(a), z), color=color, quat=q,
                     collide=False)


_FACE = [parts.cylinder(HEAD_R - 0.0015, 0.0008, at=(0.0, 0.0, FACE_Z + 0.0004), color=(0.97, 0.97, 0.95),
                        collide=False)]
_FACE += [_tick(t, 0.0045 if t % 50 == 0 else 0.0030, 0.0008, (0.08, 0.08, 0.08), FACE_Z + 0.0010)
          for t in range(0, 221, 10)]
_FACE += [_tick(t, 0.0018, 0.0005, (0.08, 0.08, 0.08), FACE_Z + 0.0010) for t in range(12, 60, 2) if t % 10]
_FACE += [_tick(ICE_F, 0.0060, 0.0012, (0.85, 0.08, 0.08), FACE_Z + 0.0011)]

HEAD = parts.Hinge(
    "head", pivot=(0.0, 0.0, HEAD_Z0), axis=(0.0, 0.0, 1.0), lower=-0.6, upper=0.6, mass=0.015,
    spring=HEAD_STIFF, damping=0.005, max_effort=HEAD_HOLD,  # the friction fit: a hold capped at 0.08 N m
    shapes=[  # a hollow stainless case: back plate, 16-sided wall, and the lens over the face (collided)
        parts.cylinder(HEAD_R, 0.002, at=(0.0, 0.0, HEAD_Z0 + 0.001), color=STEEL),
        *_ring_boxes(HEAD_R - 0.0025, HEAD_R, HEAD_Z0 + HEAD_H / 2, HEAD_H, 16, STEEL),
        parts.cylinder(HEAD_R - 0.0025, 0.0015, at=(0.0, 0.0, FACE_Z - 0.00075), color=(0.97, 0.97, 0.95)),
        *_FACE],
)
# The pointer sits on the bimetal coil's spindle, which runs up the stem: it turns with the stem, not the head. The
# coil is not simulated. The pointer is drawn as one of 131 visual-only needles on the stem, one per 0.5 °F from 20 to
# 85 °F; each control step the coil model shows the needle for the stem's temperature plus the dial's error.
NEEDLE_T = tuple(20.0 + 0.5 * i for i in range(131))
NEEDLE_Z = FACE_Z + 0.0022


def _needle(i: int, temp_f: float):
    a = -K * (temp_f - ICE_F)  # on the stem; the face's 32 °F tick is at angle q_head
    c, sn = math.cos(a), math.sin(a)
    w = 0.0005
    pts = [(r * c - d * sn, r * sn + d * c, NEEDLE_Z) for r, d in ((0.0015, -w), (POINTER_LEN, -w),
                                                                     (POINTER_LEN, w), (0.0015, w))]
    return parts.Visual(f"needle{i}", points=tuple(pts), triangles=((0, 1, 2), (0, 2, 3)), color=(0.05, 0.05, 0.05))


NEEDLES = tuple(_needle(i, t) for i, t in enumerate(NEEDLE_T))
_THERMO_BASE.append(parts.cylinder(0.0016, 0.0010, at=(0.0, 0.0, NEEDLE_Z), color=(0.05, 0.05, 0.05),
                                   collide=False))  # the pointer's hub


def needle_index(shown_f: float) -> int:
    return int(np.clip(round((shown_f - NEEDLE_T[0]) / 0.5), 0, len(NEEDLE_T) - 1))


THERMO = parts.Part("thermometer", pos=(XS, YS, TOP + RING_TOP), shapes=_THERMO_BASE, mass=0.018,
                    joints=[HEAD], friction=(0.6, 0.5), contact_offset=0.001, visuals=NEEDLES)

# --- the wrench: part frame origin at the jaw's centre, handle toward -x (turned so it points to the robot's left) --
JAW = 0.0108  # 10.8 mm jaw opening on the 10.6 mm nut (0.1 mm play a side)
W_T = 0.004  # steel thickness
_WRENCH_SHAPES = [
    # prongs: straight jaw faces from the throat to 4.5 mm past the nut's centre, then tips flared 20 degrees
    parts.box((0.011, 0.005, W_T), at=(-0.001, JAW / 2 + 0.0025, 0.0), color=STEEL),
    parts.box((0.011, 0.005, W_T), at=(-0.001, -JAW / 2 - 0.0025, 0.0), color=STEEL),
    parts.box((0.0026, 0.004, W_T), at=(0.0045 + 0.00054, JAW / 2 + 0.00232, 0.0), color=STEEL,
              quat=(0.0, 0.0, math.sin(math.radians(10)), math.cos(math.radians(10)))),
    parts.box((0.0026, 0.004, W_T), at=(0.0045 + 0.00054, -JAW / 2 - 0.00232, 0.0), color=STEEL,
              quat=(0.0, 0.0, -math.sin(math.radians(10)), math.cos(math.radians(10)))),
    parts.box((0.005, JAW + 0.010, W_T), at=(-0.009, 0.0, 0.0), color=STEEL),  # throat
    parts.box((0.045, 0.008, W_T), at=(-0.034, 0.0, 0.0), color=STEEL),  # shank
    parts.box((0.095, 0.012, 0.008), at=(-0.1035, 0.0, 0.0), color=(0.15, 0.35, 0.80)),  # dipped handle
]
W_GRIP = -0.135  # where the left hand holds the handle (part x)
RISER = Box("riser", pos=(0.175, 0.170, TOP + 0.015), size=(0.120, 0.050, 0.030), color=(0.62, 0.45, 0.28))
W_JAW0 = (0.300, 0.170, TOP + 0.030 + 0.004)  # jaw centre; the wrench rests on its 8 mm handle on the riser
WRENCH = parts.Part("wrench", pos=W_JAW0, shapes=_WRENCH_SHAPES, mass=0.040, friction=(0.9, 0.8),
                    contact_offset=0.002)

OBJECTS = [CUP, THERMO, WRENCH, RISER]

# Per-trial starting error of the dial (°F, + reads high) and the room temperature (°F) the stem starts at.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", error=6.0, room=75.0, place={"cup": CUP.pos}),
    lots.Lot("B", error=-9.0, room=69.0, place={"cup": CUP.pos}),
    lots.Lot("C", error=-4.5, room=72.0, place={"cup": CUP.pos}),
)


# --- geometry helpers ------------------------------------------------------------------------------------------
def _R(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _yaw(q) -> float:
    r = _R(q)
    return math.atan2(r[1, 0], r[0, 0])


def _tilt(q) -> float:
    return math.degrees(math.acos(float(np.clip(_R(q)[2, 2], -1.0, 1.0))))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def dial_reading(th) -> float:
    """What the dial shows (°F): the coil puts the pointer at (stem temperature + the dial's error) on the stem; turning
    the head by q (rad, counter-clockwise from above) moves the face's scale under it by q / K."""
    return _S["temp"] + lots.current().error + float(th.joints["head"]) / K


def wet_fraction(th, cup) -> float:
    """Share of the stem's 51 mm sensing area that is under the water line inside the cup (0 to 1)."""
    rt = _R(th.quat_xyzw)
    axis = rt[:, 2]
    tip = np.asarray(th.pos, float) - STEM_L * axis
    rc = _R(cup.quat_xyzw)
    local = rc.T @ (tip - np.asarray(cup.pos, float))
    if math.hypot(local[0], local[1]) > CUP_R_IN or local[2] < 0.0:
        return 0.0
    water_z = float(cup.pos[2]) + WATER_H
    if axis[2] <= 0.1:
        return 0.0
    under = (water_z - tip[2]) / axis[2]
    return float(np.clip(under / SENSE, 0.0, 1.0))


def hanging(th, cup) -> bool:
    """In its clip: the nut's underside resting on the ring (within 2 mm above it), the stem within 2.5 mm of the
    ring's centre, tilted at most 8 degrees."""
    rc = _R(cup.quat_xyzw)
    ring = np.asarray(cup.pos, float) + rc @ np.array([0.0, 0.0, RING_TOP])
    p = np.asarray(th.pos, float)
    d = rc.T @ (p - ring)
    return bool(-0.001 - EPS <= d[2] <= HANG_Z + EPS and math.hypot(d[0], d[1]) <= HANG_XY + EPS
                and _tilt(th.quat_xyzw) <= HANG_TILT + EPS)


def wrench_seated(th, wr) -> bool:
    """The wrench's jaw is seated on the nut: its centre within 1.5 mm of the stem's axis, its mid-plane within 2.5 mm
    of the nut's mid-height, lying flat (within 10 degrees) and square to the nut's flats within 6 degrees."""
    rt, rw = _R(th.quat_xyzw), _R(wr.quat_xyzw)
    nut = np.asarray(th.pos, float) + rt @ np.array([0.0, 0.0, NUT_H / 2])
    d = rt.T @ (np.asarray(wr.pos, float) - nut)
    rel = math.degrees(_yaw(wr.quat_xyzw) - _yaw(th.quat_xyzw))
    off = abs((rel + 30.0) % 60.0 - 30.0)
    return bool(math.hypot(d[0], d[1]) <= SEAT_XY + EPS and abs(d[2]) <= SEAT_Z + EPS and off <= SEAT_YAW + EPS
                and _tilt(wr.quat_xyzw) <= 10.0 and _tilt(th.quat_xyzw) <= 10.0)


def _gap(a, b) -> float:
    """Gap between two world-aligned boxes (m; 0 when they overlap)."""
    d = np.maximum(0.0, np.maximum(np.asarray(a.low) - np.asarray(b.high), np.asarray(b.low) - np.asarray(a.high)))
    return float(np.linalg.norm(d))


# --- the record: stem temperature, hands, whole-trial faults ------------------------------------------------------
_S = parts.__dict__.setdefault("thermo_state", {})


def reset_state() -> None:
    needle, catch_tau = _S.get("needle"), _S.get("catch_tau")
    _S.clear()
    _S.update(catch_tau=catch_tau, head_target=0.0, known=False, temp=lots.current().room, wet_s=0.0, hold=0, clear=False,
              spilled=False, needle=needle)


if "temp" not in _S:
    reset_state()


def record(objects, clear: bool, dt: float = DT) -> None:
    """One control step: integrate the stem's temperature (first order toward the bath or the room, by the share of
    the sensing area under water), note whether every finger and gripper is clear, and count the still hold."""
    _S["known"] = True
    th, cup = objects["thermometer"], objects["cup"]
    f = wet_fraction(th, cup)
    target = ICE_F * f + lots.current().room * (1.0 - f)
    _S["temp"] += (target - _S["temp"]) * (1.0 - math.exp(-dt / TAU_S))
    _S["wet_s"] = _S["wet_s"] + dt if f >= 1.0 - EPS else 0.0
    _S["clear"] = bool(clear)
    if _tilt(cup.quat_xyzw) > CUP_TILT + EPS:
        _S["spilled"] = True
    _S["hold"] = _S["hold"] + 1 if _placed(objects) else 0


def _placed(objects) -> bool:
    th, cup, w = objects["thermometer"], objects["cup"], objects["wrench"]
    return bool(_S["clear"] and hanging(th, cup) and _S["wet_s"] >= WET_S - EPS and th.resting() and w.resting()
                and cup.resting() and _gap(w, th) >= WRENCH_GAP - EPS)


def score(objects):
    """1.0: the dial reads 32 ± 2 °F (the Food Code's accuracy); 0.5: 32 ± 4 °F. Either needs, held for 1 s: the thermometer hanging in its clip
    with the sensing area in the water for the last 30 s, the wrench put down clear of it, hands clear, all still.
    0 if the cup ever tipped past 10 degrees, or the hands' state is unknown."""
    if not _S["known"] or _S["spilled"] or _S["hold"] < HOLD_STEPS or not _placed(objects):
        return 0.0
    err = abs(dial_reading(objects["thermometer"]) - ICE_F)
    return 1.0 if err <= FULL_TOL + EPS else 0.5 if err <= HALF_TOL + EPS else 0.0


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    return abs(dial_reading(objects["thermometer"]) - ICE_F)


HAND_LINKS = ("tip_left", "tip_right", "gripper")


FACE_STEP = 0.003  # m between sample points on each hand hull face


def surface_points(hull) -> np.ndarray:
    """Points covering a convex hull's surface (in its own frame): its vertices plus each face sampled about every
    3 mm, so a flat gripper face near a part counts, not only its corners."""
    pts = [hull.vertices]
    _, first = np.unique(np.round(hull.planes, 6), axis=0, return_index=True)  # each distinct face once
    planes = hull.planes[np.sort(first)]
    for n, d in zip(planes[:, :3], planes[:, 3]):
        face = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(face) < 3:
            continue
        c = face.mean(axis=0)
        u = face[0] - c
        if np.linalg.norm(u) < 1e-9:
            continue
        u /= np.linalg.norm(u)
        v = np.cross(n, u)
        order = np.argsort(np.arctan2((face - c) @ v, (face - c) @ u))
        ring = face[order]
        for i in range(len(ring)):  # fan triangles from the face centre, sampled on a barycentric grid
            a_, b_ = ring[i], ring[(i + 1) % len(ring)]
            k = max(1, int(np.ceil(max(np.linalg.norm(a_ - c), np.linalg.norm(b_ - c)) / FACE_STEP)))
            for x in range(k + 1):
                for y in range(k + 1 - x):
                    pts.append((c + (a_ - c) * x / k + (b_ - c) * y / k)[None])
    return np.concatenate(pts)


_HAND_POINTS: dict[str, np.ndarray] = {}


def hand_points(link: str) -> np.ndarray:
    """Cached surface points of one hand link's collision hulls, in the link frame."""
    if link not in _HAND_POINTS:
        from yam_arena import analysis

        pts = np.concatenate([surface_points(h) for h in analysis.link_hulls()[link]])
        # one point per 1 mm cell (the meshes' many coplanar triangles sample the same faces repeatedly)
        _, keep = np.unique(np.floor(pts / 0.001).astype(np.int64), axis=0, return_index=True)
        _HAND_POINTS[link] = pts[np.sort(keep)]
    return _HAND_POINTS[link]


def clear_of_parts(points_world: np.ndarray, st, names=("thermometer", "wrench")) -> bool:
    """True when every point is more than 5 mm from every shape of the named parts (the thermometer and the wrench)."""
    from yam_arena import analysis

    for name, spec in (("thermometer", THERMO), ("wrench", WRENCH)):
        if name not in names:
            continue
        s = st[name]
        near = np.all((points_world > np.asarray(s.low) - 0.01) & (points_world < np.asarray(s.high) + 0.01), axis=1)
        if not near.any():
            continue  # no hand point within 10 mm of the part's bounding box
        links = {"base": (np.asarray(s.pos), _R(s.quat_xyzw))}
        links.update({j: (np.asarray(p), _R(q)) for j, (p, q) in s.links.items()})
        for _, c, r, size in spec.world_boxes(links):
            if analysis.box_signed_distance(points_world[near], c, r, size).min() <= CLEAR + EPS:
                return False
    return True


def _hands_clear(sim, st) -> bool:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    pts = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in HAND_LINKS:
                pts.append(hand_points(n) @ _R(poses[i, 3:]).T + (poses[i, :3] - origin))
    if len(pts) < 2 * len(HAND_LINKS):
        return False  # hand state unknown: no credit
    pts = np.concatenate(pts)
    return clear_of_parts(pts, st)


@parts.on_reset
def _on_reset(sim) -> None:
    reset_state()
    _DEMO["read"] = [None] * len(READ_T)
    _S["needle"] = None
    _S["catch_tau"] = None
    _catch_torque(sim, 0.0)
    for v in NEEDLES:
        sim.set_visible("thermometer", v.name, False)
    _show_needle(sim)


def _catch_torque(sim, tau: float) -> None:
    """Put the fit's reaction torque on the wrench instead of the stem: +tau about the vertical on the stem (cancelling
    the fit's -tau there) and -tau on the wrench, held until changed. Zero when the jaw is not seated."""
    if _S.get("catch_tau") == tau:
        return
    import logging

    import torch

    base = sim.env.unwrapped
    dev = base.device
    for name, t in (("thermometer", tau), ("wrench", -tau)):
        asset = base.scene[name]
        body = [asset.body_names.index("base")] if name == "thermometer" else None
        f = torch.zeros((1, 1, 3), dtype=torch.float32, device=dev)
        m = torch.tensor([[[0.0, 0.0, float(t)]]], dtype=torch.float32, device=dev)
        log = logging.getLogger(type(asset).__module__)
        level = log.level
        log.setLevel(logging.ERROR)
        try:
            import warnings

            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                asset.set_external_force_and_torque(f, m, body_ids=body, env_ids=torch.tensor([0], device=dev),
                                                    is_global=True)
        finally:
            log.setLevel(level)
    _S["catch_tau"] = tau


def _show_needle(sim) -> None:
    """Show the needle for the coil's angle: the stem's temperature plus the dial's error."""
    i = needle_index(_S["temp"] + lots.current().error)
    if i != _S.get("needle"):
        if _S.get("needle") is not None:
            sim.set_visible("thermometer", NEEDLES[_S["needle"]].name, False)
        sim.set_visible("thermometer", NEEDLES[i].name, True)
        _S["needle"] = i


@parts.on_step
def _on_step(sim) -> None:
    st = sim.states()
    try:
        clear = _hands_clear(sim, st)
    except Exception:  # unknown hand state: no credit
        clear = False
    record(st, clear)
    th, wr = st["thermometer"], st["wrench"]
    q, tgt = float(th.joints["head"]), _S.get("head_target", 0.0)
    # The fit holds the head at a rest point with up to 0.08 N m; turned past that it slips and stays where it slipped.
    give = HEAD_HOLD / HEAD_STIFF
    if abs(q - tgt) > give:
        tgt = q - math.copysign(give, q - tgt)
    if tgt != _S.get("head_target"):
        sim.set_target("thermometer", "head", tgt)
        _S["head_target"] = tgt
    # The scripted catch: while the jaw is seated, the nut's turn is locked to the wrench's. The fit's torque on the
    # stem (the reaction to the head's) is taken off the stem and put on the wrench, so the hand holding the wrench
    # has to resist it.
    tau = float(np.clip(HEAD_STIFF * (tgt - q), -HEAD_HOLD, HEAD_HOLD))  # the fit's torque on the head (z, N m)
    seated = wrench_seated(th, wr)
    _catch_torque(sim, tau if seated else 0.0)
    _show_needle(sim)
    _demo_read(sim.t, st)


# --- demo -------------------------------------------------------------------------------------------------------
# The demo reads the dial the way a person does: at fixed times it takes the angle between the pointer and the red
# 32 °F tick (both drawn on the dial face, seen by the right wrist camera) and turns the head by that angle while the
# wrench holds the nut. It never reads the lot, the starting error or the simulated stem temperature.
TIP = 0.013
OPEN = -0.035
HEAD_GRIP_F = -(HEAD_R - 0.003)  # pads 3 mm past the head's rim
HANDLE_GRIP_F = -0.003  # pads 3 mm past the 12 mm handle's faces
READ_T = (32.0, 37.0, 42.0, 47.0, 52.0)  # s: the dial has been in the ice water for 30 s or more
_DEMO: dict = {"read": [None] * len(READ_T), "turns": [], "after": []}
RIGHT_YAW = 0.0  # fingers close along world y, across the head
LEFT_YAW = 0.0  # fingers close along world y, across the handle


def _demo_read(t: float, st) -> None:
    for i, when in enumerate(READ_T):
        if _DEMO["read"][i] is None and t >= when and len(_DEMO["turns"]) == len(READ_T):
            th = st["thermometer"]
            stem = _yaw(th.quat_xyzw)
            face = _yaw(th.links["head"][1]) - stem  # the 32 °F tick's angle on the stem
            needle = -K * (NEEDLE_T[_S["needle"]] - ICE_F)  # the needle on show, on the stem
            shown = ICE_F + _wrap(face - needle) / K  # what the dial shows
            _DEMO["read"][i] = shown
            # turn the face under the pointer by the error; within 0.3 °F, leave it
            turn = -K * (shown - ICE_F) if abs(shown - ICE_F) > 0.3 else 0.0
            # the wrist keeps the new angle from this turn on: this turn, the holds after it and the let-go
            for w in _DEMO["after"][i]:
                w.yaw = float(w.yaw + turn)


def demo():
    x, y = XS, YS
    nut_mid = TOP + RING_TOP + NUT_H / 2
    head_mid = TOP + RING_TOP + HEAD_Z0 + HEAD_H / 2
    rg = TOP + RING_TOP + HEAD_Z0 + 0.003 + TIP  # right TCP: pad tips 3 mm above the head's underside
    up = TOP + 0.180
    # left: the wrench's handle, lying on the riser, held across its width
    hx, hy = W_JAW0[0] + W_GRIP, W_JAW0[1]
    hz = W_JAW0[2] + TIP - 0.001  # pad tips 3 mm above the 8 mm handle's underside
    lift = TOP + 0.175
    sag = 0.003  # the loaded left arm settles about 3-4 mm low: command it higher
    approach_x = x - 0.045 + W_GRIP  # the jaw 45 mm short of the nut (clear of the head), on the robot's side
    seat_x = x + 0.0005 + W_GRIP  # the throat against the nut
    wz = nut_mid + (hz - W_JAW0[2]) + sag
    wy = y - 0.001  # the left arm settles about 1 mm to the robot's left of its target here
    left = [
        Waypoint(1.5, (hx, hy, lift), OPEN, "over the wrench", LEFT_YAW),
        Waypoint(1.5, (hx, hy, hz), OPEN, "down to the handle", LEFT_YAW),
        Waypoint(0.8, (hx, hy, hz), HANDLE_GRIP_F, "grip the handle", LEFT_YAW),
        Waypoint(1.2, (hx, hy, lift), HANDLE_GRIP_F, "lift", LEFT_YAW),
        Waypoint(2.0, (approach_x, wy, lift), HANDLE_GRIP_F, "carry in front of the nut", LEFT_YAW),
        Waypoint(1.5, (approach_x, wy, wz), HANDLE_GRIP_F, "down to the nut's height", LEFT_YAW),
        Waypoint(2.0, (seat_x, wy, wz), HANDLE_GRIP_F, "slide the jaw onto the nut", LEFT_YAW),
        Waypoint(51.0, (seat_x, wy, wz), HANDLE_GRIP_F, "hold the nut", LEFT_YAW),
        Waypoint(1.5, (approach_x, wy, wz), HANDLE_GRIP_F, "slide the jaw off", LEFT_YAW),
        Waypoint(1.2, (approach_x, wy, lift), HANDLE_GRIP_F, "lift", LEFT_YAW),
        Waypoint(2.0, (hx, hy, lift), HANDLE_GRIP_F, "carry back", LEFT_YAW),
        Waypoint(1.2, (hx, hy, hz + 0.006), HANDLE_GRIP_F, "lower onto the riser", LEFT_YAW),
        Waypoint(0.6, (hx, hy, hz + 0.006), OPEN, "let go", LEFT_YAW),
        Waypoint(1.2, (hx, hy, lift), OPEN, "up", LEFT_YAW),
    ]
    turns = [Waypoint(2.0, (x, y, rg), HEAD_GRIP_F, f"turn the head by reading {i + 1}", RIGHT_YAW)
             for i in range(len(READ_T))]
    _DEMO["turns"] = turns
    _DEMO["read"] = [None] * len(READ_T)
    right = [
        Waypoint(14.0, (x + 0.06, y - 0.08, up), OPEN, "wait", RIGHT_YAW),
        Waypoint(2.0, (x + 0.06, y, rg), OPEN, "beyond the head, at its height", RIGHT_YAW),
        Waypoint(2.0, (x, y, rg), OPEN, "fingers round the head", RIGHT_YAW),
        Waypoint(1.0, (x, y, rg), HEAD_GRIP_F, "pinch the head", RIGHT_YAW),
        Waypoint(13.0, (x, y, rg), HEAD_GRIP_F, "wait for the dial to settle", RIGHT_YAW),
        turns[0],
        Waypoint(3.0, (x, y, rg), HEAD_GRIP_F, "hold, let the dial steady", RIGHT_YAW),
        turns[1],
        Waypoint(3.0, (x, y, rg), HEAD_GRIP_F, "hold, let the dial steady", RIGHT_YAW),
        turns[2],
        Waypoint(3.0, (x, y, rg), HEAD_GRIP_F, "hold, let the dial steady", RIGHT_YAW),
        turns[3],
        Waypoint(3.0, (x, y, rg), HEAD_GRIP_F, "hold, let the dial steady", RIGHT_YAW),
        turns[4],
        Waypoint(2.0, (x, y, rg), HEAD_GRIP_F, "hold", RIGHT_YAW),
        Waypoint(1.0, (x, y, rg), OPEN, "let go of the head", RIGHT_YAW),
        Waypoint(1.5, (x + 0.06, y, rg), OPEN, "back off beyond the head", RIGHT_YAW),
        Waypoint(2.0, (x + 0.03, y - 0.14, up), OPEN, "away", RIGHT_YAW),
    ]
    _DEMO["after"] = [right[right.index(t):right.index(turns[-1]) + 3] for t in turns]  # to "let go of the head"
    return {"left": left, "right": right}
