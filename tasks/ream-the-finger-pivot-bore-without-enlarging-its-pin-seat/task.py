"""A hand-fabrication bench. An ivory 3D-printed robot-hand finger (Yale OpenHand Model T42 pin-flexure finger,
117.6 x 30.6 x 31.75 mm, its black urethane pad and amber urethane flexure already cast) is clamped flat in a small
green drill-press vise, its pivot knuckle overhanging the end of the jaws. The knuckle has a 6.35 mm pivot bore
through two 3 mm ears with an open spring pocket between them. A 3.3 mm routing-pin seat opens on the top face 12 mm
from the pivot. A black ratcheting T-handle tap wrench (ratchet housing with the T-bar through its top, chuck nut below it) holding a
6.38 mm carbide reamer hangs by its chuck nut in a
slotted steel tool stand. A grey hand base, two printed mold halves and a finished finger in a tray lie at the back
of the bench.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / 'fixture'))
import parts  # noqa: E402  (before lots: its reset hooks run before the lot moves)
import lots  # noqa: E402
import finger_geometry as fg  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
MM = 0.001
EPS = 1e-6
SUCCESS_HOLD_S = 0.0  # the one-second still hold is measured inside score()

# ---- native finger (STL frame, mm) placed in the vise -----------------------------------------------------------
BORE_XY = (21.7947, 9.6443)  # pivot bore axis in the STL (6.70 mm CAD bore, 6.35 mm pin)
SEAT_XY = (19.6322, 21.8193)  # adjacent 3.325 mm CAD routing-pin seat, through the full thickness
SEAT_R = 3.325 / 2
EAR_TOP = 27.25  # upper ear 24.25-27.25, lower ear 4.50-7.50 (STL z, mm)
EARS = ((24.25, 27.25), (4.50, 7.50))
CAD_R = 3.35  # CAD bore radius (mm)
LUG_Y = 19.5  # STL y below which the knuckle has no colliders (declared cutting zone)
BED = 0.032  # finger bottom face above the bench (m), on the vise bed

# Cutting rule (declared, fixture text): mm of reamed depth per clockwise revolution, loads in N.
FEED = 1.0
LEAD = 0.4  # 45-degree tip chamfer: full diameter 0.4 mm above the tip
EAR_CUT = 3.0 + LEAD  # tip travel that finishes one 3 mm ear
LOAD_MIN, LOAD_MAX = 1.5, 25.0
# Cutting load (stated estimate): while the reamer's lead bears on uncut ABS and turns clockwise, a torque of 0.15 N m
# resists the turn (about 0.03 N m of cutting at ~0.1 J/mm3 for ABS on a 0.06 mm x 1.0 mm/rev chip at 3.2 mm radius,
# plus about 0.12 N m of margin rub as the printed ABS springs back) and the vise takes the reaction. The feed
# resistance is the stop itself: it gives way only as material is removed, so the reamer must be pressed at 1.5 N+.
T_CUT = 0.15  # N m at full engagement
W_SLIP = 0.3  # rad/s: the torque builds up over the first 0.3 rad/s of clockwise turning (no drive at rest)
E_START = 0.5  # mm: a reamer starts an ear only within 0.16 mm CAD clearance plus its 0.4 mm chamfer of the axis
T_BELL = 5.0  # degrees of tilt while cutting: the hold is levering the carbide reamer hard against the ear
NOGO_R = 6.70 / 2  # mm: no-go gauge pin = the 6.70 mm CAD free-clearance bore (0.35 mm over the 6.35 mm pin)
MOUTH = (0.5, 2.0)  # mm below each ear's entry face where the swept bore is measured (no-go enters 2 mm)
_U = np.stack([np.cos(np.arange(36) * math.pi / 18), np.sin(np.arange(36) * math.pi / 18)], axis=1)
_GRID = np.stack(np.meshgrid(np.linspace(-0.3, 0.3, 41), np.linspace(-0.3, 0.3, 41)), axis=-1).reshape(-1, 2)


def swept_inradius(points):
    """Radius (mm) of the largest circle inside the convex hull of the reamer-axis positions (2D, mm) recorded
    while cutting: the reamed bore is then at least REAMER_R plus this. Support-function form over 36 directions,
    which can only overestimate it (fails safe)."""
    P = np.asarray(points, float).reshape(-1, 2)
    if len(P) < 3:
        return 0.0
    h = (P @ _U.T).max(axis=0)
    C = P.mean(axis=0) + _GRID
    return float(max(0.0, ((h[None, :] - C @ _U.T).min(axis=1)).max()))


def bell_mouthed(r, key):
    """True if the no-go pin would enter this ear 2 mm: the swept bore admits it at both mouth planes."""
    pts = r['centres'][key]
    return all(REAMER_R + swept_inradius(pts[i]) >= NOGO_R - EPS for i in range(len(MOUTH)))
GOUGE_R = 4.5  # mm from the bore axis: reamer material beyond this inside the knuckle is a gouge
REAMER_R = 6.3754 / 2
SHANK_R = 0.00315
TIP_Z, FLUTE_TOP = -0.073, -0.0444  # tool frame (m): the reamer below the chuck nut
POCKET = EARS[0][0] - EARS[1][1]  # 16.75 mm open spring pocket between the ears
PARK = 28.0  # mm: the stop leaves the bore once both ears are cut
K_FRONT, M_FRONT = 5000.0, 0.05


def f2p(p):
    """STL mm -> vise part frame (m). The part origin is the bore axis at bench level."""
    p = np.asarray(p, float)
    return np.stack([(p[..., 0] - BORE_XY[0]) * MM, (p[..., 1] - BORE_XY[1]) * MM, BED + p[..., 2] * MM], axis=-1)


def p2f(p):
    p = np.asarray(p, float)
    return np.stack([p[..., 0] / MM + BORE_XY[0], p[..., 1] / MM + BORE_XY[1], (p[..., 2] - BED) / MM], axis=-1)


def fbox(lo, hi, color=(0.86, 0.84, 0.76), **kw):
    lo, hi = f2p(lo), f2p(hi)
    return parts.box(tuple(float(v) for v in hi - lo), tuple(float(v) for v in (lo + hi) / 2), color, **kw)


def pbox(lo, hi, color, **kw):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(float(v) for v in hi - lo), tuple(float(v) for v in (lo + hi) / 2), color, **kw)


IVORY, PAD, FLEX = (0.86, 0.84, 0.76), (0.10, 0.10, 0.11), (0.82, 0.55, 0.22)
CAST_IRON, STEEL, DARK = (0.22, 0.34, 0.27), (0.58, 0.60, 0.63), (0.10, 0.10, 0.11)

# Scaffold colliders (hidden under the original mesh). The knuckle (STL y < 19.5) has none: the reamer passes it
# only through the declared cutting rule below, and any hand or tool inside its material is scored as damage.
SCAFFOLD = [((10.0, 19.5, 0.0), (31.0, 63.5, 31.75)), ((14.5, 63.5, 0.0), (26.5, 78.0, 31.75)),
            ((5.0, 78.0, 0.0), (28.0, 92.0, 31.75)), ((1.5, 92.0, 0.0), (16.0, 116.0, 31.75))]
# Cast urethane (estimated cavity fill, STL mm): the gripping pad and the distal flexure.
URETHANE = {'pad': ((22.5, 24.0, 1.0), (30.5, 58.0, 30.75)), 'flexure': ((15.0, 64.0, 1.0), (26.0, 79.0, 30.75))}
KNUCKLE_BOX = ((12.5, 0.0, 4.0), (31.5, LUG_Y, 27.75))

_V, _T = fg.mesh()
FINGER_VISUAL = parts.Visual('finger_stl', tuple(map(tuple, f2p(_V))), tuple(map(tuple, _T)), IVORY)

JAW_Y = (f2p((0, 20.0, 0))[1], f2p((0, 83.5, 0))[1])  # 63.5 mm jaws on the rigid proximal scaffold
PRINT_R = 3.125  # mm: as-printed (undersize) bore radius, estimate; the 6.350 mm go pin does not enter


def ring(r_in, thick, z0, z1, color, n=16, **kw):
    """n boxes whose inner faces make a regular n-gon of inradius r_in (m) about the bore axis, z0..z1 (m)."""
    w = 2 * r_in * math.tan(math.pi / n) * 1.06
    out = []
    for i in range(n):
        a = 2 * math.pi * i / n
        rc = r_in + thick / 2
        out.append(parts.box((thick, w, z1 - z0), (rc * math.cos(a), rc * math.sin(a), (z0 + z1) / 2), color,
                             quat=(0.0, 0.0, math.sin(a / 2), math.cos(a / 2)), **kw))
    return out


# The ears' native CAD bores (6.70 mm) guide the reamer: hidden ring colliders under the STL.
EAR_RINGS = [s for z0, z1 in EARS for s in ring(CAD_R * MM, 0.0015, BED + z0 * MM, BED + z1 * MM, IVORY, hidden=True,
                                                 friction=(0.15, 0.12))]  # carbide on reamed ABS, estimate
VISE_SHAPES = [
    *[fbox(lo, hi, IVORY, hidden=True) for lo, hi in SCAFFOLD],
    *EAR_RINGS,
    *[fbox(lo, hi, PAD if k == 'pad' else FLEX, collide=False) for k, (lo, hi) in URETHANE.items()],
    pbox((-0.045, 0.014, 0.0), (0.062, 0.080, 0.020), CAST_IRON),  # base
    pbox((-0.040, 0.016, 0.020), (0.050, 0.078, BED), CAST_IRON),  # bed and ways
    pbox((-0.0398, JAW_Y[0], 0.020), (-0.0178, JAW_Y[1], 0.0701), STEEL),  # fixed jaw, 38.1 mm deep
    pbox((-0.0178, f2p((0, 24.0, 0))[1], BED), (-0.0118, f2p((0, 60.0, 0))[1], BED + 0.025), STEEL),  # 6 mm parallel
    pbox((0.0092, JAW_Y[0], 0.020), (0.0292, JAW_Y[1], 0.0701), STEEL),  # moving jaw
    parts.cylinder(0.006, 0.046, (0.0522, (JAW_Y[0] + JAW_Y[1]) / 2, 0.040), STEEL, axis='x'),  # screw
    parts.cylinder(0.004, 0.080, (0.077, (JAW_Y[0] + JAW_Y[1]) / 2, 0.040), STEEL, axis='y'),  # tommy bar
]
# The uncut, as-printed material: a 6.25 mm ring at the cut front, on a stiff slider the cutting rule moves down.
FRONT_SHAPES = tuple(ring(PRINT_R * MM, 0.0010, BED + EAR_TOP * MM - 0.0006, BED + EAR_TOP * MM, (0.30, 0.29, 0.26)))
FRONT = parts.Slider('front', FRONT_SHAPES,
                     axis=(0.0, 0.0, -1.0), travel=0.030, mass=M_FRONT, spring=K_FRONT, damping=40.0,
                     max_effort=150.0)
VISE_A = (0.290, -0.085, TOP)
VISE = parts.Part('vise', pos=VISE_A, mass=2.0, shapes=VISE_SHAPES, joints=(FRONT,), visuals=(FINGER_VISUAL,),
                  friction=(0.6, 0.5), link_friction=(('front', (0.05, 0.05)),))  # the stop carries axial load only

# ---- ratcheting T-handle tap wrench with the 0.2510 in reamer (tool frame: chuck-nut face, axis up) -----------
GRIP_Z = 0.045  # TCP height on the wrench body above the chuck-nut face
TOOL_SHAPES = [
    parts.cylinder(REAMER_R * MM, FLUTE_TOP - TIP_Z - LEAD * MM, (0.0, 0.0, (TIP_Z + LEAD * MM + FLUTE_TOP) / 2),
                   (0.72, 0.74, 0.77)),
    parts.hull([(r * math.cos(a), r * math.sin(a), z) for z, r in ((TIP_Z, (REAMER_R - LEAD) * MM),
                                                                     (TIP_Z + LEAD * MM, REAMER_R * MM))
                for a in np.arange(16) * math.pi / 8],
               [tuple(range(16))[::-1], tuple(range(16, 32))]
               + [(k, (k + 1) % 16, 16 + (k + 1) % 16, 16 + k) for k in range(16)], (0.72, 0.74, 0.77)),  # 45 deg lead
    parts.cylinder(SHANK_R, -FLUTE_TOP, (0.0, 0.0, FLUTE_TOP / 2), (0.62, 0.64, 0.67)),
    *[parts.box((0.0009, 0.0009, FLUTE_TOP - TIP_Z - 0.002),
                (0.0029 * math.cos(a), 0.0029 * math.sin(a), (TIP_Z + FLUTE_TOP) / 2), DARK,
                quat=(0.0, 0.0, math.sin(a / 2), math.cos(a / 2)), collide=False) for a in np.arange(3) * 2 * math.pi / 3],
    parts.cylinder(0.0105, 0.022, (0.0, 0.0, 0.011), (0.20, 0.20, 0.21)),  # chuck nut: the ratchet's output
    parts.cylinder(0.0090, 0.080, (0.0, 0.0, 0.062), DARK),  # ratchet housing, T-bar through its top (the grip)
    parts.cylinder(0.0060, 0.006, (0.0, 0.0, 0.105), (0.75, 0.12, 0.10)),  # direction knob (set to tighten)
    parts.cylinder(0.0032, 0.090, (0.0, 0.0, 0.092), (0.78, 0.78, 0.80), axis='x'),  # sliding T-bar
]
RACK_H = 0.080
RACK_A = (0.200, -0.200, TOP)
TOOL_YAW = math.radians(60.0)  # T-bar square to the fingers of a +60 degree grip
TOOL = parts.Part('tool', pos=(RACK_A[0], RACK_A[1], TOP + RACK_H), yaw=TOOL_YAW, mass=0.16, shapes=TOOL_SHAPES,
                  friction=(1.0, 0.9))
RACK = parts.Part('rack', pos=RACK_A, mass=1.5, friction=(0.5, 0.4), shapes=[
    pbox((0.012, -0.045, 0.0), (0.070, 0.045, 0.010), (0.30, 0.31, 0.33)),  # foot, clear of the hanging reamer
    pbox((0.030, -0.008, 0.010), (0.046, 0.008, RACK_H), (0.30, 0.31, 0.33)),
    pbox((-0.030, 0.006, RACK_H - 0.006), (0.030, 0.030, RACK_H), (0.40, 0.41, 0.43)),
    pbox((-0.030, -0.030, RACK_H - 0.006), (0.030, -0.006, RACK_H), (0.40, 0.41, 0.43)),
    pbox((0.006, -0.006, RACK_H - 0.006), (0.030, 0.006, RACK_H), (0.40, 0.41, 0.43)),
])

# ---- the rest of the bench (fixed): hand base, mold halves, a finished finger in a tray -------------------------
_tray_finger = parts.Visual('finished_finger', tuple(map(tuple, (_V - np.array([15.0, 59.0, 0.0])) * MM
                                                         + np.array([0.0, 0.0, 0.006]))), tuple(map(tuple, _T)), IVORY)
BENCH = parts.Part('bench', pos=(0.56, 0.0, TOP), shapes=[
    pbox((-0.050, -0.045, 0.0), (0.045, 0.045, 0.040), (0.50, 0.52, 0.55)),  # hand base (printed palm)
    pbox((-0.040, 0.075, 0.0), (0.020, 0.200, 0.020), (0.35, 0.40, 0.70)),  # mold half
    pbox((0.030, 0.075, 0.0), (0.075, 0.200, 0.020), (0.35, 0.40, 0.70)),  # mold half
    pbox((-0.040, -0.200, 0.0), (0.060, -0.080, 0.006), (0.25, 0.25, 0.27)),  # parts tray
], visuals=(parts.Visual('tray_finger', tuple(map(tuple, (_V - np.array([15.0, 59.0, 0.0])) * MM
                                                      + np.array([0.010, -0.140, 0.006]))), tuple(map(tuple, _T)),
                         IVORY),))

OBJECTS = [VISE, TOOL, RACK, BENCH]
# The vise (an articulated part: the cutting stop is a joint) is never moved between lots; lot B moves the stand.
RACK_B = (0.335, -0.235, TOP)
LOTS = lots.declare(
    OBJECTS,
    lots.Lot('A', place={'tool': TOOL.pos, 'rack': RACK_A}),  # zero move: settles like lot B
    lots.Lot('B', place={'rack': RACK_B, 'tool': (RACK_B[0], RACK_B[1], TOP + RACK_H)}, yaw={'vise': math.pi}),
)
DEMO_TOUCHES = {'tool', 'rack'}

# ---- geometry helpers --------------------------------------------------------------------------------------------


def rot(st):
    return parts._quat_to_matrix(st.quat_xyzw)


def tool_samples():
    pts = []
    for z in np.arange(TIP_Z, -0.0005, 0.001):
        r = REAMER_R * MM if z <= FLUTE_TOP else SHANK_R
        a = np.arange(12) * math.pi / 6
        pts.append(np.c_[r * np.cos(a), r * np.sin(a), np.full(12, z)])
        pts.append([[0.0, 0.0, z]])
    return np.concatenate(pts)


REAMER_PTS = tool_samples()
WRENCH_PTS = np.concatenate([np.c_[r * np.cos(a), r * np.sin(a), np.full(len(a), z)]
                             for z, r in [(z, 0.0105 if z < 0.022 else 0.009) for z in np.arange(0.0, 0.103, 0.005)]
                             for a in [np.arange(16) * math.pi / 8]]
                            + [np.c_[np.linspace(-0.045, 0.045, 19), np.zeros(19), np.full(19, 0.092)]])


def box_dist(p, lo, hi):
    c, h = (np.asarray(lo) + np.asarray(hi)) / 2, (np.asarray(hi) - np.asarray(lo)) / 2
    return np.linalg.norm(np.maximum(np.abs(p - c) - h, 0.0), axis=1)


FINGER_BOXES = [KNUCKLE_BOX, *SCAFFOLD]


def finger_dist(pf):
    """Distance (mm) from STL-frame points to the finger envelope (knuckle box plus scaffold boxes)."""
    if not len(pf):
        return np.array([np.inf])
    return np.min([box_dist(pf, lo, hi) for lo, hi in FINGER_BOXES], axis=0)


def in_urethane(pf):
    return any(np.all((pf >= np.asarray(lo)) & (pf <= np.asarray(hi)), axis=1).any() for lo, hi in URETHANE.values())


def knuckle_solid(pf):
    m = pf[:, 1] < LUG_Y
    return fg.solid(pf[m]) if m.any() else np.zeros(0, bool), m


def tool_dist(points_tool):
    """Distance (m) from points in the tool frame to the tool (reamer, nut, body, T-bar cylinders)."""
    p = np.asarray(points_tool)
    if not len(p):
        return np.inf
    r = np.hypot(p[:, 0], p[:, 1])
    out = np.full(len(p), np.inf)
    for z0, z1, rad in ((TIP_Z, 0.0, REAMER_R * MM), (0.0, 0.022, 0.0105), (0.022, 0.108, 0.009)):
        dz = np.maximum(np.maximum(z0 - p[:, 2], p[:, 2] - z1), 0.0)
        out = np.minimum(out, np.hypot(np.maximum(r - rad, 0.0), dz))
    dx = np.maximum(np.abs(p[:, 0]) - 0.045, 0.0)
    out = np.minimum(out, np.hypot(dx, np.maximum(np.hypot(p[:, 1], p[:, 2] - 0.092) - 0.0032, 0.0)))
    return float(out.min())


def front_depth(r):
    if r['c2'] < EAR_CUT - EPS:
        return r['c2']
    if r['c1'] < EAR_CUT - EPS:
        return (EAR_TOP - EARS[1][1]) + r['c1']
    return PARK


def fresh():
    return dict(known=False, fail='', c2=0.0, c1=0.0, phi=None, loads=[], load=0.0, e=0.0, tilt=0.0, d_tip=-99.0,
                prev_t=None, hold=0.0, anchor=None, in_knuckle=False, released=False, tool_clear=False, why='',
                turned=0.0, vise0=None, peak=0.0, cut_events=0, reversed=0.0, torque=0.0, spin=0.0,
                centres={'c2': ([], []), 'c1': ([], [])})


REC = fresh()


def observe(objects, hands, t, q_front):
    """Update the trial record from one control step. ``hands``: per-arm world point clouds (None: unknown)."""
    r = REC
    if r['prev_t'] is not None and t <= r['prev_t'] + EPS:
        return
    dt = 0.05 if r['prev_t'] is None else min(0.1, max(0.0, t - r['prev_t']))
    r['prev_t'] = float(t)
    if hands is None or q_front is None or not all(k in objects for k in ('vise', 'tool')):
        r['known'] = False
        r['fail'] = r['fail'] or 'hand or object state unavailable'
        r['hold'] = 0.0
        return
    r['known'] = True
    vs, ts = objects['vise'], objects['tool']
    Rv, pv, Rt, pt = rot(vs), np.asarray(vs.pos), rot(ts), np.asarray(ts.pos)
    if r['vise0'] is None:
        r['vise0'] = (pv.copy(), Rv.copy())
    if np.linalg.norm(pv - r['vise0'][0]) > 0.030 or Rv[2, 2] < math.cos(math.radians(10)):
        r['fail'] = r['fail'] or 'vise knocked out of place'
    to_vise = Rv.T @ Rt
    tip = Rv.T @ (Rt @ np.array([0.0, 0.0, TIP_Z]) + pt - pv)
    axis = to_vise[:, 2]
    tipf = p2f(tip)
    r['tipv'] = [round(float(tip[0]) * 1000, 3), round(float(tip[1]) * 1000, 3)]
    r['d_tip'] = float(EAR_TOP - tipf[2])
    r['tilt'] = float(math.degrees(math.acos(max(-1.0, min(1.0, axis[2])))))
    ear_mid = (sum(EARS[0]) / 2) if r['c2'] < EAR_CUT - EPS else (sum(EARS[1]) / 2)
    zp = BED + ear_mid * MM
    at = tip + axis * (zp - tip[2]) / max(axis[2], 1e-6)
    r['e'] = float(np.hypot(at[0], at[1]) / MM)
    # Ratchet: turned by its housing, only clockwise turns (seen from above) drive the reamer; back-swings are pawl
    # slips. Turned by the chuck nut (the ratchet's output), a back-swing turns the reamer backwards.
    xv = to_vise[:, 0]
    phi = math.atan2(xv[1], xv[0])
    turn, back = 0.0, 0.0
    if r['phi'] is not None:
        d = (phi - r['phi'] + math.pi) % (2 * math.pi) - math.pi
        turn, back = max(0.0, -d), max(0.0, d)
    r['phi'] = phi
    started = (r['c2'] > 0.0 or r['c1'] > 0.0) and math.hypot(tip[0], tip[1]) <= 0.003 and r['d_tip'] > 0.0
    if back > 0 and started and hands is not None:
        on_nut = False
        for pts in hands.values():
            if len(pts):
                ht = (np.asarray(pts) - pt) @ Rt
                rr = np.hypot(ht[:, 0], ht[:, 1])
                on_nut |= bool(np.any((rr <= 0.0105 + 0.002) & (ht[:, 2] >= -0.002) & (ht[:, 2] <= 0.022)))
        if on_nut:
            r['reversed'] += back
            if r['reversed'] > math.radians(20.0) + EPS:
                r['fail'] = r['fail'] or 'reamer turned backwards by its chuck nut'
    load = K_FRONT * (q_front - front_depth(r) * MM) - M_FRONT * 9.81
    r['load'] = float(load)
    r['loads'] = (r['loads'] + [float(load)])[-5:]
    r['peak'] = max(r['peak'], float(np.mean(r['loads'])))
    if np.mean(r['loads']) > LOAD_MAX + EPS:
        r['fail'] = r['fail'] or 'axial force over 25 N split an ear'
    # Material removal: a clockwise turn under load, with the tip on the uncut front.
    front = front_depth(r)
    on_bore = math.hypot(tip[0], tip[1]) <= 0.003 + EPS  # the stop is only reachable through the bore
    engaged = on_bore and r['d_tip'] >= front - 0.5 and front < PARK
    spin = float(np.asarray(ts.ang_vel) @ (Rt[:, 2])) - float(np.asarray(vs.ang_vel) @ (Rt[:, 2]))
    r['spin'] = spin
    r['torque'] = (T_CUT * min(1.0, max(0.0, load) / LOAD_MIN) * min(1.0, max(0.0, -spin) / W_SLIP)
                   if engaged else 0.0)
    if turn > 0 and engaged and load >= LOAD_MIN - EPS:
        key, base = ('c2', 0.0) if r['c2'] < EAR_CUT - EPS else ('c1', EAR_TOP - EARS[1][1])
        if r[key] <= 0.0 and r['e'] > E_START + EPS:
            r['fail'] = r['fail'] or 'reamer turned beside the bore: ear face cut'
        elif r['tilt'] > T_BELL + EPS:
            r['fail'] = r['fail'] or 'reamer levered over 5 degrees while cutting'
        else:
            step = FEED * turn / (2 * math.pi)
            reach = max(r[key], r['d_tip'] - base + 0.2)
            r[key] = float(min(EAR_CUT, r[key] + step, reach))
            r['turned'] += turn
            r['cut_events'] += 1
            top = EARS[0][1] if key == 'c2' else EARS[1][1]
            for i, depth in enumerate(MOUTH):
                zp = BED + (top - depth) * MM
                c = tip + axis * (zp - tip[2]) / max(axis[2], 1e-6)
                r['centres'][key][i].append((float(c[0] / MM), float(c[1] / MM)))
            if bell_mouthed(r, key):
                r['fail'] = r['fail'] or 'ear bell-mouthed: the 6.70 mm no-go pin would enter'
    # Reamer inside knuckle material (beyond the bore's float zone) is a gouge; on the scaffold face it marks it.
    rp = Rv.T @ (REAMER_PTS @ Rt.T + pt - pv).T
    rf = p2f(rp.T)
    solid, m = knuckle_solid(rf)
    if solid.any():
        sub = rf[m][solid]
        rad = np.hypot(sub[:, 0] - BORE_XY[0], sub[:, 1] - BORE_XY[1])
        if (rad > GOUGE_R + EPS).any():
            r['fail'] = r['fail'] or 'reamer cut into the knuckle wall'
    if turn > 0 and tipf[2] > 31.75 - 0.5 and tipf[2] < 32.75 and tipf[1] >= LUG_Y:
        seat = math.hypot(tipf[0] - SEAT_XY[0], tipf[1] - SEAT_XY[1]) < REAMER_R + SEAT_R
        if finger_dist(tipf[None])[0] < 0.5 or seat:
            r['fail'] = r['fail'] or ('reamer turned on the routing-pin seat' if seat else 'reamer turned on the scaffold face')
    wf = p2f((Rv.T @ (WRENCH_PTS @ Rt.T + pt - pv).T).T)
    if in_urethane(rf) or in_urethane(wf):
        r['fail'] = r['fail'] or 'tool in the cast pad or flexure'
    wsolid, _ = knuckle_solid(wf)
    if wsolid.any():
        r['fail'] = r['fail'] or 'wrench pushed into the knuckle'
    r['in_knuckle'] = bool(np.any(np.all((rf >= np.asarray(KNUCKLE_BOX[0]) - 1.0)
                                         & (rf <= np.asarray(KNUCKLE_BOX[1]) + 1.0), axis=1)))
    # Hands: never inside the knuckle or the cast urethane; released and clear for the final hold.
    hand_finger, hand_tool = np.inf, np.inf
    for pts in hands.values():
        if not len(pts):
            continue
        hf = p2f((Rv.T @ (np.asarray(pts) - pv).T).T)
        near = hf[finger_dist(hf) < 40.0]
        if len(near):
            solid, m = knuckle_solid(near)
            if solid.any():
                r['fail'] = r['fail'] or 'gripper pressed into the knuckle'
            if in_urethane(near):
                r['fail'] = r['fail'] or 'gripper pressed the cast pad or flexure'
            hand_finger = min(hand_finger, float(finger_dist(near).min()))
        ht = (np.asarray(pts) - pt) @ Rt
        hand_tool = min(hand_tool, tool_dist(ht[np.linalg.norm(ht, axis=1) < 0.2]))
    allpts = np.concatenate([rf, p2f((Rv.T @ (WRENCH_PTS @ Rt.T + pt - pv).T).T)])
    tool_finger = float(finger_dist(allpts).min())
    r['released'] = bool(hand_tool >= 0.020 - EPS and hand_finger >= 20.0 - EPS)
    r['tool_clear'] = bool(tool_finger >= 20.0 - EPS and pt[2] > TOP - 0.010)
    done = r['c2'] >= EAR_CUT - EPS and r['c1'] >= EAR_CUT - EPS
    marks = np.concatenate([REAMER_PTS[::40] @ Rt.T + pt, np.array([pv, pv + Rv @ np.array([0, 0.05, 0])])])
    still = objects['tool'].resting(0.02, 0.5) and objects['vise'].resting(0.02, 0.5)
    if not (done and not r['fail'] and r['released'] and r['tool_clear'] and still):
        r['anchor'], r['hold'] = None, 0.0
    elif r['anchor'] is None or np.linalg.norm(marks - r['anchor'], axis=1).max() > 0.001 + EPS:
        r['anchor'], r['hold'] = marks.copy(), 0.0
    else:
        r['hold'] += dt


def score(objects):
    r = REC
    if not r['known'] or r['fail'] or r['in_knuckle']:
        return 0.0
    if r['c2'] >= EAR_CUT - EPS and r['c1'] >= EAR_CUT - EPS:
        return 1.0 if r['hold'] >= 1.0 - EPS else 0.75
    if r['c2'] >= EAR_CUT - EPS:
        return 0.5
    if r['c2'] >= 1.5 + LEAD - EPS:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) >= 1.0 - EPS


# ---- hands: face-sampled collision hulls ------------------------------------------------------------------------
_FACE_CACHE = {}


def face_points(name):
    if name in _FACE_CACHE:
        return _FACE_CACHE[name]
    from yam_arena import analysis
    pts = []
    for h in analysis.link_hulls().get(name, []):
        pts.append(h.vertices)
        for plane in np.unique(np.round(h.planes, 7), axis=0):
            n, d = plane[:3], plane[3]
            on = h.vertices[np.abs(h.vertices @ n - d) < 1e-6]
            if len(on) < 3:
                continue
            c = on.mean(axis=0)
            u = on[0] - c
            u /= np.linalg.norm(u)
            v = np.cross(n, u)
            ring = on[np.argsort(np.arctan2((on - c) @ v, (on - c) @ u))]
            for a, b in zip(ring, np.roll(ring, -1, axis=0)):
                count = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / .003)))
                ii, jj = np.meshgrid(np.arange(count + 1), np.arange(count + 1))
                valid = ii + jj <= count
                pts.append(c + (a - c) * ii[valid, None] / count + (b - c) * jj[valid, None] / count)
    if not pts:
        raise ValueError('missing hand hull ' + name)
    p = np.concatenate(pts)
    _, ix = np.unique(np.round(p / .003).astype(int), axis=0, return_index=True)
    _FACE_CACHE[name] = p[np.sort(ix)]
    return _FACE_CACHE[name]


def hand_clouds(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out, tcp = {}, {}
    for arm in ('left', 'right'):
        asset = base.scene[arm + '_robot']
        poses = parts._np(asset.data.body_link_pose_w)[0]
        pts = []
        for n in ('gripper', 'tip_left', 'tip_right'):
            i = list(asset.body_names).index(n)
            pose = poses[i]
            R = parts._quat_to_matrix(pose[3:])
            pts.append(face_points(n) @ R.T + pose[:3] - origin)
            if n == 'gripper':
                tcp[arm] = (R @ np.array([0.0, 0.0, 0.134]) + pose[:3] - origin).tolist()
        out[arm] = np.concatenate(pts)
    if not all(np.isfinite(p).all() for p in out.values()):
        raise ValueError('nonfinite hands')
    return out, tcp


def _cutting_torque(sim, torque):
    """Hold the cutting torque on the wrench (about its own axis, against a clockwise turn) and its reaction on the
    vise until the next control step."""
    import logging
    import torch

    base = sim.env.unwrapped
    env = torch.tensor([0], device=base.device)
    z = torch.zeros((1, 1, 3), dtype=torch.float32, device=base.device)
    t = torch.tensor([[[0.0, 0.0, float(torque)]]], dtype=torch.float32, device=base.device)
    tool, vise = base.scene['tool'], base.scene['vise']
    log = logging.getLogger(type(tool).__module__)
    vlog = logging.getLogger(type(vise).__module__)
    levels = log.level, vlog.level
    log.setLevel(logging.ERROR)
    vlog.setLevel(logging.ERROR)
    try:
        tool.set_external_force_and_torque(z, t, env_ids=env)
        vise.set_external_force_and_torque(z, -t, body_ids=[list(vise.body_names).index('base')], env_ids=env)
    finally:
        log.setLevel(levels[0])
        vlog.setLevel(levels[1])


@parts.on_reset
def _reset(sim):
    global REC
    REC = fresh()
    sim.set_target('vise', 'front', 0.0)
    _cutting_torque(sim, 0.0)


@parts.on_step
def _step(sim):
    try:
        objects = sim.states()
        hands, tcp = hand_clouds(sim)
        q = objects['vise'].joints.get('front') if hasattr(objects['vise'], 'joints') else None
        observe(objects, hands, float(sim.t), None if q is None else float(q))
        sim.set_target('vise', 'front', front_depth(REC) * MM)
        _cutting_torque(sim, REC.get('torque', 0.0) if REC['known'] else 0.0)
        if os.environ.get('TRACE_OUT') and round(sim.t * 20) % 2 == 0:
            ts = objects['tool']
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, c2=REC['c2'], c1=REC['c1'],
                       load=round(REC['load'], 3), q=q, e=round(REC['e'], 3), tilt=round(REC['tilt'], 3),
                       d_tip=round(REC['d_tip'], 3), fail=REC['fail'], hold=REC['hold'], score=score(objects),
                       released=REC['released'], clear=REC['tool_clear'], ink=REC['in_knuckle'],
                       tool=np.round(ts.pos, 5).tolist(), toolq=np.round(ts.quat_xyzw, 5).tolist(),
                       vise=np.round(objects['vise'].pos, 5).tolist(), visq=np.round(objects['vise'].quat_xyzw, 5).tolist(),
                       tcp=tcp, phi=REC['phi'], turned=round(REC['turned'], 3), tipv=REC.get('tipv'),
                       torque=round(REC.get('torque', 0.0), 4), spin=round(REC.get('spin', 0.0), 3))
            with Path(os.environ['TRACE_OUT']).with_name('ream.jsonl').open('a') as f:
                f.write(json.dumps(row) + '\n')
    except Exception as e:  # fail closed
        REC['known'] = False
        REC['fail'] = REC['fail'] or 'scorer error'
        if os.environ.get('TRACE_OUT'):
            with Path(os.environ['TRACE_OUT']).with_name('ream-errors.txt').open('a') as f:
                f.write(repr(e) + '\n')
        raise


# ---- demo: open loop from the visible layout ---------------------------------------------------------------------
CAL = json.loads((HERE / 'fixture' / 'calibration.json').read_text())
Y_HI, Y_LO = math.radians(60.0), math.radians(-60.0)
GRIP = -0.005  # fingers 4 mm past each side of the 18 mm body
N_STROKES = (18, 21)
PRELOAD = 4.0  # mm the TCP is commanded below first contact
DESCENT = 0.25  # mm per stroke (the wrist swings about 100 degrees: 0.27 mm of cut)


def demo():
    lot = lots.current()
    vp, vyaw = np.asarray(lot.pos('vise')), lot.yaw_of('vise')
    tp = np.asarray(lot.pos('tool'))
    arm = 'right' if vp[1] < 0 else 'left'
    other = 'left' if arm == 'right' else 'right'
    cal = CAL[arm]
    g = np.asarray(cal['grasp'])
    b = np.asarray(cal['bore'])

    def W(d, p, f, label, yaw=Y_HI):
        return Waypoint(float(d), tuple(float(v) for v in p), float(f), label, yaw=float(yaw))

    def tcp_tip(depth_mm, yaw=Y_HI):
        """TCP over the bore with the reamer tip ``depth_mm`` below the upper ear's top face."""
        z = TOP + BED + (EAR_TOP - depth_mm) * MM - TIP_Z + GRIP_Z
        return np.array([vp[0], vp[1], z]) + b

    above = tp + np.array([0, 0, 0.128]) + g  # finger tips 7 mm over the knob before turning to the grip yaw
    at = tp + np.array([0, 0, GRIP_Z]) + g
    lift = at + np.array([0, 0, 0.010])
    out = lift + np.array([-0.050, 0, 0])
    wx = min(out[0], vp[0] - 0.090)  # pass west of the vise body
    transfer = TOP + BED + 0.03175 * 1 + 0.006 - TIP_Z + GRIP_Z
    seq = [W(2.5, above, FINGER_OPEN, 'over the wrench'), W(2.0, at, FINGER_OPEN, 'around the wrench body'),
           W(1.0, at, GRIP, 'grip the ratchet housing'), W(1.0, lift, GRIP, 'lift the chuck nut off the stand'),
           W(2.0, out, GRIP, 'slide the reamer out of the slot'),
           W(2.0, (out[0], out[1], transfer), GRIP, 'raise the reamer'),
           W(2.0, (wx, out[1], transfer), GRIP, 'west of the vise'),
           W(2.0, (wx, vp[1] + b[1], transfer), GRIP, 'clear of the stand'),
           W(2.5, tcp_tip(-6.0)[:2].tolist() + [transfer], GRIP, 'over the pivot bore'),
           W(2.0, tcp_tip(-1.5), GRIP, 'lower the reamer to the bore'),
           W(1.5, tcp_tip(-1.5), GRIP, 'settle over the bore'),
           W(2.0, tcp_tip(PRELOAD), GRIP, 'seat the reamer with light pressure')]
    depth = PRELOAD
    for ear, n in enumerate(N_STROKES):
        for k in range(n):
            depth += DESCENT
            seq += [W(0.9, tcp_tip(depth), GRIP, f'ear {ear + 1}: turn clockwise', Y_LO),
                    W(0.6, tcp_tip(depth), GRIP, f'ear {ear + 1}: ratchet back', Y_HI)]
        if ear == 0:
            depth = (EAR_TOP - EARS[1][1]) + PRELOAD
            seq += [W(2.0, tcp_tip(depth - 0.5), GRIP, 'through the spring pocket'),
                    W(1.0, tcp_tip(depth), GRIP, 'seat on the lower ear')]
    # Withdraw turning forward (ratchet strokes while lifting), as reamers are withdrawn.
    for k in range(1, 4):
        d = depth - k * (depth + 3.0) / 3
        seq += [W(0.9, tcp_tip(d), GRIP, 'withdraw turning clockwise', Y_LO),
                W(0.6, tcp_tip(d), GRIP, 'ratchet back', Y_HI)]
    seq += [
            W(1.5, tcp_tip(-6.0)[:2].tolist() + [transfer], GRIP, 'lift clear of the knuckle'),
            W(2.5, (wx, vp[1] + b[1], transfer), GRIP, 'back clear of the vise'),
            W(2.0, (wx, out[1], transfer), GRIP, 'west of the vise'),
            W(2.0, (out[0], out[1], transfer), GRIP, 'back to the stand'),
            W(2.5, out, GRIP, 'lower beside the slot'), W(2.0, lift, GRIP, 'slide into the slot'),
            W(1.5, at + np.array([0, 0, 0.002]), GRIP, 'set the chuck nut on the stand'),
            W(1.0, at + np.array([0, 0, 0.002]), FINGER_OPEN, 'release'),
            W(2.0, above, FINGER_OPEN, 'lift clear of the knob'),
            W(2.0, above + np.array([-0.080, 0, 0]), FINGER_OPEN, 'stand clear'),
            W(2.5, above + np.array([-0.080, 0, 0]), FINGER_OPEN, 'hold still')]
    total = sum(w.duration for w in seq)
    side = 1.0 if other == 'left' else -1.0
    rest = [W(total, (0.20, side * 0.30, TOP + 0.16), FINGER_OPEN, "stay clear", 0.0)]
    return {arm: seq, other: rest}
