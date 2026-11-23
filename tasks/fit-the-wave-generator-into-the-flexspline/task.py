"""A robot-joint assembly bench. A grey printed robot joint housing stands output side down in a black printed
assembly cradle. In its top sits a size-14 strain-wave gear (Harmonic Drive CSF-14 component set): the dark steel
circular spline ring is bolted flush with the housing's top face, and inside it the bright steel flexspline cup
stands open end up, level with that face, its thin toothed rim round and empty. The wave generator that belongs in
it lies on a silver parts tray beside the cradle, steel plug down: a 30 mm steel plug (an elliptical cam inside a
thin bearing), a black Oldham coupling disc on top of it and a 16 mm steel hub with a 6 mm bore on top. At the back
of the bench are the joint's stepper motor on its printed adapter plate and the joint's printed cover.
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
import parts  # noqa: E402  (before lots: its reset hooks run before the lot moves)
import lots  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
MM = 0.001
EPS = 1e-6
SUCCESS_HOLD_S = 0.0  # the one-second still hold is measured inside score()

# ---- the gear in its housing (housing frame: origin on the gear axis at bench level, mm) ------------------------
Z_TOP = 60.0  # circular spline face = flexspline open end ("located axially on the same plane", maker's catalogue)
CS_R = (17.8, 25.0)  # circular spline: tooth tips to outside (50 mm pilot)
CS_Z = (52.0, 60.0)
FS_R = (15.5, 17.7)  # flexspline bore (round, before the wave generator goes in) and rim outside
FS_Z = (31.5, 60.0)  # 28.5 mm cup, diaphragm on the output flange
CHAMFER = 1.0  # mm lead-in at the flexspline's open end (estimate)
HOUSING_R = (25.5, 36.0)
CRADLE = 10.0  # mm plate under the housing

# ---- the declared mesh rule (fidelity text) ------------------------------------------------------------------------
MOUTH = 1.5  # mm: the plug's chamfer and the rim's lead-in go in freely; the teeth zone starts here
HOME = 8.4  # mm: plug bottom when the generator is home (8.0 mm plug, catalogue insertion depth 0.4 mm)
FEED = 4.0  # mm of insertion per full turn of the generator (either way) while it presses on the teeth zone
LOAD_MIN = 1.5  # N: a lighter press only turns the generator on the rim
F_DED = 20.0  # N: a straight push (turning under 0.3 rad/s) this hard for 0.25 s forces the teeth a tooth over
F_TILT = 8.0  # N: pressed this hard while tilted over TILT_MAX also forces an eccentric mesh
TILT_MAX = 3.0  # degrees: the generator jams in the rim beyond this
W_TURN = 0.3  # rad/s: slower than this is not turning
LOAD_MAX = 30.0  # N, 0.25 s mean: overload of the generator's thin bearing ("Avoid ... excessive force")
E_MAX = 1.0  # mm: plug centre off the gear axis (the rim allows 0.3 mm)
K_STOP, M_STOP = 10000.0, 0.05
PARK = 13.0  # mm: stop position limit (travel)

STEEL, BRIGHT, DARK, BLACK = (0.62, 0.64, 0.67), (0.80, 0.81, 0.83), (0.24, 0.25, 0.27), (0.08, 0.08, 0.09)
PETG, CRADLE_C, ALU = (0.55, 0.58, 0.62), (0.12, 0.12, 0.13), (0.74, 0.75, 0.77)
QZ = (0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5))  # turns a prism's x axis to z


def ring(r_in, r_out, z0, z1, color, n=16, **kw):
    """n boxes whose inner faces make a regular n-gon of inradius r_in (mm) about the axis, z0..z1 (mm); m out."""
    thick = r_out - r_in
    w = 2 * r_out * math.tan(math.pi / n) * 1.02
    out = []
    for i in range(n):
        a = 2 * math.pi * i / n
        rc = r_in + thick / 2
        out.append(parts.box((thick * MM, w * MM, (z1 - z0) * MM),
                             (rc * MM * math.cos(a), rc * MM * math.sin(a), (z0 + z1) / 2 * MM), color,
                             quat=(0.0, 0.0, math.sin(a / 2), math.cos(a / 2)), **kw))
    return out


def rim_segments(n=16):
    """The flexspline's top 10 mm as n convex wedges with a 1 mm lead-in chamfer at the open end."""
    prof = [(FS_R[0], 50.0), (FS_R[1], 50.0), (FS_R[1], Z_TOP), (FS_R[0] + CHAMFER, Z_TOP), (FS_R[0], Z_TOP - CHAMFER)]
    out = []
    half = math.pi / n
    for i in range(n):
        a = 2 * math.pi * i / n
        pts = []
        for b in (a - half * 1.02, a + half * 1.02):
            c, s = math.cos(b), math.sin(b)
            for r, z in prof:
                rr = r / math.cos(half)  # chord at inradius r
                pts.append((rr * c * MM, rr * s * MM, z * MM))
        m = len(prof)
        faces = [tuple(range(m))[::-1], tuple(range(m, 2 * m))] + [(k, (k + 1) % m, m + (k + 1) % m, m + k)
                                                                     for k in range(m)]
        out.append(parts.hull(pts, faces, BRIGHT, friction=(0.15, 0.12)))  # oiled steel on steel, estimate
    return out


STOP_SHAPES = tuple(ring(11.0, 15.0, Z_TOP - MOUTH - 0.6, Z_TOP - MOUTH, BRIGHT, hidden=True))
STOP = parts.Slider('stop', STOP_SHAPES, axis=(0.0, 0.0, -1.0), travel=PARK * MM, mass=M_STOP, spring=K_STOP,
                    damping=40.0, max_effort=300.0)
JOINT_SHAPES = [
    parts.box((0.120, 0.120, CRADLE * MM), (0.0, 0.0, CRADLE / 2 * MM), CRADLE_C),  # assembly cradle plate
    *ring(HOUSING_R[0], HOUSING_R[1], CRADLE, Z_TOP, PETG),  # printed joint housing
    *ring(CS_R[0], CS_R[1], CS_Z[0], CS_Z[1], DARK),  # circular spline, bolted in the housing's pilot
    *ring(FS_R[0], FS_R[1], FS_Z[0], 50.0, BRIGHT, friction=(0.15, 0.12)),  # flexspline cup below the rim
    *rim_segments(),
    parts.cylinder(FS_R[1] * MM, (FS_Z[0] - 25.0) * MM, (0.0, 0.0, (FS_Z[0] + 25.0) / 2 * MM), BRIGHT),  # diaphragm
    parts.cylinder(25.0 * MM, (25.0 - CRADLE) * MM, (0.0, 0.0, (25.0 + CRADLE) / 2 * MM), DARK),  # output flange
    *[parts.cylinder(1.6 * MM, 0.6 * MM, (22.0 * MM * math.cos(a), 22.0 * MM * math.sin(a), (Z_TOP + 0.3) * MM),
                     BLACK, collide=False) for a in np.arange(8) * math.pi / 4],  # M3 heads on the 44 mm circle
]
JOINT_A = (0.290, -0.090, TOP)
JOINT = parts.Part('joint', pos=JOINT_A, mass=1.6, shapes=JOINT_SHAPES, joints=(STOP,), friction=(0.8, 0.7),
                   link_friction=(('stop', (0.6, 0.5)),))

# ---- the wave generator (its frame: origin at the plug's bottom face centre, axis up, mm) -------------------------
PLUG_R, PLUG_H = 15.2, 8.0
LEAD = 1.2  # plug bottom chamfer
OLD_R, OLD_Z = 11.5, (8.0, 11.0)
HUB_R, HUB_Z = 8.0, (11.0, 17.6)
_N = 24
_lead_pts = [((PLUG_R - LEAD) * math.cos(a) * MM, (PLUG_R - LEAD) * math.sin(a) * MM, 0.0)
             for a in np.arange(_N) * 2 * math.pi / _N] + \
            [(PLUG_R * math.cos(a) * MM, PLUG_R * math.sin(a) * MM, LEAD * MM) for a in np.arange(_N) * 2 * math.pi / _N]
_lead_faces = [tuple(range(_N))[::-1], tuple(range(_N, 2 * _N))] + \
              [(k, (k + 1) % _N, _N + (k + 1) % _N, _N + k) for k in range(_N)]
WG_SHAPES = [
    parts.hull(_lead_pts, _lead_faces, STEEL, friction=(0.15, 0.12)),
    parts.prism(PLUG_R * MM, (PLUG_H - LEAD) * MM, _N, (0.0, 0.0, (PLUG_H + LEAD) / 2 * MM), STEEL, quat=QZ),
    parts.prism(OLD_R * MM, (OLD_Z[1] - OLD_Z[0]) * MM, _N, (0.0, 0.0, sum(OLD_Z) / 2 * MM), BLACK, quat=QZ),
    parts.prism(HUB_R * MM, (HUB_Z[1] - HUB_Z[0]) * MM, _N, (0.0, 0.0, sum(HUB_Z) / 2 * MM), BRIGHT, quat=QZ),
    parts.cylinder(3.0 * MM, 0.4 * MM, (0.0, 0.0, (HUB_Z[1] + 0.05) * MM), BLACK, collide=False),  # 6 mm bore
    parts.box((2.0 * MM, 6.0 * MM, 0.3 * MM), (0.0, 11.0 * MM, (PLUG_H + 0.05) * MM), (0.85, 0.15, 0.12),
              collide=False),  # red index mark on the plug
]
TRAY_FLOOR = 4.0
TRAY_A = (0.195, -0.225, TOP)
WG = parts.Part('wg', pos=(TRAY_A[0], TRAY_A[1], TOP + TRAY_FLOOR * MM), mass=0.030, shapes=WG_SHAPES)
TRAY = parts.Part('tray', pos=TRAY_A, mass=0.40, friction=(0.6, 0.5), shapes=[
    parts.box((0.140, 0.100, TRAY_FLOOR * MM), (0.0, 0.0, TRAY_FLOOR / 2 * MM), ALU),
    parts.box((0.140, 0.004, 0.015), (0.0, 0.048, 0.0075), ALU),
    parts.box((0.140, 0.004, 0.015), (0.0, -0.048, 0.0075), ALU),
    parts.box((0.004, 0.092, 0.015), (0.068, 0.0, 0.0075), ALU),
    parts.box((0.004, 0.092, 0.015), (-0.068, 0.0, 0.0075), ALU),
])
# ---- the rest of the bench (fixed): the joint's stepper motor on its adapter plate, the joint's cover ------------
BENCH = parts.Part('bench', pos=(0.540, 0.020, TOP), shapes=[
    parts.box((0.060, 0.060, 0.008), (0.0, 0.0, 0.004), PETG),  # printed motor adapter plate
    parts.box((0.042, 0.042, 0.048), (0.0, 0.0, 0.032), BLACK),  # NEMA 17 stepper, 42 x 42 x 48 mm
    parts.cylinder(0.0025, 0.020, (0.0, 0.0, 0.066), BRIGHT),  # 5 mm shaft
    parts.cylinder(0.036, 0.020, (0.0, -0.110, 0.010), PETG),  # printed joint cover
])

OBJECTS = [JOINT, WG, TRAY, BENCH]
# The housing (an articulated part: the mesh stop is a joint) is never moved between lots; lot B moves the tray.
TRAY_B = (0.385, -0.215, TOP)
LOTS = lots.declare(
    OBJECTS,
    lots.Lot('A', place={'tray': TRAY_A, 'wg': WG.pos}),  # zero move: settles like lot B
    lots.Lot('B', place={'tray': TRAY_B, 'wg': (TRAY_B[0], TRAY_B[1], TOP + TRAY_FLOOR * MM)},
             yaw={'wg': math.pi / 2}),
)
DEMO_TOUCHES = {'wg', 'tray'}

# ---- geometry helpers --------------------------------------------------------------------------------------------


def rot(st):
    return parts._quat_to_matrix(st.quat_xyzw)


def wg_dist(p):
    """Distance (m) from points in the generator frame to the generator (plug, coupling disc and hub)."""
    p = np.asarray(p, float)
    if not len(p):
        return np.inf
    r = np.hypot(p[:, 0], p[:, 1])
    out = np.full(len(p), np.inf)
    for z0, z1, rad in ((0.0, PLUG_H, PLUG_R), OLD_Z + (OLD_R,), HUB_Z + (HUB_R,)):
        dz = np.maximum(np.maximum(z0 * MM - p[:, 2], p[:, 2] - z1 * MM), 0.0)
        out = np.minimum(out, np.hypot(np.maximum(r - rad * MM, 0.0), dz))
    return float(out.min())


def fresh():
    return dict(known=False, fail='', ded=False, front=MOUTH, phi=None, loads=[], load=0.0, peak=0.0, e=0.0,
                tilt=0.0, d=-99.0, prev_t=None, hold=0.0, anchor=None, released=False, turned=0.0, joint0=None,
                spin=0.0, why='', slow=0.0, events=0)


REC = fresh()


def observe(objects, hands, t, q_stop):
    """Update the trial record from one control step. ``hands``: per-arm world point clouds (None: unknown)."""
    r = REC
    if r['prev_t'] is not None and t <= r['prev_t'] + EPS:
        return
    dt = 0.05 if r['prev_t'] is None else min(0.1, max(0.0, t - r['prev_t']))
    r['prev_t'] = float(t)
    if hands is None or q_stop is None or not all(k in objects for k in ('joint', 'wg')):
        r['known'] = False
        r['fail'] = r['fail'] or 'hand or object state unavailable'
        r['hold'] = 0.0
        return
    r['known'] = True
    js, ws = objects['joint'], objects['wg']
    Rj, pj, Rw, pw = rot(js), np.asarray(js.pos), rot(ws), np.asarray(ws.pos)
    if r['joint0'] is None:
        r['joint0'] = (pj.copy(), Rj.copy())
    twist = (Rj @ r['joint0'][1].T)
    if np.linalg.norm(pj - r['joint0'][0]) > 0.005 or Rj[2, 2] < math.cos(math.radians(5)) \
            or abs(math.atan2(twist[1, 0], twist[0, 0])) > math.radians(5.0):
        r['fail'] = r['fail'] or 'joint housing knocked out of its cradle position'
    to_j = Rj.T @ Rw
    axis = to_j[:, 2]
    base = Rj.T @ (pw - pj)  # plug bottom centre, housing frame (m)
    r['tilt'] = float(math.degrees(math.acos(max(-1.0, min(1.0, axis[2])))))
    r['e'] = float(np.hypot(base[0], base[1]) / MM)
    r['d'] = float(Z_TOP - base[2] / MM)  # plug bottom depth below the circular spline face (mm)
    # The generator's turn about its own axis, relative to the housing (either way counts).
    xv = to_j[:, 0]
    phi = math.atan2(xv[1], xv[0])
    turn = 0.0
    if r['phi'] is not None:
        turn = abs((phi - r['phi'] + math.pi) % (2 * math.pi) - math.pi)
    r['phi'] = phi
    # The stop's target is still last step's front here, so its deflection is the press on the plug (N).
    load = K_STOP * (q_stop - (r['front'] - MOUTH) * MM) - M_STOP * 9.81
    upright = r['tilt'] < 30.0
    engaged = upright and r['e'] <= E_MAX + EPS and r['d'] >= r['front'] - 0.5 and r['d'] > MOUTH - 0.5
    load = float(load) if engaged else 0.0
    r['load'] = load
    r['loads'] = (r['loads'] + [load])[-5:]
    mean = float(np.mean(r['loads']))
    r['peak'] = max(r['peak'], mean)
    if mean > LOAD_MAX + EPS:
        r['fail'] = r['fail'] or 'wave generator bearing overloaded (over 30 N)'
    spin = float(np.asarray(ws.ang_vel) @ Rw[:, 2]) - float(np.asarray(js.ang_vel) @ Rw[:, 2])
    r['spin'] = spin
    on_teeth = engaged and r['front'] < HOME - EPS
    # Withdrawn: the flexspline springs back round; the mesh starts again from the mouth.
    if r['d'] < 0.5 - EPS:
        if r['front'] > MOUTH + EPS or r['ded']:
            r['loads'] = []
        r['front'], r['ded'] = MOUTH, False
    if on_teeth and not r['ded']:
        if mean >= F_DED - EPS and abs(spin) < W_TURN:
            r['slow'] += dt
        else:
            r['slow'] = 0.0
        if r['slow'] >= 0.25 - EPS or (r['tilt'] > TILT_MAX + EPS and mean >= F_TILT - EPS):
            r['ded'] = True  # the flexspline's teeth are forced a tooth over on one side
            r['front'] = HOME
            r['loads'] = []
            r['events'] += 1
        elif turn > 0 and load >= LOAD_MIN - EPS and r['tilt'] <= TILT_MAX + EPS:
            step = FEED * turn / (2 * math.pi)
            r['front'] = float(min(HOME, r['front'] + step, max(r['front'], r['d'] + 0.2)))
            r['turned'] += turn
    # Hands: never on the gear teeth below the circular spline face; released and clear for the final hold.
    hand_wg = np.inf
    for pts in hands.values():
        if not len(pts):
            continue
        pts = np.asarray(pts)
        hj = (pts - pj) @ Rj
        rr = np.hypot(hj[:, 0], hj[:, 1])
        if np.any((rr < FS_R[1] * MM) & (hj[:, 2] < (Z_TOP - 0.5) * MM) & (hj[:, 2] > FS_Z[0] * MM)):
            r['fail'] = r['fail'] or 'gripper on the gear teeth inside the flexspline'
        hw = (pts - pw) @ Rw
        near = hw[np.linalg.norm(hw, axis=1) < 0.2]
        hand_wg = min(hand_wg, wg_dist(near))
    r['released'] = bool(hand_wg >= 0.020 - EPS)
    # Dropped: falling faster than 0.4 m/s (a 10 mm free fall) with no gripper on it, anywhere but inside the gear
    # (the eccentric mesh lets a pressed plug drop home inside the rim; that is scored by its own rule).
    out_of_gear = r['e'] > E_MAX + EPS or r['d'] < MOUTH - 0.5
    if float(np.asarray(ws.lin_vel)[2]) < -0.40 and hand_wg >= 0.005 - EPS and out_of_gear:
        r['fail'] = r['fail'] or 'wave generator dropped'
    home = (not r['ded']) and r['front'] >= HOME - EPS and r['d'] >= HOME - 0.5 and r['e'] <= E_MAX + EPS \
        and r['tilt'] <= TILT_MAX + EPS
    marks = np.array([pw, pw + Rw @ np.array([0.0, 0.0, 0.0176]), pw + Rw @ np.array([0.0152, 0.0, 0.004]), pj])
    still = objects['wg'].resting(0.02, 0.5) and objects['joint'].resting(0.02, 0.5)
    if not (home and not r['fail'] and r['released'] and still):
        r['anchor'], r['hold'] = None, 0.0
    elif r['anchor'] is None or np.linalg.norm(marks - r['anchor'], axis=1).max() > 0.001 + EPS:
        r['anchor'], r['hold'] = marks.copy(), 0.0
    else:
        r['hold'] += dt
    r['home'] = bool(home)


def score(objects):
    r = REC
    if not r['known'] or r['fail'] or r['ded'] or r['d'] < MOUTH - 0.5 or r['e'] > E_MAX + EPS or r['tilt'] > 30.0:
        return 0.0
    if r.get('home'):
        return 1.0 if r['hold'] >= 1.0 - EPS else 0.75
    depth = min(r['front'], r['d'] + 0.5)  # where the gear holds the plug, if the plug is there
    if depth >= MOUTH + (HOME - MOUTH) / 2 - EPS:
        return 0.5
    if depth >= MOUTH + 1.0 - EPS:
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
            ring_ = on[np.argsort(np.arctan2((on - c) @ v, (on - c) @ u))]
            for a, b in zip(ring_, np.roll(ring_, -1, axis=0)):
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


@parts.on_reset
def _reset(sim):
    global REC
    REC = fresh()
    sim.set_target('joint', 'stop', 0.0)


@parts.on_step
def _step(sim):
    try:
        objects = sim.states()
        hands, tcp = hand_clouds(sim)
        q = objects['joint'].joints.get('stop') if hasattr(objects['joint'], 'joints') else None
        observe(objects, hands, float(sim.t), None if q is None else float(q))
        sim.set_target('joint', 'stop', (REC['front'] - MOUTH) * MM)
        if os.environ.get('TRACE_OUT') and round(sim.t * 20) % 2 == 0:
            ws = objects['wg']
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, front=round(REC['front'], 3),
                       d=round(REC['d'], 3), load=round(REC['load'], 3), q=q, e=round(REC['e'], 3),
                       tilt=round(REC['tilt'], 3), ded=REC['ded'], fail=REC['fail'], hold=REC['hold'],
                       score=score(objects), released=REC['released'], turned=round(REC['turned'], 3),
                       wg=np.round(ws.pos, 5).tolist(), wgq=np.round(ws.quat_xyzw, 5).tolist(),
                       joint=np.round(objects['joint'].pos, 5).tolist(), tcp=tcp, phi=REC['phi'],
                       spin=round(REC.get('spin', 0.0), 3),
                       peak=round(REC['peak'], 3))
            with Path(os.environ['TRACE_OUT']).with_name('wave.jsonl').open('a') as f:
                f.write(json.dumps(row) + '\n')
    except Exception as e:  # fail closed
        REC['known'] = False
        REC['fail'] = REC['fail'] or 'scorer error'
        if os.environ.get('TRACE_OUT'):
            with Path(os.environ['TRACE_OUT']).with_name('wave-errors.txt').open('a') as f:
                f.write(repr(e) + '\n')
        raise


# ---- demo: open loop from the visible layout ---------------------------------------------------------------------
CAL = json.loads((HERE / 'fixture' / 'calibration.json').read_text())
Y_HI, Y_LO = math.radians(60.0), math.radians(-60.0)
GRIP = -0.004  # fingers 4 mm past each side of the 16 mm hub
OPEN = -0.020  # fingers 40 mm apart: clear of the hub and the coupling disc, well inside the tray's rims
TCP_ABOVE_PLUG = 0.025  # TCP height above the plug's bottom face in the grip (finger tips 1 mm over the coupling disc)
PRELOAD = 6.0  # mm the plug is commanded below the mesh front (the arm is soft: about 3 to 4 N, trace 1)
DESCENT = 1.33  # mm per 120-degree stroke (the rule feeds 1.33 mm)
N_STROKES = 10


def demo():
    lot = lots.current()
    jp = np.asarray(lot.pos('joint'))
    wp = np.asarray(lot.pos('wg'))
    arm = 'right'
    other = 'left'
    cal = CAL[arm]
    g = np.asarray(cal['grasp'])
    b = np.asarray(cal['bore'])

    def W(d, p, f, label, yaw=0.0):
        return Waypoint(float(d), tuple(float(v) for v in p), float(f), label, yaw=float(yaw))

    def tcp_at(depth_mm):
        """TCP over the gear axis with the plug bottom ``depth_mm`` below the circular spline face."""
        return np.array([jp[0], jp[1], jp[2] + (Z_TOP - depth_mm) * MM + TCP_ABOVE_PLUG]) + b

    at = wp + np.array([0.0, 0.0, TCP_ABOVE_PLUG]) + g
    above = at + np.array([0.0, 0.0, 0.030])
    travel = jp[2] + (Z_TOP + 12.0) * MM + TCP_ABOVE_PLUG
    seq = [W(2.5, above, OPEN, 'over the wave generator'),
           W(2.0, at, OPEN, 'fingers round the hub'),
           W(1.0, at, GRIP, 'grip the hub'),
           W(1.5, (at[0], at[1], travel), GRIP, 'lift the generator'),
           W(2.5, tcp_at(-12.0), GRIP, 'over the flexspline'),
           W(1.5, tcp_at(-3.0), GRIP, 'lower to the open end'),
           W(1.5, tcp_at(-3.0), GRIP, 'square over the rim'),
           W(1.5, tcp_at(MOUTH - 0.3), GRIP, 'plug into the lead-in'),
           W(1.5, tcp_at(MOUTH + PRELOAD), GRIP, 'press lightly on the teeth')]
    depth = MOUTH + PRELOAD
    yaw = 0.0
    targets = [Y_LO, Y_HI]
    if os.environ.get('DEMO_ROUTE') == 'straight':  # the naive route, for the bypass table: push, never turn
        seq += [W(4.0, tcp_at(HOME + 3 * PRELOAD), GRIP, 'push straight down, no turning'),
                W(4.0, tcp_at(HOME + 3 * PRELOAD), GRIP, 'keep pushing')]
        N = 0
    else:
        N = N_STROKES
    for k in range(N):
        depth = min(HOME + PRELOAD, depth + DESCENT)
        yaw = targets[k % 2]
        seq.append(W(0.9 if k else 0.6, tcp_at(depth), GRIP, f'turn it in, stroke {k + 1}', yaw))
    seq += [W(1.0, tcp_at(HOME + 0.3), GRIP, 'ease the press off at home', yaw),
            W(1.0, tcp_at(HOME + 0.3), OPEN, 'release the hub', yaw),
            W(1.5, tcp_at(-15.0), OPEN, 'lift clear', yaw),
            W(2.0, tcp_at(-15.0) + np.array([-0.080, -0.040, 0.020]), FINGER_OPEN, 'stand clear', 0.0),
            W(2.5, tcp_at(-15.0) + np.array([-0.080, -0.040, 0.020]), FINGER_OPEN, 'hold still', 0.0)]
    total = sum(w.duration for w in seq)
    rest = [W(total, (0.20, 0.30, TOP + 0.16), FINGER_OPEN, 'stay clear', 0.0)]
    return {arm: seq, other: rest}
