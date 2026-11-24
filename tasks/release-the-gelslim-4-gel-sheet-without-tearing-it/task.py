"""A fabrication counter with a cured GelSlim 4.0 gel mold. The mold is a black printed hexagonal outer ring with a
clear acrylic hexagon seated on its bottom ledge; a pale pink 3 mm cured silicone gel sheet fills the ring above the
acrylic. The acrylic's back is exposed through the ring's bottom opening. Beside the mold stands an upturned clear
glass crystallizing dish, narrower than that opening, used as a push-out stand. A millimetre rule lies at the back of
the counter, in front of a grey upstand.
"""
from __future__ import annotations
import hashlib, json, math, os, pathlib, sys
SOURCE_SHA256 = hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()
from types import SimpleNamespace
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts, lots, tilt
from fixture import pick
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint, FINGER_OPEN, FINGER_CLOSED

SCENE = 'station'; TOP = TABLE_TOP[SCENE]; SCORE_OVER = 'end'; SUCCESS_HOLD_S = .05
EPS = 1e-7
# Native mold (original outer_ring.STL / hex_acrylic_base.DXF), metres.
R = .08505256; RO = .09659956; AF = RO * math.sqrt(3) / 2; H = .003; BH = .00635
N = 7; W = 2 * R / N
GEL_MASS = .077
# Phone figures: technician who works with this product, Nov 2026 (READINGS in revision evidence).
STRIP_FORCE = .5   # N per 24 mm strip WIDTH, not length in the peel direction
STRIP_WIDTH = .024
PEEL_BOUND = 1.2   # declared engineering envelope on proportional width extrapolation; enrollment required
PEEL_RAMP = .0004  # m numerical cohesive ramp, not a measured material stiffness
OPEN = .001       # m: a band at the peel front lets go once its underside centre is opened 1 mm ...
PEEL_LOAD_FACTOR = 1.01 # retain the1% extra resistance; no lagged velocity switch
PEEL_ENERGY = PEEL_BOUND * STRIP_FORCE / STRIP_WIDTH  # J/m²: inferred90deg peel-area work,25J/m²
TOUCH_N = .05     # N: ... done while a robot link touches that band or the freed flap joined to it
BEND_K = .001    # N m/rad zero-rest bending, uncalibrated
BEND_DAMP = .0005 # N m s/rad, uncalibrated
BEND_LIMIT = .025 # N m effort cap, uncalibrated
GRIP = 300.       # N/m: a bonded band is also held where it was cast (0.9 N at the 3 mm tear limit)
YANK = .005       # a front band lifted 5 mm before it lets go tears (pulled too fast)
HOLE = .0025      # a bonded band that is not at the front, lifted 2.5 mm, tears (pulled from the middle)
SHEAR = .003      # a bonded band dragged 3 mm along the acrylic tears
CRUSH = 15.0      # N summed robot contact on one band crushes/cuts it
RING_MASS = .14
FIT_TOUCH = 7.0   # N actual robot-applied downward load, nominal standard ring (phone: 5-15 N)
FIT = FIT_TOUCH + RING_MASS * 9.81  # counter ring gravity: robot still supplies at least 7 N
FIT_TRAVEL = .002
FIT_WORK = FIT_TOUCH * FIT_TRAVEL  # J, supplied robot load over the whole release travel
# Layout: the assembled mold is in the same place in both layouts (its gel is a jointed part); the stand moves.
MX, MY = .380, 0.
STAND_R, STAND_H = .045, .050
STAND_XY = {'A': (.200, 0.), 'B': (.226, 0.)}
LAY = {'A': (.270, .200), 'B': (.270, .200)}   # where the demo lays the sheet (centre); any flat free place scores
BLACK = (.16, .17, .19); CLEAR = (.65, .82, .86); GEL = (.92, .70, .68); GLASS = (.80, .88, .90)


def prism(poly, z0, z1, color, friction=None):
    p = [(float(x), float(y), float(z)) for z in (z0, z1) for x, y in poly]; n = len(poly)
    fs = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))] + [(j, (j + 1) % n, (j + 1) % n + n, j + n) for j in range(n)]
    return parts.hull(p, fs, color, friction=friction)
def hexagon(r): return [(r * math.cos(i * math.pi / 3), r * math.sin(i * math.pi / 3)) for i in range(6)]
def clip(poly, k, edge, keep_high):
    out = []
    for p, q in zip(poly, poly[1:] + poly[:1]):
        pin = p[k] >= edge if keep_high else p[k] <= edge
        qin = q[k] >= edge if keep_high else q[k] <= edge
        if pin: out.append(p)
        if pin != qin:
            a = (edge - p[k]) / (q[k] - p[k]); out.append(tuple(p[j] + a * (q[j] - p[j]) for j in range(2)))
    return out


def ring_shapes():
    shapes = []
    # Native section: bottom ledge 0-5 mm; cavity 5-14.35; relief 14.35-15.35; upper wall 15.35-18 mm.
    for z0, z1, ri in [(0, .005, .06773205), (.005, .01435, R + .0002), (.01435, .01535, .08620725), (.01535, .018, R + .0002)]:
        outer, inner = hexagon(RO), hexagon(ri)
        for i in range(6):
            shapes.append(prism([inner[i], outer[i], outer[(i + 1) % 6], inner[(i + 1) % 6]], z0, z1, BLACK))
    return shapes
# PLA print on a rubber fingertip / laminate: friction an estimate.
RING = parts.Part('outer_ring', pos=(MX, MY, TOP), mass=RING_MASS, shapes=ring_shapes(), friction=(.7, .6), contact_offset=.0002, report=('base',))
BASE = parts.Part('acrylic_base', pos=(MX, MY, TOP + .005 + BH / 2), mass=.145, shapes=(prism(hexagon(R), -BH / 2, BH / 2, CLEAR),),
                  friction=(.45, .4), contact_offset=.0002)
STRIPS = []; AREAS = []
for k in range(N):
    poly = clip(clip(hexagon(R), 0, -R + k * W + (.00003 if k else 0.), True), 0, -R + (k + 1) * W - (.00003 if k < N - 1 else 0.), False)
    AREAS.append(abs(sum(p[0] * q[1] - p[1] * q[0] for p, q in zip(poly, poly[1:] + poly[:1]))) / 2)
    STRIPS.append((prism(poly, -H / 2, H / 2, GEL),))
# Max transverse front width anywhere within each band. This deliberately bounds the varying hexagon front.
FRONT_WIDTH = np.array([max(y for x, y in clip(clip(hexagon(R), 0, -R + k * W, True), 0, -R + (k + 1) * W, False)) * 2 for k in range(N)])
FRONT_FORCE = PEEL_BOUND * STRIP_FORCE * FRONT_WIDTH / STRIP_WIDTH
NOMINAL_AREAS=[]
for k in range(N):
    poly=clip(clip(hexagon(R),0,-R+k*W,True),0,-R+(k+1)*W,False)
    NOMINAL_AREAS.append(abs(sum(p[0]*q[1]-p[1]*q[0] for p,q in zip(poly,poly[1:]+poly[:1])))/2)
FRONT_WORK = PEEL_ENERGY * np.array(NOMINAL_AREAS)
LINK = ['base'] + [f's{i}' for i in range(1, N)]
GEL_SHEET = parts.Part('gel_sheet', pos=(MX, MY, TOP + .005 + BH + H / 2), mass=GEL_MASS * AREAS[0] / sum(AREAS), shapes=STRIPS[0],
    joints=tuple(parts.Hinge(LINK[k], shapes=STRIPS[k], pivot=(-R + k * W, 0., 0.), axis=(0., 1., 0.), lower=-2.6, upper=2.6,
                             mass=GEL_MASS * AREAS[k] / sum(AREAS), spring=BEND_K, damping=BEND_DAMP, max_effort=BEND_LIMIT, parent=LINK[k - 1]) for k in range(1, N)),
    friction=(.7, .55), contact_offset=.00015, report=tuple(LINK))
# Upturned PYREX 3140 crystallizing dish, 90 mm x 50 mm (glass; mass an estimate). A 24-sided prism stands in for the round wall.
DISH = [(STAND_R * math.cos(2 * math.pi * i / 24), STAND_R * math.sin(2 * math.pi * i / 24)) for i in range(24)]
STAND = parts.Part('push_out_stand', pos=(*STAND_XY['A'], TOP + STAND_H / 2), mass=.19, shapes=(prism(DISH, -STAND_H / 2, STAND_H / 2, GLASS),),
                   friction=(.45, .4), contact_offset=.0002)
RULER = parts.Part('millimetre_rule', pos=(.56, -.22, TOP + .0005), shapes=(parts.box((.15, .018, .001), color=(.9, .9, .85)),), friction=(.4, .3))
COUNTER_BACK = parts.Part('counter_upstand', pos=(.72, 0., TOP + .055), shapes=(parts.box((.018, 1.3, .11), color=(.65, .66, .66)),))
PICK_POS = (.35, -.22, TOP + .013)
# Fixed native grasp calibration from an unloaded pickup; no state is read by demo().
PICK_GRASP_APEX = np.array([.000015,-.017095,.004899])
PICK = parts.Part('opening_pick', pos=PICK_POS, mass=.022,
    shapes=(parts.hull(pick.VERTICES, pick.FACES, (.08,.25,.85)), parts.box(pick.CLIP_SIZE, at=pick.CLIP_AT, color=(.3,.3,.35)))+tuple(parts.box(pick.RAIL_SIZE,at=p,color=(.23,.23,.28)) for p in pick.RAIL_AT),
    friction=(.6,.5), contact_offset=.00005, report=('base',))
OBJECTS = [RING, BASE, GEL_SHEET, STAND, RULER, COUNTER_BACK, PICK]
LOTS = lots.declare(OBJECTS, lots.Lot('A'), lots.Lot('B', place={'push_out_stand': (*STAND_XY['B'], TOP + STAND_H / 2)}))
DEMO_TOUCHES = set()
SCORED = (RING, BASE, GEL_SHEET, PICK)


def fresh():
    return dict(bond=np.ones(N, bool), delta=None, bond_points=None, bond_gap=np.zeros(N), contact_points=np.full((N,3),np.nan), contact_weights=np.zeros(N), contact_motion=np.zeros((N,3)), brake_f=np.zeros((N,3)), brake_body=np.full(N,-1,int), brake_point=np.zeros((N,3)), brake_local=np.zeros((N,3)), brake_base_local=np.zeros((N,3)), work=np.zeros(N), gap=np.zeros(N), touch=np.zeros(N, bool), ring_touch=0., ring_press=0., tool_grip=0., fit_prev=0., fit_loaded=0., fit_work=0., contact_prev=None, robot_work=np.zeros(N), fvec=np.zeros((N, 3)), zero=False, why='', known=False, steps=0, prev=None,
                still=None, clear=None, t=None, views=None, peak=0., peak_by_band=np.zeros(N), seated=True, phys=0, events=[], arms=None)
S = fresh()
def fail(reason):
    if not S['zero']: S.update(zero=True, why=str(reason))
def event(text):
    if len(S['events']) < 40: S['events'].append(text)
def poses(s):
    p = {'base': (np.array(s.pos), parts._quat_to_matrix(s.quat_xyzw))}
    p.update({n: (np.array(v[0]), parts._quat_to_matrix(v[1])) for n, v in getattr(s, 'links', {}).items()})
    return p


def gel_points(s):
    """World position of the centre of each band's underside."""
    pp = poses(s); out = []
    for k, n in enumerate(LINK):
        p, r = pp[n]; out.append(p + r @ (np.array((-R + (k + .5) * W, 0., -H / 2)) - GEL_SHEET.joint_origin(n)))
    return np.array(out)


def box_gap(points, c, r, size): return np.linalg.norm(np.maximum(np.abs((points - np.asarray(c)) @ r) - np.asarray(size) / 2, 0), axis=1)
def spec_gap(points, spec, st):
    return float(min(box_gap(points, c, r, size).min() for _, c, r, size in spec.world_boxes(poses(st))))
# Sparse, face-sampled hand hulls; includes large faces, not only vertices.
_HAND = {}
def local_hand(n):
    if n not in _HAND:
        from yam_arena import analysis
        chunks = []
        for h in analysis.link_hulls()[n]:
            chunks.append(h.vertices)
            _, plane_ids = np.unique(np.round(h.planes, 6), axis=0, return_index=True)
            for normal, dd in zip(h.planes[plane_ids, :3], h.planes[plane_ids, 3]):
                face = h.vertices[np.abs(h.vertices @ normal - dd) < 1e-6]
                if len(face) < 3: continue
                c = face.mean(0); u = face[np.argmax(np.linalg.norm(face - c, axis=1))] - c
                if np.linalg.norm(u) < 1e-12: continue
                u /= np.linalg.norm(u); v = np.cross(normal, u)
                uv = np.column_stack(((face - c) @ u, (face - c) @ v)); order = np.lexsort((uv[:, 1], uv[:, 0]))
                def turn(i, j, k):
                    aa = uv[j] - uv[i]; bb = uv[k] - uv[j]; return aa[0] * bb[1] - aa[1] * bb[0]
                lo = []; hi = []
                for ii_ in order:
                    while len(lo) > 1 and turn(lo[-2], lo[-1], ii_) <= 1e-12: lo.pop()
                    lo.append(int(ii_))
                for ii_ in order[::-1]:
                    while len(hi) > 1 and turn(hi[-2], hi[-1], ii_) <= 1e-12: hi.pop()
                    hi.append(int(ii_))
                face = face[lo[:-1] + hi[:-1]]
                for aa, bb in zip(face, np.roll(face, -1, axis=0)):
                    nn = max(1, math.ceil(max(np.linalg.norm(aa - c), np.linalg.norm(bb - c), np.linalg.norm(bb - aa)) / .004))
                    ii, jj = np.meshgrid(np.arange(nn + 1), np.arange(nn + 1), indexing='ij')
                    valid = ii + jj <= nn
                    chunks.append(c + (aa - c) * ii[valid, None] / nn + (bb - c) * jj[valid, None] / nn)
        all_pts = np.concatenate(chunks)
        _, idx = np.unique(np.round(all_pts / .002).astype(np.int64), axis=0, return_index=True)
        _HAND[n] = all_pts[np.sort(idx)]
    return _HAND[n]
def hands(sim):
    base = sim.env.unwrapped; org = parts._np(base.scene.env_origins[0]); out = []
    for arm in ('left_robot', 'right_robot'):
        a = base.scene[arm]; p = parts._np(a.data.body_link_pose_w)[0]
        for n in ('link5', 'gripper', 'tip_left', 'tip_right'):
            st = p[list(a.body_names).index(n)]; out.append(local_hand(n) @ parts._quat_to_matrix(st[3:]).T + st[:3] - org)
    out = np.concatenate(out)
    if not np.isfinite(out).all(): raise ValueError('unknown arm state')
    return out


def hands_clear(h, o):
    # 38 mm numeric margin: an 8 mm sampling guard on the stated 30 mm clearance.
    for spec in SCORED:
        st = o[spec.name]; near = h[np.all((h >= st.low - .039) & (h <= st.high + .039), axis=1)]
        if len(near) and spec_gap(near, spec, st) < .038: return False
    return True


_TOPO = {}
def surface_polys(spec, st):
    out = []; pp = poses(st)
    for link, origin, shapes in spec.links:
        p, r = pp[link]
        for sh in shapes:
            if sh.kind != 'hull': raise ValueError('clearance requires complete convex surface geometry')
            key = id(sh)
            if key not in _TOPO:
                v = np.asarray(sh.points, float); centre = v.mean(0); faces = []; normals = []; edges = set()
                for indices in sh.faces:
                    ids = list(indices); f = v[ids]; normal = None
                    for j in range(1, len(f) - 1):
                        n = np.cross(f[j] - f[0], f[j + 1] - f[0]); ln = np.linalg.norm(n)
                        if ln > 1e-12: normal = n / ln; break
                    if normal is None: continue
                    if np.dot(normal, f.mean(0) - centre) < 0: normal = -normal; ids = ids[::-1]
                    faces.append(np.asarray(ids, int)); normals.append(normal)
                    for aa, bb in zip(ids, ids[1:] + ids[:1]):
                        if np.linalg.norm(v[aa] - v[bb]) > 1e-12: edges.add(tuple(sorted((aa, bb))))
                _TOPO[key] = (v, faces, np.asarray(normals), np.asarray(sorted(edges), int))
            v, faces, normals, edges = _TOPO[key]; wv = p + (v - origin) @ r.T
            out.append((wv, faces, normals @ r.T, edges))
    return out


def poly_gap(pa, pb):
    a, af, an, ae = pa; b, bf, bn, be = pb
    au = a[ae[:, 1]] - a[ae[:, 0]]; bu = b[be[:, 1]] - b[be[:, 0]]
    cross = np.cross(au[:, None, :], bu[None, :, :]).reshape(-1, 3)
    axes = np.concatenate((an, bn, cross)); lens = np.linalg.norm(axes, axis=1); axes = axes[lens > 1e-10] / lens[lens > 1e-10, None]
    ap = a @ axes.T; bp = b @ axes.T
    gap = np.maximum(bp.min(0) - ap.max(0), ap.min(0) - bp.max(0)).max()
    if gap <= 1e-10: return 0.
    if gap >= .005: return float(gap)  # sufficient certified lower bound for the 5 mm predicate
    def vertex_edge(v, poly, edges):
        aa = poly[edges[:, 0]]; u = poly[edges[:, 1]] - aa; d = v[:, None, :] - aa[None, :, :]
        t = np.clip(np.sum(d * u[None, :, :], axis=2) / np.sum(u * u, axis=1)[None, :], 0, 1)
        return float(np.linalg.norm(d - t[:, :, None] * u[None, :, :], axis=2).min())
    best = min(vertex_edge(a, b, be), vertex_edge(b, a, ae))
    for v, poly, faces, normals in ((a, b, bf, bn), (b, a, af, an)):
        for ids, n in zip(faces, normals):
            f = poly[ids]; distance = (v - f[0]) @ n; projected = v - distance[:, None] * n
            e = np.roll(f, -1, axis=0) - f
            inside = np.all(np.sum(np.cross(e[None, :, :], projected[:, None, :] - f[None, :, :]) * n, axis=2) >= -1e-10, axis=1)
            if inside.any(): best = min(best, float(np.abs(distance[inside]).min()))
    aa = a[ae[:, 0]][:, None, :]; bb = b[be[:, 0]][None, :, :]; u = au[:, None, :]; v = bu[None, :, :]; w = aa - bb
    ac = np.sum(u * u, axis=2); bc = np.sum(u * v, axis=2); cc = np.sum(v * v, axis=2); dd = np.sum(u * w, axis=2); ee = np.sum(v * w, axis=2)
    denom = ac * cc - bc * bc; valid = denom > 1e-16; safe = np.where(valid, denom, 1.)
    ss = (bc * ee - cc * dd) / safe; tt = (ac * ee - bc * dd) / safe
    valid &= (ss >= 0) & (ss <= 1) & (tt >= 0) & (tt <= 1)
    if valid.any(): best = min(best, float(np.linalg.norm(w + ss[:, :, None] * u - tt[:, :, None] * v, axis=2)[valid].min()))
    return best


def complete_clear(a, sa, b, sb):
    for pa in surface_polys(a, sa):
        av = pa[0]
        for pb in surface_polys(b, sb):
            bv = pb[0]; delta = np.maximum(np.maximum(av.min(0) - bv.max(0), bv.min(0) - av.max(0)), 0)
            if np.linalg.norm(delta) >= .005 - EPS: continue
            if poly_gap(pa, pb) < .005 - EPS: return False
    return True


# A scored part must leave the whole mold's outer volume, including its cavity and bottom opening.
RING_ENVELOPE = parts.Part('clearance_volume', pos=RING.pos, shapes=(prism(hexagon(RO), 0., .018, BLACK),))
def separated(o): return complete_clear(GEL_SHEET, o['gel_sheet'], BASE, o['acrylic_base'])
def ring_clear(o):
    return bool(complete_clear(BASE, o['acrylic_base'], RING_ENVELOPE, o['outer_ring']) and
                complete_clear(GEL_SHEET, o['gel_sheet'], RING_ENVELOPE, o['outer_ring']))


def bottoms(spec, st): return [float(c[2] - np.abs(r[2]) @ (size / 2)) for _, c, r, size in spec.world_boxes(poses(st))]
def level(st, deg=10.): return all(r[2, 2] >= math.cos(math.radians(deg)) for _, r in poses(st).values())
def gel_flat(st):
    # Every band on the counter: lowest point within 1.5 mm of it, tilted 10 degrees or less, top within 4.5 mm.
    inside = bool(st.low[0] >= -.185 and st.high[0] <= .905 and st.low[1] >= -.645 and st.high[1] <= .645 and st.high[2] <= TOP + H + .0015)
    return bool(inside and level(st) and all(TOP - .002 <= v <= TOP + .0015 for v in bottoms(GEL_SHEET, st)) and st.resting(.008, .12))
def ring_down(st):
    b = bottoms(RING, st); return bool(on_counter(st) and level(st) and TOP - .002 <= min(b) <= TOP + .0015 and st.resting(.008, .12))
def on_counter(st):
    return bool(st.low[0] >= -.185 and st.high[0] <= .905 and st.low[1] >= -.645 and st.high[1] <= .645)
def base_down(st, stand):
    b = min(bottoms(BASE, st))
    if not level(st) or not st.resting(.008, .12): return False
    counter = on_counter(st) and TOP - .002 <= b <= TOP + .0015
    support = False
    if stand is not None and level(stand, 2.) and stand.resting(.008, .12) and on_counter(stand):
        # Dish fully on counter, base COM over central 20 mm of its actual circular top;
        # require the entire 90 mm support disk inside the hexagonal acrylic footprint.
        sc = np.asarray(stand.pos); sr = parts._quat_to_matrix(st.quat_xyzw)
        local = (sc - st.pos) @ sr
        normals = np.array([(math.cos((i + .5) * math.pi / 3), math.sin((i + .5) * math.pi / 3)) for i in range(6)])
        contained = np.all(normals @ local[:2] + STAND_R <= R * math.sqrt(3) / 2 + EPS)
        aligned = float(sr[:,2] @ parts._quat_to_matrix(stand.quat_xyzw)[:,2]) >= math.cos(math.radians(2.))
        support = (np.linalg.norm(local[:2]) <= .020 and contained and aligned and
                   TOP - .002 <= min(bottoms(STAND, stand)) <= TOP + .0015 and
                   float(stand.high[2]) - .002 <= b <= float(stand.high[2]) + .0015)
    return bool(counter or support)


def evaluate(o):
    if not S['known'] or S['zero'] or S['seated'] or S['steps'] == 0 or S['phys'] == 0: return 0.
    if not ring_clear(o): return 0.
    clear1 = S['clear'] is not None and S['t'] - S['clear'] >= 1. - EPS
    held = S['clear'] is not None and S['still'] is not None and S['t'] - max(S['clear'], S['still']) >= 2. - EPS
    free = bool(not S['bond'].any() and separated(o))
    flat = gel_flat(o['gel_sheet'])
    rest = base_down(o['acrylic_base'], o.get('push_out_stand')) and ring_down(o['outer_ring'])
    if free and flat and rest and held: return 1.
    if free and flat and clear1: return .75
    if free and clear1: return .5
    if rest and clear1: return .25
    return 0.
def score(o):
    try: return float(evaluate(o))
    except (KeyError, ValueError, TypeError, AttributeError, IndexError): return 0.
def success(o): return score(o) == 1.
def peel_force(gap, k=0):
    """Opening resistance (N) of a bonded band at underside gap `gap` (m): rises over PEEL_RAMP, capped at the width-derived FRONT_FORCE[k]."""
    return float(FRONT_FORCE[k]) * min(1., max(float(gap), 0.) / PEEL_RAMP)
def touching(k, touch, bond):
    """Robot contact on band k itself or on the freed flap joined to it (the free bands next to it, either side)."""
    if touch[k]: return True
    for step in (-1, 1):
        j = k + step
        while 0 <= j < N and not bond[j]:
            if touch[j]: return True
            j += step
    return False


def is_front(k, bond):
    return k in (0,N-1) or not bond[k-1] or not bond[k+1]

def contact_groups(bond):
    groups=[]
    for k in np.flatnonzero(bond):
        if not is_front(k,bond):continue
        nodes={int(k)}
        for d in (-1,1):
            j=k+d
            while 0<=j<N and not bond[j]:nodes.add(int(j));j+=d
        fronts={int(k)};merged=True
        while merged:
            merged=False
            for i,(ff,nn) in enumerate(groups):
                if nodes&nn:fronts|=ff;nodes|=nn;groups.pop(i);merged=True;break
        groups.append((fronts,nodes))
    return groups

def brake_work(o):
    """Signed work against the actual previous substep's peel load at its material contact point.
    Both gel and acrylic motion/rotation count; closing returns credit. No contact/pose-only work is invented.
    """
    out=np.zeros(N);pp=poses(o['gel_sheet']);b=o['acrylic_base'];br=parts._quat_to_matrix(b.quat_xyzw)
    for k,j in enumerate(S['brake_body']):
        if j<0:continue
        gp,gr=pp[LINK[j]]
        moved_g=gp+gr@S['brake_local'][k]
        moved_b=np.asarray(b.pos)+br@S['brake_base_local'][k]
        value=-float(S['brake_f'][k]@(moved_g-moved_b))
        out[k]=0. if value>0 and np.linalg.norm(S['brake_f'][k])<FRONT_FORCE[k]-1e-9 else value
    return out

def prepare_brakes(o,dt=1/240):
    """A real downward peel load on the contacted front/freed flap, with paired acrylic reaction.
    It ramps with that front's own opening. A closed next front does not load a merely supported freed flap.
    One load per active front; shared contact bears the sum, without reused energy.
    """
    S['brake_f']=np.zeros((N,3));S['brake_body']=np.full(N,-1,int)
    pp=poses(o['gel_sheet']);b=o['acrylic_base'];br=parts._quat_to_matrix(b.quat_xyzw)
    for fronts,nodes in contact_groups(S['bond']):
        candidates=[j for j in nodes if S['touch'][j] and np.isfinite(S['contact_points'][j]).all()]
        if not candidates:continue
        j=max(candidates,key=lambda v:S['contact_weights'][v]);cp=S['contact_points'][j]
        gp,gr=pp[LINK[j]]
        for k in fronts:
            magnitude=PEEL_LOAD_FACTOR*FRONT_FORCE[k]*min(1.,max(0.,S['gap'][k])/PEEL_RAMP)
            S['brake_f'][k]=-magnitude*br[:,2]
            S['brake_body'][k]=j;S['brake_point'][k]=cp
            S['brake_local'][k]=(cp-gp)@gr
            S['brake_base_local'][k]=(cp-np.asarray(b.pos))@br

def strip_area(lo,hi):
    total=0.
    for aa,bb,m,c in ((-R,-R/2,2*math.sqrt(3),2*math.sqrt(3)*R),(-R/2,R/2,0.,math.sqrt(3)*R),(R/2,R,-2*math.sqrt(3),2*math.sqrt(3)*R)):
        x,y=max(lo,aa),min(hi,bb)
        if y>x:total+=m*(y*y-x*x)/2+c*(y-x)
    return total

def front_boundary(k,fraction,direction):
    a=-R+k*W;b=a+W;lo,hi=a,b
    target=(fraction if direction>0 else 1.-fraction)*NOMINAL_AREAS[k]
    for _ in range(18):
        mid=(lo+hi)/2
        if strip_area(a,mid)<target:lo=mid
        else:hi=mid
    return lo if direction>0 else hi  # numerical bound keeps slightly MORE material bonded

def adhesive_step(o, dt=0., touch=None, robot_work=None, resisted_work=None):
    """Area-energy peel propagation. The rule records contact work; no object is moved or driven.
    Damage tests the still-bonded remainder of a front, retaining the same5/2.5/3mm limits.
    """
    pp=gel_points(o['gel_sheet']);b=o['acrylic_base'];br=parts._quat_to_matrix(b.quat_xyzw)
    touch=np.zeros(N,bool) if touch is None else np.asarray(touch,bool)
    robot_work=np.zeros(N) if robot_work is None else np.asarray(robot_work,float)
    resisted_work=brake_work(o) if resisted_work is None else np.asarray(resisted_work,float)
    if not np.isfinite(robot_work).all() or not np.isfinite(resisted_work).all():raise ValueError('unknown signed peel work')
    old=S['bond'].copy();progress=np.clip(S['work']/FRONT_WORK,0.,1.)
    centre_anchors=np.array([(-R+(k+.5)*W,0.,BH/2) for k in range(N)])
    centre_delta=(pp-b.pos)@br-centre_anchors
    bond_points=pp.copy();delta=centre_delta.copy();gp=poses(o['gel_sheet'])
    for k in np.flatnonzero(old):
        if not is_front(k,old):continue
        # Previous progress, so one violent substep cannot erase the patch before its damage is tested.
        left=(k==0 or (k>0 and not old[k-1]));right=(k==N-1 or (k<N-1 and not old[k+1]))
        xs=([front_boundary(k,progress[k]/2,1),front_boundary(k,progress[k]/2,-1)] if left and right
            else [front_boundary(k,progress[k],1 if left else -1)])
        pos,rot=gp[LINK[k]];candidates=[]
        for x in xs:
            point=pos+rot@(np.array([x,0.,-H/2])-GEL_SHEET.joint_origin(LINK[k]))
            d=(point-b.pos)@br-np.array([x,0.,BH/2]);candidates.append((point,d))
        bond_points[k],delta[k]=max(candidates,key=lambda v:v[1][2])
    S['delta']=delta;S['bond_points']=bond_points;S['bond_gap']=delta[:,2].copy()
    returned=resisted_work.copy()
    for k in np.flatnonzero(old):
        if S['brake_body'][k]<0:
            returned[k]+=min(0.,FRONT_FORCE[k]*(centre_delta[k,2]-S['gap'][k]))
    for fronts,nodes in contact_groups(old):
        ids=sorted(fronts)
        supplied=sum(robot_work[j] for j in nodes if touch[j])
        resisted=sum(returned[k] for k in ids)
        # One signed energy budget: the same returned work is NOT charged twice.
        # Net both ledgers before allocation, including opposite front contributions.
        change=min(supplied,resisted)
        credit=sum(S['work'][k] for k in ids)
        if change<0 and credit>0:
            for k in ids:S['work'][k]*=max(0.,1.+change/credit)
        elif change>0:
            weights={k:max(0.,returned[k]) if touching(k,touch,old) else 0. for k in ids}
            total=sum(weights.values())
            if total>0:
                for k in ids:S['work'][k]+=min(change,total)*weights[k]/total
    S['gap']=centre_delta[:,2].copy()
    for k in range(N):
        if not old[k]:S['work'][k]=0.;continue
        front=is_front(k,old);gap=float(delta[k,2])
        if np.linalg.norm(delta[k,:2])>SHEAR:fail(f'tear: bonded remainder of band {k} sheared over3mm')
        if not front:
            if gap>HOLE:fail(f'tear: non-front band {k} opened over2.5mm')
            continue
        if gap>YANK:fail(f'tear: still-bonded remainder of front {k} opened over5mm')
        if centre_delta[k,2]>=OPEN-EPS and S['work'][k]>=FRONT_WORK[k]-1e-9 and touching(k,touch,old) and not S['zero']:
            S['bond'][k]=False;event(f'band {k} released t={S.get("time",0.):.2f} area work={1000*S["work"][k]:.2f}mJ')
    return pp,delta


def bond_wrenches(o):
    """World forces of the bond and press-fit rules. Returns per-band (force, point, extra moment) on the sheet, (force, moment
    about its centre) on the acrylic, and (force, point) on the ring; the set sums to zero force and zero moment."""
    pp = S['bond_points'] if S['bond_points'] is not None else gel_points(o['gel_sheet']); br = parts._quat_to_matrix(o['acrylic_base'].quat_xyzw); nn = br[:, 2]
    delta = S.get('delta'); bpos = np.asarray(o['acrylic_base'].pos, float)
    f = np.zeros((N, 3)); torque=np.zeros((N,3)); bf = np.zeros(3); bt = np.zeros(3)
    for k in range(N):
        if not S['bond'][k]: continue
        gap = float(delta[k, 2]) if delta is not None else 0.
        f[k] = np.zeros(3) if S['brake_body'][k]>=0 else -peel_force(gap,k)*nn
        if delta is not None:   # in-plane hold of the bond (adhesion resists sliding as well as lifting)
            f[k] = f[k] - GRIP * (br @ np.array((delta[k, 0], delta[k, 1], 0.)))
        bf -= f[k]; bt += np.cross(pp[k] - bpos, -f[k])   # equal and opposite, at the same point, on the acrylic
    for k,j in enumerate(S['brake_body']):
        if j<0 or not S['bond'][k]:continue
        force=S['brake_f'][k];cp=S['brake_point'][k]
        f[j]+=force;torque[j]+=np.cross(cp-pp[j],force)
        bf-=force;bt+=np.cross(cp-bpos,-force)
    rr = parts._quat_to_matrix(o['outer_ring'].quat_xyzw); rpos = np.asarray(o['outer_ring'].pos, float) + rr[:, 2] * .009
    rf = np.zeros(3)
    if S['seated']:
        rf = FIT * rr[:, 2]; bf -= rf; bt += np.cross(rpos - bpos, -rf)
    return (f, pp, torque), (bf, bt), (rf, rpos)


def apply_wrenches(sim, o):
    """Bond forces (gel bands onto the acrylic) and the ring's press fit, with equal and opposite reactions."""
    import torch
    base = sim.env.unwrapped; org = parts._np(base.scene.env_origins[0])
    a = base.scene['gel_sheet']; b = base.scene['acrylic_base']; c = base.scene['outer_ring']
    (fk, pk, gel_torque), (bf, bt), (rf, rpos) = bond_wrenches(o)
    f = np.zeros((len(a.body_names), 3)); at = np.zeros_like(f); gt=np.zeros_like(f); bpos = o['acrylic_base'].pos
    for k, n in enumerate(LINK):
        i = list(a.body_names).index(n); at[i] = pk[k] + org; f[i] = fk[k];gt[i]=gel_torque[k]
    for asset, forces, points in ((a, f, at), (b, bf[None, :], (bpos + org)[None, :]), (c, rf[None, :], (rpos + org)[None, :])):
        moments = gt if asset is a else (bt[None,:] if asset is b else np.zeros_like(forces))
        ff = torch.tensor(forces[None, :, :], dtype=torch.float32, device=asset.device)
        tt = torch.tensor(moments[None, :, :], dtype=torch.float32, device=asset.device)
        pq = torch.tensor(points[None, :, :], dtype=torch.float32, device=asset.device); ids = torch.tensor([0], device=asset.device)
        composer = getattr(asset, 'permanent_wrench_composer', None)
        if composer is not None: composer.set_forces_and_torques_index(forces=ff, torques=tt, positions=pq, env_ids=ids, is_global=True)
        else: asset.set_external_force_and_torque(ff, tt, positions=pq, env_ids=ids, is_global=True)


def fit_step(o, ring_touch=0.):
    """Only travel made under >=7 N robot load counts. A stationary delayed touch cannot release."""
    ring = o['outer_ring']; rr = parts._quat_to_matrix(ring.quat_xyzw)
    z = float(((o['acrylic_base'].pos - ring.pos) @ rr)[2]) - (.005 + BH / 2)
    dz = z - S['fit_prev']; S['fit_prev'] = z
    if not S['seated']: return
    if dz < 0:
        S['fit_loaded'] = max(0., S['fit_loaded'] + dz)
        S['fit_work'] = max(0., S['fit_work'] + FIT_TOUCH * dz)
    elif ring_touch >= FIT_TOUCH:
        S['fit_loaded'] += dz
        S['fit_work'] += ring_touch * dz
    if S['fit_loaded'] >= FIT_TRAVEL - EPS and S['fit_work'] >= FIT_WORK - 1e-9:
        S['seated'] = False
        event(f'press fit released t={S.get("time", 0.):.2f} ({ring_touch:.1f} N robot, {1000*S["fit_loaded"]:.2f} mm loaded travel, {1000*S["fit_work"]:.2f} mJ)')


# Contact sensors on the seven gel bands and the ring, read at every 240 Hz physics substep.
def arr(x):
    x = x.torch if hasattr(x, 'torch') else x
    return np.asarray(x.detach().cpu().numpy() if hasattr(x, 'detach') else (x.numpy() if hasattr(x, 'numpy') else x))
def contact_view():
    from isaaclab_physx.physics import PhysxManager
    from isaaclab.sim.utils import get_current_stage
    from pxr import Usd, UsdPhysics
    stage = get_current_stage()
    arms = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies())
            if any(k in str(p.GetPath()) for k in ('/LeftArm/', '/RightArm/')) and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if not arms: raise ValueError('arm contacts unavailable')
    S['arms'] = arms
    view = PhysxManager.get_physics_sim_view()
    gel = view.create_rigid_contact_view([parts._PATHS['gel_sheet'] + '/' + n for n in LINK], filter_patterns=[arms + [parts._PATHS['opening_pick']] for _ in LINK], max_contact_data_count=512)
    ring = view.create_rigid_contact_view([parts._PATHS['outer_ring']], filter_patterns=[arms + [parts._PATHS['opening_pick']]], max_contact_data_count=256)
    tool = view.create_rigid_contact_view([parts._PATHS['opening_pick']], filter_patterns=[arms], max_contact_data_count=256)
    return gel, ring, tool
def seg_sum(vals, counts, starts, what):
    """Sum vals[start:start + count] for every (count, start) pair, vectorised; checks the buffer bounds."""
    counts = counts.reshape(-1).astype(np.int64); starts = starts.reshape(-1).astype(np.int64)
    out = np.zeros((len(counts),) + vals.shape[1:])
    if (counts < 0).any(): raise ValueError(what + ' count invalid')
    nz = counts > 0
    if not nz.any(): return out
    if (starts[nz] < 0).any() or (starts[nz] + counts[nz] > len(vals)).any(): raise ValueError(what + ' buffer incomplete')
    reps = counts[nz]; seg = np.repeat(np.flatnonzero(nz), reps)
    src = np.repeat(starts[nz], reps) + np.arange(reps.sum()) - np.repeat(np.cumsum(reps) - reps, reps)
    np.add.at(out, seg, vals[src]); return out
def signed_loads(view, rows, dt, include_tool=True):
    """Net contact force vector (normal + friction) of the robot on each sensed body, and summed contact magnitudes."""
    fm = arr(view.get_contact_force_matrix(dt=dt)).reshape(rows, -1, 3)
    fr, _, fc, fs = (arr(x) for x in view.get_friction_data(dt=dt))
    friction = seg_sum(fr.reshape(-1, 3), fc, fs, 'friction').reshape(rows, -1, 3)
    if not include_tool: fm[:, -1] = 0.; friction[:, -1] = 0.
    vec = fm.sum(1) + friction.sum(1)
    force, _, _, _, counts, starts = (arr(x) for x in view.get_contact_data(dt=dt))
    magnitudes = seg_sum(np.abs(force.reshape(-1)), counts, starts, 'contact').reshape(rows, -1)
    if not include_tool: magnitudes[:, -1] = 0.
    mag = magnitudes.sum(1)
    if not np.isfinite(vec).all() or not np.isfinite(mag).all(): raise ValueError('contact load unknown')
    return vec, mag
def contact_work(view, o, dt, org, include_tool=True):
    """Robot work on each band relative to the acrylic over one physics step (J).

    Each current contact point is mapped back through the previous band and acrylic poses. Their relative
    displacement dotted with the signed normal/friction force counts translation AND the edge-lifting moment.
    This rejects a top press, a pose change without a force, and a stationary touch after an uncontacted lift.
    """
    current = poses(o['gel_sheet']); current['acrylic'] = (np.asarray(o['acrylic_base'].pos), parts._quat_to_matrix(o['acrylic_base'].quat_xyzw))
    previous = S['contact_prev']; S['contact_prev'] = current
    if previous is None: return np.zeros(N)
    force, point, normal, _, counts, starts = (arr(x) for x in view.get_contact_data(dt=dt))
    f = force.reshape(-1, 1) * normal.reshape(-1, 3)
    # Sign and units are checked against PhysX's net normal-force matrix, rather than assumed.
    reconstructed = seg_sum(f, counts, starts, 'normal work').reshape(N, -1, 3)
    native = arr(view.get_contact_force_matrix(dt=dt)).reshape(N, -1, 3)
    if not np.allclose(reconstructed, native, atol=1e-4, rtol=1e-3):
        raise ValueError('signed normal contact reconstruction differs from PhysX')
    S['contact_points']=np.full((N,3),np.nan);S['contact_weights']=np.zeros(N);S['contact_motion']=np.zeros((N,3))
    cc=counts.reshape(N,-1);ss=starts.reshape(N,-1);weights=np.abs(force.reshape(-1));points=point.reshape(-1,3)-org
    for k in range(N):
        total=0.;weighted=np.zeros(3)
        for j,(count,start) in enumerate(zip(cc[k],ss[k])):
            if not include_tool and j==cc.shape[1]-1:continue
            count,start=int(count),int(start)
            if count:
                ww=weights[start:start+count];weighted+=(points[start:start+count]*ww[:,None]).sum(0);total+=ww.sum()
        if total>0:
            cp=weighted/total;S['contact_points'][k]=cp;S['contact_weights'][k]=total
            gp,gr=current[LINK[k]];gp0,gr0=previous[LINK[k]]
            bp,br=current['acrylic'];bp0,br0=previous['acrylic']
            old_gel=gp0+gr0@(gr.T@(cp-gp));old_base=bp0+br0@(br.T@(cp-bp))
            S['contact_motion'][k]=old_base-old_gel
    def work(vectors, points, cc, ss):
        cc = cc.reshape(N, -1); ss = ss.reshape(N, -1); out = np.zeros(N)
        bp, br = current['acrylic']; bp0, br0 = previous['acrylic']
        for k, name in enumerate(LINK):
            gp, gr = current[name]; gp0, gr0 = previous[name]
            for j, (count, start) in enumerate(zip(cc[k], ss[k])):
                if not include_tool and j == cc.shape[1]-1: continue
                count, start = int(count), int(start)
                if not count: continue
                pts = points[start:start + count] - org
                old_gel = ((pts - gp) @ gr) @ gr0.T + gp0
                old_base = ((pts - bp) @ br) @ br0.T + bp0
                out[k] += float(np.sum(vectors[start:start + count] * (old_base - old_gel)))
        return out
    out = work(f, point.reshape(-1, 3), counts, starts)
    S['normal_work']=out.copy()
    friction, fp, fc, fs = (arr(x) for x in view.get_friction_data(dt=dt))
    S['friction_work']=work(friction.reshape(-1, 3), fp.reshape(-1, 3), fc, fs)
    out += S['friction_work']
    if not np.isfinite(out).all(): raise ValueError('robot opening work unknown')
    return out
def contact_debug(view, dt):
    """Trace only: per band, the arm links in contact and the force-weighted contact point (world)."""
    force, point, _, _, counts, starts = (arr(x) for x in view.get_contact_data(dt=dt))
    counts = counts.reshape(N, -1); starts = starts.reshape(N, -1); f = np.abs(force.reshape(-1)); pt = point.reshape(-1, 3); out = {}
    fm = arr(view.get_contact_force_matrix(dt=dt)).reshape(N, -1, 3)
    fr, _, fc, fs = (arr(x) for x in view.get_friction_data(dt=dt))
    fm += seg_sum(fr.reshape(-1, 3), fc, fs, 'debug friction').reshape(N, -1, 3)
    for k in range(N):
        for j in range(counts.shape[1]):
            n, s0 = int(counts[k, j]), int(starts[k, j])
            if n <= 0: continue
            w = f[s0:s0 + n]; c = (pt[s0:s0 + n] * w[:, None]).sum(0) / max(w.sum(), 1e-9)
            out.setdefault(k, []).append(dict(link=('Tool/' if j == len(S['arms']) else ('LeftArm/' if '/LeftArm/' in S['arms'][j] else 'RightArm/')) + (S['arms'] + ['Tool/opening_pick'])[j].split('/')[-1], normal_N=round(float(w.sum()), 3), force_N=np.round(fm[k, j], 4).tolist(), point=np.round(c, 4).tolist()))
    return out
def contact_damage(loads):
    loads = np.asarray(loads, float)
    if not np.isfinite(loads).all(): raise ValueError('contact load unknown')
    if loads.shape != (N,): raise ValueError('contact load shape unknown')
    S['peak_by_band']=np.maximum(S['peak_by_band'],loads)
    S['peak'] = max(S['peak'], float(loads.max(initial=0.)))
    if loads.max(initial=0.) > CRUSH: fail(f'tear: {loads.max():.1f} N robot contact on band {int(loads.argmax())}')
def held_pick(tm, arms):
    # Actual opposed fingertip loads on the same arm, not two arbitrary links touching the tool.
    for side in ('LeftArm','RightArm'):
        ids = [[i for i,p in enumerate(arms) if '/'+side+'/' in p and p.endswith('/'+n)] for n in ('tip_left','tip_right')]
        if all(len(ii)==1 for ii in ids):
            a,b = (tm[ii[0]] for ii in ids); na,nb=np.linalg.norm(a),np.linalg.norm(b)
            if min(na,nb) >= .5 and float(a@b) <= -.8*na*nb: return True
    return False

def material_states(sim):
    base = sim.env.unwrapped; org = parts._np(base.scene.env_origins[0]); a = base.scene['gel_sheet']
    p = parts._np(a.data.body_link_pose_w)[0]; bi = list(a.body_names).index('base'); q = parts._np(a.data.joint_pos)[0]
    gel = SimpleNamespace(pos=p[bi, :3] - org, quat_xyzw=p[bi, 3:],
                          links={n: (p[list(a.body_names).index(n), :3] - org, p[list(a.body_names).index(n), 3:]) for n in LINK if n != 'base'},
                          joints={n: float(q[i]) for i, n in enumerate(a.joint_names)})
    out = {'gel_sheet': gel}
    for name in ('acrylic_base', 'outer_ring'):
        bp = parts._np(base.scene[name].data.root_link_pose_w)[0]
        if not np.isfinite(bp).all(): raise ValueError('material state unknown')
        out[name] = SimpleNamespace(pos=bp[:3] - org, quat_xyzw=bp[3:])
    if not np.isfinite(p).all() or not np.isfinite(q).all(): raise ValueError('material state unknown')
    return out, org


@parts.on_reset
def reset(sim):
    global S
    S = fresh()


@parts.on_physics_step
def physics(sim):
    try:
        dt = float(sim.env.unwrapped.physics_dt); S['time'] = float(sim.t)
        if S['views'] is None: S['views'] = contact_view()
        gel_view, ring_view, tool_view = S['views']
        tm = arr(tool_view.get_contact_force_matrix(dt=dt)).reshape(-1,3)
        S['tool_grip'] = float(np.linalg.norm(tm,axis=1).sum())
        tool_held = held_pick(tm, S['arms'])
        fvec, loads = signed_loads(gel_view, N, dt, tool_held)
        if os.environ.get('GEL_LOG') and S['phys'] % 12 == 0: S['cdbg'] = contact_debug(gel_view, dt)
        rvec, rmag = signed_loads(ring_view, 1, dt, tool_held)
        material, org = material_states(sim)
        S['fvec'] = fvec; S['touch'] = loads >= TOUCH_N; S['ring_touch'] = float(rmag[0])
        rn = parts._quat_to_matrix(material['outer_ring'].quat_xyzw)[:, 2]
        S['ring_press'] = float(-rvec[0] @ rn)
        S['robot_work'] = contact_work(gel_view, material, dt, org, tool_held)
        damage_loads = loads if tool_held else signed_loads(gel_view, N, dt, True)[1]
        contact_damage(damage_loads)
        S['phys'] += 1
        fit_step(material, S['ring_press']); adhesive_step(material, dt, S['touch'], S['robot_work']); prepare_brakes(material,dt); apply_wrenches(sim, material)
    except Exception as e:
        S.update(known=False); fail('material state unavailable: ' + str(e))


def observe(o, h, t):
    # Whole-trial history; pure, so offline checks can drive it.
    if h is None or len(h) == 0 or not np.isfinite(h).all():
        S['known'] = False; fail('arm state unknown'); return
    chunks = []; corners = np.array([(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    for spec in (RING, BASE, GEL_SHEET, STAND, PICK):
        st = o[spec.name]
        if not np.isfinite(st.pos).all() or not np.isfinite(st.quat_xyzw).all():
            S['known'] = False; fail('object state unknown'); return
        if st.low[2] < TOP - .01: fail(spec.name + ' fell off the counter')
        for _, c, r, size in spec.world_boxes(poses(st)): chunks.append(c + (corners * (size / 2)) @ r.T)
    snap = np.concatenate(chunks)
    clear = hands_clear(h, o)
    if S['prev'] is None or np.max(np.linalg.norm(snap - S['prev'], axis=1)) > .001:
        S['still'] = float(t); S['prev'] = snap.copy()
    S['clear'] = (S['clear'] if S['clear'] is not None else float(t)) if clear else None
    S.update(steps=S['steps'] + 1, t=float(t), known=True)


@parts.on_step
def step(sim):
    if S.get('last_t') == sim.t: return
    S['last_t'] = sim.t
    try:
        o = sim.states(); h = hands(sim)
        observe(o, h, float(sim.t))
        log = os.environ.get('GEL_LOG') or (str(pathlib.Path(os.environ['TRACE_OUT']).with_name('gel.jsonl')) if os.environ.get('TRACE_OUT') else None)
        if log and (score(o) == 1. or S['steps'] % int(os.environ.get('GEL_LOG_EVERY', '5')) == 0):
            pp = gel_points(o['gel_sheet'])
            row = dict(source_sha256=SOURCE_SHA256, t=float(sim.t), lot=lots.current().name, phys=S['phys'], score=score(o), bonds=S['bond'].astype(int).tolist(),
                       gap_mm=np.round(1000 * S['gap'], 3).tolist(), bonded_gap_mm=np.round(1000*S['bond_gap'],3).tolist(), area_work_required_mJ=np.round(1000*FRONT_WORK,3).tolist(), brake_force_N=np.round(S['brake_f'],3).tolist(), tool_held=held_pick(arr(S['views'][2].get_contact_force_matrix(dt=float(sim.env.unwrapped.physics_dt))).reshape(-1,3),S['arms']), work_mJ=np.round(1000 * S['work'], 3).tolist(), touch=S['touch'].astype(int).tolist(),
                       fz=np.round(S['fvec'][:, 2], 3).tolist(), fx=np.round(S['fvec'][:, 0], 3).tolist(), ring_touch=round(S['ring_touch'], 2), ring_press=round(S['ring_press'], 3), tool_grip=round(S['tool_grip'],3), tool=np.round(o['opening_pick'].pos,4).tolist(), tool_quat=np.round(o['opening_pick'].quat_xyzw,4).tolist(), robot_work_uJ=np.round(1e6 * S['robot_work'], 3).tolist(), normal_work_uJ=np.round(1e6*S.get('normal_work',np.zeros(N)),3).tolist(), friction_work_uJ=np.round(1e6*S.get('friction_work',np.zeros(N)),3).tolist(), contact_motion_um=np.round(1e6*S['contact_motion'],3).tolist(), cdbg=S.get('cdbg'), zero=S['zero'], why=S['why'], peak=round(S['peak'], 3), normal_peak_N=np.round(S['peak_by_band'],3).tolist(), seated=S['seated'],
                       fit_loaded_mm=1000*S['fit_loaded'], fit_work_mJ=1000*S['fit_work'], base_supported=base_down(o['acrylic_base'],o.get('push_out_stand')), ring_supported=ring_down(o['outer_ring']), gel_flat=gel_flat(o['gel_sheet']), still_since=S['still'], clear_since=S['clear'], events=S['events'], clear=bool(S['clear'] is not None),
                       ring=np.round(o['outer_ring'].pos, 4).tolist(), base=np.round(o['acrylic_base'].pos, 4).tolist(),
                       stand=np.round(o['push_out_stand'].pos, 4).tolist(), gel=np.round(pp, 4).tolist(), gel_origin=np.round(o['gel_sheet'].pos,6).tolist(), gel_quat=np.round(o['gel_sheet'].quat_xyzw,6).tolist(),
                       joints={k: round(v, 3) for k, v in o['gel_sheet'].joints.items()},
                       ring_quat=np.round(o['outer_ring'].quat_xyzw, 4).tolist(), base_quat=np.round(o['acrylic_base'].quat_xyzw, 4).tolist())
            base_env = sim.env.unwrapped; org = parts._np(base_env.scene.env_origins[0]); row['grippers'] = {}
            for arm in ('left_robot', 'right_robot'):
                ar = base_env.scene[arm]; vals = parts._np(ar.data.body_link_pose_w)[0]
                row['grippers'][arm] = {n: np.round(vals[list(ar.body_names).index(n), :3] - org, 4).tolist() for n in ('gripper', 'tip_left', 'tip_right')}
                row['grippers'][arm]['quat'] = np.round(vals[list(ar.body_names).index('gripper'), 3:], 4).tolist()
            with open(log, 'a') as f: f.write(json.dumps(row) + '\n')
    except Exception as e:
        S.update(known=False); fail('state unavailable: ' + str(e))


# ---------------------------------------------------------------------------------------------------------------
# Open-loop demo from the visible layout: squeeze-lift the ring by its two flats, set the acrylic's exposed back on
# the stand and press the ring down off it; hold the stack with one hand while the other works the held plastic pick in at the
# near corner. The left hand grips the supported flap; the right returns to counterhold while the left peels and lays it flat.
def demo():
    lot = lots.current(); sx, sy = lot.pos('push_out_stand')[:2]
    from yam_arena import ik, station
    from yam_arena.demo import READY
    def rest(arm):
        p, r = ik.tcp_pose(np.array(READY), station.BASES[arm]); return np.asarray(p), tilt._TiltedYaw(0., r)
    (lr, ly), (rr, ry) = rest('left'), rest('right')
    def park(d, arm, label):
        p, y = (lr, ly) if arm == 'left' else (rr, ry)
        return Waypoint(float(d), tuple(float(v) for v in p), FINGER_CLOSED, label, y)
    def w(d, p, f, label, yaw=0., a=0., heading=0.):
        return tilt.Waypoint(float(d), tuple(float(v) for v in p), float(f), label, yaw=yaw, tilt=a, heading=heading)
    CL = FINGER_CLOSED
    PI2 = math.pi / 2
    # ---- 1. pinch the ring's top rim at its two flats, set the acrylic's exposed back on the stand, let go, then
    # press the ring down off the acrylic with closed fingers on the rim.
    WC = (AF + (R + .0002) * math.sqrt(3) / 2) / 2   # middle of the ring wall at a flat
    zt = TOP + .016 + .0128   # fingertips 16 mm up the 18 mm wall: 1.65 mm above the gel's top face
    lift = .062               # ring bottom 62 mm up: the acrylic's back clears the 50 mm stand
    seat = .045               # ring bottom when the acrylic's back rests on the stand
    opn1, pin = -.016, -.0025 # 32 mm open; 5 mm commanded on the 9.8 mm wall
    L = []; Rr = []
    D = .032                  # the two pinches sit 32 mm either side of the flats' middles, so the forearms stay apart
    def both(d, x, y, dy_, z, f, label):
        L.append(w(d, (x + D, y + dy_, z), f, label)); Rr.append(w(d, (x - D, y - dy_, z), f, label))
    both(2.0, MX, MY, WC, zt + .05, opn1, 'above the ring rim at both flats')
    both(1.5, MX, MY, WC, zt, opn1, 'fingers astride the rim')
    both(1.0, MX, MY, WC, zt, pin, 'pinch the rim')
    both(2.5, MX, MY, WC, zt + lift, pin, 'lift the mold')
    both(6.0, sx, sy, WC, zt + lift, pin, 'carry the mold over the stand')
    both(2.0, sx, sy, WC, zt + seat + .008, pin, 'lower the acrylic toward the stand')
    both(1.5, sx, sy, WC, zt + seat + .0005, pin, 'set the acrylic back on the stand')
    both(.8, sx, sy, WC, zt + seat + .0005, opn1, 'let go of the rim')
    both(.8, sx, sy, WC, zt + seat + .015, opn1, 'fingers up')
    # ---- 2. counterhold (left) and corner start (right).
    zb = TOP + STAND_H + BH   # acrylic face (bond line) on the stand
    gel_top = zb + H
    hold = (sx - .030, sy + .025)   # near the peel start, inside the dish support; avoids tipping with the pick
    xt = sx - R               # near corner of the gel
    # Balanced opposite-flat press supplies the nominal ring load without rocking the dish.
    ring_top = TOP + seat + .018
    # Balanced press on opposite ring flats: the old single-corner press rocks the dish at higher load.
    pw=AF-.001
    def flat_press(d,z,label):
        L.append(w(d,(sx+D,sy+pw,z),CL,label))
        Rr.append(w(d,(sx-D,sy-pw,z),CL,label))
    flat_press(1.,ring_top+.0128+.012,'closed fingertips above opposite rim flats')
    flat_press(1.5,ring_top+.0128-.008,'press opposite rim flats down under the real load')
    flat_press(.5,ring_top+.0128-.008,'complete the loaded ring travel')
    flat_press(1.,ring_top+.0128+.035,'withdraw both press fingers')
    t_press=sum(x.duration for x in Rr)
    L.append(w(1.2,(sx,sy+.20,TOP+.12),CL,'left clears for tool collection'))
    Rr.append(w(1.2,(sx,sy-.20,TOP+.12),CL,'right clears for tool collection'))
    t_press=sum(x.duration for x in Rr)
    # Pick the loose handled plastic pick from the counter with a physical pinch.
    px,py,pz=PICK_POS
    px-=.003  # align the30mm handle body with the full27.4mm pad width
    Rr.append(w(2.,(px,py,pz+.065),-.020,'above the pick handle'))
    Rr.append(w(1.5,(px,py,pz+.0028),-.020,'fingers astride the pick handle'))
    Rr.append(w(1.,(px,py,pz+.0028),-.007,'grip the pick handle'))
    Rr.append(w(1.,(px,py,pz+.075),-.007,'lift the loose pick'))
    tl = sum(x.duration for x in L)
    L.append(w(t_press - tl, (sx, sy + .20, TOP + .12), CL, 'wait for the ring to drop'))
    left_support_approach=tilt.rotation(0.,80.,-PI2)[:,2]
    left_stack_c=np.array([sx-.03645,sy+.0665,TOP+STAND_H+(BH+H)/2])
    left_stack_tcp=left_stack_c-.0078*left_support_approach
    def lsw(d,p,f,label):return w(d,tuple(p),f,label,0.,80.,-PI2)
    L.append(lsw(2.,left_stack_tcp-.060*left_support_approach,-.014,'open outside the left edge of the stack'))
    L.append(lsw(.5,left_stack_tcp-.060*left_support_approach,-.014,'settle the left wrist outside the edge'))
    L.append(lsw(2.,left_stack_tcp,-.014,'slide left fingers over and under the side edge'))
    L.append(lsw(1.5,left_stack_tcp,-.0036,'pinch the stack from its left edge'))
    L.append(lsw(.5,left_stack_tcp,-.0036,'settle the left stack grasp'))
    # Enter normal to the straight upper seam within the near corner, avoiding the unstable vertex.
    tool_heading=-math.pi/6;tool_pitch=math.radians(10.)
    entry_dir=np.array([math.cos(tool_heading),math.sin(tool_heading)])
    seam_corner=xt-.0011  # prior native push-out puts acrylic1.1mm left of the dish centre
    edge=np.array([seam_corner+.003,sy+math.sqrt(3)*.003])
    def tw(d,xy,z,label):
        rot=tilt.rotation(tool_heading,10.,tool_heading+math.pi)
        tcp=np.array([xy[0],xy[1],z])-rot@PICK_GRASP_APEX
        Rr.append(w(d,tuple(tcp),-.007,label,tool_heading,10.,tool_heading+math.pi))
    Rr.append(w(1.5,(xt-.0356,sy-.18,zb+.080),-.007,'route the pick on the right side clear of the counterhold'))
    Rr.append(w(1.,(xt-.0356,sy-.18,zb+.035+.0028),-.007,'lower beside the perimeter'))
    tw(2.,edge-.005*entry_dir,zb+.035,'bring the held pick above the corner seam')
    tw(1.5,edge-.002*entry_dir,zb+.0001,'pick underside at the acrylic face')
    tw(2.,edge+.007*entry_dir,zb+.0001,'insert7mm normal to the straight seam near the corner')
    tw(3.,(seam_corner+.0165,sy+.0017),zb+.014,'physically pry and lift the bonded corner on the held pick')
    tw(1.5,(seam_corner+.023,sy+.0017),zb+.021,'support the raised corner with the held pick')
    Rr.append(w(1.,Rr[-1].pos,-.007,'settle the physically supported corner',tool_heading,10.,tool_heading+math.pi))
    # The right hand supports the flap while the left hand grips its faces.
    # Positions are fixed from the visible layout and earlier contact traces; no state is read by the plan.
    t_support = sum(x.duration for x in Rr)
    L.append(lsw(t_support-sum(x.duration for x in L),left_stack_tcp,-.0036,'hold the stack while the pick raises the corner'))
    # Fixed native calibration of supported material at21.3s inr2core15. No runtime state query.
    raised_r=parts._quat_to_matrix(np.array([-0.012176, 0.588465, 0.016723, 0.808258]))
    normal=raised_r[:,2]
    approach=raised_r[:,0]/math.sqrt(3)-raised_r[:,1]*math.sqrt(2/3)
    grip_rotation=np.column_stack((-normal,np.cross(approach,-normal),approach))
    hinge0=np.array([sx+(np.float64(-0.055710217750759045)),sy+(np.float64(-0.0013517121215109195)),zb+(np.float64(0.0014997145809602097))])
    material_offset=-.015*raised_r[:,0]+.006*raised_r[:,1]
    contact=hinge0+material_offset
    grasp=tuple(contact-.0078*approach)
    sf=-.00045  # physical pinch command;15N damage threshold unchanged
    def lw(d, p, f, label):
        return Waypoint(float(d), tuple(float(x) for x in p), float(f), label, tilt._TiltedYaw(0., grip_rotation))
    L.append(lsw(.7,left_stack_tcp,-.014,'release the stack before the flap handoff'))
    L.append(lsw(1.2,left_stack_tcp-.060*left_support_approach,-.014,'withdraw left fingers outside the edge'))
    L.append(w(1.,(hold[0],sy+.20,gel_top+.06),CL,'left hand clears on its side'))
    L.append(lw(1.5,(grasp[0],sy+.16,grasp[2]+.05),-.006,'open beside the supported flap'))
    L.append(lw(1.,(grasp[0],sy+.16,grasp[2]),-.006,'lower beside the flap'))
    staging=(grasp[0],grasp[1]+.025,grasp[2])
    L.append(lw(2.,staging,-.006,'stage25mm from the flap clear of the held pick'))
    L.append(lw(.3,staging,-.006,'settle before the coordinated transfer'))
    t_transfer=sum(x.duration for x in L)
    L.append(lw(.8,grasp,sf,'advance and close while the held pick moves aside'))
    L.append(lw(.5,grasp,sf,'settle the physical flap pinch'))
    t_grasp=sum(x.duration for x in L)
    Rr.append(w(t_transfer-sum(x.duration for x in Rr),Rr[-1].pos,-.007,'support the flap until the left hand is staged',tool_heading,10.,tool_heading+math.pi))
    transfer_clear=(Rr[-1].pos[0],Rr[-1].pos[1]-.035,Rr[-1].pos[2])
    Rr.append(w(.8,transfer_clear,-.007,'withdraw the held pick sideways as the left hand closes',tool_heading,10.,tool_heading+math.pi))
    Rr.append(w(t_grasp-sum(x.duration for x in Rr),transfer_clear,-.007,'wait for the physical pinch to settle',tool_heading,10.,tool_heading+math.pi))
    clear = (Rr[-1].pos[0] - .030,Rr[-1].pos[1],Rr[-1].pos[2]+.030)
    Rr.append(w(1.2,clear,-.007,'withdraw the held pick'))
    Rr.append(w(1.2,(px,py,pz+.065),-.007,'carry the pick back to its counter position'))
    Rr.append(w(1.,(px,py,pz+.0028),-.007,'put the pick on the counter'))
    Rr.append(w(.7,(px,py,pz+.0028),-.020,'release the pick handle'))
    Rr.append(w(.4,(px,py,pz+.0028),-.020,'let the loose pick settle with both fingers open'))
    Rr.append(w(1.,(px,py,pz+.065),-.020,'withdraw open fingers fully clear of the pick'))
    Rr.append(w(.3,(px,py,pz+.065),CL,'close the empty hand above the stored pick'))
    # Actual side pinch of the supported acrylic/gel stack; lower finger enters outside the90mm dish.
    # Fixed prior full-reset trace calibration after the tool handoff (r2full1,58s).
    # The plate translates/yaws during physical prying; do not aim at its initial side edge.
    support_yaw=2*math.atan2(.0916,.9958)
    cc,ss=math.cos(support_yaw),math.sin(support_yaw)
    support_r=np.array([[cc,-ss,0.],[ss,cc,0.],[0.,0.,1.]])
    support_c=np.array([sx+.0083,sy-.0042,TOP+STAND_H])
    # Actual closed-finger downward counterpress, over the dish rather than a failed edge pinch.
    # Fixed prior-run plate geometry only; no runtime object/contact query.
    def press_tcp(u,bare=False):
        top=support_c+support_r@np.array([u,-.012,BH+(0. if bare else H)])
        return top+np.array([0.,0.,.0128-.016])
    press=press_tcp(-.002)
    def sw(d,p,f,label):return w(d,tuple(p),f,label)
    Rr.append(sw(2.5,press+np.array([0.,0.,.060]),CL,'closed fingers above the near bonded region over the dish'))
    Rr.append(sw(.5,press+np.array([0.,0.,.060]),CL,'settle above the counterpress'))
    Rr.append(sw(2.,press+np.array([0.,0.,.020]),CL,'lower above the near bonded region'))
    Rr.append(sw(1.5,press,CL,'counterpress the stack onto the dish'))
    Rr.append(sw(.5,press,CL,'settle the actual downward support'))
    L.append(lw(sum(x.duration for x in Rr)-sum(x.duration for x in L),grasp,sf,'hold the flap while the right counterpresses'))
    # Fixed prior native contact calibration after handoff/pick placement.
    # Open before moving within the flap plane; the right central press supports the base.
    # Unload the grip first, then compensate measured lower-jaw travel while opening.
    L.append(lw(.2,grasp,-0.0018555225242210124,'unload the grip while the lower pad remains under the flap'))
    support_start=tuple(np.asarray(grasp)+0.0016444774757789877*normal)
    L.append(lw(.2,support_start,-.0035,'open the upper jaw while keeping the lower pad in place'))
    post_r=parts._quat_to_matrix(np.array([-0.050896, 0.536246, 0.079646, 0.838753]))
    grip_rotation=np.array([[-0.8914483289389946, 0.22790190626797852, 0.39163835097101485], [-0.17079799954229585, 0.6315547397381411, -0.7562847704843983], [-0.41969979769380894, -0.7410798417402209, -0.5240732276906209]])
    normal=post_r[:,2]
    approach=grip_rotation[:,2]
    material_offset=-.015*post_r[:,0]+.006*post_r[:,1]-H/2*normal
    grip_contact_offset=np.array([0.002104505281385727, 0.009426925113357791, 0.006147664515448587])
    grip_bias=np.array([-0.00016259916744848502, -0.0014043227148423602, 0.0012528415852712804])
    grasp=(sx+-0.062423363774251395,sy+-0.013507505777841437,zb+0.026841861163669045)
    support_end=tuple(np.asarray(grasp)+0.0016444774757789877*normal)
    L.append(lw(1.,support_end,-.0035,'slide the supporting lower pad to the flap interior'))
    sf=CL
    L.append(lw(.3,grasp,-0.0018555225242210124,'close the upper pad while maintaining the lower pad position'))
    L.append(lw(1.,grasp,sf,'slowly tighten the interior pinch with the lower pad in place'))
    L.append(lw(.4,grasp,sf,'settle the interior pinch before pulling'))
    # Keep the observed corner orientation; move its hinge through the geometric peel arc.
    peel_forward=support_r[:,0]
    def side_tcp(ex,ez):
        return tuple(np.asarray(grasp)+(ex/1000-W)*peel_forward+np.array([0.,0.,ez/1000]))
    peel = []
    for k, e in ((1, 65), (2, 65), (3, 65), (4, 68), (5, 72), (6, 75)):
        fk = W * 1000 * k; lk = W * 1000 * (k - 1); el = math.radians(e)
        peel.append((fk - lk * math.cos(el), lk * math.sin(el) + (4. if k == 1 else 5.)))
    for k, (ex, ez) in enumerate(peel):
        L.append(lw(2.2 if k else 1.6, side_tcp(ex, ez), sf, f'left hand peels band {k + 1}'))
        L.append(lw(.4, side_tcp(ex, ez), sf, f'finish opening band {k + 1}'))
        if k == 2:
            # With the near bands exposed, close the current front slightly before leaving the press.
            # This unloads the partial opening while the dish supports the acrylic during regrasp.
            held=np.asarray(side_tcp(ex,ez));unload=held-np.array([0.,0.,.004])
            L.append(lw(.7,unload,sf,'lower the flap slightly during the support change'))
            Rr.append(sw(sum(x.duration for x in L)-sum(x.duration for x in Rr),press,CL,'maintain central support until the flap is lowered'))
            Rr.append(sw(.7,press+np.array([0.,0.,.050]),CL,'lift the counterpress above the sheet'))
            # Lower-left sloped edge, with7mm insertion; exposed band1 lies above the upper finger.
            edge=np.array([-.050,-math.sqrt(3)*(R-.050)+.007,BH/2])
            bare_c=support_c+support_r@edge
            bare_heading=support_yaw+PI2;bare_yaw=support_yaw
            bare_approach=tilt.rotation(bare_yaw,80.,bare_heading)[:,2]
            bare_tcp=bare_c-.0078*bare_approach
            def bw(d,p,f,label):return w(d,tuple(p),f,label,bare_yaw,80.,bare_heading)
            Rr.append(bw(1.5,bare_tcp-.060*bare_approach,-.014,'open outside exposed acrylic on the near sloped edge'))
            Rr.append(bw(1.5,bare_tcp,-.014,'slide fingers around the exposed acrylic'))
            Rr.append(bw(1.2,bare_tcp,-.0018,'pinch bare acrylic without the gel between the pads'))
            Rr.append(bw(.4,bare_tcp,-.0018,'settle the bare-acrylic hold'))
            L.append(lw(sum(x.duration for x in Rr)-sum(x.duration for x in L),unload,sf,'hold the slightly lowered flap during acrylic pickup'))
            L.append(lw(.7,held,sf,'resume the peel with the acrylic held'))
    last = (W * 1000 * 7 - W * 1000 * 6 * math.cos(math.radians(80)), W * 1000 * 6 * math.sin(math.radians(80)) + 2)
    L.append(lw(2.2, side_tcp(*last), sf, 'left hand peels the far corner'))
    Rr.append(bw(sum(x.duration for x in L)-sum(x.duration for x in Rr),bare_tcp,-.0018,'hold bare acrylic through the last peel'))
    Rr.append(bw(.7,bare_tcp,-.014,'release the acrylic on the dish'))
    Rr.append(bw(1.5,bare_tcp-.045*bare_approach,-.014,'withdraw the right fingers'))
    Rr.append(park(2.,'right','right hand clear'))
    # Once the planned peel is complete, a gentler pinch suffices for carrying and laying the free sheet.
    sf=-.00065
    xl0, yl = LAY[lot.name][0] - R, LAY[lot.name][1]
    hang = W * 6 + .002
    def held_tcp(x,y,z):
        return tuple(np.array([x,y,z])+material_offset-grip_rotation@grip_contact_offset+grip_bias)
    top_z = TOP + .018 + hang + .006
    cur = side_tcp(*last)
    L.append(lw(1.5, (cur[0], cur[1], max(cur[2], held_tcp(0, 0, top_z)[2])), sf, 'raise the side-held sheet'))
    L.append(lw(2.5, held_tcp(xl0 + 2 * R + .004, (sy + yl) / 2, top_z), sf, 'carry the sheet off the stand'))
    L.append(lw(2., held_tcp(xl0 + 2 * R + .004, yl, top_z), sf, 'over the free counter'))
    L.append(lw(1.5, held_tcp(xl0 + 2 * R + .004, yl, TOP + hang), sf, 'far corner down'))
    end = held_tcp(xl0 + W, yl, TOP + .0015)
    L.append(lw(7., end, sf, 'lay the sheet toward the near corner'))
    L.append(lw(1., end, -.030, 'release the side pinch'))
    L.append(lw(1.5, (end[0], end[1], end[2] + .08), -.030, 'left fingers clear'))
    L.append(park(2.5, 'left', 'left hand clear'))
    total = max(sum(x.duration for x in L), sum(x.duration for x in Rr)) + 3.5
    for plan, arm in ((L, 'left'), (Rr, 'right')):
        plan.append(park(total - sum(x.duration for x in plan), arm, 'hold still'))
    return {'left': L, 'right': Rr}


parts._gel_namespace = globals()
