"""A fabrication counter with a cured GelSlim 4.0 gel mold. The mold is a black printed hexagonal outer ring with a
clear acrylic hexagon seated on its bottom ledge; a pale pink 3 mm cured silicone gel sheet fills the ring above the
acrylic. The acrylic's back is exposed through the ring's bottom opening. Beside the mold stands an upturned clear
glass crystallizing dish, narrower than that opening, used as a push-out stand. A millimetre rule lies at the back of
the counter, in front of a grey upstand.
"""
from __future__ import annotations
import json, math, os, pathlib, sys
from types import SimpleNamespace
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts, lots, tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint, FINGER_OPEN, FINGER_CLOSED

SCENE = 'station'; TOP = TABLE_TOP[SCENE]; SCORE_OVER = 'end'; SUCCESS_HOLD_S = .05
EPS = 1e-7
# Native mold (original outer_ring.STL / hex_acrylic_base.DXF), metres.
R = .08505256; RO = .09659956; AF = RO * math.sqrt(3) / 2; H = .003; BH = .00635
N = 7; W = 2 * R / N
GEL_MASS = .077
# Declared material rules (rule 9 stand-ins; see task.json fidelity).
BOND = .45        # N per bonded 24.3 mm band, pulling it onto the acrylic
GRIP = 300.       # N/m: a bonded band is also held where it was cast (0.9 N at the 3 mm tear limit)
LIFT = .001       # a band at the peel front lets go after its centre is held >= 1 mm up ...
DWELL = .20       # ... for 0.2 s (peel no faster than the front can follow)
YANK = .005       # a front band lifted 5 mm before it lets go tears (pulled too fast)
HOLE = .0025      # a bonded band that is not at the front, lifted 2.5 mm, tears (pulled from the middle)
SHEAR = .003      # a bonded band dragged 3 mm along the acrylic tears
CRUSH = 15.0      # N summed robot contact on one band crushes/cuts it
NAIL_N = .3       # N: a fingertip pressed into a corner's cut edge at the bond line starts the peel there
NAIL_Z = .0025    # below the sheet's top face (within 2.5 mm of the acrylic face) ...
NAIL_R = .008     # ... and within 8 mm of the corner
CURL = -1.1        # rad: the started corner band's hinge is driven toward this (0.001 N m/rad spring), standing in for
                  # the fingertip sliding under it; it settles near 45 degrees up and is set back to 0 once the next band lets go
FIT = 2.5         # N press fit of the acrylic (and silicone grip) in the ring, lost after 2 mm of push-out travel
FIT_TRAVEL = .002
# Layout: the assembled mold is in the same place in both layouts (its gel is a jointed part); the stand moves.
MX, MY = .380, 0.
STAND_R, STAND_H = .045, .050
STAND_XY = {'A': (.200, 0.), 'B': (.226, 0.)}
LAY = {'A': (.270, -.200), 'B': (.255, -.215)}   # where the demo lays the sheet (centre); any flat free place scores
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
RING = parts.Part('outer_ring', pos=(MX, MY, TOP), mass=.14, shapes=ring_shapes(), friction=(.7, .6), contact_offset=.0002)
BASE = parts.Part('acrylic_base', pos=(MX, MY, TOP + .005 + BH / 2), mass=.145, shapes=(prism(hexagon(R), -BH / 2, BH / 2, CLEAR),),
                  friction=(.45, .4), contact_offset=.0002)
STRIPS = []; AREAS = []
for k in range(N):
    poly = clip(clip(hexagon(R), 0, -R + k * W + (.00003 if k else 0.), True), 0, -R + (k + 1) * W - (.00003 if k < N - 1 else 0.), False)
    AREAS.append(abs(sum(p[0] * q[1] - p[1] * q[0] for p, q in zip(poly, poly[1:] + poly[:1]))) / 2)
    STRIPS.append((prism(poly, -H / 2, H / 2, GEL),))
LINK = ['base'] + [f's{i}' for i in range(1, N)]
GEL_SHEET = parts.Part('gel_sheet', pos=(MX, MY, TOP + .005 + BH + H / 2), mass=GEL_MASS * AREAS[0] / sum(AREAS), shapes=STRIPS[0],
    joints=tuple(parts.Hinge(LINK[k], shapes=STRIPS[k], pivot=(-R + k * W, 0., 0.), axis=(0., 1., 0.), lower=-2.6, upper=2.6,
                             mass=GEL_MASS * AREAS[k] / sum(AREAS), spring=.001, damping=.0005, max_effort=.025, parent=LINK[k - 1]) for k in range(1, N)),
    friction=(.7, .55), contact_offset=.00015, report=tuple(LINK))
# Upturned PYREX 3140 crystallizing dish, 90 mm x 50 mm (glass; mass an estimate). A 24-sided prism stands in for the round wall.
DISH = [(STAND_R * math.cos(2 * math.pi * i / 24), STAND_R * math.sin(2 * math.pi * i / 24)) for i in range(24)]
STAND = parts.Part('push_out_stand', pos=(*STAND_XY['A'], TOP + STAND_H / 2), mass=.19, shapes=(prism(DISH, -STAND_H / 2, STAND_H / 2, GLASS),),
                   friction=(.45, .4), contact_offset=.0002)
RULER = parts.Part('millimetre_rule', pos=(.56, -.22, TOP + .0005), shapes=(parts.box((.15, .018, .001), color=(.9, .9, .85)),), friction=(.4, .3))
COUNTER_BACK = parts.Part('counter_upstand', pos=(.72, 0., TOP + .055), shapes=(parts.box((.018, 1.0, .11), color=(.65, .66, .66)),))
OBJECTS = [RING, BASE, GEL_SHEET, STAND, RULER, COUNTER_BACK]
LOTS = lots.declare(OBJECTS, lots.Lot('A'), lots.Lot('B', place={'push_out_stand': (*STAND_XY['B'], TOP + STAND_H / 2)}))
DEMO_TOUCHES = set()
SCORED = (RING, BASE, GEL_SHEET)


def fresh():
    return dict(bond=np.ones(N, bool), started=None, curl=0, delta=None, lifted=np.zeros(N), zero=False, why='', known=False, steps=0, prev=None,
                still=None, clear=None, t=None, views=None, peak=0., seated=True, phys=0, events=[], arms=None)
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
    b = bottoms(RING, st); return bool(level(st) and TOP - .002 <= min(b) <= TOP + .0015 and st.resting(.008, .12))
def base_down(st, stand):
    # Flat on the counter or flat on the stand's top.
    b = min(bottoms(BASE, st)); tops = [TOP] + ([float(stand.high[2])] if stand is not None else [])
    return bool(level(st) and any(t - .002 <= b <= t + .0015 for t in tops) and st.resting(.008, .12))


def evaluate(o):
    if not S['known'] or S['zero'] or S['steps'] == 0 or S['phys'] == 0: return 0.
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
def distance(o): return float(np.linalg.norm(o['gel_sheet'].pos - o['acrylic_base'].pos))


def adhesive_step(o, dt=0.):
    """Declared peel rule on the bands' underside centres, in the acrylic's frame."""
    pp = gel_points(o['gel_sheet']); b = o['acrylic_base']; r = parts._quat_to_matrix(b.quat_xyzw)
    anchors = np.array([(-R + (k + .5) * W, 0., BH / 2) for k in range(N)])
    delta = (pp - b.pos) @ r - anchors
    old = S['bond'].copy(); S['delta'] = delta
    for k in range(N):
        if not old[k]: S['lifted'][k] = 0.; continue
        gap = float(delta[k, 2]); front = (k > 0 and not old[k - 1]) or (k < N - 1 and not old[k + 1])
        if float(np.linalg.norm(delta[k, :2])) > SHEAR: fail(f'tear: bonded band {k} dragged {1000 * np.linalg.norm(delta[k, :2]):.1f} mm along the acrylic')
        if not front:
            S['lifted'][k] = 0.
            if gap > HOLE: fail(f'tear: band {k} pulled {1000 * gap:.1f} mm up before the peel front reached it')
            continue
        if gap > YANK: fail(f'tear: front band {k} lifted {1000 * gap:.1f} mm before letting go (peeled too fast)')
        S['lifted'][k] = S['lifted'][k] + dt if gap > LIFT else 0.
        if S['lifted'][k] >= DWELL - EPS and not S['zero']:
            S['bond'][k] = False; event(f'band {k} released t={S.get("time", 0.):.2f}')
    return pp, delta


def apply_wrenches(sim, o):
    """Bond forces (gel bands onto the acrylic) and the ring's press fit, with equal and opposite reactions."""
    import torch
    base = sim.env.unwrapped; org = parts._np(base.scene.env_origins[0])
    a = base.scene['gel_sheet']; b = base.scene['acrylic_base']; c = base.scene['outer_ring']
    pp = gel_points(o['gel_sheet']); br = parts._quat_to_matrix(o['acrylic_base'].quat_xyzw); nn = br[:, 2]
    delta = S.get('delta')
    f = np.zeros((len(a.body_names), 3)); at = np.zeros_like(f); bf = np.zeros(3); bt = np.zeros(3); bpos = o['acrylic_base'].pos
    for k, n in enumerate(LINK):
        i = list(a.body_names).index(n); at[i] = pp[k] + org
        if S['bond'][k]:
            f[i] = -BOND * nn
            if delta is not None:   # in-plane hold of the bond (adhesion resists sliding as well as lifting)
                f[i] = f[i] - GRIP * (br @ np.array((delta[k, 0], delta[k, 1], 0.)))
            bf -= f[i]; bt += np.cross(pp[k] - bpos, -f[i])
    rf = np.zeros(3); rpos = np.asarray(o['outer_ring'].pos) + parts._quat_to_matrix(o['outer_ring'].quat_xyzw)[:, 2] * .009
    if S['seated']:
        rn = parts._quat_to_matrix(o['outer_ring'].quat_xyzw)[:, 2]; rf = FIT * rn; bf -= FIT * rn
    for asset, forces, points in ((a, f, at), (b, bf[None, :], (bpos + org)[None, :]), (c, rf[None, :], (rpos + org)[None, :])):
        moments = np.zeros_like(forces)
        if asset is b: moments = bt[None, :]
        ff = torch.tensor(forces[None, :, :], dtype=torch.float32, device=asset.device)
        tt = torch.tensor(moments[None, :, :], dtype=torch.float32, device=asset.device)
        pq = torch.tensor(points[None, :, :], dtype=torch.float32, device=asset.device); ids = torch.tensor([0], device=asset.device)
        composer = getattr(asset, 'permanent_wrench_composer', None)
        if composer is not None: composer.set_forces_and_torques_index(forces=ff, torques=tt, positions=pq, env_ids=ids, is_global=True)
        else: asset.set_external_force_and_torque(ff, tt, positions=pq, env_ids=ids, is_global=True)


def fit_step(o):
    ring = o['outer_ring']; rr = parts._quat_to_matrix(ring.quat_xyzw)
    z = float(((o['acrylic_base'].pos - ring.pos) @ rr)[2]) - (.005 + BH / 2)
    if S['seated'] and z > FIT_TRAVEL: S['seated'] = False; event(f'press fit released t={S.get("time", 0.):.2f}')


# Contact sensors on the seven gel bands, read at every 240 Hz physics substep.
def arr(x): return np.asarray(x.numpy() if hasattr(x, 'numpy') else x)
def contact_view():
    from isaaclab_physx.physics import PhysxManager
    from isaaclab.sim.utils import get_current_stage
    from pxr import Usd, UsdPhysics
    stage = get_current_stage()
    arms = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies())
            if any(k in str(p.GetPath()) for k in ('/LeftArm/', '/RightArm/')) and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if not arms: raise ValueError('arm contacts unavailable')
    paths = arms
    S['arms'] = paths; S['fingers'] = [i for i, p in enumerate(paths) if p.rstrip('/').split('/')[-1] in ('tip_left', 'tip_right')]
    if not S['fingers']: raise ValueError('fingertip contacts unavailable')
    return PhysxManager.get_physics_sim_view().create_rigid_contact_view([parts._PATHS['gel_sheet'] + '/' + n for n in LINK],
                                                                       filter_patterns=[paths for _ in LINK], max_contact_data_count=512)
def contact_damage(loads):
    loads = np.asarray(loads, float)
    if not np.isfinite(loads).all(): raise ValueError('contact load unknown')
    S['peak'] = max(S['peak'], float(loads.max(initial=0.)))
    if loads.max(initial=0.) > CRUSH: fail(f'tear: {loads.max():.1f} N robot contact on one band')
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
        force, point, _, _, counts, starts = (arr(x) for x in S['views'].get_contact_data(dt=dt))
        counts = counts.reshape(N, -1); starts = starts.reshape(N, -1); flat = force.reshape(-1); pts = point.reshape(-1, 3)
        material, org = material_states(sim)
        b = material['acrylic_base']; br = parts._quat_to_matrix(b.quat_xyzw); loads = np.zeros(N)
        for k in range(N):
            nail = 0.
            for j, (n, s0) in enumerate(zip(counts[k], starts[k])):
                n, s0 = int(n), int(s0)
                if n < 0: raise ValueError('contact count invalid')
                if not n: continue
                if s0 < 0 or s0 + n > len(flat): raise ValueError('contact buffer incomplete')
                fk = np.abs(flat[s0:s0 + n]); loads[k] += float(fk.sum())
                if k in (0, N - 1) and S['bond'][k] and S['started'] is None and j in S['fingers']:
                    loc = (pts[s0:s0 + n] - org - b.pos) @ br
                    corner = np.array((-R if k == 0 else R, 0.))
                    ok = (np.linalg.norm(loc[:, :2] - corner, axis=1) <= NAIL_R) & (loc[:, 2] <= BH / 2 + NAIL_Z) & (loc[:, 2] >= BH / 2 - .0005)
                    nail += float(fk[ok].sum())
            if nail >= NAIL_N and S['started'] is None:
                S['bond'][k] = False; S['started'] = k; event(f'corner {k} started by a fingertip at the bond line t={S["time"]:.2f}')
        contact_damage(loads)
        S['phys'] += 1
        fit_step(material); adhesive_step(material, dt); apply_wrenches(sim, material)
        if S['started'] is not None:
            joint = 's1' if S['started'] == 0 else f's{N - 1}'
            nxt = 1 if S['started'] == 0 else N - 2
            want = 2 if not S['bond'][nxt] else 1
            if S['curl'] != want:
                sim.set_target('gel_sheet', joint, 0. if want == 2 else (CURL if S['started'] == 0 else -CURL))
                S['curl'] = want; event(f'corner hinge target {"released" if want == 2 else "raised"} t={S["time"]:.2f}')
    except Exception as e:
        S.update(known=False); fail('material state unavailable: ' + str(e))


def observe(o, h, t):
    # Whole-trial history; pure, so offline checks can drive it.
    if h is None or len(h) == 0 or not np.isfinite(h).all():
        S['known'] = False; fail('arm state unknown'); return
    chunks = []; corners = np.array([(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    for spec in (RING, BASE, GEL_SHEET, STAND):
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
        if log and S['steps'] % 5 == 0:
            pp = gel_points(o['gel_sheet'])
            row = dict(t=float(sim.t), lot=lots.current().name, phys=S['phys'], score=score(o), bonds=S['bond'].astype(int).tolist(),
                       lifted=np.round(S['lifted'], 3).tolist(), zero=S['zero'], why=S['why'], peak=round(S['peak'], 3), seated=S['seated'],
                       events=S['events'], clear=bool(S['clear'] is not None),
                       ring=np.round(o['outer_ring'].pos, 4).tolist(), base=np.round(o['acrylic_base'].pos, 4).tolist(),
                       stand=np.round(o['push_out_stand'].pos, 4).tolist(), gel=np.round(pp, 4).tolist(),
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
# the stand and press the ring down off it; hold the stack with one hand while the other works a fingertip in at the
# near corner, pinches the lifted corner and peels the sheet back toward itself; lay the sheet flat and let go.
S2 = math.sqrt(2)
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
    hold = (sx + .0486, sy)   # on band 5, inside the stand's support
    xt = sx - R               # near corner of the gel
    def uv(u, v):             # (u, v) mm in the 45-degree finger frame -> world point at the near corner
        return (xt + (u + v) / S2 / 1000, sy, zb + .0007 + (u - v) / S2 / 1000)
    PI2 = math.pi / 2
    def rw(d, p, f, label): Rr.append(w(d, p, f, label, PI2, 45., 0.))
    uV, vV = .5 / S2, -.5 / S2
    Hu, Hv = (W * 1000 + 1.5) / S2, (W * 1000 - 1.5) / S2
    opn = -.020
    # Press one corner of the ring down off the acrylic, as the maker does ("Press a corner..."): the right hand's
    # lower fingertip on the ring's near corner rim, then on to the gel's near corner.
    ring_top = TOP + seat + .018
    ptcp = (sx - .0855, sy, ring_top + .026)        # lower fingertip (tilted 45 deg) over the rim at 92 mm out
    pre = uv(uV + 20 - 2.0, vV - 12.8)
    # The left hand first moves well clear (+y), so the right hand can turn its fingers without meeting it.
    L.append(w(1.2, (sx, sy + .20, TOP + .12), CL, 'left hand out of the way'))
    Rr.append(w(1.2, Rr[-1].pos, Rr[-1].finger, 'wait for the left hand', Rr[-1].yaw))
    Rr.append(w(1.5, (ptcp[0], ptcp[1], ptcp[2] + .02), opn, 'turn the fingers over the ring corner', PI2, 45., 0.))
    rw(1.0, (ptcp[0], ptcp[1], ptcp[2] + .001), opn, 'fingertip on the ring corner')
    rw(1.5, (ptcp[0], ptcp[1], ptcp[2] - .010), opn, 'press the ring corner down off the acrylic')
    t_press = sum(x.duration for x in Rr)
    rw(1.2, (pre[0], pre[1], pre[2] + .02), opn, 'above the near corner of the sheet')
    tl = sum(x.duration for x in L)
    L.append(w(t_press - tl, (sx, sy + .20, TOP + .12), CL, 'wait for the ring to drop'))
    L.append(w(2.0, (hold[0], hold[1], gel_top + .045), CL, 'above the far side of the sheet'))
    L.append(w(1.5, (hold[0], hold[1], gel_top + .0128 - .002), CL, 'hold the stack down on the stand'))
    rw(t_press + 3.5 + .3 - sum(x.duration for x in Rr), (pre[0], pre[1], pre[2] + .02), opn, 'wait for the hold')
    rw(2.0, pre, opn, 'fingertip beside the corner at the acrylic face')
    rw(1.5, uv(uV + 20 + 1.2, vV - 12.8), opn, 'work the fingertip in at the bond line')
    RP = 26.                  # pinch point: TCP 26 mm up the corner band from its lower edge (fingers over its outer 11 mm)
    u1, v1 = uV + 20 + 1.2, vV - 12.8
    rw(1.0, uv(u1, v1), opn, 'corner lifts off the fingertip')
    pinch = uv(Hu, Hv - RP)
    rw(2.5, pinch, -.0011, 'close on the raised corner')
    rw(.6, pinch, -.0011, 'settle the pinch')
    # ---- 3. peel back toward the right hand, band by band, keeping the peeled part taut at 65-75 degrees.
    def flap_tcp(ex, ez):  # TCP RP mm up the held corner band from its lower edge (ex, ez), mm from (xt, zb)
        return (xt + (ex - RP / S2) / 1000, sy, zb + (ez + 1.5 + RP / S2) / 1000)
    peel = []
    for k, e in ((1, 65), (2, 65), (3, 65), (4, 68), (5, 72), (6, 75)):
        f_k = W * 1000 * k; Lk = W * 1000 * (k - 1); el = math.radians(e)
        peel.append((f_k - Lk * math.cos(el), Lk * math.sin(el) + (4.0 if k == 1 else 5.0)))
    t_hold = None
    for k, (ex, ez) in enumerate(peel):
        rw(2.2 if k else 1.6, flap_tcp(ex, ez), -.0011, f'peel band {k + 1}')
        rw(.4, flap_tcp(ex, ez), -.0011, f'let band {k + 1} go')
        if k == 3: t_hold = sum(x.duration for x in Rr)
    last = (W * 1000 * 7 - W * 1000 * 6 * math.cos(math.radians(80)), W * 1000 * 6 * math.sin(math.radians(80)) + 2)
    rw(2.2, flap_tcp(*last), -.0011, 'peel the far corner')
    t_peel_end = sum(x.duration for x in Rr)
    # The counterhold lets go once band 4 has let go (front on band 5).
    t_lift = t_hold - 1.4
    tl = sum(x.duration for x in L)
    L.append(w(t_lift - tl, (hold[0], hold[1], gel_top + .0128 - .002), CL, 'keep holding'))
    L.append(w(1.0, (hold[0], hold[1], gel_top + .05), CL, 'release the stack'))
    L.append(park(2.0, 'left', 'left arm clear'))
    # ---- 4. carry the hanging sheet to the free counter and lay it flat, far corner first.
    xl0, yl = LAY[lot.name][0] - R, LAY[lot.name][1]   # laid sheet spans x from xl0 to xl0 + 170 mm
    ex_end = peel[-1][0]
    hang = W * 6 + .002               # bands 1-6 hang below the held band's lower edge
    def e_tcp(x, y, ez):              # TCP for the held band's lower edge at world (x, y, ez)
        return (x - RP / 1000 / S2, y, ez + .0015 + RP / 1000 / S2)
    top_z = TOP + .018 + hang + .006
    cur = flap_tcp(*last)
    rw(1.5, (cur[0], cur[1], max(cur[2], e_tcp(0, 0, top_z)[2])), -.0011, 'raise the peeled sheet')
    rw(2.5, e_tcp(xl0 + 2 * R + .004, sy - .5 * (sy - yl), top_z), -.0011, 'carry the sheet off the stand')
    rw(2.0, e_tcp(xl0 + 2 * R + .004, yl, top_z), -.0011, 'over the free counter')
    rw(1.5, e_tcp(xl0 + 2 * R + .004, yl, TOP + hang), -.0011, 'far corner down to the counter')
    rw(7.0, e_tcp(xl0 + W, yl, TOP + .0015), -.0011, 'lay the sheet back toward the near corner')
    end = e_tcp(xl0 + W, yl, TOP + .0015)
    rw(.8, (end[0], end[1], end[2] + .002), -.0011, 'ease up')
    rw(1.0, (end[0], end[1], end[2] + .002), -.003, 'open the pinch')
    back = (end[0] - .025 / S2, end[1], end[2] + .002 + .025 / S2)
    rw(1.5, back, -.003, 'slide the fingers off the corner band')
    rw(1.0, (back[0], back[1], back[2] + .04), -.003, 'lift clear')
    Rr.append(park(2.5, 'right', 'right arm clear'))
    total = max(sum(x.duration for x in L), sum(x.duration for x in Rr)) + 3.5
    for plan, arm in ((L, 'left'), (Rr, 'right')):
        plan.append(park(total - sum(x.duration for x in plan), arm, 'hold still'))
    return {'left': L, 'right': Rr}


parts._gel_namespace = globals()
