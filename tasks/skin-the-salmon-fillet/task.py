"""A chilled skin-on salmon portion lies skin side down on a white cutting board on the kitchen counter, its tail end
to the robot's left. A 40 mm tab of bare grey skin sticks out beyond the tail end of the pink flesh. A black-handled
flexible filleting knife lies flat on the counter to the right of the board, blade pointing away from the robot and
its cutting edge toward the right. A white side plate stands on the counter to the left of the board.
"""
from __future__ import annotations
import json
import math
import os
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import parts
import lots
import tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import FINGER_OPEN

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # score() and success() share the measured one-second hold
EPS = 1e-6

# -- scene (metres; x away from the robot, y to its left, z up) -------------------------------------------------
BOARD_T = .020
BOARD_TOP = TOP + BOARD_T
SKIN_T = .0012
IFACE = BOARD_TOP + SKIN_T          # skin-flesh interface: the flesh's bottom face
FX = .335                           # portion centre line, x
FW = .062                           # portion width (x)
YT, YH = .070, -.070                # tail end and head end of the flesh (140 mm long, along y)
TAB = .040                          # bare skin tab beyond the tail end
PINK = ((.96, .55, .42), (.93, .47, .35))
SKIN = (.52, .55, .57)
STEEL = (.74, .76, .78)

# Flesh: 5 lengthwise strips (belly edge thin, back thick) x 4 segments (tail end thinner). Origin on the interface.
STRIP_H = (.014, .021, .024, .022, .016)
SEG_SCALE = (.80, .92, 1.0, 1.0)    # tail -> head
STRIP_W = FW / 5
SEG_L = (YT - YH) / 4
_FLESH_SHAPES = []
_vol = 0.
for i, h0 in enumerate(STRIP_H):
    for k, sc in enumerate(SEG_SCALE):
        h = h0 * sc
        x = -FW / 2 + (i + .5) * STRIP_W
        y = YT - (k + .5) * SEG_L
        _FLESH_SHAPES.append(parts.box((STRIP_W - .0002 if 0 < i < 4 else STRIP_W, SEG_L, h), at=(x, y, h / 2),
                                       color=PINK[(i + k) % 2]))
        _vol += STRIP_W * SEG_L * h
FLESH_MASS = round(_vol * 1050, 3)   # raw salmon about 1.05 g/cm3
FLESH = parts.Part('flesh', pos=(FX, 0., IFACE), mass=FLESH_MASS, shapes=_FLESH_SHAPES, friction=(.6, .5),
                   contact_offset=.0005, angular_damping=2.)

# Skin: a hinged chain. The tab is the root link; links s1..s7 (20 mm each) lie under the flesh, held to it by
# declared bonds (fixed joints) until the cutting rule releases them.
NLINK = 7
LINK_L = (YT - YH) / NLINK
SKIN_MASS_TAB = .0032
SKIN_MASS_LINK = .0016
SKINPART = parts.Part('skin', pos=(FX, YT, IFACE - SKIN_T / 2), mass=SKIN_MASS_TAB,
    shapes=[parts.box((FW, TAB, SKIN_T), at=(0, TAB / 2, 0), color=SKIN)],
    joints=[parts.Hinge(f's{i}', shapes=[parts.box((FW, LINK_L - .0002, SKIN_T), at=(0, -LINK_L * (i - 1) - LINK_L / 2, 0), color=SKIN)],
                        pivot=(0, -LINK_L * (i - 1), 0), axis=(1, 0, 0), lower=-2.7, upper=2.7, mass=SKIN_MASS_LINK,
                        spring=0., damping=.00001, max_effort=.002, parent='base' if i == 1 else f's{i-1}')
            for i in range(1, NLINK + 1)],
    contact_offset=.0003, friction=(.60, .50), link_friction=(('base', (.90, .80)),), angular_damping=1.,
    report=('base',) + tuple(f's{i}' for i in range(1, NLINK + 1)))

# Knife (frame at the handle centre; blade along +x, cutting edge on the -y side, blade in the handle mid-plane).
HANDLE = (.140, .030, .023)
KNIFE_REST_Z = TOP + HANDLE[2] / 2
KNIFE = parts.Part('knife', pos=(.205, -.235, KNIFE_REST_Z), mass=.085, friction=(.9, .7), torsion=.012,
    contact_offset=.0004, angular_damping=2., report=('base',), shapes=[
        parts.box(HANDLE, at=(0, 0, 0), color=(.07, .07, .08)),
        parts.box((.170, .016, .0016), at=(.155, -.003, 0), color=STEEL),
        parts.box((.030, .008, .0016), at=(.255, -.007, 0), color=STEEL)])
EDGE = np.array([[.080, -.011, -.0008], [.270, -.011, -.0008]])   # sharpened edge line, knife frame
BLADE_FRICTION = (.30, .25)   # steel blade on wet skin; the handle keeps the knife's 0.9/0.7 (textured grip)

BOARD = parts.Part('board', pos=(.550, 0., TOP), friction=(.7, .6), shapes=[
    parts.box((.5359, .3683, BOARD_T), at=(0, 0, BOARD_T / 2), color=(.93, .93, .91))])
PLATE_C = (.370, .310)
PLATE_R = .100
PLATE_TOP = TOP + .006
PLATE = parts.Part('plate', pos=(*PLATE_C, TOP), shapes=[parts.cylinder(PLATE_R, .006, at=(0, 0, .003), color=(.97, .97, .96))] +
    [parts.box((.010, .022, .010), at=(float((PLATE_R - .005) * math.cos(a)), float((PLATE_R - .005) * math.sin(a)), .011),
               quat=(0, 0, float(math.sin(a / 2)), float(math.cos(a / 2))), color=(.97, .97, .96)) for a in np.arange(28) * 2 * math.pi / 28])
BACKSPLASH = parts.Part('backsplash', pos=(.870, 0., TOP), shapes=[parts.box((.012, 1.10, .30), at=(0, 0, .15), color=(.86, .85, .82))])
OBJECTS = [BOARD, FLESH, SKINPART, KNIFE, PLATE, BACKSPLASH]
DEMO_TOUCHES = {'board'}

# Two layouts: the knife starts in a different place on the counter. The portion is the same in both.
LOTS = lots.declare(OBJECTS, lots.Lot('A'), lots.Lot('B', place={'knife': (.215, -.265, KNIFE_REST_Z)}))

# -- interface grid (flesh frame) ------------------------------------------------------------------------------
NX, NY = 12, 28
CX = -FW / 2 + (np.arange(NX) + .5) * FW / NX
CY = YT - (np.arange(NY) + .5) * (YT - YH) / NY
CELLS = np.array([(x, y) for x in CX for y in CY])          # index = ix*NY + iy
ROW = np.tile(np.arange(NY), NX)
HIT = .0026                       # the edge line passes within 2.6 mm of a cell centre: it crosses the cell
BAND_LO, BAND_HI = -.0010, .0060  # edge height above the interface that separates the skin
CLEAN = .0010                     # flesh left on the skin over a cell = edge height above 1 mm
GOUGE = .0060                     # the edge inside the flesh higher than this: the flesh is cut into
LOAD_MIN, LOAD_MAX = .5, 40.      # N, blade on skin
STEEP = 25.                       # deg edge-down while loaded: the edge cuts through the skin
EDGE_WINDOW = .5
MAX_OFF_PLANE = math.radians(45)
CELL_AREA = (FW / NX) * ((YT - YH) / NY)
CELL_W = FW / NX
# Cutting resistance while the edge is in uncut interface (estimates; see README): per metre of engaged edge, a feed
# resistance against edge-first motion and a drag against motion along the blade. 62 mm engaged: 2.5 N and 0.9 N.
FEED_N_PER_M = 40.
DRAG_N_PER_M = 15.
SMOOTH = .25                      # s: load and edge-speed window
V_FULL = .005                     # m/s of edge speed at which the resistance reaches its full value
ENGAGE = .008                     # uncut cells within one cell pitch plus HIT of the edge line (in the band) resist the blade


def fresh():
    return dict(known=False, bad=False, why='', cut=np.zeros(NX * NY, bool), left=np.zeros(NX * NY), bonds=set(range(1, NLINK + 1)),
                prev_edge=None, prev_t=None, edge_hist=[], load=0., prev_load=0., maxload=0., tab_held=False, knife_held=False,
                steep=0., clear=False, hold=0., anchor=None, cutting=False, force_on=False, resist=np.zeros(3), resist_link=None,
                engaged=0., load_hist=[])


REC = fresh()


def rot(st): return parts._quat_to_matrix(st.quat_xyzw)
def local(st, p): return (np.asarray(p) - np.asarray(st.pos)) @ rot(st)
def world(st, p): return np.asarray(p) @ rot(st).T + np.asarray(st.pos)
def cut_fraction(): return float(np.mean(REC['cut']))
def loss_fraction(): return float(REC['left'].sum() * CELL_AREA * 1050 / FLESH_MASS)
def fail(why):
    if not REC['bad']: REC['why'] = str(why)
    REC['bad'] = True


def flesh_top(x, y):
    i = np.clip(((x + FW / 2) / STRIP_W).astype(int), 0, 4)
    k = np.clip(((YT - y) / SEG_L).astype(int), 0, 3)
    return np.asarray(STRIP_H)[i] * np.asarray(SEG_SCALE)[k]


def edge_leads(d, blade):
    """True if a blade (rotation in the flesh frame) moving by d cuts: edge first or drawn along its length, within
    45 degrees of its own plane, never spine first or flat side first. Under 1 mm of motion in 0.5 s does not cut."""
    d = np.asarray(d, float); size = np.linalg.norm(d)
    if size < .001: return False
    normal = blade[:, 2]; facing = -blade[:, 1]
    inplane = d - (d @ normal) * normal
    if math.atan2(abs(d @ normal), np.linalg.norm(inplane)) > MAX_OFF_PLANE + EPS: return False
    return d @ facing >= -.2 * size


def sweep(prev, now, cutting):
    """Move the edge (flesh frame, 2x3) from prev to now in <=1 mm steps. Cuts cells when `cutting`; always checks
    for the edge inside the flesh above the interface band."""
    n = max(1, int(math.ceil(np.linalg.norm(now - prev, axis=1).max() / .001)))
    for w in np.linspace(0., 1., n + 1):
        a, b = prev[0] + (now[0] - prev[0]) * w, prev[1] + (now[1] - prev[1]) * w
        m = max(2, int(math.ceil(np.linalg.norm(b - a) / .002)) + 1)
        pts = a + (b - a) * np.linspace(0, 1, m)[:, None]
        inside = (np.abs(pts[:, 0]) <= FW / 2) & (pts[:, 1] <= YT) & (pts[:, 1] >= YH)
        if np.any(inside & (pts[:, 2] > GOUGE + EPS) & (pts[:, 2] < flesh_top(pts[:, 0], pts[:, 1]))):
            fail('knife edge cut into the flesh more than 6 mm above the skin')
        if not cutting: continue
        d2 = b[:2] - a[:2]
        u = np.clip(((CELLS - a[:2]) @ d2) / max(float(d2 @ d2), 1e-15), 0., 1.)
        dist = np.linalg.norm(CELLS - a[:2] - u[:, None] * d2, axis=1)
        h = a[2] + u * (b[2] - a[2])
        hit = (dist <= HIT + EPS) & (h >= BAND_LO - EPS) & (h <= BAND_HI + EPS) & ~REC['cut']
        REC['left'][hit] = np.maximum(REC['left'][hit], np.maximum(0., h[hit] - CLEAN))
        REC['cut'] |= hit


def segment_distance(points, a, b):
    d = b - a
    t = np.clip((points - a) @ d / max(float(d @ d), 1e-15), 0., 1.)
    return np.linalg.norm(points - a - t[:, None] * d, axis=1)


def shape_boxes(part, st):
    poses = {'base': (np.asarray(st.pos), rot(st))}
    poses.update({n: (np.asarray(p), parts._quat_to_matrix(q)) for n, (p, q) in getattr(st, 'links', {}).items()})
    return [(c, r, np.asarray(size)) for _, c, r, size in part.world_boxes(poses)]


def min_gap(points, boxes, cap=.05):
    """Smallest signed distance from the points to the boxes, exact below `cap` (points farther than `cap` from a
    box's bounding sphere are skipped; the result is then at least `cap`)."""
    from yam_arena.analysis import box_signed_distance
    best = float('inf')
    for c, r, s in boxes:
        near = points[np.linalg.norm(points - c, axis=1) <= np.linalg.norm(s) / 2 + cap]
        if len(near): best = min(best, float(box_signed_distance(near, c, r, s).min()))
    return min(best, cap) if best == float('inf') else best


def native_grasps(knife, hands):
    from yam_arena.analysis import box_signed_distance
    out = {arm: False for arm in hands}
    c = world(knife, [[0, 0, 0]])[0]
    for arm in hands:
        pads = hands.pads.get(arm); q = hands.fingers.get(arm)
        if pads is None or q is None or not np.isfinite(q).all() or float(np.sum(np.abs(q))) >= .070: continue
        sides = []
        for pad in pads:
            contact = pad[box_signed_distance(pad, c, rot(knife), np.asarray(HANDLE)) <= .002 + EPS]
            if not len(contact): sides = []; break
            sides.append(np.median(local(knife, contact), axis=0))
        # Pads on opposite faces of the handle: either its two narrow sides (y) or its two flat sides (z).
        out[arm] = len(sides) == 2 and any(sides[0][k] * sides[1][k] < 0 and min(abs(sides[0][k]), abs(sides[1][k])) >= HANDLE[k] / 2 - .004
                                           for k in (1, 2))
    return out


def on_plate(st):
    boxes = shape_boxes(FLESH, st)
    corners = np.array([(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) / 2
    for c, r, s in boxes:
        p = c + (corners * s) @ r.T
        if np.max(np.linalg.norm(p[:, :2] - np.asarray(PLATE_C), axis=1)) > PLATE_R - .008 + EPS: return False
    return abs(float(st.low[2]) - PLATE_TOP) <= .003 + EPS


def skin_off_plate(st):
    return all(np.linalg.norm(c[:2] - np.asarray(PLATE_C)) > PLATE_R + EPS for c, _, _ in shape_boxes(SKINPART, st))


def supported(st, levels):
    return min(abs(float(st.low[2]) - z) for z in levels) <= .003 + EPS


def marks(objects):
    out = [world(objects['flesh'], [[x, y, z] for x in (-.03, .03) for y in (-.06, .06) for z in (0, .01)]),
           world(objects['knife'], [[x, y, 0] for x in (-.07, .27) for y in (-.012, .012)]),
           np.array([objects['skin'].pos] + [p for p, _ in objects['skin'].links.values()])]
    return np.concatenate(out)


def observe(objects, hands, t, load):
    """One control step of the declared cutting rule and the whole-trial records."""
    r = REC
    if r['prev_t'] is not None and t <= r['prev_t']: return
    dt = .05 if r['prev_t'] is None else min(.1, max(0., t - r['prev_t']))
    r['prev_t'] = float(t)
    if hands is None or not all(n in objects for n in ('flesh', 'skin', 'knife')) or not np.isfinite(load):
        r['known'] = False; r['hold'] = 0.; fail('unknown hand, object or load state'); return
    r['known'] = True
    flesh, skin, knife = objects['flesh'], objects['skin'], objects['knife']
    for n in ('flesh', 'skin', 'knife'):
        if float(objects[n].low[2]) < TOP - .010: fail(n + ' fell off the counter')
    held = native_grasps(knife, hands)
    r['knife_held'] = any(held.values())
    edge_w = world(knife, EDGE)
    # Safety: the edge never near a hand that is not holding the knife by its handle; the blade never on the holder.
    blade = shape_boxes(KNIFE, knife)[1:]
    for arm, pts in hands.items():
        if held[arm]:
            if min_gap(pts, blade) < .002 - EPS: fail('blade touched the gripper holding the knife')
        elif float(segment_distance(pts, edge_w[0], edge_w[1]).min()) < .010 - EPS:
            fail('knife edge within 10 mm of the other gripper')
    tab = shape_boxes(SKINPART, skin)[:1]
    r['tab_held'] = any(min_gap(pts, tab) <= .002 + EPS for arm, pts in hands.items() if not held[arm])
    r['load'] = float(load); r['maxload'] = max(r['maxload'], float(load))
    if load > LOAD_MAX + EPS: fail('blade pressed on the skin with more than 40 N')
    facing = -rot(knife)[:, 1]
    r['steep'] = math.degrees(math.asin(float(np.clip(-facing[2], -1, 1))))
    e = local(flesh, edge_w)
    near_skin = e[:, 2].min() <= .003 and np.any((np.abs(e[:, 0]) <= FW / 2 + .02) & (e[:, 1] <= YT + TAB) & (e[:, 1] >= YH - .005))
    if load >= LOAD_MIN - EPS and near_skin and r['steep'] > STEEP + EPS: fail('edge pressed into the skin steeper than 25 degrees: skin cut through')
    r['edge_hist'] = [(tt, ee) for tt, ee in r['edge_hist'] if tt >= t - EDGE_WINDOW - 1e-9] + [(float(t), e.copy())]
    blade_rot = rot(flesh).T @ rot(knife)
    leads = edge_leads(e.mean(0) - r['edge_hist'][0][1].mean(0), blade_rot)
    # Load over the last 0.25 s (contact chatters at 20 Hz); edge speed over the last 0.25 s in the flesh frame.
    r['load_hist'] = [(tt, ll) for tt, ll in r['load_hist'] if tt >= t - SMOOTH - 1e-9] + [(float(t), float(load))]
    loaded = max(ll for _, ll in r['load_hist']) >= LOAD_MIN - EPS
    recent = [ee for tt, ee in r['edge_hist'] if tt >= t - SMOOTH - 1e-9]
    span = max(SMOOTH, 1e-3)
    v = (e.mean(0) - recent[0].mean(0)) / span; v[2] = 0.
    f_hat = -blade_rot[:, 1].copy(); f_hat[2] = 0.; f_hat /= max(np.linalg.norm(f_hat), 1e-9)
    a_hat = blade_rot[:, 0].copy(); a_hat[2] = 0.; a_hat /= max(np.linalg.norm(a_hat), 1e-9)
    ready = bool(leads and r['knife_held'] and r['tab_held'] and loaded)
    # Cells are credited only in a step during which the cutting resistance was acting on the knife.
    r['cutting'] = bool(ready and r['force_on'])
    before = r['prev_edge']
    if before is not None: sweep(before, e, r['cutting'])
    r['prev_edge'] = e.copy()
    # Resistance for the next step: uncut cells within ENGAGE of the edge line, in the band, while the stroke goes on.
    # It opposes the edge's recent motion and grows with speed up to its full value at V_FULL, so a stopped knife
    # is not pushed back (like friction, not a spring).
    r['force_on'] = False; r['resist'] = np.zeros(3); r['resist_link'] = None; r['engaged'] = 0.
    if ready and before is not None:
        a, b = e[0], e[1]; d2 = b[:2] - a[:2]
        u = np.clip(((CELLS - a[:2]) @ d2) / max(float(d2 @ d2), 1e-15), 0., 1.)
        dist = np.linalg.norm(CELLS - a[:2] - u[:, None] * d2, axis=1); h = a[2] + u * (b[2] - a[2])
        eng = ~r['cut'] & (dist <= ENGAGE + EPS) & (h >= BAND_LO - EPS) & (h <= BAND_HI + EPS)
        cols = len(set((np.flatnonzero(eng) // NY).tolist()))
        vf = max(0., float(v @ f_hat)); va = float(v @ a_hat)
        if cols and (vf > 1e-4 or abs(va) > 1e-4):
            L = cols * CELL_W
            f_loc = -L * (FEED_N_PER_M * min(1., vf / V_FULL) * f_hat + DRAG_N_PER_M * float(np.clip(va / V_FULL, -1., 1.)) * a_hat)
            r['resist'] = rot(flesh) @ f_loc
            r['force_on'] = True; r['engaged'] = L
            ymid = float(e[:, 1].mean())
            r['resist_link'] = int(np.clip(math.floor((YT - ymid) / LINK_L) + 1, 1, NLINK))
    if loss_fraction() > .10 + EPS: fail('more than 10% of the flesh left on the skin')
    # Hold: released, clear, supported and still for 1 s.
    allpts = np.concatenate(list(hands.values()))
    boxes = shape_boxes(FLESH, flesh) + shape_boxes(SKINPART, skin) + shape_boxes(KNIFE, knife)
    r['clear'] = min_gap(allpts, boxes) >= .030 - EPS
    still = all(objects[n].resting(.02, .5) for n in ('flesh', 'skin', 'knife'))
    sup = (supported(flesh, (TOP, BOARD_TOP, IFACE, PLATE_TOP)) and supported(knife, (TOP, BOARD_TOP, IFACE, PLATE_TOP))
           and supported(skin, (TOP, BOARD_TOP, PLATE_TOP)))
    cur = marks(objects)
    if not (still and sup and r['clear'] and not r['bad']):
        r['anchor'] = None; r['hold'] = 0.
    elif r['anchor'] is None or np.linalg.norm(cur - r['anchor'], axis=1).max() > .001 + EPS:
        r['anchor'] = cur.copy(); r['hold'] = 0.
    else:
        r['hold'] += dt


def released_links():
    out = []
    for i in range(1, NLINK + 1):
        rows = (ROW >= 4 * (i - 1)) & (ROW < 4 * i)
        if REC['cut'][rows].all(): out.append(i)
    return out


def score(objects):
    r = REC
    if not r['known'] or r['bad'] or not r['clear'] or r['hold'] < 1. - EPS: return 0.
    if loss_fraction() > .10 + EPS: return 0.
    q = cut_fraction()
    if q >= 1. - EPS:
        return 1. if (on_plate(objects['flesh']) and skin_off_plate(objects['skin']) and loss_fraction() <= .05 + EPS) else .75
    return .5 if q >= .5 - EPS else .25 if q >= .25 - EPS else 0.


def success(objects): return score(objects) == 1.


# -- hands: face-sampled hulls (3 mm), pads and finger joints -----------------------------------------------
_FACE_CACHE = {}


def face_points(name):
    if name in _FACE_CACHE: return _FACE_CACHE[name]
    from yam_arena import analysis
    pts = []
    for h in analysis.link_hulls().get(name, []):
        pts.append(h.vertices)
        for plane in np.unique(np.round(h.planes, 7), axis=0):
            n, d = plane[:3], plane[3]
            on = h.vertices[np.abs(h.vertices @ n - d) < 1e-6]
            if len(on) < 3: continue
            c = on.mean(axis=0); u = on[0] - c; u /= np.linalg.norm(u); v = np.cross(n, u)
            ring = on[np.argsort(np.arctan2((on - c) @ v, (on - c) @ u))]
            for a, b in zip(ring, np.roll(ring, -1, axis=0)):
                count = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / .003)))
                ii, jj = np.meshgrid(np.arange(count + 1), np.arange(count + 1)); valid = ii + jj <= count
                pts.append(c + (a - c) * ii[valid, None] / count + (b - c) * jj[valid, None] / count)
    if not pts: raise ValueError('missing hand hull ' + name)
    p = np.concatenate(pts); _, ix = np.unique(np.round(p / .0015).astype(int), axis=0, return_index=True)
    _FACE_CACHE[name] = p[np.sort(ix)]; return _FACE_CACHE[name]


class HandClouds(dict):
    """Collision-face points plus native pad and finger observations; unavailable data never implies a grasp."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs); self.pads = {}; self.fingers = {}


def hand_clouds(sim):
    base = sim.env.unwrapped; origin = parts._np(base.scene.env_origins[0]); out = HandClouds()
    for arm in ('left', 'right'):
        asset = base.scene[arm + '_robot']; poses = parts._np(asset.data.body_link_pose_w)[0]; pts = []; named = {}
        for n in ('gripper', 'tip_left', 'tip_right'):
            i = list(asset.body_names).index(n); pose = poses[i]
            named[n] = face_points(n) @ parts._quat_to_matrix(pose[3:]).T + pose[:3] - origin; pts.append(named[n])
        out[arm] = np.concatenate(pts)
        out.pads[arm] = (named['tip_left'], named['tip_right'])
        q = parts._np(asset.data.joint_pos)[0]; out.fingers[arm] = np.array([q[list(asset.joint_names).index(n)] for n in ('joint7', 'joint8')])
    if not all(np.isfinite(p).all() for p in out.values()): raise ValueError('nonfinite hands')
    return out


# -- declared bonds, knife-flesh filter, skin root -------------------------------------------------------------
_PATHS = {}; _BONDS = {}; _orig_author = parts._author


def author(part, root_path, translation, orientation):
    root = _orig_author(part, root_path, translation, orientation); _PATHS[part.name] = root_path
    if not all(n in _PATHS for n in ('flesh', 'skin', 'knife')) or _BONDS: return root
    from pxr import Gf, Sdf, UsdPhysics
    from isaaclab.sim.utils import get_current_stage
    stage = get_current_stage()
    # The tab (the chain's first link) is the articulation root, not a middle link.
    UsdPhysics.ArticulationRootAPI(stage.GetPrimAtPath(_PATHS['skin'])).GetPrim().RemoveAPI(UsdPhysics.ArticulationRootAPI)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(_PATHS['skin'] + '/base'))
    for i in range(1, NLINK + 1):
        j = UsdPhysics.FixedJoint.Define(stage, _PATHS['skin'] + f'/bond{i}')
        j.CreateBody0Rel().SetTargets([Sdf.Path(_PATHS['flesh'])]); j.CreateBody1Rel().SetTargets([Sdf.Path(_PATHS['skin'] + f'/s{i}')])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(0., float(YT - LINK_L * (i - 1)), float(-SKIN_T / 2)))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1.)); j.CreateLocalRot1Attr().Set(Gf.Quatf(1.))
        j.CreateExcludeFromArticulationAttr(True); j.CreateCollisionEnabledAttr().Set(True); _BONDS[i] = str(j.GetPath())
    # The knife passes through the flesh: its cutting is the declared rule above. Knife-skin and knife-board contact stay.
    pairs = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(_PATHS['knife']))
    pairs.CreateFilteredPairsRel().SetTargets([Sdf.Path(_PATHS['flesh'])])
    # The steel blade gets its own (wet steel) friction; the handle keeps the knife's material.
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.materials.physics_materials import spawn_physics_material
    from isaaclab.sim.utils import bind_physics_material
    mat = _PATHS['knife'] + '/blade_friction'
    spawn_physics_material(mat, sim_utils.RigidBodyMaterialCfg(static_friction=BLADE_FRICTION[0], dynamic_friction=BLADE_FRICTION[1]))
    for i in range(1, len(KNIFE.shapes)): bind_physics_material(_PATHS['knife'] + f'/s{i}', mat, stage=stage)
    return root


parts._author = author


def bond(i, on):
    from pxr import UsdPhysics
    from isaaclab.sim.utils import get_current_stage
    UsdPhysics.Joint(get_current_stage().GetPrimAtPath(_BONDS[i])).CreateJointEnabledAttr().Set(bool(on))


@parts.on_pre_reset
def prepare_reset(sim):
    for i in _BONDS: bond(i, True)
    REC['force_on'] = False
    try:
        apply_resistance(sim)
    except Exception:
        pass


@parts.on_reset
def reset(sim):
    global REC
    REC = fresh()
    for i in _BONDS: bond(i, True)
    try:
        apply_resistance(sim)
    except Exception:
        pass


_VIEW = {}


def tab_load(sim):
    """Contact force between the grippers' finger tips and the skin tab (logged only; not used for scoring)."""
    try:
        from isaaclab_physx.physics import PhysxManager
        from isaaclab.sim.utils import get_current_stage
        from pxr import Usd, UsdPhysics
        base = sim.env.unwrapped
        if 'tab' not in _VIEW:
            stage = get_current_stage()
            tips = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies())
                    if p.GetName() in ('tip_left', 'tip_right') and p.HasAPI(UsdPhysics.RigidBodyAPI)]
            _VIEW['tab'] = PhysxManager.get_physics_sim_view().create_rigid_contact_view(_PATHS['skin'] + '/base', filter_patterns=tips, max_contact_data_count=64) if tips else None
        if _VIEW['tab'] is None: return float('nan')
        fm = parts._np(_VIEW['tab'].get_contact_force_matrix(dt=base.physics_dt)).reshape(-1, 3)
        return float(np.linalg.norm(fm.sum(axis=0)))
    except Exception:
        _VIEW['tab'] = None
        return float('nan')


def blade_load(sim):
    from isaaclab_physx.physics import PhysxManager
    base = sim.env.unwrapped
    if 'v' not in _VIEW:
        sv = PhysxManager.get_physics_sim_view()
        _VIEW['v'] = sv.create_rigid_contact_view(_PATHS['knife'], filter_patterns=[_PATHS['skin'] + '/' + n for n in ('base',) + tuple(f's{i}' for i in range(1, NLINK + 1))], max_contact_data_count=128)
    fm = parts._np(_VIEW['v'].get_contact_force_matrix(dt=base.physics_dt)).reshape(-1, 3)
    return float(np.linalg.norm(fm, axis=1).sum()) if np.isfinite(fm).all() else float('nan')


def _set_force(base, name, forces):
    import logging, warnings, torch
    asset = base.scene[name]; dev = asset.device
    f = torch.tensor([forces], dtype=torch.float32, device=dev)
    log = logging.getLogger(type(asset).__module__); level = log.level; log.setLevel(logging.ERROR)
    try:
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            asset.set_external_force_and_torque(f, torch.zeros_like(f), env_ids=torch.tensor([0], device=dev), is_global=True)
    finally:
        log.setLevel(level)


def apply_resistance(sim):
    """The cutting resistance on the knife (at its centre of mass) and the equal reaction on the skin link under the
    edge: the knife hand has to push through it and the hand on the tab has to hold the skin against it."""
    base = sim.env.unwrapped
    F = np.asarray(REC['resist'], float) if REC['force_on'] else np.zeros(3)
    knife = base.scene['knife']; _set_force(base, 'knife', [list(map(float, F))] * int(knife.num_bodies))
    skin = base.scene['skin']; names = list(skin.body_names)
    forces = [[0., 0., 0.] for _ in names]
    if REC['force_on'] and REC['resist_link'] is not None:
        forces[names.index(f"s{REC['resist_link']}")] = list(map(float, -F))
    _set_force(base, 'skin', forces)


@parts.on_step
def step(sim):
    try:
        objects = sim.states(); hands = hand_clouds(sim); load = blade_load(sim)
        observe(objects, hands, float(sim.t), load)
        apply_resistance(sim)
        for i in released_links():
            if i in REC['bonds']: bond(i, False); REC['bonds'].discard(i)
        log = os.environ.get('SKIN_LOG') or (str(Path(os.environ['TRACE_OUT']).with_name('skinning.jsonl')) if os.environ.get('TRACE_OUT') else None)
        if log and round(sim.t * 20) % 5 == 0:
            from yam_arena import ik, station
            base = sim.env.unwrapped
            row = dict(t=float(sim.t), q=cut_fraction(), uncut=np.flatnonzero(~REC['cut']).tolist()[:40], bonds=sorted(REC['bonds']), loss=loss_fraction(), load=REC['load'], maxload=REC['maxload'],
                       tab=REC['tab_held'], tab_load=tab_load(sim), held=REC['knife_held'], cutting=REC['cutting'], resist=float(np.linalg.norm(REC['resist'])) if REC['force_on'] else 0., engaged=REC['engaged'], resist_link=REC['resist_link'], steep=REC['steep'], bad=REC['bad'], why=REC['why'],
                       clear=REC['clear'], hold=REC['hold'], score=score(objects), lot=lots.current().name,
                       edge=(REC['prev_edge'].tolist() if REC['prev_edge'] is not None else None),
                       flesh=objects['flesh'].pos.tolist(), flesh_q=objects['flesh'].quat_xyzw.tolist(), flesh_low=float(objects['flesh'].low[2]),
                       skin=objects['skin'].pos.tolist(), skin_links={k: v[0].tolist() for k, v in objects['skin'].links.items()},
                       knife=objects['knife'].pos.tolist(), knife_q=objects['knife'].quat_xyzw.tolist(), tcp={}, fingers={})
            for side in ('left', 'right'):
                robot = base.scene[side + '_robot']; ix = sim.embodiment._ids[side + '_robot'][0]
                qj = parts._np(robot.data.joint_pos)[0]
                row['tcp'][side] = ik.tcp_pose(qj[ix], station.BASES[side])[0].tolist()
                row['fingers'][side] = [float(qj[list(robot.joint_names).index(n)]) for n in ('joint7', 'joint8')]
            with Path(log).open('a') as f: f.write(json.dumps(row) + '\n')
    except Exception as e:
        REC['known'] = False; fail(e)
        if os.environ.get('TRACE_OUT'):
            with Path(os.environ['TRACE_OUT']).with_name('skinning-errors.txt').open('a') as f: f.write(repr(e) + '\n')
        raise


# -- open-loop demo from the published layout (no object, force or hidden state feedback) ----------------------
CAL = dict(knife_tcp_above_handle=.0035, press=.0035, pin_press=.030, sag_knife=0., sag_flesh=0., grip_above=.016, pin_lean=40.)


def demo():
    lot = lots.current()
    kp = np.array(lot.pos('knife'))
    def wp(d, p, f, label, yaw=0., lean=0., heading=0.):
        return tilt.Waypoint(d, tuple(float(v) for v in p), f, label, yaw=yaw, tilt=lean, heading=heading)
    up = TOP + .115
    gz = kp[2] + CAL['knife_tcp_above_handle']
    hx = .200
    # Handle centre during the cut: blade bottom on the skin, edge on the line y; TCP above the handle centre.
    cutz = IFACE + .0008 + CAL['knife_tcp_above_handle'] - CAL['press'] + CAL['sag_knife']
    y0 = YT + .004 + .011            # edge 4 mm out on the tab
    y1 = YH - .028 + .011            # edge 28 mm past the head end: the whole blade clear of the flesh
    lean = dict(lean=4., heading=math.pi / 2)  # edge 4 degrees down
    right = [wp(2., (kp[0], kp[1], up), FINGER_OPEN, 'above knife handle'),
             wp(2., (kp[0], kp[1], gz), FINGER_OPEN, 'around handle'),
             wp(1., (kp[0], kp[1], gz), -.010, 'grip handle'),
             wp(1.5, (kp[0], kp[1], up), -.010, 'lift knife flat'),
             wp(2.5, (hx, y0, up), -.010, 'carry over the tab', **lean),
             wp(2., (hx, y0, cutz + .012), -.010, 'lower toward the tab', **lean),
             wp(1., (hx, y0, cutz), -.010, 'blade on the tab at the tail end', **lean),
             wp(2.5, (hx, y0, cutz), -.010, 'wait for the tab to be held', **lean)]
    n = 18
    for k in range(1, n + 1):
        y = y0 + (y1 - y0) * k / n
        right.append(wp(.55, (hx, y, cutz), -.010, 'push the edge along the interface', **lean))
    right += [wp(.6, (hx, y1, cutz), -.010, 'let the blade catch up past the head end', **lean),
              wp(1.5, (hx, y1, up), -.010, 'lift knife clear'),
              wp(2.5, (kp[0], kp[1], up), -.010, 'carry knife back'),
              wp(2., (kp[0], kp[1], gz + .004), -.010, 'lower knife to the counter'),
              wp(1., (kp[0], kp[1], gz + .004), FINGER_OPEN, 'release knife'),
              wp(1.5, (kp[0], kp[1], up + .05), FINGER_OPEN, 'clear knife'),
              wp(12., (.15, -.25, up + .05), FINGER_OPEN, 'stay clear')]
    t_cut_end = sum(w.duration for w in right[:8 + n]) + 2.1
    # Closed fingers leaned 40 degrees: their lowest point is 10.8 mm below the TCP and 5.5 mm toward -y.
    pin = np.array([FX, YT + .036, IFACE + .0108 - CAL['pin_press']])
    pl = dict(lean=CAL['pin_lean'], heading=-math.pi / 2)  # wrist leaned away from the knife arm
    fz = IFACE + CAL['grip_above'] + CAL['sag_flesh']
    drop = np.array([PLATE_C[0], PLATE_C[1], PLATE_TOP + .004 + CAL['grip_above'] + CAL['sag_flesh']])
    t_down = sum(w.duration for w in right[:7])
    left = [wp(t_down - 2., (pin[0], pin[1], up + .03), 0., 'wait above the tab', **pl),
            wp(2., pin + [0, 0, .02], 0., 'lower closed fingers to the tab', **pl),
            wp(1., pin, 0., 'hold the tab down on the board', **pl),
            wp(t_cut_end - t_down - 1., pin, 0., 'hold the tab while the knife cuts', **pl),
            wp(1., pin + [0, 0, .04], FINGER_OPEN, 'let go of the tab', **pl),
            wp(2., (FX, 0., fz + .06), FINGER_OPEN, 'above the flesh', yaw=-math.pi / 2),
            wp(1.5, (FX, 0., fz), FINGER_OPEN, 'around the flesh', yaw=-math.pi / 2),
            wp(1., (FX, 0., fz), -.026, 'grip the flesh', yaw=-math.pi / 2),
            wp(1.5, (FX, 0., fz + .07), -.026, 'lift the flesh off the skin', yaw=-math.pi / 2),
            wp(2.5, drop + [0, 0, .06], -.026, 'carry to the plate', yaw=-math.pi / 2),
            wp(1.5, drop, -.026, 'lower onto the plate', yaw=-math.pi / 2),
            wp(1., drop, FINGER_OPEN, 'release the flesh', yaw=-math.pi / 2),
            wp(1.5, drop + [0, 0, .08], FINGER_OPEN, 'clear the plate', yaw=-math.pi / 2),
            wp(4., (.20, .30, up + .05), FINGER_OPEN, 'stay clear')]
    return {'left': left, 'right': right}
