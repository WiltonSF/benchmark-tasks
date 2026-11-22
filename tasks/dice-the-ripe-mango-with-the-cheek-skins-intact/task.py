"""One mango cheek, already cut off the stone, lies skin side down on a white cutting board, its flat orange cut face
up and its long axis pointing away from the robot and to the right. A black-handled 76 mm paring knife stands on the flat underside
of its handle near the right edge of the board, blade pointing right, edge on the board. A small white ramekin
stands upside down in the far right corner of the board. A steel ruler lies beyond the board. The cut face has no marks.
"""
from __future__ import annotations
import json
import math
import os
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts
import lots
from yam_arena.task_file import TABLE_TOP
import tilt



def _matrix_to_quat(m):
    """xyzw, w >= 0. The kit's per-component sign rule loses the axis signs for half-turn rotations (w near 0), which
    the scoring knife pose reaches; this standard branch-on-largest-diagonal form keeps them."""
    m = np.asarray(m, float); tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.) * 2; q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, .25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1. + m[0, 0] - m[1, 1] - m[2, 2]) * 2; q = [.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1. + m[1, 1] - m[0, 0] - m[2, 2]) * 2; q = [(m[0, 1] + m[1, 0]) / s, .25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1. + m[2, 2] - m[0, 0] - m[1, 1]) * 2; q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, .25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    return (-q if q[3] < 0 else q) / np.linalg.norm(q)


parts._matrix_to_quat = _matrix_to_quat
SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SCORE_OVER = 'end'
SUCCESS_HOLD_S = 0.1
DEMO_TOUCHES = ('board',)
parts.CONTACT_OFFSET = 0.001
ARMS = ('left', 'right')

# -- scene ---------------------------------------------------------------------------------------------------------
BOARD_H = 0.016
SURFACE = TOP + BOARD_H
BOARD_C = np.array([0.29, 0.0])
BOARD_SIZE = np.array([0.3683, 0.53594])
FLESH = (0.98, 0.62, 0.10)
SKIN = (0.62, 0.30, 0.16)
SKIN_DARK = (0.36, 0.42, 0.14)

# Cheek: one side of a ripe mango cut about 6 mm from the centre plane, beside the flat stone. Cheek frame: origin at
# the centre of the flat cut face, x along the long axis, y across, z up out of the cut face. The skin's outer surface
# is half an ellipsoid (semi-axes A, B, H); the skin is S thick, so the flesh is H - S = 24 mm thick at the centre and
# thins to nothing at the rim.
A, B, H, S = 0.056, 0.036, 0.0255, 0.0015
N_SLAB = 6
SLAB_H = H / N_SLAB


def _cheek_shapes():
    out, top = [], []
    for j in range(N_SLAB):
        zm = -H + (j + .5) * SLAB_H
        k = math.sqrt(1 - (zm / H) ** 2)
        m = 11 if j >= 2 else 7
        edges = np.linspace(-k * B, k * B, m + 1)
        for y0, y1 in zip(edges, edges[1:]):
            ym = (y0 + y1) / 2
            xh = k * A * math.sqrt(max(0., 1 - (ym / (k * B)) ** 2))
            if xh < .002:
                continue
            out.append(parts.box((2 * xh, y1 - y0, SLAB_H), (0, ym, zm), SKIN if j else SKIN_DARK))
    # Flesh-coloured cut face in 2 mm strips, inset 1.5 mm so a thin skin rim shows from above (drawn only).
    for y0 in np.arange(-B + .0015, B - .0015 - 1e-9, .002):
        y1 = min(y0 + .002, B - .0015); ym = (y0 + y1) / 2
        xh = (A - .0015) * math.sqrt(max(0., 1 - (max(abs(y0), abs(y1)) / (B - .0015)) ** 2))
        if xh > .001:
            top.append(parts.box((2 * xh, y1 - y0, .0006), (0, ym, .0003), FLESH, solid=False))
    return out + top


CHEEK_SHAPES = _cheek_shapes()
# Cut marks (drawn only, never scored): 4 mm dark squares on the cut face, hidden at reset and shown once the scorer
# records a completed cut (within 6 mm of the skin) under them, so the video shows where the flesh has been scored.
TILE = .004
TILE_XY = [(x, y) for x in np.arange(-.054, .0541, TILE) for y in np.arange(-.034, .0341, TILE)
           if (x / A) ** 2 + (y / B) ** 2 <= .93]
TILE_START = len(CHEEK_SHAPES)
CHEEK_SHAPES = CHEEK_SHAPES + [parts.box((.0036, .0036, .0004), (x, y, .0008), (.36, .17, .04), solid=False) for x, y in TILE_XY]
CHEEK_Z = SURFACE + H + 0.0003
CHEEK_YAW = -math.pi / 4   # long axis pointing away and to the right; its narrow near end toward the robot's left
CHEEK = parts.Part('cheek', pos=(0.310, -0.090, CHEEK_Z), yaw=CHEEK_YAW, mass=0.115, friction=(0.5, 0.4), shapes=CHEEK_SHAPES)
SOLID_BOXES = [(np.asarray(s.at), s.half) for s in CHEEK_SHAPES if s.solid]
BOX_C = np.array([c for c, _ in SOLID_BOXES]); BOX_H = np.array([h for _, h in SOLID_BOXES])

# Paring knife, frame at the handle's grip centre; blade along +x, edge at z = -12 mm (the handle's flat underside).
Z_KNIFE = SURFACE + 0.012 + 0.0005
KNIFE = parts.Part('knife', pos=(0.240, -0.135, Z_KNIFE), yaw=-math.pi / 2, mass=0.05, friction=(0.7, 0.6), shapes=[
    parts.box((.100, .022, .024), color=(.07, .07, .07)),
    parts.box((.010, .030, .026), (.050, 0, .001), (.09, .09, .09)),
    parts.box((.056, .002, .018), (.078, 0, -.003), (.68, .70, .71), solid=False),
    parts.box((.020, .002, .008), (.116, 0, -.008), (.68, .70, .71), solid=False),
    parts.box((.072, .0025, .0015), (.088, 0, -.0112), (.93, .94, .94), solid=False)])
# Blade: 18 mm tall to 20 mm from the point, then an 8 mm point section (the spine drops toward the point).
BLADE = [(np.array([.078, 0., -.003]), np.array([.056, .002, .018])), (np.array([.116, 0., -.008]), np.array([.020, .002, .008]))]
EDGE = np.column_stack((np.linspace(.050, .126, 77), np.zeros(77), np.full(77, -.012)))
SPINE = np.column_stack((EDGE[:, 0], np.zeros(77), np.where(EDGE[:, 0] <= .106, .006, -.004)))
# Both faces of the 2 mm thick blade (1 mm either side of the edge's plane), each sampled in eight rows from just above
# the edge up to the spine (2.25 mm apart at the heel, 1 mm in the point section) and every 1 mm along the blade, so a
# blade whose edge passes under the cheek's rim while its face or spine is in the flesh cannot cross the skin unseen.
FACE = np.vstack([EDGE + f * (SPINE - EDGE) + np.array([0., side, 0.]) for f in np.arange(1, 9) / 8. for side in (-.001, .001)])
BLADE_PTS = np.vstack((EDGE, FACE))
N_EDGE = len(EDGE)

# Upturned porcelain ramekin: open rim down, base up; its flat base is what the cheek's skin is pressed onto.
RAM_TOP = 0.032
RAMEKIN = parts.Part('ramekin', pos=(0.410, -0.210, SURFACE + 0.0005), mass=0.10, friction=(0.6, 0.5), shapes=[
    parts.cylinder(.030, .006, (0, 0, .003), (.95, .95, .93)),
    parts.cylinder(.027, .020, (0, 0, .016), (.96, .96, .94)),
    parts.cylinder(.024, .006, (0, 0, .029), (.97, .97, .95))])
BOARD = parts.Part('board', pos=(BOARD_C[0], BOARD_C[1], TOP), friction=(0.7, 0.6), shapes=[
    parts.box((BOARD_SIZE[0], BOARD_SIZE[1], BOARD_H), (0, 0, BOARD_H / 2), (0.91, 0.91, 0.87)),
    *[parts.box((0.012, 0.02, 0.004), (x, y, 0.002), (0.16, 0.17, 0.18)) for x in (-0.165, 0.165) for y in (-0.24, 0.24)]])
RULER = parts.Part('ruler', pos=(0.49, 0., TOP + 0.001), shapes=[
    parts.box((0.020, 0.30, 0.002), color=(0.62, 0.65, 0.66)),
    *[parts.box((0.012 if i % 5 == 0 else 0.006, 0.001, 0.0004), (0, i * .01 - .15, .0012), (0.08, .08, .08), solid=False) for i in range(31)]])
OBJECTS = [BOARD, RULER, CHEEK, KNIFE, RAMEKIN]
MOVE = (CHEEK, KNIFE, RAMEKIN)

lots.ABSOLUTE = True
LOTS = lots.declare(OBJECTS,
    lots.Lot('A', place={'cheek': CHEEK.pos, 'knife': KNIFE.pos, 'ramekin': RAMEKIN.pos},
             yaw={'cheek': CHEEK_YAW, 'knife': -math.pi / 2, 'ramekin': 0.}),
    lots.Lot('B', place={'cheek': (0.300, -0.072, CHEEK_Z), 'knife': (0.215, -0.140, Z_KNIFE), 'ramekin': (0.400, -0.200, SURFACE + 0.0005)},
             yaw={'cheek': CHEEK_YAW + math.radians(12), 'knife': -math.pi / 2, 'ramekin': 0.}))

# -- declared material rule (all lengths metres, cheek frame) --------------------------------------------------------
DEPTH_GAP = 0.006      # a cut separates flesh where the edge reached within 6 mm of the skin
SKIN_CUT = 0.001       # an edge more than 1 mm into the 1.5 mm skin has cut through it
RIM_FREE = 0.002       # ...except within 2 mm of the cheek's edge, where the skin turns up to the cut face
RIM_BAND = 0.007       # flesh within 7 mm of the edge is rim, not part of a cube
CUBE_CELLS = (80, 800)  # a cube's top: 80-800 mm2
CUBE_SPAN = 0.030      # ...and no two points more than 30 mm apart
FULL_CUBES, HALF_CUBES = 10, 5
LINE_SPAN = 0.030      # 0.25: one cut to depth at least 30 mm long
DRAG, FEED = 1.0, 1.0  # N: drag opposing the stroke, and feed resistance while the blade moves down, in the flesh
SMOOTH = 5             # control steps (0.25 s) over which the edge's motion is averaged to decide whether a cut counts
LOAD_STEPS = 2         # control steps (0.1 s) over which the edge speed that sets the cutting load is taken
MIN_SPEED = 0.001      # m/s: slower edge motion is not a stroke (no cut)
FULL_SPEED = 0.010     # m/s: drag and feed resistance reach their full 1 N at this edge speed
# Push: full credit needs the skin's middle pushed up 28 mm, 2.5 mm past flat (the 25.5 mm deep cup turned out into a
# dome, so every cut opens at the top and the cubes fan apart); over 35 mm (9.5 mm past flat) while held is a split.
PUSH_MIN, PUSH_MAX, PUSH_R = 0.028, 0.035, 0.015
# Declared skin-bending resistance while the ramekin's base presses into the skin: 0.2 N per mm, at most 5 N (reached
# at 25 mm), changing by at most 0.5 N per control step; it pushes the cheek away from the base and the ramekin back
# down (equal reaction). Full credit needs the full 5 N acting on the step before the push reads 28 mm.
PUSH_K, PUSH_CAP, PUSH_RATE = 200., 5.0, 0.5
PUSH_NEED = min(PUSH_K * PUSH_MIN, PUSH_CAP)
BLADE_CLEAR, TOUCH, HAND_CLEAR = 0.010, 0.0015, 0.005
SETUP_TOL = (0.004, 3.0)

GX = np.round(np.arange(-A, A + 1e-9, .001), 6)
GY = np.round(np.arange(-B, B + 1e-9, .001), 6)
CX, CY = np.meshgrid(GX, GY)
R2 = (CX / A) ** 2 + (CY / B) ** 2
INSIDE = R2 <= 1.
_ring = np.stack((A * np.cos(np.linspace(0, 2 * math.pi, 2000, endpoint=False)), B * np.sin(np.linspace(0, 2 * math.pi, 2000, endpoint=False))), 1)
_pts = np.stack((CX.ravel(), CY.ravel()), 1)
EDGE_DIST = np.min([np.linalg.norm(_pts[:, None, :] - _ring[None, k:k + 250], axis=2).min(1) for k in range(0, 2000, 250)], axis=0).reshape(CX.shape)
del _pts
ELIGIBLE = INSIDE & (EDGE_DIST >= RIM_BAND)


_TI = np.array([[int(round((x + dx + A) / .001)), int(round((y + dy + B) / .001))]
                for x, y in TILE_XY for dx in (-.0015, -.0005, .0005, .0015) for dy in (-.0015, -.0005, .0005, .0015)])
TILE_CELLS = (np.clip(_TI[:, 1], 0, len(GY) - 1) * len(GX) + np.clip(_TI[:, 0], 0, len(GX) - 1)).reshape(len(TILE_XY), 16)


def outer_z(x, y):
    """Outer skin surface below the cut face (cheek frame)."""
    return -H * np.sqrt(np.clip(1 - (np.asarray(x) / A) ** 2 - (np.asarray(y) / B) ** 2, 0., 1.))


def inner_z(x, y):
    """Bottom of the flesh, on the skin (at most the cut face)."""
    return np.minimum(outer_z(x, y) + S, 0.)


def edge_distance(x, y):
    i = np.clip(np.round((np.asarray(x) + A) / .001).astype(int), 0, len(GX) - 1)
    j = np.clip(np.round((np.asarray(y) + B) / .001).astype(int), 0, len(GY) - 1)
    return EDGE_DIST[j, i]


def rz(a): return parts._rot((0, 0, 1), a)
def ry(a): return parts._rot((0, 1, 0), a)
def rot(q): return parts._quat_to_matrix(q)
def xyz(p): return tuple(float(v) for v in p)
def to_local(st, pts): return (np.asarray(pts) - np.asarray(st.pos)) @ rot(st.quat_xyzw)


def distance_box(pts, c, r, size):
    return np.linalg.norm(np.maximum(np.abs((pts - c) @ r) - np.asarray(size) / 2, 0.), axis=1)


def cheek_distance_local(loc):
    """Distance from cheek-frame points to the cheek's solid colliders (inf for points far outside)."""
    loc = np.asarray(loc)
    out = np.full(len(loc), np.inf)
    near = np.all((loc > np.array([-A - .01, -B - .01, -H - .01])) & (loc < np.array([A + .01, B + .01, .01])), axis=1)
    if near.any():
        q = loc[near]
        d = np.linalg.norm(np.maximum(np.abs(q[:, None, :] - BOX_C[None]) - BOX_H[None], 0.), axis=2).min(1)
        out[near] = d
    return out


# -- recorder --------------------------------------------------------------------------------------------------------
_REC = {}
_SIM = {'broken': None}
_PATHS = {}


def fresh():
    _REC.clear()
    _REC.update(armed=False, invalid=False, setup=None, zero=None, t=None, prev=None, minz=np.full(CX.shape, np.inf),
                force_on=False, force=np.zeros(3), push_force=0., push_dir=np.array([0., 0., 1.]), peak_push_force=0.,
                press=0., from_below=False, touch=False, inverted=False, inv_cubes=0, push=0., clear=False, still=False,
                level=0., since=None, stamp=None, knife_arm=[], pinch_arm=[], min_clear=1., pieces=None, pieces_key=None,
                loaded_steps=0, peak_force=0.)


fresh()


def zero(why):
    if _REC['zero'] is None:
        _REC['zero'] = why


def stamp(objects):
    return tuple(float(v) for n in ('cheek', 'knife', 'ramekin') for v in np.round(np.r_[objects[n].pos, objects[n].quat_xyzw], 7))


def pieces(minz=None):
    """Cubes of the cut face: 4-connected regions of uncut flesh (cut cells and the rim band are borders)."""
    minz = _REC['minz'] if minz is None else minz
    wall = minz < np.minimum(0., inner_z(CX, CY) + DEPTH_GAP)
    key = wall.tobytes()
    if minz is _REC['minz'] and _REC['pieces_key'] == key:
        return _REC['pieces']
    free = ELIGIBLE & ~wall
    labels = np.zeros(free.shape, int); n = 0; cubes = 0; sizes = []
    for j0, i0 in zip(*np.nonzero(free)):
        if labels[j0, i0]:
            continue
        n += 1; stack = [(j0, i0)]; labels[j0, i0] = n; cells = []
        while stack:
            j, i = stack.pop(); cells.append((j, i))
            for jj, ii in ((j + 1, i), (j - 1, i), (j, i + 1), (j, i - 1)):
                if 0 <= jj < free.shape[0] and 0 <= ii < free.shape[1] and free[jj, ii] and not labels[jj, ii]:
                    labels[jj, ii] = n; stack.append((jj, ii))
        c = np.array(cells); sizes.append(len(c))
        if CUBE_CELLS[0] <= len(c) <= CUBE_CELLS[1]:
            p = np.stack((GX[c[:, 1]], GY[c[:, 0]]), 1)
            span = float(np.linalg.norm(p[:, None] - p[None], axis=2).max()) + .0014
            if span <= CUBE_SPAN + 1e-9:
                cubes += 1
    # Longest cut line: the largest 8-connected group of cut cells, by its diagonal.
    # Only cuts inside the rim band count, so a scratch along the thin rim earns nothing.
    longest = 0.; seen = np.zeros(wall.shape, bool); w = wall & ELIGIBLE
    for j0, i0 in zip(*np.nonzero(w)):
        if seen[j0, i0]:
            continue
        stack = [(j0, i0)]; seen[j0, i0] = True; js = []; is_ = []
        while stack:
            j, i = stack.pop(); js.append(j); is_.append(i)
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    jj, ii = j + dj, i + di
                    if 0 <= jj < w.shape[0] and 0 <= ii < w.shape[1] and w[jj, ii] and not seen[jj, ii]:
                        seen[jj, ii] = True; stack.append((jj, ii))
        longest = max(longest, math.hypot((max(is_) - min(is_)) * .001, (max(js) - min(js)) * .001))
    out = dict(cubes=cubes, pieces=n, longest=longest, cut_cells=int(w.sum()))
    if minz is _REC['minz']:
        _REC['pieces'], _REC['pieces_key'] = out, key
    return out


# -- hands -----------------------------------------------------------------------------------------------------------
class HandClouds(dict):
    def __init__(self):
        super().__init__(); self.fingers = {}; self.opening = {}


def native_handle(clouds, arm, knife):
    """Both handle sides held by opposing fingers in a closed grip; pushing the blade is not a grasp."""
    if not hasattr(clouds, 'fingers') or not .014 <= clouds.opening.get(arm, -1) <= .035:
        return False
    sides = []; r = rot(knife.quat_xyzw)
    for name in ('tip_left', 'tip_right'):
        pts = clouds.fingers.get(arm, {}).get(name)
        if pts is None or not len(pts):
            return False
        q = (pts - knife.pos) @ r
        d = np.linalg.norm(np.maximum(np.abs(q) - np.array([.050, .011, .012]), 0.), axis=1)
        near = q[d <= .003]
        if not len(near):
            return False
        sides.append(float(near[:, 1].mean()))
    return bool(sides[0] * sides[1] < 0 and min(abs(v) for v in sides) > .007)


def cheek_pinch(clouds, arm, cheek):
    """Two fingertips on opposite sides of the cheek, both touching it (within 3 mm)."""
    if not hasattr(clouds, 'fingers') or not .010 <= clouds.opening.get(arm, -1) <= .090:
        return False
    centres = []
    for name in ('tip_left', 'tip_right'):
        pts = clouds.fingers.get(arm, {}).get(name)
        if pts is None or not len(pts):
            return False
        loc = to_local(cheek, pts)
        near = loc[cheek_distance_local(loc) <= .003]
        if not len(near):
            return False
        centres.append(near.mean(0))
    return bool(np.linalg.norm((centres[0] - centres[1])[:2]) >= .015)


def blade_distance(cloud, knife):
    r = rot(knife.quat_xyzw)
    return min(float(distance_box(cloud, knife.pos + r @ c, r, size).min()) for c, size in BLADE)


def hand_distance(cloud, spec, st):
    lower = float(np.linalg.norm(np.maximum(np.maximum(st.low - cloud, cloud - st.high), 0.), axis=1).min())
    if lower > .010:
        return lower
    if spec is CHEEK:
        return float(cheek_distance_local(to_local(st, cloud)).min())
    r = rot(st.quat_xyzw)
    return min(float(distance_box(cloud, c, rr, size).min()) for _, c, rr, size in spec.world_boxes({'base': (np.asarray(st.pos), r)}))


# -- per-step update -------------------------------------------------------------------------------------------------
def setup_error(objects, lot=None):
    lot = lot or lots.current(); worst = [0., 0.]
    for spec in MOVE:
        st = objects[spec.name]
        worst[0] = max(worst[0], float(np.linalg.norm(np.asarray(st.pos) - np.asarray(lot.pos(spec.name)))))
        rr = rot(st.quat_xyzw).T @ rz(lot.yaw_of(spec.name))
        worst[1] = max(worst[1], math.degrees(math.acos(float(np.clip((np.trace(rr) - 1) / 2, -1, 1)))))
    return worst


def in_flesh(p):
    """Cheek-frame points inside the fruit (between the cut face and the outer skin)."""
    r2 = (p[:, 0] / A) ** 2 + (p[:, 1] / B) ** 2
    return (r2 < 1.) & (p[:, 2] < 0.) & (p[:, 2] > outer_z(p[:, 0], p[:, 1]))


def skin_cut(p):
    """Points more than 1 mm into the skin (or through it), more than 2 mm in from the cheek's edge."""
    r2 = (p[:, 0] / A) ** 2 + (p[:, 1] / B) ** 2
    zo = outer_z(p[:, 0], p[:, 1])
    hit = (r2 < 1.) & (p[:, 2] < zo + S - SKIN_CUT) & (p[:, 2] > zo - .003)
    if not hit.any():
        return False
    return bool(np.any(edge_distance(p[hit, 0], p[hit, 1]) > RIM_FREE))


def record_cut(p):
    """Lowest edge height reached in each 1 mm cell of the cut face (loaded strokes only)."""
    ok = in_flesh(p)
    if not ok.any():
        return
    q = p[ok]
    i = np.clip(np.round((q[:, 0] + A) / .001).astype(int), 0, len(GX) - 1)
    j = np.clip(np.round((q[:, 1] + B) / .001).astype(int), 0, len(GY) - 1)
    np.minimum.at(_REC['minz'], (j, i), q[:, 2])


def ramekin_push(cheek, ram, latched, touching):
    """(push, split depth, press, from_below, touch), m. The flat base is sampled at its centre and on rings of 8, 16
    and 24 mm radius (its edge), 45 points. touch: some base point is inside the cheek's outline and above its
    undisturbed outer skin this step, at any height and whatever the ramekin's attitude. The base counts as pressing
    (from_below) only if, on the first step it touches (it did not touch on the step before), the ramekin is base up
    relative to the cheek (axes within 30 degrees) with the base's centre inside the cheek's outline and below its
    cut face; it then stays pressing for as long as the base keeps touching with its centre inside the outline, so a
    press driven past the face still counts. A ramekin on the cut-face side of the cheek (above a cheek lying skin
    side down), one brought down onto it from that side (even through a lifted cheek), one slid in from the side, or
    a cheek turned face down over it reads nothing. push: at the base's centre, within 15 mm of the cheek's middle
    (past the cup's 25.5 mm depth the skin's middle is above its rim: turned out); split depth and press: the deepest
    base point (the press sets the skin-bending resistance)."""
    r = rot(ram.quat_xyzw)
    disc = [np.zeros(2)] + [rr * np.array([math.cos(a), math.sin(a)]) for rr, k in ((.008, 8), (.016, 12), (.024, 24))
                            for a in np.linspace(0, 2 * math.pi, k, endpoint=False)]
    pts = np.array([ram.pos + r @ np.array([d[0], d[1], RAM_TOP]) for d in disc])
    loc = to_local(cheek, pts)
    r2 = (loc[:, 0] / A) ** 2 + (loc[:, 1] / B) ** 2
    pen = np.where(r2 < 1., loc[:, 2] - outer_z(loc[:, 0], loc[:, 1]), -1.)
    worst = float(pen.max())
    touch = worst > 0.
    aligned = float(rot(cheek.quat_xyzw)[:, 2] @ r[:, 2]) > math.cos(math.radians(30))
    from_below = touch and aligned and r2[0] < 1. and (latched or (not touching and loc[0, 2] <= 0.))
    if not from_below:
        return -1., -1., 0., False, touch
    centre_ok = math.hypot(loc[0, 0], loc[0, 1]) <= PUSH_R and r[2, 2] > math.cos(math.radians(30))
    return (float(pen[0]) if centre_ok else -1.), worst, worst, True, touch


def update(objects, clouds, t):
    if not _REC['armed']:
        return
    if clouds is None or any(a not in clouds or not len(clouds[a]) or not np.isfinite(clouds[a]).all() for a in ARMS):
        _REC['invalid'] = True; return
    if _REC['setup'] is None:
        _REC['setup'] = setup_error(objects)
        if _REC['setup'][0] > SETUP_TOL[0] or _REC['setup'][1] > SETUP_TOL[1]:
            _REC['invalid'] = True
    if _REC['t'] is not None and not 0 < t - _REC['t'] <= .10001:
        _REC['invalid'] = True
    _REC['t'] = t
    cheek, knife, ram = objects['cheek'], objects['knife'], objects['ramekin']
    rk = rot(knife.quat_xyzw)
    now = to_local(cheek, knife.pos + BLADE_PTS @ rk.T)
    knife_arm = [a for a in ARMS if native_handle(clouds, a, knife)]
    pinch_arm = [a for a in ARMS if cheek_pinch(clouds, a, cheek)]
    for arm in ARMS:
        d = blade_distance(clouds[arm], knife)
        if d <= TOUCH:
            zero('blade touched a gripper')
        if arm not in knife_arm:
            _REC['min_clear'] = min(_REC['min_clear'], d)
            if d < BLADE_CLEAR:
                zero('blade within 10 mm of a gripper not holding the knife by its handle')
    prev = _REC['prev']
    loaded = False
    force = np.zeros(3)
    if prev is not None and not _REC['inverted']:
        old = prev['lines']
        step = now - old
        moved = float(np.linalg.norm(step, axis=1).max())
        fl = in_flesh(now[:N_EDGE]) | in_flesh(old[:N_EDGE])
        # Edge motion relative to the cheek, averaged over the last 0.25 s, so a stroke is judged by where the edge is
        # going rather than by step-to-step jitter.
        hist = _REC.setdefault('edge_hist', [])
        hist.append(now[:N_EDGE].mean(0)); del hist[:-SMOOTH - 1]
        d = (hist[-1] - hist[0]) / max(1, len(hist) - 1)
        speed = float(np.linalg.norm(d)) * 20.
        normal = rot(cheek.quat_xyzw).T @ rk[:, 1]
        along = d - (d @ normal) * normal
        # Edge first or drawn along the blade (within 45 degrees of its plane), with the blade within 45 degrees of
        # upright: a flat or sideways blade does not score the flesh.
        in_plane = (speed >= MIN_SPEED and abs(float(normal[2])) <= math.sin(math.radians(45)) + 1e-9
                    and math.atan2(abs(float(d @ normal)), float(np.linalg.norm(along))) <= math.radians(45) + 1e-9)
        holder = [a for a in pinch_arm if a not in knife_arm]
        loaded = bool(_REC['force_on'] and knife_arm and holder and in_plane and fl.any())
        n = max(1, int(math.ceil(moved / .0005)))
        for w in np.linspace(0, 1, n + 1)[1:]:
            p = old + step * w
            if skin_cut(p):
                zero('the knife cut through the skin')
            if loaded:
                record_cut(p[:N_EDGE])
        if loaded:
            _REC['loaded_steps'] += 1
        if fl.any():
            # Drag along the blade against the edge's motion along it (a blade in flesh slides along its own plane),
            # and feed resistance against the edge going down. Both rise with speed and reach their full value at
            # FULL_SPEED, like cutting resistance that grows with how fast the edge parts the flesh. The speed is the
            # edge's motion over the last 0.1 s: short enough that the load follows the knife without a lag (a load
            # taken from the 0.25 s average lagged the motion and set the knife bouncing 5 mm up and down mid-cut),
            # long enough that a one-step shake does not drive it.
            back = hist[-1 - LOAD_STEPS] if len(hist) > LOAD_STEPS else hist[0]
            v = rot(cheek.quat_xyzw) @ (hist[-1] - back) / max(1, min(LOAD_STEPS, len(hist) - 1)) * 20.
            u = np.array([rk[0, 0], rk[1, 0], 0.])
            if np.linalg.norm(u) > 1e-6:
                u /= np.linalg.norm(u)
                force -= DRAG * float(np.clip(v @ u / FULL_SPEED, -1., 1.)) * u
            force[2] += FEED * float(np.clip(-v[2] / FULL_SPEED, 0., 1.))
    elif prev is not None:
        for w in np.linspace(0, 1, max(1, int(math.ceil(float(np.linalg.norm(now - prev['lines'], axis=1).max()) / .0005))) + 1)[1:]:
            if skin_cut(prev['lines'] + (now - prev['lines']) * w):
                zero('the knife cut through the skin')
    _REC['force'] = force; _REC['force_on'] = bool(np.linalg.norm(force) > 0.)
    _REC['peak_force'] = max(_REC['peak_force'], float(np.linalg.norm(force)))
    # Declared hedgehog: pressing the held cheek's skin centre onto the ramekin's base, 28 mm or more past the skin.
    # The resistance acting over the last step (what the hand had to push against to get here).
    applied = _REC['push_force']
    push, worst, press, from_below, touch = ramekin_push(cheek, ram, _REC['from_below'], _REC['touch'])
    _REC['push'] = max(_REC['push'], push)
    # Declared skin-bending resistance: grows with how far the base presses into the skin, capped, rising by at most
    # PUSH_RATE per step, falling at once with the press, and gone as soon as the base stops pressing.
    want = min(PUSH_K * press, PUSH_CAP)
    f_push = min(want, _REC['push_force'] + PUSH_RATE)
    _REC.update(push_force=f_push, push_dir=rot(cheek.quat_xyzw)[:, 2].copy(), press=press, from_below=from_below, touch=touch,
                peak_push_force=max(_REC['peak_push_force'], f_push))
    # Split: the base pushed past a full inside-out turn (35 mm) while a hand holds the cheek.
    if worst > PUSH_MAX and pinch_arm:
        zero('the skin was pushed more than 35 mm in (split)')
    upright = rot(cheek.quat_xyzw)[2, 2] > math.cos(math.radians(30))
    # Full push: 28 mm or more (the skin turned past flat), against the full 5 N resistance a steady press meets there,
    # so a jab that outruns the resistance does not count.
    if not _REC['inverted'] and push >= PUSH_MIN and applied >= PUSH_NEED - 1e-6 and pinch_arm and upright:
        _REC['inverted'] = True
        _REC['inv_cubes'] = pieces()['cubes']
    clear = all(hand_distance(clouds[a], s, objects[s.name]) > HAND_CLEAR for a in ARMS for s in MOVE)
    still = pose_hold(objects, clear, t)
    level = end_level(objects) if clear and _REC['zero'] is None and not _REC['invalid'] else 0.
    if level <= 0 or level != _REC['level']:
        _REC['since'] = t if level > 0 else None
    elif _REC['since'] is None:
        _REC['since'] = t
    _REC.update(level=level, clear=clear, still=still, stamp=stamp(objects), prev={'lines': now},
                knife_arm=knife_arm, pinch_arm=pinch_arm, loaded=loaded)


def pose_hold(objects, clear, t):
    """Every loose object within 1 mm and 2 degrees over a rolling 1 s window with both hands clear."""
    if not clear:
        _REC['pose_history'] = []; return False
    now = {n: (np.asarray(objects[n].pos).copy(), np.asarray(objects[n].quat_xyzw).copy()) for n in ('cheek', 'knife', 'ramekin')}
    history = _REC.setdefault('pose_history', []); history.append((float(t), now))
    while len(history) > 1 and history[1][0] <= float(t) - 1. + 1e-9:
        history.pop(0)
    if float(t) - history[0][0] < 1. - 1e-9:
        return False
    anchor = history[0][1]
    for _, poses in history:
        for n, (p, q) in poses.items():
            p0, q0 = anchor[n]
            angle = 2 * math.acos(float(np.clip(abs(q @ q0) / (np.linalg.norm(q) * np.linalg.norm(q0)), 0, 1)))
            if np.linalg.norm(p - p0) > .001 + 1e-9 or angle > math.radians(2) + 1e-9:
                return False
    return True


def on_board(st, spec):
    lo, hi = st.low, st.high
    return bool(lo[0] >= BOARD_C[0] - BOARD_SIZE[0] / 2 - .001 and hi[0] <= BOARD_C[0] + BOARD_SIZE[0] / 2 + .001
                and lo[1] >= BOARD_C[1] - BOARD_SIZE[1] / 2 - .001 and hi[1] <= BOARD_C[1] + BOARD_SIZE[1] / 2 + .001
                and abs(float(lo[2]) - SURFACE) <= .003)


def end_level(objects):
    cheek, knife, ram = objects['cheek'], objects['knife'], objects['ramekin']
    if not (on_board(cheek, CHEEK) and on_board(knife, KNIFE) and on_board(ram, RAMEKIN)):
        return 0.
    if rot(cheek.quat_xyzw)[2, 2] < math.cos(math.radians(30)):
        return 0.
    # The knife must be out of the fruit: no blade point inside the cheek or within 2 mm of it.
    blade = to_local(cheek, knife.pos + BLADE_PTS @ rot(knife.quat_xyzw).T)
    if in_flesh(blade).any() or (cheek_distance_local(blade) <= .002).any():
        return 0.
    p = pieces()
    if _REC['inverted'] and _REC['inv_cubes'] >= FULL_CUBES:
        return 1.
    if p['cubes'] >= FULL_CUBES:
        return .75
    if p['cubes'] >= HALF_CUBES:
        return .5
    if p['longest'] >= LINE_SPAN:
        return .25
    return 0.


def score(objects):
    try:
        if (not _REC['armed'] or _REC['invalid'] or _REC['zero'] is not None or _REC['stamp'] != stamp(objects)
                or not _REC['clear'] or not _REC['still'] or _REC['since'] is None or _REC['t'] - _REC['since'] < 1. - 1e-9):
            return 0.
        return min(_REC['level'], end_level(objects))
    except (KeyError, TypeError, ValueError, AttributeError):
        return 0.


def success(objects): return score(objects) == 1.


# -- simulator hooks -------------------------------------------------------------------------------------------------
_MESH = {}
_original_author = parts._author


def _author(part, root_path, translation, orientation):
    root = _original_author(part, root_path, translation, orientation)
    _PATHS[part.name] = root_path
    if part.name == 'cheek':
        try:
            from pxr import UsdGeom
            from isaaclab.sim.utils import get_current_stage
            stage = get_current_stage()
            for i in range(len(TILE_XY)):
                UsdGeom.Imageable(stage.GetPrimAtPath(f'{root_path}/s{TILE_START + i}')).MakeInvisible()
        except Exception as e:  # drawing only; scoring never depends on it
            _SIM['tiles'] = repr(e)
    if part.name in ('cheek', 'ramekin') and all(n in _PATHS for n in ('cheek', 'ramekin')):
        # The ramekin passes into the rigid cheek instead of bending its skin: the declared push rule reads the overlap.
        from pxr import Sdf, UsdPhysics
        from isaaclab.sim.utils import get_current_stage
        stage = get_current_stage()
        pairs = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(_PATHS['ramekin']))
        pairs.CreateFilteredPairsRel().SetTargets([Sdf.Path(_PATHS['cheek'])])
    return root


parts._author = _author


def hand_mesh():
    if _MESH:
        return _MESH
    from yam_arena import analysis
    for name in ('gripper', 'tip_left', 'tip_right'):
        pts = []
        for h in analysis.link_hulls().get(name, []):
            pts.extend(h.vertices)
            for pl in np.unique(h.planes.round(7), axis=0):
                face = h.vertices[np.abs(h.vertices @ pl[:3] - pl[3]) < 2e-6]
                if len(face) < 3:
                    raise ValueError('unknown hand hull')
                c = face.mean(0); u = face[0] - c; u /= np.linalg.norm(u); v = np.cross(pl[:3], u)
                face = face[np.argsort(np.arctan2((face - c) @ v, (face - c) @ u))]
                for p, q in zip(face, np.roll(face, -1, axis=0)):
                    steps = max(1, math.ceil(max(np.linalg.norm(p - c), np.linalg.norm(q - c), np.linalg.norm(p - q)) / .003))
                    for j in range(steps + 1):
                        for k in range(steps - j + 1):
                            pts.append(c + (p - c) * (j / steps) + (q - c) * (k / steps))
        arr = np.asarray(pts); _, keep = np.unique(np.floor(arr / .002).astype(int), axis=0, return_index=True)
        _MESH[name] = arr[keep]
    return _MESH


def clouds(sim):
    base = sim.env.unwrapped; origin = parts._np(base.scene.env_origins[0]); out = HandClouds()
    for arm in ARMS:
        a = base.scene[f'{arm}_robot']; poses = parts._np(a.data.body_link_pose_w)[0]; chunks = []; fingers = {}
        for n, p in hand_mesh().items():
            pose = poses[a.body_names.index(n)]
            world = p @ rot(pose[3:]).T + pose[:3] - origin
            chunks.append(world)
            if n in ('tip_left', 'tip_right'):
                fingers[n] = world
        out[arm] = np.vstack(chunks); out.fingers[arm] = fingers
        q = parts._np(a.data.joint_pos)[0]
        out.opening[arm] = -sum(float(q[a.joint_names.index(n)]) for n in ('joint7', 'joint8'))
        if not math.isfinite(out.opening[arm]):
            raise ValueError('unknown finger opening')
    return out


def apply_force(sim, force, push=0., push_dir=(0., 0., 1.)):
    """Cutting load (knife +, cheek -) and skin-bending resistance (cheek pushed along its own up axis, ramekin back)."""
    import torch
    base = sim.env.unwrapped
    cut = np.asarray(force, float); up = np.asarray(push_dir, float) * float(push)
    for name, vec in (('knife', cut), ('cheek', up - cut), ('ramekin', -up)):
        obj = base.scene[name]; dev = obj.device
        f = torch.zeros((1, 1, 3), device=dev); m = torch.zeros_like(f)
        f[0, 0, :] = torch.tensor([float(v) for v in vec], device=dev)
        # Declared resultants: applied at each body's centre of mass; their moments are not simulated.
        obj.set_external_force_and_torque(f, m, env_ids=torch.tensor([0], device=dev), is_global=True)


def show_cuts(reset=False):
    """Drawing only: reveal the cut-mark tile over every 4 mm square that holds a completed cut (hide all at reset)."""
    if _SIM.get('tiles') or _SIM.get('cheek_path') is None:
        return
    try:
        from pxr import UsdGeom
        from isaaclab.sim.utils import get_current_stage
        stage = get_current_stage()
        if reset:
            for i in np.nonzero(_SIM.get('shown', np.ones(len(TILE_XY), bool)))[0]:
                UsdGeom.Imageable(stage.GetPrimAtPath(f"{_SIM['cheek_path']}/s{TILE_START + i}")).MakeInvisible()
            _SIM['shown'] = np.zeros(len(TILE_XY), bool)
            return
        wall = (_REC['minz'] < np.minimum(0., inner_z(CX, CY) + DEPTH_GAP)).ravel()
        new = ~_SIM['shown'] & wall[TILE_CELLS].any(1)
        for i in np.nonzero(new)[0]:
            UsdGeom.Imageable(stage.GetPrimAtPath(f"{_SIM['cheek_path']}/s{TILE_START + i}")).MakeVisible()
        _SIM['shown'] |= new
    except Exception as e:
        _SIM['tiles'] = repr(e)


@parts.on_reset
def reset(sim):
    fresh(); _SIM['broken'] = None
    try:
        from pxr import Usd as _Usd, UsdPhysics as _UsdPhysics
        from isaaclab.sim.utils import get_current_stage as _stage
        base = sim.env.unwrapped
        if _SIM.get('cheek_path') is None:
            for prim in _Usd.PrimRange(_stage().GetPrimAtPath(base.scene.env_prim_paths[0])):
                if prim.GetName() == 'cheek' and prim.HasAPI(_UsdPhysics.RigidBodyAPI):
                    _SIM['cheek_path'] = str(prim.GetPath()); break
        show_cuts(reset=True)
    except Exception as e:
        _SIM['tiles'] = repr(e)
    try:
        base = sim.env.unwrapped
        apply_force(sim, np.zeros(3))
        for arm in ARMS:
            robot = base.scene[f'{arm}_robot']
            ids = robot.find_joints(['joint7', 'joint8'], preserve_order=True)[0]
            robot.write_joint_effort_limit_to_sim_index(limits=25., joint_ids=ids)
        from pxr import Usd, UsdPhysics, PhysxSchema
        from isaaclab.sim.utils import get_current_stage
        touched = 0
        for prim in Usd.PrimRange(get_current_stage().GetPrimAtPath(base.scene.env_prim_paths[0])):
            path = str(prim.GetPath()).lower()
            if prim.HasAPI(UsdPhysics.CollisionAPI) and any(k in path for k in ('/knife/', 'tip_left', 'tip_right')):
                cfg = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                cfg.CreateTorsionalPatchRadiusAttr().Set(.010)
                cfg.CreateMinTorsionalPatchRadiusAttr().Set(.008)
                touched += 1
        if touched < 5:
            raise ValueError(f'expected knife and fingertip colliders, found {touched}')
    except Exception as e:
        _SIM['broken'] = repr(e); _REC['invalid'] = True
        print('[mango] reset failed', repr(e), flush=True)
    _REC['armed'] = True


@parts.on_step
def step(sim):
    try:
        st = sim.states(); hands = clouds(sim); update(st, hands, float(sim.t))
        apply_force(sim, _REC['force'], _REC['push_force'], _REC['push_dir'])
        if int(round(sim.t * 20)) % 4 == 0:
            show_cuts()
        if os.environ.get('TRACE_OUT'):
            from yam_arena.constants import TCP_OFFSET
            base = sim.env.unwrapped; origin = parts._np(base.scene.env_origins[0])
            row = {k: _REC.get(k) for k in ('t', 'zero', 'invalid', 'setup', 'clear', 'still', 'level', 'since', 'inverted',
                                             'inv_cubes', 'push', 'knife_arm', 'pinch_arm', 'loaded', 'min_clear', 'loaded_steps', 'peak_force',
                                             'push_force', 'press', 'peak_push_force', 'touch', 'from_below')}
            row['force'] = np.round(_REC['force'], 3).tolist()
            row['hands'] = {}
            for arm in ARMS:
                robot = base.scene[f'{arm}_robot']; pose = parts._np(robot.data.body_link_pose_w)[0, robot.body_names.index('gripper')]
                row['hands'][arm] = {'tcp': np.round(pose[:3] - origin + rot(pose[3:]) @ np.array(TCP_OFFSET), 5).tolist(), 'quat': np.round(pose[3:], 5).tolist()}
            row['opening'] = {a: round(v, 4) for a, v in hands.opening.items()}
            row['poses'] = {n: np.round(np.r_[st[n].pos, st[n].quat_xyzw], 5).tolist() for n in ('cheek', 'knife', 'ramekin')}
            tip = to_local(st['cheek'], (st['knife'].pos + rot(st['knife'].quat_xyzw) @ EDGE[-1])[None])[0]
            row['tip'] = np.round(tip, 5).tolist()
            if int(round(sim.t * 20)) % 20 == 0:
                row['pieces'] = pieces()
            row['broken'] = _SIM['broken']; row['score'] = score(st); row['lot'] = lots.current().name
            row['tiles'] = [_SIM.get('tiles'), _SIM.get('cheek_path'), int(np.sum(_SIM.get('shown', 0)))]
            with pathlib.Path(os.environ['TRACE_OUT']).with_name('mango.jsonl').open('a') as f:
                f.write(json.dumps(row) + '\n')
    except Exception as e:
        if _SIM['broken'] is None:
            print('[mango] step failed', repr(e), flush=True)
        _SIM['broken'] = repr(e); _REC['invalid'] = True


# -- open-loop demo: reads only the published layout (lots.current()) and fixed calibration constants ---------------
TIP = EDGE[-1]
GRASP = np.array([-0.015, 0., 0.004])   # TCP in the knife frame: on the handle, 15 mm behind its centre
PITCH = math.radians(45)                # blade tipped 45 degrees point-down while scoring
CROSS_X = (0.038, 0.024, 0.008, -0.006)  # cuts across the cheek, far end first; the steadying grip is at x = -50 mm
LONG_Y = (-0.016, 0.0, 0.016)           # cuts along the cheek, from the far edge to x = LONG_END
LONG_END = -0.011
HOLD_X = -0.050                         # steadying pinch across the cheek's near end
HOLD_TILT = 45.                         # ...leaning its fingertips toward the far end, keeping its wrist back
INSET = 0.0035                          # every cut starts and ends 3.5 mm inside the cheek's edge...
CROSS_END = 0.0015                      # ...except that cuts across end 5 mm inside the edge where they finish
AIM = 0.003                             # point aimed 3 mm above the skin (see aim_at for the ends of each cut)
LIFT = 0.06
# Fixed per-layout corrections (m, cheek frame) for the knife arm's tracking error, from traced runs; never read live.
# (dx, dy, lead, dz at the point's entry, dz while scoring, end lead, dz at the end), cheek frame, m: the knife arm
# sits a little off, lags behind along the cut and rides up under the stated cutting load, so the scoring waypoints
# lead along the cut and aim lower. The end lead is no longer used (the point lifts straight out instead of resting);
# dz at the end raises the last waypoint of a cut (the cuts across end close to the rim, and in traced runs of both
# layouts a stop sprang the point 2-3 mm on into the rising skin there). Measured from traced runs of each layout and
# frozen; never read live.
CAL = {'A': {'cross': (-.0012, .0017, .0056, .0005, -.0011, .0005, .0035),
             'long': (-.0008, .0006, .0047, -.0002, -.0007, .0003, 0.)},
       'B': {'cross': (-.0017, .0035, .0057, .0008, -.0004, .0027, .0025),
             'long': (.0009, .0020, .0053, .0007, -.0006, .0019, 0.)}}
SCORE_SPEED = 0.014                     # m/s along a cut while scoring
PRESS = 0.0425                          # m: how far below the ramekin's base the demo sends the cheek's lowest skin point
D0 = tilt.rotation(0.)
GRIP_TURN = math.pi


def cheek_world(lot):
    return np.array(lot.pos('cheek')), rz(lot.yaw_of('cheek'))


def half_width(x, zlo, zhi):
    """Widest solid half-width of the cheek across y at length x, between heights zlo and zhi (cheek frame)."""
    w = 0.
    for c, h in SOLID_BOXES:
        if abs(x - c[0]) <= h[0] and c[2] + h[2] >= zlo and c[2] - h[2] <= zhi:
            w = max(w, abs(c[1]) + h[1])
    return w


def knife_wp(dur, tip, phi, label):
    """Waypoint putting the blade point at `tip` (world), blade heading `phi`, tipped PITCH point-down. The knife is
    gripped with the hand turned half a turn from the blade (GRIP_TURN), which keeps the wrist inside its range."""
    rk = rz(phi) @ ry(PITCH)
    yaw = math.atan2(math.sin(phi + GRIP_TURN), math.cos(phi + GRIP_TURN))
    w = tilt.Waypoint(dur, xyz(np.asarray(tip) - rk @ (TIP - GRASP)), -.004, label, yaw=yaw)
    w.yaw = tilt._TiltedYaw(yaw, rk @ rz(GRIP_TURN) @ D0)
    return w


def aim_at(family, s, total):
    """Commanded height of the point above the skin (m) at distance s along a cut of length total: 1 mm higher over
    the first 8 mm; over the last 12 mm, 1 mm lower on a cut across and 0.5 mm higher on a cut along (where traced
    points ran 2-3 mm below their waypoints) - before the per-layout corrections in CAL."""
    return AIM + (.001 if s < .008 else (-.001 if family == 'cross' else .0005) if total - s < .012 else 0.)


SAFE_BELOW = 0.0005                     # m: deepest the commanded point may sit below the skin's inner surface
SLOW_SPEED = 0.007                      # m/s over 8-25 mm of a cut along, where the point rode high in traced runs
END_SPEED = 0.004                       # m/s over the last 12 mm of a cut across, where it rose with the rising skin


def segment_speeds(points, family):
    """Speed (m/s) of the point over each segment of a cut, ending at points[1:]: slow over the last 12 mm of a cut
    across (the point rises up the rim there), and over 8-25 mm (steepest descent) and the last 10 mm of a cut along
    (so it reaches the end before lifting out), full elsewhere."""
    total = sum(np.linalg.norm(np.subtract(b, a)) for a, b in zip(points, points[1:]))
    out, s = [], 0.
    for a, b in zip(points, points[1:]):
        ln = float(np.linalg.norm(np.subtract(b, a))); mid = s + ln / 2; s += ln
        slow = total - mid < .012 if family == 'cross' else (.008 <= mid < .025 or total - mid < .010)
        out.append((END_SPEED if family == 'cross' else SLOW_SPEED) if slow else SCORE_SPEED)
    return out


def stroke(points, family, lot, cal=True):
    """Blade-point path along one cut: cheek-frame (x, y) points, each 3 mm (4 mm near the ends) above the skin, plus
    the frozen tracking correction for this layout and cut family."""
    c, r = cheek_world(lot)
    dx, dy, lead, dz0, dz1, lead_end, dz_end = CAL[lot.name][family] if cal else (0.,) * 7
    along = np.subtract(points[-1], points[0]); along = along / np.linalg.norm(along)
    speeds = [SCORE_SPEED] + segment_speeds(points, family)
    pts = []
    total = sum(np.linalg.norm(np.subtract(b, a)) for a, b in zip(points, points[1:]))
    s = 0.
    for k, (x, y) in enumerate(points):
        if k:
            s += float(np.linalg.norm(np.subtract(points[k], points[k - 1])))
        aim = aim_at(family, s, total)
        z = float(inner_z(x, y)) + aim
        # The lead (the arm's lag) scales with the point's speed, and tapers to nothing over the last 12 mm, so the
        # point is never sent past the end of the cut.
        ld = lead * min(1., (total - s) / .012) * speeds[k] / SCORE_SPEED if k else 0.
        # The point is raised by dz_end at the last waypoint (rising from the one before): when the stroke stops, the
        # arm's stored lag springs the point a couple of millimetres on toward the rim, where the skin rises steeply.
        shift = np.array([dx + ld * along[0], dy + ld * along[1], (dz1 + (dz_end if k == len(points) - 1 else 0.)) if k else dz0])
        p = np.array([x, y, z]) + shift
        # Never send the point more than 0.5 mm into the skin at the commanded spot itself, so an arm that followed
        # its waypoints exactly (no lag, no offset) would still not cut the skin.
        p[2] = max(p[2], float(inner_z(p[0], p[1])) - SAFE_BELOW)
        pts.append(c + r @ p)
    return pts


def line_points(family, value, forward):
    if family == 'cross':
        yb = B * math.sqrt(1 - (value / A) ** 2) - INSET
        ye = yb - CROSS_END   # cuts across stop further in from the edge they finish at (the blade's upper front reaches ahead)
        ys = np.linspace(yb, -ye, max(3, int(round((yb + ye) / .008)) + 1))
        pts = [(value, y) for y in ys]
    else:
        xa = A * math.sqrt(1 - (value / B) ** 2) - INSET
        xs = np.linspace(xa, LONG_END, max(3, int(round((xa - LONG_END) / .008)) + 1))
        pts = [(x, value) for x in xs]
    return pts if forward else pts[::-1]


def knife_plan(lot):
    c, r = cheek_world(lot); cy = lot.yaw_of('cheek')
    kp = np.array(lot.pos('knife')); ky = lot.yaw_of('knife') + GRIP_TURN
    kg = kp + rz(lot.yaw_of('knife')) @ GRASP
    plan = [tilt.Waypoint(2.5, xyz(kg + [0, 0, LIFT]), -.025, 'above knife handle', yaw=ky),
            tilt.Waypoint(1.5, xyz(kg), -.025, 'around handle', yaw=ky),
            tilt.Waypoint(0.8, xyz(kg), -.004, 'grip handle', yaw=ky),
            tilt.Waypoint(0.4, xyz(kg), -.004, 'settle grip', yaw=ky),
            tilt.Waypoint(1.2, xyz(kg + [0, 0, .07]), -.004, 'lift knife', yaw=ky)]
    up = r @ np.array([0, 0, 1.])

    turn = np.array([.21, -.19, SURFACE + .12])   # where the hand turns the level knife between the two cut families
    for f, (family, values, phi) in enumerate((('cross', CROSS_X, cy - math.pi / 2), ('long', LONG_Y, cy + math.pi))):
        yaw = math.atan2(math.sin(phi + GRIP_TURN), math.cos(phi + GRIP_TURN))
        if f:
            plan.append(tilt.Waypoint(1.5, xyz(turn), -.004, f'{family}: turn the level knife', yaw=yaw))
        for k, v in enumerate(values):
            # Point first every time: the rising blade behind the point stays over deeper flesh, clear of the skin.
            path = stroke(line_points(family, v, True), family, lot)
            start, end = path[0], path[-1]
            above = start + up * (.006 - (start - c) @ up)
            if k == 0:
                plan.append(knife_wp(2.5, above + up * .02, phi, f'{family} cut 1: point down over its start'))
                plan.append(knife_wp(0.8, above, phi, f'{family} cut 1: point over the face'))
            else:
                plan.append(knife_wp(1.2, above, phi, f'{family} cut {k + 1}: over its start'))
            plan.append(knife_wp(0.9, start, phi, f'{family} cut {k + 1}: point into the flesh'))
            for a, b, spd in zip(path, path[1:], segment_speeds(line_points(family, v, True), family)):
                plan.append(knife_wp(max(.3, float(np.linalg.norm(b - a)) / spd), b, phi, f'{family} cut {k + 1}: score'))
            # Lift straight out at the end of the cut (no rest at depth).
            out = end + up * (.006 - (end - c) @ up)
            plan.append(knife_wp(0.8, out, phi, f'{family} cut {k + 1}: lift out'))
        plan.append(knife_wp(0.8, out + up * .02, phi, f'{family}: clear of the fruit'))
        if family == 'long':
            # The last cut ends near the steadying hand: back the point away along the cheek before levelling it.
            away = out + up * .05 + r @ np.array([.05, 0., 0.])
            plan.append(knife_wp(1.2, away, phi, f'{family}: point away from the other hand'))
            over_turn = turn + np.array([0., 0., .06]) + rz(phi) @ ry(PITCH) @ (TIP - GRASP)
            plan.append(knife_wp(1.5, over_turn, phi, f'{family}: carry the knife, still tipped, over the board'))
        plan.append(tilt.Waypoint(2.0, xyz(turn), -.004, f'{family}: knife level, clear of the fruit', yaw=yaw))
    plan += [tilt.Waypoint(2.0, xyz(kg + [0, 0, .07]), -.004, 'carry knife back', yaw=ky),
             tilt.Waypoint(1.5, xyz(kg + [0, 0, .002]), -.004, 'stand knife on its handle', yaw=ky),
             tilt.Waypoint(0.8, xyz(kg + [0, 0, .002]), -.025, 'release knife', yaw=ky),
             tilt.Waypoint(1.5, xyz(kg + [0, 0, LIFT]), -.025, 'lift clear', yaw=ky)]
    return plan


def holder_plan(lot, wait):
    """Left hand: steady the cheek by its near end while the knife scores, then let go and move away."""
    c, r = cheek_world(lot); cy = lot.yaw_of('cheek')
    end = c + r @ np.array([HOLD_X, 0, .006])   # fingertips on the top 4 mm of the sides, where they are upright
    w_end = max(half_width(x, -.004, 0.) for x in np.arange(HOLD_X - .010, HOLD_X + .0161, .001))
    lean = dict(yaw=cy, tilt=HOLD_TILT, heading=cy)
    plan = [tilt.Waypoint(3.0, xyz(end + [0, 0, LIFT]), -.040, 'above the near end', **lean),
            tilt.Waypoint(2.0, xyz(end), -.040, 'fingers either side of the near end', **lean),
            tilt.Waypoint(1.0, xyz(end), -(w_end - .004), 'steady the cheek', **lean)]
    hold = wait - sum(w.duration for w in plan)
    plan += [tilt.Waypoint(hold, xyz(end), -(w_end - .004), 'hold while the knife scores', **lean),
             tilt.Waypoint(0.8, xyz(end), -.040, 'let go of the end', **lean),
             tilt.Waypoint(1.0, xyz(end + [0, 0, .05]), -.040, 'lift fingers', **lean),
             tilt.Waypoint(1.5, xyz(end + [-.06, .10, .10]), -.040, 'move away', yaw=cy)]
    return plan


def turnout_plan(lot):
    """Right hand, after the knife is back: hold both sides of the rim, press the skin's middle onto the ramekin's
    base, then set the cheek back skin side down and let go."""
    c, r = cheek_world(lot); cy = lot.yaw_of('cheek')
    rp = np.array(lot.pos('ramekin'))
    mid = c + r @ np.array([0, 0, -.002])
    w_mid = half_width(0., -.015, 0.)
    lift = np.array([0, 0, .045])
    over = np.array([rp[0], rp[1], mid[2] + .045])
    # Press: the cheek's lowest skin point is sent PRESS below the ramekin's base; the arm gives under the declared
    # skin-bending resistance, so the skin's middle ends less far up (frozen from traced runs).
    press = np.array([rp[0], rp[1], rp[2] + RAM_TOP - PRESS + H - .002])
    back = mid + np.array([0, 0, .002])
    g = -(w_mid - .004)
    return [tilt.Waypoint(1.6, xyz(mid + [0, 0, .04]), -.045, 'over the middle of the cheek', yaw=cy),
            tilt.Waypoint(1.3, xyz(mid), -.045, 'fingers either side of the middle', yaw=cy),
            tilt.Waypoint(0.8, xyz(mid), g, 'hold both sides of the rim', yaw=cy),
            tilt.Waypoint(1.2, xyz(mid + lift), g, 'lift the cheek', yaw=cy),
            tilt.Waypoint(1.7, xyz(over), g, 'over the ramekin', yaw=cy),
            tilt.Waypoint(1.7, xyz(press), g, 'press the skin centre onto the ramekin', yaw=cy),
            tilt.Waypoint(1.2, xyz(press), g, 'hold the skin pushed up', yaw=cy),
            tilt.Waypoint(1.1, xyz(over), g, 'lift off the ramekin', yaw=cy),
            tilt.Waypoint(1.7, xyz(back + lift), g, 'carry back', yaw=cy),
            tilt.Waypoint(1.3, xyz(back), g, 'set it down skin side down', yaw=cy),
            tilt.Waypoint(0.8, xyz(back), -.045, 'release', yaw=cy),
            tilt.Waypoint(1.2, xyz(back + [0, 0, LIFT]), -.045, 'lift clear', yaw=cy)]


def demo():
    lot = lots.current()
    knife = knife_plan(lot)
    # The steadying hand holds until the knife is back on its handle and the knife hand has lifted clear.
    wait = sum(w.duration for w in knife)
    holder = holder_plan(lot, wait)
    knife.append(tilt.Waypoint(3.3, knife[-1].pos, knife[-1].finger, 'wait for the other hand to move away', yaw=knife[-1].yaw))
    knife += turnout_plan(lot)
    total = max(sum(w.duration for w in knife), sum(w.duration for w in holder)) + 2.
    for plan in (knife, holder):
        rest = total - sum(w.duration for w in plan)
        plan.append(tilt.Waypoint(rest, plan[-1].pos, plan[-1].finger, 'still', yaw=plan[-1].yaw))
    return {'left': holder, 'right': knife}
