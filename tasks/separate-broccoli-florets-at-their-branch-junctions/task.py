"""A trimmed green broccoli head lies on its side on a white cutting board, its thick main stem toward one arm and six
rounded floret crowns fanned toward the other arm on short pale-green stalks. A black-handled 76 mm paring knife
stands on the flat underside of its handle near the right-hand edge of the board, its edge resting on the board.
A steel ruler lies beside the board. The second layout mirrors the scene and exchanges the two arms' roles.
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
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP
import tilt

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SCORE_OVER = 'end'
SUCCESS_HOLD_S = 0.1
DEMO_TOUCHES = ('board',)
parts.CONTACT_OFFSET = 0.001
BOARD_H = 0.016
SURFACE = TOP + BOARD_H
GREEN = (0.16, 0.38, 0.12)
STALK = (0.43, 0.66, 0.25)
BUD = (0.08, 0.28, 0.08)
STEM_R = 0.017
Z_STEM = SURFACE + STEM_R + 0.0005          # stem axis = stalk axis height when the head lies on the board
Z_KNIFE = SURFACE + 0.012 + 0.0005          # handle origin when the knife stands on its handle underside
ANCHOR = np.array([0., -0.065, 0.])
ANGLES = np.radians([-22, -49, -76, -104, -131, -158])
RADII = np.array([0.067, 0.072, 0.075, 0.075, 0.072, 0.067])
NAMES = tuple(f'floret_{i+1}' for i in range(6))
WIDTHS = (0.038, 0.040, 0.042, 0.042, 0.040, 0.038)
# Cutting resistance on the blade while its edge is inside a stalk, per metre of edge inside the 16 mm section:
# feed resistance opposing the edge's advance, and drag opposing the blade's motion relative to the stalk.
# Estimates: blade tests on parsley stems give 2.25 mJ/mm^2 specific cutting energy (Chem. Proc. 2022, 10, 16), about
# 2.25 N per mm of edge; a tender
# broccoli stalk under a sharp, thin paring blade is taken as about a fifth of that (0.4 N/mm), plus 0.15 N/mm of
# face friction. Across the full 16 mm section that is 6.4 N feed + 2.4 N drag.
FEED = 400.
DRAG = 150.
SLOW = 0.005          # m/s: drag acts above this relative speed; feed stops when the edge is drawn back faster than this
SMOOTH = 3            # control steps over which the edge's motion relative to the stalk is measured (0.15 s)
STALK_R = 0.008
ARMS = ('left', 'right')
FORK = 0.026          # severing reference plane to the first bud-bearing side branch
ALONG = (-0.030, 0.024)   # section rays from 56 mm to 2 mm below the first side branch: any full cut there separates
SETUP_TOL = (0.004, 3.0)


def rz(a): return parts._rot((0, 0, 1), a)
def rot(q): return parts._quat_to_matrix(q)
def xyz(p): return tuple(float(v) for v in p)

BOARD = parts.Part('board', pos=(0.29, 0., TOP), friction=(0.7, 0.6), shapes=[
    parts.box((0.3683, 0.53594, BOARD_H), (0, 0, BOARD_H/2), (0.91, 0.91, 0.87)),
    *[parts.box((0.012, 0.02, 0.004), (x, y, 0.002), (0.16, 0.17, 0.18)) for x in (-0.165, 0.165) for y in (-0.24, 0.24)]])
RULER = parts.Part('ruler', pos=(0.49, 0., TOP+0.001), shapes=[
    parts.box((0.020, 0.30, 0.002), color=(0.62, 0.65, 0.66)),
    *[parts.box((0.012 if i % 5 == 0 else 0.006, 0.001, 0.0004), (0, i*.01-.15, .0012), (0.08, .08, .08), solid=False) for i in range(31)]])
# The main stem body includes the six 16 mm shoots from the stem end up to 1 mm short of each severing reference
# plane. Each distal floret is its own body, held to the stem by a tissue joint until it is cut.
base_shapes = [parts.cylinder(STEM_R, .100, (0, -.015, 0), STALK, axis='y')]
CUTS = []
for angle, radius in zip(ANGLES, RADII):
    u = np.array([math.cos(angle), math.sin(angle), 0.])
    CUTS.append(ANCHOR+radius*u)
    base_shapes.append(parts.cylinder(.008, radius-.001, xyz(ANCHOR+(radius-.001)/2*u), STALK, axis='x', yaw=float(angle)))

# Layout A: stem along +y toward the left arm, florets toward the right arm. B mirrors A across y = 0.
HEAD_A = np.array([0.270, -0.030, Z_STEM])
HEAD_YAW_A = math.radians(40)
KNIFE_A = (np.array([0.200, -0.225, Z_KNIFE]), 0.0)
STEM = parts.Part('stem', pos=xyz(HEAD_A), yaw=HEAD_YAW_A, mass=.10, friction=(.6, .5), shapes=base_shapes)
FLORETS = []
for i, (c, a, w) in enumerate(zip(CUTS, ANGLES, WIDTHS)):
    shapes = [parts.cylinder(.008, .035, (.0175, 0, 0), STALK, axis='x'),
              parts.cylinder(.003, .020, (FORK, 0, 0), STALK, axis='y')]
    shapes += [parts.cylinder(w*.27, .025, (.042, 0, .004), GREEN)]
    for phi in np.linspace(0, 2*math.pi, 7)[:-1]:
        shapes.append(parts.cylinder(w*.16, .017, (.042+w*.29*math.cos(phi), w*.29*math.sin(phi), .009), BUD))
    FLORETS.append(parts.Part(NAMES[i], pos=xyz(HEAD_A+rz(HEAD_YAW_A)@c), yaw=float(HEAD_YAW_A+a), mass=.025, friction=(.6, .5), shapes=shapes))
# Handle origin is the native grip centre; handle, finger guard and edge share one flat underside (z = -12 mm).
KNIFE = parts.Part('knife', pos=xyz(KNIFE_A[0]), yaw=KNIFE_A[1], mass=.05, friction=(.7, .6), shapes=[
    parts.box((.100, .022, .024), color=(.07, .07, .07)),
    parts.box((.010, .030, .026), (.050, 0, .001), (.09, .09, .09)),
    parts.box((.076, .002, .018), (.088, 0, -.003), (.68, .70, .71), solid=False),
    parts.box((.072, .0025, .0015), (.088, 0, -.0112), (.93, .94, .94), solid=False)])
OBJECTS = [BOARD, RULER, STEM, KNIFE, *FLORETS]
KNIFE_SOLID = type('KnifeSolid', (), {'shapes': KNIFE.shapes})()   # whole knife, blade included, for the on-board check
MOVE = [STEM, KNIFE, *FLORETS]


PIVOT = np.array([0., -0.065, -STEM_R])   # bottom of the stem's head end: the head tips forward about it
def rx(a): return parts._rot((1, 0, 0), a)

def rest_pitch():
    """Forward tilt (rad) at which the lowest crown touches the board while the stem's head end stays down."""
    lo, hi = 0., math.radians(15)
    for _ in range(60):
        b = (lo+hi)/2; r = rx(b); low = 0.
        for i, spec in enumerate(FLORETS):
            for sh in spec.shapes:
                if sh.kind != 'cylinder': continue
                for ph in np.linspace(0, 2*math.pi, 72, endpoint=False):
                    for zz in (-sh.size[1]/2, sh.size[1]/2):
                        a = np.asarray(sh.at)+(np.array([0, 0, zz]) if sh.axis == 'z' else np.zeros(3))
                        d = np.array([math.cos(ph), math.sin(ph), 0.])*sh.size[0] if sh.axis == 'z' else np.zeros(3)
                        pt = CUTS[i]+rz(ANGLES[i])@(a+d)
                        low = min(low, float((r@(pt-PIVOT))[2]))
        if low < 0: hi = b
        else: lo = b
    return lo
PITCH = rest_pitch()

def head_layout(origin, yaw):
    """Exact resting poses of stem and florets: the head lies on its stem and tips forward onto its crowns."""
    turn = rz(yaw); r = turn@rx(PITCH)
    base = np.asarray(origin)+turn@PIVOT-r@PIVOT
    place = {'stem': xyz(base)}; quat = {'stem': parts._matrix_to_quat(r)}; yaws = {'stem': float(yaw)}
    for i, n in enumerate(NAMES):
        place[n] = xyz(base+r@CUTS[i])
        quat[n] = parts._matrix_to_quat(r@rz(ANGLES[i])); yaws[n] = float(yaw+ANGLES[i])
    return place, yaws, quat

PA, YA, QA = head_layout(HEAD_A, HEAD_YAW_A)
PA['knife'] = xyz(KNIFE_A[0]); YA['knife'] = KNIFE_A[1]
# The fan is mirror-symmetric about the stem axis, so the mirror image of A is the same head turned.
PB, YB, QB = head_layout(HEAD_A*np.array([1, -1, 1]), math.pi-HEAD_YAW_A)
PB['knife'] = xyz(KNIFE_A[0]*np.array([1, -1, 1])); YB['knife'] = -KNIFE_A[1]
lots.ABSOLUTE = True
LOTS = lots.declare(OBJECTS, lots.Lot('A', place=PA, yaw=YA, quat=QA), lots.Lot('B', place=PB, yaw=YB, quat=QB))

def lot_rotation(lot, name):
    q = lot.roles.get('quat', {}).get(name)
    return rot(q) if q is not None else rz(lot.yaw_of(name))

# Disk of stalk cross-section, in a branch frame with x along the stalk, y across, z up.
GRID = np.array([[y, z] for y in np.linspace(-.008, .008, 7) for z in np.linspace(-.008, .008, 7) if y*y+z*z <= .008**2+1e-10])
EDGE_X = np.linspace(.054, .124, 36)
_REC = {}
_JOINS = {}
_MESH = {}
_SIM = {'generation': 0, 'broken': None}

def fresh():
    _REC.clear()
    _REC.update(armed=False, invalid=False, setup=None, zero=None, t=None, prev=None, cuts=set(), damaged=set(),
                coverage={i: np.zeros(len(GRID), bool) for i in range(6)}, cut_axial={i: np.full(len(GRID), np.nan) for i in range(6)},
                cut_stalks={}, engaged=set(), loaded_time={i: 0. for i in range(6)}, clear=False, still=False, since=None,
                level=0., stamp=None, applied=set(), next_forces={}, min_clearance=1., forces=0., chords={}, track={}, pending={})
fresh()

def stamp(objects):
    return tuple(float(v) for n in ('stem', 'knife', *NAMES) for v in np.round(np.r_[objects[n].pos, objects[n].quat_xyzw, objects[n].lin_vel, objects[n].ang_vel], 7))

def edge(st):
    r = rot(st.quat_xyzw)
    return np.asarray(st.pos)+np.column_stack((EDGE_X, np.zeros(len(EDGE_X)), np.full(len(EDGE_X), -.012)))@r.T

def chord(knife, food, shift=(0., 0., 0.)):
    """Length (m) and midpoint (world) of the knife edge, moved by shift, inside the stalk cylinder (scored window)."""
    r = rot(food.quat_xyzw); a, b = (edge(knife)[[0, -1]]+np.asarray(shift)-np.asarray(food.pos))@r; d = b-a
    qa = d[1]**2+d[2]**2; qb = 2*(a[1]*d[1]+a[2]*d[2]); qc = a[1]**2+a[2]**2-STALK_R**2
    if qa < 1e-14:
        if qc >= 0: return 0., None
        s0, s1 = 0., 1.
    else:
        disc = qb*qb-4*qa*qc
        if disc <= 0: return 0., None
        root = math.sqrt(disc); s0 = max(0., (-qb-root)/(2*qa)); s1 = min(1., (-qb+root)/(2*qa))
    if abs(d[0]) > 1e-12:
        sa = (ALONG[0]-a[0])/d[0]; sb = (.035-a[0])/d[0]
        s0 = max(s0, min(sa, sb)); s1 = min(s1, max(sa, sb))
    elif not ALONG[0] <= a[0] <= .035: return 0., None
    if s1 <= s0: return 0., None
    return float((s1-s0)*np.linalg.norm(d)), np.asarray(food.pos)+r@(a+d*(s0+s1)/2)-np.asarray(shift)


def edge_mid(knife, food):
    """Midpoint of the knife edge in the stalk's own frame."""
    return (np.asarray(knife.pos)+rot(knife.quat_xyzw)@np.array([.089, 0., -.012])-np.asarray(food.pos))@rot(food.quat_xyzw)


def cutting_force(oldknife, knife, oldfood, food, dt, v=None):
    """Resisting force on the knife (world, N), its point and the edge length in the stalk, for the coming control step.

    The force depends on where the edge is now: it acts only while the edge is inside the stalk section. Feed resistance
    (FEED per metre of edge inside) pushes back against the edge-first direction unless the edge is being drawn back out
    faster than SLOW; drag (DRAG per metre) opposes the edge's motion relative to the stalk when that is faster than
    SLOW. v is that relative velocity (world, m/s), measured over the last SMOOTH steps by update(); without it, the
    motion from oldknife/oldfood over dt is used."""
    length, point = chord(knife, food)
    if length <= 0: return np.zeros(3), None, 0.
    if v is None:
        v = rot(food.quat_xyzw)@((edge_mid(knife, food)-edge_mid(oldknife, oldfood))/dt) if dt > 0 else np.zeros(3)
    facing = rot(knife.quat_xyzw)@np.array([0., 0., -1.])
    force = np.zeros(3)
    if v@facing > -SLOW: force -= FEED*length*facing
    speed = float(np.linalg.norm(v))
    if speed > SLOW: force -= DRAG*length*v/speed
    return force, point, length


def edge_sides(line, facing):
    """For each section point: signed distance ahead of the edge line (along its facing), the stalk-axis position of
    the edge at that point, and whether the point lies within the edge's length. Everything in the stalk frame."""
    a, b = line; d = b-a; d2 = d[1:]; length2 = float(d2@d2)
    n2 = np.array([-d2[1], d2[0]])
    if length2 < 1e-12 or abs(float(n2@facing[1:])) < 1e-9*math.sqrt(length2): return None
    n2 = n2/math.sqrt(length2)
    if n2@facing[1:] < 0: n2 = -n2
    rel = GRID-a[1:]; s = (rel@d2)/length2
    return rel@n2, a[0]+s*d[0], (s >= 0.) & (s <= 1.)


def edge_hits(oldknife, knife, oldfood, food):
    """Section points the edge passes, edge-first, between two control steps, judged RELATIVE to the stalk.

    Both edge positions are expressed in the stalk's own frame at their own time, so knife and food moving together
    pass nothing. A point is passed when it goes from ahead of the edge line (along the edge's facing, in the section
    plane) to on or behind it, with the point within the edge's length at both steps. The crossing's position along
    the stalk is interpolated between the two steps.
    """
    f0 = rot(oldfood.quat_xyzw); f1 = rot(food.quat_xyzw)
    old = (edge(oldknife)[[0, -1]]-np.asarray(oldfood.pos))@f0
    new = (edge(knife)[[0, -1]]-np.asarray(food.pos))@f1
    facing = f1.T@(rot(knife.quat_xyzw)@np.array([0., 0., -1.]))
    hits = np.zeros(len(GRID), bool); axial = np.full(len(GRID), np.nan)
    before = edge_sides(old, facing); after = edge_sides(new, facing)
    if before is None or after is None: return hits, axial
    (d0, x0, in0), (d1, x1, in1) = before, after
    crossed = (d0 > 0) & (d1 <= 0) & in0 & in1
    frac = np.where(crossed, d0/np.maximum(d0-d1, 1e-15), 0.)
    x = x0+(x1-x0)*frac
    crossed &= (x >= ALONG[0]-1e-9) & (x <= ALONG[1]+1e-9)
    hits[crossed] = True; axial[crossed] = x[crossed]
    return hits, axial


def distance_box(pts, c, r, size):
    return np.linalg.norm(np.maximum(np.abs((pts-c)@r)-np.asarray(size)/2, 0.), axis=1)

def shapes(spec, st):
    return [(c, r, s) for _, c, r, s in spec.world_boxes({'base': (np.asarray(st.pos), rot(st.quat_xyzw))})]

class HandClouds(dict):
    def __init__(self):
        super().__init__(); self.fingers = {}; self.opening = {}


def native_grasp(clouds, arm, stem):
    """Two opposed fingertips contacting the native stem cylinder, with a closed physical opening."""
    if not hasattr(clouds, 'fingers') or arm not in clouds.fingers: return False
    if not .020 <= clouds.opening.get(arm, -1) <= .055: return False
    directions = []
    r = rot(stem.quat_xyzw)
    for name in ('tip_left', 'tip_right'):
        pts = clouds.fingers[arm].get(name)
        if pts is None or not len(pts): return False
        q = (pts-stem.pos)@r
        radial = np.linalg.norm(q[:, [0, 2]], axis=1)-STEM_R
        axial = np.abs(q[:, 1]+.015)-.050
        d = np.linalg.norm(np.column_stack((np.maximum(radial, 0.), np.maximum(axial, 0.))), axis=1)
        near = q[d <= .003]
        if not len(near): return False
        v = near[:, [0, 2]].mean(axis=0)
        if np.linalg.norm(v) < .008: return False
        directions.append(v/np.linalg.norm(v))
    return bool(directions[0]@directions[1] < -.5)


def native_handle(clouds, arm, knife):
    """Both native handle sides contacted by opposing fingers; a blade push is not a grasp."""
    if not hasattr(clouds, 'fingers') or not .014 <= clouds.opening.get(arm, -1) <= .035: return False
    sides = []; r = rot(knife.quat_xyzw)
    for name in ('tip_left', 'tip_right'):
        pts = clouds.fingers.get(arm, {}).get(name)
        if pts is None or not len(pts): return False
        q = (pts-knife.pos)@r
        d = np.linalg.norm(np.maximum(np.abs(q)-np.array([.050, .011, .012]), 0.), axis=1)
        near = q[d <= .003]
        if not len(near): return False
        sides.append(float(near[:, 1].mean()))
    return bool(sides[0]*sides[1] < 0 and min(abs(v) for v in sides) > .007)


def blade_clearance(clouds, previous, arm, objects, oldobjects):
    """Full blade box versus swept hand faces, at <=3 mm interpolation intervals."""
    knife = objects['knife']; oldknife = knife if oldobjects is None else oldobjects['knife']
    current = clouds[arm]; old = current if previous is None else previous[arm]
    q0 = np.asarray(oldknife.quat_xyzw); q1 = np.asarray(knife.quat_xyzw)
    if q0@q1 < 0: q1 = -q1
    motion = max(float(np.linalg.norm(current-old, axis=1).max()), float(np.linalg.norm(knife.pos-oldknife.pos))+float(np.linalg.norm(q1-q0))*.14)
    count = max(1, int(math.ceil(motion/.003)))
    closest = float('inf'); contact = False
    for alpha in np.linspace(0, 1, count+1):
        q = (1-alpha)*q0+alpha*q1; q /= np.linalg.norm(q)
        r = rot(q); p = (1-alpha)*oldknife.pos+alpha*knife.pos
        hand = (1-alpha)*old+alpha*current
        centre = p+r@np.array([.088, 0, -.003])
        distance = float(distance_box(hand, centre, r, (.076, .002, .018)).min())
        closest = min(closest, distance)
        if distance <= .0015: contact = True
        # Contact with the native handle or guard also invalidates a holding hand.
        for size, at in (((.100, .022, .024), (0, 0, 0)), ((.010, .030, .026), (.05, 0, .001))):
            if float(distance_box(hand, p+r@np.asarray(at), r, size).min()) <= .0015: contact = True
    return closest, contact


def bud_local(local, spec, depth):
    buds = spec.shapes[2:]
    bounds = [(np.asarray(sh.at)-np.array([sh.size[0], sh.size[0], sh.size[1]/2]), np.asarray(sh.at)+np.array([sh.size[0], sh.size[0], sh.size[1]/2])) for sh in buds]
    lo = np.min([b[0] for b in bounds], axis=0)+depth
    hi = np.max([b[1] for b in bounds], axis=0)-depth
    local = local[np.all((local >= lo) & (local <= hi), axis=1)]
    if not len(local): return False
    for shape in buds:
        q = local-np.asarray(shape.at)
        radial = shape.size[0]-np.linalg.norm(q[:, :2], axis=1)
        axial = shape.size[1]/2-np.abs(q[:, 2])
        if np.any(np.minimum(radial, axial) > depth+1e-9): return True
    return False


def bud_interior(points, st, spec, depth):
    """Points deeper than depth inside actual bud cylinders."""
    return bud_local((np.asarray(points)-st.pos)@rot(st.quat_xyzw), spec, depth)


def bud_sweep(oldpoints, points, oldfood, food, spec, depth):
    """Relative hand/edge and food sweep, including food crossing a stationary edge."""
    before = np.asarray(oldpoints)-oldfood.pos; after = np.asarray(points)-food.pos
    radius = max(float(np.linalg.norm(sh.at))+math.hypot(sh.size[0], sh.size[1]/2) for sh in spec.shapes[2:])
    delta = after-before; dd = np.sum(delta*delta, axis=1)
    fraction = np.clip(-np.sum(before*delta, axis=1)/np.maximum(dd, 1e-20), 0, 1)
    keep = np.linalg.norm(before+fraction[:, None]*delta, axis=1) <= radius
    if not keep.any(): return False
    before = before[keep]; after = after[keep]
    q0 = np.asarray(oldfood.quat_xyzw); q1 = np.asarray(food.quat_xyzw)
    if q0@q1 < 0: q1 = -q1
    angle = 2*math.acos(float(np.clip(q0@q1/(np.linalg.norm(q0)*np.linalg.norm(q1)), 0, 1)))
    travel = float(np.linalg.norm(after-before, axis=1).max())+radius*angle
    steps = max(1, int(math.ceil(travel/.002)))
    for alpha in np.linspace(0, 1, steps+1):
        q = (1-alpha)*q0+alpha*q1; q /= np.linalg.norm(q)
        local = ((1-alpha)*before+alpha*after)@rot(q)
        if bud_local(local, spec, depth): return True
    return False

BLADE_CORNERS = np.array([[x, y, z] for x in (.050, .126) for y in (-.001, .001) for z in (-.012, .006)])
BOX_EDGES = [(i, i ^ bit) for i in range(8) for bit in (1, 2, 4) if i < (i ^ bit)]


def polygon_radius(points):
    """Exact distance from the origin to the convex hull of 2D points."""
    pts = sorted(set(tuple(float(x) for x in p) for p in points))
    if len(pts) == 1: return float(np.linalg.norm(pts[0]))
    def cross(a, b, c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower = []; upper = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0: lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0: upper.pop()
        upper.append(p)
    hull = np.asarray(lower[:-1]+upper[:-1]); n = len(hull)
    if n >= 3 and all(cross(hull[i], hull[(i+1) % n], (0., 0.)) >= -1e-14 for i in range(n)): return 0.
    a = hull; b = np.roll(hull, -1, axis=0); d = b-a
    f = np.clip(-np.sum(a*d, axis=1)/np.maximum(np.sum(d*d, axis=1), 1e-30), 0, 1)
    return float(np.linalg.norm(a+f[:, None]*d, axis=1).min())


def blade_in_buds(vertices, spec, depth):
    """Exact OBB/inset-cylinder intersection, via clipping to each bud's axial slab."""
    for sh in spec.shapes[2:]:
        v = vertices-np.asarray(sh.at); radius = sh.size[0]-depth-1e-9; half = sh.size[1]/2-depth-1e-9
        if radius <= 0 or half <= 0: continue
        if np.any(v.min(0) > [radius, radius, half]) or np.any(v.max(0) < [-radius, -radius, -half]): continue
        points = list(v[(v[:, 2] >= -half) & (v[:, 2] <= half)])
        for i, j in BOX_EDGES:
            d = v[j]-v[i]
            if abs(d[2]) < 1e-15: continue
            for z in (-half, half):
                f = (z-v[i, 2])/d[2]
                if 0 <= f <= 1: points.append(v[i]+f*d)
        if points and polygon_radius(np.asarray(points)[:, :2]) < radius: return True
    return False


def blade_damage(oldknife, knife, oldfood, food, spec, depth=.003):
    def pair(a, b):
        a = np.asarray(a); b = np.asarray(b)
        if a@b < 0: b = -b
        angle = 2*math.acos(float(np.clip(a@b/(np.linalg.norm(a)*np.linalg.norm(b)), 0, 1)))
        return a, b, angle
    k0, k1, ka = pair(oldknife.quat_xyzw, knife.quat_xyzw); f0, f1, fa = pair(oldfood.quat_xyzw, food.quat_xyzw)
    travel = float(np.linalg.norm(knife.pos-oldknife.pos)+np.linalg.norm(food.pos-oldfood.pos))+.14*ka+.10*fa
    steps = max(1, int(math.ceil(travel/.001)))
    for alpha in np.linspace(0, 1, steps+1):
        k = (1-alpha)*k0+alpha*k1; k /= np.linalg.norm(k)
        f = (1-alpha)*f0+alpha*f1; f /= np.linalg.norm(f)
        p = (1-alpha)*(oldknife.pos-oldfood.pos)+alpha*(knife.pos-food.pos)
        vertices = (BLADE_CORNERS@rot(k).T+p)@rot(f)
        if blade_in_buds(vertices, spec, depth): return True
    return False


def hand_distance(cloud, spec, st):
    # AABB broad phase: above 10 mm this lower bound gives the same answers at the 3/5 mm gates.
    lower = float(np.linalg.norm(np.maximum(np.maximum(st.low-cloud, cloud-st.high), 0.), axis=1).min())
    if lower > .010: return lower
    return min(float(distance_box(cloud, c, r, size).min()) for c, r, size in shapes(spec, st))

def zero(why):
    if _REC['zero'] is None: _REC['zero'] = why

def true_bounds(spec, st):
    """Exact world bounds for the primitive cylinders, rather than rotated bounding-box corners."""
    r = rot(st.quat_xyzw); lows = []; highs = []
    for sh in spec.shapes:
        c = np.asarray(st.pos)+r@np.asarray(sh.at)
        if sh.kind == 'cylinder':
            axis = (r@rz(getattr(sh, 'yaw', 0.)))[:, {'x': 0, 'y': 1, 'z': 2}[sh.axis]]
            extent = sh.size[1]/2*np.abs(axis)+sh.size[0]*np.sqrt(np.maximum(0., 1-axis*axis))
        else: extent = np.abs(r)@(np.asarray(sh.size)/2)
        lows.append(c-extent); highs.append(c+extent)
    return np.min(lows, axis=0), np.max(highs, axis=0)


def pose_hold(objects, clear, t):
    """A rolling one-second window, rather than drift since first release."""
    if not clear:
        _REC['pose_history'] = []; return False
    now = {n: (np.asarray(objects[n].pos).copy(), np.asarray(objects[n].quat_xyzw).copy()) for n in ('stem', 'knife', *NAMES)}
    history = _REC.setdefault('pose_history', []); history.append((float(t), now))
    while len(history) > 1 and history[1][0] <= float(t)-1.+1e-9: history.pop(0)
    if float(t)-history[0][0] < 1.-1e-9: return False
    anchor = history[0][1]
    for _, poses in history:
        for n, (p, q) in poses.items():
            p0, q0 = anchor[n]
            angle = 2*math.acos(float(np.clip(abs(q@q0)/(np.linalg.norm(q)*np.linalg.norm(q0)), 0, 1)))
            if np.linalg.norm(p-p0) > .002+1e-9 or angle > math.radians(2)+1e-9: return False
    return True

def end_level(objects):
    n = 0
    for i in _REC['cuts']:
        f = objects[NAMES[i]]
        lo, hi = true_bounds(FLORETS[i], f)
        # Exact cylinder support and footprint; pose hold is enforced for the whole scene.
        if (i not in _REC['damaged'] and i in _REC['cut_stalks']
            and all(.01-1e-9 <= v <= .03+1e-9 for v in _REC['cut_stalks'][i]) and .03 <= .9*WIDTHS[i] <= .05
            and lo[0] >= .29-.3683/2-.001 and hi[0] <= .29+.3683/2+.001
            and lo[1] >= -.53594/2-.001 and hi[1] <= .53594/2+.001
            and abs(float(lo[2])-SURFACE) <= .003):
            n += 1
    level = 1. if n >= 6 else .67 if n >= 3 else .33 if n else 0.
    # The knife must be left resting on the board, wholly inside its outline.
    lo, hi = true_bounds(KNIFE_SOLID, objects['knife'])
    if not (lo[0] >= .29-.3683/2-.001 and hi[0] <= .29+.3683/2+.001 and lo[1] >= -.53594/2-.001 and hi[1] <= .53594/2+.001
            and abs(float(lo[2])-SURFACE) <= .003): level = 0.
    d = len(_REC['damaged'])
    return min(level, .33 if d >= 3 else .67 if d else 1.)


def setup_error(objects, lot=None):
    """Largest start-pose error (m, deg) of the loose bodies against the lot's declared pose; None if unreadable."""
    lot = lot or lots.current()
    worst = [0., 0.]
    for spec in MOVE:
        st = objects[spec.name]
        worst[0] = max(worst[0], float(np.linalg.norm(np.asarray(st.pos)-np.asarray(lot.pos(spec.name)))))
        r = rot(st.quat_xyzw).T@lot_rotation(lot, spec.name)
        worst[1] = max(worst[1], math.degrees(math.acos(float(np.clip((np.trace(r)-1)/2, -1, 1)))))
    return worst

def update(objects, clouds, t):
    newly = []
    if not _REC['armed']: return newly
    if clouds is None or any(a not in clouds or not len(clouds[a]) or not np.isfinite(clouds[a]).all() for a in ARMS):
        _REC['invalid'] = True; return newly
    if _REC['setup'] is None:
        # Fail closed if the reset did not reproduce the declared layout (reported as an invalid trial).
        _REC['setup'] = setup_error(objects)
        if _REC['setup'][0] > SETUP_TOL[0] or _REC['setup'][1] > SETUP_TOL[1]: _REC['invalid'] = True
    dt = 0 if _REC['t'] is None else t-_REC['t']
    if _REC['t'] is not None and not 0 < dt <= .10001: _REC['invalid'] = True
    _REC['t'] = t
    knife, stem = objects['knife'], objects['stem']
    oldobjects = None if _REC['prev'] is None else _REC['prev'][1]
    e = edge(knife)
    hold = []
    for arm in ARMS:
        if native_grasp(clouds, arm, stem): hold.append(arm)
        # Crushing a crown is prohibited for the whole trial.
        for i, spec in enumerate(FLORETS):
            st = objects[NAMES[i]]
            oldcloud = _REC.get('clouds', {}).get(arm, clouds[arm])
            oldst = st if oldobjects is None else oldobjects[NAMES[i]]
            if bud_sweep(oldcloud, clouds[arm], oldst, st, spec, .002): zero('gripper entered a bud crown')
    for arm in set(hold) | set(_REC.get('holding', [])):
        d, contact = blade_clearance(clouds, _REC.get('clouds'), arm, objects, oldobjects)
        _REC['min_clearance'] = min(_REC['min_clearance'], d)
        if d < .030: zero('full blade came within 30 mm of the holding gripper')
        if contact: zero('knife contacted the holding gripper')
    tool_holding = [a for a in ARMS if native_handle(clouds, a, knife)]
    two_grips = bool(hold and any(a not in hold for a in tool_holding))
    _REC['tool_holding'] = tool_holding
    if not two_grips:
        for i in range(6): _REC['loaded_time'][i] = 0.
    prev = _REC['prev']; in_stalk = False
    for i in range(6):
        st = objects[NAMES[i]]; r = rot(st.quat_xyzw)
        local = (e-st.pos)@r
        touching = bool(np.any((local[:, 0] >= ALONG[0]-.002) & (local[:, 0] <= .035) & (np.linalg.norm(local[:, 1:], axis=1) < .014)))
        if touching and i not in _REC['cuts']:
            in_stalk = True
            _REC['engaged'].add(i)
        # The resistance stays armed through the complete cut, including the last rim points;
        # it releases after a 30 mm withdrawal or separation.
        if i in _REC['engaged']:
            if i in _REC['cuts'] or float(np.linalg.norm(local, axis=1).min()) > .030:
                _REC['engaged'].discard(i)
                if i not in _REC['cuts']:
                    _REC['coverage'][i][:] = False
                    _REC['cut_axial'][i][:] = np.nan
                    _REC['loaded_time'][i] = 0.
            else: in_stalk = True
        if i in _REC['applied'] and dt > 0 and two_grips: _REC['loaded_time'][i] += dt
        # Bud damage remains attached to original crown identity even if later discarded.
        oldst = st if oldobjects is None else oldobjects[NAMES[i]]
        oldknife = knife if oldobjects is None else oldobjects['knife']
        if blade_damage(oldknife, knife, oldst, st, FLORETS[i], .003): _REC['damaged'].add(i)
        if prev is None or i in _REC['cuts']: continue
        _, oldstates = prev
        crossing, axial = edge_hits(oldstates['knife'], knife, oldstates[NAMES[i]], st)
        pending = _REC['pending'].pop(i, None)
        if two_grips and i in _REC['engaged'] and i in _REC['applied']:
            # Only the path the edge followed while the cutting resistance acted on it is credited. The simulator
            # applies each force for the next 50 ms control step, so points crossed in the step in which the edge
            # entered the stalk count once the resistance acts in the following step.
            for mask, where in ([pending] if pending is not None else [])+[(crossing, axial)]:
                first = mask & ~_REC['coverage'][i]
                _REC['cut_axial'][i][first] = where[first]
                _REC['coverage'][i] |= mask
            if _REC['coverage'][i].all() and _REC['loaded_time'][i] >= .1-1e-9:
                swept_axial = _REC['cut_axial'][i]
                _REC['cut_stalks'][i] = (float(FORK-swept_axial.max()), float(FORK-swept_axial.min()))
                _REC['cuts'].add(i); newly.append(i)
        elif two_grips and i in _REC['engaged'] and crossing.any():
            _REC['pending'][i] = (crossing, axial)
    # Intact bodies drifting apart without the declared cut are not accepted as tearing.
    sr = rot(stem.quat_xyzw)
    for i in range(6):
        if i not in _REC['cuts']:
            want = stem.pos+sr@CUTS[i]
            if np.linalg.norm(objects[NAMES[i]].pos-want) > .018: zero('uncut branch tore away')
    clear = all(hand_distance(clouds[a], s, objects[s.name]) > .005 for a in ARMS for s in (STEM, KNIFE, *FLORETS))
    still = pose_hold(objects, clear, t)
    level = end_level(objects) if clear and _REC['zero'] is None else 0.
    if level <= 0 or level != _REC['level']: _REC['since'] = t if level > 0 else None
    elif _REC['since'] is None: _REC['since'] = t
    # The resistance acts whenever the edge is in a stalk, held or not; only the credit above needs both grips.
    forces = {}
    for i in range(6):
        if i not in _REC['engaged'] or i in _REC['cuts']: _REC['track'].pop(i, None); continue
        track = _REC['track'].setdefault(i, [])
        track.append((t, edge_mid(knife, objects[NAMES[i]]))); del track[:-(SMOOTH+1)]
        span = track[-1][0]-track[0][0]
        v = rot(objects[NAMES[i]].quat_xyzw)@((track[-1][1]-track[0][1])/span) if span > 0 else np.zeros(3)
        f, point, length = cutting_force(knife, knife, objects[NAMES[i]], objects[NAMES[i]], 0., v)
        if point is not None and np.linalg.norm(f) > 1e-9: forces[i] = (f, point, length)
    _REC.update(level=level, clear=clear, still=still, stamp=stamp(objects), prev=(e, dict(objects)), next_forces=forces,
                clouds={a: clouds[a] for a in ARMS}, holding=list(hold))
    return newly

def score(objects):
    try:
        if (not _REC['armed'] or _REC['invalid'] or _REC['zero'] is not None or _REC['stamp'] != stamp(objects)
            or not _REC['clear'] or not _REC['still'] or _REC['since'] is None or _REC['t']-_REC['since'] < 1.-1e-9): return 0.
        return min(_REC['level'], end_level(objects))
    except (KeyError, TypeError, ValueError, AttributeError): return 0.
def success(objects): return score(objects) == 1.

# Simulator material joints, external wrench and face-sampled hands.
def stage():
    from isaaclab.sim.utils import get_current_stage
    return get_current_stage()

def body_path(base, name):
    from pxr import Usd, UsdPhysics
    env = base.scene.env_prim_paths[0]
    for p in Usd.PrimRange(stage().GetPrimAtPath(env)):
        if p.GetName() == name and p.HasAPI(UsdPhysics.RigidBodyAPI): return str(p.GetPath())
    raise ValueError('missing rigid body '+name)

def make_joints(base):
    """Author the six stem-floret tissue joints once, disabled."""
    from pxr import Gf, Sdf, UsdPhysics
    for i, n in enumerate(NAMES):
        if i in _JOINS: continue
        _SIM['generation'] += 1
        path = f'{base.scene.env_prim_paths[0]}/broccoli_tissue_{_SIM["generation"]}'
        j = UsdPhysics.FixedJoint.Define(stage(), path)
        j.CreateJointEnabledAttr().Set(False)
        j.CreateBody0Rel().SetTargets([Sdf.Path(body_path(base, 'stem'))])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body_path(base, n))])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*xyz(CUTS[i])))
        j.CreateLocalRot0Attr().Set(parts._usd_quat(rz(float(ANGLES[i]))))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1.))
        j.CreateExcludeFromArticulationAttr().Set(True)
        j.CreateCollisionEnabledAttr().Set(False)
        _JOINS[i] = path

def set_joints(enabled):
    from pxr import UsdPhysics
    for i, path in _JOINS.items():
        UsdPhysics.FixedJoint.Get(stage(), path).GetJointEnabledAttr().Set(bool(i in enabled))

def mark_applied(on):
    """Record which stalks the coming step's resistance acts on (the simulator calls this as it writes the forces)."""
    _REC['applied'] = set(_REC['next_forces']) if on else set()
    total = sum((f for f, _, _ in _REC['next_forces'].values()), np.zeros(3)) if on else np.zeros(3)
    _REC['forces'] = float(np.linalg.norm(total))
    _REC['chords'] = {i: round(l*1000, 1) for i, (_, _, l) in _REC['next_forces'].items()} if on else {}

def floret_com(spec):
    """Centre of mass of a floret's colliders (uniform density, as the simulator computes it), body frame."""
    vol = []; at = []
    for sh in spec.shapes:
        if sh.solid: vol.append(math.pi*sh.size[0]**2*sh.size[1]); at.append(np.asarray(sh.at))
    vol = np.asarray(vol); return (np.asarray(at)*vol[:, None]).sum(0)/vol.sum()
FLORET_COM = [floret_com(f) for f in FLORETS]

def knife_com():
    """Centre of mass of the knife's colliders (handle and finger guard; the blade has none), body frame."""
    vol = np.array([float(np.prod(sh.size)) for sh in KNIFE.shapes if sh.solid])
    at = np.array([sh.at for sh in KNIFE.shapes if sh.solid], float)
    return (at*vol[:, None]).sum(0)/vol.sum()
KNIFE_COM = knife_com()

def knife_wrench(knife, forces):
    """Total resistance on the knife and the moment it would have about the knife's centre of mass (world frame),
    taking each stalk's force where the edge is in that stalk. Only the force is applied (see apply_load); the moment
    is logged in traces."""
    com = np.asarray(knife.pos)+rot(knife.quat_xyzw)@KNIFE_COM
    total = np.zeros(3); moment = np.zeros(3)
    for force, point, _ in forces.values():
        total += force; moment += np.cross(np.asarray(point)-com, force)
    return total, moment

def apply_load(sim, on):
    """Cutting resistance: on the knife at its centre of mass (where the demo grips the handle), without the moment
    about it; the equal and opposite reaction on the stalk where the edge is. With the moment applied (trace of
    21 Nov), the first stalk contact turned the knife about 33 degrees in the fingertip pinch within one control step."""
    import torch
    base = sim.env.unwrapped
    mark_applied(on)
    origin = parts._np(base.scene.env_origins[0])
    states = sim.states() if on and _REC['next_forces'] else {}
    total = np.zeros(3); moment = np.zeros(3)
    if states: total, moment = knife_wrench(states['knife'], _REC['next_forces'])
    for i, n in enumerate(NAMES):
        obj = base.scene[n]; dev = obj.device
        f = torch.zeros((1, 1, 3), device=dev); m = torch.zeros_like(f)
        if on and i in _REC['next_forces']:
            force, point, _ = _REC['next_forces'][i]
            st = states[n]; com = np.asarray(st.pos)+rot(st.quat_xyzw)@FLORET_COM[i]
            f[0, 0] = torch.tensor(-force, device=dev, dtype=f.dtype)
            m[0, 0] = torch.tensor(np.cross(np.asarray(point)-com, -force), device=dev, dtype=f.dtype)
        obj.set_external_force_and_torque(f, m, env_ids=torch.tensor([0], device=dev), is_global=True)
    obj = base.scene['knife']; dev = obj.device
    f = torch.tensor(total, device=dev, dtype=torch.float32).reshape(1, 1, 3)
    m = torch.zeros_like(f)
    _REC['grip_moment'] = float(np.linalg.norm(moment))
    obj.set_external_force_and_torque(f, m, env_ids=torch.tensor([0], device=dev), is_global=True)


def hand_mesh():
    if _MESH: return _MESH
    from yam_arena import analysis
    for name in ('gripper', 'tip_left', 'tip_right'):
        pts = []
        for h in analysis.link_hulls().get(name, []):
            pts.extend(h.vertices)
            for pl in np.unique(h.planes.round(7), axis=0):
                face = h.vertices[np.abs(h.vertices@pl[:3]-pl[3]) < 2e-6]
                if len(face) < 3: raise ValueError('unknown hand hull')
                c = face.mean(0); u = face[0]-c; u /= np.linalg.norm(u); v = np.cross(pl[:3], u)
                face = face[np.argsort(np.arctan2((face-c)@v, (face-c)@u))]
                for p, q in zip(face, np.roll(face, -1, axis=0)):
                    steps = max(1, math.ceil(max(np.linalg.norm(p-c), np.linalg.norm(q-c), np.linalg.norm(p-q))/.003))
                    for j in range(steps+1):
                        for k in range(steps-j+1): pts.append(c+(p-c)*(j/steps)+(q-c)*(k/steps))
        arr = np.asarray(pts); _, keep = np.unique(np.floor(arr/.002).astype(int), axis=0, return_index=True)
        _MESH[name] = arr[keep]
    return _MESH

def clouds(sim):
    base = sim.env.unwrapped; origin = parts._np(base.scene.env_origins[0]); out = HandClouds()
    for arm in ARMS:
        a = base.scene[f'{arm}_robot']; poses = parts._np(a.data.body_link_pose_w)[0]; chunks = []; fingers = {}
        for n, p in hand_mesh().items():
            pose = poses[a.body_names.index(n)]
            world = p@rot(pose[3:]).T+pose[:3]-origin
            chunks.append(world)
            if n in ('tip_left', 'tip_right'): fingers[n] = world
        out[arm] = np.vstack(chunks)
        out.fingers[arm] = fingers
        q = parts._np(a.data.joint_pos)[0]
        out.opening[arm] = -sum(float(q[a.joint_names.index(n)]) for n in ('joint7', 'joint8'))
        if not math.isfinite(out.opening[arm]): raise ValueError('unknown finger opening')
    return out


def _enable_tissue(emb):
    """Runs inside the layout placement, after every body sits at its exact lot pose and before the settle."""
    try:
        set_joints(set(range(6)))
    except Exception as e:
        _SIM['broken'] = repr(e); _REC['invalid'] = True
        print('[broccoli] tissue enable failed', repr(e), flush=True)

if _enable_tissue not in lots.BEFORE_SETTLE: lots.BEFORE_SETTLE.append(_enable_tissue)

@parts.on_reset
def reset(sim):
    # Runs after the stock reset and before the layout placement. Every tissue joint is disabled and every load is
    # cleared here; the placement then writes absolute poses and zero velocities, re-enables all six joints and
    # settles. The first scored step checks the result against the declared layout.
    fresh(); _SIM['broken'] = None
    try:
        base = sim.env.unwrapped
        make_joints(base); set_joints(set()); apply_load(sim, False)
        for arm in ARMS:
            robot = base.scene[f'{arm}_robot']
            ids = robot.find_joints(['joint7', 'joint8'], preserve_order=True)[0]
            robot.write_joint_effort_limit_to_sim_index(limits=40., joint_ids=ids)
        from pxr import Usd, UsdPhysics, PhysxSchema
        touched = 0
        env = base.scene.env_prim_paths[0]
        for prim in Usd.PrimRange(stage().GetPrimAtPath(env)):
            path = str(prim.GetPath()).lower()
            if prim.HasAPI(UsdPhysics.CollisionAPI) and any(k in path for k in ('/knife/', 'tip_left', 'tip_right')):
                cfg = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                cfg.CreateTorsionalPatchRadiusAttr().Set(.010)
                cfg.CreateMinTorsionalPatchRadiusAttr().Set(.008)
                touched += 1
        if touched < 5: raise ValueError(f'expected knife and fingertip colliders, found {touched}')
    except Exception as e:
        _SIM['broken'] = repr(e); _REC['invalid'] = True
        print('[broccoli] reset failed', repr(e), flush=True)
    _REC['armed'] = True

@parts.on_step
def step(sim):
    try:
        st = sim.states(); hands = clouds(sim); newly = update(st, hands, float(sim.t))
        if newly: set_joints(set(range(6))-_REC['cuts'])
        apply_load(sim, bool(_REC.get('next_forces')))
        if os.environ.get('TRACE_OUT'):
            row = {k: _REC[k] for k in ('t', 'zero', 'invalid', 'setup', 'clear', 'still', 'level', 'since', 'forces', 'min_clearance')}
            row['grip_moment'] = round(_REC.get('grip_moment', 0.), 3)
            row['hands'] = {}
            from yam_arena.constants import TCP_OFFSET
            base = sim.env.unwrapped; origin = parts._np(base.scene.env_origins[0])
            for arm in ARMS:
                robot = base.scene[f'{arm}_robot']; pose = parts._np(robot.data.body_link_pose_w)[0, robot.body_names.index('gripper')]
                row['hands'][arm] = {'tcp': np.round(pose[:3]-origin+rot(pose[3:])@np.array(TCP_OFFSET), 5).tolist(), 'quat': np.round(pose[3:], 5).tolist()}
            row['holding'] = _REC.get('holding', []); row['tool_holding'] = _REC.get('tool_holding', [])
            row['opening'] = dict(hands.opening)
            # Observation-only object poses for diagnosis; the demo never reads them.
            row['poses'] = {n: np.round(np.r_[st[n].pos, st[n].quat_xyzw], 5).tolist() for n in ('stem', 'knife', *NAMES)}
            row.update(cuts=sorted(_REC['cuts']), damaged=sorted(_REC['damaged']), coverage={str(i): int(x.sum()) for i, x in _REC['coverage'].items()},
                       loaded_time={str(i): round(v, 3) for i, v in _REC['loaded_time'].items()}, chords=_REC['chords'], cut_stalks={str(i): v for i, v in _REC['cut_stalks'].items()},
                       broken=_SIM['broken'], score=score(st), lot=lots.current().name)
            with pathlib.Path(os.environ['TRACE_OUT']).with_name('broccoli.jsonl').open('a') as f: f.write(json.dumps(row)+'\n')
    except Exception as e:
        if _SIM['broken'] is None: print('[broccoli] step failed', repr(e), flush=True)
        _SIM['broken'] = repr(e); _REC['invalid'] = True


# -- open-loop demo: reads only the published layout (lots.current()), never simulated state or cut flags -----------
ORDER = (5, 4, 3, 2, 1, 0)       # outermost first; every cut leaves the next stalk's clockwise side open
AIM = 0.006                      # cut 6 mm beyond the severing reference plane: about 20 mm of stalk is retained
SLIDE = 0.062                    # after each cut the head is moved 62 mm straight back along that floret's axis
HOP = 0.015                      # ...lifted 15 mm while it moves, so board friction cannot hold it back
PUSH = 0.030                     # the knife is commanded 30 mm below the board: the arm must push through the resistance
X_CUT = 0.104                    # edge point (knife frame) placed under the stalk axis; the tip passes 20 mm beyond
GRASP = np.array([0.0065, 0., 0.004])   # TCP in the knife frame: near the estimated handle centre of mass
GRIP_Y = 0.010                   # stem grasp 75 mm behind the branch bases, toward the stem's butt
TRANSIT = SURFACE+0.062          # edge height while the knife moves between cuts
PARK_A = KNIFE_A
HOLD_YAW = -math.pi/2            # holding grip turned so the fingers close across the stem
LIFT = 0.06                      # hand lift above a grasp height
HOLD_TILT = 45.                  # holding grip leans its fingertips toward the head, keeping its wrist back
OBLIQUE = {5: math.radians(25)}  # the first cut turns the knife 25 degrees so its handle points away from the holding hand
AIM_BY = {5: 0.009}              # that oblique cut aims 3 mm further out, keeping its whole 7 mm spread inside the 10-30 mm band
# Per-cut aim corrections (mm, + = further from the stem), one per cut in order, from the traced cut positions of each
# layout: the loaded arm lands a few mm off its command, differently for the two arms. They centre each cut's
# retained stalk near 20 mm. They depend only on the published layout, never on simulated state.
AIM_FIX = {'A': (2.0, -0.5, 3.0, 0.5, 2.5, 3.5), 'B': (-5.0, -3.0, -3.5, -2.0, -2.5, -4.5)}



def plan_a(fix=(0.,)*6):
    """Waypoints (holding, cutting) in layout A's frame; B is its mirror image with the arms exchanged."""
    yaw_hold = HEAD_YAW_A+HOLD_YAW
    R = rz(HEAD_YAW_A)                       # heading of the head; it also rests tilted forward by PITCH
    tilted = rot(QA['stem'])
    head = np.array(PA['stem'])              # published resting pose of the stem body
    grip = tilted@np.array([0., GRIP_Y, 0.])+np.array([0., 0., 0.004])
    g = head+grip
    hold = [Waypoint(2.5, xyz(g+[0, 0, LIFT]), -.030, 'above stem', yaw_hold), Waypoint(2.0, xyz(g), -.030, 'around stem', yaw_hold),
            Waypoint(1.0, xyz(g), -.010, 'grip stem', yaw_hold), Waypoint(3.0, xyz(g), -.010, 'hold stem', yaw_hold)]
    k0, th0 = KNIFE_A
    kg = k0+rz(th0)@GRASP
    knife = [Waypoint(2.5, xyz(kg+[0, 0, LIFT]), -.025, 'above handle', th0), Waypoint(2.0, xyz(kg), -.025, 'around handle', th0),
             Waypoint(1.0, xyz(kg), -.004, 'grip handle', th0), Waypoint(1.0, xyz(kg), -.004, 'settle handle', th0),
             Waypoint(2.0, xyz(kg+[0, 0, TRANSIT-SURFACE]), -.004, 'lift knife', th0)]

    def tcp(edge_point, theta):
        origin = np.asarray(edge_point)-rz(theta)@np.array([X_CUT, 0, -.012])
        return xyz(origin+rz(theta)@GRASP)
    for visit, i in enumerate(ORDER):
        u = R@np.array([math.cos(ANGLES[i]), math.sin(ANGLES[i]), 0.])
        c = head+tilted@(ANCHOR+(RADII[i]+AIM_BY.get(i, AIM)+fix[visit]/1000)*np.array([math.cos(ANGLES[i]), math.sin(ANGLES[i]), 0.]))
        theta = float(HEAD_YAW_A+ANGLES[i]+math.pi/2+OBLIQUE.get(i, 0.))
        knife += [Waypoint(2.5, tcp((c[0], c[1], TRANSIT), theta), -.004, f'over stalk {i+1}', theta),
                  Waypoint(1.5, tcp((c[0], c[1], Z_STEM+.014), theta), -.004, 'edge above stalk', theta),
                  Waypoint(4.0, tcp((c[0], c[1], SURFACE-PUSH), theta), -.004, 'cut to board', theta),
                  Waypoint(0.7, tcp((c[0], c[1], SURFACE-PUSH), theta), -.004, 'edge on board', theta),
                  Waypoint(1.5, tcp((c[0], c[1], TRANSIT), theta), -.004, 'lift edge', theta)]
        hold += [Waypoint(10.2, xyz(g), -.010, f'hold for cut {i+1}', yaw_hold)]
        if visit < len(ORDER)-1:
            # Lift the head clear of the board, carry it straight back along this floret's axis, set it down.
            up = np.array([0., 0., HOP])
            hold += [Waypoint(0.8, xyz(g+up), -.010, 'lift head', yaw_hold)]
            head = head-SLIDE*u
            g = head+grip
            hold += [Waypoint(2.2, xyz(g+up), -.010, 'move head clear', yaw_hold), Waypoint(0.8, xyz(g), -.010, 'set head down', yaw_hold)]
            knife += [Waypoint(3.8, knife[-1].pos, -.004, 'wait', theta)]
    p0, th = PARK_A
    pg = p0+rz(th)@GRASP
    knife += [Waypoint(3.0, xyz(pg+[0, 0, TRANSIT-SURFACE]), -.004, 'carry knife back', th),
              Waypoint(2.0, xyz(pg+[0, 0, .002]), -.004, 'set knife down', th),
              Waypoint(1.0, xyz(pg+[0, 0, .002]), -.025, 'release knife', th),
              Waypoint(2.0, xyz(pg+[0, 0, LIFT]), -.025, 'lift clear', th),
              Waypoint(3.0, xyz(pg+[0, 0, LIFT]), -.025, 'still', th)]
    hold += [Waypoint(1.0, xyz(g), -.030, 'release stem', yaw_hold), Waypoint(2.0, xyz(g+[0, 0, LIFT]), -.030, 'lift clear', yaw_hold)]
    rest = sum(w.duration for w in knife)-sum(w.duration for w in hold)
    hold += [Waypoint(max(rest, 3.0), xyz(g+[0, 0, LIFT]), -.030, 'still', yaw_hold)]
    heading = HEAD_YAW_A-math.pi/2       # along the stem toward the branches: away from the holding arm's base
    hold = [tilt.Waypoint(w.duration, w.pos, w.finger, w.label, float(w.yaw), tilt=HOLD_TILT, heading=heading) for w in hold]
    return hold, knife


def demo():
    name = lots.current().name
    hold, knife = plan_a(AIM_FIX[name])
    if name == 'B':
        def mirror(w):
            if getattr(w, 'tilt', 0.):
                return tilt.Waypoint(w.duration, (w.pos[0], -w.pos[1], w.pos[2]), w.finger, w.label, -float(w.yaw), tilt=w.tilt, heading=-w.heading)
            return Waypoint(w.duration, (w.pos[0], -w.pos[1], w.pos[2]), w.finger, w.label, -float(w.yaw))
        return {'right': [mirror(w) for w in hold], 'left': [mirror(w) for w in knife]}
    return {'left': hold, 'right': knife}
