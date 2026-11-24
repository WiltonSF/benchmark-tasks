"""A fresh half head of broccoli lies cut-face down on a white cutting board. Six short, unequal branches grow
from different points along its pale-green main stem, three on each side. The green crowns face the cutting arm.
A black-handled 76 mm paring knife stands on the flat underside of its handle near the board edge, blade toward -x and edge down.
A steel ruler lies beside the board. The second layout mirrors the head and exchanges the arms' roles.
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
# Representative half-head, informed by Nov 23 readings: 4/4 halves had six usable florets; 23-39 mm free stalks,
# 5-10 mm thickness (median 7 mm), 29-48 mm crowns. Only these size ranges/count and whole-half masses were measured.
# The symmetric branch origins/angles, main-stem dimensions and branch heights below are explicit geometric estimates.
STEM_R = .020        # estimated radius of original main stem; its upper semicircle remains after halving
STEM_Y = (-.085,.060)
STEM_TIP_R = .016     # estimated taper: original diameter32 mm over the last25 mm
STEM_TIP_END = -.060
Z_STEM = SURFACE + .0005       # flat cut face, not the centre of a whole cylinder
Z_KNIFE = SURFACE + .012 + .0005
ANGLES = np.radians([-10, -40, -65, -115, -140, -170])
ORIGINS = np.array([[.016,-.015,.013], [.016,-.045,.013], [.010,-.070,.013],
                    [-.010,-.070,.013], [-.016,-.045,.013], [-.016,-.015,.013]])
LENGTHS = np.array([.027, .032, .033, .033, .032, .027])  # origin to first side branch, within measured range
NAMES = tuple(f'floret_{i+1}' for i in range(6))
CROWN_WIDTHS = np.array([.038, .040, .044, .044, .040, .038])
WIDTHS = CROWN_WIDTHS/.9  # compound crown's widest dimension is .9 times this construction parameter
# Blade contact estimate, per metre of edge in actual stalk tissue. Chen et al.2018, DOI
# 10.11975/j.issn.1002-6819.2018.23.005: Table3 13.50 +/-1.69 kPa (straight8-degree blade), Table5 up to17.44 kPa.
# Those stresses imply0.52-0.67 N on a7 mm circular section. This model keeps its previous engineering coefficients,
# hence3.85 N for a square advancing7 mm cut (2.8 feed+1.05 drag). It is an uncalibrated allowance for a different
# 15-degree Mercer edge and stalk variation, not a measurement, fit or proven upper bound. Replace FEED/DRAG with
# setup-specific readings when available. No coefficient was reduced for the measured thinner geometry.
FEED = 400.
DRAG = 150.
SLOW = 0.005          # m/s: drag acts above this relative speed; feed stops when the edge is drawn back faster than this
SMOOTH = 3            # control steps over which the edge's motion relative to the stalk is measured (0.15 s)
STALK_R = .0035     # measured sample median thickness 7 mm; representative constant across all six branches
ARMS = ('left', 'right')
FORK = .026         # reference plane to first side branch; not the origin-to-branch length
ALONG = tuple((float(FORK-length), FORK-.002) for length in LENGTHS)  # only existing, unbranched tissue
SETUP_TOL = (0.004, 3.0)
# Unmeasured physical properties. Replace these constants when setup-specific readings are supplied.
KNIFE_MASS = .05       # kg, estimate from 0.11 lb retailer shipping weight, not a weighed knife
HALF_MASS = .125       # kg, one measured half (observed halves 112, 125, 131, 133 g)
FLORET_MASS = .014     # kg per distal floret, estimated allocation, not individually weighed
STEM_MASS = HALF_MASS-6*FLORET_MASS  # remaining main stem + attached proximal branches, 41 g
BOARD_FRICTION = (.7, .6)
PRODUCE_FRICTION = (.6, .5)
KNIFE_FRICTION = (.7, .6)
HANDLE_SIZE = (.100, .022, .024)       # m; estimates, not maker dimensions
GUARD_SIZE = (.010, .030, .026)
GUARD_AT = (.050, 0., .001)
BLADE_SIZE = (.076, .002, .018)        # 76 mm maker blade length; thickness and height estimates
BLADE_AT = (.088, 0., -.003)


def rz(a): return parts._rot((0, 0, 1), a)
def rot(q): return parts._quat_to_matrix(q)
def xyz(p): return tuple(float(v) for v in p)

BOARD = parts.Part('board', pos=(0.29, 0., TOP), friction=BOARD_FRICTION, shapes=[
    parts.box((0.3683, 0.53594, BOARD_H), (0, 0, BOARD_H/2), (0.91, 0.91, 0.87)),
    *[parts.box((0.012, 0.02, 0.004), (x, y, 0.002), (0.16, 0.17, 0.18)) for x in (-0.165, 0.165) for y in (-0.24, 0.24)]])
RULER = parts.Part('ruler', pos=(0.49, 0., TOP+0.001), shapes=[
    parts.box((0.020, 0.30, 0.002), color=(0.62, 0.65, 0.66)),
    *[parts.box((0.012 if i % 5 == 0 else 0.006, 0.001, 0.0004), (0, i*.01-.15, .0012), (0.08, .08, .08), solid=False) for i in range(31)]])
# Flat-faced half stem with an estimated narrower head end, approximated by2 mm horizontal layers.
base_shapes = []
for low_y,high_y,radius in ((STEM_TIP_END,STEM_Y[1],STEM_R),(STEM_Y[0],STEM_TIP_END,STEM_TIP_R)):
    for z in np.arange(.001,radius,.002):
        width = 2*math.sqrt(radius**2-z**2)
        base_shapes.append(parts.box((width,high_y-low_y,.002),(0,(high_y+low_y)/2,z),STALK))
STEM_MAIN_SHAPES = len(base_shapes)
CUTS = []
for i, angle in enumerate(ANGLES):
    u = np.array([math.cos(angle), math.sin(angle), 0.])
    length = LENGTHS[i]-FORK
    CUTS.append(ORIGINS[i]+length*u)
    base_shapes.append(parts.cylinder(STALK_R, length, xyz(ORIGINS[i]+length/2*u), STALK, axis='x', yaw=float(angle)))
# Layout A: bare stem butt toward the holding arm. B reflects the symmetric representative half-head.
HEAD_A = np.array([.240,-.030,Z_STEM])
HEAD_YAW_A = math.radians(40)
WORK_HEAD_A = np.array([.240,-.030,Z_STEM])
WORK_YAW_A = math.radians(40)
KNIFE_A = (np.array([.330, -.225, Z_KNIFE]), math.pi)
STEM = parts.Part('stem', pos=xyz(HEAD_A), yaw=HEAD_YAW_A, mass=STEM_MASS, friction=PRODUCE_FRICTION, shapes=base_shapes)
FLORETS = []
for i, (c, a, w) in enumerate(zip(CUTS, ANGLES, WIDTHS)):
    shapes = [parts.cylinder(STALK_R, .035, (.0175, 0, 0), STALK, axis='x'),
              parts.cylinder(.0025, .020, (FORK, 0, 0), STALK, axis='y'),
              parts.cylinder(w*.27, .025, (.042, 0, 0), GREEN)]
    for phi in np.linspace(0, 2*math.pi, 7)[:-1]:
        shapes.append(parts.cylinder(w*.16, .017, (.042+w*.29*math.cos(phi), w*.29*math.sin(phi), .009), BUD))
    FLORETS.append(parts.Part(NAMES[i], pos=xyz(HEAD_A+rz(HEAD_YAW_A)@c), yaw=float(HEAD_YAW_A+a), mass=FLORET_MASS, friction=PRODUCE_FRICTION, shapes=shapes))
# Handle origin is the native grip centre; handle, finger guard and edge share one flat underside (z = -12 mm).
KNIFE = parts.Part('knife', pos=xyz(KNIFE_A[0]), yaw=KNIFE_A[1], mass=KNIFE_MASS, friction=KNIFE_FRICTION, shapes=[
    parts.box(HANDLE_SIZE, color=(.07, .07, .07)),
    parts.box(GUARD_SIZE, GUARD_AT, (.09, .09, .09)),
    parts.box(BLADE_SIZE, BLADE_AT, (.68, .70, .71)),
    parts.box((.072, .0025, .0015), (.088, 0, -.0112), (.93, .94, .94), solid=False)])
OBJECTS = [BOARD, RULER, STEM, KNIFE, *FLORETS]
KNIFE_SOLID = type('KnifeSolid', (), {'shapes': KNIFE.shapes})()   # whole knife, blade included, for the on-board check
MOVE = [STEM, KNIFE, *FLORETS]


def rx(a): return parts._rot((1, 0, 0), a)
PITCH = 0.  # the halved main stem rests on its flat face; no imposed crown-support tilt
PIVOT = np.zeros(3)

def head_layout(origin, yaw):
    """Exact resting poses of stem and florets: the halved main stem rests on its flat cut face."""
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
GRID = np.array([[y,z] for y in np.linspace(-STALK_R,STALK_R,7) for z in np.linspace(-STALK_R,STALK_R,7) if y*y+z*z <= STALK_R**2+1e-10])
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

def chord(knife, food, shift=(0., 0., 0.), i=0):
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
        sa = (ALONG[i][0]-a[0])/d[0]; sb = (.035-a[0])/d[0]
        s0 = max(s0, min(sa, sb)); s1 = min(s1, max(sa, sb))
    elif not ALONG[i][0] <= a[0] <= .035: return 0., None
    if s1 <= s0: return 0., None
    return float((s1-s0)*np.linalg.norm(d)), np.asarray(food.pos)+r@(a+d*(s0+s1)/2)-np.asarray(shift)


def edge_mid(knife, food):
    """Midpoint of the knife edge in the stalk's own frame."""
    return (np.asarray(knife.pos)+rot(knife.quat_xyzw)@np.array([.089, 0., -.012])-np.asarray(food.pos))@rot(food.quat_xyzw)


def cutting_force(oldknife, knife, oldfood, food, dt, v=None, i=0):
    """Resisting force on the knife (world, N), its point and the edge length in the stalk, for the coming control step.

    The force depends on where the edge is now: it acts only while the edge is inside the stalk section. Feed resistance
    (FEED per metre of edge inside) pushes back against the edge-first direction unless the edge is being drawn back out
    faster than SLOW; drag (DRAG per metre) opposes the edge's motion relative to the stalk when that is faster than
    SLOW. v is that relative velocity (world, m/s), measured over the last SMOOTH steps by update(); without it, the
    motion from oldknife/oldfood over dt is used."""
    length, point = chord(knife, food, i=i)
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


def edge_hits(oldknife, knife, oldfood, food, i=0):
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
    crossed &= (x >= ALONG[i][0]-1e-9) & (x <= ALONG[i][1]+1e-9)
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
    """Opposed fingertips contact the physical half-stem layers, within a closed physical opening."""
    if not hasattr(clouds, 'fingers') or arm not in clouds.fingers: return False
    if not .020 <= clouds.opening.get(arm, -1) <= .055: return False
    directions = []
    r = rot(stem.quat_xyzw)
    for name in ('tip_left', 'tip_right'):
        pts = clouds.fingers[arm].get(name)
        if pts is None or not len(pts): return False
        q = (pts-stem.pos)@r
        distances = [np.linalg.norm(np.maximum(np.abs(q-np.asarray(sh.at))-np.asarray(sh.size)/2, 0.), axis=1)
                     for sh in STEM.shapes[:STEM_MAIN_SHAPES]]
        near = q[np.min(distances,axis=0) <= .003]
        if not len(near): return False
        v = (near[:, [0, 2]]-np.array([0., .010])).mean(axis=0)
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
        d = np.linalg.norm(np.maximum(np.abs(q)-np.asarray(HANDLE_SIZE)/2, 0.), axis=1)
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
        for size, at in ((HANDLE_SIZE, (0, 0, 0)), (GUARD_SIZE, GUARD_AT)):
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
        touching = bool(np.any((local[:, 0] >= ALONG[i][0]-.002) & (local[:, 0] <= .035) & (np.linalg.norm(local[:, 1:], axis=1) < .014)))
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
        crossing, axial = edge_hits(oldstates['knife'], knife, oldstates[NAMES[i]], st, i=i)
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
        f, point, length = cutting_force(knife, knife, objects[NAMES[i]], objects[NAMES[i]], 0., v, i=i)
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
    """Centre of mass of the knife's colliders (handle, finger guard and blade), body frame."""
    vol = np.array([float(np.prod(sh.size)) for sh in KNIFE.shapes if sh.solid])
    at = np.array([sh.at for sh in KNIFE.shapes if sh.solid], float)
    return (at*vol[:, None]).sum(0)/vol.sum()
KNIFE_COM = knife_com()

def grip_deviation(hand_rotation, knife_rotation, reference):
    """Observation-only knife rotation in the grip since its first opposed-handle contact, in degrees."""
    relative = np.asarray(hand_rotation).T@np.asarray(knife_rotation)
    delta = np.asarray(reference).T@relative
    return math.degrees(math.acos(float(np.clip((np.trace(delta)-1)/2, -1., 1.))))


def knife_wrench(knife, forces):
    """Total resistance on the knife and its moment about the knife's centre of mass (world frame), taking each
    stalk's force where the edge is in that stalk. apply_load applies both, so the load acts at the blade."""
    com = np.asarray(knife.pos)+rot(knife.quat_xyzw)@KNIFE_COM
    total = np.zeros(3); moment = np.zeros(3)
    for force, point, _ in forces.values():
        total += force; moment += np.cross(np.asarray(point)-com, force)
    return total, moment

def apply_load(sim, on):
    """Cutting resistance at the blade: the knife gets the summed force at its centre of mass plus the moment of that
    force about it (equivalent to the force acting where the edge is in each stalk); each stalk gets the equal and
    opposite force at the same point, with its moment about the floret's centre of mass."""
    import torch
    base = sim.env.unwrapped
    mark_applied(on)
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
    m = torch.tensor(moment, device=dev, dtype=torch.float32).reshape(1, 1, 3)
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
        from pxr import Usd, UsdPhysics, PhysxSchema, Sdf
        # The main half-stem stays physical; only the six branch stalks/crowns use the cutting proxy.
        knife_path = body_path(base, 'knife'); stem_path = body_path(base, 'stem')
        blade_prim = stage().GetPrimAtPath(knife_path+'/s2')
        if not blade_prim.HasAPI(UsdPhysics.CollisionAPI): raise ValueError('blade collider missing')
        targets = [Sdf.Path(body_path(base, n)) for n in NAMES]
        for shape_index in range(STEM_MAIN_SHAPES, STEM_MAIN_SHAPES+6):
            stalk_path = stem_path+f'/s{shape_index}'
            if not stage().GetPrimAtPath(stalk_path).HasAPI(UsdPhysics.CollisionAPI):
                raise ValueError('proximal stalk collider missing: '+stalk_path)
            targets.append(Sdf.Path(stalk_path))
        UsdPhysics.FilteredPairsAPI.Apply(blade_prim).CreateFilteredPairsRel().SetTargets(targets)
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
            row['com_moment'] = round(_REC.get('grip_moment', 0.), 3)
            if 'knife_com' not in _SIM:
                try:   # the simulator's own centre of mass for the knife body, against KNIFE_COM (trace check only)
                    data = sim.env.unwrapped.scene['knife'].data
                    _SIM['knife_com'] = np.round(parts._np(getattr(data, 'body_com_pos_b', None))[0].reshape(-1)[:3], 5).tolist()
                except Exception as e: _SIM['knife_com'] = repr(e)
                row['knife_com'] = _SIM['knife_com']; row['knife_com_declared'] = np.round(KNIFE_COM, 5).tolist()
            row['hands'] = {}
            from yam_arena.constants import TCP_OFFSET
            base = sim.env.unwrapped; origin = parts._np(base.scene.env_origins[0])
            for arm in ARMS:
                robot = base.scene[f'{arm}_robot']; pose = parts._np(robot.data.body_link_pose_w)[0, robot.body_names.index('gripper')]
                row['hands'][arm] = {'tcp': np.round(pose[:3]-origin+rot(pose[3:])@np.array(TCP_OFFSET), 5).tolist(), 'quat': np.round(pose[3:], 5).tolist()}
                # Observation only: arm joints are rotary (N m); the last two gripper joints are prismatic (N).
                row['hands'][arm]['joint_names'] = list(robot.joint_names)
                for field in ('joint_pos', 'applied_torque', 'joint_pos_target', 'joint_effort_limits'):
                    try: row['hands'][arm][field] = np.round(parts._np(getattr(robot.data, field))[0], 4).tolist()
                    except Exception as ex: row['hands'][arm][field] = repr(ex)

            row['grip_moment'] = 0.; row['knife_in_grip_deg'] = None
            tool = _REC.get('tool_holding', [])
            if tool:
                arm = tool[0]; grip = np.asarray(row['hands'][arm]['tcp'])
                moment = sum((np.cross(np.asarray(p)-grip, f) for f, p, _ in _REC['next_forces'].values()), np.zeros(3))
                row['grip_moment'] = round(float(np.linalg.norm(moment)), 3)
                gr = rot(row['hands'][arm]['quat']); kr = rot(st['knife'].quat_xyzw)
                refs = _REC.setdefault('grip_refs', {})
                if arm not in refs: refs[arm] = gr.T@kr
                row['knife_in_grip_deg'] = round(grip_deviation(gr, kr, refs[arm]), 3)
            else: _REC.pop('grip_refs', None)
            row['grip_rotation_warning'] = row['knife_in_grip_deg'] is not None and row['knife_in_grip_deg'] > 5.
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


# -- board-supported open-loop side cuts; no simulated state feedback --------------------
ORDER = (5,4,3,2,1,0)
AIM = .006
AIM_BY = {5:.008,4:.009,3:.010,2:.0115,1:.010,0:.006}
AIM_FIX = {'A':(0.,)*6,'B':(0.,2.,0.,0.,0.,0.)}  # mm toward first branch on Bsecond, unchanged10-30 mm score band
X_CUT_FIX = {'A':(0.,)*6,'B':(0.,-.003,0.,0.,0.,0.)}  # metres along blade; second B stroke keeps extra tip span
SLIDE = .035
MID_SHIFT = np.zeros(3)
GRASP = np.array([.0275,0.,0.])
KNIFE_GRIP_FLIP = math.pi  # exchange the opposed pads; native handle pinch is unchanged
GRIP_X = 0.
GRIP_Y = .060
GRIP_Z = .016
HOLD_TILT = 45.
HOLD_YAW = -math.pi/2
HEAD_LIFT = 0.
CARRY_TIME = 0.
FIRST_APPROACH = 8.5
LIFT = .060
TRANSIT = SURFACE+.065
X_CUT = .118
X_CUT_BY = {5: 0.118, 4: 0.118, 3: 0.118, 2: 0.118, 1: 0.118, 0: 0.118}
CUT_PITCH = 60.
PITCH_BY = {5:45,4:45,3:45,2:45,1:50,0:65}
CHECK_PAST_POSES = True
SIDE_Z_OFFSET = -.001
SIDE_TRAVEL = .010
SIDE_TRAVEL_BY = {5:.016,4:.020,3:.014,2:.020,1:.020,0:.014}
SIDE_START_BY = {5:.008,4:.008,2:.008,1:.008}
FEED_TIME_BY = {2:8.}  # slower fourth stroke, unchanged force law and positions
FINISH_TIME_BY = {2:2.}  # allow the unchanged loaded section to settle before lifting
# Side-entry blade headings, expressed as obliquity from branch+90; the opposed-pad pickup keeps the wrist reachable.
OBLIQUE = {i:math.radians(v) for i,v in {5: 195.0, 4: 140.0, 3: 175.0, 2: 220.0, 1: 190.0, 0: 190.0}.items()}
PARK_A = KNIFE_A


def plan_a(fix=(0.,)*6, xcut_fix=(0.,)*6):
    R=rz(HEAD_YAW_A);head=np.array(PA['stem']);grip=R@np.array([GRIP_X,GRIP_Y,GRIP_Z]);g=head+grip;yawH=HEAD_YAW_A+HOLD_YAW
    hold=[Waypoint(2.5,xyz(g+[0,0,LIFT]),-.030,'above stem',yawH),Waypoint(2.,xyz(g),-.030,'around stem',yawH),Waypoint(1.,xyz(g),-.010,'grip stem',yawH),Waypoint(3.,xyz(g),-.010,'steady board-supported half',yawH)]
    k0,th0=KNIFE_A;kg=k0+rz(th0)@GRASP
    knife=[Waypoint(2.5,xyz(kg+[0,0,LIFT]),-.025,'above handle',th0+KNIFE_GRIP_FLIP),Waypoint(2.,xyz(kg),-.025,'around handle',th0+KNIFE_GRIP_FLIP),Waypoint(1.,xyz(kg),-.004,'grip handle',th0+KNIFE_GRIP_FLIP),Waypoint(1.,xyz(kg),-.004,'seat handle',th0+KNIFE_GRIP_FLIP),Waypoint(2.,xyz(kg+[0,0,TRANSIT-SURFACE]),-.004,'lift knife',th0+KNIFE_GRIP_FLIP)]
    for visit,i in enumerate(ORDER):
        theta=float(HEAD_YAW_A+ANGLES[i]+math.pi/2+OBLIQUE.get(i,0.));pitch=PITCH_BY.get(i,CUT_PITCH);xcut=X_CUT_BY.get(i,X_CUT)+xcut_fix[visit]
        kr=rz(theta)@parts._rot((0,1,0),math.radians(pitch));u=rz(theta)@np.array([1.,0.,0.]);axis=R@np.array([math.cos(ANGLES[i]),math.sin(ANGLES[i]),0.])
        c=head+R@(CUTS[i]+(AIM_BY.get(i,AIM)+fix[visit]/1000)*np.array([math.cos(ANGLES[i]),math.sin(ANGLES[i]),0.]))
        travel=SIDE_TRAVEL_BY.get(i,SIDE_TRAVEL);c=c+np.array([0.,0.,SIDE_Z_OFFSET]);p0=c+SIDE_START_BY.get(i,travel)*u;p1=c-travel*u
        def tcp(edge):return xyz(np.asarray(edge)+kr@(GRASP-np.array([xcut,0.,-.012])))
        def wp(d,p,label):return tilt.Waypoint(d,tcp(p),-.004,label,theta+KNIFE_GRIP_FLIP,tilt=-pitch,heading=theta)
        feed_time=FEED_TIME_BY.get(i,4.);finish_time=FINISH_TIME_BY.get(i,.7)
        knife += [wp(2.5,(p0[0],p0[1],TRANSIT),f'over stalk {i+1}'),wp(1.5,p0,'beside stalk'),wp(feed_time,p1,'slice across section'),wp(finish_time,p1,'finish section'),wp(1.5,(p1[0],p1[1],TRANSIT),'lift edge')]
        hold.append(Waypoint(10.2+feed_time-4.+finish_time-.7,xyz(g),-.010,f'steady cut {i+1}',yawH))
        if visit<5:
            head=head-SLIDE*axis
            if visit==2:head=head+MID_SHIFT
            g=head+grip;hold.append(Waypoint(3.8,xyz(g),-.010,'slide remaining half on board',yawH));neutral=xyz(kg+[0,0,TRANSIT-SURFACE+.040]);knife += [Waypoint(1.9,neutral,-.004,'neutral between cuts',th0+KNIFE_GRIP_FLIP),Waypoint(1.9,neutral,-.004,'settle neutral',th0+KNIFE_GRIP_FLIP)]
    p0,th=PARK_A;pg=p0+rz(th)@GRASP
    knife += [Waypoint(3.,xyz(pg+[0,0,TRANSIT-SURFACE]),-.004,'carry knife back',th+KNIFE_GRIP_FLIP),Waypoint(2.,xyz(pg+[0,0,.002]),-.004,'set knife down',th+KNIFE_GRIP_FLIP),Waypoint(1.,xyz(pg+[0,0,.002]),-.025,'release knife',th+KNIFE_GRIP_FLIP),Waypoint(2.,xyz(pg+[0,0,LIFT]),-.025,'lift clear',th+KNIFE_GRIP_FLIP),Waypoint(3.,xyz(pg+[0,0,LIFT]),-.025,'still',th+KNIFE_GRIP_FLIP)]
    hold += [Waypoint(1.,xyz(g),-.030,'release stem',yawH),Waypoint(2.,xyz(g+[0,0,LIFT]),-.030,'lift clear',yawH)]
    rest=sum(w.duration for w in knife)-sum(w.duration for w in hold);hold.append(Waypoint(max(rest,3.),xyz(g+[0,0,LIFT]),-.030,'still',yawH))
    hold=[tilt.Waypoint(w.duration,w.pos,w.finger,w.label,float(w.yaw),tilt=HOLD_TILT,heading=HEAD_YAW_A-math.pi/2) for w in hold]
    return hold,knife


def demo():
    name = lots.current().name
    hold, knife = plan_a(AIM_FIX[name],X_CUT_FIX[name])
    if name == 'B':
        def mirror(w):
            if getattr(w, 'tilt', 0.):
                return tilt.Waypoint(w.duration, (w.pos[0], -w.pos[1], w.pos[2]), w.finger, w.label, -float(w.yaw), tilt=w.tilt, heading=-w.heading)
            return Waypoint(w.duration, (w.pos[0], -w.pos[1], w.pos[2]), w.finger, w.label, -float(w.yaw))
        return {'right': [mirror(w) for w in hold], 'left': [mirror(w) for w in knife]}
    return {'left': hold, 'right': knife}
