"""A cold saucepan contains brown stock with a yellow floating fat layer. A loose
metal ladle rests on a utensil rest. An empty waste cup stands beside the pan.
The fat and stock are visible, conserved two-layer material proxies.
"""

import json
import itertools
import math
import os
import pathlib
import sys
from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts
import lots
import tilt
from yam_arena.demo import FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.10
SCORE_OVER = "end"
CLEAR = 0.005
EPS = 1e-9  # numerical comparison tolerance, metres or dimensionless
PAN_R, PAN_H = 0.070, 0.085
PAN_BOTTOM = 0.005
BOWL_R, BOWL_D = 0.035, 0.0234
BOWL = np.array((0.130, 0., -0.035))
INITIAL_FAT, INITIAL_BROTH = 160e-6, 650e-6
FLOW_C = 0.55 * 2/3 * math.sqrt(2*9.81)
STEEL, BLACK = (0.62, 0.65, 0.67), (0.13, 0.14, 0.15)
FAT, BROTH = (0.98, 0.77, 0.12), (0.34, 0.14, 0.045)


def wedge(r0, r1, z0, z1, a0, a1, color, center=(0, 0, 0), inner_top=None, outer_top=None):
    """One convex wall sector; adjacent sectors meet without open slots."""
    rt0 = r0 if inner_top is None else inner_top
    rt1 = r1 if outer_top is None else outer_top
    vertices = np.array([(r*math.cos(a), r*math.sin(a), z)
        for z, radii in ((z0,(r0,r1)), (z1,(rt0,rt1)))
        for r,a in ((radii[0],a0),(radii[1],a0),(radii[1],a1),(radii[0],a1))]) + center
    lo,hi=vertices.min(0),vertices.max(0)
    return parts.Shape("mesh", tuple(hi-lo), tuple((hi+lo)/2), color,
                       vertices=tuple(tuple(float(x) for x in v) for v in vertices))


def wall_ring(radius, wall, height, z, color, n=32):
    return [wedge(radius-wall/2, radius+wall/2, z-height/2, z+height/2,
                  2*math.pi*i/n, 2*math.pi*(i+1)/n, color) for i in range(n)]


PAN = parts.Part("saucepan", pos=(0.51, 0.09, TOP+0.002), mass=0.99,
    friction=(1.0, 0.8), shapes=[
        parts.cylinder(PAN_R+0.002, PAN_BOTTOM, (0, 0, PAN_BOTTOM/2), STEEL),
        *wall_ring(PAN_R+0.002, 0.004, PAN_H-PAN_BOTTOM,
                   (PAN_H+PAN_BOTTOM)/2, STEEL),
        parts.box((0.025, 0.095, 0.014), (0, 0.122, 0.057), BLACK),
        parts.box((0.018, 0.038, 0.020), (0, 0.085, 0.055), STEEL),
    ])
# A dished metal bowl has an oblate hemispherical cavity, not a solid scoop.
ladle_shapes = [
    parts.box((0.135, 0.025, 0.020), (-0.035, 0, 0), BLACK),
    parts.box((0.050, 0.016, 0.006), (0.0575, 0, -0.006), STEEL),
    parts.box((0.028, 0.016, 0.018), (0.092, 0, -0.014), STEEL),
    parts.box((0.027, 0.016, 0.018), (0.113, 0, -0.026), STEEL),
    parts.cylinder(0.0112, 0.0015, tuple(BOWL+(0, 0, -BOWL_D)), STEEL),
]
for z0,z1 in zip(np.linspace(-BOWL_D+0.001, 0, 8)[:-1], np.linspace(-BOWL_D+0.001, 0, 8)[1:]):
    r0=BOWL_R*math.sqrt(max(0.,1-(z0/BOWL_D)**2))
    r1=BOWL_R*math.sqrt(max(0.,1-(z1/BOWL_D)**2))
    for i in range(24):
        ladle_shapes.append(wedge(r0, r0+0.0015, float(z0), float(z1),
            2*math.pi*i/24, 2*math.pi*(i+1)/24, STEEL, center=BOWL,
            inner_top=r1, outer_top=r1+0.0015))
LADLE = parts.Part("ladle", pos=(0.34, -0.25, TOP+0.090), mass=0.10,
                   friction=(1.0, 0.8), shapes=ladle_shapes)
CUP_R, CUP_H = 0.060, 0.065
CUP = parts.Part("waste_cup", pos=(0.40, -0.14, TOP), mass=0.15,
    friction=(1.0, 0.8), shapes=[
        parts.cylinder(CUP_R+0.002, 0.004, (0, 0, 0.002), (0.2, 0.36, 0.55)),
        *wall_ring(CUP_R+0.0015, 0.003, CUP_H-0.004,
                   (CUP_H+0.004)/2, (0.2, 0.36, 0.55)),
    ])
REST = parts.Part("utensil_rest", pos=(0.34, -0.295, TOP), shapes=[
    parts.box((0.260, 0.170, 0.008), (0.040, 0, 0.004), (0.3, 0.4, 0.46)),
    parts.box((0.030, 0.008, 0.080), (0, -0.045, 0.040), (0.3, 0.4, 0.46)),
    parts.box((0.030, 0.008, 0.080), (0, 0.045, 0.040), (0.3, 0.4, 0.46)),
    parts.box((0.080, 0.160, 0.032), (0.130, 0, 0.016), (0.3, 0.4, 0.46)),
])
MAT = parts.Part("pan_mat", pos=(0.51, 0.09, TOP), shapes=[
    parts.cylinder(0.080, 0.002, (0,0,0.001), BLACK)])
OBJECTS = [PAN, LADLE, CUP, REST, MAT]
LOTS = lots.declare(OBJECTS,
    lots.Lot("A", place={"ladle": LADLE.pos}),
    lots.Lot("B", place={"ladle": (0.34, -0.34, TOP+0.090)}))
DEMO_TOUCHES = set()

# Equal-area surface cells carry fat. Conservative relaxation redistributes it.
CELL = 0.010
GRID = np.array([(x, y) for x in np.arange(-0.075, 0.076, CELL)
                for y in np.arange(-0.075, 0.076, CELL)
                if x*x+y*y < (PAN_R-0.004)**2])
AREA = math.pi*PAN_R**2
CELL_AREA = AREA/len(GRID)
RIM = np.array([(BOWL_R*math.cos(a), BOWL_R*math.sin(a), 0.)
                for a in np.linspace(0, 2*math.pi, 72, endpoint=False)])
RIM_DS = 2*math.pi*BOWL_R/len(RIM)
# Quadrature of the actual curved cavity. Weights sum to 60.04 mL.
vox, weights = [], []
dx = 2*BOWL_R/25
for x in np.linspace(-BOWL_R+dx/2, BOWL_R-dx/2, 25):
    for y in np.linspace(-BOWL_R+dx/2, BOWL_R-dx/2, 25):
        r2 = (x*x+y*y)/BOWL_R**2
        if r2 >= 1:
            continue
        depth = BOWL_D*math.sqrt(1-r2)
        for j in range(12):
            vox.append((x, y, -depth+(j+0.5)*depth/12))
            weights.append(dx*dx*depth/12)
VOX = np.array(vox)
WEIGHTS = np.array(weights)
CAPACITY = 2/3*math.pi*BOWL_R**2*BOWL_D
WEIGHTS *= CAPACITY/WEIGHTS.sum()
CUP_VOX = np.array([(x, y, z) for x in np.linspace(-CUP_R, CUP_R, 21)
    for y in np.linspace(-CUP_R, CUP_R, 21) if x*x+y*y < CUP_R**2
    for z in np.linspace(0.004+(CUP_H-0.004)/24, CUP_H-(CUP_H-0.004)/24, 12)])


def transform(state, points):
    r = parts._quat_to_matrix(state.quat_xyzw)
    return np.asarray(points) @ r.T + state.pos


def cavity(state):
    points = transform(state, VOX+BOWL)
    rim = transform(state, RIM+BOWL)
    order = np.argsort(points[:, 2])
    heights, cumulative = points[order, 2], np.cumsum(WEIGHTS[order])
    capacity = float(WEIGHTS[points[:, 2] < rim[:, 2].min()].sum())
    return rim, heights, cumulative, capacity


def level(volume, heights, cumulative):
    return float(np.interp(max(volume, 0.), cumulative, heights,
                           left=heights[0]-0.001, right=heights[-1]+0.001))


@dataclass
class Liquid:
    fat: np.ndarray = field(default_factory=lambda: np.full(len(GRID), INITIAL_FAT/len(GRID)))
    broth: float = INITIAL_BROTH
    ladle_fat: float = 0.
    ladle_broth: float = 0.
    cup_fat: float = 0.
    cup_broth: float = 0.
    spill_fat: float = 0.
    spill_broth: float = 0.
    invalid: bool = False

    def conserved(self):
        fs = self.fat.sum()+self.ladle_fat+self.cup_fat+self.spill_fat
        bs = self.broth+self.ladle_broth+self.cup_broth+self.spill_broth
        values = np.r_[self.fat, self.broth, self.ladle_fat, self.ladle_broth,
                       self.cup_fat, self.cup_broth, self.spill_fat, self.spill_broth]
        return (np.isfinite(values).all() and values.min() >= -1e-12
                and abs(fs-INITIAL_FAT) < 1e-10 and abs(bs-INITIAL_BROTH) < 1e-10)

    def receive(self, f, b, exit_point, pan, cup, allow_cup=True):
        """Trace a gravity stream to the first upright vessel aperture below it."""
        interiors, destinations = [], []
        for name, s, radius, height in (("pan", pan, PAN_R, PAN_H),
                                         ("cup", cup, CUP_R, CUP_H)):
            if name == "cup" and not allow_cup:
                continue
            r = parts._quat_to_matrix(s.quat_xyzw)
            bottom = PAN_BOTTOM if name == "pan" else 0.004
            if r[2, 2] > 0.999 and exit_point[2] >= s.pos[2]+bottom:
                if np.linalg.norm(exit_point[:2]-s.pos[:2]) <= radius-0.003:
                    destinations.append((s.pos[2]+height, name))
                    if exit_point[2] <= s.pos[2]+height:
                        interiors.append((radius, name))
        # A submerged rim returns overflow directly to the containing pan.
        if (np.linalg.norm(exit_point[:2]-pan.pos[:2]) < PAN_R-0.003
                and pan.pos[2]+PAN_BOTTOM <= exit_point[2] <= pan.pos[2]+PAN_H):
            destinations.append((pan.pos[2]+PAN_H, "pan"))
        name = (min(interiors)[1] if interiors else
                (max(destinations)[1] if destinations else "spill"))
        if name == "pan":
            k = int(np.argmin(np.sum((GRID-(exit_point[:2]-pan.pos[:2]))**2, axis=1)))
            self.fat[k] += f
            self.broth += b
        elif name == "cup":
            self.cup_fat += f; self.cup_broth += b
            excess = max(0., self.cup_fat+self.cup_broth-math.pi*CUP_R**2*(CUP_H-0.004))
            ff = min(self.cup_fat, excess)
            bb = excess-ff
            self.cup_fat -= ff; self.cup_broth -= bb
            self.spill_fat += ff; self.spill_broth += bb
        else:
            self.spill_fat += f; self.spill_broth += b

    def advance(self, dt, pan, ladle, cup):
        if self.invalid or not (0 <= dt <= 0.100001) or not self.conserved():
            self.invalid = True; return
        for state in (pan, ladle, cup):
            if not 0.99 <= np.linalg.norm(state.quat_xyzw) <= 1.01:
                self.invalid = True; return
        cup_rim = transform(cup, np.c_[RIM[:, :2]*(CUP_R/BOWL_R), np.full(len(RIM), CUP_H)])
        cup_points = transform(cup, CUP_VOX)
        cup_capacity = math.pi*CUP_R**2*(CUP_H-0.004)*np.mean(cup_points[:, 2] < cup_rim[:, 2].min())
        cup_excess = max(0., self.cup_fat+self.cup_broth-cup_capacity)
        cf = min(cup_excess, self.cup_fat); cb = cup_excess-cf
        self.cup_fat -= cf; self.cup_broth -= cb
        self.receive(cf, cb, cup_rim[np.argmin(cup_rim[:, 2])], pan, cup, allow_cup=False)
        rim, heights, cumulative, capacity = cavity(ladle)
        # Spill any volume that the rotated curved bowl can no longer retain.
        excess = max(0., self.ladle_fat+self.ladle_broth-capacity)
        ff = min(self.ladle_fat, excess)  # floating layer leaves first on tilting
        bb = excess-ff
        self.ladle_fat -= ff; self.ladle_broth -= bb
        self.receive(ff, bb, rim[np.argmin(rim[:, 2])], pan, cup)
        self.fat += (self.fat.mean()-self.fat)*(-math.expm1(-dt/0.4))
        if parts._quat_to_matrix(pan.quat_xyzw)[2, 2] < 0.999:
            self.invalid = True; return
        interface = pan.pos[2]+PAN_BOTTOM+self.broth/AREA
        inner = level(self.ladle_fat+self.ladle_broth, heights, cumulative)
        available = max(0., capacity-self.ladle_fat-self.ladle_broth)
        for p in rim:
            if available <= 1e-12 or np.linalg.norm(p[:2]-pan.pos[:2]) >= PAN_R-0.003:
                continue
            k = int(np.argmin(np.sum((GRID-(p[:2]-pan.pos[:2]))**2, axis=1)))
            surface = interface+self.fat[k]/CELL_AREA
            head, back = max(0., surface-p[2]), max(0., inner-p[2])
            if head <= back:
                continue
            # Integrate the top layer's head separately; deep rims admit broth.
            total = FLOW_C*RIM_DS*(head**1.5-back**1.5)*dt
            fat_head = max(0., surface-max(p[2], interface))
            fat_back = max(0., min(back, fat_head))
            f_share = np.clip((fat_head**1.5-fat_back**1.5)/(head**1.5-back**1.5), 0., 1.)
            total = min(total, available)
            f = min(total*f_share, self.fat[k])
            b = min(total*(1-f_share), self.broth)
            self.fat[k] -= f; self.broth -= b
            self.ladle_fat += f; self.ladle_broth += b
            available -= f+b
        if not self.conserved():
            self.invalid = True


@lru_cache(maxsize=1)
def hand_samples():
    """Cache a <=3 mm barycentric mesh over every collision-hull face."""
    from yam_arena import analysis
    result = {}
    for name in ("gripper", "tip_left", "tip_right"):
        cloud = []
        for h in analysis.link_hulls().get(name, []):
            cloud.extend(h.vertices)
            for plane in np.unique(h.planes.round(7), axis=0):
                face = h.vertices[np.abs(h.vertices@plane[:3]-plane[3]) < 2e-6]
                if len(face) < 3:
                    raise ValueError("unknown collision face")
                c = face.mean(0); u = face[0]-c; u /= np.linalg.norm(u)
                v = np.cross(plane[:3], u)
                face = face[np.argsort(np.arctan2((face-c)@v, (face-c)@u))]
                for a, b in zip(face, np.roll(face, -1, axis=0)):
                    n = max(1, math.ceil(max(np.linalg.norm(a-c), np.linalg.norm(b-c), np.linalg.norm(a-b))/0.003))
                    for i in range(n+1):
                        for j in range(n-i+1):
                            cloud.append(c+(a-c)*(i/n)+(b-c)*(j/n))
        if not cloud:
            raise ValueError("missing hand hull")
        cloud = np.asarray(cloud)
        _, keep = np.unique(np.floor(cloud/0.002).astype(int), axis=0, return_index=True)
        result[name] = cloud[keep]
    return result


def convex_clear(vertices, center, rotation, size, clearance=CLEAR):
    """Certify convex-hull/box separation, including face and edge contact.

    A support-plane distance is a lower bound on true separation. Only a lower
    bound strictly over the required gap earns clearance; failed convergence
    returns False. No vertex-only approximation can certify clearance.
    """
    a = np.asarray(vertices, float)
    center, rotation, size = (np.asarray(x, float) for x in (center, rotation, size))
    if (a.ndim != 2 or a.shape[1] != 3 or len(a) == 0
            or not all(np.isfinite(x).all() for x in (a, center, rotation, size))):
        return False
    signs = np.array(list(itertools.product((-1., 1.), repeat=3)))
    b = center + (signs * size/2) @ rotation.T
    gap = np.maximum(np.maximum(b.min(0)-a.max(0), a.min(0)-b.max(0)), 0)
    if np.linalg.norm(gap) > clearance + 1e-9:
        return True

    def closest(simplex):
        best = None
        for count in range(1, min(4, len(simplex))+1):
            for ids in itertools.combinations(range(len(simplex)), count):
                points = np.array([simplex[i] for i in ids])
                if count == 1:
                    weights = np.ones(1)
                else:
                    edges = (points[1:]-points[0]).T
                    tail = np.linalg.lstsq(edges, -points[0], rcond=None)[0]
                    weights = np.r_[1-tail.sum(), tail]
                if np.min(weights) < -1e-10:
                    continue
                weights = np.maximum(weights, 0)
                weights /= weights.sum()
                point = weights @ points
                distance = float(point @ point)
                if best is None or distance < best[0]:
                    best = (distance, point, [p for p,w in zip(points,weights) if w > 1e-10])
        return best

    simplex = [a[0]-b[0]]
    v = simplex[0]
    for _ in range(32):
        length = float(np.linalg.norm(v))
        if length <= clearance + 1e-9:
            return False
        support = a[np.argmin(a @ v)] - b[np.argmax(b @ v)]
        if float(v @ support)/length > clearance + 1e-9:
            return True
        if any(np.linalg.norm(support-p) < 1e-12 for p in simplex):
            return False
        simplex.append(support)
        found = closest(simplex)
        if found is None:
            return False
        _, v, simplex = found
        if not np.isfinite(v).all():
            return False
    return False


def clear_of_shapes(hulls, boxes, clearance=CLEAR):
    """Vectorized conservative bounds skip distant hull/shape pairs."""
    if not hulls or not boxes:
        return False
    vertices = [np.asarray(h.vertices, float) for _, h in hulls]
    if any(v.ndim != 2 or v.shape[1] != 3 or len(v)==0 or not np.isfinite(v).all()
           for v in vertices):
        return False
    centers = np.array([c for _,c,_,_ in boxes])
    rotations = np.array([r for _,_,r,_ in boxes])
    sizes = np.array([s for _,_,_,s in boxes])
    if not all(np.isfinite(v).all() for v in (centers,rotations,sizes)):
        return False
    amin, amax = np.array([v.min(0) for v in vertices]), np.array([v.max(0) for v in vertices])
    half = np.einsum("bij,bj->bi", np.abs(rotations), sizes/2)
    bmin, bmax = centers-half, centers+half
    gap = np.maximum(np.maximum(bmin[None,:,:]-amax[:,None,:],
                                amin[:,None,:]-bmax[None,:,:]), 0)
    near = np.argwhere(np.sum(gap*gap,axis=2) <= (clearance+1e-9)**2)
    return all(convex_clear(vertices[i],centers[j],rotations[j],sizes[j],clearance)
               for i,j in near)




def sampled_clear(points, boxes):
    """A 7 mm allowance covers the face mesh and <=3.5 mm voxel merge."""
    if not points or not boxes:
        return False
    for cloud in points:
        if cloud.ndim != 2 or cloud.shape[1] != 3 or len(cloud) == 0 or not np.isfinite(cloud).all():
            return False
        lo, hi = cloud.min(0), cloud.max(0)
        for _, c, r, size in boxes:
            if not np.isfinite(np.r_[c, r.ravel(), size]).all() or np.any(size <= 0):
                return False
            half = np.abs(r)@(size/2)
            gap = np.maximum(np.maximum(c-half-hi, lo-c-half), 0.)
            if np.linalg.norm(gap) > CLEAR+0.007:
                continue
            q = (cloud-c)@r
            distance = np.linalg.norm(np.maximum(np.abs(q)-size/2, 0.), axis=1)
            if distance.min() <= CLEAR+0.007:
                return False
    return True


def hand_geometry(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    clouds, centers = [], []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for name, local in hand_samples().items():
            p = poses[asset.body_names.index(name)]
            clouds.append(local@parts._quat_to_matrix(p[3:]).T+p[:3]-origin)
            centers.append(p[:3]-origin)
    return clouds, np.concatenate(clouds)


def boxes(spec, state):
    return spec.world_boxes({"base": (state.pos, parts._quat_to_matrix(state.quat_xyzw))})


def stamp(objects):
    return tuple(float(x) for name in ("saucepan", "ladle", "waste_cup")
                 for x in np.r_[objects[name].pos, objects[name].quat_xyzw,
                                 objects[name].lin_vel, objects[name].ang_vel,
                                 objects[name].low, objects[name].high])


def on_rest(ladle):
    """Two actual underside contact points must bear on the two-height rest."""
    if parts._quat_to_matrix(ladle.quat_xyzw)[2,2] < 0.99-EPS:
        return False
    grip_bottom, bowl_bottom = transform(ladle, np.array(((0,0,-0.010),
        tuple(BOWL+(0,0,-BOWL_D-0.00075)))))
    def bears(p, shape):
        c = np.asarray(REST.pos)+shape.at
        half = shape.half
        return bool(np.all(np.abs(p[:2]-c[:2]) <= half[:2]-0.0005)
                    and abs(p[2]-(c[2]+half[2])) <= 0.002+EPS)
    return bool(any(bears(grip_bottom,s) for s in REST.shapes[1:3])
            and bears(bowl_bottom,REST.shapes[3])
            and ladle.low[2] >= TOP-EPS)


@dataclass
class Trial:
    liquid: Liquid = field(default_factory=Liquid)
    last_t: float | None = None
    known: bool = False
    invalid: bool = False
    violation: bool = False
    held: float = 0.
    certificate: tuple | None = None
    hands: np.ndarray | None = None

    def advance(self, t, objects, clear, hands, support=None):
        self.known = False
        try:
            current = stamp(objects)
            dt = 0. if self.last_t is None else float(t-self.last_t)
            if self.last_t is not None and dt <= 0:
                raise ValueError("non-advancing physical interval")
            if (clear is None or hands is None or np.asarray(hands).ndim != 2
                    or np.asarray(hands).shape[1] != 3 or len(hands) < 6):
                raise ValueError("unknown hand state")
            if not np.isfinite(np.r_[current, t, np.asarray(hands).ravel()]).all() or not 0 <= dt <= 0.100001:
                raise ValueError("unknown physical interval")
            pan, ladle, cup = (objects[n] for n in ("saucepan", "ladle", "waste_cup"))
            # The pot must stay standing on its original mat throughout.
            if np.linalg.norm(pan.pos-np.asarray(PAN.pos)) > 0.005+EPS or parts._quat_to_matrix(pan.quat_xyzw)[2, 2] < 0.999-EPS:
                self.violation = True
            if np.linalg.norm(cup.pos-np.asarray(CUP.pos)) > 0.005+EPS or parts._quat_to_matrix(cup.quat_xyzw)[2, 2] < 0.999-EPS:
                self.violation = True
            self.liquid.advance(dt, pan, ladle, cup)
            still = all(s.resting(max_speed=0.01, max_spin=0.10) for s in (pan, ladle, cup))
            hstill = (self.hands is not None and dt > 0 and np.max(np.linalg.norm(hands-self.hands, axis=1))/dt < 0.01)
            ready = (clear and still and hstill and support is True and on_rest(ladle)
                     and abs(cup.low[2]-TOP) <= 0.002+EPS
                     and parts._quat_to_matrix(cup.quat_xyzw)[2, 2] >= 0.999-EPS)
            self.held = self.held+dt if ready else 0.
            self.certificate = current
            self.last_t = float(t); self.hands = np.asarray(hands).copy(); self.known = True
        except (KeyError, AttributeError, ValueError, TypeError):
            self.invalid = True


_trial = Trial()
_support_view = None
_support_force = None
_support_error = None
_support_detail = None


def support_contact(sim):
    """Measured upward ladle/rest contact load; unknown data never certifies support."""
    global _support_view, _support_force, _support_error, _support_detail
    try:
        if _support_view is None:
            from isaaclab_physx.physics import PhysxManager
            physics = PhysxManager.get_physics_sim_view()
            _support_view = physics.create_rigid_contact_view(
                [parts.ROOT_PATHS["ladle"]],
                filter_patterns=[[parts.ROOT_PATHS["utensil_rest"]+s
                                  for s in ("/s1","/s2","/s3")]],
                max_contact_data_count=128)
        raw = _support_view.get_contact_force_matrix(dt=sim.env.unwrapped.sim.get_physics_dt())
        if hasattr(raw, "torch"): raw=raw.torch
        f = raw.detach().cpu().numpy() if hasattr(raw,"detach") else np.asarray(raw.numpy())
        if f.shape != (1,3,3) or not np.isfinite(f).all():
            raise ValueError("unknown support contact data")
        _support_force=f.sum(axis=(0,1)).tolist()
        _support_detail=f[0].tolist()
        _support_error=None
        # s1/s2 are the alternative handle bridges for A/B; s3 is the bowl pad.
        return bool(_support_force[2] >= 0.75 and f[0,:2,2].sum() >= .02
                    and f[0,2,2] >= .02)
    except Exception as error:
        _support_force=None; _support_detail=None; _support_error=type(error).__name__
        return None


def arms_clear(sim, ladle):
    """Other arm links cannot prop the tool while fingertips are clear."""
    from yam_arena import analysis
    base=sim.env.unwrapped
    origin=parts._np(base.scene.env_origins[0])
    hulls=[]
    for arm in ("left_robot","right_robot"):
        asset=base.scene[arm];poses=parts._np(asset.data.body_link_pose_w)[0]
        frames={}
        for i,name in enumerate(asset.body_names):
            if name in ("gripper","tip_left","tip_right"): continue
            m=np.eye(4);m[:3,:3]=parts._quat_to_matrix(poses[i,3:]);m[:3,3]=poses[i,:3]-origin
            if not np.isfinite(m).all(): return False
            frames[name]=m
        hulls.extend(analysis.world_hulls(frames))
    return clear_of_shapes(hulls,boxes(LADLE,ladle))


def show_liquid(sim):
    from pxr import Gf, UsdGeom
    from isaaclab.sim.utils import get_current_stage
    stage = get_current_stage()
    states = sim.states(); liq = _trial.liquid

    def disc(name, radius, height, xyz, color):
        path = "/World/skimming_"+name
        prim = stage.GetPrimAtPath(path)
        if not prim:
            geom = UsdGeom.Cylinder.Define(stage, path)
            geom.CreateAxisAttr("Z"); geom.CreateDisplayColorAttr([color])
            geom.AddTranslateOp()
        geom = UsdGeom.Cylinder(stage.GetPrimAtPath(path))
        geom.GetRadiusAttr().Set(float(radius)); geom.GetHeightAttr().Set(float(max(height, 0.00005)))
        geom.GetOrderedXformOps()[0].Set(Gf.Vec3d(*(float(x) for x in xyz)))
        UsdGeom.Imageable(geom).MakeVisible() if height > 1e-6 else UsdGeom.Imageable(geom).MakeInvisible()
    pan = states["saucepan"]
    h = liq.broth/AREA
    disc("stock", PAN_R-0.003, h, pan.pos+(0, 0, PAN_BOTTOM+h/2), BROTH)
    def surface(name, points, colors, faces=None):
        path = "/World/skimming_surface_"+name
        geom = UsdGeom.Mesh.Define(stage, path)
        geom.CreateSubdivisionSchemeAttr("none")
        geom.CreateDoubleSidedAttr(True)
        geom.GetPointsAttr().Set([Gf.Vec3f(*(float(v) for v in p)) for p in points])
        faces = [list(range(len(points)))] if faces is None and len(points)>2 else (faces or [])
        geom.GetFaceVertexCountsAttr().Set([len(f) for f in faces])
        geom.GetFaceVertexIndicesAttr().Set([i for f in faces for i in f])
        geom.CreateDisplayColorAttr([Gf.Vec3f(*(float(v) for v in c)) for c in colors])
        geom.GetDisplayColorPrimvar().SetInterpolation("vertex")

    xy = np.array([(0.,0.)]+[(float(rr*math.cos(a)), float(rr*math.sin(a)))
        for rr in np.linspace((PAN_R-.001)/8, PAN_R-.001, 8)
        for a in np.linspace(0,2*math.pi,48,endpoint=False)])
    nearest = np.argmin(np.sum((xy[:,None,:]-GRID[None,:,:])**2,axis=2),axis=1)
    heights = liq.fat[nearest]/CELL_AREA
    alpha = np.clip(heights/.003,0,1)
    colors = alpha[:,None]*FAT+(1-alpha[:,None])*BROTH
    points = pan.pos+np.c_[xy, PAN_BOTTOM+h+heights]
    faces = [[0,1+i,1+(i+1)%48] for i in range(48)]
    for ring in range(7):
        for i in range(48):
            a,b=1+ring*48+i,1+ring*48+(i+1)%48
            faces.append([a,b,b+48,a+48])
    surface("fat",points,colors,faces)
    for tag, s, f, b, radius, bottom in (
        ("ladle", states["ladle"], liq.ladle_fat, liq.ladle_broth, BOWL_R-0.004, -BOWL_D),
        ("waste", states["waste_cup"], liq.cup_fat, liq.cup_broth, CUP_R-0.003, 0.004)):
        total = f+b
        color = tuple((np.array(FAT)*f+np.array(BROTH)*b)/max(total, 1e-12))
        if tag == "ladle":
            rim, heights, cumulative, capacity = cavity(s)
            zz = level(total, heights, cumulative)
            r = parts._quat_to_matrix(s.quat_xyzw)
            center = transform(s, BOWL[None,:])[0]
            axes = r@np.diag((BOWL_R,BOWL_R,BOWL_D))
            normal = axes[2,:]
            nn = float(normal@normal)
            offset = zz-center[2]
            unit_center = normal*offset/nn
            rad = math.sqrt(max(0.,1-offset*offset/nn))
            u = np.cross(normal,(1.,0.,0.))
            if np.linalg.norm(u)<1e-8: u=np.cross(normal,(0.,1.,0.))
            u/=np.linalg.norm(u); v=np.cross(normal/np.linalg.norm(normal),u)
            polygon = [unit_center+rad*(math.cos(a)*u+math.sin(a)*v)
                       for a in np.linspace(0,2*math.pi,96,endpoint=False)]
            clipped=[]
            for a,b in zip(polygon,polygon[1:]+polygon[:1]):
                if (a[2]<=0)!=(b[2]<=0): clipped.append(a+(b-a)*a[2]/(a[2]-b[2]))
                if b[2]<=0: clipped.append(b)
            points = (np.array(clipped)@axes.T+center) if total>1e-8 and clipped else np.empty((0,3))
            surface(tag,points,[color]*len(points))
        else:
            hh = total/(math.pi*radius**2)
            disc(tag, radius, hh, s.pos+(0, 0, bottom+hh/2), color)


@parts.on_reset
def reset(sim):
    global _trial, _support_view, _support_force, _support_error, _support_detail
    _trial = Trial()
    _support_view=None;_support_force=None;_support_error=None;_support_detail=None
    # The installed Isaac ContactSensor enables this same task-process setting.
    # Raw tensor views do not enable contact reporting on their own.
    from isaaclab.app.settings_manager import get_settings_manager
    get_settings_manager().set_bool("/physics/disableContactProcessing",False)
    base=sim.env.unwrapped
    try:
        masses={name:parts._np(base.scene[name].root_physx_view.get_masses()).tolist()
                for name in ("saucepan","ladle","waste_cup")}
        print("[physical-check]",json.dumps(dict(mass_kg=masses,
              physics_dt=base.sim.get_physics_dt(),control_dt=base.step_dt)),flush=True)
    except (AttributeError,TypeError,ValueError) as error:
        print("[physical-check] unavailable:",type(error).__name__,flush=True)
    show_liquid(sim)


@parts.on_step
def step(sim):
    objects = sim.states()
    try:
        clouds, hands = hand_geometry(sim)
        shape_boxes = [b for spec in (PAN, LADLE, CUP) for b in boxes(spec, objects[spec.name])]
        clear = sampled_clear(clouds, shape_boxes)
        if clear and on_rest(objects["ladle"]):
            clear=arms_clear(sim,objects["ladle"])
    except (KeyError, AttributeError, ValueError, IndexError):
        clear, hands = None, None
    support=support_contact(sim)
    _trial.advance(sim.t, objects, clear, hands,support)
    show_liquid(sim)
    path = os.environ.get("TRACE_OUT")
    if path and round(sim.t*20) % 5 == 0:
        liq = _trial.liquid
        row = {k: v for k, v in vars(liq).items() if k != "fat"}
        row.update(t=float(sim.t),lot=lots.current().name,fat_left=float(liq.fat.sum()),clear=clear,
                   invalid=_trial.invalid, violation=_trial.violation, held=_trial.held)
        row.update(support_force=_support_force,support_error=_support_error,
                   support_detail=_support_detail,
                   liquid_invalid=liq.invalid,support=support)
        asset=sim.env.unwrapped.scene["left_robot"]
        measured=parts._np(asset.data.body_link_pose_w)[0,asset.body_names.index("gripper")]
        row["left_gripper_pose"]=measured.tolist()
        row["left_finger_q"]=parts._np(asset.data.joint_pos)[0,-2:].tolist()
        row["left_finger_qdot"]=parts._np(asset.data.joint_vel)[0,-2:].tolist()
        with pathlib.Path(path).with_name("rule.jsonl").open("a") as f:
            f.write(json.dumps(row)+"\n")


def score(objects):
    try:
        q = _trial.liquid
        if not (_trial.known and not _trial.invalid and not _trial.violation and not q.invalid
                and _trial.certificate == stamp(objects) and q.conserved()):
            return 0.
        if q.spill_fat+q.spill_broth > 2e-6+1e-12 or q.cup_broth > q.cup_fat/4+1e-12:
            return 0.
        if _trial.held < 2.-1e-9:
            return 0.
        fraction = q.cup_fat/(0.7*INITIAL_FAT)
        if fraction >= 1-1e-9 and _trial.held >= 2.-1e-9:
            return 1.
        return float(min(0.9, fraction)*0.9) if q.cup_fat >= 5e-6 else 0.
    except (KeyError, AttributeError, ValueError):
        return 0.


def success(objects):
    return score(objects) == 1.


def demo():
    """Open-loop side grip and visible-layout paths; no simulator feedback."""
    px,py,pz=lots.current().pos("ladle")
    grasp=np.array((-.055,.005,0.))
    grip=-.0082  # Actual20mm pad contact is q=-.00997; nominal17.7N total.
    side=tilt.rotation(0.,90.,-math.pi/2)
    off=grasp-BOWL
    def body(pitch=0.,roll=0.):
        p,r=math.radians(pitch),math.radians(roll)
        ry=np.array(((math.cos(p),0.,math.sin(p)),(0.,1.,0.),(-math.sin(p),0.,math.cos(p))))
        rx=np.array(((1.,0.,0.),(0.,math.cos(r),-math.sin(r)),(0.,math.sin(r),math.cos(r))))
        return ry@rx
    def w(t,xyz,fingers=grip,label="move",pitch=0.,roll=0.):
        return Waypoint(t,tuple(float(v) for v in xyz),fingers,label,
                        tilt._TiltedYaw(0.,body(pitch,roll)@side))
    def tcp(center,pitch):
        return np.asarray(center)+body(pitch)@off
    start=np.array((px,py,pz))+grasp
    beside=start+np.array((0.,.10,0.))
    lifted=start.copy();lifted[2]=TOP+.151
    air_center=np.array((PAN.pos[0]+.02,PAN.pos[1]-.02,TOP+.145))
    air=tcp(air_center,-15.)
    plan=[w(2.5,beside,FINGER_OPEN,"beside handle"),
          w(1.5,start,FINGER_OPEN,"enter overhang"),
          w(1.,start,grip,"grasp top and bottom"),
          w(1.4,lifted,grip,"lift",pitch=-15.)]
    tucked=lifted.copy();tucked[0]=air[0]
    plan.append(w(1.3,tucked,label="back corridor",pitch=-15.))
    for k in range(5):
        fat_depth=(INITIAL_FAT-k*24e-6)/AREA
        low=PAN.pos[2]+PAN_BOTTOM+INITIAL_BROTH/AREA+fat_depth-.0025
        center=np.array((PAN.pos[0],PAN.pos[1],low+BOWL_R*math.sin(math.radians(15.))))
        skim=tcp(center,15.)
        # Fixed open-loop height bias for the observed compliant side grip.
        # This is a development estimate, never runtime fluid/pose feedback.
        skim[2] += .010
        lower=skim.copy();lower[2]-=24e-6/AREA
        plan.append(w(1.6,air,label="above stock",pitch=-15.))
        stages=[]
        for j,pitch in enumerate((-7.5,0.,7.5,15.)):
            frac=(pitch+15)/30
            xy=air_center[:2]*(1-frac)+center[:2]*frac
            h=np.r_[xy,0.]+body(pitch)@off
            h[2]=skim[2] if pitch==15. else TOP+(.145 if pitch==7.5 else .140)
            stages.append((h,pitch))
            plan.append(w(.7,h,label="lower rim",pitch=pitch))
        plan += [w(4.,lower,label="follow falling surface",pitch=15.),
                 w(5.,lower,label="finish collecting",pitch=15.)]
        for h,pitch in reversed(stages[:-1]):
            plan.append(w(.7,h,label="lift bowl",pitch=pitch))
        plan.append(w(.7,air,label="clear pan",pitch=-15.))
        pour=tcp((CUP.pos[0],CUP.pos[1],TOP+.145),-15.)
        out=pour.copy();out[0]=air[0]
        plan += [w(1.2,out,label="back corridor",pitch=-15.),
                 w(1.6,pour,label="over waste",pitch=-15.)]
        waste_center=np.array((CUP.pos[0],CUP.pos[1],TOP+.145))
        # Follow the bowl centre while rolling instead of sweeping its outlet
        # sideways around a stationary wrist and outside the receiving mouth.
        for roll in (13.,26.,39.,52.,65.):
            plan.append(w(.6,waste_center+body(-15.,roll)@off,
                          label="pour",pitch=-15.,roll=roll))
        plan.append(w(.8,waste_center+body(-15.,65.)@off,
                      label="drain",pitch=-15.,roll=65.))
        for roll in (52.,39.,26.,13.,0.):
            plan.append(w(.6,waste_center+body(-15.,roll)@off,
                          label="level",pitch=-15.,roll=roll))
        back=pour.copy();back[0]=air[0]
        plan.append(w(1.2,back,label="back corridor",pitch=-15.))
    plan += [w(1.6,lifted,label="back above rest",pitch=-15.),
             w(1.4,start,label="lower to rest"),
             w(1.,start,FINGER_OPEN,"release"),
             w(1.5,beside,FINGER_OPEN,"retreat"),
             w(3.,beside,FINGER_OPEN,"clear")]
    return {"left":plan}
