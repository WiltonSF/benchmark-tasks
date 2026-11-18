"""Conservative face-sampled release geometry. No command-based clearance."""
import math,itertools
from functools import lru_cache
import numpy as np
import parts
CLEAR=.005
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


def array(v):
    if hasattr(v,"torch"):v=v.torch
    if hasattr(v,"detach"):v=v.detach().cpu().numpy()
    elif hasattr(v,"numpy"):v=v.numpy()
    return np.asarray(v,float)


def boxes(spec,state):
    poses={"base":(state.pos,parts._quat_to_matrix(state.quat_xyzw))}
    poses.update({name:(p,parts._quat_to_matrix(q)) for name,(p,q) in state.links.items()})
    return spec.world_boxes(poses)

