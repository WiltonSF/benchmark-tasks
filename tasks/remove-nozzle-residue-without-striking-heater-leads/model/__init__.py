"""Declared bristle-contact and residue-removal rule (SI units; stated estimates, not measured constants).

Bristles: 27 bundles on the brush head, each a straight 14 mm ray along the head's bristle axis with a 1.75 mm
footprint radius and a 25 N/m spring. A bundle whose ray meets the heater block, nozzle or shroud is compressed by
the overlap; its spring force and friction are applied back to the free brush body.

Residue: one standard PETG film coupon bonded to the block underside (see the coupon constants below), scored as
0.5 mm area cells. A cell counts as removed once bundles pressing it with at least F_MIN have slid at least
SWEEP_MIN across it in any direction that does not head toward the lead exit (the block's back left face, where the
leads leave). Sliding toward the lead exit drags the loosened residue into it, so it earns nothing; so do stationary
pressure, proximity and time.
Strike: any bundle ray (within its radius) or the rigid brush touching a heater or thermistor lead latches a zero.
Wood touch: the rigid wood head coming within 0.1 mm of the block or nozzle latches a zero.
"""
import math
import numpy as np

BUNDLE_RADIUS = .00175
BRISTLE_LENGTH = .014
TRACTION = .35  # brass on brass/PETG sliding friction estimate, used for the reaction only
# Basis: about 110 brass wires per tuft, 0.08 mm diameter, 14 mm free length, E = 100 GPa; 3EI/L^3 per wire is
# 0.22 N/m, so about 25 N/m per tuft.
BUNDLE_K = 25.
F_MIN = BUNDLE_K * .0015  # 0.0375 N per tuft: bristles bent at least 1.5 mm
SWEEP_MIN = .004  # m of loaded sliding per cell
LEAD_DIR = np.array((math.cos(math.radians(35)), math.sin(math.radians(35)), 0.))  # normal of the lead-exit face
# The head lies along the brush x axis; bundle roots in the brush frame, bristles along brush +z.
ROOTS = np.array([(.075 + x, y, .006) for x in np.linspace(-.014, .014, 9) for y in (-.004, 0, .004)])


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))


BLOCK_R = rotz(math.radians(35.))
# Standard residue: one printed single-layer PETG coupon, COUPON_THICKNESS thick, bonded to the block underside. In the block
# frame (x' toward the lead-exit face, y' along it) it covers the underside inset COUPON_INSET from each edge, leaves a
# LEAD_STRIP bare strip along the lead-exit face and a NOZZLE_CLEAR hole around the nozzle shoulder. It is scored as
# equal-area cells on a CELL grid; removed() is the cleared fraction of the coupon area.
BLOCK_SIZE = (.013, .020, .012)
COUPON_THICKNESS = .0002
COUPON_INSET = .0005
LEAD_STRIP = .003
NOZZLE_CLEAR = .0045
CELL = .0005
_xs = np.arange(-BLOCK_SIZE[0] / 2 + COUPON_INSET + CELL / 2, BLOCK_SIZE[0] / 2 - LEAD_STRIP + 1e-9, CELL)
_ys = np.arange(-BLOCK_SIZE[1] / 2 + COUPON_INSET + CELL / 2, BLOCK_SIZE[1] / 2 - COUPON_INSET + 1e-9, CELL)
_local = np.array([(x, y, -BLOCK_SIZE[2] / 2) for x in _xs for y in _ys if math.hypot(x, y) >= NOZZLE_CLEAR])
LOCAL_CELLS = _local
CELLS = _local @ BLOCK_R.T
N = len(CELLS)
NORMALS = np.tile((0., 0., -1.), (N, 1))
COUPON_AREA_MM2 = N * (CELL * 1000) ** 2


def ray_box(roots, direction, centre, rotation, size):
    """First intersection along each finite bristle with a box, and the box's outward normal there."""
    q = (roots - centre) @ rotation
    v = np.asarray(direction) @ rotation
    lo = np.full(len(q), -np.inf); hi = np.full(len(q), np.inf)
    axis = np.zeros(len(q), int); sign = np.zeros(len(q)); possible = np.ones(len(q), bool)
    for k in range(3):
        if abs(v[k]) < 1e-12:
            possible &= np.abs(q[:, k]) <= size[k] / 2
        else:
            a = (-size[k] / 2 - q[:, k]) / v[k]; b = (size[k] / 2 - q[:, k]) / v[k]
            entry = np.minimum(a, b); take = entry > lo
            axis[take] = k; sign[take] = -np.sign(v[k])
            lo = np.maximum(lo, entry); hi = np.minimum(hi, np.maximum(a, b))
    valid = possible & (lo >= 0) & (lo <= BRISTLE_LENGTH) & (hi >= lo)
    normals = np.zeros((len(q), 3)); normals[np.arange(len(q)), axis] = sign
    return np.where(valid, lo, np.inf), normals @ rotation.T


def ray_cone(roots, d, centre):
    """Nozzle frustum z = -12..-7 mm, r = 0.6..4 mm below the block centre, plus its tip face."""
    q = roots - centre; d = np.asarray(d); s = .0034 / .005
    k = .0006 + s * (q[:, 2] + .012); kd = s * d[2]
    A = d[0] ** 2 + d[1] ** 2 - kd ** 2; B = 2 * (q[:, 0] * d[0] + q[:, 1] * d[1] - k * kd); C = q[:, 0] ** 2 + q[:, 1] ** 2 - k * k
    disc = B * B - 4 * A * C; roots_t = []
    if abs(A) > 1e-12:
        for sign in (-1, 1):
            roots_t.append((-B + sign * np.sqrt(np.maximum(disc, 0))) / (2 * A))
    else:
        roots_t.append(-C / np.where(abs(B) > 1e-12, B, np.inf))
    best = np.full(len(q), np.inf); normal = np.zeros_like(q)
    for t in roots_t:
        point = q + t[:, None] * d
        good = (disc >= 0) & (t >= 0) & (t <= BRISTLE_LENGTH) & (point[:, 2] >= -.012) & (point[:, 2] <= -.007)
        take = good & (t < best); best[take] = t[take]
        n = np.c_[point[:, 0], point[:, 1], -s * (.0006 + s * (point[:, 2] + .012))]
        n /= np.maximum(np.linalg.norm(n, axis=1)[:, None], 1e-12); normal[take] = n[take]
    if abs(d[2]) > 1e-12:
        t = (-.012 - q[:, 2]) / d[2]; point = q + t[:, None] * d
        good = (t >= 0) & (t <= BRISTLE_LENGTH) & (np.linalg.norm(point[:, :2], axis=1) <= .0006) & (t < best)
        best[good] = t[good]; normal[good] = (0, 0, -1)
    return best, normal


def obb_depth(ca, ra, sa, cb, rb, sb):
    """Overlap depth of two oriented boxes (0 when apart), by separating axes."""
    axes = [*ra.T, *rb.T, *[np.cross(a, b) for a in ra.T for b in rb.T]]; depth = math.inf
    for a in axes:
        norm = np.linalg.norm(a)
        if norm < 1e-9:
            continue
        a = a / norm
        gap = float(abs(a @ ra) @ (np.asarray(sa) / 2) + abs(a @ rb) @ (np.asarray(sb) / 2) - abs(a @ (ca - cb)))
        if gap <= 0:
            return 0.
        depth = min(depth, gap)
    return depth


TRAY_HALF = np.array((.059, .039))


class Residue:
    def __init__(self, centre):
        self.centre = np.asarray(centre, float)
        self.attached = np.ones(N, bool); self.sweep = np.zeros(N)
        self.pos = CELLS + self.centre + NORMALS * COUPON_THICKNESS / 2; self.vel = np.zeros((N, 3))
        self.state = np.zeros(N, int)  # 0 bonded, 1 falling, 2 in tray, 4 on the bed
        self.tray_local = {}
        self.time = 0.; self.prev_contacts = None; self.prev_surface = None; self.prev_normals = None
        self.peak = 0.; self.force = 0.; self.invalid = False; self.wire_hit = False; self.damage = 0.; self.hit_count = 0
        self.last_lengths = np.full(len(ROOTS), BRISTLE_LENGTH); self.reaction = np.zeros(3); self.torque = np.zeros(3)
        self.max_bundle = 0.

    def advance(self, dt, brush_pos, brush_rot, boxes, pan_pos, pan_rot, bed_z, wire_segments):
        if not 0 < dt <= .051 or not np.isfinite(np.r_[brush_pos, brush_rot.ravel(), pan_pos, pan_rot.ravel()]).all():
            self.invalid = True; return
        roots = ROOTS @ brush_rot.T + brush_pos; direction = brush_rot[:, 2]
        best = np.full(len(roots), np.inf); normals = np.zeros_like(roots)
        kind = np.zeros(len(roots), int); surface = np.full(len(roots), -1, int)
        for index, (centre, rotation, size, tag) in enumerate(boxes):
            tt, nn = ray_box(roots, direction, np.asarray(centre), rotation, np.asarray(size))
            take = tt < best; best[take] = tt[take]; normals[take] = nn[take]; kind[take] = tag; surface[take] = index
        tt, nn = ray_cone(roots, direction, self.centre)
        take = tt < best; best[take] = tt[take]; normals[take] = nn[take]; kind[take] = 1; surface[take] = len(boxes)
        hit = np.isfinite(best); points = roots + direction * np.where(hit, best, 0.)[:, None]
        self.last_lengths = np.minimum(best, BRISTLE_LENGTH)
        load = BUNDLE_K * np.where(hit, BRISTLE_LENGTH - best, 0.)
        self.force = float(load.sum()); self.peak = max(self.peak, self.force); self.hit_count = int(hit.sum())
        self.max_bundle = max(self.max_bundle, float(load.max()))
        self.reaction = (normals * load[:, None]).sum(0)
        self.torque = np.cross(points - brush_pos, normals * load[:, None]).sum(0)
        # Wood touch: the rigid wood head (not the soft bristles) within 0.1 mm of the block or nozzle.
        head = brush_pos + brush_rot @ np.array((.075, 0., 0.))
        if np.linalg.norm(head - self.centre) < .040:
            wood = np.array((.0342, .0122, .0122))  # head box grown by 0.1 mm on every side
            hard = obb_depth(head, brush_rot, wood, self.centre, BLOCK_R, np.array((.013, .020, .012)))
            hard = max(hard, obb_depth(head, brush_rot, wood, self.centre + np.array((0, 0, -.0095)), np.eye(3), np.array((.008, .008, .005))))
            self.damage = max(self.damage, hard)
        # Any bristle bundle (radius 1.75 mm plus 0.125 mm sampling allowance) touching a lead latches a strike.
        length = np.minimum(best, BRISTLE_LENGTH)
        for a, b, r in wire_segments:
            a, b = np.asarray(a), np.asarray(b); n = max(2, math.ceil(np.linalg.norm(b - a) / .00025) + 1)
            wp = a + np.linspace(0., 1., n)[:, None] * (b - a)
            v = wp[:, None, :] - roots[None, :, :]; axial = v @ direction
            radial = np.linalg.norm(v - axial[:, :, None] * direction, axis=2)
            if np.any((radial <= BUNDLE_RADIUS + r + .000125) & (axial >= -r) & (axial <= length[None, :] + r)):
                self.wire_hit = True
        delta = np.zeros_like(points); continuous = np.zeros(len(roots), bool)
        if self.prev_contacts is not None:
            continuous = hit & (surface == self.prev_surface) & (np.sum(normals * self.prev_normals, axis=1) > .75)
            delta[continuous] = points[continuous] - self.prev_contacts[continuous]
        for j in np.flatnonzero(hit):
            # Sliding is motion of the contact point over the surface, not motion of the bristle root.
            tangent = delta[j] - normals[j] * (delta[j] @ normals[j]); ds = float(np.linalg.norm(tangent))
            if ds > 1e-12:
                friction = -TRACTION * load[j] * tangent / ds
                self.reaction += friction; self.torque += np.cross(points[j] - brush_pos, friction)
            if kind[j] != 1 or not continuous[j] or load[j] < F_MIN - 1e-12 or ds > .003:
                continue
            if float(tangent @ LEAD_DIR) > 0:
                continue  # heading toward the lead exit
            dist = np.linalg.norm((CELLS + self.centre) - points[j], axis=1)
            under = (dist <= BUNDLE_RADIUS) & (NORMALS @ normals[j] > .25) & self.attached
            self.sweep[under] += ds
            done = under & (self.sweep >= SWEEP_MIN - 1e-12)
            self.attached[done] = False; self.state[done] = 1; self.vel[done] = NORMALS[done] * .02
        # Removed residue is drawn falling under gravity into the catch tray or onto the bed; it is not scored.
        for i in np.flatnonzero(self.state == 1):
            self.vel[i, 2] -= 9.81 * dt; self.pos[i] = self.pos[i] + self.vel[i] * dt
            local = (self.pos[i] - pan_pos) @ pan_rot
            if np.all(np.abs(local[:2]) <= TRAY_HALF) and local[2] <= .001 + CELL / 2:
                local[2] = .001 + CELL / 2; self.tray_local[i] = local; self.state[i] = 2; self.vel[i] = 0.
            elif self.pos[i, 2] <= bed_z + CELL / 2:
                self.pos[i, 2] = bed_z + CELL / 2; self.state[i] = 4; self.vel[i] = 0.
        for i, local in self.tray_local.items():
            self.pos[i] = pan_pos + pan_rot @ local
        self.prev_contacts = points.copy(); self.prev_surface = surface.copy(); self.prev_normals = normals.copy(); self.time += dt

    def removed(self):
        return int((~self.attached).sum())

    def fraction(self):
        return self.removed() / N

    def endpoint(self, known, clear, still, released, hold):
        if not known or self.invalid or self.wire_hit or self.damage > 0. or not clear or not still or not released or hold < 2. - 1e-9:
            return 0.
        fraction = self.fraction()
        return 1. if fraction >= .95 - 1e-9 else .75 if fraction >= .80 - 1e-9 else .5 if fraction >= .5 - 1e-9 else .25 if fraction >= .25 - 1e-9 else 0.
