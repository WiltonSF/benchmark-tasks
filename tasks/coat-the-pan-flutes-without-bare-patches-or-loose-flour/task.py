"""A kitchen counter (the station table) with an aluminium half-sheet baking pan on it. On the sheet pan stands an
empty, clean 6-cup fluted tube cake pan (a Bundt pan) with two small cast handles; its inside is silver non-stick.
Beside the cake pan, on the sheet pan, stands a small white ramekin of solid white vegetable shortening. On one side
of the sheet pan a pastry brush lies on a ceramic spoon rest; on the other side stands a stainless flour dredger,
filled with flour. Shortening on the cake pan shows as a yellowish sheen, stuck flour as a cream-white coat and loose
flour as bright white. Which side the brush, the ramekin and the dredger are on changes between trials.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
D = math.radians

# ======================================================================================================================
# The inside of the cake pan and the declared shortening and flour layers on it (rules: README.md)
# ======================================================================================================================


# -- the profile (one half-section of the inside, the ridge surface between two flutes) -------------------------------
# Nordic Ware Original Bundt pan, 6-cup (51202): 9 1/4 x 8 1/8 x 3 1/4 in (235 mm across its two handles, 206 mm
# across the body, 83 mm tall; retail listing). The inside shape below is an estimate from the maker's product
# photos and the 6-cup capacity (this profile holds 1.50 L to the brim; 6 US cups are 1.42 L).
RIM_Z = 0.083
WALL_T = 0.0035  # cast wall thickness (estimate)
FLUTES = 12
FLUTE_A = 0.006  # flute depth at the outer wall and floor; less toward the tube (scaled with radius below 50 mm)
FLUTE_PHASE = math.pi / 2  # a ridge (no flute depth) lies at the handles' azimuths (+-90 degrees)
P_RIM = (0.0915, RIM_Z)  # ridge radius at the rim's inner edge
P_WALL = (0.0775, 0.0365)  # bottom of the straight outer wall (16.8 degrees off vertical)
ARC_C, ARC_R = (0.0505, 0.0365), 0.027  # rounded floor: lowest ridge point 9.5 mm up, valley 3.5 mm up
P_TUBE = (0.0235, 0.0365)
P_TOP = (0.016, RIM_Z)  # tube's outer radius at its top (9.2 degrees taper); 12.5 mm bore through it
BORE_R = P_TOP[0] - WALL_T
BODY_R = 0.1015  # outside radius of the body at the rim
HANDLE_R = 0.1175  # handle ends, 235 mm across

N_TH, N_S = 96, 52  # cells around and along the profile
FINE = 0.0005  # profile sampling step


def _profile():
    """(r, z) points every 0.5 mm from the rim's inner edge down the outer wall, round the floor and up the tube."""
    pts = []

    def line(a, b):
        n = max(2, int(math.ceil(math.dist(a, b) / FINE)))
        for k in range(n):
            pts.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))

    line(P_RIM, P_WALL)
    n = int(math.ceil(math.pi * ARC_R / FINE))
    for k in range(n):
        a = -math.pi * k / n  # from angle 0 (outer side) through -90 (bottom) to -180 (tube side)
        pts.append((ARC_C[0] + ARC_R * math.cos(a), ARC_C[1] + ARC_R * math.sin(a)))
    line(P_TUBE, P_TOP)
    pts.append(P_TOP)
    p = np.asarray(pts, float)
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    t = np.gradient(p, s, axis=0)
    t /= np.linalg.norm(t, axis=1)[:, None]
    n_out = np.stack([-t[:, 1], t[:, 0]], axis=1)  # out of the cavity, into the metal
    return s, p, n_out


S_F, P_F, NOUT_F = _profile()
S_TOTAL = float(S_F[-1])
S_WALL = math.dist(P_RIM, P_WALL)
S_ARC = math.pi * ARC_R
SECTION_NAMES = ("outer wall", "floor", "tube")


def amp(r):
    """Flute depth at profile radius r: full at the outer wall and floor, scaled down with radius below 50 mm."""
    return FLUTE_A * np.minimum(1.0, np.asarray(r, float) / 0.050)


def flute(theta):
    """0 on a ridge, 1 at the bottom of a flute."""
    return 0.5 * (1.0 - np.cos(FLUTES * (np.asarray(theta, float) - FLUTE_PHASE)))


def surface(theta, s):
    """Pan-frame points of the inside surface at azimuth theta (rad) and profile length s (m)."""
    theta, s = np.broadcast_arrays(np.asarray(theta, float), np.asarray(s, float))
    r = np.interp(s, S_F, P_F[:, 0])
    z = np.interp(s, S_F, P_F[:, 1])
    nr = np.interp(s, S_F, NOUT_F[:, 0])
    nz = np.interp(s, S_F, NOUT_F[:, 1])
    d = amp(r) * flute(theta)
    rr, zz = r + d * nr, z + d * nz
    return np.stack([rr * np.cos(theta), rr * np.sin(theta), zz], axis=-1)


# -- cells ------------------------------------------------------------------------------------------------------------
D_TH = 2 * math.pi / N_TH
D_S = S_TOTAL / N_S
TH_C = (np.arange(N_TH) + 0.5) * D_TH
S_C = (np.arange(N_S) + 0.5) * D_S
TH_GRID, S_GRID = np.meshgrid(TH_C, S_C, indexing="ij")  # (N_TH, N_S)
CELL_TH = TH_GRID.ravel()
CELL_S = S_GRID.ravel()
N = N_TH * N_S
X = surface(CELL_TH, CELL_S)  # (N, 3) cell centres
_h = 1e-5
_ds = (surface(CELL_TH, CELL_S + _h) - surface(CELL_TH, CELL_S - _h)) / (2 * _h)
_dt = (surface(CELL_TH + _h, CELL_S) - surface(CELL_TH - _h, CELL_S)) / (2 * _h)
_cr = np.cross(_ds, _dt)
AREA_M2 = np.linalg.norm(_cr, axis=1) * D_S * D_TH
AREA = AREA_M2 * 1e6  # mm^2
_n = _cr / np.linalg.norm(_cr, axis=1)[:, None]
_base_out = np.stack([np.interp(CELL_S, S_F, NOUT_F[:, 0]) * np.cos(CELL_TH),
                      np.interp(CELL_S, S_F, NOUT_F[:, 0]) * np.sin(CELL_TH),
                      np.interp(CELL_S, S_F, NOUT_F[:, 1])], axis=1)
NOUT = np.where((np.sum(_n * _base_out, axis=1) < 0)[:, None], -_n, _n)  # into the metal
NIN = -NOUT
SECTION = np.where(CELL_S < S_WALL, 0, np.where(CELL_S < S_WALL + S_ARC, 1, 2))
FLUTE_OF = (np.floor(((CELL_TH - FLUTE_PHASE) % (2 * math.pi)) / (2 * math.pi / FLUTES)).astype(int)) % FLUTES
ZONE = FLUTE_OF * 3 + SECTION  # 36 zones: 12 flutes (ridge to ridge) x outer wall, floor, tube
TOTAL_AREA = float(AREA.sum())

# corner grid for drawing (the seam is repeated so that the mesh closes)
_thc = np.arange(N_TH + 1) * D_TH
_sc = np.arange(N_S + 1) * D_S
CORNERS = surface(*np.meshgrid(_thc, _sc, indexing="ij"))  # (N_TH+1, N_S+1, 3)


def cell(i, j):
    return i * N_S + j


# 8 neighbours on the grid; off the rim edge and off the tube's top edge are exits (rim exit N + i, bore exit N+N_TH+i)
RIM_EXIT0 = N
BORE_EXIT0 = N + N_TH
NB = np.full((N, 8), -1, int)
for _i in range(N_TH):
    for _j in range(N_S):
        k = 0
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                ii, jj = (_i + di) % N_TH, _j + dj
                if jj < 0:
                    NB[cell(_i, _j), k] = RIM_EXIT0 + ii
                elif jj >= N_S:
                    NB[cell(_i, _j), k] = BORE_EXIT0 + ii
                else:
                    NB[cell(_i, _j), k] = cell(ii, jj)
                k += 1
RIM_EDGE = surface(TH_C, np.zeros(N_TH))
RIM_EDGE[:, 2] += 0.001
BORE_EDGE = surface(TH_C, np.full(N_TH, S_TOTAL))
BORE_EDGE[:, :2] *= (BORE_R / P_TOP[0])
NODES = np.concatenate([X, RIM_EDGE, BORE_EDGE])  # cells, then exits

# What a vertical line straight down onto the upright pan meets first: for each column of cells and each 1 mm of
# radius, the cell it lands on (-1: no inside surface there).
RHO_BINS = 120
LAND = np.full((N_TH, RHO_BINS), -1, int)
_sf = np.arange(0.0, S_TOTAL, 0.00025)
for _i in range(N_TH):
    pts = surface(np.full(len(_sf), TH_C[_i]), _sf)
    rho, zz = np.hypot(pts[:, 0], pts[:, 1]), pts[:, 2]
    for b in range(RHO_BINS):
        q = (b + 0.5) * 0.001
        cross = np.flatnonzero((rho[:-1] - q) * (rho[1:] - q) <= 0)
        if len(cross):
            best = cross[np.argmax(zz[cross])]
            LAND[_i, b] = cell(_i, min(N_S - 1, int(_sf[best] / D_S)))
RIM_RHO = np.hypot(RIM_EDGE[:, 0], RIM_EDGE[:, 1])


def column(theta):
    return (np.floor((np.asarray(theta) % (2 * math.pi)) / D_TH).astype(int)) % N_TH


# -- the declared rules: numbers ---------------------------------------------------------------------------------------
G_MIN = 0.02  # mm of shortening for a cell to count as greased
LUMP = 2.0  # mm: a deposit (shortening plus stuck flour) thicker than this is a lump
SHORTENING_RATE = 0.08  # mm/s laid on a cell while loaded bristles press on it
G_FILM = 0.03  # mm: loaded bristles leave at least this film on any cell they touch
SHORTENING_DENSITY = 0.9e-3  # g/mm^3
BRUSH_LOAD_MAX = 3.0  # g the bristles hold
LOAD_RATE = 2.0  # g/s taken up while bristles are in the shortening
PRESS_MIN, PRESS_MAX = 0.0, 0.015  # m: a bristle point counts from touching the surface to 15 mm pressed in
SPLAY = 0.004  # m: bristles spread this far around each pressing point
F_MIN = 0.005  # mg/mm^2 of stuck flour for a greased cell to count as floured (0.5 mg/cm^2)
F_CAP = 0.020  # mg/mm^2 a greased cell can hold stuck (2 mg/cm^2)
FLOUR_DENSITY = 0.5  # mg/mm^3 of a stuck flour layer
REPOSE = math.radians(45.0)  # loose flour stays on a surface sloped less than this
DREDGE_RATE = 800.0  # mg/s of flour out of the dredger while it is turned over (more than 100 degrees)
DREDGE_TILT = math.radians(100.0)
PLUME_R = 0.040  # m: flour from the dredger lands within this radius of the point below its lid
_PLUME = np.array([(r * math.cos(a), r * math.sin(a)) for r, n in ((0.0, 1), (0.014, 8), (0.027, 14), (0.039, 20))
                   for a in np.linspace(0, 2 * math.pi, n, endpoint=False)])
_PLUME_W = np.array([r ** 0 * (1.0) for r in np.hypot(_PLUME[:, 0], _PLUME[:, 1])])
_PLUME_W /= _PLUME_W.sum()


class Field:
    """The layers on the pan's inside for one trial."""

    def __init__(self):
        self.g = np.zeros(N)  # shortening, mm
        self.f = np.zeros(N)  # stuck flour, mg/mm^2
        self.loose = np.zeros(N)  # loose flour, mg per cell
        self.brush_load = 0.0  # g of shortening on the bristles
        self.dredger = 25000.0  # mg of flour in the dredger
        self.caught = 0.0  # mg of flour landed on the sheet pan
        self.spilled = 0.0  # mg landed on the counter
        self.used = 0.0  # g of shortening laid on the pan

    # -- shortening ------------------------------------------------------------------------------------------------
    def load(self, bristles_in_fat_fraction: float, dt: float) -> None:
        if bristles_in_fat_fraction > 0:
            self.brush_load = min(BRUSH_LOAD_MAX, self.brush_load + LOAD_RATE * dt * bristles_in_fat_fraction)

    def brush(self, pts, roots, dt: float) -> int:
        """Bristle points ``pts`` and their roots at the ferrule ``roots`` (pan frame, m). Returns cells touched."""
        if self.brush_load <= 1e-6:
            return 0
        ok = above_inside(roots)
        if not ok.any():
            return 0
        pts = pts[ok]
        d2 = ((pts[:, None, :] - X[None, :, :]) ** 2).sum(axis=2)
        near = np.argmin(d2, axis=1)
        depth = np.sum((pts - X[near]) * NOUT[near], axis=1)
        if not (depth < 0.0).any():
            return 0  # no part of the bristles is in the pan's opening: they are on its outside
        touch = (depth >= PRESS_MIN - 1e-9) & (depth <= PRESS_MAX + 1e-9)
        if not touch.any():
            return 0
        foot = pts[touch] - depth[touch, None] * NOUT[near[touch]]
        hit = (((foot[:, None, :] - X[None, :, :]) ** 2).sum(axis=2) <= SPLAY ** 2).any(axis=0)
        hit[near[touch]] = True
        ids = np.flatnonzero(hit)
        target = np.maximum(self.g[ids] + SHORTENING_RATE * dt, G_FILM)  # mm
        want = (target - self.g[ids]) * AREA[ids] * SHORTENING_DENSITY  # g
        k = min(1.0, self.brush_load / max(want.sum(), 1e-12))
        self.g[ids] += (target - self.g[ids]) * k
        self.brush_load -= float(want.sum()) * k
        self.used += float(want.sum()) * k
        return len(ids)

    # -- flour -----------------------------------------------------------------------------------------------------
    def dust(self, lid_world, pan_pos, pan_rot, sheet_inside, dt: float) -> float:
        """Flour leaving the turned-over dredger's lid falls straight down."""
        q = min(DREDGE_RATE * dt, self.dredger)
        if q <= 0:
            return 0.0
        self.dredger -= q
        upright = pan_rot[2, 2] >= math.cos(math.radians(15.0))
        local = pan_rot.T @ (np.asarray(lid_world, float) - pan_pos)
        landed = 0.0
        if upright and local[2] > RIM_Z:
            # an even fall of q over a disc of radius PLUME_R below the lid; each cell of the inside under it takes
            # the share that falls on its area as seen from above
            under = ((X[:, 0] - local[0]) ** 2 + (X[:, 1] - local[1]) ** 2) <= PLUME_R ** 2
            share = q / (math.pi * PLUME_R ** 2 * 1e6) * AREA * np.maximum(NIN[:, 2], 0.0) * under
            self.loose += share
            landed = float(share.sum())
        rest = q - landed  # through the bore, off the rim and handles, or past the cake pan
        if sheet_inside(lid_world[0], lid_world[1]):
            self.caught += rest
        else:
            self.spilled += rest
        return q

    def settle(self, pan_pos, pan_rot, sheet_inside) -> None:
        """One control step of the loose flour: it sticks to greased cells up to their hold, and moves off any cell
        sloped 45 degrees or more, to the lowest neighbouring cell or off the rim or down the tube's bore."""
        self.stick()
        if not (self.loose > 1e-9).any():
            return
        world = NODES @ pan_rot.T + pan_pos
        nz = NIN @ pan_rot[2]  # world-up part of each cell's inward normal
        steep = (nz < math.cos(REPOSE)) & (self.loose > 1e-9)
        if not steep.any():
            return
        ids = np.flatnonzero(steep)
        nbz = world[NB[ids], 2]
        k = np.argmin(nbz, axis=1)
        dest = NB[ids, k]
        lower = nbz[np.arange(len(ids)), k] < world[ids, 2]
        opening_down = pan_rot[2, 2] < 0.0
        falls = (nz[ids] < 0.0) & opening_down
        move = lower | falls
        ids, dest, falls = ids[move], dest[move], falls[move]
        m = self.loose[ids].copy()
        self.loose[ids] = 0.0
        out = falls | (dest >= N)
        stay = ~out
        np.add.at(self.loose, dest[stay], m[stay])
        if out.any():
            at = np.where(falls[out][:, None], world[ids[out]], world[dest[out]])
            for (x, y), q in zip(at[:, :2], m[out]):
                if sheet_inside(x, y):
                    self.caught += q
                else:
                    self.spilled += q
        self.stick()

    def stick(self) -> None:
        greased = self.g >= G_MIN - 1e-12
        room = np.where(greased, np.maximum(F_CAP - self.f, 0.0) * AREA, 0.0)
        take = np.minimum(self.loose, room)
        self.f += np.where(AREA > 0, take / AREA, 0.0)
        self.loose -= take

    # -- what a person reads at the end --------------------------------------------------------------------------
    def covered(self) -> np.ndarray:
        return (self.g >= G_MIN - 1e-12) & (self.f >= F_MIN - 1e-12)

    def height(self) -> np.ndarray:
        """mm of deposit on each cell: shortening plus the stuck flour layer."""
        return self.g + self.f / FLOUR_DENSITY

    def bare_spots(self, min_len: float = 0.010):
        """Connected uncovered patches whose longest straight extent is ``min_len`` or more: list of cell arrays."""
        bare = ~self.covered()
        seen = np.zeros(N, bool)
        out = []
        for start in np.flatnonzero(bare):
            if seen[start]:
                continue
            stack, comp = [start], []
            seen[start] = True
            while stack:
                c = stack.pop()
                comp.append(c)
                for d in NB[c]:
                    if 0 <= d < N and bare[d] and not seen[d]:
                        seen[d] = True
                        stack.append(d)
            comp = np.asarray(comp)
            if extent(comp) >= min_len - 1e-9:
                out.append(comp)
        return out

    def zones_done(self) -> int:
        bad = np.zeros(36, bool)
        for comp in self.bare_spots():
            bad[np.unique(ZONE[comp])] = True
        return int(36 - bad.sum())


def extent(comp) -> float:
    """Longest distance across a patch of cells (cell centres, plus one cell's size)."""
    p = X[comp]
    if len(p) > 400:
        lo, hi = p.min(axis=0), p.max(axis=0)
        return float(np.linalg.norm(hi - lo)) + D_S
    d = np.sqrt(((p[:, None, :] - p[None, :, :]) ** 2).sum(axis=2)).max() if len(p) > 1 else 0.0
    return float(d) + D_S


def above_inside(pts) -> np.ndarray:
    """Pan-frame points from which a bristle can reach the inside: over the pan's opening (inside the rim, outside the
    tube) and above its inside surface, or anywhere above the rim's top out to 15 mm past its outer edge (bristles bent over
    the rim's or the tube's top edge). A bristle rooted anywhere else (beside or under the pan) is brushing its outside."""
    pts = np.atleast_2d(pts)
    rho = np.hypot(pts[:, 0], pts[:, 1])
    th = np.arctan2(pts[:, 1], pts[:, 0])
    i = column(th)
    b = np.clip((rho / 0.001).astype(int), 0, RHO_BINS - 1)
    c = LAND[i, b]
    zs = np.where(c >= 0, X[np.maximum(c, 0), 2], np.inf)
    over = (rho > P_TOP[0]) & (rho < RIM_RHO[i]) & (c >= 0) & (pts[:, 2] > zs)
    over_rim = (pts[:, 2] > RIM_Z) & (rho < BODY_R + 0.015)
    return over | over_rim


# ======================================================================================================================
# The objects. Positions are world metres: x away from the arms, y to the robot's left, z up; TOP is the counter.
# ======================================================================================================================
ALU = (0.74, 0.75, 0.76)  # cast aluminium, the cake pan's outside
SHEET_ALU = (0.82, 0.83, 0.84)
GLASS = (0.80, 0.87, 0.89)
SHORTENING_WHITE = (0.97, 0.96, 0.90)
STEEL = (0.68, 0.69, 0.71)
BEECH = (0.82, 0.66, 0.46)
BRISTLE = (0.90, 0.82, 0.64)
DARK = (0.22, 0.22, 0.24)
BARE = (0.55, 0.56, 0.58)  # the silver non-stick inside
GREASED = (0.70, 0.66, 0.50)
FLOURED = (0.92, 0.90, 0.84)
LOOSE = (1.0, 1.0, 1.0)
LUMPY = (0.95, 0.84, 0.52)


def _qz(a: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _ring(r_out, wall, z0, z1, n, color, phase=0.0):
    """A round wall as ``n`` flat boxes (outer faces ``r_out`` from the axis), from z0 to z1."""
    out = []
    width = 2 * r_out * math.tan(math.pi / n) + 0.0004
    for k in range(n):
        a = phase + 2 * math.pi * (k + 0.5) / n
        rm = r_out - wall / 2
        out.append(parts.box((wall, width, z1 - z0), at=(rm * math.cos(a), rm * math.sin(a), (z0 + z1) / 2),
                             color=color, quat=_qz(a)))
    return out


def _box_visual(name, size, at, color):
    sx, sy, sz = (s / 2 for s in size)
    pts = tuple((at[0] + a * sx, at[1] + b * sy, at[2] + c * sz) for a in (-1, 1) for b in (-1, 1) for c in (-1, 1))
    quads = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))
    tris = tuple(t for q in quads for t in ((q[0], q[1], q[2]), (q[0], q[2], q[3])))
    return parts.Visual(name, pts, tris, color)


# -- the cake pan: Nordic Ware Original Bundt pan, 6-cup (51202), cast aluminium, silver non-stick inside -------------------
# Collision: the wall as 24 x 9 convex slabs that follow the flutes, set 2 mm back from the inside surface so that
# the drawn inside (the coat mesh, below) is always in front of them; the rim, the tube's top and two handles.
PAN_MASS = 0.55  # kg, estimate: shipping weight 1.37 lb (0.62 kg) with its box
HANDLE_LEN, HANDLE_W, HANDLE_T = HANDLE_R - BODY_R, 0.050, 0.008  # each handle 16 mm out, 50 mm wide, 8 mm thick
HANDLE_ZC = RIM_Z - HANDLE_T / 2


def _pan_shapes():
    out = []
    ths = FLUTE_PHASE + np.arange(25) * (2 * math.pi / 24)
    ss = np.linspace(0.0, S_TOTAL, 10)
    for a, b in zip(ths[:-1], ths[1:]):
        for s0, s1 in zip(ss[:-1], ss[1:]):
            th = np.array([a, b, a, b, (a + b) / 2])
            sv = np.array([s0, s0, s1, s1, (s0 + s1) / 2])
            p = surface(th, sv)
            n = np.stack([np.interp(sv, S_F, NOUT_F[:, 0]) * np.cos(th), np.interp(sv, S_F, NOUT_F[:, 0]) * np.sin(th),
                          np.interp(sv, S_F, NOUT_F[:, 1])], axis=1)
            pts = np.concatenate([p + 0.002 * n, p + WALL_T * n])
            out.append(parts.hull(pts.tolist(), [tuple(range(len(pts)))], color=ALU))
    out += _ring(BODY_R, BODY_R - 0.0975, RIM_Z - 0.003, RIM_Z, 24, ALU)  # flat top of the rim
    out += _ring(P_TOP[0], P_TOP[0] - BORE_R, RIM_Z - 0.003, RIM_Z, 12, ALU)  # top of the tube
    for sgn in (1, -1):
        out.append(parts.box((HANDLE_W, HANDLE_LEN + 0.002, HANDLE_T),
                             at=(0.0, sgn * (BODY_R + HANDLE_LEN / 2 - 0.001), HANDLE_ZC), color=ALU))
    return out


# -- the sheet pan: Nordic Ware Naturals half sheet (43100), aluminium, 17.9 x 12.9 x 1 in (454 x 328 x 25 mm) -------
SHEET_C = (0.36, 0.0)
SHEET_LX, SHEET_LY, SHEET_H, SHEET_T = 0.328, 0.454, 0.025, 0.001
SHEET_FLOOR = TOP + SHEET_T


def _sheet_shapes():
    hx, hy = SHEET_LX / 2, SHEET_LY / 2
    return [
        parts.box((SHEET_LX, SHEET_LY, SHEET_T), at=(0.0, 0.0, SHEET_T / 2), color=SHEET_ALU),
        parts.box((SHEET_T, SHEET_LY, SHEET_H), at=(hx - SHEET_T / 2, 0.0, SHEET_H / 2), color=SHEET_ALU),
        parts.box((SHEET_T, SHEET_LY, SHEET_H), at=(-hx + SHEET_T / 2, 0.0, SHEET_H / 2), color=SHEET_ALU),
        parts.box((SHEET_LX, SHEET_T, SHEET_H), at=(0.0, hy - SHEET_T / 2, SHEET_H / 2), color=SHEET_ALU),
        parts.box((SHEET_LX, SHEET_T, SHEET_H), at=(0.0, -hy + SHEET_T / 2, SHEET_H / 2), color=SHEET_ALU),
    ]


def sheet_inside(x: float, y: float) -> bool:
    return abs(x - SHEET_C[0]) <= SHEET_LX / 2 - SHEET_T + EPS and abs(y - SHEET_C[1]) <= SHEET_LY / 2 - SHEET_T + EPS


# -- the shortening: a 4 oz porcelain ramekin (3.5 in across, 1.75 in tall; commodity catalogue size), about 180 g,
# holding 25 mm (about 120 g) of solid white vegetable shortening (Crisco All-Vegetable Shortening).
RAM_R, RAM_H, RAM_T = 0.0445, 0.044, 0.005
RAM_RIN = RAM_R - RAM_T
FAT_Z = 0.030  # top of the shortening, ramekin frame (floor 5 mm thick)


def _ramekin_shapes():
    return ([parts.cylinder(RAM_R - 0.0004, 0.005, at=(0.0, 0.0, 0.0025), color=(0.95, 0.95, 0.93)),
             parts.cylinder(RAM_RIN - 0.0003, FAT_Z - 0.005, at=(0.0, 0.0, (FAT_Z + 0.005) / 2), color=SHORTENING_WHITE)]
            + _ring(RAM_R, RAM_T, 0.005, RAM_H, 20, (0.95, 0.95, 0.93)))


# -- the spoon rest: a ceramic spoon rest (estimate: 200 x 70 mm, 300 g) with a raised pad under the lower end of the
# brush's handle and a notch near its far end, so the brush lies off the counter, its tips toward the far side.
REST_PAD_Z, REST_NOTCH_Z = 0.022, 0.042  # tops of the handle pad and the notch bar, rest frame
PAD_AT, NOTCH_AT = 0.090, 0.175  # along the brush from its tips: the handle's lower end, the notch


# -- the brush: Nordic Ware natural-bristle pastry brush (02122), beech, 8 in (203 mm) long, 0.2 lb (91 g) -------------
# Part frame: origin at the middle of the bristle tips, z along the brush toward the handle's end. Head 1.5 in (38 mm)
# wide and 0.5 in (12.7 mm) thick (maker's catalogue); bristles 55 mm long and handle 22 x 11 mm (estimates).
BR_W, BR_T, BR_LEN = 0.038, 0.010, 0.055
BRUSH_LEN = 0.203
BRUSH_MASS = 0.091
GRIP_FROM_TIPS = 0.120  # where the hand holds it, 43 mm above the ferrule
L_T = GRIP_FROM_TIPS
_bx = np.linspace(-0.017, 0.017, 5)
_by = np.linspace(-0.004, 0.004, 3)
_bz = np.linspace(0.001, 0.050, 6)
BRISTLE_PTS = np.array([(x, y, z) for x in _bx for y in _by for z in _bz])
BRISTLE_ROOTS = np.array([(x, y, BR_LEN) for x in _bx for y in _by for z in _bz])
LOADED_SHOWN = 0.2  # g of shortening on the bristles above which they are drawn coated


def _brush_shapes():
    return [
        parts.box((BR_W, BR_T, BR_LEN), at=(0.0, 0.0, BR_LEN / 2), color=BRISTLE, collide=False),
        parts.box((0.040, 0.0127, 0.022), at=(0.0, 0.0, BR_LEN + 0.011), color=BEECH, collide=False),
        parts.box((0.022, 0.011, BRUSH_LEN - BR_LEN - 0.022), at=(0.0, 0.0, (BRUSH_LEN + BR_LEN + 0.022) / 2),
                  color=BEECH),
    ]


def _rest_incline():
    """Incline of the brush lying on the rest: the handle's underside on the pad near its lower end and on the notch."""
    best = None
    for k in range(3000):
        th = math.radians(k * 0.01)
        zf = PAD_AT * math.sin(th) - 0.0055 * math.cos(th)
        zn = NOTCH_AT * math.sin(th) - 0.0055 * math.cos(th)
        err = abs((zn - zf) - (REST_NOTCH_Z - REST_PAD_Z))
        if best is None or err < best[0]:
            best = (err, th)
    return best[1]


REST_TH = _rest_incline()


def brush_rest(rest_xy):
    """Where the brush lies on the spoon rest whose middle is at rest_xy: (tips position, rotation). The brush points
    along +x (tips away from the robot), rising toward its handle at REST_TH."""
    th = REST_TH
    u = np.array([-math.cos(th), 0.0, math.sin(th)])
    v = np.array([math.sin(th), 0.0, math.cos(th)])
    xb = np.array([0.0, -1.0, 0.0])
    zt = TOP + REST_PAD_Z - PAD_AT * math.sin(th) + 0.0055 * math.cos(th) + 0.0005
    tips = np.array([rest_xy[0] + 0.10, rest_xy[1], zt])
    return tips, np.column_stack([xb, v, u])


def _rest_shapes():
    """The spoon rest in its own frame: origin at its middle on the counter, long side along x."""
    th = REST_TH
    tips_x = 0.10
    pad_x = tips_x - PAD_AT * math.cos(th) + 0.0055 * math.sin(th)
    notch_x = tips_x - NOTCH_AT * math.cos(th) + 0.0055 * math.sin(th)
    c = (0.93, 0.91, 0.86)
    return [parts.box((0.200, 0.070, 0.006), at=(0.0, 0.0, 0.003), color=c),
            parts.box((0.020, 0.018, REST_PAD_Z - 0.006), at=(pad_x, 0.0, (REST_PAD_Z + 0.006) / 2), color=c),
            parts.box((0.012, 0.030, REST_NOTCH_Z - 0.006), at=(notch_x, 0.0, (REST_NOTCH_Z + 0.006) / 2), color=c),
            parts.box((0.012, 0.006, REST_NOTCH_Z + 0.010 - 0.006), at=(notch_x, 0.018, (REST_NOTCH_Z + 0.016) / 2), color=c),
            parts.box((0.012, 0.006, REST_NOTCH_Z + 0.010 - 0.006), at=(notch_x, -0.018, (REST_NOTCH_Z + 0.016) / 2), color=c)]


# -- the dredger: Tablecraft 159 stainless dredge, 10 oz, 2 13/16 in across, 3.7 in tall (71 x 94 mm) ---------------
DR_R, DR_H = 0.0355, 0.094
DR_MASS = 0.115  # kg: about 90 g of steel (estimate) and 25 g of flour


def _dredger_shapes():
    holes = [parts.box((0.003, 0.003, 0.0006), at=(r * math.cos(a), r * math.sin(a), DR_H + 0.0001), color=DARK,
                       collide=False)
             for r, n in ((0.0, 1), (0.010, 6), (0.020, 12), (0.029, 16)) for a in np.linspace(0, 2 * math.pi, n,
                                                                                               endpoint=False)]
    return ([parts.cylinder(DR_R - 0.0003, 0.001, at=(0.0, 0.0, 0.0005), color=STEEL)]
            + _ring(DR_R, 0.0012, 0.001, DR_H - 0.006, 20, STEEL)
            + [parts.cylinder(DR_R + 0.0002, 0.006, at=(0.0, 0.0, DR_H - 0.003), color=STEEL)] + holes)


# -- layouts -----------------------------------------------------------------------------------------------------------
_PAN_SHAPES = _pan_shapes()
PAN_LOW = float(min(min(q[2] for q in sh.points) if sh.kind == "hull" else sh.at[2] - sh.half[2] for sh in _PAN_SHAPES))
PAN_A = (0.33, 0.0, SHEET_FLOOR - PAN_LOW + 0.0005)  # its lowest flute bottoms 0.5 mm above the sheet pan's floor
RAM_A, RAM_B = (0.25, -0.18, SHEET_FLOOR + 0.0005), (0.25, 0.18, SHEET_FLOOR + 0.0005)  # on the sheet pan
REST_A, REST_B = (0.34, -0.40, TOP + 0.0005), (0.34, 0.40, TOP + 0.0005)
DR_A, DR_B = (0.30, 0.315, TOP + 0.0005), (0.30, -0.315, TOP + 0.0005)
_tips_a, _rot_a = brush_rest(REST_A)

SHEET = parts.Part("half_sheet_pan", pos=(SHEET_C[0], SHEET_C[1], TOP), shapes=_sheet_shapes(), friction=(0.5, 0.4))
PAN = parts.Part("bundt_pan", pos=PAN_A, mass=PAN_MASS, shapes=_PAN_SHAPES, friction=(0.6, 0.5))
RAMEKIN = parts.Part("shortening_ramekin", pos=RAM_A, mass=0.30, shapes=_ramekin_shapes(), friction=(0.7, 0.6))
REST = parts.Part("spoon_rest", pos=REST_A, mass=0.30, shapes=_rest_shapes(), friction=(0.8, 0.7))
BRUSH = parts.Part("pastry_brush", pos=tuple(float(v) for v in _tips_a), mass=BRUSH_MASS, shapes=_brush_shapes(),
                   quat=tuple(float(v) for v in parts._matrix_to_quat(_rot_a)), friction=(1.0, 0.9),
                   visuals=(_box_visual("loaded", (0.0404, 0.0124, 0.030), (0.0, 0.0, 0.0148), SHORTENING_WHITE),))
DREDGER = parts.Part("flour_dredger", pos=DR_A, mass=DR_MASS, shapes=_dredger_shapes(), friction=(0.9, 0.8))
OBJECTS = [SHEET, PAN, RAMEKIN, REST, BRUSH, DREDGER]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={PAN.name: PAN_A}, brush_arm="right", dredger_arm="left"),
    lots.Lot("B", place={RAMEKIN.name: RAM_B, REST.name: REST_B, BRUSH.name: tuple(float(v) for v in brush_rest(REST_B)[0]),
                         DREDGER.name: DR_B}, brush_arm="left", dredger_arm="right"),
)
SUCCESS_HOLD_S = 1.5  # the 1 s hold is enforced in score() itself; this keeps a run going past success
DEMO_TOUCHES = {"bundt_pan", "pastry_brush", "flour_dredger", "shortening_ramekin", "spoon_rest", "half_sheet_pan"}

# ======================================================================================================================
# The per-trial record: the layers, what the arms and the pan did, and the end check
# ======================================================================================================================
DT = 0.05
DROP_SPEED = 0.6  # m/s downward: a pan let fall from about 18 mm or more
SPILL_MAX = 2000.0  # mg of flour on the counter (outside the sheet pan)
LOOSE_FULL, LOOSE_MAX = 500.0, 2000.0  # mg of loose flour left in the pan
UPRIGHT = D(10.0)
CLEAR = 0.030  # m between any part of either hand and the cake pan at the end
STILL_V, STILL_W = 0.005, 0.05  # m/s, rad/s
HOLD_S = 1.0
_rec: dict = parts.__dict__.setdefault("bundt_pan_record", {})  # shared by every copy of this module in a process


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", field=Field(), dropped=False, since=None, held=False, t=0.0,
                gap=None, version=0, cache=None, last_t=None, shown=None)


if not _rec:
    reset_record()


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def _valid(st) -> None:
    for name, n in (("pos", 3), ("quat_xyzw", 4), ("lin_vel", 3), ("ang_vel", 3), ("low", 3), ("high", 3)):
        a = np.asarray(getattr(st, name), float)
        if a.shape != (n,) or not np.isfinite(a).all():
            raise ValueError("object state unknown")


def pan_rest(objects) -> bool:
    """The cake pan stands upright on the sheet pan's floor, inside its walls."""
    st = objects[PAN.name]
    r = _rot(st.quat_xyzw)
    return bool(r[2, 2] >= math.cos(UPRIGHT) - EPS and float(st.low[2]) <= SHEET_FLOOR + 0.004 + EPS
                and sheet_inside(float(st.pos[0]), float(st.pos[1])))


def tools_out(objects) -> bool:
    """Neither the brush nor the dredger is in or on the cake pan (their boxes do not overlap the pan's)."""
    p = objects[PAN.name]
    for name in (BRUSH.name, DREDGER.name):
        t = objects[name]
        if np.all(np.asarray(t.low) <= np.asarray(p.high)) and np.all(np.asarray(t.high) >= np.asarray(p.low)):
            return False
    return True


def still(st) -> bool:
    return float(np.linalg.norm(st.lin_vel)) <= STILL_V + EPS and float(np.linalg.norm(st.ang_vel)) <= STILL_W + EPS


def observe(objects, hand_gap_fn, t: float) -> None:
    """One control step: the layers change, then the whole-trial rules and the end check are read."""
    r = _rec
    r["steps"] += 1
    r["t"] = float(t)
    try:
        for spec in (PAN, RAMEKIN, BRUSH, DREDGER):
            _valid(objects[spec.name])
        pan, jar, brush, dr = (objects[s.name] for s in (PAN, RAMEKIN, BRUSH, DREDGER))
        fld = r["field"]
        pp, pr = np.asarray(pan.pos, float), _rot(pan.quat_xyzw)
        br, bp = _rot(brush.quat_xyzw), np.asarray(brush.pos, float)
        tips = BRISTLE_PTS @ br.T + bp
        roots = BRISTLE_ROOTS @ br.T + bp
        # shortening: taken up in the jar, laid on the pan's inside by loaded bristles pressed on it
        jl = (tips - np.asarray(jar.pos, float)) @ _rot(jar.quat_xyzw)
        in_fat = (np.hypot(jl[:, 0], jl[:, 1]) < RAM_RIN) & (jl[:, 2] > 0.005) & (jl[:, 2] < FAT_Z)
        fld.load(float(in_fat.mean()), DT)
        if np.min(np.hypot(*(roots[:, :2] - pp[:2]).T)) < HANDLE_R + 0.02:
            if fld.brush(( tips - pp) @ pr, (roots - pp) @ pr, DT):
                r["version"] += 1
        # flour: out of the turned-over dredger, onto the pan, the sheet pan or the counter; then it settles
        drr = _rot(dr.quat_xyzw)
        if math.acos(max(-1.0, min(1.0, drr[2, 2]))) > DREDGE_TILT and fld.dredger > 0:
            fld.dust(np.asarray(dr.pos, float) + drr @ np.array([0.0, 0.0, DR_H]), pp, pr, sheet_inside, DT)
            r["version"] += 1
        before = fld.loose.sum()
        fld.settle(pp, pr, sheet_inside)
        if abs(fld.loose.sum() - before) > 1e-9:
            r["version"] += 1
        if float(pan.lin_vel[2]) < -DROP_SPEED:
            r["dropped"] = True
        done = pan_rest(objects) and tools_out(objects) and still(pan)
        gap = hand_gap_fn() if done else None
        if done and (gap is None or not math.isfinite(gap)):
            raise ValueError("arm state unknown")
        done = done and gap >= CLEAR - EPS
        r["gap"] = gap
    except (KeyError, ValueError, TypeError, AttributeError, IndexError) as e:
        r.update(known=False, reason=str(e), since=None, held=False)
        return
    if not done:
        r["since"] = None
    elif r["since"] is None:
        r["since"] = float(t)
    r["held"] = r["since"] is not None and float(t) - r["since"] >= HOLD_S - EPS


def readings() -> dict:
    """What a person measures at the end (cached while the layers do not change)."""
    r = _rec
    if r["cache"] is not None and r["cache"][0] == r["version"]:
        return r["cache"][1]
    fld = r["field"]
    out = dict(zones=fld.zones_done(), loose_mg=float(fld.loose.sum()), height_mm=float(fld.height().max()),
               spilled_mg=float(fld.spilled), covered=float((fld.covered() * AREA).sum() / TOTAL_AREA),
               greased=float(((fld.g >= G_MIN) * AREA).sum() / TOTAL_AREA))
    r["cache"] = (r["version"], out)
    return out


# ======================================================================================================================
# Scoring
# ======================================================================================================================
def score(objects):
    """0 for the whole trial if the cake pan was ever let fall (more than 0.6 m/s downward), or if the scene or the
    arms could not be read. 0 unless, at the end, the cake pan stands upright on the sheet pan, the brush and the
    dredger are out of it, no hand is within 30 mm of it and it has been still for 1 s. Then, with Z the number of
    the 36 zones (12 flutes x outer wall, floor, tube) with no bare spot 10 mm or longer: 1.0 if Z = 36, at most
    0.5 g of loose flour is left in the pan, no deposit is more than 1.5 mm thick and at most 2 g of flour fell on
    the counter; 0.75 if Z = 36 and at most 2 g loose; 0.5 if Z >= 24 and at most 2 g loose; 0.25 if Z >= 12. More
    than 2 g of loose flour left, a lump or more than 2 g on the counter caps the score at 0.25."""
    try:
        r = _rec
        if not r["known"] or r["dropped"] or r["steps"] == 0 or not r["held"]:
            return 0.0
        for spec in (PAN, BRUSH, DREDGER):
            _valid(objects[spec.name])
        if not (pan_rest(objects) and tools_out(objects) and still(objects[PAN.name])):
            return 0.0
        m = readings()
        z, loose = m["zones"], m["loose_mg"]
        if z == 36 and loose <= LOOSE_FULL + EPS and m["height_mm"] <= LUMP + EPS and m["spilled_mg"] <= SPILL_MAX:
            return 1.0
        if z == 36 and loose <= LOOSE_MAX + EPS:
            value = 0.75
        elif z >= 24 and loose <= LOOSE_MAX + EPS:
            value = 0.5
        elif z >= 12:
            value = 0.25
        else:
            value = 0.0
        if loose > LOOSE_MAX + EPS or m["height_mm"] > LUMP + EPS or m["spilled_mg"] > SPILL_MAX:
            value = min(value, 0.25)
        return value
    except (KeyError, ValueError, TypeError, AttributeError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - 1e-9


def distance(objects):
    try:
        return (36 - readings()["zones"]) / 36.0 + readings()["loose_mg"] / 10000.0
    except Exception:  # noqa: BLE001
        return 1.0


# ======================================================================================================================
# In the simulator: the arms as point clouds (hull faces sampled every 3 mm), the drawn layers, the per-step hook
# ======================================================================================================================
HAND_LINKS = ("link5", "gripper", "tip_left", "tip_right")
_FACES: dict = {}


def _hull_face_points(hull, step) -> np.ndarray:
    pts = [hull.vertices]
    for n, d in zip(hull.planes[:, :3], hull.planes[:, 3]):
        on = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 = e1 / np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for i in range(len(ring)):
            a, b = ring[i], ring[(i + 1) % len(ring)]
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step)))
            ii, jj = np.meshgrid(np.arange(m + 1), np.arange(m + 1), indexing="ij")
            keep = (ii + jj) <= m
            s_, t_ = ii[keep][:, None] / m, jj[keep][:, None] / m
            pts.append(c + (a - c) * s_ + (b - c) * t_)
    pts = np.concatenate(pts)
    _, idx = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(idx)]])


def link_points(name, step=0.003) -> np.ndarray:
    key = (name, step)
    if key not in _FACES:
        from yam_arena import analysis

        hs = analysis.link_hulls().get(name, [])
        if not hs:
            raise ValueError(f"no collision hull for {name}")
        _FACES[key] = np.concatenate([_hull_face_points(h, step) for h in hs])
    return _FACES[key]


def _arm_poses(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = {}
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in HAND_LINKS:
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                out[(arm, n)] = m
    if len(out) != 2 * len(HAND_LINKS) or not all(np.isfinite(m).all() for m in out.values()):
        return None
    return out


def pan_gap(pts, st) -> float:
    """Closest distance (m) from any point to the cake pan's own box (its body and handles, in its frame)."""
    lo, hi = PAN._local_box()
    loc = (np.asarray(pts, float) - np.asarray(st.pos, float)) @ _rot(st.quat_xyzw)
    q = np.maximum(np.maximum(lo - loc, loc - hi), 0.0)
    return float(np.linalg.norm(q, axis=1).min())


def _hand_gap(sim, objects):
    poses = _arm_poses(sim)
    if poses is None:
        return None
    pts = np.concatenate([link_points(n) @ m[:3, :3].T + m[:3, 3] for (_, n), m in poses.items()])
    return pan_gap(pts, objects[PAN.name])


def cell_colors(fld) -> np.ndarray:
    c = np.tile(np.array(BARE, np.float32), (N, 1))
    c[fld.g >= G_MIN] = GREASED
    c[fld.covered()] = FLOURED
    c[fld.loose / AREA > 0.004] = LOOSE
    c[fld.height() > LUMP] = LUMPY
    return c


def _coat_path():
    return parts._PATHS.get(PAN.name, "") + "/coat"


def _draw(sim, force=False) -> None:
    from isaaclab.sim.utils import get_current_stage
    from pxr import UsdGeom, Vt

    stage = get_current_stage()
    fld = _rec["field"]
    key = (_rec["version"], fld.brush_load > LOADED_SHOWN)
    if not force and _rec.get("shown") == key:
        return
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(_coat_path()))
    if mesh:
        mesh.GetDisplayColorPrimvar().Set(Vt.Vec3fArray.FromNumpy(cell_colors(fld)))
    shown = _rec.get("shown")
    if force or shown is None or shown[1] != key[1]:
        sim.set_visible(BRUSH.name, "loaded", key[1])
    _rec["shown"] = key


def _author_coat(root_path) -> None:
    """The pan's inside, drawn as one mesh of the 96 x 52 cells, 0.3 mm in front of the surface; its cell colours
    show the layers (dark: bare; pale: shortening; white: flour)."""
    from isaaclab.sim.utils import get_current_stage
    from pxr import Gf, UsdGeom, Vt

    stage = get_current_stage()
    th = np.arange(N_TH + 1) * D_TH
    s = np.clip(np.arange(N_S + 1) * D_S, 1e-6, S_TOTAL - 1e-6)
    tt, ss = np.meshgrid(th, s, indexing="ij")
    p = surface(tt, ss)
    dds = (surface(tt, ss + 1e-5) - surface(tt, ss - 1e-5))
    ddt = (surface(tt + 1e-5, ss) - surface(tt - 1e-5, ss))
    n = np.cross(dds, ddt)
    n /= np.linalg.norm(n, axis=-1)[..., None]
    base_in = -np.stack([np.interp(ss, S_F, NOUT_F[:, 0]) * np.cos(tt), np.interp(ss, S_F, NOUT_F[:, 0]) * np.sin(tt),
                         np.interp(ss, S_F, NOUT_F[:, 1])], axis=-1)
    n = np.where((np.sum(n * base_in, axis=-1) < 0)[..., None], -n, n)
    pts = (p + 0.0003 * n).reshape(-1, 3)
    idx = []
    for i in range(N_TH):
        for j in range(N_S):
            a, b, c, d = i * (N_S + 1) + j, (i + 1) * (N_S + 1) + j, (i + 1) * (N_S + 1) + j + 1, i * (N_S + 1) + j + 1
            idx += [a, d, c, b]
    mesh = UsdGeom.Mesh.Define(stage, root_path + "/coat")
    mesh.CreatePointsAttr([Gf.Vec3f(*map(float, q)) for q in pts])
    mesh.CreateFaceVertexCountsAttr([4] * N)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    pv = mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform)
    pv.Set(Vt.Vec3fArray.FromNumpy(cell_colors(Field())))


_author_plain = parts._author


def _author_with_coat(part, root_path, translation, orientation):
    root = _author_plain(part, root_path, translation, orientation)
    try:
        if part.name == PAN.name:
            _author_coat(root_path)
        if part.name == BRUSH.name:
            from isaaclab.sim.utils import get_current_stage
            from pxr import UsdGeom

            prim = get_current_stage().GetPrimAtPath(f"{root_path}/vis_loaded")
            if prim.IsValid():
                UsdGeom.Imageable(prim).MakeInvisible()
    except Exception as e:  # noqa: BLE001  drawing never decides a score
        print(f"[pan] coat mesh not drawn: {e}", file=sys.stderr, flush=True)
    return root


parts._author = _author_with_coat
_stock_apply = lots._apply


def _apply_lot(emb) -> bool:
    """Lay out the lot (the stock move and settle), then start the trial's record with a clean pan."""
    moved = _stock_apply(emb)
    reset_record()
    try:
        _draw(parts.Sim(emb), force=True)
        for _ in range(lots.RENDERS):
            emb._base.sim.render()
    except Exception:  # noqa: BLE001
        pass
    return moved


lots._apply = _apply_lot


@parts.on_reset
def _new_trial(sim) -> None:
    reset_record()
    try:
        _draw(sim, force=True)
    except Exception:  # noqa: BLE001
        pass


@parts.on_step
def _step(sim) -> None:
    if _rec.get("last_t") == sim.t:  # a second copy of this module in the process: observe once per step
        return
    _rec["last_t"] = sim.t
    try:
        objects = sim.states()
    except Exception:  # noqa: BLE001  unknown state: no credit
        objects = {}

    def gap():
        try:
            return _hand_gap(sim, objects)
        except Exception:  # noqa: BLE001
            return None

    observe(objects, gap, float(sim.t))
    try:
        _draw(sim)
    except Exception:  # noqa: BLE001
        pass
    log = os.environ.get("PAN_LOG") or (str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("pan.jsonl"))
                                        if os.environ.get("TRACE_OUT") else None)
    if log and _rec["steps"] % 5 == 0 and objects:
        try:
            fld = _rec["field"]
            m = readings() if _rec["steps"] % 40 == 0 or _rec["held"] else (_rec["cache"][1] if _rec["cache"] else readings())
            row = dict(t=round(float(sim.t), 2), lot=lots.current().name, score=score(objects), known=_rec["known"],
                       reason=_rec["reason"], dropped=_rec["dropped"], held=_rec["held"], gap=_rec["gap"],
                       zones=m["zones"], covered=round(m["covered"], 4), greased=round(m["greased"], 4),
                       loose_mg=round(m["loose_mg"], 1), height=round(m["height_mm"], 3),
                       spilled=round(fld.spilled, 1), caught=round(fld.caught, 1), dredger=round(fld.dredger, 1),
                       load=round(fld.brush_load, 3), used=round(fld.used, 3))
            for spec in (PAN, BRUSH, DREDGER, RAMEKIN):
                st = objects[spec.name]
                row[spec.name] = np.round(np.r_[st.pos, st.quat_xyzw], 4).tolist()
            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            for arm in ("left_robot", "right_robot"):
                a = base.scene[arm]
                poses = parts._np(a.data.body_link_pose_w)[0]
                row[arm] = np.round(poses[list(a.body_names).index("gripper"), :3] - origin, 4).tolist()
                row[arm + "_q"] = np.round(parts._np(a.data.joint_pos)[0], 3).tolist()
            with open(log, "a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with open(log, "a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# ======================================================================================================================
# Demo: open loop from the layout the top camera shows (where the brush, the ramekin, the dredger and the pan stand).
# It reads no layer, no score and no simulator pose.
# ======================================================================================================================
def _w(d, p, finger, label, rot):
    return Waypoint(d, tuple(float(v) for v in p), finger, label, tilt._TiltedYaw(0.0, np.asarray(rot, float)))


def _down(yaw):
    from yam_arena.demo import grip_rotation

    return grip_rotation(float(yaw))


def _rest_pose(arm):
    from yam_arena import ik, station
    from yam_arena.demo import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return tuple(float(v) for v in pos), rot


def _axis_rot(axis, ang):
    return parts._rot(axis, ang)


def _frame(closing, approach):
    a = np.asarray(approach, float) / np.linalg.norm(approach)
    c = np.asarray(closing, float)
    c = c - a * (c @ a)
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


PRESS = 0.007  # m the bristles' leading edge is set past the bottom of a flute
GRIP_ANGLE = D(45.0)  # the handle crosses the fingers at 45 degrees to the hand's approach
U_G = np.array([0.0, math.sin(GRIP_ANGLE), -math.cos(GRIP_ANGLE)])  # brush axis (tips to handle), hand frame
# The brush is held in one pose for all of the brushing and dipping: it leans BRUSH_POSE[0] degrees up from horizontal
# with its top toward heading BRUSH_POSE[1] (degrees, world), the hand turned BRUSH_POSE[2] (rad) about it. Found
# offline with the arm's inverse kinematics (right arm; the left arm's is its mirror image) so that every point of
# every pass round the ring is in reach without turning the wrist. Each pass goes once round the ring at one height
# of the profile.
BRUSH_POSE = (80.0, -150.0, -3.1416)
PASS_S = (0.003, 0.012, 0.021, 0.030, 0.039) + tuple(S_WALL + d for d in (0.002, 0.013, 0.024, 0.035, 0.046, 0.057,
    0.068, 0.079)) + tuple(S_TOTAL - d for d in (0.047, 0.032, 0.017, 0.004))  # outer x5, floor x8, tube x4 (m)
# Short extra passes where the leaning brush meets the rising inner floor edge-on (the far quarter on the dredger's
# side), pressed harder: (profile length, press, from and to azimuth in degrees, world, right arm).
EXTRA_PASSES = ((S_TOTAL - 0.064, 0.003, -20.0, 110.0), (S_TOTAL - 0.059, 0.002, -20.0, 110.0),
                (S_WALL + 0.052, 0.009, -40.0, 70.0), (S_TOTAL - 0.045, 0.012, -10.0, 70.0), (0.047, 0.009, 160.0, 240.0),
                (S_WALL + 0.050, 0.010, 90.0, 150.0), (0.046, 0.010, 50.0, 100.0), (0.005, 0.010, 60.0, 15.0),
                (0.044, 0.010, 10.0, 60.0), (S_TOTAL - 0.055, 0.002, 0.0, 60.0),
                # the far side of the pan from the brushing arm, where the arm sags and falls short and low (measured
                # in GPU runs): the outer wall pressed harder, the foot of the tube lightly and a little higher
                (S_WALL + 0.007, 0.014, 10.0, 95.0), (0.031, 0.012, 90.0, 25.0), (0.018, 0.012, 60.0, 30.0),
                (S_TOTAL - 0.051, 0.002, -10.0, 60.0), (S_TOTAL - 0.047, 0.002, 0.0, 60.0), (0.120, 0.004, 50.0, 5.0),
                (0.121, 0.009, 5.0, 45.0), (0.124, 0.010, 35.0, 0.0), (0.097, 0.009, 230.0, 190.0),
                (0.1215, 0.003, 15.0, 50.0), (0.1225, 0.008, 50.0, 15.0),
                (0.1155, 0.006, 2.0, 28.0), (0.1185, -0.002, 28.0, 2.0), (0.1215, 0.006, 2.0, 28.0), (0.123, 0.006, 28.0, 2.0))
PASS_STEPS = 20
PASS_SPEED = 0.065  # m/s of the bristles round the ring
RELOAD_BEFORE = (4, 8, 12, 16, 20, 23)


def hand_for(b, omega):
    """Hand rotation holding the brush with its tips pointing along b, turned omega about the brush."""
    u = -np.asarray(b, float) / np.linalg.norm(b)
    v = np.cross(U_G, u)
    s, c = np.linalg.norm(v), float(U_G @ u)
    r0 = parts._rot(v / s, math.atan2(s, c)) if s > 1e-9 else np.eye(3)
    return parts._rot(u, omega) @ r0


def brush_hand():
    """(hand rotation, direction the bristle tips point) while brushing."""
    e, g, om = BRUSH_POSE
    b = -(math.sin(D(e)) * np.array([0.0, 0.0, 1.0]) + math.cos(D(e)) * np.array([math.cos(D(g)), math.sin(D(g)), 0.0]))
    return hand_for(b, om), b


def pass_tips(phi, s, pan_c, r_hand, press=PRESS):
    """Middle of the bristle tips for the pass at profile length s, azimuth phi (world): the bristles' farthest point
    toward the metal set PRESS past the bottom of a flute there (the deepest part of the inside at that height)."""
    r = float(np.interp(s, S_F, P_F[:, 0]))
    z = float(np.interp(s, S_F, P_F[:, 1]))
    nr = float(np.interp(s, S_F, NOUT_F[:, 0]))
    nz = float(np.interp(s, S_F, NOUT_F[:, 1]))
    a = float(amp(r))
    radial = np.array([math.cos(phi), math.sin(phi), 0.0])
    surf = np.asarray(pan_c, float) + (r + a * nr) * radial + np.array([0.0, 0.0, z + a * nz])
    n_out = nr * radial + np.array([0.0, 0.0, nz])
    rb = r_hand @ brush_in_hand()  # the brush's frame
    support = float(np.max((BRISTLE_PTS @ rb.T) @ n_out))
    return surf + (press - support) * n_out


MIRROR = np.diag([1.0, -1.0, 1.0])


def _mirror(wps):
    """The same plan for the other arm: reflected across the arms' middle plane (y -> -y); the hand's closing axis is
    turned round so that each rotation stays a rotation (the fingers are the same either way)."""
    out = []
    for w in wps:
        r = MIRROR @ np.asarray(w.yaw.rotation, float) @ MIRROR
        r[:, 0] *= -1.0
        r[:, 1] *= -1.0
        out.append(_w(w.duration, MIRROR @ np.asarray(w.pos, float), w.finger, w.label, r))
    return out


def _off_rim(p, pan_c, rim_z):
    """Where the bristles come off the pan from point p: up to 12 mm over the rim's height, or, near the rim, 5 mm up
    and 15 mm in toward the middle (a high lift at the far rim is out of the arm's reach)."""
    if p[2] < rim_z - 0.02:
        return np.array([0.0, 0.0, max(0.0, rim_z + 0.012 - p[2])])
    rel = np.asarray(p, float)[:2] - np.asarray(pan_c, float)[:2]
    return np.r_[-0.015 * rel / max(np.linalg.norm(rel), 1e-9), 0.005]


def brush_grasp(rest):
    """Hand pose that takes the brush lying on the spoon rest: fingers across the handle's width, 120 mm from the
    tips, the handle crossing the fingers at 45 degrees (the hand leans away from the robot by 45 degrees less the
    brush's own rise)."""
    tips0, rb0 = brush_rest(rest)
    u0 = rb0[:, 2]
    lean = D(45.0) - REST_TH
    a = np.array([math.sin(lean), 0.0, -math.cos(lean)])
    c = np.array([0.0, -1.0, 0.0])
    return tips0, u0, np.column_stack([c, np.cross(a, c), a])


_HANDLE_PTS = np.array([(x, y, z) for x in (-0.011, 0.0, 0.011) for y in (-0.0055, 0.0055)
                        for z in np.linspace(0.077, 0.14, 10)])
HANDLE_CLEAR = 0.006  # m the brush's handle is kept from the tube's top and the rim while brushing
# Where each arm's brush passes are asked to go, shifted (m, world): in GPU runs the brush settled in the fingers turned
# 2-4 degrees from the planned grasp, putting its bristles this far the other way (measured over the passes of two runs;
# a fixed correction, not feedback)
BRUSH_SHIFT = {"right": (-0.003, 0.0055, 0.0), "left": (0.0025, -0.007, 0.0)}


def handle_clearance(tips, r_hand, pan_c):
    """Closest distance (m) from the brush's handle to the top of the tube or the rim, for the bristle tips at
    ``tips`` (the tube's top and the rim are the only parts of the pan a near-upright handle can reach)."""
    rb = r_hand @ brush_in_hand()
    pts = _HANDLE_PTS @ rb.T + np.asarray(tips, float) - np.asarray(pan_c, float)
    r, z = np.hypot(pts[:, 0], pts[:, 1]), pts[:, 2]
    above = np.maximum(z - RIM_Z, 0.0)
    tube = np.hypot(np.maximum(r - P_TOP[0], 0.0), above)
    rim = np.hypot(np.maximum(P_RIM[0] - r, 0.0), above)
    return float(min(tube.min(), rim.min()))


def _clear_handle(tips, r_hand, pan_c):
    """The tips lifted (in 1 mm steps, up to 30 mm) until the handle is HANDLE_CLEAR from the tube's top and the rim."""
    p = np.asarray(tips, float).copy()
    for _ in range(30):
        if handle_clearance(p, r_hand, pan_c) >= HANDLE_CLEAR - 1e-9:
            break
        p[2] += 0.001
    return p


def brush_in_hand():
    """The brush's frame in the hand's frame once it is taken off the rest (fixed by the grasp)."""
    tips0, rb0 = brush_rest(REST_A)
    _, _, rg = brush_grasp(REST_A)
    return rg.T @ rb0


def brush_plan(rest, ram, pan_c, pan_yaw):
    """Right-arm plan (lot A's side): take the brush off the rest, load it in the ramekin, brush the inside in passes
    round the ring from the rim to the tube's top (reloading five times), and lay it back on the rest."""
    tips0, u0, rg = brush_grasp(rest)
    grip0 = tips0 + L_T * u0
    f_brush = -0.005  # pads 10 mm apart: 6 mm past each side of the 22 mm handle, a firm hold (it slipped at -8.5 mm)
    back = grip0 - 0.07 * rg[:, 2]
    lifted = grip0 + np.array([0.0, 0.0, 0.06])
    w = [_w(2.5, back + (0, 0, 0.03), FINGER_OPEN, "behind the brush's handle, hand leaned", rg),
         _w(1.5, back, FINGER_OPEN, "in line with the handle", rg),
         _w(2.0, grip0, FINGER_OPEN, "fingers either side of the handle", rg),
         _w(1.0, grip0, f_brush, "close on the handle", rg),
         _w(0.8, grip0, f_brush, "settle", rg),
         _w(2.0, lifted, f_brush, "lift the brush off the rest", rg)]
    rdip, bdip = brush_hand()  # dipped in the same pose as it brushes, so the wrist does not turn
    over_ram = np.array([ram[0], ram[1], ram[2] + RAM_H + 0.012]) - L_T * bdip
    in_fat = np.array([ram[0], ram[1], ram[2] + 0.0105]) - L_T * bdip

    def dip(label):
        return [_w(2.5, over_ram, f_brush, f"brush upright over the ramekin ({label})", rdip),
                _w(1.5, in_fat, f_brush, f"bristles down into the shortening ({label})", rdip),
                _w(3.0, in_fat, f_brush, "work it in", rdip),
                _w(1.5, over_ram, f_brush, "lift out of the ramekin", rdip)]

    w += dip("first load")
    rim_z = pan_c[2] + RIM_Z
    r, b = brush_hand()
    above = np.array([pan_c[0], pan_c[1], rim_z + 0.03]) - L_T * b
    passes = [(s_, PRESS, 180.0 + 40.0 * n, 540.0 + 40.0 * n) for n, s_ in enumerate(PASS_S)] + list(EXTRA_PASSES)
    for n, (s_, press, a0, a1) in enumerate(passes):
        if n in RELOAD_BEFORE:
            w += dip(f"reload {RELOAD_BEFORE.index(n) + 1}")
        steps = max(4, int(round(PASS_STEPS * (a1 - a0) / 360.0)))
        pts = [pass_tips(D(a0 + (a1 - a0) * k / steps), s_, pan_c, r, press) for k in range(steps + 1)]
        # where the leaning handle would come over the tube's top or the rim, the bristles are lifted to keep it clear
        pts = [_clear_handle(p, r, pan_c) for p in pts]
        start = pts[0] - L_T * b
        w.append(_w(2.5 if n in (0,) + RELOAD_BEFORE else 1.2, start + _off_rim(pts[0], pan_c, rim_z), f_brush,
                    f"over the start of pass {n + 1}", r))
        w.append(_w(1.0, start, f_brush, f"bristles onto the pan (pass {n + 1})", r))
        step_t = max(0.08, float(np.linalg.norm(pts[1][:2] - pts[0][:2])) / PASS_SPEED)
        for p in pts[1:]:
            w.append(_w(step_t, p - L_T * b, f_brush, f"pass {n + 1} round the ring", r))
        w.append(_w(0.6, pts[-1] - L_T * b + _off_rim(pts[-1], pan_c, rim_z), f_brush, f"lift off (pass {n + 1})", r))
    w.append(_w(2.0, above, f_brush, "brush up out of the cake pan", r))
    w += [_w(2.5, lifted, f_brush, "brush back over the rest", rg),
          _w(1.5, grip0 + np.array([0.0, 0.0, 0.002]), f_brush, "lay it back on the rest", rg),
          _w(1.0, grip0 + np.array([0.0, 0.0, 0.002]), FINGER_OPEN, "let go of the brush", rg),
          _w(1.5, back + (0, 0, 0.03), FINGER_OPEN, "hand back off the handle", rg)]
    return w


DR_GRASP_LEAN = 60.0
DR_RINGS = (0.072, 0.032)  # the lid goes round the outer part of the ring, then the inner part
DR_RING_R = DR_RINGS[0]
DR_LID_Z = RIM_Z + 0.070  # lid this high above the pan's base while dusting
DR_SQUEEZE = 0.003
DR_BACK = (0.34, 0.44)  # m, where the dredger is stood after dusting (y on its own side), clear of that arm


def _lean_grip(alpha_deg, heading, roll=0.0):
    return tilt.rotation(heading, alpha_deg, heading) @ parts._rot((0, 0, 1), roll)


def dredger_plan(arm, at, pan_c, back=None):
    """Take the dredger across its middle with the hand leaned 60 degrees, carry it over the cake pan, turn it over
    there (the wrist turns 155 degrees: the dredger ends up 115 degrees from upright), dust round the outer part of
    the ring and then the inner part, turn it back and stand it on the counter at ``back`` (where it was if None)."""
    sgn = 1.0 if arm == "left" else -1.0
    hd = D(30.0) * sgn  # lean heading, found offline to keep every dusting point in reach
    roll = D(155.0) * sgn
    grip = _lean_grip(DR_GRASP_LEAN, hd)
    f = -(DR_R - DR_SQUEEZE)
    gp = np.array([at[0], at[1], TOP + DR_H / 2])
    gb = gp if back is None else np.array([back[0], back[1], TOP + DR_H / 2])
    hv = np.array([math.cos(hd), math.sin(hd), 0.0])
    w = [_w(2.5, gp - 0.07 * hv + (0, 0, 0.09), FINGER_OPEN, "over the dredger, hand leaned", grip),
         _w(2.0, gp - 0.07 * hv, FINGER_OPEN, "beside the dredger", grip),
         _w(1.5, gp, FINGER_OPEN, "fingers either side of the dredger", grip),
         _w(1.0, gp, f, "close on the dredger", grip),
         _w(1.5, gp + (0, 0, 0.10), f, "lift it, upright", grip)]
    r_tcp_to_d = grip.T  # the dredger's frame in the hand's, from the grasp (dredger upright, unturned)
    turned = _lean_grip(DR_GRASP_LEAN, hd, roll)

    def tcp_for(lid, rot):
        rd = rot @ r_tcp_to_d
        return np.asarray(lid, float) - rd @ np.array([0.0, 0.0, DR_H / 2])

    start_phi = math.pi
    z_lid = pan_c[2] + DR_LID_Z
    lid0 = np.array([pan_c[0] + DR_RINGS[0] * math.cos(start_phi), pan_c[1] + DR_RINGS[0] * math.sin(start_phi), z_lid])
    pivot = tcp_for(lid0, turned)  # the hand stays here while the wrist turns the dredger over
    high = np.array([gp[0], gp[1], pivot[2] + 0.01])
    w[-1] = _w(2.0, high, f, "lift it, upright, above the cake pan's rim", grip)
    w.append(_w(3.0, pivot + (0, 0, 0.01), f, "carry it over the cake pan", grip))
    w.append(_w(1.0, pivot, f, "steady", grip))
    for k in (1, 2, 3):
        rot = _lean_grip(DR_GRASP_LEAN, hd, roll * k / 3)
        w.append(_w(1.0, pivot, f, f"turn the wrist: dredger turning over ({k}/3)", rot))
    n = 16
    for ring, rr in enumerate(DR_RINGS):
        for k in range(0 if ring else 1, n + 1):
            ph = start_phi + 2 * math.pi * k / n * (-sgn)
            lid = np.array([pan_c[0] + rr * math.cos(ph), pan_c[1] + rr * math.sin(ph), z_lid])
            w.append(_w(0.7 if k else 1.0, tcp_for(lid, turned), f, f"dust round ring {ring + 1} ({k}/{n})", turned))
    lid_end = np.array([pan_c[0] + DR_RINGS[-1] * math.cos(start_phi), pan_c[1] + DR_RINGS[-1] * math.sin(start_phi),
                        z_lid])
    pivot_end = tcp_for(lid_end, turned)
    for k in (2, 1, 0):
        rot = _lean_grip(DR_GRASP_LEAN, hd, roll * k / 3)
        w.append(_w(0.9, pivot_end, f, "turn the wrist back", rot))
    w.append(_w(2.5, np.array([gb[0], gb[1], high[2]]), f, "carry the dredger back, above the rim", grip))
    w += [_w(2.0, gb + (0, 0, 0.002), f, "stand it down", grip),
          _w(0.8, gb + (0, 0, 0.002), FINGER_OPEN, "let go of the dredger", grip),
          _w(1.5, gb - 0.07 * hv, FINGER_OPEN, "hand back off the dredger", grip),
          _w(1.5, gb - 0.07 * hv + (0, 0, 0.09), FINGER_OPEN, "hand up", grip)]
    return w


FLIP_Z = 0.170  # handle height above the counter while the pan is turned over (its swing clears the sheet pan)
HANDLE_REACH = 0.122  # TCP this far from the pan's axis: pad tips 8.5 mm in over the handle
# Fingers opened only part way for the handles: a 32 mm gap round the 8 mm handle. Fully open, the pads stand 47-71 mm
# either side of the TCP and the lower one lands on the handle or the sheet pan's wall on the way down.
FLIP_OPEN = -0.016
FLIP_SIDE = -0.065  # m along the handle's width the hand comes down beside it (the far end): the pads clear the handle


def _joint_path(arm, pos, rot, seed, n=4, total=4.0, label="toward the handle", finger=None):
    """Waypoints from the rest pose to (pos, rot) that the arm's own inverse kinematics can follow: the poses along a
    straight line in joint space (forward kinematics only). A straight line in the hand's own space passes poses out
    of reach on the way to a side grip."""
    from yam_arena import ik, station
    from yam_arena.demo import READY

    q1, _, _ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), q_init=seed, base_pos=station.BASES[arm],
                        iters=400)
    q0 = np.array(READY, float)
    out = []
    for k in range(1, n):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / n, station.BASES[arm])
        out.append(_w(total / n, p, FINGER_OPEN if finger is None else finger, label, r))
    return out


def flip_plans(pan_c, pan_yaw, flip_sign=1.0, roll_l=-1.0, roll_r=1.0):
    """Both hands take a handle each, lift the pan, turn it over above the sheet pan, hold it there, turn it back and
    stand it down where it was."""
    h = np.array([-math.sin(pan_yaw), math.cos(pan_yaw), 0.0])  # toward the left handle
    zc = pan_c[2] + HANDLE_ZC + 0.002
    c = np.array([pan_c[0], pan_c[1], zc])
    up = np.array([0.0, 0.0, 1.0])
    rl0 = _frame(up * roll_l, -h)
    rr0 = _frame(up * roll_r, h)
    f = -0.001
    plans = {}
    for arm, sgn, r0 in (("left", 1.0, rl0), ("right", -1.0, rr0)):
        g = c + sgn * HANDLE_REACH * h
        out = g - FLIP_SIDE * np.array([math.cos(pan_yaw), math.sin(pan_yaw), 0.0])  # slide on along the handle's width
        lifted = np.array([g[0], g[1], TOP + FLIP_Z])
        seed = [0.0, 2.2, 0.7, 1.5, 1.5 * sgn, 1.5 * sgn]
        w = _joint_path(arm, out + (0, 0, 0.04), r0, seed, finger=FLIP_OPEN)
        w += [_w(1.5, out + (0, 0, 0.04), FLIP_OPEN, "beside the handle", r0),
             _w(1.5, out, FLIP_OPEN, "level with the handle", r0),
             _w(2.0, g, FLIP_OPEN, "fingers slide over and under the handle", r0),
             _w(1.0, g, f, "close on the handle", r0),
             _w(2.5, lifted, f, "lift the pan clear of the sheet pan", r0)]
        for k in (1, 2, 3, 4):
            w.append(_w(1.0, lifted, f, f"turn the pan over ({k}/4)", _axis_rot(h, flip_sign * k * math.pi / 4) @ r0))
        w.append(_w(3.0, lifted, f, "hold it upside down over the sheet pan", _axis_rot(h, flip_sign * math.pi) @ r0))
        for k in (3, 2, 1, 0):
            w.append(_w(1.0, lifted, f, f"turn it back ({k}/4)", _axis_rot(h, flip_sign * k * math.pi / 4) @ r0))
        w += [_w(2.5, g + (0, 0, 0.002), f, "stand the pan back down", r0),
              _w(1.0, g + (0, 0, 0.002), FLIP_OPEN, "let go of the handle", r0),
              _w(1.5, out + (0, 0, 0.002), FLIP_OPEN, "hand out from the handle", r0),
              _w(2.0, np.array([out[0], out[1], TOP + FLIP_Z]), FLIP_OPEN, "hand up, well clear of the pan", r0)]
        plans[arm] = w
    return plans


def demo():
    seen = lots.current()
    jar = seen.pos(RAMEKIN.name)
    rest_xy = seen.pos(REST.name)
    dr = seen.pos(DREDGER.name)
    pan_c = seen.pos(PAN.name)
    pan_yaw = seen.yaw_of(PAN.name)
    barm = "right" if jar[1] < 0 else "left"
    darm = "left" if barm == "right" else "right"
    plans = {barm: [], darm: []}
    rest = {a: _rest_pose(a) for a in ("left", "right")}
    for a in plans:
        plans[a] += [_w(2.0, rest[a][0], FINGER_OPEN, "to the rest pose", rest[a][1])]
    if os.environ.get("PAN_DEMO") == "nobrush":  # development: dust and turn over only
        bw = []
    elif barm == "right":
        bw = brush_plan(rest_xy, jar, pan_c, pan_yaw)
    else:  # the mirror image of the right arm's plan, for the brush and ramekin on the left
        bw = _mirror(brush_plan(MIRROR @ np.asarray(rest_xy), MIRROR @ np.asarray(jar), MIRROR @ np.asarray(pan_c),
                                -pan_yaw))
    shift = np.array(BRUSH_SHIFT[barm])
    bw = [_w(w.duration, np.asarray(w.pos, float) + shift, w.finger, w.label, w.yaw.rotation)
          if "round the ring" in w.label or "bristles onto" in w.label else w for w in bw]
    bw += [_w(2.0, rest[barm][0], FINGER_OPEN, "back to rest", rest[barm][1])]
    plans[barm] += bw
    if bw:
        plans[darm] += [_w(sum(x.duration for x in bw), rest[darm][0], FINGER_CLOSED, "wait at rest", rest[darm][1])]
    # stood down nearer the robot than it came from: its own spot is in the arm's way when that hand takes the handle
    dw = dredger_plan(darm, dr, pan_c, (DR_BACK[0], math.copysign(DR_BACK[1], dr[1]))) + [_w(2.0, rest[darm][0], FINGER_OPEN, "back to rest", rest[darm][1])]
    plans[darm] += dw
    plans[barm] += [_w(sum(x.duration for x in dw), rest[barm][0], FINGER_CLOSED, "wait at rest", rest[barm][1])]
    fl = flip_plans(pan_c, pan_yaw)
    for a in plans:
        plans[a] += fl[a] + [_w(3.0, fl[a][-1].pos, FLIP_OPEN, "hold still, hands clear", fl[a][-1].yaw.rotation)]
    return plans
