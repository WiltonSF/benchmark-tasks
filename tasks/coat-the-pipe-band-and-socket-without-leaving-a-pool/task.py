"""A plumber's bench with two small bench vises. The left-hand vise (nearer the robot's right arm) holds a white
1-1/2 inch PVC pipe upright; a black line is drawn round the pipe 18.3 mm below its top end. The other vise holds a
short upright pipe stub with a white PVC coupling pushed onto it, its upper socket open and facing up. A small can
of blue PVC cement stands on the bench with its lid resting loosely on the neck; a wool dauber on a wire hangs from
the lid into the cement. The can stands in a different place in some trials.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys
from functools import lru_cache

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
EPS = 1e-9

# ---------------------------------------------------------------------------------------------------------------
# Sizes (m). Pipe: 1-1/2 in Schedule 40 PVC, OD 1.900 in, minimum wall 0.145 in (maker's catalogue).
# Coupling: 1-1/2 in PVC DWV hub x hub coupling, 1-9/16 in long with a 1/8 in centre stop (maker's catalogue), so
# socket dimensions below are technician figures obtained by phone (Nov 2026).
# ---------------------------------------------------------------------------------------------------------------
R_PIPE = 0.02413  # pipe outside radius
R_BORE = 0.02045  # pipe inside radius (also the coupling stop's inside radius)
DEPTH = 0.0183  # m: technician socket depth; also the pipe coating-band depth
R_SOCK = 0.0486 / 2  # m: technician socket mouth diameter, represented by a straight socket
R_COUP = 0.056 / 2  # m: technician coupling outside diameter
STOP = 0.003175  # centre stop height
COUPLING_LEN = 2 * DEPTH + STOP  # 39.775 mm from technician depth + catalogue stop

H_PIPE = TOP + 0.115  # top end of the pipe in its vise
H_COUP = TOP + 0.105  # top (mouth) of the coupling
Z_STOP = H_COUP - DEPTH  # top face of the coupling's stop (bottom of the socket)
PLATE_H = 0.015  # vise base plate

# Dauber (part frame: origin at the lid centre, z up when hanging). Lid 40 x 16 mm; wire 45 mm; wool swab 19 x 25 mm.
LID_R, LID_H = 0.020, 0.016
WIRE_TOP, WIRE_BOT = -0.008, -0.053
SWAB_R, SWAB_TOP, SWAB_TIP = 0.0095, -0.053, -0.078

# Can: 8 fl oz (237 mL) round can, 70 mm across, 87 mm to the shoulder, neck 36 mm across to 95 mm (estimates).
CAN_R, CAN_H, NECK_R, NECK_IN, NECK_H = 0.035, 0.087, 0.018, 0.015, 0.008
CEMENT_LEVEL = 0.040  # cement surface above the can's bottom (about 140 mL left)

# Dry new 3/4 in Oatey dauber: 19 mm ball and about 40 g/mm radial squash, obtained by phone from a technician
# who works with this product (Nov 2026), kitchen scale at 1/3/5 mm, three runs. Convert g-force/mm to N/m.
# Use that stiffness at EACH compliant contact point; the aggregate simulation resistance can be higher because
# PhysX reports multiple contacts. It must not be described as a calibrated wet-wool material curve.
TECH_G_PER_MM = 40.0
SWAB_K = TECH_G_PER_MM * 0.00980665 * 1000  # 392.266 N/m per contact; at least the dry technician resistance
SWAB_DAMP = 0.5  # N s/m, contact damping (0.005 N at 10 mm/s of indentation speed; keeps the contact from ringing)
SWAB_FRICTION = (0.20, 0.15)  # wool wet with cement on PVC (estimate: the solvent film lubricates the surface)
F_COAT = 0.20  # N: least total normal force on a surface for the stroke to leave cement on it (estimate)
CONTACT_ATTR_TOL = 0.002  # m: contact-point tolerance for assigning a dauber contact to the wool shape
FOOT_ARC = 0.006  # m: a window is coated only within 6 mm (round the surface) of a reported contact point
LOAD_AREA = 3.2e-3  # m^2 of window area one dip can coat (estimate: about 0.5 mL at a 0.15 mm film)
DIP_DEPTH = 0.008  # swab tip at least this far below the cement surface reloads it

WHITE = (0.93, 0.93, 0.90)
GREY = (0.42, 0.44, 0.46)
STEEL = (0.30, 0.33, 0.36)
JAW = (0.20, 0.22, 0.24)
LID = (0.10, 0.20, 0.62)
CAN_COLOR = (0.70, 0.72, 0.74)
SWAB = (0.20, 0.36, 0.86)
WIRE = (0.52, 0.52, 0.50)
MARK = (0.03, 0.03, 0.03)
CEMENT = (0.06, 0.25, 0.85)
LABEL = (0.05, 0.22, 0.70)
parts.VISUAL_ONLY.update({MARK, CEMENT, LABEL})  # the drawn line, the can's label and the cement surface: no collider
parts.SOFT[SWAB] = (SWAB_K, SWAB_DAMP, *SWAB_FRICTION)  # the wool swab: a springy contact everywhere it touches


def ring(r_in, r_out, z0, z1, color, at=(0.0, 0.0)):
    """A tube wall from z0 to z1 (part frame) as overlapping vertical cylinders."""
    t = r_out - r_in
    mid = (r_in + r_out) / 2
    n = max(12, math.ceil(2 * math.pi * mid / t * 1.3))
    return [parts.cylinder(t / 2, z1 - z0, (at[0] + mid * math.cos(a), at[1] + mid * math.sin(a), (z0 + z1) / 2), color)
            for a in np.linspace(0, 2 * math.pi, n, endpoint=False)]


def vise(top_of_jaws, clamp_r):
    """A small bench vise: base plate, two jaws closed on a round part of radius clamp_r, screw and handle."""
    jt = top_of_jaws - TOP
    return [
        parts.box((0.12, 0.10, PLATE_H), (0.0, 0.0, PLATE_H / 2), GREY),
        parts.box((0.07, 0.018, jt - PLATE_H), (0.0, clamp_r + 0.009, (PLATE_H + jt) / 2), JAW),
        parts.box((0.07, 0.018, jt - PLATE_H), (0.0, -clamp_r - 0.009, (PLATE_H + jt) / 2), JAW),
        parts.cylinder(0.006, 0.040, (0.0, clamp_r + 0.038, PLATE_H + 0.020), STEEL, axis="y"),
        parts.cylinder(0.004, 0.090, (0.0, clamp_r + 0.060, PLATE_H + 0.020), STEEL, axis="x"),
    ]


# Fixed layout (lot A). x away from the arms, y to the robot's left; heights above the bench.
PIPE_XY = (0.290, -0.220)
COUP_XY = (0.200, -0.100)
CAN_A = (0.180, -0.270)
CAN_B = (0.245, -0.345)

_pipe_shapes = vise(TOP + 0.068, R_PIPE) + ring(R_BORE, R_PIPE, PLATE_H, H_PIPE - TOP, WHITE)
_pipe_shapes += [parts.cylinder(0.0009, 0.0025, ((R_PIPE + 0.0003) * math.cos(a), (R_PIPE + 0.0003) * math.sin(a),
                                                 H_PIPE - TOP - DEPTH - 0.00125), MARK)
                 for a in np.linspace(0, 2 * math.pi, 90, endpoint=False)]
PIPE_VISE = parts.Part("pipe_vise", pos=(PIPE_XY[0], PIPE_XY[1], TOP), shapes=_pipe_shapes, friction=(0.6, 0.5),
                       kinematic=True)

_cb = H_COUP - TOP - COUPLING_LEN  # coupling bottom (part frame)
_coup_shapes = vise(TOP + 0.058, R_PIPE)
_coup_shapes += ring(R_BORE, R_PIPE, PLATE_H, _cb + DEPTH, WHITE)  # stub pushed into the lower hub
_coup_shapes += ring(R_SOCK, R_COUP, _cb, _cb + DEPTH, WHITE)  # lower hub
_coup_shapes += ring(R_BORE, R_COUP, _cb + DEPTH, _cb + DEPTH + STOP, WHITE)  # centre stop
_coup_shapes += ring(R_SOCK, R_COUP, _cb + DEPTH + STOP, H_COUP - TOP, WHITE)  # upper hub (the socket)
COUPLING_VISE = parts.Part("coupling_vise", pos=(COUP_XY[0], COUP_XY[1], TOP), shapes=_coup_shapes,
                           friction=(0.6, 0.5), kinematic=True)

CAN = parts.Part("cement_can", pos=(CAN_A[0], CAN_A[1], TOP + CAN_H / 2), mass=0.25, friction=(0.6, 0.5), shapes=[
    parts.cylinder(CAN_R - 0.001, 0.002, (0, 0, -CAN_H / 2 + 0.001), CAN_COLOR),  # bottom
    *ring(CAN_R - 0.004, CAN_R, -CAN_H / 2 + 0.002, CAN_H / 2 - 0.002, CAN_COLOR),  # wall
    *ring(NECK_IN, CAN_R, CAN_H / 2 - 0.002, CAN_H / 2, CAN_COLOR),  # top, open in the middle
    *ring(NECK_IN, NECK_R, CAN_H / 2, CAN_H / 2 + NECK_H, CAN_COLOR),  # neck
    parts.cylinder(CAN_R + 0.0004, 0.040, (0, 0, -0.008), LABEL),  # label band (drawn only)
    parts.cylinder(CAN_R - 0.0045, 0.0008, (0, 0, -CAN_H / 2 + CEMENT_LEVEL), CEMENT),  # cement surface (drawn only)
])
_lid_z = TOP + CAN_H + NECK_H + LID_H / 2
DAUBER = parts.Part("dauber", pos=(CAN_A[0], CAN_A[1], _lid_z), mass=0.012, friction=(0.9, 0.8), shapes=[
    parts.cylinder(LID_R, LID_H, (0, 0, 0), LID),
    parts.cylinder(0.00125, WIRE_TOP - WIRE_BOT, (0, 0, (WIRE_TOP + WIRE_BOT) / 2), WIRE),
    parts.cylinder(SWAB_R, SWAB_TOP - SWAB_TIP, (0, 0, (SWAB_TOP + SWAB_TIP) / 2), SWAB),  # the whole swab collides
], contact_offset=0.002, report=True, report_bench=True)
OBJECTS = [PIPE_VISE, COUPLING_VISE, CAN, DAUBER]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"cement_can": CAN.pos, "dauber": DAUBER.pos}),
    lots.Lot("B", place={"cement_can": (CAN_B[0], CAN_B[1], CAN.pos[2]), "dauber": (CAN_B[0], CAN_B[1], _lid_z)}),
)

# ---------------------------------------------------------------------------------------------------------------
# Scoring windows: 6 rows x 48 columns on the pipe band (top end down to the line) and on the socket wall (mouth down
# to the stop), each about 3.0 x 3.2 mm, sampled at 3 x 3 points. A window is coated when any of its points is.
# ---------------------------------------------------------------------------------------------------------------
ROWS, COLS, SUB = 6, 48, 3  # SUB x SUB points per window at 10, 50 and 90 percent


def _windows(radius, z_low):
    dz, da = DEPTH / ROWS, 2 * math.pi / COLS
    pts, owner = [], []
    for r in range(ROWS):
        for c in range(COLS):
            for i in range(SUB):
                for j in range(SUB):
                    z = z_low + (r + (0.1 + 0.4 * i)) * dz  # 10%, 50% and 90% across the window
                    a = (c + (0.1 + 0.4 * j)) * da
                    pts.append((radius * math.cos(a), radius * math.sin(a), z))
                    owner.append(r * COLS + c)
    return np.array(pts), np.array(owner), dz * da * radius


P_PTS, P_OWN, P_WIN_AREA = _windows(R_PIPE, H_PIPE - DEPTH)  # relative to the pipe axis at bench x/y
S_PTS, S_OWN, S_WIN_AREA = _windows(R_SOCK, Z_STOP)
NWIN = ROWS * COLS

# Swab sample points (dauber frame), for the bore and bench rules.
_SW = [(r * math.cos(a), r * math.sin(a), z) for z in np.linspace(SWAB_TIP, SWAB_TOP, 6)
       for r in (0.0, SWAB_R / 2, SWAB_R) for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
SWAB_PTS = np.unique(np.round(np.array(_SW), 9), axis=0)
AXIS_PTS = np.array([(0.0, 0.0, z) for z in np.linspace(SWAB_TIP, SWAB_TOP, 11)])


def fresh():
    return dict(
        p_coats=np.zeros(NWIN, int), p_last=np.full(NWIN, -1),
        s_coats=np.zeros(NWIN, int), s_last=np.full(NWIN, -1),
        load=LOAD_AREA, load_id=0, wet=True, in_cement=True, fail=None, contamination=[],
        p_before=np.zeros(NWIN, bool), p_after=np.zeros(NWIN, bool),
        socket_seen=False, socket_inside=False, socket_exit_dip=-1,
        pass_angle={"before": None, "after": None}, pass_travel={"before": 0.0, "after": 0.0},
        credited={"pipe": 0, "socket": 0}, f_coat={"pipe": [], "socket": []}, f_max={"pipe": 0.0, "socket": 0.0}, steps=0, contact_steps=0,
    )


M = fresh()
H: dict = {}


def mat(q):
    return parts._quat_to_matrix(q)


def swab_world(pos, rot, pts):
    return np.asarray(pts) @ rot.T + pos


def in_swab(pos, rot, pts):
    loc = (np.asarray(pts) - pos) @ rot
    return (loc[:, 2] >= SWAB_TIP - EPS) & (loc[:, 2] <= SWAB_TOP + EPS) & (np.hypot(loc[:, 0], loc[:, 1]) <= SWAB_R + EPS)


# ---------------------------------------------------------------------------------------------------------------
# Contacts. On the GPU every control step reads the simulator's own contact report for the dauber against the pipe
# (with its vise), the coupling (with its vise) and the can: each contact point and its normal force. Each point is
# sorted by where it is. Offline controls supply explicitly synthetic records. contacts_from_pose() is only a geometric debugging
# estimate and never supplies paint in the simulation.
# ---------------------------------------------------------------------------------------------------------------
def classify(part, p, normal=None):
    """Which surface a contact point (env frame, m) is on: pipe, socket, pipe_bore, coupling_bore or other."""
    if part == "pipe_vise":
        r, z = float(np.hypot(p[0] - PIPE_XY[0], p[1] - PIPE_XY[1])), float(p[2])
        if r < (R_BORE + R_PIPE) / 2 and z < H_PIPE - 0.0005:
            return "pipe_bore"  # the inside of the pipe below its end
        if abs(r - R_PIPE) <= 0.004 and TOP + 0.069 <= z <= H_PIPE + 0.0005:
            return "pipe"  # the pipe's outside above the vise jaws
        return "other"  # the cut end, the vise
    if part == "coupling_vise":
        r, z = float(np.hypot(p[0] - COUP_XY[0], p[1] - COUP_XY[1])), float(p[2])
        # The wall and stop meet here: height alone mistakes a low side-wall contact for stop paint.
        # A stop-face contact has a vertical normal; cylinder wall normals are horizontal.
        if (R_BORE - 0.001 <= r <= R_COUP + 0.001 and abs(z - Z_STOP) <= 0.0008
                and (normal is None or abs(float(normal[2])) >= 0.5)):
            return "stop"
        if r < R_BORE + 0.001 and z < Z_STOP - 0.0005:
            return "coupling_bore"  # inside the stop's ring and below it
        if R_SOCK - 0.004 <= r <= R_SOCK + 0.0015 and Z_STOP + 0.0003 < z <= H_COUP + 0.0005:
            return "socket"  # the socket wall, mouth (its inner edge included) down to the stop
        return "other"  # the stop face, the rim, the outside, the vise
    if part == "bench":
        return "bench"
    return "other"  # the can


def contacts_from_pose(pos, rot):
    """Geometric debugging estimate only: indentation x SWAB_K, not a contact measurement or scoring input."""
    out = []
    ax = swab_world(pos, rot, AXIS_PTS)
    for part, xy in (("pipe_vise", PIPE_XY), ("coupling_vise", COUP_XY)):
        r = np.hypot(ax[:, 0] - xy[0], ax[:, 1] - xy[1])
        if part == "pipe_vise":
            on = (ax[:, 2] <= H_PIPE) & (r > R_PIPE)
            depth = np.where(on, R_PIPE + SWAB_R - r, -1.0)
            radius, top = R_PIPE, H_PIPE
        else:
            on = (ax[:, 2] <= H_COUP) & (ax[:, 2] >= Z_STOP) & (r < R_SOCK)
            depth = np.where(on, r + SWAB_R - R_SOCK, -1.0)
            radius, top = R_SOCK, H_COUP
        k = int(np.argmax(depth))
        if depth[k] <= 0:
            continue
        a = math.atan2(ax[k, 1] - xy[1], ax[k, 0] - xy[0])
        tip_z = swab_world(pos, rot, [(0, 0, SWAB_TIP)])[0][2]
        z = (max(tip_z, (Z_STOP if part != "pipe_vise" else H_PIPE - DEPTH)) + top) / 2
        out.append((part, np.array((xy[0] + radius * math.cos(a), xy[1] + radius * math.sin(a), z)),
                    float(SWAB_K * depth[k])))
    return out


def contact_surface(record):
    return classify(record[0], record[1], record[3] if len(record) > 3 else None)


def violations(pos, rot, can_pos, surfaces=()):
    """Visible contamination comes only from a reported wet-wool contact, never from entering empty air."""
    for surf, message in (("pipe_bore", "cement inside the pipe bore"),
                          ("coupling_bore", "cement past the stop into the bore"),
                          ("stop", "cement on the coupling stop"), ("bench", "cement on the bench")):
        if surf in surfaces:
            return message
    return None


def wool_contact(pos, rot, point):
    """Exclude lid and wire contact forces from coating and wet-wool contamination."""
    q = (np.asarray(point, float) - pos) @ rot
    return (SWAB_TIP - CONTACT_ATTR_TOL <= q[2] <= SWAB_TOP + CONTACT_ATTR_TOL
            and np.hypot(q[0], q[1]) <= SWAB_R + CONTACT_ATTR_TOL)


def socket_entry(pos, rot):
    """A coarse visible event: any of the wool enters the open socket below its mouth."""
    sw = swab_world(pos, rot, SWAB_PTS)
    return bool(np.any((np.hypot(sw[:, 0] - COUP_XY[0], sw[:, 1] - COUP_XY[1]) < R_SOCK)
                       & (sw[:, 2] < H_COUP) & (sw[:, 2] > Z_STOP)))


def apply_contacts(m, pos, rot, contacts, can_pos):
    """Advance the declared coating state by one control step (pure NumPy; used on the GPU and offline).
    contacts: (part, point, normal force N[, contact normal]) records for the swab this step."""
    m["credited"] = {"pipe": 0, "socket": 0}
    if m["fail"]:
        return
    m["steps"] += 1
    tip = swab_world(pos, rot, [(0, 0, SWAB_TIP)])[0]
    in_cement = (np.hypot(tip[0] - can_pos[0], tip[1] - can_pos[1]) < NECK_IN
                 and tip[2] < can_pos[2] - CAN_H / 2 + CEMENT_LEVEL - DIP_DEPTH)
    if in_cement and not m["in_cement"]:
        m["load_id"] += 1
    if in_cement:
        m["load"], m["wet"] = LOAD_AREA, True
    m["in_cement"] = in_cement
    # Missing / malformed / non-finite contact evidence never earns paint.
    if any(not np.isfinite(np.r_[np.asarray(r[1], float), r[2], r[3] if len(r) > 3 else []]).all()
           or r[2] < 0 for r in contacts):
        m["fail"] = "unreadable contact record"
        return
    recs = [(contact_surface(r), np.asarray(r[1], float), float(r[2]))
            for r in contacts if r[2] > 0.0 and wool_contact(pos, rot, r[1])]
    inside = socket_entry(pos, rot)
    if inside:
        m["socket_seen"] = True
        m["p_after"][:] = False  # only the pass after the final socket visit can count
        m["pass_angle"]["after"], m["pass_travel"]["after"] = None, 0.0
    elif m["socket_inside"]:
        m["socket_exit_dip"] = m["load_id"]
    m["socket_inside"] = inside
    if m["wet"]:
        v = violations(pos, rot, can_pos, {s for s, _, _ in recs})
        if v:
            m["fail"] = v
            m["contamination"] = [(s, p.tolist()) for s, p, _ in recs
                                  if s in ("pipe_bore", "coupling_bore", "stop", "bench")]
            return
    force = {"pipe": 0.0, "socket": 0.0}
    for s, _, f in recs:
        if s in force:
            force[s] += f
    if force["pipe"] > 0 or force["socket"] > 0:
        m["contact_steps"] += 1
    for surf in force:
        m["f_max"][surf] = max(m["f_max"][surf], force[surf])
    for name, surf, pts, own, area, xy, radius in (("p", "pipe", P_PTS, P_OWN, P_WIN_AREA, PIPE_XY, R_PIPE),
                                                   ("s", "socket", S_PTS, S_OWN, S_WIN_AREA, COUP_XY, R_SOCK)):
        if force[surf] < F_COAT - EPS or m["load"] <= 0 or not m["wet"]:
            continue
        if name == "p":
            phase = "before" if not m["socket_seen"] else "after"
            if phase == "before" or (not inside and m["load_id"] > m["socket_exit_dip"] >= 0):
                angle = math.atan2(pos[1] - xy[1], pos[0] - xy[0])
                previous = m["pass_angle"][phase]
                if previous is not None:
                    m["pass_travel"][phase] += (angle - previous + math.pi) % (2 * math.pi) - math.pi
                m["pass_angle"][phase] = angle
        world = pts + (xy[0], xy[1], 0.0)
        a_win = np.arctan2(pts[:, 1], pts[:, 0])
        near = np.zeros(len(pts), bool)
        for s, p, _ in recs:
            if s == surf:
                d = np.abs((a_win - math.atan2(p[1] - xy[1], p[0] - xy[0]) + math.pi) % (2 * math.pi) - math.pi)
                near |= d * radius <= FOOT_ARC + EPS
        hit = np.unique(own[in_swab(pos, rot, world) & near])
        new = hit[m[name + "_last"][hit] < m["load_id"]]
        if not len(new):
            continue
        take = min(len(new), int(math.floor(m["load"] / area + 1e-9)))
        if take <= 0:
            m["load"] = 0.0
            continue
        new = new[:take]
        m["f_coat"][surf].append(force[surf])
        m["credited"][surf] += take
        if name == "p":
            if not m["socket_seen"]:
                m["p_before"][new] = True
            elif not inside and m["load_id"] > m["socket_exit_dip"] >= 0:
                m["p_after"][new] = True
        m[name + "_coats"][new] += 1
        m[name + "_last"][new] = m["load_id"]
        m["load"] = max(0.0, m["load"] - take * area)


def coverage(m):
    return dict(pipe1=float(np.mean(m["p_coats"] >= 1)), pipe2=float(np.mean(m["p_coats"] >= 2)),
                socket=float(np.mean(m["s_coats"] >= 1)))


def force_summary(m):
    """Min / median / max total normal force (N) over the steps that left cement, per surface."""
    out = {}
    for surf, fs in m["f_coat"].items():
        out[surf] = [round(float(np.min(fs)), 3), round(float(np.median(fs)), 3), round(float(np.max(fs)), 3),
                     len(fs)] if fs else [0, 0, 0, 0]
    return out


def stowed(dauber, can):
    """Dauber back in the can: lid resting on the neck, centred within 6 mm, hanging within 10 degrees of upright."""
    r = mat(dauber.quat_xyzw)
    lid_bottom = dauber.pos[2] - LID_H / 2
    neck_top = can.pos[2] + CAN_H / 2 + NECK_H
    return (abs(lid_bottom - neck_top) <= 0.003 and np.hypot(*(dauber.pos[:2] - can.pos[:2])) <= 0.006
            and r[2, 2] >= math.cos(math.radians(10)))


def in_order(m):
    """Simulator phase masks represent whole passes before first entry and after final exit/fresh dip.
    Real grading uses the visible whole passes and final coverage photographs, not per-window repaint history."""
    return bool(m["socket_seen"] and np.all(m["p_before"]) and np.all(m["p_after"])
                and all(abs(m["pass_travel"][phase]) >= 2 * math.pi - EPS for phase in ("before", "after")))


def tier(cov, back_in_can, order=True):
    full_p1, full_p2, full_s = cov["pipe1"] >= 1 - EPS, cov["pipe2"] >= 1 - EPS, cov["socket"] >= 1 - EPS
    if full_p2 and full_s and order and back_in_can:
        return 1.0
    if full_p1 and full_s:
        return 0.75
    if (full_p1 and cov["socket"] >= 0.5 - EPS) or (full_s and cov["pipe1"] >= 0.5 - EPS):
        return 0.5
    if full_p1 or full_s:
        return 0.25
    return 0.0


# ---------------------------------------------------------------------------------------------------------------
# Hands-clear check: every face of both grippers' hulls sampled at about 3 mm.
# ---------------------------------------------------------------------------------------------------------------
@lru_cache(maxsize=1)
def face_clouds():
    from yam_arena import analysis

    result = {}
    for n in ("link4", "link5", "gripper", "tip_left", "tip_right"):
        points = []
        for hull in analysis.link_hulls()[n]:
            points.extend(hull.vertices)
            for plane in np.unique(hull.planes.round(7), axis=0):
                face = hull.vertices[np.abs(hull.vertices @ plane[:3] - plane[3]) < 2e-6]
                if len(face) < 3:
                    continue
                c = face.mean(0)
                u = face[0] - c
                u /= np.linalg.norm(u)
                v = np.cross(plane[:3], u)
                face = face[np.argsort(np.arctan2((face - c) @ v, (face - c) @ u))]
                for a, b in zip(face, np.roll(face, -1, axis=0)):
                    count = max(1, math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / 0.003))
                    for i in range(count + 1):
                        for j in range(count - i + 1):
                            points.append(c + (a - c) * i / count + (b - c) * j / count)
        cloud = np.array(points)
        _, ids = np.unique(np.floor(cloud / 0.002).astype(int), axis=0, return_index=True)
        result[n] = cloud[ids]
    return result


def hand_cloud(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    cloud = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        poses = parts._np(robot.data.body_link_pose_w)[0]
        if not np.isfinite(poses).all():
            raise ValueError("unknown hands")
        for n, p in face_clouds().items():
            q = poses[robot.body_names.index(n)]
            cloud.append(p @ mat(q[3:]).T + q[:3] - origin)
    return np.concatenate(cloud)


def gap(cloud, spec, state):
    p = (cloud - state.pos) @ mat(state.quat_xyzw)
    lo, hi = spec._local_box()
    return float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0), axis=1).min())


# ---------------------------------------------------------------------------------------------------------------
# Drawing the coat (blue dots on coated windows; darker after a second coat).
# ---------------------------------------------------------------------------------------------------------------
_stage, _dots = None, {}


def _win_centres(radius, z_low, outward):
    out = []
    for r in range(ROWS):
        for c in range(COLS):
            a = (c + 0.5) * 2 * math.pi / COLS
            rr = radius + outward
            out.append((rr * math.cos(a), rr * math.sin(a), z_low + (r + 0.5) * DEPTH / ROWS))
    return np.array(out)


P_DOTS = _win_centres(R_PIPE, H_PIPE - DEPTH, 0.0009)
S_DOTS = _win_centres(R_SOCK, Z_STOP, -0.0009)


def draw():
    global _stage, _dots
    from isaaclab.sim.utils import get_current_stage
    from pxr import Gf, UsdGeom

    stage = get_current_stage()
    if stage is not _stage:
        _stage, _dots = stage, {}
    # Visible blue traces of the same actual contacts that trigger the contamination fault.
    for i, (surface, point) in enumerate(M["contamination"]):
        k = ("contamination", i)
        entry = _dots.get(k)
        if entry is not None and entry[1] == 1:
            continue
        point = np.asarray(point, float)
        if surface in ("stop", "bench"):
            point = point + (0, 0, 0.0007)
        else:
            xy = np.asarray(PIPE_XY if surface == "pipe_bore" else COUP_XY)
            radial = point[:2] - xy
            point[:2] -= radial / max(np.linalg.norm(radial), EPS) * 0.0007
        if entry is None:
            g = UsdGeom.Sphere.Define(stage, f"/World/CementCoat/contamination{i}")
            g.CreateRadiusAttr().Set(0.0017)
            ops = UsdGeom.Xformable(g.GetPrim()).GetOrderedXformOps()
            (ops[0] if ops else g.AddTranslateOp()).Set(Gf.Vec3d(*point))
        else:
            g = entry[0]
            UsdGeom.Xformable(g.GetPrim()).GetOrderedXformOps()[0].Set(Gf.Vec3d(*point))
        UsdGeom.Imageable(g.GetPrim()).MakeVisible()
        g.CreateDisplayColorAttr().Set([Gf.Vec3f(*CEMENT)])
        _dots[k] = (g, 1)
    for key, centres, coats, xy in (("p", P_DOTS, M["p_coats"], PIPE_XY), ("s", S_DOTS, M["s_coats"], COUP_XY)):
        for i in np.flatnonzero(coats > 0):
            k = (key, int(i))
            level = min(2, int(coats[i]))
            entry = _dots.get(k)
            if entry is not None and entry[1] == level:
                continue  # only write to the stage when a window changes
            if entry is None:
                g = UsdGeom.Sphere.Define(stage, f"/World/CementCoat/{key}{int(i)}")
                g.CreateRadiusAttr().Set(0.0017)
                ops = UsdGeom.Xformable(g.GetPrim()).GetOrderedXformOps()
                (ops[0] if ops else g.AddTranslateOp()).Set(Gf.Vec3d(*(centres[i] + (xy[0], xy[1], 0.0))))
            else:
                g = entry[0]
            UsdGeom.Imageable(g.GetPrim()).MakeVisible()
            shade = (0.03, 0.16, 0.62) if level >= 2 else (0.10, 0.36, 0.95)
            g.CreateDisplayColorAttr().Set([Gf.Vec3f(*shade)])
            _dots[k] = (g, level)


# ---------------------------------------------------------------------------------------------------------------
# Simulator hooks
# ---------------------------------------------------------------------------------------------------------------
FILTERS = ("pipe_vise", "coupling_vise", "cement_can", "bench")
_VIEW = {"view": None}


def contact_view():
    """The simulator's contact report for the dauber, filtered to the pipe, the coupling and the can (made once)."""
    if _VIEW["view"] is None:
        try:
            from isaaclab_physx.physics import PhysxManager

            sv = PhysxManager.get_physics_sim_view()
        except ImportError:
            import omni.physics.tensors as tensors

            sv = tensors.create_simulation_view("torch")
            sv.set_subspace_roots("/")
        view = sv.create_rigid_contact_view(parts._PATHS["dauber"], filter_patterns=[parts._PATHS[n] for n in FILTERS],
                                            max_contact_data_count=256)
        if view.filter_count != len(FILTERS):
            raise RuntimeError(f"contact view has {view.filter_count} filters, wanted {len(FILTERS)}")
        print("[coat] contact view:", view.sensor_paths, view.filter_paths, flush=True)
        _VIEW["view"] = view
    return _VIEW["view"]


def _arr(x) -> np.ndarray:
    """A NumPy copy of a simulator buffer (torch tensor, warp array or array)."""
    x = x.torch if hasattr(x, "torch") else x
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy(), np.float64)
    return np.asarray(x, np.float64)


def read_contacts(sim):
    """(part, point in the env frame, normal force N) for every contact the simulator reports between the dauber and
    the pipe, the coupling or the can at the last physics step. Fails closed (ValueError) on unreadable data."""
    view = contact_view()
    dt = float(sim.env.unwrapped.physics_dt)
    forces, points, normals, _seps, counts, starts = view.get_contact_data(dt)
    forces = _arr(forces).reshape(-1)
    points = _arr(points).reshape(-1, 3)
    normals = _arr(normals).reshape(-1, 3)
    counts = _arr(counts).reshape(-1).astype(int)
    starts = _arr(starts).reshape(-1).astype(int)
    if len(counts) != len(FILTERS) or len(starts) != len(FILTERS):
        raise ValueError("contact buffer shape")
    origin = parts._np(sim.env.unwrapped.scene.env_origins[0])
    out = []
    for i, name in enumerate(FILTERS):
        a, n = int(starts[i]), int(counts[i])
        if n < 0 or a < 0 or a + n > len(forces):
            raise ValueError("contact buffer")
        f, pts, ns = forces[a:a + n], points[a:a + n], normals[a:a + n]
        if not (np.isfinite(f).all() and np.isfinite(pts).all() and np.isfinite(ns).all()):
            raise ValueError("contact data not finite")
        out += [(name, pts[k] - origin, float(f[k]), ns[k]) for k in range(n)]
    return out


@parts.on_reset
def reset(sim):
    global M, H, _stage, _dots
    from isaaclab.app.settings_manager import get_settings_manager
    from isaaclab.sim.utils import get_current_stage

    from pxr import Gf, UsdGeom

    # Keep stable visual prim identities. Removing runtime USD prims left stale rendered geometry.
    # Collapse and hide every old mark by path; draw() restores only marks earned by the new specimen.
    stage = get_current_stage()
    root = stage.GetPrimAtPath("/World/CementCoat")
    hidden = 0
    if root.IsValid():
        for prim in root.GetChildren():
            if prim.GetTypeName() != "Sphere":
                continue
            geom = UsdGeom.Sphere(prim)
            geom.GetRadiusAttr().Set(0.000001)
            UsdGeom.Imageable(prim).MakeInvisible()
            ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
            if ops:
                ops[0].Set(Gf.Vec3d(0.0, 0.0, -10.0))
            hidden += 1
    print(f"[coat] reset paint: collapsed_hidden_marks={hidden}", flush=True)
    _stage, _dots = stage, {}
    _VIEW["view"] = None  # rebind contact buffers after every physics reset
    get_settings_manager().set_bool("/physics/disableContactProcessing", False)
    M = fresh()
    H = dict(known=False, bad=None, t=0.0, released=0.0, start=None, log_k=0)


@parts.on_step
def step(sim):
    states = sim.states()
    dt = float(sim.t) - H.get("t", 0.0)
    H["t"] = float(sim.t)
    contacts = []
    try:
        d, can = states["dauber"], states["cement_can"]
        if not np.isfinite(np.r_[d.pos, d.quat_xyzw, can.pos, can.quat_xyzw]).all() or not 0 < dt <= 0.2:
            raise ValueError("unknown state")
        if H["start"] is None:
            H["start"] = can.pos.copy()
        if mat(can.quat_xyzw)[2, 2] < math.cos(math.radians(15)):
            H["bad"] = H["bad"] or "can tipped (spill)"
        if np.hypot(*(can.pos[:2] - H["start"][:2])) > 0.020:
            H["bad"] = H["bad"] or "can knocked across the bench"
        contacts = read_contacts(sim)
        apply_contacts(M, d.pos, mat(d.quat_xyzw), contacts, can.pos)
        still = d.resting(0.003, 0.03) and can.resting(0.003, 0.03)
        clear = False
        if still:
            cloud = hand_cloud(sim)
            clear = min(gap(cloud, DAUBER, d), gap(cloud, CAN, can)) >= 0.030
        else:  # fail closed if the hands cannot be read
            for arm in ("left_robot", "right_robot"):
                if not np.isfinite(parts._np(sim.env.unwrapped.scene[arm].data.body_link_pose_w)).all():
                    raise ValueError("unknown hands")
        H["released"] = H["released"] + dt if (still and clear) else 0.0
        H["known"] = True
        draw()
    except (ValueError, KeyError, IndexError, AttributeError, RuntimeError) as e:
        H["known"] = False
        H["bad"] = H["bad"] or f"unreadable: {e}"
    # A one-line record every second (in run.txt): coverage and the coating forces so far, per surface.
    H["log_k"] = H.get("log_k", 0) + 1
    if H["log_k"] % 20 == 0:
        print(f"[coat] lot {lots.current().name} t={float(sim.t):.1f} cov={coverage(M)} load_id={M['load_id']} "
              f"fail={M['fail']} bad={H['bad']} f_max={ {k: round(v, 3) for k, v in M['f_max'].items()} } "
              f"coat_force_min_med_max_n={force_summary(M)} "
              f"phase_degrees={ {k: round(math.degrees(v), 1) for k, v in M['pass_travel'].items()} } "
              f"phase_coverage={[float(np.mean(M[k])) for k in ('p_before', 'p_after')]}", flush=True)
    if os.environ.get("COAT_FORCE_LOG"):  # optional per-step record of the reported coating forces
        force = {"pipe": 0.0, "socket": 0.0}
        for record in contacts:
            key = contact_surface(record)
            if key in force and wool_contact(d.pos, mat(d.quat_xyzw), record[1]):
                force[key] += record[2]
        with open(os.environ["COAT_FORCE_LOG"], "a") as fh:
            fh.write(json.dumps(dict(lot=lots.current().name, t=round(float(sim.t), 3), pipe_n=round(force["pipe"], 4),
                                     socket_n=round(force["socket"], 4), cov=coverage(M), load_id=M["load_id"],
                                     credited=M["credited"], phase_travel=M["pass_travel"], known=H["known"],
                                     fail=M["fail"], bad=H["bad"], score=score(states),
                                     dauber_pos=d.pos.tolist() if H["known"] else None,
                                     dauber_quat=d.quat_xyzw.tolist() if H["known"] else None,
                                     can_pos=can.pos.tolist() if H["known"] else None)) + "\n")
    if os.environ.get("TRACE_OUT"):
        robot = sim.env.unwrapped.scene["right_robot"]
        poses = parts._np(robot.data.body_link_pose_w)[0]
        origin = parts._np(sim.env.unwrapped.scene.env_origins[0])
        g = poses[robot.body_names.index("gripper")]
        tcp = mat(g[3:]) @ np.array((0.0, 0.0, 0.134)) + g[:3] - origin
        dd = states["dauber"]
        rot = mat(dd.quat_xyzw)
        tip = swab_world(dd.pos, rot, [(0, 0, SWAB_TIP)])[0]
        geo = {part: round(f / SWAB_K * 1000, 3) for part, _, f in contacts_from_pose(dd.pos, rot)}  # mm indentation
        by = {}
        for record in contacts:
            part, pt, f = record[:3]
            key = contact_surface(record) if part != "cement_can" else "can"
            by.setdefault(key, [0, 0.0])
            by[key][0] += 1
            by[key][1] = round(by[key][1] + f, 4)
        row = dict(t=float(sim.t), lot=lots.current().name, tcp=tcp.tolist(), dauber=dd.pos.tolist(),
                   quat=dd.quat_xyzw.tolist(), tip=tip.tolist(), indent_mm=geo, contacts=by,
                   can=states["cement_can"].pos.tolist(),
                   fingers={n: float(parts._np(robot.data.joint_pos)[0, i])
                            for i, n in enumerate(robot.joint_names) if n in ("joint7", "joint8")},
                   pts=[[r[0], [round(float(v), 5) for v in r[1]], round(r[2], 4), r[3].tolist()] for r in contacts if r[2] > 0][:24],
                   cov=coverage(M), load=M["load"], load_id=M["load_id"], fail=M["fail"], bad=H["bad"],
                   released=H["released"], score=score(states))
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("coating.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


def score(objects):
    try:
        if not H.get("known") or H.get("bad") or M["fail"] or H["released"] < 1.0 - 1e-9:
            return 0.0
        return tier(coverage(M), stowed(objects["dauber"], objects["cement_can"]), in_order(M))
    except (KeyError, TypeError, ValueError):
        return 0.0


def success(objects):
    return score(objects) == 1.0


# ---------------------------------------------------------------------------------------------------------------
# Scripted demo: open loop from the published layout (pipe, coupling and can positions as a camera sees them).
# ---------------------------------------------------------------------------------------------------------------
GRIP_ABOVE_LID = 0.007  # TCP above the lid centre: pad tips 2 mm above the lid's lower edge
TCP_TO_TIP = GRIP_ABOVE_LID - SWAB_TIP  # 85 mm
FINGER_LID = -(LID_R - 0.008)  # close 8 mm past the lid's sides: about 40 N of grip (5,000 N/m finger drive, 50 N limit)
# Fixed corrections measured once in a trace (arm sag with the dauber held; not runtime feedback).
SAG_Z = 0.0004  # trace 1: the swab end ran 0.4 mm low
PIPE_FIX = (-0.0005, 0.0005)  # traces 1-2: the swab's orbit round the pipe ran about 0.5 mm off plan
SOCK_FIX = (-0.0004, -0.0002)  # traces 2 (lot A) and 7 (lot B): 0.2-0.7 mm off plan
INDENT = 0.0025  # planned indentation of the swab into the pipe band and the socket wall
# The arm gives way sideways under a light load (a static press test: commanding the swab 5 mm into the pipe gave
# 0.3-0.45 N with 0.9 mm of actual indentation), so the path is planned this much further in than the swab goes.
# These path corrections do not enter the scorer or its synthetic controls.
ARM_GIVE_PIPE = 0.0035  # commanded 6.0 mm in
ARM_GIVE_SOCK = 0.0005  # commanded 3.0 mm out
PICK_FIX_B = (0.0011, -0.0010)  # trace: open TCP at B was 1.1 mm x-low, 1.0 mm y-high
STOW_FIX_B = (0.0040, 0.0)  # bracketed return: x correction 0 gave -7.33 mm; +7.3 gave +6.03 mm lid error
HUB = (0.215, -0.290)  # a point clear of the pipe and coupling, used when travelling to and from the socket


def demo():
    lot = lots.current()
    can = np.asarray(lot.pos("cement_can"))
    lid = np.asarray(lot.pos("dauber"))
    neck_top = can[2] + CAN_H / 2 + NECK_H
    f = FINGER_LID
    W = []

    def wp(t, xy, tcp_z, finger=f, label="", fix=(0.0, 0.0)):
        x = float(xy[0] + fix[0])
        y = float(xy[1] + fix[1])
        W.append(Waypoint(float(t), (x, y, float(tcp_z + SAG_Z)), float(finger), label, 0.0))

    def tcp_for_tip(tip_z):
        return tip_z + TCP_TO_TIP

    carry = tcp_for_tip(neck_top + 0.006)  # swab tip 6 mm above the can's neck
    lid_tcp = lid[2] + GRIP_ABOVE_LID

    # 1. take the dauber by its lid and lift it straight out of the can
    pick_fix = PICK_FIX_B if lot.name == "B" else (0.0, 0.0)
    wp(2.5, lid[:2], lid_tcp + 0.05, FINGER_OPEN, "above the lid", pick_fix)
    wp(2.0, lid[:2], lid_tcp, FINGER_OPEN, "fingers round the lid", pick_fix)
    if lot.name == "B":
        # Touch both sides gently before applying the full grip; the light lid must not be kicked sideways.
        wp(1.0, lid[:2], lid_tcp, -LID_R, "touch both sides of the lid", pick_fix)
        wp(1.5, lid[:2], lid_tcp, f, "grip the lid", pick_fix)
        wp(3.0, lid[:2], carry, f, "lift the dauber out of the can", pick_fix)
    else:
        wp(1.0, lid[:2], lid_tcp, f, "grip the lid")
        wp(2.5, lid[:2], carry, f, "lift the dauber out of the can")

    def dip(label):
        wp(2.0, can[:2], carry, f, f"over the can ({label})")
        wp(1.5, can[:2], lid_tcp + 0.003, f, f"dip ({label})")
        wp(1.0, can[:2], lid_tcp + 0.003, f, f"soak ({label})")
        wp(2.0, can[:2], carry, f, f"lift out ({label})")

    px, py = PIPE_XY
    band_tcp = tcp_for_tip(H_PIPE - DEPTH - 0.0034)  # swab spans the band with about 3 mm to spare each end
    r_pipe = R_PIPE + SWAB_R - INDENT - ARM_GIVE_PIPE
    a0 = math.radians(200)  # start on the side facing the arm's base, away from the coupling

    def pipe_coat(label):
        start = (px + (r_pipe + 0.012) * math.cos(a0), py + (r_pipe + 0.012) * math.sin(a0))
        wp(2.5, start, carry, f, f"beside the pipe ({label})")
        wp(1.5, start, band_tcp, f, f"down beside the band ({label})")
        for k, a in enumerate(np.linspace(a0, a0 + math.radians(390), 27)):
            wp(1.2 if k == 0 else 0.35, (px + r_pipe * math.cos(a), py + r_pipe * math.sin(a)), band_tcp, f,
               f"round the band ({label})", PIPE_FIX)
        a1 = a0 + math.radians(390)
        out = (px + (r_pipe + 0.012) * math.cos(a1), py + (r_pipe + 0.012) * math.sin(a1))
        wp(0.8, out, band_tcp, f, f"off the band ({label})")
        wp(1.5, out, carry, f, f"up ({label})")

    cx, cy = COUP_XY
    rim_tcp = tcp_for_tip(H_COUP + 0.004)
    sock_tcp = tcp_for_tip(Z_STOP + 0.0024)  # swab end 2.4 mm above the stop as planned; traces ran 0.5-1 mm low
    r_sock = R_SOCK - SWAB_R + INDENT + ARM_GIVE_SOCK
    b0 = math.radians(270)

    def socket_coat():
        wp(1.2, can[:2], rim_tcp, f, "up to rim height")
        wp(1.5, HUB, rim_tcp, f, "round the pipe vise")
        wp(2.5, (cx, cy), rim_tcp, f, "over the socket")
        wp(1.5, (cx, cy), sock_tcp + 0.003, f, "down into the socket")  # 3 mm higher while over the bore
        for k, a in enumerate(np.linspace(b0, b0 + math.radians(390), 27)):
            wp(1.0 if k == 0 else 0.35, (cx + r_sock * math.cos(a), cy + r_sock * math.sin(a)), sock_tcp, f,
               "round the socket wall", SOCK_FIX)
        wp(1.0, (cx, cy), sock_tcp + 0.004, f, "back to the centre")
        wp(1.5, (cx, cy), rim_tcp, f, "up out of the socket")
        wp(2.5, HUB, rim_tcp, f, "back round the pipe vise")

    pipe_coat("first coat")
    dip("for the socket")
    socket_coat()
    dip("second coat")
    pipe_coat("second coat")
    # put the dauber back: lid on the neck, let go, hands clear
    stow_fix = STOW_FIX_B if lot.name == "B" else (0.0, 0.0)
    wp(2.5, can[:2], carry, f, "over the can", stow_fix)
    wp(2.0, can[:2], lid_tcp + 0.002, f, "lid onto the neck", stow_fix)
    wp(1.0, can[:2], lid_tcp + 0.002, FINGER_OPEN, "let go", stow_fix)
    wp(1.5, can[:2], lid_tcp + 0.06, FINGER_OPEN, "hands clear", stow_fix)
    wp(4.0, can[:2], lid_tcp + 0.06, FINGER_OPEN, "still", stow_fix)
    return {"right": W}
