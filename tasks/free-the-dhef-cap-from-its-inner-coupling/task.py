"""On the station table: a Universal Robots UR5e arm, powered off with its brakes on, stands at the far left of the
table with its wrist folded low, so that the Festo DHEF-20 adaptive shape gripper on its tool flange lies level and
points across the table toward the right-hand arm. The gripper's white silicone inverting cap is on its nose, held
by a black knurled bayonet catch ring with a white index line; the gripper body has a white "locked" mark in line
with that index line and a second white "open" mark a short turn away. The gripper's air is exhausted. A grey
anti-static service mat lies on the table in front of the right-hand arm.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402  (before lots, so parts' reset hooks run before the lots' moves)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
DEMO_TOUCHES = ("dhef",)  # offline check: the fingers straddle the catch ring and the releasing ring open
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
EPS = 1e-6
MM = 0.001

UR_GREY, UR_CAP, DARK = (0.80, 0.81, 0.82), (0.30, 0.33, 0.37), (0.16, 0.16, 0.17)
ALU, ALU_DARK, BLACK = (0.66, 0.68, 0.70), (0.45, 0.47, 0.50), (0.07, 0.07, 0.08)
SILICONE, SILICONE_IN = (0.90, 0.90, 0.86), (0.80, 0.80, 0.75)
WHITE, STEEL, MAT_GREY = (0.97, 0.97, 0.97), (0.74, 0.75, 0.77), (0.36, 0.42, 0.48)
SLEEVE_BLUE = (0.18, 0.35, 0.62)

# -- where the gripper is ------------------------------------------------------------------------------------------
# The UR5e holds the DHEF level, its axis along world y, the cap pointing to -y (toward the right-hand arm). The
# gripper's own frame ("local"): x' along its axis, pointing from the cap toward the robot flange; y' across; z up
# from the table top. Turned +90 deg about z into the world, local (x', y', z) sits at world
# (X_AX - y', Y_F + x', TOP + z). u is the distance from the flange face toward the cap tip: x' = -u.
X_AX = 0.270  # world x of the gripper's axis
Y_F = 0.165  # world y of the gripper's mounting face (the UR5e tool flange)
H_AX = 0.080  # axis height above the table
Z_AX = TOP + H_AX
YAW = math.pi / 2
AXIS_W = np.array([0.0, -1.0, 0.0])  # world direction from the flange toward the cap tip (+u)


def to_world(p) -> np.ndarray:
    p = np.asarray(p, float)
    return np.array([X_AX - p[1], Y_F + p[0], TOP + p[2]])


def cyl(u, r=0.0, th=0.0) -> np.ndarray:
    """A gripper-local point at distance u (m) from the flange face, radius r off the axis, angle th (rad, 0 along
    +y', pi/2 straight up)."""
    return np.array([-u, r * math.cos(th), H_AX + r * math.sin(th)])


def _qx(angle) -> tuple[float, float, float, float]:
    return (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))


def seg(u0, u1, r0, r1, th, dth, color, org=np.zeros(3), collide=True):
    """One box of a ring: axial u0..u1, radial r0..r1, centred on angle th, dth wide (closes the ring at r1)."""
    w = 2 * r1 * math.tan(dth / 2)
    c = cyl((u0 + u1) / 2, (r0 + r1) / 2, th) - org
    return parts.box((u1 - u0, w, r1 - r0), at=tuple(c), color=color, quat=_qx(th - math.pi / 2), collide=collide)


def ring(u0, u1, r0, r1, n, color, org=np.zeros(3), collide=True, start=0.0, colors=None):
    dth = 2 * math.pi / n
    return [seg(u0, u1, r0, r1, start + k * dth, dth, (colors[k % len(colors)] if colors else color), org, collide)
            for k in range(n)]


def arc(u0, u1, r0, r1, th0, th1, color, org=np.zeros(3), max_step=math.radians(10)):
    n = max(1, int(math.ceil((th1 - th0) / max_step)))
    dth = (th1 - th0) / n
    return [seg(u0, u1, r0, r1, th0 + (k + 0.5) * dth, dth, color, org) for k in range(n)]


def xcyl(u0, u1, r, color, org=np.zeros(3), collide=True):
    """A cylinder on the axis from u0 to u1."""
    return parts.cylinder(r, u1 - u0, at=tuple(cyl((u0 + u1) / 2) - org), color=color, axis="x", collide=collide)


def xprism(u0, u1, r, color, org=np.zeros(3), collide=True, sides=24):
    return parts.prism(r, u1 - u0, sides, at=tuple(cyl((u0 + u1) / 2) - org), color=color, collide=collide)


# -- the UR5e, from its published joint geometry (DH: d1 162.5, a2 425, a3 392.2, d4 133.3, d5 99.7, d6 99.6 mm),
# folded so that wrist 2's axis is vertical and wrist 3's is the gripper's axis; braked, so it is one fixed body.
UR_D1, UR_A2, UR_A3, UR_D4, UR_D5, UR_D6 = 0.1625, 0.425, 0.3922, 0.1333, 0.0997, 0.0996
UR_LATERAL = (0.138, 0.007, 0.134)  # upper arm, forearm and wrist 1 planes, out from the base axis along the joints
W2 = np.array([UR_D6, 0.0, H_AX])  # wrist 3's centre on the wrist-2 axis (vertical), behind the flange
W1 = W2 + np.array([0.0, 0.0, UR_D5])
UR_BACK = 0.280  # the base stands 280 mm beyond the wrist (clear of the left-hand arm's start pose)
UR_SIDE = 1.0
UR_BASE = np.array([W2[0] + UR_BACK, -UR_SIDE * UR_D4, 0.0])


def _ur_elbow():
    s = np.array([UR_BASE[0], UR_BASE[2] + UR_D1])
    p = np.array([W1[0], W1[2]])
    d = p - s
    L = float(np.linalg.norm(d))
    alpha = math.acos((UR_A2 ** 2 + L ** 2 - UR_A3 ** 2) / (2 * UR_A2 * L))
    phi = math.atan2(d[1], d[0]) - alpha
    return s, s + UR_A2 * np.array([math.cos(phi), math.sin(phi)]), p


SHOULDER, ELBOW, WRIST1 = _ur_elbow()


def _q_to(direction):
    return tuple(float(v) for v in parts._matrix_to_quat(parts._x_to(direction)))


def _tube(a, b, radius, color):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return parts.prism(radius, float(np.linalg.norm(b - a)), 20, at=tuple((a + b) / 2), color=color, quat=_q_to(b - a))


def _lcyl(center, radius, length, axis, color):
    return parts.cylinder(radius, length, at=tuple(np.asarray(center, float)), color=color, axis=axis)


def _housing(center, radius, length, axis, cap_end, cap_r, cap_t=0.006):
    c = np.asarray(center, float)
    k = "xyz".index(axis)
    capc = c.copy()
    capc[k] += cap_end * (length / 2 + cap_t / 2)
    return [_lcyl(c, radius, length, axis, UR_GREY), _lcyl(capc, cap_r, cap_t, axis, UR_CAP)]


def _ur_shapes():
    sx, sz = SHOULDER
    ex, ez = ELBOW
    wx, wz = WRIST1
    y_ua, y_fa, y_w1 = (UR_BASE[1] + UR_SIDE * v for v in UR_LATERAL)
    b = UR_BASE
    return [
        _lcyl((b[0], b[1], 0.006), 0.0745, 0.012, "z", DARK),  # mounting flange, 149 mm
        _lcyl((b[0], b[1], 0.012 + 0.050), 0.0575, 0.100, "z", UR_GREY),  # base joint
        *_housing((b[0], b[1] + UR_SIDE * 0.065, sz), 0.0575, 0.245, "y", UR_SIDE, 0.050),  # shoulder
        _tube((sx, y_ua, sz), (ex, y_ua, ez), 0.0445, UR_GREY),  # upper arm
        *_housing((ex, (y_ua + y_fa) / 2, ez), 0.0480, 0.215, "y", UR_SIDE, 0.042),  # elbow
        _tube((ex, y_fa, ez), (wx, y_fa, wz), 0.0375, UR_GREY),  # forearm
        *_housing((wx, (y_fa + y_w1) / 2 + UR_SIDE * 0.005, wz), 0.0400, 0.205, "y", -UR_SIDE, 0.035),  # wrist 1
        *_housing((W2[0], 0.0, (W2[2] + W1[2]) / 2 + 0.010), 0.0400, UR_D5 + 0.055, "z", 1, 0.035),  # wrist 2
        _lcyl(((W2[0] + 0.040) / 2 + 0.003, 0.0, H_AX), 0.0375, W2[0] + 0.040 - 0.006, "x", UR_GREY),  # wrist 3
        _lcyl((0.003, 0.0, H_AX), 0.0315, 0.006, "x", ALU),  # tool flange, 63 mm
    ]


# -- the Festo DHEF-20-A-RA1 (catalogue 2021/06 p.11: 224 mm long, 62.5 mm flange, 57.5 mm catch ring, 82.5 mm cap;
# internal bayonet, rod and coupling sizes are estimates). Distances u from the mounting face, mm in comments. -------
U_BODY_FRONT = 0.1165  # front face of the body's locking cover; the catch ring sits in front of it
U_NOSE = 0.128  # front of the nose inside the catch ring
R_RING_IN, R_RING_OUT = 0.0255, 0.02875  # catch ring wall (57.5 mm across)
U_RING0, U_RING1 = 0.117, 0.1415  # catch ring, back and front faces; its front is the cap's rim
R_NOSE, R_LIP = 0.019, 0.024
U_LIP0, U_LIP1 = 0.1215, 0.1255  # bayonet lip on the nose; the groove behind it is 116.5..121.5
R_LUG0 = 0.021
U_LUG0, U_LUG1 = 0.117, 0.121
LUG_W = math.radians(18)
GAP_W = math.radians(50)
TURN = math.radians(45)  # the catch turns 45 deg from locked to open
LOCK_TH = [math.radians(90 + 120 * k) for k in range(3)]  # lug centres when locked (one lug straight up)
OPEN_TH = [t - TURN for t in LOCK_TH]  # and when open (each lines up with a gap in the lip)
STOP_W = math.radians(8)


def _body_shapes():
    out = _ur_shapes()
    out += [
        xprism(0.000, 0.012, 0.03125, ALU),  # mounting flange, 62.5 mm (D1)
        xprism(0.012, 0.104, 0.0225, ALU),  # body tube with its sensor slots (about 45 mm across)
        xprism(0.104, 0.110, 0.0255, DARK),  # locking cover
        xprism(0.110, U_BODY_FRONT, R_RING_OUT, DARK),
        xprism(U_BODY_FRONT, U_NOSE, R_NOSE, DARK),  # nose inside the catch ring
    ]
    for k in range(3):  # bayonet lip: three arcs, each gap lined up with a lug's open position
        g0, g1 = OPEN_TH[k] + GAP_W / 2, OPEN_TH[(k + 1) % 3] - GAP_W / 2
        if g1 < g0:
            g1 += 2 * math.pi
        out += arc(U_LIP0, U_LIP1, R_NOSE - 0.0005, R_LIP, g0, g1, DARK)
        # stops in the groove: one past the open position, one past the locked position
        out.append(seg(U_BODY_FRONT, U_LIP0, R_NOSE - 0.0005, R_LIP, OPEN_TH[k] - LUG_W / 2 - 0.5 * STOP_W
                       - math.radians(3), STOP_W, DARK))
        out.append(seg(U_BODY_FRONT, U_LIP0, R_NOSE - 0.0005, R_LIP, LOCK_TH[k] + LUG_W / 2 + 0.5 * STOP_W
                       + math.radians(3), STOP_W, DARK))
    # marks on the locking cover, just behind the ring: "locked" in line with the ring's index line, "open" 45 deg on
    for th, w in ((LOCK_TH[0], 0.0035), (OPEN_TH[0], 0.0035)):
        out.append(seg(0.1095, 0.1160, R_RING_OUT - 0.0002, R_RING_OUT + 0.0006, th, w / R_RING_OUT, WHITE,
                       collide=False))
    out.append(seg(0.1030, 0.1095, R_RING_OUT - 0.0002, R_RING_OUT + 0.0006, OPEN_TH[0], 0.0090 / R_RING_OUT, WHITE,
                   collide=False))  # the open mark is a wider bar (an open-lock symbol on the real cover)
    return out


# the piston rod (exhausted: it floats, with seal friction) and its coupling at the front ------------------------
R_ROD = 0.005
U_CPL0, U_POCKET0, U_POCKET1, U_CPL1 = 0.188, 0.201, 0.212, 0.216  # coupling: back, pocket, keepers' back, front
R_CPL = 0.0095
POCKET_HALF = 0.007  # square pocket 14 x 14 mm for the cap's stud head
U_SLEEVE0, U_SLEEVE1 = 0.187, 0.2005
R_SLEEVE = 0.011
ROD_TRAVEL = 0.055  # m the rod may be pushed back into the body (the model's limit; real stroke 66 mm)
SLEEVE_TRAVEL = 0.003  # the releasing ring's travel toward the rod
KEEP_OPEN = 0.004  # each keeper opens 4 mm


def _rod_shapes():
    return [
        xcyl(0.060, U_CPL0, R_ROD, STEEL),  # piston rod, 10 mm
        xprism(0.182, U_CPL0, 0.0065, ALU_DARK, collide=False, sides=6),  # hex flat on the rod
        xcyl(U_CPL0, U_POCKET0, R_CPL, ALU),  # coupling body behind the pocket
        # the pocket's walls (inside the coupling's round outline): floor, ceiling and sides
        parts.box((U_POCKET1 - U_POCKET0, 2 * R_CPL, R_CPL - POCKET_HALF),
                  at=tuple(cyl((U_POCKET0 + U_POCKET1) / 2) + [0, 0, -(R_CPL + POCKET_HALF) / 2]), color=ALU),
        parts.box((U_POCKET1 - U_POCKET0, 2 * R_CPL, R_CPL - POCKET_HALF),
                  at=tuple(cyl((U_POCKET0 + U_POCKET1) / 2) + [0, 0, (R_CPL + POCKET_HALF) / 2]), color=ALU),
        *[parts.box((U_CPL1 - U_POCKET0, R_CPL - POCKET_HALF, 2 * POCKET_HALF),
                    at=tuple(cyl((U_POCKET0 + U_CPL1) / 2) + [0, s * (R_CPL + POCKET_HALF) / 2, 0]), color=ALU)
          for s in (-1, 1)],
    ]


def _keeper_shapes(s):
    """The coupling's latch, drawn as two keepers (top s=+1, bottom s=-1) in front of the stud's head."""
    z0, z1 = 0.0035, R_CPL
    return [parts.box((U_CPL1 - U_POCKET1, 2 * (POCKET_HALF - 0.0015), z1 - z0),
                      at=tuple(cyl((U_POCKET1 + U_CPL1) / 2) + [0, 0, s * (z0 + z1) / 2]), color=ALU)]


ROD = parts.Slider("rod", shapes=tuple(_rod_shapes()), axis=(1.0, 0.0, 0.0), travel=ROD_TRAVEL, mass=0.060,
                   spring=10000.0, damping=10.0, max_effort=15.0)
SLEEVE = parts.Slider("sleeve", shapes=(xcyl(U_SLEEVE0, U_SLEEVE1, R_SLEEVE, SLEEVE_BLUE),), axis=(1.0, 0.0, 0.0),
                      travel=SLEEVE_TRAVEL, mass=0.010, spring=1000.0, damping=2.0, max_effort=20.0, parent="rod")
KEEP_TOP = parts.Slider("keep_top", shapes=tuple(_keeper_shapes(1)), axis=(0.0, 0.0, 1.0), travel=KEEP_OPEN,
                        mass=0.008, spring=5000.0, damping=5.0, max_effort=40.0, parent="rod")
KEEP_BOT = parts.Slider("keep_bot", shapes=tuple(_keeper_shapes(-1)), axis=(0.0, 0.0, -1.0), travel=KEEP_OPEN,
                        mass=0.008, spring=5000.0, damping=5.0, max_effort=40.0, parent="rod")
DHEF_ORIGIN_W = (X_AX, Y_F, TOP)
DHEF = parts.Part("dhef", pos=DHEF_ORIGIN_W, yaw=YAW, shapes=tuple(_body_shapes()),
                  joints=(ROD, SLEEVE, KEEP_TOP, KEEP_BOT), friction=(0.8, 0.7),
                  link_friction=(("sleeve", (1.2, 1.0)),), contact_offset=0.0015)

# -- the cap assembly: catch ring + threaded bush + knurled nut (one rigid carrier) and the silicone cap, whose tip
# carries the stud that the rod's coupling holds. Its own frame: origin on the axis at the rim (u 141.5 mm). -------
U_RIM = U_RING1
CAP_ORG = cyl(U_RIM)  # gripper-local
U_HEAD0, U_HEAD1, U_NECK1, U_TIP1 = 0.2015, 0.2115, 0.2195, 0.224
R_HEAD, R_NECK, R_TIP = 0.0065, 0.003, 0.0125
V_TIP0 = U_NECK1 - U_RIM  # the tip's inner face is 78 mm in front of the rim
SHELL_LEN = 0.082  # the cap wall's length along its axis
R_CAP_RIM, R_CAP_TIP = 0.0205, 0.0125  # 41 mm at the rim (D3), 25 mm at the tip (D4)
S_FULL = 0.1325  # fully everted: the tip 55 mm behind the rim (30 mm behind the ring's back face)
S_STRETCH_MAX = 0.015  # the joint's hard end, 15 mm past full eversion (the silicone stretching)
Q_EVERTED = 0.1225  # tier: tip at least 20 mm behind the ring's back face
U_BUSH1 = U_RIM + 0.005  # the bush's collar, with the cap's rim over it


def _carrier_shapes():
    o = CAP_ORG
    knurl = [BLACK, (0.15, 0.15, 0.16)]
    out = ring(U_RING0, U_RING1, R_RING_IN, R_RING_OUT, 24, BLACK, o, colors=knurl)  # knurled catch ring
    out += ring(U_RING1 - 0.005, U_RING1, 0.015, R_RING_IN, 16, BLACK, o)  # its front wall, onto the bush
    out += ring(U_RIM, U_BUSH1, 0.015, 0.0185, 12, ALU, o)  # bush collar (the cap's rim is stretched over it)
    out += ring(0.130, U_RING1 - 0.005, 0.015, 0.019, 12, ALU_DARK, o, collide=False)  # knurled nut, inside
    for th in LOCK_TH:  # bayonet lugs inside the ring's back end
        out.append(seg(U_LUG0, U_LUG1, R_LUG0, R_RING_IN + 0.0005, th, LUG_W, BLACK, o))
    out.append(seg(U_RING0 + 0.0005, U_RING1 - 0.0005, R_RING_OUT - 0.0002, R_RING_OUT + 0.0006, LOCK_TH[0],
                   0.0035 / R_RING_OUT, WHITE, o, collide=False))  # index line
    out.append(xcyl(U_RIM, U_BUSH1, R_CAP_RIM, SILICONE, o, collide=False))  # the cap's rim bead over the collar
    return out


def _tip_shapes():
    o = CAP_ORG
    return [
        xprism(U_HEAD0, U_HEAD1, R_HEAD, STEEL, o, sides=16),  # stud head (moulded into the tip)
        xcyl(U_HEAD1, U_NECK1, R_NECK, STEEL, o),  # stud neck
        xcyl(U_NECK1, U_TIP1, R_TIP, SILICONE, o, collide=False),  # the cap's tip (silicone)
    ]


# the cap wall's shape at eversion s (m the tip has moved back relative to the rim), drawn as one of N_STAGES meshes
N_STAGES = 17
STAGE_Q = [S_FULL * k / (N_STAGES - 1) for k in range(N_STAGES)]
N_SIDES = 20


def shell_profile(s):
    """(v, r) points along the cap wall from the rim forward to the fold and back to the tip; v is the distance in
    front of the rim, r the radius. Rolling: the fold sits halfway between the wall's length and the tip."""
    vt = V_TIP0 - s
    vf = max((SHELL_LEN + vt) / 2, vt + 0.002)

    def r_out(v):
        return R_CAP_RIM - (R_CAP_RIM - R_CAP_TIP) * min(max(v, 0.0), V_TIP0) / V_TIP0

    pts = [(0.005 + (vf - 0.005) * k / 6, r_out(0.005 + (vf - 0.005) * k / 6)) for k in range(7)]
    rf = r_out(vf)
    ri_f = rf - 0.003
    for a in (0.5, 1.0):  # round over the fold
        pts.append((vf + 0.0015 * math.sin(math.pi * a), rf - (rf - ri_f) * (1 - math.cos(math.pi * a)) / 2))
    for k in range(1, 9):
        v = vf + (vt - vf) * k / 8
        r = ri_f + (R_CAP_TIP - 0.001 - ri_f) * k / 8
        if v < 0.006:
            r = min(r, 0.0135)  # through the bush's 30 mm bore
        pts.append((v, r))
    return pts


def _shell_visual(k):
    prof = shell_profile(STAGE_Q[k])
    pts, tris = [], []
    for v, r in prof:
        for j in range(N_SIDES):
            th = 2 * math.pi * j / N_SIDES
            pts.append(tuple(float(x) for x in (np.array([-v, r * math.cos(th), r * math.sin(th)]))))
    for i in range(len(prof) - 1):
        for j in range(N_SIDES):
            a, b = i * N_SIDES + j, i * N_SIDES + (j + 1) % N_SIDES
            c, d = a + N_SIDES, b + N_SIDES
            tris += [(a, b, d), (a, d, c)]
    return parts.Visual(f"shell{k}", tuple(pts), tuple(tris), color=SILICONE, link="base", visible=(k == 0))


EVERT = parts.Slider("evert", shapes=tuple(_tip_shapes()), axis=(1.0, 0.0, 0.0), travel=S_FULL + S_STRETCH_MAX,
                     mass=0.008, spring=5000.0, damping=1.0, max_effort=40.0)
CAP_W = to_world(CAP_ORG)
CAP = parts.Part("cap", pos=tuple(float(v) for v in CAP_W), yaw=YAW, mass=0.065, shapes=tuple(_carrier_shapes()),
                 joints=(EVERT,), visuals=tuple(_shell_visual(k) for k in range(N_STAGES)),
                 friction=(1.2, 1.0), link_friction=(("evert", (0.4, 0.3)),), contact_offset=0.0015)

# -- the service mat (loose: its place changes between lots) ---------------------------------------------------------
MAT_SIZE = (0.230, 0.300, 0.003)  # x, y, thickness
MAT_TOP = TOP + MAT_SIZE[2]
MAT_A = (0.215, -0.215, TOP + MAT_SIZE[2] / 2)
MAT_B = (0.395, -0.215, TOP + MAT_SIZE[2] / 2)
MAT = parts.Part("mat", pos=MAT_A, mass=0.40, shapes=(parts.box(MAT_SIZE, color=MAT_GREY),), friction=(0.9, 0.8))

OBJECTS = [DHEF, CAP, MAT]

# Layout lots: trial k uses lot k mod 2. The mat lies nearer the arms (A) or farther out (B).
LOTS = lots.declare(OBJECTS, lots.Lot("A", place={"mat": MAT_A}), lots.Lot("B", place={"mat": MAT_B}))

# -- the declared mechanisms (run every control step) ------------------------------------------------------------
# Silicone rolling: the cap wall stays where it is rolled to until about F_ROLL pushes it on (a stick-slip drive:
# the joint's spring target follows the joint, staying F_ROLL / spring behind it). Past full eversion it stretches
# with K_STRETCH. Rod seals: the exhausted rod stays put until about F_ROD moves it. The coupling's latch: the
# keepers open while the releasing ring is pushed at least 1.5 mm toward the rod and close again under 1.0 mm.
F_ROLL, K_STRETCH, F_ROD = 4.0, 2000.0, 6.0
SLEEVE_OPEN, SLEEVE_CLOSE = 0.0015, 0.0010
HAND_CLEAR = 0.030
PUNCTURE = 0.003  # a finger this far inside the cap wall's outline has gone through the silicone
HOLD_STEPS = int(SUCCESS_HOLD_S * 20)
_S = parts.__dict__.setdefault("_dhef_cap_state", {})


def reset_state():
    _S.clear()
    _S.update(known=False, broken=False, punctured=False, stretched=0.0, dropped=False, clear=False, hold=0,
              t_rod=0.0, t_evert=0.0, latch_open=False, stage=0, views=None, log_t=-1.0, max_q=0.0, max_rod=0.0,
              puncture_depth=0.0)


if not _S:
    reset_state()


def stick_slip(q, target, band, lo=0.0):
    """The spring target that keeps a stick-slip joint's force within band x spring: unchanged while the joint stays
    within ``band`` of it, dragged along (``band`` behind) once the joint moves farther."""
    if q > target + band:
        return q - band
    if q < target - band:
        return max(lo, q + band)
    return target


def evert_target(q, target):
    if q > S_FULL:  # stretching: the spring pulls back with K_STRETCH x the stretch
        return S_FULL + (q - S_FULL) * (1.0 - K_STRETCH / EVERT.spring)
    return min(stick_slip(q, target, F_ROLL / EVERT.spring), S_FULL)


# -- geometry for the scorer ----------------------------------------------------------------------------------------
def _frame(st):
    return np.asarray(st.pos, float), parts._quat_to_matrix(st.quat_xyzw)


def _link(st, name):
    p, q = st.links[name]
    return np.asarray(p, float), parts._quat_to_matrix(q)


def cap_point(st, local):
    """A point given in the cap's own frame (at evert 0) on its carrier, in the world."""
    p, r = _frame(st)
    return p + r @ np.asarray(local, float)


def head_back_world(cap_st):
    """Centre of the stud head's back face, in the world (it moves with the cap's tip link)."""
    p, r = _link(cap_st, "evert")
    o = np.asarray(EVERT.origin, float)
    return p + r @ (cyl(U_HEAD0) - CAP_ORG - o)


def coupling_front_world(dhef_st):
    p, r = _link(dhef_st, "rod")
    o = np.asarray(ROD.origin, float)
    return p + r @ (cyl(U_CPL1) - o)


def bayonet_off(cap_st) -> bool:
    """The catch ring drawn at least 10 mm forward off the body (its lugs have passed the lip): measured along the
    gripper's axis from the ring's start, or the cap is off the axis altogether."""
    p, _ = _frame(cap_st)
    d = p - CAP_W
    along = float(d @ AXIS_W)
    off_axis = float(np.linalg.norm(d - along * AXIS_W))
    return along >= 0.010 - EPS or off_axis >= 0.015 - EPS


def uncoupled(cap_st, dhef_st) -> bool:
    """The stud head is out of the coupling: its back face at least 5 mm in front of the coupling's front face, or
    15 mm or more off the coupling's axis."""
    h, c = head_back_world(cap_st), coupling_front_world(dhef_st)
    d = h - c
    along = float(d @ AXIS_W)
    off = float(np.linalg.norm(d - along * AXIS_W))
    return along >= 0.005 - EPS or off >= 0.015 - EPS


def everted(cap_st) -> bool:
    return float(cap_st.joints.get("evert", 0.0)) >= Q_EVERTED - EPS


def cap_boxes(cap_st):
    poses = {"base": _frame(cap_st), "evert": _link(cap_st, "evert")}
    return [(c, r, s) for _, c, r, s in CAP.world_boxes(poses)]


def cap_corners(cap_st, links=("base",)):
    poses = {"base": _frame(cap_st), "evert": _link(cap_st, "evert")}
    out = []
    for name, c, r, s in CAP.world_boxes(poses):
        if name not in links:
            continue
        signs = np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)], float)
        out.append(c + (signs * (np.asarray(s) / 2)) @ r.T)
    return np.concatenate(out)


def on_mat(cap_st, mat_st) -> bool:
    """The carrier lies on the mat: every corner of its boxes over the mat's outline, its lowest point within 3 mm of
    the mat's top, and the stud above the mat."""
    k = cap_corners(cap_st, ("base",))
    mp, mr = _frame(mat_st)
    local = (k - mp) @ mr
    inside = bool(np.all(np.abs(local[:, 0]) <= MAT_SIZE[0] / 2 + EPS) and np.all(np.abs(local[:, 1]) <= MAT_SIZE[1] / 2 + EPS))
    top = float(mp[2] + MAT_SIZE[2] / 2)
    low = float(k[:, 2].min())
    stud_low = float(cap_corners(cap_st, ("evert",))[:, 2].min())
    return bool(inside and top - 0.001 - EPS <= low <= top + 0.003 + EPS and stud_low >= top - 0.001 - EPS)


# -- scoring ---------------------------------------------------------------------------------------------------------
def _ruled_out() -> bool:
    return bool(not _S["known"] or _S["broken"] or _S["punctured"] or _S["dropped"])


def level(objects) -> float:
    """The tier the end state reaches, before the whole-trial rules and the hands-clear / still hold."""
    cap, dhef, mat = objects["cap"], objects["dhef"], objects["mat"]
    if uncoupled(cap, dhef):
        return 1.0 if on_mat(cap, mat) else 0.75
    if everted(cap):
        return 0.5
    if bayonet_off(cap):
        return 0.25
    return 0.0


def score(objects):
    """The tier reached, judged once both hands are 30 mm clear of the cap and of the gripper's front, and the cap has
    been still for 1 s; 0 if a whole-trial rule tripped."""
    if _ruled_out() or not _S["clear"] or _S["hold"] < HOLD_STEPS or not objects["cap"].resting():
        return 0.0
    return level(objects)


def success(objects):
    return score(objects) == 1.0


# -- simulator side: drives, latch, cap wall, rules, hands -------------------------------------------------------------
HAND_LINKS = ("gripper", "tip_left", "tip_right")
_FACE_PTS: dict[str, np.ndarray] = {}


def _hull_surface(h, spacing=0.004):
    from scipy.spatial import ConvexHull

    v = h.vertices
    out = [v]
    for tri in ConvexHull(v).simplices:
        a, b, c = v[tri]
        n = max(1, int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / spacing)))
        out.append(np.array([a + (b - a) * i / n + (c - a) * j / n for i in range(n + 1) for j in range(n + 1 - i)]))
    return np.unique(np.round(np.concatenate(out) / 0.002).astype(np.int64), axis=0) * 0.002


def hand_points(base) -> np.ndarray:
    """Points on the faces (every 4 mm) of both arms' gripper and finger collision hulls, in the world."""
    from yam_arena import analysis

    origin = parts._np(base.scene.env_origins[0])
    pts = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n not in HAND_LINKS:
                continue
            if n not in _FACE_PTS:
                _FACE_PTS[n] = np.concatenate([_hull_surface(h) for h in analysis.link_hulls().get(n, [])])
            pts.append(_FACE_PTS[n] @ parts._quat_to_matrix(poses[i, 3:]).T + (poses[i, :3] - origin))
    if len(pts) != 6:
        raise RuntimeError("hand links not found")
    return np.concatenate(pts)


def puncture_depth(points, cap_st) -> float:
    """How far (m) the deepest hand point lies inside the cap wall's outline (0 if none): the wall rolled to the
    cap's current eversion, plus the tip."""
    p, r = _frame(cap_st)
    local = (points - p) @ r
    v = -local[:, 0]
    rad = np.hypot(local[:, 1], local[:, 2])
    near = (rad < 0.03) & (v > -0.20) & (v < 0.10)
    if not np.any(near):
        return 0.0
    v, rad = v[near], rad[near]
    s = float(cap_st.joints.get("evert", 0.0))
    prof = shell_profile(min(s, S_FULL))
    pv = np.array([a for a, _ in prof])
    pr = np.array([b for _, b in prof])
    vt = V_TIP0 - s
    vf = pv.max()
    depth = np.zeros_like(v)
    outer = (v >= 0.0) & (v <= vf)  # the wall's outer layer covers 0..vf at its outer radius
    rr = np.interp(v[outer], pv[:7], pr[:7])
    depth[outer] = rr - rad[outer]
    inner = (v >= min(vt, vf)) & (v <= max(vt, vf)) & ~outer  # the rolled-back layer behind the rim
    order = np.argsort(pv[7:])
    ri = np.interp(v[inner], pv[7:][order], pr[7:][order])
    depth[inner] = np.maximum(depth[inner], ri - rad[inner])
    tip = (v >= vt) & (v <= vt + (U_TIP1 - U_NECK1))  # the tip itself, wherever it has rolled to
    depth[tip] = np.maximum(depth[tip], R_TIP - rad[tip])
    return float(max(0.0, depth.max()))


def _cyl_dist(a, rho, a0, a1, r):
    """Distance from points (axial coordinate a, radius rho) to a solid cylinder a0..a1, radius r (0 inside)."""
    da = np.maximum(0.0, np.maximum(a0 - a, a - a1))
    dr = np.maximum(0.0, rho - r)
    return np.hypot(da, dr)


def clearance(points, objects) -> float:
    """Smallest distance (m) from the hand points to the cap assembly and the gripper's front (catch region, rod,
    coupling, releasing ring and keepers), each taken as a solid cylinder round its own axis."""
    cap, dhef = objects["cap"], objects["dhef"]
    p, r = _frame(cap)
    loc = (points - p) @ r
    a, rho = -loc[:, 0], np.hypot(loc[:, 1], loc[:, 2])  # a: distance in front of the rim
    s = float(cap.joints.get("evert", 0.0))
    vt = V_TIP0 - s
    vf = max((SHELL_LEN + vt) / 2, vt + 0.002)
    d = _cyl_dist(a, rho, U_RING0 - U_RIM, U_BUSH1 - U_RIM, R_RING_OUT)  # carrier
    d = np.minimum(d, _cyl_dist(a, rho, min(vt, 0.0) - (U_NECK1 - U_HEAD0) - 0.001, max(vf, U_TIP1 - U_NECK1 + vt),
                                R_CAP_RIM))  # cap wall, tip and stud
    p, r = _frame(dhef)
    loc = (points - p) @ r
    u, rho = -loc[:, 0], np.hypot(loc[:, 1], loc[:, 2])
    q = float(dhef.joints.get("rod", 0.0))
    d = np.minimum(d, _cyl_dist(u, rho, 0.100, U_NOSE, R_RING_OUT))  # locking cover and nose
    d = np.minimum(d, _cyl_dist(u, rho, U_NOSE, U_CPL0 - q, R_ROD))  # rod
    d = np.minimum(d, _cyl_dist(u, rho, U_CPL0 - q, U_CPL1 - q, R_CPL + KEEP_OPEN))  # coupling, ring, keepers
    return float(d.min()) if len(d) else 1.0


def hands_clear(points, objects) -> bool:
    return clearance(points, objects) >= HAND_CLEAR - EPS


def _show_stage(sim, k):
    if k == _S["stage"]:
        return
    sim.set_visible("cap", f"shell{k}", True)
    sim.set_visible("cap", f"shell{_S['stage']}", False)
    _S["stage"] = k


@parts.on_reset
def _on_reset(sim) -> None:
    stage = _S.get("stage", 0)
    reset_state()
    for k in range(N_STAGES):
        if k == 0 or k == stage:
            try:
                sim.set_visible("cap", f"shell{k}", k == 0)
            except Exception as exc:  # noqa: BLE001
                print(f"[dhef] could not reset shell{k}: {exc}", file=sys.stderr)


@parts.on_step
def _on_step(sim) -> None:
    try:
        base = sim.env.unwrapped
        st = sim.states()
        cap, dhef = st["cap"], st["dhef"]
        q_e = float(cap.joints["evert"])
        q_r = float(dhef.joints["rod"])
        q_s = float(dhef.joints["sleeve"])
        # the declared mechanisms
        _S["t_evert"] = evert_target(q_e, _S["t_evert"])
        sim.set_target("cap", "evert", _S["t_evert"])
        _S["t_rod"] = stick_slip(q_r, _S["t_rod"], F_ROD / ROD.spring)
        sim.set_target("dhef", "rod", _S["t_rod"])
        if not _S["latch_open"] and q_s >= SLEEVE_OPEN - EPS:
            _S["latch_open"] = True
        elif _S["latch_open"] and q_s < SLEEVE_CLOSE:
            _S["latch_open"] = False
        k_open = KEEP_OPEN if _S["latch_open"] else 0.0
        sim.set_target("dhef", "keep_top", k_open)
        sim.set_target("dhef", "keep_bot", k_open)
        # the cap wall's look
        _show_stage(sim, int(np.argmin([abs(min(q_e, S_FULL) - q) for q in STAGE_Q])))
        # whole-trial rules
        _S["max_q"] = max(_S["max_q"], q_e)
        _S["max_rod"] = max(_S["max_rod"], q_r)
        _S["stretched"] = max(_S["stretched"], q_e - S_FULL)  # logged only: how far past full eversion
        corners = cap_corners(cap, ("base", "evert"))
        p = np.asarray(cap.pos, float)
        mp = np.asarray(st["mat"].pos, float)
        if (float(corners[:, 2].min()) < TOP - 0.010 or abs(p[1]) > 0.64 or not (-0.18 < p[0] < 0.90)
                or mp[2] < TOP - 0.010 or abs(mp[1]) > 0.64 or not (-0.18 < mp[0] < 0.90)):
            _S["dropped"] = True  # the cap or the mat has left the table
        pts = hand_points(base)
        depth = puncture_depth(pts, cap)
        _S["puncture_depth"] = max(_S["puncture_depth"], depth)
        if depth >= PUNCTURE - EPS:
            _S["punctured"] = True
        _S["clear"] = hands_clear(pts, st)
        _S["known"] = True
        _S["hold"] = _S["hold"] + 1 if (_S["clear"] and cap.resting()) else 0
        if sim.t - _S["log_t"] >= 1.0 - 1e-6:
            _S["log_t"] = sim.t
            print("[dhef] " + json.dumps(dict(
                t=round(float(sim.t), 2), evert=round(q_e * 1000, 1), rod=round(q_r * 1000, 1),
                sleeve=round(q_s * 1000, 2), latch=_S["latch_open"],
                keepers=[round(float(dhef.joints["keep_top"]) * 1000, 1), round(float(dhef.joints["keep_bot"]) * 1000, 1)],
                cap=[round(float(x), 4) for x in cap.pos], cap_q=[round(float(x), 4) for x in cap.quat_xyzw],
                v=round(float(np.linalg.norm(cap.lin_vel)) * 1000, 1), w=round(float(np.linalg.norm(cap.ang_vel)), 2),
                off=bayonet_off(cap), uncoupled=uncoupled(cap, dhef), mat=on_mat(cap, st["mat"]),
                level=level(st), punct=round(_S["puncture_depth"] * 1000, 1),
                stretched=round(max(0.0, _S["stretched"]) * 1000, 1),
                dropped=_S["dropped"], clear=_S["clear"], hold=_S["hold"])), flush=True)
    except Exception as exc:  # unreadable states or hands: no credit for this trial
        _S["broken"] = True
        print(f"[dhef] step check failed: {exc!r}", file=sys.stderr, flush=True)


# -- the scripted demo: open loop from the visible layout (the gripper, its catch ring and the mat's place) ----------
YAW_R = math.pi / 2  # the right hand pinches across the gripper's axis (fingers closing along world x)
YAW_L = -math.pi / 2
LEAN = 50.0  # degrees the right hand leans to turn the catch (the ring stops at 45)
GRIP_RING = -(2 * R_RING_OUT - 0.006) / 2  # finger target 3 mm inside each face of the 57.5 mm ring
GRIP_SLEEVE = -(2 * R_SLEEVE - 0.006) / 2
U_GRIP_RING = 0.136  # where the right hand pinches the ring (its centre is at 129.25): toward its front
PULL = S_FULL + 0.030  # how far the hand draws the ring forward: full eversion plus the arm's give (it lags ~25 mm)
U_GRIP_SLEEVE = 0.192
PUSH = 0.020  # the left hand pushes the releasing ring this far toward the rod (3 mm travel plus the arm's give)
DRAW = 0.035  # then the right hand draws the cap off the opened coupling
PARK = {"left": (0.13, 0.22, TOP + 0.15), "right": (0.15, -0.22, TOP + 0.15)}
READY_LEAN = 55.6  # the arms' start pose points the fingers 55.6 deg off straight down, toward +x
BACK_OUT = (0.20, 0.305, TOP + 0.30)  # the left hand first backs straight out, keeping that pose, from beside the UR5e


def _wy(u):
    """World y of the gripper's axis at distance u from its mounting face."""
    return float(Y_F - u)


def _w(dur, pos, finger, label, yaw, lean=0.0, heading=0.0):
    return tilt.Waypoint(float(dur), tuple(float(v) for v in pos), float(finger), label, float(yaw), tilt=float(lean),
                         heading=float(heading))


def demo():
    lot = lots.current()
    mx, my, mz = lot.pos("mat")  # where the mat lies, as the camera sees it
    yr = _wy(U_GRIP_RING)
    R = [
        _w(2.5, (X_AX, yr, Z_AX + 0.05), FINGER_OPEN, "over the catch ring", YAW_R),
        _w(1.5, (X_AX, yr, Z_AX), FINGER_OPEN, "down round the ring", YAW_R),
        _w(1.0, (X_AX, yr, Z_AX), GRIP_RING, "pinch the ring", YAW_R),
        _w(2.5, (X_AX, yr, Z_AX), GRIP_RING, "turn the catch to open", YAW_R, LEAN, 0.0),
        _w(0.5, (X_AX, yr, Z_AX), GRIP_RING, "settle", YAW_R, LEAN, 0.0),
        _w(8.0, (X_AX, yr - PULL, Z_AX), GRIP_RING, "draw the ring forward: the cap turns inside out", YAW_R, LEAN, 0.0),
        _w(1.0, (X_AX, yr - PULL, Z_AX), GRIP_RING, "hold", YAW_R, LEAN, 0.0),
    ]
    t_everted = sum(w.duration for w in R)
    if os.environ.get("DHEF_DEMO") == "yank":  # a check, not the demo: draw on hard without releasing the coupling
        R += [_w(4.0, (X_AX, yr - PULL - 0.120, Z_AX), GRIP_RING, "pull on hard", YAW_R, LEAN, 0.0),
              _w(1.0, (X_AX, yr - PULL - 0.120, Z_AX), GRIP_RING, "hold", YAW_R, LEAN, 0.0),
              _w(0.8, (X_AX, yr - PULL - 0.120, Z_AX), FINGER_OPEN, "let go", YAW_R, LEAN, 0.0),
              _w(1.5, (X_AX, yr - PULL - 0.120, Z_AX + 0.06), FINGER_OPEN, "up", YAW_R),
              _w(2.0, PARK["right"], FINGER_OPEN, "park", YAW_R), _w(3.0, PARK["right"], FINGER_OPEN, "wait", YAW_R)]
        total = sum(w.duration for w in R)
        return {"right": R, "left": [_w(2.0, BACK_OUT, FINGER_OPEN, "up and back", 0.0, READY_LEAN, 0.0),
                                     _w(total - 2.0, PARK["left"], FINGER_OPEN, "stay clear", YAW_L)]}
    ys = _wy(U_GRIP_SLEEVE)
    L = [
        _w(2.0, BACK_OUT, FINGER_OPEN, "up and back from beside the robot's wrist", 0.0, READY_LEAN, 0.0),
        _w(t_everted - 2.0, PARK["left"], FINGER_OPEN, "wait", YAW_L),
        _w(2.5, (X_AX, ys, Z_AX + 0.05), FINGER_OPEN, "over the releasing ring", YAW_L),
        _w(1.5, (X_AX, ys, Z_AX), FINGER_OPEN, "down round it", YAW_L),
        _w(1.0, (X_AX, ys, Z_AX), GRIP_SLEEVE, "pinch the releasing ring", YAW_L),
        _w(1.5, (X_AX, ys + PUSH, Z_AX), GRIP_SLEEVE, "push it toward the rod", YAW_L),
    ]
    t_pushed = sum(w.duration for w in L)
    R.append(_w(t_pushed - t_everted + 0.5, (X_AX, yr - PULL, Z_AX), GRIP_RING, "hold while the ring is pushed",
                YAW_R, LEAN, 0.0))
    R.append(_w(2.0, (X_AX, yr - PULL - DRAW, Z_AX), GRIP_RING, "draw the cap off the coupling", YAW_R, LEAN, 0.0))
    t_off = sum(w.duration for w in R)
    L += [
        _w(t_off - t_pushed + 0.3, (X_AX, ys + PUSH, Z_AX), GRIP_SLEEVE, "keep the ring pushed", YAW_L),
        _w(0.8, (X_AX, ys + PUSH, Z_AX), FINGER_OPEN, "let go of the ring", YAW_L),
        _w(1.2, (X_AX, ys + PUSH, Z_AX + 0.05), FINGER_OPEN, "up", YAW_L),
        _w(2.0, PARK["left"], FINGER_OPEN, "park", YAW_L),
    ]
    t_left_gone = sum(w.duration for w in L)
    drop_z = MAT_TOP + R_RING_OUT + 0.006
    cx = mx
    R += [
        _w(max(0.5, t_left_gone - 2.0 - t_off), (X_AX, yr - PULL - DRAW, Z_AX), GRIP_RING, "wait for the left hand",
           YAW_R, LEAN, 0.0),
        _w(2.0, (X_AX, yr - PULL - DRAW, Z_AX + 0.06), GRIP_RING, "lift the cap assembly clear", YAW_R),
        _w(3.0, (cx, my + 0.02, drop_z + 0.05), GRIP_RING, "carry it over the mat", YAW_R),
        _w(1.5, (cx, my + 0.02, drop_z), GRIP_RING, "lower it", YAW_R),
        _w(0.8, (cx, my + 0.02, drop_z), FINGER_OPEN, "let go", YAW_R),
        _w(1.2, (cx, my + 0.02, drop_z + 0.10), FINGER_OPEN, "up", YAW_R),
        _w(2.0, PARK["right"], FINGER_OPEN, "park", YAW_R),
        _w(2.5, PARK["right"], FINGER_OPEN, "wait", YAW_R),
    ]
    total = max(sum(w.duration for w in R), sum(w.duration for w in L))
    for arm, w in (("right", R), ("left", L)):
        rest = total - sum(x.duration for x in w)
        if rest > 1e-6:
            w.append(_w(rest, PARK[arm], FINGER_OPEN, "stay clear", YAW_R if arm == "right" else YAW_L))
    return {"right": R, "left": L}
