"""On a kitchen counter a Samsung Galaxy S24 phone stands upside down in an aluminium desk phone stand, screen
toward the robot, so its USB-C charging port is on top. Four small balls of off-white lint sit inside the port, one
in each corner of the two narrow gaps beside the flat tongue in its middle, about 4.5 mm down. A clean, dry, soft
toothbrush lies bristles-down on the counter to the robot's right, and a folded white paper towel lies flat on the
counter beyond it.
"""

from __future__ import annotations

import json
from dataclasses import replace
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402  (before lots, so parts' reset hooks run before the lots' moves)
from contact_audit.geometry import shaft_phone_penetrations  # noqa: E402
import lots  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
EPS = 1e-6
HZ = 20

GLASS, FRAME, BACK = (0.05, 0.06, 0.08), (0.42, 0.43, 0.45), (0.56, 0.57, 0.60)
PORT_DARK, TONGUE, ALU = (0.03, 0.03, 0.03), (0.15, 0.15, 0.16), (0.70, 0.71, 0.73)
HANDLE, NECK, BRISTLE, BLUE = (0.12, 0.50, 0.72), (0.14, 0.55, 0.78), (0.93, 0.94, 0.95), (0.20, 0.45, 0.85)
TOWEL, LINT = (0.97, 0.97, 0.95), (0.80, 0.74, 0.62)

# -- stated estimates: each is one named constant so a real reading can replace it -------------------------------
E_NYLON = 4.0e9  # Pa: nylon 6,12 bristle filament at 50 % RH (DuPont Tynex brochure, Table 1: 580,000 psi)
D_FIL = 0.15e-3  # m: filament diameter of a soft toothbrush (estimate; a soft-brush patent, EP4295727A1, uses 0.15)
N_FIL = 72  # equivalent reference filaments for stiffness, not this product's count (EP4295727A1: 72 +/- 4)
L_TUFT = 0.011  # m: free length; technician phone figure, Nov 2026
TUFT_DIAMETER = 0.00115  # m: tuft diameter; technician phone figure, Nov 2026
TUFT_LEAN = math.radians(16.0)  # tufts angled along the head, neighbouring columns opposite ways (estimate)
MU_BRISTLE = (0.20, 0.15)  # effective pair coefficients via min combining, unmeasured estimates
NATIVE_FORCE_HEADROOM = 0.0001  # native cap headroom above the unchanged force gate
F_LINT = 0.03  # N: a lint ball's resistance to a bristle tip moving through it (estimate)
F_WIPE = 0.02  # N: least contact force of a tuft on the towel for its lint to wipe off (estimate)
EI = E_NYLON * math.pi * D_FIL ** 4 / 64
# Discrete cantilever: n equal segments/two bending swings per segment. This preserves 3*N_FIL*EI/L^3.
N_SEG = 3
K_SEG = N_FIL * EI / L_TUFT * (N_SEG+1)*(2*N_SEG+1)/(2*N_SEG)
JOINT_SOLVER_TOL = 0.02  # rad: numerical validity allowance beyond declared joint stops
MAX_SHAFT_OVERLAP = 0.0001  # m: native-solver validity tolerance, independently audited OBB overlap
CONTACT = 0.0001  # m: contact offset of the phone and the toothbrush (sub-millimetre gaps)

# -- the phone: Samsung Galaxy S24 (SM-S921B), 147.0 x 70.6 x 7.6 mm, 167 g --------------------------------------
# Part frame: x through the thickness (screen at -x), y across the width, z along the length; +z is the end with
# the charging port, which is on top (the phone stands upside down).
P_LEN, P_WID, P_THK, P_MASS = 0.147, 0.0706, 0.0076, 0.167
LEAN = 0.0  # rad: upright fixture setting; actual adjustable-holder placement still needs verification
R_PHONE = parts._rot((0.0, 1.0, 0.0), LEAN)
EDGE = np.array([0.250, -0.040, TOP + 0.007])  # centre of the phone's lower edge (its real top edge), on the ledge
GAP = 0.0003  # start clearance to the stand
PHONE_POS = EDGE + R_PHONE @ np.array([0.0, 0.0, P_LEN / 2 + GAP])
# USB-C receptacle (USB Type-C receptacle drawings): opening 8.34 x 2.56 mm, tongue 6.69 x 0.70 mm. Depth 7.5 mm and
# depth obtained by phone from a technician who works with this product (Nov 2026); tongue top estimated.
PORT_V, PORT_U = 0.00834 / 2, 0.00256 / 2  # half opening across the width (v) and the thickness (u)
TONGUE_V, TONGUE_U = 0.00669 / 2, 0.00070 / 2
PORT_DEPTH, TONGUE_TOP = 0.0075, 0.0020
END = 0.014  # the port end of the phone: frame round the opening, end walls, cavity floor and tongue


def _phone_shapes():
    T, W, L = P_THK, P_WID, P_LEN
    z_end = L / 2 - END
    s = [parts.box((T, W, L - END), at=(0.0, 0.0, -END / 2), color=FRAME)]  # body
    # looks: screen glass, back glass, camera rings near the real top (now at -z), side keys
    s += [parts.box((0.0004, W - 0.0016, L - 0.0016), at=(-T / 2 - 0.0001, 0.0, 0.0), color=GLASS, collide=False),
          parts.box((0.0004, W - 0.0016, L - 0.0016), at=(T / 2 + 0.0001, 0.0, 0.0), color=BACK, collide=False)]
    for dz in (0.016, 0.029, 0.042):
        s.append(parts.cylinder(0.0055, 0.0016, at=(T / 2 + 0.0008, W / 2 - 0.013, -L / 2 + dz), color=GLASS,
                                axis="x", collide=False))
    s += [parts.box((0.002, 0.0006, 0.020), at=(0.0, -W / 2 - 0.0002, 0.025), color=FRAME, collide=False),
          parts.box((0.002, 0.0006, 0.010), at=(0.0, -W / 2 - 0.0002, -0.002), color=FRAME, collide=False)]
    # the port end, all colliding: the frame either side of the opening, in front of and behind it, the cavity floor
    # and the tongue
    side = W / 2 - PORT_V
    zc = z_end + END / 2
    s += [parts.box((T, side, END), at=(0.0, sgn * (PORT_V + side / 2), zc), color=FRAME) for sgn in (-1, 1)]
    wall = T / 2 - PORT_U
    s += [parts.box((wall, 2 * PORT_V, END), at=(sgn * (PORT_U + wall / 2), 0.0, zc), color=FRAME) for sgn in (-1, 1)]
    floor = END - PORT_DEPTH
    s.append(parts.box((2 * PORT_U, 2 * PORT_V, floor), at=(0.0, 0.0, z_end + floor / 2), color=PORT_DARK))
    tl = PORT_DEPTH - TONGUE_TOP
    s.append(parts.box((2 * TONGUE_U, 2 * TONGUE_V, tl), at=(0.0, 0.0, L / 2 - PORT_DEPTH + tl / 2), color=TONGUE))
    return s


PHONE = parts.Part("phone", pos=tuple(float(v) for v in PHONE_POS), quat=tuple(parts._matrix_to_quat(R_PHONE)),
                   mass=P_MASS, shapes=_phone_shapes(), friction=(0.6, 0.5), contact_offset=CONTACT, report=("base",))

# -- the stand: an aluminium desk phone stand (about 76 x 76 mm base, about 120 g), loose on the counter ----------
STAND_POS = np.array([0.268, -0.040, TOP + 0.0015])
Q_LEAN = tuple(parts._matrix_to_quat(R_PHONE))


def _stand_shapes():
    def at_phone(local):  # a point given in the phone's frame (relative to the phone centre) -> stand frame
        return tuple(float(v) for v in PHONE_POS - STAND_POS + R_PHONE @ np.asarray(local, float))

    h = P_LEN / 2 + GAP
    s = [parts.box((0.076, 0.076, 0.003), at=(0.0, 0.0, 0.0), color=ALU)]
    # ledge under the phone's lower edge, front lip before the screen, upright back rest
    s.append(parts.box((0.016, 0.050, 0.004), at=at_phone((0.0012, 0.0, -h - 0.002)), color=ALU, quat=Q_LEAN))
    s.append(parts.box((0.002, 0.050, 0.008), at=at_phone((-P_THK / 2 - 0.0006 - 0.001, 0.0, -h + 0.004)),
                       color=ALU, quat=Q_LEAN))
    s.append(parts.box((0.004, 0.060, 0.085), at=at_phone((P_THK / 2 + GAP + 0.002, 0.0, -h - 0.004 + 0.0425)),
                       color=ALU, quat=Q_LEAN))
    # strut from the back of the base to the back of the rest
    a = np.array([0.036, 0.0, 0.0015])
    b = np.asarray(at_phone((P_THK / 2 + GAP + 0.004, 0.0, -0.030)))
    d = b - a
    ang = math.atan2(d[0], d[2])
    s.append(parts.box((0.004, 0.030, float(np.linalg.norm(d))), at=tuple((a + b) / 2), color=ALU,
                       quat=tuple(parts._matrix_to_quat(parts._rot((0.0, 1.0, 0.0), ang)))))
    return s


STAND = parts.Part("phone_stand", pos=tuple(float(v) for v in STAND_POS), mass=0.120, shapes=_stand_shapes(),
                   friction=(0.8, 0.6))

# -- the toothbrush: Oral-B CrossAction All-in-One Soft (sizes estimated, see README), 16 g ------------------------
# Part frame: origin on the handle's centre line where the robot holds it, y along the handle toward the head, z up
# (bristles toward -z). The bristles are 40 tufts (4 rows across the head, 10 along it), each three jointed segments
# with two passive bending swings at every segment, angled TUFT_LEAN along the head.
HANDLE_W, HANDLE_T = 0.013, 0.009
HEAD_Y = 0.085  # centre of the head from the hold point
ROOT_Z = -0.006  # the head's underside, where the tufts are set
ROWS_X = (-0.00375, -0.00125, 0.00125, 0.00375)
COLS_Y = tuple(HEAD_Y + 0.0022 * (k - 4.5) for k in range(10))
BLADE = (0.0005, TUFT_DIAMETER)  # a tuft's filaments drawn as one flat bundle: across the head, along it
ROOT_GAP = 0.0004  # the bundle starts this far below the head
KINK = math.radians(4.0)  # the tip segment leans this much further than the root segment (a tuft's natural curve)
# Reference filament mass, not this product's measured mass or count; the remaining16g tool mass is plastic.
RHO_NYLON = 1060.0  # kg/m3, DuPont brochure Table 16
FILAMENT_MASS_PER_TUFT = RHO_NYLON * N_FIL * math.pi * D_FIL**2 / 4 * L_TUFT
SEG_MASS = FILAMENT_MASS_PER_TUFT / N_SEG
KNOT_MASS = 0.
TUFT_DAMP = .6611570247933884 * L_TUFT**2 * (N_SEG+1)*(2*N_SEG+1)/(6*N_SEG)
BRUSH_MASS = 0.016


def _seg(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    z = (b - a) / np.linalg.norm(b - a)
    x = np.cross([0.0, 1.0, 0.0], z)
    x /= np.linalg.norm(x)
    r = np.stack([x, np.cross(z, x), z], axis=1)
    return parts.box((BLADE[0], BLADE[1], float(np.linalg.norm(b - a))), at=tuple((a + b) / 2), color=BRISTLE,
                     quat=tuple(parts._matrix_to_quat(r)))


def _tufts():
    joints,tufts=[],[]
    for i,x in enumerate(ROWS_X):
        for j,y in enumerate(COLS_Y):
            name,parent=f"t{i}_{j}","base";point=np.array([x,y,ROOT_Z]);sgn=1. if j%2==0 else -1.
            for segment in range(N_SEG):
                angle=sgn*(TUFT_LEAN+KINK*segment/(N_SEG-1))
                direction=parts._rot((1,0,0),angle)@np.array([0,0,-1.])
                end=point+direction*(L_TUFT/N_SEG)
                link=name if segment==N_SEG-1 else name+f"s{segment}"
                start=point+direction*ROOT_GAP if segment==0 else point
                joints.append(parts.Hinge(link,shapes=(_seg(start,end),),pivot=tuple(point),axis=tuple(-direction),
                                          lower=-1.4,upper=1.4,mass=SEG_MASS,spring=K_SEG,damping=TUFT_DAMP,
                                          parent=parent,bend2=True))
                if segment==N_SEG-1:tufts.append((name,abs(x)<.002,point.copy(),end.copy()))
                point,parent=end,link
    return joints,tufts


TUFT_JOINTS, TUFTS = _tufts()
N_TUFT = len(TUFTS)
TUFT_NAMES = [t[0] for t in TUFTS]
SEGS = tuple(j.name for j in TUFT_JOINTS)
TIP0 = np.array([t[3] for t in TUFTS])
KNEE0 = np.array([t[2] for t in TUFTS])
TIP_Z = float(TIP0[:, 2].min())
BRUSH_SHAPES = [parts.box((HANDLE_W, 0.130, HANDLE_T), at=(0.0, -0.026, 0.0), color=HANDLE),
                parts.box((0.007, 0.032, 0.005), at=(0.0, 0.055, -0.002), color=NECK),
                parts.box((0.011, 0.028, 0.005), at=(0.0, HEAD_Y, -0.0035), color=NECK),
                parts.box((0.0102, 0.008, 0.0004), at=(0.0, HEAD_Y - 0.006, -0.0062), color=BLUE, collide=False)]
_BASE_MASS = BRUSH_MASS - sum(j.mass for j in TUFT_JOINTS)

# Lying bristles-down it rests on the handle's far end and the tips of the tufts nearest the handle: tilted about x.
_END = np.array([0.0, -0.091, -HANDLE_T / 2])
_NEAR = TIP0[np.argmin(TIP0[:, 1])]
REST_TILT = math.atan2(_END[2] - _NEAR[2], _NEAR[1] - _END[1])
R_BRUSH = parts._rot((1.0, 0.0, 0.0), REST_TILT)
REST_H = -float((R_BRUSH @ _END)[2])  # hold point height above the counter at rest
BRUSH_A = np.array([0.220, -0.220, TOP + REST_H])
# Bake the resting orientation into native geometry/axes, matching the qualified CPU material fixture.
# TIP0/KNEE0 remain in the planning frame; native endpoint offsets are turned by R_BRUSH below.
def _native_shape(sh):
    return replace(sh, at=tuple(R_BRUSH@np.asarray(sh.at)),
                   quat=tuple(parts._matrix_to_quat(R_BRUSH@sh.R)))
TUFT_JOINTS = [replace(j, shapes=tuple(_native_shape(sh) for sh in j.shapes),
                       pivot=tuple(R_BRUSH@np.asarray(j.pivot)), axis=tuple(R_BRUSH@np.asarray(j.axis)))
               for j in TUFT_JOINTS]
BRUSH_SHAPES = [_native_shape(sh) for sh in BRUSH_SHAPES]
SENSOR_MASS=1e-9
SENSOR_NAMES=['force_'+n for n in TUFT_NAMES]
SENSOR_JOINTS=[parts.Fixed(n,(),tuple(R_BRUSH@TIP0[k]),mass=SENSOR_MASS,parent=TUFT_NAMES[k],instrument=True) for k,n in enumerate(SENSOR_NAMES)]
ALL_BRUSH_JOINTS=TUFT_JOINTS+SENSOR_JOINTS
BRUSH = parts.Part("toothbrush", pos=tuple(float(v) for v in BRUSH_A),
                   mass=_BASE_MASS-N_TUFT*SENSOR_MASS, shapes=BRUSH_SHAPES, joints=ALL_BRUSH_JOINTS, friction=(0.8, 0.6),
                   contact_offset=CONTACT, report=("base",) + SEGS, link_friction=tuple((n, MU_BRISTLE) for n in SEGS),
                   self_collide=False)

# -- the paper towel: one Bounty sheet folded in quarters, about 140 x 130 mm, lying flat (fixed) -----------------
TOWEL_SIZE = (0.140, 0.130, 0.0015)
TOWEL_C = np.array([0.320, -0.200, TOP + TOWEL_SIZE[2] / 2])
TOWEL_TOP = TOP + TOWEL_SIZE[2]
TOWEL_PART = parts.Part("paper_towel", pos=tuple(float(v) for v in TOWEL_C), mass=None,
                        shapes=[parts.box(TOWEL_SIZE, color=TOWEL)])

OBJECTS = [PHONE, STAND, BRUSH, TOWEL_PART]
LOT_B_SHIFT = np.array([0.0, -0.020, 0.0])  # lot B: the phone in its stand 20 mm further right
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A"),  # the screenshot's layout
    lots.Lot("B", place={"phone": tuple(float(v) for v in PHONE_POS + LOT_B_SHIFT),
                         "phone_stand": tuple(float(v) for v in STAND_POS + LOT_B_SHIFT)}),
)

# -- the lint (a stated rule; README "Lint rule") ------------------------------------------------------------------
# Port coordinates: u through the thickness (+u toward the back glass), v across the width, d depth below the mouth.
LINT_UVD = np.array([(su * 0.000815, sv * 0.0022, 0.0045) for su in (-1, 1) for sv in (-1, 1)])
N_LINT = len(LINT_UVD)
LINT_R = 0.0010  # a lint ball is about 2 mm across: a tip within 1 mm of its centre along v and in depth is in it
PACK_D = 0.00575  # a tip in a ball's column pushed deeper than this, past the ball, packs it onto the floor
CATCH_TRAVEL = 0.002  # tip travel through the ball, against its resistance, to hook it out
STEP_MAX = 0.003  # a larger jump in one control step counts no travel
OUT_D = -0.0005  # a carried ball above this depth (0.5 mm above the mouth) is out of the port
BACK_IN_D = 0.0005  # a carried ball that is out and comes back below this depth is left in the port again
SCRAPE_V = PORT_V - 0.00025  # a carried ball dragged this far along the slot, below the mouth, is scraped off
TOWEL_GAP = 0.0010  # a carried ball this close above the towel's top, over it, with its tuft pressed on it ...
TOWEL_WIPE = 0.010  # ... wipes off once it has slid this far over the towel
HAND_CLEAR = 0.030
HOLD_STEPS = int(SUCCESS_HOLD_S * HZ)
PHONE_MOVE, PHONE_TURN = 0.006, math.radians(4.0)
IN_PORT, CARRIED, ON_TOWEL, PACKED = 0, 1, 2, 3

_S = parts.__dict__.setdefault("_usbc_lint_state", {})


def _box_surface(c, r, size, step=0.004):
    h = np.asarray(size) / 2
    g = [np.linspace(-h[i], h[i], max(2, int(math.ceil(2 * h[i] / step)) + 1)) for i in range(3)]
    pts = np.array([[a, b, d] for a in g[0] for b in g[1] for d in g[2]
                    if abs(abs(a) - h[0]) < 1e-9 or abs(abs(b) - h[1]) < 1e-9 or abs(abs(d) - h[2]) < 1e-9])
    return c + pts @ r.T


HARD = [s for s in BRUSH.shapes if s.collide]  # handle, neck, head
_HARD_PTS = np.concatenate([_box_surface(np.asarray(s.at), s.R, 2 * s.half, 0.002) for s in HARD])


def reset_state():
    _S.clear()
    _S.update(mode=np.zeros(N_LINT, int), travel=np.zeros(N_LINT), out=np.zeros(N_LINT, bool), wiped=np.zeros(N_LINT),
              last_w=np.full((N_LINT, 3), np.nan), retained_work=np.zeros(N_LINT), capture_depth=np.full(N_LINT,np.nan), rider=np.full(N_LINT, -1), world=np.zeros((N_LINT, 3)),
              prev_tips=None, known=False, broken=False, hard_in_port=False, knocked=False,
              dropped=False, clear=False, supported=False, hold=0, deepest=0.0, log_t=-1.0, views=None,
              f_phone=0.0, f_max=0.0, f_left=0.0, f_left_max=0.0, drag=0.0, bend=0.0, low_z=None, overlap=0.0, overlap_peak=0.0, applied_lint=np.zeros((N_TUFT, 3)))


if not _S:
    reset_state()


def _pose(st):
    return np.asarray(st.pos, float), parts._quat_to_matrix(st.quat_xyzw)


def port_frame(phone_pose):
    """Mouth centre and the unit axes (u, v, a: a points out of the port) in the world."""
    p, r = phone_pose
    return p + r @ np.array([0.0, 0.0, P_LEN / 2]), r[:, 0], r[:, 1], r[:, 2]


def to_port(points, phone_pose):
    m, u, v, a = port_frame(phone_pose)
    rel = np.asarray(points, float) - m
    return np.stack([rel @ u, rel @ v, -(rel @ a)], axis=-1)


def from_port(uvd, phone_pose):
    m, u, v, a = port_frame(phone_pose)
    uvd = np.asarray(uvd, float)
    return m + uvd[..., :1] * u + uvd[..., 1:2] * v - uvd[..., 2:3] * a


def in_cavity(uvd):
    uvd = np.asarray(uvd, float)
    return (np.abs(uvd[..., 0]) < PORT_U) & (np.abs(uvd[..., 1]) < PORT_V) & (uvd[..., 2] > 0.0) & (
        uvd[..., 2] < PORT_DEPTH)


def brush_tips(brush_state) -> np.ndarray:
    """World position of each tuft's tip, from its (deflected) end segment's pose."""
    out = np.zeros((N_TUFT, 3))
    for k, name in enumerate(TUFT_NAMES):
        p, q = brush_state.links[name]
        out[k] = np.asarray(p, float) + parts._quat_to_matrix(q) @ (R_BRUSH @ (TIP0[k] - KNEE0[k]))
    return out


def lint_world(phone_pose, tips):
    """Where each lint ball is now (world): in the port, on a tuft's tip or on the towel."""
    out = np.zeros((N_LINT, 3))
    for i in range(N_LINT):
        if _S["mode"][i] in (IN_PORT, PACKED):
            uvd = LINT_UVD[i] if _S["mode"][i] == IN_PORT else np.array([*LINT_UVD[i][:2], PORT_DEPTH - LINT_R])
            out[i] = from_port(uvd, phone_pose)
        elif _S["mode"][i] == CARRIED:
            out[i] = tips[_S["rider"][i]]
        else:
            out[i] = _S["world"][i]
    return out


def _side(uvd, i):
    """Tips in lint ball i's gap (its side of the tongue, inside the opening)."""
    su = np.sign(LINT_UVD[i][0])
    return (su * uvd[:, 0] > TONGUE_U) & (np.abs(uvd[:, 0]) < PORT_U) & (np.abs(uvd[:, 1]) < PORT_V)


def carried_lint_contacts_phone(uvd, expanded):
    """Ellipsoid-vs-solid contact of the carried proxy, not entry into an empty coordinate window.

    The rolled 2 mm coupon is represented as 0.8 mm across u while compressed in the gap, returning to a
    nominal 2 mm outside it (an estimate; fibre expansion is not simulated). All phone solids remain native
    colliders. This geometric proxy contact supplies resistance and only previously applied resistance can
    leave the carried lint behind.
    """
    centre = np.array([uvd[0], uvd[1], P_LEN / 2 - uvd[2]])
    radii = np.array([LINT_R if expanded else 0.0004, LINT_R, LINT_R])
    for sh in PHONE.shapes:
        if not sh.collide:
            continue
        local = sh.R.T @ (centre - np.asarray(sh.at))
        delta = local - np.clip(local, -sh.half, sh.half)
        if float(np.sum((delta / radii) ** 2)) <= 1.0 + EPS:
            return True
    return False


def record(phone_pose, tips, f_towel, f_lint=None, force_error=None):
    """One control step of the lint rule, from the tufts' deflected tip positions (world) and each tuft's contact
    force on the towel (``f_towel``, N) and the lint resistance applied during the preceding physics interval
    (``f_lint``, world-force vector in N per tuft; missing means no force and no credit). Returns the lint's resistance to apply: a list of
    (tuft index, force on that tuft's tip in the world, N); the phone takes the equal and opposite force.

    A ball still in the port resists any tuft tip moving through it (within 1 mm of its centre along the slot and in
    depth, in its gap) with F_LINT against the tip's motion. Once tips have moved 2 mm through it against that
    resistance, across the port's cross-section (along or across the slot, not in depth; positive credit is capped by the most-moved tip each step, and actual assisting work from all loaded tips
    remains a debt that must be repaid), it is hooked onto the nearest of them and
    rides on that tip. A tip in its gap and within 1 mm of it along the slot pushed deeper than 5.75 mm packs it onto
    the port's floor: it stays there. A carried ball dragged to within 0.25 mm of the slot's end below the mouth is
    scraped off and stays in the port; carried back into the port after it was out, it is left there again; it stays
    on the towel once it has slid 10 mm over it, within 1 mm of its top, while its tuft presses on the towel with at
    least F_WIPE."""
    f_lint = np.zeros((N_TUFT, 3)) if f_lint is None else np.asarray(f_lint, float)
    if f_lint.shape != (N_TUFT, 3):
        raise ValueError("lint resistance must be an applied world-force vector per tuft")
    error=np.zeros(N_TUFT) if force_error is None else np.broadcast_to(np.asarray(force_error,float),(N_TUFT,))
    if not np.isfinite(error).all() or np.any(error<0):raise ValueError('Invalid native force uncertainty')
    load = np.linalg.norm(f_lint, axis=1)
    loaded = load - error >= F_LINT - (EPS if force_error is None else 0.0)
    uvd = to_port(tips, phone_pose)
    inside = in_cavity(uvd)
    _S["deepest"] = float(uvd[inside, 2].max()) if inside.any() else 0.0
    prev = _S["prev_tips"]
    _S["prev_tips"] = uvd
    move = np.zeros(N_TUFT) if prev is None else np.linalg.norm((uvd - prev)[:, :2], axis=1)  # across the port
    move3 = np.zeros(N_TUFT) if prev is None else np.linalg.norm(uvd - prev, axis=1)
    delta = np.zeros_like(tips) if prev is None else from_port(uvd, phone_pose) - from_port(prev, phone_pose)
    # Only resisting work counts. A stale force that assists a reversed stroke earns nothing.
    opposition = np.clip((-np.einsum("ij,ij->i", f_lint, delta)-error*move3) / np.maximum((load+error) * move3, 1e-20), 0.0, 1.0)
    resisted = loaded & (opposition > 0.0)
    drags = []
    newly_hooked = set()
    for i in range(N_LINT):
        if _S["mode"][i] != IN_PORT:
            continue
        u0, v0, d0 = LINT_UVD[i]
        col = _side(uvd, i) & (np.abs(uvd[:, 1] - v0) <= LINT_R + EPS)
        inball = col & (np.abs(uvd[:, 2] - d0) <= LINT_R + EPS)
        was = np.zeros(N_TUFT, bool) if prev is None else (
            _side(prev, i) & (np.abs(prev[:, 1] - v0) <= LINT_R + EPS) & (np.abs(prev[:, 2] - d0) <= LINT_R + EPS))
        through = inball & was & loaded & (move <= STEP_MAX + EPS)
        if through.any():
            # Positive credit is capped by one tip's across-slot resisted motion. Assistance from any loaded
            # tip is repaid at its actual work/F_LINT, including reversals from the one-control-step force lag.
            # Keep negative debt: discarding it would let a zero-net-work vibration eventually capture lint.
            credit = float((move * opposition)[through].max())
            assistance = np.maximum(0.0, np.einsum("ij,ij->i", f_lint, delta)+error*move3) / F_LINT
            _S["travel"][i] += credit - float(assistance[through].sum())
        # The virtual fibre volume exerts resistance before any outcome credit. A deep jab is also loaded.
        contact = col & (np.abs(uvd[:, 2] - d0) <= LINT_R + EPS)
        contact |= col & (uvd[:, 2] > d0) & (uvd[:, 2] < PORT_DEPTH)
        for k in np.flatnonzero(contact & (move3 > 1e-5) & (move3 <= STEP_MAX + EPS)):
            dirn = from_port(uvd[k], phone_pose) - from_port(prev[k], phone_pose)
            drags.append((int(k), -F_LINT * dirn / max(np.linalg.norm(dirn), 1e-9)))
        if np.any(col & resisted & (uvd[:, 2] > PACK_D + EPS)):
            _S["mode"][i] = PACKED
            continue
        eligible = through & (opposition > 0.0) & (move > 0.0)
        if eligible.any() and _S["travel"][i] >= CATCH_TRAVEL - EPS:
            ks = np.flatnonzero(eligible)
            k = ks[np.argmin(np.linalg.norm(uvd[ks] - LINT_UVD[i], axis=1))]
            _S["mode"][i], _S["rider"][i], _S["out"][i] = CARRIED, int(k), False
            _S["retained_work"][i], _S["capture_depth"][i] = 0.0, float(uvd[k,2])
            newly_hooked.add(i)
    for i in range(N_LINT):  # carried balls: out of the port, back into it, or onto the towel
        if _S["mode"][i] != CARRIED:
            continue
        k = _S["rider"][i]
        w, d = tips[k], uvd[k]
        last = _S["last_w"][i].copy()
        if not _S["out"][i] and i not in newly_hooked and prev is not None:
            _S["retained_work"][i] -= float(np.dot(f_lint[k], delta[k])+error[k]*move3[k])
        if not _S["out"][i] and d[2] < OUT_D:
            # Retention also needs net real work over the whole withdrawal; an opposing last interval alone
            # would permit zero-net-work force-lag pumping to extract a captured ball.
            depth0 = float(_S["capture_depth"][i])
            needed = F_LINT * max(0.0, depth0 - float(d[2]))
            paid = float(_S["retained_work"][i])
            if (not resisted[k] or move3[k] > STEP_MAX + EPS or not math.isfinite(depth0)
                    or paid + F_LINT*EPS < needed):
                if not _S["broken"]:print(f"[validity] unresisted extraction: paid {paid:.9g} J, needs {needed:.9g} J",flush=True)
                _S["broken"] = True
            else:
                _S["out"][i] = True
        retained = not _S["out"][i]
        if retained and i not in newly_hooked and prev is not None and 1e-5 < move3[k] <= STEP_MAX + EPS:
            direction = from_port(uvd[k], phone_pose) - from_port(prev[k], phone_pose)
            drags.append((int(k), -F_LINT * direction / max(np.linalg.norm(direction), 1e-9)))
        deposits = ((not _S["out"][i] and in_cavity(d) and abs(d[1]) >= SCRAPE_V) or (
                _S["out"][i] and in_cavity(d) and d[2] > BACK_IN_D))
        deposits = bool(deposits and carried_lint_contacts_phone(d, bool(_S["out"][i])))
        if deposits and prev is not None and move3[k] > 1e-5 and move3[k] <= STEP_MAX + EPS:
            direction = from_port(uvd[k], phone_pose) - from_port(prev[k], phone_pose)
            drags.append((int(k), -F_LINT * direction / max(np.linalg.norm(direction), 1e-9)))
        if deposits and resisted[k]:
            _S["mode"][i], _S["travel"][i], _S["out"][i], _S["rider"][i] = IN_PORT, 0.0, False, -1
            _S["wiped"][i], _S["last_w"][i] = 0.0, np.nan
            _S["retained_work"][i], _S["capture_depth"][i] = 0.0, np.nan
            continue
        near = (_S["out"][i] and on_towel_xy(w) and TOWEL_TOP - 0.001 <= w[2] <= TOWEL_TOP + TOWEL_GAP + EPS
                and f_towel[k] >= F_WIPE - EPS)
        if near:  # sliding over the towel, counted between consecutive steps that both press on it
            slide = float(np.linalg.norm((w - last)[:2])) if np.all(np.isfinite(last)) else 0.0
            _S["wiped"][i] += slide if slide <= STEP_MAX + EPS else 0.0
            _S["last_w"][i] = w
        else:
            _S["wiped"][i], _S["last_w"][i] = 0.0, np.nan
        if near and _S["wiped"][i] >= TOWEL_WIPE - EPS:
            _S["mode"][i], _S["world"][i] = ON_TOWEL, np.array([w[0], w[1], TOWEL_TOP + LINT_R])
    return drags


def on_towel_xy(p) -> bool:
    return bool(abs(p[0] - TOWEL_C[0]) <= TOWEL_SIZE[0] / 2 + EPS and abs(p[1] - TOWEL_C[1]) <= TOWEL_SIZE[1] / 2 + EPS)


def remaining(phone_pose, tips) -> int:
    """Lint balls in the port now: never hooked, packed, or hooked but still inside the opening."""
    out = 0
    for i in range(N_LINT):
        m = _S["mode"][i]
        if m in (IN_PORT, PACKED) or (m == CARRIED and (not _S["out"][i] or in_cavity(to_port(tips[_S["rider"][i]], phone_pose)))):
            out += 1
    return out


def on_towel() -> int:
    return int(np.sum(_S["mode"] == ON_TOWEL))


# -- end-state geometry --------------------------------------------------------------------------------------------
_BRUSH_PTS = np.concatenate([_box_surface(np.asarray(s.at), s.R, 2 * s.half) for s in HARD])


def brush_points(brush_state):
    """Points on the toothbrush's handle, neck and head, and its tuft tips (world)."""
    p, r = _pose(brush_state)
    return np.concatenate([p + _BRUSH_PTS @ r.T, brush_tips(brush_state)])


def phone_boxes(phone_state):
    p, r = _pose(phone_state)
    return [(c, rr, s) for _, c, rr, s in PHONE.world_boxes({"base": (p, r)})]


def stand_boxes(stand_state):
    p, r = _pose(stand_state)
    return [(c, rr, s) for _, c, rr, s in STAND.world_boxes({"base": (p, r)})]


def brush_set_down(objects) -> bool:
    """Lying on the counter or the towel (its lowest point within 4.5 mm of the counter top) and at least 5 mm from
    the phone and its stand."""
    from yam_arena import analysis

    br = objects["toothbrush"]
    if float(br.low[2]) > TOP + 0.0045 + EPS:
        return False
    pts = brush_points(br)
    boxes = phone_boxes(objects["phone"]) + stand_boxes(objects["phone_stand"])
    return all(float(analysis.box_signed_distance(pts, c, r, s).min()) >= 0.005 - EPS for c, r, s in boxes)


def phone_home():
    """Where the phone starts in the trial's lot."""
    return np.asarray(lots.current().pos("phone"), float)


def phone_in_place(phone_state) -> bool:
    p, r = _pose(phone_state)
    turn = math.acos(float(np.clip((np.trace(R_PHONE.T @ r) - 1) / 2, -1.0, 1.0)))
    return bool(np.linalg.norm(p - phone_home()) <= PHONE_MOVE + EPS and turn <= PHONE_TURN + EPS)


# -- scoring ---------------------------------------------------------------------------------------------------------
def _ruled_out() -> bool:
    return bool(not _S["known"] or _S["broken"] or _S["hard_in_port"] or _S["knocked"] or _S["dropped"])


def brush_joints_valid(brush_state):
    """Reject corrupt/missing/nonfinite joint coordinates even if their current geometry happens to be clear."""
    limits={j.name:(j.lower,j.upper) for j in TUFT_JOINTS}
    names=sorted(limits,key=len,reverse=True)
    covered=set()
    coordinate_keys={name:[] for name in limits}
    coordinates = getattr(brush_state, "joints", None)
    if not isinstance(coordinates, dict): return False
    for key,value in coordinates.items():
        name=key if key in limits else next((n for n in names if key.startswith(n) and key[len(n):].startswith(("_",":","."))),None)
        if name is None:return False
        try:q=float(value)
        except (TypeError,ValueError):return False
        lo,hi=limits[name]
        if not math.isfinite(q) or not lo-JOINT_SOLVER_TOL<=q<=hi+JOINT_SOLVER_TOL:return False
        covered.add(name)
        coordinate_keys[name].append(key)
    for j in TUFT_JOINTS:
        keys=coordinate_keys[j.name]
        # Offline Part.state() uses a scalar alias; native D6 exports two separate swing coordinates.
        expected=2 if getattr(j,"bend2",False) and j.name not in keys else 1
        if len(keys)!=expected:return False
    return covered==set(limits)


def audit_brush_joints(brush_state):
    if not brush_joints_valid(brush_state):
        if not _S["broken"]:print("[validity] invalid bristle joint coordinates",flush=True)
        _S["broken"]=True


def plug_seats():
    """Conservative rule proxy for the post-photo clean-port seating test; not a simulated connector."""
    return not any(m == PACKED for m in _S["mode"])


def score(objects):
    """Lint out of the port, judged once the toothbrush is set down, both hands are 30 mm clear of the phone and
    the toothbrush, and everything has been still for 1 s."""
    try:
        if _ruled_out() or not plug_seats() or not _S["clear"] or _S["hold"] < HOLD_STEPS:
            return 0.0
        ph, br = objects["phone"], objects["toothbrush"]
        if not brush_joints_valid(br):return 0.0
        if not (ph.resting() and br.resting() and phone_in_place(ph) and brush_set_down(objects)):
            return 0.0
        left = remaining(_pose(ph), brush_tips(br))
        if left == 0 and on_towel() == N_LINT:
            return 1.0
        if left == 0:
            return 0.75
        if left <= 2:
            return 0.5
        if left < N_LINT:
            return 0.25
        return 0.0
    except (KeyError, ValueError, TypeError, IndexError):
        return 0.0


def success(objects):
    return score(objects) == 1.0


# -- simulator side ---------------------------------------------------------------------------------------------------
HAND_LINKS = ("gripper", "tip_left", "tip_right")
_FACE_PTS: dict[str, np.ndarray] = {}
_LINT_OPS: list = []
_LINT_SCALE_OPS: list = []


def _hull_surface(h, spacing=0.003):
    from scipy.spatial import ConvexHull

    v = h.vertices
    out = [v]
    for tri in ConvexHull(v).simplices:
        a, b, c = v[tri]
        n = max(1, int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / spacing)))
        out.append(np.array([a + (b - a) * x / n + (c - a) * y / n for x in range(n + 1) for y in range(n + 1 - x)]))
    return np.unique(np.round(np.concatenate(out) / 0.0025).astype(np.int64), axis=0) * 0.0025


def hand_points(base) -> np.ndarray:
    """Points on the faces (every 3 mm) of both arms' gripper and finger collision hulls (env frame)."""
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


def hands_clear(points, objects) -> bool:
    from yam_arena import analysis

    br = objects["toothbrush"]
    boxes = phone_boxes(objects["phone"]) + [(c, rr, s) for _, c, rr, s in BRUSH.world_boxes(
        {"base": _pose(br), **{n: (np.asarray(p), parts._quat_to_matrix(q)) for n, (p, q) in br.links.items()}})]
    return all(float(analysis.box_signed_distance(points, c, rr, s).min()) >= HAND_CLEAR - EPS for c, rr, s in boxes)


def _arr(x):
    x = x.torch if hasattr(x, "torch") else x
    return x.detach().cpu().numpy().astype(np.float64) if hasattr(x, "detach") else np.asarray(x, float)


def _views(base=None):
    """Contact views on the tufts' segments: forces against the phone, and all contact forces."""
    if _S.get("views") is None:
        import omni.physics.tensors as tensors
        from isaaclab.app.settings_manager import get_settings_manager

        get_settings_manager().set_bool("/physics/disableContactProcessing", False)
        from isaaclab_physx.physics import PhysxManager
        sv = PhysxManager.get_physics_sim_view()
        root = parts._PATHS["toothbrush"]
        paths = [f"{root}/{n}" for n in SEGS]
        filters = [[parts._PATHS["phone"], parts._PATHS["paper_towel"] + "/s0"]] * len(paths)
        view = sv.create_rigid_contact_view(paths, filter_patterns=filters, max_contact_data_count=16 * len(paths))
        env = parts._PATHS["phone"].rsplit("/", 1)[0]
        if base is None:
            raise RuntimeError("contact views need the live scene at initialization")
        hard_paths = [f"{root}/base"]
        for arm in ("left_robot", "right_robot"):
            asset = base.scene[arm]
            link_paths = asset.root_physx_view.link_paths[0]
            hard_paths += [link_paths[list(asset.body_names).index(n)] for n in HAND_LINKS]
        hard = sv.create_rigid_contact_view(hard_paths, filter_patterns=[[parts._PATHS["phone"]]] * len(hard_paths),
                                            max_contact_data_count=2048)
        _S["views"] = (sv, view, hard)
    return _S["views"][1:]


def tuft_forces(dt):
    """Per tuft, all segments: native contact on the phone and on the towel, separately (world, N)."""
    view, _ = _views()
    forces = _arr(view.get_contact_force_matrix(dt)).reshape(N_TUFT, N_SEG, 2, 3).sum(axis=1)
    return forces[:, 0], forces[:, 1]


def tip_towel_forces(dt, tips, origin):
    """Native normal force on each end segment at points within 1 mm of its carried-fibre tip."""
    view, _ = _views()
    forces, points, _, _, counts, starts = [_arr(x) for x in view.get_contact_data(dt)]
    out = np.zeros(N_TUFT)
    counts, starts = counts.reshape(len(SEGS), 2), starts.reshape(len(SEGS), 2)
    for k, name in enumerate(TUFT_NAMES):
        segment = SEGS.index(name)
        first, count = int(starts[segment, 1]), int(counts[segment, 1])
        ids = np.arange(first, first + count)
        if len(ids):
            local = points[ids] - origin
            near = np.linalg.norm(local - tips[k], axis=1) <= LINT_R + EPS
            out[k] = float(np.abs(forces[ids].reshape(-1))[near].sum())
    return out


def hard_port_contact(dt, phone_pose, origin):
    """Native contact point and nonzero normal force from the rigid brush or a hand on the port end."""
    _, view = _views()
    forces, points, normals, separation, counts, starts = [_arr(x) for x in view.get_contact_data(dt)]
    ids = [j for count, start in zip(counts.ravel(), starts.ravel())
           for j in range(int(start), int(start + count))]
    if not ids:
        return False
    pts = points[ids] - origin
    local = (pts - phone_pose[0]) @ phone_pose[1]
    hits = (np.abs(forces[ids].ravel()) > 1e-4) & (local[:, 2] >= P_LEN / 2 - END)
    if hits.any() and not _S["hard_in_port"]:
        names=["brush/base"]+[arm+"/"+link for arm in ("left","right") for link in HAND_LINKS]
        rows=[]
        for hit in np.flatnonzero(hits)[:4]:
            j=ids[hit]
            body=next(i for i,(count,start) in enumerate(zip(counts.ravel(),starts.ravel())) if start<=j<start+count)
            rows.append(dict(body=names[body],force_N=float(forces[j].ravel()[0]),point=pts[hit].tolist()))
        print("[validity] hard port contact "+json.dumps(rows),flush=True)
    return bool(hits.any())


from contact_audit.velocity_resistance import VelocityResistance
_NATIVE_LINT=VelocityResistance({'lint':dict(phone='phone',brush='toothbrush',names=list(SENSOR_NAMES),offsets=[np.zeros(3) for k in range(N_TUFT)])},F_LINT+NATIVE_FORCE_HEADROOM)

def _native_lint_contacts(phone_pose,tips):
    uvd=to_port(tips,phone_pose);contacts=np.zeros((N_LINT,N_TUFT),bool)
    for i in range(N_LINT):
        if _S['mode'][i]==IN_PORT:
            _,v0,d0=LINT_UVD[i];col=_side(uvd,i)&(np.abs(uvd[:,1]-v0)<=LINT_R+EPS)
            contacts[i]=col&((np.abs(uvd[:,2]-d0)<=LINT_R+EPS)|((uvd[:,2]>d0)&(uvd[:,2]<PORT_DEPTH)))
        elif _S['mode'][i]==CARRIED:
            k=int(_S['rider'][i]);p=uvd[k];out=bool(_S['out'][i])
            deposits=((not out and in_cavity(p) and abs(p[1])>=SCRAPE_V) or (out and in_cavity(p) and p[2]>BACK_IN_D))
            contacts[i,k]=(not out) or (deposits and carried_lint_contacts_phone(p,out))
    return contacts

def _apply_drags(base,drags,tips):
    if not _S['known']:_NATIVE_LINT.reset()


def _draw(sim, pts):
    """Draw compressed fibre proxies where the rule has them (0.8 x 2 x 2 mm ellipsoids, no collider)."""
    from isaaclab.sim.utils import get_current_stage
    from pxr import Gf, UsdGeom

    stage = get_current_stage()
    if _LINT_OPS and _LINT_OPS[0].GetAttr().GetPrim().GetStage() != stage:
        _LINT_OPS.clear()
        _LINT_SCALE_OPS.clear()
    if not _LINT_OPS:
        env = parts._PATHS["phone"].rsplit("/", 1)[0]
        for i in range(N_LINT):
            g = UsdGeom.Sphere.Define(stage, f"{env}/lint_{i}")
            g.CreateRadiusAttr(1.0)
            g.CreateDisplayColorAttr().Set([Gf.Vec3f(*LINT)])
            _LINT_OPS.append(g.AddTranslateOp())
            qx, qy, qz, qw = parts._matrix_to_quat(R_PHONE)
            g.AddOrientOp().Set(Gf.Quatf(float(qw), Gf.Vec3f(float(qx), float(qy), float(qz))))
            _LINT_SCALE_OPS.append(g.AddScaleOp())
    for i, (op, scale, q) in enumerate(zip(_LINT_OPS, _LINT_SCALE_OPS, pts)):
        op.Set(Gf.Vec3d(*(float(v) for v in q)))
        expanded = _S["mode"][i] == ON_TOWEL or (_S["mode"][i] == CARRIED and _S["out"][i])
        scale.Set(Gf.Vec3f(float(LINT_R if expanded else 0.0004), float(LINT_R), float(LINT_R)))


@parts.on_reset
def _on_reset(sim) -> None:
    reset_state()
    if not str(sim.env.unwrapped.device).startswith("cpu"):
        raise RuntimeError("This brush requires CPU PhysX: pass -E device=cpu. CUDA field tests were rejected.")
    if len(sim.env.unwrapped.scene["toothbrush"].joint_names)!=2*len(TUFT_JOINTS):
        raise RuntimeError("Incomplete native two-swing bristle coordinates")
    try:
        _apply_drags(sim.env.unwrapped, [], np.zeros((N_TUFT, 3)))
    except Exception as exc:  # noqa: BLE001
        _S["broken"] = True
        print(f"[lint] could not clear the lint forces: {exc!r}", file=sys.stderr, flush=True)


@parts.on_step
def _on_step(sim) -> None:
    try:
        base = sim.env.unwrapped
        st = sim.states()
        ph, br = st["phone"], st["toothbrush"]
        audit_brush_joints(br)
        pp = _pose(ph)
        tips = brush_tips(br)
        if not np.all(np.isfinite(tips)):
            raise ValueError("toothbrush pose is not finite")
        _views(base)
        fp, fo = tuft_forces(float(base.physics_dt))
        overlap, _ = shaft_phone_penetrations(TUFT_JOINTS, PHONE.shapes, br, pp)
        _S["overlap"] = float(overlap.max())
        _S["overlap_peak"] = max(_S["overlap_peak"], _S["overlap"])
        if _S["overlap"] > MAX_SHAFT_OVERLAP:
            if not _S["broken"]:print(f"[validity] shaft overlap {_S['overlap']*1000:.6f} mm",flush=True)
            _S["broken"] = True
        ft = tip_towel_forces(float(base.physics_dt), tips, parts._np(base.scene.env_origins[0]))
        # The official physics-step callback owns contact updates and lint state; this callback audits the robot.
        _S["f_phone"] = float(np.linalg.norm(fp, axis=1).sum())
        _S["f_max"] = max(_S["f_max"], _S["f_phone"])
        _S["bend"] = float(max(abs(v) for v in br.joints.values()))
        _S["f_left"] = float(np.linalg.norm(_arr(_views()[1].get_contact_force_matrix(float(base.physics_dt)))[1:4], axis=-1).sum())
        _S["f_left_max"] = max(_S["f_left_max"], _S["f_left"])
        if hard_port_contact(float(base.physics_dt), pp, parts._np(base.scene.env_origins[0])):
            _S["hard_in_port"] = True
        hands = hand_points(base)
        if not phone_in_place(ph):
            _S["knocked"] = True
        if float(br.low[2]) < TOP - 0.02 or float(ph.low[2]) < TOP - 0.02:
            _S["dropped"] = True
        _S["clear"] = hands_clear(hands, st)
        _S["supported"] = brush_set_down(st)
        _S["known"] = True
        still = ph.resting() and br.resting()
        _S["hold"] = _S["hold"] + 1 if (_S["clear"] and _S["supported"] and still) else 0
        try:
            _draw(sim, lint_world(pp, tips))
        except Exception as exc:  # noqa: BLE001  (unreadable outcome must not be approved)
            _S["broken"] = True
            if _S["log_t"] < 0:
                print(f"[lint] could not draw the lint: {exc!r}", file=sys.stderr, flush=True)
        if sim.t - _S["log_t"] >= 0.5 - 1e-6:
            _S["log_t"] = sim.t
            uvd = to_port(tips, pp)
            gap = [k for k in range(N_TUFT) if in_cavity(uvd[k])]
            print("[lint] " + json.dumps(dict(
                t=round(float(sim.t), 2), mode=[int(m) for m in _S["mode"]],
                travel=[round(float(x) * 1000, 2) for x in _S["travel"]],
                wiped=[round(float(x) * 1000, 1) for x in _S["wiped"]], deepest=round(_S["deepest"] * 1000, 2),
                in_port=[[TUFT_NAMES[k], *(round(float(x) * 1000, 2) for x in uvd[k]),
                          round(float(np.linalg.norm(fp[k])), 3)] for k in gap],
                nearest=[[TUFT_NAMES[k], *(round(float(x) * 1000, 2) for x in uvd[k])] for k in
                         sorted(range(N_TUFT), key=lambda j: abs(uvd[j, 0]) + abs(uvd[j, 1]))[:8]],
                f_phone=round(_S["f_phone"], 3), f_phone_peak=round(_S["f_max"], 3), overlap_peak_mm=round(_S["overlap_peak"]*1000, 4), f_towel=round(float(ft.sum()), 3), f_towel_all=round(float(np.linalg.norm(fo, axis=1).sum()), 3),
                overlap_mm=round(_S["overlap"] * 1000, 4), f_left=round(_S.get("f_left", 0.0), 3), f_left_peak=round(_S["f_left_max"], 3), retained_work_uj=[round(float(x)*1e6,3) for x in _S["retained_work"]], drag=round(_S["drag"], 3), bend_deg=round(math.degrees(_S["bend"]), 1),
                left=remaining(pp, tips), towel=on_towel(), hard=_S["hard_in_port"], knocked=_S["knocked"],
                dropped=_S["dropped"], clear=_S["clear"], supported=_S["supported"], hold=_S["hold"],
                score=score(st), broken=_S["broken"], phone=[round(float(x), 4) for x in ph.pos],
                brush=[round(float(x), 4) for x in br.pos], brush_q=[round(float(x), 4) for x in br.quat_xyzw])),
                flush=True)
    except Exception as exc:  # unreadable poses, contacts or hands: no credit for this trial
        _S["broken"] = True
        print(f"[lint] step check failed: {exc!r}", file=sys.stderr, flush=True)


# -- scripted head grasp and contact brace; no attachment to the gripper -------------------------------------
GRIP = 0.0
GRASP_Y = HEAD_Y
HEAD_GRASP_Z = -0.0035
TCP_ABOVE = 0.010
CARRY_Z = TOP + 0.184
YAW = math.pi/2

def demo():
 work_yaw=pick_yaw=YAW
 grasp_z=HEAD_GRASP_Z
 r=parts._rot((0,0,1),work_yaw-pick_yaw) @ R_BRUSH
 def wp(d,p,f,l,y=work_yaw): return Waypoint(d,tuple(float(x) for x in p),f,l,y)
 ph=np.asarray(lots.current().pos('phone'))
 hold=np.asarray(lots.current().pos('toothbrush'))
 grasp=hold+R_BRUSH@[0,GRASP_Y,grasp_z]+[0,0,TCP_ABOVE]
 m,u,v,a=port_frame((ph-R_PHONE@[0,0,GAP],R_PHONE))
 # Columns 4 and 7 have opposed lean and nearly coincident tip lines.
 ks=[k for k,(name,*_) in enumerate(TUFTS) if int(name.split('_')[1]) in (4,7)]
 tip=r@(TIP0[ks].mean(axis=0)-[0,GRASP_Y,grasp_z])-[0,0,TCP_ABOVE]
 low=r@np.array([0,TIP0[:,1].mean()-GRASP_Y,TIP_Z-grasp_z])-[0,0,TCP_ABOVE]
 wipe_base=np.array([TOWEL_C[0],TOWEL_C[1],TOWEL_TOP-0.0005])-low
 w=[wp(2,grasp+[0,0,.08],FINGER_OPEN,'over brush head',pick_yaw),wp(3,grasp,FINGER_OPEN,'around brush head',pick_yaw),wp(3,grasp,FINGER_OPEN,'settle before grip',pick_yaw),wp(1.2,grasp,GRIP,'grip brush head',pick_yaw),wp(2,grasp+[0,0,.06],GRIP,'lift',pick_yaw),wp(2,[grasp[0],grasp[1],CARRY_Z],GRIP,'raise brush')]
 for su in (1,-1):
  wipe=wipe_base+np.array([-.020 if su==1 else .020,0.,0.])  # keep the two deposits visibly separate
  if abs(work_yaw-pick_yaw)<1e-8:
   row=2 if su==1 else 1
   ks=[k for k,(name,*_) in enumerate(TUFTS) if name.startswith(f"t{row}_")]
   tip=r@(TIP0[ks].mean(axis=0)-[0,GRASP_Y,grasp_z])-[0,0,TCP_ABOVE]
  mouth=m+su*.000815*u-tip
  into=mouth-a*.0055
  above=mouth+a*.001
  w += [wp(2,[above[0],above[1],CARRY_Z],GRIP,'over port'),wp(1.2,above,GRIP,'line up'),wp(3,into-v*.0010,GRIP,'press compliant tips'),wp(2,into+v*.0010,GRIP,'stroke along gap'),wp(2,into-v*.0010,GRIP,'return stroke'),wp(.8,into,GRIP,'centre'),wp(2,above,GRIP,'withdraw'),wp(1,[above[0],above[1],CARRY_Z],GRIP,'clear port'),wp(2,[wipe[0],wipe[1],CARRY_Z],GRIP,'over towel'),wp(2,wipe,GRIP,'press on towel'),wp(2,wipe+[0,-.025,0],GRIP,'wipe lint off'),wp(1.5,[wipe[0],wipe[1]-.025,CARRY_Z],GRIP,'lift clean brush')]
 release=sum(q.duration for q in w)
 w += [wp(2,grasp+[0,0,.06],GRIP,'return brush',pick_yaw),wp(1.5,grasp+[0,0,.0015],GRIP,'lay down',pick_yaw),wp(.8,grasp+[0,0,.0015],FINGER_OPEN,'release',pick_yaw),wp(1,grasp+[0,0,.08],FINGER_OPEN,'up',pick_yaw),wp(1.5,[.15,-.26,TOP+.20],FINGER_OPEN,'clear',pick_yaw),wp(3,[.15,-.26,TOP+.20],FINGER_OPEN,'wait',pick_yaw)]
 total=sum(q.duration for q in w)
 # Low side approach keeps the left forearm out of the right hand's working volume.
 bx=float(EDGE[0]+.01882);outside=float(ph[1]+.170);near=float(ph[1]+.1035);bz=TOP+.022
 def lw(d,p,l):return wp(d,p,-.045,l,-math.pi/4)
 side=(bx,near,bz);away=(bx,outside,bz)
 left=[lw(3,(bx,outside,TOP+.18),'outside phone'),lw(4,away,'lower outside phone'),lw(3,side,'approach phone corner'),lw(release-10,side,'steady phone'),lw(3,away,'release phone'),lw(2,(bx,outside,TOP+.18),'up'),lw(2,[.15,.26,TOP+.2],'clear'),lw(max(1,total-release-7),[.15,.26,TOP+.2],'wait')]
 return {'right':w,'left':left}

_full_demo=demo
_GRASP_VIEW=None
@parts.on_step
def _robot_diagnostic(sim):
    global _GRASP_VIEW
    if int(round(float(sim.t)*20))%10!=1:return
    try:
        from yam_arena import ik,station
        base=sim.env.unwrapped;row={'t':round(float(sim.t),3)}
        for arm in ('left','right'):
            asset=base.scene[arm+'_robot'];names=list(asset.joint_names)
            actual=_arr(asset.data.joint_pos)[0];target=_arr(asset.data.joint_pos_target)[0]
            q=np.array([actual[names.index('joint'+str(i))] for i in range(1,7)])
            qt=np.array([target[names.index('joint'+str(i))] for i in range(1,7)])
            p,r=ik.tcp_pose(q,station.BASES[arm]);pt,rt=ik.tcp_pose(qt,station.BASES[arm])
            row[arm]={'tcp':p.tolist(),'target_tcp':pt.tolist(),'tracking_mm':float(np.linalg.norm(p-pt))*1000,
                      'joints':dict(zip(names,actual.tolist())),'target_joints':dict(zip(names,target.tolist())),
                      'root_pose':_arr(asset.data.root_link_pose_w)[0].tolist(),
                      'hands':{name:_arr(asset.data.body_link_pose_w)[0,list(asset.body_names).index(name)].tolist() for name in HAND_LINKS}}
        if _GRASP_VIEW is None:
            sv=_S['views'][0];root=parts._PATHS['toothbrush'];paths=[root+'/base']+[root+'/'+n for n in SEGS]
            asset=base.scene['right_robot'];bodypaths=asset.root_physx_view.link_paths[0]
            filters=[bodypaths[list(asset.body_names).index(n)] for n in HAND_LINKS]
            _GRASP_VIEW=sv.create_rigid_contact_view(paths,filter_patterns=[filters]*len(paths),max_contact_data_count=4096)
        forces=_arr(_GRASP_VIEW.get_contact_force_matrix(float(base.physics_dt)))
        row['grasp_base_N']=float(np.linalg.norm(forces[0],axis=-1).sum());row['grasp_field_N']=float(np.linalg.norm(forces[1:],axis=-1).sum())
        print('[robot] '+json.dumps(row),flush=True)
    except Exception as exc:
        import traceback
        traceback.print_exc();print('[robot] diagnostic unavailable '+repr(exc),flush=True)


# Read-only USD local-shape audit, independent of the analytical source transform.
@parts.on_reset
def _native_shape_coordinates(sim):
    from isaaclab.sim.utils import get_current_stage
    from pxr import UsdGeom,Gf
    stage=get_current_stage();corners=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])
    errors=[];count=0
    for j in TUFT_JOINTS:
        for i,sh in enumerate(j.shapes):
            if not sh.collide or sh.kind!='box':continue
            path=parts._PATHS['toothbrush']+'/'+j.name+'/s'+str(i)
            prim=stage.GetPrimAtPath(path)
            if not prim.IsValid():raise RuntimeError('Missing native collider '+path)
            matrix=UsdGeom.Xformable(prim).GetLocalTransformation()
            native=np.array([list(matrix.Transform(Gf.Vec3d(*map(float,v)))) for v in corners])
            expected=np.asarray(sh.at)-j.origin+(corners*np.asarray(sh.size))@sh.R.T
            error=np.linalg.norm(native[:,None,:]-expected[None,:,:],axis=-1).min(axis=1).max()
            errors.append(float(error));count+=1
    print('[geometry] '+json.dumps(dict(shafts=count,max_local_corner_error_mm=max(errors)*1000)),flush=True)

# Read-only closure checks for nominally locked joint translation/twist.
_JOINT_PEAK=[0.,0.]
@parts.on_reset
def _reset_joint_measurement(sim):
    _JOINT_PEAK[:]=[0.,0.]
@parts.on_step
def _native_joint_measurement(sim):
    st=sim.states()['toothbrush'];gaps=[];twists=[]
    for j in TUFT_JOINTS:
        if j.parent=='base':p0,r0=_pose(st)
        else:
            p0,q0=st.links[j.parent];p0=np.asarray(p0);r0=parts._quat_to_matrix(q0)
        p1,q1=st.links[j.name];r1=parts._quat_to_matrix(q1)
        anchor=p0+r0@(j.origin-BRUSH.joint_origin(j.parent))
        gaps.append(float(np.linalg.norm(anchor-np.asarray(p1))))
        fr=parts._x_to(j.axis);rel=(r0@fr).T@(r1@fr);q=parts._matrix_to_quat(rel)
        twist=2*math.atan2(float(q[0]),float(q[3]));twists.append(abs((twist+math.pi)%(2*math.pi)-math.pi))
    gm=max(gaps);tm=max(twists);_JOINT_PEAK[0]=max(_JOINT_PEAK[0],gm);_JOINT_PEAK[1]=max(_JOINT_PEAK[1],tm)
    if int(round(float(sim.t)*20))%10==1:
        print('[joint_closure] '+json.dumps(dict(t=round(float(sim.t),3),max_anchor_mm=gm*1000,peak_anchor_mm=_JOINT_PEAK[0]*1000,max_twist_deg=math.degrees(tm),peak_twist_deg=math.degrees(_JOINT_PEAK[1]),anchor_joint=TUFT_JOINTS[int(np.argmax(gaps))].name,twist_joint=TUFT_JOINTS[int(np.argmax(twists))].name)),flush=True)

# Controller calibration from a virtual held-pickup run, not a physical product reading.
TOOL_T=np.array([0.00019536370387180683, 0.08443655175133875, 0.013079078974164968])
TOOL_R=np.array([[0.9999884643815251, -0.004802392908463982, -9.01456158395447e-05], [-0.004801969451190004, -0.9999795544746128, 0.0042227623831459], [-0.0001104231368880113, -0.004222280794476716, -0.999991080036029]])
WORK_YAW=1.565994315666521

@parts.on_step
def _native_resistance_diagnostics(sim):
    if int(round(float(sim.t)*20))%10==1:
        print('[native_lint] '+json.dumps(dict(t=round(float(sim.t),3),active=int(_NATIVE_LINT.active['lint'].sum()),native_total_force_N=float(_S['drag']),travel_mm=(_S['travel']*1000).tolist(),retained_work_uj=(_S['retained_work']*1e6).tolist())),flush=True)

# The task scorer remains active for this complete two-gap robot candidate.

# Experimental official physics-step recorder. Native poses/velocities are read directly, without cached data.
_MICRO_BASE=None;_MICRO_SUB=None;_MICRO_IDS=None;_MICRO_N=0;_MICRO_ERRORS=0;_CONTROL_PREV=None;_MICRO_NEAR=False
_MICRO_OFFSETS=np.array([R_BRUSH@(TIP0[k]-KNEE0[k]) for k in range(N_TUFT)])
def _native_array(value):
    if hasattr(value,'torch'):value=value.torch
    if hasattr(value,'detach'):return value.detach().cpu().numpy()
    if hasattr(value,'numpy'):return value.numpy()
    return np.asarray(value)
def _rotate_many(q,v):
    return v+2*np.cross(q[:,:3],np.cross(q[:,:3],v)+q[:,3:4]*v)
def _micro_lint(dt):
    global _MICRO_IDS,_MICRO_N,_MICRO_ERRORS
    if _MICRO_BASE is None or not _S['known'] or not _MICRO_NEAR:return
    if _S['broken']:
        _NATIVE_LINT.set_counts('lint',np.zeros(N_TUFT,int));return
    try:
        base=_MICRO_BASE;brush=base.scene['toothbrush'];bv=brush.root_physx_view
        if _MICRO_IDS is None:
            paths=bv.link_paths[0];_MICRO_IDS=[next(i for i,x in enumerate(paths) if x.endswith('/'+n)) for n in TUFT_NAMES]
        poses=_native_array(bv.get_link_transforms())[0,_MICRO_IDS]
        offs=_rotate_many(poses[:,3:],_MICRO_OFFSETS)
        tipworld=poses[:,:3]+offs
        phone=base.scene['phone'];pose=_native_array(phone.root_physx_view.get_transforms())[0]
        rot=parts._quat_to_matrix(pose[3:]);origin=parts._np(base.scene.env_origins[0])
        overlap,name,pair=_MICRO_SHAFT.observe(bv,pose[:3],rot)
        _S['overlap_peak']=max(_S['overlap_peak'],overlap)
        if overlap>MAX_SHAFT_OVERLAP:raise RuntimeError('Per-physics shaft overlap '+str((name,pair,overlap)))
        counts=_NATIVE_LINT.counts['lint'].copy()
        total_force,_=_NATIVE_FORCES.observe(base,float(dt),counts,_NATIVE_LINT.cap)
        forces=total_force/np.maximum(counts,1)[:,None]
        uncertainty=CREDIT_MARGIN_N+float(_NATIVE_FORCES.last.get('coverage_error_N',0.))
        tips=tipworld-origin;pp=(pose[:3]-origin,rot)
        if not np.isfinite(tips).all() or not np.isfinite(forces).all():raise RuntimeError('Nonfinite native lint receipt')
        ft=np.zeros(N_TUFT)
        near_towel=(np.abs(tips[:,0]-TOWEL_C[0])<=TOWEL_SIZE[0]/2)&(np.abs(tips[:,1]-TOWEL_C[1])<=TOWEL_SIZE[1]/2)&(np.abs(tips[:,2]-TOWEL_TOP)<.002)
        if near_towel.any():ft=tip_towel_forces(float(dt),tips,origin)
        _S['applied_lint']=forces
        record(pp,tips,ft,forces,force_error=uncertainty)
        contacts=np.zeros((N_LINT,N_TUFT),bool) if _S['broken'] else _native_lint_contacts(pp,tips)
        _NATIVE_LINT.set_counts('lint',contacts.sum(axis=0));_S['native_contacts']=contacts
        _S['drag']=float(np.linalg.norm(total_force,axis=1).sum())
        _S['native_force_balance']=dict(_NATIVE_FORCES.last)
        _MICRO_N+=1
    except Exception as exc:
        _MICRO_ERRORS+=1;_S['broken']=True
        import traceback
        traceback.print_exc();print('[micro_lint_error] '+repr(exc),flush=True)
@parts.on_reset
def _install_micro_lint(sim):
    global _MICRO_BASE,_MICRO_SUB,_MICRO_IDS,_MICRO_N,_MICRO_ERRORS,_MICRO_PHONE_COM,_CONTROL_PREV,_MICRO_COM_LOCAL,_MICRO_NEAR
    import omni.physx
    _MICRO_BASE=sim.env.unwrapped;_MICRO_IDS=None;_MICRO_N=0;_MICRO_ERRORS=0;_CONTROL_PREV=None;_MICRO_NEAR=False
    brush=_MICRO_BASE.scene['toothbrush'];_MICRO_COM_LOCAL=_native_array(brush.data.body_com_pose_b)[0,[list(brush.body_names).index(n) for n in TUFT_NAMES],:3].copy()
    _NATIVE_LINT.reset();_MICRO_SHAFT.ids=None
    phone=_MICRO_BASE.scene['phone'];pose=parts._np(phone.data.root_link_pose_w)[0]
    _MICRO_PHONE_COM=parts._quat_to_matrix(pose[3:]).T@(parts._np(phone.data.body_com_pos_w)[0,0]-pose[:3])
    if _MICRO_SUB is None:_MICRO_SUB=omni.physx.get_physx_interface().subscribe_physics_step_events(_micro_lint)
@parts.on_step
def _micro_control_guard(sim):
    global _CONTROL_PREV,_MICRO_NEAR
    tips=brush_tips(sim.states()['toothbrush']);pp=_pose(sim.states()['phone']);uvd=to_port(tips,pp)
    if _CONTROL_PREV is not None:
        moved=np.linalg.norm(uvd-_CONTROL_PREV,axis=1)
        # Preserve the original3mm per-control-step travel exclusion alongside finer physical contact updates.
        if np.any((in_cavity(uvd)|in_cavity(_CONTROL_PREV))&(moved>STEP_MAX+EPS)):
            _S['broken']=True;print('[validity] excessive control-interval tip travel',flush=True)
    _CONTROL_PREV=uvd.copy()
    _MICRO_NEAR=bool(np.any(np.linalg.norm(uvd,axis=1)<.030) or np.any(_S['mode']==CARRIED))
    if not _MICRO_NEAR:
        _NATIVE_LINT.set_counts('lint',np.zeros(N_TUFT,int));_S['prev_tips']=None

    if int(round(float(sim.t)*20))%10==1:
        print('[micro_lint] '+json.dumps(dict(t=round(float(sim.t),3),callbacks=_MICRO_N,errors=_MICRO_ERRORS,active=int(_NATIVE_LINT.active['lint'].sum()),force_N=_S['drag'],travel_mm=(_S['travel']*1000).tolist(),mode=_S['mode'].tolist(),force_balance=_S.get('native_force_balance',{}))),flush=True)


from contact_audit.tip_force_sensors import TipForceSensors as NativeLeafForces,CREDIT_MARGIN_N
_NATIVE_FORCES=NativeLeafForces('toothbrush',TUFT_NAMES,ALL_BRUSH_JOINTS)
@parts.on_reset
def _reset_native_force_balance(sim):_NATIVE_FORCES.reset()
@parts.on_step
def _native_balance_log(sim):
    if int(round(float(sim.t)*20))%10==1:print('[native_balance] '+json.dumps(dict(t=float(sim.t),**_NATIVE_FORCES.last)),flush=True)

from contact_audit.geometry import box_penetrations
from contact_audit.native_shaft_audit import NativeShaftAudit
_MICRO_SHAFT=NativeShaftAudit(TUFT_JOINTS,PHONE.shapes,box_penetrations)
_sensor_author=parts._author
def _author_sensor_inertia(part,root_path,translation,orientation):
    result=_sensor_author(part,root_path,translation,orientation)
    from isaaclab.sim.utils import get_current_stage
    from pxr import UsdPhysics,PhysxSchema
    for prim in get_current_stage().Traverse():
        if prim.IsA(UsdPhysics.Scene):
            api=PhysxSchema.PhysxSceneAPI.Apply(prim);api.CreateSolverTypeAttr('PGS');api.CreateEnableExternalForcesEveryIterationAttr(False)

    if part.name=='toothbrush':
        from isaaclab.sim.utils import get_current_stage
        from pxr import UsdPhysics,Gf
        stage=get_current_stage()
        from pxr import PhysxSchema
        for name in SEGS:PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(root_path+'/'+name)).CreateEnableSpeculativeCCDAttr(True)
        for name in SENSOR_NAMES:
            mass=UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(root_path+'/'+name))
            mass.CreateCenterOfMassAttr(Gf.Vec3f(0.));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-15));mass.CreatePrincipalAxesAttr(Gf.Quatf(1.))
    return result
parts._author=_author_sensor_inertia

@parts.on_reset
def _solver_receipt(sim):
 from isaaclab.sim.utils import get_current_stage
 from pxr import UsdPhysics,PhysxSchema
 print('[solver_settings] '+json.dumps(dict(dt=sim.env.unwrapped.physics_dt,scenes=[dict(path=str(p.GetPath()),solver=PhysxSchema.PhysxSceneAPI(p).GetSolverTypeAttr().Get(),external_each_iteration=PhysxSchema.PhysxSceneAPI(p).GetEnableExternalForcesEveryIterationAttr().Get()) for p in get_current_stage().Traverse() if p.IsA(UsdPhysics.Scene)])),flush=True)

# Virtual held-tool calibration atPGS1920/128; not a physical product measurement.
TOOL_T=np.array([-0.0012215147833655928, 0.08480781196677939, 0.013409024876335964])
TOOL_R=np.array([[0.9999572635605252, 0.008464495025267676, 0.0037179801658677827], [0.008435054484470926, -0.9999335015676399, 0.007863987444200134], [0.0037842976086154105, -0.007832290000105416, -0.9999621664467931]])
WORK_YAW=1.5792313667654574
# Scripted contact work, using virtual held-tool calibration; no object pose or grip attachment writes.
def demo():
 from yam_arena.demo import grip_rotation
 plan=_full_demo();right=plan['right'][:6];rotation=grip_rotation(WORK_YAW)
 ph=np.asarray(lots.current().pos('phone'));m,u,v,a=port_frame((ph-R_PHONE@[0,0,GAP],R_PHONE))
 held_tips=(R_BRUSH@TIP0.T).T@TOOL_R.T+TOOL_T
 held_world=held_tips@rotation.T
 def w(d,p,l,f=GRIP,y=WORK_YAW):return Waypoint(d,tuple(map(float,p)),f,l,y)
 for su,row in [(1,2),(-1,1)]:
  ks=[k for k,n in enumerate(TUFT_NAMES) if n.startswith('t'+str(row)+'_')]
  tip=held_world[ks].mean(axis=0)
  mouth=m+su*.000815*u-tip;above=mouth+.001*a;into=mouth-.0055*a
  comp=-su*.00125*u+.0005*v-.0065*a  # 169 (PGS, CCD100) delivered 6.3-6.6 mm and packed; 172 (7.1 mm) delivered 5.3-5.7 mm
  right += [w(2.,[above[0],above[1],CARRY_Z],'over port'),w(1.2,above,'align calibrated tool'),w(1.,above,'settle alignment'),w(6.,into-.00125*v+comp,'press compliant tips'),w(2.5,into+.0015*v+comp,'stroke along gap'),w(3.,into-.00375*v+comp,'return stroke'),w(3.,into+.0015*v+comp,'second outward stroke'),w(3.,into-.00375*v+comp,'second return stroke'),w(.8,into+comp,'centre'),w(3.5,above,'slow withdrawal'),w(1.,[above[0],above[1],CARRY_Z],'clear port')]
  # Contacting rider tips are planned0.5mm below the towel surface; actual native contact still gates transfer.
  ride=[k for k,n in enumerate(TUFT_NAMES) if n.startswith('t'+str(row)+'_') and int(n.split('_')[1]) in (4,7)]
  wipe=np.array([TOWEL_C[0]+(-.020 if su==1 else .020),TOWEL_C[1],TOWEL_TOP-.0005])-held_world[ride].mean(axis=0)
  right += [w(1.5,[wipe[0],wipe[1],CARRY_Z],'over towel'),w(2.,wipe,'press on towel'),w(2.,wipe+[0,-.025,0],'wipe lint off'),w(1.,[wipe[0],wipe[1]-.025,CARRY_Z],'lift clean brush')]
 release=sum(q.duration for q in right)
 # Return through the original pickup frame after both contact passes.
 grasp=np.asarray(lots.current().pos('toothbrush'))+R_BRUSH@[0,GRASP_Y,HEAD_GRASP_Z]+[0,0,TCP_ABOVE]
 right += [w(2.,grasp+[0,0,.06],'return brush',y=YAW),w(1.5,grasp+[0,0,.0015],'lay down',y=YAW),w(.8,grasp+[0,0,.0015],'release',FINGER_OPEN,YAW),w(1.,grasp+[0,0,.08],'up',FINGER_OPEN,YAW),w(1.5,[.15,-.26,TOP+.20],'clear',FINGER_OPEN,YAW),w(3.,[.15,-.26,TOP+.20],'wait',FINGER_OPEN,YAW)]
 total=sum(q.duration for q in right)
 bx=float(EDGE[0]+ph[0]-PHONE_POS[0]+.01882);outside=float(ph[1]+.170);near=float(ph[1]+.1035);bz=TOP+.022
 def lw(d,p,l):return w(d,p,l,-.045,-math.pi/4)
 side=(bx,near,bz);away=(bx,outside,bz)
 left=[lw(3.,(bx,outside,TOP+.18),'outside phone'),lw(4.,away,'lower outside phone'),lw(3.,side,'approach phone corner'),lw(release-10.,side,'steady phone'),lw(3.,away,'release phone'),lw(2.,(bx,outside,TOP+.18),'up'),lw(2.,[.15,.26,TOP+.20],'clear'),lw(max(1.,total-release-7.),[.15,.26,TOP+.20],'wait')]
 assert total<=90.,total
 return {'left':left,'right':right}

@parts.on_reset
def _ccd_receipt(sim):
 from isaaclab.sim.utils import get_current_stage
 from pxr import PhysxSchema
 stage=get_current_stage();count=sum(bool(PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(parts._PATHS['toothbrush']+'/'+n)).GetEnableSpeculativeCCDAttr().Get()) for n in SEGS)
 if count!=len(SEGS):raise RuntimeError('Missing native angularCCD on shaft')
 print('[ccd_settings] '+json.dumps(dict(shafts=count,contact_offset_m=CONTACT)),flush=True)

@parts.on_reset
def _armature_receipt(sim):
 try:
  values=_native_array(sim.env.unwrapped.scene['toothbrush'].root_physx_view.get_dof_armatures()).reshape(-1)
  print('[armature_settings] '+json.dumps(dict(dofs=len(values),min_kgm2=float(values.min()),max_kgm2=float(values.max()))),flush=True)
 except Exception as exc:print('[armature_diagnostic_error] '+repr(exc),flush=True)
