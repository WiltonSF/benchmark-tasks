"""On the station table, at a tactile-sensor build bench: an unpainted TacTip optical tactile skin (black cast
rubber, a 40 mm dome with 532 short pins and a cylindrical skirt), turned inside out so its pins face outward, sits
on an orange printed painting former with a square handle. The former hangs by its flange in an empty 250 mL glass
beaker, the dome inside the beaker and clear of the glass. A clear square acrylic bar lies on the table with white
enamel paint freshly spread on one end; that painted end faces the beaker's side of the table.
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
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
EPS = 1e-6

BLACK, WHITE, ORANGE = (0.04, 0.04, 0.045), (0.96, 0.96, 0.94), (0.93, 0.45, 0.10)
GLASS, ACRYLIC = (0.80, 0.88, 0.92), (0.86, 0.92, 0.95)

# -- the skin on its former (former frame F: origin at the dome's centre, +z up the handle; the apex is at -z) ----
R_FORMER = 0.019  # the former's dome and body (the everted skin's inside)
R_SKIN = 0.020  # the skin's surface, 1 mm of Vytaflex 60 over the former: the pins stand on it
PIN_L, PIN_D = 0.002, 0.001  # task-designed pin length and hexagon across corners; specimen/pins.csv
R_TIP = R_SKIN + PIN_L  # the pin tips' sphere
N_PINS = 532  # Open TacTip cast skin: 40 mm dome, 532 pins (Lepora 2021, Table I)
SKIRT_TOP = 0.025  # the skin's cylindrical skirt over the former's body, from the dome's equator up 25 mm
SHANK = (0.012, SKIRT_TOP, 0.031)  # bare former above the skirt: radius, bottom, top
FLANGE = (0.040, 0.031, 0.035)  # 80 mm flange that hangs the former on the beaker's rim
HANDLE = (0.010, 0.035, 0.075)  # 20 mm square handle
GRIP_Z = 0.052  # the TCP on the handle's axis: the fingertips 4 mm above the flange
FORMER_MASS = 0.040  # printed PLA former (about 33 g at 20 % infill, an estimate) and the 6.5 g skin


def _pin_layout():
    """532 pin directions (unit vectors in F, pointing out of the dome) in rings round the apex. The cast skin's
    exact layout is not published: rings of near-equal 2.2 mm spacing out to 87 degrees from the apex."""
    rings = 15
    d_theta = math.radians(87.0) / (rings - 1 + 0.5)
    thetas = [0.0] + [d_theta * (k + 0.5) for k in range(1, rings)]
    raw = [1] + [2 * math.pi * math.sin(t) / d_theta for t in thetas[1:]]
    scale = (N_PINS - 1) / sum(raw[1:])
    counts = [1] + [max(6, int(round(r * scale))) for r in raw[1:]]
    while sum(counts) > N_PINS:
        counts[-1 - (sum(counts) - N_PINS) % (rings - 1)] -= 1
    while sum(counts) < N_PINS:
        counts[-1 - (N_PINS - sum(counts)) % (rings - 1)] += 1
    dirs, ring_of = [], []
    for k, (t, n) in enumerate(zip(thetas, counts)):
        for j in range(n):
            a = 2 * math.pi * (j + 0.5 * (k % 2)) / n
            dirs.append((math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), -math.cos(t)))
            ring_of.append(k)
    return np.array(dirs), np.array(ring_of), [round(math.degrees(t), 2) for t in thetas], counts


PIN_DIRS, PIN_RING, RING_DEG, RING_COUNT = _pin_layout()
assert len(PIN_DIRS) == N_PINS
PIN_THETA_MAX = float(np.degrees(np.arccos(-PIN_DIRS[:, 2])).max())


def _orth(d):
    a = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(d, a)
    u /= np.linalg.norm(u)
    return u, np.cross(d, u)


def _pin_mesh():
    """All 532 pins as one black mesh: hexagonal posts 1 mm across from the skin to the tips."""
    pts, tris = [], []
    for d in PIN_DIRS:
        u, v = _orth(d)
        ring = [(math.cos(2 * math.pi * k / 6) * u + math.sin(2 * math.pi * k / 6) * v) * PIN_D / 2 for k in range(6)]
        b = len(pts)
        pts += [tuple(R_SKIN * 0.995 * d + r) for r in ring] + [tuple(R_TIP * d + r) for r in ring]
        pts.append(tuple(R_TIP * d))
        for k in range(6):
            k2 = (k + 1) % 6
            tris += [(b + k, b + k2, b + 6 + k2), (b + k, b + 6 + k2, b + 6 + k), (b + 6 + k, b + 6 + k2, b + 12)]
    return tuple(pts), tuple(tris)


def _cap_visual(i):
    """Pin i's white paint: a hexagon on its tip, hidden until the pin is painted."""
    d = PIN_DIRS[i]
    u, v = _orth(d)
    c = (R_TIP + 0.00008) * d
    ring = [c + (math.cos(2 * math.pi * k / 6) * u + math.sin(2 * math.pi * k / 6) * v) * PIN_D * 0.62 for k in range(6)]
    pts = tuple(tuple(float(x) for x in p) for p in [c, *ring])
    tris = tuple((0, 1 + k, 1 + (k + 1) % 6) for k in range(6))
    return parts.Visual(f"cap{i}", pts, tris, color=WHITE, visible=False)


def _tip_hull_points():
    """The pins' tip envelope (a hemisphere of radius R_TIP): the dome's collider, hidden."""
    pts = [(0.0, 0.0, -R_TIP)]
    for k in range(1, 12):
        t = math.radians(90.0 * k / 11)
        n = 24
        for j in range(n):
            a = 2 * math.pi * (j + 0.5 * (k % 2)) / n
            pts.append((R_TIP * math.sin(t) * math.cos(a), R_TIP * math.sin(t) * math.sin(a), -R_TIP * math.cos(t)))
    return pts


def _former_shapes():
    out = [
        parts.hull(_tip_hull_points()),  # the pin tips (collider only)
        parts.sphere(R_SKIN, color=BLACK, collide=False),  # the skin of the dome, between the pins
        parts.cylinder(R_SKIN, SKIRT_TOP, at=(0.0, 0.0, SKIRT_TOP / 2), color=BLACK),  # the skin's skirt
        parts.cylinder(SHANK[0], SHANK[2] - SHANK[1], at=(0.0, 0.0, (SHANK[1] + SHANK[2]) / 2), color=ORANGE),
        parts.cylinder(FLANGE[0], FLANGE[2] - FLANGE[1], at=(0.0, 0.0, (FLANGE[1] + FLANGE[2]) / 2), color=ORANGE),
        parts.box((2 * HANDLE[0], 2 * HANDLE[0], HANDLE[2] - HANDLE[1]), at=(0.0, 0.0, (HANDLE[1] + HANDLE[2]) / 2),
                  color=ORANGE),  # 20 mm square handle
    ]
    return out


def _former_visuals():
    p, t = _pin_mesh()
    return (parts.Visual("pins", p, t, color=BLACK), *[_cap_visual(i) for i in range(N_PINS)])


# -- the beaker (Corning Pyrex 1000-250, 250 mL low-form Griffin: 68 mm across, 90 mm tall) ----------------------
BK_R, BK_H, BK_WALL, BK_BASE = 0.034, 0.090, 0.0018, 0.0025
BK_RIN = BK_R - BK_WALL
BK_MASS = 0.110  # an estimate for 250 mL borosilicate
BK_STAVES = 28


def _beaker_shapes():
    out = [parts.cylinder(BK_R, BK_BASE, at=(0.0, 0.0, BK_BASE / 2), color=GLASS)]
    rm = BK_R - BK_WALL / 2
    w = 2 * math.pi * rm / BK_STAVES * 1.06
    for k in range(BK_STAVES):
        a = 2 * math.pi * k / BK_STAVES
        out.append(parts.box((BK_WALL, w, BK_H - BK_BASE), at=(rm * math.cos(a), rm * math.sin(a), (BK_H + BK_BASE) / 2),
                             color=GLASS, quat=(0.0, 0.0, math.sin(a / 2), math.cos(a / 2))))
    return out


# -- the paint bar: clear cast acrylic square bar, 1.5 in (38.1 mm), cut to 6 in (152.4 mm), white enamel on one end
BAR_R, BAR_L = 0.01905, 0.1524  # half its width, its length
ACRYLIC_DENSITY = 1190.0  # kg/m3, PERSPEX cast acrylic typical physical properties; calculated mass, not weighed
BAR_MASS = (2 * BAR_R) ** 2 * BAR_L * ACRYLIC_DENSITY
WET_R = 0.016  # the wet paint: a 32 mm circle centred on the end face
PAINT_T = 0.00015  # the enamel film, 0.15 mm (an estimate)


def _bar_shapes():
    return [
        parts.box((BAR_L, 2 * BAR_R, 2 * BAR_R), color=ACRYLIC),  # along the part's x
        parts.cylinder(WET_R, PAINT_T, at=(BAR_L / 2 + PAINT_T / 2, 0.0, 0.0), color=WHITE, axis="x", collide=False),
    ]


# -- where things are (lot A: beaker on the left arm's side, the bar on the right arm's) ---------------------------
BK_POS = (0.235, 0.195)
FORMER_HANG_Z = TOP + BK_H - FLANGE[1] + 0.0004  # its flange on the rim
BAR_POS = (0.300, -0.085)
BAR_Z = TOP + BAR_R + 0.0003


def _qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


FORMER_YAW = math.radians(55.0)  # the handle's flats, as it hangs (visible from above)
FORMER = parts.Part("former", pos=(BK_POS[0], BK_POS[1], FORMER_HANG_Z), yaw=FORMER_YAW, shapes=_former_shapes(),
                    mass=FORMER_MASS,
                    friction=(0.9, 0.8), visuals=_former_visuals(), contact_offset=0.002, report=("base",))
BEAKER = parts.Part("beaker", pos=(BK_POS[0], BK_POS[1], TOP + 0.0002), shapes=_beaker_shapes(), mass=BK_MASS,
                    friction=(0.5, 0.45), contact_offset=0.002)
BAR = parts.Part("bar", pos=(BAR_POS[0], BAR_POS[1], BAR_Z), yaw=math.pi / 2, shapes=_bar_shapes(), mass=BAR_MASS,
                 friction=(0.9, 0.8), contact_offset=0.002, report=("base",))
OBJECTS = [BEAKER, FORMER, BAR]

# Lot A: beaker on the left, bar on the right with its painted end toward +y. Lot B: the mirror image.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", mirror=1, place={"bar": (BAR_POS[0], BAR_POS[1], BAR_Z)}),
    lots.Lot("B", mirror=-1, place={"beaker": (BK_POS[0], -BK_POS[1], TOP + 0.0002),
                                    "former": (BK_POS[0], -BK_POS[1], FORMER_HANG_Z),
                                    "bar": (BAR_POS[0], -BAR_POS[1], BAR_Z)},
             yaw={"bar": -math.pi / 2, "former": -FORMER_YAW}),
)

# -- the paint rule ---------------------------------------------------------------------------------------------
CONTACT_EPS = 0.0001  # m numerical surface classification tolerance; never creates a contact
F_MIN = 0.4  # N transfer press, obtained by phone from a technician who works with this product (Nov 2026)
K_FIELD = 8.3  # N/mm^2: the pin field yields sqrt(F / K_FIELD) mm under F newtons (0.6 mm at 3 N; an estimate)
F_FOLD = 19.0  # N: harder than this folds the pins over (1.5 mm), and the skin between them touches the paint
SLIDE_MAX = 0.002  # m the dome may slide over the wet face within one press before it smears the paint
THETA_SKIRT = 92.0  # degrees from the apex: a press centred past this puts the skirt in the paint
HAND_CLEAR = 0.030
TOUCH_GAP = 0.0003  # m: pin tips this close to the table, the beaker or a hand have touched it
HOLD_STEPS = int(SUCCESS_HOLD_S * 20)
TIERS = ((N_PINS, 1.0), (int(math.ceil(0.9 * N_PINS)), 0.75), (int(math.ceil(0.5 * N_PINS)), 0.5),
         (int(math.ceil(0.25 * N_PINS)), 0.25))
_S = parts.__dict__.setdefault("_tactip_state", {})


def reset_state():
    shown = set(_S.get("shown", ()))
    _S.clear()
    _S.update(painted=np.zeros(N_PINS, bool), wet=False, ruled=set(), slide=0.0, prev=None, known=False,
              broken=False, clear=False, hold=0, views=None, shown=shown, maxforce=0.0, presses=0, log_t=-1.0,
              loaded=False, bk0=None, slide_v=np.zeros(3), path=0.0, twist=0.0, off=0)


if not _S:
    reset_state()
    _S["shown"] = set()


def compression(force):
    """mm the pin field yields under ``force`` newtons."""
    return math.sqrt(max(force, 0.0) / K_FIELD)


def former_frame(st):
    return np.asarray(st.pos, float), parts._quat_to_matrix(st.quat_xyzw)


def bar_face(st):
    """(centre of the painted end face, its outward normal) in the world."""
    c, r = np.asarray(st.pos, float), parts._quat_to_matrix(st.quat_xyzw)
    n = r[:, 0]
    return c + n * BAR_L / 2, n


def tips_world(fst):
    o, r = former_frame(fst)
    return o + (R_TIP * PIN_DIRS) @ r.T


SLIDE_DEADBAND = 0.0001  # m a step: contact jitter below this is not counted in the path
TWIST_MAX = 15.0  # degrees the dome may turn about the face's normal within one contact (summed by size)
TWIST_DEADBAND = 0.3  # degrees a step: contact jitter below this is not counted
F_LIGHT = 0.1  # N: motion over the wet paint counts toward slide and twist from this light a touch
OFF_STEPS = 3  # control steps without a qualifying loaded wet-face contact before a new contact starts


# Point samples below are used ONLY by the offline demo clearance planner, never paint-state updates.
def _former_rim_points():
    """Sample points on the skirt, shank, flange (rim and both faces) and square handle (its four faces), in F: none
    of these may touch the paint."""
    a = np.linspace(0.0, 2 * math.pi, 36, endpoint=False)
    out = []
    for r, z0, z1, k in ((R_SKIN, 0.0, SKIRT_TOP, 6), (SHANK[0], SHANK[1], SHANK[2], 2), (FLANGE[0], FLANGE[1], FLANGE[2], 2)):
        for z in np.linspace(z0, z1, k):
            out += [(r * math.cos(t), r * math.sin(t), z) for t in a]
    for r in np.linspace(SHANK[0], FLANGE[0], 5):  # the flange's two flat faces
        for z in (FLANGE[1], FLANGE[2]):
            out += [(r * math.cos(t), r * math.sin(t), z) for t in a]
    h = HANDLE[0]
    for u in np.linspace(-h, h, 5):  # the handle's four flat faces
        for z in np.linspace(HANDLE[1], HANDLE[2], 5):
            out += [(h, u, z), (-h, u, z), (u, h, z), (u, -h, z)]
    return np.array(out)


FORMER_RIM_PTS = _former_rim_points()


def _contact_reset():
    _S["prev"] = None
    _S["slide"] = 0.0
    _S["slide_v"] = np.zeros(3)
    _S["path"] = 0.0
    _S["twist"] = 0.0
    _S["off"] = 0
    _S["loaded"] = False


def wet_face_contacts(contacts, rst):
    """Select actual loaded, non-separated contacts on the wet end, with face-normal reaction.

    contacts: (normal load N, world point, world normal, separation m) tuples from PhysX.
    Point tolerances classify a reported contact; they never generate contact from proximity.
    """
    p, n = bar_face(rst)
    out = []
    for load, point, normal, separation in contacts:
        point, normal = np.asarray(point, float), np.asarray(normal, float)
        h = float((point - p) @ n)
        lateral = point - p - h * n
        alignment = abs(float(normal @ n))
        if (load > 0.0 and separation <= 0.0 and alignment >= 0.95
                and abs(h) <= PAINT_T + CONTACT_EPS
                and np.linalg.norm(lateral) <= WET_R + EPS):
            out.append((float(load) * alignment, point))
    return out


def dome_contact(point, fst):
    """Classify a reported contact by nearest actual collision surface, not a pose-only trigger.

    Contact points lie on the pair interface and may be on the bar side of penetration or a margin.
    A hemisphere's equatorial edge must not be confused with the 2 mm smaller skirt beside it.
    """
    o, r = former_frame(fst)
    x, y, z = r.T @ (np.asarray(point)-o)
    radial = math.hypot(x, y)
    dome = abs(math.sqrt(x*x+y*y+z*z)-R_TIP) if z <= 0 else math.hypot(radial-R_TIP, z)
    def cylinder_distance(radius, z0, z1):
        q = np.array([radial-radius, abs(z-(z0+z1)/2)-(z1-z0)/2])
        return abs(float(np.linalg.norm(np.maximum(q, 0)) + min(float(q.max()), 0)))
    others = [cylinder_distance(R_SKIN, 0, SKIRT_TOP),
              cylinder_distance(SHANK[0], SHANK[1], SHANK[2]),
              cylinder_distance(FLANGE[0], FLANGE[1], FLANGE[2])]
    q = np.abs(np.array([x,y,z-(HANDLE[1]+HANDLE[2])/2])) - np.array([HANDLE[0],HANDLE[0],(HANDLE[2]-HANDLE[1])/2])
    others.append(abs(float(np.linalg.norm(np.maximum(q,0)) + min(float(q.max()),0))))
    return dome < min(others)


def record_press(contacts, fst, rst):
    """Accumulate paint and smear only from actual loaded wet-face contacts.

    The 0.4 N transfer load is technician-reported; footprint, fold and smear thresholds are uncalibrated estimates.
    Both bodies' rotations enter relative slip/twist; common rigid motion contributes zero.
    """
    o, r = former_frame(fst)
    rc, rr = np.asarray(rst.pos, float), parts._quat_to_matrix(rst.quat_xyzw)
    p, n = bar_face(rst)
    wet_contacts = wet_face_contacts(contacts, rst)
    dome_force = 0.0
    for load, point in wet_contacts:
        local = r.T @ (point - o)
        if dome_contact(point, fst):
            dome_force += load
        elif (np.linalg.norm(local) <= 0.10 and load >= F_LIGHT - EPS):
            if "the former or the skirt touched the paint" not in _S["ruled"]:
                print("[tactip-contact-classification] " + json.dumps(dict(load=load, local_mm=(local*1000).tolist(), norm_mm=float(np.linalg.norm(local)*1000))), flush=True)
            _S["ruled"].add("the former or the skirt touched the paint")
    force = dome_force
    if force < F_LIGHT - EPS:
        no_press()
        return
    centre = o - R_TIP * n
    gap = float((o - p) @ n) - R_TIP
    lateral = centre - p - ((centre - p) @ n) * n
    # Explicit contact data, not centre proximity, determines whether a contact exists.
    _S["off"] = 0
    if force > F_FOLD + EPS:
        _S["ruled"].add("pins folded: the skin between the pins touched the paint")
    m_now = r.T @ (centre - o)
    relative = rr.T @ r
    prev = _S.get("prev")
    if prev is not None:
        m_prev, face_prev, relative_prev = prev
        d = rr.T @ (o + r @ m_prev - rc) - face_prev
        d[0] = 0.0
        _S["slide_v"] += d
        step = float(np.linalg.norm(d))
        if step > SLIDE_DEADBAND:
            _S["path"] += step
        _S["slide"] = max(float(np.linalg.norm(_S["slide_v"])), _S["path"])
        dr = relative @ relative_prev.T
        # Rotation-vector projection onto the wet-face normal (bar-frame +x).
        skew = np.array([dr[2, 1]-dr[1, 2], dr[0, 2]-dr[2, 0], dr[1, 0]-dr[0, 1]]) / 2
        sine = float(np.linalg.norm(skew))
        angle = math.atan2(sine, (float(np.trace(dr)) - 1) / 2)
        if sine > 1e-10:
            turn = math.degrees(angle * skew[0] / sine)
        elif angle > math.pi / 2:
            # A near-180-degree sampled jump has ambiguous axis; fail closed as a smear.
            turn = 180.0
        else:
            turn = 0.0
        if abs(turn) > TWIST_DEADBAND:
            _S["twist"] += abs(turn)
    _S["prev"] = (m_now, rr.T @ (centre - rc), relative)
    if _S["slide"] > SLIDE_MAX + EPS:
        _S["ruled"].add("the dome slid over the wet paint and smeared it")
    if _S["twist"] > TWIST_MAX + EPS:
        _S["ruled"].add("the dome turned on the wet paint and smeared it")
    if force < F_MIN - EPS:
        _S["loaded"] = False
        return
    if not _S["loaded"]:
        _S["presses"] += 1
    _S["loaded"] = True
    c_f = r.T @ (-n)
    theta = math.degrees(math.acos(np.clip(-c_f[2], -1, 1)))
    _S["last"] = dict(theta=round(theta, 1), lat_mm=round(float(np.linalg.norm(lateral))*1000, 1),
                      gap_mm=round(gap*1000, 2))
    if theta > THETA_SKIRT + EPS:
        _S["ruled"].add("the skirt touched the paint")
    tips = tips_world(fst)
    h = (tips - p) @ n
    lat = tips - p - np.outer(h, n)
    reach = compression(force) / 1000 + PAINT_T
    hit = (h <= reach + EPS) & (h >= -reach - EPS) & (np.linalg.norm(lat, axis=1) <= WET_R + EPS)
    if hit.any():
        _S["painted"] |= hit
        _S["wet"] = True


def no_press():
    """A step with no reported force: the contact ends once the dome has been off the face for OFF_STEPS steps."""
    _S["loaded"] = False
    _S["off"] = _S.get("off", 0) + 1
    if _S["off"] >= OFF_STEPS:
        _contact_reset()


def n_painted() -> int:
    return int(_S["painted"].sum())


# -- geometry of the end state -------------------------------------------------------------------------------------
def beaker_frame(st):
    return np.asarray(st.pos, float), parts._quat_to_matrix(st.quat_xyzw)


def tips_touch_beaker(fst, bst, only=None) -> bool:
    """Any pin tip (of those marked in ``only``, if given) within TOUCH_GAP of the beaker's glass (its wall, inside or
    out, its rim or its floor)."""
    c, r = beaker_frame(bst)
    tw = tips_world(fst) if only is None else tips_world(fst)[only]
    if len(tw) == 0:
        return False
    t = (tw - c) @ r  # beaker frame, z up from its base
    rad = np.hypot(t[:, 0], t[:, 1])
    z = t[:, 2]
    in_height = (z >= -TOUCH_GAP) & (z <= BK_H + TOUCH_GAP)
    wall = in_height & (rad >= BK_RIN - TOUCH_GAP) & (rad <= BK_R + TOUCH_GAP)
    rim = (np.hypot(np.maximum(0.0, np.abs(rad - (BK_R + BK_RIN) / 2) - BK_WALL / 2), z - BK_H) <= TOUCH_GAP)
    floor = (rad <= BK_RIN) & (z <= BK_BASE + TOUCH_GAP) & (z >= -TOUCH_GAP)
    return bool((wall | rim | floor).any())


def tips_touch_table(fst, only=None) -> bool:
    tw = tips_world(fst) if only is None else tips_world(fst)[only]
    return bool(len(tw) and tw[:, 2].min() <= TOP + TOUCH_GAP)


def former_hung(fst, bst) -> bool:
    """Hanging in the beaker: upright within 8 degrees, its flange on the rim (its underside within 2 mm of the rim's
    height), its axis within 15 mm of the beaker's, and no pin touching the glass."""
    o, r = former_frame(fst)
    c, rb = beaker_frame(bst)
    up = float(r[2, 2])
    flange_z = float((o + r @ np.array([0.0, 0.0, FLANGE[1]]))[2])
    rim_z = float(c[2] + BK_H)
    off = np.hypot(o[0] - c[0], o[1] - c[1])
    return bool(up >= math.cos(math.radians(8.0)) - EPS and abs(flange_z - rim_z) <= 0.002 + EPS
                and off <= 0.015 + EPS and not tips_touch_beaker(fst, bst))


def beaker_ok(bst) -> bool:
    """Standing on the table (base within 2 mm of it, upright within 5 degrees), moved no more than 15 mm."""
    c, r = beaker_frame(bst)
    lot = lots.current()
    x0, y0, _ = lot.pos("beaker")
    return bool(float(r[2, 2]) >= math.cos(math.radians(5.0)) - EPS and TOP - 0.002 - EPS <= c[2] <= TOP + 0.002 + EPS
                and math.hypot(c[0] - x0, c[1] - y0) <= 0.015 + EPS)


def bar_down(rst) -> bool:
    """Resting on the table with its wet circle touching nothing: the bar's lowest point within 2 mm of the table,
    and every point of the wet circle at least 1 mm above it."""
    c, r = np.asarray(rst.pos, float), parts._quat_to_matrix(rst.quat_xyzw)
    ax = r[:, 0]
    ang = np.linspace(0.0, 2 * math.pi, 48, endpoint=False)
    u, v = r[:, 1], r[:, 2]
    rim_pts = np.array([c + e * ax * BAR_L / 2 + y * BAR_R * u + z * BAR_R * v
                        for e in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    wet_pts = c + ax * BAR_L / 2 + WET_R * (np.outer(np.cos(ang), u) + np.outer(np.sin(ang), v))
    low = float(rim_pts[:, 2].min())
    return bool(TOP - 0.001 - EPS <= low <= TOP + 0.002 + EPS and float(wet_pts[:, 2].min()) >= TOP + 0.001 - EPS)


# -- scoring ---------------------------------------------------------------------------------------------------------
def ruled_out() -> bool:
    return bool(not _S["known"] or _S["broken"] or _S["ruled"])


def score(objects):
    """Pins painted, judged once the former hangs in the beaker, the bar lies on the table, both hands are 30 mm
    clear and everything has been still for 1 s. Zero for the whole trial after any breaking rule."""
    if ruled_out() or not _S["clear"] or _S["hold"] < HOLD_STEPS:
        return 0.0
    f, b, rd = objects["former"], objects["beaker"], objects["bar"]
    if not (f.resting() and b.resting() and rd.resting()):
        return 0.0
    if not (former_hung(f, b) and beaker_ok(b) and bar_down(rd)):
        return 0.0
    n = n_painted()
    for need, s in TIERS:
        if n >= need:
            return s
    return 0.0


def success(objects):
    return score(objects) == 1.0


# -- simulator side: contact views, hands, the paint's look ----------------------------------------------------------
HAND_LINKS = ("gripper", "tip_left", "tip_right")
_FACE_PTS: dict[str, np.ndarray] = {}


def _arr(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy())
    return np.asarray(x)


def _make_views(base):
    from isaaclab_physx.physics import PhysxManager
    from isaaclab.sim.utils import get_current_stage
    from pxr import UsdPhysics

    sv = PhysxManager.get_physics_sim_view()
    stage = get_current_stage()
    hands, tables = [], []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "/env_0/" not in path:
            continue
        if prim.GetName() in HAND_LINKS and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            hands.append(path)
        if prim.GetName().lower() == "table":
            tables.append(path)
    if len(hands) != 6 or len(tables) != 1:
        raise RuntimeError(f"contact targets not found: {len(hands)} hands, {len(tables)} tables")
    filters = [parts._PATHS["bar"], parts._PATHS["beaker"], tables[0], *hands]
    press = sv.create_rigid_contact_view(parts._PATHS["former"], filter_patterns=filters,
                                         max_contact_data_count=1024)
    handpaint = sv.create_rigid_contact_view(parts._PATHS["bar"], filter_patterns=hands,
                                             max_contact_data_count=512)
    print(f"[tactip] contact targets: {filters}", file=sys.stderr, flush=True)
    return {"press": press, "handpaint": handpaint}


def contact_rows(view, dt, origin):
    """Decode each sensor/filter pair; use only reported points, normal loads and separations."""
    loads, points, normals, separations, counts, starts = map(_arr, view.get_contact_data(dt=dt))
    loads, separations = loads.reshape(-1), separations.reshape(-1)
    points, normals = points.reshape(-1, 3), normals.reshape(-1, 3)
    counts, starts = counts.reshape(-1), starts.reshape(-1)
    if int(counts.sum()) >= len(loads):
        raise RuntimeError("contact buffer full; contamination cannot be excluded")
    pairs = []
    for count, start in zip(counts, starts):
        pairs.append([(float(loads[i]), points[i] - origin, normals[i], float(separations[i]))
                      for i in range(int(start), int(start + count))])
    return pairs


def painted_pin_contact(contacts, fst):
    """Actual loaded collision point intersects a painted tip's disk-sized neighbourhood.

    Continuous PhysX collider queries replace sampled hand surfaces. The rigid tip envelope remains a proxy.
    """
    tw = tips_world(fst)[_S["painted"]]
    return bool(len(tw) and any(load >= F_LIGHT - EPS and separation <= 0.0
                and np.linalg.norm(tw - point, axis=1).min() <= PIN_D / 2 + CONTACT_EPS
                for load, point, normal, separation in contacts))


def _hull_surface(h, spacing=0.004):
    from scipy.spatial import ConvexHull

    v = h.vertices
    out = [v]
    for tri in ConvexHull(v).simplices:
        a, b, c = v[tri]
        n = max(1, int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / spacing)))
        out.append(np.array([a + (b - a) * u / n + (c - a) * w / n for u in range(n + 1) for w in range(n + 1 - u)]))
    return np.unique(np.round(np.concatenate(out) / 0.003).astype(np.int64), axis=0) * 0.003


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


def hands_clear(points, objects) -> bool:
    from yam_arena import analysis

    for name in ("former", "bar", "beaker"):
        st = objects[name]
        spec = {"former": FORMER, "bar": BAR, "beaker": BEAKER}[name]
        poses = {"base": (np.asarray(st.pos, float), parts._quat_to_matrix(st.quat_xyzw))}
        for _, c, r, s in spec.world_boxes(poses):
            if float(analysis.box_signed_distance(points, c, r, s).min()) < HAND_CLEAR - EPS:
                return False
    return True


@parts.on_reset
def _on_reset(sim) -> None:
    views = _S.get("views")
    shown = set(_S.get("shown", ()))
    reset_state()
    _S["views"] = views
    for i in shown:
        try:
            sim.set_visible("former", f"cap{i}", False)
        except Exception as exc:  # noqa: BLE001
            print(f"[tactip] could not hide cap{i}: {exc}", file=sys.stderr)
    _S["shown"] = set()


@parts.on_step
def _on_step(sim) -> None:
    try:
        base = sim.env.unwrapped
        if _S.get("views") is None:
            _S["views"] = _make_views(base)
        v = _S["views"]
        dt = base.physics_dt
        st = sim.states()
        f, rd, bk = st["former"], st["bar"], st["beaker"]
        origin = parts._np(base.scene.env_origins[0])
        contacts = contact_rows(v["press"], dt, origin)
        force = sum(load for load, _ in wet_face_contacts(contacts[0], rd))
        _S["maxforce"] = max(_S["maxforce"], force)
        was = bool(_S["loaded"])
        before = n_painted()
        record_press(contacts[0], f, rd)
        if _S.get("press_log") is not None and _S.get("prev") is not None:
            _S["press_log"]["slide"] = max(_S["press_log"]["slide"], _S["slide"])
        if _S["loaded"]:
            pk = _S.setdefault("press_log", dict(peak=0.0, new=0, slide=0.0, t0=float(sim.t)))
            if force > pk["peak"]:
                pk.update(_S.get("last", {}))
            pk["peak"] = max(pk["peak"], force)
            pk["new"] += n_painted() - before
            pk["slide"] = max(pk["slide"], _S["slide"])
        elif was and _S.get("press_log") and _S.get("prev") is None:
            pk = _S.pop("press_log")
            print("[tactip-press] " + json.dumps(dict(t=round(pk["t0"], 2), peak=round(pk["peak"], 2), new=pk["new"],
                                                      slide_mm=round(pk["slide"] * 1000, 2), painted=n_painted(),
                                                      theta=pk.get("theta"), lat_mm=pk.get("lat_mm"),
                                                      gap_mm=pk.get("gap_mm"), off_face=_S.get("off_face", 0))),
                  flush=True)
        pts = hand_points(base)
        handcontacts = contact_rows(v["handpaint"], dt, origin)
        if any(sum(load for load, _ in wet_face_contacts(pair, rd)) >= F_LIGHT - EPS for pair in handcontacts):
            _S["ruled"].add("a hand touched the wet paint")
        if _S["wet"]:
            for pair, label in ((contacts[1], "beaker"), (contacts[2], "table"),
                                ([row for pair in contacts[3:] for row in pair], "hand")):
                if painted_pin_contact(pair, f):
                    _S["ruled"].add(f"painted pins touched the {label}")
        if not beaker_ok(bk) and float(parts._quat_to_matrix(bk.quat_xyzw)[2, 2]) < math.cos(math.radians(30.0)):
            _S["ruled"].add("the beaker was knocked over")
        still = f.resting() and rd.resting() and bk.resting()
        _S["clear"] = bool(still and hands_clear(pts, st))  # judged only once everything is still
        _S["known"] = True
        _S["hold"] = _S["hold"] + 1 if (_S["clear"] and still) else 0
        for i in np.flatnonzero(_S["painted"]):
            if int(i) not in _S["shown"]:
                sim.set_visible("former", f"cap{int(i)}", True)
                _S["shown"].add(int(i))
        if sim.t - _S["log_t"] >= 0.5 - 1e-6:
            _S["log_t"] = sim.t
            o, r = former_frame(f)
            print("[tactip] " + json.dumps(dict(
                t=round(float(sim.t), 2), force=round(force, 2), maxforce=round(_S["maxforce"], 2),
                painted=n_painted(), missing=(np.flatnonzero(~_S["painted"]).tolist() if n_painted() >= N_PINS-10 else None), presses=_S["presses"], slide=round(_S["slide"] * 1000, 2),
                ruled=sorted(_S["ruled"]), clear=_S["clear"], hold=_S["hold"],
                former=[round(float(x), 4) for x in o], axis=[round(float(x), 3) for x in r[:, 2]],
                bar=[round(float(x), 4) for x in rd.pos], bar_q=[round(float(x), 4) for x in rd.quat_xyzw],
                beaker=[round(float(x), 4) for x in bk.pos])), flush=True)
    except Exception as exc:  # unreadable contacts, hands or poses: no credit for this trial
        _S["broken"] = True
        print(f"[tactip] step check failed: {exc!r}", file=sys.stderr, flush=True)


# -- the scripted demo: open loop from the visible layout (where the beaker and the bar stand) and the skin's
# published size. One hand holds the former by its handle, the other the bar: the bar's painted end is tipped up to
# meet the dome at angles the wrist alone cannot reach, and the dome is dabbed onto it one patch of pins at a time,
# lifted straight off, never slid. Halfway through, the former is hung back in the beaker and gripped again with
# the hand turned half a turn, because the wrist cannot turn the dome all the way round. ----------------------------
GRIP_HANDLE = -0.004  # finger target: 8 mm between the pads on the 20 mm handle (a firm squeeze)
GRIP_BAR = -0.013  # 26 mm on the 38.1 mm bar
ALPHA_PLAN = float(os.environ.get("TACTIP_ALPHA", "13.0"))  # degrees: the patch each dab is planned to paint
OVERLAP = float(os.environ.get("TACTIP_OVERLAP", "0.022"))  # m past first touch, used to check reach (the arms give way)
PRESS = {0.0: 0.026, 22.0: 0.026, 40.0: 0.020}  # m past first touch at each tip of the bar: about 2.5 N (from traces)
STANDOFF = 0.010  # m off the face before and after each dab
BETAS = (0.0, 22.0, 40.0)  # degrees the bar's painted end is tipped up toward the dome
FACE_Z = {b: max(0.072, BAR_L * math.sin(math.radians(b)) + BAR_R * math.cos(math.radians(b)) + 0.010)
          for b in BETAS}  # the end face's centre above the table: the bar's dry end stays 10 mm clear of it
MEET = (0.315, 0.0)  # the end face's centre, x and y (lot A)
BAR_GRIP = 0.0  # the bar is held at its middle (its weight then turns nothing in the grip)
BAR_SAG = {0.0: (0.005, 0.0, 0.006), 22.0: (0.005, 0.0, 0.006), 40.0: (0.005, 0.0, 0.006)}  # m, from traces
B_BAR_X_CORRECTION = -0.008  # m demo compensation, measured B-vs-A loaded x bias in round-2 traces
GRASPS = (math.radians(55.0), math.radians(-125.0))  # the left hand's grip yaw on the handle, first and second
YAW_RANGE = (-140.0, 72.0)  # degrees of grip yaw the left arm turns through comfortably (right arm: mirrored)
TILT_MAX = 46.0
CENTRE_MAX = 78.0  # degrees from the apex: no dab is aimed past this, so a few degrees of droop cannot reach the skirt
DAB_SPOTS = [(0.0, 0.0)]  # every dab aimed at the face's centre
UP = np.array([0.0, 0.0, 1.0])
PARK = (MEET[0] - 0.02, MEET[1] + 0.09, TOP + 0.155)


def _axis_angle(axis, ang):
    from yam_arena.urdf import axis_angle_matrix

    axis = np.asarray(axis, float)
    nrm = np.linalg.norm(axis)
    if nrm < 1e-12 or abs(ang) < 1e-12:
        return np.eye(3)
    return axis_angle_matrix(axis / nrm, ang)


def _rot_between(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    c = float(np.clip(a @ b, -1.0, 1.0))
    ax = np.cross(a, b)
    if np.linalg.norm(ax) < 1e-9:
        return np.eye(3) if c > 0 else _axis_angle(_orth(a)[0], math.pi)
    return _axis_angle(ax, math.acos(c))


def decompose(r):
    """(yaw rad, tilt deg, heading rad) with tilt.rotation(yaw, tilt, heading) == r."""
    a = r @ np.array([0.0, 0.0, 1.0])  # the approach axis
    t = math.degrees(math.acos(float(np.clip(-a[2], -1.0, 1.0))))
    h = math.atan2(a[1], a[0]) if t > 1e-6 else 0.0
    lean = tilt.rotation(0.0, t, h) @ tilt._ORIGINAL(0.0).T
    rz = lean.T @ r @ tilt._ORIGINAL(0.0).T
    return math.atan2(rz[1, 0], rz[0, 0]), t, h


def _wp(dur, pos, finger, label, yaw=0.0, t=0.0, h=0.0, m=1):
    """A waypoint given in lot A's frame, mirrored into lot B's (y, yaw and heading negated) when m is -1."""
    x, y, z = (float(v) for v in pos)
    return tilt.Waypoint(float(dur), (x, m * y, z), float(finger), label, yaw=m * float(yaw), tilt=float(t),
                         heading=m * float(h))


def _fib(n, tmax):
    k = np.arange(n) + 0.5
    z = 1 - k / n * (1 - math.cos(math.radians(tmax)))
    rr = np.sqrt(1 - z * z)
    a = math.pi * (1 + 5 ** 0.5) * k
    return np.c_[rr * np.cos(a), rr * np.sin(a), -z]


def _cover(alpha_deg):
    """Dab centres (unit directions in F): every direction out to the last pin ring lies within alpha of one, so the
    plan does not depend on how the skin happens to sit on the former."""
    need = _fib(6000, PIN_THETA_MAX + 0.6)
    cand = _fib(1500, CENTRE_MAX)
    near = (cand @ need.T) >= math.cos(math.radians(alpha_deg))
    left = np.ones(len(need), bool)
    chosen = []
    while left.any():
        gain = (near & left).sum(axis=1)
        i = int(np.argmax(gain))
        if gain[i] == 0:
            break
        chosen.append(cand[i])
        left &= ~near[i]
    return np.array(chosen)


def bar_pose(beta_deg):
    """Lot A: (the bar hand's TCP, the end face's normal, the face's centre) with the bar's painted end centred on
    MEET, tipped up beta degrees."""
    b = math.radians(beta_deg)
    n = np.array([0.0, math.cos(b), math.sin(b)])
    face = np.array([MEET[0], MEET[1], TOP + FACE_Z[beta_deg]])
    return face - n * (BAR_L / 2 + BAR_GRIP), n, face


def bar_hand_target(beta_deg):
    """The bar hand's commanded TCP: bar_pose's, raised by the bar hand's measured sag under the bar and the dabs."""
    tcp = bar_pose(beta_deg)[0]
    correction = np.array([B_BAR_X_CORRECTION, 0.0, 0.0]) if lots.current().name == "B" else np.zeros(3)
    return tcp + np.asarray(BAR_SAG.get(beta_deg, (0.0, 0.0, 0.0))) + correction


_SEEDS = ([0.0, 1.0, 1.0, -0.6, 0.0, 0.0], [0.3, 0.8, 0.8, -0.8, 0.5, 1.0], [-0.3, 1.2, 1.4, -0.4, -0.5, -1.0])


def reachable(pos, r, arm="left"):
    """Joint angles reaching the TCP pose (lot A frame) within 0.8 mm and 0.4 degrees (offline IK, a few seeds), or
    None."""
    from yam_arena import ik, station

    for seed in _SEEDS:
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(r), seed, base_pos=station.BASES[arm], iters=80)
        if pe < 0.0008 and math.degrees(re) < 0.4:
            return q
    return None


_B_Q: dict[float, np.ndarray] = {}
_B_HULLS: dict[float, list] = {}


def _bar_hand_q(beta):
    if beta not in _B_Q:
        _B_Q[beta] = reachable(bar_pose(beta)[0], tilt.rotation(math.pi / 2, beta, math.pi / 2), "right")
    return _B_Q[beta]


def _bar_points(beta):
    tcp, n, face = bar_pose(beta)
    u = np.array([1.0, 0.0, 0.0])
    w = np.cross(n, u)
    g = np.linspace(-1, 1, 5)
    mid = tcp + BAR_GRIP * n
    return np.array([mid + a * n * BAR_L / 2 + b * u * BAR_R + c * w * BAR_R for a in g for b in g for c in g])


def clear_of_each_other(q_a, beta, o, rf):
    """Offline check that the two arms (FK hulls), the former and the bar keep apart at a dab: arms 12 mm, the former
    and the bar 5 mm from the other hand."""
    from yam_arena import analysis, station

    q_b = _bar_hand_q(beta)
    if q_b is None:
        return False
    links = ("link3", "link4", "link5", "gripper", "tip_left", "tip_right")
    ha = analysis.world_hulls(analysis.fk_link_poses_world({f"joint{i + 1}": float(v) for i, v in enumerate(q_a)},
                                                           station.BASES["left"]), list(links))
    if beta not in _B_HULLS:
        _B_HULLS[beta] = analysis.world_hulls(analysis.fk_link_poses_world(
            {f"joint{i + 1}": float(v) for i, v in enumerate(q_b)}, station.BASES["right"]), list(links))
    hb = _B_HULLS[beta]
    if analysis.hull_set_distance(ha, hb)[0] < 0.012:
        return False
    fp = o + np.concatenate([FORMER_RIM_PTS, R_TIP * PIN_DIRS[::4]]) @ rf.T
    bp = _bar_points(beta)
    for pts, hulls in ((fp, hb), (bp, ha)):
        lo, hi = pts.min(axis=0) - 0.01, pts.max(axis=0) + 0.01
        for _, h in hulls:
            if (h.vertices.min(axis=0) > hi).any() or (h.vertices.max(axis=0) < lo).any():
                continue
            if float((pts @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1).min()) < 0.005:
                return False
    return True


def _dab_options(d, n, grasp):
    """The former hand's (cost, yaw, tilt, heading, R_F) choices that put direction d of the dome (in F) against a
    face of normal n with the grip turned ``grasp`` on the handle: one per spin about n inside the wrist's
    comfortable range (lean at most TILT_MAX, away from the arm's base, yaw inside YAW_RANGE)."""
    r1 = _rot_between(d, -n)
    ra0 = tilt._ORIGINAL(grasp)
    rf0 = parts._rot((0.0, 0.0, 1.0), FORMER_YAW)
    options = []
    for s in np.radians(np.arange(0.0, 360.0, 5.0)):
        rf = _axis_angle(n, s) @ r1
        yaw, t, h = decompose(rf @ rf0.T @ ra0)
        if t > TILT_MAX:
            continue
        if t > 3.0 and not -110.0 <= math.degrees((h + math.pi) % (2 * math.pi) - math.pi) <= 20.0:
            continue
        yd = math.degrees(yaw)
        if not YAW_RANGE[0] <= yd <= YAW_RANGE[1]:
            continue
        options.append((t + 0.05 * abs(yd), yaw, t, h, rf, float(np.degrees(s))))
    return options


def _verify(o, n, touch, beta):
    """The option reaches its press and its stand-off and keeps clear of the other hand and the bar."""
    rf = o[4]
    r = tilt.rotation(o[1], o[2], o[3])
    qs = [reachable(touch + along * n + rf @ np.array([0.0, 0.0, GRIP_Z]), r) for along in (-OVERLAP, STANDOFF + 0.010)]
    return all(q is not None for q in qs) and clear_of_each_other(qs[0], beta, touch - OVERLAP * n, rf)


_PLANS: dict[float, tuple] = {}


def plan_dabs(alpha=None):
    """Every dab in order: dicts with the grasp (0 or 1), the bar's tip beta, the grip (yaw, tilt, heading), the
    dome's centre at first touch, R_F and the face normal. Also the directions no option reaches."""
    alpha = ALPHA_PLAN if alpha is None else alpha
    if alpha in _PLANS:
        return _PLANS[alpha]
    groups: dict[tuple[int, float], list] = {}
    missed = []
    for d in _cover(alpha):
        opts = []
        for g in (0, 1):
            for b in BETAS:
                _, n, face = bar_pose(b)
                opts += [(o[0] + 3.0 * g, g, b, o) for o in _dab_options(d, n, GRASPS[g])]
        opts.sort(key=lambda x: x[0])
        best = None
        tried = set()
        for c, g, b, o in opts:
            key = (g, b, round(o[1], 2))
            if key in tried:
                continue
            tried.add(key)
            _, n, face = bar_pose(b)
            if _verify(o, n, face + R_TIP * n, b):
                best = (g, b, o)
                break
            if len(tried) >= 30:
                break
        if best is None:
            missed.append(d)
            continue
        g, b, (_, yaw, t, h, rf, spin) = best
        groups.setdefault((g, b), []).append(dict(grasp=g, beta=b, yaw=yaw, tilt=t, heading=h, rf=rf, d=d, spin=spin))
    order = [(0, b) for b in BETAS] + [(1, b) for b in reversed(BETAS)]
    dabs = []
    for key in order:
        grp = sorted(groups.get(key, []), key=lambda e: e["yaw"] * (1 if key[0] == 0 else -1))
        for k, e in enumerate(grp):
            _, n, face = bar_pose(e["beta"])
            u, v = _orth(n)
            sx, sy = DAB_SPOTS[k % len(DAB_SPOTS)]
            e["n"] = n
            e["touch"] = face + sx * u + sy * v + R_TIP * n  # the dome's centre when its pins first touch
            dabs.append(e)
    _PLANS[alpha] = (dabs, missed)
    return dabs, missed


def _dab_from_row(row):
    """One dab from a DAB_TABLE row (grasp, beta, spin about the face's normal in degrees, dome direction)."""
    g, b, spin, dx, dy, dz = row
    d = np.array([dx, dy, dz])
    d /= np.linalg.norm(d)
    _, n, face = bar_pose(b)
    rf = _axis_angle(n, math.radians(spin)) @ _rot_between(d, -n)
    yaw, t, h = decompose(rf @ parts._rot((0.0, 0.0, 1.0), FORMER_YAW).T @ tilt._ORIGINAL(GRASPS[g]))
    return dict(grasp=g, beta=b, yaw=yaw, tilt=t, heading=h, rf=rf, d=d, spin=spin, n=n, touch=face + R_TIP * n)


def dab_list():
    """The demo's dabs: DAB_TABLE, which plan_dabs() wrote offline from the same constants (it takes a minute and a
    half to verify every pose's reach and clearance, too long to repeat at every reset)."""
    if os.environ.get("TACTIP_REPLAN") or not DAB_TABLE:
        return plan_dabs()[0]
    dabs = [_dab_from_row(r) for r in DAB_TABLE]
    if lots.current().name == "B":
        # Waypoint mirroring reflects local y, so these rows target the reflected atlas IDs.
        extra = _dab_from_row(B_FIRST_GRASP_DAB)
        at = next(i for i, e in enumerate(dabs)
                  if (e["grasp"] == 0 and e["beta"] == 40.0 and e["yaw"] > extra["yaw"]) or e["grasp"] == 1)
        dabs.insert(at, extra)
        dabs.extend(_dab_from_row(r) for r in B_COMPLETION_DABS)
    return dabs


def table_rows(dabs):
    return tuple((e["grasp"], e["beta"], round(e["spin"], 3), *(round(float(v), 6) for v in e["d"])) for e in dabs)


DAB_TABLE: tuple = (  # (grasp, beta deg, spin deg, dome direction x, y, z), from plan_dabs() with ALPHA_PLAN 13
    (0, 0.0, 315.000, 0.928087, 0.291724, -0.231410),
    (0, 0.0, 315.000, 0.967427, 0.085496, -0.238275),
    (0, 0.0, 320.000, 0.961524, -0.124005, -0.245140),
    (0, 0.0, 325.000, 0.899481, -0.375796, -0.222961),
    (0, 0.0, 325.000, 0.796504, -0.559250, -0.229826),
    (0, 0.0, 330.000, 0.687065, -0.676260, -0.265734),
    (0, 0.0, 330.000, 0.556894, -0.799358, -0.225602),
    (0, 0.0, 330.000, 0.371650, -0.898797, -0.232466),
    (0, 0.0, 330.000, 0.169701, -0.955993, -0.239331),
    (0, 0.0, 335.000, 0.009964, -0.961324, -0.275239),
    (0, 0.0, 335.000, -0.039457, -0.968417, -0.246196),
    (0, 0.0, 335.000, -0.168718, -0.957214, -0.235107),
    (0, 0.0, 335.000, -0.369673, -0.897102, -0.241971),
    (0, 0.0, 335.000, -0.552684, -0.795375, -0.248836),
    (0, 0.0, 335.000, -0.696160, -0.686880, -0.208704),
    (0, 0.0, 340.000, -0.792871, -0.558140, -0.244612),
    (0, 0.0, 340.000, -0.917209, -0.330531, -0.222433),
    (0, 22.0, 295.000, 0.815275, 0.280634, -0.506529),
    (0, 22.0, 325.000, 0.842219, -0.295283, -0.451083),
    (0, 22.0, 350.000, 0.350583, -0.787051, -0.507585),
    (0, 22.0, 0.000, -0.011738, -0.853284, -0.521315),
    (0, 22.0, 10.000, -0.399250, -0.793672, -0.459004),
    (0, 22.0, 30.000, -0.752843, -0.417746, -0.508641),
    (0, 40.0, 270.000, 0.530128, 0.179754, -0.828645),
    (0, 40.0, 275.000, 0.565140, 0.110543, -0.817556),
    (0, 40.0, 285.000, 0.800080, -0.065576, -0.596299),
    (0, 40.0, 295.000, 0.604441, -0.045009, -0.795377),
    (0, 40.0, 305.000, 0.571946, -0.171131, -0.802242),
    (0, 40.0, 310.000, 0.367278, -0.171356, -0.914190),
    (0, 40.0, 315.000, 0.638306, -0.483567, -0.598939),
    (0, 40.0, 320.000, 0.546084, -0.470782, -0.692934),
    (0, 40.0, 325.000, 0.473073, -0.591652, -0.652801),
    (0, 40.0, 330.000, 0.240565, -0.284769, -0.927920),
    (0, 40.0, 350.000, 0.195423, -0.605327, -0.771615),
    (0, 40.0, 0.000, -0.048274, -0.291411, -0.955379),
    (0, 40.0, 0.000, -0.059918, -0.671754, -0.738347),
    (0, 40.0, 15.000, -0.150346, -0.326394, -0.933201),
    (0, 40.0, 20.000, -0.466747, -0.583090, -0.664947),
    (0, 40.0, 15.000, -0.557389, -0.566603, -0.606860),
    (0, 40.0, 35.000, -0.374951, -0.363197, -0.852936),
    (0, 40.0, 60.000, -0.222022, 0.035668, -0.974389),
    (0, 40.0, 55.000, -0.534437, -0.206207, -0.819668),
    (1, 40.0, 235.000, 0.189706, 0.124925, -0.973861),
    (1, 40.0, 245.000, 0.469513, 0.402445, -0.785872),
    (1, 40.0, 220.000, 0.177743, 0.182553, -0.966996),
    (1, 40.0, 225.000, 0.330043, 0.487981, -0.808051),
    (1, 40.0, 210.000, 0.221002, 0.556111, -0.801186),
    (1, 40.0, 175.000, 0.050795, 0.755368, -0.653329),
    (1, 40.0, 170.000, -0.083328, 0.527365, -0.845543),
    (1, 40.0, 165.000, -0.110032, 0.343533, -0.932672),
    (1, 40.0, 155.000, -0.113455, 0.754461, -0.646465),
    (1, 40.0, 140.000, -0.245049, 0.753108, -0.610557),
    (1, 40.0, 115.000, -0.466828, 0.420552, -0.777951),
    (1, 40.0, 100.000, -0.511606, 0.225236, -0.829173),
    (1, 40.0, 75.000, -0.617381, 0.020379, -0.786400),
    (1, 40.0, 65.000, -0.748934, 0.116779, -0.652273),
    (1, 40.0, 45.000, -0.797705, -0.074639, -0.598411),
    (1, 22.0, 260.000, 0.582465, 0.580651, -0.568840),
    (1, 22.0, 220.000, 0.273038, 0.816867, -0.508113),
    (1, 22.0, 115.000, -0.421439, 0.786408, -0.451611),
    (1, 22.0, 105.000, -0.527973, 0.631558, -0.567784),
    (1, 22.0, 70.000, -0.790439, 0.343656, -0.507057),
    (1, 0.0, 310.000, 0.845196, 0.484999, -0.224546),
    (1, 0.0, 300.000, 0.733959, 0.625500, -0.264678),
    (1, 0.0, 295.000, 0.641917, 0.715468, -0.275767),
    (1, 0.0, 295.000, 0.626904, 0.744752, -0.228770),
    (1, 0.0, 280.000, 0.474276, 0.838304, -0.268902),
    (1, 0.0, 265.000, 0.333042, 0.913672, -0.232994),
    (1, 0.0, 210.000, 0.129306, 0.965477, -0.226130),
    (1, 0.0, 135.000, -0.049806, 0.962613, -0.266262),
    (1, 0.0, 65.000, -0.209523, 0.950283, -0.230354),
    (1, 0.0, 25.000, -0.409352, 0.884581, -0.223489),
    (1, 0.0, 10.000, -0.625148, 0.740836, -0.245668),
    (1, 0.0, 0.000, -0.770999, 0.590367, -0.238803),
    (1, 0.0, 355.000, -0.881286, 0.411752, -0.231938),
    (1, 0.0, 350.000, -0.950714, 0.213272, -0.225074),
    (1, 0.0, 350.000, -0.963572, 0.034553, -0.265206),
    (1, 0.0, 350.000, -0.956471, -0.093941, -0.276295),
    (1, 0.0, 345.000, -0.965209, -0.125673, -0.229298),
    # Completion dabs follow the original 79-dab path so they preserve earlier coverage.
    (1, 40.0, 260.000, 0.836797715, 0.056017016, -0.544639035),  # atlas pin 217
    (1, 40.0, 230.000, 0.253404407, 0.253404407, -0.933580426),  # atlas pin 26
)

# Additional visible-layout reference motions for residual lot-B gaps observed in contact traces.
B_FIRST_GRASP_DAB = (0, 40.0, 340.0, 0.270598050, -0.653281482, -0.707106781)  # B atlas141
B_COMPLETION_DABS = (
    (1, 40.0, 230.0, 0.439727553, -0.112902846, -0.891006524),  # B atlas45
    (1, 40.0, 230.0, 0.098224626, 0.035750840, -0.994521895),   # apex/9-degree ring
    (1, 40.0, 210.0, 0.270598050, 0.653281482, -0.707106781),  # B atlas166
)



def demo():
    lot = lots.current()
    m = int(getattr(lot, "mirror", 1))
    a_arm, b_arm = ("left", "right") if m == 1 else ("right", "left")
    bx, by, _ = lot.pos("beaker")
    rx, ry, rz = lot.pos("bar")
    by, ry = m * by, m * ry  # into lot A's frame
    dabs = dab_list()
    A, B = [], []

    def both(dur, wa, wb):
        A.append(wa(dur))
        B.append(wb(dur))

    hang = np.array([bx, by, FORMER_HANG_Z])
    grip_rest = hang + GRIP_Z * UP
    lift = grip_rest + 0.078 * UP
    bar_c = np.array([rx, ry - BAR_GRIP, rz])  # the grip: the bar's middle
    yb = math.pi / 2  # the bar hand's fingers close along x, across the bar

    def a_at(pos, finger, label, yaw, t=0.0, h=0.0):
        return lambda dur: _wp(dur, pos, finger, label, yaw, t, h, m)

    def b_at(pos, finger, label, beta=0.0):
        return lambda dur: _wp(dur, pos, finger, label, yb, beta, math.pi / 2, m)

    def b_hold(beta):
        return b_at(bar_hand_target(beta), GRIP_BAR, "hold the bar's end to the skin", beta)

    def a_pose(e, along):
        o = e["touch"] + along * e["n"]
        return o + e["rf"] @ np.array([0.0, 0.0, GRIP_Z])

    def turn(pos, finger, y0, y1, label, wb):
        """Turn the former hand's grip from yaw y0 to y1 at pos, untilted, in steps of at most 80 degrees."""
        k = max(1, int(math.ceil(abs(math.degrees(y1 - y0)) / 80.0)))
        for i in range(1, k + 1):
            both(0.5 + 0.006 * abs(math.degrees(y1 - y0)) / k, a_at(pos, finger, label, y0 + (y1 - y0) * i / k), wb)

    g0 = GRASPS[0]
    # pick up: the former out of the beaker, the bar off the table
    both(2.2, a_at(grip_rest + 0.05 * UP, FINGER_OPEN, "over the former's handle", g0),
         b_at(bar_c + 0.06 * UP, FINGER_OPEN, "over the bar's dry end"))
    both(1.4, a_at(grip_rest, FINGER_OPEN, "down to the handle", g0), b_at(bar_c, FINGER_OPEN, "down to the bar"))
    both(0.8, a_at(grip_rest, GRIP_HANDLE, "grip the handle", g0), b_at(bar_c, GRIP_BAR, "grip the bar"))
    both(1.6, a_at(lift, GRIP_HANDLE, "lift the skin out of the beaker", g0),
         b_at(bar_c + 0.05 * UP, GRIP_BAR, "lift the bar"))
    cur = (0, None)
    yaw_now = g0
    for e in dabs:
        key = (e["grasp"], e["beta"])
        if key != cur:
            wb = b_hold(cur[1]) if cur[1] is not None else b_at(bar_c + 0.05 * UP, GRIP_BAR, "hold the bar up")
            if e["grasp"] != cur[0]:  # hang the former back and grip it again, turned half a turn
                g1 = GRASPS[1]
                both(1.4, a_at(PARK, GRIP_HANDLE, "lift the skin clear", yaw_now), wb)
                turn(PARK, GRIP_HANDLE, yaw_now, g0, "turn back", wb)
                both(1.8, a_at(lift, GRIP_HANDLE, "back over the beaker", g0), wb)
                both(1.6, a_at(grip_rest + 0.001 * UP, GRIP_HANDLE, "hang the skin in the beaker", g0), wb)
                both(0.6, a_at(grip_rest + 0.001 * UP, FINGER_OPEN, "let go", g0), wb)
                both(0.6, a_at(grip_rest + 0.03 * UP, FINGER_OPEN, "up", g0), wb)
                turn(grip_rest + 0.03 * UP, FINGER_OPEN, g0, g1, "turn the hand half a turn", wb)
                both(0.8, a_at(grip_rest, FINGER_OPEN, "down to the handle", g1), wb)
                both(0.8, a_at(grip_rest, GRIP_HANDLE, "grip the handle again", g1), wb)
                both(1.6, a_at(lift, GRIP_HANDLE, "lift it out", g1), wb)
                yaw_now = g1
            both(1.4, a_at(PARK, GRIP_HANDLE, "hold the skin clear", yaw_now),
                 b_at(bar_hand_target(e["beta"]), GRIP_BAR, f"tip the bar's end up {e['beta']:.0f} degrees", e["beta"]))
            turn(PARK, GRIP_HANDLE, yaw_now, e["yaw"], "turn to the next pins", b_hold(e["beta"]))
            cur = key
            yaw_now = e["yaw"]
        hb = b_hold(e["beta"])
        yaw, t, h = e["yaw"], e["tilt"], e["heading"]
        far, off, press = a_pose(e, STANDOFF + 0.010), a_pose(e, STANDOFF), a_pose(e, -PRESS[e["beta"]])
        dyaw = abs(math.degrees(yaw - yaw_now))
        both(0.5 + 0.006 * dyaw, a_at(far, GRIP_HANDLE, "turn fresh pins to the paint", yaw, t, h), hb)
        both(0.3, a_at(off, GRIP_HANDLE, "line up", yaw, t, h), hb)
        both(0.3, a_at(press, GRIP_HANDLE, "dab", yaw, t, h), hb)
        both(0.2, a_at(press, GRIP_HANDLE, "hold the dab", yaw, t, h), hb)
        both(0.3, a_at(far, GRIP_HANDLE, "lift straight off", yaw, t, h), hb)
        yaw_now = yaw
    # put both back: the bar on the table where it lay, the former into the beaker
    gl = GRASPS[cur[0]]
    both(1.4, a_at(PARK, GRIP_HANDLE, "lift the skin clear", yaw_now), b_hold(cur[1]))
    turn(PARK, GRIP_HANDLE, yaw_now, gl, "turn back", b_hold(cur[1]))
    both(2.2, a_at(lift, GRIP_HANDLE, "over the beaker", gl), b_at(bar_c + 0.05 * UP, GRIP_BAR, "carry the bar back"))
    both(1.6, a_at(grip_rest + 0.001 * UP, GRIP_HANDLE, "lower the skin into the beaker", gl),
         b_at(bar_c + 0.0015 * UP, GRIP_BAR, "lay the bar down"))
    both(0.7, a_at(grip_rest + 0.001 * UP, FINGER_OPEN, "let go", gl), b_at(bar_c + 0.0015 * UP, FINGER_OPEN, "let go"))
    both(1.0, a_at(grip_rest + 0.06 * UP, FINGER_OPEN, "up", gl), b_at(bar_c + 0.07 * UP, FINGER_OPEN, "up"))
    both(1.5, a_at((0.15, 0.26, TOP + 0.20), FINGER_OPEN, "clear", 0.0),
         b_at((0.15, -0.26, TOP + 0.20), FINGER_OPEN, "clear"))
    both(2.0, a_at((0.15, 0.26, TOP + 0.20), FINGER_OPEN, "wait", 0.0),
         b_at((0.15, -0.26, TOP + 0.20), FINGER_OPEN, "wait"))
    return {a_arm: A, b_arm: B}
