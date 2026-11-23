"""Printable replica of the DHEF-20 gripper's front end (bayonet catch, piston rod coupling, releasing ring) for the
real trial, built to the dimensions that ../task.py simulates, plus a fallback mould for the cap.

    python make_replica.py                 # nominal sizes (task.py's estimates for the cap's knob)
    python make_replica.py --knob-head-d 12.6 --knob-neck-d 5.8 --knob-head-len 9.5   # after measuring a real cap

Needs Python 3.9+ with numpy, trimesh and manifold3d (matplotlib for the drawings). It reads task.py's constants
without importing the simulator, writes stl/*.stl, drawing.pdf, assembly.png, CHECKS.txt and SHA256SUMS, and exits
non-zero if a check fails. Every length here is in mm.

STL frame (every part in its assembled place): z along the gripper's axis, measured from the gripper's mounting face
toward the cap's tip (task.py's u); y up; x level, across the axis (world +x in the task's setup). An angle phi in the
x-y plane, from +x toward +y, equals 180 deg minus task.py's angle th. Seen from the cap's tip, +phi is
counterclockwise, the direction that turns the catch from locked to open.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import math
import pathlib
import sys

import numpy as np
import trimesh
from manifold3d import CrossSection, Manifold

HERE = pathlib.Path(__file__).resolve().parent
TASK = HERE.parent / "task.py"
OUT = HERE / "stl"
EPS = 0.01

# ---------------------------------------------------------------------------------------------------------------------
# 1. task.py's dimensions, read from its source (module-level assignments and the Slider keyword arguments)


def task_constants(path=TASK) -> dict:
    tree = ast.parse(path.read_text())
    env: dict = {"math": math}
    out: dict = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        tgt = node.targets[0]
        if isinstance(node.value, ast.Call) and getattr(node.value.func, "attr", "") == "Slider" and isinstance(tgt, ast.Name):
            kw = {}
            for k in node.value.keywords:
                try:
                    kw[k.arg] = eval(compile(ast.Expression(k.value), "task.py", "eval"), env)  # noqa: S307
                except Exception:  # noqa: BLE001  (shapes etc.)
                    pass
            out[tgt.id] = kw
            continue
        try:
            val = eval(compile(ast.Expression(node.value), "task.py", "eval"), env)  # noqa: S307
        except Exception:  # noqa: BLE001  (anything that needs the simulator)
            continue
        if isinstance(tgt, ast.Name):
            names, vals = [tgt.id], [val]
        elif isinstance(tgt, ast.Tuple) and isinstance(val, tuple) and len(val) == len(tgt.elts):
            names, vals = [e.id for e in tgt.elts], list(val)
        else:
            continue
        for n, v in zip(names, vals):
            env[n] = v
            out[n] = v
    return out


T = task_constants()
mm = lambda name: 1000.0 * T[name]  # noqa: E731  task.py metres -> mm
deg = math.degrees

# the values the replica must share with the simulation (mm, deg, N/m), each from task.py
U_BODY_FRONT, U_NOSE = mm("U_BODY_FRONT"), mm("U_NOSE")
R_RING_IN, R_RING_OUT = mm("R_RING_IN"), mm("R_RING_OUT")
U_RING0, U_RING1 = mm("U_RING0"), mm("U_RING1")
R_NOSE, R_LIP = mm("R_NOSE"), mm("R_LIP")
U_LIP0, U_LIP1 = mm("U_LIP0"), mm("U_LIP1")
R_LUG0, U_LUG0, U_LUG1 = mm("R_LUG0"), mm("U_LUG0"), mm("U_LUG1")
LUG_W, GAP_W, TURN, STOP_W = deg(T["LUG_W"]), deg(T["GAP_W"]), deg(T["TURN"]), deg(T["STOP_W"])
LOCK_TH = [deg(t) for t in T["LOCK_TH"]]
R_ROD = mm("R_ROD")
U_CPL0, U_POCKET0, U_POCKET1, U_CPL1 = mm("U_CPL0"), mm("U_POCKET0"), mm("U_POCKET1"), mm("U_CPL1")
R_CPL, POCKET_HALF = mm("R_CPL"), mm("POCKET_HALF")
U_SLEEVE0, U_SLEEVE1, R_SLEEVE = mm("U_SLEEVE0"), mm("U_SLEEVE1"), mm("R_SLEEVE")
ROD_TRAVEL, SLEEVE_TRAVEL, KEEP_OPEN = mm("ROD_TRAVEL"), mm("SLEEVE_TRAVEL"), mm("KEEP_OPEN")
SLEEVE_OPEN, SLEEVE_CLOSE = mm("SLEEVE_OPEN"), mm("SLEEVE_CLOSE")
SLEEVE_RATE = T["SLEEVE"]["spring"] / 1000.0  # N/mm
F_ROD, F_ROLL = T["F_ROD"], T["F_ROLL"]
U_HEAD0, U_HEAD1, U_NECK1, U_TIP1 = mm("U_HEAD0"), mm("U_HEAD1"), mm("U_NECK1"), mm("U_TIP1")
R_HEAD, R_NECK, R_TIP = mm("R_HEAD"), mm("R_NECK"), mm("R_TIP")
R_CAP_RIM, R_CAP_TIP, SHELL_LEN = mm("R_CAP_RIM"), mm("R_CAP_TIP"), mm("SHELL_LEN")
U_RIM, U_BUSH1 = U_RING1, mm("U_BUSH1")
H_AX = mm("H_AX")
KEEPER_Z0 = 3.5  # task.py _keeper_shapes: z0 = 0.0035 (a local inside a function, so stated here and checked below)

# ---------------------------------------------------------------------------------------------------------------------
# 2. the replica's own design values (not in the simulation; each one is named in BUILD.md)

P = dict(
    seg=96,  # facets round a circle
    fit=0.15,  # radial clearance of sliding fits (printed)
    # the cap's knob: task.py's estimates until a real cap is measured (see ENROLMENT.md)
    knob_head_d=2 * R_HEAD, knob_neck_d=2 * R_NECK, knob_head_len=U_HEAD1 - U_HEAD0,
    rim_seat_d=37.0,  # the bush collar the cap's rim is stretched over (task.py draws it 37 mm across)
    # rod and its parts (catalogue parts in BOM.csv)
    rod_d=10.0, rod_len=200.0, rod_front=199.0,  # rod end inside the coupling (u)
    collar_od=20.0, collar_w=10.0,  # rod collar = the rod's forward stop, inside the nose
    spring_free=31.75, spring_rate=1.00, spring_od=12.19, spring_id=10.36,  # Lee Spring LC 036G 06 M
    spring_preload=0.3,  # N at rest: keeps the releasing ring on its forward stop
    spacer_od=14.0,
    brake_u=52.0,  # rod brake screw (M4 nylon tip) through the rear body into the rod bulkhead
    board_t=19.0,  # pine board the bracket is screwed to (1 x 6 in nominal: 19 mm)
    cam_rate=2.0,  # keeper lift per mm of releasing-ring travel
    pin_d=1.0, pin_z=8.3, pin_u0=215.0,  # keeper pin (1 x 6 mm dowel) in the drawbar, at rest
    bar_w=1.6, bar_z0=7.6, bar_z1=9.0,  # drawbar section (y' x z')
    cheek_y0=0.85, cheek_y1=4.65,  # each keeper cheek spans |y'| 0.85..4.65 (task.py keeper: |y'| < 5.5)
    cast_wall=1.5,  # fallback cast cap wall (genuine wall thickness unpublished; about 1 mm estimated in task.py)
)


def args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--knob-head-d", type=float)
    ap.add_argument("--knob-neck-d", type=float)
    ap.add_argument("--knob-head-len", type=float)
    ap.add_argument("--rim-seat-d", type=float)
    ap.add_argument("--no-drawings", action="store_true")
    a = ap.parse_args()
    for k in ("knob_head_d", "knob_neck_d", "knob_head_len", "rim_seat_d"):
        if getattr(a, k) is not None:
            P[k] = getattr(a, k)
    # A measured knob changes the first cam slope, preserving 1.5 mm release travel.
    inner = max(KEEPER_Z0, P["knob_neck_d"] / 2 + 0.5)
    overlap = P["knob_head_d"] / 2 - inner
    if not (0.5 <= overlap < KEEP_OPEN and P["knob_head_d"] / 2 + 0.5 <= R_CPL - 1.0
            and 0 < P["knob_head_len"] <= U_POCKET1 - U_POCKET0
            and 31.0 <= P["rim_seat_d"] <= 41.0):
        ap.error("measured cap is outside this coupling's design envelope; do not print or enrol it")
    P["cam_rate"] = overlap / SLEEVE_OPEN
    return a


# ---------------------------------------------------------------------------------------------------------------------
# 3. geometry helpers (STL frame: z = u along the axis)


def phi(th):
    """task.py angle th (deg, from +y' toward up) -> STL angle phi (deg, from +x toward +y)."""
    return 180.0 - th


def cyl(u0, u1, r, seg=None):
    return Manifold.cylinder(u1 - u0, r, r, seg or P["seg"]).translate((0, 0, u0))


def cone(u0, u1, r0, r1, seg=None):
    return Manifold.cylinder(u1 - u0, r0, r1, seg or P["seg"]).translate((0, 0, u0))


def tube(u0, u1, r0, r1, seg=None):
    return cyl(u0, u1, r1, seg) - cyl(u0 - 1, u1 + 1, r0, seg)


def sector(u0, u1, r0, r1, p0, p1, n=None):
    """Annular sector between angles p0..p1 (deg, STL frame), axial u0..u1."""
    n = n or max(4, int(abs(p1 - p0) / 2))
    a = np.radians(np.linspace(p0, p1, n + 1))
    pts = [(r1 * math.cos(t), r1 * math.sin(t)) for t in a] + [(r0 * math.cos(t), r0 * math.sin(t)) for t in a[::-1]]
    return Manifold.extrude(CrossSection([pts]), u1 - u0).translate((0, 0, u0))


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def revolve(profile, seg=None):
    """Solid of revolution about z from (r, u) points."""
    pts = [(max(r, 0.0), u) for r, u in profile]
    area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]))
    return Manifold.revolve(CrossSection([pts if area > 0 else pts[::-1]]), seg or P["seg"])


def at_phi(r, p):
    return r * math.cos(math.radians(p)), r * math.sin(math.radians(p))


def radial_hole(u, p, r_out, d, r_in=0.0):
    """A hole along the radius at angle p (STL deg), from r_in out to r_out, at axial u."""
    h = Manifold.cylinder(r_out - r_in + 2, d / 2, d / 2, 24).translate((0, 0, r_in - 1))  # along z
    h = h.rotate((-90, 0, 0))  # now along +y
    return h.rotate((0, 0, p - 90)).translate((0, 0, u))


def axial_hole(u0, u1, r, p, d, seg=24):
    x, y = at_phi(r, p)
    return Manifold.cylinder(u1 - u0, d / 2, d / 2, seg).translate((x, y, u0))


def to_trimesh(m: Manifold) -> trimesh.Trimesh:
    g = m.to_mesh()
    return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=False)


# ---------------------------------------------------------------------------------------------------------------------
# 4. the parts

LOCK = [phi(t) for t in LOCK_TH]  # lug centres, locked
OPEN = [p + TURN for p in LOCK]  # and open (task.py: OPEN_TH = LOCK_TH - TURN)
U_COLLAR1 = U_NOSE - 6.0  # the nose's front bore (rod collar stop) is 6 mm long
U_BULK1 = U_COLLAR1 - P["collar_w"] - ROD_TRAVEL  # rod bulkhead front face: the collar meets it after ROD_TRAVEL
U_REAR_JOINT = 104.0  # rear body | front body
U_SPRING0 = U_SLEEVE0 - (P["spring_free"] - P["spring_preload"] / P["spring_rate"])  # spring's back end (spacer front)


def body_rear():
    s = cyl(0, 12, 31.25) + cyl(12, U_REAR_JOINT, 22.5)  # flange 62.5 mm; body tube about 45 mm (task.py)
    s -= cyl(-1, U_REAR_JOINT + 1, P["rod_d"] / 2 + P["fit"] + 0.15)  # rod guide in the flange and the bulkhead
    s -= cyl(-1, U_BULK1 - 10.0, 11.0)  # hollow behind the bulkhead (the collar's way back)
    s -= cyl(U_BULK1, U_REAR_JOINT + 1, 11.0)
    s += cyl(U_BULK1 - 16.0, U_BULK1 - 10.0, 11.0) - cone(U_BULK1 - 16.01, U_BULK1 - 9.99, 11.0, 5.4)  # 45 deg under the bulkhead
    s -= radial_hole(P["brake_u"], -90, 23, 5.0, 9.0)  # access to the brake screw from below
    s -= radial_hole(P["brake_u"], -90, 9.5, 3.3)  # M4 pilot, 3.6 mm long: the set screw forms its thread here
    for p in (45, 135, 225, 315):  # 4 x M3 into the bracket, on 54 mm (instead of ISO 9409-50's M6 on 50)
        s -= axial_hole(-1, 13, 27.0, p, 3.4)
        s -= axial_hole(9, 13, 27.0, p, 6.2)
    for p in (30, 150, 270):  # heat-set inserts for the front body
        s -= axial_hole(U_REAR_JOINT - 6, U_REAR_JOINT + 1, 15.0, p, 4.0)
    return s


def body_front():
    s = cyl(U_REAR_JOINT, 110, 25.5) + cyl(110, U_BODY_FRONT, R_RING_OUT) + cyl(U_BODY_FRONT - 0.5, U_NOSE, R_NOSE)
    for k in range(3):  # bayonet lip: three arcs between the gaps that line up with the lugs' open positions
        g0, g1 = OPEN[k] + GAP_W / 2, OPEN[k] + 120 - GAP_W / 2  # to the next gap, counterclockwise
        s += sector(U_LIP0, U_LIP1, R_NOSE - 0.5, R_LIP, g0, g1)
        # stops in the groove: past the open position and past the locked position (task.py's offsets)
        po = OPEN[k] + LUG_W / 2 + STOP_W / 2 + 3
        pl = LOCK[k] - LUG_W / 2 - STOP_W / 2 - 3
        for c in (po, pl):
            s += sector(U_BODY_FRONT - 0.5, U_LIP0 + 0.01, R_NOSE - 0.5, R_LIP, c - STOP_W / 2, c + STOP_W / 2)
    s -= cyl(U_REAR_JOINT - 1, U_COLLAR1, P["collar_od"] / 2 + 0.6)  # the collar's bore (it retreats 55 mm)
    s -= cyl(U_COLLAR1 - 1, U_NOSE + 1, P["spacer_od"] / 2 + 0.5)  # front bore: the collar stops on its step
    for p in (30, 150, 270):  # M3 x 25 from the nose's front face into the rear body's inserts
        s -= axial_hole(U_REAR_JOINT - 1, U_NOSE + 1, 15.0, p, 3.4)
        s -= axial_hole(U_NOSE - 3.2, U_NOSE + 1, 15.0, p, 6.0)
    # paint grooves on the locking cover: 'locked' in line with the ring's index line, 'open' 45 deg on (wider)
    s -= sector(110.5, U_BODY_FRONT - 0.3, R_RING_OUT - 0.4, R_RING_OUT + 1, LOCK[0] - 3.5, LOCK[0] + 3.5)
    s -= sector(110.5, U_BODY_FRONT - 0.3, R_RING_OUT - 0.4, R_RING_OUT + 1, OPEN[0] - 9.0, OPEN[0] + 9.0)
    return s


def carrier():
    """Catch ring (with its lugs and front wall) and the cap's bush, one print: task.py's rigid carrier."""
    s = tube(U_RING0, U_RING1, R_RING_IN, R_RING_OUT)
    for j in range(24):  # knurl: 24 shallow flutes
        s -= sector(U_RING0 + 1.5, U_RING1 - 1.5, R_RING_OUT - 0.5, R_RING_OUT + 1, j * 15 + 3, j * 15 + 10)
    s += tube(U_RING1 - 5.0, U_RING1, 15.0, R_RING_IN + 0.1)  # front wall onto the bush (bore 30 mm)
    s += cyl(U_RING1 - 12.0, U_RING1 - 5.0, R_RING_IN + 0.1) - cone(
        U_RING1 - 12.01, U_RING1 - 4.99, R_RING_IN + 0.1, 18.4)  # 45 deg support under the front wall (printing)
    s += tube(130.0, U_BUSH1, 15.0, P["rim_seat_d"] / 2)  # bush, with the cap's rim seat at its front
    s -= tube(U_RIM + 1.5, U_BUSH1 - 1.5, P["rim_seat_d"] / 2 - 0.6, P["rim_seat_d"] / 2 + 1)  # bead groove
    for p in LOCK:  # bayonet lugs inside the ring's back end
        s += sector(U_LUG0, U_LUG1, R_LUG0, R_RING_IN + 0.5, p - LUG_W / 2, p + LUG_W / 2)
    s -= sector(U_RING0 + 0.5, U_RING1 - 0.5, R_RING_OUT - 0.5, R_RING_OUT + 1, LOCK[0] - 3.5 / R_RING_OUT * 57.3 / 2,
                LOCK[0] + 3.5 / R_RING_OUT * 57.3 / 2)  # index line (paint groove), 3.5 mm wide
    return s


def coupling():
    """On the rod's end: pocket for the knob's head, two keeper windows, two drawbar channels."""
    kp = keeper_params()
    s = cyl(U_CPL0, U_CPL1, R_CPL)
    s -= cyl(U_CPL0 - 1, P["rod_front"], P["rod_d"] / 2 + 0.05)  # rod bore (bonded)
    s -= cyl(U_POCKET0, U_POCKET1 + 0.01, kp["pocket_r"])  # pocket (round; task.py draws it 14 mm square)
    s -= cyl(U_POCKET1, U_CPL1 + 1, kp["pocket_r"])  # the front passage the head leaves by (keepers block it)
    for sg in (1, -1):
        y0, y1 = sorted((sg * (KEEPER_Z0 - 0.1), sg * (R_CPL + 1)))
        s -= box(-P["cheek_y1"] - 0.1, P["cheek_y1"] + 0.1, y0, y1, U_POCKET1 - 0.1, U_CPL1 + 1)  # keeper window
        s -= box(-P["cheek_y1"] - 1.0, P["cheek_y1"] + 1.0, y0, y1, 213.0 - 0.1, 215.0 + 0.1)  # tongue grooves
        y0, y1 = sorted((sg * (P["bar_z0"] - 0.1), sg * (R_CPL + 1)))
        s -= box(-P["bar_w"] / 2 - 0.1, P["bar_w"] / 2 + 0.1, y0, y1, 190.0, U_CPL1 + 1)  # drawbar channel
    return s


def keeper_params():
    hr = P["knob_head_d"] / 2
    return dict(
        head_r=hr,
        pocket_r=hr + 0.5,
        inner=max(KEEPER_Z0, P["knob_neck_d"] / 2 + 0.5),  # keeper inner edge (task.py: 3.5 mm off the axis)
        lift=KEEP_OPEN,
    )


def keeper_lift(s):
    """Keeper lift (mm, outward) for releasing-ring travel s (mm toward the rod): the cam slot's shape."""
    # The nominal curve is exactly min(2*s, 4). A sized insert still clears at
    # SLEEVE_OPEN and reaches the same 4 mm full opening at 2 mm sleeve travel.
    s = max(s, 0.0)
    at_release = P["cam_rate"] * SLEEVE_OPEN
    if s <= SLEEVE_OPEN:
        return P["cam_rate"] * s
    return min(at_release + (KEEP_OPEN - at_release) * (s - SLEEVE_OPEN) / (2.0 - SLEEVE_OPEN), KEEP_OPEN)


def cheek(top=True, right=True, lift=0.0):
    """One keeper half ('cheek'); four in all (two per keeper, either side of its drawbar)."""
    kp = keeper_params()
    y0, y1 = (P["cheek_y0"], P["cheek_y1"]) if right else (-P["cheek_y1"], -P["cheek_y0"])
    z = box(y0, y1, kp["inner"], R_CPL + 2, U_POCKET1, U_CPL1)
    ty0, ty1 = (P["cheek_y1"] - 0.1, P["cheek_y1"] + 0.85) if right else (-P["cheek_y1"] - 0.85, -P["cheek_y1"] + 0.1)
    z += box(ty0, ty1, kp["inner"] + 0.6, R_CPL + 2, 213.0, 215.0)  # tongue in the side jaw's groove
    z = z ^ cyl(U_POCKET1 - 1, U_CPL1 + 1, R_CPL)  # trimmed to the coupling's 19 mm outline
    # cam slot, in the cheek's own frame: the pin's path relative to the cheek over 0..3 mm of ring travel
    pts = []
    for s in np.linspace(0, SLEEVE_TRAVEL + 0.3, 34):
        pts.append((P["pin_u0"] - s, P["pin_z"] - keeper_lift(s)))
    slot = None
    for (u_a, z_a), (u_b, z_b) in zip(pts[:-1], pts[1:]):
        seg = Manifold.hull_points(np.array([[x, z0 + dz, u0 + du] for (u0, z0) in ((u_a, z_a), (u_b, z_b))
                                              for x in (-6, 6)
                                              for dz, du in ((math.cos(t) * (P["pin_d"] / 2 + 0.05),
                                                              math.sin(t) * (P["pin_d"] / 2 + 0.05))
                                                             for t in np.linspace(0, 2 * math.pi, 12, endpoint=False))]))
        slot = seg if slot is None else slot + seg
    z -= slot
    if not top:
        z = z.mirror((0, 1, 0))
    return z.translate((0, lift if top else -lift, 0))


def sleeve(travel=0.0):
    """Releasing ring with its two drawbars and their keeper pins (one print; the pins are dowels)."""
    s = tube(U_SLEEVE0 + 1.0, U_SLEEVE1, R_CPL + P["fit"], R_SLEEVE)
    s += tube(U_SLEEVE0, U_SLEEVE0 + 1.0, P["rod_d"] / 2 + P["fit"] + 0.2, R_SLEEVE)  # rear wall: spring seat
    for sg in (1, -1):
        y0, y1 = sorted((sg * P["bar_z0"], sg * P["bar_z1"]))
        bar = box(-P["bar_w"] / 2, P["bar_w"] / 2, y0, y1, 193.0, U_CPL1)
        k0, k1 = sorted((sg * P["bar_z0"], sg * (R_CPL + 0.5)))
        bar += box(-P["bar_w"] / 2, P["bar_w"] / 2, k0, k1, 193.0, U_SLEEVE1)  # key into the ring
        bar -= Manifold.cylinder(8, P["pin_d"] / 2, P["pin_d"] / 2, 16).rotate((0, 90, 0)).translate(
            (-4, sg * P["pin_z"], P["pin_u0"]))
        s += bar
    return s.translate((0, 0, -travel))


def pins(travel=0.0):
    out = None
    for sg in (1, -1):
        p = Manifold.cylinder(6.0, P["pin_d"] / 2, P["pin_d"] / 2, 16).rotate((0, 90, 0)).translate(
            (-3.0, sg * P["pin_z"], P["pin_u0"] - travel))
        out = p if out is None else out + p
    return out


def rod(push=0.0):
    r = cyl(P["rod_front"] - P["rod_len"], P["rod_front"], P["rod_d"] / 2, 48)
    return r.translate((0, 0, -push))


def collar(push=0.0):
    return tube(U_COLLAR1 - P["collar_w"], U_COLLAR1, P["rod_d"] / 2, P["collar_od"] / 2).translate((0, 0, -push))


def spacer():
    return tube(U_COLLAR1, U_SPRING0, P["rod_d"] / 2 + 0.2, P["spacer_od"] / 2)


def spring_envelope():
    return tube(U_SPRING0, U_SLEEVE0, P["spring_id"] / 2, P["spring_od"] / 2, 48)


def knob():
    """Stand-in for the cap's knob (sizes are task.py's estimates or caliper values). Printed, it is also the insert
    cast into the fallback cap."""
    hr, nr = P["knob_head_d"] / 2, P["knob_neck_d"] / 2
    u0 = U_HEAD1 - P["knob_head_len"]
    k = cyl(u0, U_HEAD1, hr, 64) + cyl(U_HEAD1 - 0.1, U_NECK1 + 1.0, nr, 48)
    a = cyl(U_NECK1 + 1.0, U_NECK1 + 3.0, 8.0, 64)  # anchor disc, inside the tip's silicone
    for p in (0, 90, 180, 270):
        a -= axial_hole(U_NECK1, U_NECK1 + 4, 5.5, p, 3.0)
    return k + a


def bracket():
    """Stands the replica's mounting face upright, its axis H_AX above the table, screwed to a board clamped at the
    table's edge."""
    floor = -(H_AX - P["board_t"])  # y of the board's top face
    s = box(-60, 60, floor, floor + 8, -110, 0)  # base plate
    s += box(-45, 45, floor, 38, -10, 0)  # upright plate, behind the mounting face
    for x in (-45, 37):  # two ribs behind the upright plate
        s += box(x, x + 8, floor + 8, floor + 50, -70, -10)
    s -= cyl(-11, 1, P["rod_d"] / 2 + P["fit"] + 0.15)  # rod guide
    for p in (45, 135, 225, 315):
        s -= axial_hole(-7, 1, 27.0, p, 4.0)  # heat-set inserts
    for x in (-50, 50):
        for z in (-95, -25):
            s -= Manifold.cylinder(12, 2.25, 2.25, 24).rotate((-90, 0, 0)).translate((x, floor - 1, z))  # #8 screws
    return s


def cap_profile_out(v):
    """task.py's cap outline: radius at distance v in front of the rim (rim 41 mm, tip 25 mm)."""
    vt = (U_NECK1 - U_RIM)
    return R_CAP_RIM - (R_CAP_RIM - R_CAP_TIP) * min(max(v, 0.0), vt) / vt


def mould_parts():
    """Fallback: a two-half outer mould and a core for casting a cap in platinum silicone (one-time; the knob
    insert sits in the core's tip pocket). Cap axis vertical, rim up. Frame as the other STLs (z = u)."""
    vt, vtip = U_NECK1 - U_RIM, U_TIP1 - U_RIM
    w = P["cast_wall"]
    outer = [(0, U_TIP1), (R_CAP_TIP - 1.5, U_TIP1), (R_CAP_TIP, U_TIP1 - 1.5)] + [
        (cap_profile_out(v), U_RIM + v) for v in np.linspace(vt, 0, 20)] + [(R_CAP_RIM, U_RIM - 12), (0, U_RIM - 12)]
    cap_out = revolve(outer)
    inner = [(0, U_NECK1)] + [(cap_profile_out(v) - w, U_RIM + v) for v in np.linspace(vt, 5.0, 20)] + [
        (P["rim_seat_d"] / 2 - 0.5, U_RIM + 4.5), (P["rim_seat_d"] / 2 - 0.5, U_RIM), (R_CAP_RIM - 0.1, U_RIM),
        (R_CAP_RIM - 0.1, U_RIM - 12), (0, U_RIM - 12)]
    core = revolve(inner) + cyl(U_RIM - 22, U_RIM - 12 + 0.01, 30.0)  # core, its plug and locating lid
    core -= cyl(U_HEAD1 - P["knob_head_len"] - 0.5, U_HEAD1 + 0.3, P["knob_head_d"] / 2 + 0.2, 64)  # knob pocket
    core -= cyl(U_HEAD1, U_NECK1 + 0.5, P["knob_neck_d"] / 2 + 0.2, 48)
    for p in (0, 120, 240):  # overflow vents through the lid over the rim
        core -= axial_hole(U_RIM - 23, U_RIM + 0.5, (P["rim_seat_d"] / 2 + R_CAP_RIM) / 2 - 0.3, p, 1.6)
    block = cyl(U_RIM - 12, U_TIP1 + 5, 30.0) - cap_out
    halves = []
    for sx in (1, -1):
        h = block ^ box(0 if sx > 0 else -40, 40 if sx > 0 else 0, -40, 40, U_RIM - 20, U_TIP1 + 10)
        for u in (U_RIM - 2, U_RIM + 40, U_TIP1 - 2):  # M3 bolts across the split
            for y in (-25.0, 25.0):
                h -= Manifold.cylinder(70, 1.7, 1.7, 16).rotate((0, 90, 0)).translate((-35, y, u))
        halves.append(h)
    return halves[0], halves[1], core


# ---------------------------------------------------------------------------------------------------------------------
# 5. checks


class Checks:
    def __init__(self):
        self.lines, self.failed = [], 0

    def __call__(self, ok, what, got="", want=""):
        tag = "ok  " if ok else "FAIL"
        self.failed += 0 if ok else 1
        self.lines.append(f"{tag} {what}" + (f": {got}" if got != "" else "") + (f" (task.py: {want})" if want != "" else ""))

    def near(self, got, want, what, tol=0.02):
        self(abs(got - want) <= tol, what, f"{got:.3f}", f"{want:.3f}")


def inside_slice(m: Manifold, u, pts):
    """Even-odd test of 2D points against the part's slice at axial u."""
    inside = np.zeros(len(pts), bool)
    for poly in m.slice(u).to_polygons():
        poly = np.asarray(poly)
        x0, y0 = poly[:, 0], poly[:, 1]
        x1, y1 = np.roll(x0, -1), np.roll(y0, -1)
        for i, (px, py) in enumerate(pts):
            c = ((y0 > py) != (y1 > py)) & (px < (x1 - x0) * (py - y0) / (y1 - y0 + 1e-300) + x0)
            inside[i] ^= bool(np.count_nonzero(c) % 2)
    return inside


def arcs(m: Manifold, u, r, n=1440):
    """Angular intervals (deg, STL frame) where a circle of radius r at axial u lies inside the part."""
    t = np.linspace(0, 360, n, endpoint=False)
    pts = np.c_[r * np.cos(np.radians(t)), r * np.sin(np.radians(t))]
    inside = inside_slice(m, u, pts)
    if inside.all():
        return [(0.0, 360.0)]
    k0 = int(np.argmin(inside))  # start on a gap
    order = np.roll(np.arange(n), -k0)
    out, start = [], None
    for k in order:
        if inside[k] and start is None:
            start = t[k]
        if not inside[k] and start is not None:
            out.append((start, t[k] if t[k] > start else t[k] + 360))
            start = None
    if start is not None:
        out.append((start, t[k0] + 360))
    return out


def centre_width(iv):
    a, b = iv
    return ((a + b) / 2) % 360, b - a


def angdiff(a, b):
    return abs((a - b + 180) % 360 - 180)


def run_checks(parts, C: Checks):
    for name, m in parts.items():
        tm = to_trimesh(m)
        C(tm.is_watertight and tm.is_winding_consistent and tm.volume > 0 and m.genus() >= 0,
          f"{name}: closed, consistently wound solid", f"{tm.volume / 1000:.2f} cm3, genus {m.genus()}")
    tm = {k: to_trimesh(v) for k, v in parts.items()}
    step = 360.0 / P["seg"]

    # outer sizes
    C.near(float(np.hypot(*tm["carrier"].vertices[:, :2].T).max()), R_RING_OUT, "catch ring radius (57.5 mm across)")
    C.near(float(tm["carrier"].bounds[0][2]), U_RING0, "catch ring back face, u")
    C.near(float(tm["carrier"].bounds[1][2]), U_BUSH1, "bush collar front, u (cap rim seat)")
    C.near(float(np.hypot(*tm["coupling"].vertices[:, :2].T).max()), R_CPL, "coupling radius (19 mm across)")
    C.near(float(tm["coupling"].bounds[0][2]), U_CPL0, "coupling back face, u")
    C.near(float(tm["coupling"].bounds[1][2]), U_CPL1, "coupling front face, u")
    sl = tm["sleeve"]
    C.near(float(np.hypot(*sl.vertices[:, :2].T).max()), R_SLEEVE, "releasing ring radius (22 mm across)")
    ring_part = sl.vertices[np.hypot(*sl.vertices[:, :2].T) > R_CPL + 0.6]
    C.near(float(ring_part[:, 2].min()), U_SLEEVE0, "releasing ring back face, u")
    C.near(float(ring_part[:, 2].max()), U_SLEEVE1, "releasing ring front face, u (13.5 mm long)")
    fr = tm["body_front"]
    C.near(float(fr.bounds[1][2]), U_NOSE, "nose front, u")
    C.near(float(np.hypot(*fr.vertices[fr.vertices[:, 2] > U_LIP1 + 0.5][:, :2].T).max()), R_NOSE, "nose radius")
    C.near(float(np.hypot(*fr.vertices[:, :2].T).max()), R_RING_OUT, "locking cover radius")
    C.near(float(tm["body_rear"].bounds[0][2]), 0.0, "mounting face, u")

    # bayonet: lugs on the ring, lip gaps and stops on the nose
    lug_arcs = arcs(parts["carrier"], (U_LUG0 + U_LUG1) / 2, (R_LUG0 + R_RING_IN) / 2)
    C(len(lug_arcs) == 3, "three lugs", len(lug_arcs), 3)
    for iv in lug_arcs:
        c, w = centre_width(iv)
        k = int(np.argmin([angdiff(c, p) for p in LOCK]))
        C(angdiff(c, LOCK[k]) <= step and abs(w - LUG_W) <= step + 0.5, f"lug at {c:.1f} deg, {w:.1f} deg wide",
          "", f"{LOCK[k] % 360:.1f} deg, {LUG_W:.1f}")
    lip_arcs = arcs(parts["body_front"], (U_LIP0 + U_LIP1) / 2, (R_NOSE + R_LIP) / 2 + 0.5)
    gaps = [(((lip_arcs[i][1] + ((lip_arcs[(i + 1) % len(lip_arcs)][0] - lip_arcs[i][1]) % 360) / 2) % 360),
             (lip_arcs[(i + 1) % len(lip_arcs)][0] - lip_arcs[i][1]) % 360) for i in range(len(lip_arcs))]
    C(len(gaps) == 3, "three gaps in the lip", len(gaps), 3)
    for c, w in gaps:
        k = int(np.argmin([angdiff(c, p) for p in OPEN]))
        C(angdiff(c, OPEN[k]) <= step and abs(w - GAP_W) <= step + 0.5, f"lip gap at {c:.1f} deg, {w:.1f} deg wide",
          "", f"{OPEN[k] % 360:.1f} deg, {GAP_W:.1f}")
    # lugs and lip seen along the axis: locked they overlap (a pull is blocked), open they pass
    for name, rot, want in (("locked", 0.0, True), ("turned open", TURN, False)):
        lugs = parts["carrier"].rotate((0, 0, rot)) ^ cyl(U_LUG0 - 1, U_LUG1 + 1, R_RING_IN)
        lip_prism = (parts["body_front"] ^ cyl(U_LIP0 + 0.5, U_LIP1 - 0.5, 30)).project()
        lug_prism = lugs.project()
        ov = (lip_prism ^ lug_prism).area()
        C((ov > 1.0) == want, f"bayonet {name}: lugs {'behind' if want else 'clear of'} the lip seen along the axis",
          f"overlap {ov:.1f} mm2")
    # stops: turning on past open or back past locked is blocked (lugs hit a stop at TURN + 3 deg / -3 deg)
    for name, rot in (("past open", TURN + 4), ("back past locked", -4)):
        lugs = parts["carrier"].rotate((0, 0, rot)) ^ cyl(U_LUG0, U_LUG1, R_RING_IN + 0.4)
        hit = (lugs ^ parts["body_front"]).volume()
        C(hit > 0.5, f"turning {name} hits a stop", f"{hit:.2f} mm3 overlap")
    for name, rot in (("locked", 0.0), ("open", TURN)):
        hit = (parts["carrier"].rotate((0, 0, rot)) ^ parts["body_front"]).volume()
        C(hit < 0.05, f"catch ring {name} does not overlap the body", f"{hit:.3f} mm3")

    # the latch: interference-free through the releasing ring's whole travel, and when it lets the knob go
    kp = keeper_params()
    if abs(P["knob_neck_d"] - 2 * R_NECK) < 1e-9:
        C.near(kp["inner"], KEEPER_Z0, "keeper inner edge off the axis (closed)")
    else:
        C(kp["inner"] >= KEEPER_Z0, "sized keeper edge (nominal simulation 3.5 mm)", f"{kp['inner']:.3f} mm")
    C.near(keeper_lift(SLEEVE_TRAVEL), KEEP_OPEN, "keeper lift at full ring travel (4 mm)")
    s_clear = (kp["head_r"] - kp["inner"]) / P["cam_rate"]
    C.near(s_clear, SLEEVE_OPEN, "ring travel at which the keepers clear the knob's head (sim: latch opens)")
    C(SLEEVE_CLOSE < s_clear, "below 1.0 mm of ring travel the keepers hold the head (sim: latch closes)",
      f"clear only from {s_clear:.2f} mm", f"{SLEEVE_CLOSE:.1f}")
    cpl = parts["coupling"]
    worst = 0.0
    for s in np.linspace(0, SLEEVE_TRAVEL, 13):
        d = keeper_lift(s)
        ch = [cheek(t, r, d) for t in (True, False) for r in (True, False)]
        movers = [("sleeve", sleeve(s)), ("pins", pins(s))] + [(f"cheek{i}", c) for i, c in enumerate(ch)]
        for n1, m1 in movers:
            if n1 != "pins":
                worst = max(worst, (m1 ^ cpl).volume())
        for c in ch:
            worst = max(worst, (c ^ sleeve(s)).volume(), (c ^ pins(s)).volume())
        if s <= s_clear - 0.05:
            blocked = sum((c ^ knob().translate((0, 0, 1.0))).volume() for c in ch)
            C(blocked > 0.1, f"ring {s:.2f} mm: the knob drawn 1 mm forward meets the keepers",
              f"radial overlap {kp['head_r'] - kp['inner'] - d:.2f} mm, {blocked:.1f} mm3")
        if s >= s_clear + 0.05:
            free = sum((c ^ knob().translate((0, 0, U_CPL1 - U_HEAD0 + 6))).volume() for c in ch)
            path = sum((c ^ cyl(U_POCKET1 - 1, U_CPL1 + 1, kp["head_r"])).volume() for c in ch)
            C(path < 0.05 and free < 0.05, f"ring {s:.2f} mm: the head's path through the keepers is clear",
              f"{path:.3f} mm3")
    C(worst < 0.05, "no part of the latch overlaps another anywhere in the ring's 3 mm travel", f"{worst:.3f} mm3")
    stop_gap = 193.0 - 190.0
    C.near(stop_gap, SLEEVE_TRAVEL, "releasing ring's travel to its stop (drawbar key meets the channel end)")
    hit = (sleeve(SLEEVE_TRAVEL + 0.2) ^ cpl).volume()
    C(hit > 0.05, "pushed past 3 mm the releasing ring is stopped", f"{hit:.2f} mm3")
    hit = (sleeve(-0.2) ^ cpl).volume()
    C(hit > 0.05, "the releasing ring's rest stop (rear wall on the coupling)", f"{hit:.2f} mm3")
    C.near(P["spring_rate"] * 1000, SLEEVE_RATE * 1000, "return spring rate, N/m (Lee Spring LC 036G 06 M)", tol=0.5)

    # rod: its 55 mm stroke ends on the bulkhead; its forward stop is the collar on the nose's step
    hit = (collar(ROD_TRAVEL + 0.2) ^ parts["body_rear"]).volume()
    free = (collar(ROD_TRAVEL - 0.2) ^ parts["body_rear"]).volume() + (collar(ROD_TRAVEL - 0.2) ^ parts["body_front"]).volume()
    C(hit > 0.05 and free < 0.05, "rod pushes back 55 mm, then the collar meets the bulkhead", f"{ROD_TRAVEL:.0f} mm")
    hit = (collar(-0.2) ^ parts["body_front"]).volume()
    C(hit > 0.05, "rod's forward stop (collar on the nose's step)")
    for name in ("body_rear", "body_front", "bracket"):
        hit = (rod() ^ parts[name]).volume() + (rod(ROD_TRAVEL) ^ parts[name]).volume()
        C(hit < 0.05, f"rod slides free in the {name.replace('_', ' ')}", f"{hit:.3f} mm3")
    C.near(P["rod_d"] / 2, R_ROD, "rod radius")
    L_inst = U_SLEEVE0 - U_SPRING0
    C(6.63 + SLEEVE_TRAVEL < L_inst < P["spring_free"] and U_SPRING0 > U_NOSE + 5,
      "spring fits between the spacer and the ring, not solid at full travel (solid height 6.63 mm)",
      f"installed {L_inst:.2f} mm, {L_inst - SLEEVE_TRAVEL:.2f} at 3 mm; preload {(P['spring_free'] - L_inst) * P['spring_rate']:.2f} N")
    for name, m in (("spacer", spacer()), ("spring", spring_envelope())):
        hit = sum((m ^ parts[n]).volume() for n in ("body_front", "carrier", "coupling"))
        C(hit < 0.05, f"{name} clear of the nose, carrier and coupling", f"{hit:.3f} mm3")

    # fixed parts against each other at rest
    for a, b in (("body_rear", "body_front"), ("body_front", "carrier"), ("carrier", "coupling"),
                 ("body_rear", "bracket"), ("coupling", "sleeve")):
        hit = (parts[a] ^ parts[b]).volume()
        C(hit < 0.05, f"{a} and {b} do not overlap", f"{hit:.3f} mm3")
    C.near(-tm["bracket"].bounds[0][1] + P["board_t"], H_AX, "axis height above the table (bracket on its board)")
    hit = (parts["carrier"] ^ cyl(100, 230, R_SLEEVE)).volume()
    C(hit < 0.05, "the bush's 30 mm bore passes the releasing ring (the cap rolls through it)")
    # the knob in place: in the pocket, clear of the keepers
    hit = (knob() ^ parts["coupling"]).volume() + sum((cheek(t, r) ^ knob()).volume() for t in (True, False) for r in (True, False))
    C(hit < 0.05, "knob seated in the pocket without touching the coupling or keepers", f"{hit:.3f} mm3")


# ---------------------------------------------------------------------------------------------------------------------
# 6. drawings


def drawings(parts):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    from matplotlib.path import Path as PlotPath
    from matplotlib.patches import PathPatch

    col = {"body_rear": "#9aa0a6", "body_front": "#3c4043", "carrier": "#202124", "coupling": "#bdc1c6",
           "sleeve": "#1a5fb4", "keeper": "#7d8590", "rod": "#80868b", "collar": "#5f6368", "spacer": "#c8ccd0",
           "knob": "#8a6d3b", "bracket": "#e8a33d"}

    def fill_loops(ax, polygons, color, alpha=0.85, reflect_u=False):
        # Compound nonzero winding preserves bores/holes in manifold's oriented loops.
        loops = []
        for polygon in polygons:
            e = np.asarray(polygon).copy()
            if reflect_u:
                e[:, 0] *= -1
            e = np.vstack([e, e[0]])
            codes = [PlotPath.MOVETO] + [PlotPath.LINETO] * (len(e) - 2) + [PlotPath.CLOSEPOLY]
            loops.append(PlotPath(e, codes))
        if loops:
            ax.add_patch(PathPatch(PlotPath.make_compound_path(*loops), facecolor=color,
                                   alpha=alpha, lw=0.3, edgecolor="k"))

    def section(ax, m, c, plane_x=0.0):
        fill_loops(ax, m.rotate((0, -90, 0)).slice(plane_x).to_polygons(), c, reflect_u=True)

    def show(ax, items, title, xlim, ylim, plane_x=0.0):
        for m, c in items:
            section(ax, m, c, plane_x)
        ax.set_aspect("equal")
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_title(title, fontsize=9)
        ax.set_xlabel("u: mm from the mounting face along the axis (toward the cap tip)", fontsize=7)
        ax.set_ylabel("mm above the axis", fontsize=7)
        ax.tick_params(labelsize=6)
        ax.grid(alpha=0.25, lw=0.4)

    def dim(ax, x0, x1, y, text, vertical=False):
        if vertical:
            ax.annotate("", (y, x0), (y, x1), arrowprops=dict(arrowstyle="<->", lw=0.6))
            ax.text(y + 0.6, (x0 + x1) / 2, text, fontsize=6, va="center")
        else:
            ax.annotate("", (x0, y), (x1, y), arrowprops=dict(arrowstyle="<->", lw=0.6))
            ax.text((x0 + x1) / 2, y + 0.6, text, fontsize=6, ha="center")

    rest = [(parts["bracket"], col["bracket"]), (parts["body_rear"], col["body_rear"]),
            (parts["body_front"], col["body_front"]), (parts["carrier"], col["carrier"]), (rod(), col["rod"]),
            (collar(), col["collar"]), (spacer(), col["spacer"]), (parts["coupling"], col["coupling"]),
            (sleeve(), col["sleeve"]), (knob(), col["knob"])] + [(cheek(t, r), col["keeper"]) for t in (1, 0) for r in (1, 0)]
    pdf_path = HERE / "drawing.pdf"
    with PdfPages(pdf_path) as pdf:
        fig, ax = plt.subplots(figsize=(11.7, 8.3))
        show(ax, rest, "DHEF-20 front-end replica, section in the vertical plane through the axis (CAD, not a photo). "
                       "Locked, rod forward, releasing ring at rest. Cap not shown.", (-115, 230), (-65, 45))
        dim(ax, 0, U_NOSE, 40, f"nose front {U_NOSE:.1f}")
        dim(ax, U_RING0, U_RING1, 33, f"catch ring {U_RING0:.1f}..{U_RING1:.1f}, 57.5 across")
        dim(ax, U_CPL0, U_CPL1, 16, f"coupling {U_CPL0:.0f}..{U_CPL1:.0f}, 19 across")
        dim(ax, U_SLEEVE0, U_SLEEVE1, 13, f"releasing ring {U_SLEEVE0:.0f}..{U_SLEEVE1:.1f}, 22 across", )
        dim(ax, -H_AX + P["board_t"], 0, -100, f"axis {H_AX:.0f} above the table\n(board {P['board_t']:.0f})", vertical=True)
        ax.text(-110, -64, "Units mm. Generated by make_replica.py from task.py; checks in CHECKS.txt.", fontsize=6)
        pdf.savefig(fig)
        fig.savefig(HERE / "section.png", dpi=110)
        plt.close(fig)

        fig, axs = plt.subplots(1, 3, figsize=(11.7, 4.6))
        for ax, s in zip(axs, (0.0, SLEEVE_OPEN, SLEEVE_TRAVEL)):
            d = keeper_lift(s)
            items = [(parts["coupling"], col["coupling"]), (sleeve(s), col["sleeve"]), (rod(), col["rod"]),
                     (knob(), col["knob"])] + [(cheek(t, True, d), col["keeper"]) for t in (True, False)]
            show(ax, items, f"Ring pushed {s:.1f} mm toward the rod:\nkeepers out {d:.1f} mm", (184, 222),
                 (-15, 15), plane_x=2.75)
            for sg in (1, -1):
                ax.add_patch(plt.Circle((P["pin_u0"] - s, sg * P["pin_z"]), P["pin_d"] / 2, color="k"))
        fig.suptitle("Coupling latch, section 2.75 mm off the axis through one keeper cheek (CAD). The pins (dowels) in "
                     "the ring's drawbars ride the cheeks' cam slots.", fontsize=9)
        fig.tight_layout()
        pdf.savefig(fig)
        fig.savefig(HERE / "latch.png", dpi=110)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(8.3, 8.3))
        for m, c, u in ((parts["carrier"], "#f0c27a", (U_LUG0 + U_LUG1) / 2),
                        (parts["body_front"], "#6b8fb8", (U_LIP0 + U_LIP1) / 2)):
            fill_loops(ax, m.slice(u).to_polygons(), c, alpha=0.6)
        for k in range(3):
            for p, txt in ((LOCK[k], "lug (locked)"), (OPEN[k], "gap / lug open")):
                x, y = at_phi(31, p)
                ax.text(x, y, f"{txt}\n{p % 360:.0f} deg", fontsize=6, ha="center")
        ax.set_aspect("equal")
        ax.set_xlim(-36, 36)
        ax.set_ylim(-36, 36)
        ax.set_title("Bayonet seen from the cap's tip (CAD), locked. Blue: nose with its lip (u 121.5..125.5);\n"
                     "orange: catch ring with its three lugs (u 117..121). Lugs 18 deg, lip gaps 50 deg; the catch\n"
                     "turns 45 deg counterclockwise to open, lining each lug up with a gap. Angles from +x.", fontsize=8)
        ax.tick_params(labelsize=6)
        pdf.savefig(fig)
        fig.savefig(HERE / "bayonet.png", dpi=110)
        plt.close(fig)

    # an exploded view, shaded faces (CAD render)
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(14, 4.0))
    ax = fig.add_axes((-0.25, -0.6, 1.5, 2.0), projection="3d")
    explode = [("bracket", parts["bracket"], -40), ("body_rear", parts["body_rear"], -20),
               ("body_front", parts["body_front"], 0), ("rod", rod(), 30), ("collar", collar(), 30),
               ("spacer", spacer(), 30), ("carrier", parts["carrier"], 60), ("sleeve", sleeve(), 90),
               ("coupling", parts["coupling"], 120), ("knob", knob(), 150)]
    explode += [("keeper", cheek(t, r), 120) for t in (True, False) for r in (True, False)]
    light = np.array([0.4, 0.8, 0.45])
    for name, m, dz in explode:
        tmx = to_trimesh(m.translate((0, 0, dz)))
        tri = tmx.vertices[tmx.faces][:, :, [2, 0, 1]]
        n = np.nan_to_num(tmx.face_normals[:, [2, 0, 1]])
        shade = 0.45 + 0.55 * np.clip(np.sum(n * (light / np.linalg.norm(light))[None, :], axis=1), 0, 1)
        base = np.array(matplotlib.colors.to_rgb(col[name]))
        pc = Poly3DCollection(tri, facecolors=np.clip(base[None] * shade[:, None] + 0.08, 0, 1), linewidths=0)
        ax.add_collection3d(pc)
    ax.set_xlim(-120, 380)
    ax.set_ylim(-60, 60)
    ax.set_zlim(-70, 50)
    ax.set_box_aspect((500, 120, 120), zoom=1.0)
    ax.view_init(elev=18, azim=-62)
    ax.set_axis_off()
    ax.set_title("Exploded CAD render (not a photo): bracket, rear body, front body with bayonet lip, rod with collar "
                 "and spacer, catch ring and bush, releasing ring, coupling with keepers, knob stand-in", fontsize=8)
    fig.savefig(HERE / "assembly.png", dpi=110, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------------------------------------------------------------


def main():
    a = args()
    OUT.mkdir(exist_ok=True)
    parts = {
        "bracket": bracket(), "body_rear": body_rear(), "body_front": body_front(), "carrier": carrier(),
        "coupling": coupling(), "sleeve": sleeve(), "spacer": spacer(), "knob": knob(),
    }
    for t in (True, False):
        for r in (True, False):
            parts[f"keeper_{'top' if t else 'bottom'}_{'xpos' if r else 'xneg'}"] = cheek(t, r)
    m1, m2, mc = mould_parts()
    parts.update(mould_half_xpos=m1, mould_half_xneg=m2, mould_core=mc)
    C = Checks()
    run_checks(parts, C)
    for name, m in parts.items():
        to_trimesh(m).export(OUT / f"{name}.stl")
    if not a.no_drawings:
        drawings(parts)
    head = [
        "make_replica.py checks (generated; do not edit)",
        f"knob head {P['knob_head_d']:.2f} mm across, {P['knob_head_len']:.2f} long; neck {P['knob_neck_d']:.2f}; "
        f"rim seat {P['rim_seat_d']:.2f} mm",
        f"{len(C.lines) - C.failed} passed, {C.failed} failed",
        "CAD geometry checks only; no physical fit, force, strength or cycle validation.",
        f"First cam slope {P['cam_rate']:.6f} mm/mm; nominal simulation knob remains 13 x 10 mm, neck 6 mm.",
        "",
    ]
    (HERE / "CHECKS.txt").write_text("\n".join(head + C.lines) + "\n")
    files = sorted([p for p in HERE.rglob("*") if p.is_file() and p.name not in ("SHA256SUMS",)
                    and not {"__pycache__", ".venv"}.intersection(p.parts)])
    (HERE / "SHA256SUMS").write_text("".join(f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(HERE)}\n"
                                             for p in files))
    print("\n".join(head + C.lines))
    sys.exit(1 if C.failed else 0)


if __name__ == "__main__":
    main()
