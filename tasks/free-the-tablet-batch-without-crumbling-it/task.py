"""A reusable tablet-demolding practice kit: sixty white resin tablet replicas have a chosen 0.1 mm pushed-face recess in a black resin plate
with Torpac-size tapered cavities, narrow openings up. Two aluminium bars park the plate above a parchment sheet.
A long rim extension ends in a hollow upright grip tube for the supporting hand; it must carry the plate clear
of the parking supports during pushing. The parts are reusable training specimens, not medication.

An adapted white polymer spreader has a central 1.8 mm pin projecting 12 mm below its card for individual pushes,
a thick upper grip boss and an offset sensor cover. It stands upright in a slotted resin block beside the plate.
Push the sixty replicas onto the parchment with the pin within the stated force limits, then set the tool and
plate down and move both hands clear.
"""
from __future__ import annotations
import hashlib, json, math, os, pathlib, sys
SOURCE_SHA256 = hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts, lots
parts.SOLVER_ITERATIONS = (128, 128)
_original_patch_scene = parts._PATCHES['yam_arena.scene']
def _fine_physics(scene):
    _original_patch_scene(scene)
    scene.PHYSICS_DT = 1. / 240.
    original_env_callback=scene.env_cfg_callback
    def pgs_env_callback(control_hz=None):
        original_configure=original_env_callback(control_hz)
        def configure(cfg):
            cfg=original_configure(cfg)
            from isaaclab_physx.physics.physx_manager_cfg import PhysxCfg
            if cfg.sim.physics is None: cfg.sim.physics=PhysxCfg()
            cfg.sim.physics.solver_type=0
            return cfg
        return configure
    scene.env_cfg_callback=pgs_env_callback
parts._PATCHES['yam_arena.scene'] = _fine_physics
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint, FINGER_OPEN, FINGER_CLOSED

SCENE = 'station'; TOP = TABLE_TOP[SCENE]; SCORE_OVER = 'end'; SUCCESS_HOLD_S = .05
EPS = 1e-7

# ---- Geometry, metres. Cavity plate of the X-MRDT60x075 mold (maker brochure 2509: 120 x 152 mm mold, holes 6.5 mm
# upper / 5.5 mm lower diameter, 3.7 mm high, 0.10 ml, 75 mg tablets). Hole pitch is estimated from the maker's photos.
PW, PL, PH = .120, .152, .0037            # plate x (across), y (along), thickness = hole height
NCOL, NROW = 6, 10
PITCH_X, PITCH_Y = .0125, .014
SIDES = 8                                  # holes and tablets are 8-sided frustums standing in for round ones
A_TOP, A_BOT = .00275, .00325              # hole apothems: narrow 5.5 mm end up (plate inverted), wide 6.5 mm end down
CLEAR = .00004                            # m radial clearance; technician phone figure, Nov 2026
N = NCOL * NROW
PARCH_T = .0002                            # parchment sheet (modelled 0.2 mm)
PT = TOP + PARCH_T                         # parchment top
BAR = .0127; BAR_L = .1524                 # 1/2 in square 6061 aluminium bar, 6 in long
PX, PY = .275, 0.                          # plate centre (same in every lot)
BAR_DX = PW / 2 - BAR / 2                  # bars under the plate's two long margins
PLATE_Z = PT + BAR + PH / 2
BACK = PLATE_Z + PH / 2                    # the plate's upturned back face
RISE = 0.                                 # bottom face starts at the lower plate plane; CLEAR remains a real radial gap
RECESS = .0001                            # m pushed-face recess, replica design; phone shrinkage concerns the fill face
PROUD = -RECESS
TAB_H = PH - RISE + PROUD                  # modelled tablet height (3.6 mm)
TAB_Z = PLATE_Z - PH / 2 + RISE + TAB_H / 2
HOLES = np.array([((c - (NCOL - 1) / 2) * PITCH_X, (r - (NROW - 1) / 2) * PITCH_Y) for r in range(NROW) for c in range(NCOL)])
TAB_M = .000120                           # 120 mg printed-resin estimate from CAD and assumed 1.2 g/cm3 density
PLATE_M, BAR_M, PARK_M, BLOCK_M, TOOL_M = 0.160, .066, .012, .106, .055  # kg, unmeasured CAD/material estimates
# Powder spreader (white rigid plastic card; size, mass and friction are estimates from the maker's photos).
CW, CT, CH = .090, .003, .070
ROD = CT / 2                               # its lower edge is rounded (a 1.5 mm radius rod)
PIN_X, PIN_D, PIN_L = 0., .0018, .012                  # integrated polymer spreader pin (fallback design)
EDGE_Z = -CH / 2 + ROD                     # rod centre in the card frame
# Printed resin slot block holding the instrumented spreader upright.
BLK = (.100, .040, .020); SLOT_FLOOR = .006; SLOT_W = .0045
GUIDE_GAP=.0050; GUIDE_SIZE=(.030,.010,.019); GUIDE_X=.030; GUIDE_Z=.0285  # upper end guides, top38mm
HOLD = {'A': (.220, -.200), 'B': (.410, -.190)}
SPAWN_HOLD=HOLD['B'] if os.environ.get('TASK_LOT','A').upper()=='B' else HOLD['A']
HANDLE_Y, HANDLE_H, LIFT = .250, .055, .0100
LEFT_X_BIAS, LEFT_Y_BIAS = 0., -.004
LEFT_FINGER_HOLD = -.007  # firmer partial close below the stock actuator saturation target; physical grip only
PLATE_RISE = .0050  # right-arm height estimate, distinct from compliant left-hand lift command
HANDLE_CANT = math.radians(2.)
HANDLE_PITCH = math.radians(2.)  # second-axis physical cant from measured loaded plate pitch
_sx,_cx=math.sin(HANDLE_CANT/2),math.cos(HANDLE_CANT/2)
_sy,_cy=math.sin(HANDLE_PITCH/2),math.cos(HANDLE_PITCH/2)
HANDLE_Q = (_sx*_cy,_cx*_sy,-_sx*_sy,_cx*_cy)  # Ry(pitch) @ Rx(cant)
HANDLE_R = parts._quat_to_matrix(HANDLE_Q)
HANDLE_S = (.030, .030, HANDLE_H)
HANDLE_C = np.array((0., HANDLE_Y, PH / 2 + HANDLE_H / 2))
NECK_S=np.array((.012,.012,.0035)); NECK_C=np.array((0.,HANDLE_Y,PH/2+.05575))
CAP_S=np.array((.012,.044,.003)); CAP_C=np.array((.002,HANDLE_Y,PH/2+.0588))
FOOT_S = np.array((.030, .177, .010)); FOOT_C = np.array((0., .1645, (.010-PH)/2))
PARK_S = np.array((.030, .025, BAR + PARCH_T)); PARK_Y = .220
SUPPORT_GAP = .0005; FIXTURE_MOVE = .005
LAY = (.300, -.300)                         # where the demo lays the spreader down (any free place scores)
# ---- Declared release / damage rule (stand-ins for the tablets' ring retention and protocol force limit).
# Technician phone figures supplied Nov 2026: .1 mm recess, .4 N release, 4 N crush, .04 mm radial gap.
# They describe dried pharmaceutical tablets; pushed-face placement is a replica choice, not a phone observation.
PIN_CONTACT_TOL = .0004  # two 0.2 mm contact offsets; classifies existing force contacts, never creates one
F_REL = .40     # N: own pin force, technician phone figure for dried tablets (Nov 2026)
# Retention is a native plate-to-tablet bond: its equal/opposite constraint forces act on both bodies.
# It resists downward travel with a capped implicit drive; disables only at own contact >= F_REL after STROKE.
# The temporary guide represents ring engagement inside the bore; released bodies have no joint or guide.
STROKE = .0022   # m: actual downward travel through proposed ring engagement before free flight
RET_MAX = .45   # N, resistance cap above the supplied 0.4 N real push-out figure
RET_K = RET_MAX / STROKE  # N/m: rises to the same cap over the physical stroke, not an effectively rigid latch
RET_C = 2 * math.sqrt(RET_K * TAB_M)  # N s/m: critical damping from puck mass, not a measured ring coefficient
F_TAB = 4.0     # N: per-tablet crush, technician phone figure (Nov 2026); replica separately qualified
F_HI = 6.0      # N: protocol aggregate plate-load limit; not a measured crush force
TOUCH_N = .02   # N: no gripper contact on a tablet is allowed (no-touch handling: hard fingertips, not a finger)
KNOCK = .010; KNOCK_DEG = 5.  # plate shifted 10 mm or tilted 5 degrees on its bars: knocked
BLACK = (.12, .12, .13); WHITE = (.96, .96, .93); ALU = (.72, .73, .75); WOOD = (.62, .45, .28); PAPER = (.93, .90, .82)


def octagon(a, z):
    r = a / math.cos(math.pi / SIDES)
    return [(r * math.cos(math.pi / SIDES + 2 * math.pi * i / SIDES), r * math.sin(math.pi / SIDES + 2 * math.pi * i / SIDES), z) for i in range(SIDES)]


def frustum(a_top, a_bot, h, cx=0., cy=0., cz=0.):
    top = [(x + cx, y + cy, z + cz) for x, y, z in octagon(a_top, h / 2)]
    bot = [(x + cx, y + cy, z + cz) for x, y, z in octagon(a_bot, -h / 2)]
    n = SIDES
    faces = [tuple(range(n)), tuple(range(2 * n - 1, n - 1, -1))] + [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)]
    return top + bot, faces


def ray_box(theta, hx, hy):
    c, s = math.cos(theta), math.sin(theta)
    t = min(hx / abs(c) if abs(c) > 1e-12 else 1e9, hy / abs(s) if abs(s) > 1e-12 else 1e9)
    return (t * c, t * s)


def cell_pieces(cx, cy):
    """The plate around one hole: SIDES convex pieces between the tapered hole wall and the cell's rectangle."""
    hx, hy = PITCH_X / 2, PITCH_Y / 2; out = []
    corners = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
    for i in range(SIDES):
        t0 = math.pi / SIDES + 2 * math.pi * i / SIDES; t1 = t0 + 2 * math.pi / SIDES
        ring = [ray_box(t0, hx, hy)]
        for c in corners:
            ang = math.atan2(c[1], c[0]) % (2 * math.pi)
            lo, hi = t0 % (2 * math.pi), t1 % (2 * math.pi)
            inside = (lo < ang < hi) if lo < hi else (ang > lo or ang < hi)
            if inside: ring.append(c)
        ring.append(ray_box(t1, hx, hy))
        rt, rb = A_TOP / math.cos(math.pi / SIDES), A_BOT / math.cos(math.pi / SIDES)
        pts = []
        for z, r in ((PH / 2, rt), (-PH / 2, rb)):
            pts += [(cx + r * math.cos(t0), cy + r * math.sin(t0), z), (cx + r * math.cos(t1), cy + r * math.sin(t1), z)]
            pts += [(cx + x, cy + y, z) for x, y in ring]
        m = len(pts) // 2
        faces = [tuple(range(m)), tuple(range(2 * m - 1, m - 1, -1))] + [(j, (j + 1) % m, m + (j + 1) % m, m + j) for j in range(m)]
        out.append(parts.hull(pts, faces, BLACK))
    return out


def plate_shapes():
    shapes = []
    for hx, hy in HOLES: shapes += cell_pieces(hx, hy)
    fx, fy = NCOL * PITCH_X / 2, NROW * PITCH_Y / 2
    shapes.append(parts.box((PW / 2 - fx, PL, PH), (+(PW / 2 + fx) / 2, 0, 0), BLACK))
    shapes.append(parts.box((PW / 2 - fx, PL, PH), (-(PW / 2 + fx) / 2, 0, 0), BLACK))
    shapes.append(parts.box((2 * fx, PL / 2 - fy, PH), (0, +(PL / 2 + fy) / 2, 0), BLACK))
    shapes.append(parts.box((2 * fx, PL / 2 - fy, PH), (0, -(PL / 2 + fy) / 2, 0), BLACK))
    return tuple(shapes)


# Proposed resin replica plate. Pucks have temporary native ring-retention guides, then fully free rigid motion.
NAMES = [f'tablet_{k:02d}' for k in range(N)]
PLATE = parts.Part('cavity_plate', pos=(PX, PY, PLATE_Z), mass=PLATE_M, shapes=plate_shapes() + (
                       parts.box(FOOT_S, FOOT_C, WHITE),
                       parts.box(CAP_S,CAP_C,WHITE),
                       parts.box(NECK_S,NECK_C,WHITE),
                       parts.box((.002,.026,HANDLE_H-.004), HANDLE_C, WHITE, quat=HANDLE_Q),
                       *(parts.box((.030,.002,HANDLE_H), HANDLE_C+HANDLE_R@np.array((0.,dy,0.)), WHITE, quat=HANDLE_Q) for dy in (-.014,.014)),
                       *(parts.box((.030,.026,.002), HANDLE_C+HANDLE_R@np.array((0.,0.,dz)), WHITE, quat=HANDLE_Q) for dz in (-HANDLE_H/2+.001,HANDLE_H/2-.001))),
                   friction=(.2, .16), contact_offset=.0002, report=('base',))
_tp, _tf = frustum(A_BOT - CLEAR - (A_BOT - A_TOP) * TAB_H / PH, A_BOT - CLEAR, TAB_H)
TABLETS = [parts.Part(n, pos=(PX + float(HOLES[k][0]), PY + float(HOLES[k][1]), TAB_Z), mass=TAB_M,
                      shapes=(parts.hull(_tp, _tf, WHITE),), friction=(.2, .16), contact_offset=.0002, report=('base',))
           for k, n in enumerate(NAMES)]
_BONDS = {}
_author = parts._author
def _author_bonds(spec, path, translation, orientation):
    root = _author(spec, path, translation, orientation)
    from pxr import UsdPhysics, PhysxSchema, Sdf, Gf
    from isaaclab.sim.utils import get_current_stage
    stage = get_current_stage()
    if spec.mass is not None and not spec.joints:
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(path))
        api.CreateSolverPositionIterationCountAttr(128)
        api.CreateSolverVelocityIterationCountAttr(128)
    if spec.name in NAMES:
        k = NAMES.index(spec.name)
        joint = UsdPhysics.PrismaticJoint.Define(stage, path + '/retention')
        joint.CreateAxisAttr('Z')
        joint.CreateLowerLimitAttr(-.006)
        joint.CreateUpperLimitAttr(0.)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'linear')
        drive.CreateTypeAttr('force')
        drive.CreateTargetPositionAttr(0.)
        drive.CreateTargetVelocityAttr(0.)
        drive.CreateStiffnessAttr(RET_K)
        drive.CreateDampingAttr(RET_C)
        drive.CreateMaxForceAttr(RET_MAX)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(parts._PATHS['cavity_plate'])])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(float(HOLES[k][0]), float(HOLES[k][1]), TAB_Z - PLATE_Z))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0., 0., 0.))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.))
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateCollisionEnabledAttr(False)
        _BONDS[k] = path + '/retention'
    return root
parts._author = _author_bonds
BARS = [parts.Part(f'bar_{s}', pos=(PX + sg * BAR_DX, PY, PT + BAR / 2), mass=BAR_M, shapes=(parts.box((BAR, BAR_L, BAR), color=ALU),),
                   friction=(.6, .5), contact_offset=.0002) for s, sg in (('far', 1), ('near', -1))]
PARK = parts.Part('handle_support', pos=(PX, PY+PARK_Y, TOP+float(PARK_S[2])/2), mass=PARK_M, shapes=(parts.box(PARK_S, color=WHITE),), friction=(.6,.5), contact_offset=.0002)
PARCHMENT = parts.Part('parchment', pos=(PX, PY, TOP + PARCH_T / 2), shapes=(parts.box((.26, .30, PARCH_T), color=PAPER),), friction=(.5, .4))
_bx, _by, _bz = BLK; _ww = (_by - SLOT_W) / 2
BLOCK = parts.Part('spreader_block', pos=(*SPAWN_HOLD, TOP), mass=BLOCK_M, friction=(.5, .4), contact_offset=.0002, shapes=(
    parts.box((_bx, _by, SLOT_FLOOR), (0, 0, SLOT_FLOOR / 2), WOOD),
    *(parts.box((.044,_ww,_bz-SLOT_FLOOR),(sx*.028,sy*(SLOT_W+_ww)/2,(SLOT_FLOOR+_bz)/2),WOOD)
      for sx in (-1,1) for sy in (-1,1)),
    *(parts.box((.012,.01475,_bz-SLOT_FLOOR),(0,sy*.012625,(SLOT_FLOOR+_bz)/2),WOOD) for sy in (-1,1)),
    parts.box((.004, SLOT_W, _bz - SLOT_FLOOR), (+(_bx / 2 - .002), 0, (SLOT_FLOOR + _bz) / 2), WOOD),
    parts.box((.004, SLOT_W, _bz - SLOT_FLOOR), (-(_bx / 2 - .002), 0, (SLOT_FLOOR + _bz) / 2), WOOD),
    parts.box((.038,SLOT_W,PIN_L),(-.025,0.,SLOT_FLOOR+PIN_L/2),WOOD),
    parts.box((.038,SLOT_W,PIN_L),(.025,0.,SLOT_FLOOR+PIN_L/2),WOOD),
    *(parts.box(GUIDE_SIZE,(GUIDE_X,sy*(GUIDE_GAP/2+GUIDE_SIZE[1]/2),GUIDE_Z),WOOD) for sy in (-1,1))))

CARD_Z = TOP + SLOT_FLOOR + PIN_L + CH / 2
SPREADER = parts.Part('spreader', pos=(*SPAWN_HOLD, CARD_Z), mass=TOOL_M, friction=(.4, .35), contact_offset=.0002, shapes=(
    parts.box((CW, CT, CH - ROD), (0, 0, (CH / 2 + EDGE_Z) / 2), WHITE),
    parts.prism(ROD, CW, 16, (0, 0, EDGE_Z), WHITE),
    parts.cylinder(PIN_D / 2, PIN_L, (PIN_X, 0, -CH / 2 - PIN_L / 2), WHITE),
    parts.box((.020, .012, .025), (0., 0., .0225), WHITE),
    parts.box((.058, .016, .019), (-.020, 0., -.019), WHITE),
    parts.box((.012,.014,.025),(-.040,0.,.0005),WHITE),
    parts.cylinder(.00436,.005,(0.,0.,-.03185),ALU)))
OBJECTS = [PARCHMENT, *BARS, PARK, PLATE, *TABLETS, BLOCK, SPREADER]
LOTS = lots.declare(OBJECTS,
                    lots.Lot('A', place={'spreader_block': (*HOLD['A'], TOP), 'spreader': (*HOLD['A'], CARD_Z)}),
                    lots.Lot('B', place={'spreader_block': (*HOLD['B'], TOP), 'spreader': (*HOLD['B'], CARD_Z)}))
def _initialize_lot(emb):
    """Use the SDK whole-scene reset API once at trial setup, then settle normally."""
    import torch
    base=emb._base; lot=lots.current()
    state=base.scene.get_state(is_relative=True)
    for name in ('spreader','spreader_block'):
        asset=state['rigid_object'][name]
        pose=asset['root_pose']
        pose[:,:3]=torch.tensor(lot.pos(name),dtype=pose.dtype,device=pose.device)
        pose[:,2]+=.010  # reset-only clearance: let the assembly settle onto its declared counter location
        pose[:,3:]=torch.tensor([0.,0.,0.,1.],dtype=pose.dtype,device=pose.device)
        asset['root_velocity'].zero_()
    base.reset_to(state,env_ids=None,is_relative=True)
    for _ in range(max(1,round(lots.SETTLE_S/base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(lots.RENDERS): base.sim.render()
    o=parts.Sim(emb).states()
    S['lot_setup']=dict(lot=lot.name,parts={n:dict(expected=lot.pos(n),actual=list(map(float,o[n].pos))) for n in ('spreader','spreader_block')})
    return True
lots._apply=_initialize_lot

DEMO_TOUCHES = set()


# ---- Record of the trial (module state; reset on each scene reset).
def fresh():
    return dict(held=np.ones(N, bool), freed=np.zeros(N, bool), zero=False, why='', known=False, phys=0, steps=0, t=None, still=None, clear=None,
                prev=None, views=None, peak=0., press=0., plate_press=0., hand_tab=0., events=[], released_at={}, release_N={},
                tab_peak=np.zeros(N), tab_peak_held=np.zeros(N), bodies=None, native_poses=None, travel=np.zeros(N), release_m={}, release_phys={}, release_gap={}, support_gap=None, release_v={}, flight_samples=[], flight_watch={})
S = fresh()
def fail(reason):
    if not S['zero']: S.update(zero=True, why=str(reason)); event('ZERO: ' + str(reason))
def event(text):
    if len(S['events']) < 200: S['events'].append(text)


def edge_in_plate(card_pos, card_rot, plate_pos, plate_rot):
    """The spreader's rounded lower edge (centre line ends) in the plate's frame."""
    ends = np.array([(-CW / 2, 0., EDGE_Z), (CW / 2, 0., EDGE_Z)])
    return (np.asarray(card_pos) + ends @ np.asarray(card_rot).T - np.asarray(plate_pos)) @ np.asarray(plate_rot)


def rule_step(push, hand_tab, total, t=0., travel=None, support_gap=None, all_push=None, nonpin_force=0.):
    """Declared release / damage rule for one physics step. push: array of N, the force (N) with which the spreader
    itself presses on each tablet's face, along the plate's normal (PhysX contact force between the spreader and that
    tablet; contact with the plate or a neighbour adds nothing); hand_tab: largest gripper force on any tablet (N);
    total: aggregate external normal compression on plate/tablets from the spreader and both grippers.
    A scale under the parchment cannot resolve this while the plate is held. Returns the tablets released this step (their bond is then removed and they
    leave the hole under the push and gravity)."""
    push = np.asarray(push, dtype=np.float64)
    all_push = push if all_push is None else np.asarray(all_push,dtype=np.float64)
    if all_push.shape != (N,) or not np.isfinite(all_push).all() or not np.isfinite(nonpin_force): raise ValueError('spreader contact state unknown')
    travel = np.asarray(travel, dtype=np.float64)
    if travel.shape != (N,) or not np.isfinite(travel).all(): raise ValueError("tablet travel unknown")
    S["travel"] = travel.copy()
    if support_gap is None or not np.isfinite(support_gap): raise ValueError("support gap unknown")
    S["support_gap"] = float(support_gap)
    if push.shape != (N,) or not np.isfinite(push).all() or not all(np.isfinite([hand_tab, total])):
        raise ValueError('contact state unknown')
    S['peak'] = max(S['peak'], float(total)); S['hand_tab'] = max(S['hand_tab'], float(hand_tab))
    S['tab_peak'] = np.maximum(S['tab_peak'], all_push)
    S['tab_peak_held'] = np.where(S['held'], np.maximum(S['tab_peak_held'], all_push), S['tab_peak_held'])
    if total > F_HI + EPS: fail(f'{total:.2f} N aggregate compression exceeds the {F_HI} N protocol plate-load limit')
    if all_push.max() > F_TAB + EPS: fail(f'{all_push.max():.2f} N on tablet {int(all_push.argmax())} (over {F_TAB} N): crushed')
    if nonpin_force > TOUCH_N + EPS: fail(f'non-pin spreader contact with workpiece ({nonpin_force:.3f} N)')
    if hand_tab > TOUCH_N + EPS: fail(f'a gripper touched a tablet ({hand_tab:.3f} N)')
    if S['zero']: return np.zeros(N, bool)
    out = (push >= F_REL - EPS) & (travel >= STROKE - EPS) & S['held']
    if out.any() and support_gap < SUPPORT_GAP - EPS:
        fail('loaded push while plate still on parking supports'); return np.zeros(N, bool)
    for k in np.flatnonzero(out):
        S['freed'][k] = True; S['held'][k] = False
        S['released_at'][int(k)] = round(float(t), 4); S['release_N'][int(k)] = float(push[k])
        S['release_m'][int(k)] = float(travel[k])
        S['release_phys'][int(k)] = S['phys']
        S['release_gap'][int(k)] = float(support_gap)
        event(f't={t:.3f} tablet {int(k)} released at {push[k]:.3f} N on its face after {1000*travel[k]:.3f} mm')
    return out


# ---- Scoring helpers.
def tilt_deg(q):
    return math.degrees(math.acos(max(-1., min(1., float(parts._quat_to_matrix(q)[2, 2])))))
def tablet_pos(o, k):
    return np.asarray(o[NAMES[k]].pos, dtype=np.float64)
def tablet_box(o, k):
    st=o[NAMES[k]]
    vertices=np.asarray(_tp)@parts._quat_to_matrix(st.quat_xyzw).T+np.asarray(st.pos)
    if not np.isfinite(vertices).all(): raise ValueError('tablet geometry state unknown')
    return vertices.min(axis=0),vertices.max(axis=0)
def on_parchment(o, k, v=0.):
    """Tablet k out of its hole lying flat on the parchment (lowest point within 1 mm of it, top at most 5 mm up),
    on the sheet, moving under 10 mm/s."""
    lo, hi = tablet_box(o, k); c = tablet_pos(o, k)
    px, py = PARCHMENT.pos[0], PARCHMENT.pos[1]
    return bool(PT - .001 <= lo[2] <= PT + .001 and hi[2] <= PT + .005 + EPS
                and abs(c[0] - px) <= .13 and abs(c[1] - py) <= .15 and v <= .01 + EPS)
def count(o):
    v = S.get('tab_v', np.zeros(N))
    return int(sum(1 for k in range(N) if S['freed'][k] and not S['held'][k] and on_parchment(o, k, float(v[k]))))
def knocked(st):
    return bool(np.linalg.norm(np.asarray(st.pos) - np.asarray(PLATE.pos)) > KNOCK + EPS or tilt_deg(st.quat_xyzw) > KNOCK_DEG + EPS)


def evaluate(o):
    if not S['known'] or S['zero'] or S['steps'] == 0 or S['phys'] == 0 or S['t'] is None: return 0.
    if knocked(o['cavity_plate']): return 0.
    clear1 = S['clear'] is not None and S['t'] - S['clear'] >= 1. - EPS
    still1 = S['still'] is not None and S['t'] - S['still'] >= 1. - EPS
    if not clear1: return 0.
    n = count(o)
    if n == N and still1: return 1.
    if n >= 45: return .75
    if n >= 30: return .5
    if n >= 15: return .25
    return 0.
def score(o):
    try: return float(evaluate(o))
    except (KeyError, ValueError, TypeError, AttributeError, IndexError): return 0.
def success(o): return score(o) == 1.
def distance(o): return float(N - count(o)) if S['known'] else float(N)


# ---- Hands: sparse face-sampled hulls of the gripper links (large faces included, not only vertices).
_HAND = {}
def local_hand(n):
    if n not in _HAND:
        from yam_arena import analysis
        chunks = []
        for h in analysis.link_hulls()[n]:
            chunks.append(h.vertices)
            _, ids = np.unique(np.round(h.planes, 6), axis=0, return_index=True)
            for normal, dd in zip(h.planes[ids, :3], h.planes[ids, 3]):
                face = h.vertices[np.abs(h.vertices @ normal - dd) < 1e-6]
                if len(face) < 3: continue
                c = face.mean(0)
                for p in face:
                    for f in np.linspace(0, 1, 9)[1:-1]: chunks.append((c + (p - c) * f)[None, :])
                for p, q in zip(face, np.roll(face, -1, axis=0)):
                    for f in np.linspace(0, 1, 9)[1:-1]: chunks.append((p + (q - p) * f)[None, :])
        pts = np.concatenate(chunks)
        _, idx = np.unique(np.round(pts / .002).astype(np.int64), axis=0, return_index=True)
        _HAND[n] = pts[np.sort(idx)]
    return _HAND[n]
def hands(sim):
    base = sim.env.unwrapped; org = parts._np(base.scene.env_origins[0]); out = []
    for arm in ('left_robot', 'right_robot'):
        a = base.scene[arm]; p = parts._np(a.data.body_link_pose_w)[0]
        for n in ('link5', 'link6', 'gripper', 'tip_left', 'tip_right'):
            if n not in a.body_names: continue
            st = p[list(a.body_names).index(n)]
            out.append(local_hand(n) @ parts._quat_to_matrix(st[3:]).T + st[:3] - org)
    out = np.concatenate(out)
    if not np.isfinite(out).all(): raise ValueError('unknown arm state')
    return out


def box_gap(points, c, r, size): return np.linalg.norm(np.maximum(np.abs((points - np.asarray(c)) @ r) - np.asarray(size) / 2, 0), axis=1)
WORK_C = np.array((PX, PY, (PT + BACK) / 2)); WORK_S = np.array((PW, PL, BACK - PT))
def hands_clear(h, o):
    """No hand point within 30 mm of the work (plate, bars, tablets, parchment under the plate) or the spreader."""
    pr = parts._quat_to_matrix(o['cavity_plate'].quat_xyzw)
    if box_gap(h, WORK_C, np.eye(3), WORK_S + 2 * .002).min() < .030 - EPS: return False
    if box_gap(h, o['cavity_plate'].pos, pr, np.array((PW, PL, PH))).min() < .030 - EPS: return False
    if box_gap(h, np.asarray(o['cavity_plate'].pos) + pr @ HANDLE_C, pr @ HANDLE_R, HANDLE_S).min() < .030 - EPS: return False
    if box_gap(h, np.asarray(o['cavity_plate'].pos) + pr @ FOOT_C, pr, FOOT_S).min() < .030 - EPS: return False
    for c,size in ((CAP_C,CAP_S),(NECK_C,NECK_S)):
        if box_gap(h,np.asarray(o['cavity_plate'].pos)+pr@c,pr,size).min()<.030-EPS:return False
    park = o['handle_support']
    if box_gap(h, park.pos, parts._quat_to_matrix(park.quat_xyzw), PARK_S).min() < .030 - EPS: return False
    sp = o['spreader']; sr = parts._quat_to_matrix(sp.quat_xyzw)
    for c, size in (((0.,0.,0.),(CW,CT,CH)), ((PIN_X,0.,-CH/2-PIN_L/2),(PIN_D,PIN_D,PIN_L)),
                    ((0.,0.,.0225),(.020,.012,.025)), ((-.020,0.,-.019),(.058,.016,.019)),
                    ((-.040,0.,.0005),(.012,.014,.025)), ((0.,0.,-.03185),(.00872,.00872,.005))):
        if box_gap(h, np.asarray(sp.pos) + sr @ np.asarray(c), sr, size).min() < .030 - EPS: return False
    return True


def observe(o, h, t):
    """Whole-trial checks at each control step; pure, so offline checks can drive it."""
    if h is None or len(h) == 0 or not np.isfinite(h).all():
        S['known'] = False; fail('arm state unknown'); return
    tabs = np.array([tablet_pos(o, k) for k in range(N)])
    for n in ['cavity_plate', 'spreader', *NAMES]:
        st = o[n]
        if not np.isfinite(st.pos).all() or not np.isfinite(st.quat_xyzw).all():
            S['known'] = False; fail('object state unknown'); return
    if not np.isfinite(tabs).all(): S['known'] = False; fail('object state unknown'); return
    for k,n in enumerate(NAMES):
        if tablet_box(o,k)[0][2] < TOP - .001: fail(n + ' fell off the counter')
    for spec in [*BARS, PARK, BLOCK]:
        if np.linalg.norm(np.asarray(o[spec.name].pos) - np.asarray(lots.current().pos(spec.name))) > FIXTURE_MOVE + EPS:
            fail('parking fixture moved over 5 mm: ' + spec.name)
    if knocked(o['cavity_plate']):
        st=o['cavity_plate'];dist=float(np.linalg.norm(np.asarray(st.pos)-np.asarray(PLATE.pos)))
        fail(f'plate pose limit: moved {dist*1000:.3f} mm (cap10), tilt {tilt_deg(st.quat_xyzw):.3f} deg (cap5)')
    if S.get('tab_prev') is not None and S['t'] is not None and t > S['t']:
        S['tab_v'] = np.linalg.norm(tabs - S['tab_prev'], axis=1) / (t - S['t'])
    S['tab_prev'] = tabs
    snap = np.concatenate([np.array([o['cavity_plate'].pos, o['spreader'].pos]), tabs])
    if S['prev'] is None or np.max(np.linalg.norm(snap - S['prev'], axis=1)) > .001:
        S['still'] = float(t); S['prev'] = snap.copy()
    S['clear'] = (S['clear'] if S['clear'] is not None else float(t)) if hands_clear(h, o) else None
    S.update(steps=S['steps'] + 1, t=float(t), known=True)


# ---- Simulator hooks.
def _pose(base, name):
    org = parts._np(base.scene.env_origins[0]); p = parts._np(base.scene[name].data.root_link_pose_w)[0]
    if not np.isfinite(p).all(): raise ValueError(name + ' pose unknown')
    return p[:3] - org, parts._quat_to_matrix(p[3:])


def make_views(sim):
    from isaaclab_physx.physics import PhysxManager
    from yam_arena.robot import asset_link_paths
    env = sim.env.unwrapped.scene.env_prim_paths[0]; lp = asset_link_paths()
    hand = [env + '/' + a + lp[n] for a in ('LeftArm', 'RightArm') for n in ('link5', 'link6', 'gripper', 'tip_left', 'tip_right') if n in lp]
    filters = [parts._PATHS['spreader'], *hand]
    sensors = [parts._PATHS['cavity_plate'], *[parts._PATHS[n] for n in NAMES]]
    v = PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors, filter_patterns=[filters for _ in sensors],
                                                                       max_contact_data_count=2048)
    return v, len(filters)


def arr(x):
    x = x.torch if hasattr(x, 'torch') else x
    if hasattr(x, 'detach'): return x.detach().cpu().numpy().astype(np.float64)
    if hasattr(x, 'numpy'): return np.asarray(x.numpy(), dtype=np.float64)
    return np.asarray(x, dtype=np.float64)


def contact_loads(sim, n):
    """Per-tablet push of the spreader, the total downward push, the largest gripper force on a tablet, and the
    spreader's signed force on the plate (for the log). n: the plate's back-face normal (world). The contact view reports
    the force on each sensor body (plate, tablets) from each filter body (spreader, gripper links); a push is its
    compressive part along -n, summed per pair, never an absolute value of a sum."""
    v, nf = S['views']
    fs, points, normals, separations, counts, starts = v.get_contact_data(dt=sim.env.unwrapped.physics_dt)
    fs = arr(fs).reshape(-1); normals = arr(normals).reshape(-1, 3); points=arr(points).reshape(-1,3)
    poses=S['native_poses']
    if poses is None: raise ValueError('spreader pose unavailable for contact classification')
    tool_p=poses[N+4,:3]; tool_r=parts._quat_to_matrix(poses[N+4,3:])
    org=arr(sim.env.unwrapped.scene.env_origins[0])
    pin_down=np.zeros(N+1); nonpin_force=0.
    S['plate_hand_points']=[]
    counts = arr(counts).reshape(N + 1, nf); starts = arr(starts).reshape(N + 1, nf)
    if not np.isfinite(np.r_[counts.ravel(), starts.ravel()]).all(): raise ValueError('unknown contact indices')
    down = np.zeros((N + 1, nf)); magnitude = np.zeros_like(down); signed = np.zeros_like(down)
    for body in range(N + 1):
        for tool in range(nf):
            start, count = int(starts[body, tool]), int(counts[body, tool])
            if count == 0: continue
            if count < 0 or start < 0 or start + count > len(fs): raise ValueError('contact buffer overflow')
            force = fs[start:start + count, None] * normals[start:start + count]
            if not np.isfinite(force).all(): raise ValueError('non-finite actual contact forces')
            axial = force @ np.asarray(n, dtype=np.float64)
            down[body, tool] = np.maximum(-axial, 0.).sum()
            signed[body, tool] = axial.sum()
            magnitude[body, tool] = np.linalg.norm(force, axis=1).sum()
            if body == 0 and tool > 0 and magnitude[body,tool] > TOUCH_N:
                hp=(points[start:start+count]-org-poses[0,:3])@parts._quat_to_matrix(poses[0,3:])
                S['plate_hand_points'].append(dict(filter_index=tool-1, local_mm=np.round(hp*1000,3).tolist(), force_w_N=np.round(force,4).tolist()))
            if tool == 0:
                used=points[start:start+count]
                if not np.isfinite(used).all(): raise ValueError('contact points unknown')
                local=(used-org-tool_p)@tool_r
                onpin=(np.linalg.norm(local[:,:2]-np.array((PIN_X,0.)),axis=1)<=PIN_D/2+PIN_CONTACT_TOL)
                onpin &= (local[:,2]>=-CH/2-PIN_L-PIN_CONTACT_TOL)&(local[:,2]<=-CH/2+PIN_CONTACT_TOL)
                pin_down[body]=np.maximum(-axial[onpin],0.).sum()
                bad_force=float(np.linalg.norm(force[~onpin],axis=1).sum())
                nonpin_force=max(nonpin_force,bad_force)
                if bad_force > TOUCH_N:
                    S.setdefault('nonpin_samples',[]).append(dict(phys=S['phys'],body=body,local=local[~onpin].tolist(),force=force[~onpin].tolist(),separation=arr(separations).reshape(-1)[start:start+count][~onpin].tolist(),tool_pose=poses[N+4].tolist()))
                    S['nonpin_samples']=S['nonpin_samples'][-12:]
    S['plate_hand_by_link']=magnitude[0,1:].tolist()
    S['plate_hand_force_magnitude']=float(magnitude[0,1:].sum())
    return pin_down[1:], float(down.sum()), float(magnitude[1:, 1:].sum(axis=1).max()), float(signed[0, 0]), down[1:,0], nonpin_force


def set_bond(sim, ks, on):
    """Enable retention at reset, or disable it immediately on that tablet's own contact force."""
    from isaaclab.sim.utils import get_current_stage
    from pxr import UsdPhysics
    stage = get_current_stage()
    for k in ks:
        joint = UsdPhysics.Joint(stage.GetPrimAtPath(_BONDS[int(k)]))
        if not joint: raise ValueError('missing tablet retention joint')
        joint.CreateCollisionEnabledAttr(not bool(on))  # retained bore guide replaces redundant wall contacts; all contacts restore at release
        joint.CreateJointEnabledAttr(bool(on))

def tablet_travel(sim):
    """One native pose transfer per substep for plate, pucks and parking supports."""
    from isaaclab_physx.physics import PhysxManager
    base = sim.env.unwrapped
    names = ['cavity_plate', *NAMES, *[b.name for b in BARS], 'handle_support', 'spreader']
    if S['bodies'] is None:
        paths = [parts._PATHS[n] for n in names]
        view = PhysxManager.get_physics_sim_view().create_rigid_body_view(paths)
        found = list(view.prim_paths)
        if len(found) != N+5 or len(set(found)) != N+5: raise ValueError('body view incomplete')
        S['bodies'] = (view, np.array([found.index(p) for p in paths]))
        try:
            ids=S['bodies'][1]
            masses=arr(view.get_masses()).reshape(-1)[ids]
            inertias=arr(view.get_inertias()).reshape(N+5,3,3)[ids]
            S['native_mass_properties']=dict(plate_kg=float(masses[0]),puck_kg=float(masses[1]),tool_kg=float(masses[N+4]),puck_inertia_backend=inertias[1].tolist())
        except Exception as e: S['mass_diagnostic_error']=str(e)
    view, ids = S['bodies']
    poses = arr(view.get_transforms()).reshape(N+5,7)[ids].copy()
    poses[:, :3] -= parts._np(base.scene.env_origins[0])
    if not np.isfinite(poses).all(): raise ValueError('native body poses unknown')
    S['native_poses'] = poses
    pp, pr = poses[0,:3], parts._quat_to_matrix(poses[0,3:])
    rel = (poses[1:N+1,:3] - pp) @ pr
    return (TAB_Z - PLATE_Z) - rel[:,2]


def support_gap(sim):
    poses = S['native_poses']
    if poses is None: raise ValueError('support poses unavailable')
    pp, pr = poses[0,:3], parts._quat_to_matrix(poses[0,3:])
    corners = np.array([(x,y,-PH/2) for x in (-PW/2,PW/2) for y in (-PL/2,PL/2)]) @ pr.T + pp
    tops=[]
    for i in (N+1,N+2):
        br=parts._quat_to_matrix(poses[i,3:])
        tops.append(poses[i,2] + (np.abs(br[2]) @ np.array((BAR,BAR_L,BAR)))/2)
    foot=pp+pr@FOOT_C
    foot_low=foot[2]-(np.abs(pr[2])@FOOT_S)/2
    hp,hr=poses[N+3,:3],parts._quat_to_matrix(poses[N+3,3:])
    park_top=hp[2]+(np.abs(hr[2])@PARK_S)/2
    return min(float(corners[:,2].min()-max(tops)),float(foot_low-park_top))


def plate_normal(base):
    return _pose(base, 'cavity_plate')[1][:, 2]

@parts.before_reset
def before(sim):
    set_bond(sim, range(N), True)

@parts.on_reset
def reset(sim):
    """Reset is the only place tablet poses are written; no live carry or pose correction."""
    global S
    S = fresh()
    from isaaclab.app.settings_manager import get_settings_manager
    get_settings_manager().set_bool('/physics/disableContactProcessing', False)
    import torch
    base = sim.env.unwrapped; org = base.scene.env_origins[0:1]
    try:
        from isaaclab.sim.utils import get_current_stage
        scene_prim=get_current_stage().GetPrimAtPath(base.sim.cfg.physics_prim_path)
        S['solver_type']=str(scene_prim.GetAttribute('physxScene:solverType').Get())
    except Exception as e: S['solver_diagnostic_error']=str(e)
    set_bond(sim, range(N), True)
    for n in NAMES:
        obj = base.scene[n]
        pose = obj.data.default_root_pose
        pose = (pose.torch if hasattr(pose, 'torch') else pose).clone()
        pose[:, :3] += org
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=obj.device))


@parts.on_physics_step
def physics(sim):
    if S.get('dead'): return
    try:
        base = sim.env.unwrapped
        if S['views'] is None: S['views'] = make_views(sim)
        travel = tablet_travel(sim)
        n = parts._quat_to_matrix(S['native_poses'][0,3:])[:,2]
        push, total, hand_tab, plate_f, all_push, nonpin_force = contact_loads(sim, n)
        S['press'] = float(push.sum()); S['plate_press'] = plate_f; S['push_now'] = push; S['nonpin_force'] = nonpin_force
        out = rule_step(push, hand_tab, total, float(sim.t), travel, support_gap(sim), all_push=all_push, nonpin_force=nonpin_force)
        if out.any():
            set_bond(sim, np.flatnonzero(out), False)
            for k in np.flatnonzero(out): S['flight_watch'][int(k)] = 12
        if S['flight_watch']:
            try:
                view, ids = S['bodies']; vv = arr(view.get_velocities()).reshape(N+5,6)[ids]
                for k in list(S['flight_watch']):
                    if out[k]: S['release_v'][str(k)] = vv[k+1].tolist()
                    S['flight_samples'].append(dict(phys=S['phys'], k=k, v=vv[k+1].tolist(), plate_v=vv[0].tolist(), pos=S['native_poses'][k+1,:3].tolist()))
                    S['flight_watch'][k] -= 1
                    if not S['flight_watch'][k]: del S['flight_watch'][k]
                S['flight_samples'] = S['flight_samples'][-144:]
            except Exception as e: S['velocity_diagnostic_error'] = str(e)

        S['phys'] += 1
    except Exception as e:
        S.update(known=False, dead=True); fail('contact state unavailable: ' + str(e))


@parts.on_step
def step(sim):
    if S.get('last_t') == sim.t: return
    S['last_t'] = sim.t
    try:
        o = sim.states(); h = hands(sim)
        observe(o, h, float(sim.t))
        log = os.environ.get('TABLET_LOG') or (str(pathlib.Path(os.environ['TRACE_OUT']).with_name('tablets.jsonl')) if os.environ.get('TRACE_OUT') else None)
        if log and S['steps'] % 5 == 0:
            base = sim.env.unwrapped; org = parts._np(base.scene.env_origins[0])
            cp, cr = _pose(base, 'spreader'); pp, pr = _pose(base, 'cavity_plate'); e = edge_in_plate(cp, cr, pp, pr)
            row = dict(lot_setup=S.get('lot_setup'),fixture_positions={x.name:list(map(float,o[x.name].pos)) for x in (*BARS,PARK,BLOCK)},plate_hand_points=S.get('plate_hand_points',[]),solver_type=S.get('solver_type'),solver_diagnostic_error=S.get('solver_diagnostic_error'),control=os.environ.get('TABLET_NEGATIVE_CONTROL', 'demo'), source_sha256=SOURCE_SHA256, physics_dt=float(base.physics_dt), t=round(float(sim.t), 3), lot=lots.current().name, phys=S['phys'], score=score(o), support_gap=S['support_gap'], held=int(S['held'].sum()),
                       freed=int(S['freed'].sum()),
                       on_parchment=count(o), zero=S['zero'], why=S['why'], press=round(S['press'], 3), peak=round(S['peak'], 3),
                       plate_hand_by_link=np.round(S.get('plate_hand_by_link',[]),4).tolist(), plate_hand_force_magnitude=round(S.get('plate_hand_force_magnitude',0.),3), nonpin_force=round(S.get('nonpin_force',0.),4), plate_f=round(S['plate_press'], 3), push_max=round(float(np.max(S.get('push_now', np.zeros(N)))), 3),
                       tab_peak_held_max=round(float(S['tab_peak_held'].max()), 3),
                       hand_tab=round(S['hand_tab'], 4), clear=S['clear'] is not None,
                       still=S['still'], plate=np.round(o['cavity_plate'].pos, 4).tolist(), plate_tilt=round(tilt_deg(o['cavity_plate'].quat_xyzw), 2),
                       plate_quat=np.round(o['cavity_plate'].quat_xyzw,6).tolist(), tool_quat=np.round(o['spreader'].quat_xyzw,6).tolist(), card=np.round(o['spreader'].pos, 4).tolist(), pin=np.round(cp + cr @ np.array([PIN_X, 0., -CH / 2 - PIN_L]), 5).tolist(), card_tilt=round(tilt_deg(o['spreader'].quat_xyzw), 1),
                       edge=np.round(e, 4).tolist(), bars=[np.round(o[b.name].pos, 4).tolist() for b in BARS],
                       tab_z=[round(float(tablet_pos(o, k)[2] - TAB_Z) * 1000, 3) for k in range(12)], events=S['events'][S.get('logged_ev', 0):],
                       stray=[dict(k=k, pos=np.round(tablet_pos(o, k), 4).tolist(), low=round(1000 * (tablet_box(o,k)[0][2] - PT), 3), proxy_low=round(1000*(o[NAMES[k]].low[2]-PT),3),
                                   v=round(float(S.get('tab_v', np.zeros(N))[k]), 4))
                              for k in range(N) if S['freed'][k] and not on_parchment(o, k, float(S.get('tab_v', np.zeros(N))[k]))][:12])
            if os.environ.get('TABLET_DEBUG'):
                row['dbg'] = dict(nonpin_samples=S.get('nonpin_samples',[]),native_mass_properties=S.get('native_mass_properties'),mass_diagnostic_error=S.get('mass_diagnostic_error'), release_velocity=S.get('release_v',{}), flight_samples=S.get('flight_samples',[]), velocity_diagnostic_error=S.get('velocity_diagnostic_error'), travel_mm=np.round(1000 * S['travel'], 4).tolist(), edge_um=[round(float((x[2] - PH / 2 - ROD) * 1e6), 1) for x in e],
                                  dz_um=[round(1e6 * (o[n].pos[2] - TAB_Z), 1) for n in NAMES],
                                  push=np.round(S.get('push_now', np.zeros(N)), 3).tolist())
            S['logged_ev'] = len(S['events'])
            if S['freed'].any() or S['zero']:
                row['release'] = dict(t=S['released_at'], N=S['release_N'], m=S['release_m'], physics_step=S['release_phys'], support_gap_m=S['release_gap'], peak_held=np.round(S['tab_peak_held'], 3).tolist(),
                                      peak=np.round(S['tab_peak'], 3).tolist())
            row['grippers'] = {}
            for arm in ('left_robot', 'right_robot'):
                ar = base.scene[arm]; vals = parts._np(ar.data.body_link_pose_w)[0]
                row['grippers'][arm] = {n: np.round(vals[list(ar.body_names).index(n), :3] - org, 4).tolist() for n in ('gripper', 'tip_left', 'tip_right')}
                row.setdefault('arm_body_poses',{})[arm]={n:np.r_[vals[list(ar.body_names).index(n),:3]-org,vals[list(ar.body_names).index(n),3:]].tolist() for n in ('link3','link4','link5','link6') if n in ar.body_names}
                row.setdefault('gripper_quats',{})[arm]=np.round(vals[list(ar.body_names).index('gripper'),3:],6).tolist()
                row.setdefault('tip_quats',{})[arm]={n:np.round(vals[list(ar.body_names).index(n),3:],6).tolist() for n in ('tip_left','tip_right')}
            with open(log, 'a') as f: f.write(json.dumps(row) + '\n')
    except Exception as e:
        S.update(known=False); fail('state unavailable: ' + str(e))


# ---- Open-loop demo: grasp and support the plate, then push each recessed face with the pin, park both parts.
GRIP_BELOW_TOP = .015                       # TCP this far below the spreader's top edge
CARD_BELOW_TCP = CH + PIN_L - GRIP_BELOW_TOP        # spreader's lower edge below the TCP
AIM_X, AIM_Y = -.0015, .001                 # fixed motion calibration from native development traces, no state feedback
TILT_X, TILT_Y = -.012, -.009               # visible plate-plane slope estimate from trace, radians approx
DEPTH = .0065                                # commanded below first contact (the arm's compliance sets the force)
Y0 = .071                                   # sweep from the far end margin to the near one (mid-margin, 71 mm out)


def demo():
    lot = lots.current(); hx, hy = lot.pos('spreader')[:2]
    def rest(arm): return (.300, .300 if arm == 'left' else -.300, TOP + .18)
    def w(d, p, f, label): return Waypoint(float(d), tuple(float(v) for v in p), float(f), label, 0.)
    card_top = TOP + SLOT_FLOOR + PIN_L + CH
    grip = card_top - GRIP_BELOW_TOP
    touch = BACK + PLATE_RISE + CARD_BELOW_TCP
    order = [r * NCOL + c for r in range(NROW - 1, -1, -1)
             for c in (range(NCOL) if r % 2 else range(NCOL - 1, -1, -1))]
    # Development-only run length; it changes the motion, never the scoring or release rule.
    order = order[int(os.environ.get('TABLET_PILOT_START', 0)):]  # motion-only diagnostic offset
    pilot_count=int(os.environ.get('TABLET_PILOT_COUNT', N))
    order = order[:pilot_count]
    if pilot_count>=N and int(os.environ.get('TABLET_PILOT_START',0))==0 and not os.environ.get('TABLET_NEGATIVE_CONTROL'):
        order += [r*NCOL for r in range(8)]  # second physical pass over the most offset column, within200s
    x0, y0 = HOLES[order[0]]
    lot_aim_x=0.  # offset ledge holds plate closer to its declared x; remove the old-grip correction
    first_aim_x=AIM_X+lot_aim_x-(.0015 if order[0]%NCOL==5 else 0.)
    R = [w(2., (hx, hy, grip + .06), FINGER_OPEN, 'above spreader'),
         w(1.5, (hx, hy, grip), FINGER_OPEN, 'around spreader'),
         w(1., (hx, hy, grip), FINGER_CLOSED, 'grasp spreader'),
         w(1.5, (hx, hy, grip + .05), FINGER_CLOSED, 'lift spreader'),
         w(3., (PX+x0-PIN_X+first_aim_x, PY+y0+AIM_Y, touch+.02+TILT_X*x0+TILT_Y*y0), FINGER_CLOSED, 'above first tablet')]
    for order_index, k in enumerate(order):
        c = k % NCOL
        settled_x=(-.002 if c==0 else -.001 if c==1 else 0.) if order_index>=2*NCOL else 0.
        aim_x = AIM_X + lot_aim_x + settled_x - (.0015 if c==5 else 0.)  # measured last-column axis error; other PGS targets track closely  # column correction from measured pin/plate coordinates
        x, y = HOLES[k]; dz=TILT_X*x+TILT_Y*y; x += PX - PIN_X + aim_x; y += PY + AIM_Y
        audit=os.environ.get('TABLET_NEGATIVE_CONTROL') == 'shape-audit'
        card_only=audit and order_index==1
        if card_only: x,y,dz=PX+PW/2+.010,PY,0.
        if os.environ.get('TABLET_NEGATIVE_CONTROL') == 'plate-only' or (audit and order_index==2):
            x, y = PX - PIN_X, PY + Y0
        push_z=touch+dz-DEPTH-(PIN_L-.002 if card_only else 0.)
        press_s=1.5 if order_index>=N else 2.
        R.extend([w(.3, (x, y, touch+dz+.006), FINGER_CLOSED, f'above tablet {k}'),
                  w(press_s, (x, y, push_z), FINGER_CLOSED, f'push tablet {k}'),
                  w(.1, (x, y, push_z), FINGER_CLOSED, f'finish push {k}'),
                  w(.2, (x, y, touch+dz+.006), FINGER_CLOSED, f'withdraw {k}')])
    R.extend([w(1.5, (PX, PY-Y0, touch+.03), FINGER_CLOSED, 'lift clear'),
              w(2.5, (LAY[0], LAY[1], TOP+.005+CARD_BELOW_TCP+.03), FINGER_CLOSED, 'over free counter'),
              w(1.5, (LAY[0], LAY[1], TOP+.005+CARD_BELOW_TCP), FINGER_CLOSED, 'pin 5 mm above counter'),
              w(1., (LAY[0], LAY[1], TOP+.005+CARD_BELOW_TCP), FINGER_OPEN, 'release spreader'),
              w(1.5, (LAY[0], LAY[1], TOP+.005+CARD_BELOW_TCP+.08), FINGER_OPEN, 'hand up'),
              Waypoint(2.5, rest('right'), FINGER_CLOSED, 'right arm clear', 0.),
              Waypoint(3., rest('right'), FINGER_CLOSED, 'hold still', 0.)])
    total = sum(x.duration for x in R)
    hg = BACK + HANDLE_H - GRIP_BELOW_TOP
    lx,ly=PX+LEFT_X_BIAS,PY+HANDLE_Y+LEFT_Y_BIAS
    def support(d,z,label):return w(d,(lx,ly,z),LEFT_FINGER_HOLD,label)
    L=[w(2.,(lx,ly,hg+.05),FINGER_OPEN,'above grip tab'),
       w(1.5,(lx,ly,hg),FINGER_OPEN,'around grip tab'),
       support(1.,hg,'grasp plate'),support(1.5,hg+LIFT,'lift plate'),
       support(3.,hg+LIFT,'settle support')]
    for support_index,k in enumerate(order):
        press_s=1.5 if support_index>=N else 2.
        L.extend([support(.3,hg+LIFT,'support before push'),
                  support(press_s,hg+LIFT,'support loaded push'),
                  support(.1,hg+LIFT,'finish support'),
                  support(.2,hg+LIFT,'hold after push')])
    L.extend([support(5.5,hg+LIFT,'support until tool clear'),support(2.,hg,'park plate'),
              w(1.,(lx,ly,hg),FINGER_OPEN,'release plate'),
              w(1.5,(lx,ly,hg+.06),FINGER_OPEN,'lift left hand'),
              Waypoint(3.5,rest('left'),FINGER_CLOSED,'left hand clear',0.)])
    assert abs(sum(wp.duration for wp in L)-total)<1e-6
    return {'left':L,'right':R}




parts._tablet_namespace = globals()
