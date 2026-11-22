"""A pharmacy compounding counter with a cooled batch of sixty white blank tablets in the black cavity plate of a
Torpac ProMold RDTM2G 75 mg tablet mold. The plate lies upside down on two aluminium bars standing on a sheet of
baking parchment, so the narrow ends of its sixty tapered through-holes face up, each with a tablet's back face in
it, and there is room under the plate for tablets to drop onto the parchment. The mold's white plastic powder
spreader stands upright in a slotted wooden block beside the plate.
"""
from __future__ import annotations
import json, math, os, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts, lots
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
CLEAR = .00002                             # radial clearance of a tablet in its hole
N = NCOL * NROW
PARCH_T = .0002                            # parchment sheet (modelled 0.2 mm)
PT = TOP + PARCH_T                         # parchment top
BAR = .0127; BAR_L = .1524                 # 1/2 in square 6061 aluminium bar, 6 in long
PX, PY = .300, 0.                          # plate centre (same in every lot)
BAR_DX = PW / 2 - BAR / 2                  # bars under the plate's two long margins
PLATE_Z = PT + BAR + PH / 2
BACK = PLATE_Z + PH / 2                    # the plate's upturned back face
RISE = CLEAR * PH / (A_BOT - A_TOP)        # a seated tablet rises this far into the taper
RECESS = .0001                             # ... and its back face sits 0.1 mm below the plate's back face
TAB_H = PH - RISE - RECESS                 # modelled tablet height (3.45 mm)
TAB_Z = PLATE_Z - PH / 2 + RISE + TAB_H / 2
HOLES = np.array([((c - (NCOL - 1) / 2) * PITCH_X, (r - (NROW - 1) / 2) * PITCH_Y) for r in range(NROW) for c in range(NCOL)])
TAB_M = .000075                            # 75 mg nominal tablet (maker: density 0.75 g/ml x 0.10 ml)
# Powder spreader (white rigid plastic card; size, mass and friction are estimates from the maker's photos).
CW, CT, CH = .090, .003, .070
ROD = CT / 2                               # its lower edge is rounded (a 1.5 mm radius rod)
EDGE_Z = -CH / 2 + ROD                     # rod centre in the card frame
# Slotted wooden block holding the spreader upright.
BLK = (.100, .040, .020); SLOT_FLOOR = .005; SLOT_W = .0045
HOLD = {'A': (.220, -.200), 'B': (.410, -.190)}
LAY = (.300, -.300)                         # where the demo lays the spreader down (any free place scores)
# ---- Declared release / damage rule (stand-ins for the tablets' bond to the coated plate and their strength).
F_LO = 1.0      # N: the spreader edge must press at least this hard on the plate as it passes over a tablet's hole
F_HI = 6.0      # N: a total downward push (spreader and grippers) over this on the plate and tablets crushes them
TOUCH_N = .02   # N: no gripper contact on a tablet is allowed (no-touch handling: hard fingertips, not a finger)
EDGE_GAP = .0003  # m: the edge counts as on the plate within 0.3 mm of its back face
KNOCK = .010; KNOCK_DEG = 5.  # plate shifted 10 mm or tilted 5 degrees on its bars: knocked
HOLD_F = 2 * TAB_M * 9.81   # N, upward: holds a tablet in its hole until it lets go (twice its weight)
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


# Hard-anodized, PTFE-coated aluminium plate (mass from its aluminium volume, an estimate).
PLATE = parts.Part('cavity_plate', pos=(PX, PY, PLATE_Z), mass=.166, shapes=plate_shapes(), friction=(.2, .16), contact_offset=.001,
                    report=('base',))
_tp, _tf = frustum(A_BOT - CLEAR - (A_BOT - A_TOP) * TAB_H / PH, A_BOT - CLEAR, TAB_H)
TABLETS = [parts.Part(f'tablet_{k:02d}', pos=(PX + float(HOLES[k][0]), PY + float(HOLES[k][1]), TAB_Z), mass=TAB_M,
                      shapes=(parts.hull(_tp, _tf, WHITE),), friction=(.2, .16), contact_offset=.0015,
                      report=('base',)) for k in range(N)]
BARS = [parts.Part(f'bar_{s}', pos=(PX + sg * BAR_DX, PY, PT + BAR / 2), mass=.066, shapes=(parts.box((BAR, BAR_L, BAR), color=ALU),),
                   friction=(.6, .5), contact_offset=.001) for s, sg in (('far', 1), ('near', -1))]
PARCHMENT = parts.Part('parchment', pos=(PX, PY, TOP + PARCH_T / 2), shapes=(parts.box((.26, .30, PARCH_T), color=PAPER),), friction=(.5, .4))
_bx, _by, _bz = BLK; _ww = (_by - SLOT_W) / 2
BLOCK = parts.Part('spreader_block', pos=(*HOLD['A'], TOP), mass=.080, friction=(.5, .4), contact_offset=.001, shapes=(
    parts.box((_bx, _by, SLOT_FLOOR), (0, 0, SLOT_FLOOR / 2), WOOD),
    parts.box((_bx, _ww, _bz - SLOT_FLOOR), (0, +(SLOT_W + _ww) / 2, (SLOT_FLOOR + _bz) / 2), WOOD),
    parts.box((_bx, _ww, _bz - SLOT_FLOOR), (0, -(SLOT_W + _ww) / 2, (SLOT_FLOOR + _bz) / 2), WOOD),
    parts.box((.004, SLOT_W, _bz - SLOT_FLOOR), (+(_bx / 2 - .002), 0, (SLOT_FLOOR + _bz) / 2), WOOD),
    parts.box((.004, SLOT_W, _bz - SLOT_FLOOR), (-(_bx / 2 - .002), 0, (SLOT_FLOOR + _bz) / 2), WOOD)))
CARD_Z = TOP + SLOT_FLOOR + CH / 2
SPREADER = parts.Part('spreader', pos=(*HOLD['A'], CARD_Z), mass=.018, friction=(.4, .35), contact_offset=.001, shapes=(
    parts.box((CW, CT, CH - ROD), (0, 0, (CH / 2 + EDGE_Z) / 2), WHITE),
    parts.prism(ROD, CW, 16, (0, 0, EDGE_Z), WHITE)))
OBJECTS = [PARCHMENT, *BARS, PLATE, *TABLETS, BLOCK, SPREADER]
NAMES = [t.name for t in TABLETS]
LOTS = lots.declare(OBJECTS,
                    lots.Lot('A', place={'spreader_block': (*HOLD['A'], TOP), 'spreader': (*HOLD['A'], CARD_Z)}),
                    lots.Lot('B', place={'spreader_block': (*HOLD['B'], TOP), 'spreader': (*HOLD['B'], CARD_Z)}))
DEMO_TOUCHES = set()


# ---- Record of the trial (module state; reset on each scene reset).
def fresh():
    return dict(held=np.ones(N, bool), freed=np.zeros(N, bool), zero=False, why='', known=False, phys=0, steps=0, t=None, still=None, clear=None,
                prev=None, views=None, peak=0., press=0., hand_tab=0., events=[], released_at={})
S = fresh()
def fail(reason):
    if not S['zero']: S.update(zero=True, why=str(reason)); event('ZERO: ' + str(reason))
def event(text):
    if len(S['events']) < 80: S['events'].append(text)


def edge_in_plate(card_pos, card_rot, plate_pos, plate_rot):
    """The spreader's rounded lower edge (centre line ends) in the plate's frame."""
    ends = np.array([(-CW / 2, 0., EDGE_Z), (CW / 2, 0., EDGE_Z)])
    return (np.asarray(card_pos) + ends @ np.asarray(card_rot).T - np.asarray(plate_pos)) @ np.asarray(plate_rot)


def edge_geometry(e):
    a, b = e[0], e[1]; d = b[:2] - a[:2]; L2 = max(float(d @ d), 1e-12)
    t = ((HOLES - a[:2]) @ d) / L2
    dist = np.linalg.norm(HOLES - (a[:2] + t[:, None] * d), axis=1)
    zb = a[2] + t * (b[2] - a[2]) - ROD - PH / 2
    return t, dist, zb


def under_edge(e):
    """Holes the edge is over, on the plate (bottom of the edge within EDGE_GAP of the back face, the hole's centre
    within its 2.75 mm narrow-end radius of the edge line, between the edge's ends)."""
    t, dist, zb = edge_geometry(e)
    return (t >= 0) & (t <= 1) & (dist <= A_TOP + EPS) & (zb <= EDGE_GAP + EPS)


def edge_clear(e):
    """Holes the edge is no longer touching (moved past or lifted off), so a freed tablet can drop."""
    t, dist, zb = edge_geometry(e)
    return (t < -.02) | (t > 1.02) | (dist > A_TOP + ROD + .001) | (zb > .002)


def rule_step(press, hand_tab, total, e, t=0.):
    """Declared release / damage rule for one physics step. press: N the spreader pushes down on the plate and
    tablets; hand_tab: largest gripper force on any tablet (N); total: N pushed down on the plate and tablets by the
    spreader and both grippers together (what a scale under the parchment would read above its tare);
    e: the spreader edge in the plate frame. Returns the tablets freed this step."""
    S['peak'] = max(S['peak'], float(total)); S['hand_tab'] = max(S['hand_tab'], float(hand_tab))
    if not all(np.isfinite([press, hand_tab, total])) or not np.isfinite(e).all(): raise ValueError('contact state unknown')
    if total > F_HI + EPS: fail(f'{total:.2f} N pushed down on the plate (over {F_HI} N): tablets crushed')
    if hand_tab > TOUCH_N + EPS: fail(f'a gripper touched a tablet ({hand_tab:.3f} N)')
    if S['zero'] or press < F_LO - EPS: return np.zeros(N, bool)
    out = under_edge(e) & ~S['freed']
    for k in np.flatnonzero(out):
        S['freed'][k] = True; S['released_at'][int(k)] = round(float(t), 3)
    if out.any(): event(f't={t:.2f} freed {int(out.sum())} at {press:.2f} N, {int((~S["freed"]).sum())} left')
    return out


def drop_step(e):
    """A freed tablet drops (its hold is removed) once the spreader's edge has passed it or lifted off."""
    out = S['freed'] & S['held'] & edge_clear(e)
    S['held'][out] = False
    return out


# ---- Scoring helpers.
def tilt_deg(q):
    return math.degrees(math.acos(max(-1., min(1., float(parts._quat_to_matrix(q)[2, 2])))))
def on_parchment(st):
    """A tablet out of its hole lying flat on the parchment, at rest."""
    px, py = PARCHMENT.pos[0], PARCHMENT.pos[1]
    return bool(PT - .001 <= st.low[2] <= PT + .001 and st.high[2] <= PT + .005 + EPS
                and abs(st.pos[0] - px) <= .13 and abs(st.pos[1] - py) <= .15
                and np.linalg.norm(st.lin_vel) <= .01 + EPS)
def count(o):
    return int(sum(1 for k, n in enumerate(NAMES) if S['freed'][k] and not S['held'][k] and on_parchment(o[n])))
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
    sp = o['spreader']
    if box_gap(h, sp.pos, parts._quat_to_matrix(sp.quat_xyzw), np.array((CW, CT, CH))).min() < .030 - EPS: return False
    return True


def observe(o, h, t):
    """Whole-trial checks at each control step; pure, so offline checks can drive it."""
    if h is None or len(h) == 0 or not np.isfinite(h).all():
        S['known'] = False; fail('arm state unknown'); return
    for n in ['cavity_plate', 'spreader', *NAMES]:
        st = o[n]
        if not np.isfinite(st.pos).all() or not np.isfinite(st.quat_xyzw).all():
            S['known'] = False; fail('object state unknown'); return
    if knocked(o['cavity_plate']): fail('plate knocked on its bars (moved over 10 mm or tilted over 5 degrees)')
    for n in NAMES:
        if o[n].low[2] < TOP - .005: fail(n + ' fell off the counter')
    snap = np.array([o[n].pos for n in ['cavity_plate', 'spreader', *NAMES]])
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


def contact_loads(sim):
    v, nf = S['views']
    m = arr(v.get_contact_force_matrix(dt=sim.env.unwrapped.physics_dt))
    m = m.reshape(N + 1, nf, 3)
    if not np.isfinite(m).all(): raise ValueError('non-finite contact forces')
    press = abs(float(m[:, 0, 2].sum()))                       # spreader on plate + tablets, along the vertical
    total = abs(float(m[:, :, 2].sum()))                        # spreader and grippers together
    hand_tab = float(np.linalg.norm(m[1:, 1:, :], axis=2).max()) if nf > 1 else 0.
    return press, hand_tab, total


def set_hold(sim, k, on):
    sim.set_wrench(NAMES[k], (0., 0., HOLD_F if on else 0.), (0., 0., 0.))


@parts.on_reset
def reset(sim):
    """Seat every tablet in its hole (the stock settle lets them drop) and hold it there until it lets go."""
    global S
    S = fresh()
    from isaaclab.app.settings_manager import get_settings_manager
    get_settings_manager().set_bool('/physics/disableContactProcessing', False)
    import torch
    base = sim.env.unwrapped; org = base.scene.env_origins[0:1]
    for k, n in enumerate(NAMES):
        obj = base.scene[n]
        pose = obj.data.default_root_pose; pose = (pose.torch if hasattr(pose, 'torch') else pose).clone()
        pose[:, :3] += org
        obj.write_root_pose_to_sim_index(root_pose=pose)
        vel = obj.data.default_root_vel; vel = vel.torch if hasattr(vel, 'torch') else vel
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(vel))
        set_hold(sim, k, True)


@parts.on_physics_step
def physics(sim):
    if S.get('dead'): return
    try:
        base = sim.env.unwrapped
        if S['views'] is None: S['views'] = make_views(sim)
        press, hand_tab, total = contact_loads(sim)
        cp, cr = _pose(base, 'spreader'); pp, pr = _pose(base, 'cavity_plate')
        S['press'] = press
        e = edge_in_plate(cp, cr, pp, pr)
        rule_step(press, hand_tab, total, e, float(sim.t))
        for k in np.flatnonzero(drop_step(e)): set_hold(sim, int(k), False)
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
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, phys=S['phys'], score=score(o), held=int(S['held'].sum()),
                       freed=int(S['freed'].sum()),
                       on_parchment=count(o), zero=S['zero'], why=S['why'], press=round(S['press'], 3), peak=round(S['peak'], 3),
                       hand_tab=round(S['hand_tab'], 4), clear=S['clear'] is not None,
                       still=S['still'], plate=np.round(o['cavity_plate'].pos, 4).tolist(), plate_tilt=round(tilt_deg(o['cavity_plate'].quat_xyzw), 2),
                       card=np.round(o['spreader'].pos, 4).tolist(), card_tilt=round(tilt_deg(o['spreader'].quat_xyzw), 1),
                       edge=np.round(e, 4).tolist(), bars=[np.round(o[b.name].pos, 4).tolist() for b in BARS],
                       tab_z=[round(float(o[n].pos[2] - TAB_Z) * 1000, 2) for n in NAMES[:12]], events=S['events'][-6:],
                       stray=[dict(k=k, pos=np.round(o[n].pos, 4).tolist(), low=round(float(o[n].low[2] - PT) * 1000, 2),
                                   high=round(float(o[n].high[2] - PT) * 1000, 2), tilt=round(tilt_deg(o[n].quat_xyzw), 1),
                                   v=round(float(np.linalg.norm(o[n].lin_vel)), 4))
                              for k, n in enumerate(NAMES) if S['freed'][k] and not S['held'][k] and not on_parchment(o[n])][:12])
            row['grippers'] = {}
            for arm in ('left_robot', 'right_robot'):
                ar = base.scene[arm]; vals = parts._np(ar.data.body_link_pose_w)[0]
                row['grippers'][arm] = {n: np.round(vals[list(ar.body_names).index(n), :3] - org, 4).tolist() for n in ('gripper', 'tip_left', 'tip_right')}
            with open(log, 'a') as f: f.write(json.dumps(row) + '\n')
    except Exception as e:
        S.update(known=False); fail('state unavailable: ' + str(e))


# ---- Open-loop demo from the visible layout: take the spreader from its block, lower its edge onto the plate's far
# end margin, press down gently and sweep it once along the plate over all ten rows, lift, lay it on the counter.
GRIP_BELOW_TOP = .015                       # TCP this far below the spreader's top edge
CARD_BELOW_TCP = CH - GRIP_BELOW_TOP        # spreader's lower edge below the TCP
DEPTH = .008                                # commanded below first contact (the arm's compliance sets the force)
Y0 = .071                                   # sweep from the far end margin to the near one (mid-margin, 71 mm out)


def demo():
    lot = lots.current(); hx, hy = lot.pos('spreader')[:2]
    def rest(arm): return (.300, .300 if arm == 'left' else -.300, TOP + .18)
    def w(d, p, f, label): return Waypoint(float(d), tuple(float(v) for v in p), float(f), label, 0.)
    card_top = TOP + SLOT_FLOOR + CH
    grip = card_top - GRIP_BELOW_TOP
    touch = BACK + CARD_BELOW_TCP           # TCP when the edge just meets the back face
    R = [
        w(2.0, (hx, hy, grip + .06), FINGER_OPEN, 'above the spreader'),
        w(1.5, (hx, hy, grip), FINGER_OPEN, 'fingers either side of its top'),
        w(1.0, (hx, hy, grip), FINGER_CLOSED, 'pinch the spreader'),
        w(1.5, (hx, hy, grip + .05), FINGER_CLOSED, 'lift it out of the block'),
        w(2.5, (PX, PY + Y0, touch + .02), FINGER_CLOSED, 'over the far end of the plate'),
        w(1.5, (PX, PY + Y0, touch + .002), FINGER_CLOSED, 'edge just above the margin'),
        w(1.5, (PX, PY + Y0, touch - DEPTH), FINGER_CLOSED, 'press the edge down'),
        w(7.0, (PX, PY - Y0, touch - DEPTH), FINGER_CLOSED, 'sweep along the plate over all rows'),
        w(0.8, (PX, PY - Y0, touch - DEPTH), FINGER_CLOSED, 'stop on the near margin'),
        w(1.5, (PX, PY - Y0, touch + .03), FINGER_CLOSED, 'lift off the plate'),
        w(2.5, (LAY[0], LAY[1], TOP + .005 + CARD_BELOW_TCP + .03), FINGER_CLOSED, 'over a free spot'),
        w(1.5, (LAY[0], LAY[1], TOP + .005 + CARD_BELOW_TCP), FINGER_CLOSED, 'edge 5 mm above the counter'),
        w(1.0, (LAY[0], LAY[1], TOP + .005 + CARD_BELOW_TCP), FINGER_OPEN, 'let go'),
        w(1.5, (LAY[0], LAY[1], TOP + .005 + CARD_BELOW_TCP + .08), FINGER_OPEN, 'hand up'),
        Waypoint(2.5, rest('right'), FINGER_CLOSED, 'right arm clear', 0.),
    ]
    total = sum(x.duration for x in R) + 3.0
    R.append(Waypoint(3.0, rest('right'), FINGER_CLOSED, 'hold still', 0.))
    L = [Waypoint(total, rest('left'), FINGER_CLOSED, 'left arm stays clear', 0.)]
    return {'left': L, 'right': R}


parts._tablet_namespace = globals()
