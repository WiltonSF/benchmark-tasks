"""A complete Original Prusa MK4S stands on its counter, switched off after heating. Its print carriage is parked
low above the build plate. Dark PETG residue coats the underside of the installed heater block and the brass
nozzle taper. Two red heater leads and two white thermistor leads leave the back left face of the block (seen from the
front) at its mid-height, about 5 mm above its underside. A shallow steel catch tray lies on the build plate under the nozzle. A wood-handled
brass nozzle brush lies bristles up on the counter to the left of the printer.
"""
import json, math, os, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts, lots, model, observation
observation.install_views()
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint, FINGER_OPEN, grip_rotation
from yam_arena.constants import GRIPPER
GRIPPER['effort'] = 40.0  # the declared per-finger force, in this process only
SCENE = 'station'; TOP = TABLE_TOP[SCENE]; SCORE_OVER = 'end'; SUCCESS_HOLD_S = .1
BLACK = (.11, .12, .13); ORANGE = (.92, .29, .025); STEEL = (.65, .68, .70); BRASS = (.73, .55, .18); WOOD = (.53, .31, .15)
HOT = np.array((.27, 0., TOP + .182)); BED_Z = TOP + .100
UNDERSIDE_Z = HOT[2] - .006
# Complete 500 x 550 x 400 mm printer at full size; its front faces the arms (-x). Millimetres to metres, not rescaled.
shapes = []
for y in (-.205, .205):
    shapes += [parts.box((.430, .040, .040), (.035, y, .030), BLACK), parts.box((.018, .018, .350), (.100, y, .215), ORANGE), parts.cylinder(.004, .340, (.095, y - .018, .220), STEEL)]
for x in (-.180, .250):
    shapes.append(parts.box((.040, .450, .040), (x, 0, .030), BLACK))
shapes += [parts.box((.050, .450, .020), (.100, 0, .390), ORANGE), parts.box((.110, .040, .080), (.100, .218, .075), BLACK), parts.box((.110, .040, .080), (.100, -.218, .075), BLACK), parts.box((.140, .040, .250), (.180, -.230, .185), BLACK), parts.box((.060, .060, .200), (.175, .215, .160), BLACK)]
# Bed (100 mm above the counter), Y rods, belt and the front control panel.
shapes += [parts.box((.260, .260, .006), (-.035, 0, .094), BLACK), parts.box((.240, .240, .003), (-.035, 0, .0985), (.34, .34, .31)), parts.cylinder(.004, .380, (.020, .090, .075), STEEL, 'x'), parts.cylinder(.004, .380, (.020, -.090, .075), STEEL, 'x'), parts.box((.380, .006, .004), (.020, 0, .066), BLACK), parts.box((.030, .105, .048), (-.265, -.105, .045), BLACK), parts.box((.002, .080, .032), (-.281, -.105, .050), (.12, .30, .30))]
# X rods across the gantry; the print carriage is its own fixed body.
shapes += [parts.cylinder(.004, .370, (.040, 0, .252), STEEL, 'y'), parts.cylinder(.004, .370, (.040, 0, .202), STEEL, 'y')]
carriage_shapes = [parts.box((.045, .065, .075), (.015, 0, .060), BLACK), parts.cylinder(.025, .025, (-.015, 0, .060), BLACK, 'x'), parts.box((.040, .012, .040), (.012, .044, .038), BLACK), parts.box((.040, .012, .040), (.012, -.044, .038), BLACK), parts.box((.035, .065, .008), (0, 0, .016), BLACK), parts.cylinder(.0025, .016, (0, 0, .014), BRASS)]
carriage_shapes += [parts.cylinder(.009, .008, (-.028, .017, .043), BLACK, 'x'), parts.cylinder(.009, .008, (-.028, -.017, .043), BLACK, 'x')]
# Fan shroud: side walls and back wall around the block, open at the front.
carriage_shapes += [parts.box((.047, .008, .023), (0, .025, .005), BLACK), parts.box((.047, .008, .023), (0, -.025, .005), BLACK), parts.box((.008, .050, .023), (.027, 0, .005), BLACK)]
CARRIAGE = parts.Part('native_nextruder_carriage', pos=tuple(HOT), shapes=carriage_shapes, contact_offset=.0003)
shapes += [parts.box((.040, .180, .008), (.150, .020, .396), BLACK), parts.box((.008, .008, .120), (.060, .035, .330), BLACK)]
PRINTER = parts.Part('complete_mk4s', pos=(.305, 0, TOP), shapes=shapes, contact_offset=.0003)
# Heater block 13 x 20 x 12 mm (estimated from maker photographs and the 13 mm wrench), turned 35 degrees as installed.
BLOCK = parts.Part('installed_heaterblock', pos=tuple(HOT), yaw=math.radians(35), shapes=[parts.box((.013, .020, .012), (0, 0, 0), STEEL)], contact_offset=.0002)
# Nozzle: 4 mm hex shoulder and a taper from 4 mm to 0.6 mm radius, 6 mm below the block.
points = []
for z, r in ((-.012, .0006), (-.007, .004)):
    points += [(r * math.cos(a), r * math.sin(a), z) for a in np.arange(24) * math.tau / 24]
faces = [tuple(range(23, -1, -1)), tuple(range(24, 48))] + [(i, (i + 1) % 24, (i + 1) % 24 + 24, i + 24) for i in range(24)]
NOZZLE = parts.Part('installed_brass_nozzle', pos=tuple(HOT), shapes=[parts.hull(points, faces, BRASS), parts.cylinder(.004, .001, (0, 0, -.0065), BRASS)], contact_offset=.00015)
# Leads leave the block's back left face (seen from the front) at mid-height, square to the face: two heater leads
# and two thermistor leads, 4 to 8.8 mm from the face's centre line. They run 12 mm out from the face and turn up to
# the carriage. Block frame: x' along the face normal, turned 35 degrees like the block.
LEAD_TURN = math.radians(35)
WIRE_SEGMENTS = []; wires = []
_lr = parts._rot((0, 0, 1), LEAD_TURN)
for j, (yl, r, col) in enumerate(((.0040, .0011, (.65, .10, .07)), (.0062, .0011, (.65, .10, .07)), (.0078, .00065, (.85, .85, .82)), (.0090, .00065, (.85, .85, .82)))):
    a = HOT + _lr @ np.array((.0065, yl, 0.)); b = HOT + _lr @ np.array((.0185, yl, 0.)); c = b + np.array((0., 0., .029))
    WIRE_SEGMENTS.extend(((a, b, r), (b, c, r)))
    wires += [parts.cylinder(r, .012, (.0125, yl, 0.), col, 'x'), parts.cylinder(r, .029, (.0185, yl, .0145), col, 'z')]
WIRES = parts.Part('heater_thermistor_leads', pos=tuple(HOT), yaw=LEAD_TURN, shapes=wires, contact_offset=.0001)
# Brass nozzle brush, 172 mm, 35 g: 12 x 12 mm wood handle, wood head; the bristles are the declared bundle model.
BRUSH = parts.Part('brass_brush', pos=(.250, .265, TOP + .006), mass=.035, friction=(.9, .8), angular_damping=2., contact_offset=.0003, report=('base',), shapes=[parts.box((.125, .012, .012), (-.0175, 0, 0), WOOD), parts.box((.034, .012, .012), (.075, 0, 0), WOOD), parts.box((.018, .010, .012), (.051, 0, 0), WOOD)])
# Shallow stainless catch tray, 120 x 80 x 15 mm, 1 mm sheet, loose on the build plate.
PAN = parts.Part('service_tray', pos=(.250, -.010, BED_Z), mass=.14, friction=(.45, .35), contact_offset=.0003, shapes=[parts.box((.120, .080, .001), (0, 0, .0005), STEEL), parts.box((.120, .001, .015), (0, .0395, .0075), STEEL), parts.box((.120, .001, .015), (0, -.0395, .0075), STEEL), parts.box((.001, .078, .015), (.0595, 0, .0075), STEEL), parts.box((.001, .078, .015), (-.0595, 0, .0075), STEEL)])
OBJECTS = [PRINTER, CARRIAGE, BLOCK, NOZZLE, WIRES, BRUSH, PAN]
# Lot B moves the brush 80 mm forward and the tray 40 mm back and 20 mm left; the printer is fixed in both.
LOTS = lots.declare(OBJECTS, lots.Lot('A', place={BRUSH.name: BRUSH.pos, PAN.name: PAN.pos}), lots.Lot('B', place={BRUSH.name: (.170, .255, TOP + .006), PAN.name: (.290, .010, BED_Z)}))
DEMO_TOUCHES = ()
_model = model.Residue(HOT); _known = False; _bad = False; _hold = 0.; _time = 0.; _cert = None; _active = False; _sim = None
_visual = []; _fibers = []; _last_trace = -1.; _clear = False; _still = False; _released = False; _last_wrench = None
_motion_history = []; _brush_com = np.zeros(3)


def R(s): return parts._quat_to_matrix(s.quat_xyzw)


def _log_path():
    # Optional diagnostics only (TRACE_OUT from the trace wrapper, or NOZZLE_LOG); never read back by the demo or scorer.
    return os.environ.get('TRACE_OUT') or os.environ.get('NOZZLE_LOG')


def signature(st): return tuple(float(v) for n in (BRUSH.name, PAN.name) for v in np.r_[st[n].pos, st[n].quat_xyzw, st[n].lin_vel, st[n].ang_vel])


def boxes():
    # Block (tag 1, residue surface) and the carriage and shroud near the hotend (tag 2) stop the bristles.
    out = [(c, r, s, 1) for _, c, r, s in BLOCK.world_boxes({'base': (np.asarray(BLOCK.pos), BLOCK.rotation)})]
    for _, c, r, s in CARRIAGE.world_boxes({'base': (np.asarray(CARRIAGE.pos), CARRIAGE.rotation)}):
        if np.linalg.norm(c - HOT) < .13:
            out.append((c, r, s, 2))
    return out


_BOXES = boxes()


@parts.before_reset
def before_reset(sim):
    global _active
    _active = False
    if sim is not None:
        sim.set_wrench(BRUSH.name, (0, 0, 0), (0, 0, 0))


def box_gap(cloud, spec, state):
    best = 10.
    for _, c, r, size in spec.world_boxes({'base': (np.asarray(state.pos), R(state))}):
        nearby = np.all(np.abs(cloud - c) <= (np.abs(r) @ (size / 2) + .035), axis=1)
        if nearby.any():
            best = min(best, float(np.linalg.norm(np.maximum(np.abs((cloud[nearby] - c) @ r) - size / 2, 0), axis=1).min()))
    return best


def brush_released(state):
    # Lying on the counter (its lowest point within 2 mm of the counter), not on the printer.
    return abs(float(state.low[2]) - TOP) <= .002 and float(state.low[1]) > .225


def tray_ok(p):
    # Upright on the build plate with the nozzle still above its floor.
    return abs(float(p.low[2]) - BED_Z) <= .002 and R(p)[2, 2] >= math.cos(math.radians(5)) and bool(np.all(np.abs(((HOT - p.pos) @ R(p))[:2]) <= np.array((.052, .032))))


def contacts_and_clear(sim, st):
    clouds = observation.hand_state(sim); clear = True
    for cloud in clouds:
        # A finger within 3 mm of the hot block or nozzle counts as touching it.
        if min(box_gap(cloud, BLOCK, st[BLOCK.name]), box_gap(cloud, NOZZLE, st[NOZZLE.name])) <= .003:
            _model.invalid = True
        for spec in (BRUSH, PAN, BLOCK, NOZZLE):
            if box_gap(cloud, spec, st[spec.name]) <= .030:
                clear = False
    bs = st[BRUSH.name]
    for a, b, r in WIRE_SEGMENTS:
        for t in np.linspace(0, 1, 12):
            if box_gap(np.asarray([a + t * (b - a)]), BRUSH, bs) <= r + .00015:
                _model.wire_hit = True
    return clear


def native_tick(dt):
    global _bad, _last_wrench
    if not _active or _sim is None:
        return
    try:
        st = _sim.states(); b, p = st[BRUSH.name], st[PAN.name]
        _model.advance(dt, np.asarray(b.pos), R(b), _BOXES, np.asarray(p.pos), R(p), BED_Z, WIRE_SEGMENTS)
        # The bristle reaction is applied to the free brush body; no pose or grip is set.
        force_local = R(b).T @ _model.reaction
        wrench = np.r_[force_local, R(b).T @ _model.torque - np.cross(_brush_com, force_local)]
        if _last_wrench is None or np.max(np.abs(wrench - _last_wrench)) > 1e-5:
            _sim.set_wrench(BRUSH.name, wrench[:3], wrench[3:]); _last_wrench = wrench
        for cloud in observation.near_hot_clouds(_sim, HOT, .025):
            if min(box_gap(cloud, BLOCK, st[BLOCK.name]), box_gap(cloud, NOZZLE, st[NOZZLE.name])) <= .003:
                _model.invalid = True
        for a, bb, radius in WIRE_SEGMENTS:
            pts = np.array([a + t * (bb - a) for t in np.linspace(0., 1., 12)])
            if box_gap(pts, BRUSH, b) <= radius + .00015:
                _model.wire_hit = True
    except (ValueError, KeyError, AttributeError, IndexError, TypeError) as exc:
        _bad = True
        if _log_path():
            with pathlib.Path(_log_path()).with_name('nozzle-errors.txt').open('a') as f:
                f.write(repr(exc) + '\n')


@parts.on_reset
def reset(sim):
    global _model, _known, _bad, _hold, _time, _cert, _active, _sim, _visual, _fibers, _last_trace, _clear, _still, _released
    global _last_wrench, _motion_history, _brush_com
    _active = False; _last_wrench = None; _motion_history = []; _model = model.Residue(HOT); _known = False; _bad = False
    _hold = 0.; _time = float(sim.t); _cert = None; _sim = sim; _last_trace = -1.; _clear = False; _still = False; _released = False
    base = sim.env.unwrapped
    _brush_com = parts._np(base.scene[BRUSH.name].data.body_com_pos_b)[0, 0].copy()
    if _brush_com.shape != (3,) or not np.isfinite(_brush_com).all():
        raise ValueError('unknown brush centre of mass')
    from pxr import UsdGeom, Gf, UsdLux
    from isaaclab.sim.utils import get_current_stage
    from yam_arena.station import linear_rgb
    stage = get_current_stage(); path = base.scene.env_prim_paths[0]; origin = parts._np(base.scene.env_origins[0])
    light = UsdLux.SphereLight.Define(stage, path + '/maintenance_inspection_light')
    light.CreateIntensityAttr(14000.); light.CreateRadiusAttr(.015); light.CreateColorAttr(Gf.Vec3f(1., .98, .92))
    ops = light.GetOrderedXformOps(); op = ops[0] if ops else light.AddTranslateOp()
    op.Set(Gf.Vec3d(*map(float, HOT + np.array((-.09, -.08, -.025)) + origin)))
    if not getattr(base.scene, '_nozzle_tick', False):
        original = base.scene.update

        def update(dt, *args, **kw):
            original(dt, *args, **kw); native_tick(float(dt))
        base.scene.update = update; base.scene._nozzle_tick = True
    _visual = []; _fibers = []
    for i in range(model.N):
        sp = UsdGeom.Cylinder.Define(stage, path + '/petg_residue/patch_' + str(i)); sp.CreateRadiusAttr(model.PATCH_R); sp.CreateHeightAttr(model.THICKNESS)
        sp.CreateDisplayColorAttr([Gf.Vec3f(*linear_rgb((.17, .09, .025)))])
        ops = sp.GetOrderedXformOps(); op = ops[0] if ops else sp.AddTranslateOp(); op.Set(Gf.Vec3d(*map(float, _model.pos[i] + origin)))
        q = parts._matrix_to_quat(parts._x_to(model.NORMALS[i]) @ parts._rot((0, 1, 0), math.pi / 2))
        orient = ops[1] if len(ops) > 1 else sp.AddOrientOp(); orient.Set(Gf.Quatf(float(q[3]), Gf.Vec3f(*map(float, q[:3])))); _visual.append(op)
    for i in range(len(model.ROOTS)):
        sp = UsdGeom.Cylinder.Define(stage, path + '/brass_fibers/bundle_' + str(i)); sp.CreateRadiusAttr(model.BUNDLE_RADIUS); sp.CreateHeightAttr(model.BRISTLE_LENGTH)
        sp.CreateDisplayColorAttr([Gf.Vec3f(*linear_rgb(BRASS))]); ops = sp.GetOrderedXformOps()
        tr = ops[0] if ops else sp.AddTranslateOp(); ori = ops[1] if len(ops) > 1 else sp.AddOrientOp(); _fibers.append((tr, ori, sp.GetHeightAttr()))
    sim.set_wrench(BRUSH.name, (0, 0, 0), (0, 0, 0)); _active = True


@parts.on_step
def step(sim):
    global _known, _bad, _hold, _time, _cert, _active, _sim, _last_trace, _clear, _still, _released
    _sim = sim
    try:
        st = sim.states(); dt = float(sim.t) - _time
        if not 0 < dt <= .100001:
            raise ValueError('unknown timestep')
        if not _active:
            _active = True
        for name in (BRUSH.name, PAN.name):
            s = st[name]
            if not np.isfinite(np.r_[s.pos, s.quat_xyzw, s.lin_vel, s.ang_vel]).all():
                raise ValueError('unknown object pose')
            if s.low[2] < TOP - .003:
                _bad = True
        _clear = contacts_and_clear(sim, st); _known = True
        b, p = st[BRUSH.name], st[PAN.name]
        _released = bool(tray_ok(p) and brush_released(b))
        corners = np.array([(x, y, z) for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)])
        features = []
        for spec, state in ((BRUSH, b), (PAN, p)):
            for _, centre, rotation, size in spec.world_boxes({'base': (np.asarray(state.pos), R(state))}):
                features.append(centre + (corners * size) @ rotation.T)
        pts = np.vstack(features); now = float(sim.t)
        _motion_history.append((now, pts.copy(), bool(_clear and _released and _known and not _bad and not _model.invalid)))
        while len(_motion_history) > 1 and _motion_history[1][0] <= now - 2. - 1e-9:
            _motion_history.pop(0)
        duration = now - _motion_history[0][0]
        span = np.linalg.norm(np.ptp(np.stack([v[1] for v in _motion_history]), axis=0), axis=1).max()
        _still = bool(duration >= 2. - 1e-9 and span <= .001 + 1e-9)
        _hold = duration if _still and all(v[2] for v in _motion_history) else 0.
        _time = now; _cert = signature(st)
        from pxr import Gf
        origin = parts._np(sim.env.unwrapped.scene.env_origins[0])
        for i, op in enumerate(_visual):
            op.Set(Gf.Vec3d(*map(float, _model.pos[i] + origin)))
        br = R(b); q = b.quat_xyzw
        for i, (root, (tr, ori, height)) in enumerate(zip(model.ROOTS, _fibers)):
            length = float(_model.last_lengths[i]); height.Set(max(length, .0001))
            tr.Set(Gf.Vec3d(*map(float, b.pos + br @ (root + np.array((0, 0, length / 2))) + origin))); ori.Set(Gf.Quatf(float(q[3]), Gf.Vec3f(*map(float, q[:3]))))
        if _log_path() and float(sim.t) - _last_trace >= .099:
            row = dict(t=_time, lot=lots.current().name, score=score(st), removed=_model.removed(), states=np.bincount(_model.state, minlength=5).tolist(),
                       sweep=[round(float(v), 5) for v in _model.sweep], load=_model.force, peak=_model.peak, max_bundle=_model.max_bundle,
                       lengths=[round(float(v), 5) for v in _model.last_lengths], contacts=_model.hit_count, known=_known, bad=_bad,
                       invalid=_model.invalid, wire_hit=_model.wire_hit, damage=_model.damage, clear=_clear, still=_still, released=_released,
                       hold=_hold, brush=b.pos.tolist(), brush_quat=b.quat_xyzw.tolist(), brush_low=b.low.tolist(), pan=p.pos.tolist(),
                       reaction=_model.reaction.tolist())
            from yam_arena.constants import TCP_OFFSET
            base = sim.env.unwrapped; row['tcp'] = {}; row['finger_q'] = {}; row['gripper_quat'] = {}
            for arm in ('left_robot', 'right_robot'):
                asset = base.scene[arm]; pose = parts._np(asset.data.body_link_pose_w)[0, asset.body_names.index('gripper')]
                row['tcp'][arm] = (pose[:3] - origin + parts._quat_to_matrix(pose[3:]) @ np.array(TCP_OFFSET)).tolist()
                row['finger_q'][arm] = {n: float(parts._np(asset.data.joint_pos)[0, asset.joint_names.index(n)]) for n in ('joint7', 'joint8')}
                row['gripper_quat'][arm] = pose[3:].tolist()
            with pathlib.Path(_log_path()).with_name('nozzle.jsonl').open('a') as f:
                f.write(json.dumps(row) + '\n')
            _last_trace = float(sim.t)
    except (ValueError, KeyError, AttributeError, IndexError, TypeError) as exc:
        _known = False; _bad = True
        if _log_path():
            with pathlib.Path(_log_path()).with_name('nozzle-errors.txt').open('a') as f:
                f.write(repr(exc) + '\n')


def score(objects):
    try:
        if _bad or _cert != signature(objects):
            return 0.
        return _model.endpoint(_known, _clear, _still, _released, _hold)
    except (KeyError, ValueError, AttributeError, TypeError):
        return 0.


def success(objects): return score(objects) == 1.


# ---- scripted demo: fixed waypoints from the visible layout only (never reads residue, forces or hidden state) ----
GRIP_X = 0.         # grip point along the handle, 75 mm behind the head centre
GRIP_DEPTH = .0095  # handle centre sits 9.5 mm below the TCP, between the pads
GRASP_Z = TOP + .0155
SAG = np.array((0., -.003, .007))  # extra lift commanded while the brush is up (arm compliance under the bristle load)
YAW = -.022         # rad; the loaded wrist turned about 2.5 degrees in a lot A trace; half of that is turned back
PRESS = .003        # intended bristle bend at the block underside, m
# (lane y, back x, front x) of the head centre from the nozzle axis, and a press change: the lead-side lane presses
# 3 mm less because fewer bristles bear on the block there, so the arm yields less and the wood would reach the nozzle.
STROKES = [(-.010, .005, -.003, 0.), (-.006, .005, -.003, 0.), (-.002, .005, -.003, 0.), (0., .005, -.003, 0.), (.002, .005, -.003, 0.),
           (.004, -.012, -.020, -.0015), (.006, -.013, -.021, -.003), (.008, -.013, -.021, -.003)]


def tcp_for_head(head):
    """Straight-down TCP that puts the bristle-root centre of the head at `head`."""
    return np.asarray(head) - np.array((.075, 0., .006)) + np.array((GRIP_X, 0., GRIP_DEPTH))


def demo():
    lot = lots.current(); brush = np.asarray(lot.pos(BRUSH.name))
    def w(t, p, f, label, yaw=0.): return Waypoint(t, tuple(map(float, p)), f, label, yaw)
    opened = -.012; closed = 0.
    grasp = np.array((brush[0] + GRIP_X, brush[1], GRASP_Z)); above = grasp + (0, 0, .085)
    root_z = UNDERSIDE_Z - (model.BRISTLE_LENGTH - PRESS)  # root plane for the intended bend
    low_z = HOT[2] - .012 - model.BRISTLE_LENGTH - .004    # bristle tips 4 mm below the nozzle tip
    def head(x, y, z): return np.array((HOT[0] + x, HOT[1] + y, z))
    def up(x, y, z): return tcp_for_head(head(x, y, z)) + SAG
    plan = [w(2.5, above, opened, 'above the brush handle'), w(2., grasp, opened, 'fingers either side of the handle'),
            w(1., grasp, closed, 'grip the handle'), w(1.5, grasp + (0, 0, .10), closed, 'lift the brush'),
            w(3., up(-.050, STROKES[0][0], low_z + .008), closed, 'bring the head to the open front of the shroud', YAW),
            w(2., up(STROKES[0][1], STROKES[0][0], low_z), closed, 'head under the block, bristles below the nozzle', YAW)]
    for i, (y, back, front, dz) in enumerate(STROKES):
        z = root_z + dz
        if i:
            plan += [w(1., up(prev_front, prev_y, prev_z - .004), closed, 'ease off', YAW),
                     w(1.2, up(back, y, z - .004), closed, 'next lane, bristles eased', YAW)]
        plan.append(w(1.5, up(back, y, z), closed, 'press bristles on the residue', YAW))
        for k in range(1):
            plan += [w(1.8, up(front, y, z), closed, 'stroke toward the front, away from the leads', YAW),
                     w(1.5, up(back, y, z), closed, 'return stroke', YAW)]
        plan.append(w(1.8, up(front, y, z), closed, 'last stroke toward the front', YAW))
        prev_y, prev_front, prev_z = y, front, z
    plan += [w(1.5, up(prev_front, prev_y, low_z), closed, 'lower the bristles clear of the nozzle', YAW),
             w(2., up(-.050, prev_y, low_z), closed, 'withdraw through the open front', YAW),
             w(3., above + (0, 0, .015), closed, 'carry the brush back over its place'),
             w(2., grasp + (0, 0, .001), closed, 'set the brush down'),
             w(1., grasp + (0, 0, .001), opened, 'release the brush'),
             w(2., grasp + (0, 0, .09), opened, 'lift the fingers clear'),
             w(2., (.12, .30, TOP + .16), FINGER_OPEN, 'hand clear'), w(5., (.12, .30, TOP + .16), FINGER_OPEN, 'still hold')]
    total = sum(x.duration for x in plan)
    return {'left': plan, 'right': [w(2., (.12, -.30, TOP + .16), FINGER_OPEN, 'other hand clear'), w(max(1., total - 2.), (.12, -.30, TOP + .16), FINGER_OPEN, 'remain clear')]}
