"""An 812.8 mm exterior head jamb is clamped to a door-frame assembly bench.
A loose Pemko Q102BL strip lies beside its 1/8-inch kerf, offset 20 mm along
the jamb. A smooth wood pressing block lies on the near side. This is an
estimated-force build; the related-product force basis is not a product measurement.
"""
from pathlib import Path
import math
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / 'mechanics'))
import parts
import seal_physics
import lots
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint, FINGER_CLOSED, FINGER_OPEN

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
WORK = TOP + .058
SCORE_OVER = 'end'
SUCCESS_HOLD_S = .05
RUN_LIMIT_STEPS = 2400
DEMO_TOUCHES={'head_jamb'}
N = 34
LENGTH = .8128
PITCH = LENGTH / N
FACE = .265
KERF = .003175
DEPTH = .009525
FLOOR = WORK + .01905
SEAL_Z = FLOOR + .00635
HOME = FACE - .0079375
START_X = HOME - .015525
SHIFT = -.020
GREY = (.70, .70, .68)
BLACK = (.08, .08, .08)

# The slot is empty collision geometry, not a trigger region.
_lo, _hi = SEAL_Z - KERF / 2, SEAL_Z + KERF / 2
JAMB = parts.Part('head_jamb', pos=(0., 0., WORK), shapes=[
    parts.box((.1158875, LENGTH, .01905), (.280, 0., .009525), GREY),
    parts.box((.073, LENGTH, _lo - FLOOR), (FACE + .0365, 0., (_lo + FLOOR) / 2 - WORK), GREY),
    parts.box((.073, LENGTH, FLOOR + .0127 - _hi),
              (FACE + .0365, 0., (FLOOR + .0127 + _hi) / 2 - WORK), GREY),
    parts.box((.073 - DEPTH, LENGTH, KERF),
              (FACE + (DEPTH + .073) / 2, 0., SEAL_Z - WORK), GREY)], friction=(.5, .4))

DECK = parts.Part('assembly_deck', pos=(.280, 0., TOP), shapes=[
    parts.box((.160, .950, .018), (0., 0., .049), (.60, .43, .25)),
    parts.box((.160, .040, .040), (0., -.30, .020), (.60, .43, .25)),
    parts.box((.160, .040, .040), (0., .30, .020), (.60, .43, .25))])

# BESSEY CM30: 76.2 mm opening, 44.45 mm throat. The raised assembly
# deck gives its lower jaw real access under the deck, above the station table.
CLAMPS = []
for side in (-1, 1):
    CLAMPS.append(parts.Part('screw_clamp_' + ('left' if side > 0 else 'right'),
        pos=(.371, side * .33, TOP), shapes=[
            parts.box((.012, .025, .092), (0., 0., .076), (.63, .65, .67)),
            parts.box((.052, .025, .008), (-.020, 0., .118), (.63, .65, .67)),
            parts.box((.052, .025, .008), (-.020, 0., .034), (.63, .65, .67)),
            parts.box((.021, .021, .002), (-.044, 0., .039), (.18, .18, .18)),
            parts.cylinder(.004, .02025, (-.044, 0., .103875), (.65, .65, .65)),
            parts.box((.021, .021, .004), (-.044, 0., .09175), (.18, .18, .18))]))

SEALS = []
for i in range(N):
    # Bulb envelope and fin are true-size. Stepped barb lands are an uncalibrated
    # approximation pending profile measurement; compliant contact resists insertion.
    shapes = [parts.box((.015875, PITCH - .0002, .0127), (0., 0., 0.), BLACK),
              parts.box((DEPTH, PITCH - .0002, .0015), (.0079375 + DEPTH / 2, 0., 0.), BLACK)]
    shapes.append(parts.Shape('wedge',(.0035,PITCH-.0002,.0036),(.01375,0.,0.),BLACK))
    SEALS.append(parts.Part(f'seal_{i:02}', pos=(START_X, -LENGTH / 2 + (i + .5) * PITCH + SHIFT, SEAL_Z + .0002),
        mass=.045 / N, shapes=shapes, friction=(.7, .55)))
seal_physics.SEGMENTS[:] = [s.name for s in SEALS]

BLOCK = parts.Part('smooth_wood_block', pos=(.175, -.19, TOP + .010), mass=.025,
    shapes=[parts.box((.040, .030, .020), (0., 0., 0.), (.72, .51, .28))], friction=(.6, .5))
OBJECTS = [DECK, JAMB, *CLAMPS, *SEALS, BLOCK]
# All connected objects keep one start pose. Only the rigid pressing block moves.
LOTS = lots.declare(OBJECTS,
    lots.Lot('A', place={BLOCK.name: BLOCK.pos}),
    lots.Lot('B', place={BLOCK.name: (.175, -.23, TOP + .010)}))


def _rotation(q):
    q = np.asarray(q, float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1.) > .01:
        raise ValueError('Unknown rotation')
    return parts._quat_to_matrix(q)


def measure(objects,with_flags=False):
    centers, rotations = [], []
    block = objects[BLOCK.name]
    if not np.isfinite(np.r_[block.pos, block.lin_vel, block.ang_vel]).all():
        raise ValueError('Unknown block state')
    _rotation(block.quat_xyzw)
    for spec in SEALS:
        st = objects[spec.name]
        if not np.isfinite(np.r_[st.pos, st.lin_vel, st.ang_vel]).all():
            raise ValueError('Unknown seal state')
        centers.append(np.asarray(st.pos))
        rotations.append(_rotation(st.quat_xyzw))
    centers = np.array(centers)
    r = np.array(rotations)
    # Both ends of every span must be home: no credit for diagonal tip crossing.
    corners = np.array([[.0079375 + DEPTH, y, z] for y in (-PITCH / 2, PITCH / 2) for z in (-.00075, .00075)])
    tips = np.einsum('nij,kj->nki', r, corners) + centers[:, None, :]
    front_corners = np.array([[.0079375, y, z] for y in (-PITCH / 2, PITCH / 2) for z in (-.00635, .00635)])
    bulb_fronts = np.einsum('nij,kj->nki',r,front_corners) + centers[:,None,:]
    seated = ((tips[:, :, 0].min(axis=1) >= FACE + DEPTH - .0015) &
              ((FACE-bulb_fronts[:,:,0]).max(axis=1) <= .0015) &
              (tips[:, :, 0].max(axis=1) <= FACE + DEPTH + .0005) &
              (np.abs(tips[:, :, 2] - SEAL_Z).max(axis=1) <= KERF / 2 + .0001) &
              (tips[:, :, 1].min(axis=1) >= -LENGTH / 2 - .002) &
              (tips[:, :, 1].max(axis=1) <= LENGTH / 2 + .002))
    ends = np.array([centers[0] - r[0] @ np.array([0., PITCH / 2, 0.]),
                     centers[-1] + r[-1] @ np.array([0., PITCH / 2, 0.])])
    flush = abs(ends[0, 1] + LENGTH / 2) <= .002 and abs(ends[1, 1] - LENGTH / 2) <= .002
    gaps = [np.linalg.norm(centers[i] + r[i] @ np.array([0., PITCH / 2, 0.]) -
                           centers[i + 1] + r[i + 1] @ np.array([0., PITCH / 2, 0.])) for i in range(N - 1)]
    fin_gaps=[np.linalg.norm(centers[i]+r[i]@np.array([x,PITCH/2,0.])-
                            centers[i+1]-r[i+1]@np.array([x,-PITCH/2,0.]))
              for i in range(N-1) for x in (.0079375,.0079375+DEPTH)]
    intact = max([*gaps,*fin_gaps]) <= .003
    still = all(objects[s.name].resting(.003, .05) for s in [*SEALS, BLOCK])
    return (seated if with_flags else int(seated.sum())), bool(flush), bool(intact), bool(still)


def tier(count, flush, released, still, held_s, intact=True, known=True):
    """Visible score contract; used independently of the physics implementation."""
    if not known or not intact or not released or not still:
        return 0.
    if count == N and flush and held_s >= 1. - 1e-8:
        return 1.
    if count >= 31:
        return .5
    if count >= 9:
        return .25
    return 0.


_sensor=None
_state = dict(known=False, invalid=False, damaged=False, held=0., time=None, certificate=None, clear=False, eligible=False,
              force_flags=np.zeros(N,bool))


def _stamp(objects):
    return tuple(float(v) for spec in [*SEALS, BLOCK] for v in np.r_[objects[spec.name].pos,
        objects[spec.name].quat_xyzw, objects[spec.name].lin_vel, objects[spec.name].ang_vel])


def faces_clear(objects, clouds):
    if len(clouds) != 2 or any(c.ndim != 2 or c.shape[1] != 3 or not len(c) or not np.isfinite(c).all() for c in clouds):
        return False
    bounds = [(c.min(0),c.max(0)) for c in clouds]
    for spec in [*SEALS, BLOCK]:
        st = objects[spec.name]
        r = _rotation(st.quat_xyzw)
        for _, center, rotation, size in spec.world_boxes({'base': (st.pos, r)}):
            # 3 mm face sampling plus 1.5 mm voxel thinning has at most
            # 5.6 mm covering error; a 6 mm guard certifies 10 mm clearance.
            extent = np.abs(rotation) @ (size/2) + .016
            for cloud, (lo,hi) in zip(clouds,bounds):
                if np.any(hi < center-extent) or np.any(lo > center+extent):
                    continue
                candidates=cloud[np.all((cloud>=center-extent)&(cloud<=center+extent),axis=1)]
                if not len(candidates):
                    continue
                distances = np.linalg.norm(np.maximum(np.abs((candidates - center) @ rotation) - size / 2, 0.), axis=1)
                if distances.min() <= .016:
                    return False
    return True


def tool_clear(objects):
    from geometry import box_distance
    block = objects[BLOCK.name]
    if not np.isfinite(np.r_[block.pos, block.lin_vel, block.ang_vel]).all():
        return False
    rotation = _rotation(block.quat_xyzw)
    size_block=np.array(BLOCK.size)
    block_extent=np.abs(rotation)@(size_block/2)
    for spec in SEALS:
        st = objects[spec.name]
        for _, center, r, size in spec.world_boxes({'base': (st.pos, _rotation(st.quat_xyzw))}):
            if np.linalg.norm(np.maximum(np.abs(block.pos-center)-block_extent-np.abs(r)@(size/2),0.)) > .010:
                continue
            if box_distance(block.pos,rotation,size_block,center,r,size) < .010:
                return False
    return True


@parts.on_reset
def reset(sim):
    global _sensor
    from measurement import require_calibration
    record=require_calibration()
    from isaaclab.app.settings_manager import get_settings_manager
    get_settings_manager().set_bool('/physics/disableContactProcessing',False)
    import force
    from types import SimpleNamespace
    geometry=SimpleNamespace(SEALS=SEALS,N=N,DEPTH=DEPTH,FACE=FACE,SEAL_Z=SEAL_Z,KERF=KERF)
    _sensor=force.install(sim,geometry,float(record['estimated_insertion_force_N']))
    _state.update(known=False, invalid=False, damaged=False, held=0., time=None, certificate=None, clear=False, eligible=False,
                  force_flags=np.zeros(N,bool))


@parts.on_step
def step(sim):
    from release import hand_clouds
    objects = sim.states()
    try:
        flags, flush, intact, still = measure(objects,True)
        if _sensor is None or not _sensor.observed or _sensor.error:raise ValueError('Unknown insertion force')
        _state['force_flags']=_sensor.certified.copy()
        count=int((flags&_state['force_flags']).sum())
        clouds = hand_clouds(sim)
        if len(clouds) != 2 or any(c.ndim != 2 or c.shape[1] != 3 or not len(c) or not np.isfinite(c).all() for c in clouds):
            raise ValueError('Unknown hand observations')
        clear = faces_clear(objects, clouds) and tool_clear(objects)
        t = float(sim.t)
        prev = _state['time']
        dt = 0. if prev is None else t - prev
        if prev is not None and not 0 < dt <= .100001:
            raise ValueError('Non-monotonic time')
        _state['damaged'] |= not intact or _sensor.damaged
        eligible = count == N and flush and intact and still and clear and not _state['damaged'] and not _state['invalid']
        _state['held'] = _state['held'] + dt if eligible and _state['eligible'] else 0.
        _state.update(known=True, time=t, clear=clear, certificate=_stamp(objects), eligible=eligible)
    except (KeyError, ValueError, AttributeError, TypeError):
        _state.update(known=False, invalid=True, held=0., clear=False, certificate=None, eligible=False)


def score(objects):
    try:
        if not _state['known'] or _state['invalid'] or _state['certificate'] != _stamp(objects):
            return 0.
        flags, flush, intact, still = measure(objects,True)
        count=int((flags&_state['force_flags']).sum())
        return tier(count, flush, _state['clear'] and tool_clear(objects), still, _state['held'], intact and not _state['damaged'])
    except (KeyError, ValueError, AttributeError, TypeError):
        return 0.


def success(objects):
    return score(objects) == 1.


def demo():
    """Open-loop plan from the published visible layout; no state feedback."""
    plans={};grip_z=FLOOR+.0237;zup=WORK+.060
    for arm,sign in [('left',1),('right',-1)]:
        yaw=-sign*math.pi/2;y=sign*.23+SHIFT;drag_y=y-SHIFT+.029
        plans[arm]=[Waypoint(2.,(START_X,y,TOP+.11),FINGER_CLOSED,'approach bulb',yaw),
          Waypoint(1.,(START_X,y,grip_z),FINGER_CLOSED,'press onto bulb',yaw),
          Waypoint(1.,(START_X,y,grip_z),FINGER_CLOSED,'settle pressure',yaw),
          Waypoint(2.,(START_X,drag_y,grip_z),FINGER_CLOSED,'drag ends into alignment',yaw),
          Waypoint(.3,(START_X,drag_y,grip_z),FINGER_OPEN,'release shallow top contact',yaw),
          Waypoint(.4,(START_X,drag_y,grip_z+.025),FINGER_OPEN,'lift clear',yaw),
          Waypoint(.3,(START_X,sign*.23,grip_z+.025),FINGER_CLOSED,'close clear of seal',yaw)]
        if arm=='left':
            plans[arm]+=[Waypoint(.7,(START_X,sign*.23,grip_z),FINGER_CLOSED,'steady loose half',yaw)]
        else:
            plans[arm]+=[Waypoint(.7,(START_X-.013,-LENGTH/2+PITCH/2,zup),FINGER_CLOSED,'behind first span',yaw)]
    for arm,indices in [('right',range(17)),('left',range(17,34))]:
        other='left' if arm=='right' else 'right';plan=plans[arm]
        yaw=-math.pi/2 if arm=='left' else math.pi/2
        if arm=='left':
            last=plan[-1].pos
            plan += [Waypoint(.3,last,FINGER_OPEN,'release shallow holding contact',yaw),
                     Waypoint(.4,(last[0],last[1],last[2]+.025),FINGER_OPEN,'lift holding hand clear',yaw),
                     Waypoint(.3,(last[0],last[1],last[2]+.025),FINGER_CLOSED,'close clear',yaw)]
        for i in indices:
            y=-LENGTH/2+(i+.5)*PITCH;back=START_X-.0079375-.020-.008;end=HOME-.0079375-.020+.070
            plan += [Waypoint(.5,(back,y,zup),FINGER_CLOSED,'over next span',yaw),
                     Waypoint(.3,(back,y,FLOOR+.016),FINGER_CLOSED,'lower behind bulb',yaw),
                     Waypoint(.65,(end,y,FLOOR+.016),FINGER_CLOSED,'press span',yaw),
                     Waypoint(.55,(end,y,FLOOR+.016),FINGER_CLOSED,'hold insertion load',yaw),
                     Waypoint(.35,(end,y,zup),FINGER_CLOSED,'lift before retreat',yaw),
                     Waypoint(.25,(back,y,zup),FINGER_CLOSED,'retreat above seal',yaw)]
        sign=-1 if arm=='right' else 1
        plan += [Waypoint(1.,(.13,sign*.27,TOP+.14),FINGER_OPEN,'withdraw',0.)]
        wait=sum(w.duration for w in plan)-sum(w.duration for w in plans[other])
        plans[other].append(Waypoint(wait,plans[other][-1].pos,plans[other][-1].finger,'wait or steady loose half',plans[other][-1].yaw))
    total=max(sum(w.duration for w in p) for p in plans.values())
    for arm,p in plans.items():
        sign=-1 if arm=='right' else 1
        p.append(Waypoint(total-sum(w.duration for w in p)+2.,(.13,sign*.27,TOP+.14),FINGER_OPEN,'hands clear',0.))
    return plans
