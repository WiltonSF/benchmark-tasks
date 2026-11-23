"""An open Hammond 1591ESBK enclosure is clamped to the bench. Its near wall
contains a loose HUMMEL HSK-K M20 cable gland. A prepared 10 mm cable rests on
two open supports in front of it. A 24 mm open-end spanner with a removable
grasp sleeve rests on the right. The green witness band marks the snug range.
"""
from __future__ import annotations
import json
import math
import os
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts
import lots
import tilt
import observation
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint, FINGER_OPEN, grip_rotation

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = .1
SCORE_OVER = 'end'
GREY=(.65,.67,.70); BLACK=(.12,.13,.15); STEEL=(.67,.70,.73)
GREEN=(.12,.68,.23); ORANGE=(.95,.42,.08)
AXIS=np.array((.280, .015, TOP+.035))
CAP_X=-.026
LEVER=.165
TORQUE=4.5
BRAKE_K=180.
YIELD=TORQUE/BRAKE_K
FIRST_CONTACT=.75*2*math.pi
SNUG=(1.25*2*math.pi, 1.55*2*math.pi)
DAMAGE=1.75*2*math.pi

def ring(inner, outer, length, x, color, n=12, phase=0.):
    """Tangent slabs leave an actual open, approximately round bore."""
    shapes=[]
    for i in range(n):
        a=i*2*math.pi/n+phase
        r=(inner+outer)/2
        rot=parts._rot((1,0,0),a)
        shapes.append(parts.box((length,outer-inner,2*outer*math.tan(math.pi/n)),
            (x,r*math.cos(a),r*math.sin(a)),color,rot=rot))
    return shapes

def hex_ring(x, width=.024, length=.009):
    # The opening remains hollow: six 60-degree wall segments provide real flats.
    return ring(.0068,width/2,length,x,GREY,n=6,phase=math.pi/4)

# Full enclosure dimensions; 2.4 mm walls follow the maker drawing. The front
# wall has a genuine 20.5 mm opening around the gland, not a solid collision box.
walls=[parts.box((.110,.1904,.0024),(.055,0,-.0338),BLACK),
       parts.box((.0024,.1904,.0584),(.1088,0,-.0046),BLACK),
       parts.box((.110,.0024,.0584),(.055,-.094,-.0046),BLACK),
       parts.box((.110,.0024,.0584),(.055,.094,-.0046),BLACK),
       parts.box((.0024,.1904,.020),(.0012,0,-.025),BLACK),
       parts.box((.0024,.1904,.0108),(.0012,0,.0204),BLACK),
       parts.box((.0024,.070,.0476),(.0012,-.0602,0),BLACK),
       parts.box((.0024,.070,.0476),(.0012,.0602,0),BLACK)]
walls+=ring(.01025,.025,.0024,.0012,BLACK)
ENCLOSURE=parts.Part('enclosure',pos=tuple(AXIS),shapes=walls,friction=(.4,.35),contact_offset=.0005)

# The cap turns and advances at the actual M20 lead. Four radial sealing pads
# are coupled to the same coordinate, a declared cone approximation. No code
# writes poses, adds a grasp weld, or locks the cable after insertion.
cap=parts.Hinge('cap',shapes=[parts.box((.002,.002,.002),(CAP_X,0,0),GREY,collide=False)],
    pivot=(CAP_X,0,0),axis=(1,0,0),lower=-.1,upper=2*math.pi*2.,mass=.006,
    spring=BRAKE_K,damping=.04,max_effort=TORQUE)
nut_shapes=hex_ring(CAP_X)+[parts.box((.0003,.012,.002),(CAP_X-.0046,.017,0),(.12,.32,.95),collide=False)]
nut=parts.Slider('nut',shapes=nut_shapes,axis=(1,0,0),travel=.0032,back=.0001,
    mass=.010,damping=.01,parent='cap',thread=('cap',.0015))
seals=[]
for i,(dy,dz) in enumerate(((1,0),(-1,0),(0,1),(0,-1))):
    at=(-.010,dy*.008,dz*.008)
    shape=parts.box((.010,.003 if dy else .008,.003 if dz else .008),at,BLACK)
    seals.append(parts.Slider('seal'+str(i),shapes=[shape],axis=(0,-dy,-dz),
        travel=.004,mass=.006,damping=.2,thread=('cap',.002)))
GLAND=parts.Part('gland',pos=tuple(AXIS),shapes=ring(.0065,.012,.012,-.007,GREY),
    joints=[cap,nut,*seals],friction=(.4,.35),contact_offset=.00025,
    link_materials={f'seal{i}':(.8,.75,50000.,35.) for i in range(4)},self_collision=False)

# A short straight cable specimen: the scored end is sheathed, with the last
# 20 mm represented by three insulated cores. The straight segment is a declared
# rigid proxy; cable bending calibration is outstanding, not measured evidence.
CABLE=parts.Part('cable',pos=(.115,.015,TOP+.035),mass=.0155,
    shapes=[parts.cylinder(.005,.100,(.040,0,0),ORANGE,axis='x'),
            *[parts.cylinder(.0018,.020,(.100,dy,dz),c,axis='x')
              for dy,dz,c in ((-.002,0,(.22,.25,.65)),(.002,0,(.36,.21,.08)),(0,.002,(.25,.65,.15)))],
            parts.box((.008,.0105,.001),(.065,0,.005),GREEN,collide=False)],
    joints=[parts.Hinge('tail_near',shapes=[parts.cylinder(.005,.050,(-.035,0,0),ORANGE,axis='x')],
                pivot=(-.010,0,0),axis=(0,0,1),lower=-1.5,upper=1.5,mass=.00775,spring=.12,damping=.02),
            parts.Hinge('tail_far',shapes=[parts.cylinder(.005,.050,(-.085,0,0),ORANGE,axis='x')],
                pivot=(-.060,0,0),axis=(0,0,1),lower=-1.5,upper=1.5,mass=.00775,spring=.12,damping=.02,parent='tail_near')],
    friction=(.4,.35),contact_offset=.00025,self_collision=False)

# GEDORE 894 24 / 6575570, nominal 217 mm (maker total 215 mm), 24 mm
# jaw, 15-degree jaw set, 9 mm head thickness, 154 g before the grasp sleeve.
HEAD_ANGLE=math.pi/12
HEAD_ROT=parts._rot((1,0,0),HEAD_ANGLE)
head=[]
for size,at in [((.009,.029,.013),(0,.005,.0187)),
                ((.009,.029,.013),(0,.005,-.0187)),
                ((.009,.012,.0494),(0,.025,0))]:
    head.append(parts.box(size,HEAD_ROT@np.array(at),STEEL,rot=HEAD_ROT))
# y minimum of the angled jaw is -15.7 mm; this 199.3 mm handle end
# preserves the maker's 215 mm overall length without a second wrench head.
SPANNER=parts.Part('spanner',pos=(.205,.050,TOP+.035),mass=.204,
    shapes=[parts.box((.009,.1768,.016),(0,.1109,0),STEEL),*head,
            parts.box((.024,.060,.050),(0,LEVER,0),ORANGE)],
    friction=(.7,.6),contact_offset=.0003)

def support(name,x,y,z):
    return parts.Part(name,pos=(x,y,TOP),shapes=[
        parts.box((.018,.035,z-.006),(0,0,(z-.006)/2),GREY),
        parts.box((.018,.010,.015),(0,-.0125,z-.0065),GREY),
        parts.box((.018,.010,.015),(0,.0125,z-.0065),GREY)],contact_offset=.00025)
SUPPORTS=[support('cable_support_rear',.025,.015,.035),support('cable_support_front',.195,.015,.035)]
CLAMPS=[]
for side in (-1,1):
    CLAMPS.append(parts.Part('bench_clamp_'+str(side).replace('-','m'),pos=(.355,.015+side*.115,TOP),
        shapes=[parts.box((.050,.025,.015),(0,0,.0075),STEEL),
                parts.box((.025,.020,.070),(0,0,.040),STEEL),
                parts.box((.045,.035,.012),(-.01,-side*.020,.065),BLACK)],contact_offset=.0005))
# Tool sleeve sits on this cradle. It does not hold, turn, or couple the head.
CRADLE=parts.Part('tool_cradle',pos=(.205,.200,TOP),shapes=[
    parts.box((.050,.075,.010),(0,0,.005),GREY)],contact_offset=.0003)
HEAD_REST=parts.Part('tool_head_rest',pos=(.195,.050,TOP+.0041),
    shapes=[parts.box((.060,.040,.0082),(0,0,0),GREY)],contact_offset=.0002)
PARK=parts.Part('tool_parking_pad',pos=(.13,.21,TOP+.001),
    shapes=[parts.box((.09,.16,.002),(0,0,0),GREEN)],contact_offset=.00025)
marks=[]
for i in range(36):
    a=i*math.pi/18
    # Green 90--198 degrees repeats after one revolution; video also counts
    # the completed revolution. Red starts at 270 degrees on the second turn.
    color=GREEN if 9<=i<=19 else ((.85,.10,.08) if i>=27 else GREY)
    marks.append(parts.box((.0004,.004,.003),(CAP_X-.006,.025*math.cos(a),.025*math.sin(a)),
        color,rot=parts._rot((1,0,0),a),collide=False))
DIAL=parts.Part('witness_dial',pos=tuple(AXIS),shapes=marks,contact_offset=.0001)
OBJECTS=[ENCLOSURE,GLAND,CABLE,SPANNER,*SUPPORTS,*CLAMPS,CRADLE,HEAD_REST,PARK,DIAL]
LOTS=lots.declare(OBJECTS,
    lots.Lot('A',place={SPANNER.name:SPANNER.pos}),
    lots.Lot('B',place={SPANNER.name:(.185,.050,TOP+.035)}))

def inserted(s):
    r=parts._quat_to_matrix(s.quat_xyzw)
    end=s.pos+r@np.array((.090,0,0))
    start=s.pos+r@np.array((-.010,0,0))
    # Test the cable where it actually traverses both ends of the body bore,
    # rather than constraining the free sheath end farther inside the box.
    if not end[0]>=AXIS[0]+.014 or r[0,0]<.5 or start[0]>AXIS[0]-.013:return False
    for x in (AXIS[0]-.013,AXIS[0]+.009):
        at=np.asarray(s.pos)+r[:,0]*((x-s.pos[0])/r[0,0])
        if np.linalg.norm((at-AXIS)[1:])>.0065:return False
    return True

def released(clouds,states):
    if not clouds or any(len(c)==0 or not np.isfinite(c).all() for c in clouds):return False
    for spec in (CABLE,SPANNER,GLAND):
        s=states[spec.name]
        poses={'base':(s.pos,parts._quat_to_matrix(s.quat_xyzw))}
        poses.update({n:(p,parts._quat_to_matrix(q)) for n,(p,q) in getattr(s,'links',{}).items()})
        for _,c,r,size in spec.world_boxes(poses):
            if any(np.min(np.linalg.norm(np.maximum(np.abs((cl-c)@r)-size/2,0),axis=1))<=.008 for cl in clouds):return False
    return True

_known=False;_invalid=False;_damage=False;_anchor=0.;_held=0.;_last_t=0.
_q=0.;_work=0.;_last_q=0.;_peak_finger=0.;_clear=False;_pull_ok=False
_cert=None;_pull_start=None;_pull_time=0.;_pull_displacement=0.
_peak_pull=0.;_parked=False
_previous_joints=None
_pull_q=0.

def snapshot(objects):
    values=[]
    for spec in (CABLE,GLAND,SPANNER):
        s=objects[spec.name]
        p=np.asarray(s.pos);q=np.asarray(s.quat_xyzw)
        if p.shape!=(3,) or q.shape!=(4,) or abs(np.linalg.norm(q)-1.)>.01:raise ValueError('unknown pose')
        values.extend(np.r_[p,q,s.lin_vel,s.ang_vel])
        values.extend(float(s.joints[j.name]) for j in spec.joints)
    if not np.isfinite(values).all():raise ValueError('unknown task state')
    return tuple(values)

def cable_force(sim, force):
    """Independent operator pull; the installed gland takes the contact reaction."""
    import torch
    asset=sim.env.unwrapped.scene[CABLE.name]
    vector=torch.tensor([[[float(force),0.,0.]]],device=sim.env.unwrapped.device)
    torque=torch.zeros_like(vector)
    # Isaac Lab 6's native rigid body external-wrench buffer is applied at each
    # physics substep until replaced. No velocity or pose is ever prescribed.
    body=asset.body_names.index('base')
    asset.set_external_force_and_torque(vector,torque,body_ids=[body],is_global=True)

@parts.on_reset
def reset(sim):
    global _known,_invalid,_damage,_anchor,_held,_last_t,_q,_work,_last_q,_peak_finger,_clear,_pull_ok,_cert
    global _pull_start,_pull_time,_pull_displacement,_peak_pull,_parked
    global _previous_joints
    global _pull_q
    _known=False;_invalid=False;_damage=False;_anchor=0.;_held=0.;_last_t=0.
    _q=0.;_work=0.;_last_q=0.;_peak_finger=0.;_clear=False;_pull_ok=False;_cert=None
    _pull_start=None;_pull_time=0.;_pull_displacement=0.;_peak_pull=0.;_parked=False
    _previous_joints=None
    _pull_q=0.
    cable_force(sim,0.)
    observation.reset()
    from isaaclab.app.settings_manager import get_settings_manager
    get_settings_manager().set_bool('/physics/disableContactProcessing',False)
    for arm in ('left_robot','right_robot'):
        robot=sim.env.unwrapped.scene[arm]
        ids=robot.find_joints(['joint7','joint8'],preserve_order=True)[0]
        robot.write_joint_effort_limit_to_sim(40.,joint_ids=ids)

@parts.on_step
def update(sim):
    global _known,_invalid,_damage,_anchor,_held,_last_t,_q,_work,_last_q,_peak_finger,_clear,_pull_ok,_cert
    global _pull_start,_pull_time,_pull_displacement,_peak_pull,_parked
    global _previous_joints
    global _pull_q
    states=sim.states();s=states[CABLE.name];g=states[GLAND.name]
    dt=float(sim.t)-_last_t;_last_t=float(sim.t)
    try:
        _q=float(g.joints['cap'])
        clouds,force=observation.hand_state(sim)
        _known=np.isfinite(_q) and 0<dt<=.10001
        _peak_finger=max(_peak_finger,force)
        _clear=released(clouds,states)
        _damage |= _q>=DAMAGE
        # An elasto-plastic friction brake. Its rest point only advances after
        # the 4.5 Nm yield load has been exceeded. The drive never pulls the cap
        # into tighter positions; loading and reaction are native joint forces.
        load=min(TORQUE,max(0.,BRAKE_K*(_q-_anchor)))
        if _q>_last_q and _q>FIRST_CONTACT:
            _work+=max(0.,_q-max(_last_q,FIRST_CONTACT))*load
        _anchor=max(_anchor,_q-YIELD)
        sim.set_target(GLAND.name,'cap',_anchor)
        positions=np.array([*s.joints.values(),*g.joints.values()])
        joint_still=_previous_joints is not None and np.max(np.abs(positions-_previous_joints))<=.03*dt
        still=s.resting(.003,.03) and states[SPANNER.name].resting(.003,.03) and joint_still
        _previous_joints=positions.copy()
        _held=_held+dt if _known and _clear and still else 0.
        _last_q=_q
        # Retention test is a physical operator force, never a pose correction.
        # It runs only after release and repeats if cable/nut move afterwards.
        park=np.asarray(lots.current().pos(SPANNER.name))
        _parked=np.linalg.norm(states[SPANNER.name].pos-park)<.035 and states[SPANNER.name].resting(.003,.03)
        if _pull_ok and (_pull_start is None or np.linalg.norm(s.pos-_pull_start)>=.002 or abs(_q-_pull_q)>.02):
            _pull_ok=False;_pull_start=None;_pull_time=0.;_peak_pull=0.
        if not _clear or not SNUG[0]<=_q<SNUG[1] or not inserted(s):
            _pull_ok=False;_pull_start=None;_pull_time=0.
            cable_force(sim,0.)
        elif (_held>=1. or _pull_start is not None) and not _pull_ok:
            if _pull_start is None:
                _pull_start=np.asarray(s.pos).copy();_pull_time=0.;_pull_displacement=0.;_pull_q=_q;_peak_pull=0.
            _pull_time+=dt
            _pull_displacement=max(_pull_displacement,float(np.linalg.norm(s.pos-_pull_start)))
            force=50.*min(1.,_pull_time)
            _peak_pull=max(_peak_pull,force)
            cable_force(sim,-force if _pull_time<2. else 0.)
            if _pull_time>=2.:
                _pull_ok=_pull_displacement<.002 and _peak_pull>=50.-1e-6
                if not _pull_ok:_invalid=True
        else:cable_force(sim,0.)
        _cert=snapshot(states)
    except (ValueError,KeyError,AttributeError,IndexError):
        _known=False;_invalid=True;_clear=False
    if os.environ.get('TRACE_OUT'):
        row=dict(t=float(sim.t),lot=lots.current().name,q=_q,turns=_q/(2*math.pi),work=_work,
            inserted=inserted(s),cable=s.pos.tolist(),known=bool(_known),invalid=bool(_invalid),
            clear=bool(_clear),held=_held,damage=bool(_damage),peak_finger=_peak_finger,
            pull_ok=bool(_pull_ok),pull_time=_pull_time,pull_displacement=_pull_displacement,peak_pull=_peak_pull,
            parked=bool(_parked),brake_load=load,brake_anchor=_anchor,
            spanner=states[SPANNER.name].pos.tolist(),score=score(states))
        try:
            asset=sim.env.unwrapped.scene[GLAND.name]
            raw=asset.root_view.get_dof_projected_joint_forces()
            raw=raw.torch if hasattr(raw,'torch') else raw
            effort=np.asarray(raw.detach().cpu().numpy() if hasattr(raw,'detach') else raw.numpy())[0]
            row['native_cap_effort']=float(effort[asset.joint_names.index('cap')])
        except (ValueError,AttributeError,IndexError) as e:
            row['native_cap_effort']=None;row['native_cap_effort_error']=str(e)
        row['hands']={}
        from yam_arena.constants import TCP_OFFSET
        for arm in ('left_robot','right_robot'):
            robot=sim.env.unwrapped.scene[arm]
            pose=parts._np(robot.data.body_link_pose_w)[0,robot.body_names.index('gripper')]
            origin=parts._np(sim.env.unwrapped.scene.env_origins[0])
            row['hands'][arm]=dict(tcp=(pose[:3]-origin+parts._quat_to_matrix(pose[3:])@np.array(TCP_OFFSET)).tolist(),quat=pose[3:].tolist())
        with pathlib.Path(os.environ['TRACE_OUT']).with_name('gland.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')

def score(objects):
    try:
        s=objects[CABLE.name];g=objects[GLAND.name]
        cert=snapshot(objects)
        if not _known or _invalid or _damage or cert!=_cert or not inserted(s):return 0.
        # Release and one second of stillness gate every endpoint tier.
        if not _clear or _held<1.:return 0.
        if _q<FIRST_CONTACT:return .25
        if _q<SNUG[0]:return .5
        if SNUG[0]<=_q<SNUG[1] and _work>=TORQUE*(SNUG[0]-FIRST_CONTACT)-.15:
            return 1. if _pull_ok and _parked else .75
        return 0.
    except (ValueError,KeyError,AttributeError,TypeError):return 0.

def success(objects):return score(objects)==1.

def demo():
    """Open-loop published layout coordinates; no simulated state feedback."""
    pick=np.array(lots.current().pos(SPANNER.name))+np.array((0,LEVER,.008))
    # The shifted layout needs an 8 mm offset within the 24 mm sleeve to keep
    # the pickup and return inside the wrist's joint limit.
    if lots.current().name == 'B':
        pick += np.array((.008,0,0))
    # Constant measured approach correction for the loaded arm; it is part of
    # the open-loop motion and does not query the simulated pose during demo.
    correction=np.array((.008,-.013,.011))
    tcp=np.array((AXIS[0]+CAP_X,AXIS[1]+LEVER,AXIS[2]+.008))+correction
    rot=parts._rot((1,0,0),-math.pi/3)@grip_rotation(-math.pi/2)
    def w(t,p,f,label,a=0.):
        return Waypoint(t,tuple(float(v) for v in p),f,label,
            tilt._TiltedYaw(0,parts._rot((1,0,0),a)@rot))
    right=[w(2,pick+(0,0,.075),FINGER_OPEN,'approach spanner'),
        w(2,pick,FINGER_OPEN,'lower'),w(1.5,pick,0.,'grasp sleeve'),
        w(2,pick+(0,0,.035),0.,'lift from cradle')]
    left_pick=np.array(CABLE.pos)+np.array((-.055,0,.008))
    left_goal=left_pick+np.array((.105,0,0))
    left=[Waypoint(2,tuple(left_pick+(0,0,.07)),FINGER_OPEN,'approach cable',0.),
        Waypoint(2,tuple(left_pick),FINGER_OPEN,'lower to sheath',0.),
        Waypoint(1.5,tuple(left_pick),0.,'grasp sheath',0.),
        Waypoint(3,tuple(left_goal),0.,'feed through gland',0.)]
    # Nine 60-degree strokes. Tool withdrawal and reseating occur with real
    # collision geometry. Symmetry of the hex permits the next 60-degree bite.
    angle=math.pi/3
    for i in range(9):
        shift=np.array((.0015*i/6,0,0))
        a=tcp+shift
        b=AXIS+np.array((CAP_X+.0015*(i+1)/6,0,0))+parts._rot((1,0,0),angle)@np.array((0,LEVER,.008))+correction
        right += [w(1.5,a+HEAD_ROT@np.array((0,.045,0)),0.,'approach jaw'),
            w(1.5,a,0.,'seat jaw on flats'),
            w(3.5,b,0.,'tighten against 4.5 Nm',angle),
            w(1.5,b+np.array((-.020,0,0)),0.,'withdraw jaw axially',angle),
            w(2,a+HEAD_ROT@np.array((0,.045,0)),0.,'return spanner')]
    elapsed=sum(x.duration for x in right)
    left.append(Waypoint(elapsed-sum(x.duration for x in left),tuple(left_goal),0.,'hold cable against seal twist',0.))
    left += [Waypoint(1.5,tuple(left_goal),FINGER_OPEN,'release cable',0.),
        Waypoint(2,tuple(left_goal+(0,-.070,.08)),FINGER_OPEN,'hands clear',0.),
        Waypoint(15,tuple(left_goal+(0,-.070,.08)),FINGER_OPEN,'wait for independent pull',0.)]
    right += [w(2,pick+(0,0,.035),0.,'return to cradle'),w(2,pick,0.,'lower tool'),
        w(1.5,pick,FINGER_OPEN,'release spanner'),w(2,pick+(0,0,.08),FINGER_OPEN,'clear'),
        w(10,pick+(0,0,.08),FINGER_OPEN,'stillness')]
    return {'right':left,'left':right}
