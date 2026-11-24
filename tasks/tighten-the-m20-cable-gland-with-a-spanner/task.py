"""An open Hammond 1591ESBK enclosure is clamped to the bench. Its near wall
carries a HUMMEL HSK-K M20 cable gland whose cap has been run down by hand to
the reset mark. A prepared 10 mm cable with a green sheath mark rests on two
open supports in front of it. A 24 mm open-end spanner with a grasp sleeve
rests on the right. The dial on the cap shows first contact, the green snug
sector and the red over-torque sector.
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
from yam_arena.demo import Waypoint, FINGER_OPEN

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = .1
SCORE_OVER = 'end'

# ---------------------------------------------------------------------------
# Calibration constants. Each one is a single value so that a reading taken on
# the named gland and cable can replace it without changing anything else.
# Sources: CAP_TORQUE_NM maker datasheet; FREE_DRAG_NM, SNUG_DEG and
# RETENTION_N technician phone figures (Nov 2026), recorded in README.
# Labelled estimates (derivations in README): everything marked ESTIMATE.
CAP_TORQUE_NM = 4.5      # HUMMEL cap installation torque, M20 6-12 mm (maker page, VDE 40012033)
SNUG_LOAD_HOLD_S = 1.0  # benchmark sustained tool-contact load certificate
TORQUE_TOL_NM = 0.0      # no credit below maker torque
DAMAGE_TORQUE_NM = 5.0   # benchmark red line: round(1.1*4.5, 1); no measured damage onset
FREE_DRAG_NM = 0.3       # technician phone figure (Nov 2026), before seal touches sheath
FIRST_GRIP_PRELOAD_M = .00016  # estimated geometry: .41mm - 6mm*15/360 = .16mm; reset-only 10 N calibration: 45 deg fails and 60 deg passes; physical C remains unmeasured
FIRST_CONTACT_PULL_N = 10.  # benchmark first-grip force: one fifth of50N retention check
FIRST_CONTACT_SLIP_MM = 1.  # benchmark movement limit: half the2mm retention limit
FIRST_CONTACT_STEP_DEG = 15. # benchmark sampling: one sixth of90deg loaded turn
FIRST_CONTACT_PAIRS = 3     # physical enrollment uses the median of three stocked pairs
FIRST_CONTACT_DEG = 60.  # cap angle from the reset mark at which the seal first grips (reset is set from it)
SNUG_DEG = 90.           # technician phone figure (Nov 2026), first grip to 4.5 Nm
SEAL_EXPONENT = 3.       # ESTIMATE: stiffening (cubic) seal torque rise between contact and snug
RETENTION_N = 50.        # technician phone figure (Nov 2026), <=2 mm motion at 4.5 Nm
PULL_COUNT = 3           # pulls run in simulation (the standard specifies 50 one-second pulls)
PULL_HOLD_S = 1.         # each pull is held 1 s at RETENTION_N
PULL_SLIP_MM = 2.        # allowed cable displacement after the load is removed
PULL_WITNESS_FROM_SHEATH_END_MM = 90.  # physical reference at the rigid lead origin
MARK_FROM_SHEATH_END_MM = 50.  # centre of the green sheath mark, measured from the sheath end
GLAND_BORE_RADIUS_M = .0065  # estimated primitive bore radius, already used in gland geometry
CABLE_RADIUS_M = .005  # named 10 mm cable
MARK_TOL_MM = 3.         # mark centre must finish within this distance of the cap's outer face
# ---------------------------------------------------------------------------

GREY=(.65,.67,.70); BLACK=(.12,.13,.15); STEEL=(.67,.70,.73)
GREEN=(.12,.68,.23); ORANGE=(.95,.42,.08); RED=(.85,.10,.08); BLUE=(.12,.32,.95)
FIXTURE_RISER_M=.030     # reusable wood block under the clamped enclosure
FIXTURE_SHIFT_X_M=-.025  # move the enclosure/cable setup 25 mm toward the arms
AXIS=np.array((.280+FIXTURE_SHIFT_X_M, .015, TOP+.035+FIXTURE_RISER_M))
CAP_X=-.0225            # outer face -27 mm, inner body thread end +9 mm: nominal H+GL36
HEX_HALF_LEN=.0045
CAP_LEAD=.0015           # ESTIMATE cap axial lead, matching the published M20 mounting pitch; dome-cap thread unmeasured
SEAL_LEAD=.006           # modelled radial pad closure per cap turn (cone approximation)
SEAL_GAP=SEAL_LEAD*FIRST_CONTACT_DEG/360.-FIRST_GRIP_PRELOAD_M  # reset gap0.84mm
LEVER=.165  # sleeve centre; nominal working grasp is 5 mm toward the head, plus jaw-throat bias
BRAKE_K=180.             # Nm/rad elastic stiffness of the cap/thread before it slips

def cap_torque(deg):
    """Torque (Nm) needed to advance the cap at this angle from the reset mark."""
    if deg<FIRST_CONTACT_DEG:return FREE_DRAG_NM
    x=(deg-FIRST_CONTACT_DEG)/SNUG_DEG
    return FREE_DRAG_NM+(CAP_TORQUE_NM-FREE_DRAG_NM)*x**SEAL_EXPONENT

def angle_at(torque):
    """Cap angle (deg) at which cap_torque first reaches ``torque``."""
    return FIRST_CONTACT_DEG+SNUG_DEG*((torque-FREE_DRAG_NM)/(CAP_TORQUE_NM-FREE_DRAG_NM))**(1/SEAL_EXPONENT)

SNUG_LOW_DEG=angle_at(CAP_TORQUE_NM-TORQUE_TOL_NM)   # 150 degrees at the strict 4.5 Nm threshold
RED_DEG=angle_at(DAMAGE_TORQUE_NM)                  # about 153.44 degrees at the 5 Nm benchmark limit
WINDUP_DEG=math.degrees(DAMAGE_TORQUE_NM/BRAKE_K)    # largest elastic spring-back after unloading

def seal_work(a,b,n=200):
    """Work (J) to turn the cap from a to b degrees against cap_torque."""
    xs=np.linspace(a,b,n+1)
    return float(np.trapezoid([cap_torque(x) for x in xs],np.radians(xs)))

SNUG_WORK_J=seal_work(FIRST_CONTACT_DEG,SNUG_LOW_DEG)

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

def hex_ring(x, flats=.024, length=2*HEX_HALF_LEN, inner=.0068):
    """Sharp regular24mm hex collision idealization: six13.856mm flat slabs,
    27.713mm across corners. Exact HUMMEL moulded corners are unmeasured.
    Replaces the old six11.92mm flats plus six1.68mm corner slivers.
    """
    shapes=[]
    for i in range(6):
        a=i*math.pi/3+math.pi/4
        r=(inner+flats/2)/2
        shapes.append(parts.box((length,flats/2-inner,flats*math.tan(math.pi/6)),
            (x,r*math.cos(a),r*math.sin(a)),GREY,rot=parts._rot((1,0,0),a)))
    return shapes

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

# The cap turns and advances at the M20 lead. Its joint drive is a native
# elasto-plastic friction joint whose slip torque follows cap_torque(); the
# drive target only trails the cap and is updated each step (update()). Four
# radial NBR pads are coupled to the cap angle (a declared cone approximation)
# and grip the sheath by contact. No code writes poses, adds a weld, or locks
# the cable.
cap=parts.Hinge('cap',shapes=[parts.box((.002,.002,.002),(CAP_X,0,0),GREY,collide=False)],
    pivot=(CAP_X,0,0),axis=(1,0,0),lower=-.1,upper=math.radians(230.),mass=.006,
    spring=BRAKE_K,damping=.04,max_effort=FREE_DRAG_NM)
nut_shapes=hex_ring(CAP_X)+[parts.box((.0003,.012,.002),(CAP_X-HEX_HALF_LEN-.0016,.017,0),BLUE,collide=False)]
nut=parts.Slider('nut',shapes=nut_shapes,axis=(1,0,0),travel=.0032,back=.0001,
    mass=.010,damping=.01,parent='cap',thread=('cap',CAP_LEAD))
seals=[]
PAD_R=.005+SEAL_GAP+.0015
for i,(dy,dz) in enumerate(((1,0),(-1,0),(0,1),(0,-1))):
    wide=(.003 if dy else .008,.003 if dz else .008)
    flat=parts.box((.007,*wide),(-.0085,dy*PAD_R,dz*PAD_R),BLACK)
    # 5 mm lead-in on the cap side, tilted 15 degrees outward like the seal cone.
    tilt_axis=(0,-dz,dy)
    lead=parts.box((.005,*wide),(-.0145,dy*(PAD_R+.00067),dz*(PAD_R+.00067)),BLACK,
        rot=parts._rot(tilt_axis,math.radians(-15)))
    seals.append(parts.Slider('seal'+str(i),shapes=[flat,lead],axis=(0,-dy,-dz),
        travel=.004,mass=.006,damping=.2,thread=('cap',SEAL_LEAD)))
GLAND=parts.Part('gland',pos=tuple(AXIS),shapes=ring(GLAND_BORE_RADIUS_M,.012,.012,-.007,GREY),
    joints=[cap,nut,*seals],friction=(.4,.35),contact_offset=.00025,
    link_materials={f'seal{i}':(.8,.75,30000.,35.) for i in range(4)},self_collision=False)

# A short straight cable specimen: the scored end is sheathed, with the last
# 20 mm represented by three insulated cores. The 100 mm lead is rigid and the
# tail has two hinged 50 mm links (declared approximation of bending).
MARK_LOCAL_X=.090-MARK_FROM_SHEATH_END_MM/1000.
PULL_WITNESS_LOCAL_X=.090-PULL_WITNESS_FROM_SHEATH_END_MM/1000.
CABLE=parts.Part('cable',pos=(.115+FIXTURE_SHIFT_X_M,.015,TOP+.035+FIXTURE_RISER_M),mass=.0155,
    shapes=[parts.cylinder(CABLE_RADIUS_M,.100,(.040,0,0),ORANGE,axis='x'),
            *[parts.cylinder(.0018,.020,(.100,dy,dz),c,axis='x')
              for dy,dz,c in ((-.002,0,(.22,.25,.65)),(.002,0,(.36,.21,.08)),(0,.002,(.25,.65,.15)))],
            *[parts.box((.004,.0075,.001),(MARK_LOCAL_X,-.005*math.sin(a),.005*math.cos(a)),GREEN,collide=False,
              rot=parts._rot((1,0,0),a)) for a in (0.,math.pi/2,math.pi,3*math.pi/2)],
            *[parts.box((.001,.0075,.0004),(PULL_WITNESS_LOCAL_X,-.005*math.sin(a),.005*math.cos(a)),(.95,.95,.95),collide=False,
              rot=parts._rot((1,0,0),a)) for a in (0.,math.pi/2,math.pi,3*math.pi/2)]],
    joints=[parts.Hinge('tail_near',shapes=[parts.cylinder(CABLE_RADIUS_M,.050,(-.035,0,0),ORANGE,axis='x')],
                pivot=(-.010,0,0),axis=(0,0,1),lower=-1.5,upper=1.5,mass=.00775,spring=.12,damping=.02),
            parts.Hinge('tail_far',shapes=[parts.cylinder(CABLE_RADIUS_M,.050,(-.085,0,0),ORANGE,axis='x')],
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
SPANNER=parts.Part('spanner',pos=(.215,.040,TOP+.035),mass=.204,
    shapes=[parts.box((.009,.1768,.016),(0,.1109,0),STEEL),*head,
            parts.box((.024,.060,.050),(0,LEVER,0),ORANGE)],
    friction=(.7,.6),contact_offset=.0003)

def support(name,x,y,z):
    return parts.Part(name,pos=(x,y,TOP),shapes=[
        parts.box((.018,.035,z-.006),(0,0,(z-.006)/2),GREY),
        parts.box((.018,.010,.015),(0,-.0125,z-.0065),GREY),
        parts.box((.018,.010,.015),(0,.0125,z-.0065),GREY)],contact_offset=.00025)
SUPPORTS=[support('cable_support_rear',.025+FIXTURE_SHIFT_X_M,.015,.035+FIXTURE_RISER_M),
          support('cable_support_front',.195+FIXTURE_SHIFT_X_M,.015,.035+FIXTURE_RISER_M)]
RISER=parts.Part('enclosure_riser',pos=(AXIS[0]+.055,AXIS[1],TOP+FIXTURE_RISER_M/2),
    shapes=[parts.box((.110,.1904,FIXTURE_RISER_M),(0,0,0),GREY)],contact_offset=.00025)
CLAMPS=[]
for side in (-1,1):
    CLAMPS.append(parts.Part('bench_clamp_'+str(side).replace('-','m'),pos=(.355+FIXTURE_SHIFT_X_M,.015+side*.115,TOP),
        shapes=[parts.box((.050,.025,.015),(0,0,.0075),STEEL),
                parts.box((.025,.020,.070+FIXTURE_RISER_M),(0,0,.040+FIXTURE_RISER_M/2),STEEL),
                parts.box((.045,.035,.012),(-.01,-side*.020,.065+FIXTURE_RISER_M),BLACK)],contact_offset=.0005))
# Tool sleeve sits on this cradle. It does not hold, turn, or couple the head.
CRADLE=parts.Part('tool_cradle',pos=(.215,.235,TOP),shapes=[
    parts.box((.050,.010,.010),(0,0,.005),GREY)],contact_offset=.0003)
HEAD_REST=parts.Part('tool_head_rest',pos=(.205,.040,TOP+.0041),
    shapes=[parts.box((.060,.040,.0082),(0,0,0),GREY)],contact_offset=.0002)
PARK=parts.Part('tool_parking_pad',pos=(.13,.21,TOP+.001),
    shapes=[parts.box((.09,.16,.002),(0,0,0),GREEN)],contact_offset=.00025)
# Fixed dial behind the cap pointer, 5-degree ticks over the first turn from the
# reset mark: blue at first contact, green over the snug sector, red from the
# over-torque line.
marks=[]
for i in range(72):
    d=5*i;a=math.radians(d)
    color=BLUE if d==FIRST_CONTACT_DEG else (GREEN if SNUG_LOW_DEG-2.5<=d<RED_DEG else (RED if RED_DEG<=d<=RED_DEG+45 else GREY))
    marks.append(parts.box((.0004,.004,.002 if color==GREY else .003),
        (CAP_X-HEX_HALF_LEN-.0015,.025*math.cos(a),.025*math.sin(a)),color,rot=parts._rot((1,0,0),a),collide=False))
DIAL=parts.Part('witness_dial',pos=tuple(AXIS),shapes=marks,contact_offset=.0001)
OBJECTS=[ENCLOSURE,GLAND,CABLE,SPANNER,RISER,*SUPPORTS,*CLAMPS,CRADLE,HEAD_REST,PARK,DIAL]
LOTS=lots.declare(OBJECTS,
    lots.Lot('A',place={SPANNER.name:SPANNER.pos}),
    lots.Lot('B',place={SPANNER.name:(.195,.040,TOP+.035)}))

def inserted(s):
    r=parts._quat_to_matrix(s.quat_xyzw)
    end=s.pos+r@np.array((.090,0,0))
    start=s.pos+r@np.array((-.010,0,0))
    # Test the cable where it actually traverses both ends of the body bore,
    # rather than constraining the free sheath end farther inside the box.
    if not end[0]>=AXIS[0]+.014 or r[0,0]<.5 or start[0]>AXIS[0]-.013:return False
    # A tilted cylinder projects an ellipse onto each bore-end plane.
    clearance=GLAND_BORE_RADIUS_M-CABLE_RADIUS_M/r[0,0]
    if clearance<0.:return False
    for x in (AXIS[0]-.013,AXIS[0]+.009):
        at=np.asarray(s.pos)+r[:,0]*((x-s.pos[0])/r[0,0])
        if np.linalg.norm((at-AXIS)[1:])>clearance+1.e-9:return False
    return True

def mark_error_mm(s,g):
    """Signed distance (mm) of the sheath-mark centre from the cap's outer face."""
    r=parts._quat_to_matrix(s.quat_xyzw)
    mark=float((np.asarray(s.pos)+r@np.array((MARK_LOCAL_X,0,0)))[0])
    face=AXIS[0]+CAP_X-HEX_HALF_LEN+float(g.joints['nut'])
    return 1000.*(mark-face)

def mark_ok(s,g):return abs(mark_error_mm(s,g))<=MARK_TOL_MM

def jaw_seated(sp,g):
    """Spanner jaw centre on the cap hex: within 8 mm of the axis and over the hex length."""
    p=np.asarray(sp.pos)
    hx=AXIS[0]+CAP_X+float(g.joints['nut'])
    return bool(np.linalg.norm((p-AXIS)[1:])<=.008 and abs(p[0]-hx)<=HEX_HALF_LEN+.003)

def released(clouds,states):
    if not clouds or any(len(c)==0 or not np.isfinite(c).all() for c in clouds):return False
    bounds=[(cl,np.min(cl,axis=0),np.max(cl,axis=0)) for cl in clouds]
    for spec in (CABLE,SPANNER,GLAND):
        s=states[spec.name]
        poses={'base':(s.pos,parts._quat_to_matrix(s.quat_xyzw))}
        poses.update({n:(p,parts._quat_to_matrix(q)) for n,(p,q) in getattr(s,'links',{}).items()})
        for _,c,r,size in spec.world_boxes(poses):
            # AABB rejection is a conservative broad phase; the original exact
            # oriented-box distance still decides contact for every candidate.
            extent=np.abs(r)@(size/2)+.008
            lo=c-extent;hi=c+extent
            for cl,clo,chi in bounds:
                if np.any(chi<lo) or np.any(clo>hi):continue
                candidate=cl[np.all((cl>=lo)&(cl<=hi),axis=1)]
                if len(candidate) and np.min(np.linalg.norm(np.maximum(np.abs((candidate-c)@r)-size/2,0),axis=1))<=.008:return False
    return True

_STATE=dict(_known=False,_invalid=False,_damage=False,_anchor=0.,_held=0.,_last_t=0.,_q=0.,_last_q=0.,
    _q_peak=0.,_tool_peak=0.,_tool_work=0.,_untooled_work=0.,_peak_torque=0.,_effort=0.,_seated=False,
    _contact_torque=0.,_contact_load=0.,_loaded_hold=0.,_max_loaded_hold=0.,_peak_finger=0.,_clear=False,_pull_ok=False,_cert=None,_pull_start=None,_pull_time=0.,_pull_index=0,
    _pull_residual=0.,_pull_peak_disp=0.,_pull_q=0.,_pull_quat=None,_pull_cert_pos=None,_pull_force_cmd=0.,_parked=False,_mark_mm=float('nan'),_previous_joints=None,
    _still_why='',_observation_error='')
globals().update(_STATE)

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
    global _pull_force_cmd
    import torch
    asset=sim.env.unwrapped.scene[CABLE.name]
    vector=torch.tensor([[[float(force),0.,0.]]],device=sim.env.unwrapped.device)
    torque=torch.zeros_like(vector)
    # Isaac Lab 6's native rigid body external-wrench buffer is applied at each
    # physics substep until replaced. No velocity or pose is ever prescribed.
    body=asset.body_names.index('base')
    asset.set_external_force_and_torque(vector,torque,body_ids=[body],is_global=True)
    _pull_force_cmd=float(force)

def _cap_ids(sim):
    asset=sim.env.unwrapped.scene[GLAND.name]
    return asset,asset.find_joints(['cap'],preserve_order=True)[0]

def set_slip_torque(sim,torque):
    """Native drive force limit of the cap joint: the torque at which it slips."""
    asset,ids=_cap_ids(sim)
    asset.write_joint_effort_limit_to_sim(float(torque),joint_ids=ids)

def cap_reaction(sim):
    """Native projected joint force on the cap (Nm), read from the physics engine."""
    asset,_=_cap_ids(sim)
    raw=asset.root_view.get_dof_projected_joint_forces()
    raw=raw.torch if hasattr(raw,'torch') else raw
    effort=np.asarray(raw.detach().cpu().numpy() if hasattr(raw,'detach') else raw.numpy())[0]
    value=float(effort[asset.joint_names.index('cap')])
    if not np.isfinite(value):raise ValueError('unknown cap reaction')
    return value

@parts.on_reset
def reset(sim):
    globals().update(_STATE)
    cable_force(sim,0.)
    set_slip_torque(sim,FREE_DRAG_NM)
    observation.reset()
    from isaaclab.app.settings_manager import get_settings_manager
    get_settings_manager().set_bool('/physics/disableContactProcessing',False)
    for arm in ('left_robot','right_robot'):
        robot=sim.env.unwrapped.scene[arm]
        ids=robot.find_joints(['joint7','joint8'],preserve_order=True)[0]
        robot.write_joint_effort_limit_to_sim(40.,joint_ids=ids)

def snug_reached():
    """Loaded tightening into the snug sector was carried by the seated spanner."""
    return (_tool_peak>=SNUG_LOW_DEG and _tool_work>=.9*SNUG_WORK_J
            and _max_loaded_hold>=SNUG_LOAD_HOLD_S)

def pull_certificate_valid(s):
    """A completed pull is stale after translation, cable rotation or cap change."""
    if _pull_start is None or _pull_quat is None or _pull_cert_pos is None:return False
    rotation=2.*math.acos(float(np.clip(abs(np.dot(np.asarray(s.quat_xyzw),_pull_quat)),0.,1.)))
    return (np.linalg.norm(np.asarray(s.pos)-_pull_start)*1000.<=PULL_SLIP_MM
            and np.linalg.norm(np.asarray(s.pos)-_pull_cert_pos)*1000.<=PULL_SLIP_MM
            and rotation<=math.radians(1.) and abs(_q-_pull_q)<=1.)

def _pull(sim,s,dt):
    """Three axial pulls at RETENTION_N, each held PULL_HOLD_S, displacement read after unloading."""
    global _pull_start,_pull_time,_pull_index,_pull_residual,_pull_peak_disp,_pull_ok,_invalid,_pull_q,_pull_quat,_pull_cert_pos
    if _pull_start is None:
        _pull_start=np.asarray(s.pos).copy();_pull_time=0.;_pull_index=0;_pull_residual=0.;_pull_peak_disp=0.;_pull_q=_q;_pull_quat=np.asarray(s.quat_xyzw).copy();_pull_cert_pos=None
    _pull_time+=dt
    disp=1000.*float(np.linalg.norm(np.asarray(s.pos)-_pull_start))
    cycle=.5+PULL_HOLD_S+.5
    phase=_pull_time-_pull_index*cycle
    if phase<.5:force=RETENTION_N*phase/.5
    elif phase<.5+PULL_HOLD_S:force=RETENTION_N;_pull_peak_disp=max(_pull_peak_disp,disp)
    else:force=0.
    cable_force(sim,-force)
    if phase>=cycle:
        _pull_residual=max(_pull_residual,disp)
        _pull_index+=1
        if _pull_index>=PULL_COUNT:
            _pull_ok=_pull_residual<=PULL_SLIP_MM
            if not _pull_ok:_invalid=True
            else:
                # The pull itself may rotate the flexible specimen. Freshness
                # is measured AFTER the completed test; its original2mm residual
                # condition remains enforced separately in pull_certificate_valid.
                _pull_cert_pos=np.asarray(s.pos).copy()
                _pull_quat=np.asarray(s.quat_xyzw).copy();_pull_q=_q
            cable_force(sim,0.)

@parts.on_step
def update(sim):
    global _known,_invalid,_damage,_anchor,_held,_last_t,_q,_last_q,_q_peak,_tool_peak,_tool_work,_untooled_work
    global _peak_torque,_effort,_seated,_peak_finger,_clear,_pull_ok,_cert,_pull_start,_pull_time,_parked,_mark_mm
    global _previous_joints,_still_why,_contact_torque,_contact_load,_observation_error,_loaded_hold,_max_loaded_hold
    observed_pull_force=_pull_force_cmd
    states=sim.states();s=states[CABLE.name];g=states[GLAND.name];sp=states[SPANNER.name]
    dt=float(sim.t)-_last_t;_last_t=float(sim.t)
    torque=FREE_DRAG_NM
    try:
        _q=math.degrees(float(g.joints['cap']))
        clouds,force=observation.hand_state(sim)
        _known=np.isfinite(_q) and 0<dt<=.10001
        _peak_finger=max(_peak_finger,force)
        _clear=released(clouds,states)
        _effort=cap_reaction(sim)
        _contact_torque,_contact_load=observation.spanner_cap_load(sim,AXIS)
        _seated=jaw_seated(sp,g)
        torque=cap_torque(_q)
        _peak_torque=max(_peak_torque,torque,abs(_effort),max(0.,_contact_torque))
        # Full snug credit needs a sustained measured native load at the real
        # acceptance threshold; an isolated peak or an 80% work sample cannot pass.
        loaded=min(torque,abs(_effort),_contact_torque)>=CAP_TORQUE_NM-TORQUE_TOL_NM
        _loaded_hold=_loaded_hold+dt if loaded else 0.
        _max_loaded_hold=max(_max_loaded_hold,_loaded_hold)
        _damage|=(_q>=RED_DEG or max(abs(_effort),_contact_torque)>DAMAGE_TORQUE_NM)
        # Credit only new cap maxima under both native cap reaction and signed
        # spanner-to-nut contact torque. Nearby geometry alone earns no work.
        if _q>_q_peak:
            dw=seal_work(max(_q_peak,FIRST_CONTACT_DEG),_q,8) if _q>FIRST_CONTACT_DEG else 0.
            if abs(_effort)>=.8*cap_torque(_q_peak) and _contact_torque>=.8*cap_torque(_q_peak):
                _tool_work+=dw;_tool_peak=_q
            else:_untooled_work+=dw
            _q_peak=_q
        # Elasto-plastic friction joint: the rest point only follows the cap once
        # the slip torque is exceeded, in either direction. It never leads the cap.
        k=math.radians(1.)*BRAKE_K
        _anchor=min(max(_anchor,_q-torque/k),_q+torque/k)
        set_slip_torque(sim,torque)
        sim.set_target(GLAND.name,'cap',math.radians(_anchor))
        # Stillness is read from the change of pose between control steps (what a
        # video shows): at most 3 mm/s and 0.03 rad/s for cable, spanner and joints.
        positions=np.array([*s.joints.values(),*g.joints.values()])
        rots=(parts._quat_to_matrix(s.quat_xyzw),parts._quat_to_matrix(sp.quat_xyzw))
        joint_still=parts_still=False
        if _previous_joints is not None:
            pj,pp,pr=_previous_joints
            joint_still=np.max(np.abs(positions-pj))<=.03*dt
            moved=max(np.linalg.norm(np.asarray(s.pos)-pp[0]),np.linalg.norm(np.asarray(sp.pos)-pp[1]))
            turned=max(math.acos(max(-1.,min(1.,(np.trace(r.T@q)-1)/2))) for r,q in zip(rots,pr))
            parts_still=moved<=.003*dt and turned<=.03*dt
        _still_why='' if joint_still and parts_still else ('joint' if not joint_still else 'parts')
        _previous_joints=(positions.copy(),(np.asarray(s.pos).copy(),np.asarray(sp.pos).copy()),rots)
        _held=_held+dt if _known and _clear and joint_still and parts_still else 0.
        _last_q=_q
        park=np.asarray(lots.current().pos(SPANNER.name))
        _parked=bool(np.linalg.norm(sp.pos-park)<.035 and parts_still)
        _mark_mm=mark_error_mm(s,g)
        ready=_clear and snug_reached() and inserted(s) and mark_ok(s,g) and not _damage
        if _pull_ok and not pull_certificate_valid(s):
            _pull_ok=False;_pull_start=None;_pull_time=0.
        if not ready:
            if _pull_start is not None and not _pull_ok:_pull_start=None;_pull_time=0.
            cable_force(sim,0.)
        elif not _pull_ok and (_held>=1. or _pull_start is not None):_pull(sim,s,dt)
        else:cable_force(sim,0.)
        _cert=snapshot(states)
    except (ValueError,KeyError,AttributeError,IndexError,RuntimeError,TypeError) as exc:
        _known=False;_invalid=True;_clear=False;_observation_error=str(exc)
    if os.environ.get('TRACE_OUT'):
        row=dict(t=float(sim.t),lot=lots.current().name,q_deg=_q,q_peak=_q_peak,tool_peak=_tool_peak,
            tool_work=_tool_work,untooled_work=_untooled_work,torque=torque,peak_torque=_peak_torque,
            nut_advance_m=float(g.joints['nut']),nut_link_pos=np.asarray(g.links['nut'][0]).tolist(),nut_link_quat=np.asarray(g.links['nut'][1]).tolist(),native_contact_min_distance_m=observation.last_spanner_cap_min_distance,
            native_cap_effort=_effort,contact_torque=_contact_torque,contact_load=_contact_load,contact_force=np.asarray(observation.last_spanner_cap_force).tolist(),
            loaded_hold=_loaded_hold,max_loaded_hold=_max_loaded_hold,
            seated=bool(_seated),anchor=_anchor,inserted=inserted(s),
            mark_mm=_mark_mm,cable=np.asarray(s.pos).tolist(),cable_quat=np.asarray(s.quat_xyzw).tolist(),
            known=bool(_known),invalid=bool(_invalid),observation_error=_observation_error,clear=bool(_clear),held=_held,still_why=_still_why,cable_vel=np.r_[s.lin_vel,s.ang_vel].tolist(),
            damage=bool(_damage),peak_finger=_peak_finger,pull_ok=bool(_pull_ok),pull_applied_n=-observed_pull_force,pull_command_n=-_pull_force_cmd,pull_time=_pull_time,
            pull_index=_pull_index,pull_residual=_pull_residual,pull_peak_disp=_pull_peak_disp,parked=bool(_parked),
            spanner=np.asarray(sp.pos).tolist(),spanner_min_z=float(sp.low[2]-TOP),spanner_quat=np.asarray(sp.quat_xyzw).tolist(),
            seals=[float(g.joints['seal'+str(i)]) for i in range(4)],score=score(states))
        row['hands']={}
        from yam_arena.constants import TCP_OFFSET
        for arm in ('left_robot','right_robot'):
            robot=sim.env.unwrapped.scene[arm]
            pose=parts._np(robot.data.body_link_pose_w)[0,robot.body_names.index('gripper')]
            origin=parts._np(sim.env.unwrapped.scene.env_origins[0])
            row['hands'][arm]=dict(joints=parts._np(robot.data.joint_pos)[0].tolist(),joint_names=list(robot.joint_names),tcp=(pose[:3]-origin+parts._quat_to_matrix(pose[3:])@np.array(TCP_OFFSET)).tolist(),quat=pose[3:].tolist())
        try:
            row['finger_spanner_wrench']=observation.finger_spanner_wrench(sim,row['hands']['left_robot']['tcp'],AXIS)
        except (ValueError,AttributeError,RuntimeError,TypeError,IndexError) as exc:
            # Optional diagnostic only; required cap/hand observations above
            # still fail closed. Never use missing finger data as force proof.
            row['finger_spanner_wrench']={'diagnostic_error':str(exc)}
        with pathlib.Path(os.environ['TRACE_OUT']).with_name('gland.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')

def score(objects):
    try:
        s=objects[CABLE.name];g=objects[GLAND.name]
        cert=snapshot(objects)
        if not _known or _invalid or _damage or _peak_torque>DAMAGE_TORQUE_NM or cert!=_cert or not inserted(s):return 0.
        # Release and one second of stillness gate every tier; so does the mark depth.
        if not _clear or _held<1. or not mark_ok(s,g):return 0.
        q=math.degrees(float(g.joints['cap']))
        if q>=RED_DEG:return 0.
        if q<FIRST_CONTACT_DEG:return .25
        if not snug_reached() or q<_q_peak-WINDUP_DEG-1.:return .5
        return 1. if _pull_ok and _parked and _tool_work>0. else .75
    except (ValueError,KeyError,AttributeError,TypeError):return 0.

def success(objects):return score(objects)==1.

# Open-loop strokes, degrees about the gland axis: (seat, swing, ease-off). The
# Raised enclosure fixture clears the sleeve at negative re-seat angles.
# Offline IK grid: the shifted fixture and -65-degree pinch reach -14..62 degrees. Under load the compliant arm lags its target (about 5.6
# degrees per Nm, measured in traces), so: stroke 1 turns the free cap 60 degrees;
# stroke 2 stops where the seal torque holds the arm about 7 degrees short of 62,
# which leaves the next flat at -5; stroke 3 runs into the snug sector.
ARM_LAG_DEG_PER_NM=4.955
JAW_PHASE_ALLOWANCE_DEG=2.
THIRD_FORCE_TRIM_DEG=21.0  # fixed controller trim from native underload diagnostics; not a task threshold
THIRD_CREEP_DEG=16.0  # fixed angular span of the slow finish; not a task threshold
THIRD_CREEP_S=32.
LOADED_WRIST_BOOST_DEG=1.0  # controller estimate: a trial without it peaked just below the strict modeled snug angle
THIRD_APPROACH_S=12.
THIRD_EASE_S=2.
THIRD_WRIST_TRAIL_COEFF=42./(86.3**2)  # controller estimate from simulated underload trials: 6 deg extra TCP travel, 42 deg final wrist relief; creep 0.5 deg/s
JAW_THROAT_DEPTH_M=.003
THIRD_SWING_DEG=FIRST_CONTACT_DEG+SNUG_DEG-120.+JAW_PHASE_ALLOWANCE_DEG+ARM_LAG_DEG_PER_NM*CAP_TORQUE_NM+THIRD_FORCE_TRIM_DEG
STROKES=((-1.,61.,57.),(-8.,61.,49.),(-11.,THIRD_SWING_DEG,33.))

# Simulated unloaded third-stroke grasp calibration (lot A, 40.75 s) on the independently calibrated .16mm-preload sharp6/64-8 physics. Stored GP excludes the separately added throat bias.
# Rigid grasp and robot tracking transforms only; no online state feedback.
THIRD_GRASP_P=np.array([5.312255082742835e-05, 0.16233060219595086, 0.010157388218313553])
THIRD_GRASP_R=np.array([[-0.9999996883810252, 0.00043609495846602377, 0.0006580722145179979], [-0.0002924129997360691, 0.5696800565683982, -0.821866502324352], [-0.0007333024545884587, -0.8218664386440255, -0.5696797515254481]])
THIRD_TRACK_P=np.array([0.0016254489305499442, -0.0024798848861224965, 0.005794635872552578])
THIRD_TRACK_R=np.array([[0.9998142842833285, 0.0006661930255939567, -0.019260143557790443], [-0.00015315045015607838, 0.9996454338090218, 0.026626738623271005], [0.019271053109616607, -0.02661884391976893, 0.9994598859686266]])

# Layout B: same unloaded third-stroke calibration sample time, measured on layout B, whose pickup holds the
# sleeve about 2 mm nearer the head. Fixed constants; no online state feedback.
THIRD_GRASP_P_B=np.array([5.204689655891374e-05, 0.16033983295132936, 0.008197478530376629])
THIRD_GRASP_R_B=np.array([[-0.9999998010191954, 7.66699832366277e-05, 0.0006261655404033663], [-0.000460826427933478, 0.5890750700963777, -0.8080781827459216], [-0.0004308138503300638, -0.8080783105075037, -0.5890749175501918]])
THIRD_TRACK_P_B=np.array([0.001541880146810587, -0.002749468621235651, 0.005783123715169647])
THIRD_TRACK_R_B=np.array([[0.9997999628132737, 0.0034399640875306756, -0.019702817200907618], [-0.002915563034608119, 0.999642365897322, 0.026582697293457698], [0.019787214325596943, -0.026519934959966462, 0.9994524296828494]])


# Simulated unloaded second-stroke grasp calibration (lot A, 28.5 s, sharp hex, 64/8 solver). GPX=0 is the geometric centering estimate for the -4mm servo target.
SECOND_GP=np.array([0.0, 0.16428586883668841, 0.01068858366308502])
SECOND_GR=np.array([[-0.9999998525716691, 0.0002512546805620246, 0.0004813810608646979], [-0.0003123929734254763, 0.4589150680997137, -0.8884800856978536], [-0.000444147802446367, -0.8884801050907783, -0.45891492195243455]])
SECOND_EP=np.array([0.002058503481138868, -0.0022700916127885995, 0.00652947673109483])
SECOND_ER=np.array([[0.9998292156712398, -0.005591835079413206, -0.0176145073924415], [0.006118031502406232, 0.9995323131996017, 0.029962052005648453], [0.017438726466382157, -0.030064701067834543, 0.9993958197675896]])

def demo():
    """Open-loop published layout coordinates; no simulated state feedback."""
    throat=HEAD_ROT@np.array((0.,JAW_THROAT_DEPTH_M,0.))
    grasp_lever=LEVER-.005
    pick=np.array(lots.current().pos(SPANNER.name))+np.array((0,grasp_lever,.008))+throat
    # URDF finger axes close along gripper-localX, parallel to glandX here.
    # The shifted layout needs an 8 mm offset within the 60 mm sleeve to keep
    # the pickup and return inside the wrist's joint limit.
    if lots.current().name == 'B':
        pick += np.array((.008,0,0))
    # Constant measured approach correction for the loaded arm; it is part of
    # the open-loop motion and does not query the simulated pose during demo.
    correction=np.array((-.0025,-.001,.0137))
    if lots.current().name == 'B':correction[0]+=.008
    from yam_arena.demo import grip_rotation
    rot=parts._rot((1,0,0),math.radians(-65.))@grip_rotation(-math.pi/2)
    second_active=False
    third_active=False
    pickup_labels={'approach spanner','lower','grasp sleeve','lift from cradle','return to cradle','lower tool','release spanner','clear','stillness'}
    lot_b=lots.current().name=='B'
    third_grasp_P,third_grasp_R=(THIRD_GRASP_P_B,THIRD_GRASP_R_B) if lot_b else (THIRD_GRASP_P,THIRD_GRASP_R)
    third_track_P,third_track_R=(THIRD_TRACK_P_B,THIRD_TRACK_R_B) if lot_b else (THIRD_TRACK_P,THIRD_TRACK_R)
    def w(t,p,f,label,a=0.):
        wrist_a=a
        if third_active:
            if label.startswith('tighten stroke') or label in {'maintain tightening through loaded creep','sustain tightening load'}:
                wrist_a=a-THIRD_WRIST_TRAIL_COEFF*(a-seat)**2
                if label in {'maintain tightening through loaded creep','sustain tightening load'}:
                    wrist_a+=LOADED_WRIST_BOOST_DEG*float(np.clip((a-(swing-THIRD_CREEP_DEG))/THIRD_CREEP_DEG,0.,1.))
            elif label=='ease off':
                wrist_a=a-THIRD_WRIST_TRAIL_COEFF*(swing-seat)**2*(a-ease)/(swing-ease)
                wrist_a+=LOADED_WRIST_BOOST_DEG*(a-ease)/(swing-ease)
        R=third_track_R@parts._rot((1,0,0),math.radians(wrist_a))@third_grasp_R if third_active else SECOND_ER@parts._rot((1,0,0),math.radians(a))@SECOND_GR if second_active else parts._rot((1,0,0),math.radians(a))@rot
        if lots.current().name=='B' and label in pickup_labels:
            R=parts._rot((1,0,0),math.radians(1.))@R  #26deg B pickup/parking; A unchanged
        if (third_active or second_active) and f==0.:f=-.004  #40N nominal at12mm gap, with restoring force instead of saturated neutral zone
        if third_active and label in {'maintain tightening through loaded creep','sustain tightening load'}:f=-.003  # closing target only; existing40N drivecaps unchanged
        return Waypoint(t,tuple(float(v) for v in p),f,label,
            tilt._TiltedYaw(0,R))
    def at(angle_deg,x_shift):
        R=parts._rot((1,0,0),math.radians(angle_deg))
        center=AXIS+np.array((CAP_X+x_shift,0,0))-R@throat
        if second_active:return center+R@SECOND_GP+SECOND_EP
        if third_active:
            p=center+R@(third_grasp_P+throat)+third_track_P
            # Centered pinch: third stroke does not retain the B pickup offset.
            return p
        return center+R@(np.array((0,grasp_lever,.008))+throat)+correction
    from yam_arena import ik as _ik,station as _station
    from yam_arena.constants import READY
    ready_p,ready_R=_ik.tcp_pose(np.array(READY),_station.BASES['left'])
    right=[Waypoint(9.,tuple(ready_p),FINGER_OPEN,'wait for cable feed',tilt._TiltedYaw(0.,ready_R)),w(2,pick+(0,0,.050),FINGER_OPEN,'approach spanner'),
        w(2,pick,FINGER_OPEN,'lower'),w(1.5,pick,0.,'grasp sleeve'),
        w(2,pick+(0,0,.035),0.,'lift from cradle')]
    left_pick=np.array(CABLE.pos)+np.array((-.055,0,.008))
    # Feed so the mark centre meets the reset cap outer face.
    feed=AXIS[0]+CAP_X-HEX_HALF_LEN-(CABLE.pos[0]+MARK_LOCAL_X)
    left_goal=left_pick+np.array((feed,0,0))
    left=[Waypoint(2,tuple(left_pick+(0,0,.07)),FINGER_OPEN,'approach cable',0.),
        Waypoint(2,tuple(left_pick),FINGER_OPEN,'lower to sheath',0.),
        Waypoint(1.5,tuple(left_pick),0.,'grasp sheath',0.),
        Waypoint(3,tuple(left_goal),0.,'feed to the mark',0.)]
    def axial():return np.array((-.025,0,0))
    def radial(angle_deg):
        return parts._rot((1,0,0),math.radians(angle_deg))@HEAD_ROT@np.array((0,.028,0))
    x=0.;turned=0.
    right += [w(1.5,at(0.,x)+radial(0.),0.,'approach jaw',0.)]
    def arc(start,end,duration,label,x):
        steps=max(1,math.ceil(abs(end-start)/5.))
        return [w(duration/steps,at(float(a),x),0.,label,float(a)) for a in np.linspace(start,end,steps+1)[1:]]
    for k,(seat,swing,ease) in enumerate(STROKES):
        second_active=k==1
        third_active=k==2
        if k==1:right.append(w(2.,at(0.,x)+radial(0.),0.,'settle before second seat',0.))
        if k==2:
            right.append(w(1.5,at(0.,x)+radial(0.),0.,'calibrated third staging',0.))
            right.append(w(2.,at(0.,x)+radial(0.),0.,'settled third staging',0.))
        right += [w(1.5,at(seat,x),0.,'seat jaw on flats',seat)]
        approach_end=swing-THIRD_CREEP_DEG if k==2 else swing
        right += arc(seat,approach_end,THIRD_APPROACH_S if k==2 else 3.5,'tighten stroke '+str(k+1),x)
        if k==2:right += arc(approach_end,swing,THIRD_CREEP_S,'maintain tightening through loaded creep',x)
        if k==len(STROKES)-1:right.append(w(3.,at(swing,x),0.,'sustain tightening load',swing))
        right += arc(swing,ease,THIRD_EASE_S if k==2 else .75,'ease off',x)
        right += [w(1.,at(ease,x)+radial(ease),0.,'slide jaw off the flats',ease)]
        turned+=ease+2.-seat
        x=CAP_LEAD*turned/360.
        nxt=STROKES[k+1][0] if k+1<len(STROKES) else 0.
        # Short chords keep the free jaw outside the hex during the return.
        steps=max(1,math.ceil(abs(ease)/8.))
        for angle in np.linspace(ease,0.,steps+1)[1:]:
            right.append(w(2./steps,at(float(angle),x)+radial(float(angle)),0.,'return outside cap',float(angle)))
    third_active=False
    elapsed=sum(p.duration for p in right)
    left.append(Waypoint(elapsed-sum(p.duration for p in left),tuple(left_goal),0.,'hold cable at the mark',0.))
    left += [Waypoint(1.5,tuple(left_goal),FINGER_OPEN,'release cable',0.),
        Waypoint(2,tuple(left_goal+(0,-.070,.08)),FINGER_OPEN,'hands clear',0.),
        Waypoint(18,tuple(left_goal+(0,-.070,.08)),FINGER_OPEN,'wait for independent pull',0.)]
    right += [w(2,pick+(0,0,.035),0.,'return to cradle'),w(2,pick,0.,'lower tool'),
        w(1.5,pick,FINGER_OPEN,'release spanner'),w(2,pick+(0,0,.08),FINGER_OPEN,'clear'),
        w(14,pick+(0,0,.08),FINGER_OPEN,'stillness')]
    return {'right':left,'left':right}
