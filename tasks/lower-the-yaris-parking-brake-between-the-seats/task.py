"""A Toyota Yaris parking-brake lever stands on its ratchet between two seat-bolster envelopes and a slotted centre console.
The lever, its 35 mm grip and its end button are full size and unmodified. A labelled angle scale stands beside the lever end.
"""
from __future__ import annotations
import math, os, pathlib, sys
from dataclasses import replace
from functools import lru_cache
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts, lots, tilt, shoes, motors
parts._PATCHES['yam_arena.robot']=motors.patch_robot
shoes.add_hulls()
from yam_arena import ik, station
from yam_arena.task_file import TABLE_TOP, Box
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.constants import READY, FINGER_OPEN, FINGER_CLOSED
SCENE='station'; TOP=TABLE_TOP[SCENE]; SCORE_OVER='end'; SUCCESS_HOLD_S=0.
VERSION='0.19.4'
PHYSICS_HZ=int(os.environ.get('HB_PHYSICS_HZ','480'))
if PHYSICS_HZ not in (480,960):raise ValueError('HB_PHYSICS_HZ must be 480 or 960')
parts.HANDBRAKE_PHYSICS_HZ=PHYSICS_HZ
parts.HANDBRAKE_SOLVER_TYPE=0  # PhysX PGS, numeric solve only; physical parameters unchanged

# ---- Every load and friction figure is one named constant; a real reading replaces it without a redesign. ----
# Obtained by phone from a technician who works with this product (Nov 2026), measured on the car
# a 2018 Yaris XP130: 54 N at four clicks, scale radius 210 mm; roughly linear with click count.
# Use the normal five-click setting. External pull is applied as cable-equivalent resistance, with model gravity
# added conservatively (not subtracted). Scaling follows the reported proportionality, not additional measurements.
# The click-angle map assumes click zero at the fully down stop.
MEASURED_FOUR_CLICK_PULL_N=54.
MEASURED_CLICKS=4.
PITCH_DEG=5.
START_CLICKS=5.
START_DEG=START_CLICKS*PITCH_DEG
RELEASE_PULL_N=MEASURED_FOUR_CLICK_PULL_N*START_CLICKS/MEASURED_CLICKS
LOAD_RISE_N_PER_DEG=MEASURED_FOUR_CLICK_PULL_N/(MEASURED_CLICKS*PITCH_DEG)
SLACK_PULL_N=3.9
GRIP_CENTRE_R=.210
PAWL_CLEAR_DEG=1.          # estimate, unchanged from the prior declared mechanism
BUTTON_FORCE_N=7.4
BUTTON_TRAVEL=.008
BUTTON_MASS=.020
# Constant passive return load envelopes the reported button force throughout its stroke.
# Worst-case slider gravity cannot reduce inward-motion resistance below 7.4 N.
BUTTON_RETURN_N=BUTTON_FORCE_N+BUTTON_MASS*9.81
BUTTON_K=0.
BUTTON_RELEASE=BUTTON_TRAVEL
BUTTON_REENGAGE=.0045
BUTTON_LOCKED=.001
GRIP_FRICTION=(0.,0.)       # conservative friction-free grip; passive shoes carry the load by bearing
LEVER_MASS=.80             # ESTIMATE, kg
LEVER_DAMPING=.5           # N m s/rad, ESTIMATE of cable and pivot drag
DROP_DEG=8.; DROP_S=.1     # a lever that falls 8 degrees within 0.1 s (3 video frames at 30 fps) was dropped
CLEAR_M=.030; HOLD_S=1.

ANGLE=math.radians(START_DEG); PITCH=math.radians(PITCH_DEG); LENGTH=.297
# Conservative bench-fixture reference envelope. Exact spring rate comes from TOHATSU TM20x45;
# the35mm effective original-rod radius and1mm preload are labelled design estimates, not Yaris readings.
# A front-spring8mm compression stop changes two-series rate27240N/m to54480N/m after16mm total.
# Physical enrollment must fit both the reported lower bound and this upper reference+gravity bound.
BENCH_LINK_R_EST=.035
BENCH_PRELOAD_EST=.001
BENCH_SPRING_RATE_N_PM=54480.
BENCH_FRONT_COMPRESSION_STOP=.008
BENCH_TOTAL_COMPRESSION_STOP=.0221

def reported_pull(theta):
    return max(SLACK_PULL_N,RELEASE_PULL_N+LOAD_RISE_N_PER_DEG*(math.degrees(theta)-START_DEG))
def bench_reference_moment(theta):
    th=max(0.,float(theta));x=BENCH_PRELOAD_EST+BENCH_LINK_R_EST*math.sin(th)
    series_rate=BENCH_SPRING_RATE_N_PM/2
    rod_force=series_rate*(x+max(0.,x-2*BENCH_FRONT_COMPRESSION_STOP))
    return BENCH_LINK_R_EST*max(0.,math.cos(th))*rod_force
def cable_moment(theta):
    return max(GRIP_CENTRE_R*reported_pull(theta),bench_reference_moment(theta))
def cable_pull(theta):
    """Conservative cable-equivalent pull at the reported210mm scale radius, before extra lever gravity."""
    return cable_moment(theta)/GRIP_CENTRE_R

YAW=0.; BASE_R=parts._rot((0,0,1),YAW)
PIVOT=np.array((.300,-.420,TOP+.068)); AXIS=np.array((1.,0.,0.))
def installed(v):return tuple(float(x) for x in PIVOT+BASE_R@np.asarray(v))
R0=parts._rot(AXIS,ANGLE); U0=R0@np.array((0.,1.,0.))
INSTRUCTION='Raise the lever slightly, hold its end button in while lowering it fully, then move both hands clear.'
STEEL=(.58,.60,.62); BLACK=(.06,.06,.07); TRIM=(.17,.18,.20); BLUE=(.19,.25,.35)

def turn(shape):
    return replace(shape,at=tuple(R0@np.asarray(shape.at)),quat=tuple(parts._matrix_to_quat(R0)))
# Stock lever only: 12 x 26 mm steel arm, 35 mm grip from 170 to 272 mm, button boss; nothing is added to it.
lever_shapes=[turn(parts.box((.012,.20,.026),(0,.070,0),STEEL)),
              replace(turn(parts.cylinder(.0175,.102,(0,.221,0),BLACK,axis='y')),friction=GRIP_FRICTION,friction_combine="max"),
              replace(turn(parts.cylinder(.0125,.017,(0,.2805,0),STEEL,axis='y')),collide=False)]
# The original button slides in a hollow boss. A full solid cap at y=289 mm incorrectly blocks even a
# fingertip placed on the button centre at full travel. Annular wall colliders keep that bore open.
BOSS_BORE_R=.0075; BOSS_OUTER_R=.0125; BOSS_SEGMENTS=16  # geometric estimates, outer envelope unchanged
for k in range(BOSS_SEGMENTS):
    phi=2*math.pi*k/BOSS_SEGMENTS;radial=parts._rot((0,1,0),-phi)
    shape=parts.box((BOSS_OUTER_R-BOSS_BORE_R,.017,2*BOSS_BORE_R*math.tan(math.pi/BOSS_SEGMENTS)),
                    ((BOSS_OUTER_R+BOSS_BORE_R)/2*math.cos(phi),.2805,(BOSS_OUTER_R+BOSS_BORE_R)/2*math.sin(phi)),STEEL)
    lever_shapes.append(replace(turn(shape),quat=tuple(parts._matrix_to_quat(R0@radial))))
button_shape=turn(parts.cylinder(.0075,.016,(0,.289,0),(.84,.84,.86),axis='y'))
LEVER=parts.Hinge('lever',tuple(lever_shapes),(0.,0.,0.),(1.,0.,0.),0.,math.radians(12),mass=LEVER_MASS,
                  spring=0.,damping=LEVER_DAMPING,max_effort=100.)
BUTTON=parts.Slider('button',(button_shape,),tuple(-U0),BUTTON_TRAVEL,mass=BUTTON_MASS,spring=BUTTON_K,damping=2.,max_effort=200.,parent='lever')
BRAKE=parts.Part('parking_brake',pos=tuple(float(v) for v in PIVOT),yaw=YAW,shapes=[parts.box((.150,.080,.012),(0,0,-.042),STEEL),
  *[parts.box((.008,.060,.048),(x,0,-.024),STEEL) for x in (-.024,.024)],
  parts.cylinder(.007,.040,(0,0,0),STEEL,axis='x')],joints=[LEVER,BUTTON],friction=(.8,.7),
  reset_joint_limits=(('lever',0.,math.radians(12)),('button',0.,BUTTON_LOCKED)),
  reset_joint_efforts=(('lever',-cable_moment(ANGLE)),('button',-BUTTON_RETURN_N)))
# Rigid access envelopes of the inner parts of the two seats and of the console; the floor board lies on the table.
FLOOR=Box('installed_floor',pos=installed((0,.150,-.059)),size=(.440,.580,.018),yaw=YAW,color=(.36,.28,.18))
SEATS=[Box('seat_near',pos=installed((-.1545,.150,.020)),size=(.089,.560,.140),yaw=YAW,color=BLUE),
       Box('seat_far',pos=installed((.1545,.150,.020)),size=(.089,.560,.140),yaw=YAW,color=BLUE)]
CONSOLE=[Box('console_near',pos=installed((-.059,.135,-.031)),size=(.038,.420,.038),yaw=YAW,color=TRIM),
         Box('console_far',pos=installed((.059,.135,-.031)),size=(.038,.420,.038),yaw=YAW,color=TRIM),
         Box('console_rear',pos=installed((0.,-.060,-.031)),size=(.080,.030,.038),yaw=YAW,color=TRIM)]
# A protractor plate beside the lever end; tick labels are lever angles in degrees.
scale_shapes=[parts.box((.002,.120,.210),(0,.300,.075),(.16,.16,.16))]
font={'0':['111','101','101','101','111'],'1':['010','110','010','010','111'],
      '2':['111','001','111','100','111'],'3':['111','001','111','001','111'],
      '4':['101','101','111','001','001'],'6':['111','100','111','101','111'],
      '8':['111','101','111','101','111']}
for deg in range(0,33,4):
    a=math.radians(deg)
    scale_shapes.append(replace(parts.box((.003,.012,.004),(0,LENGTH*math.cos(a),LENGTH*math.sin(a)),(.9,.3,.1)),
                                quat=tuple(float(v) for v in parts._matrix_to_quat(parts._rot(AXIS,a))),collide=False))
    if deg%8==0:
        c=np.array((0.,(LENGTH+.024)*math.cos(a),(LENGTH+.024)*math.sin(a)))
        for digit,char in enumerate(str(deg)):
            for row,line in enumerate(font[char]):
                for col,v in enumerate(line):
                    if v=='1':scale_shapes.append(replace(parts.box((.0045,.0045,.0045),c+(-.0003,(digit*4+col)*.0055,-row*.0055),(.95,.95,.95)),collide=False))
SCALE=parts.Part('angle_scale',pos=tuple(float(v) for v in PIVOT+BASE_R@np.array((.100,0,0))),yaw=YAW,shapes=scale_shapes)

# Rear cartridge overhang: the actual table edge is world y=-0.650 m.
# These fixed Parts use the front mounting datum on the table, not the rear box centre.
# Their full offset envelopes include the real overhang; no inaccessible steel is hidden.
GUARD=Box('rod_guard_tunnel',pos=installed((0.,-.168,-.010)),size=(.116,.180,.116),yaw=YAW,color=(.36,.28,.18))
ANCHOR_LOCAL=np.array((0.,-.134,-.010))
def fixed_guard(name,size,centre,color=(.36,.28,.18)):
    return parts.Part(name,pos=installed(ANCHOR_LOCAL),yaw=YAW,shapes=[parts.box(size,np.asarray(centre)-ANCHOR_LOCAL,color)])
# Complete conservative OEM mount: two flat base bars, receipt-selected shims, clamp straps,
# and actual through-60mm-bench studs/spreaders. All fixed hardware is inside these bounds.
MOUNTS=[fixed_guard('oem_base_bar_'+str(i),(.220,.050,.00635),(0.,y,-.046825),STEEL) for i,y in enumerate((-.050,0.,.050))]
# Two real50x80 shim stacks register the fixed flange; union includes air/receipt adjustment.
MOUNTS += [fixed_guard('oem_shim_envelope',(.150,.080,.010),(0.,0.,-.03865),STEEL)]
# Welded212x50x12.7 capture bar, rear20x30 relief for the unmodified lever tail.
# Bounds include external3mm weld beads; the relief is machined after welding.
MOUNTS += [fixed_guard('oem_capture_side_'+str(i),(.099,.056,.0277),(x,0.,-.03415),STEEL) for i,x in enumerate((-.0595,.0595))]
MOUNTS += [fixed_guard('oem_capture_front',(.218,.023,.0277),(0.,.0165,-.03415),STEEL)]
MOUNTS += [fixed_guard('oem_through_bench_stud_'+str(i),(.030,.040,.172),(x,0.,-.082),STEEL) for i,x in enumerate((-.090,.090))]
# Four positive front/rear keys and paired158mm studs; all fixed union bounds are explicit.
# Key centre |x|30..50, |y|42.5..52.5mm; welded stack36x25x25.4, common steel spreaders.
MOUNTS += [fixed_guard('oem_positive_key_'+str(i),(.062,.050,.172),(x,y,-.082),STEEL)
 for i,(x,y) in enumerate(((-.040,-.0475),(-.040,.0475),(.040,-.0475),(.040,.0475)))]
REAR_GUARD=fixed_guard('rear_load_guard',(.116,.240,.180),(0.,-.360,-.020))
GUARD_BASE=fixed_guard('rear_support_envelope',(.220,.260,.060),(0.,-.360,-.115))
SUPPORTS=[fixed_guard('support_rail_'+str(i),(.00635,.400,.025),(x,-.340,-.0555),STEEL) for i,x in enumerate((-.090,.090))]
# Whole fixed stud/foot/underside-spreader envelopes include 60 mm tabletop penetration.
SUPPORTS += [fixed_guard('bench_foot_'+str(i),(.080,.100,.120),(x,-.180,-.098),STEEL) for i,x in enumerate((-.090,.090))]
SUPPORTS += [fixed_guard('shelf_cross_support_'+str(i),(.220,.025,.00635),(0.,y,-.131175),STEEL) for i,y in enumerate((-.280,-.450))]
# Four physical hanger strips from edge-on rails to the lower rear shelf.
SUPPORTS += [fixed_guard('shelf_hanger_'+str(i),(.00635,.025,.085),(x,y,-.0855),STEEL) for i,(x,y) in enumerate(((-.090,-.280),(.090,-.280),(-.090,-.450),(.090,-.450)))]
OBJECTS=[BRAKE,FLOOR,*SEATS,*CONSOLE,SCALE,GUARD,REAR_GUARD,GUARD_BASE,*SUPPORTS,*MOUNTS]
# The brake assembly is identical in both lots; a loose card changes place so each lot is a separate declared trial.
CARD=Box('layout_card',pos=(.72,-.55,TOP+.005),size=(.060,.080,.010),mass=.04,color=(.8,.75,.1));OBJECTS.append(CARD)
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={CARD.name:CARD.pos}),lots.Lot('B',place={CARD.name:(.72,.45,TOP+.005)}))
from mechanism import Judge,Sample
from mechanism.native import Monitor
_judge=Judge(start=ANGLE,pitch=PITCH,clear=math.radians(PAWL_CLEAR_DEG),release=BUTTON_RELEASE,reengage=BUTTON_REENGAGE,
             travel=BUTTON_TRAVEL,locked=BUTTON_LOCKED,drop=math.radians(DROP_DEG),drop_s=DROP_S,clear_m=CLEAR_M,hold_s=HOLD_S)
_monitor=None
_unit_certificate={'value':None}

@lru_cache(maxsize=1)
def hand_mesh():
    from yam_arena import analysis
    out={}
    for name in ('gripper','tip_left','tip_right'):
        cloud=[]
        for hull in analysis.link_hulls().get(name,[]):
            cloud.extend(hull.vertices)
            for plane in np.unique(hull.planes.round(7),axis=0):
                face=hull.vertices[np.abs(hull.vertices@plane[:3]-plane[3])<2e-6]
                if len(face)<3:raise ValueError('missing hand face')
                c=face.mean(0);u=face[0]-c;u/=np.linalg.norm(u);v=np.cross(plane[:3],u)
                face=face[np.argsort(np.arctan2((face-c)@v,(face-c)@u))]
                for a,b in zip(face,np.roll(face,-1,axis=0)):
                    n=max(1,math.ceil(max(np.linalg.norm(a-c),np.linalg.norm(b-c),np.linalg.norm(a-b))/.003))
                    for i in range(n+1):
                        for j in range(n-i+1):cloud.append(c+(a-c)*i/n+(b-c)*j/n)
        arr=np.asarray(cloud)
        _,keep=np.unique(np.floor(arr/.001).astype(int),axis=0,return_index=True)
        out[name]=arr[keep]
    return out

def signature(s):
    return np.r_[s.pos,s.quat_xyzw,s.joints['lever'],s.joints['button']]

_written_limits={}
def write_limits(asset,low,button_hi):
    """Change only safe bounds. Never advance a bound past the current physical pose.

    An unchanged, already active bound may contain a tiny solver overshoot. Retaining that bound
    while changing its opposite is not a new projection; it never follows the overshoot outward.
    """
    import torch
    from mechanism.native import arr
    q=arr(asset.root_view.get_dof_positions())[0]
    specs=(('lever',float(low),math.radians(12)),('button',0.,float(button_hi)))
    events=[];live=None
    for name,lo,hi in specs:
        i=asset.joint_names.index(name);key=(id(asset),name)
        if key not in _written_limits:
            if live is None:live=arr(asset.root_view.get_dof_limits())[0]
            _written_limits[key]=tuple(float(v) for v in live[i])
        previous_lo,previous_hi=_written_limits[key]
        if (previous_lo,previous_hi)==(lo,hi):continue
        if (lo!=previous_lo and lo>float(q[i])) or (hi!=previous_hi and hi<float(q[i])):continue
        limits=torch.tensor([[[lo,hi]]],device=asset.device)
        asset.write_joint_position_limit_to_sim_index(limits=limits,joint_ids=[i])
        _written_limits[key]=(lo,hi)
        events.append(dict(joint=name,lower=lo,upper=hi,q=float(q[i]),
                           previous_lower=previous_lo,previous_upper=previous_hi))
    return events

def set_cable_effort(asset,theta):
    """Apply passive cable and button resistance through their joints, with equal body reactions."""
    import torch
    ids=[asset.joint_names.index(n) for n in ('lever','button')]
    asset.set_joint_effort_target_index(target=torch.tensor([[-cable_moment(theta),-BUTTON_RETURN_N]],device=asset.device),joint_ids=ids)
    asset.write_data_to_sim()

@parts.on_pre_reset
def before_reset(sim):
    if _monitor is not None:_monitor.disable()

@parts.on_reset
def reset(sim):
    global _monitor
    from isaaclab.app.settings_manager import get_settings_manager
    get_settings_manager().set_bool('/physics/disableContactProcessing',False)
    asset=sim.env.unwrapped.scene[BRAKE.name]
    _written_limits.clear()
    write_limits(asset,0.,BUTTON_LOCKED);set_cable_effort(asset,ANGLE)
    _unit_certificate['value']=None
    _monitor=Monitor(globals(),sim);_monitor.start()

@parts.on_step
def update(sim):
    if _monitor is not None:_monitor.log()

def score(objects):
    try:
        state=objects[BRAKE.name];cert=_monitor.certificate if _monitor is not None else _unit_certificate['value']
        if cert is None or not np.isfinite(signature(state)).all() or not np.allclose(signature(state),cert,atol=1e-6,rtol=0):return 0.
        return _judge.score()
    except (AttributeError,TypeError,ValueError,KeyError):return 0.
def success(objects):return score(objects)==1.

def point(r,angle,offset=0.):return PIVOT+BASE_R@np.array((offset,r*math.cos(angle),r*math.sin(angle)))

# ---- Scripted demonstration: open loop, from the declared fixture geometry and loads only. ----
ARM_K=np.array((80.,80.,80.,10.,10.,10.))   # stock joint stiffness, N m/rad
HOLD_FINGER=-.019                         # 38 mm command; 27 mm toes bear below the grip with side clearance
R_HOLD=.228                                  # pinch radius on the grip, m
LIFT_DEG=float(os.environ.get('HB_LIFT_DEG','1.5'))      # commanded lift above the start tooth
PULL_GAIN=float(os.environ.get('HB_PULL_GAIN','1.0'))    # multiplies the feed-forward pull
PULL_BIAS=float(os.environ.get('HB_PULL_BIAS','4.0'))    # N added to the feed-forward pull
PRESS_N=float(os.environ.get('HB_PRESS_N','12.0'))        # commanded button press, N
PRESS_TILT=math.radians(float(os.environ.get('HB_PRESS_TILT','55')))
def _rx(a):return parts._rot((1,0,0),a)
def _loaded_pose(arm,pos,rot,force,seed,contact_offset=None):
    """Fixed spring feed-forward at the declared contact point, not an assumed wrist TCP.

    contact_offset is the contact's fixed local vector from TCP. Only hand CAD/declared pose is used.
    """
    base=station.BASES[arm]
    q,_,_=ik.solve(np.asarray(pos,float),ik.matrix_to_quat_xyzw(rot),np.asarray(seed,float),base_pos=base,iters=400)
    offset=np.zeros(3) if contact_offset is None else np.asarray(contact_offset,float)
    def contact(q0):
        p0,r0=ik.tcp_pose(q0,base)
        return p0+r0@offset
    p=contact(q);J=np.zeros((3,6))
    for j in range(6):
        d=q.copy();d[j]+=1e-6;J[:,j]=(contact(d)-p)/1e-6
    target=np.clip(q+(J.T@np.asarray(force,float))/ARM_K,ik.LOWER+.01,ik.UPPER-.01)
    tp,tr=ik.tcp_pose(target,base)
    return tp,tr,q
def _wp(t,pos,rot,finger,label):return Waypoint(t,tuple(float(v) for v in pos),finger,label,tilt._TiltedYaw(0.,np.asarray(rot)))
def lever_weight_pull(theta):return LEVER_MASS*9.81*.125*math.cos(theta)/R_HOLD   # N at the pinch radius, estimate

@lru_cache(maxsize=1)
def _pressing_hand_vertices():
    # Stock closed-hand CAD only: the pressing hand has no shoes. Geometry, never live observations.
    from yam_arena import YAM_URDF
    from yam_arena.urdf import forward_kinematics
    from yam_arena.constants import TCP_OFFSET
    fk=forward_kinematics(YAM_URDF,{f'joint{i}':0. for i in range(1,9)})
    vertices=[]
    for name in ('gripper','tip_left','tip_right'):
        transform=np.linalg.inv(fk['gripper'])@fk[name]
        vertices.append(np.concatenate([h.vertices for h in shoes.STOCK_HULLS[name]])@transform[:3,:3].T+transform[:3,3]-TCP_OFFSET)
    return np.concatenate(vertices)

def _press_offset(th,rot):
    # Place the leading rounded corner on the button centre, keeping the remaining hand ahead of the boss.
    vertices=_pressing_hand_vertices()@rot.T
    axis=np.array((0.,math.cos(th),math.sin(th)))
    return -vertices[np.argmin(vertices@axis)]

@lru_cache(maxsize=1)
def _plan():
    R,L=[],[]
    normal=lambda th:np.array((0.,-math.sin(th),math.cos(th)))
    along=lambda th:np.array((0.,math.cos(th),math.sin(th)))
    hold_rot=lambda th:_rx(th)@grip_rotation(math.pi/2)
    press_rot=_rx(-PRESS_TILT)@grip_rotation(math.pi/4)
    press_axis=np.array((0.,-math.sin(PRESS_TILT),-math.cos(PRESS_TILT)))
    def hold(th,scale=1.,seed=None):
        f=scale*(PULL_GAIN*cable_moment(th)/R_HOLD+lever_weight_pull(th)+PULL_BIAS)
        return _loaded_pose('right',point(R_HOLD,th),hold_rot(th),f*normal(th),READY if seed is None else seed)
    def press(th,depth,force,seed=None):
        tip=point(LENGTH-depth,th)
        offset=_press_offset(th,press_rot)
        return _loaded_pose('left',tip+offset,press_rot,-force*along(th),READY if seed is None else seed,
                            contact_offset=press_rot.T@(-offset))
    a0=ANGLE;a1=ANGLE+math.radians(LIFT_DEG)
    grasp=point(R_HOLD,a0);up=normal(a0)
    # right hand: over the near seat, down onto the grip, pinch, lift, lower, let go, withdraw
    R.append(_wp(3.5,grasp+up*.070,hold_rot(a0),FINGER_OPEN,'move the open holding hand above the grip'))
    R.append(_wp(2.5,grasp,hold_rot(a0),FINGER_OPEN,'lower open fingers either side of the grip'))
    R.append(_wp(1.5,grasp,hold_rot(a0),HOLD_FINGER,'place the bearing toes beneath the grip'))
    p,r,qr=hold(a0,.5);R.append(_wp(1.5,p,r,HOLD_FINGER,'take up half the load'))
    p,r,qr=hold(a1,1.,qr);R.append(_wp(2.5,p,r,HOLD_FINGER,'lift the lever off its tooth'))
    R.append(_wp(3.5,p,r,HOLD_FINGER,'hold the lever lifted while the button goes in'))
    steps=int(math.ceil(math.degrees(a1)))
    for k in range(1,steps+1):
        th=a1*(1-k/steps)
        p,r,qr=hold(th,1.,qr);R.append(_wp(.35,p,r,HOLD_FINGER,'lower the lever with the button held in'))
    rest=point(R_HOLD,0.)
    R.append(_wp(3.,rest,hold_rot(0.),HOLD_FINGER,'rest the lever on its stop while the button comes out'))
    # The normal low axial slide stalls base yaw and twists the hand into the console. Keep base yaw
    # fixed instead: the URDF's 35.6 mm last-joint offset plus 134 mm TCP gives 169.6 mm of wrist reach.
    # A forward lean asin(delta_y/169.6 mm) moves the TCP along the lever without changing that yaw.
    R.append(_wp(2.,rest,hold_rot(0.),HOLD_FINGER,'wait until the pressing hand is clear'))
    loose=-.019  # 38 mm pad gap, within the unchanged 80 mm console slot
    under=rest-np.array((0.,0.,.020))
    R.append(_wp(1.5,under,hold_rot(0.),loose,'relax pinch and lower the bearing feet clear'))
    R.append(_wp(1.,under,hold_rot(0.),loose,'settle below the grip before withdrawal'))
    wrist_to_tcp=.0356+.134  # metres, supplied YAM URDF and stock TCP, not a measured lever dimension
    exit_r=.320
    for radius in np.linspace(R_HOLD,exit_r,21)[1:]:
        lean=math.asin((radius-R_HOLD)/wrist_to_tcp)
        R.append(_wp(.25,point(radius,0.)-np.array((0.,0.,.020)),_rx(lean)@hold_rot(0.),loose,
                     'lean forward and slide the unloaded feet off the lever with fixed base yaw'))
    exit_rot=_rx(math.asin((exit_r-R_HOLD)/wrist_to_tcp))@hold_rot(0.)
    for height in np.linspace(-.020,.110,11)[1:]:
        R.append(_wp(.25,point(exit_r,0.)+np.array((0.,0.,height)),exit_rot,loose,
                     'raise the feet beyond the button before opening'))
    end=point(exit_r,0.)+np.array((0.,0.,.110))
    R.append(_wp(1.5,end,exit_rot,FINGER_OPEN,'open above the console'))
    R.append(_wp(2.5,end,exit_rot,FINGER_OPEN,'holding hand clear and still'))
    # left hand: closed fingertips to the button, press once the lever is lifted, follow the lever down, withdraw
    t_lift=sum(w.duration for w in R[:5])
    off=lambda th,d:point(LENGTH,th)+_press_offset(th,press_rot)-press_axis*d
    L.append(_wp(3.,off(a0,.120)+np.array((0.,.04,.10)),press_rot,FINGER_CLOSED,'move the closed pressing hand toward the lever end'))
    L.append(_wp(3.,off(a1,.025),press_rot,FINGER_CLOSED,'bring the fingertips in line with the button'))
    L.append(_wp(t_lift-6.+.5,off(a1,.008),press_rot,FINGER_CLOSED,'wait at the button until the lever is lifted'))
    p,r,ql=press(a1,.007,10.);L.append(_wp(2.,p,r,FINGER_CLOSED,'press toward full travel'))
    p,r,ql=press(a1,BUTTON_TRAVEL+.0001,PRESS_N,ql);L.append(_wp(.7,p,r,FINGER_CLOSED,'finish the last millimetre slowly'))
    L.append(_wp(.3,p,r,FINGER_CLOSED,'hold the button in'))
    for k in range(1,steps+1):
        th=a1*(1-k/steps)
        p,r,ql=press(th,BUTTON_TRAVEL+.0001,PRESS_N,ql);L.append(_wp(.35,p,r,FINGER_CLOSED,'follow the lever down with the button held in'))
    L.append(_wp(1.,p,r,FINGER_CLOSED,'keep the button in until the lever is on its stop'))
    L.append(_wp(1.5,off(0.,.060),press_rot,FINGER_CLOSED,'let the button out'))
    L.append(_wp(2.5,off(0.,.120)+np.array((0.,.05,.16)),press_rot,FINGER_CLOSED,'lift the pressing hand clear'))
    L.append(_wp(2.5,off(0.,.120)+np.array((0.,.08,.22)),press_rot,FINGER_CLOSED,'pressing hand clear and still'))
    return {'left':L,'right':R}

DEMO_MODE=os.environ.get('HB_DEMO_MODE','normal')
if DEMO_MODE not in ('normal','no_button','no_lift','let_go','empty_close'):raise ValueError('unknown demo control')
def demo():
    # Fixed waypoints computed from the declared geometry, loads and stock joint stiffness.
    # No observations, scorer state, layout name or pose writes enter this plan.
    plan={arm:list(ws) for arm,ws in _plan().items()}
    if DEMO_MODE=='normal':return plan
    total=max(sum(w.duration for w in ws) for ws in plan.values())
    if DEMO_MODE=='empty_close':
        # Fixed open-loop hardware-contact control, not a model/scorer branch.
        hover=point(R_HOLD,ANGLE)+np.array((0.,-math.sin(ANGLE),math.cos(ANGLE)))*.070
        rot=_rx(ANGLE)@grip_rotation(math.pi/2)
        plan['right']=[_wp(3.5,hover,rot,FINGER_OPEN,'empty shoe contact control: approach'),
                       _wp(2.,hover,rot,FINGER_CLOSED,'close against the opposite steel foot'),
                       _wp(1.,hover,rot,FINGER_CLOSED,'hold empty closure'),
                       _wp(2.,hover,rot,FINGER_OPEN,'reopen'),_wp(1.,hover,rot,FINGER_OPEN,'clear')]
        pos,rot=ik.tcp_pose(READY,station.BASES['left'])
        plan['left']=[_wp(9.5,pos,rot,FINGER_OPEN,'keep the pressing arm at its initial pose')]
    elif DEMO_MODE=='no_button':
        p=point(LENGTH,ANGLE)+np.array((0.,.15,.15))
        plan['left']=[_wp(total,p,_rx(-PRESS_TILT)@grip_rotation(math.pi/4),FINGER_CLOSED,'no-button native control')]
    elif DEMO_MODE=='no_lift':
        p=point(R_HOLD,ANGLE);r=_rx(ANGLE)@grip_rotation(math.pi/2)
        plan['right']=[_wp(3.5,p+np.array((0.,0.,.070)),r,FINGER_OPEN,'above grip'),
                       _wp(2.5,p,r,FINGER_OPEN,'at grip'),_wp(1.5,p,r,FINGER_CLOSED,'close without lifting'),
                       _wp(total-7.5,p,r,FINGER_CLOSED,'no-lift native control')]
    else:
        ws=plan['right'];p=point(R_HOLD,ANGLE)+np.array((0.,-.05,.18));r=_rx(ANGLE)@grip_rotation(math.pi/2)
        # Open and withdraw after the button press; the still-pressed lever then loses its support.
        plan['right']=ws[:6]+[_wp(.2,point(R_HOLD,ANGLE),r,FINGER_OPEN,'release support'),
                             _wp(total-sum(w.duration for w in ws[:6])-.2,p,r,FINGER_OPEN,'let-go native control')]
    return plan
