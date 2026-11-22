"""A Toyota Yaris parking-brake lever lies between two seat-bolster envelopes and a slotted centre console.
The lever, installed bracket and axial release button retain their declared full dimensions. A 200 mm extension on a 512 mm custom spine adds leverage through keyed adapters. An angle scale is beside the lever.
"""
from __future__ import annotations
import json, math, os, pathlib, sys
from dataclasses import replace
from functools import lru_cache
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts, lots, tilt
from yam_arena.task_file import TABLE_TOP, Box
from yam_arena.demo import Waypoint, grip_rotation
SCENE='station'; TOP=TABLE_TOP[SCENE]; SCORE_OVER='end'; SUCCESS_HOLD_S=0.
VERSION='0.10.1'
ANGLE=math.radians(24); PITCH=math.radians(4); LENGTH=.297
CABLE_K=112.2; CABLE_PRELOAD=3.; APPLIED_MOMENT=CABLE_PRELOAD+CABLE_K*ANGLE
CABLE_MODE=os.environ.get("HB_CABLE_MODEL","effort")
BUTTON_K=1000.; BUTTON_RELEASE=.005; BUTTON_TRAVEL=.008
YAW=1.1974210820574076; BASE_R=parts._rot((0,0,1),YAW)
PIVOT=np.array((.670,-.100,TOP+.1725)); AXIS=np.array((1.,0.,0.))
WORLD_AXIS=BASE_R@AXIS
def installed(v):return tuple(float(x) for x in PIVOT+BASE_R@np.asarray(v))
R0=parts._rot(AXIS,ANGLE); U0=R0@np.array((0.,1.,0.))
INSTRUCTION='Raise the lever slightly, hold its end button in while lowering it fully, then move both hands clear.'
STEEL=(.58,.60,.62); BLACK=(.06,.06,.07); TRIM=(.17,.18,.20); BLUE=(.19,.25,.35)

def turn(shape):
    return replace(shape,at=tuple(R0@np.asarray(shape.at)),quat=tuple(parts._matrix_to_quat(R0)))
lever_shapes=[turn(parts.box((.012,.20,.026),(0,.070,0),STEEL)),
              turn(parts.cylinder(.0175,.102,(0,.221,0),BLACK,axis='y')),
              turn(parts.cylinder(.0125,.017,(0,.2805,0),STEEL,axis='y'))]
# Full-size 200 mm RRS DTMVHHBE extension on a declared offset clamp adapter.
# The stock axial button is untouched and remains independently accessible.
lever_shapes.extend([turn(parts.box((.040,.020,.040),(0,r,0),STEEL)) for r in (.100,.150)])
lever_shapes.extend([turn(parts.box((.120,.020,.020),(-.040,r,0),STEEL)) for r in (.100,.150)])
lever_shapes.extend([turn(parts.box((.020,.512,.020),(-.090,.346,0),STEEL)),
 turn(parts.cylinder(.015,.100,(-.090,.642,0),(.16,.16,.17),axis='y')),
 replace(turn(parts.cylinder(.01607,.100,(-.090,.742,0),(.15,.28,.20),axis='y')),friction=(1.6,1.4),friction_combine="max")])
button_shape=turn(parts.cylinder(.0075,.016,(0,.289,0),(.84,.84,.86),axis='y'))
LEVER=parts.Hinge('lever',tuple(lever_shapes),(0.,0.,0.),(1.,0.,0.),-ANGLE,math.radians(12),mass=1.57,
                  spring=CABLE_K,damping=1.5,max_effort=100.)
BUTTON=parts.Slider('button',(button_shape,),tuple(-U0),BUTTON_TRAVEL,mass=.020,spring=BUTTON_K,damping=2.,max_effort=200.,parent='lever')
BRAKE=parts.Part('parking_brake',pos=tuple(float(v) for v in PIVOT),yaw=YAW,shapes=[parts.box((.060,.060,.008),(0,0,-.038),STEEL),
  parts.cylinder(.007,.040,(0,0,0),STEEL,axis='x')],joints=[LEVER,BUTTON],friction=(.8,.7))
# These are rigid access envelopes of the inner portions of the two seats;
# the outer cushions do not bound the robot workspace. All dimensions are metres.
FLOOR=Box('installed_floor',pos=installed((0,.150,-.059)),size=(.440,.580,.018),yaw=YAW,color=(.36,.28,.18))
SEATS=[Box('seat_near',pos=installed((-.155,.150,.025)),size=(.090,.560,.150),yaw=YAW,color=BLUE),
       Box('seat_far',pos=installed((.155,.150,.025)),size=(.090,.560,.150),yaw=YAW,color=BLUE)]
CONSOLE=[Box('console_near',pos=installed((-.060,.135,-.032)),size=(.040,.420,.036),yaw=YAW,color=TRIM),
         Box('console_far',pos=installed((.060,.135,-.032)),size=(.040,.420,.036),yaw=YAW,color=TRIM),
         Box('console_rear',pos=installed((0.,-.060,-.032)),size=(.080,.030,.036),yaw=YAW,color=TRIM)]
# A physical protractor face beside the button arc; static tick labels are angles.
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
        text=str(deg)
        for digit,char in enumerate(text):
            for row,line in enumerate(font[char]):
                for col,v in enumerate(line):
                    if v=='1':scale_shapes.append(replace(parts.box((.0045,.0045,.0045),c+(-.0003,(digit*4+col)*.0055,-row*.0055),(.95,.95,.95)),collide=False))
SCALE=parts.Part('angle_scale',pos=tuple(float(v) for v in PIVOT+BASE_R@np.array((.085,0,0))),yaw=YAW,shapes=scale_shapes)

OBJECTS=[BRAKE,FLOOR,*SEATS,*CONSOLE,SCALE]
# Mechanical installed pose remains identical; two visible loose cards provide declared layout trials.
CARD=Box('layout_card',pos=(.82,-.55,TOP+.005),size=(.060,.080,.010),mass=.04,color=(.8,.75,.1));OBJECTS.append(CARD)
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={CARD.name:CARD.pos}),lots.Lot('B',place={CARD.name:(.82,.55,TOP+.005)}))
# The task does not claim that these cosmetic lots prevent memorized replay.
from mechanism import Judge,Sample
from mechanism.native import Monitor
_judge=Judge(ANGLE,PITCH)
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

def write_limits(asset,low,button_hi):
    import torch
    ids=[asset.joint_names.index('lever'),asset.joint_names.index('button')]
    limits=torch.tensor([[[float(low),math.radians(12)],[0.,float(button_hi)]]],device=asset.device)
    asset.write_joint_position_limit_to_sim_index(limits=limits,joint_ids=ids)

def set_cable_effort(asset,theta):
    import torch
    ids=[asset.joint_names.index('lever')]
    asset.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[0.]],device=asset.device),joint_ids=ids)
    asset.set_joint_effort_target_index(target=torch.tensor([[-(CABLE_PRELOAD+CABLE_K*theta)]],device=asset.device),joint_ids=ids)
    asset.write_data_to_sim()

@parts.on_pre_reset
def before_reset(sim):
    if _monitor is not None:
        _monitor.disable();asset=sim.env.unwrapped.scene[BRAKE.name]
        write_limits(asset,0.,.001);set_cable_effort(asset,ANGLE)

@parts.on_reset
def reset(sim):
    global _monitor
    from isaaclab.app.settings_manager import get_settings_manager
    import torch
    get_settings_manager().set_bool('/physics/disableContactProcessing',False)
    base=sim.env.unwrapped;asset=base.scene[BRAKE.name]
    for side in ('left','right'):
        hand=base.scene[side+'_robot'];ids=[hand.joint_names.index(n) for n in ('joint7','joint8')]
        hand.write_joint_effort_limit_to_sim_index(limits=torch.tensor([[40.,40.]],device=hand.device),joint_ids=ids)
    write_limits(asset,0.,.001);set_cable_effort(asset,ANGLE)
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

def demo():
    # Fixed FK waypoints derived offline from the visible fixture and stock YAM limits.
    # No observations, hidden scorer state, layout ID or pose writes enter this plan.
    path=json.loads(pathlib.Path(__file__).with_name('mechanism').joinpath('demo_path.json').read_text())
    return {arm:[Waypoint(w['duration'],tuple(w['pos']),w['finger'],w['label'],tilt._TiltedYaw(0.,np.array(w['rot']))) for w in ws] for arm,ws in path.items()}
