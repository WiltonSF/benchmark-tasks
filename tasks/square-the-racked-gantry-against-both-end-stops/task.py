"""Two real-size captured carriage ends carry a racked tabletop gantry beam.
Both rail stops and the carriage groove lips are physical contact geometry.
"""
import json,math,os,pathlib,sys
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts,lots,tilt,observation
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint,FINGER_OPEN,grip_rotation
SCENE='station';TOP=TABLE_TOP[SCENE];SCORE_OVER='end';SUCCESS_HOLD_S=.1
GREY=(.58,.61,.65);SILVER=(.78,.8,.83);RED=(.78,.13,.09)
RAIL_Y=.170;STOP_X=.280;BLOCK_X=.0454;BEAM_Z=TOP+.023;HOME=STOP_X-BLOCK_X/2
rails=[];stops=[]
for sign,label in ((1,'left'),(-1,'right')):
 rails.append(parts.Part(label+'_captured_rail',pos=(.155,sign*RAIL_Y,TOP),shapes=[
  parts.box((.250,.012,.003),(0,0,.0015),GREY),
  parts.box((.250,.008,.002),(0,0,.004),GREY),
  parts.box((.250,.012,.003),(0,0,.0065),GREY)],friction=(.2,.2)))
 stops.append(parts.Part(label+'_stop',pos=(STOP_X+.005,sign*RAIL_Y,TOP+.010),
  shapes=[parts.box((.010,.030,.020),(0,0,0),RED)],friction=(.2,.2)))
shapes=[parts.box((.020,.400,.020),(0,0,0),SILVER),
 parts.box((.040,.050,.040),(.030,0,.010),(.94,.38,.06))]
for sign in (1,-1):
 y=sign*RAIL_Y
 shapes.append(parts.box((BLOCK_X,.027,.00475),(0,y,.010625-.023),GREY))
 for side in (1,-1):
  shapes.append(parts.box((BLOCK_X,.00725,.010),(0,y+side*.009875,.008-.023),GREY))
  shapes.append(parts.box((BLOCK_X,.002,.0015),(0,y+side*.00525,.004-.023),GREY))
BEAM=parts.Part('captured_gantry',pos=(.216,0,BEAM_Z),yaw=math.radians(-.45),mass=.8,shapes=shapes,friction=(.2,.2))
OBJECTS=[*rails,*stops,BEAM]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={BEAM.name:BEAM.pos},yaw={BEAM.name:math.radians(-.45)}),
 lots.Lot('B',place={BEAM.name:(.205,0,BEAM_Z)},yaw={BEAM.name:math.radians(.45)}))

def stamp(state):return tuple(np.r_[state.pos,state.quat_xyzw,state.lin_vel,state.ang_vel])

def measure(state):
 p=np.asarray(state.pos);q=np.asarray(state.quat_xyzw)
 if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(np.r_[p,q,state.lin_vel,state.ang_vel]).all() or np.linalg.norm(q)<1e-6:
  return dict(known=False,engaged=False,face=False,near=False,fronts=np.zeros(2),gaps=np.full((2,4),np.inf),height_error=math.inf,yaw=math.inf)
 r=parts._quat_to_matrix(q);gaps=[];fronts=[];engaged=True;height_error=0.
 yaw=abs(math.atan2(r[1,0],r[0,0]));upright=math.acos(float(np.clip(r[2,2],-1,1)))
 for sign in (1,-1):
  corners=np.array([(.0227,sign*RAIL_Y+y,z-.023) for y in (-.0135,.0135) for z in (.003,.013)])@r.T+p
  center=p+r@np.array((0,sign*RAIL_Y,.008-.023));fronts.append(float(corners[:,0].mean()))
  gg=STOP_X-corners[:,0];gaps.append(gg)
  top=np.array([(x,sign*RAIL_Y+y,.013-.023) for x in (-.0227,.0227) for y in (-.0135,.0135)])@r.T+p
  hz=float(np.max(np.abs(top[:,2]-(TOP+.013))));height_error=max(height_error,hz)
  engaged &= bool(abs(center[1]-sign*RAIL_Y)<=.0006 and hz<=.001 and yaw<=math.radians(.7) and upright<=math.radians(.7) and center[0]-.0227>=.030-.0005 and gg.min()>=-.0001)
 gaps=np.asarray(gaps)
 near=engaged and bool(gaps.min()>=-.0001 and gaps.max()<=.005)
 face=engaged and height_error<=.0005 and bool(gaps.min()>=-.0001 and gaps.max()<=.00005)
 return dict(known=True,engaged=engaged,face=face,near=near,fronts=np.array(fronts),gaps=gaps,height_error=height_error,yaw=yaw)

def released(clouds,state):
 if not clouds or any(p.ndim!=2 or p.shape[1]!=3 or not len(p) or not np.isfinite(p).all() for p in clouds):return False
 if not measure(state)['known']:return False
 for spec,s in [(BEAM,state),*((p,p.state()) for p in (*rails,*stops))]:
  for _,center,r,size in spec.world_boxes({'base':(s.pos,parts._quat_to_matrix(s.quat_xyzw))}):
   if any(np.linalg.norm(np.maximum(np.abs((p-center)@r)-size/2,0.),axis=1).min()<=.012 for p in clouds):return False
 return True

_known=False;_invalid=False;_sim_feasible=True;_peak=0.;_time=0.;_held=0.;_previous=None;_initial=None;_certificate=();_slid=np.zeros(2);_offrail_travel=0.;_previous_eligible=False
@parts.on_reset
def reset(sim):
 global _known,_invalid,_sim_feasible,_peak,_time,_held,_previous,_initial,_certificate,_slid,_offrail_travel,_previous_eligible
 _known=False;_invalid=False;_sim_feasible=True;_peak=0.;_time=0.;_held=0.;_certificate=();_slid=np.zeros(2);_offrail_travel=0.
 _previous=None;_initial=None;_previous_eligible=False;observation.reset()
 from isaaclab.app.settings_manager import get_settings_manager
 get_settings_manager().set_bool('/physics/disableContactProcessing',False)
@parts.on_step
def step(sim):
 global _known,_invalid,_sim_feasible,_peak,_time,_held,_previous,_initial,_certificate,_slid,_offrail_travel,_previous_eligible
 s=sim.states()[BEAM.name];m=measure(s);dt=float(sim.t)-_time;clear=False
 try:
  clouds,force=observation.hand_state(sim);_known=True;_peak=max(_peak,force);_sim_feasible &= force<=40.01;clear=released(clouds,s)
 except (ValueError,AttributeError,KeyError,IndexError):_known=False;_invalid=True
 if not m['known'] or not 0<dt<=.100001:_invalid=True;_known=False
 if _previous is None and m['known']:_initial=m['fronts'].copy()
 if _previous is not None and m['known']:
  delta=m['fronts']-_previous['fronts']
  if _previous['engaged'] and m['engaged']:_slid+=delta
  else:_offrail_travel+=float(np.linalg.norm(delta))
 still=s.resting(.003,.03)
 eligible=_known and not _invalid and m['face'] and clear and still
 _held=_held+dt if eligible and _previous_eligible else 0.
 _previous_eligible=eligible
 _time=float(sim.t);_previous=m;_certificate=stamp(s)
 if os.environ.get('TRACE_OUT'):
  row=dict(t=_time,pos=s.pos.tolist(),quat=s.quat_xyzw.tolist(),velocity=s.lin_vel.tolist(),angular=s.ang_vel.tolist(),
   known=_known,invalid=_invalid,sampled_sim_feasible=bool(_sim_feasible),peak_finger=_peak,clear=bool(clear),held=_held,slid=_slid.tolist(),offrail_travel=_offrail_travel,
   metrics={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in m.items()},score=score({BEAM.name:s}))
  from yam_arena.constants import TCP_OFFSET
  base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);row['grippers']={}
  for arm in ('left_robot','right_robot'):
   robot=base.scene[arm];pose=parts._np(robot.data.body_link_pose_w)[0,robot.body_names.index('gripper')];joints=parts._np(robot.data.joint_pos)[0]
   row['grippers'][arm]=dict(tcp=(pose[:3]-origin+parts._quat_to_matrix(pose[3:])@np.array(TCP_OFFSET)).tolist(),quat=pose[3:].tolist(),joint_q={n:float(joints[robot.joint_names.index(n)]) for n in robot.joint_names})
  with pathlib.Path(os.environ['TRACE_OUT']).with_name('gantry.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')

def score(objects):
 try:
  s=objects[BEAM.name];m=measure(s)
  if not _known or _invalid or not m['known'] or not m['engaged'] or _offrail_travel>.002 or _certificate!=stamp(s) or np.min(_slid)<.025:return 0.
  value=.25
  if _offrail_travel<=.002 and m['near'] and np.all(_slid>=m['fronts']-_initial-.002):
   value=.5
   if m['face']:
    value=.75
    if _held>=1.-1e-9:value=1.
  return value
 except (ValueError,AttributeError,KeyError,TypeError):return 0.
def success(objects):return score(objects)==1.
def demo():
 p=np.asarray(lots.current().pos(BEAM.name));yaw=lots.current().yaw.get(BEAM.name,BEAM.yaw);r=parts._rot((0,0,1),yaw);plan={}
 for arm,sign,angle in [('left',1,-math.pi/2),('right',-1,math.pi/2)]:
  start=p+r@np.array((0,sign*RAIL_Y,.014));end=np.array((HOME+.007+(.0025 if arm=='left' else 0.),sign*RAIL_Y,TOP+.037))
  orient=grip_rotation(angle+yaw);endrot=grip_rotation(angle)
  def w(t,point,f,label,rot=orient):return Waypoint(t,tuple(point),f,label,tilt._TiltedYaw(0,rot))
  plan[arm]=[w(2.,start+(0,0,.075),FINGER_OPEN,'approach beam end'),w(2.,start,FINGER_OPEN,'lower to beam'),
   w(1.5,start,-.0075,'grasp beam end'),w(1.,start,-.0075,'settle'),w(5.,end,-.0075,'advance carriages together',endrot),
   w(2.,end,-.0075,'hold stops',endrot),w(1.5,end,FINGER_OPEN,'release',endrot),w(2.,end+(0,0,.080),FINGER_OPEN,'withdraw',endrot),
   w(3.,end+(0,0,.080),FINGER_OPEN,'released stillness',endrot)]
 return plan
