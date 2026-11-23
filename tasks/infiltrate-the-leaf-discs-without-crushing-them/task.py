"""A clear 50 mL syringe, fresh spinach discs, a cup and a small laboratory tray.
The syringe rests in two open saddles; its piston and barrel are separate loose parts.
"""
import json,math,os,pathlib,sys
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts,lots,tilt,observation
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint,FINGER_OPEN
SCENE='station'; TOP=TABLE_TOP[SCENE]; SCORE_OVER='end'; SUCCESS_HOLD_S=.1
CLEAR=(.72,.89,.96);GREY=(.56,.59,.62);GREEN=(.10,.38,.07);WHITE=(.92,.92,.88)
X=.29; Z=TOP+.090; BPOS=(X,0.,Z); AXIS=np.array((0.,1.,0.));AREA=math.pi*.013**2
# BD309653: current50mL product, formerly marked60mL. Dimensions are declared estimates.
wall=[parts.cylinder(.00075,.135,(.01375*math.cos(a),0,.01375*math.sin(a)),CLEAR,axis='y') for a in np.arange(64)*2*math.pi/64]
wall += [parts.box((.010,.004,.030),(x,-.0685,0),WHITE) for x in (-.020,.020)]
# Four end-cap sectors leave an actual4mm opening; the tip is a hollow ring of ribs.
wall += [parts.box((.012,.002,.028),(x,.0685,0),CLEAR) for x in (-.008,.008)]
wall += [parts.box((.004,.002,.012),(0,.0685,z),CLEAR) for z in (-.008,.008)]
wall += [parts.cylinder(.0012,.014,(.0028*math.cos(a),.0765,.0028*math.sin(a)),WHITE,axis='y') for a in np.arange(8)*math.pi/4]
BARREL=parts.Part('syringe_barrel',BPOS,shapes=wall,mass=.034,friction=(.4,.3))
PLUNGER=parts.Part('plunger',(X,0.,Z),mass=.018,friction=(.7,.6),shapes=[parts.cylinder(.0126,.012,(0,0,0),GREY,axis='y'),parts.box((.018,.116,.018),(0,-.064,0),WHITE),parts.cylinder(.0225,.005,(0,-.126,0),WHITE,axis='y')])
ss=[]
for y in (-.043,.043):
 ss += [parts.box((.070,.008,.006),(0,y,.003),GREY),parts.box((.008,.008,.073),(-.030,y,.0395),GREY),parts.box((.008,.008,.073),(.030,y,.0395),GREY),parts.box((.050,.008,.006),(0,y,.073),GREY)]
ss += [parts.box((.008,.006,.034),(x,-.075,.085),GREY) for x in (-.022,.022)]
ss += [parts.box((.046,.012,.004),(-.014,-.025,.109),GREY),parts.box((.006,.012,.036),(-.034,-.025,.093),GREY)]
STAND=parts.Part('open_saddles',(X,0.,TOP),shapes=ss)
def cup_shapes():
 return [parts.cylinder(.032,.002,(0,0,.001),CLEAR)]+[parts.cylinder(.0015,.056,(.032*math.cos(a),.032*math.sin(a),.030),CLEAR) for a in np.arange(32)*2*math.pi/32]
CUP=parts.Part('clear_cup',(.28,.17,TOP),shapes=cup_shapes(),mass=.052,friction=(.8,.6))
TRAY=parts.Part('plunger_tray',(.20,-.24,TOP),shapes=[parts.box((.13,.11,.004),(0,0,.002),(.6,.72,.8))])
DISCS=[parts.Part(f'leaf_{i+1}',(X, .020+i*.0006,Z-.003),shapes=[parts.cylinder(.003,.0003,(0,0,0),GREEN)]) for i in range(10)]
OBJECTS=[BARREL,PLUNGER,STAND,CUP,TRAY,*DISCS]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={CUP.name:CUP.pos}),lots.Lot('B',place={CUP.name:(.25,.18,TOP)}))
DEMO_TOUCHES={'open_saddles'}

def finite(s):
 try:return np.isfinite(np.r_[s.pos,s.quat_xyzw,s.lin_vel,s.ang_vel]).all() and abs(np.linalg.norm(s.quat_xyzw)-1)<.01
 except (AttributeError,ValueError,TypeError):return False

def geometry(b,p):
 R=parts._quat_to_matrix(b.quat_xyzw);u=R@AXIS;local=R.T@(p.pos-b.pos)
 return R,u,float(-local[1]),float(np.linalg.norm(local[[0,2]])),b.pos+R@np.array((0,.0835,0)),b.pos+R@np.array((0,-.0685,0))

_disk_x=np.linspace(-.032,.032,129)
_disk_w=2*np.sqrt(np.maximum(0.,.032**2-_disk_x**2))
def cup_capacity_ml(rzz):
 if not np.isfinite(rzz):raise ValueError('unknown cup orientation')
 if rzz<=0:return 0.
 tan=math.sqrt(max(0.,1-rzz*rzz))/max(rzz,1e-9)
 height=np.clip(.056-.032*tan+_disk_x*tan,0.,.056)
 return float(np.trapezoid(_disk_w*height,_disk_x)*1e6) if hasattr(np,'trapezoid') else float(np.trapz(_disk_w*height,_disk_x)*1e6)

def cup_capture(c,mouth):
 # Declared draining follows gravity through the cup's actual rotated rim plane.
 mouth=np.asarray(mouth,dtype=float)
 if not finite(c) or mouth.shape!=(3,) or not np.isfinite(mouth).all():raise ValueError('unknown cup capture geometry')
 R=parts._quat_to_matrix(c.quat_xyzw);normal=R[:,2]
 if normal[2]<=.05:return False
 rim=c.pos+R@np.array((0.,0.,.058))
 drop=float(np.dot(normal,mouth-rim)/normal[2])
 if not np.isfinite(drop):raise ValueError('unknown gravity intersection')
 if not -1e-6<=drop<=.072+1e-6:return False
 hit=mouth-np.array((0.,0.,drop));local=R.T@(hit-c.pos)
 return float(np.linalg.norm(local[:2]))<=.023+1e-6

class Tissue:
 """Declared ten-disc model: a full loaded stroke, held 10s, followed by venting.
 Each disc has an explicitly frozen exposure threshold, not a score awarded for the stroke alone.
 """
 def __init__(self):
  self.exposure=np.zeros(10);self.infiltrated=np.zeros(10,dtype=bool);self.crushed=np.zeros(10,dtype=bool)
  self.location=np.zeros(10,dtype=int);self.pos=np.zeros((10,3));self.drain=0.;self.zero=False;self.held=0.;self.time=0.;self.force=0.;self.sealed=False;self.clear=False;self.known=False;self.cert=None;self.down=False;self.wait=0.;self.removed=False;self.seal_q=0.;self.trapped_volume=30e-6;self.pressure=0.;self.vacuum_force=0.;self.seal_force=0.;self.best_exposure=np.zeros(10);self.syringe_ml=2.;self.cup_ml=40.;self.spilled_ml=0.;self.damage_load=np.zeros(10);self.mass_amounts=None;self.mass_cpu_buffers=[];self.mass_env_ids=None;self.native_masses=None;self.rest_anchor=None;self.cup_anchor=None
 def vacuum(self,q,v,sealed,aligned,dt):
  if not np.isfinite([q,v,dt]).all() or not 0<dt<=.100001:self.zero=True;return 0.
  was_sealed=self.sealed
  self.sealed=bool(sealed and aligned and -.050<=q<.061 and not self.removed)
  volume=30e-6+AREA*q if self.sealed else 30e-6
  if volume<=1e-8:self.zero=True;return 0.
  if self.sealed and not was_sealed:
   self.seal_q=q;self.trapped_volume=volume
  pressure=101325*(1-self.trapped_volume/volume) if self.sealed else 0.
  self.pressure=pressure
  alpha=1-math.exp(-dt/.12)
  target=float(np.clip(pressure*AREA,-32.,32.))
  self.vacuum_force+=(target-self.vacuum_force)*alpha
  seal_target=1.0*math.tanh(v/.004) if aligned and -.050<=q<.061 and not self.removed else 0.
  self.seal_force+=(seal_target-self.seal_force)*alpha
  # Passive impulse bound prevents20Hz friction from launching an18g piston.
  drag=math.copysign(min(abs(self.seal_force),.009*abs(v)/dt),v) if self.seal_force*v>0 else 0.
  if not self.sealed:self.vacuum_force=0.
  self.force=self.vacuum_force+drag
  loaded=(self.location==0)&~self.crushed
  eligible=self.sealed and .030-1e-6<=q<=.055+1e-6 and q-self.seal_q>=.030-1e-6 and pressure>=25000
  if eligible:self.exposure[loaded]+=dt
  else:
   self.best_exposure=np.maximum(self.best_exposure,self.exposure);self.exposure[:]=0.
  restored=(not self.sealed) or (was_sealed and q<=self.seal_q+.003 and pressure<6000)
  if restored:
   best=np.maximum(self.best_exposure,self.exposure)
   self.infiltrated[loaded] |= best[loaded]>=np.linspace(8.2,10.,10)[loaded]-1e-6
   self.exposure[:]=0.;self.best_exposure[:]=0.
  return self.force
 def value(self,accounted,sunk,released):
  if self.zero or not self.known or self.crushed.any() or not accounted:return 0.
  n=int(np.count_nonzero(sunk & self.infiltrated & ~self.crushed))
  if n==0:return 0.
  if not released or not self.down or self.wait<15.-1e-6:return 0.
  return .25 if n<5 else .5 if n<8 else .75 if n<10 else (1. if self.held>=1.-1e-6 else .75)
T=Tissue();_lastq=0.;_lastt=0.;_stage=None;_ops=[];_previous_positions=None

def clearance(clouds,objects,margin=.030):
 if len(clouds)!=6 or any(not len(c) or c.ndim!=2 or c.shape[1]!=3 or not np.isfinite(c).all() for c in clouds):return False
 for spec in (BARREL,PLUNGER,CUP):
  state=objects[spec.name]
  boxes=spec.world_boxes({'base':(state.pos,parts._quat_to_matrix(state.quat_xyzw))})
  if not boxes:return False
  low=np.min([c-np.abs(r)@(size/2) for _,c,r,size in boxes],axis=0)
  high=np.max([c+np.abs(r)@(size/2) for _,c,r,size in boxes],axis=0)
  center=(low+high)/2;half=(high-low)/2
  near=[p[np.linalg.norm(np.maximum(np.abs(p-center)-half,0),axis=1)<margin] for p in clouds]
  near=[p for p in near if len(p)]
  if not near:continue
  for _,c,r,size in boxes:
   if any(np.linalg.norm(np.maximum(np.abs((p-c)@r)-size/2,0),axis=1).min()<margin-1e-6 for p in near):return False
 return True

def finger_cap(clouds,tip,contact_flags):
 flags=np.asarray(contact_flags)
 if flags.shape!=(4,) or not np.isfinite(flags).all():raise ValueError('unknown sealing contacts')
 return any(bool(flags[j]) and float(np.linalg.norm(clouds[i]-tip,axis=1).min())<=.003 for i,j in ((1,0),(2,1),(4,2),(5,3)))

def tool_distance(spec,state,position):
 if not finite(state):raise ValueError('unknown tool state')
 distance=math.inf
 for _,c,r,size in spec.world_boxes({'base':(state.pos,parts._quat_to_matrix(state.quat_xyzw))}):
  q=np.abs((position-c)@r)-size/2
  signed=float(np.linalg.norm(np.maximum(q,0.))+min(float(q.max()),0.))
  distance=min(distance,signed)
 return distance

def tool_tissue_load(spec,state,position):
 return 1000*max(0.,.003-tool_distance(spec,state,position))

def rear_exit_clear(p,mouth):
 # The complete stopper AND stem must clear the 26mm rear aperture.
 return tool_distance(PLUNGER,p,mouth)>.016

def drop_path_clear(objects,clouds,mouth,c):
 # Declared3mm-radius gravity stream must pass solids and hands before the rim.
 R=parts._quat_to_matrix(c.quat_xyzw);n=R[:,2]
 end=mouth-np.array((0.,0.,float(np.dot(n,mouth-(c.pos+R@np.array((0.,0.,.058))))/n[2])))
 for spec in (BARREL,PLUNGER,STAND,TRAY):
  state=objects[spec.name]
  if not finite(state):raise ValueError('unknown drop obstruction')
  for _,center,rotation,size in spec.world_boxes({'base':(state.pos,parts._quat_to_matrix(state.quat_xyzw))}):
   a=(mouth-center)@rotation;d=(end-mouth)@rotation;half=size/2+.003
   lo=0.;hi=1.
   for j in range(3):
    if abs(d[j])<1e-12:
     if abs(a[j])>half[j]:lo=2.;break
    else:
     v=sorted(((-half[j]-a[j])/d[j],(half[j]-a[j])/d[j]));lo=max(lo,v[0]);hi=min(hi,v[1])
   if lo<=hi:return False
 delta=end-mouth;length2=float(np.dot(delta,delta))
 for cloud in clouds:
  if cloud.ndim!=2 or cloud.shape[1]!=3 or not len(cloud) or not np.isfinite(cloud).all():raise ValueError('unknown drop hand geometry')
  along=np.clip((cloud-mouth)@delta/max(length2,1e-12),0.,1.)
  if np.linalg.norm(cloud-(mouth+along[:,None]*delta),axis=1).min()<=.003+1e-9:return False
 return True

def quiet_from_pose(states,attribute):
 # A ruler/video stillness check: at most1mm and1degree from a fixed anchor.
 # Anchor is retained through the entire counted interval, so slow drift cannot accumulate.
 anchor=getattr(T,attribute)
 poses=[(s.pos.copy(),s.quat_xyzw.copy()) for s in states]
 quiet=anchor is not None and all(np.linalg.norm(pos-oldpos)<=.001+1e-9 and 2*math.acos(float(np.clip(abs(np.dot(quat,oldquat))/(np.linalg.norm(quat)*np.linalg.norm(oldquat)),0.,1.)))<=math.radians(1)+1e-9 for (pos,quat),(oldpos,oldquat) in zip(poses,anchor))
 if not quiet:setattr(T,attribute,poses)
 return quiet

def stamp(objects):return tuple(np.concatenate([np.r_[objects[n].pos,objects[n].quat_xyzw,objects[n].lin_vel,objects[n].ang_vel] for n in ('syringe_barrel','plunger','clear_cup')]))

def wrench(sim,name,force,point,state=None):
 import torch
 asset=sim.env.unwrapped.scene[name];s=state if state is not None else sim.states()[name]
 torque=np.cross(point-s.pos,force)
 f=torch.tensor(force,dtype=torch.float32,device=asset.device).reshape(1,1,3)
 m=torch.tensor(torque,dtype=torch.float32,device=asset.device).reshape(1,1,3)
 asset.set_external_force_and_torque(f,m,is_global=True)

def update_liquid_mass(sim):
 amounts=(T.syringe_ml,T.cup_ml)
 if T.mass_amounts==amounts:return
 import warp as wp
 # PhysX model properties consume CPU arrays. Keep ready buffers alive for the trial,
 # avoiding the SDK index setter's asynchronous GPU-to-pinned-CPU staging path.
 if T.mass_env_ids is None:
  T.mass_env_ids=wp.array(np.array([0],dtype=np.int32),dtype=wp.int32,device='cpu')
 T.native_masses={}
 for name,mass in [('syringe_barrel',.032+T.syringe_ml*.001),('clear_cup',.012+T.cup_ml*.001)]:
  asset=sim.env.unwrapped.scene[name]
  if asset.num_instances!=1 or asset.num_bodies!=1:raise ValueError('mass writer requires one body/environment')
  cpu=wp.array(np.array([[mass]],dtype=np.float32),dtype=wp.float32,device='cpu',pinned=True)
  T.mass_cpu_buffers.append(cpu)
  wp.copy(asset.data._body_mass,cpu);wp.synchronize()
  asset.root_view.set_masses(cpu,indices=T.mass_env_ids);wp.synchronize()
  actual=np.asarray(asset.root_view.get_masses().numpy(),dtype=float).reshape(-1)
  if actual.size!=1 or not np.isfinite(actual).all() or abs(float(actual[0])-mass)>1e-6:raise ValueError('native mass property mismatch')
  T.native_masses[name]=float(actual[0])
 T.mass_amounts=amounts

@parts.on_reset
def reset(sim):
 global T,_lastq,_lastt,_stage,_ops,_previous_positions
 T=Tissue();_lastq=geometry(sim.states()[BARREL.name],sim.states()[PLUNGER.name])[2];_lastt=0.;_previous_positions=None;observation.reset()
 from isaaclab.app.settings_manager import get_settings_manager
 get_settings_manager().set_bool('/physics/disableContactProcessing',False)
 from isaaclab.sim.utils import get_current_stage
 from pxr import UsdGeom,UsdPhysics
 _stage=get_current_stage();_ops=[]
 from pxr import UsdPhysics
 guide=UsdPhysics.Joint(_stage.GetPrimAtPath(sim.env.unwrapped.scene.env_prim_paths[0]+'/plunger/removable_bearing'));guide.CreateJointEnabledAttr(True)
 base=sim.env.unwrapped
 for d in DISCS:
  path=base.scene.env_prim_paths[0]+'/'+d.name
  prim=_stage.GetPrimAtPath(path)
  if not prim.IsValid():raise RuntimeError('missing disc visual')
  x=UsdGeom.Xformable(prim);ops=x.GetOrderedXformOps();op=next(a for a in ops if a.GetOpType()==UsdGeom.XformOp.TypeTranslate);_ops.append(op);op.Set(__import__('pxr.Gf',fromlist=['Vec3d']).Vec3d(*d.pos))
  for child in prim.GetChildren():
   if child.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)

@parts.on_step
def step(sim):
 global _lastq,_lastt,_previous_positions
 objects=sim.states();b=objects[BARREL.name];p=objects[PLUNGER.name];c=objects[CUP.name];dt=float(sim.t)-_lastt
 try:
  if not all(finite(s) for s in (b,p,c)):raise ValueError('nonfinite object')
  clouds,load=observation.hand_state(sim)
  if not np.isfinite(load):raise ValueError('nonfinite force')
  R,u,q,rad,tip,mouth=geometry(b,p);v=(q-_lastq)/dt if dt>0 else 0.
  # Either hand may seal; the same fingertip must touch the native barrel at the outlet.
  pairs=observation.pair_forces(sim)
  cap=finger_cap(clouds,tip,np.linalg.norm(pairs[:,1,:],axis=1)>.05)
  aligned=rad<.002 and np.dot(parts._quat_to_matrix(p.quat_xyzw)@AXIS,u)>.90
  force=T.vacuum(q,v,cap,aligned,dt)
  if not np.isfinite([force,T.pressure]).all():raise ValueError('nonfinite vacuum load')
  if q>.078 and aligned:
   from pxr import UsdPhysics
   guide=UsdPhysics.Joint(_stage.GetPrimAtPath(sim.env.unwrapped.scene.env_prim_paths[0]+'/plunger/removable_bearing'));guide.CreateJointEnabledAttr(False);T.removed=True
  wrench(sim,PLUNGER.name,force*u,p.pos,p);wrench(sim,BARREL.name,-force*u,tip,b)
  # Crushing permanently zeros the trial: piston enters the 10mm specimen zone under >1N,
  # or a sampled hand intrudes into a loaded or exposed disc. The threshold is a conservative declared estimate.
  # Direct axial contact and local compliant intrusion cover supported-piston compression.
  pair_force=-pairs[:,0,:].sum(0)
  axial=abs(float(np.dot(pair_force,u)))
  if aligned and rad<.002:
   for i in range(10):
    if T.location[i]==0 and .006-q>=.017+(i//2)*.0012:
     T.damage_load[i]=max(T.damage_load[i],axial)
     if axial>1.:T.crushed[i]=True
  if load>40.01:T.zero=True
  origin=parts._np(sim.env.unwrapped.scene.env_origins[0])
  draining=T.removed and rear_exit_clear(p,mouth) and u[2]>.35
  capture=cup_capture(c,mouth) if draining else False
  flowing=draining and (not capture or drop_path_clear(objects,clouds,mouth,c))
  for i in range(10):
   if T.location[i]==0:
    T.pos[i]=b.pos+R@np.array(((-1 if i%2 else 1)*.002, .020+(i//2)*.0012,-.002))
    local_force=tool_tissue_load(PLUNGER,p,T.pos[i])
    T.damage_load[i]=max(T.damage_load[i],local_force)
    if local_force>1.:T.crushed[i]=True
    nearest=min(float(np.linalg.norm(cloud-T.pos[i],axis=1).min()) for cloud in clouds)
    hand_force=1000*max(0.,.003-nearest)
    T.damage_load[i]=max(T.damage_load[i],hand_force)
    if hand_force>1.:T.crushed[i]=True
    if flowing:
     # Open rear is downhill: its actual mouth must project inside the actual cup rim.
     if capture:
      T.drain+=dt/10
      if T.drain>=(i+1)*.12:T.location[i]=1
     else:T.location[i]=2;T.zero=True
   if T.location[i]==1:
    a=i*2*math.pi/10;T.pos[i]=c.pos+parts._quat_to_matrix(c.quat_xyzw)@np.array((.018*math.cos(a),.018*math.sin(a),.003 if T.infiltrated[i] else .0125))
   if T.location[i]==1:
    # Declared compliant tissue: 1000N/m contact, rather than an unrelated hand-load maximum.
    nearest=min(float(np.linalg.norm(cloud-T.pos[i],axis=1).min()) for cloud in clouds)
    local_force=1000*max(0.,.003-nearest)
    for tool in (BARREL,PLUNGER,STAND,TRAY):
     local_force=max(local_force,tool_tissue_load(tool,objects[tool.name],T.pos[i]))
    T.damage_load[i]=max(T.damage_load[i],local_force)
    if local_force>1.:T.crushed[i]=True
   from pxr import Gf
   _ops[i].Set(Gf.Vec3d(*(T.pos[i]+origin)))
  # Declared liquid inventory is a constraint only; the score still counts tissue.
  if flowing and T.syringe_ml>0:
   amount=min(T.syringe_ml,dt)
   T.syringe_ml-=amount
   if capture:T.cup_ml+=amount
   else:T.spilled_ml+=amount
  rzz=float(parts._quat_to_matrix(c.quat_xyzw)[2,2]);capacity=cup_capacity_ml(rzz)
  if rzz<=0.:T.zero=True
  if T.cup_ml>capacity:T.spilled_ml+=T.cup_ml-capacity;T.cup_ml=capacity
  if T.spilled_ml>2.+1e-6 or c.pos[2]<TOP-.005:T.zero=True
  if abs(T.syringe_ml+T.cup_ml+T.spilled_ml-42.)>1e-6:raise ValueError('liquid inventory')
  update_liquid_mass(sim)
  if any(s.pos[2]<TOP-.005 or np.any(np.abs(s.pos[:2])>.60) for s in (b,p,c)):T.zero=True
  T.clear=clearance(clouds,objects)
  down=(b.low[2]<=TOP+.008 or (b.low[2]<=TOP+.079 and np.linalg.norm(b.pos[:2]-np.array(BPOS[:2]))<.012)) and p.low[2]<=TOP+.008 and rzz>=math.cos(math.radians(5))-1e-9
  T.down=bool(down)
  still=quiet_from_pose((b,p,c),'rest_anchor')
  cup_still=quiet_from_pose((c,),'cup_anchor')
  settled=bool(np.all(T.location==1))
  T.held=T.held+dt if T.clear and down and still and settled else 0.
  T.wait=T.wait+dt if T.clear and down and cup_still and settled else 0.
  T.known=True;T.cert=stamp(objects);T.time=float(sim.t)
 except (AttributeError,KeyError,ValueError,IndexError,TypeError,RuntimeError) as err:
  T.zero=True;T.known=False
  if os.environ.get('TRACE_OUT'):print('leaf model error:',str(err),flush=True)
 _lastt=float(sim.t)
 if 'q' in locals():_lastq=q
 if os.environ.get('TRACE_OUT'):
  row=dict(cup_quat=c.quat_xyzw.tolist(),native_masses=T.native_masses,lot=lots.current().name,t=sim.t,cap_distances=[float(np.linalg.norm(clouds[i]-tip,axis=1).min()) for i in (1,2,4,5)] if 'clouds' in locals() and 'tip' in locals() else None,cap_contacts=np.linalg.norm(pairs[:,1,:],axis=1).tolist() if 'pairs' in locals() else None,pair_forces=pairs.tolist() if 'pairs' in locals() else None,removed=T.removed,wait=T.wait,down=T.down,clear=T.clear,barrel_quat=b.quat_xyzw.tolist(),plunger_quat=p.quat_xyzw.tolist(),velocity=locals().get('v'),q=locals().get('q'),radial=locals().get('rad'),cap=locals().get('cap'),aligned=locals().get('aligned'),force=T.force,pressure=T.pressure,seal_q=T.seal_q,liquid=[T.syringe_ml,T.cup_ml,T.spilled_ml],exposure=T.exposure.tolist(),infiltrated=T.infiltrated.tolist(),location=T.location.tolist(),crushed=T.crushed.tolist(),zero=T.zero,known=T.known,held=T.held,barrel=b.pos.tolist(),plunger=p.pos.tolist(),cup=c.pos.tolist(),load=locals().get('load'),score=score(objects))
  from yam_arena.constants import TCP_OFFSET
  base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);row['hands']={}
  for arm in ('left_robot','right_robot'):
   asset=base.scene[arm];pose=parts._np(asset.data.body_link_pose_w)[0,asset.body_names.index('gripper')];joints=parts._np(asset.data.joint_pos)[0]
   row['hands'][arm]=dict(orientation_raw=pose[3:].tolist(),tcp=(pose[:3]-origin+parts._quat_to_matrix(pose[3:])@np.array(TCP_OFFSET)).tolist(),fingers={n:float(joints[asset.joint_names.index(n)]) for n in ('joint7','joint8')})
  with pathlib.Path(os.environ['TRACE_OUT']).with_name('leaf.jsonl').open('a') as f:f.write(json.dumps(row,default=lambda v: v.item() if isinstance(v,np.generic) else v.tolist())+'\n')

def score(objects):
 try:
  if T.cert!=stamp(objects) or not all(finite(objects[n]) for n in ('syringe_barrel','plunger','clear_cup')):return 0.
  return T.value(bool(np.all(T.location==1)),T.location==1,T.clear)
 except (ValueError,KeyError,AttributeError,TypeError):return 0.
def success(objects):return score(objects)==1.

def demo():
 # Fixed motions from the published visible layout; no hidden-state feedback.
 cup=np.asarray(lots.current().pos(CUP.name));left=[];right=[]
 pick_x=.280;grip_y=.005;rear=.0685+grip_y;pour_x=float(cup[0])-rear*math.cos(math.radians(24))
 def L(t,x,y,z,f=FINGER_OPEN,label='move',**kw):left.append(tilt.Waypoint(t,(x,y,TOP+z),f,label,**kw))
 def R(t,x,y,z,f=FINGER_OPEN,label='move',**kw):right.append(tilt.Waypoint(t,(x,y,TOP+z),f,label,yaw=math.pi/2,**kw))
 L(2,X,.131,.16);L(3,X,.131,.16);L(2,X,.131,.093);L(2,X,.126,.093,label='cap outlet');L(20.5,X,.126,.093,label='hold finger cap')
 R(2,X,-.105,.16);R(2,X,-.105,.090);R(2,X,-.105,.090,-.0045,label='grasp plunger stem');R(3,X,-.105,.090,-.0045,label='wait for finger cap')
 R(5,X,-.230,.090,-.0045,label='loaded vacuum draw');R(12,X,-.230,.090,-.0045,label='hold vacuum');R(3.5,X,-.105,.090,-.0045,label='restore pressure')
 R(3,X,-.305,.090,-.0045,label='remove piston');R(2,.20,-.24,.035,-.0045);R(1,.20,-.24,.035,label='release plunger');R(2,.18,-.27,.15)
 L(2,X,.13,.14);L(3,pick_x,grip_y,.16);L(2,pick_x,grip_y,.093,yaw=-math.pi/2);L(1,pick_x,grip_y,.093,-.0120,label='central barrel purchase',yaw=-math.pi/2)
 L(2,X+.04,grip_y,.093,-.0120,label='slide clear of open side',yaw=-math.pi/2)
 L(2,X+.06,grip_y,.14,-.0120,label='lift barrel',yaw=-math.pi/2,tilt=10,heading=-math.pi/2)
 L(2,X+.06,grip_y,.14,-.0120,label='qualify lifted barrel',yaw=-math.pi/2,tilt=10,heading=-math.pi/2)
 L(4,pour_x,float(cup[1]),.15,-.0120,yaw=0,tilt=20,heading=0)
 L(3,pour_x+.008,float(cup[1]),.14,-.0120,label='pour through open rear',yaw=0,tilt=24,heading=math.pi)
 L(6,pour_x+.008,float(cup[1]),.14,-.0120,yaw=0,tilt=24,heading=math.pi)
 L(2,pour_x+.008,float(cup[1]),.17,-.0120,label='lift clear of cup',yaw=0)
 L(4,.17,.02,.14,-.0120,yaw=-math.pi/2);L(3,.17,.02,.022,-.0120,yaw=-math.pi/2)
 L(1,.17,.02,.022,label='set syringe on clear counter',yaw=-math.pi/2);L(2,.20,.20,.15);L(20,.20,.20,.15,label='undisturbed count')
 return dict(left=left,right=right)
