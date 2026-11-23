"""A Leica RM2125 RTS procedure fixture occupies the station bench. A white cassette with a pale specimen block
is held upright above an exposed knife in holder E. The handwheel handle is pulled out, and the red hinged guard
rests forward. A loose grey tray sits to the left. The housing, specimen guide and knife holder limit access.
"""
from __future__ import annotations
import math, json, os, pathlib, sys, hashlib
from functools import lru_cache
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts, lots, tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint, FINGER_OPEN, grip_rotation
SOURCE_SHA256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()
SCENE='station'; TOP=TABLE_TOP[SCENE]; SCORE_OVER='end'; SUCCESS_HOLD_S=.05
CREAM=(.87,.86,.81); DARK=(.22,.24,.27); METAL=(.66,.69,.72); RED=(.84,.13,.08)
A0=-math.pi/3; R=.0295; KNOB_R=.085
C0=.190-R*(1-math.cos(A0)); CX=.300; CY=.055
WPOS=(.380,-.255,TOP+.140)
GPOS=(.2505,CY,TOP+.117)
# Helpers take coordinates relative to the bench, making the fabrication coordinates explicit.
def bx(size,center,color=DARK):return parts.box(size,at=center,color=color)
def cyl(r,h,p,color=METAL,axis='y'):return parts.cylinder(r,h,at=p,color=color,axis=axis)
# Full 438 x 472 x 265 mm working envelope. No scaled-down machine.
HOUSING=parts.Part('instrument_housing',pos=(.596,-.016,TOP),shapes=(
 bx((.472,.438,.018),(0,0,.009),CREAM),bx((.012,.438,.247),(.230,0,.1415),CREAM),
 bx((.448,.012,.247),(0,.213,.1415),CREAM),bx((.430,.012,.247),(.021,-.213,.1415),CREAM),
 bx((.026,.012,.106),(-.223,-.213,.071),CREAM),bx((.026,.012,.109),(-.223,-.213,.2105),CREAM),
 bx((.472,.438,.012),(0,0,.259),CREAM),
 bx((.012,.180,.235),(-.230,-.123,.1355),CREAM),bx((.012,.126,.235),(-.230,.150,.1355),CREAM)))
# Side lock receiver has a real 13 mm square aperture. The shaft and pin have separate holes.
receiver=[]
# Full backplate blocks inward pin travel everywhere except the shaft and upper-detent holes.
for x0,x1,z0,z1 in [(-.100,-.016,-.100,.110),(.016,.100,-.100,.110),
 (-.016,.016,-.100,-.016),(-.016,.016,.016,.0785),(-.016,.016,.0915,.110),
 (-.016,-.0065,.0785,.0915),(.0065,.016,.0785,.0915)]:
 receiver.append(bx((x1-x0,.012,z1-z0),((x0+x1)/2,-.002,(z0+z1)/2),CREAM))
LOCK_WALL=parts.Part('lock_receiver',pos=WPOS,shapes=tuple(receiver))
knob0=np.array((KNOB_R*math.sin(A0),-.075,KNOB_R*math.cos(A0)))
pin0=np.array((R*math.sin(A0),.294,R*math.cos(A0)))
wheel=parts.Hinge('wheel',pivot=(0,0,0),axis=(0,1,0),lower=-4*math.pi,upper=4*math.pi,mass=.45,
 damping=1000.,max_effort=.58,shapes=(cyl(.100,.012,(0,-.018,0),CREAM),cyl(.004,.310,(0,.130,0)),
 cyl(.038,.012,(0,.280,0)),cyl(.005,.028,pin0)))
handle=parts.Slider('handle',axis=(0,1,0),travel=.010,mass=.055,damping=2000.,max_effort=4.,parent='wheel',
 shapes=(cyl(.012,.080,knob0,DARK),cyl(.004,.064,knob0+(0,.031,0))))
WHEEL=parts.Part('handwheel',pos=WPOS,shapes=(cyl(.015,.016,(0,0,0)),bx((.040,.016,.125),(0,0,-.0775)),bx((.080,.060,.006),(0,.010,-.137))),joints=(wheel,handle))
# The crank pin bears on the yoke; the carriage is never position-controlled.
HEAD_POS=(.380,CY,TOP)
head_shapes=[bx((.024,.050,.046),(-.058,0,C0-.016)),
 bx((.024,.008,.032),(-.058,-.021,C0+.023)),bx((.024,.008,.032),(-.058,.021,C0+.023)),
 bx((.022,.030,.026),(-.065,0,C0)),
 bx((.020,.030,.006),(-.080,0,C0-.0172))]
# Slot horizontal along x, 11.5 mm high; the crank pin is 10 mm diameter.
pz=.140+R*math.cos(A0)-.00075
for z in (pz-.00975,pz+.00975):head_shapes.append(bx((.084,.020,.008),(0,-.010,z),METAL))
head_shapes += [bx((.010,.020,.0115),(x,-.010,pz),METAL) for x in (-.047,.047)]
# The fixed spring bridge rides on the head at the outboard operating end (y=-145mm),
# clear of the specimen and knife. Its frame is collidable; captive coils use the hinge drive.
arm_top=C0+.0262+.004
head_shapes += [bx((.040,.080,.012),(-.045,.055,C0-.016),METAL),
 bx((.012,.012,.086),(-.041,.039,C0+.021),METAL),
 bx((.012,.277,.008),(-.041,-.0915,C0+.068),METAL),
 bx((.0445,.012,.008),(-.06325,-.177,C0+.068),METAL),
 bx((.012,.012,.036),(-.0855,-.177,arm_top+.021),METAL),
 bx((.032,.016,.006),(-.0855,-.179,arm_top+.003),METAL),
 bx((.033,.034,.004),(-.0855,-.200,arm_top+.04696),METAL)]
head_shapes += [bx((.020,.138,.012),(-.060,0,C0-.048),METAL),bx((.020,.050,.014),(-.060,0,C0-.037),METAL)]
head_shapes += [bx((.038,.018,.012),(-.040,y,C0-.048),METAL) for y in (-.060,.060)]
head_shapes += [bx((.033,.004,.046),(-.0855,-.200+y,arm_top+.023),METAL) for y in (-.015,.015)]
# Two-end shaft support travels with the head; all other small mounting details are declared separately.
head_shapes += [bx((.025,.024,.020),(-.041,-.221,C0+.068),METAL),
 bx((.016,.012,.034),(-.0455,-.221,C0+.046),METAL),
 cyl(.008,.008,(-.0455,-.221,C0+.0262),axis='y')]
head=parts.Slider('head',axis=(0,0,1),travel=.01475,lower_limit=-.04425,mass=.35,damping=1.,shapes=tuple(head_shapes))
# Lip lever: spring and lip act at 40 mm radius. Operating foot is 101 mm below pivot.
PIVOT=(-.0455,0,C0+.0262)
clamp_shapes=(bx((.048,.030,.006),(-.0645,0,C0+.0172)),
 bx((.003,.030,.006),(-.0855,0,C0+.015)),
 bx((.010,.211,.009),(-.040,-.100,C0+.0262)),
 bx((.015,.022,.014),(-.0455,0,C0+.024)),
 bx((.049,.024,.008),(-.0665,-.200,C0+.0262)),
 bx((.014,.024,.101),(-.0855,-.200,C0-.0243)))
# Equivalent linear rotational spring: 4 Lee LC022D03S, two series pairs in parallel.
# Exact geometry and force are documented and checked in fixture/force_check.py.
clamp=parts.Hinge('clamp',pivot=PIVOT,axis=(0,1,0),parent='head',lower=-.01,upper=math.radians(15.5),
 mass=.18,spring=1.4,target=-(.06/1.4),damping=.025,max_effort=2.,shapes=clamp_shapes)
# Fixed upper guide clamps use a low neck so the validated A wrist path remains clear.
upper_guide_shapes=[]
for y in (-.060,.060):
 upper_guide_shapes += [bx((.014,.014,.010),(-.029,y,.2825),METAL),
  bx((.008,.008,.014),(-.0215,y,.272),METAL),
  bx((.043,.008,.010),(-.0025,y,.270),METAL),
  cyl(.005,.010,(0,y,.270),axis='z'),cyl(.005,.010,(.014,y,.270),axis='z')]
HEAD=parts.Part('specimen_guide',pos=HEAD_POS,shapes=(
 cyl(.00397,.250,(-.029,-.060,.1625),axis='z'),cyl(.00397,.250,(-.029,.060,.1625),axis='z'),
 bx((.022,.138,.008),(-.015,0,.249),METAL),bx((.022,.138,.008),(-.015,0,.0415),METAL))+tuple(upper_guide_shapes),joints=(head,clamp))
KNIFE=parts.Part('knife_holder_e',pos=(.260,CY,TOP),shapes=(bx((.100,.110,.030),(0,0,.015)),
 bx((.040,.100,.070),(0,0,.065)),bx((.014,.100,.016),(.027,0,.095),METAL),bx((.0005,.080,.008),(.033,0,.101),(.85,.88,.92))))
# 46 mm forked guard encloses the edge without intersecting it at the cover stop.
guard_shapes=(bx((.046,.100,.004),(-.023,0,0),RED),bx((.004,.100,.010),(-.044,0,-.006),RED),bx((.024,.094,.008),(-.034,.090,.002),RED),cyl(.012,.030,(-.034,.110,.016),RED))
guard=parts.Hinge('guard',pivot=(0,0,0),axis=(0,1,0),lower=0,upper=math.radians(190),mass=.055,
 damping=.002,shapes=guard_shapes)
GUARD=parts.Part('knife_guard',pos=GPOS,shapes=(bx((.012,.008,.015),(0,-.058,-.004),METAL),
 bx((.012,.008,.015),(0,.058,-.004),METAL)),joints=(guard,))
CASS=parts.Part('cassette',pos=(CX,CY,TOP+C0),mass=.012,friction=(.6,.5),shapes=(
 bx((.0065,.0404,.0284),(0,0,0),(.94,.94,.90)),bx((.006,.030,.022),(-.00625,0,0),(.94,.85,.58))))
TRAY=parts.Part('grey_tray',pos=(.215,.215,TOP),mass=.15,friction=(.8,.6),shapes=(
 bx((.148,.108,.003),(0,0,.0015),(.45,.47,.50)),
 bx((.004,.108,.022),(-.072,0,.011),(.45,.47,.50)),bx((.004,.108,.022),(.072,0,.011),(.45,.47,.50)),
 bx((.140,.004,.022),(0,-.052,.011),(.45,.47,.50)),bx((.140,.004,.022),(0,.052,.011),(.45,.47,.50))))
OBJECTS=[HOUSING,LOCK_WALL,WHEEL,HEAD,KNIFE,GUARD,CASS,TRAY]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={'grey_tray':TRAY.pos}),
 lots.Lot('B',place={'grey_tray':(.200,.320,TOP)},yaw={'grey_tray':math.radians(90)}))
DEMO_TOUCHES=set()

def fresh():return dict(known=False,invalid=False,why='',t=0.,held=0.,prev=None,certificate=None,view=None,peaks={},forces={},clear=False)
S=fresh()
def invalidate(why):S.update(invalid=True,why=S['why'] or str(why))
def signature(o):
 result=[]
 for name in ('handwheel','specimen_guide','knife_guard','cassette','grey_tray'):
  s=o[name]
  joint_values=[s.joints[k] for k in sorted(s.joints)]
  link_values=[v for k in sorted(s.links) for a in s.links[k] for v in a]
  result.append(tuple(np.r_[s.pos,s.quat_xyzw,s.lin_vel,s.ang_vel,s.low,s.high,joint_values,link_values]))
 return tuple(result)

def valid(o):
 try:
  for part in OBJECTS:
   s=o[part.name];joint_names={j.name for j in part.joints}
   if set(s.joints)!=joint_names or set(s.links)!=joint_names:return False
   v=np.r_[s.pos,s.quat_xyzw,s.lin_vel,s.ang_vel,s.low,s.high,list(s.joints.values())]
   if not np.isfinite(v).all() or np.linalg.norm(s.quat_xyzw)<.1:return False
   for p,q in s.links.values():
    if not np.isfinite(np.r_[p,q]).all() or np.linalg.norm(q)<.1:return False
  return True
 except (KeyError,AttributeError,ValueError,TypeError):return False

def states(o):
 w=o['handwheel'].joints;h=o['specimen_guide'].joints
 locked=abs(A0+w['wheel'])<=math.radians(2.5) and w['handle']>=.006
 cover=o['knife_guard'].joints['guard']>=math.radians(175)
 c=o['cassette'];start=np.array((CX,CY,TOP+C0+h['head']))
 out=np.linalg.norm(c.pos-start)>=.015
 return locked,cover,out

def in_tray(o):
 c,t=o['cassette'],o['grey_tray'];r=parts._quat_to_matrix(t.quat_xyzw)
 if r[2,2]<.98:return False
 boxes=CASS.world_boxes({'base':(c.pos,parts._quat_to_matrix(c.quat_xyzw))})
 corners=[]
 for _,p,rot,sz in boxes:
  corners.extend((p+np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])*(sz/2)@rot.T-t.pos)@r)
 q=np.array(corners)
 return bool(np.max(abs(q[:,0]))<=.070+1e-6 and np.max(abs(q[:,1]))<=.050+1e-6 and .002<=q[:,2].min()<=.0045 and q[:,2].max()<=.045)

@lru_cache(maxsize=1)
def gripper_meshes():
 from yam_arena import analysis
 out={}
 for name,hulls in analysis.link_hulls().items():
  if name not in ('gripper','tip_left','tip_right'):continue
  pts=[]
  for hull in hulls:
   # Sample complete convex hull faces at <=3 mm spacing, including broad flat faces.
   from scipy.spatial import ConvexHull
   for tri in hull.vertices[ConvexHull(hull.vertices).simplices]:
    n=max(1,math.ceil(max(np.linalg.norm(tri[i]-tri[j]) for i,j in ((0,1),(1,2),(2,0)))/.003))
    pts.extend(tri[0]+i/n*(tri[1]-tri[0])+j/n*(tri[2]-tri[0]) for i in range(n+1) for j in range(n-i+1))
  v=np.array(pts);_,idx=np.unique(np.round(v/.0015).astype(int),axis=0,return_index=True);out[name]=v[idx]
 return out

def hands(sim):
 base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);clouds=[]
 for arm in ('left_robot','right_robot'):
  a=base.scene[arm];poses=parts._np(a.data.body_link_pose_w)[0]
  for name,points in gripper_meshes().items():
   pose=poses[a.body_names.index(name)];clouds.append(points@parts._quat_to_matrix(pose[3:]).T+pose[:3]-origin)
 if len(clouds)!=6 or any(not np.isfinite(p).all() for p in clouds):raise ValueError('unknown hand geometry')
 return clouds

def gap_points(clouds,box):
 _,c,r,sz=box
 return min(float(np.linalg.norm(np.maximum(abs((p-c)@r)-sz/2,0),axis=1).min()) for p in clouds)

def released(o,clouds):
 for part in (WHEEL,HEAD,GUARD,CASS,TRAY):
  s=o[part.name];poses={'base':(s.pos,parts._quat_to_matrix(s.quat_xyzw))}
  poses.update({n:(p,parts._quat_to_matrix(q)) for n,(p,q) in getattr(s,'links',{}).items()})
  if any(gap_points(clouds,b)<=.005 for b in part.world_boxes(poses)):return False
 return True

def observe(o,clouds,dt):
 S['known']=valid(o) and bool(clouds) and 0<dt<=.100001
 if not S['known']:invalidate('missing or invalid observation');return
 lk,gd,out=states(o);h=o['specimen_guide'].joints['head']
 # Knife region includes the full cassette and every sampled gripper face.
 if not(lk and gd):
  for p in clouds:
   mask=(p[:,0]>=.280)&(p[:,0]<=.328)&(abs(p[:,1]-CY)<=.041)&(p[:,2]>=TOP+.100)&(p[:,2]<=TOP+C0+h+.030)
   if mask.any():invalidate('hand at specimen before both safeguards')
  if out:invalidate('cassette removed before both safeguards')
 if o['cassette'].low[2]<TOP-.020:invalidate('cassette fell off bench')
 S['clear']=released(o,clouds)
 sig=signature(o);still=all(o[n].resting(.003,.03) for n in ('cassette','grey_tray'))
 if S['prev'] is not None:
  for a,b in zip(sig,S['prev']):
   if np.max(abs(np.array(a[:3])-b[:3]))>.001:still=False
  # Joint movement <1 mm at the longest feature; rotational pose change also counts.
  for n,lever in (('handwheel',.10),('specimen_guide',.15),('knife_guard',.046)):
   old=S.get('jprev',{}).get(n,{})
   if any(abs(v-old.get(k,v))*(1 if k in ('head','handle') else lever)>.001 for k,v in o[n].joints.items()):still=False
 eligible=lk and gd and out and in_tray(o) and S['clear'] and still and not S['invalid']
 S['held']=S['held']+dt if eligible and S.get('eligible',False) else 0.
 points=[]
 for part in (WHEEL,HEAD,GUARD,CASS,TRAY):
  st=o[part.name];poses={'base':(st.pos,parts._quat_to_matrix(st.quat_xyzw))}
  poses.update({n:(p,parts._quat_to_matrix(q)) for n,(p,q) in getattr(st,'links',{}).items()})
  for _,p,r,sz in part.world_boxes(poses):
   points.extend(p+np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])*(sz/2)@r.T)
 pts=np.array(points)
 hist=S.get('still_history',[])
 if not eligible:hist=[]
 if eligible:
  if any(np.max(np.linalg.norm(pts-old,axis=1))>.001+1e-9 for old in hist):
   hist=[];S['held']=0.
  hist.append(pts)
  hist=hist[-21:]
 S['eligible']=bool(eligible);S['still_history']=hist
 S['prev']=sig;S['jprev']={n:dict(o[n].joints) for n in ('handwheel','specimen_guide','knife_guard')};S['certificate']=sig

def contact_view(sim):
 from isaaclab_physx.physics import PhysxManager
 from isaaclab.sim.utils import get_current_stage
 from pxr import Usd,UsdPhysics
 stage=get_current_stage();paths={}
 for p in Usd.PrimRange(stage.GetPseudoRoot(),Usd.TraverseInstanceProxies()):
  if p.HasAPI(UsdPhysics.RigidBodyAPI):paths[str(p.GetPath())]=p
 # Explicit contact reporting on all mechanism links and the two grippers.
 rows=[];filters=[];labels=[]
 for suffix,label in (('/handwheel/wheel','wheel'),('/handwheel/handle','handle'),('/specimen_guide/clamp','clamp'),('/knife_guard/guard','guard')):
  found=[p for p in paths if p.endswith(suffix)]
  if len(found)!=1:raise ValueError('contact body absent '+suffix)
  rows+=found;labels.append(label);filters.append([p for p in paths if '/LeftArm/' in p or '/RightArm/' in p])
 if not all(filters):raise ValueError('arm contact filters absent')
 S['contact_labels']=labels;S['contact_filters']=filters
 return PhysxManager.get_physics_sim_view().create_rigid_contact_view(rows,filter_patterns=filters,max_contact_data_count=1024)

@parts.on_reset
def reset(sim):
 S.clear();S.update(fresh())
 sim.set_target('specimen_guide','clamp',clamp.target)
 # Enable PhysX contact reporting; runtime reads fail closed if it is unavailable.
 from isaaclab.app.settings_manager import get_settings_manager
 get_settings_manager().set_bool('/physics/disableContactProcessing',False)

@parts.on_step
def step(sim):
 o=sim.states();dt=float(sim.t)-S['t'];S['t']=float(sim.t)
 try:
  clouds=hands(sim)
  if S['view'] is None:S['view']=contact_view(sim)
  def array(v):
   v=v.torch if hasattr(v,'torch') else v
   return np.asarray(v.numpy() if hasattr(v,'numpy') else v)
  f=array(S['view'].get_contact_force_matrix(dt=sim.env.unwrapped.physics_dt)).reshape(4,-1,3).copy()
  nf,npnt,normals,_,nc,ns=map(array,S['view'].get_contact_data(dt=sim.env.unwrapped.physics_dt))
  tf,tpnt,tc,ts=map(array,S['view'].get_friction_data(dt=sim.env.unwrapped.physics_dt))
  nf=nf.reshape(-1);npnt=npnt.reshape(-1,3);normals=normals.reshape(-1,3);tf=tf.reshape(-1,3);tpnt=tpnt.reshape(-1,3)
  nc=nc.reshape(f.shape[:2]);ns=ns.reshape(f.shape[:2]);tc=tc.reshape(f.shape[:2]);ts=ts.reshape(f.shape[:2])
  if nc.sum()>=1024 or tc.sum()>=1024:raise ValueError('contact buffer exhausted')
  origin=parts._np(sim.env.unwrapped.scene.env_origins[0]);S['finger_forces']={};S['net_forces']={};S['contact_moments']={}
  for k,label in enumerate(S['contact_labels']):
   pivot=np.array(WPOS) if label in ('wheel','handle') else np.array(GPOS) if label=='guard' else o['specimen_guide'].links['clamp'][0]
   moment=np.zeros(3)
   for j,path in enumerate(S['contact_filters'][k]):
    na,nn=int(ns[k,j]),int(nc[k,j]);fa,fn=int(ts[k,j]),int(tc[k,j])
    if min(nn,fn)<0 or (nn and (na<0 or na+nn>len(nf))) or (fn and (fa<0 or fa+fn>len(tf))):raise ValueError('invalid force buffer')
    if not np.isfinite(np.r_[nf[na:na+nn],npnt[na:na+nn].ravel(),normals[na:na+nn].ravel(),tf[fa:fa+fn].ravel(),tpnt[fa:fa+fn].ravel()]).all():raise ValueError('nonfinite contact data')
    normal=nf[na:na+nn,None]*normals[na:na+nn]
    tangential=tf[fa:fa+fn]
    if fn:f[k,j]+=tangential.sum(axis=0)
    moment+=np.cross(npnt[na:na+nn]-origin-pivot,normal).sum(axis=0)
    moment+=np.cross(tpnt[fa:fa+fn]-origin-pivot,tangential).sum(axis=0)
    if path.endswith(('/tip_left','/tip_right')):S['finger_forces'][path]=S['finger_forces'].get(path,0.)+float(np.linalg.norm(f[k,j]))
   S['net_forces'][label]=f[k].sum(axis=0).tolist();S['contact_moments'][label]=moment.tolist()
   S['forces'][label]=float(np.linalg.norm(f[k],axis=1).sum());S['peaks'][label]=max(S['peaks'].get(label,0),S['forces'][label])
  if not np.isfinite(f).all():raise ValueError('nonfinite force')
  observe(o,clouds,dt)
 except Exception as e:
  S['known']=False;invalidate(type(e).__name__+': '+str(e))
 proof=os.environ.get('TRACE_OUT') or os.environ.get('TASK_PROOF_OUT')
 if proof:
  row=dict(source_sha256=SOURCE_SHA256,lot=lots.current().name,t=sim.t,score=score(o),known=S['known'],invalid=S['invalid'],why=S['why'],held=S['held'],clear=S['clear'],
   forces=S['forces'],peaks=S['peaks'],finger_forces=S.get('finger_forces',{}),net_forces=S.get('net_forces',{}),contact_moments=S.get('contact_moments',{}),joints={n:o[n].joints for n in ('handwheel','specimen_guide','knife_guard')},
   cassette=o['cassette'].pos.tolist(),tray=o['grey_tray'].pos.tolist())
  from yam_arena.constants import TCP_OFFSET
  base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);row['arms']={}
  for arm in ('left_robot','right_robot'):
   actor=base.scene[arm];pose=parts._np(actor.data.body_link_pose_w)[0,actor.body_names.index('gripper')]
   row['arms'][arm]=dict(tcp=(pose[:3]-origin+parts._quat_to_matrix(pose[3:])@np.array(TCP_OFFSET)).tolist(),q=parts._np(actor.data.joint_pos)[0].tolist())
  with pathlib.Path(proof).with_name('mechanisms.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')

def score(o):
 try:
  if not valid(o) or not S['known'] or S['invalid'] or S['certificate']!=signature(o) or not S['clear']:return 0.
  lk,gd,out=states(o)
  if not lk:return 0.
  if not gd:return .25
  if not out:return .5
  if in_tray(o) and S['held']>=1.-1e-8:return 1.
  return .75
 except (KeyError,AttributeError,ValueError,TypeError):return 0.
def success(o):return score(o)==1.

def demo():
 # Camera-visible geometry and the clock define the plan; only arm encoders close the motor loop.
 def wp(t,p,g,label,rot=None,yaw=0.):
  return Waypoint(t,tuple(float(x) for x in p),g,label,yaw if rot is None else tilt._TiltedYaw(yaw,rot))
 left=[];right=[]
 def w(arm,t,p,g,label,rot=None,yaw=0.):arm.append(wp(t,p,g,label,rot,yaw))
 rr0=tilt.rotation(math.pi/2,90.,0.);rr=rr0
 k=np.array(WPOS)+knob0+(0,-.010,0)
 w(right,3.,k+(-.055,-.015,.025),FINGER_OPEN,'approach handwheel',rr)
 w(right,2.,k,FINGER_OPEN,'align knob',rr)
 w(right,1.5,k,-.0088,'hold handwheel knob',rr)
 for a in np.linspace(A0,math.radians(2),13)[1:]:
  p=np.array(WPOS)+(KNOB_R*math.sin(a),-.085,KNOB_R*math.cos(a))
  rr=parts._rot((0,1,0),a-A0)@rr0
  w(right,.6,p,-.0088,'turn to upper detent',rr)
 p=np.array(WPOS)+(KNOB_R*math.sin(math.radians(2)),-.085,KNOB_R*math.cos(math.radians(2)))
 w(right,2.,p,-.0088,'settle at upper detent',rr)
 w(right,4.,p+(0,.016,0),-.0088,'push lock pin',rr)
 for a in map(math.radians,(0.,3.,1.,2.)):
  p=np.array(WPOS)+(KNOB_R*math.sin(a),-.085,KNOB_R*math.cos(a))
  rr=parts._rot((0,1,0),a-A0)@rr0
  w(right,.5,p+(0,.016,0),-.0088,'settle pin through detent',rr)
 w(right,2.5,p+(.060,.016,.060),FINGER_OPEN,'release and clear knob radially',rr)
 w(right,1.5,p+(.060,-.10,.060),FINGER_OPEN,'clear outside knob before axial retreat',rr)
 w(right,1.5,(.20,-.37,TOP+.10),FINGER_OPEN,'clear outside wheel rim')
 w(right,10.3,(.24,-.18,TOP+.16),FINGER_OPEN,'wait outside for guard')
 # Grip the24 mm fixture operating knob and carry it through110 degrees, then gravity completes coverage.
 safe=(.18,.24,TOP+.18)
 # Both lots use the validated end-face guard grasp and staged release.
 guard_A=True
 gr=grip_rotation(0. if guard_A else -math.pi/2)
 guard_point=np.array((-.034,.110,.034 if guard_A else .016))
 guard_grip=-.0135 if guard_A else -.0105
 g=np.array(GPOS)+guard_point
 w(left,22.,safe,FINGER_OPEN,'wait outside specimen region')
 w(left,2.,g+(0,.080,.020),FINGER_OPEN,'approach guard knob',gr)
 w(left,2.,g,FINGER_OPEN,'straddle guard knob',gr)
 w(left,1.5,g,guard_grip,'hold guard knob',gr)
 for a in np.linspace(0,math.radians(110),12)[1:]:
  ry=parts._rot((0,1,0),a);point=np.array(GPOS)+ry@guard_point
  w(left,.6,point,guard_grip,'fold guard past balance point',gr)
 if guard_A:
  escape=point+(.025,.010,.025)
  w(left,1.,escape,-.030,'release guard outward and above',gr)
  w(left,.8,escape+(0,.080,0),-.030,'clear guard outboard before full opening',gr)
 else:
  w(left,.8,point,FINGER_OPEN,'release guard toward covered stop',gr)
 w(left,2.,(.23,.21,TOP+.18),FINGER_OPEN,'clear covered edge')
 # Open the same sprung lever by grasping its24 mm-wide foot and pulling along its actual arc.
 # The reaction goes into the physical rocker, spring, carriage and crank lock.
 cr=tilt.rotation(0.,60.,0.);foot=np.array((.2945,CY-.200,TOP+.128))
 pivot=np.array((.3345,CY-.200,TOP+.2162));ry=parts._rot((0,1,0),math.radians(15))
 opened=pivot+ry@(foot-pivot)
 w(right,2.,foot+(-.030,0,0),FINGER_OPEN,'approach clamp foot',cr)
 w(right,2.,foot,FINGER_OPEN,'straddle clamp foot',cr)
 w(right,1.5,foot,-.0105,'hold clamp foot',cr)
 w(right,3.,opened,-.0105,'open sprung clamp',ry@cr)
 w(right,12.,opened,-.0105,'hold clamp open',ry@cr)
 w(right,1.5,opened,FINGER_OPEN,'release spring lever',ry@cr)
 w(right,2.,(.18,-.23,TOP+.16),FINGER_OPEN,'withdraw from machine')
 lr=tilt.rotation(0.,60.,0.);pick=np.array((CX,CY,TOP+.195))
 w(left,4.,(.245,CY,TOP+.195),FINGER_OPEN,'approach cassette',lr)
 w(left,2.,pick,FINGER_OPEN,'straddle cassette ends',lr)
 w(left,1.5,pick,-.0185,'grasp cassette',lr)
 w(left,3.,pick+(-.060,0,0),-.0185,'withdraw cassette while clamp held',lr)
 target=np.array(lots.current().pos('grey_tray'))
 w(left,3.,(target[0],target[1],TOP+.155),-.0185,'carry to visible tray')
 w(left,3.,(target[0],target[1],TOP+.026),-.0185,'lower into tray')
 w(left,1.5,(target[0],target[1],TOP+.026),-.030,'release cassette')
 w(left,2.,(target[0],target[1],TOP+.15),-.030,'hands clear')
 w(left,6.,(target[0],target[1],TOP+.15),FINGER_OPEN,'one second still hold')
 return {'left':left,'right':right}

# Optional demonstration controller: only the same observed joint angles supplied to any policy.
# Object waypoints and their order remain time-based; no scene, contact or scoring state steers actions.
def _install_encoder_controller(module):
 cls=module.YamScriptedPolicy
 if getattr(cls,'_microtome_encoder_controller',False):return
 old_reset,old_act=cls.reset,cls.act
 def reset_policy(self,scene):
  old_reset(self,scene)
  self._encoder_bias=np.zeros(12);self._previous_nominal=None
 def act_policy(self,observation):
  from inspect_robots.types import Action,ActionChunk
  from inspect_robots_yam.packing import STATE_KEY
  from yam_arena import ik
  chunk=old_act(self,observation)
  raw=np.asarray(chunk.actions[0].data,dtype=float).copy()
  observed=np.asarray(observation.state[STATE_KEY],dtype=float)
  idx=np.array([0,1,2,3,4,5,7,8,9,10,11,12])
  nominal=raw[idx].copy()
  if self._previous_nominal is not None:
   error=self._previous_nominal-observed[idx]
   self._encoder_bias=np.clip(self._encoder_bias+.12*error,-.30,.30)
  raw[idx]=np.clip(nominal+self._encoder_bias,np.tile(ik.LOWER,2),np.tile(ik.UPPER,2))
  self._previous_nominal=nominal
  return ActionChunk(actions=[Action(data=raw,meta=chunk.actions[0].meta)],control_hz=chunk.control_hz)
 cls.reset=reset_policy;cls.act=act_policy;cls._microtome_encoder_controller=True
parts._PATCHES['yam_arena.scripted']=_install_encoder_controller
if 'yam_arena.scripted' in sys.modules:_install_encoder_controller(sys.modules['yam_arena.scripted'])
