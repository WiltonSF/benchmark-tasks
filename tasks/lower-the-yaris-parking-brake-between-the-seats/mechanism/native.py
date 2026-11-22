"""Per-physics-step observations from PhysX; no pose writes or robot control."""
import json, math, os, pathlib
from itertools import product
import numpy as np
from . import Sample

def arr(x):
 x=x.torch if hasattr(x,'torch') else x
 return np.asarray(x.detach().cpu().numpy() if hasattr(x,'detach') else x.numpy(),float)

class Monitor:
 def __init__(self,g,sim):
  self.g=g;self.env=sim.env;self.base=sim.env.unwrapped;self.judge=g['_judge'];self.active=False
  self.time=0.;self.view=None;self.certificate=None;self.metrics={};self.history=0
  self.asset=self.base.scene[g['BRAKE'].name]
  self.qids={n:self.asset.joint_names.index(n) for n in ('lever','button')}
  self.bids={n:self.asset.body_names.index(n) for n in ('base','lever','button')}
  self.robots=[self.base.scene[a+'_robot'] for a in ('left','right')]
  self.origin=arr(self.base.scene.env_origins[0]);self.rows=[];self.sensor_keys=[]
  from isaaclab_physx.physics import PhysxManager
  manager=PhysxManager
  self.manager=manager
  original=getattr(manager,'_handbrake_original_step',manager.step)
  manager._handbrake_original_step=original
  def step(*args,**kwargs):
   result=original(*args,**kwargs)
   if self.active:self.tick(float(self.base.sim.get_physics_dt()))
   return result
  manager.step=staticmethod(step)
 def disable(self):self.active=False
 def start(self):
  self.time=0.;self.view=None;self.certificate=None;self.metrics={};self.history=0;self.cable_theta=self.g['ANGLE'];self.judge.reset();self.active=True
 def contact(self,pivot,axis):
  if self.view is None:
   from isaaclab_physx.physics import PhysxManager
   from yam_arena.robot import asset_link_paths
   linkpaths=asset_link_paths();prefix=self.base.scene.env_prim_paths[0]
   paths=[]
   for h,arm in enumerate(('LeftArm','RightArm')):
    for name in ('link1','link2','link3','link4','link5','gripper','tip_left','tip_right'):
     paths.append(prefix+'/'+arm+linkpaths[name]);self.sensor_keys.append((h,name))
   self.view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths,
       filter_patterns=[[prefix+'/parking_brake/lever',prefix+'/parking_brake/button'] for _ in paths],max_contact_data_count=2048)
  normal,points,normals,_,count,start=self.view.get_contact_data(dt=self.base.sim.get_physics_dt())
  normal,points,normals,count,start=map(arr,(normal,points,normals,count,start))
  ff,fp,fc,fs=map(arr,self.view.get_friction_data(dt=self.base.sim.get_physics_dt()))
  points=points-self.origin;fp=fp-self.origin
  if count.sum()>=2048 or fc.sum()>=2048:raise ValueError('contact buffer exhausted')
  forces=np.zeros((2,2,3));moments=np.zeros((2,2));other=0.;details=[]
  for i,(h,name) in enumerate(self.sensor_keys):
   for target in (0,1):
    a,n=int(start[i,target]),int(count[i,target]);b,k=int(fs[i,target]),int(fc[i,target])
    nf=normal[a:a+n].reshape(-1,1)*normals[a:a+n]
    tf=ff[b:b+k]
    total=nf.sum(0)+tf.sum(0)
    moment=-float((np.cross(points[a:a+n]-pivot,nf).sum(0)+np.cross(fp[b:b+k]-pivot,tf).sum(0))@axis)
    if name in ('gripper','tip_left','tip_right'):
     forces[h,target]+=total;moments[h,target]+=moment
    else:other+=float(np.linalg.norm(nf,axis=1).sum()+np.linalg.norm(tf,axis=1).sum())
    if n or k:details.append([h,name,target,n,k,total.tolist(),moment])
  return forces,moments,other,details
 def poses(self):
  bp=arr(self.asset.root_view.get_link_transforms())[0]
  bp=bp.copy();bp[:,:3]-=self.origin
  qs=arr(self.asset.root_view.get_dof_positions())[0]
  rp=[]
  for robot in self.robots:
   p=arr(robot.root_view.get_link_transforms())[0].copy();p[:,:3]-=self.origin;rp.append(p)
  if any(not np.isfinite(p).all() or not np.allclose(np.linalg.norm(p[:,3:],axis=1),1.,atol=1e-4,rtol=0) for p in [bp,*rp]):raise ValueError('nonfinite or nonunit live pose')
  if not np.isfinite(qs).all():raise ValueError('nonfinite live joint state')
  return bp,qs,rp
 def integrity(self,bp,q):
  if not np.isfinite(bp).all() or not np.allclose(np.linalg.norm(bp[:,3:],axis=1),1.,atol=1e-4,rtol=0):return False
  part=self.g['BRAKE'];expected=part.state(joints={n:q[i] for n,i in self.qids.items()})
  poses={'base':(expected.pos,expected.quat_xyzw),**expected.links}
  rot=self.g['parts']._quat_to_matrix
  for name,(pos,quat) in poses.items():
   actual=bp[self.bids[name]]
   if not np.isfinite(rot(actual[3:])).all():return False
   if np.linalg.norm(actual[:3]-pos)>.001:return False
   if np.linalg.norm(rot(actual[3:])-rot(quat))>.015:return False
  return bool(np.isfinite(bp).all() and np.isfinite(q).all())
 def clear(self,bp,rp):
  g=self.g;rot=g['parts']._quat_to_matrix
  if any(not np.isfinite(p).all() or not np.allclose(np.linalg.norm(p[:,3:],axis=1),1.,atol=1e-4,rtol=0) for p in rp):raise ValueError('invalid robot pose')
  poses={n:(bp[i,:3],rot(bp[i,3:])) for n,i in self.bids.items()}
  boxes=[(c,r,size) for name,c,r,size in g['BRAKE'].world_boxes(poses) if name!='base']
  from yam_arena import analysis
  corners=np.array(list(product((-1,1),repeat=3)))
  nearest=math.inf;clouds=None
  for h,robot in enumerate(self.robots):
   for name in ('gripper','tip_left','tip_right'):
    p=rp[h][robot.body_names.index(name)];r=rot(p[3:]);v=np.concatenate([x.vertices for x in analysis.link_hulls()[name]])
    lo,hi=v.min(0),v.max(0);pts=(corners*(hi-lo)/2+(hi+lo)/2)@r.T+p[:3];low,high=pts.min(0),pts.max(0)
    near=[]
    for c,br,size in boxes:
     be=np.abs(br)@(size/2);gap=np.maximum(np.maximum(c-be-high,low-c-be),0.)
     lb=float(np.linalg.norm(gap))
     if lb<.035:near.append((c,br,size))
     else:nearest=min(nearest,lb)
    if near:
     if clouds is None:clouds=g['hand_mesh']()
     v=clouds[name]@r.T+p[:3]
     for c,br,size in near:
      d=float(np.linalg.norm(np.maximum(np.abs((v-c)@br)-size/2,0.),axis=1).min())
      nearest=min(nearest,d)
  return max(0.,nearest-.005)
 def tick(self,dt):
  self.time+=dt
  g=self.g;j=self.judge
  try:
   bp,q,rp=self.poses();part=g['BRAKE'];theta=g['ANGLE']+float(q[self.qids['lever']]);button=float(q[self.qids['button']])
   rot=g['parts']._quat_to_matrix;axis=rot(bp[self.bids['base'],3:])@np.asarray(g['LEVER'].axis)
   lever_axis=rot(bp[self.bids['lever'],3:])@g['U0']
   pivot=bp[self.bids['base'],:3]
   forces,moments,other,details=self.contact(pivot,axis)
   effort=float(arr(self.asset.root_view.get_dof_actuation_forces())[0,self.qids['lever']])
   clear=0.
   if j.pressed_once and theta<=20*math.pi/180 and not j.released:clear=self.clear(bp,rp)
   s=Sample(self.time,theta,button,forces,moments,forces[:,1]@lever_axis,clear,self.integrity(bp,q),effort,other,cable_reference_theta=self.cable_theta)
   j.step(s)
   lower=getattr(j,'lower',j.tooth)-g['ANGLE'];hi=getattr(j,'button_hi',.001)
   g['write_limits'](self.asset,lower,hi)
   g['set_cable_effort'](self.asset,theta)
   applied_reference=self.cable_theta;self.cable_theta=theta
   self.certificate=None if j.invalid else np.r_[bp[self.bids['base']],q[self.qids['lever']],q[self.qids['button']]]
   self.metrics=dict(t=self.time,theta_deg=math.degrees(theta),button_mm=button*1000,forces=forces.tolist(),moments=moments.tolist(),
      press=s.press.tolist(),cable_effort=effort,cable_reference_deg=math.degrees(applied_reference),other_link_force=other,clear=clear,supported=getattr(j,'supported',False),
      released=j.released,lifted=j.lifted_once,pressed=j.pressed_once,held=j.hold,invalid=j.invalid,reason=j.reason,score=j.score(),contacts=details,
      gripper_poses=[p[r.body_names.index('gripper')].tolist() for p,r in zip(rp,self.robots)],robot_q=[arr(r.root_view.get_dof_positions())[0].tolist() for r in self.robots],
      peak_moment=j.peak_moment.tolist(),peak_support=j.peak_support.tolist(),peak_press=j.peak_press.tolist(),lift_work=j.work.tolist(),tooth_deg=math.degrees(j.tooth))
  except Exception as exc:
   self.certificate=None;j.fail('native observation: '+type(exc).__name__+': '+str(exc));self.metrics=dict(t=self.time,invalid=True,reason=j.reason,score=0.)
   # Missing observations may never leave a released ratchet in force.
   g['write_limits'](self.asset,j.tooth-g['ANGLE'],.001)
 def log(self):
  if os.environ.get('TRACE_OUT'):
   with pathlib.Path(os.environ['TRACE_OUT']).with_name('handbrake-v3.jsonl').open('a') as f:f.write(json.dumps(self.metrics)+'\n')
