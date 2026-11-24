"""Per-physics-step observations from PhysX and the ratchet stops; no pose writes and no robot control."""
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
  self.time=0.;self.view=None;self.certificate=None;self.metrics={};self.history=0;self.sensor_keys=[]
  self.peak_contact=0.;self.window_peak=0.;self.peak_lift=0.;self.peak_press=0.;self.cable_theta=self.g['ANGLE'];self.judge.reset();self.active=True
 def contact(self,pivot,axis,lever_axis):
  self.grip_contact=False
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
  forces=np.zeros((2,2,3));moments=np.zeros((2,2));other=0.;details=[];self.patches=[]
  for i,(h,name) in enumerate(self.sensor_keys):
   for target in (0,1):
    a,n=int(start[i,target]),int(count[i,target]);b,k=int(fs[i,target]),int(fc[i,target])
    nf=normal[a:a+n].reshape(-1,1)*normals[a:a+n]
    tf=ff[b:b+k]
    total=nf.sum(0)+tf.sum(0)
    if not all(np.isfinite(x).all() for x in (nf,tf,points[a:a+n],fp[b:b+k])):raise ValueError('nonfinite contact data')
    if os.environ.get('HB_TRACE_FULL'):
     for pt,f in zip(points[a:a+n],nf):
      if np.linalg.norm(f)>1e-6:self.patches.append(dict(hand=h,link=name,target=target,kind='normal',point=pt.tolist(),force_on_hand=f.tolist()))
     for pt,f in zip(fp[b:b+k],tf):
      if np.linalg.norm(f)>1e-6:self.patches.append(dict(hand=h,link=name,target=target,kind='friction',point=pt.tolist(),force_on_hand=f.tolist()))
    moment=-float((np.cross(points[a:a+n]-pivot,nf).sum(0)+np.cross(fp[b:b+k]-pivot,tf).sum(0))@axis)
    if name in ('gripper','tip_left','tip_right'):
     forces[h,target]+=total;moments[h,target]+=moment
     if target==0:
      # Only physical contact on the original 170-272 mm grip counts as support, not the steel boss.
      for pts,fsum in ((points[a:a+n],nf),(fp[b:b+k],tf)):
       delta=pts-pivot;along=delta@lever_axis
       radial=np.linalg.norm(delta-along[:,None]*lever_axis,axis=1)
       if np.any((along>=.170)&(along<=.272)&(radial<=.020)&(np.linalg.norm(fsum,axis=1)>1e-6)):
        self.grip_contact=True
    else:other+=float(np.linalg.norm(nf,axis=1).sum()+np.linalg.norm(tf,axis=1).sum())
    if n or k:details.append([h,name,target,n,k,total.tolist(),moment])
  return forces,moments,other,details
 def drive_diagnostics(self):
  # Read-only audit channels. The primary scorer does not use any of these quantities.
  names=('get_dof_projected_joint_forces','get_dof_velocities','get_dof_position_targets',
         'get_dof_velocity_targets','get_dof_max_forces','get_dof_stiffnesses','get_dof_dampings')
  out={}
  for name in names:
   try:out[name]=[arr(getattr(r.root_view,name)())[0].tolist() for r in self.robots]
   except Exception as exc:out[name]={'unavailable':type(exc).__name__+': '+str(exc)}
  try:
   if getattr(self,'all_contact_view',None) is None:
    from yam_arena.robot import asset_link_paths
    prefix=self.base.scene.env_prim_paths[0];links=asset_link_paths()
    paths=[prefix+'/'+arm+links[name] for arm in ('LeftArm','RightArm')
           for name in ('link1','link2','link3','link4','link5','gripper','tip_left','tip_right')]
    self.all_contact_view=self.manager.get_physics_sim_view().create_rigid_contact_view(paths)
   out['net_all_contacts_on_robot_links']=arr(self.all_contact_view.get_net_contact_forces(dt=self.base.sim.get_physics_dt())).tolist()
  except Exception as exc:out['net_all_contacts_on_robot_links']={'unavailable':type(exc).__name__+': '+str(exc)}
  return out
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
   bp,q,rp=self.poses();theta=g['ANGLE']+float(q[self.qids['lever']]);button=float(q[self.qids['button']])
   rot=g['parts']._quat_to_matrix;axis=rot(bp[self.bids['base'],3:])@np.asarray(g['LEVER'].axis)
   lever_axis=rot(bp[self.bids['lever'],3:])@g['U0']
   pivot=bp[self.bids['base'],:3]
   # Contact forces are logged as evidence only; the judge never reads them.
   forces,moments,other,details=self.contact(pivot,axis,lever_axis)
   actuation=arr(self.asset.root_view.get_dof_actuation_forces())[0]
   effort=float(actuation[self.qids['lever']]);button_effort=float(actuation[self.qids['button']])
   clear=0.
   if j.released_once and not j.free and theta<=20*math.pi/180+1e-6:clear=self.clear(bp,rp)
   hand_on_grip=self.grip_contact
   j.step(Sample(self.time,theta,button,clear,self.integrity(bp,q),hand_on_grip))
   motor_caps_ok=all(g['motors'].limits_valid(arr(r.root_view.get_dof_max_forces())[0],r.joint_names) for r in self.robots)
   if not motor_caps_ok:j.fail('declared motor cap exceeded or unavailable')
   expected=g['cable_moment'](self.cable_theta)
   if not math.isfinite(effort) or abs(effort+expected)>.5:j.fail('declared cable load absent')
   if not math.isfinite(button_effort) or abs(button_effort+g['BUTTON_RETURN_N'])>.01:j.fail('declared button load absent')
   # Pawl on its tooth = lever lower stop; loaded pawl = button stop. Stops are never set past the present pose.
   limit_writes=g['write_limits'](self.asset,j.lower-g['ANGLE'],j.button_hi)
   g['set_cable_effort'](self.asset,theta);self.cable_theta=theta
   self.certificate=None if j.invalid else np.r_[bp[self.bids['base']],q[self.qids['lever']],q[self.qids['button']]]
   norms=np.linalg.norm(forces,axis=2);press=forces[:,1]@lever_axis
   peak=float(norms.max());self.peak_contact=max(self.peak_contact,peak);self.window_peak=max(self.window_peak,peak)
   self.peak_lift=max(self.peak_lift,float(moments[:,0].sum()));self.peak_press=max(self.peak_press,float(press.max()))
   self.metrics=dict(t=self.time,theta_deg=math.degrees(theta),button_mm=button*1000,tooth_deg=math.degrees(j.tooth),free=j.free,
      lever_contact_N=norms[:,0].tolist(),button_contact_N=norms[:,1].tolist(),lift_moment_Nm=moments[:,0].tolist(),
      press_N=press.tolist(),button_effort=button_effort,cable_effort=effort,cable_moment=g['cable_moment'](theta),other_link_force=other,clear=clear,
      hand_on_grip=hand_on_grip,unheld_descent_deg=math.degrees(j.unheld_descent),resting=j.resting,released=j.released_once,dropped=j.dropped,held=j.hold,invalid=j.invalid,reason=j.reason,score=j.score(),
      motor_caps_ok=motor_caps_ok,window_peak_contact_N=self.window_peak,peak_contact_N=self.peak_contact,peak_lift_moment_Nm=self.peak_lift,peak_press_N=self.peak_press,
      button_hi_mm=j.button_hi*1000,lower_deg=math.degrees(j.lower),contacts=details,
      gripper_poses=[p[r.body_names.index('gripper')].tolist() for p,r in zip(rp,self.robots)],
      robot_q=[arr(r.root_view.get_dof_positions())[0].tolist() for r in self.robots],
      robot_effort=[arr(r.root_view.get_dof_actuation_forces())[0].tolist() for r in self.robots])
   if os.environ.get('HB_TRACE_FULL'):self.metrics['drive_audit']=self.drive_diagnostics()
   if os.environ.get('TRACE_OUT') and os.environ.get('HB_TRACE_FULL'):
    with pathlib.Path(os.environ['TRACE_OUT']).with_name('handbrake-physics.jsonl').open('a') as f:
     f.write(json.dumps(dict(t=self.time,dt=dt,theta_deg=math.degrees(theta),button_mm=button*1000,lever_contact_N=norms[:,0].tolist(),button_contact_N=norms[:,1].tolist(),button_hi_mm=j.button_hi*1000,free=j.free,patches=self.patches,limit_writes=limit_writes,hand_on_grip=hand_on_grip,cable_effort=effort,button_effort=button_effort,invalid=j.invalid,reason=j.reason,joint_limits=arr(self.asset.root_view.get_dof_limits())[0].tolist(),joint_velocities=arr(self.asset.root_view.get_dof_velocities())[0].tolist(),robot_q=self.metrics['robot_q'],robot_effort=self.metrics['robot_effort'],drive_audit=self.metrics.get('drive_audit')))+'\n')
  except Exception as exc:
   # A failed observation invalidates the trial and leaves the stops as they were.
   self.certificate=None;j.fail('native observation: '+type(exc).__name__+': '+str(exc));self.metrics=dict(t=self.time,invalid=True,reason=j.reason,score=0.)
 def log(self):
  if os.environ.get('TRACE_OUT'):
   with pathlib.Path(os.environ['TRACE_OUT']).with_name('handbrake.jsonl').open('a') as f:f.write(json.dumps(self.metrics)+'\n')
  self.window_peak=0.
