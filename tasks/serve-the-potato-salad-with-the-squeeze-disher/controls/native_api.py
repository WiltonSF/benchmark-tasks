"""Independent reset/callback contracts. Native backend is explicitly mocked.
These controls do not demonstrate robot motion, a native API call or food calibration.
"""
import pathlib,sys,types,numpy as np,hashlib,itertools
from types import SimpleNamespace as NS
T=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(T));import task as m
h=m.native_history;rt=m.material_runtime
class Raw:
 def __init__(self,a,device='cpu'):self.a=np.asarray(a);self.device=device
 def numpy(self):return self.a
wp=types.ModuleType('warp');wp.float32=np.float32;wp.uint32=np.uint32;wp.array=lambda a,dtype,device:Raw(np.array(a,dtype=dtype),device);sys.modules['warp']=wp
for name in ('isaaclab','isaaclab.app','isaaclab_physx','omni'):
 mod=types.ModuleType(name);mod.__path__=[];sys.modules[name]=mod
settings=types.ModuleType('isaaclab.app.settings_manager');settings.get_settings_manager=lambda:NS(set_bool=lambda *a:None);sys.modules[settings.__name__]=settings
subscriptions=[];physx=types.ModuleType('omni.physx');physx.get_physx_interface=lambda:NS(subscribe_physics_on_step_events=lambda fn,**kw:subscriptions.append((fn,kw)) or object());sys.modules['omni.physx']=physx;sys.modules['omni'].physx=physx
robotmodule=types.ModuleType('yam_arena.robot');robotmodule.asset_link_paths=lambda:{'tip_left':'/base_link/gripper/tip_left','tip_right':'/base_link/gripper/tip_right'};sys.modules['yam_arena.robot']=robotmodule
projected=np.zeros((1,8));contacts=np.zeros((4,3));creation=[];force_failure=False
class Contact:
 def get_net_contact_forces(self,dt):
  if force_failure:raise RuntimeError('native force read unavailable')
  assert abs(dt-1/240)<1e-9
  return Raw(contacts)
class PhysicsView:
 def create_rigid_contact_view(self,paths,**kw):creation.append((paths,kw));return Contact()
physmodule=types.ModuleType('isaaclab_physx.physics');physmodule.PhysxManager=NS(get_physics_sim_view=lambda:PhysicsView());sys.modules[physmodule.__name__]=physmodule
class View:
 def __init__(self,spec):
  state=spec.state();self.poses=np.array([np.r_[state.pos,state.quat_xyzw]]+[np.r_[p,q] for p,q in state.links.values()]);self.friction=np.zeros((1,len(spec.joints),3),np.float32);self.joints=np.zeros((1,len(spec.joints)))
 def get_coms(self):return Raw(np.tile((0,0,0,0,0,0,1),(len(self.poses),1)))
 def get_dof_friction_properties(self):return Raw(self.friction)
 def set_dof_friction_properties(self,data,ids):self.friction=data.numpy().copy()
 def get_transforms(self):return Raw(self.poses)
 def get_velocities(self):return Raw(np.zeros((len(self.poses),6)))
 def get_link_transforms(self):return Raw(self.poses[None])
 def get_link_velocities(self):return Raw(np.zeros((1,len(self.poses),6)))
 def get_dof_positions(self):return Raw(self.joints)
 def get_dof_velocities(self):return Raw(self.joints*0)
 def apply_forces_and_torques_at_position(self,*args):pass
class Drive:
 def __init__(self):self.caps=np.array([[27.,27.,27.,7.,7.,7.,50.,35.]],np.float32);self.ignore_write=False
 def get_dof_projected_joint_forces(self):return Raw(projected)
 def get_dof_max_forces(self):return Raw(self.caps)
 def set_dof_max_forces(self,data,ids):
  assert ids.numpy().tolist()==[0]
  if not self.ignore_write:self.caps=data.numpy().copy()
class Scene(dict):__getattr__=dict.__getitem__
scene=Scene(env_origins=np.zeros((1,3)),env_prim_paths=['/World/envs/env_0'])
for spec in m.OBJECTS:scene[spec.name]=NS(root_view=View(spec),body_names=['base']+[j.name for j in spec.joints],joint_names=[j.name for j in spec.joints],permanent_wrench_composer=NS(reset=lambda:None))
for name in ('left_robot','right_robot'):scene[name]=NS(root_view=Drive(),joint_names=[f'joint{i}' for i in range(1,9)])
base=NS(scene=scene,device='cuda:0',sim=NS(get_physics_dt=lambda:1/240));sim=NS(env=NS(unwrapped=base),set_target=lambda *a:None)
verts=np.array(list(itertools.product((-.003,.003),repeat=3)));hand_center=np.array((3.,3.,3.))
m.native_hand_hulls=lambda sim:[('test_hand',NS(vertices=verts+hand_center))]
def pair():rt._physics(1/240);h._physics(1/240)
def reset():
 global projected,contacts,force_failure,hand_center
 projected[:]=0;contacts[:]=0;force_failure=False;hand_center=np.array((3.,3.,3.))
 for spec in m.OBJECTS:scene[spec.name].root_view.poses[0,:3]=spec.pos
 for name in ('left_robot','right_robot'):scene[name].root_view.caps=np.array([[27.,27.,27.,7.,7.,7.,50.,35.]],np.float32)
 m.lots.use('A');m.reset(sim)
reset();assert len(subscriptions)==2 and subscriptions[0][1]['pre_step'] is False and subscriptions[1][1]['pre_step'] is True
for name in ('left_robot','right_robot'):
 assert scene[name].root_view.caps.tolist()==[[27.,27.,27.,7.,7.,7.,40.,35.]]
 assert m._safety.drive_caps[name]==[40.,35.]
for _ in range(5):
 pair();assert m._history.ticks==m._food.ticks and not m._history.invalid
assert len(creation)==1 and len(creation[0][0])==4
contacts[2,2]=40.02;pair();assert not m._history.invalid and m._safety.overload and m._safety.peak_contact==40.02 and m._safety.peak_event['index']==2
contacts[:]=0;pair();assert not m._history.invalid and m._history.ticks==m._food.ticks
old=m._history;reset();assert m._history is not old and not m._history.invalid and len(subscriptions)==2 and h._contact_view is None
pair();assert len(creation)==2 and m._history.ticks==m._food.ticks==1
projected[0,6]=40.02;pair();assert not m._history.invalid and m._safety.overload
reset();force_failure=True;pair();assert not m._history.invalid and m._history.ticks==m._food.ticks==1 and 'unavailable' in m._safety.error
reset();projected[0,6]=np.nan;pair();assert not m._history.invalid and m._safety.error
reset();scene['left_robot'].root_view.caps[0,6]=50.;pair();assert not m._history.invalid and 'cap changed' in m._safety.error
reset();scene['bowl'].root_view.poses[0,0]+=.031;pair();assert m._history.invalid
scene['bowl'].root_view.poses[0,0]-=.031;pair();assert m._history.invalid
# Contact for one physics step, then restored before the next control observation.
reset();objs={n:rt._state(n)[0] for n in ('disher','bowl','plate')};hand_center=m.food_boxes(objs)[0][1].copy();pair();assert m._history.invalid and m._history.hand_food_contact
hand_center=np.array((3.,3.,3.));pair();assert m._history.invalid and m._history.ticks==m._food.ticks
# A failed readback is an operational startup failure, not a fabricated passing cap.
reset();scene['left_robot'].root_view.caps[0,6]=50.;scene['left_robot'].root_view.ignore_write=True
try:h._configure_drive_caps()
except ValueError as e:assert 'readback' in str(e)
else:raise AssertionError('failed safety cap was accepted')
scene['left_robot'].root_view.ignore_write=False
for lot in m.LOTS:
 starts={n:np.array(lot.pos(n)) for n in ('bowl','plate')};v={n:(p.copy(),np.eye(3)) for n,p in starts.items()}
 hist=h.History(expected=starts);v['plate']=(starts['plate']+(.030,0,0),np.eye(3));hist.observe(1/240,v,True);assert not hist.invalid
 v['plate']=(starts['plate']+(.030002,0,0),np.eye(3));hist.observe(1/240,v,True);assert hist.invalid
 for deg,bad in ((10.,False),(10.001,True)):
  hist=h.History(expected=starts);v={n:(p.copy(),np.eye(3)) for n,p in starts.items()};v['bowl']=(starts['bowl'],m.parts._rot((1,1,0),np.deg2rad(deg)));hist.observe(1/240,v,True);assert hist.invalid==bad
print('PASS: actual reset/callback contracts under mocked backend, preserved other/lower drive caps, readback failure, separate overload/NaN/unavailable/cap-drift safety alarms, primary vessel and transient convex hand-food latching. No native success claimed.')
for f in ('task.py','native_history/__init__.py','material_runtime/__init__.py'):print(f,hashlib.sha256((T/f).read_bytes()).hexdigest())
