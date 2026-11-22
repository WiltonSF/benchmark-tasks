import sys,pathlib,os,json,math
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent.parent))
import parts
from yam_arena.demo import Waypoint,FINGER_CLOSED
SCENE='station';PT=.002;G=.001;S={}
# Upper mass22g and100mm square inertia envelope; lower fixed; no gravity/contact for identification.
OBJECTS=[parts.Part('slotted_plate',pos=(.325,0,1.2),shapes=(parts.box((.1,.1,.002)),)),parts.Part('notched_plate',pos=(.325,0,1.203),mass=.022,shapes=(parts.box((.1,.1,.002)),))]
def build_joint():
    from pxr import UsdPhysics, Gf, Sdf
    from isaaclab.sim.utils import get_current_stage
    stage=get_current_stage();path=parts._PATHS['slotted_plate']+'/native_bonds'
    joint=UsdPhysics.Joint.Define(stage,path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parts._PATHS['slotted_plate'])])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(parts._PATHS['notched_plate'])])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0,0,(PT+G)/2))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0,0,-(PT+G)/2))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1));joint.CreateLocalRot1Attr().Set(Gf.Quatf(1))
    drives=[]
    for name in ('transX','transY','transZ','rotX','rotY','rotZ'):
        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),name);drive.CreateTypeAttr('force')
        drive.CreateTargetPositionAttr(0.);drive.CreateTargetVelocityAttr(0.)
        lin=name.startswith('trans');base_k=(1500. if name!='transZ' else 700.) if lin else .2/(180/math.pi)
        drive.CreateStiffnessAttr(base_k);drive.CreateDampingAttr(2. if lin else .01/(180/math.pi));drive.CreateMaxForceAttr(12. if lin else .15)
        drives.append((drive,base_k))
    joint.CreateJointEnabledAttr(True);S['joint']=joint;S['drives']=drives;S['joint_n']=15

_original_build=build_joint
def build_joint():
 _original_build()
 from pxr import Usd,UsdPhysics,PhysxSchema
 from isaaclab.sim.utils import get_current_stage
 stage=get_current_stage()
 for name in ['slotted_plate','notched_plate']:
  prim=stage.GetPrimAtPath(parts._PATHS[name]);PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
  for pr in Usd.PrimRange(prim):
   if pr.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(pr).CreateCollisionEnabledAttr(False)
CASES=[]
for kind,amps in [('position',(.001,.030)),('velocity',(.030,))]:
 for axis in range(3):
  for amp in amps:
   for sign in [-1,1]:CASES.append((kind,axis,amp*sign))
for kind,amps in [('angle',(.030,1.)),('omega',(.3,))]:
 for axis in range(3):
  for amp in amps:
   for sign in [-1,1]:CASES.append((kind,axis,amp*sign))
D={'i':0,'phase':0}
@parts.on_reset
def reset(sim):
 D.update(i=0,phase=0);build_joint()
@parts.on_physics_step
def test(sim):
 import torch
 if D['i']>=len(CASES):return
 asset=sim.env.unwrapped.scene['notched_plate'];dt=float(sim.env.unwrapped.physics_dt);kind,axis,amp=CASES[D['i']]
 if D['phase']==1:
  v=parts._np(asset.data.root_link_lin_vel_w)[0];w=parts._np(asset.data.root_link_ang_vel_w)[0]
  force=.022*(v-D['v'])/dt;inertia=np.array([.022*(.1**2+.002**2)/12]*2+[.022*(2*.1**2)/12]);torque=inertia*(w-D['w'])/dt
  row=dict(case=D['i'],kind=kind,axis=axis,command=amp,force=force.tolist(),torque=torque.tolist(),velocity=v.tolist(),omega=w.tolist())
  with open(os.environ['COUPON_OUT'],'a') as f:f.write(json.dumps(row)+'\n')
  D['i']+=1;D['phase']=0
  return
 p=np.array([.325,0,1.203]);q=np.array([0.,0,0,1]);v=np.zeros(3);w=np.zeros(3)
 if kind=='position':p[axis]+=amp
 if kind=='velocity':v[axis]=amp
 if kind=='angle':q[axis]=math.sin(amp/2);q[3]=math.cos(amp/2)
 if kind=='omega':w[axis]=amp
 org=parts._np(sim.env.unwrapped.scene.env_origins[0]);pose=torch.tensor([[*(p+org),*q]],dtype=torch.float32,device=asset.device);vel=torch.tensor([[*v,*w]],dtype=torch.float32,device=asset.device)
 asset.write_root_pose_to_sim_index(root_pose=pose);asset.write_root_velocity_to_sim_index(root_velocity=vel);D.update(phase=1,v=v,w=w)
def score(o):return 0.
def success(o):return False
def demo():
 from yam_arena import ik,station
 from yam_arena.demo import READY
 out={}
 for arm in ['left','right']:
  p,r=ik.tcp_pose(np.array(READY),station.BASES[arm]);out[arm]=[Waypoint(4.,tuple(p),FINGER_CLOSED,'inair test')]
 return out
parts._cassette_namespace=globals()
