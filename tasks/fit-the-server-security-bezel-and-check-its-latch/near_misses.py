"""Released engagement, visible flag and physical proof certificate are independent."""
import math,numpy as np
import task
from yam_arena.task_file import ObjectState
B=task.BEZEL.state(pos=task.TARGET,joints={'latch':0.})
def probe(**kw):
    p=ObjectState(np.zeros(3),np.array((0,0,0,1)),np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3))
    object.__setattr__(p,'bezel_probe',dict(known=True,fresh=True,clear=True,bad=False,hold=1.1,still=True,proof_ok=True,proof=1.,proof_ticks=240,proof_force=[-10.,0.,0.])|kw);return p
def case(b=B,**kw):return {'bezel':b,'measurement':probe(**kw)}
a=math.radians(8);openB=task.BEZEL.state(pos=(.370-.215*math.sin(a),-.217+.215*math.cos(a),task.TARGET[2]),quat_xyzw=(0,0,math.sin(a/2),math.cos(a/2)),joints={'latch':0})
CASES=[
 ('complete released fitted proof',case(),1.,True),
 ('fitted before pull test',case(proof_ok=False),.5,False),
 ('retracted latch cannot complete',case(task.BEZEL.state(pos=task.TARGET,joints={'latch':.005})),.5,False),
 ('tabs engaged but bezel open',case(openB,proof_ok=False),.25,False),
 ('tabs not inserted sideways',case(task.BEZEL.state(pos=task.TARGET+(0,.020,0))),0.,False),
 ('bezel10mm proud',case(task.BEZEL.state(pos=task.TARGET+(-.010,0,0))),0.,False),
 ('bezel20mm high',case(task.BEZEL.state(pos=task.TARGET+(0,0,.020))),0.,False),
 ('held',case(clear=False),0.,False),
 ('unknown native measurement',case(known=False),0.,False),
 ('stale native observer cannot credit fitted bezel',case(fresh=False),0.,False),
 ('overload or failed native proof',case(bad=True),0.,False),
 ('not still',case(still=False),0.,False),
 ('short release',case(hold=.9),0.,False),
 ('short native pull cannot complete',case(proof=.999),.5,False),
 ('no native proof samples cannot complete',case(proof_ticks=0),.5,False),
 ('wrong native force cannot complete',case(proof_force=[-9.,0.,0.]),.5,False),
 ('face inward2mm inclusive',case(task.BEZEL.state(pos=task.TARGET+(.002,0,0))),1.,True),
 ('face beyond2mm allowance',case(task.BEZEL.state(pos=task.TARGET+(.002002,0,0))),.25,False),
 ('lateral+3mm inclusive',case(task.BEZEL.state(pos=task.TARGET+(0,.003,0))),1.,True),
 ('lateral beyond+3mm allowance',case(task.BEZEL.state(pos=task.TARGET+(0,.003002,0))),.25,False),
 ('lateral-3mm inclusive',case(task.BEZEL.state(pos=task.TARGET+(0,-.003,0))),1.,True),
 ('lateral beyond-3mm allowance',case(task.BEZEL.state(pos=task.TARGET+(0,-.003002,0))),.25,False),
]
# Isolate published face/height/tilt gates: finite tab clearance can be stricter.
for dz,expected in ((.002,True),(.002002,False),(-.002,True),(-.002002,False)):
 assert task.frame_aligned(task.BEZEL.state(pos=task.TARGET+(0,0,dz))) is expected
for da,expected in ((0,True),(2e-6,False)):
 a=math.radians(3)+da
 assert task.frame_aligned(task.BEZEL.state(pos=task.TARGET,quat_xyzw=(math.sin(a/2),0,0,math.cos(a/2)))) is expected
from yam_arena import analysis
v=np.array([(x,y,z) for x in (-.06,.06) for y in (-.04,.04) for z in (.0005,.0105)])
h=analysis.Hull(v,np.array([(1,0,0,.06),(-1,0,0,.06),(0,1,0,.04),(0,-1,0,.04),(0,0,1,.0105),(0,0,-1,-.0005)],float))
b=(np.array((0,0,-.005)),np.eye(3),np.array((.02,.02,.01)))
assert analysis.box_signed_distance(v,*b).min()>.005
assert analysis.box_signed_distance(task.release.faces(h),*b).min()<.001
# Full native release path: hands far, a forearm face0.5mm above the bezel.
# Its vertices alone are >5mm away, so this also exercises face sampling.
from types import SimpleNamespace as N
class Tensor:
 def __init__(self,a):self.a=np.asarray(a)
 def detach(self):return self
 def cpu(self):return self
 def numpy(self):return self.a
saved_hulls=analysis.link_hulls;saved_faces=task.FACES
names=list(saved_hulls());fake_hulls={n:[h] for n in names}
poses=np.tile((2.,2.,2.,0.,0.,0.,1.),(len(names),1));poses[names.index('link3')]=(0,0,0,0,0,0,1)
arm=N(body_names=names,data=N(body_link_pose_w=Tensor(poses[None,:,:])))
class Scene(dict):env_origins=Tensor(np.zeros((1,3)))
# _np is intentionally a Torch-shaped adapter, including native indexing.
Tensor.__getitem__=lambda self,k:Tensor(self.a[k])
sim=N(env=N(unwrapped=N(scene=Scene(left_robot=arm,right_robot=arm))))
objects={'bezel':N(low=np.array((-.01,-.01,-.01)),high=np.array((.01,.01,0))), 'rack_and_chassis':N(low=np.ones(3)*4,high=np.ones(3)*5)}
try:
 analysis.link_hulls=lambda:fake_hulls;task.FACES={}
 assert task.clear_hands(sim,objects) is False
 CASES.append(('hands far but forearm face0.5mm from bezel',case(clear=False),0.,False))
 poses[names.index('link3'),:3]=2.
 assert task.clear_hands(sim,objects) is True
 arm.body_names=names[:-1];arm.data.body_link_pose_w=Tensor(poses[None,:-1,:])
 assert task.clear_hands(sim,objects) is None
finally:
 analysis.link_hulls=saved_hulls;task.FACES=saved_faces
