"""Endpoint and causal-law controls, including exact tier edges and persistent faults."""
import pathlib,sys
import numpy as np
from yam_arena.task_file import load
folder=pathlib.Path(__file__).resolve().parent
# Use the already-loaded task module where the factory installs it under a generated name.
m=next((v for v in list(sys.modules.values()) if hasattr(v,'Tissue') and getattr(v,'__file__','') and pathlib.Path(v.__file__).resolve()==folder/'task.py'),None)
if m is None:
 import gc
 m=next((v for v in gc.get_objects() if isinstance(v,type(sys)) and hasattr(v,'Tissue') and getattr(v,'__file__','') and pathlib.Path(v.__file__).resolve()==folder/'task.py'),None)
if m is None:
 import importlib.util
 spec=importlib.util.spec_from_file_location('leaf_control_task',folder/'task.py');m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)

class Endpoint(dict):
 def __init__(self,n=10,**flags):
  self.n=n;self.flags=flags
  super().__init__({m.BARREL.name:m.BARREL.state(),m.PLUNGER.name:m.PLUNGER.state(pos=(.20,-.24,m.TOP+.018)),m.CUP.name:m.CUP.state()})
 def items(self):
  t=m.Tissue();t.infiltrated[:self.n]=True;t.location[:]=1;t.known=True;t.clear=True;t.down=True;t.wait=15.;t.held=1.
  for k,v in self.flags.items():
   if k=='missing':t.location[0]=0
   elif k=='damage':t.crushed[0]=True
   elif k=='stale':continue
   else:setattr(t,k,v)
  t.cert=m.stamp(self)
  if self.flags.get('stale'):t.cert=()
  m.T=t
  return super().items()

CASES=[('no motion',{},0.,False)]
for n,expected in [(0,0.),(1,.25),(4,.25),(5,.5),(7,.5),(8,.75),(9,.75),(10,1.)]:CASES.append((f'{n} intact sinking discs',Endpoint(n),expected,expected==1.))
for name,flags in [('missing disc',dict(missing=True)),('crushed then restored pose',dict(damage=True)),('persistent spill flag',dict(zero=True)),('unknown force',dict(known=False)),('hands near',dict(clear=False)),('apparatus airborne',dict(down=False)),('14.999s count',dict(wait=14.999)),('stale endpoint',dict(stale=True))]:CASES.append((name,Endpoint(**flags),0.,False))
CASES.extend([('still hold below1s',Endpoint(held=.999),.75,False),('exact1s',Endpoint(held=1.),1.,True),('exact15s',Endpoint(wait=15.),1.,True)])

# The causal model itself must reject the easy routes, independent of fabricated endpoints.
def law_controls():
 t=m.Tissue()
 t.vacuum(0.,0.,True,True,.05)
 for _ in range(240):t.vacuum(.035,.0,False,True,.05)
 assert not t.infiltrated.any(),'open-tip draw infiltrates'
 t=m.Tissue()
 t.vacuum(0.,0.,True,True,.05)
 for _ in range(220):t.vacuum(.029,.0,True,True,.05)
 t.vacuum(.029,0,False,True,.05);assert not t.infiltrated.any(),'short stroke infiltrates'
 t=m.Tissue()
 t.vacuum(0.,0.,True,True,.05)
 for _ in range(220):t.vacuum(.035,0,True,False,.05)
 assert not t.infiltrated.any(),'out-of-bore stroke infiltrates'
 for seconds,n in [(8.,0),(8.2,1),(8.8,4),(9.,5),(9.4,7),(9.6,8),(9.8,9),(10.,10)]:
  t=m.Tissue()
  t.vacuum(0.,0.,True,True,.05)
  for _ in range(round(seconds/.05)):t.vacuum(.035,0,True,True,.05)
  assert not t.infiltrated.any(),'vacuum without pressure restoration'
  t.vacuum(.035,0,False,True,.05);assert t.infiltrated.sum()==n,(seconds,t.infiltrated)
 t=m.Tissue();t.vacuum(0.,0.,True,True,.05);f=t.vacuum(.035,.1,True,True,.05)
 assert 0<f<32.,'loaded draw lacks resisting force'
 for _ in range(100):f=t.vacuum(.035,0,True,True,.05)
 assert 15<f<32.,'held vacuum lacks restorative force'
 t.vacuum(float('nan'),0,True,True,.05);assert t.zero,'NaN fails open'
 t.vacuum(.035,0,True,True,.05);assert t.zero,'damage resets within trial'
 # Closing the outlet after an atmospheric draw cannot create vacuum.
 t=m.Tissue()
 for _ in range(30):t.vacuum(.035,0,False,True,.05)
 for _ in range(240):t.vacuum(.035,0,True,True,.05)
 t.vacuum(.035,0,False,True,.05)
 assert not t.infiltrated.any() and t.pressure==0.,'late-cap bypass'
 # Empty-syringe pumping must not affect specimens already in the cup.
 t=m.Tissue();t.location[:]=1;t.vacuum(0,0,True,True,.05)
 for _ in range(240):t.vacuum(.035,0,True,True,.05)
 t.vacuum(0,0,True,True,.05)
 assert not t.infiltrated.any(),'empty syringe bypass'
 # Two short interrupted holds cannot be added together.
 t=m.Tissue();t.vacuum(0,0,True,True,.05)
 for _ in range(120):t.vacuum(.035,0,True,True,.05)
 t.vacuum(.02,0,True,True,.05)
 for _ in range(120):t.vacuum(.035,0,True,True,.05)
 t.vacuum(0,0,True,True,.05)
 assert not t.infiltrated.any(),'interrupted exposure accumulates'
 # Restoring pressure with the tip still capped is a valid cycle.
 t=m.Tissue();t.vacuum(0,0,True,True,.05)
 for _ in range(200):t.vacuum(.035,0,True,True,.05)
 t.vacuum(0,0,True,True,.05)
 assert t.infiltrated.all(),'capped pressure restoration omitted'
 assert m.cup_capacity_ml(1.)>42 and m.cup_capacity_ml(0.)==0
 # Independent final-review regressions.
 t=m.Tissue();t.removed=True;t.vacuum(-.060,0,False,False,.05)
 assert not t.zero,'removed piston creates fictitious negative gas inventory'
 clouds=[np.array([[10.,10.,10.]]) for _ in range(6)];tip=np.zeros(3)
 clouds[4]=np.array([[0.,0.,0.]])
 assert m.finger_cap(clouds,tip,[False,False,True,False]),'right fingertip cannot seal'
 clouds[4]=np.array([[10.,10.,10.]]);clouds[1]=np.array([[0.,0.,0.]])
 assert m.finger_cap(clouds,tip,[True,False,False,False]),'left fingertip cannot seal'
 clouds[1]=np.array([[10.,10.,10.]])
 assert not m.finger_cap(clouds,tip,[True,True,True,True]),'no fingertip falsely seals'
 # Piston nose is held down onto a cup disc: contact must exceed1N and latch damage.
 quat=(-2**-.5,0.,0.,2**-.5);disc=np.array((.28,.17,m.TOP+.003))
 tool=m.PLUNGER.state(pos=tuple(disc+np.array((0.,0.,.006))),quat_xyzw=quat)
 load=m.tool_tissue_load(m.PLUNGER,tool,disc)
 assert load>1.,'tool-transmitted tissue damage omitted'
 t=m.Tissue();t.known=True;t.infiltrated[:]=True;t.location[:]=1;t.down=True;t.wait=15.05;t.held=15.05
 t.damage_load[0]=load;t.crushed[0]=load>1.
 assert t.value(True,np.ones(10,dtype=bool),True)==0.,'removed-piston crush can earn full score'
 remote=m.PLUNGER.state(pos=(.20,-.24,m.TOP+.0265))
 assert m.tool_tissue_load(m.PLUNGER,remote,disc)==0.,'unrelated remote tool damages cup'
 clouds[1]=np.array([[0.,0.,0.]])
 assert not m.finger_cap(clouds,tip,[False]*4),'hover without native contact seals'
 # A removed-and-reinserted piston still physically blocks the rear opening.
 b=m.BARREL.state();p=m.PLUNGER.state()
 mouth=m.geometry(b,p)[5]
 assert not m.rear_exit_clear(p,mouth),'seated shaft permits pouring'
 assert not m.rear_exit_clear(m.PLUNGER.state(pos=(m.X,-.07,m.Z)),mouth),'stopper blocks rear mouth'
 assert m.rear_exit_clear(remote,mouth),'remote piston blocks a clear opening'
 # A supported piston compresses in-barrel tissue even without finger-piston contact.
 d=b.pos+np.array((.002,.020,-.002))
 p=m.PLUNGER.state(pos=(m.X,.020,m.Z))
 assert m.tool_tissue_load(m.PLUNGER,p,d)>1.,'supported piston intrusion misses damage'
 # A clear mouth and valid rim do not permit teleporting through the removed piston.
 mouth=np.array(m.CUP.pos)+np.array((0.,0.,.120))
 objects={o.name:o.state() for o in m.OBJECTS}
 objects[m.PLUNGER.name]=remote
 far=[np.array([[5.,5.,5.]]) for _ in range(6)]
 assert m.cup_capture(objects[m.CUP.name],mouth)
 assert m.drop_path_clear(objects,far,mouth,objects[m.CUP.name]),'clear drop blocked'
 objects[m.PLUNGER.name]=m.PLUNGER.state(pos=np.array(m.CUP.pos)+np.array((0.,0.,.0805)))
 assert m.rear_exit_clear(objects[m.PLUNGER.name],mouth),'test piston blocks the mouth itself'
 assert not m.drop_path_clear(objects,far,mouth,objects[m.CUP.name]),'drop passes through piston'
 objects[m.PLUNGER.name]=remote
 blocked=far.copy();blocked[0]=np.array([mouth-[0,0,.020]])
 assert not m.drop_path_clear(objects,blocked,mouth,objects[m.CUP.name]),'drop passes through hand'
 blocked[0]=np.array([mouth+[.003,0,-.020]])
 assert not m.drop_path_clear(objects,blocked,mouth,objects[m.CUP.name]),'3mm path boundary omitted'
 blocked[0]=np.array([mouth+[.003001,0,-.020]])
 assert m.drop_path_clear(objects,blocked,mouth,objects[m.CUP.name]),'outside3mm path rejected'
 return 37
law_controls()
