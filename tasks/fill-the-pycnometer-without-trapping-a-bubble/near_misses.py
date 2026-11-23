"""Independent endpoint expectations and causal controls; fabricated endpoints are not robot evidence."""
import sys,pathlib,numpy as np
folder=pathlib.Path(__file__).resolve().parent
m=next((v for v in list(sys.modules.values()) if getattr(v,'__file__','') and pathlib.Path(v.__file__).resolve()==folder/'task.py'),None)
if m is None:
 import gc
 m=next((v for v in gc.get_objects() if isinstance(v,type(sys)) and getattr(v,'__file__','') and pathlib.Path(v.__file__).resolve()==folder/'task.py'),None)
if m is None:
 import importlib.util
 sp=importlib.util.spec_from_file_location('pyc_controls',folder/'task.py');m=importlib.util.module_from_spec(sp);sys.modules[sp.name]=m;sp.loader.exec_module(m)
class Endpoint(dict):
 def __init__(self,**flags):
  self.flags=flags;pos=(.315,.03,m.TOP+.006)
  bq=flags.get('bottle_quat',(0.,0.,0.,1.));R=m.parts._quat_to_matrix(bq)
  sp=np.asarray(pos)+R@np.array((flags.get('stopper_dx',0.),0.,.04985+flags.get('stopper_dz',0.)))
  super().__init__({m.BOTTLE.name:m.BOTTLE.state(pos=pos,quat_xyzw=bq),m.STOPPER.name:m.STOPPER.state(pos=sp,quat_xyzw=flags.get('stopper_quat')),m.BATH.name:m.BATH.state()})
 def items(self):
  t=m.fluid.Water();t.water=50.;t.air=0.;t.pinned=False;t.filled=True;t.cleared=True;t.seated=True;t.seal=True;t.overflow=2.;t.counterheld_overflow=2.;t.known=True;t.clear=True;t.down=True;t.support=np.array((.4,.19));t.held=1.;t.loaded_steps=100
  for k,v in self.flags.items():
   if k=='stale' or k.startswith(('stopper_','bottle_')):continue
   setattr(t,k,v)
  t.cert=m.stamp(self)
  if self.flags.get('stale'):t.cert=()
  m.T=t
  return super().items()
CASES=[('no-op',{},0.,False)]
for label,flags,value in [
 ('released full sequence',{},1.),('fill only',dict(water=45.,air=1.2,cleared=False,seated=False),.25),
 ('45mL exact',dict(water=45.,air=1.2),.25),('under45mL',dict(water=44.999),0.),
 ('1mL air exact',dict(air=1.,cleared=False,seated=False),.5),('above1mL',dict(air=1.0001),.25),
 ('0.2mL exact no seat',dict(air=.2,seated=False),.75),('above0.2mL',dict(air=.2001),.5),
 ('seat with retained shoulder air',dict(water=48.8,air=1.2,cleared=False,seated=False),.25),
 ('clear without cumulative fill',dict(filled=False),0.),('seat without clearance history',dict(cleared=False),.5),
 ('overflow without counterholding',dict(counterheld_overflow=0.),.75),('counterheld overflow below0.15',dict(counterheld_overflow=.1499),.75),('counterheld overflow exact0.15',dict(counterheld_overflow=.15),1.),('NaN counterheld overflow',dict(counterheld_overflow=float('nan')),0.),
 ('no contact-established seal',dict(seal=False),.75),('NaN capillary loss',dict(capillary_out=float('nan')),0.),('NaN bypass',dict(bypass_out=float('nan')),0.),('NaN applied pressure',dict(applied_pressure=float('nan')),0.),('no overflow',dict(overflow=0.,seated=False),.75),('overflow below0.15mL',dict(overflow=.1499),.75),
 ('overflow exact0.15mL',dict(overflow=.15),1.),('mass deficit is missing internal air volume',dict(water=49.79),.5),
 ('exterior film cannot replace missing internal water',dict(water=49.5,external=.5),.5),
 ('exterior2g excluded from correct interior mass',dict(external=2.),1.),
 ('NaN water',dict(water=float('nan')),0.),('NaN air',dict(air=float('nan')),0.),('NaN overflow',dict(overflow=float('nan')),0.),('NaN exterior inventory',dict(external=float('nan')),0.),
 ('unsupported stopper',dict(support=np.array((.4,0.))),0.),('unsupported bottle',dict(support=np.array((0.,.19))),0.),('NaN contact support',dict(support=np.array((.4,float('nan')))),0.),
 ('20deg stopper',dict(stopper_quat=(np.sin(np.radians(10)),0,0,np.cos(np.radians(10)))),.75),('0.75mm short with proper volume',dict(stopper_dz=.00075,water=50.+m.fluid.PLUG_AREA*.00075*1e6),1.),('withdraw2mm creates air',dict(stopper_dz=.002),.5),('radial2mm exact',dict(stopper_dx=.002),1.),('radial over2mm',dict(stopper_dx=.0021),.75),
 ('bottle9deg stopper18deg',dict(bottle_quat=m.parts._matrix_to_quat(m.parts._rot((0,1,0),np.radians(9))),stopper_quat=m.parts._matrix_to_quat(m.parts._rot((0,1,0),np.radians(18)))),.75),
 ('bottle9deg stopper9deg',dict(bottle_quat=m.parts._matrix_to_quat(m.parts._rot((0,1,0),np.radians(9))),stopper_quat=m.parts._matrix_to_quat(m.parts._rot((0,1,0),np.radians(9)))),1.),
 ('hands near',dict(clear=False),0.),('airborne',dict(down=False),0.),('hold0.999s',dict(held=.999),0.),
 ('unknown observations',dict(known=False),0.),('permanent glass/spill damage',dict(zero=True),0.),
 ('no physical-load updates',dict(loaded_steps=0),0.),('stale snapshot',dict(stale=True),0.)]:
 CASES.append((label,Endpoint(**flags),value,value==1.))

def causal_controls():
 Z=np.zeros(3);I=np.eye(3)
 t=m.fluid.Water()
 for _ in range(200):t.update(.05,I,Z,Z,-.01,0.,0.,True,True)
 assert t.water==44. and abs(t.air-1.2)<1e-6,'no-op generates fill'
 for _ in range(200):t.update(.05,I,Z,Z,.006,1.,0.,True,True)
 assert t.water>=45 and abs(t.air-1.2)<1e-6 and t.pinned,'straight filling erases adhering bubble'
 for d in np.linspace(0,.01315,100):t.update(.05,I,Z,Z,.006,1.,d,False,True,seal_contact=True)
 assert not t.seated and abs(t.air-1.2)<1e-6,'seat erases bubble'
 t=m.fluid.Water()
 for _ in range(80):t.update(.05,I,Z,Z,.01,1.,0.,True,True)
 for heading in (np.pi/4,-np.pi/4):
  rot=m.parts._rot((np.sin(heading),-np.cos(heading),0),np.radians(30))
  for _ in range(120):t.update(.05,rot,Z,Z,.01,1.,0.,True,True)
  for _ in range(120):t.update(.05,I,Z,Z,.01,1.,0.,True,True)
 assert t.air<=.2 and t.water>51.8 and t.cleared and not t.zero,'actual migration fails'
 for d in np.linspace(0,.01315,100):t.update(.05,I,Z,Z,.006,1.,d,False,True,seal_contact=True)
 assert t.seated and abs(t.water-50)<1e-6 and t.overflow>=.15,'actual displacement fails'
 # Check the entire published physical preparation tolerance,not only its center.
 for pocket in (.09,.095,.1,.105,.11):
  t=m.fluid.Water();t.air=12*pocket;t.head=m.fluid.OPEN_CAPACITY-t.water-t.air
  for _ in range(80):t.update(.05,I,Z,Z,.01,1.,0.,True,True)
  assert abs(t.air-12*pocket)<1e-6 and t.pinned,'admitted pocket clears without tilt'
  for heading in (np.pi/4,-np.pi/4):
   rot=m.parts._rot((np.sin(heading),-np.cos(heading),0),np.radians(30))
   for _ in range(120):t.update(.05,rot,Z,Z,.01,1.,0.,True,True)
   for _ in range(120):t.update(.05,I,Z,Z,.01,1.,0.,True,True)
  assert t.air<=.2 and t.cleared and not t.zero,'admitted pocket cannot clear under modeled loads'
 t=m.fluid.Water()
 for _ in range(200):t.update(.05,m.parts._rot((0,1,0),-.698),Z,Z,.02,1.,0.,False,False)
 for _ in range(200):t.update(.05,I,Z,Z,.006,1.,0.,False,False)
 assert abs(t.air-1.2)<1e-6,'blocked mouth clears air'
 t=m.fluid.Water();t.water=m.fluid.OPEN_CAPACITY;t.air=0.;t.filled=t.cleared=True;t.bath_taken=t.water-t.initial
 for d in np.linspace(0,.01315,100):t.update(.05,I,Z,Z,.006,1.,d,False,True,False,seal_contact=True)
 assert t.overflow==0 and not t.seated and t.zero,'blocked incompressible insertion must invalidate'
 for _ in range(100):t.update(.05,I,Z,Z,.006,1.,m.fluid.FULL_DEPTH,False,True,True,seal_contact=True)
 assert abs(t.water-50.)<1e-6 and t.overflow>2.4 and t.zero,'unblocking retains excess or erases invalidation'
 # Repeated shallow pokes do not create the full swept volume.
 t=m.fluid.Water();t.water=m.fluid.OPEN_CAPACITY;t.air=0.;t.filled=t.cleared=True;t.bath_taken=t.water-t.initial
 for _ in range(12):
  t.update(.05,I,Z,Z,.006,1.,.001,False,True);t.update(.05,I,Z,Z,.006,1.,0.,False,True)
 assert t.overflow<m.fluid.PLUG_AREA*.001*1e6+1e-6 and not t.seated,'shallow pumping creates final volume'
 t=m.fluid.Water();t.update(.05,I,np.array((.1,0,0)),Z,.01,1.,0.,True,True)
 assert t.hydro[2]>0 and t.drag[0]<0 and np.linalg.norm(t.hydro)<=1.2 and np.linalg.norm(t.drag)<=.6,'force-free declared water'
 t.update(float('nan'),I,Z,Z,.01,1.,0.,True,True);assert t.zero
 t.update(.05,I,Z,Z,.01,1.,0.,True,True);assert t.zero,'fault erased'
 # Natural small grasp jitter must not perform the scored bubble clearance.
 for angle in (0.,2.,5.,10.):
  t=m.fluid.Water()
  for _ in range(80):t.update(.05,I,Z,Z,.01,1.,0.,True,True)
  for _ in range(100):t.update(.05,m.parts._rot((0,1,0),-np.radians(angle)),Z,Z,.01,1.,0.,True,True)
  assert abs(t.air-1.2)<1e-6 and not t.cleared and not t.path.any(),'jitter removes shoulder air'
 bath=m.BATH.state();dry=m.BOTTLE.state(pos=(.18,-.19,m.TOP+.004))
 mouth,waterline,inside,volume,center=m.immersion(dry,bath,I)
 assert not inside and volume==0.,'dry inspection pad admits water or buoyancy'
 t=m.fluid.Water()
 for _ in range(100):t.update(.05,I,Z,Z,-.1,0.,0.,True,True,displaced_volume=0.)
 assert t.water==44. and np.linalg.norm(t.hydro)<1e-12,'dry fill bypass'
 # Each actual independently vented pocket is cumulative evidence of clearing.
 t=m.fluid.Water()
 for _ in range(80):t.update(.05,I,Z,Z,.01,1.,0.,True,True)
 for _ in range(80):t.update(.05,m.parts._rot((0,1,0),-np.radians(20)),Z,Z,.01,1.,0.,True,True)
 for _ in range(100):t.update(.05,I,Z,Z,.01,1.,0.,True,True)
 assert 0<t.air<1.2 and 0<np.count_nonzero(t.volumes>1e-9)<12,'independent pockets not retained'
 t=m.fluid.Water();t.water=m.fluid.OPEN_CAPACITY;t.air=0.;t.bath_taken=t.water-t.initial
 before=t.water
 for _ in range(100):t.update(.05,m.parts._rot((0,1,0),np.radians(63)),Z,Z,-.1,0.,0.,True,True,displaced_volume=0.)
 assert t.water<49. and t.out>1. and not t.zero,'dry tilted bottle retains impossible water'
 # Resisting torque may not reverse a free glass body's angular momentum within a control step.
 for closed,cap in ((False,True),(True,True),(True,False)):
  t=m.fluid.Water();before=t.water
  for _ in range(20):t.update(.05,m.parts._rot((1,0,0),np.pi),Z,Z,.01,1.,0.,not closed,True,cap)
  assert t.water==before and not t.filled,'inverted free-fill bypass'
 t=m.fluid.Water();before=t.water;t.seal=True
 for _ in range(20):t.update(.05,m.parts._rot((1,0,0),np.pi),Z,Z,-.01,0.,.01315,False,True,False)
 assert t.water==before,'sealed blocked stopper drains'
 for speed in (0.,.01,1.,10.,100.):
  w=np.array((0.,0.,speed));tau=m.fluid.rotational_drag(w,.05)
  assert np.dot(w,tau)<=1e-15 and np.linalg.norm(tau)*.05<=.20*1e-5*speed+1e-12,'angular damper injects energy'
 print('causal water controls: PASS')
causal_controls()

def callback_controls():
 # Prescribed states isolate the callback; these are not robot demonstrations.
 import types
 saved={name:getattr(m,name) for name in ('liquid_mass','wrench','_bubble_op','_bubble_size','_flow_op','_flow_size','_time','_prev','_flow_prev','T')}
 old_pxr=sys.modules.get('pxr');obs={name:getattr(m.observation,name) for name in ('hand_state','nonfinger_contact_load','support_loads','counterhold_forces','last_glass_contact')}
 class Op:
  def Set(self,*args):pass
 class Sim:
  t=0.
  env=types.SimpleNamespace(unwrapped=types.SimpleNamespace(scene=types.SimpleNamespace(env_origins=[np.zeros(3)])))
  def states(self):return self.objects
 try:
  sys.modules['pxr']=types.SimpleNamespace(Gf=types.SimpleNamespace(Vec3d=lambda *v:v,Vec3f=lambda *v:v))
  m._bubble_op=[Op() for _ in range(12)];m._bubble_size=[Op() for _ in range(12)];m._flow_op=Op();m._flow_size=Op()
  m.liquid_mass=lambda sim:None;m.wrench=lambda *args:None;m._time=0.;m._prev=None;m._flow_prev=0.;m.T=m.fluid.Water()
  m.observation.hand_state=lambda sim:([np.array([[-2.,-2.,3.]]) for _ in range(6)],0.)
  m.observation.nonfinger_contact_load=lambda sim:0.;m.observation.support_loads=lambda sim:np.array((.4,.19));m.observation.counterhold_forces=lambda sim:np.array(((2.,0.),(2.,0.),(0.,1.),(0.,1.)));m.observation.last_glass_contact=.3
  sim=Sim();sim.objects={p.name:p.state() for p in m.OBJECTS}
  def advance(pos,R=np.eye(3),n=1):
   sim.objects[m.BOTTLE.name]=m.BOTTLE.state(pos=pos,quat_xyzw=m.parts._matrix_to_quat(R))
   for _ in range(n):sim.t+=.05;m.step(sim)
  wet=(.315,.03,m.TOP+.006);advance(wet,n=100)
  for h in (np.pi/4,-np.pi/4):
   advance((.315,.03,m.TOP+.1),m.parts._rot((np.sin(h),-np.cos(h),0),np.pi/6),120);advance(wet,n=120)
  assert m.T.air<1e-8 and m.score(sim.objects)==.75,'control sequence failed'
  import copy
  cleared=copy.deepcopy(m.T);m.observation.last_glass_contact=0.
  for depth in np.linspace(0.,m.fluid.FULL_DEPTH,80):
   sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=np.asarray(wet)+np.array((0.,0.,.063-depth)))
   advance(wet)
  assert m.T.overflow==0. and m.T.counterheld_overflow==0. and m.T.bypass_out>2.4 and not m.T.seal,'pose-only unsealed flow credited as capillary'
  m.T=copy.deepcopy(cleared);m._prev=0.;m.observation.last_glass_contact=.3
  sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=np.asarray(wet)+np.array((0.,0.,.04985)))
  advance(wet)
  assert m.T.overflow==0. and not m.T.seal,'unloaded first travel slice established seal'
  advance(wet)
  assert m.T.counterheld_overflow==0.,'stationary pressure created new displacement witness'
  m.T=cleared;m._prev=0.;m.observation.last_glass_contact=.3
  for depth in np.linspace(0.,m.fluid.FULL_DEPTH,80):
   sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=np.asarray(wet)+np.array((0.,0.,.063-depth)))
   advance(wet)
  shelf=np.asarray(m.SHELF.pos)+np.array((0.,0.,.028))
  sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=shelf+np.array((0.,0.,.04985)))
  advance(shelf,n=25)
  assert m.T.down and m.score(sim.objects)==1.,'supported shelf finish rejected'
  sim.objects[m.STOPPER.name]=m.STOPPER.state();advance(wet,n=100)
  before=m.T.out;advance((.24,.33,m.TOP+.15),m.parts._rot((-1,0,0),np.pi/2),30)
  assert m.T.out>before+1. and m.T.zero,'mouth outside catch was not latched'
  advance(wet,n=160);assert m.score(sim.objects)==0.,'refilling erased outside-catch spill'
  m.T=m.fluid.Water();m.T.water=50.;m.T.air=0.;m.T.bath_taken=6.;m.T.filled=m.T.cleared=True;m.T.overflow=m.T.counterheld_overflow=.2;m.T.seal=True
  inverted=m.parts._rot((1,0,0),np.pi);outside=np.array((.315,.4,m.TOP+.2))
  sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=outside+inverted@np.array((0.,0.,.04985)),quat_xyzw=m.parts._matrix_to_quat(inverted))
  advance(outside,inverted,100)
  assert m.T.capillary_out>1. and m.T.zero and m.T.overflow==.2,'inverted seated capillary loss bypass'
  sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=np.asarray(wet)+np.array((0.,0.,.04985)))
  advance(wet,n=25);assert m.score(sim.objects)==0.,'upright return erases capillary spill'
  m.T=m.fluid.Water();m.T.water=50.;m.T.air=0.;m.T.bath_taken=6.;m.T.filled=m.T.cleared=True
  sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=(.315,.03,m.TOP+.3))
  m.observation.support_loads=lambda sim:np.array((.4,0.))
  advance(wet,n=30);assert m.score(sim.objects)==0. and not m.T.down,'unsupported stopper credited'
  print('callback spill/support/shelf regressions: PASS (prescribed states, not robot evidence)')
 finally:
  for name,value in saved.items():setattr(m,name,value)
  for name,value in obs.items():setattr(m.observation,name,value)
  if old_pxr is None:sys.modules.pop('pxr',None)
  else:sys.modules['pxr']=old_pxr
if __name__=='__main__':callback_controls()
