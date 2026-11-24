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
  t=m.fluid.Water();t.water=50.;t.air=0.;t.filled=True;t.cleared=True;t.seated=True;t.pressed=True;t.seal=True;t.overflow=2.;t.attempt_overflow=2.;t.counterheld_overflow=2.;t.known=True;t.clear=True;t.down=True;t.support=np.array((.4,.19));t.held=1.;t.still=True;t.loaded_steps=100
  for k,v in self.flags.items():
   if k=='stale' or k.startswith(('stopper_','bottle_')):continue
   setattr(t,k,v)
   if k=='overflow' and 'attempt_overflow' not in self.flags:t.attempt_overflow=v
  t.cert=m.stamp(self)
  if self.flags.get('stale'):t.cert=()
  m.T=t
  return super().items()
CASES=[('no-op',{},0.,False)]
for label,flags,value in [
 ('released full sequence',{},1.),('fill only without bubble recovery',dict(water=45.,air=1.2,cleared=False,seated=False),0.),
 ('45mL exact',dict(water=45.,air=1.2),.25),('under45mL',dict(water=44.999),0.),
 ('1mL air exact after recovery',dict(air=1.,seated=False),.5),('above1mL',dict(air=1.0001),.25),
 ('0.2mL exact no seat',dict(air=.2,seated=False),.75),('above0.2mL',dict(air=.2001),.5),
 ('stopper seated over1.2mL unfilled volume',dict(water=48.8,air=1.2,cleared=False,seated=False),0.),('normal first-fill with0.25mL shoulder air',dict(water=52.199385,air=.25,filled=True,cleared=False,seated=False,overflow=0.,counterheld_overflow=0.,seal=False),0.),
 ('clear without cumulative fill',dict(filled=False),0.),('seat without clearance history',dict(cleared=False),0.),
 ('overflow without counterholding',dict(counterheld_overflow=0.),.75),('counterheld overflow below0.15',dict(counterheld_overflow=.1499),.75),('counterheld overflow exact0.15',dict(counterheld_overflow=.15),1.),('NaN counterheld overflow',dict(counterheld_overflow=float('nan')),0.),
 ('prior insertions cannot supply final flow',dict(attempt_overflow=.1499,overflow=2.),.75),('NaN final-insertion overflow',dict(attempt_overflow=float('nan')),0.),
 ('no native axial seating press',dict(pressed=False),.75),('no contact-established seal',dict(seal=False),.5),('NaN capillary loss',dict(capillary_out=float('nan')),0.),('NaN bypass',dict(bypass_out=float('nan')),0.),('NaN applied pressure',dict(applied_pressure=float('nan')),0.),('no overflow',dict(overflow=0.,seated=False),.75),('overflow below0.15mL',dict(overflow=.1499),.75),
 ('overflow exact0.15mL',dict(overflow=.15),1.),('mass deficit is missing internal air volume',dict(water=49.79),.5),
 ('exterior film cannot replace missing internal water',dict(water=49.5,external=.5),.5),
 ('exterior2g excluded from correct interior mass',dict(external=2.),1.),
 ('NaN water',dict(water=float('nan')),0.),('NaN air',dict(air=float('nan')),0.),('NaN overflow',dict(overflow=float('nan')),0.),('NaN exterior inventory',dict(external=float('nan')),0.),
 ('unsupported stopper',dict(support=np.array((.4,0.))),0.),('unsupported bottle',dict(support=np.array((0.,.19))),0.),('NaN contact support',dict(support=np.array((.4,float('nan')))),0.),
 ('20deg stopper',dict(stopper_quat=(np.sin(np.radians(10)),0,0,np.cos(np.radians(10)))),.25),('0.75mm short with proper volume',dict(stopper_dz=.00075,water=50.+m.fluid.PLUG_AREA*.00075*1e6),1.),('withdraw2mm creates air',dict(stopper_dz=.002),.5),('radial2mm exact',dict(stopper_dx=.002),1.),('radial over2mm',dict(stopper_dx=.0021),.75),
 ('bottle9deg stopper18deg',dict(bottle_quat=m.parts._matrix_to_quat(m.parts._rot((0,1,0),np.radians(9))),stopper_quat=m.parts._matrix_to_quat(m.parts._rot((0,1,0),np.radians(18)))),.75),
 ('bottle9deg stopper9deg',dict(bottle_quat=m.parts._matrix_to_quat(m.parts._rot((0,1,0),np.radians(9))),stopper_quat=m.parts._matrix_to_quat(m.parts._rot((0,1,0),np.radians(9)))),1.),
 ('hands near',dict(clear=False),0.),('airborne',dict(down=False),0.),('hold0.999s',dict(held=.999),0.),('moving at scoring',dict(still=False),0.),
 ('unknown observations',dict(known=False),0.),('permanent glass/spill damage',dict(zero=True),0.),
 ('no physical-load updates',dict(loaded_steps=0),0.),('stale snapshot',dict(stale=True),0.)]:
 CASES.append((label,Endpoint(**flags),value,value==1.))

def causal_controls():
 Z=np.zeros(3);I=np.eye(3)
 def run(t,R=I,wet=True,n=80,depth=0.,cap=True,contact=False):
  for _ in range(n):t.update(.05,R,Z,Z,.004 if wet else -.1,1. if wet else 0.,depth,depth<=0,True,cap,seal_contact=contact)
 def rock(t):
  for heading in (np.pi/4,-np.pi/4):
   R=m.tilt.rotation(0.,30.,heading)@m.tilt.rotation(0.,0.,0.).T
   run(t,R,wet=True,n=120);run(t,wet=True,n=40)
  run(t,wet=True,n=100)
 t=m.fluid.Water();run(t,wet=False,n=200)
 assert abs(t.pocket-.25)<1e-9 and t.pinned and abs(t.water-m.START_WATER_ML)<1e-6,'no-op clears natural shoulder bubble'
 run(t,n=200)
 assert t.pocket==.25 and not t.cleared,'upright immersion silently clears shoulder air'
 # Model tolerance covers the full shoulder band and azimuth, not one-degree locations.
 # For every direction there is an ordinary45deg tilt toward its radial normal.
 tested=0
 for volume in (.20,.25,.30):
  for polar in (15.,30.,35.):
   for phi in np.linspace(0.,2*np.pi,8,endpoint=False):
    t=m.fluid.Water();t.pocket=volume;t.air=volume;t.water=t.initial=m.fluid.OPEN_CAPACITY-volume
    t.normal=np.array((np.sin(np.radians(polar))*np.cos(phi),np.sin(np.radians(polar))*np.sin(phi),np.cos(np.radians(polar))))
    run(t,n=10);assert t.pinned and abs(t.pocket-volume)<1e-9,'broad start not stable'
    # Gravity tilted toward the opposite radial direction increases tangential buoyancy.
    axis=np.array((-np.sin(phi),np.cos(phi),0.))
    R=m.parts._rot(axis,np.radians(45.))
    run(t,R,n=120);run(t,n=200)
    assert t.pocket<1e-6 and not t.zero,'broad shoulder direction not recoverable'
    tested+=1
 print(f'reset model sensitivity:{tested} volume/polar/azimuth combinations PASS;not hardware repeatability')
 # The fixed two-rock recovery also covers the stated broad preparation sector.
 for volume in (.23,.25,.27):
  for polar in (15.,30.,35.):
   for phi in np.radians((150.,165.,180.,195.,210.)):
    t=m.fluid.Water();t.pocket=t.air=volume;t.water=t.initial=m.fluid.OPEN_CAPACITY-volume
    t.normal=np.array((np.sin(np.radians(polar))*np.cos(phi),np.sin(np.radians(polar))*np.sin(phi),np.cos(np.radians(polar))))
    for heading in (np.pi/4,-np.pi/4):
     R=m.tilt.rotation(0.,40.,heading)@m.tilt.rotation(0.,0.,0.).T
     run(t,R,n=120);run(t,n=40)
    run(t,n=100)
    assert t.pocket<1e-6,'fixed rocks fail within preparation sector'
 print('fixed-rock model sensitivity:45 volume/polar/azimuth combinations PASS;not robot episodes')

 t=m.fluid.Water();rock(t)
 assert t.air<1e-6 and abs(t.water-m.fluid.OPEN_CAPACITY)<1e-6 and t.cleared,'rock/top-up route fails'
 for d in np.linspace(0.,m.fluid.FULL_DEPTH,100):run(t,n=1,depth=float(d),contact=True)
 assert t.seated and abs(t.water-50.)<1e-6 and t.overflow>=.15,'seat after bubble release fails'
 t=m.fluid.Water()
 for d in np.linspace(0.,m.fluid.FULL_DEPTH,100):run(t,n=1,depth=float(d),contact=True)
 assert t.air>=.25-1e-6 and not t.cleared and not t.seated,'stoppering hides untreated bubble'
 # Venting in air never adds water; dumping creates an independently scored deficit.
 t=m.fluid.Water();before=t.water
 run(t,m.parts._rot((1,0,0),np.pi),wet=False,n=100)
 assert t.water<45. and t.out>1. and not t.zero,'inversion retains impossible water'
 assert t.pocket<1e-6,'dry wall retains an impossible pinned gas bubble'
 run(t,n=200)
 assert t.air<1e-6 and abs(t.water-m.fluid.OPEN_CAPACITY)<1e-6,'drain/refill alternative loses water inventory'
 t=m.fluid.Water();t.pocket=t.air=0.;t.water=m.fluid.OPEN_CAPACITY;t.bath_taken=t.water-t.initial
 for d in np.linspace(0.,m.fluid.FULL_DEPTH,100):run(t,n=1,depth=float(d),cap=False,contact=True)
 assert t.zero and t.overflow==0.,'blocked capillary credited'
 t=m.fluid.Water();run(t,n=5)
 assert t.hydro[2]>0 and np.linalg.norm(t.hydro)<=1.2,'force-free water'
 t.update(float('nan'),I,Z,Z,.01,1.,0.,True,True);assert t.zero
 run(t);assert t.zero,'fault erased'
 for speed in (0.,.01,1.,10.,100.):
  w=np.array((0.,0.,speed));tau=m.fluid.rotational_drag(w,.05)
  assert np.dot(w,tau)<=1e-15 and np.linalg.norm(tau)*.05<=.20*1e-5*speed+1e-12
 print('causal water controls: PASS')
causal_controls()

def assembled_fit_controls():
 # Independent whole-bottle check; neck-only matching misses shoulder obstructions.
 tip=m.STOPPER_REFERENCE_TIP_Z_M
 for shape in m.BOTTLE.shapes:
  if shape.kind!='cylinder':continue
  lo=shape.at[2]-shape.size[1]/2;hi=shape.at[2]+shape.size[1]/2
  if min(hi,tip+m.STOPPER_GROUND_LENGTH_M)<=max(lo,tip):continue
  z=min(hi,tip+m.STOPPER_GROUND_LENGTH_M)
  inner=float(np.linalg.norm(shape.at[:2]))-shape.size[0]
  required=m.plug_r0+m.GROUND_RADIUS_SLOPE*(z-tip)
  assert inner>=required-1e-9,'shoulder collider blocks the reference stopper'
 assembled=m.STOPPER.state(pos=(0.,0.,tip))
 assert abs(assembled.high[2]-.1016)<1e-9,'assembled native height mismatch'
 print('whole-bottle reference fit: PASS; no shoulder obstruction,101.6mm assembled;not robot evidence')
assembled_fit_controls()

def callback_controls():
 # Prescribed states isolate the callback; these are not robot demonstrations.
 import types
 saved={name:getattr(m,name) for name in ('liquid_mass','wrench','_bubble_op','_bubble_size','_flow_op','_flow_size','_time','_prev','_flow_prev','T')}
 old_pxr=sys.modules.get('pxr');obs={name:getattr(m.observation,name) for name in ('hand_state','nonfinger_contact_load','support_loads','counterhold_forces','last_glass_contact','last_glass_vectors','last_hold_total_vectors')}
 class Op:
  def Set(self,*args):pass
 class Sim:
  t=0.
  env=types.SimpleNamespace(unwrapped=types.SimpleNamespace(scene=types.SimpleNamespace(env_origins=[np.zeros(3)])))
  def states(self):return self.objects
 try:
  sys.modules['pxr']=types.SimpleNamespace(Gf=types.SimpleNamespace(Vec3d=lambda *v:v,Vec3f=lambda *v:v))
  m._bubble_op=Op();m._bubble_size=Op();m._flow_op=Op();m._flow_size=Op()
  m.liquid_mass=lambda sim:None;m.wrench=lambda *args:None;m._time=0.;m._prev=None;m._flow_prev=0.;m.T=m.fluid.Water()
  m.observation.hand_state=lambda sim:([np.array([[-2.,-2.,3.]]) for _ in range(6)],0.)
  m.observation.nonfinger_contact_load=lambda sim:0.;m.observation.support_loads=lambda sim:np.array((.4,.19));m.observation.counterhold_forces=lambda sim:np.array(((2.,0.),(2.,0.),(0.,1.),(0.,1.)));m.observation.last_glass_contact=2.0;m.observation.last_glass_vectors=np.array(((0.,0.,-2.),(0.,0.,2.)))
  m.observation.last_hold_total_vectors=np.zeros((4,2,3));m.observation.last_hold_total_vectors[2:4,1,2]=1.
  sim=Sim();sim.objects={p.name:p.state() for p in m.OBJECTS}
  def advance(pos,R=np.eye(3),n=1):
   sim.objects[m.BOTTLE.name]=m.BOTTLE.state(pos=pos,quat_xyzw=m.parts._matrix_to_quat(R))
   for _ in range(n):sim.t+=.05;m.step(sim)
  start=np.asarray(m.BPOS);advance(start,n=40)
  assert m.T.air==.25 and abs(m.T.water-m.START_WATER_ML)<1e-6 and m.score(sim.objects)==0.,'untouched first fill earns credit'
  sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=(.27,-.15,m.TOP+.006))
  wet=(.315,.03,m.TOP+.006);advance(wet,n=100)
  for heading in (np.pi/4,-np.pi/4):
   advance(wet,m.tilt.rotation(0.,30.,heading)@m.tilt.rotation(0.,0.,0.).T,120);advance(wet,n=40)
  advance(wet,n=100)
  assert m.T.air<1e-8 and m.score(sim.objects)==.25,'cleared open bottle tier'
  import copy
  cleared=copy.deepcopy(m.T);m.observation.last_glass_contact=0.
  for depth in np.linspace(0.,m.fluid.FULL_DEPTH,80):
   sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=np.asarray(wet)+np.array((0.,0.,.063-depth)))
   advance(wet)
  assert m.T.overflow==0. and m.T.counterheld_overflow==0. and m.T.bypass_out>2.4 and not m.T.seal,'pose-only unsealed flow credited as capillary'
  for vectors in (np.array(((-2.,0.,0.),(2.,0.,0.))),np.array(((0.,0.,-1.961),(0.,0.,1.961)))):
   m.T=copy.deepcopy(cleared);m.observation.last_glass_contact=2.;m.observation.last_glass_vectors=vectors
   for depth in np.linspace(0.,m.fluid.FULL_DEPTH,80):
    sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=np.asarray(wet)+np.array((0.,0.,.063-depth)));advance(wet)
   advance(wet,n=25)
   assert not m.T.pressed and m.score(sim.objects)==.75,'side or subthreshold load credited as seating press'
  m.T=copy.deepcopy(cleared);m.observation.last_glass_contact=2.;m.observation.last_glass_vectors=np.array(((0.,0.,-2.),(0.,0.,2.)));m.observation.last_hold_total_vectors[2:4,1,2]=1.961/2
  for depth in np.linspace(0.,m.fluid.FULL_DEPTH,80):
   sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=np.asarray(wet)+np.array((0.,0.,.063-depth)));advance(wet)
  assert not m.T.pressed,'glass reaction/weight substituted for subthreshold fingertip input'
  m.observation.last_hold_total_vectors[2:4,1,2]=1.
  m.T=cleared;m._prev=0.;m.observation.last_glass_contact=2.0;m.observation.last_glass_vectors=np.array(((0.,0.,-1.962),(0.,0.,1.962)))
  for depth in np.linspace(0.,m.fluid.FULL_DEPTH,80):
   sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=np.asarray(wet)+np.array((0.,0.,.063-depth)))
   advance(wet)
  shelf=np.asarray(m.SHELF.pos)+np.array((0.,0.,.028))
  sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=shelf+np.array((0.,0.,.04985)))
  advance(shelf,n=25)
  assert m.T.down and m.score(sim.objects)==1.,'supported shelf finish rejected'
  # Stillness reads pose change, not reported body velocity.
  finish=copy.deepcopy(m.T);top=shelf+np.array((0.,0.,.04985))
  for k in range(25):
   sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=top+np.array((.0002*(k+1),0.,0.)));advance(shelf+np.array((.0002*(k+1),0.,0.)))
  assert m.score(sim.objects)==0.,'4mm/s drift credited as still'
  m.T=copy.deepcopy(finish);sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=top);advance(shelf,n=25)
  for k in range(25):
   sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=top,lin_vel=(0.,0.,-.03));advance(shelf)
  assert m.score(sim.objects)==1.,'stationary pose with reported solver velocity rejected'
  m.T=finish;sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=top);advance(shelf,n=25)
  sim.objects[m.STOPPER.name]=m.STOPPER.state();advance(wet,n=100)
  before=m.T.out;advance((.24,.33,m.TOP+.15),m.parts._rot((-1,0,0),np.pi/2),30)
  assert m.T.out>before+1. and m.T.zero,'mouth outside catch was not latched'
  advance(wet,n=160);assert m.score(sim.objects)==0.,'refilling erased outside-catch spill'
  m.T=m.fluid.Water();m.T.water=50.;m.T.air=0.;m.T.pocket=0.;m.T.bath_taken=m.T.water-m.T.initial;m.T.filled=m.T.cleared=True;m.T.overflow=m.T.counterheld_overflow=.2;m.T.seal=True
  inverted=m.parts._rot((1,0,0),np.pi);outside=np.array((.315,.4,m.TOP+.2))
  sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=outside+inverted@np.array((0.,0.,.04985)),quat_xyzw=m.parts._matrix_to_quat(inverted))
  advance(outside,inverted,100)
  assert m.T.capillary_out>1. and m.T.zero and m.T.overflow==.2,'inverted seated capillary loss bypass'
  sim.objects[m.STOPPER.name]=m.STOPPER.state(pos=np.asarray(wet)+np.array((0.,0.,.04985)))
  advance(wet,n=25);assert m.score(sim.objects)==0.,'upright return erases capillary spill'
  m.T=m.fluid.Water();m.T.water=50.;m.T.air=0.;m.T.pocket=0.;m.T.bath_taken=m.T.water-m.T.initial;m.T.filled=m.T.cleared=True
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

def native_force_controls():
 # Actual force-vector projection controls,not robot/hardware measurements.
 z=np.array((0.,0.,1.));f=m.observation.compressive_axial
 assert f(-2*z,2*z,z)==2.
 assert f(2*z,-2*z,z)==0.,'tension mistaken for seating'
 assert f(np.array((10.,0.,0.)),np.array((-10.,0.,0.)),z)==0.,'side bump mistaken for seating'
 assert f(-3*z,1.5*z,z)==1.5,'one-sided force accepted'
 try:f(np.full(3,np.nan),2*z,z)
 except ValueError:pass
 else:raise AssertionError('unknown contact accepted')
 print('native compressive axial contact controls: PASS;equal/opposite vectors,side/tension/unknown rejection')
if __name__=='__main__':native_force_controls()


def support_view_controls():
 import types
 o=m.observation;saved={k:getattr(o,k) for k in ('_support_contact_view','last_glass_contact','last_glass_pair','last_glass_vectors','last_glass_normal_vectors','last_glass_friction_vectors')}
 raw=np.zeros((2,15,3));raw[0,:6,2]=.003;raw[1,1:6,2]=.147
 friction=np.zeros_like(raw)
 class View:
  # Separate, sparse friction anchors deliberately use DIFFERENT indices/counts.
  def __init__(self):self.counts=np.zeros((2,15),dtype=np.int32);self.starts=self.counts.copy()
  def get_contact_force_matrix(self,**kw):return raw
  def get_contact_data(self,**kw):
   self.counts[:]=1;self.starts[:]=np.arange(30).reshape(2,15)
   f=np.full((128,1),np.nan);n=np.full((128,3),np.nan)
   f[:30,0]=np.linalg.norm(raw,axis=2).ravel();n[:30]=np.divide(raw.reshape(-1,3),f[:30],out=np.tile([0.,0.,1.],(30,1)),where=f[:30]!=0)
   return f,None,n,None,self.counts,self.starts
  def get_friction_data(self,**kw):
   self.counts[:]=0;self.starts[:]=-1
   values=np.full((128,3),np.nan)
   for i,j in np.ndindex(2,15):
    if np.linalg.norm(friction[i,j]):
     k=60+i*15+j;self.counts[i,j]=1;self.starts[i,j]=k;values[k]=friction[i,j]
   return values,None,self.counts,self.starts
 sim=types.SimpleNamespace(env=types.SimpleNamespace(unwrapped=types.SimpleNamespace(sim=types.SimpleNamespace(get_physics_dt=lambda:1/240))))
 try:
  o._support_contact_view=View();support=o.support_loads(sim)
  assert support[0]<.01 and abs(support[1]-.147)<1e-12,'duplicate static filters inflated support'
  raw[0,14]=[0,0,-4];raw[1,13]=[0,0,4];o.support_loads(sim)
  assert o.compressive_axial(*o.last_glass_vectors,np.array((0.,0.,1.)))==4,'normal pair indexing'
  friction[0,14]=[0,0,3];friction[1,13]=[0,0,-3];o.support_loads(sim)
  assert o.last_glass_contact==4 and o.compressive_axial(*o.last_glass_vectors,np.array((0.,0.,1.)))==1,'opposing friction falsely qualified1.962N pressure'
  assert np.allclose(o.last_glass_vectors.sum(axis=0),0),'unequal total reaction'
  assert np.allclose(o.last_glass_vectors,o.last_glass_normal_vectors+o.last_glass_friction_vectors)
  try:o._sum_pairs(np.zeros((5,3)),np.array([[2]]),np.array([[4]]),(1,1))
  except ValueError:pass
  else:raise AssertionError('truncated contact buffer accepted')
  try:o._sum_pairs(np.full((5,3),np.nan),np.array([[1]]),np.array([[0]]),(1,1))
  except ValueError:pass
  else:raise AssertionError('unknown active friction accepted')
  signed=o._sum_pairs(np.array([[-4.],[0.],[0.]]),np.array([[1]]),np.array([[0]]),(1,1),np.array([[0.,0.,1.],[0.,0.,0.],[0.,0.,0.]]))
  assert np.allclose(signed[0,0],[0,0,-4]),'reverse pair signed normal lost'
  zero=o._sum_pairs(np.zeros((3,1)),np.array([[1]]),np.array([[0]]),(1,1),np.zeros((3,3)))
  assert not zero.any(),'zero impulse invented load'
  print('detailed normal+friction contacts: PASS;shared count buffer,distinct anchor indices,opposing friction,unknown/truncated rejection;support deduplication')
 finally:
  for k,v in saved.items():setattr(o,k,v)
if __name__=='__main__':support_view_controls()


def pressure_reset_controls():
 # Prescribed inventory endpoint:old seating pressure never survives open-neck refill.
 w=m.fluid.Water();w.pressed=True;w.seal=True;w.attempt_overflow=.4;w.counterheld_overflow=.3;w.seal_max_depth=m.fluid.FULL_DEPTH
 w.update(.05,np.eye(3),np.zeros(3),np.zeros(3),.004,.8,0.,True,True)
 assert not w.pressed and not w.seal and w.attempt_overflow==0. and w.counterheld_overflow==0.,'old pressure/overflow survived open-neck refill'
 w.update(.05,np.eye(3),np.zeros(3),np.zeros(3),.004,.8,m.fluid.FULL_DEPTH,False,True,seal_contact=True)
 assert w.seal and not w.pressed,'wet contact restored pressure without native force witness'
 print('fresh pressure after open-neck refill: PASS (prescribed control)')
if __name__=='__main__':pressure_reset_controls()

def finger_input_controls():
 o=m.observation;saved=o.last_hold_total_vectors;z=np.array((0.,0.,1.))
 try:
  v=np.zeros((4,2,3));v[2:4,1,2]=1.;o.last_hold_total_vectors=v
  assert o.stopper_hand_press(z,True,False)==2.
  assert o.stopper_hand_press(z,False,True)==0.,'wrong hand supplied pressure'
  v[2:4,1,2]=-.14715/2
  assert o.stopper_hand_press(z,True,False)==0.,'passive weight support credited as downward input'
  v[2:4,1]=[10.,0.,0.]
  assert o.stopper_hand_press(z,True,False)==0.,'side grip magnitude credited as axial input'
  v[2,1,2]=np.nan
  try:o.stopper_hand_press(z,True,False)
  except ValueError:pass
  else:raise AssertionError('unknown finger input accepted')
  print('actual stopper-hand input controls: PASS;weight/side/wrong-hand/unknown rejection')
 finally:o.last_hold_total_vectors=saved
if __name__=='__main__':finger_input_controls()
