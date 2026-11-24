"""Declared water and air accounting, not CFD. SI forces, mL inventories."""
import math
import numpy as np
G=9.81; CAPACITY=50.; START_DEPTH=0.; PLUG_AREA=math.pi*.0077**2; FULL_DEPTH=.01315; OPEN_CAPACITY=CAPACITY+PLUG_AREA*FULL_DEPTH*1e6
def configure(**estimates):
 """Task-level physical estimates are supplied once, before any Water instance."""
 globals().update(estimates)
# Deterministic inner-volume samples for gravity-retained open-bottle capacity.
_axis=np.arange(-.024,.0241,.003)
_inner_grid=np.stack(np.meshgrid(_axis,_axis,np.arange(.003,.06986,.003),indexing='ij'),axis=-1).reshape(-1,3)
_neck_r=.0083+np.clip((_inner_grid[:,2]-.049)/.01885,0.,1.)*.0022
_inner_mask=(np.linalg.norm(_inner_grid-np.array((0,0,.0254)),axis=1)<=.023)|((np.linalg.norm(_inner_grid[:,:2],axis=1)<=_neck_r)&(_inner_grid[:,2]>=.047))
_INNER=_inner_grid[_inner_mask]
def retained_capacity_ml(R):
 up=R.T@np.array((0.,0.,1.))
 lip=.06985*up[2]-.0105*float(np.linalg.norm(up[:2]))
 return OPEN_CAPACITY*float(np.count_nonzero(_INNER@up<=lip))/len(_INNER)
def rotational_drag(omega,dt):
 """Passive20Hz impulse bound,using conservative1e-5kgm² minimum glass inertia."""
 omega=np.asarray(omega,float);speed=float(np.linalg.norm(omega))
 if not np.isfinite(np.r_[omega,dt]).all() or dt<=0:raise ValueError('unknown angular drag state')
 if speed<1e-12:return np.zeros(3)
 magnitude=min(5e-6*speed**2,.015,.20*1e-5*speed/dt)
 return -omega*(magnitude/speed)
class Water:
 def __init__(self):
  # A single naturally occurring first-fill shoulder bubble; no per-pocket placement.
  self.pocket=START_BUBBLE;self.pinned=True;self.normal=np.array((-math.sin(math.radians(START_POLAR)),0.,math.cos(math.radians(START_POLAR))))
  self.path=0.;self.water=START_WATER;self.air=self.pocket;self.out=0.;self.overflow=0.;self.external=0.
  self.force=np.zeros(3);self.hydro=np.zeros(3);self.drag=np.zeros(3)
  self.zero=False;self.filled=False;self.cleared=False;self.seated=False;self.loaded_steps=0;self.held=0.;self.known=False;self.clear=False;self.down=False;self.support=np.zeros(2);self.cert=();self.prev_depth=START_DEPTH;self.initial=START_WATER;self.bath_taken=0.;self.cap_blocked=False
  self.pressed=False;self.counterheld_overflow=0.;self.attempt_overflow=0.;self.capillary_out=0.;self.bypass_out=0.;self.seal=False;self.seal_max_depth=0.;self.applied_pressure=0.
 def update(self,dt,R,velocity,omega,mouth_depth,submerged,seat_depth,open_neck,neck_clear,cap_open=True,displaced_volume=None,capillary_height=None,seal_contact=False):
  values=np.r_[dt,R.ravel(),velocity,omega,mouth_depth,submerged,seat_depth,self.water,self.air,self.pocket,self.normal,self.path,self.out,self.overflow,self.capillary_out,self.bypass_out,self.counterheld_overflow,self.applied_pressure,self.seal_max_depth,self.prev_depth,self.external,self.force,self.hydro,self.drag,self.initial,self.bath_taken]
  if not np.isfinite(values).all() or not 0<dt<=.100001:self.zero=True;return
  up=R.T@np.array((0.,0.,1.));wet=mouth_depth>.001;self.cap_blocked=not cap_open
  # A genuine open-neck withdrawal/refill starts a new seating attempt.
  # Its final pressure must be witnessed again;prior seating cannot qualify it.
  if open_neck or seat_depth<=0:
   self.pressed=False;self.counterheld_overflow=0.;self.attempt_overflow=0.
  # Wet-ground-joint seal is a declared contact proxy,not a watertight collision mesh.
  # Task-supplied wet-contact threshold establishes it. Final seating pressure is independently witnessed. Withdrawal
  # or loss of aligned neck engagement breaks it; no pose crossing can establish it.
  if open_neck or seat_depth<=0 or seat_depth<self.seal_max_depth-.0002:self.seal=False;self.seal_max_depth=0.
  if seal_contact and seat_depth>0 and not open_neck:self.seal=True
  if self.seal:self.seal_max_depth=max(self.seal_max_depth,seat_depth)
  # Buoyant wall migration against an explicitly unmeasured capillary-retention
  # proxy. One normal describes a broad shoulder sector, not twelve precision sites.
  tangent=up-self.normal*np.dot(up,self.normal)
  tangent_size=float(np.linalg.norm(tangent))
  buoy=RHO*G*self.pocket*1e-6
  if buoy*tangent_size>RETENTION_N*(max(self.pocket,0.)/START_BUBBLE)**(1/3):self.pinned=False
  if not self.pinned and self.pocket>0 and tangent_size>1e-9:
   old=self.normal.copy();speed=min(.05,buoy*tangent_size/MOBILITY)
   self.normal+=tangent/tangent_size*speed*dt/.023;self.normal/=np.linalg.norm(self.normal)
   self.path+=.023*float(np.linalg.norm(self.normal-old))
  center=np.array((0.,0.,.0254))+.023*self.normal
  radius=(3*max(self.pocket,0.)*1e-6/(4*math.pi))**(1/3)
  # A drained pocket fully above the remaining water joins the free headspace;
  # capillary adhesion cannot retain a gas bubble on an otherwise dry wall.
  levels=_INNER@up
  fraction=float(np.clip(self.water/OPEN_CAPACITY,0.,1.))
  surface=float(np.quantile(levels,fraction)) if fraction>0 else float(levels.min())-.001
  exposed=float(center@up)-radius>surface
  reaches_neck=not self.pinned and np.linalg.norm(center[:2])+radius<.0083 and center[2]>.046
  if (reaches_neck or exposed) and open_neck and neck_clear:
   loss=min(self.pocket,VENT_RATE*dt);self.pocket-=loss
   # Venting alone creates no water. Bath top-up replaces the vacated volume.
  if wet and open_neck and neck_clear and (.06985-.0254)*up[2]>.024:
   flow=min(max(0.,OPEN_CAPACITY-self.pocket-self.water),dt*FILL_RATE*math.sqrt(min(mouth_depth,.05)/.01))
   self.water+=flow;self.bath_taken+=flow
  # A nonimmersed mouth below the internal upper water level drains; inverted dumping cannot clear air for credit.
  if not wet and self.water>0 and (open_neck or not self.seal) and neck_clear:
   retained=retained_capacity_ml(R)
   loss=min(self.water,dt*DRAIN_RATE,max(0.,self.water+self.pocket-retained))
   self.water-=loss;self.out+=loss
  # A dry open capillary also drains an inverted seated assembly. Its actual outlet
  # defines the gravity plane; Poiseuille flow is an estimate of two-phase leakage.
  # This is loss, never insertion-overflow credit.
  if self.seal and seat_depth>0 and cap_open and capillary_height is not None:
   if not math.isfinite(capillary_height):self.zero=True;return
   heights=_INNER@up
   retained=max(0.,OPEN_CAPACITY*float(np.count_nonzero(heights<=capillary_height))/len(_INNER)-PLUG_AREA*seat_depth*1e6)
   head=max(0.,float(heights.max())-capillary_height)
   rate=math.pi*CAP_RADIUS**4*RHO*G*head/(8*VISCOSITY*CAP_LENGTH)*1e6
   loss=min(self.water,rate*dt,max(0.,self.water+self.pocket-retained))
   self.water-=loss;self.out+=loss;self.capillary_out+=loss
  # Native stopper swept volume displaces surplus through its capillary, never from a seated flag.
  delta=max(0.,seat_depth-max(0.,self.prev_depth));self.prev_depth=seat_depth
  displaced=delta*PLUG_AREA*1e6
  available=OPEN_CAPACITY-PLUG_AREA*max(0.,seat_depth)*1e6
  surplus=max(0.,self.water+self.pocket-available)
  if seat_depth>0 and surplus>1e-6 and not cap_open:
   # Incompressible penetration is outside this model; never credit an impossible blocked insertion.
   self.zero=True
  if cap_open and seat_depth>0 and surplus>0.:
   # Actual swept volume drives insertion flow; retained excess can also relax after unblocking.
   # Estimated0.6mL/s pressure relaxation,not additional robot travel or new water creation.
   flow=min(surplus,max(displaced,.6*dt))
   self.water-=flow;self.out+=flow;self.external+=flow
   if self.seal:self.overflow+=flow;self.attempt_overflow+=flow
   else:self.bypass_out+=flow
  # Trapped air exists only under an engaged stopper: its contained volume not filled by water.
  # Air vents freely through an open neck; an engaged plug exchanges none through the capillary.
  self.air=max(self.pocket,available-self.water) if (not open_neck and seat_depth>0) else self.pocket
  self.filled |= self.water>=45.-1e-6
  self.cleared |= self.filled and self.air<=.2+1e-6
  self.seated=bool(self.seal and seat_depth>=FULL_DEPTH-.002-1e-6 and self.air<=.2+1e-6 and self.cleared and self.attempt_overflow>=.15 and cap_open)
  # Hydrostatic load includes flooded glass and trapped air displacement, plus immersed contained-water displacement.
  # Water's own weight is included separately on glass, avoiding spurious weight-free handling.
  vol=(DRY_MASS/2230.+(self.water+self.air)*1e-6)*np.clip(submerged,0.,1.) if displaced_volume is None else max(0.,displaced_volume)
  target_h=np.array((0.,0.,RHO*G*vol));v=np.asarray(velocity)
  target_d=-.5*RHO*1.0*.0020*np.clip(submerged,0.,1.)*np.linalg.norm(v)*v
  alpha=1-math.exp(-dt/.12);self.hydro+=(target_h-self.hydro)*alpha;self.drag+=(target_d-self.drag)*alpha
  for f,limit in ((self.hydro,1.2),(self.drag,.6)):
   norm=np.linalg.norm(f)
   if norm>limit:f*=limit/norm
  if float(np.dot(self.drag,v))>0:self.drag[:]=0.
  drag_norm=float(np.linalg.norm(self.drag));impulse_bound=.20*DRY_MASS*float(np.linalg.norm(v))/dt
  if drag_norm>impulse_bound:self.drag*=impulse_bound/max(drag_norm,1e-12)
  self.force=self.hydro+self.drag
  self.loaded_steps+=1
  if self.water<0 or self.air<0 or abs(self.initial+self.bath_taken-self.water-self.out)>1e-6:self.zero=True
  return center if self.pocket>1e-6 else np.array((0.,0.,.0484))
 def value(self,mass_error,seat_ok,still,contained_capacity=CAPACITY,closed=True):
  if not np.isfinite(np.r_[self.water,self.air,self.pocket,self.normal,self.path,self.out,self.overflow,self.attempt_overflow,self.counterheld_overflow,self.external,self.capillary_out,self.bypass_out,self.applied_pressure,self.seal_max_depth,self.prev_depth,self.loaded_steps,self.initial,self.bath_taken,self.held,mass_error,contained_capacity,self.support]).all():return 0.
  if np.asarray(self.support).shape!=(2,) or min(self.support)<.01:return 0.
  if self.zero or not self.known or not self.clear or not self.down or not still or self.held<1.-1e-6 or self.loaded_steps<1:return 0.
  if not self.filled or not self.cleared or self.water<45.-1e-6:return 0.
  residual_air=max(self.air,max(0.,CAPACITY-self.water),max(0.,contained_capacity-self.water))
  if not closed or residual_air>1.+1e-6:return .25
  if residual_air>.2+1e-6 or not self.seal:return .5
  if not self.pressed or not self.seal or not self.seated or not seat_ok or self.attempt_overflow<.15 or self.counterheld_overflow<.15-1e-6 or mass_error>.2+1e-6:return .75
  return 1.
