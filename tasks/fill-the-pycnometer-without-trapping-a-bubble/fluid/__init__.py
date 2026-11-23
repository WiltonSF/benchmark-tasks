"""Declared water and air accounting, not CFD. SI forces, mL inventories."""
import math
import numpy as np
RHO=998.; G=9.81; CAPACITY=50.; PLUG_AREA=math.pi*.0077**2; FULL_DEPTH=.01315; OPEN_CAPACITY=CAPACITY+PLUG_AREA*FULL_DEPTH*1e6
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
  self.water=44.;self.volumes=np.full(12,.1);self.head=OPEN_CAPACITY-self.water-self.air;self.out=0.;self.overflow=0.;self.external=0.
  alpha=math.radians(67.);phi=np.linspace(math.pi/2,3*math.pi/2,12)
  self.normals=np.array([(math.sin(alpha)*math.cos(a),math.sin(alpha)*math.sin(a),math.cos(alpha)) for a in phi]);self.pins=np.ones(12,dtype=bool)
  self.path=np.zeros(12);self.force=np.zeros(3);self.hydro=np.zeros(3);self.drag=np.zeros(3)
  self.pin_force=.00097;self.zero=False;self.filled=False;self.cleared=False;self.seated=False;self.loaded_steps=0;self.held=0.;self.known=False;self.clear=False;self.down=False;self.support=np.zeros(2);self.cert=();self.prev_depth=0.;self.initial=44.;self.bath_taken=0.;self.cap_blocked=False
  self.counterheld_overflow=0.;self.capillary_out=0.;self.bypass_out=0.;self.seal=False;self.seal_max_depth=0.;self.applied_pressure=0.
 @property
 def air(self):return float(self.volumes.sum())
 @air.setter
 def air(self,value):self.volumes=np.full(12,float(value)/12.)
 @property
 def pinned(self):return bool(np.any(self.pins & (self.volumes>1e-9)))
 @pinned.setter
 def pinned(self,value):self.pins[:]=bool(value)
 @property
 def normal(self):return np.average(self.normals,axis=0,weights=np.maximum(self.volumes,1e-9))
 def update(self,dt,R,velocity,omega,mouth_depth,submerged,seat_depth,open_neck,neck_clear,cap_open=True,displaced_volume=None,capillary_height=None,seal_contact=False):
  values=np.r_[dt,R.ravel(),velocity,omega,mouth_depth,submerged,seat_depth,self.water,self.volumes,self.head,self.out,self.overflow,self.capillary_out,self.bypass_out,self.counterheld_overflow,self.applied_pressure,self.seal_max_depth,self.prev_depth,self.external,self.force,self.hydro,self.drag,self.normals.ravel(),self.path,self.pin_force,self.initial,self.bath_taken]
  if not np.isfinite(values).all() or not 0<dt<=.100001:self.zero=True;return
  up=R.T@np.array((0.,0.,1.));wet=mouth_depth>.001;self.cap_blocked=not cap_open
  # Wet-ground-joint seal is a declared contact proxy,not a watertight collision mesh.
  # Native0.2N glass press plus prior applied resistance establishes it. Withdrawal
  # or loss of aligned neck engagement breaks it; no pose crossing can establish it.
  if open_neck or seat_depth<=0 or seat_depth<self.seal_max_depth-.0002:self.seal=False;self.seal_max_depth=0.
  if seal_contact and seat_depth>0 and not open_neck:self.seal=True
  if self.seal:self.seal_max_depth=max(self.seal_max_depth,seat_depth)
  # Twelve0.1mL pockets occupy a nonoverlapping upper shoulder arc. Each has its own trajectory and inventory.
  for i in range(12):
   if self.volumes[i]<=0:continue
   normal=self.normals[i];tangent=up-normal*np.dot(up,normal)
   ft=RHO*G*self.volumes[i]*1e-6*float(np.linalg.norm(tangent))
   # Estimated normalized retention over the admitted0.09-0.11mL range.
   # Keeping the same retention/buoyancy ratio avoids admitting smaller pockets
   # whose maximum buoyancy could never exceed an absolute nominal threshold.
   if ft>self.pin_force*self.volumes[i]/.1:self.pins[i]=False
   if not self.pins[i]:
    speed=min(.05,ft/.025);old=normal.copy()
    normal+=tangent*speed*dt/.023/max(np.linalg.norm(tangent),1e-12);normal/=np.linalg.norm(normal)
    self.path[i]+=.023*float(np.linalg.norm(normal-old))
   center=np.array((0.,0.,.0254))+.023*normal
   radius=(3*self.volumes[i]*1e-6/(4*math.pi))**(1/3)
   at_neck=np.linalg.norm(center[:2])+radius<.0083 and center[2]>.046
   if at_neck and not self.pins[i] and open_neck and neck_clear and wet:
    loss=min(float(self.volumes[i]),.6*dt);self.volumes[i]-=loss;self.water+=loss;self.bath_taken+=loss
  bubble_local=np.array((0.,0.,.0254))+.023*self.normals
  if wet and open_neck and neck_clear and (.06985-.0254)*up[2]>.024:
   flow=min(max(0.,OPEN_CAPACITY-self.air-self.water),dt*10.*math.sqrt(min(mouth_depth,.05)/.01))
   self.water+=flow;self.bath_taken+=flow
  # A nonimmersed mouth below the internal upper water level drains; inverted dumping cannot clear air for credit.
  if not wet and self.water>0 and (open_neck or not self.seal) and neck_clear:
   retained=retained_capacity_ml(R)
   loss=min(self.water,dt*12.,max(0.,self.water+self.air-retained))
   self.water-=loss;self.out+=loss
  # A dry open capillary also drains an inverted seated assembly. Its actual outlet
  # defines the gravity plane; Poiseuille flow is an estimate of two-phase leakage.
  # This is loss, never insertion-overflow credit.
  if self.seal and seat_depth>0 and cap_open and capillary_height is not None:
   if not math.isfinite(capillary_height):self.zero=True;return
   heights=_INNER@up
   retained=max(0.,OPEN_CAPACITY*float(np.count_nonzero(heights<=capillary_height))/len(_INNER)-PLUG_AREA*seat_depth*1e6)
   head=max(0.,float(heights.max())-capillary_height)
   rate=math.pi*.0007**4*RHO*G*head/(8*.001*.052)*1e6
   loss=min(self.water,rate*dt,max(0.,self.water+self.air-retained))
   self.water-=loss;self.out+=loss;self.capillary_out+=loss
  # Native stopper swept volume displaces surplus through its capillary, never from a seated flag.
  delta=max(0.,seat_depth-max(0.,self.prev_depth));self.prev_depth=seat_depth
  displaced=delta*PLUG_AREA*1e6
  available=OPEN_CAPACITY-PLUG_AREA*max(0.,seat_depth)*1e6
  surplus=max(0.,self.water+self.air-available)
  if seat_depth>0 and surplus>1e-6 and not cap_open:
   # Incompressible penetration is outside this model; never credit an impossible blocked insertion.
   self.zero=True
  if cap_open and seat_depth>0 and surplus>0.:
   # Actual swept volume drives insertion flow; retained excess can also relax after unblocking.
   # Estimated0.6mL/s pressure relaxation,not additional robot travel or new water creation.
   flow=min(surplus,max(displaced,.6*dt))
   self.water-=flow;self.out+=flow;self.external+=flow
   if self.seal:self.overflow+=flow
   else:self.bypass_out+=flow
  self.filled |= self.water>=45.-1e-6
  self.cleared |= self.filled and self.air<=.2+1e-6
  self.seated=bool(self.seal and seat_depth>=FULL_DEPTH-.002-1e-6 and self.air<=.2+1e-6 and self.cleared and self.overflow>=.15 and cap_open)
  # Hydrostatic load includes flooded glass and trapped air displacement, plus immersed contained-water displacement.
  # Water's own weight is included separately on glass, avoiding spurious weight-free handling.
  vol=(.055/2230.+(self.water+self.air)*1e-6)*np.clip(submerged,0.,1.) if displaced_volume is None else max(0.,displaced_volume)
  target_h=np.array((0.,0.,RHO*G*vol));v=np.asarray(velocity)
  target_d=-.5*RHO*1.0*.0020*np.clip(submerged,0.,1.)*np.linalg.norm(v)*v
  alpha=1-math.exp(-dt/.12);self.hydro+=(target_h-self.hydro)*alpha;self.drag+=(target_d-self.drag)*alpha
  for f,limit in ((self.hydro,1.2),(self.drag,.6)):
   norm=np.linalg.norm(f)
   if norm>limit:f*=limit/norm
  if float(np.dot(self.drag,v))>0:self.drag[:]=0.
  drag_norm=float(np.linalg.norm(self.drag));impulse_bound=.20*.055*float(np.linalg.norm(v))/dt
  if drag_norm>impulse_bound:self.drag*=impulse_bound/max(drag_norm,1e-12)
  self.force=self.hydro+self.drag
  self.loaded_steps+=1
  if self.water<0 or self.air<0 or abs(self.initial+self.bath_taken-self.water-self.out)>1e-6:self.zero=True
  return bubble_local
 def value(self,mass_error,seat_ok,still,contained_capacity=CAPACITY):
  if not np.isfinite(np.r_[self.water,self.air,self.out,self.overflow,self.counterheld_overflow,self.external,self.capillary_out,self.bypass_out,self.applied_pressure,self.seal_max_depth,self.prev_depth,self.loaded_steps,self.initial,self.bath_taken,self.head,self.held,mass_error,contained_capacity,self.support]).all():return 0.
  if np.asarray(self.support).shape!=(2,) or min(self.support)<.01:return 0.
  if self.zero or not self.known or not self.clear or not self.down or not still or self.held<1.-1e-6 or self.loaded_steps<1:return 0.
  if not self.filled or self.water<45.-1e-6:return 0.
  residual_air=max(self.air,max(0.,CAPACITY-self.water),max(0.,contained_capacity-self.water))
  if residual_air>1.+1e-6:return .25
  if residual_air>.2+1e-6 or not self.cleared:return .5
  if not self.seal or not self.seated or not seat_ok or self.overflow<.15 or self.counterheld_overflow<.15-1e-6 or mass_error>.2+1e-6:return .75
  return 1.
