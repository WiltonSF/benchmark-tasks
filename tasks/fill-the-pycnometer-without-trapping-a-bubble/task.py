"""A native-size50mL spherical density bottle, its perforated stopper, a clear water bath and spill tray.
The bottle is partly filled and a visible air pocket adheres under its shoulder. Water is ordinary tap water.
"""
import json,math,os,pathlib,sys
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts,lots,tilt,observation,fluid
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint,FINGER_OPEN
SCENE='station';TOP=TABLE_TOP[SCENE];SCORE_OVER='end';SUCCESS_HOLD_S=.1
parts.CONTACT_OFFSET=.00025;parts.REST_OFFSET=0.
GLASS=(.68,.85,.88);GREY=(.54,.58,.62);BLUE=(.16,.43,.78)
BPOS=(.18,.03,TOP+.028);SPOS=(.27,-.15,TOP+.006)
# Lathed spherical body with flat bottom, shoulder and an open tapered ground neck.
shapes=[parts.cylinder(.012,.003,(0,0,.0015),GLASS)]
for k in range(11):
 z=.004+k*.0044;rz=math.sqrt(max(0.,.0254**2-(z+.0022-.0254)**2));inner=max(.008,rz-.0018)
 for a in np.arange(32)*2*math.pi/32:
  # Native spherical form, overlapping small vertical ribs preserve real rounded contacts.
  shapes.append(parts.cylinder(.0017,.006,(max(.0103,rz-.0017)*math.cos(a),max(.0103,rz-.0017)*math.sin(a),z+.0022),GLASS))
for z in np.linspace(.049,.06785,7):
 inner=.0083+(z-.049)/(.06785-.049)*.0022
 for a in np.arange(24)*2*math.pi/24:
  shapes.append(parts.cylinder(.0012,.004,(float((inner+.0012)*math.cos(a)),float((inner+.0012)*math.sin(a)),float(z)),GLASS))
BOTTLE=parts.Part('density_bottle',BPOS,shapes=shapes,mass=.055,friction=(.55,.45))
# Perforated stopper has a true open central bore; no collider spans its capillary.
st=[]
for z,r,h in [(.003,.0067,.006),(.009,.0073,.006),(.015,.0079,.006),(.021,.0085,.006),(.036,.010,.0315)]:
 for a in np.arange(16)*2*math.pi/16:st.append(parts.cylinder(.0012 if z<.025 else .0017,h,(r*math.cos(a),r*math.sin(a),z),GLASS))
for z,h,maxr in [(.012,.024,.006),(.036,.0315,.008)]:
 for r in np.arange(.0018,maxr,.0022):
  for a in np.arange(max(8,int(2*math.pi*r/.0018)))*2*math.pi/max(8,int(2*math.pi*r/.0018)):
   st.append(parts.cylinder(.0013,h,(float(r*math.cos(a)),float(r*math.sin(a)),z),GLASS))
# Capillary in the native stopper is represented by inventory flow; the handle exterior remains full size.
STOPPER=parts.Part('perforated_stopper',SPOS,shapes=st,mass=.020,friction=(.55,.45))
# Open rack supports the native stopper by its shoulder, with no added handle or retention.
stand=[parts.box((.065,.100,.004),(0,0,.002),GREY)]
for sign in (-1,1):stand.append(parts.box((.012,.050,.020),(sign*.023,0,.014),GREY))
RACK=parts.Part('stopper_rack',(.27,-.15,TOP),shapes=stand)
SHELF=parts.Part('fill_start_shelf',(.18,.03,TOP),shapes=[parts.box((.075,.15,.024),(0,0,.016),GREY)])
# Water bath:460x340x90mm, placed inside a500x400 spill tray. Bottom datum4mm.
bath=[parts.box((.46,.34,.004),(0,0,.002),(.77,.84,.88))]
for y in (-.171,.171):bath.append(parts.box((.464,.002,.090),(0,y,.049),GLASS))
for x in (-.231,.231):bath.append(parts.box((.002,.34,.090),(x,0,.049),GLASS))
# Open four-post rest stabilizes final placement; the robot lifts clear for all air-clearing motions.
for x,y,size in [(.0244,0,(.006,.012,.010)),(-.0244,0,(.006,.012,.010)),(0,.0244,(.012,.006,.010)),(0,-.0244,(.012,.006,.010))]:
 bath.append(parts.box(size,(.025+x,-.09+y,.009),GREY))
BATH=parts.Part('water_bath',(.29,.12,TOP),shapes=bath,mass=13.2,friction=(.9,.8))
# Bath is physically held by ordinary nonslip locating blocks, equal load reaction reaches this body.
blocks=[parts.box((.020,.03,.02),(x,y,.010),GREY) for x in (-.245,.245) for y in (-.13,.13)]
LOCATORS=parts.Part('bath_locators',(.29,.12,TOP),shapes=blocks)
PARK=parts.Part('inspection_pad',(.18,-.19,TOP),shapes=[parts.box((.11,.10,.004),(0,0,.002),(.87,.87,.78))])
REST_PAD=parts.Part('bottle_rest_pad',(.315,.03,TOP+.004),shapes=[parts.cylinder(.013,.002,(0,0,.001),(.25,.35,.38))],friction=(.8,.6),compliance=(20000.,40.))
OBJECTS=[BOTTLE,STOPPER,RACK,BATH,LOCATORS,PARK,SHELF,REST_PAD]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={BOTTLE.name:BPOS,STOPPER.name:SPOS}),lots.Lot('B',place={BOTTLE.name:(.18,.075,TOP+.028),STOPPER.name:(.27,-.18,TOP+.006)}))
DEMO_TOUCHES={'stopper_rack'}
GRID=.003
_grid=np.stack(np.meshgrid(np.arange(-.0255,.0256,GRID),np.arange(-.0255,.0256,GRID),np.arange(.0015,.06986,GRID),indexing='ij'),axis=-1).reshape(-1,3)
_outer=(np.linalg.norm(_grid-np.array((0,0,.0254)),axis=1)<=.0254)|((np.linalg.norm(_grid[:,:2],axis=1)<=.0117)&(_grid[:,2]>=.047))
OUTER_POINTS=_grid[_outer];OUTER_VOLUME=len(OUTER_POINTS)*GRID**3
T=fluid.Water();_time=0.;_stage=None;_bubble_op=None;_bubble_size=None;_prev=None;_flow_op=None;_flow_size=None;_flow_prev=0.;_dry_properties=None;_mass_at=None;_cg_local=None

def finite(s):
 try:return np.isfinite(np.r_[s.pos,s.quat_xyzw,s.lin_vel,s.ang_vel]).all() and abs(np.linalg.norm(s.quat_xyzw)-1)<.01
 except (AttributeError,ValueError,TypeError):return False

def stamp(objects):return tuple(np.concatenate([np.r_[objects[n].pos,objects[n].quat_xyzw,objects[n].lin_vel,objects[n].ang_vel] for n in (BOTTLE.name,STOPPER.name,BATH.name)]))

def geometry(b,s):
 R=parts._quat_to_matrix(b.quat_xyzw);Rs=parts._quat_to_matrix(s.quat_xyzw);local=R.T@(s.pos-b.pos)
 depth=.063-local[2];rad=float(np.linalg.norm(local[:2]));aligned=float(np.dot(R[:,2],Rs[:,2]))>=math.cos(math.radians(10))-1e-6
 return R,local,depth,rad,aligned

def immersion(b,bath,R):
 mouth=b.pos+R@np.array((0.,0.,.06985));waterline=float(bath.pos[2]+.080)
 inside=bool(abs(mouth[0]-bath.pos[0])<.230 and abs(mouth[1]-bath.pos[1])<.170)
 points=OUTER_POINTS@R.T+b.pos
 mask=(points[:,2]<=waterline)&(np.abs(points[:,0]-bath.pos[0])<.230)&(np.abs(points[:,1]-bath.pos[1])<.170)
 volume=float(np.count_nonzero(mask))*GRID**3
 center=points[mask].mean(0) if mask.any() else b.pos.copy()
 return mouth,waterline,inside,volume,center

def clearance(clouds,objects):
 if len(clouds)!=6 or any(p.ndim!=2 or p.shape[1]!=3 or not len(p) or not np.isfinite(p).all() for p in clouds):return False
 for spec in (BOTTLE,STOPPER):
  s=objects[spec.name];rot=parts._quat_to_matrix(s.quat_xyzw)
  # Conservative solid native envelope, not sparse surface vertices.
  for p in clouds:
   q=(p-s.pos)@rot
   if spec is BOTTLE:
    ball=np.linalg.norm(q-np.array((0,0,.0254)),axis=1)-.0254
    neck=np.maximum(np.linalg.norm(q[:,:2],axis=1)-.0113,np.maximum(.047-q[:,2],q[:,2]-.06985))
    if np.minimum(ball,neck).min()<=.008:return False
   else:
    dist=np.linalg.norm(np.maximum(np.abs(q-np.array((0,0,.0259)))-np.array((.012,.012,.0259)),0),axis=1)
    if dist.min()<=.008:return False
 return True

def liquid_mass(sim):
 """Contained water moves with glass; update mass,COM and inertia,not just its weight."""
 global _dry_properties,_mass_at,_cg_local
 import torch
 asset=sim.env.unwrapped.scene[BOTTLE.name];b=sim.states()[BOTTLE.name];R=parts._quat_to_matrix(b.quat_xyzw)
 if _dry_properties is None:
  _dry_properties=parts.mass_properties(BOTTLE.shapes,.055)
 if _mass_at is not None and abs(_mass_at-T.water)<1e-8:return
 dry_com,dry_inertia=_dry_properties;water_mass=T.water*fluid.RHO*1e-6;mass=.055+water_mass
 water_com=np.array((0.,0.,.0254));_cg_local=(.055*dry_com+water_mass*water_com)/mass
 def shift(v):return np.dot(v,v)*np.eye(3)-np.outer(v,v)
 inertia=dry_inertia+.055*shift(dry_com-_cg_local)+.4*water_mass*.023**2*np.eye(3)+water_mass*shift(water_com-_cg_local)
 if not np.isfinite(np.r_[mass,_cg_local,inertia.ravel()]).all() or mass<=0:raise ValueError('unknown water inertia')
 masses=torch.tensor([[mass]],dtype=torch.float32,device=asset.device)
 coms=torch.tensor(np.r_[_cg_local,0.,0.,0.,1.],dtype=torch.float32,device=asset.device).reshape(1,1,7)
 inertias=torch.tensor(inertia,dtype=torch.float32,device=asset.device).reshape(1,1,9)
 # The backend copies properties into pinned CPU buffers before PhysX consumes them.
 # Finish each device-to-host copy before that CPU read; no shared runtime files change.
 import warp as wp
 torch.cuda.synchronize(asset.device)
 original_copy=wp.copy
 def completed_copy(dest,src,*args,**kwargs):
  result=original_copy(dest,src,*args,**kwargs)
  if dest.device.is_cpu and src.device.is_cuda:wp.synchronize_device(src.device)
  return result
 wp.copy=completed_copy
 try:
  asset.set_masses_index(masses=masses)
  asset.set_coms_index(coms=coms)
  asset.set_inertias_index(inertias=inertias)
 finally:wp.copy=original_copy
 _mass_at=T.water

def wrench(sim,name,force,torque):
 import torch
 asset=sim.env.unwrapped.scene[name]
 origin=parts._np(sim.env.unwrapped.scene.env_origins[0]);com=parts._np(asset.data.body_com_pose_w)[0,0,:3]-origin
 state=sim.states()[name]
 if name==BOTTLE.name and _cg_local is not None:com=state.pos+parts._quat_to_matrix(state.quat_xyzw)@_cg_local
 torque=np.asarray(torque)-np.cross(com-state.pos,force)
 if not np.isfinite(np.r_[force,torque,com]).all():raise ValueError('unknown wrench/COM')
 asset.set_external_force_and_torque(torch.tensor(force,dtype=torch.float32,device=asset.device).reshape(1,1,3),torch.tensor(torque,dtype=torch.float32,device=asset.device).reshape(1,1,3),is_global=True)

@parts.on_reset
def reset(sim):
 global T,_time,_stage,_bubble_op,_bubble_size,_prev,_flow_op,_flow_size,_flow_prev,_mass_at
 T=fluid.Water();_time=0.;_prev=None;_flow_prev=0.;_mass_at=None;observation.reset()
 liquid_mass(sim)
 from isaaclab.app.settings_manager import get_settings_manager
 get_settings_manager().set_bool('/physics/disableContactProcessing',False)
 import omni.usd
 from pxr import UsdGeom,Gf
 _stage=omni.usd.get_context().get_stage();root=sim.env.unwrapped.scene.env_prim_paths[0]
 from pxr import PhysxSchema
 # Silicone splash sleeves give each real pad a finite contact footprint. Point
 # contacts otherwise have zero torsional friction and let a sphere roll in-place.
 # Estimated2mm patch radius (1.5mm minimum),not an attachment or pose constraint.
 from pxr import Usd,UsdPhysics
 from yam_arena.robot import asset_link_paths
 links=asset_link_paths();pad_shapes=0
 for arm in ('LeftArm','RightArm'):
  for tip in ('tip_left','tip_right'):
   for prim in Usd.PrimRange(_stage.GetPrimAtPath(root+'/'+arm+links[tip])):
    if prim.HasAPI(UsdPhysics.CollisionAPI):
     api=PhysxSchema.PhysxCollisionAPI.Apply(prim)
     api.CreateTorsionalPatchRadiusAttr(.002);api.CreateMinTorsionalPatchRadiusAttr(.0015);pad_shapes+=1
 if pad_shapes<4:raise ValueError('missing covered finger collision geometry')
 for name in (BOTTLE.name,STOPPER.name,BATH.name):
  api=PhysxSchema.PhysxRigidBodyAPI(_stage.GetPrimAtPath(root+'/'+name));api.CreateSolverPositionIterationCountAttr(64);api.CreateSolverVelocityIterationCountAttr(8)
 _bubble_op=[];_bubble_size=[]
 for i in range(12):
  prim=UsdGeom.Sphere.Define(_stage,root+'/air_pocket_visual_'+str(i));prim.CreateRadiusAttr((3*.1e-6/(4*math.pi))**(1/3));prim.CreateDisplayColorAttr([(1.,.96,.77)])
  xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder();_bubble_op.append(xf.AddTranslateOp());_bubble_size.append(xf.AddScaleOp())
 for name in (BOTTLE.name,STOPPER.name,BATH.name):
  for p in _stage.Traverse():
   if p.GetPath().pathString.startswith(root+'/'+name+'/') and p.IsA(UsdGeom.Gprim):
    UsdGeom.Gprim(p).CreateDisplayOpacityAttr([.30 if name!=BATH.name else .16])
 from pxr import UsdShade
 for name in (BOTTLE.name,STOPPER.name,BATH.name):
  for p in _stage.Traverse():
   path=p.GetPath().pathString
   if path.startswith(root+'/'+name+'/looks/') and p.IsA(UsdShade.Shader):
    shader=UsdShade.Shader(p);inp=shader.GetInput('opacity')
    if inp:inp.Set(.28 if name!=BATH.name else .2)
 # Decorative water surface has no collider and no score flag.
 for i,(p,size) in enumerate([((.29,-.049,TOP+.080),(.456,.001,.001)),((.29,.289,TOP+.080),(.456,.001,.001)),((.061,.12,TOP+.080),(.001,.338,.001)),((.519,.12,TOP+.080),(.001,.338,.001))]):
  water=UsdGeom.Cube.Define(_stage,root+'/waterline_'+str(i));water.CreateSizeAttr(1.);water.CreateDisplayColorAttr([(.1,.4,.8)])
  xf=UsdGeom.Xformable(water);xf.ClearXformOpOrder();xf.AddTranslateOp().Set(Gf.Vec3d(*p));xf.AddScaleOp().Set(Gf.Vec3f(*size))
 # Hide the obsolete full-plane primitive if a process is reused.
 old=_stage.GetPrimAtPath(root+'/water_surface_visual')
 if old:UsdGeom.Imageable(old).MakeInvisible()
 import rendering
 rendering.glass(_stage,root)
 jet=UsdGeom.Cylinder.Define(_stage,root+'/capillary_overflow_visual');jet.CreateRadiusAttr(.001);jet.CreateHeightAttr(.009);jet.CreateDisplayColorAttr([(.12,.4,.9)])
 xf=UsdGeom.Xformable(jet);xf.ClearXformOpOrder();_flow_op=xf.AddTranslateOp();_flow_size=xf.AddScaleOp()


@parts.on_step
def step(sim):
 global _time,_prev,_flow_prev
 dt=float(sim.t)-_time;objects=sim.states()
 try:
  b=objects[BOTTLE.name];s=objects[STOPPER.name];bath=objects[BATH.name]
  if not all(finite(x) for x in (b,s,bath)) or not 0<dt<=.100001:raise ValueError('unknown body or clock')
  R,local,depth,rad,aligned=geometry(b,s);clouds,load=observation.hand_state(sim);nonfinger=observation.nonfinger_contact_load(sim)
  T.support=observation.support_loads(sim)
  mouth,waterline,mouth_in_bath,displaced,buoy_center=immersion(b,bath,R)
  open_neck=not (aligned and rad<.009 and depth>0.)
  submerged=displaced/OUTER_VOLUME
  # Neck clearance uses real stopper geometry; a hand across the aperture blocks flow.
  neck_clear=not any(np.linalg.norm(c-mouth,axis=1).min()<.006 for c in clouds)
  effective_depth=depth if aligned and rad<=.002+1e-6 and 0<depth<.027 else 0.
  cap_top=s.pos+parts._quat_to_matrix(s.quat_xyzw)@np.array((0,0,.05175))
  cap_open=not any(np.linalg.norm(c-cap_top,axis=1).min()<.003 for c in clouds)
  liquid_center=b.pos+R@np.array((0,0,.0254))
  relative_v=b.lin_vel+np.cross(b.ang_vel,liquid_center-b.pos)-bath.lin_vel-np.cross(bath.ang_vel,liquid_center-bath.pos)
  previous_out=T.out;previous_overflow=T.overflow;previous_capillary_out=T.capillary_out;prior_pressure=T.applied_pressure
  glass_contact=observation.last_glass_contact
  if not np.isfinite(glass_contact):raise ValueError('unknown glass pair contact')
  cap_wet=abs(cap_top[0]-bath.pos[0])<.230 and abs(cap_top[1]-bath.pos[1])<.170 and cap_top[2]<waterline
  bubble_local=T.update(dt,R,relative_v,b.ang_vel,waterline-mouth[2] if mouth_in_bath else -.1,submerged,effective_depth,open_neck,neck_clear,cap_open,displaced_volume=displaced,capillary_height=None if cap_wet else float(cap_top[2]-b.pos[2]),seal_contact=glass_contact>=.2 and prior_pressure>=.005)
  def caught(point):return -.035<=point[0]<=.615 and -.295<=point[1]<=.355
  overflow_delta=T.overflow-previous_overflow
  if overflow_delta>1e-9:
   hold=observation.counterhold_forces(sim)
   lr=bool(min(hold[0,0],hold[1,0])>=1.3 and min(hold[2,1],hold[3,1])>=.3)
   rl=bool(min(hold[2,0],hold[3,0])>=1.3 and min(hold[0,1],hold[1,1])>=.3)
   if (lr or rl) and prior_pressure>=.005:T.counterheld_overflow+=overflow_delta
  capillary_delta=T.capillary_out-previous_capillary_out
  if overflow_delta+capillary_delta>1e-9 and not caught(cap_top):T.zero=True
  if T.out-previous_out-overflow_delta-capillary_delta>1e-9 and not caught(mouth):T.zero=True
  if bubble_local is None:raise ValueError('water model failure')
  liquid_mass(sim)
  points=bubble_local@R.T+b.pos
  point=np.average(points,axis=0,weights=np.maximum(T.volumes,1e-9))
  liquid_center=b.pos+R@np.array((0,0,.0254))
  hydro_torque=np.cross(buoy_center-b.pos,T.hydro)+np.cross(liquid_center-b.pos,T.drag)
  # Rotational drag is passive, capped, and equal/opposite on bath.
  rot_drag=fluid.rotational_drag(b.ang_vel,dt)
  air_moment=np.zeros(3)
  for p,v in zip(points,T.volumes):
   air_sub=float(np.clip((waterline-p[2]+.00288)/.00576,0.,1.)) if mouth_in_bath else 0.
   air_moment+=np.cross(p-liquid_center,np.array((0.,0.,998*9.81*float(v)*1e-6*air_sub)))
  torque=hydro_torque+rot_drag+air_moment
  wrench(sim,BOTTLE.name,T.force,torque)
  reaction=-(T.hydro+T.drag)
  water_moment=np.zeros(3)
  wrench(sim,BATH.name,reaction,-(torque-water_moment)+np.cross(b.pos-bath.pos,reaction))
  # Pressure and viscous flow in the capillary oppose actual insertion, with reaction on the counterheld bottle.
  insertion_v=0. if _prev is None else max(0.,effective_depth-_prev)/dt
  pressure=min(.12,998*9.81*max(0.,waterline-mouth[2])*math.pi*.0077**2+20.*insertion_v) if effective_depth>0 else 0.
  press_force=pressure*R[:,2];contact=s.pos+parts._quat_to_matrix(s.quat_xyzw)@np.array((0.,0.,.012))
  wrench(sim,STOPPER.name,press_force,np.cross(contact-s.pos,press_force))
  if pressure:wrench(sim,BOTTLE.name,T.force-press_force,torque+np.cross(contact-b.pos,-press_force))
  T.applied_pressure=pressure
  _prev=effective_depth
  if load>40.01 or nonfinger>40.01:T.zero=True
  if b.low[2]<TOP-.005 or s.low[2]<TOP-.005 or np.linalg.norm(bath.pos-np.array(BATH.pos))>.005 or abs(float(bath.quat_xyzw[3]))<math.cos(math.radians(1)):T.zero=True
  # Loss outside the actual bath or inspection pad is permanent.
  in_bath=abs(b.pos[0]-bath.pos[0])<.20 and abs(b.pos[1]-bath.pos[1])<.14
  on_pad=np.linalg.norm(b.pos[:2]-np.array(PARK.pos[:2]))<.04
  if not in_bath and not on_pad and b.low[2]<TOP+.015:T.zero=True
  on_shelf=abs(b.pos[0]-SHELF.pos[0])<.0375 and abs(b.pos[1]-SHELF.pos[1])<.075
  near_rest=np.linalg.norm(b.pos[:2]-np.array((.315,.03)))<.018
  floor_error=abs(b.low[2]-(bath.pos[2]+.004));pad_error=abs(b.low[2]-(TOP+.006)) if near_rest else math.inf
  T.clear=clearance(clouds,objects) and nonfinger<=.05;T.down=bool((in_bath and min(floor_error,pad_error)<.002+1e-6) or (on_pad and abs(b.low[2]-(TOP+.004))<.002) or (on_shelf and abs(b.low[2]-(TOP+.028))<.002+1e-6))
  T.down=T.down and min(T.support)>=.01
  still=b.resting(.003,.03) and s.resting(.003,.03)
  T.held=T.held+dt if T.clear and T.down and still else 0.
  T.known=True;T.cert=stamp(objects)
  from pxr import Gf
  origin=parts._np(sim.env.unwrapped.scene.env_origins[0])
  for i in range(12):
   _bubble_op[i].Set(Gf.Vec3d(*(points[i]+origin)))
   scale=(max(float(T.volumes[i]),0.)/.1)**(1/3) if T.volumes[i]>0 else .0001;_bubble_size[i].Set(Gf.Vec3f(scale,scale,scale))
  flowing=T.overflow-_flow_prev>1e-7;_flow_prev=T.overflow
  _flow_op.Set(Gf.Vec3d(*(cap_top+origin+np.array((.003,0.,-.003)))))
  a=1. if flowing else .0001;_flow_size.Set(Gf.Vec3f(a,a,a))
 except (AttributeError,KeyError,ValueError,IndexError,TypeError,RuntimeError) as err:
  T.zero=True;T.known=False
  if os.environ.get('TRACE_OUT'):print('pycnometer model error:',str(err),flush=True)
 _time=float(sim.t)
 if os.environ.get('TRACE_OUT'):
  row=dict(t=sim.t,lot=lots.current().name,water=T.water,air=T.air,pinned=T.pinned,normal=T.normal.tolist(),bubbles=T.volumes.tolist(),bubble_paths=T.path.tolist(),overflow=T.overflow,external=T.external,force=T.force.tolist(),hydro=T.hydro.tolist(),drag=T.drag.tolist(),filled=T.filled,cleared=T.cleared,seated=T.seated,clear=T.clear,down=T.down,held=T.held,zero=T.zero,known=T.known,load=locals().get('load'),nonfinger_load=locals().get('nonfinger'),tip_loads=observation.last_tip_loads.tolist(),hold_forces=observation.last_hold_forces.tolist(),counterheld_overflow=T.counterheld_overflow,cap_open=locals().get('cap_open'),pressure=locals().get('pressure'),seal=T.seal,bypass=T.bypass_out,glass_contact=observation.last_glass_contact,glass_pair=observation.last_glass_pair.tolist(),support=T.support.tolist(),depth=locals().get('depth'),rad=locals().get('rad'),aligned=locals().get('aligned'),score=score(objects),bottle=objects[BOTTLE.name].pos.tolist(),stopper=objects[STOPPER.name].pos.tolist(),quat=objects[BOTTLE.name].quat_xyzw.tolist(),velocity=objects[BOTTLE.name].lin_vel.tolist(),angular=objects[BOTTLE.name].ang_vel.tolist(),rot_drag=locals().get('rot_drag',np.zeros(3)).tolist(),stopper_quat=objects[STOPPER.name].quat_xyzw.tolist())
  with pathlib.Path(os.environ['TRACE_OUT']).with_name('pycnometer.jsonl').open('a') as f:f.write(json.dumps(row,default=lambda v:v.item() if isinstance(v,np.generic) else v.tolist())+'\n')

def score(objects):
 try:
  if T.cert!=stamp(objects) or not all(finite(objects[n]) for n in (BOTTLE.name,STOPPER.name,BATH.name)):return 0.
  b=objects[BOTTLE.name];s=objects[STOPPER.name];R,local,depth,rad,aligned=geometry(b,s)
  # Exterior film inventory is explicitly excluded. Reference is same bottle+stopper with50mL interior water.
  mass_error=abs((T.water-fluid.CAPACITY)*.998)
  seat_ok=aligned and rad<=.002+1e-6 and fluid.FULL_DEPTH-.002-1e-6<=depth<=fluid.FULL_DEPTH+.002+1e-6 and R[2,2]>=math.cos(math.radians(10))-1e-6 and parts._quat_to_matrix(s.quat_xyzw)[2,2]>=math.cos(math.radians(10))-1e-6
  closed=aligned and rad<.009 and depth>0.
  contained_capacity=max(0.,fluid.OPEN_CAPACITY-fluid.PLUG_AREA*depth*1e6) if closed else fluid.CAPACITY
  return T.value(mass_error,seat_ok,b.resting(.003,.03) and s.resting(.003,.03),contained_capacity)
 except (ValueError,KeyError,AttributeError,TypeError):return 0.
def success(objects):return score(objects)==1.

def demo():
 # Open-loop camera-visible published positions only; no runtime state reads.
 bp=np.asarray(lots.current().pos(BOTTLE.name));sp=np.asarray(lots.current().pos(STOPPER.name));left=[];right=[]
 def L(t,p,f=FINGER_OPEN,label='move',**kw):
  angle=kw.pop('tilt',0.);heading=kw.pop('heading',0.)
  w=tilt.Waypoint(t,tuple(float(x) for x in p),f,label,**kw)
  relative=tilt.rotation(0.,angle,heading)@tilt.rotation(0.,0.,0.).T
  w.yaw=tilt._TiltedYaw(0.,relative@tilt.rotation(0.,50.,0.))
  left.append(w)
 def S(t,p,f=FINGER_OPEN,label='move',**kw):right.append(tilt.Waypoint(t,tuple(float(x) for x in p),f,label,yaw=math.pi/2,**kw))
 grip=bp+np.array((0,0,.022));hover=grip+np.array((0,0,.075));dunk=np.array((.315,.03,TOP+.006));wetgrip=dunk+np.array((0,0,.022))
 L(2,hover);L(2,grip);L(1.5,grip,-.022,label='grasp native spherical body');L(2,grip+np.array((0,0,.028)),-.022);L(3,wetgrip+np.array((0,0,.03)),-.022);L(3,wetgrip,-.022,label='immerse and fill')
 L(1.5,wetgrip+np.array((0,0,.018)),-.022,label='lift clear of open rest')
 for heading in (math.pi/4,-math.pi/4):
  L(3,wetgrip+np.array((0,0,.018)),-.022,label='rock adhering shoulder air',tilt=30,heading=heading)
  L(3,wetgrip+np.array((0,0,.018)),-.022,label='loaded wall migration',tilt=30,heading=heading)
  L(2,wetgrip+np.array((0,0,.018)),-.022,label='neck uppermost above rest')
 L(5,wetgrip+np.array((0,0,.018)),-.022,label='air rises to open neck');L(3,wetgrip,-.022,label='lower into open rest')
 L(22,wetgrip,-.022,label='counterhold bottle while stopper displaces water');L(2,wetgrip,FINGER_OPEN,label='release bottle');L(2,wetgrip+np.array((0,0,.09)),label='vertical clearance before withdrawal');L(2,np.array((.18,.23,TOP+.14)));L(15,np.array((.18,.23,TOP+.14)),label='released inspection');L(10,np.array((.18,.23,TOP+.14)))
 # Published-layout calibration from actual contact approach offsets;no state feedback.
 bx,by=(-.0005,.00075) if lots.current().name=='B' else (-.001,-.00075)
 release=-.013 if lots.current().name=='B' else -.015
 # B settles another1.1mm only after release in the predecessor trace.
 # Finish that travel under grip;keep absolute approach/clearance/park heights.
 press=.0035 if lots.current().name=='B' else 0.
 approach=.0225+press if lots.current().name=='B' else .025
 lift=.036+press
 sg=sp+np.array((0,0,.038));sh=sg+np.array((0,0,.09));seat=dunk+np.array((bx,by,.04985+.038-press))
 S(35,np.array((.18,-.25,TOP+.14)),label='wait clear of bottle');S(2,sh);S(2,sg);S(1.5,sg,-.008,label='grasp native stopper');S(2,sh,-.008);S(1,sh,-.008,label='clear rack')
 S(2,np.array((.315,-.065,TOP+.14)),-.008,label='clear bath rim');S(3,seat+np.array((0,0,approach)),-.008);S(4,seat,-.008,label='displace excess through capillary');S(2,seat,-.008)
 S(2,seat,release,label='release stopper with26-30mm jaw gap');S(3,seat+np.array((0,0,lift)),release);S(1,seat+np.array((0,0,lift)),0.,label='close empty fingers above the stopper');S(3,seat+np.array((0,-.14,lift)),0.,label='clear bottle before other hand withdraws');S(12,seat+np.array((0,-.14,lift)),0.,label='released still inspection')
 return dict(left=left,right=right)
