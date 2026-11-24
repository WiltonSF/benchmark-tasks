"""Native-size50mL density bottle after a first fill: one0.25mL shoulder bubble.
The open bottle is on the bath shelf; its perforated stopper stands on an open rack.
"""
import json,math,os,pathlib,sys
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts,lots,tilt,observation,fluid
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint,FINGER_OPEN
# Masses, bubble volume and seating pressure obtained by phone from a technician
# who works with this product (Nov 2026). Other parameters below are estimates.
START_BUBBLE_ML=.25
BOTTLE_DRY_MASS_KG=.040
STOPPER_MASS_KG=.015
SEATING_PRESS_N=.200*9.81
SEATING_PRESS_DEPTH_M=fluid.FULL_DEPTH-.0005
START_WATER_ML=fluid.OPEN_CAPACITY-START_BUBBLE_ML
# Unmeasured nominal contact-line resistance estimate:perimeter*sigma,using equivalent
# spherical radius as estimated contact-line radius. IAPWS2014 surface tension at20C.
WATER_TEMPERATURE_K=293.15
_tau=1.-WATER_TEMPERATURE_K/647.096
SURFACE_TENSION_N_M=.2358*_tau**1.256*(1.-.625*_tau)
BUBBLE_CONTACT_LINE_RADIUS_M=(3*START_BUBBLE_ML*1e-6/(4*math.pi))**(1/3)
BUBBLE_RETENTION_N=2*math.pi*BUBBLE_CONTACT_LINE_RADIUS_M*SURFACE_TENSION_N_M
BUBBLE_START_POLAR_DEG=30.
BUBBLE_MOBILITY_NS_M=.025
BUBBLE_VENT_RATE_ML_S=.6
WET_FRICTION=(.55,.45)
WATER_DENSITY_KG_M3=998.
OPEN_FILL_RATE_ML_S=10.
OPEN_DRAIN_RATE_ML_S=12.
CAPILLARY_RADIUS_M=.0007
CAPILLARY_LENGTH_M=.052
CAPILLARY_VISCOSITY_PA_S=.001
SEAL_CONTACT_N=.2 # Wet-contact proxy,rounded up from15g stopper weight0.147N;not final seating pressure.
BODY_PAD_CONTACT_N=1.3
STOPPER_PAD_CONTACT_N=.3
PAD_PATCH_RADIUS_M=.002
PAD_MIN_PATCH_RADIUS_M=.0015
# Ground-joint profile estimates, not measured PH0122B internal dimensions.
# Matching continuous tapers replace the nonmatching stepped rib profiles.
GROUND_NECK_Z0_M=.049
GROUND_NECK_Z1_M=.06985
GROUND_NECK_R0_M=.0083
GROUND_RADIUS_SLOPE=.0022/.01885
GROUND_WALL_M=.0012
STOPPER_REFERENCE_TIP_Z_M=.04985
STOPPER_GROUND_LENGTH_M=.0245
GROUND_MESH_SECTORS=64

# Score thresholds remain in Water.value(); these are physical-model estimates.
fluid.configure(RHO=WATER_DENSITY_KG_M3,START_WATER=START_WATER_ML,
 FILL_RATE=OPEN_FILL_RATE_ML_S,DRAIN_RATE=OPEN_DRAIN_RATE_ML_S,
 CAP_RADIUS=CAPILLARY_RADIUS_M,CAP_LENGTH=CAPILLARY_LENGTH_M,
 VISCOSITY=CAPILLARY_VISCOSITY_PA_S,DRY_MASS=BOTTLE_DRY_MASS_KG,
 START_BUBBLE=START_BUBBLE_ML,RETENTION_N=BUBBLE_RETENTION_N,
 START_POLAR=BUBBLE_START_POLAR_DEG,
 MOBILITY=BUBBLE_MOBILITY_NS_M,VENT_RATE=BUBBLE_VENT_RATE_ML_S)
SCENE='station';TOP=TABLE_TOP[SCENE];SCORE_OVER='end';SUCCESS_HOLD_S=.1
parts.CONTACT_OFFSET=.00025;parts.REST_OFFSET=0.
GLASS=(.68,.85,.88);GREY=(.54,.58,.62);BLUE=(.16,.43,.78)
BPOS=(.18,.03,TOP+.028)
# Stopper is staged upright on its open rack before the normal fill is corrected.
SPOS=(.27,-.15,TOP+.006)
# Lathed spherical body with flat bottom, shoulder and an open tapered ground neck.
shapes=[parts.cylinder(.012,.003,(0,0,.0015),GLASS)]
for k in range(11):
 z=.004+k*.0044;rz=math.sqrt(max(0.,.0254**2-(z+.0022-.0254)**2));inner=max(.008,rz-.0018)
 lower=z+.0022-.003;upper=min(z+.0022+.003,GROUND_NECK_Z0_M)
 if upper<=lower:continue
 rib_height=upper-lower;rib_center=(lower+upper)/2
 for a in np.arange(32)*2*math.pi/32:
  # Native spherical form, overlapping small vertical ribs preserve real rounded contacts.
  shapes.append(parts.cylinder(.0017,rib_height,(max(.0103,rz-.0017)*math.cos(a),max(.0103,rz-.0017)*math.sin(a),rib_center),GLASS))
neck_r1=GROUND_NECK_R0_M+GROUND_RADIUS_SLOPE*(GROUND_NECK_Z1_M-GROUND_NECK_Z0_M)
shapes+=parts.conical_shell(GROUND_NECK_Z0_M,GROUND_NECK_Z1_M,GROUND_NECK_R0_M,neck_r1,GROUND_NECK_R0_M+GROUND_WALL_M,neck_r1+GROUND_WALL_M,GLASS,sectors=GROUND_MESH_SECTORS)
BOTTLE=parts.Part('density_bottle',BPOS,shapes=shapes,mass=BOTTLE_DRY_MASS_KG,friction=WET_FRICTION)
# Perforated stopper has a true open central bore; no collider spans its capillary.
plug_r0=GROUND_NECK_R0_M+GROUND_RADIUS_SLOPE*(STOPPER_REFERENCE_TIP_Z_M-GROUND_NECK_Z0_M)
st=parts.conical_shell(0.,STOPPER_GROUND_LENGTH_M,CAPILLARY_RADIUS_M,CAPILLARY_RADIUS_M,plug_r0,plug_r0+GROUND_RADIUS_SLOPE*STOPPER_GROUND_LENGTH_M,GLASS,sectors=GROUND_MESH_SECTORS)
# Native-size handle and open capillary; no collider spans its bore.
for a in np.arange(32)*2*math.pi/32:st.append(parts.cylinder(.0017,.0315,(.010*math.cos(a),.010*math.sin(a),.036),GLASS))
for r in np.arange(CAPILLARY_RADIUS_M+.0013,.008,.0022):
 for a in np.arange(max(8,int(2*math.pi*r/.0018)))*2*math.pi/max(8,int(2*math.pi*r/.0018)):
  st.append(parts.cylinder(.0013,.0315,(float(r*math.cos(a)),float(r*math.sin(a)),.036),GLASS))
# Capillary in the native stopper is represented by inventory flow; the handle exterior remains full size.
STOPPER=parts.Part('perforated_stopper',SPOS,shapes=st,mass=STOPPER_MASS_KG,friction=WET_FRICTION)
# Open rack stages the native stopper.
stand=[parts.box((.065,.100,.004),(0,0,.002),GREY)]
# Four stock blocks guide only the lower stem during pickup;23mm clear square,
# top22mm below the native head bottom24.25mm. No latch or imposed pose.
for sign in (-1,1):
 stand.append(parts.box((.012,.050,.018),(sign*.0175,0,.013),GREY))
 stand.append(parts.box((.023,.012,.018),(0,sign*.0175,.013),GREY))
RACK=parts.Part('stopper_rack',(.27,-.15,TOP),shapes=stand)
SHELF=parts.Part('fill_start_shelf',(.18,.03,TOP),shapes=[parts.box((.075,.15,.024),(0,0,.016),GREY)])
# Water bath:460x340x90mm, within the650x650mm catch area. Bottom datum4mm.
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
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={BOTTLE.name:BPOS,STOPPER.name:SPOS}),lots.Lot('B',place={BOTTLE.name:(.18,.075,TOP+.028),STOPPER.name:SPOS}))
DEMO_TOUCHES={'stopper_rack'}
GRID=.003
_grid=np.stack(np.meshgrid(np.arange(-.0255,.0256,GRID),np.arange(-.0255,.0256,GRID),np.arange(.0015,.06986,GRID),indexing='ij'),axis=-1).reshape(-1,3)
_outer=(np.linalg.norm(_grid-np.array((0,0,.0254)),axis=1)<=.0254)|((np.linalg.norm(_grid[:,:2],axis=1)<=.0117)&(_grid[:,2]>=.047))
OUTER_POINTS=_grid[_outer];OUTER_VOLUME=len(OUTER_POINTS)*GRID**3
T=fluid.Water();_time=0.;_stage=None;_bubble_op=None;_bubble_size=None;_prev=None;_flow_op=None;_flow_size=None;_flow_prev=0.;_dry_properties=None;_mass_at=None;_cg_local=None;_poses=None

def finite(s):
 try:return np.isfinite(np.r_[s.pos,s.quat_xyzw,s.lin_vel,s.ang_vel]).all() and abs(np.linalg.norm(s.quat_xyzw)-1)<.01
 except (AttributeError,ValueError,TypeError):return False

def steady(previous,state,dt):
 # True when the pose change over dt is below3mm/s and0.03rad/s.
 dot=min(1.,abs(float(np.dot(previous[1],state.quat_xyzw))))
 return bool(np.linalg.norm(state.pos-previous[0])/dt<.003 and 2*math.acos(dot)/dt<.03)
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
  _dry_properties=parts.mass_properties(BOTTLE.shapes,BOTTLE_DRY_MASS_KG)
 if _mass_at is not None and abs(_mass_at-T.water)<1e-8:return
 dry_com,dry_inertia=_dry_properties;water_mass=T.water*fluid.RHO*1e-6;mass=BOTTLE_DRY_MASS_KG+water_mass
 water_com=np.array((0.,0.,.0254));_cg_local=(BOTTLE_DRY_MASS_KG*dry_com+water_mass*water_com)/mass
 def shift(v):return np.dot(v,v)*np.eye(3)-np.outer(v,v)
 inertia=dry_inertia+BOTTLE_DRY_MASS_KG*shift(dry_com-_cg_local)+.4*water_mass*.023**2*np.eye(3)+water_mass*shift(water_com-_cg_local)
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
 global T,_time,_stage,_bubble_op,_bubble_size,_prev,_flow_op,_flow_size,_flow_prev,_mass_at,_poses
 T=fluid.Water();_time=0.;_prev=None;_flow_prev=0.;_mass_at=None;_poses=None;observation.reset()
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
     api.CreateTorsionalPatchRadiusAttr(PAD_PATCH_RADIUS_M);api.CreateMinTorsionalPatchRadiusAttr(PAD_MIN_PATCH_RADIUS_M);pad_shapes+=1
 if pad_shapes<4:raise ValueError('missing covered finger collision geometry')
 # One lens-shaped visual shows the declared air under an engaged stopper; it has no collider or score role.
 prim=UsdGeom.Sphere.Define(_stage,root+'/trapped_air_visual');prim.CreateRadiusAttr(1.);prim.CreateDisplayColorAttr([(1.,.96,.77)])
 xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder();_bubble_op=xf.AddTranslateOp();_bubble_size=xf.AddScaleOp()
 # Initialize the first observation;the unit-radius visual must never hide the kit.
 initial=sim.states();b=initial[BOTTLE.name];R=parts._quat_to_matrix(b.quat_xyzw)
 origin=parts._np(sim.env.unwrapped.scene.env_origins[0])
 bubble_local=np.array((0.,0.,.0254))+.023*T.normal
 a=(max(T.air,0.)*1e-6*9/(4*math.pi))**(1/3) if T.air>1e-3 else 1e-5
 _bubble_op.Set(Gf.Vec3d(*(b.pos+R@bubble_local+origin)));_bubble_size.Set(Gf.Vec3f(a,a,a/3))
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
 rendering.glass(_stage,root,neck_profile=[(GROUND_NECK_Z0_M,GROUND_NECK_R0_M+GROUND_WALL_M),(GROUND_NECK_Z1_M,neck_r1+GROUND_WALL_M)],plug_profile=[(0.,plug_r0),(.02025,plug_r0+GROUND_RADIUS_SLOPE*.02025),(.02025,.0117),(.05175,.0117)])
 jet=UsdGeom.Cylinder.Define(_stage,root+'/capillary_overflow_visual');jet.CreateRadiusAttr(.001);jet.CreateHeightAttr(.009);jet.CreateDisplayColorAttr([(.12,.4,.9)])
 xf=UsdGeom.Xformable(jet);xf.ClearXformOpOrder();_flow_op=xf.AddTranslateOp();_flow_size=xf.AddScaleOp()
 s=initial[STOPPER.name];cap_top=s.pos+parts._quat_to_matrix(s.quat_xyzw)@np.array((0.,0.,.05175))
 _flow_op.Set(Gf.Vec3d(*(cap_top+origin+np.array((.003,0.,-.003)))));_flow_size.Set(Gf.Vec3f(.0001,.0001,.0001))


@parts.on_step
def step(sim):
 global _time,_prev,_flow_prev,_poses
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
  previous_out=T.out;previous_overflow=T.overflow;previous_capillary_out=T.capillary_out
  glass_contact=observation.last_glass_contact
  if not np.isfinite(glass_contact):raise ValueError('unknown glass pair contact')
  axial_press=observation.compressive_axial(*observation.last_glass_vectors,R[:,2])
  hold=observation.counterhold_forces(sim)
  lr=bool(min(hold[0,0],hold[1,0])>=BODY_PAD_CONTACT_N and min(hold[2,1],hold[3,1])>=STOPPER_PAD_CONTACT_N)
  rl=bool(min(hold[2,0],hold[3,0])>=BODY_PAD_CONTACT_N and min(hold[0,1],hold[1,1])>=STOPPER_PAD_CONTACT_N)
  hand_press=observation.stopper_hand_press(R[:,2],lr,rl)
  cap_wet=abs(cap_top[0]-bath.pos[0])<.230 and abs(cap_top[1]-bath.pos[1])<.170 and cap_top[2]<waterline
  bubble_local=T.update(dt,R,relative_v,b.ang_vel,waterline-mouth[2] if mouth_in_bath else -.1,submerged,effective_depth,open_neck,neck_clear,cap_open,displaced_volume=displaced,capillary_height=None if cap_wet else float(cap_top[2]-b.pos[2]),seal_contact=glass_contact>=SEAL_CONTACT_N)
  def caught(point):return -.035<=point[0]<=.615 and -.295<=point[1]<=.355
  overflow_delta=T.overflow-previous_overflow
  if overflow_delta>1e-9:
   if lr or rl:T.counterheld_overflow+=overflow_delta
  capillary_delta=T.capillary_out-previous_capillary_out
  if overflow_delta+capillary_delta>1e-9 and not caught(cap_top):T.zero=True
  if T.out-previous_out-overflow_delta-capillary_delta>1e-9 and not caught(mouth):T.zero=True
  if bubble_local is None:raise ValueError('water model failure')
  liquid_mass(sim)
  liquid_center=b.pos+R@np.array((0,0,.0254))
  hydro_torque=np.cross(buoy_center-b.pos,T.hydro)+np.cross(liquid_center-b.pos,T.drag)
  # Rotational drag is passive, capped, and equal/opposite on bath.
  rot_drag=fluid.rotational_drag(b.ang_vel,dt)
  torque=hydro_torque+rot_drag
  wrench(sim,BOTTLE.name,T.force,torque)
  reaction=-(T.hydro+T.drag)
  water_moment=np.zeros(3)
  wrench(sim,BATH.name,reaction,-(torque-water_moment)+np.cross(b.pos-bath.pos,reaction))
  # Phone figure is final seating pressure,not constant hydraulic drag.
  # Native contacts and friction do the work;the force view records equal/opposite
  # net normal+friction compressive axial force on both glass bodies,and actual
  # stopper-hand fingertip input independently. Weight/impact alone cannot qualify.
  if effective_depth>=SEATING_PRESS_DEPTH_M and glass_contact+1e-9>=SEATING_PRESS_N and axial_press+1e-9>=SEATING_PRESS_N and hand_press+1e-9>=SEATING_PRESS_N and (lr or rl):T.pressed=True
  wrench(sim,STOPPER.name,np.zeros(3),np.zeros(3))
  pressure=0.;T.applied_pressure=0.;_prev=effective_depth
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
  # Stillness is measured from actual pose change over the control step, as a
  # camera would. PhysX's reported velocity of a stopper wedged in the taper keeps
  # about0.03m/s while its pose stays within0.001mm,so it is not used here.
  # Same limits:3mm/s and0.03rad/s for bottle and stopper.
  T.still=_poses is not None and all(steady(_poses[n],objects[n],dt) for n in (BOTTLE.name,STOPPER.name))
  still=T.still
  T.held=T.held+dt if T.clear and T.down and still else 0.
  T.known=True;T.cert=stamp(objects);_poses={n:(objects[n].pos.copy(),objects[n].quat_xyzw.copy()) for n in (BOTTLE.name,STOPPER.name)}
  from pxr import Gf
  origin=parts._np(sim.env.unwrapped.scene.env_origins[0])
  # Visual follows the single shoulder inventory; free headspace uses the neck.
  a=(max(T.air,0.)*1e-6*9/(4*math.pi))**(1/3) if T.air>1e-3 else 1e-5
  _bubble_op.Set(Gf.Vec3d(*(b.pos+R@bubble_local+origin)));_bubble_size.Set(Gf.Vec3f(a,a,a/3))
  flowing=T.overflow-_flow_prev>1e-7;_flow_prev=T.overflow
  _flow_op.Set(Gf.Vec3d(*(cap_top+origin+np.array((.003,0.,-.003)))))
  a=1. if flowing else .0001;_flow_size.Set(Gf.Vec3f(a,a,a))
 except (AttributeError,KeyError,ValueError,IndexError,TypeError,RuntimeError) as err:
  T.zero=True;T.known=False;T.still=False;_poses=None
  if os.environ.get('TRACE_OUT'):print('pycnometer model error:',str(err),flush=True)
 _time=float(sim.t)
 if os.environ.get('TRACE_OUT'):
  row=dict(t=sim.t,lot=lots.current().name,water=T.water,air=T.air,overflow=T.overflow,external=T.external,force=T.force.tolist(),hydro=T.hydro.tolist(),drag=T.drag.tolist(),filled=T.filled,cleared=T.cleared,seated=T.seated,clear=T.clear,down=T.down,held=T.held,zero=T.zero,known=T.known,load=locals().get('load'),nonfinger_load=locals().get('nonfinger'),tip_loads=observation.last_tip_loads.tolist(),hold_forces=observation.last_hold_forces.tolist(),hold_normal_vectors=observation.last_hold_normal_vectors.tolist(),hold_friction_vectors=observation.last_hold_friction_vectors.tolist(),hold_total_vectors=observation.last_hold_total_vectors.tolist(),pad_closing_axes=observation.last_closing_axes.tolist(),stopper_hand_press_N=locals().get('hand_press'),counterheld_overflow=T.counterheld_overflow,attempt_overflow=T.attempt_overflow,bubble_pinned=T.pinned,bubble_normal=T.normal.tolist(),bubble_path_m=T.path,pocket_ml=T.pocket,cap_open=locals().get('cap_open'),pressure=locals().get('pressure'),seal=T.seal,bypass=T.bypass_out,glass_contact=observation.last_glass_contact,glass_pair=observation.last_glass_pair.tolist(),glass_vectors=observation.last_glass_vectors.tolist(),glass_normal_vectors=observation.last_glass_normal_vectors.tolist(),contact_format_counts=observation.contact_format_counts.copy(),glass_friction_vectors=observation.last_glass_friction_vectors.tolist(),seating_press_N=locals().get('axial_press'),pressed=T.pressed,support=T.support.tolist(),depth=locals().get('depth'),rad=locals().get('rad'),aligned=locals().get('aligned'),score=score(objects),bottle=objects[BOTTLE.name].pos.tolist(),stopper=objects[STOPPER.name].pos.tolist(),quat=objects[BOTTLE.name].quat_xyzw.tolist(),velocity=objects[BOTTLE.name].lin_vel.tolist(),angular=objects[BOTTLE.name].ang_vel.tolist(),rot_drag=locals().get('rot_drag',np.zeros(3)).tolist(),stopper_quat=objects[STOPPER.name].quat_xyzw.tolist(),stopper_velocity=objects[STOPPER.name].lin_vel.tolist(),stopper_angular=objects[STOPPER.name].ang_vel.tolist(),hands=[c.mean(0).tolist() for c in locals().get('clouds',[])])
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
  return T.value(mass_error,seat_ok,bool(getattr(T,'still',False)),contained_capacity,closed=closed)
 except (ValueError,KeyError,AttributeError,TypeError):return 0.
def success(objects):return score(objects)==1.

def demo():
 # Open-loop camera-visible published positions only; no runtime state reads.
 bp=np.asarray(lots.current().pos(BOTTLE.name));sp=np.asarray(lots.current().pos(STOPPER.name))-np.array((0,0,.004));left=[];right=[]
 def L(t,p,f=FINGER_OPEN,label='move',**kw):
  angle=kw.pop('tilt',0.);heading=kw.pop('heading',0.)
  w=tilt.Waypoint(t,tuple(float(x) for x in p),f,label,**kw)
  relative=tilt.rotation(0.,angle,heading)@tilt.rotation(0.,0.,0.).T
  w.yaw=tilt._TiltedYaw(0.,relative@tilt.rotation(0.,50.,0.))
  left.append(w)
 def S(t,p,f=FINGER_OPEN,label='move',**kw):right.append(tilt.Waypoint(t,tuple(float(x) for x in p),f,label,yaw=kw.pop('yaw',math.pi/2),**kw))
 grip=bp+np.array((0,0,.022));hover=grip+np.array((0,0,.075));dunk=np.array((.315,.03,TOP+.006));wetgrip=dunk+np.array((0,0,.022))
 L(2,hover);L(2,grip);L(1.5,grip,-.022,label='grasp native spherical body');L(2,grip+np.array((0,0,.028)),-.022);L(3,wetgrip+np.array((0,0,.03)),-.022);L(3,wetgrip,-.022,label='immerse and fill')
 L(1.5,wetgrip+np.array((0,0,.018)),-.022,label='lift clear of open rest')
 for heading in (math.pi/4,-math.pi/4):
  L(3,wetgrip+np.array((0,0,.018)),-.022,label='rock adhering shoulder air',tilt=40,heading=heading)
  L(3,wetgrip+np.array((0,0,.018)),-.022,label='loaded wall migration',tilt=40,heading=heading)
  L(2,wetgrip+np.array((0,0,.018)),-.022,label='neck uppermost above rest')
 L(5,wetgrip+np.array((0,0,.018)),-.022,label='air rises to open neck');L(3,wetgrip,-.022,label='lower into open rest')
 L(44,wetgrip,-.022,label='counterhold through capillary overflow and final native seating press')
 # After the other hand is fully clear, place the closed assembly on the existing
 # rigid bath floor. Lift above the shelf before any lateral transfer.
 floorgrip=np.array((.180,.160,TOP+.004+.022))
 L(2,wetgrip+np.array((0,0,.035)),-.022,label='lift closed assembly clear of shelf')
 L(3,floorgrip+np.array((0,0,.035)),-.022,label='carry upright closed assembly above bath floor')
 L(3,floorgrip,-.022,label='lower assembly onto rigid bath floor')
 L(1,floorgrip,-.022,label='settle on native floor contact')
 L(2,floorgrip,FINGER_OPEN,label='release bottle on bath floor')
 L(2,floorgrip+np.array((0,0,.09)),label='vertical clearance before withdrawal')
 L(2,np.array((.18,.23,TOP+.14)))
 L(18,np.array((.18,.23,TOP+.14)),label='released inspection')
 # Published-layout calibration from actual contact approach offsets;no state feedback.
 bx,by=(.0031,-.0007) if lots.current().name=='B' else (.0000,.0003)
 release=-.013 if lots.current().name=='B' else -.015
 # Keep absolute approach and clearance heights while applying servo preload.
 press=.0055 if lots.current().name=='B' else .005
 approach=.025+press # Keep the absolute alignment height fixed while pressing deeper.
 initial_seat_yaw=math.pi/2+math.radians(5.)
 seat_yaw=math.pi/2+math.radians(10.)
 lift=.038+press # Extra2mm final press below first seat;keep absolute retreat height.
 sg=sp+np.array((0,0,.038));sh=sg+np.array((0,0,.09));seat=dunk+np.array((bx,by,.04985+.038-press))
 S(34,np.array((.18,-.25,TOP+.14)),label='wait clear while bottle is rocked');S(2,sh);S(1,sh,-.016,label='pre-close32mm above guided native head');S(2,sg,-.016);S(1.5,sg,-.008,label='grasp native stopper');S(2,sh,-.008);S(1,sh,-.008,label='clear rack')
 S(2,np.array((.315,-.065,TOP+.14)),-.008,label='clear bath rim');S(3,seat+np.array((0,0,approach)),-.011,label='gentle22mm head pinch for native self-centering');S(4,seat,-.011,label='lower with first5degrees of gentle twist',yaw=initial_seat_yaw)
 seat=seat-np.array((0.,0.,.002))
 S(2,seat,-.011,label='final native seating press',yaw=seat_yaw);S(2,seat,-.011,yaw=seat_yaw)
 # Keep the joint engaged: one insertion, followed by a higher pressure grip.
 # Free gravity settling is native contact motion, not a forced pose.
 clearance=seat+np.array((0.,0.,lift));high=seat+np.array((0.,0.,.014))
 S(2,seat,-.016,label='loosen for upper-head regrasp',yaw=seat_yaw)
 S(2,high,-.016,label='raise fingers above the bottle neck',yaw=seat_yaw)
 S(1,high,-.013,label='pre-close26mm around upper head',yaw=seat_yaw)
 S(1,high,-.009,label='firm18mm upper-head pinch',yaw=seat_yaw)
 pressure_yaw=seat_yaw+math.radians(10.)
 S(4,high-np.array((0.,0.,.024)),-.009,label='apply native final seating pressure with slight twist',yaw=pressure_yaw)
 S(1,high-np.array((0.,0.,.024)),-.009,label='hold actual native pressure',yaw=pressure_yaw)
 seat=high-np.array((0.,0.,.002));lift=float(clearance[2]-seat[2])
 S(2,seat,-.009,label='unload press before opening at upper head',yaw=pressure_yaw)
 S(2,seat,release,label='release stopper with26-30mm jaw gap',yaw=pressure_yaw)
 if lots.current().name=='B':
  # Shift only after a vertical withdrawal above the head. Corrected loaded
  # aim remains;the original empty-hand point stays within the reachable set.
  mid=clearance.copy();mid[:2]=high[:2];mid[2]-=.006
  S(2,mid,release,label='vertical clearance above native head',yaw=seat_yaw)
  clearance=clearance.copy();clearance[:2]=(.3153,.0303)
  S(1,clearance,release,label='reachable empty-hand clearance',yaw=seat_yaw)
 else:
  # Preserve the reachable empty-hand point after the loaded approach calibration.
  clearance=clearance.copy();clearance[:2]=(.3153,.0303)
  S(3,clearance,release,yaw=seat_yaw)
 S(1,clearance,0.,label='close empty fingers above the stopper',yaw=seat_yaw)
 park=clearance+np.array((0.,-.14,0.))
 S(3,park,0.,label='clear bottle before other hand withdraws',yaw=seat_yaw)
 S(10 if lots.current().name=='B' else 12,park,0.,label='released still inspection',yaw=seat_yaw)
 return dict(left=left,right=right)
