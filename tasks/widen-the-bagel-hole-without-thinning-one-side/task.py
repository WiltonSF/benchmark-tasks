"""A raw bagel ring rests on a greased aluminum sheet on a complete kitchen counter. An original bench scraper lies beside it."""
import json,math,os,pathlib,sys
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts,lots,model
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint,FINGER_OPEN
SCENE='station';TOP=TABLE_TOP[SCENE];SCORE_OVER='end';SUCCESS_HOLD_S=.05
PAN_POS=(.260,0.,TOP+.001)
SURFACE=TOP+.002
C=np.array((.230,.045,SURFACE+model.B+.0002))
N=model.N
DOUGH=(.88,.81,.66)
# Complete pan, outer455x328x25mm, usable434x307mm. Bottom is not a support jig.
PAN=parts.Part('half_sheet',pos=PAN_POS,mass=.75,shapes=(
 parts.box((.455,.328,.002),color=(.68,.70,.73)),
 parts.box((.455,.0105,.024),(0,.15875,.0125),(.70,.72,.74)),
 parts.box((.455,.0105,.024),(0,-.15875,.0125),(.70,.72,.74)),
 parts.box((.0105,.307,.024),(.22225,0,.0125),(.70,.72,.74)),
 parts.box((.0105,.307,.024),(-.22225,0,.0125),(.70,.72,.74))),friction=(.5,.4),contact_offset=.0003)
# Each convex wedge is one16th of an ordinary102g ring,6.375g. Native inner/outer
# surfaces are reconstructed from an elliptical tube. No centre pin or hidden support.
def wedge(i):
 angle=model.ANGLES[i];points=[]
 for u in (angle-math.pi/N,angle,angle+math.pi/N):
  for v in np.arange(12)*math.tau/12:
   points.append(((model.R+model.A*math.cos(v))*math.cos(u)-model.P0[i,0],
                  (model.R+model.A*math.cos(v))*math.sin(u)-model.P0[i,1],model.B*math.sin(v)))
 faces=[]
 for u in range(2):
  for v in range(12):
   n=(v+1)%12;faces.extend([(12*u+v,12*(u+1)+v,12*(u+1)+n),(12*u+v,12*(u+1)+n,12*u+n)])
 faces.extend([tuple(range(11,-1,-1)),tuple(range(24,36))])
 return parts.hull(points,faces,DOUGH)
SECTORS=[parts.Part(f'dough_{i:02d}',pos=tuple(map(float,C+model.P0[i])),mass=model.M/N,
 shapes=(wedge(i),),friction=(.55,.4),contact_offset=.0003,report=('base',)) for i in range(N)]
TOOL=parts.Part('bench_scraper',pos=(.400,-.300,TOP+.01905),mass=.247,shapes=(
 parts.box((.1524,.1016,.0015),(0,0,-.0175),(.70,.72,.73)),
 parts.cylinder(.01905,.1524,(0,-.03175,0),(.12,.12,.13),axis='x')),friction=(.5,.4),contact_offset=.0003)
# Kitchen back splash, actual counter surface is the full station table.
BACK=parts.Part('counter_back',pos=(.72,0,TOP+.09),shapes=(parts.box((.025,1.1,.18),color=(.80,.82,.83)),))
OBJECTS=[PAN,*SECTORS,TOOL,BACK]
# LotB: ring translated90mm across pan; complete pan remains long-axis left/right.
# Unlike the proposal, rotating a pan beneath a150mm-offset ring is not used.
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={PAN.name:PAN.pos}),
 lots.Lot('B',place={PAN.name:PAN.pos,**{s.name:(s.pos[0],s.pos[1]-.090,s.pos[2]) for s in SECTORS},TOOL.name:(.400,.300,TOP+.01905)}))
DEMO_TOUCHES=set()

# Native PhysX six-axis cohesive bonds, excluded from articulation. Their soft
# position drives exert reaction forces; no free body's pose is written in step().
_native_author=parts._author
_bond_paths=[]
_mesh_points={}
def author(part,root_path,translation,orientation):
 root=_native_author(part,root_path,translation,orientation)
 if part.name==SECTORS[-1].name:
  from pxr import UsdPhysics,PhysxSchema,Sdf,Gf
  from isaaclab.sim.utils import get_current_stage
  stage=get_current_stage();prefix=root_path.rsplit('/',1)[0]
  _bond_paths.clear()
  for i in range(N):
   path=root_path+'/cohesion/bond'+str(i);_bond_paths.append(path)
   j=UsdPhysics.Joint.Define(stage,path)
   j.CreateBody0Rel().SetTargets([Sdf.Path(prefix+'/'+SECTORS[i].name)])
   j.CreateBody1Rel().SetTargets([Sdf.Path(prefix+'/'+SECTORS[(i+1)%N].name)])
   delta=model.P0[(i+1)%N]-model.P0[i]
   j.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float,delta/2)))
   j.CreateLocalPos1Attr().Set(Gf.Vec3f(*map(float,-delta/2)))
   j.CreateLocalRot0Attr().Set(Gf.Quatf(1.));j.CreateLocalRot1Attr().Set(Gf.Quatf(1.))
   j.CreateExcludeFromArticulationAttr(True);j.CreateCollisionEnabledAttr(False)
   for axis in ('transX','transY','transZ'):
    d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),axis);d.CreateTypeAttr('force')
    d.CreateStiffnessAttr(model.K);d.CreateDampingAttr(model.C);d.CreateMaxForceAttr(10.);d.CreateTargetPositionAttr(0.)
   for axis in ('rotX','rotY','rotZ'):
    d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),axis);d.CreateTypeAttr('force')
    d.CreateStiffnessAttr(.008/57.2958);d.CreateDampingAttr(.001/57.2958);d.CreateMaxForceAttr(.1);d.CreateTargetPositionAttr(0.)
 return root
parts._author=author

_record=model.Record();_joints=[];_cloud_cache=None;_contact=None
_food_load=np.zeros(N);_food_force=np.zeros((N,3))
_current_sim=None;_physics_handle=None;_body_view=None;_body_index={};_env_origin=np.zeros(3);_guard_previous=None;_guard_edge=None;_guard_p=None;_guard_boxes=None

def centers(o):return np.array([o[s.name].pos for s in SECTORS])
def stamp(o):return tuple(float(v) for s in OBJECTS for v in np.r_[o[s.name].pos,o[s.name].quat_xyzw,o[s.name].lin_vel,o[s.name].ang_vel])

def hands(sim):
 global _cloud_cache,_contact,_food_load,_food_force
 from yam_arena import analysis
 if _cloud_cache is None:
  from scipy.spatial import ConvexHull
  _cloud_cache={}
  for name in ('gripper','tip_left','tip_right'):
   cloud=[]
   for h in analysis.link_hulls().get(name,[]):
    for tri in ConvexHull(h.vertices).simplices:
     a,b,c=h.vertices[tri];n=max(1,math.ceil(max(np.linalg.norm(b-a),np.linalg.norm(c-a),np.linalg.norm(c-b))/.003))
     cloud.extend(a+(b-a)*i/n+(c-a)*j/n for i in range(n+1) for j in range(n+1-i))
   cloud=np.asarray(cloud);_,idx=np.unique(np.floor(cloud/.002).astype(int),axis=0,return_index=True)
   _cloud_cache[name]=cloud[idx]
 base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);clouds=[]
 for arm in ('left_robot','right_robot'):
  asset=base.scene[arm];poses=parts._np(asset.data.body_link_pose_w)[0]
  for name,cloud in _cloud_cache.items():
   p=poses[asset.body_names.index(name)]
   clouds.append(cloud@parts._quat_to_matrix(p[3:]).T+p[:3]-origin)
 if _contact is None:
  from isaaclab_physx.physics import PhysxManager
  from yam_arena.robot import asset_link_paths
  links=asset_link_paths();envpath=base.scene.env_prim_paths[0]
  paths=[envpath+'/'+arm+links[name] for arm in ('LeftArm','RightArm') for name in ('tip_left','tip_right')]
  _contact=PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths,filter_patterns=[[parts._PATHS[spec.name] for spec in SECTORS] for _ in paths],max_contact_data_count=1024)
 raw=_contact.get_contact_force_matrix(dt=base.sim.get_physics_dt());raw=raw.torch if hasattr(raw,'torch') else raw
 force=raw.detach().cpu().numpy() if hasattr(raw,'detach') else np.asarray(raw.numpy())
 if force.shape!=(4,N,3) or not np.isfinite(force).all() or len(clouds)!=6:raise ValueError('unknown filtered food-contact state')
 _food_load=np.linalg.norm(force,axis=2).sum(axis=0)
 _food_force=force.sum(axis=0)
 return clouds,float(np.linalg.norm(force,axis=2).max())

def hand_distance(clouds,o):
 # Sample all faces; conservative5mm plus4mm sampling margin. All ring sectors,
 # scraper blade and handle participate so a held supporting scraper cannot pass.
 cells=authored_cells(o)
 low=np.vstack([cells.min(1),o[TOOL.name].low]);high=np.vstack([cells.max(1),o[TOOL.name].high])
 nearest=10.
 for cloud in clouds:
  if cloud.size==0 or not np.isfinite(cloud).all():raise ValueError('unknown hand faces')
  for a,b in zip(low,high):
   if np.linalg.norm(np.maximum(np.maximum(a-cloud.max(0),cloud.min(0)-b),0))>.05:continue
   nearest=min(nearest,float(np.linalg.norm(np.maximum(np.maximum(a-cloud,cloud-b),0),axis=1).min()))
 return nearest

def deform_colliders(p,o):
 # Volume-conserving material surfaces follow actual sector centres. Each convex
 # hull is recooked from its current cross section; there is no render-only food.
 from pxr import UsdGeom,Gf
 from isaaclab.sim.utils import get_current_stage
 stage=get_current_stage();cells=model.cell_points(p,_record.rest,_record.flatten)
 for i,spec in enumerate(SECTORS):
  rotation=parts._quat_to_matrix(o[spec.name].quat_xyzw)
  points=(cells[i]-p[i])@rotation
  # hull points are authored in the local rigid-body frame.
  mesh=UsdGeom.Mesh(stage.GetPrimAtPath(parts._PATHS[spec.name]+'/s0'))
  points=np.asarray(points)
  # Commit every shared interface together; no per-cell geometry cache.
  from scipy.spatial import ConvexHull
  hull=ConvexHull(points)
  # Render exactly the convex collision surface, including its inward chords.
  faces=[]
  for tri in hull.simplices:
   tri=list(map(int,tri));aa,bb,cc=points[tri]
   if np.dot(np.cross(bb-aa,cc-aa),(aa+bb+cc)/3-points.mean(0))<0:tri[1],tri[2]=tri[2],tri[1]
   faces.append(tri)
  mesh.GetFaceVertexCountsAttr().Set([3]*len(faces))
  mesh.GetFaceVertexIndicesAttr().Set([v for tri in faces for v in tri])
  mesh.CreateDoubleSidedAttr(True)
  mesh.GetPointsAttr().Set([Gf.Vec3f(*map(float,q)) for q in points])
  mesh.GetExtentAttr().Set([Gf.Vec3f(*map(float,np.min(points,axis=0))),Gf.Vec3f(*map(float,np.max(points,axis=0)))])
  _mesh_points[spec.name]=points.copy()


@parts.on_reset
def reset(sim):
 global _record,_joints,_contact
 _record=model.Record();_contact=None;_mesh_points.clear()
 from pxr import UsdPhysics,Gf
 from isaaclab.sim.utils import get_current_stage
 from isaaclab.app.settings_manager import get_settings_manager
 get_settings_manager().set_bool('/physics/disableContactProcessing',False)
 stage=get_current_stage();_joints=[UsdPhysics.Joint(stage.GetPrimAtPath(path)) for path in _bond_paths]
 for i,j in enumerate(_joints):
  delta=model.P0[(i+1)%N]-model.P0[i]
  j.GetLocalPos0Attr().Set(Gf.Vec3f(*map(float,delta/2)));j.GetLocalPos1Attr().Set(Gf.Vec3f(*map(float,-delta/2)))
  j.GetJointEnabledAttr().Set(True)
  _record.anchors.append((delta/2,-delta/2))

def before_reset(sim):
 # Restore native source geometry and cohesive rest anchors BEFORE reset settle.
 # Otherwise a second lot would settle on the preceding trial's deformed mesh.
 from pxr import UsdGeom,Gf
 from isaaclab.sim.utils import get_current_stage
 stage=get_current_stage()
 reset(sim)
 for spec in SECTORS:
  mesh=UsdGeom.Mesh(stage.GetPrimAtPath(parts._PATHS[spec.name]+'/s0'))
  points=np.asarray(spec.shapes[0].points)
  mesh.GetPointsAttr().Set([Gf.Vec3f(*map(float,q)) for q in points])
  mesh.GetExtentAttr().Set([Gf.Vec3f(*map(float,points.min(0))),Gf.Vec3f(*map(float,points.max(0)))])
parts._ON_BEFORE_RESET.append(before_reset)

def edge_points(st):
 rot=parts._quat_to_matrix(st.quat_xyzw)
 return np.c_[np.linspace(-.0762,.0762,62),np.full(62,.0508),np.full(62,-.0175)]@rot.T+st.pos

def cutting_edge(o,p,before=None,prior=None):
 # Interpolate both edge and food motion at<=2.5mm spacing. An edge embedded
 # through>=5mm of the core is a declared cut, even if food moves onto the edge.
 now=edge_points(o[TOOL.name])
 if before is None:before=_record.previous_edge
 if before is None:before=now
 if prior is None:prior=_record.previous if _record.previous is not None else p
 n=max(1,math.ceil((np.linalg.norm(now-before,axis=1).max()+np.linalg.norm(p-prior,axis=1).max())/.0025))
 for f in np.linspace(0.,1.,n+1):
  pp=prior*(1-f)+p*f;edge=before*(1-f)+now*f
  a,b=model.sections(pp,_record.rest,_record.flatten)
  tangent,normal,vertical=model.material_frame(pp)
  delta=edge[:,None,:]-pp[None,:,:]
  along=np.abs(np.sum(delta*tangent[None,:,:],axis=2))
  radial=np.sum(delta*normal[None,:,:],axis=2);height=delta@vertical
  core=np.any((along<model.L0*.65)&((radial/a[None,:])**2+(height/b[None,:])**2<.64),axis=1)
  if any(np.all(core[j:j+3]) for j in range(len(core)-2)):return True
 return False

def authored_cells(o):
 return np.array([np.asarray(_mesh_points.get(spec.name,spec.shapes[0].points))@parts._quat_to_matrix(o[spec.name].quat_xyzw).T+o[spec.name].pos for spec in SECTORS])

def measure(o):
 pan=o[PAN.name];pr=parts._quat_to_matrix(pan.quat_xyzw)
 p=(centers(o)-pan.pos)@pr
 cells=(authored_cells(o)-pan.pos)@pr
 return model.geometry(p,_record.rest,_record.flatten,cells=cells)

def support(o,cells):
 pan=o[PAN.name];pr=parts._quat_to_matrix(pan.quat_xyzw)
 local=(cells-pan.pos)@pr
 return bool(np.max(abs(local[:,:,0]))<=.217 and np.max(abs(local[:,:,1]))<=.1535 and np.all(abs(local[:,:,2].min(1)-.001)<=.003))

def scraper_clear(o,cells):
 # Whole-object AABBs are conservative for a turned original scraper. They
 # include every face rather than only isolated vertices or the sharp edge.
 st=o[TOOL.name];lo=cells.min(1);hi=cells.max(1)
 return bool(np.all(np.linalg.norm(np.maximum(np.maximum(lo-st.high,st.low-hi),0.),axis=1)>.005))

def native_array(value):
 value=value.torch if hasattr(value,'torch') else value
 if hasattr(value,'detach'):return value.detach().cpu().numpy().astype(float)
 if hasattr(value,'numpy'):return np.asarray(value.numpy(),float)
 return np.asarray(value,float)

def native_guard(dt):
 global _guard_previous,_guard_edge,_guard_p,_guard_boxes
 if _current_sim is None or not _record.valid_reset:return
 try:
  from yam_arena import analysis
  base=_current_sim.env.unwrapped;origin=_env_origin
  native=native_array(_body_view.get_transforms())
  if native.shape!=(N+2,7) or not np.isfinite(native).all():raise ValueError('unknown native body transforms')
  poses={spec.name:(native[_body_index[spec.name],:3]-origin,native[_body_index[spec.name],3:]) for spec in [*SECTORS,PAN,TOOL]}
  p=np.array([poses[spec.name][0] for spec in SECTORS])
  cells=np.array([np.asarray(_mesh_points[spec.name])@parts._quat_to_matrix(poses[spec.name][1]).T+poses[spec.name][0] for spec in SECTORS])
  lengths=np.linalg.norm(np.roll(p,-1,axis=0)-p,axis=1)
  tear=bool(np.any(lengths>1.8*_record.rest) or np.any(lengths<.25*model.L0))
  tool=TOOL.state(pos=poses[TOOL.name][0],quat_xyzw=poses[TOOL.name][1])
  tear|=cutting_edge({TOOL.name:tool},p,before=_guard_edge,prior=_guard_p)
  _guard_edge=edge_points(tool);_guard_p=p.copy()
  off=bool(cells[:,:,2].min()<TOP-.020 or cells[:,:,0].min()<-.20 or cells[:,:,0].max()>.90 or abs(cells[:,:,1]).max()>.65)
  _record.sub_tear|=tear;_record.invalid|=tear or off
  if _guard_boxes is None:
   _guard_boxes={}
   for name in ('gripper','tip_left','tip_right'):
    v=np.concatenate([h.vertices for h in analysis.link_hulls()[name]])
    _guard_boxes[name]=np.array(np.meshgrid(*zip(v.min(0),v.max(0)))).T.reshape(-1,3)
  lo=np.vstack([cells.min(1),tool.low]);hi=np.vstack([cells.max(1),tool.high])
  for arm in ('left_robot','right_robot'):
   robot=base.scene[arm];links=native_array(robot.root_view.get_link_transforms())[0]
   for name,box in _guard_boxes.items():
    pose=links[robot.body_names.index(name)]
    world=box@parts._quat_to_matrix(pose[3:]).T+pose[:3]-origin
    gap=np.maximum(np.maximum(lo-world.max(0),world.min(0)-hi),0.)
    if np.any(np.linalg.norm(gap,axis=1)<=.009):_record.sub_clear=False
  if not scraper_clear({TOOL.name:tool},cells):_record.sub_clear=False
  if _guard_previous is not None:
   for spec in (PAN,TOOL):
    oldp,oldq=_guard_previous[spec.name];pos,q=poses[spec.name]
    angle=2*math.acos(float(np.clip(abs(np.dot(q,oldq))/(np.linalg.norm(q)*np.linalg.norm(oldq)),0.,1.)))
    if np.linalg.norm(pos-oldp)>.005*dt+1e-7 or angle>.05*dt+1e-5:_record.sub_clear=False
  _guard_previous=poses;_record.substeps+=1
 except Exception as e:
  _record.invalid=True;_record.sub_clear=False
  if not _record.error:_record.error=str(e);print('[dough] physics guard fail closed',type(e).__name__,str(e),flush=True)

def validate_reset(sim):
 global _current_sim,_physics_handle,_guard_previous,_guard_edge,_guard_p,_body_view,_body_index,_env_origin
 _current_sim=sim
 try:
  from isaaclab_physx.physics import PhysxManager,IsaacEvents
  o=sim.states();p=centers(o)
  _env_origin=parts._np(sim.env.unwrapped.scene.env_origins[0])
  paths=[parts._PATHS[spec.name] for spec in [*SECTORS,PAN,TOOL]]
  _body_view=PhysxManager.get_physics_sim_view().create_rigid_body_view(paths)
  indices={str(path):i for i,path in enumerate(_body_view.prim_paths)}
  _body_index={spec.name:indices[parts._PATHS[spec.name]] for spec in [*SECTORS,PAN,TOOL]}
  if _body_view.count!=N+2:raise ValueError('unknown native body view coverage')
  # Shape preparation is part of reset, before any policy action. Shared cap
  # faces are authored once on the settled, still source-recipe ring.
  deform_colliders(p,o);_record.cells=authored_cells(o)
  g=measure(o)
  _record.valid_reset=bool(.022<=g['d'] and g['D']<=.028 and .075<=g['outer']<=.085 and .022<=g['h_min']<=.035 and support(o,_record.cells))
  _record.invalid|=not _record.valid_reset
  _record.previous=p.copy();_record.previous_edge=edge_points(o[TOOL.name]);_record.t=float(sim.t)
  _guard_previous=None;_guard_edge=_record.previous_edge.copy();_guard_p=p.copy()
  if _physics_handle is not None:_physics_handle.deregister()
  _physics_handle=PhysxManager.register_callback(native_guard,IsaacEvents.POST_PHYSICS_STEP,name='dough_trial_guard',wrap_weak_ref=False)
  print('[dough] validated reset',_record.valid_reset,'d/D',g['d'],g['D'],flush=True)
 except Exception as e:
  _record.invalid=True;_record.valid_reset=False;print('[dough] reset invalid',type(e).__name__,str(e),flush=True)
lots._AFTER_RESET.append(validate_reset)

@parts.on_step
def step(sim):
 global _record
 o=sim.states();p=centers(o)
 try:
  from pxr import Gf
  clouds,load=hands(sim);dist=hand_distance(clouds,o)
  _record.peak=max(_record.peak,load)
  lengths=np.linalg.norm(np.roll(p,-1,axis=0)-p,axis=1)
  dt=float(sim.t)-_record.t
  moving=_record.previous is not None and np.linalg.norm(p-_record.previous,axis=1).max()>.00001
  # Plastic flow requires actual measured hand load AND contact proximity AND
  # movement. Passive relaxation never grows a ring. No desired radius is used.
  loaded=bool(load>=.05 and dist<.004 and moving)
  tear=bool(_record.sub_tear or np.any(lengths>1.8*_record.rest) or np.any(lengths<.25*model.L0) or cutting_edge(o,p))
  for i,j in enumerate(_joints):
   local_moving=_record.previous is not None and max(np.linalg.norm(p[i]-_record.previous[i]),np.linalg.norm(p[(i+1)%N]-_record.previous[(i+1)%N]))>1e-6
   if loaded and local_moving and not tear and not _record.invalid and lengths[i]>_record.rest[i] and dt>0:
    old=_record.rest[i];new=old+min(model.RATE*(lengths[i]-old)*dt,.005*dt)
    _record.rest[i]=new
    # Reorient anchors toward observed bonded material, with the same creep rate.
    a,b=_record.anchors[i];ra=parts._quat_to_matrix(o[SECTORS[i].name].quat_xyzw);rb=parts._quat_to_matrix(o[SECTORS[(i+1)%N].name].quat_xyzw)
    delta=p[(i+1)%N]-p[i];direction=delta/max(lengths[i],1e-9)
    gain=min(1.,model.RATE*dt)
    ta=ra.T@(direction*new/2);tb=rb.T@(-direction*new/2)
    a=a*(1-gain)+ta*gain;b=b*(1-gain)+tb*gain
    a=a/max(np.linalg.norm(a),1e-12)*(new/2);b=b/max(np.linalg.norm(b),1e-12)*(new/2)
    _record.anchors[i]=(a,b)
    j.GetLocalPos0Attr().Set(Gf.Vec3f(*map(float,a)));j.GetLocalPos1Attr().Set(Gf.Vec3f(*map(float,b)))
   if tear:j.GetJointEnabledAttr().Set(False)
  if loaded and not tear and not _record.invalid and dt>0:
   # Counter-supported compression is a separate material degree of freedom,
   # not inferred from desired diameter or ring length. Its real observable is
   # the material centre's height above the pan under a downward contact load.
   _,bnow=model.sections(p,_record.rest,_record.flatten)
   pan=o[PAN.name];pr=parts._quat_to_matrix(pan.quat_xyzw)
   local=(p-pan.pos)@pr
   supported_here=abs(local[:,2]-bnow-.001)<=.003
   # Contact matrix is force on fingers from food; positive pan-normal means
   # the opposing reaction compresses food downward onto its actual support.
   downward=(_food_force@pr[:,2]>.05)&(_food_load>=.05)&supported_here
   compressed=np.maximum(.2,np.minimum(1.,(local[:,2]-.001)/np.maximum(bnow,1e-6)))
   _record.flatten=np.maximum(.2,_record.flatten*(1-np.minimum(.2*dt,.02)*(1-compressed)*downward))
  if not tear and not _record.invalid:deform_colliders(p,o)
  known=bool(_record.substeps>0 and np.isfinite(np.r_[p.ravel(),lengths,load,_record.rest,_record.flatten]).all() and all(np.isfinite(np.r_[st.pos,st.quat_xyzw,st.lin_vel,st.ang_vel]).all() and abs(np.linalg.norm(st.quat_xyzw)-1)<.01 for st in o.values()))
  _record.cells=authored_cells(o)
  supported=support(o,_record.cells)
  surface=_record.cells
  off=bool(surface[:,:,2].min()<TOP-.020 or surface[:,:,0].max()>.90 or surface[:,:,0].min()<-.20 or abs(surface[:,:,1]).max()>.65)
  props_still=all(o[spec.name].resting(.005,.05) for spec in (PAN,TOOL))
  clear=dist>.009 and props_still and scraper_clear(o,_record.cells) and _record.sub_clear
  prop_geometry=np.concatenate([np.r_[o[spec.name].low,o[spec.name].high] for spec in (PAN,TOOL)])
  if clear:
   if _record.props_anchor is None:_record.props_anchor=prop_geometry.copy()
   elif np.max(abs(prop_geometry-_record.props_anchor))>.001:clear=False
  if not clear:_record.props_anchor=None
  _record.update(float(sim.t),p,known,clear,supported,tear or off or not _record.valid_reset)
  _record.sub_clear=True
  _record.certificate=stamp(o)
  _record.previous_edge=edge_points(o[TOOL.name])
  if os.environ.get('TRACE_OUT'):
   try:g=measure(o);g={k:float(v) for k,v in g.items() if np.isscalar(v)}
   except ValueError as error:g={'unmeasurable':str(error)}
   row=dict(lot=lots.current().name,t=sim.t,p=p.tolist(),lengths=lengths.tolist(),rest=_record.rest.tolist(),flatten=_record.flatten.tolist(),load=load,distance=dist,loaded=loaded,hold=_record.hold,substeps=_record.substeps,food_load=_food_load.tolist(),invalid=_record.invalid,valid_reset=_record.valid_reset,clear=bool(clear),props_still=bool(props_still),tool_clear=bool(scraper_clear(o,_record.cells)),supported=supported,score=score(o),geometry=g)
   with pathlib.Path(os.environ['TRACE_OUT']).with_name('dough.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
   snapshot=pathlib.Path(os.environ['TRACE_OUT']).with_name('geometry_failure.json')
   if clear and 'unmeasurable' in g and not snapshot.exists():
    snapshot.write_text(json.dumps(dict(p=p.tolist(),rest=_record.rest.tolist(),flatten=_record.flatten.tolist(),cells=_record.cells.tolist(),pan_pos=o[PAN.name].pos.tolist(),pan_quat=o[PAN.name].quat_xyzw.tolist(),error=g['unmeasurable'])))
 except (ValueError,KeyError,AttributeError,TypeError,IndexError) as e:
  _record.invalid=True
  if not _record.error:_record.error=str(e);print('[dough] fail closed',type(e).__name__,str(e),flush=True)

def score(o):
 try:
  if _record.certificate!=stamp(o):return 0.
  return float(_record.score(measure(o)))
 except (ValueError,KeyError,AttributeError,TypeError,IndexError):return 0.
def success(o):return score(o)==1.

def demo():
 # Single-hand native finger expansion, same plan translated with visible food.
 # Fingers enter closed, expand inside opening, retract and reorient while clear.
 cx,cy=lots.current().pos(SECTORS[0].name)[:2];cx-=model.R
 z=SURFACE+.033
 wp=[]
 for yaw in [0.,math.pi/4,-math.pi/2,-math.pi/4,0.,-math.pi/2]:
  wp.extend([Waypoint(2.,(cx,cy,z+.055),0.,'approach hole closed',yaw),
   Waypoint(2.,(cx,cy,z),0.,'insert native fingers',yaw),
   Waypoint(3.,(cx,cy,z),-.026,'open centre gently',yaw),
   Waypoint(3.,(cx,cy,z),-.026,'hold material under load',yaw),
   Waypoint(1.,(cx,cy,z),-.022,'ease local tension',yaw),
   Waypoint(1.,(cx,cy,z),-.026,'work loaded material',yaw),
   Waypoint(1.,(cx,cy,z),-.026,'settle under load',yaw),
   Waypoint(1.,(cx,cy,z),0.,'unload fingers',yaw),
   Waypoint(2.,(cx,cy,z+.055),0.,'withdraw before changing direction',yaw)])
 wp.extend([Waypoint(2.,(.20,.25,TOP+.14),FINGER_OPEN,'hands clear'),Waypoint(4.,(.20,.25,TOP+.14),FINGER_OPEN,'released geometry')])
 return {'left':wp}
