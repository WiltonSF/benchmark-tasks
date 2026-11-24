"""Task-owned, passive under-grip shoes on the holding hand. Stock actuators/actions are unchanged.
The two 50 g shoes provide positive bearing below the OEM grip; no grip friction is needed.
Dimensions are in each finger link frame, derived from the supplied YAM URDF, metres.
"""
from __future__ import annotations
import numpy as np
# Distal foot and outer web, both 22 mm wide along the stock grip.
# At zero finger translation, foot centres are (+/-2.5,0,156) mm in the gripper frame.
# Bearing tops extend20 mm past TCP. The toes reach across the centre to +/-16 mm;
# independent box checks give32 mm empty contact opening. Outer hardware is unchanged.
SPECS={
 'tip_left':[((.0239931,-.0420119,-.0993171),(.022,.027,.004)),
             ((.0239931,-.0524,-.0863171),(.022,.0088,.030)),
             ((.0239931,-.046142982,-.0808171),(.013,.0025,.016))],
 'tip_right':[((-.0239931,.0420119,-.0993171),(.022,.027,.004)),
              ((-.0239931,.0524,-.0863171),(.022,.0088,.030)),
              ((-.0239931,.046142982,-.0808171),(.013,.0025,.016))]}
SHOE_MASS=.050
STOCK_HULLS={}

def add_hulls():
 """Include the complete shoes in conservative hand-clearance sampling (both hands, conservatively)."""
 global STOCK_HULLS
 from yam_arena import analysis
 STOCK_HULLS={n:tuple(v) for n,v in analysis.link_hulls().items()}
 from itertools import product
 from scipy.spatial import ConvexHull
 for name,boxes in SPECS.items():
  for c,size in boxes:
   v=np.array(list(product((-1,1),repeat=3)))*np.asarray(size)/2+c
   h=ConvexHull(v)
   analysis.link_hulls()[name].append(analysis.Hull(v,np.c_[h.equations[:,:3],-h.equations[:,3]]))

def patch_robot(module):
 """Attach colliders at spawn; no in-trial attachment or object/robot pose writes."""
 original=module.yam_arm_cfg
 def cfg_with_shoes(*args,**kwargs):
  cfg=original(*args,**kwargs)
  if 'RightArm' not in cfg.prim_path:return cfg
  cfg.spawn.make_uninstanceable=True
  cfg.spawn.articulation_props.enabled_self_collisions=True
  spawn=cfg.spawn.func
  from isaaclab.sim.utils import clone,get_current_stage
  from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
  @clone
  def spawn_shoes(prim_path,spawn_cfg,translation=None,orientation=None,**kw):
   root=spawn(prim_path,spawn_cfg,translation=translation,orientation=orientation,**kw)
   stage=get_current_stage()
   import isaaclab.sim as sim_utils
   from isaaclab.sim.spawners.materials.physics_materials import spawn_physics_material
   from isaaclab.sim.utils import bind_physics_material
   friction_path=prim_path+'/handbrake_zero_friction'
   spawn_physics_material(friction_path,sim_utils.RigidBodyMaterialCfg(
       static_friction=0.,dynamic_friction=0.,friction_combine_mode='max'))
   for name,boxes in SPECS.items():
    body_path=prim_path+module.asset_link_paths()[name]
    body=stage.GetPrimAtPath(body_path)
    if not body.IsValid():raise RuntimeError('shoe attachment finger absent: '+body_path)
    mass=UsdPhysics.MassAPI.Apply(body)
    old=float(mass.GetMassAttr().Get() or .069657)
    import parts
    com=np.array(mass.GetCenterOfMassAttr().Get(),float)
    diagonal=np.array(mass.GetDiagonalInertiaAttr().Get(),float)
    axes=mass.GetPrincipalAxesAttr().Get()
    quat=np.r_[np.array(axes.GetImaginary()),axes.GetReal()]
    rotation=parts._quat_to_matrix(quat)
    inertia=rotation@np.diag(diagonal)@rotation.T
    # Conservative component allocation: 20 g foot, 25 g outer web/hardware, 5 g inner backing plate.
    weights=np.array((.020,.025,.005))
    assert abs(weights.sum()-SHOE_MASS)<1e-9
    total_com=(old*com+sum(w*np.array(c) for w,(c,_) in zip(weights,boxes)))/(old+SHOE_MASS)
    def shift(m,d):return m*(float(d@d)*np.eye(3)-np.outer(d,d))
    inertia+=shift(old,com-total_com)
    for w,(c,size) in zip(weights,boxes):
     x,y,z=size
     inertia+=w/12*np.diag([y*y+z*z,x*x+z*z,x*x+y*y])+shift(w,np.array(c)-total_com)
    if not np.isfinite(inertia).all():raise RuntimeError('finger inertia unavailable')
    values,vectors=np.linalg.eigh(inertia)
    if np.linalg.det(vectors)<0:vectors[:,0]*=-1
    q=parts._matrix_to_quat(vectors)
    mass.CreateMassAttr(old+SHOE_MASS)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(*total_com))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*values))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
    for k,(centre,size) in enumerate(boxes):
     cube=UsdGeom.Cube.Define(stage,body_path+'/handbrake_shoe_'+str(k))
     cube.CreateSizeAttr(1.)
     cube.AddTranslateOp().Set(Gf.Vec3d(*centre));cube.AddScaleOp().Set(Gf.Vec3d(*size))
     cube.CreateDisplayColorAttr([Gf.Vec3f(.72,.38,.12)])
     UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
     collision=PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
     collision.CreateContactOffsetAttr(.001);collision.CreateRestOffsetAttr(0.)
    for collider in stage.Traverse():
     if str(collider.GetPath()).startswith(body_path+'/') and collider.HasAPI(UsdPhysics.CollisionAPI):
      bind_physics_material(str(collider.GetPath()),friction_path,stage=stage)
   # Keep the stock original-original self filtering, while the added hardware really collides
   # with the opposite finger/tool. Global self-collision-off would allow the steel feet through each other.
   original_colliders=[p.GetPath() for p in stage.Traverse()
       if str(p.GetPath()).startswith(prim_path+'/') and p.HasAPI(UsdPhysics.CollisionAPI)
       and '/handbrake_shoe_' not in str(p.GetPath())]
   if not original_colliders:raise RuntimeError('original robot collider collection empty')
   group=UsdPhysics.CollisionGroup.Define(stage,prim_path+'/handbrake_original_collision_group')
   collection=group.GetCollidersCollectionAPI()
   collection.CreateExpansionRuleAttr('explicitOnly')
   collection.CreateIncludesRel().SetTargets(original_colliders)
   group.CreateFilteredGroupsRel().AddTarget(group.GetPrim().GetPath())
   return root
  cfg.spawn.func=spawn_shoes
  return cfg
 module.yam_arm_cfg=cfg_with_shoes
