"""Retain stock motor caps, enforce them on drive plus external gravity feed-forward.

PhysX's ordinary drive maxForce excludes externally cached joint effort. The supported
performance-envelope API applies that same maxForce to their sum. No new actuator,
force allowance, gain, speed, action dimension or in-trial force/pose writer is added.
"""
import shoes
CAPS={**{f'joint{i}':27. for i in (1,2,3)},**{f'joint{i}':7. for i in (4,5,6)},'joint7':50.,'joint8':50.}
SOURCE='https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/rigid_bodies_articulations/articulations.html'

def limits_valid(values,names):
 import numpy as np
 values=np.asarray(values,dtype=float).reshape(-1)
 return (len(values)==len(names)==len(CAPS) and set(names)==set(CAPS)
         and np.isfinite(values).all() and all(0.<value<=CAPS[name]+1e-5 for name,value in zip(names,values)))

def apply(stage,root):
 from pxr import Usd,UsdPhysics,Sdf
 found={}
 for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
  name=prim.GetName()
  if name not in CAPS:continue
  axis='angular' if prim.IsA(UsdPhysics.RevoluteJoint) else 'linear' if prim.IsA(UsdPhysics.PrismaticJoint) else None
  if axis is None:continue
  drive=UsdPhysics.DriveAPI.Apply(prim,axis)
  drive.CreateMaxForceAttr(CAPS[name])
  schema='PhysxDrivePerformanceEnvelopeAPI:'+axis
  prim.AddAppliedSchema(schema)
  ns='physxDrivePerformanceEnvelope:'+axis+':'
  # Infinite no-load speed and zero slopes retain the existing separate stock velocity caps.
  for attr,value in [('maxActuatorVelocity',float('inf')),('velocityDependentResistance',0.),('speedEffortGradient',0.)]:
   prim.CreateAttribute(ns+attr,Sdf.ValueTypeNames.Float).Set(value)
  if schema not in prim.GetAppliedSchemas():raise RuntimeError('motor performance-envelope schema not registered')
  found[name]=CAPS[name]
 if found!=CAPS:raise RuntimeError('stock motor joints absent from spawned robot: '+str(found))
 return found

def patch_robot(module):
 shoes.patch_robot(module)
 original=module.yam_arm_cfg
 def capped_cfg(*args,**kwargs):
  cfg=original(*args,**kwargs);cfg.spawn.make_uninstanceable=True
  # Numerical convergence only: unchanged stock drives, masses, gains, caps and velocity limits.
  cfg.spawn.articulation_props.solver_position_iteration_count=128
  cfg.spawn.articulation_props.solver_velocity_iteration_count=8
  spawn=cfg.spawn.func
  from isaaclab.sim.utils import clone,get_current_stage
  @clone
  def spawn_capped(prim_path,spawn_cfg,translation=None,orientation=None,**kw):
   root=spawn(prim_path,spawn_cfg,translation=translation,orientation=orientation,**kw)
   apply(get_current_stage(),prim_path)
   return root
  cfg.spawn.func=spawn_capped
  return cfg
 module.yam_arm_cfg=capped_cfg
