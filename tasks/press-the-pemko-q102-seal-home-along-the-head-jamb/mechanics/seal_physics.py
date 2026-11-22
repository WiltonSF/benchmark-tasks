"""Local extension of the standard parts kit: a freely connected seal with contact barbs.

The finite-compliance D6 joints connect adjacent lengths only. Nothing connects the strip
to the jamb except colliders. No seating latch, pose write or capture fixture.
Contact parameters and material behavior are stated engineering estimates.
"""
import numpy as np
import parts
from measurement import require_calibration

SEGMENTS = []
PITCH = 812.8e-3 / 34
_base_author = parts._author


def author(part, root_path, translation, orientation):
    # Validate the disclosed estimate record. Offline imports are pure.
    record = require_calibration()
    root = _base_author(part, root_path, translation, orientation)
    if part.name not in SEGMENTS:
        return root
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics
    from isaaclab.sim.utils import get_current_stage
    stage = get_current_stage()
    PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(root_path)).CreateThresholdAttr(0.)
    body_api=PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(root_path))
    body_api.CreateSolverPositionIterationCountAttr(32)
    body_api.CreateSolverVelocityIterationCountAttr(8)
    material = stage.GetPrimAtPath(root_path + '/friction')
    api = PhysxSchema.PhysxMaterialAPI.Apply(material)
    api.CreateCompliantContactStiffnessAttr(float(record['model_contact_stiffness_N_per_m']))
    api.CreateCompliantContactDampingAttr(float(record['model_contact_damping_Ns_per_m']))
    i = SEGMENTS.index(part.name)
    # Ink marks travel with the seal. They are visual-only and add no forces.
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_visual_material
    from pxr import UsdGeom
    tick=UsdGeom.Cube.Define(stage,root_path+'/span_tick')
    tick.CreateSizeAttr(1.)
    tick.AddTranslateOp().Set(Gf.Vec3d(0.,-PITCH/2+.0003,.00645))
    tick.AddScaleOp().Set(Gf.Vec3f(.015 if i%5==0 else .007,.0006,.0001))
    material_path=root_path+'/tick_look'
    visual=sim_utils.PreviewSurfaceCfg(diffuse_color=(.95,.95,.95))
    visual.func(material_path,visual)
    bind_visual_material(root_path+'/span_tick',material_path,stage=stage)
    if i:
        parent = root_path.rsplit('/', 1)[0] + '/' + SEGMENTS[i - 1]
        joint = UsdPhysics.Joint.Define(stage, root_path + '/continuity')
        joint.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(root_path)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(0., PITCH / 2, 0.))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0., -PITCH / 2, 0.))
        joint.CreateCollisionEnabledAttr(False)
        for axis in ('transX','transY','transZ'):
            limit=UsdPhysics.LimitAPI.Apply(joint.GetPrim(),axis)
            limit.CreateLowAttr(1.);limit.CreateHighAttr(-1.)
        for axis,angle,key in [('rotX',35.,'model_bend_Nm_per_rad'),('rotY',20.,'model_twist_Nm_per_rad'),('rotZ',35.,'model_inplane_bend_Nm_per_rad')]:
            limit=UsdPhysics.LimitAPI.Apply(joint.GetPrim(),axis)
            limit.CreateLowAttr(-angle);limit.CreateHighAttr(angle)
            drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),axis)
            drive.CreateTypeAttr('force')
            drive.CreateStiffnessAttr(float(record[key])*np.pi/180.)
            drive.CreateDampingAttr(.0001*np.pi/180.)
            drive.CreateTargetPositionAttr(0.)
            drive.CreateTargetVelocityAttr(0.)
    return root


parts.CONTACT_OFFSET = .0001
parts.REST_OFFSET = 0.
parts._author = author
